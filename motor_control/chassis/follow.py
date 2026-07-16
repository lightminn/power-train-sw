"""앞 로봇 추종 — 국방 ⑤구간 (WP7). 순수 계산 코어.

    /detected_objects (YOLO) → [이것] → (v, ω) → /autonomy/cmd_vel

앞선 로봇을 일정 거리 뒤에서 따라간다. 인식은 **로봇팔 팀 단일 소스**이고 우리는
결과만 구독한다.

────────────────────────────────────────────────────────────────────────
★ 거리 제어와 **추돌 방지**는 다른 문제다
────────────────────────────────────────────────────────────────────────
단순 PID 로 목표 거리를 맞추면, 앞 로봇이 **급정거**할 때 우리는 관성으로 밀고 들어간다.
그래서 **접근 속도**(거리의 변화율)를 봐야 한다:
        v = kp·(거리 − 목표) − kd·(접근 속도)
    · 거리가 목표보다 멀면 가속 (kp 항)
    · **빠르게 가까워지고 있으면 미리 감속** (kd 항) ← 추돌을 막는 항
  그리고 **최소 거리 안으로 들어오면 무조건 정지**한다(v ≤ 0). PID 를 믿지 않는다.

★ 가림 중에는 마지막 거리·접근 속도로 짧게 외삽하되 매 tick 감속한다. 예측 한계를
  넘긴 첫 tick은 0속도를 내고, 그 뒤에는 `ok=False`로 재검출을 기다린다. 재검출은
  위치·bbox 크기·class·시간 연속성을 연속 frame으로 확인한다.

입력 위치는 ROS 래퍼가 TF 변환한 ``base_link`` 기준이다. x=전방거리,
y=왼쪽 횡오프셋이며 코어는 ROS 메시지나 TF에 의존하지 않는다.
"""
from dataclasses import dataclass
import math


@dataclass
class FollowConfig:
    class_name: str = "robot"       # 앞 로봇의 YOLO 클래스명
    min_confidence: float = 0.5

    target_m: float = 2.0           # 유지할 거리
    min_m: float = 1.5              # ★ 이 안으로 들어오면 무조건 정지
    band_m: tuple = (1.5, 2.5)      # 허용 간격 범위
    band_gain: float = 0.2          # 허용 범위 안에서는 거리항을 약하게
    max_m: float = 6.0              # 이보다 멀면 놓친 것으로 본다

    kp: float = 0.8                 # 거리 오차 → v
    kd: float = 1.2                 # ★ 접근 속도 → v (추돌 방지 항)
    k_yaw: float = 1.5              # 횡오프셋 → ω
    v_max: float = 0.8
    omega_max: float = 1.0

    lost_grace_s: float = 0.5       # 이만큼은 마지막 추정으로 버틴다
    predict_decay: float = 0.5      # 가림 tick마다 예측 명령 감속
    predict_limit_s: float = 1.0    # 이보다 긴 외삽은 금지
    reacquire_pos_m: float = 1.0    # 마지막 예측 위치와의 최대 차이
    reacquire_size_ratio: tuple = (0.5, 2.0)
    reacquire_max_gap_s: float = 3.0
    reacquire_confirm_n: int = 2
    ema: float = 0.4                # 거리 평활 (YOLO 지터 억제)


@dataclass(frozen=True)
class FollowResult:
    ok: bool
    v: float = 0.0
    omega: float = 0.0
    distance_m: float = 0.0
    closing_mps: float = 0.0        # + = 가까워지는 중
    reason: str = ""
    state: str = "LOST"             # TRACKING / PREDICTING / REACQUIRING / LOST


class LeadFollower:
    def __init__(self, cfg: FollowConfig = None):
        self.cfg = cfg or FollowConfig()
        self._d = None              # 평활된 거리
        self._t = None
        self._t_seen = None
        self._closing = 0.0
        self._lat = 0.0
        self._last_v = 0.0
        self._last_omega = 0.0
        self._area = None
        self._reacquire_count = 0
        self._had_loss = False
        self._stop_emitted = False

    def reset(self):
        self._d = None
        self._t = None
        self._t_seen = None
        self._closing = 0.0
        self._lat = 0.0
        self._last_v = 0.0
        self._last_omega = 0.0
        self._area = None
        self._reacquire_count = 0
        self._had_loss = False
        self._stop_emitted = False

    def update(self, detections, t: float) -> FollowResult:
        """detections: [(class_name, confidence, forward_m, left_m, bbox_area_px), ...]"""
        c = self.cfg
        hit = self._pick(detections)

        if hit is None:
            if self._had_loss and detections:
                return self._on_reacquire(detections, t)
            return self._on_loss(t)

        if (not self._had_loss and self._t_seen is not None
                and t - self._t_seen > c.lost_grace_s):
            # 빈 array callback 자체가 끊겨도 긴 stamp gap은 검출 상실이다.
            self._had_loss = True

        if self._had_loss and t - self._t_seen > c.reacquire_max_gap_s:
            # 오래 지난 검출은 과거 대상을 억지로 이어 붙이지 않고 신규 획득한다.
            self.reset()
        elif self._had_loss:
            return self._on_reacquire(detections, t)

        _, _, dist, lat, area = hit
        self._t_seen = t

        # 거리 평활 + 접근 속도
        if self._d is None:
            self._d, self._t = dist, t
            self._lat = lat
            self._area = area
            return FollowResult(False, distance_m=dist, reason="초기화",
                                state="REACQUIRING")
        dt = t - self._t
        if dt <= 0.0:
            return FollowResult(False, distance_m=self._d, reason="dt 없음",
                                state="REACQUIRING")

        d_prev = self._d
        self._d = c.ema * dist + (1 - c.ema) * self._d
        self._t = t
        closing = (d_prev - self._d) / dt              # + = 가까워지는 중
        self._closing = closing
        self._lat = lat
        self._area = area
        self._had_loss = False
        self._stop_emitted = False

        # ★ 최소 거리 안 = 무조건 정지. PID 를 믿지 않는다.
        if dist <= c.min_m or self._d <= c.min_m:
            omega = _clamp(c.k_yaw * lat, -c.omega_max, c.omega_max)
            self._last_v, self._last_omega = 0.0, omega
            return FollowResult(True, 0.0, omega, self._d, closing,
                                "최소 거리 — 정지", "TRACKING")

        # v = 거리 오차 − 접근 속도 보정 (빠르게 다가가면 미리 감속)
        kp = c.kp * c.band_gain if c.band_m[0] < self._d <= c.band_m[1] else c.kp
        v = kp * (self._d - c.target_m) - c.kd * closing
        v = _clamp(v, 0.0, c.v_max)                    # 후진은 안 한다
        omega = _clamp(c.k_yaw * lat, -c.omega_max, c.omega_max)
        self._last_v, self._last_omega = v, omega
        return FollowResult(True, v, omega, self._d, closing, "추종", "TRACKING")

    def _on_loss(self, t):
        c = self.cfg
        if self._t_seen is None or self._d is None:
            self.reset()
            return FollowResult(False, reason="앞 로봇 놓침 — 따라가지 않는다",
                                state="LOST")

        self._had_loss = True
        self._reacquire_count = 0
        elapsed = max(0.0, t - self._t_seen)
        prediction_window = min(c.lost_grace_s, c.predict_limit_s)
        predicted_d = max(0.0, self._d - self._closing * elapsed)
        if elapsed <= prediction_window:
            self._last_v *= c.predict_decay
            self._last_omega *= c.predict_decay
            if predicted_d <= c.min_m:
                self._last_v = 0.0
            return FollowResult(
                True, self._last_v, self._last_omega, predicted_d, self._closing,
                "일시적 미검출 — 감속 예측", "PREDICTING",
            )

        if not self._stop_emitted:
            self._stop_emitted = True
            self._last_v = self._last_omega = 0.0
            return FollowResult(
                True, 0.0, 0.0, predicted_d, self._closing,
                "예측 한계 — 정지", "LOST",
            )
        return FollowResult(
            False, distance_m=predicted_d, closing_mps=self._closing,
            reason="앞 로봇 놓침 — 재검출 대기", state="LOST",
        )

    def _on_reacquire(self, detections, t):
        c = self.cfg
        elapsed = max(0.0, t - self._t_seen)
        predicted_d = max(0.0, self._d - self._closing * elapsed)
        candidate, rejected = self._pick_reacquire(detections, predicted_d)
        if candidate is None:
            self._reacquire_count = 0
            return FollowResult(
                False, distance_m=predicted_d, closing_mps=self._closing,
                reason=f"재검출 심사 — {rejected}", state="REACQUIRING",
            )

        self._reacquire_count += 1
        _, _, dist, lat, area = candidate
        needed = max(1, int(c.reacquire_confirm_n))
        if self._reacquire_count < needed:
            return FollowResult(
                False, distance_m=dist, closing_mps=self._closing,
                reason=f"재검출 심사 {self._reacquire_count}/{needed}",
                state="REACQUIRING",
            )

        # 수락 프레임도 불확실 구간의 끝이다. 0속도로 다시 초기화하고 다음 fresh
        # 프레임부터 정상 추종한다.
        self._d, self._t, self._t_seen = dist, t, t
        self._closing = 0.0
        self._lat = lat
        self._area = area
        self._last_v = self._last_omega = 0.0
        self._reacquire_count = 0
        self._had_loss = False
        self._stop_emitted = False
        return FollowResult(
            False, distance_m=dist, reason="재검출 수락 — 초기화",
            state="REACQUIRING",
        )

    def _pick_reacquire(self, detections, predicted_d):
        c = self.cfg
        best = None
        rejected = "class 불일치"
        for hit in self._valid_detections(detections):
            _, _, dist, lat, area = hit
            position_error = math.hypot(dist - predicted_d, lat - self._lat)
            if position_error > c.reacquire_pos_m:
                rejected = "위치 불연속"
                continue
            if self._area is None or self._area <= 0.0 or area <= 0.0:
                rejected = "bbox 크기 없음"
                continue
            ratio = area / self._area
            if not (c.reacquire_size_ratio[0] <= ratio <= c.reacquire_size_ratio[1]):
                rejected = "bbox 크기 불연속"
                continue
            if best is None or position_error < best[0]:
                best = (position_error, hit)
        return (best[1], "") if best is not None else (None, rejected)

    def _pick(self, detections):
        best = None
        for hit in self._valid_detections(detections):
            if best is None or hit[2] < best[2]:
                best = hit
        return best

    def _valid_detections(self, detections):
        c = self.cfg
        for detection in detections:
            if len(detection) == 4:                    # 기존 호출자와의 전환 호환
                name, conf, dist, lat = detection
                area = 0.0
            else:
                name, conf, dist, lat, area = detection
            if name != c.class_name or conf < c.min_confidence:
                continue
            if not (0.0 < dist < c.max_m):
                continue
            yield name, conf, dist, lat, area


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))
