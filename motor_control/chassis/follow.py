"""앞 로봇 추종 — 국방 ⑤구간 (WP9). 순수 계산 코어.

    /detected_objects (YOLO) → [이것] → (v, ω) → /autonomy/cmd_vel

앞선 로봇을 일정 거리 뒤에서 따라간다. 인식은 **로봇팔 팀 단일 소스**이고 우리는
결과만 구독한다.

────────────────────────────────────────────────────────────────────────
★ 거리 제어와 **추돌 방지**는 다른 문제다
────────────────────────────────────────────────────────────────────────
단순 PID 로 목표 거리를 맞추면, 앞 로봇이 **급정거**할 때 우리는 관성으로 밀고 들어간다.
그래서 **접근 속도**(거리의 변화율)를 봐야 한다:
        v = kp·(거리 − 목표) + kd·(접근 속도)
    · 거리가 목표보다 멀면 가속 (kp 항)
    · **빠르게 가까워지고 있으면 미리 감속** (kd 항) ← 추돌을 막는 항
  그리고 **최소 거리 안으로 들어오면 무조건 정지**한다(v ≤ 0). PID 를 믿지 않는다.

★ **놓치면 따라가지 않는다.** 앞 로봇을 잃었는데 마지막 명령을 반복하면, 우리는 아무것도
  없는 곳으로 계속 달린다. `ok=False` 를 돌려주고 상위가 결정하게 한다.
  ⚠️ 다만 **한 프레임 놓쳤다고 급정거**해도 안 된다(YOLO 는 흔들린다) → `lost_grace_s`
     동안은 마지막 추정으로 버틴다. 그 이상이면 포기한다.

⚠️ `DetectedObject.pose` 좌표계가 계약에 미명시 — 카메라(광학) 기준 가정.
   z=전방거리, x=횡오프셋. **로봇팔 팀과 확정 필요.**
"""
from dataclasses import dataclass
import math


@dataclass
class FollowConfig:
    class_name: str = "robot"       # 앞 로봇의 YOLO 클래스명
    min_confidence: float = 0.5

    target_m: float = 1.5           # 유지할 거리
    min_m: float = 0.8              # ★ 이 안으로 들어오면 무조건 정지
    max_m: float = 6.0              # 이보다 멀면 놓친 것으로 본다

    kp: float = 0.8                 # 거리 오차 → v
    kd: float = 1.2                 # ★ 접근 속도 → v (추돌 방지 항)
    k_yaw: float = 1.5              # 횡오프셋 → ω
    v_max: float = 0.8
    omega_max: float = 1.0

    lost_grace_s: float = 0.5       # 이만큼은 마지막 추정으로 버틴다
    ema: float = 0.4                # 거리 평활 (YOLO 지터 억제)


@dataclass(frozen=True)
class FollowResult:
    ok: bool
    v: float = 0.0
    omega: float = 0.0
    distance_m: float = 0.0
    closing_mps: float = 0.0        # + = 가까워지는 중
    reason: str = ""


class LeadFollower:
    def __init__(self, cfg: FollowConfig = None):
        self.cfg = cfg or FollowConfig()
        self._d = None              # 평활된 거리
        self._t = None
        self._t_seen = None

    def reset(self):
        self._d = None
        self._t = None
        self._t_seen = None

    def update(self, detections, t: float) -> FollowResult:
        """detections: [(class_name, confidence, distance_m, lateral_m), ...]"""
        c = self.cfg
        hit = self._pick(detections)

        if hit is None:
            # ⚠️ 한 프레임 놓쳤다고 급정거하지 않는다 — YOLO 는 흔들린다.
            if self._t_seen is not None and t - self._t_seen <= c.lost_grace_s:
                return FollowResult(False, reason="일시적 미검출 — 유예 중")
            self.reset()
            return FollowResult(False, reason="앞 로봇 놓침 — 따라가지 않는다")

        _, _, dist, lat = hit
        self._t_seen = t

        # 거리 평활 + 접근 속도
        if self._d is None:
            self._d, self._t = dist, t
            return FollowResult(False, distance_m=dist, reason="초기화")
        dt = t - self._t
        if dt <= 0.0:
            return FollowResult(False, distance_m=self._d, reason="dt 없음")

        d_prev = self._d
        self._d = c.ema * dist + (1 - c.ema) * self._d
        self._t = t
        closing = (d_prev - self._d) / dt              # + = 가까워지는 중

        # ★ 최소 거리 안 = 무조건 정지. PID 를 믿지 않는다.
        if self._d <= c.min_m:
            return FollowResult(True, 0.0, _clamp(c.k_yaw * -lat, -c.omega_max, c.omega_max),
                                self._d, closing, "최소 거리 — 정지")

        # v = 거리 오차 − 접근 속도 보정 (빠르게 다가가면 미리 감속)
        v = c.kp * (self._d - c.target_m) - c.kd * closing
        v = _clamp(v, 0.0, c.v_max)                    # 후진은 안 한다
        omega = _clamp(c.k_yaw * -lat, -c.omega_max, c.omega_max)
        return FollowResult(True, v, omega, self._d, closing, "추종")

    def _pick(self, detections):
        c = self.cfg
        best = None
        for name, conf, dist, lat in detections:
            if name != c.class_name or conf < c.min_confidence:
                continue
            if not (0.0 < dist < c.max_m):
                continue
            if best is None or dist < best[2]:
                best = (name, conf, dist, lat)
        return best


def _clamp(v, lo, hi):
    return max(lo, min(hi, v))
