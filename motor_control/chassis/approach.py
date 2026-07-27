"""능동 최종접근 정렬 — 대상 pose(base_link)를 보고 폐루프로 크립·정렬한다.

    /detected_objects(팔 YOLO, TF→base_link) → [이것] → /autonomy/cmd_vel + ARRIVED 서비스

하드웨어·ROS 의존 없음. 인식은 로봇팔 팀 단일 소스, 우리는 결과만 받는다.
설계: docs/superpowers/specs/2026-07-27-vision-arrival-approach-design.md
"""
from dataclasses import dataclass

from chassis.mission_trigger import MissionTrigger, TriggerConfig, TriggerRule

# 계약 어휘 (contract.py와 동일 문자열; 순수 코어라 하드코딩)
ARRIVED_PICKUP = "ARRIVED_PICKUP"
ARRIVED_DROP = "ARRIVED_DROP"

# 상태
SEARCHING = "SEARCHING"
APPROACHING = "APPROACHING"
ALIGNED = "ALIGNED"
ARRIVED_FIRED = "ARRIVED_FIRED"
DONE = "DONE"
BACKOFF = "BACKOFF"
FAILED_HOLD = "FAILED_HOLD"


@dataclass
class ApproachConfig:
    engage_m: float = 2.0          # 이 안에 들어오면 lock(접근 시작)
    stop_m: float = 1.0            # 이 거리에서 정지(팔 리치, HIL 캘리브)
    lat_tol: float = 0.05          # 횡오차 허용(정렬 판정)
    dist_tol: float = 0.05         # 거리오차 허용(정렬 판정)
    k_yaw: float = 1.5             # 횡오차 → omega 이득
    k_dist: float = 0.8            # 거리오차 → v 이득
    omega_max: float = 0.4         # rad/s
    v_approach_max: float = 0.15   # m/s (저속 정밀 크립)
    v_settle: float = 0.03         # m/s 미만이면 "정지"로 인정
    backoff_creep: float = 0.10    # m/s (뒤로 물러나는 속도)
    backoff_time_s: float = 1.0
    align_timeout_s: float = 8.0
    max_retries: int = 2
    consecutive: int = 5           # lock 디바운스 프레임
    cooldown_s: float = 10.0       # 미션 완료 후 같은 클래스 무시
    min_confidence: float = 0.6
    lost_frames: int = 5           # 연속 이만큼 못 보면 lost
    pose_stale_s: float = 0.5      # 노드가 freshness 게이트에 참고
    pickup_class: str = "box"
    drop_class: str = "dropzone"


@dataclass
class Target:
    """base_link로 투영된 검출 1개."""
    class_name: str
    confidence: float
    x: float                       # 전방거리 m
    y: float                       # 횡오차 m (+좌)


@dataclass
class ApproachDecision:
    state: str
    active: bool                   # /approach/active
    v: float                       # /autonomy/cmd_vel linear.x 제안
    omega: float                   # angular.z 제안
    fire: str = None               # ARRIVED_* (서비스 호출) or None
    reason: str = ""
    retries: int = 0


def creep_cmd(cfg: ApproachConfig, x: float, y: float) -> tuple:
    """대상 상대위치(x 전방, y 횡) → (v, omega). 후진 없음, 전부 클램프."""
    omega = max(-cfg.omega_max, min(cfg.omega_max, -cfg.k_yaw * y))
    v = cfg.k_dist * (x - cfg.stop_m)
    v = max(0.0, min(cfg.v_approach_max, v))
    return v, omega


class ApproachController:
    """감지→능동 접근→정렬→발사. 상태와 카운터만 보유(순수)."""

    def __init__(self, cfg: ApproachConfig = None, trigger: MissionTrigger = None):
        self.cfg = cfg or ApproachConfig()
        c = self.cfg
        self.trigger = trigger or MissionTrigger(TriggerConfig(
            rules=[
                TriggerRule(c.pickup_class, ARRIVED_PICKUP, c.engage_m, c.min_confidence),
                TriggerRule(c.drop_class, ARRIVED_DROP, c.engage_m, c.min_confidence),
            ],
            consecutive=c.consecutive,
            cooldown_s=c.cooldown_s,
        ))
        self.state = SEARCHING
        self.retries = 0
        self.active_status = None       # ARRIVED_PICKUP / ARRIVED_DROP
        self.active_class = None
        self._enter_s = 0.0
        self._backoff_until = 0.0
        self._lost = 0
        self._fired = False

    def _match(self, targets):
        """active_class에 맞는 가장 가까운 유효 대상."""
        best = None
        for t in targets:
            if t.class_name != self.active_class:
                continue
            if t.confidence < self.cfg.min_confidence:
                continue
            if t.x <= 0.0:
                continue
            if best is None or t.x < best.x:
                best = t
        return best

    def _result(self, state, active, v, omega, fire=None, reason=""):
        self.state = state
        return ApproachDecision(state, active, v, omega, fire, reason, self.retries)

    def update(self, targets, speed_mps, now_s) -> ApproachDecision:
        c = self.cfg
        if self.state in (SEARCHING,):
            dets = [(t.class_name, t.confidence, t.x) for t in targets]
            hit = self.trigger.on_detections(dets, now_s)
            if hit is None:
                return self._result(SEARCHING, False, 0.0, 0.0, reason="searching")
            status, cls = hit
            self.active_status, self.active_class = status, cls
            self.retries = 0
            self._fired = False
            self._enter_s = now_s
            self._lost = 0
            # 즉시 첫 크립
            return self._enter_approaching(targets, speed_mps, now_s)

        if self.state == APPROACHING:
            return self._tick_approaching(targets, speed_mps, now_s)

        if self.state == BACKOFF:
            m = self._match(targets)
            if now_s >= self._backoff_until:
                if m is not None:
                    self._enter_s = now_s
                    self._lost = 0
                    return self._enter_approaching(targets, speed_mps, now_s)
                # 재획득 실패 → 다시 SEARCHING (쿨다운/디바운스 재적용)
                self.active_status = self.active_class = None
                return self._result(SEARCHING, False, 0.0, 0.0, reason="backoff_gaveup")
            return self._result(BACKOFF, True, -c.backoff_creep, 0.0, reason="backoff")

        if self.state == ALIGNED:
            # 발사는 이미 방출됨 → 서비스 ACK 대기(노드가 on_service_ack 호출)
            return self._result(ALIGNED, True, 0.0, 0.0, reason="await_ack")

        if self.state == ARRIVED_FIRED:
            return self._result(ARRIVED_FIRED, True, 0.0, 0.0, reason="arm_working")

        if self.state == FAILED_HOLD:
            return self._result(FAILED_HOLD, True, 0.0, 0.0, reason="align_failed")

        if self.state == DONE:
            return self._result(SEARCHING, False, 0.0, 0.0, reason="resumed")

        return self._result(self.state, False, 0.0, 0.0, reason="unknown")

    def _enter_approaching(self, targets, speed_mps, now_s):
        self.state = APPROACHING
        return self._tick_approaching(targets, speed_mps, now_s)

    def _tick_approaching(self, targets, speed_mps, now_s):
        c = self.cfg
        m = self._match(targets)
        if m is None:
            self._lost += 1
            if self._lost >= c.lost_frames:
                self._backoff_until = now_s + c.backoff_time_s
                return self._result(BACKOFF, True, -c.backoff_creep, 0.0, reason="lost")
            return self._result(APPROACHING, True, 0.0, 0.0, reason="lost_grace")
        self._lost = 0
        # 정렬 판정
        aligned = (abs(m.y) < c.lat_tol
                   and abs(m.x - c.stop_m) < c.dist_tol
                   and abs(speed_mps) < c.v_settle)
        if aligned and not self._fired:
            self._fired = True
            return self._result(ALIGNED, True, 0.0, 0.0,
                                fire=self.active_status, reason="aligned")
        # 타임아웃 → 재시도/실패
        if now_s - self._enter_s > c.align_timeout_s:
            self.retries += 1
            if self.retries > c.max_retries:
                return self._result(FAILED_HOLD, True, 0.0, 0.0, reason="align_failed")
            self._backoff_until = now_s + c.backoff_time_s
            return self._result(BACKOFF, True, -c.backoff_creep, 0.0, reason="retry")
        v, omega = creep_cmd(c, m.x, m.y)
        return self._result(APPROACHING, True, v, omega, reason="approaching")

    def on_service_ack(self, success: bool, now_s: float) -> None:
        if self.state != ALIGNED:
            return
        if success:
            self.state = ARRIVED_FIRED
            return
        # 서버 거부 → 재시도 또는 실패
        self._fired = False
        self.retries += 1
        if self.retries > self.cfg.max_retries:
            self.state = FAILED_HOLD
        else:
            self._backoff_until = now_s + self.cfg.backoff_time_s
            self.state = BACKOFF

    def on_mission_done(self, now_s: float) -> None:
        if self.state not in (ARRIVED_FIRED, ALIGNED):
            return
        if self.active_class is not None:
            self.trigger.mission_finished(self.active_class, now_s)
        self.active_status = self.active_class = None
        self._fired = False
        self.retries = 0
        self.state = DONE

    def reset(self, now_s: float) -> None:
        self.active_status = self.active_class = None
        self._fired = False
        self.retries = 0
        self._lost = 0
        self.state = SEARCHING
