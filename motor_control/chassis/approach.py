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
