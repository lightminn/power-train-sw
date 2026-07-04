"""차체 통합 제어 — 코너모듈 6개를 하나의 "차체"로 묶는다 (WP3).

`set(v, ω)` 한 줄이면 kinematics 가 바퀴별 (조향각, 구동속도)를 풀고, 50Hz 루프
(`tick()`)가 그 결과를 각 CornerModule 에 분배해 일괄 구동한다. 안전은 차체 레벨에서
총괄:
  - estop 전파 : 코너 1곳이라도 트립(fault/과전류/stale)하면 6코너 전부 정지.
  - US-100 게이팅 : `stop` 판정 → 구동 0, 조향은 유지(⚠️ v만 0으로 두면 ω가 남아
                    피벗이 돼버림 — 그래서 구동 출력만 끊는다).
  - 워치독 : `chassis.set()` 이 watchdog_ms 넘게 안 오면 구동 0(조향 유지).
             (코너 워치독은 매 tick 재-set 되어 안 먹으므로 차체가 담당.)

좌표·입력 규약은 kinematics 와 동일: v[m/s] 전진, ω[rad/s] 요레이트(>0=좌회전).
하드웨어·ROS 무관 — fake 드라이버로 pytest 되고, ROS2 노드(WP5)는 이 클래스를 감싸기만.

코너↔모터 매핑은 `WheelMap` 표(DEFAULT_WHEEL_MAP)로 주입 — 실배치는 조립 후 확인,
표 숫자만 교체하면 코드는 무관.
"""
import logging
import time
from dataclasses import dataclass, field

from chassis.kinematics import ChassisGeometry, default_geometry, solve
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.null_steer import NullSteer

logger = logging.getLogger(__name__)


# ── 설정 · 매핑 표 ────────────────────────────────────────────────────────


@dataclass
class ChassisConfig:
    geometry: ChassisGeometry = field(default_factory=default_geometry)
    corner: CornerConfig = field(default_factory=CornerConfig)
    watchdog_ms: float = 300.0         # chassis.set() 입력 타임아웃
    loop_hz: float = 50.0              # 제어 루프 주기


@dataclass(frozen=True)
class WheelMap:
    """바퀴 하나의 모터 배정. steer_can_id=None → 조향모터 없는 고정 바퀴."""
    wheel: str                         # kinematics Wheel.name 과 일치해야
    steer_can_id: int                  # AK 조향 CAN id (None=고정)
    drive_node_id: int                 # ODrive 구동 CAN node


# AK id 1~4 조향, ODrive node 11~16 구동(전부 셋업·CAN 캘리 완료 2026-07-05).
# ⚠️ node 번호는 확정이나, 어느 node가 어느 물리 바퀴인지(행↔node 배치)는
#    조립 배선 확인 후 숫자만 교체. 중간 2바퀴는 조향모터 없음(None, 고정).
DEFAULT_WHEEL_MAP = [
    WheelMap("front_left",  1,    11),
    WheelMap("front_right", 2,    12),
    WheelMap("mid_left",    None, 13),   # ⚠️ 조향 없음(고정), 구동 node 조립 후 확정
    WheelMap("mid_right",   None, 14),   # ⚠️
    WheelMap("rear_left",   3,    15),
    WheelMap("rear_right",  4,    16),
]


# ── 코너 빌더 (의존성 주입) ───────────────────────────────────────────────


def build_corners(steer_factory, drive_factory, cfg: CornerConfig = None,
                  wheel_map=None) -> dict:
    """매핑 표대로 CornerModule 6개를 만든다.

    steer_factory(can_id)->SteerActuator, drive_factory(node_id)->DriveActuator 를
    주입한다(pytest=fake, 실기=SteerAk40/DriveOdriveCan). 고정 바퀴(steer_can_id=None)
    는 팩토리와 무관하게 NullSteer 를 문다.
    """
    cfg = cfg or CornerConfig()
    corners = {}
    for wm in (wheel_map or DEFAULT_WHEEL_MAP):
        steer = NullSteer() if wm.steer_can_id is None else steer_factory(wm.steer_can_id)
        corners[wm.wheel] = CornerModule(steer, drive_factory(wm.drive_node_id), cfg)
    return corners


def build_real_corners(channel: str = "can0", cfg: CornerConfig = None,
                       wheel_map=None) -> dict:
    """실기용 — AK 조향(CAN) + ODrive 구동(CAN) 코너 6개. 하드웨어 라이브러리는
    지연 import(무하드웨어 pytest 가 python-can/odrive 없이 이 모듈을 쓰게)."""
    from corner_module.steer_ak40 import SteerAk40
    from corner_module.drive_odrive_can import DriveOdriveCan   # WP1 완료 필요
    return build_corners(
        steer_factory=lambda cid: SteerAk40(motor_id=cid, channel=channel),
        drive_factory=lambda nid: DriveOdriveCan(node_id=nid, channel=channel),
        cfg=cfg, wheel_map=wheel_map,
    )


# ── 차체 매니저 ───────────────────────────────────────────────────────────


class ChassisManager:
    def __init__(self, corners: dict, cfg: ChassisConfig = None,
                 monitor=None, clock=None):
        self.cfg = cfg or ChassisConfig()
        self.corners = corners             # {wheel_name: CornerModule}
        self.monitor = monitor             # US-100 SafetyMonitor (옵션)
        self.mode = "DISCONNECTED"
        self._v = 0.0
        self._omega = 0.0
        self._last_set_ms = None
        self._verdict = None
        self._now = clock or time.monotonic

        # 매핑 검증: geometry 의 모든 바퀴가 코너로 존재해야 (오배선 조기 발견)
        need = {w.name for w in self.cfg.geometry.wheels}
        missing = need - set(self.corners)
        if missing:
            raise ValueError(f"코너 매핑 누락: {sorted(missing)}")

    def _now_ms(self) -> float:
        return self._now() * 1000.0

    # ── 라이프사이클 ──────────────────────────────────────────────────
    def connect(self) -> None:
        for c in self.corners.values():
            c.connect()
        self.mode = "IDLE"

    def arm(self) -> None:
        for c in self.corners.values():
            c.arm()
        self._v = self._omega = 0.0
        self._last_set_ms = self._now_ms()
        self.mode = "ARMED"

    def set(self, v_mps: float, omega_rad_s: float) -> None:
        if self.mode != "ARMED":
            logger.warning("set() 무시: ARMED 아님 (mode=%s)", self.mode)
            return
        self._v = v_mps
        self._omega = omega_rad_s
        self._last_set_ms = self._now_ms()

    def disarm(self) -> None:
        for c in self.corners.values():
            c.disarm()
        self.mode = "IDLE"

    def estop(self) -> None:
        for c in self.corners.values():
            c.estop()
        self._v = self._omega = 0.0
        self.mode = "FAULT"

    def close(self) -> None:
        for c in self.corners.values():
            c.close()
        self.mode = "DISCONNECTED"

    # ── 50Hz 루프 ─────────────────────────────────────────────────────
    def tick(self) -> None:
        if self.mode != "ARMED":
            for c in self.corners.values():
                c.tick()                        # 비무장이어도 코너 통신 서비스
            return

        # estop 전파(사전): 이미 트립한 코너가 있으면 전체 정지
        if any(c.mode == "FAULT" for c in self.corners.values()):
            self.estop()
            return

        # 구동 허용 판정 — 워치독 + US-100 (조향은 항상 명령, 구동만 게이팅)
        drive_enabled = True
        if (self._last_set_ms is not None
                and (self._now_ms() - self._last_set_ms) > self.cfg.watchdog_ms):
            drive_enabled = False
        if self.monitor is not None:
            self.monitor.tick()
            self._verdict = self.monitor.verdict()
            if self._verdict is not None and self._verdict.level == "stop":
                drive_enabled = False

        # kinematics → 코너별 분배
        result = solve(self.cfg.geometry, self._v, self._omega)
        for w in self.cfg.geometry.wheels:
            wc = result.wheels[w.name]
            drive = wc.drive_turns_per_s if drive_enabled else 0.0
            self.corners[w.name].set(wc.steer_deg, drive)

        # 일괄 tick (각 코너가 자기 fault/과전류/stale/워치독 처리)
        for c in self.corners.values():
            c.tick()

        # estop 전파(사후): 이번 tick 에 트립한 코너가 있으면 전체 정지
        if any(c.mode == "FAULT" for c in self.corners.values()):
            self.estop()

    def state(self) -> dict:
        return {
            "mode": self.mode,
            "v": self._v,
            "omega": self._omega,
            "verdict": None if self._verdict is None else self._verdict.level,
            "corners": {n: c.state() for n, c in self.corners.items()},
        }

    def run(self, hz: float = None) -> None:
        """편의 제어 루프. 외부 루프가 tick() 을 직접 호출해도 된다."""
        period = 1.0 / (hz or self.cfg.loop_hz)
        while True:
            self.tick()
            time.sleep(period)
