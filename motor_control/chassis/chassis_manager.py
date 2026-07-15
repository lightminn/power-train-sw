"""차체 통합 제어 — 코너모듈 6개를 하나의 "차체"로 묶는다 (WP3).

`set(v, ω)` 한 줄이면 kinematics 가 바퀴별 (조향각, 구동속도)를 풀고, 50Hz 루프
(`tick()`)가 그 결과를 각 CornerModule 에 분배해 일괄 구동한다. 안전은 차체 레벨에서
총괄:
  - estop 전파 : 코너 1곳이라도 트립(fault/과전류/stale)하면 6코너 전부 정지.
  - 안전 interlock : 임시 hold는 구동만 0으로 제어하고, ESTOP은 6코너에 latch한다.
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
from chassis.safety_interlock import RUN, SafetyInterlock
from chassis.telemetry import ChassisSnapshot, WheelSnapshot
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.null_steer import NullSteer

logger = logging.getLogger(__name__)
_COMMAND_RECOVERY_HOLD = "command_recovery"


# ── 설정 · 매핑 표 ────────────────────────────────────────────────────────


@dataclass
class ChassisConfig:
    geometry: ChassisGeometry = field(default_factory=default_geometry)
    corner: CornerConfig = field(default_factory=CornerConfig)
    watchdog_ms: float = 300.0         # chassis.set() 입력 타임아웃
    loop_hz: float = 50.0              # 제어 루프 주기
    min_drive_turns_per_s: float = 0.0  # 최저 구동속도(turns/s). 0<|명령|<이 값이면
                                        # 부호 유지하고 이 값으로 끌어올림 → 저속 HALL
                                        # 코깅존(툭툭 끊김·기동지연 제각각) 회피. 0=off.


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

#: 🛠️ **중륜 2개를 뺀 4륜 매핑** — 중간 ODrive 보드(node 13/14)를 부하모터(다이나모)에
#: 쓰고 있을 때. `kinematics.four_wheel_geometry()` 와 **반드시 짝으로** 쓴다
#: (기하와 매핑의 바퀴 이름이 어긋나면 KeyError).
#:
#: ⚠️ 이게 없으면 node 13/14 가 버스에 없어 **구동 status stale → 코너 FAULT →
#:    ChassisManager 가 전체 estop** 으로 전파한다(`CornerModule.tick()` 의 stale 검사).
FOUR_WHEEL_MAP = [
    WheelMap("front_left",  1, 11),
    WheelMap("front_right", 2, 12),
    WheelMap("rear_left",   3, 15),
    WheelMap("rear_right",  4, 16),
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
    지연 import(무하드웨어 pytest 가 python-can/odrive 없이 이 모듈을 쓰게).

    CAN 단독 소유권은 라이브러리 함수가 아니라 실물 실행 진입점의
    ``chassis.runtime_lock.RealCanSession``이 이 함수 호출 전에 획득한다. Fake/MuJoCo
    빌더가 이 함수와 lock에 결합되지 않도록 여기서는 버스 객체만 구성한다.
    """
    from corner_module.steer_ak40 import SteerAk40
    from corner_module.drive_odrive_can import DriveOdriveCan   # WP1 완료 필요
    return build_corners(
        steer_factory=lambda cid: SteerAk40(motor_id=cid, channel=channel),
        drive_factory=lambda nid: DriveOdriveCan(node_id=nid, channel=channel),
        cfg=cfg, wheel_map=wheel_map,
    )


# ── 차체 매니저 ───────────────────────────────────────────────────────────


class ChassisManager:
    def __init__(self, corners: dict, cfg: ChassisConfig = None, clock=None):
        self.cfg = cfg or ChassisConfig()
        self.corners = corners             # {wheel_name: CornerModule}
        self.mode = "DISCONNECTED"
        self._v = 0.0
        self._omega = 0.0
        self._last_set_ms = None
        self._speed_scale = 1.0            # 전방 감속 힌트 (1.0 = 제한 없음)
        self._now = time.monotonic if clock is None else clock
        self._interlock = SafetyInterlock(clock=self._now)
        self._last_estop_error = None

        # 매핑 검증: geometry 의 모든 바퀴가 코너로 존재해야 (오배선 조기 발견)
        need = {w.name for w in self.cfg.geometry.wheels}
        actual = set(self.corners)
        missing = need - actual
        unexpected = actual - need
        if missing or unexpected:
            raise ValueError(
                f"코너 매핑 오류: 누락={sorted(missing)}, "
                f"예상하지 않은 이름={sorted(unexpected)}"
            )

    def _now_ms(self) -> float:
        return self._now() * 1000.0

    # ── 라이프사이클 ──────────────────────────────────────────────────
    def connect(self) -> None:
        for c in self.corners.values():
            c.connect()
        self.mode = "IDLE"

    def arm(self) -> bool:
        if self.mode != "IDLE" or self._interlock.snapshot().estop_latched:
            return False
        for name, c in self.corners.items():
            try:
                c.arm()
            except BaseException as exc:
                detail = f"{name}: {type(exc).__name__}: {exc}"
                self.estop("arm_failure", detail)
                if isinstance(exc, Exception):
                    return False
                raise
        self._v = self._omega = 0.0
        self._last_set_ms = self._now_ms()
        self.mode = "ARMED"
        return True

    def set(self, v_mps: float, omega_rad_s: float) -> None:
        if self.mode != "ARMED":
            logger.warning("set() 무시: ARMED 아님 (mode=%s)", self.mode)
            return
        safety = self._interlock.snapshot()
        blocking_holds = set(safety.hold_sources) - {
            "cmd_watchdog",
            _COMMAND_RECOVERY_HOLD,
        }
        if blocking_holds:
            # HOLD 중 들어온 명령으로 현재 조향 목표를 덮으면 센서/링크가 회복되는
            # 순간 운전자의 새 의사 확인 없이 과거 명령이 재생될 수 있다. 입력의
            # 생존 시각만 갱신하고 명령은 폐기한다. cmd_watchdog HOLD와 내부
            # command_recovery HOLD는 새 set() 자체가 복구 신호이므로 예외다.
            self._last_set_ms = self._now_ms()
            logger.info(
                "set() 폐기: MOTION_HOLD 활성 (sources=%s)",
                ",".join(sorted(blocking_holds)),
            )
            return
        self._v = v_mps
        self._omega = omega_rad_s
        self._last_set_ms = self._now_ms()
        self._interlock.set_motion_hold(_COMMAND_RECOVERY_HOLD, False)

    def set_speed_scale(self, scale: float) -> None:
        """전방 감속 힌트 [0,1]. **인지 계층**(depth 장애물)이 주는 배율이다.

        ★ `set()` 이 아니라 **별도 채널**인 이유: `set()` 은 `_last_set_ms` 를 갱신해
          **명령 워치독**(300 ms)을 리셋한다. 감속 힌트를 `set()` 으로 밀어넣으면 상위
          명령이 끊겨도 워치독이 영영 안 터진다 = **stale 명령 재생**. 배율은 `tick()`
          에서 곱하므로 워치독 의미를 건드리지 않고, 힌트 변화가 다음 틱(20 ms)에 즉시 먹는다.

        🛑 **안전 게이트가 아니다.** 최종 게이팅은 US-100 + `SafetyInterlock`(`MOTION_HOLD`
           /`ESTOP`)이 한다. 이건 그 앞단의 **감속 힌트**일 뿐이며, depth 는 검은 물체·
           반사체에서 구멍이 나므로 단독으로 안전을 책임지지 않는다.
        """
        self._speed_scale = min(1.0, max(0.0, float(scale)))

    def disarm(self) -> None:
        for c in self.corners.values():
            c.disarm()
        if self.mode != "ESTOP":
            self.mode = "IDLE"

    def estop(self, source="manual", detail="") -> None:
        self._interlock.trip_estop(source, detail)
        self._v = self._omega = 0.0
        first_error = None
        for c in self.corners.values():
            try:
                c.estop()
            except BaseException as exc:
                if first_error is None:
                    first_error = exc
        if self._last_estop_error is None and first_error is not None:
            self._last_estop_error = first_error
        self.mode = "ESTOP"

    def reset_estop(self) -> bool:
        safety = self._interlock.snapshot()
        if not safety.estop_latched:
            return False
        if safety.active_estop_sources:
            return False

        reset_failed = False
        failure_detail = ""
        direct_error = None
        for name, c in self.corners.items():
            was_idle = c.mode == "IDLE"
            try:
                result = c.reset_fault()
            except BaseException as exc:
                reset_failed = True
                if not failure_detail:
                    failure_detail = f"{name}: {type(exc).__name__}: {exc}"
                if not isinstance(exc, Exception) and direct_error is None:
                    direct_error = exc
                continue
            if c.mode != "IDLE" or (
                result is not True and not (result is False and was_idle)
            ):
                reset_failed = True
                if not failure_detail:
                    failure_detail = (
                        f"{name}: reset_fault returned {result!r}, "
                        f"mode={c.mode}"
                    )

        if reset_failed:
            self.estop("reset_failure", failure_detail)
            if direct_error is not None:
                raise direct_error
            return False

        if not self._interlock.reset_estop():
            self.estop("reset_failure", "active E-stop condition")
            return False

        self._last_estop_error = None
        self.mode = "IDLE"
        return True

    def update_external_safety(self, status, estop_required, detail="") -> None:
        self.set_motion_hold("us100_checking", status == "CHECKING", detail)
        self._interlock.set_estop_condition(
            "us100", bool(estop_required), detail,
        )

    def set_motion_hold(self, source, active, detail="") -> None:
        """자동복구 가능한 정지 원인을 갱신한다.

        HOLD 진입 시 현재 조향 목표는 유지하되 구동 명령은 재사용 불가로 표시한다.
        원인이 사라진 뒤 새 set()이 오면 자동으로 RUN으로 돌아간다. E-stop의
        reset/arm 절차와 달리 재무장은 요구하지 않는다.
        """
        self._interlock.set_motion_hold(source, bool(active), detail)
        if active:
            self._interlock.set_motion_hold(
                _COMMAND_RECOVERY_HOLD,
                True,
                "fresh command required after motion hold",
            )

    def set_safety_link_stale(self, active, detail="") -> None:
        self._interlock.set_estop_condition(
            "safety_topic_stale", bool(active), detail,
        )

    def set_arm_motion_hold(self, active: bool, detail: str = "") -> None:
        """Apply the robot-arm final drive gate and discard its old command."""
        active = bool(active)
        was_active = "robot_arm" in self._interlock.snapshot().hold_sources
        if active and not was_active:
            self._v = self._omega = 0.0
        self._interlock.set_motion_hold("robot_arm", active, detail)

    def close(self) -> None:
        for c in self.corners.values():
            c.close()
        self.mode = "DISCONNECTED"

    # ── 50Hz 루프 ─────────────────────────────────────────────────────
    def tick(self) -> None:
        timed_out = (
            self._last_set_ms is not None
            and self._now_ms() - self._last_set_ms > self.cfg.watchdog_ms
        )
        self._interlock.set_motion_hold("cmd_watchdog", timed_out, "set timeout")
        safety = self._interlock.snapshot()
        if safety.estop_latched:
            if self.mode != "ESTOP":
                self.estop(safety.first_source or "estop", safety.first_detail)
            return

        if self.mode != "ARMED":
            for c in self.corners.values():
                c.tick()                        # 비무장이어도 코너 통신 서비스
            return

        # estop 전파(사전): 이미 트립한 코너가 있으면 전체 정지
        faulted = [name for name, c in self.corners.items() if c.mode == "FAULT"]
        if faulted:
            self.estop("corner_fault", ",".join(faulted))
            return

        # 조향은 항상 명령하고, hold 중에는 구동만 0으로 게이팅한다.
        drive_enabled = safety.state == RUN

        # 전방 감속 힌트 적용 (인지 계층 → set_speed_scale)
        #  · **전진에만** 건다. 앞에 장애물이 있다고 **후진을 막으면 안 된다** — 빠져나갈
        #    길을 막는 꼴이다.
        #  · **ω 는 안 줄인다.** 정지 상태에서도 조향·회전으로 회피할 수 있어야 한다.
        v_eff = self._v * self._speed_scale if self._v > 0.0 else self._v

        # kinematics → 코너별 분배
        result = solve(self.cfg.geometry, v_eff, self._omega)
        mn = self.cfg.min_drive_turns_per_s
        for w in self.cfg.geometry.wheels:
            wc = result.wheels[w.name]
            drive = wc.drive_turns_per_s if drive_enabled else 0.0
            if mn > 0.0 and 0.0 < abs(drive) < mn:      # 저속 코깅존 회피(부호 유지)
                drive = mn if drive > 0.0 else -mn
            self.corners[w.name].set(wc.steer_deg, drive)

        # 일괄 tick (각 코너가 자기 fault/과전류/stale/워치독 처리)
        for c in self.corners.values():
            c.tick()

        # estop 전파(사후): 이번 tick 에 트립한 코너가 있으면 전체 정지
        faulted = [name for name, c in self.corners.items() if c.mode == "FAULT"]
        if faulted:
            self.estop("corner_fault", ",".join(faulted))

    def snapshot(self) -> ChassisSnapshot:
        wheels = []
        for wheel in self.cfg.geometry.wheels:
            corner_state = self.corners[wheel.name].state()
            drive_state = corner_state.get("drive", {})
            steer_state = corner_state.get("steer", {})
            wheels.append(WheelSnapshot(
                name=wheel.name,
                corner_mode=str(corner_state.get("mode", "DISCONNECTED")),
                drive_turns_per_s=float(drive_state.get("actual_vel", 0.0)),
                steer_deg=float(steer_state.get("actual_deg", 0.0)),
                drive_current_a=float(drive_state.get("cur_a", 0.0)),
                steer_current_a=float(steer_state.get("cur_a", 0.0)),
                drive_stale=bool(drive_state.get("stale", False)),
                steer_stale=bool(steer_state.get("stale", False)),
                drive_axis_error=int(drive_state.get("axis_error", 0)),
                steer_fault=int(steer_state.get("fault", 0)),
            ))
        wheel_tuple = tuple(wheels)
        healthy = all(
            not wheel.drive_stale
            and not wheel.steer_stale
            and wheel.drive_axis_error == 0
            and wheel.steer_fault == 0
            for wheel in wheel_tuple
        )
        return ChassisSnapshot(
            chassis_mode=self.mode,
            stop_state=self._interlock.snapshot().state,
            healthy=healthy,
            wheels=wheel_tuple,
        )

    def state(self) -> dict:
        return {
            "mode": self.mode,
            "v": self._v,
            "omega": self._omega,
            "safety": self._interlock.snapshot(),
            "last_estop_error": self._last_estop_error,
            "corners": {n: c.state() for n, c in self.corners.items()},
        }

    def run(self, hz: float = None) -> None:
        """편의 제어 루프. 외부 루프가 tick() 을 직접 호출해도 된다."""
        period = 1.0 / (hz or self.cfg.loop_hz)
        while True:
            self.tick()
            time.sleep(period)
