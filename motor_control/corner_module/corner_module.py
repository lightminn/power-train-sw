"""코너 모듈 협조 제어기.

조향 액추에이터 1 + 구동 액추에이터 1 을 묶어 (조향각, 구동속도) 명령을
안전하게 적용한다. tick() 을 외부 루프가 주기적으로 호출하거나 run() 사용.
"""
import logging
import time

from corner_module.config import CornerConfig, clamp

logger = logging.getLogger(__name__)


class CornerModule:
    def __init__(self, steer, drive, cfg: CornerConfig, clock=None):
        self.steer = steer
        self.drive = drive
        self.cfg = cfg
        self.mode = "DISCONNECTED"
        self._steer_target = 0.0
        self._drive_target = 0.0
        self._last_set_ms = None
        self._now = clock or time.monotonic  # 테스트에서 주입 가능
        self._drive_enabled = True
        self._steer_enabled = True

    def _now_ms(self) -> float:
        return self._now() * 1000.0

    def connect(self) -> None:
        self.steer.connect()
        self.drive.connect()
        self.mode = "IDLE"

    def arm(self) -> None:
        if self._steer_enabled:
            self.steer.arm()
        if self._drive_enabled:
            self.drive.arm()
        # 점프 방지: 조향 목표=현재 실제각, 구동 목표=0
        if self._steer_enabled:
            self._steer_target = self.steer.state()["actual_deg"]
        self._drive_target = 0.0
        if self._steer_enabled:
            self.steer.set_angle(self._steer_target)
        if self._drive_enabled:
            self.drive.set_velocity(0.0)
        self._last_set_ms = self._now_ms()
        self.mode = "ARMED"

    def set_drive_enabled(self, enabled: bool) -> None:
        self._drive_enabled = bool(enabled)

    def set_steer_enabled(self, enabled: bool) -> None:
        self._steer_enabled = bool(enabled)

    def set(self, steer_deg: float, drive_vel: float) -> None:
        if self.mode != "ARMED":
            logger.warning("set() 무시: ARMED 아님 (mode=%s)", self.mode)
            return
        self._steer_target = clamp(steer_deg, self.cfg.steer_min_deg, self.cfg.steer_max_deg)
        self._drive_target = clamp(drive_vel, -self.cfg.drive_vel_limit, self.cfg.drive_vel_limit)
        self._last_set_ms = self._now_ms()

    def state(self) -> dict:
        return {
            "mode": self.mode,
            "steer": self.steer.state(),
            "drive": self.drive.state(),
            "faults": [],
            "drive_enabled": self._drive_enabled,
            "steer_enabled": self._steer_enabled,
        }

    def disarm(self) -> None:
        if self._drive_enabled:
            self.drive.set_velocity(0.0)
        if self._steer_enabled:
            self.steer.disarm()
        if self._drive_enabled:
            self.drive.disarm()
        self.mode = "IDLE"

    def estop(self) -> None:
        first_error = None
        try:
            actuators = []
            if self._steer_enabled:
                actuators.append(self.steer)
            if self._drive_enabled:
                actuators.append(self.drive)
            for actuator in actuators:
                try:
                    actuator.estop()
                except BaseException as exc:
                    if first_error is None:
                        first_error = exc
        finally:
            self._drive_target = 0.0
            self.mode = "FAULT"
        if first_error is not None:
            raise first_error

    def reset_fault(self) -> bool:
        if self.mode != "FAULT":
            return False
        self._drive_target = 0.0
        if self._drive_enabled:
            self.drive.set_velocity(0.0)
        self.mode = "IDLE"
        return True

    def close(self) -> None:
        self.steer.close()
        self.drive.close()
        self.mode = "DISCONNECTED"

    def _service_receive(self) -> None:
        for name, actuator in (("steer", self.steer), ("drive", self.drive)):
            try:
                actuator.state()
            except Exception:
                logger.debug("%s 유휴 수신 서비스 실패", name, exc_info=True)

    def tick(self) -> None:
        if self.mode != "ARMED":
            # IDLE/FAULT에서도 수신 버퍼를 drain해 health 캐시가 실시간을 반영한다.
            # 반응(estop/fault 판정)은 ARMED 전용 — 여기서는 캐시 갱신만.
            self._service_receive()
            return

        st = None
        if self._steer_enabled:
            st = self.steer.state()

            # 1) 조향 fault/전류 트립
            if st["fault"] != 0:
                logger.error("조향 fault=%s → estop", st["fault"])
                self.estop()
                return
            # 2) CAN stale
            if st.get("stale"):
                logger.error("조향 status stale → estop")
                self.estop()
                return
            # 조향 과전류 트립
            if abs(st["cur_a"]) > self.cfg.steer_current_limit_a:
                logger.error("조향 과전류 %.1fA > %.1fA → estop", st["cur_a"], self.cfg.steer_current_limit_a)
                self.estop()
                return

        if self._drive_enabled:
            drive_state = self.drive.state()
            if drive_state.get("stale", False):
                logger.error("구동 status stale → estop")
                self.estop()
                return
            if drive_state.get("axis_error", 0) != 0:
                logger.error("구동 axis_error=%s → estop", drive_state["axis_error"])
                self.estop()
                return

        # 3) 워치독: 입력 타임아웃 시 구동 0
        drive_cmd = self._drive_target
        if self._last_set_ms is not None and (self._now_ms() - self._last_set_ms) > self.cfg.watchdog_ms:
            drive_cmd = 0.0

        # 4) 협조 로직(옵션): 조향 따라오기 전 구동 자제
        if self._drive_enabled and self._steer_enabled and self.cfg.steer_gate:
            err = abs(self._steer_target - st["actual_deg"])
            if err > self.cfg.gate_deg:
                drive_cmd = 0.0

        # 5) 목표 push
        if self._steer_enabled:
            self.steer.set_angle(self._steer_target)
        if self._drive_enabled:
            self.drive.set_velocity(drive_cmd)
        if self._steer_enabled:
            self.steer.tick()
        if self._drive_enabled:
            self.drive.tick()

    def run(self, hz: float = None) -> None:
        """편의 제어 루프. 외부 루프가 tick() 을 직접 호출해도 된다."""
        rate = hz or self.cfg.loop_hz
        period = 1.0 / rate
        while True:
            self.tick()
            time.sleep(period)
