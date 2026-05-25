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

    def _now_ms(self) -> float:
        return self._now() * 1000.0

    def connect(self) -> None:
        self.steer.connect()
        self.drive.connect()
        self.mode = "IDLE"

    def arm(self) -> None:
        self.steer.arm()
        self.drive.arm()
        # 점프 방지: 조향 목표=현재 실제각, 구동 목표=0
        self._steer_target = self.steer.state()["actual_deg"]
        self._drive_target = 0.0
        self.steer.set_angle(self._steer_target)
        self.drive.set_velocity(0.0)
        self._last_set_ms = self._now_ms()
        self.mode = "ARMED"

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
        }

    def disarm(self) -> None:
        self.drive.set_velocity(0.0)
        self.steer.disarm()
        self.drive.disarm()
        self.mode = "IDLE"

    def estop(self) -> None:
        self.steer.estop()
        self.drive.estop()
        self._drive_target = 0.0
        self.mode = "FAULT"

    def close(self) -> None:
        self.steer.close()
        self.drive.close()
        self.mode = "DISCONNECTED"

    def tick(self) -> None:
        if self.mode != "ARMED":
            self.steer.tick()
            self.drive.tick()
            return

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

        # 3) 워치독: 입력 타임아웃 시 구동 0
        drive_cmd = self._drive_target
        if self._last_set_ms is not None and (self._now_ms() - self._last_set_ms) > self.cfg.watchdog_ms:
            drive_cmd = 0.0

        # 4) 협조 로직(옵션): 조향 따라오기 전 구동 자제
        if self.cfg.steer_gate:
            err = abs(self._steer_target - st["actual_deg"])
            if err > self.cfg.gate_deg:
                drive_cmd = 0.0

        # 5) 목표 push
        self.steer.set_angle(self._steer_target)
        self.drive.set_velocity(drive_cmd)
        self.steer.tick()
        self.drive.tick()
