"""하드웨어 없는 단위 테스트용 Fake 액추에이터.

명령을 받아 1차 지연(매 tick 오차의 50% 수렴)으로 actual 이 target 에
수렴하는 단순 모델. arm 전/disarm 상태에서는 움직이지 않는다.
"""
from corner_module.actuator import SteerActuator, DriveActuator

_STEP = 0.5  # 매 tick 수렴 비율


class FakeSteer(SteerActuator):
    def __init__(self, start_deg: float = 0.0):
        self._target = start_deg
        self._actual = start_deg
        self._armed = False
        self._connected = False
        self.cur_a = 0.0
        self.fault = 0
        self.stale_flag = False

    def connect(self) -> None:
        self._connected = True

    def arm(self) -> None:
        self._armed = True
        self._target = self._actual  # 점프 방지

    def disarm(self) -> None:
        self._armed = False

    def set_angle(self, deg: float) -> None:
        self._target = deg

    def tick(self) -> None:
        if self._armed:
            self._actual += (self._target - self._actual) * _STEP

    def state(self) -> dict:
        return {
            "target_deg": self._target,
            "actual_deg": self._actual,
            "cur_a": self.cur_a,
            "fault": self.fault,
            "stale": self.stale_flag,
        }

    def estop(self) -> None:
        self._target = self._actual
        self._armed = False

    def close(self) -> None:
        self._connected = False


class FakeDrive(DriveActuator):
    def __init__(self, start_vel: float = 0.0):
        self._target = 0.0
        self._actual = start_vel
        self._armed = False
        self._connected = False
        self.cur_a = 0.0
        self.stale_flag = False
        self.axis_error = 0

    def connect(self) -> None:
        self._connected = True

    def arm(self) -> None:
        self._armed = True
        self._target = 0.0  # 점프 방지: 0 속도로 진입

    def disarm(self) -> None:
        self._armed = False

    def set_velocity(self, turns_per_s: float) -> None:
        self._target = turns_per_s

    def tick(self) -> None:
        if self._armed:
            self._actual += (self._target - self._actual) * _STEP
        else:
            self._actual = 0.0

    def state(self) -> dict:
        return {
            "target_vel": self._target,
            "actual_vel": self._actual,
            "cur_a": self.cur_a,
            "stale": self.stale_flag,
            "axis_error": self.axis_error,
        }

    def estop(self) -> None:
        self._target = 0.0
        self._actual = 0.0
        self._armed = False

    def close(self) -> None:
        self._connected = False
