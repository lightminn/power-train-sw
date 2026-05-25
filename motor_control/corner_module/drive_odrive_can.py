"""ODrive 3.6(CAN) 백엔드 DriveActuator — 미래 CAN-only 전환용 인터페이스 예약.

케이블(Jetson–AK–ODrive 공통 CAN) 확보 후 구현한다. 구현 시 CANSimple
(NODE_ID<<5)|cmd, fw-v0.5.6, 현재위치 동기 점프방지를 따른다
(참조: motor_control/drive/x2212_test/odrive_can_drive.py).
"""
from corner_module.actuator import DriveActuator


class DriveOdriveCan(DriveActuator):
    def __init__(self, node_id: int = 1, channel: str = "can0"):
        self._node_id = node_id
        self._channel = channel

    def connect(self) -> None:
        raise NotImplementedError("DriveOdriveCan 미구현 — CAN-only 전환 시 구현")

    def arm(self) -> None:
        raise NotImplementedError

    def disarm(self) -> None:
        raise NotImplementedError

    def set_velocity(self, turns_per_s: float) -> None:
        raise NotImplementedError

    def tick(self) -> None:
        raise NotImplementedError

    def state(self) -> dict:
        raise NotImplementedError

    def estop(self) -> None:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError
