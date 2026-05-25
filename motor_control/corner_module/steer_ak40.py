"""AK40-10 백엔드 SteerActuator. motor_control/steering/ak_control.py 재사용.

조향 출력축 각(도)을 그대로 받아 AK40 의 위치 명령으로 전달하고, 매 tick
status 를 폴링한다. AK40 내장 전류/fault 정보를 state 로 노출한다.
"""
import os
import sys
import time

import can

from corner_module.actuator import SteerActuator

_STEERING_DIR = os.path.join(os.path.dirname(__file__), "..", "steering")
sys.path.insert(0, os.path.abspath(_STEERING_DIR))
from ak_control import AK40  # noqa: E402


class SteerAk40(SteerActuator):
    def __init__(self, motor_id: int = 1, channel: str = "can0", stale_ms: float = 200.0):
        self._motor_id = motor_id
        self._channel = channel
        self._stale_ms = stale_ms
        self._bus = None
        self._ak = None
        self._target_deg = 0.0
        self._last_rx_ms = None

    def connect(self) -> None:
        try:
            self._bus = can.interface.Bus(channel=self._channel, interface="socketcan")
        except OSError as e:
            raise RuntimeError(
                f"can0 열기 실패({e}). 먼저 'bash scripts/can_setup.sh' 실행하세요."
            ) from e
        self._ak = AK40(self._bus, self._motor_id, name="steer")

    def arm(self) -> None:
        self._ak.poll(timeout=0.05)
        self._target_deg = self._ak.pos_out_deg

    def disarm(self) -> None:
        self._ak.stop()

    def set_angle(self, deg: float) -> None:
        self._target_deg = deg

    def tick(self) -> None:
        self._ak.send_pos_out(self._target_deg)
        got = self._ak.poll(timeout=0.005)
        if got:
            self._last_rx_ms = time.monotonic() * 1000.0

    def state(self) -> dict:
        stale = (
            self._last_rx_ms is None
            or (time.monotonic() * 1000.0 - self._last_rx_ms) > self._stale_ms
        )
        return {
            "target_deg": self._target_deg,
            "actual_deg": self._ak.pos_out_deg if self._ak else 0.0,
            "cur_a": self._ak.cur_a if self._ak else 0.0,
            "fault": self._ak.fault if self._ak else 0,
            "stale": stale,
        }

    def estop(self) -> None:
        if self._ak:
            self._ak.stop()

    def close(self) -> None:
        if self._ak:
            self._ak.stop()
        if self._bus:
            self._bus.shutdown()
