"""AK45-36 백엔드 SteerActuator (AK 시리즈 공용). motor_control/steering/ak_control.py 재사용.

조향 출력축 각(도)을 그대로 받아 AK 의 위치 명령으로 전달하고, 매 tick
status 를 폴링한다. 모터 내장 전류/fault 정보를 state 로 노출한다.
(드라이버 클래스명은 레거시로 AK40 유지 — 프로파일로 AK45-36 동작.)

라이브러리 import/connect 경로는 owner lock을 만들지 않는다. 실물 CLI/노드 진입점이
connect 전에 ``chassis.runtime_lock.RealCanSession``을 잡고, 이 드라이버의 close 뒤에
session을 해제한다.
"""
import os
import sys
import time

import can

from corner_module.actuator import SteerActuator

_STEERING_DIR = os.path.join(os.path.dirname(__file__), "..", "steering")
sys.path.insert(0, os.path.abspath(_STEERING_DIR))
from ak_control import AK40, PKT_STATUS_1  # noqa: E402


class SteerAk40(SteerActuator):
    def __init__(self, motor_id: int = 1, channel: str = "can0", stale_ms: float = 300.0):
        self._motor_id = motor_id
        self._channel = channel
        self._stale_ms = stale_ms
        self._bus = None
        self._ak = None
        self._target_deg = 0.0
        self._last_rx_ms = None

    def connect(self) -> None:
        # 자기 AK 의 STATUS_1(ext arb (41<<8)|id) 만 받는 필터 — 단일 can0 다중모터에서
        # ODrive RTR 등 타 프레임에 status 가 묻혀 poll 이 굶는(→false stale→estop) 걸 방지.
        flt = [{"can_id": (PKT_STATUS_1 << 8) | self._motor_id,
                "can_mask": 0xFFFF, "extended": True}]
        try:
            self._bus = can.interface.Bus(channel=self._channel, interface="socketcan",
                                          can_filters=flt)
        except OSError as e:
            raise RuntimeError(
                f"can0 열기 실패({e}). 먼저 'bash scripts/can_setup.sh' 실행하세요."
            ) from e
        self._ak = AK40(self._bus, self._motor_id, name="steer")

    def arm(self) -> None:
        # poll 성공 시 수신시각 기록 → arm 직후 state()가 stale 로 오판해
        # CornerModule.tick() 첫 호출에서 estop 되는 것을 방지.
        if self._ak.poll(timeout=0.1):
            self._last_rx_ms = time.monotonic() * 1000.0
        self._target_deg = self._ak.pos_out_deg

    def disarm(self) -> None:
        self._ak.stop()

    def set_angle(self, deg: float) -> None:
        self._target_deg = deg

    def tick(self) -> None:
        self._ak.send_pos_out(self._target_deg)
        got = self._ak.poll(timeout=0.0)
        if got:
            self._last_rx_ms = time.monotonic() * 1000.0

    def state(self) -> dict:
        # stale 판정 전에 커널 버퍼에 쌓인 status 를 논블로킹 드레인해 최신 수신
        # 시각을 반영한다. CornerModule.tick() 은 steer.tick()(수신 갱신) 전에
        # state() 로 stale 을 판정하므로, 다코너 순차 arm(코너당 ~0.2s) 뒤 첫
        # tick 처럼 마지막 poll 이후 stale_ms 가 지난 시점이면 실제 수신과 무관
        # 하게 트립해 버림 — connect() 의 CAN 필터 덕에 버퍼엔 자기 status(50Hz)
        # 만 쌓여 있어 timeout=0 드레인으로 즉시 회수된다.
        if self._ak is not None and self._ak.poll(timeout=0.0):
            self._last_rx_ms = time.monotonic() * 1000.0
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
            # E-stop 경로는 50 Hz 제어 tick 안에서 호출된다. AK40.stop()은
            # 5회 재전송 사이에 50 ms씩 대기하므로 AK 4개에서 전역 정지를
            # 약 1~2초 막는다. 첫 0 RPM 프레임을 즉시 보내고 반환하며,
            # 반복 정지는 시간 제약이 없는 disarm()/close()에만 남긴다.
            self._ak.send_rpm_out(0)

    def close(self) -> None:
        if self._ak:
            self._ak.stop()
        if self._bus:
            self._bus.shutdown()
