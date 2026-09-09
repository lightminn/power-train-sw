"""AK45-36 백엔드 SteerActuator (AK 시리즈 공용). motor_control/steering/ak_control.py 재사용.

차체 기준 조향각(+는 좌회전)을 AK 출력축 각으로 변환하고, 매 tick status 를
폴링한다. invert=True이면 지령과 피드백을 함께 반전한다.
(드라이버 클래스명은 레거시로 AK40 유지 — 프로파일로 AK45-36 동작.)

라이브러리 import/connect 경로는 owner lock을 만들지 않는다. 실물 CLI/노드 진입점이
connect 전에 ``chassis.runtime_lock.RealCanSession``을 잡고, 이 드라이버의 close 뒤에
session을 해제한다.
"""
import os
import sys
import time

import can

from corner_module.actuator import SteerActuator, FeedbackClock

_STEERING_DIR = os.path.join(os.path.dirname(__file__), "..", "steering")
sys.path.insert(0, os.path.abspath(_STEERING_DIR))
from ak_control import AK40, PKT_STATUS_1  # noqa: E402


class SteerAk40(SteerActuator):
    def __init__(self, motor_id: int = 1, channel: str = "can0",
                 stale_ms: float = 300.0, clock=None, invert: bool = False):
        self._motor_id = motor_id
        self._channel = channel
        self._stale_ms = stale_ms
        self._sign = -1.0 if invert else 1.0
        self._bus = None
        self._ak = None
        self._target_deg = 0.0
        self._last_rx_ms = None
        self._now = time.monotonic if clock is None else clock
        self._feedback_clock = FeedbackClock(self._now)
        self._feedback_rate_hz = 0.0
        self._rx_packets = 0
        self._recovery_count = 0
        self._control_tx_failures = 0

    def _now_ms(self) -> float:
        return self._now() * 1000.0

    def _record_feedback(self, received_ms=None) -> None:
        now_ms = self._now_ms() if received_ms is None else received_ms
        previous_ms = self._last_rx_ms
        if previous_ms is not None:
            interval_ms = now_ms - previous_ms
            if interval_ms > self._stale_ms:
                self._recovery_count += 1
                self._feedback_rate_hz = 0.0
            elif interval_ms > 0.0:
                self._feedback_rate_hz = 1000.0 / interval_ms
        self._last_rx_ms = now_ms
        self._rx_packets += 1

    def _receive_feedback(self) -> None:
        if self._ak is None:
            return
        if self._bus is None:
            # Legacy injected AK doubles have no socket. Production connect()
            # always creates a BusABC and takes the timestamped branch below.
            if self._ak.poll(timeout=0.0):
                self._record_feedback()
            return
        for _ in range(32):
            message = self._bus.recv(timeout=0.0)
            if message is None:
                break
            if (not message.is_extended_id or message.is_remote_frame
                    or message.is_error_frame
                    or message.arbitration_id != (PKT_STATUS_1 << 8) | self._motor_id
                    or len(message.data) < 8):
                continue
            received_ms = self._feedback_clock.received_ms(
                message.timestamp,
                allow_unstamped=not isinstance(self._bus, can.BusABC))
            if (received_ms is None or (self._last_rx_ms is not None
                                       and received_ms < self._last_rx_ms)):
                continue
            self._ak._parse_status(message.data)
            self._record_feedback(received_ms)

    def _require_sent(self, result) -> None:
        if result is False:
            self._control_tx_failures += 1
            raise can.CanOperationError(f"AK {self._motor_id} control send failed")

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
        # Use available feedback without blocking the chassis/input watchdog.
        # No reply must remain stale, never receive a synthetic fresh stamp.
        self._receive_feedback()
        self._target_deg = self._ak.pos_out_deg * self._sign

    def disarm(self) -> None:
        self.estop()

    def set_angle(self, deg: float) -> None:
        self._target_deg = deg

    def tick(self) -> None:
        self._require_sent(self._ak.send_pos_out(self._target_deg * self._sign))
        self._receive_feedback()

    def state(self) -> dict:
        # stale 판정 전에 커널 버퍼에 쌓인 status 를 논블로킹 드레인해 최신 수신
        # 시각을 반영한다. CornerModule.tick() 은 steer.tick()(수신 갱신) 전에
        # state() 로 stale 을 판정하므로, 다코너 순차 arm(코너당 ~0.2s) 뒤 첫
        # tick 처럼 마지막 poll 이후 stale_ms 가 지난 시점이면 실제 수신과 무관
        # 하게 트립해 버림 — connect() 의 CAN 필터 덕에 버퍼엔 자기 status(50Hz)
        # 만 쌓여 있어 timeout=0 드레인으로 즉시 회수된다.
        self._receive_feedback()
        return self.health_state()

    def health_state(self) -> dict:
        """Return cached health only; never receive or send CAN frames."""
        age_ms = (
            None
            if self._last_rx_ms is None
            else max(0.0, self._now_ms() - self._last_rx_ms)
        )
        stale = (
            age_ms is None
            or age_ms > self._stale_ms
        )
        return {
            "can_id": self._motor_id,
            "target_deg": self._target_deg,
            "actual_deg": self._ak.pos_out_deg * self._sign if self._ak else 0.0,
            "cur_a": self._ak.cur_a if self._ak else 0.0,
            "fault": self._ak.fault if self._ak else 0,
            "stale": stale,
            "last_feedback_age_ms": age_ms,
            "feedback_rate_hz": self._feedback_rate_hz,
            "rx_packets": self._rx_packets,
            "recovery_count": self._recovery_count,
            "control_tx_failures": self._control_tx_failures,
        }

    def estop(self) -> None:
        if self._ak:
            # E-stop 경로는 50 Hz 제어 tick 안에서 호출된다. AK40.stop()은
            # 5회 재전송 사이에 50 ms씩 대기하므로 AK 4개에서 전역 정지를
            # 약 1~2초 막는다. 첫 0 RPM 프레임을 즉시 보내고 반환하며,
            # 일반 disarm도 같은 executor에서 실행한다. 반복 정지는
            # 제어 루프 종료 후 close()에만 남긴다.
            self._require_sent(self._ak.send_rpm_out(0))

    def close(self) -> None:
        if self._ak:
            self._ak.stop()
        if self._bus:
            self._bus.shutdown()
