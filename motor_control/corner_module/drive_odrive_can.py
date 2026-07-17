"""ODrive 3.6(CAN) 백엔드 DriveActuator — CANSimple over socketcan.

DriveOdriveUsb 와 **동일 계약**을 CAN 으로 구현한다: velocity control,
폐루프 진입 시 input_vel=0 으로 점프 방지, ``state() → {target_vel,
actual_vel, cur_a}``. CornerModule 은 USB/CAN/Fake 를 교체해도 동일하게 동작.

프레임 포맷은 ``motor_control/drive/bl70200/can_drive_test.py`` 에서 10모터
단일 can0(500k) 실기 검증됨. 명령은 CANSimple ``(node_id<<5)|cmd`` (fw 0.5.x):

    0x07 Set_Axis_State        <I  (state)        8=CLOSED_LOOP 1=IDLE
    0x0B Set_Controller_Mode   <ii (ctrl,input)   2=VELOCITY, 1=PASSTHROUGH
    0x0D Set_Input_Vel         <ff (vel, tq_ff)
    0x18 Clear_Errors          (8 bytes)
    0x01 Heartbeat  (수신)      <I err, byte4 state   ← 주기 방송
    0x09 Get_Encoder_Estimates (RTR) → <ff (pos, vel)
    0x14 Get_Iq                (RTR) → <ff (iq_sp, iq_measured)

이 fw 는 pos/vel/Iq 를 자동 방송하지 않으므로(heartbeat 만 주기) tick 마다
RTR 로 폴링한다. steer_ak40 와 동일하게 각 드라이버가 자체 socketcan 소켓을
연다 — SocketCAN 브로드캐스트라 소켓마다 전 프레임을 받고, node 필터로
자기 노드만 수신한다(단일 can0 다중 모터 공존).

이 모듈은 라이브러리 드라이버라 import/connect 자체에서 owner lock을 잡지 않는다.
실물 CLI/노드 진입점이 connect 전에 ``chassis.runtime_lock.RealCanSession``을 잡고,
모든 ``DriveOdriveCan.close()`` 뒤에 session을 해제해야 한다.
"""
import struct
import time

import can

from corner_module.actuator import DriveActuator

# CANSimple command ids (fw 0.5.x)
_HEARTBEAT = 0x01
_SET_AXIS_STATE = 0x07
_GET_ENCODER_ESTIMATES = 0x09
_SET_CONTROLLER_MODE = 0x0B
_SET_INPUT_VEL = 0x0D
_GET_IQ = 0x14
_CLEAR_ERRORS = 0x18

_AXIS_IDLE = 1
_AXIS_CLOSED_LOOP = 8
_CTRL_VELOCITY = 2
_INPUT_PASSTHROUGH = 1


class DriveOdriveCan(DriveActuator):
    """ODrive 3.6 CAN 구동 드라이버 — velocity control, 노드별 socketcan.

    Parameters
    ----------
    node_id:
        ODrive CANSimple node id (구동 11~16).
    channel:
        socketcan 채널명(기본 ``can0``).
    stale_ms:
        마지막 수신 후 이 시간(ms) 넘게 프레임이 없으면 ``state()["stale"]=True``.
    bus:
        미리 연 ``can.BusABC`` 를 주입하면 ``connect()`` 가 이를 재사용한다
        (주로 단위 테스트용 — 각 드라이버가 독립 소켓을 갖는 게 기본).
    friction_ff:
        저속 마찰/코깅 보상 피드포워드(raw torque_ff 단위 — fw 0.5.1 벤치 확정 전).
        0 < |target| < v_knee 일 때만 부호 추종으로 Set_Input_Vel 2번째 필드에 실린다.
        0.0(기본) = off. 스펙 r6 §2.2b(D4).
    v_knee:
        friction_ff 적용 상한(turns/s, 기본 0.5). 경계값은 포함하지 않는다.
    """

    def __init__(self, node_id: int = 11, channel: str = "can0",
                 stale_ms: float = 200.0, bus=None, clock=None,
                 friction_ff: float = 0.0, v_knee: float = 0.5):
        self._node_id = node_id
        self._channel = channel
        self._stale_ms = stale_ms
        self._bus = bus
        self._owns_bus = bus is None
        self._friction_ff = max(0.0, float(friction_ff))
        self._v_knee = max(0.0, float(v_knee))
        self._target_vel = 0.0
        self._actual_vel = 0.0
        self._cur_a = 0.0
        self._axis_error = 0
        self._axis_state = 0
        self._last_rx_ms = None
        self._last_heartbeat_ms = None
        self._last_encoder_ms = None
        self._now = time.monotonic if clock is None else clock
        self._rx_packets = 0
        self._recovery_count = 0

    # ------------------------------------------------------------------
    # 내부 헬퍼
    # ------------------------------------------------------------------
    def _arb(self, cmd: int) -> int:
        return (self._node_id << 5) | cmd

    def _now_ms(self) -> float:
        return self._now() * 1000.0

    def _send(self, cmd: int, data: bytes = b"", rtr: bool = False) -> None:
        # 노드가 버스에 없어 ACK 못 받으면 TX 큐가 차 ENOBUFS(CanOperationError) →
        # 프레임 드롭하고 계속(제어루프가 죽지 않게). 미수신은 state()의 stale 로 드러남.
        try:
            self._bus.send(can.Message(arbitration_id=self._arb(cmd), data=data,
                                       is_extended_id=False, is_remote_frame=rtr))
        except can.CanError:
            pass

    def _set_axis_state(self, state: int) -> None:
        # 검증된 프레임과 동일하게 8바이트로 패딩(can_drive_test.py)
        self._send(_SET_AXIS_STATE, struct.pack("<I", state) + bytes(4))

    def _handle_rx(self, m) -> bool:
        if m.is_extended_id or m.is_remote_frame:
            return False
        if (m.arbitration_id >> 5) != self._node_id:
            return False
        cmd = m.arbitration_id & 0x1F
        if cmd == _HEARTBEAT and len(m.data) >= 5:
            self._axis_error = struct.unpack("<I", m.data[0:4])[0]
            self._axis_state = m.data[4]
            kind = "heartbeat"
        elif cmd == _GET_ENCODER_ESTIMATES and len(m.data) >= 8:
            self._actual_vel = struct.unpack("<ff", m.data[0:8])[1]
            kind = "encoder"
        elif cmd == _GET_IQ and len(m.data) >= 8:
            self._cur_a = struct.unpack("<ff", m.data[0:8])[1]
            kind = "iq"
        else:
            return False
        now_ms = self._now_ms()
        if (
            self._last_rx_ms is not None
            and now_ms - self._last_rx_ms > self._stale_ms
        ):
            self._recovery_count += 1
        self._last_rx_ms = now_ms
        if kind == "heartbeat":
            self._last_heartbeat_ms = now_ms
        elif kind == "encoder":
            self._last_encoder_ms = now_ms
        self._rx_packets += 1
        return True

    def _drain_available(self, max_frames: int = 16) -> int:
        handled = 0
        for _ in range(max_frames):
            msg = self._bus.recv(timeout=0.0)
            if msg is None:
                break
            if self._handle_rx(msg):
                handled += 1
        return handled

    def _poll(self, timeout: float) -> None:
        """timeout 초 동안 자기 노드 프레임을 드레인해 텔레메트리 캐시 갱신."""
        deadline = time.monotonic() + timeout
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                break
            m = self._bus.recv(timeout=remaining)
            if m is None:
                break
            self._handle_rx(m)

    # ------------------------------------------------------------------
    # Actuator 인터페이스
    # ------------------------------------------------------------------
    def connect(self) -> None:
        if self._bus is not None:
            return                                   # 주입된 버스 재사용
        flt = [{"can_id": self._node_id << 5, "can_mask": 0x7E0, "extended": False}]
        try:
            self._bus = can.interface.Bus(channel=self._channel, interface="socketcan",
                                          can_filters=flt)
        except OSError as e:
            raise RuntimeError(
                f"can0 열기 실패({e}). 먼저 'bash scripts/can_setup.sh' 실행하세요."
            ) from e

    def arm(self) -> None:
        """velocity-control + passthrough 로 폐루프 진입(input_vel=0 점프 방지)."""
        self._send(_CLEAR_ERRORS, bytes(8))
        self._send(_SET_CONTROLLER_MODE, struct.pack("<ii", _CTRL_VELOCITY, _INPUT_PASSTHROUGH))
        self._send(_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0))
        self._target_vel = 0.0
        self._set_axis_state(_AXIS_CLOSED_LOOP)
        self._poll(0.1)                              # arm 직후 stale 오판 방지: last_rx 시드

    def disarm(self) -> None:
        self._send(_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0))
        self._set_axis_state(_AXIS_IDLE)
        self._target_vel = 0.0

    def set_velocity(self, turns_per_s: float) -> None:
        """다음 tick() 에 전송할 목표 속도(turns/s)."""
        self._target_vel = turns_per_s

    def _friction_torque_ff(self) -> float:
        t = self._target_vel
        if self._friction_ff > 0.0 and 0.0 < abs(t) < self._v_knee:
            return self._friction_ff if t > 0.0 else -self._friction_ff
        return 0.0

    def tick(self) -> None:
        """제어 루프마다: 목표 속도(+저속 마찰 보상 ff) 전송 + RTR 폴링."""
        self._drain_available()
        self._send(_SET_INPUT_VEL,
                   struct.pack("<ff", self._target_vel, self._friction_torque_ff()))
        self._send(_GET_ENCODER_ESTIMATES, rtr=True)
        self._send(_GET_IQ, rtr=True)

    def state(self) -> dict:
        """정규화 텔레메트리. CornerModule 계약 키 + CAN 건강(stale/axis_error)."""
        # 6코너를 순차 arm 하는 동안 먼저 arm 된 축의 heartbeat는 커널
        # 수신 버퍼에 계속 쌓인다. 캐시 시각만 보고 stale을 판정하면 실제
        # 응답 중인 앞쪽 축까지 오탐하므로, 조향 드라이버와 동일하게
        # 비블로킹·유한 드레인 후 최신 heartbeat를 반영한다.
        if self._bus is not None:
            self._drain_available()
        return self.health_state()

    def health_state(self) -> dict:
        """Return cached health only; never receive or send CAN frames."""
        now_ms = self._now_ms()

        def age(timestamp_ms):
            if timestamp_ms is None:
                return None
            return max(0.0, now_ms - timestamp_ms)

        last_rx_age_ms = age(self._last_rx_ms)
        stale = last_rx_age_ms is None or last_rx_age_ms > self._stale_ms
        return {
            "node_id": self._node_id,
            "target_vel": self._target_vel,
            "actual_vel": self._actual_vel,
            "cur_a": self._cur_a,
            "axis_error": self._axis_error,
            "axis_state": self._axis_state,
            "stale": stale,
            "last_heartbeat_age_ms": age(self._last_heartbeat_ms),
            "last_encoder_age_ms": age(self._last_encoder_ms),
            "rx_packets": self._rx_packets,
            "recovery_count": self._recovery_count,
        }

    def estop(self) -> None:
        """즉시 정지 — input_vel=0 후 IDLE."""
        if self._bus is not None:
            self._send(_SET_INPUT_VEL, struct.pack("<ff", 0.0, 0.0))
            self._set_axis_state(_AXIS_IDLE)
        self._target_vel = 0.0

    def close(self) -> None:
        """IDLE 로 내리고, 소유한 버스면 정리."""
        if self._bus is None:
            return
        try:
            self._set_axis_state(_AXIS_IDLE)
        finally:
            if self._owns_bus:
                self._bus.shutdown()
                self._bus = None
