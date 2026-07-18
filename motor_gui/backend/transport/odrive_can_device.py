from __future__ import annotations

import struct
import time

from .base import (SIGNAL_META, ODRIVE_INPUTS, ODRIVE_TUNABLES_CAN,
                   validate_gear_ratio)
from .can_device import CanDevice

NODE_ID = 11                             # ODrive axis1 실전 node (GUI 기본; 웹에서 변경 가능)

# ── CANSimple cmd id (fw-v0.5.6), arb = (node_id << 5) | cmd ──
C_HEARTBEAT = 0x001
C_ESTOP = 0x002
C_SET_STATE = 0x007
C_GET_ENC_EST = 0x009
C_SET_CTRL_MODE = 0x00B
C_SET_INPUT_POS = 0x00C
C_SET_INPUT_VEL = 0x00D
C_SET_LIMITS = 0x00F
C_SET_TRAJ_VEL_LIMIT = 0x011
C_SET_TRAJ_ACCEL_LIMITS = 0x012
C_GET_IQ = 0x014
C_GET_TEMP = 0x015
C_GET_BUS_VI = 0x017
C_CLEAR_ERR = 0x018
C_SET_LINEAR_COUNT = 0x019
C_SET_POS_GAIN = 0x01A
C_SET_VEL_GAINS = 0x01B

AXIS_IDLE = 1
AXIS_CLOSED_LOOP = 8
AXIS_FULL_CALIB = 3

# control_mode → (ControlMode, 기본 InputMode) fw-v0.5.6 정수.
# torque 모드 제외(CAN 으로 enable_current_mode_vel_limit 설정 불가 → runaway 위험).
_CTRL = {"position": 3, "position_traj": 3, "velocity": 2}
_IN_MODE = {"position": 3, "position_traj": 5, "velocity": 2}  # POS_FILTER/TRAP_TRAJ/VEL_RAMP
_CONTROL_MODES = ["position", "position_traj", "velocity"]

# trap_vel_limit 은 vel_limit 에 결합(헤드룸) → UI 튜너블에서 제거(windup 방지).
_BASE_TUNABLES = [t for t in ODRIVE_TUNABLES_CAN if t["key"] != "trap_vel_limit"]

_DEFAULT_KT = 0.0084                     # X2212-13 추정(8.27/980KV). UI 에서 편집 가능.

# RTR 폴링 빈도. 워커는 100Hz 로 돌지만 RTR 4개를 매 사이클(=400 frame/s) 쏘면
# 500kbps 버스에서도 ODrive ACK 누락 창에 tx-error 가 쌓여 bus-off 로 갈 수 있다
# (HIL 확인). 증명된 드라이브 스크립트처럼 저빈도로만 폴링해 호스트 TX 를 줄인다.
_POLL_HZ = 15.0
_POLL_PERIOD = 1.0 / _POLL_HZ

_SIGNALS = [
    "odrive.pos", "odrive.pos_setpoint", "odrive.vel", "odrive.vel_setpoint",
    "odrive.iq_meas", "odrive.iq_set", "odrive.torque_est",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus",
    "odrive.state", "odrive.axis_err",
]


class OdriveCanDevice(CanDevice):
    """ODrive 한 축 CANSimple 제어 (node1=axis1, fw-v0.5.6). 3모드, NVM 저장 불가."""

    name = "odrive"

    def __init__(self, node_id: int = NODE_ID, gear_ratio: float = 5.0) -> None:
        self._node = node_id
        self._gear_ratio = validate_gear_ratio(gear_ratio)
        self._bus = None
        self._state = {k: 0.0 for k in _SIGNALS}
        self._mode = "position"
        self._torque_const = _DEFAULT_KT
        self._vel_limit = None
        self._cur_lim = None
        self._pos_setpoint = 0.0
        self._vel_setpoint = 0.0
        # CAN 영점은 소프트 오프셋(raw - offset). 절대엔코더에서 Set_Linear_Count(CAN)
        # 가 인코더를 못 zero 해(HIL 확인) 모놀리식 can_bus.py 방식으로 회귀.
        self._pos_offset = 0.0
        self._last_poll = 0.0        # RTR 폴링 throttle 타임스탬프
        # pair-frame 명령(두 값을 한 프레임에) 부분 업데이트 병합용 캐시.
        self._vel_gains = {}
        self._trap = {}

    # ── 프레임 송신 헬퍼 ──
    def _arb(self, cmd: int) -> int:
        return (self._node << 5) | cmd

    def _send(self, cmd: int, data: bytes = b"") -> None:
        import can
        self._bus.send(can.Message(arbitration_id=self._arb(cmd), data=data,
                                   is_extended_id=False))

    def _request(self, cmd: int) -> None:
        # 텔레메트리 RTR 은 전송 실패(ENOBUFS/일시 bus-off)를 삼킨다 — 한 폴링이
        # 빠져도 다음 주기에 재시도. 예외가 sample() 을 죽여 recv-drain 을 건너뛰면
        # 텔레메트리 전체가 멎으므로(HIL 확인) 반드시 흡수. bus-off 는 restart-ms 가 복구.
        import can
        try:
            self._bus.send(can.Message(arbitration_id=self._arb(cmd),
                                       is_remote_frame=True, is_extended_id=False))
        except (can.CanError, OSError):
            pass

    def _send_input_pos(self, user_pos: float) -> None:
        """user 좌표(영점 기준) 목표를 raw(=user+offset)로 변환해 Set_Input_Pos 송신.
        setpoint 오버레이는 user 좌표(_pos_setpoint)로 추적."""
        self._pos_setpoint = user_pos
        self._send(C_SET_INPUT_POS, struct.pack("<fhh", user_pos + self._pos_offset, 0, 0))

    def _send_limits(self) -> None:
        """Set_Limits(vel_cap, current_lim) 1프레임. TRAP 은 캡에 헤드룸.

        주의(HIL): position_traj + 극저 vel_limit(예 1.0) + 높은 pos_gain(8.0)에선
        위치보정 항이 좁은 하드캡(vl×1.3)을 넘겨 overspeed(axis_err 0x200) 트립.
        TRAP 정밀 저속은 vel_limit ≥ 3 권장(또는 position/POS_FILTER 모드 사용).
        """
        if self._vel_limit is None or self._cur_lim is None:
            return
        if self._mode == "position_traj":
            cur = abs(float(self._state.get("odrive.vel", 0.0)))
            cap = max(self._vel_limit * self._gear_ratio * 1.3, cur * 1.3)
        else:
            cap = self._vel_limit * self._gear_ratio
        self._send(C_SET_LIMITS, struct.pack("<ff", cap, self._cur_lim))

    def _sync_vel_limit(self) -> None:
        """TRAP 순항=vel_limit + 컨트롤러 하드캡(헤드룸) 동기."""
        if self._vel_limit is None:
            return
        motor_vel_limit = self._vel_limit * self._gear_ratio
        self._send(C_SET_TRAJ_VEL_LIMIT, struct.pack("<f", motor_vel_limit))
        self._send_limits()

    def attach(self, bus) -> None:
        self._bus = bus
        self._state = {k: 0.0 for k in _SIGNALS}
        self._mode = "position"
        self._pos_setpoint = 0.0
        self._vel_setpoint = 0.0
        self._pos_offset = 0.0
        self._last_poll = 0.0        # 재연결 직후 첫 sample 에서 바로 폴링

    def can_id_spec(self) -> dict | None:
        return {"id": self._node, "min": 0, "max": 63, "label": "ODrive node ID"}

    def set_can_id(self, new_id: int) -> None:
        self._node = int(new_id)

    def capabilities_fragment(self) -> dict:
        meta = {k: SIGNAL_META[k] for k in _SIGNALS if k in SIGNAL_META}
        tunables = [dict(t) for t in _BASE_TUNABLES]
        tunables.append({
            "op": "set_param", "key": "torque_constant",
            "label": "토크 상수 Kt [Nm/A]", "value": self._torque_const,
            "help": "Iq→토크 환산용. 기본값은 X2212-13 추정(8.27/KV). "
                    "USB 트랙 모터정보 readout 값으로 교체 가능.",
        })
        return {
            "devices": ["odrive"],
            "signals": list(_SIGNALS),
            "commands": {"odrive": ["set_mode", "set_input", "set_gain",
                                    "set_limit", "set_state", "calibrate",
                                    "clear_errors", "set_param", "set_origin",
                                    "estop"]},
            "control_modes": {"odrive": list(_CONTROL_MODES)},
            "inputs": {"odrive": {m: ODRIVE_INPUTS[m] for m in _CONTROL_MODES}},
            "tunables": {"odrive": tunables},
            "limits": {"odrive": {"vel": 200.0 / self._gear_ratio,
                                    "pos": 100000.0}},
            "signal_meta": meta,
            "drive_gear_ratio": self._gear_ratio,
        }

    def request(self, bus) -> None:
        # 워커 100Hz 마다가 아니라 _POLL_HZ(~15Hz)로만 RTR 송신 → 호스트 TX 격감
        # (bus-off 방지). 비폴링 사이클엔 recv-drain 만 돌아 heartbeat·지연
        # 응답을 계속 수신하므로 명령 반응성은 100Hz 유지.
        now = time.monotonic()
        if now - self._last_poll < _POLL_PERIOD:
            return
        self._last_poll = now
        for c in (C_GET_ENC_EST, C_GET_IQ, C_GET_TEMP, C_GET_BUS_VI):
            self._request(c)

    def on_rx(self, msg) -> None:
        if msg.is_extended_id:
            return
        if (msg.arbitration_id >> 5) != self._node:
            return
        cmd = msg.arbitration_id & 0x1F
        d = msg.data
        if cmd == C_HEARTBEAT and len(d) >= 5:
            self._state["odrive.axis_err"] = struct.unpack("<I", d[:4])[0]
            self._state["odrive.state"] = d[4]
        elif cmd == C_GET_ENC_EST and len(d) >= 8:
            pos, vel = struct.unpack("<ff", d[:8])
            self._state["odrive.pos"] = pos
            self._state["odrive.vel"] = vel
        elif cmd == C_GET_IQ and len(d) >= 8:
            iq_set, iq_meas = struct.unpack("<ff", d[:8])
            self._state["odrive.iq_set"] = iq_set
            self._state["odrive.iq_meas"] = iq_meas
        elif cmd == C_GET_TEMP and len(d) >= 8:
            fet, _motor = struct.unpack("<ff", d[:8])
            self._state["odrive.temp_fet"] = fet
        elif cmd == C_GET_BUS_VI and len(d) >= 8:
            vbus, ibus = struct.unpack("<ff", d[:8])
            self._state["odrive.vbus"] = vbus
            self._state["odrive.ibus"] = ibus

    def sample(self) -> dict:
        s = dict(self._state)
        s["odrive.pos"] = float(self._state.get("odrive.pos", 0.0)) - self._pos_offset
        s["odrive.vel"] = float(self._state.get("odrive.vel", 0.0)) / self._gear_ratio
        s["odrive.pos_setpoint"] = self._pos_setpoint
        s["odrive.vel_setpoint"] = self._vel_setpoint
        s["odrive.torque_est"] = float(self._state.get("odrive.iq_meas", 0.0)) * self._torque_const
        return s

    def apply(self, bus, op: str, args: dict) -> dict:
        try:
            if op == "estop":
                self._send(C_ESTOP)
            elif op == "set_mode":
                mode = args.get("control_mode")
                if mode not in _CTRL:
                    return {"ok": False, "target": "odrive", "op": op,
                            "detail": f"unsupported mode {mode!r}"}
                self._mode = mode
                self._send(C_SET_CTRL_MODE, struct.pack("<ii", _CTRL[mode], _IN_MODE[mode]))
                self._sync_vel_limit()
                if mode in ("position", "position_traj"):
                    raw = float(self._state.get("odrive.pos", 0.0))
                    self._send_input_pos(raw - self._pos_offset)  # 현재 위치 hold(user 좌표)
                else:
                    self._vel_setpoint = 0.0
            elif op == "set_input":
                if "pos" in args:
                    self._send_input_pos(float(args["pos"]))
                elif "vel" in args:
                    self._vel_setpoint = float(args["vel"])
                    motor_vel = self._vel_setpoint * self._gear_ratio
                    self._send(C_SET_INPUT_VEL, struct.pack("<ff", motor_vel, 0.0))
                else:
                    return {"ok": False, "target": "odrive", "op": op,
                            "detail": "no known input key (pos/vel)"}
            elif op == "set_gain":
                if "pos_gain" in args:
                    self._send(C_SET_POS_GAIN, struct.pack("<f", float(args["pos_gain"])))
                if "vel_gain" in args or "vel_integrator_gain" in args:
                    for k in ("vel_gain", "vel_integrator_gain"):
                        if k in args:
                            self._vel_gains[k] = float(args[k])
                    if not all(k in self._vel_gains for k in
                               ("vel_gain", "vel_integrator_gain")):
                        return {"ok": False, "target": "odrive", "op": op,
                                "detail": "CAN set_gain(vel) needs both gains "
                                          "on first set"}
                    self._send(C_SET_VEL_GAINS, struct.pack("<ff",
                               self._vel_gains["vel_gain"],
                               self._vel_gains["vel_integrator_gain"]))
                if "trap_accel_limit" in args or "trap_decel_limit" in args:
                    for k in ("trap_accel_limit", "trap_decel_limit"):
                        if k in args:
                            self._trap[k] = float(args[k])
                    if not all(k in self._trap for k in
                               ("trap_accel_limit", "trap_decel_limit")):
                        return {"ok": False, "target": "odrive", "op": op,
                                "detail": "CAN set_gain(trap) needs both limits "
                                          "on first set"}
                    self._send(C_SET_TRAJ_ACCEL_LIMITS, struct.pack("<ff",
                               self._trap["trap_accel_limit"] * self._gear_ratio,
                               self._trap["trap_decel_limit"] * self._gear_ratio))
                # trap_vel_limit 은 무시(vel_limit 결합) — set_limit 에서만.
            elif op == "set_limit":
                if "vel_limit" in args:
                    self._vel_limit = float(args["vel_limit"])
                if "current_lim" in args:
                    self._cur_lim = float(args["current_lim"])
                if self._vel_limit is None or self._cur_lim is None:
                    return {"ok": False, "target": "odrive", "op": op,
                            "detail": "CAN set_limit needs both vel_limit and "
                                      "current_lim on first set"}
                self._sync_vel_limit()
                # TRAP 진행 중 캡 변경 시 setpoint 재발행 → 새 순항속도로 재계획.
                if (self._mode == "position_traj"
                        and int(self._state.get("odrive.state", 0)) == AXIS_CLOSED_LOOP):
                    self._send(C_SET_INPUT_POS, struct.pack(
                        "<fhh", self._pos_setpoint + self._pos_offset, 0, 0))
            elif op == "set_state":
                if args.get("state") == "closed_loop":
                    raw = float(self._state.get("odrive.pos", 0.0))
                    self._send_input_pos(raw - self._pos_offset)  # jump 방지(현재 위치 hold)
                    self._send(C_SET_STATE, struct.pack("<I", AXIS_CLOSED_LOOP))
                else:
                    self._send(C_SET_STATE, struct.pack("<I", AXIS_IDLE))
            elif op == "calibrate":
                self._send(C_SET_STATE, struct.pack("<I", AXIS_FULL_CALIB))
            elif op == "clear_errors":
                self._send(C_CLEAR_ERR)
            elif op == "set_param":
                if "torque_constant" in args:
                    self._torque_const = float(args["torque_constant"])
                else:
                    return {"ok": False, "target": "odrive", "op": op,
                            "detail": "no known param key (torque_constant)"}
            elif op == "set_origin":
                # 소프트 영점: 현재 raw 위치를 offset 으로 잡아 user 좌표를 0 으로.
                # (CAN Set_Linear_Count 가 절대엔코더를 zero 못 함 — HIL 확인.)
                # 모터는 안 움직임: user 0 = 현재 물리위치 hold.
                self._pos_offset = float(self._state.get("odrive.pos", 0.0))
                self._send_input_pos(0.0)
            else:
                return {"ok": False, "target": "odrive", "op": op,
                        "detail": "unsupported op"}
            return {"ok": True, "target": "odrive", "op": op, "detail": "sent"}
        except Exception as e:
            return {"ok": False, "target": "odrive", "op": op, "detail": str(e)}

    def close(self, bus) -> None:
        try:
            self._send(C_SET_STATE, struct.pack("<I", AXIS_IDLE))
        except Exception:
            pass
