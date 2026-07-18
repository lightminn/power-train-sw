from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

from .base import (Transport, TransportError, SIGNAL_META, ODRIVE_CONTROL_MODES,
                   ODRIVE_INPUTS, ODRIVE_TUNABLES_CAN, validate_gear_ratio)

# motor_control의 공통 runtime lock과 steering/ak_control.py를 재사용한다.
_MOTOR_CONTROL_DIR = Path(__file__).resolve().parents[3] / "motor_control"
sys.path.insert(0, str(_MOTOR_CONTROL_DIR))
sys.path.insert(0, str(_MOTOR_CONTROL_DIR / "steering"))
from ak_control import AK40 as AK  # noqa: E402

NODE_ID = 11                         # ODrive 실전 node (GUI 기본; 웹에서 변경 가능)
AK_ID = 1                            # AK45-36 조향 (GUI 기본; 웹에서 변경 가능)

# ODrive CANSimple cmd id (fw-v0.5.6)
C_HEARTBEAT = 0x001
C_ESTOP = 0x002
C_SET_STATE = 0x007
C_GET_ENC_EST = 0x009
C_SET_CTRL_MODE = 0x00B
C_SET_INPUT_POS = 0x00C
C_SET_INPUT_VEL = 0x00D
C_SET_INPUT_TORQUE = 0x00E
C_SET_LIMITS = 0x00F
C_GET_IQ = 0x014
C_GET_TEMP = 0x015
C_GET_BUS_VI = 0x017
C_CLEAR_ERR = 0x018
C_SET_POS_GAIN = 0x01A
C_SET_VEL_GAINS = 0x01B
C_SET_LINEAR_COUNT = 0x019
C_SET_TRAJ_VEL_LIMIT = 0x011
C_SET_TRAJ_ACCEL_LIMITS = 0x012

AXIS_IDLE = 1
AXIS_CLOSED_LOOP = 8
AXIS_FULL_CALIB = 3

# ODrive ControlMode / InputMode (fw-v0.5.6 정수값)
CTRL = {"position": 3, "position_traj": 3, "velocity": 2, "torque": 1}
# control_mode → 안전 기본 InputMode (POS_FILTER=3 / TRAP_TRAJ=5 / VEL_RAMP=2 / PASSTHROUGH=1) — jump 방지
IN_MODE = {"position": 3, "position_traj": 5, "velocity": 2, "torque": 1}

_ODRIVE_SIGNALS = [
    "odrive.pos", "odrive.vel", "odrive.iq_meas", "odrive.iq_set",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus", "odrive.state",
    "odrive.axis_err",
]
_AK_SIGNALS = ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp", "ak.fault"]


class CanBackend(Transport):
    """can0 위 ODrive(node1) CANSimple + AK(id10) servo 동시."""

    name = "can"

    def __init__(self, channel: str = "can0", gear_ratio: float = 5.0) -> None:
        self._channel = channel
        self._gear_ratio = validate_gear_ratio(gear_ratio)
        self._bus = None
        self._ak = None
        self._can_session = None
        self._node = NODE_ID            # ODrive node (웹에서 변경 가능)
        self._ak_id = AK_ID             # AK 모터 id (웹에서 변경 가능)
        self._state = {k: 0.0 for k in _ODRIVE_SIGNALS}
        self._pos_offset = 0.0
        # CAN Set_Limits/Set_Vel_Gains 는 페어 프레임 → 부분 업데이트 병합용 캐시
        self._last_limits: dict = {}
        self._last_vel_gains: dict = {}
        self._last_trap: dict = {}

    def connect(self) -> None:
        import can
        from chassis.runtime_lock import RealCanSession

        session = RealCanSession(
            channel=self._channel,
            owner="motor_gui_can_backend",
        )
        session.__enter__()
        try:
            self._bus = can.interface.Bus(channel=self._channel,
                                          interface="socketcan")
            self._ak = AK(self._bus, self._ak_id, name="steer")
        except BaseException as exc:
            if self._bus is not None:
                try:
                    self._bus.shutdown()
                except BaseException:
                    pass
            self._bus = None
            self._ak = None
            session.close()
            if isinstance(exc, OSError):
                raise TransportError(
                    f"{self._channel} open 실패 — "
                    f"'bash scripts/can_setup.sh' 먼저 ({exc})"
                ) from exc
            raise
        self._can_session = session

    def _send(self, cmd_id: int, data: bytes = b"") -> None:
        import can
        arb = (self._node << 5) | cmd_id
        self._bus.send(can.Message(arbitration_id=arb, data=data,
                                   is_extended_id=False))

    def _request(self, cmd_id: int) -> None:
        import can
        arb = (self._node << 5) | cmd_id
        self._bus.send(can.Message(arbitration_id=arb, is_remote_frame=True,
                                   is_extended_id=False))

    def sample(self) -> dict:
        self._request(C_GET_IQ)
        self._request(C_GET_TEMP)
        self._request(C_GET_BUS_VI)
        deadline = time.monotonic() + 0.008
        while time.monotonic() < deadline:
            msg = self._bus.recv(timeout=0.002)
            if msg is None:
                break
            self._decode_odrive(msg)
        self._ak.poll(timeout=0.005)
        s = {"t_mono": time.monotonic()}
        s.update(self._state)
        s["odrive.pos"] = float(self._state.get("odrive.pos", 0.0)) - self._pos_offset
        s["odrive.vel"] = float(self._state.get("odrive.vel", 0.0)) / self._gear_ratio
        s.update({
            "ak.pos_deg": self._ak.pos_out_deg,
            "ak.speed": float(self._ak.spd_erpm),
            "ak.current": self._ak.cur_a,
            "ak.temp": float(self._ak.temp_c),
            "ak.fault": int(self._ak.fault),
        })
        return s

    def _decode_odrive(self, msg) -> None:
        if msg.is_extended_id:
            return
        cmd = msg.arbitration_id & 0x1F
        node = msg.arbitration_id >> 5
        if node != self._node:
            return
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

    def apply(self, cmd: dict) -> dict:
        target, op, args = cmd["target"], cmd["op"], cmd.get("args", {})
        try:
            if target == "ak":
                return self._apply_ak(op, args)
            return self._apply_odrive(op, args)
        except Exception as e:
            return {"ok": False, "target": target, "op": op, "detail": str(e)}

    def _apply_odrive(self, op: str, args: dict) -> dict:
        known = {"estop", "set_mode", "set_input", "set_gain", "set_limit",
                 "set_state", "calibrate", "clear_errors", "set_origin"}
        if op not in known:
            return {"ok": False, "target": "odrive", "op": op,
                    "detail": "unsupported op"}
        if op == "estop":
            self._send(C_ESTOP)
        elif op == "set_mode":
            mode = args["control_mode"]
            self._send(C_SET_CTRL_MODE, struct.pack("<ii", CTRL[mode], IN_MODE[mode]))
            if mode in ("position", "position_traj"):
                cur = float(self._state.get("odrive.pos", 0.0))
                self._send(C_SET_INPUT_POS, struct.pack("<fhh", cur, 0, 0))
        elif op == "set_input":
            if "pos" in args:
                self._send(C_SET_INPUT_POS,
                           struct.pack("<fhh", float(args["pos"]) + self._pos_offset, 0, 0))
            elif "vel" in args:
                motor_vel = float(args["vel"]) * self._gear_ratio
                self._send(C_SET_INPUT_VEL, struct.pack("<ff", motor_vel, 0.0))
            elif "torque" in args:
                self._send(C_SET_INPUT_TORQUE, struct.pack("<f", float(args["torque"])))
        elif op == "set_gain":
            if "pos_gain" in args:
                self._send(C_SET_POS_GAIN, struct.pack("<f", float(args["pos_gain"])))
            if "vel_gain" in args or "vel_integrator_gain" in args:
                merged = dict(self._last_vel_gains)
                for k in ("vel_gain", "vel_integrator_gain"):
                    if k in args:
                        merged[k] = float(args[k])
                if "vel_gain" not in merged or "vel_integrator_gain" not in merged:
                    return {"ok": False, "target": "odrive", "op": op,
                            "detail": "CAN set_gain(vel) needs both vel_gain & "
                                      "vel_integrator_gain on first set"}
                self._send(C_SET_VEL_GAINS, struct.pack("<ff",
                           merged["vel_gain"], merged["vel_integrator_gain"]))
                self._last_vel_gains = merged
            if "trap_vel_limit" in args:
                self._send(C_SET_TRAJ_VEL_LIMIT,
                           struct.pack("<f", float(args["trap_vel_limit"])
                                       * self._gear_ratio))
            if "trap_accel_limit" in args or "trap_decel_limit" in args:
                for k in ("trap_accel_limit", "trap_decel_limit"):
                    if k in args:
                        self._last_trap[k] = float(args[k])
                if "trap_accel_limit" in self._last_trap and "trap_decel_limit" in self._last_trap:
                    self._send(C_SET_TRAJ_ACCEL_LIMITS, struct.pack("<ff",
                               self._last_trap["trap_accel_limit"] * self._gear_ratio,
                               self._last_trap["trap_decel_limit"] * self._gear_ratio))
        elif op == "set_limit":
            merged = dict(self._last_limits)
            for k in ("vel_limit", "current_lim"):
                if k in args:
                    merged[k] = float(args[k])
            if "vel_limit" not in merged or "current_lim" not in merged:
                return {"ok": False, "target": "odrive", "op": op,
                        "detail": "CAN set_limit needs both vel_limit & "
                                  "current_lim on first set"}
            self._send(C_SET_LIMITS, struct.pack("<ff",
                       merged["vel_limit"] * self._gear_ratio,
                       merged["current_lim"]))
            self._last_limits = merged
        elif op == "set_state":
            st = AXIS_CLOSED_LOOP if args.get("state") == "closed_loop" else AXIS_IDLE
            self._send(C_SET_STATE, struct.pack("<I", st))
        elif op == "calibrate":
            self._send(C_SET_STATE, struct.pack("<I", AXIS_FULL_CALIB))
        elif op == "clear_errors":
            self._send(C_CLEAR_ERR)
        elif op == "set_origin":
            cur = float(self._state.get("odrive.pos", 0.0))
            self._pos_offset = cur
            self._send(C_SET_INPUT_POS, struct.pack("<fhh", cur, 0, 0))  # 현재 위치 hold
        return {"ok": True, "target": "odrive", "op": op, "detail": "sent"}

    def _apply_ak(self, op: str, args: dict) -> dict:
        known = {"estop", "set_input", "set_origin"}
        if op not in known:
            return {"ok": False, "target": "ak", "op": op,
                    "detail": "unsupported op"}
        if op == "estop":
            self._ak.stop()
        elif op == "set_input":
            if "pos_deg" in args:
                self._ak.send_pos_out(float(args["pos_deg"]))
            elif "rpm" in args:
                self._ak.send_rpm_out(float(args["rpm"]))
        elif op == "set_origin":
            self._ak.set_origin_here()
        return {"ok": True, "target": "ak", "op": op, "detail": "sent"}

    def capabilities(self) -> dict:
        return {
            "track": "can",
            "devices": ["odrive", "ak"],
            "signals": _ODRIVE_SIGNALS + _AK_SIGNALS,
            "commands": {
                "odrive": ["set_mode", "set_input", "set_gain", "set_limit",
                           "set_state", "calibrate", "clear_errors",
                           "set_origin", "estop"],
                "ak": ["set_input", "set_origin", "estop"],
            },
            "limits": {"odrive": {"vel": 200.0 / self._gear_ratio,
                                    "torque": 10.0, "pos": 100.0},
                       "ak": {"pos_deg": 360.0}},
            "control_modes": {"odrive": ODRIVE_CONTROL_MODES},
            "inputs": {"odrive": ODRIVE_INPUTS},
            "tunables": {"odrive": ODRIVE_TUNABLES_CAN},
            "signal_meta": SIGNAL_META,
            "drive_gear_ratio": self._gear_ratio,
            "can_ids": self.device_ids(),
            "notes": ["CAN 트랙 — ODrive+AK 동시. NVM 저장 불가 (USB 전용)"],
        }

    def device_ids(self) -> dict:
        return {"odrive": {"id": self._node, "min": 0, "max": 63, "label": "ODrive node ID"},
                "ak": {"id": self._ak_id, "min": 1, "max": 127, "label": "AK 모터 ID"}}

    def set_device_ids(self, mapping: dict) -> None:
        if "odrive" in mapping:
            self._node = int(mapping["odrive"])
        if "ak" in mapping:
            self._ak_id = int(mapping["ak"])

    def read_tunables(self) -> dict:
        out = {}
        out.update(self._last_vel_gains)
        out.update(self._last_limits)
        out.update(self._last_trap)
        return out

    def close(self) -> None:
        try:
            if self._bus is not None:
                try:
                    self._send(C_SET_STATE, struct.pack("<I", AXIS_IDLE))
                    if self._ak is not None:
                        self._ak.stop()
                finally:
                    self._bus.shutdown()
        except Exception:
            pass
        finally:
            self._bus = None
            self._ak = None
            if self._can_session is not None:
                self._can_session.close()
                self._can_session = None
