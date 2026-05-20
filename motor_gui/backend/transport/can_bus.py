from __future__ import annotations

import struct
import sys
import time
from pathlib import Path

from .base import (Transport, TransportError, SIGNAL_META, ODRIVE_CONTROL_MODES,
                   ODRIVE_INPUTS, ODRIVE_TUNABLES_CAN)

# steering/ak_control.py 의 AK40 클래스 재사용 (hw 로직 단일 소스)
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "motor_control" / "steering"))
from ak_control import AK40 as AK  # noqa: E402

NODE_ID = 1                          # ODrive
AK_ID = 10                           # AK 조향

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

AXIS_IDLE = 1
AXIS_CLOSED_LOOP = 8
AXIS_FULL_CALIB = 3

# ODrive ControlMode / InputMode (fw-v0.5.6 정수값)
CTRL = {"position": 3, "velocity": 2, "torque": 1}
# control_mode → 안전 기본 InputMode (POS_FILTER=3 / VEL_RAMP=2 / PASSTHROUGH=1) — jump 방지
IN_MODE = {"position": 3, "velocity": 2, "torque": 1}

_ODRIVE_SIGNALS = [
    "odrive.pos", "odrive.vel", "odrive.iq_meas", "odrive.iq_set",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus", "odrive.state",
    "odrive.axis_err",
]
_AK_SIGNALS = ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp", "ak.fault"]


class CanBackend(Transport):
    """can0 위 ODrive(node1) CANSimple + AK(id10) servo 동시."""

    name = "can"

    def __init__(self, channel: str = "can0") -> None:
        self._channel = channel
        self._bus = None
        self._ak = None
        self._state = {k: 0.0 for k in _ODRIVE_SIGNALS}
        self._pos_offset = 0.0
        # CAN Set_Limits/Set_Vel_Gains 는 페어 프레임 → 부분 업데이트 병합용 캐시
        self._last_limits: dict = {}
        self._last_vel_gains: dict = {}

    def connect(self) -> None:
        import can
        try:
            self._bus = can.interface.Bus(channel=self._channel,
                                          interface="socketcan")
        except OSError as e:
            raise TransportError(
                f"{self._channel} open 실패 — 'bash scripts/can_setup.sh' 먼저 ({e})")
        self._ak = AK(self._bus, AK_ID, name="steer")

    def _send(self, cmd_id: int, data: bytes = b"") -> None:
        import can
        arb = (NODE_ID << 5) | cmd_id
        self._bus.send(can.Message(arbitration_id=arb, data=data,
                                   is_extended_id=False))

    def _request(self, cmd_id: int) -> None:
        import can
        arb = (NODE_ID << 5) | cmd_id
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
        if node != NODE_ID:
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
            if mode == "position":
                cur = float(self._state.get("odrive.pos", 0.0))
                self._send(C_SET_INPUT_POS, struct.pack("<fhh", cur, 0, 0))
        elif op == "set_input":
            if "pos" in args:
                self._send(C_SET_INPUT_POS,
                           struct.pack("<fhh", float(args["pos"]) + self._pos_offset, 0, 0))
            elif "vel" in args:
                self._send(C_SET_INPUT_VEL, struct.pack("<ff", float(args["vel"]), 0.0))
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
                       merged["vel_limit"], merged["current_lim"]))
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
            "limits": {"odrive": {"vel": 200.0, "torque": 10.0, "pos": 100.0},
                       "ak": {"pos_deg": 360.0}},
            "control_modes": {"odrive": ODRIVE_CONTROL_MODES},
            "inputs": {"odrive": ODRIVE_INPUTS},
            "tunables": {"odrive": ODRIVE_TUNABLES_CAN},
            "signal_meta": SIGNAL_META,
            "notes": ["CAN 트랙 — ODrive+AK 동시. NVM 저장 불가 (USB 전용)"],
        }

    def read_tunables(self) -> dict:
        out = {}
        out.update(self._last_vel_gains)
        out.update(self._last_limits)
        return out

    def close(self) -> None:
        try:
            if self._bus is not None:
                self._send(C_SET_STATE, struct.pack("<I", AXIS_IDLE))
                if self._ak is not None:
                    self._ak.stop()
                self._bus.shutdown()
        except Exception:
            pass
