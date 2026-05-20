from __future__ import annotations

import time

from .base import Transport

_ODRIVE_SIGNALS = [
    "odrive.pos", "odrive.vel", "odrive.iq_meas", "odrive.iq_set",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus", "odrive.state",
    "odrive.axis_err", "odrive.motor_err", "odrive.enc_err",
    "odrive.ctrl_err", "odrive.vel_integrator",
]
_AK_SIGNALS = ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp", "ak.fault"]


class FakeTransport(Transport):
    """하드웨어 없는 시뮬 모터. odrive+ak 슈퍼셋을 노출해 모든 UI 요소를 구동.

    단순 1차 모델: velocity 모드면 vel 가 목표로 1차 수렴, position 모드면 pos 가
    목표로 수렴, torque 모드면 vel 에 토크를 적분. iq_meas 는 가속도에 비례.
    """

    name = "fake"
    DT = 0.01  # 가정 틱 (100 Hz)

    def __init__(self) -> None:
        self._pos = 0.0
        self._vel = 0.0
        self._mode = "velocity"
        self._target = 0.0
        self._ak_pos = 0.0
        self._ak_target = 0.0
        self._last_iq = 0.0

    def connect(self) -> None:
        self._pos = self._vel = self._target = 0.0

    def sample(self) -> dict:
        prev_vel = self._vel
        if self._mode == "velocity":
            self._vel += (self._target - self._vel) * 0.05
        elif self._mode == "position":
            err = self._target - self._pos
            self._vel = err * 2.0
        elif self._mode == "torque":
            self._vel += self._target * 0.1
        self._pos += self._vel * self.DT
        self._ak_pos += (self._ak_target - self._ak_pos) * 0.05
        iq = (self._vel - prev_vel) / self.DT * 0.01 + self._vel * 0.02
        self._last_iq = iq
        return {
            "t_mono": time.monotonic(),
            "odrive.pos": self._pos,
            "odrive.vel": self._vel,
            "odrive.iq_meas": iq,
            "odrive.iq_set": self._target if self._mode == "torque" else iq,
            "odrive.temp_fet": 30.0 + abs(self._vel),
            "odrive.vbus": 24.0,
            "odrive.ibus": iq * 0.5,
            "odrive.state": 8,
            "odrive.axis_err": 0,
            "odrive.motor_err": 0,
            "odrive.enc_err": 0,
            "odrive.ctrl_err": 0,
            "odrive.vel_integrator": self._vel * 0.01,
            "ak.pos_deg": self._ak_pos,
            "ak.speed": (self._ak_target - self._ak_pos) * 10.0,
            "ak.current": 0.0,
            "ak.temp": 28.0,
            "ak.fault": 0,
        }

    def apply(self, cmd: dict) -> dict:
        target, op, args = cmd["target"], cmd["op"], cmd.get("args", {})
        if op == "estop":
            self._target = 0.0
            self._vel = 0.0
            self._ak_target = self._ak_pos
            return self._ack(target, op, "estopped")
        if target == "odrive":
            if op == "set_mode":
                self._mode = args.get("control_mode", self._mode)
            elif op == "set_input":
                if "vel" in args:
                    self._target = float(args["vel"])
                elif "pos" in args:
                    self._target = float(args["pos"])
                elif "torque" in args:
                    self._target = float(args["torque"])
        elif target == "ak":
            if op == "set_input" and "pos_deg" in args:
                self._ak_target = float(args["pos_deg"])
            elif op == "set_origin":
                self._ak_pos = 0.0
                self._ak_target = 0.0
        return self._ack(target, op, "ok")

    def capabilities(self) -> dict:
        return {
            "track": "fake",
            "devices": ["odrive", "ak"],
            "signals": _ODRIVE_SIGNALS + _AK_SIGNALS,
            "commands": {
                "odrive": ["set_mode", "set_input", "set_gain", "set_limit",
                           "set_state", "calibrate", "clear_errors",
                           "save_nvm", "estop"],
                "ak": ["set_input", "set_origin", "estop"],
            },
            "limits": {
                "odrive": {"vel": 20.0, "torque": 10.0, "pos": 100.0},
                "ak": {"pos_deg": 360.0},
            },
            "notes": ["fake track — 시뮬 모터, 하드웨어 미연결"],
        }

    def close(self) -> None:
        self._target = 0.0
        self._vel = 0.0

    @staticmethod
    def _ack(target: str, op: str, detail: str) -> dict:
        return {"ok": True, "target": target, "op": op, "detail": detail}
