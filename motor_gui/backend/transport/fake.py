from __future__ import annotations

import time

from .base import (Transport, SIGNAL_META, ODRIVE_CONTROL_MODES, ODRIVE_INPUTS,
                   ODRIVE_TUNABLES_USB)

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
    목표로 수렴, torque 모드면 vel 에 토크를 적분(±_VEL_LIMIT 클램프). iq_meas 는
    가속도 + 속도 항으로 근사.
    """

    name = "fake"
    DT = 0.01            # 가정 틱 (100 Hz)
    _VEL_ALPHA = 0.05    # velocity 모드 1차 수렴 계수
    _POS_KP = 2.0        # position 모드 P-gain
    _TRQ_GAIN = 0.1      # torque → vel 적분 계수
    _VEL_LIMIT = 200.0   # vel 클램프 (capabilities limits.odrive.vel 와 일치)
    _AK_ALPHA = 0.05     # AK 위치 1차 수렴 계수
    _AK_SPEED_SCALE = 10.0

    def __init__(self) -> None:
        self._reset()

    def _reset(self) -> None:
        self._pos = 0.0
        self._vel = 0.0
        self._mode = "velocity"
        self._target = 0.0
        self._ak_pos = 0.0
        self._ak_target = 0.0
        self._pos_offset = 0.0
        self._tun = {
            "pos_gain": 20.0,
            "vel_gain": 0.16,
            "vel_integrator_gain": 0.32,
            "input_filter_bandwidth": 2.0,
            "vel_limit": 10.0,
            "current_lim": 10.0,
            "trap_vel_limit": 20.0,
            "trap_accel_limit": 20.0,
            "trap_decel_limit": 20.0,
        }

    def connect(self) -> None:
        self._reset()

    def sample(self) -> dict:
        prev_vel = self._vel
        if self._mode == "velocity":
            self._vel += (self._target - self._vel) * self._VEL_ALPHA
        elif self._mode == "position":
            err = self._target - self._pos
            self._vel = err * self._POS_KP
        elif self._mode == "torque":
            self._vel += self._target * self._TRQ_GAIN
            self._vel = max(-self._VEL_LIMIT, min(self._VEL_LIMIT, self._vel))
        self._pos += self._vel * self.DT
        self._ak_pos += (self._ak_target - self._ak_pos) * self._AK_ALPHA
        iq = (self._vel - prev_vel) / self.DT * 0.01 + self._vel * 0.02
        return {
            "t_mono": time.monotonic(),
            "odrive.pos": self._pos - self._pos_offset,
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
            "ak.speed": (self._ak_target - self._ak_pos) * self._AK_SPEED_SCALE,
            "ak.current": 0.0,
            "ak.temp": 28.0,
            "ak.fault": 0,
        }

    def apply(self, cmd: dict) -> dict:
        target, op, args = cmd["target"], cmd["op"], cmd.get("args", {})
        if op == "estop":
            self._mode = "velocity"   # position 루프 중단 → 실제 제동
            self._target = 0.0
            self._vel = 0.0
            self._ak_target = self._ak_pos
            return self._ack(target, op, "estopped")
        if target == "odrive":
            if op == "set_mode":
                m = args.get("control_mode", self._mode)
                self._mode = "position" if m in ("position", "position_traj") else m
                if self._mode == "position":
                    self._target = self._pos
            elif op == "set_origin":
                self._pos_offset = self._pos
                self._target = self._pos
            elif op == "set_input":
                if "vel" in args:
                    self._target = float(args["vel"])
                elif "pos" in args:
                    self._target = float(args["pos"]) + self._pos_offset
                elif "torque" in args:
                    self._target = float(args["torque"])
            elif op in ("set_gain", "set_limit"):
                for k, v in args.items():
                    if k in self._tun:
                        self._tun[k] = float(v)
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
                           "save_nvm", "set_origin", "estop"],
                "ak": ["set_input", "set_origin", "estop"],
            },
            "limits": {
                "odrive": {"vel": 200.0, "torque": 10.0, "pos": 100.0},
                "ak": {"pos_deg": 360.0},
            },
            "control_modes": {"odrive": ODRIVE_CONTROL_MODES},
            "inputs": {"odrive": ODRIVE_INPUTS},
            "tunables": {"odrive": ODRIVE_TUNABLES_USB},
            "signal_meta": SIGNAL_META,
            "notes": ["fake track — 시뮬 모터, 하드웨어 미연결"],
        }

    def read_tunables(self) -> dict:
        return dict(self._tun)

    def close(self) -> None:
        self._target = 0.0
        self._vel = 0.0

    @staticmethod
    def _ack(target: str, op: str, detail: str) -> dict:
        return {"ok": True, "target": target, "op": op, "detail": detail}
