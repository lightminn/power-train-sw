from __future__ import annotations

import time

from .base import (Transport, TransportError, SIGNAL_META, ODRIVE_CONTROL_MODES,
                   ODRIVE_INPUTS, ODRIVE_TUNABLES_USB)

_ODRIVE_SIGNALS = [
    "odrive.pos", "odrive.vel", "odrive.iq_meas", "odrive.iq_set",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus", "odrive.state",
    "odrive.axis_err", "odrive.motor_err", "odrive.enc_err",
    "odrive.ctrl_err", "odrive.vel_integrator",
]


class UsbOdriveBackend(Transport):
    """ODrive USB (odrive lib, axis1, fw-v0.5.6). NVM 저장 지원."""

    name = "usb"

    def __init__(self, axis_num: int = 1, timeout: float = 15.0) -> None:
        self._timeout = timeout
        self._axis_num = axis_num
        self._drv = None
        self._ax = None
        self._fet_therm = None      # fw 별 위치 달라 connect 에서 resolve
        self._pos_offset = 0.0   # set_origin 소프트웨어 영점
        self._enums: dict = {}

    def connect(self) -> None:
        import odrive
        from odrive.enums import (AxisState, ControlMode, InputMode)
        drv = odrive.find_any(timeout=self._timeout)
        if drv is None:
            raise TransportError("ODrive USB not found")
        self._drv = drv
        self._ax = drv.axis1 if self._axis_num == 1 else drv.axis0
        # FET 서미스터 위치가 fw 별로 다름 (0.5.1=axis.fet_thermistor,
        # 일부 빌드=axis.motor.fet_thermistor). connect 시 1회 resolve.
        self._fet_therm = (getattr(self._ax, "fet_thermistor", None)
                           or getattr(self._ax.motor, "fet_thermistor", None))
        self._pos_offset = 0.0
        # 토크(current) 모드에서도 vel_limit 존중 보장 (무부하 runaway 방지).
        try:
            self._ax.controller.config.enable_current_mode_vel_limit = True
        except Exception:
            pass
        # fw-v0.5.6 plain Enum → wire I/O 용 int 상수 (0.6.x IntEnum 도 .value 동작)
        self._enums = {
            "IDLE": AxisState.IDLE.value,
            "CLOSED_LOOP": AxisState.CLOSED_LOOP_CONTROL.value,
            "FULL_CALIB": AxisState.FULL_CALIBRATION_SEQUENCE.value,
            "POSITION": ControlMode.POSITION_CONTROL.value,
            "VELOCITY": ControlMode.VELOCITY_CONTROL.value,
            "TORQUE": ControlMode.TORQUE_CONTROL.value,
            "PASSTHROUGH": InputMode.PASSTHROUGH.value,
            "POS_FILTER": InputMode.POS_FILTER.value,
            "VEL_RAMP": InputMode.VEL_RAMP.value,
            "TRAP_TRAJ": InputMode.TRAP_TRAJ.value,
        }

    def sample(self) -> dict:
        ax, drv = self._ax, self._drv
        m = ax.motor.current_control
        return {
            "t_mono": time.monotonic(),
            "odrive.pos": float(ax.encoder.pos_estimate) - self._pos_offset,
            "odrive.vel": float(ax.encoder.vel_estimate),
            "odrive.iq_meas": float(m.Iq_measured),
            "odrive.iq_set": float(m.Iq_setpoint),
            "odrive.temp_fet": (float(self._fet_therm.temperature)
                                if self._fet_therm is not None else 0.0),
            "odrive.vbus": float(drv.vbus_voltage),
            "odrive.ibus": float(getattr(drv, "ibus", 0.0)),
            "odrive.state": int(ax.current_state),
            "odrive.axis_err": int(ax.error),
            "odrive.motor_err": int(ax.motor.error),
            "odrive.enc_err": int(ax.encoder.error),
            "odrive.ctrl_err": int(ax.controller.error),
            "odrive.vel_integrator": float(ax.controller.vel_integrator_torque),
        }

    def apply(self, cmd: dict) -> dict:
        ax = self._ax
        op, args = cmd["op"], cmd.get("args", {})
        try:
            if op == "estop":
                ax.requested_state = self._enums["IDLE"]
            elif op == "set_mode":
                mode = args["control_mode"]
                cm = {"position": "POSITION", "position_traj": "POSITION",
                      "velocity": "VELOCITY", "torque": "TORQUE"}[mode]
                im_def = {"position": "POS_FILTER", "position_traj": "TRAP_TRAJ",
                          "velocity": "VEL_RAMP", "torque": "PASSTHROUGH"}[mode]
                ax.controller.config.control_mode = self._enums[cm]
                im = args.get("input_mode", im_def)
                if im in self._enums:
                    ax.controller.config.input_mode = self._enums[im]
                if cm == "POSITION":
                    ax.controller.input_pos = ax.encoder.pos_estimate  # 현재 위치 hold
            elif op == "set_input":
                if "pos" in args:
                    ax.controller.input_pos = float(args["pos"]) + self._pos_offset
                elif "vel" in args:
                    ax.controller.input_vel = float(args["vel"])
                elif "torque" in args:
                    ax.controller.input_torque = float(args["torque"])
            elif op == "set_gain":
                for k in ("pos_gain", "vel_gain", "vel_integrator_gain",
                          "input_filter_bandwidth"):
                    if k in args:
                        setattr(ax.controller.config, k, float(args[k]))
                trap_map = {"trap_vel_limit": "vel_limit",
                            "trap_accel_limit": "accel_limit",
                            "trap_decel_limit": "decel_limit"}
                for k, attr in trap_map.items():
                    if k in args:
                        setattr(ax.trap_traj.config, attr, float(args[k]))
            elif op == "set_limit":
                if "vel_limit" in args:
                    ax.controller.config.vel_limit = float(args["vel_limit"])
                if "current_lim" in args:
                    ax.motor.config.current_lim = float(args["current_lim"])
            elif op == "set_state":
                if args.get("state") == "closed_loop":
                    ax.controller.input_pos = ax.encoder.pos_estimate  # jump 방지
                    ax.requested_state = self._enums["CLOSED_LOOP"]
                else:
                    ax.requested_state = self._enums["IDLE"]
            elif op == "calibrate":
                ax.requested_state = self._enums["FULL_CALIB"]
            elif op == "clear_errors":
                ax.clear_errors()
                try:
                    self._drv.clear_errors()
                except Exception:
                    pass
            elif op == "set_origin":
                # 소프트웨어 영점: 현재 절대 위치를 0 으로 기록만, 모터 명령 변화 없음.
                self._pos_offset = float(ax.encoder.pos_estimate)
                ax.controller.input_pos = ax.encoder.pos_estimate  # 현재 위치 hold
            elif op == "save_nvm":
                self._drv.save_configuration()
            return {"ok": True, "target": "odrive", "op": op, "detail": "ok"}
        except Exception as e:
            return {"ok": False, "target": "odrive", "op": op, "detail": str(e)}

    def capabilities(self) -> dict:
        return {
            "track": "usb",
            "devices": ["odrive"],
            "signals": _ODRIVE_SIGNALS,
            "commands": {"odrive": ["set_mode", "set_input", "set_gain",
                                    "set_limit", "set_state", "calibrate",
                                    "clear_errors", "save_nvm", "set_origin",
                                    "estop"]},
            "limits": {"odrive": {"vel": 200.0, "torque": 10.0, "pos": 100.0}},
            "control_modes": {"odrive": ODRIVE_CONTROL_MODES},
            "inputs": {"odrive": ODRIVE_INPUTS},
            "tunables": {"odrive": ODRIVE_TUNABLES_USB},
            "signal_meta": SIGNAL_META,
            "notes": ["USB 트랙 — ODrive 단독, NVM 저장 가능"],
        }

    def read_tunables(self) -> dict:
        if self._ax is None:
            return {}
        c = self._ax.controller.config
        out = {
            "pos_gain": float(c.pos_gain),
            "vel_gain": float(c.vel_gain),
            "vel_integrator_gain": float(c.vel_integrator_gain),
            "vel_limit": float(c.vel_limit),
            "current_lim": float(self._ax.motor.config.current_lim),
        }
        try:
            out["input_filter_bandwidth"] = float(c.input_filter_bandwidth)
        except Exception:
            pass
        try:
            tc = self._ax.trap_traj.config
            out["trap_vel_limit"] = float(tc.vel_limit)
            out["trap_accel_limit"] = float(tc.accel_limit)
            out["trap_decel_limit"] = float(tc.decel_limit)
        except Exception:
            pass
        return out

    def close(self) -> None:
        if self._ax is not None:
            try:
                self._ax.requested_state = self._enums["IDLE"]
            except Exception:
                pass
