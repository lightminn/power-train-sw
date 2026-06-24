from __future__ import annotations

import time

from .base import (Transport, TransportError, SIGNAL_META, ODRIVE_CONTROL_MODES,
                   ODRIVE_INPUTS, ODRIVE_TUNABLES_USB)

_ODRIVE_SIGNALS = [
    "odrive.pos", "odrive.pos_setpoint", "odrive.vel", "odrive.vel_setpoint",
    "odrive.iq_meas", "odrive.iq_set", "odrive.id_meas", "odrive.id_set",
    "odrive.torque_est",
    "odrive.temp_fet", "odrive.vbus", "odrive.ibus", "odrive.state",
    "odrive.axis_err", "odrive.motor_err", "odrive.enc_err",
    "odrive.ctrl_err", "odrive.vel_integrator",
]

# TRAP 순항속도는 vel_limit 에 강제 결합(set_limit 에서 동기) → 별도 노출 안 함.
# trap_vel_limit > vel_limit 이면 궤적이 속도루프보다 앞서 달려 위치오차가
# 누적되고(windup), vel_limit 을 풀면 그 오차가 한 번에 방출돼 폭주/저전압을 유발.
_USB_TUNABLES = [t for t in ODRIVE_TUNABLES_USB if t["key"] != "trap_vel_limit"]


class UsbOdriveBackend(Transport):
    """ODrive USB (odrive lib, axis1, fw-v0.5.6). NVM 저장 지원."""

    name = "usb"

    def __init__(self, axis_num: int = 1, timeout: float = 15.0) -> None:
        self._timeout = timeout
        self._axis_num = axis_num
        self._drv = None
        self._ax = None
        self._fet_therm = None      # fw 별 위치 달라 connect 에서 resolve
        self._enums: dict = {}
        self._vel_limit = 50.0       # 사용자가 의도한 속도(=TRAP 순항). 캡과 구분.
        self._torque_const = 0.0    # Iq→토크 환산 (motor.config.torque_constant)
        self._motor_info: dict = {}  # 정적 모터 파라미터 (capabilities 노출)

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
        self._vel_limit = float(self._ax.controller.config.vel_limit)
        self._sync_vel_limit()
        # 정적 모터 파라미터 (캘리브레이션 결과) — 1회 읽어 토크 환산/표시에 사용.
        mc = self._ax.motor.config
        self._torque_const = float(getattr(mc, "torque_constant", 0.0))
        self._motor_info = {
            "phase_resistance": float(getattr(mc, "phase_resistance", 0.0)),
            "phase_inductance": float(getattr(mc, "phase_inductance", 0.0)),
            "torque_constant": self._torque_const,
            "pole_pairs": int(getattr(mc, "pole_pairs", 0)),
            "current_lim": float(getattr(mc, "current_lim", 0.0)),
        }

    def _sync_vel_limit(self) -> None:
        """현재 모드에 맞춰 controller.vel_limit(하드 캡) 설정.

        TRAP: 순항=self._vel_limit, 하드캡=순항×1.3. 캡에 헤드룸을 둬야 위치오차
        보정 여력이 남아 gap 누적/방출 스파이크가 안 생긴다(피드포워드가 캡을
        다 먹으면 gap 이 쌓였다가 캡 해제 시 한 번에 튐). 단 현재 속도 아래로는
        안 내림(overspeed=CONTROLLER_FAILED 트립 방지).
        그 외(velocity/POS_FILTER): vel_limit 이 곧 속도 캡 → 그대로.
        """
        ax = self._ax
        in_trap = int(ax.controller.config.input_mode) == self._enums["TRAP_TRAJ"]
        ax.trap_traj.config.vel_limit = self._vel_limit
        if in_trap:
            cur = abs(float(ax.encoder.vel_estimate))
            ax.controller.config.vel_limit = max(self._vel_limit * 1.3, cur * 1.3)
        else:
            ax.controller.config.vel_limit = self._vel_limit

    def sample(self) -> dict:
        ax, drv = self._ax, self._drv
        m = ax.motor.current_control
        iq_meas = float(m.Iq_measured)
        return {
            "t_mono": time.monotonic(),
            "odrive.pos": float(ax.encoder.pos_estimate),
            "odrive.pos_setpoint": float(ax.controller.pos_setpoint),
            "odrive.vel": float(ax.encoder.vel_estimate),
            "odrive.vel_setpoint": float(ax.controller.vel_setpoint),
            "odrive.iq_meas": iq_meas,
            "odrive.iq_set": float(m.Iq_setpoint),
            "odrive.id_meas": float(m.Id_measured),
            "odrive.id_set": float(m.Id_setpoint),
            "odrive.torque_est": iq_meas * self._torque_const,  # τ = Iq × Kt [Nm]
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
                self._sync_vel_limit()  # 모드별 캡 재적용(TRAP=헤드룸 / 그외=정확)
                if cm == "POSITION":
                    ax.controller.input_pos = ax.encoder.pos_estimate  # 현재 위치 hold
            elif op == "set_input":
                if "pos" in args:
                    ax.controller.input_pos = float(args["pos"])
                elif "vel" in args:
                    ax.controller.input_vel = float(args["vel"])
                elif "torque" in args:
                    ax.controller.input_torque = float(args["torque"])
            elif op == "set_gain":
                for k in ("pos_gain", "vel_gain", "vel_integrator_gain",
                          "input_filter_bandwidth"):
                    if k in args:
                        setattr(ax.controller.config, k, float(args[k]))
                # trap_vel_limit 는 무시(vel_limit 에 결합) — set_limit 에서만 설정.
                trap_map = {"trap_accel_limit": "accel_limit",
                            "trap_decel_limit": "decel_limit"}
                for k, attr in trap_map.items():
                    if k in args:
                        setattr(ax.trap_traj.config, attr, float(args[k]))
            elif op == "set_limit":
                if "vel_limit" in args:
                    self._vel_limit = float(args["vel_limit"])
                    self._sync_vel_limit()  # 모드별 캡(TRAP=헤드룸) + 순항속도 적용
                    # TRAP 진행 중이면 input_pos 재발행 → 새 순항속도로 궤적 재계획
                    # (가속한계 지켜 부드럽게 변속. 재계획 없으면 옛 속도로 끝까지 감.)
                    if (int(ax.controller.config.input_mode) == self._enums["TRAP_TRAJ"]
                            and int(ax.current_state) == self._enums["CLOSED_LOOP"]):
                        ax.controller.input_pos = float(ax.controller.input_pos)
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
                # 순정 영점: IDLE 디스암 → set_linear_count(0) → input_pos=0 → 재무장.
                # 컨트롤러 setpoint 까지 0 으로 같이 리셋되어 옛 setpoint 로 튀지 않음.
                was_closed = int(ax.current_state) == self._enums["CLOSED_LOOP"]
                ax.requested_state = self._enums["IDLE"]
                time.sleep(0.2)
                ax.encoder.set_linear_count(0)
                ax.controller.input_pos = 0.0
                if was_closed:
                    ax.requested_state = self._enums["CLOSED_LOOP"]
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
            "limits": {"odrive": {"vel": 200.0, "torque": 10.0, "pos": 100000.0}},
            "control_modes": {"odrive": ODRIVE_CONTROL_MODES},
            "inputs": {"odrive": ODRIVE_INPUTS},
            "tunables": {"odrive": _USB_TUNABLES},
            "signal_meta": SIGNAL_META,
            "motor_info": {"odrive": self._motor_info},
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
            "vel_limit": float(self._vel_limit),  # 사용자 의도값(캡은 TRAP 시 헤드룸 포함)
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
