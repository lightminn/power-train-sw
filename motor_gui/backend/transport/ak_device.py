from __future__ import annotations

import sys
import time
from pathlib import Path

from .base import SIGNAL_META
from .can_device import CanDevice

# steering/ak_control.py 의 AK40 재사용 (hw 로직 단일 소스)
sys.path.insert(0, str(Path(__file__).resolve().parents[3] / "motor_control" / "steering"))
from ak_control import AK40, POLE_PAIRS, GEAR_RATIO  # noqa: E402

AK_ID = 10
PKT_STATUS_1 = 41
_AK_SIGNALS = ["ak.pos_deg", "ak.speed", "ak.current", "ak.temp", "ak.fault", "ak.tripped"]
_RESEND_HZ = 20.0


class AkDevice(CanDevice):
    """AK40 조향 모터 (확장 CAN ID). 4모드 + 워치독 재전송 + 과전류 자동정지."""

    name = "ak"

    def __init__(self, motor_id: int = AK_ID) -> None:
        self._mid = motor_id
        self._ak = None
        self._mode = "position"
        self._active = None          # 재전송할 무인자 호출 (워치독)
        self._spd = 1500.0
        self._acc = 6000.0
        self._maxcur = 5.0
        self._last_send = 0.0
        self._tripped = False

    def attach(self, bus) -> None:
        self._ak = AK40(bus, self._mid, name="ak")
        self._active = None
        self._tripped = False

    def capabilities_fragment(self) -> dict:
        meta = {k: SIGNAL_META[k] for k in _AK_SIGNALS if k in SIGNAL_META}
        return {
            "devices": ["ak"],
            "signals": list(_AK_SIGNALS),
            "commands": {"ak": ["set_mode", "set_input", "set_param",
                                 "set_origin", "estop"]},
            "control_modes": {"ak": ["position", "velocity", "brake", "duty"]},
            "inputs": {"ak": {
                "position": {"key": "pos_deg", "label": "목표 위치", "unit": "°"},
                "velocity": {"key": "rpm", "label": "목표 속도(출력축)", "unit": "RPM"},
                "brake": {"key": "brake_cur", "label": "브레이크 전류", "unit": "A"},
                "duty": {"key": "duty", "label": "듀티", "unit": ""},
            }},
            "tunables": {"ak": [
                {"op": "set_param", "key": "spd_erpm", "label": "속도제한 [ERPM]"},
                {"op": "set_param", "key": "acc_erpm_s2", "label": "가속 [ERPM/s²]"},
                {"op": "set_param", "key": "max_cur_a", "label": "최대전류(자동정지) [A]"},
            ]},
            "limits": {"ak": {"pos_deg": 100000.0, "rpm": 1000.0,
                              "brake_cur": 20.0, "duty": 1.0}},
            "signal_meta": meta,
        }

    def on_rx(self, msg) -> None:
        if not msg.is_extended_id:
            return
        pkt = (msg.arbitration_id >> 8) & 0xFF
        nid = msg.arbitration_id & 0xFF
        if pkt == PKT_STATUS_1 and nid == self._mid and len(msg.data) >= 8:
            self._ak._parse_status(msg.data)

    def _fire(self) -> None:
        if self._active is not None:
            self._active()
            self._last_send = time.monotonic()

    def tick(self, bus) -> None:
        if self._ak is None:
            return
        if abs(self._ak.cur_a) > self._maxcur:      # 과전류 자동정지
            self._ak.send_rpm_out(0)
            self._active = None
            self._tripped = True
            return
        now = time.monotonic()
        if self._active is not None and now - self._last_send >= 1.0 / _RESEND_HZ:
            self._fire()

    def sample(self) -> dict:
        a = self._ak
        if a is None:
            return {k: 0.0 for k in _AK_SIGNALS}
        out_rpm = a.spd_erpm / (POLE_PAIRS * GEAR_RATIO)
        return {
            "ak.pos_deg": float(a.pos_out_deg),
            "ak.speed": float(out_rpm),
            "ak.current": float(a.cur_a),
            "ak.temp": float(a.temp_c),
            "ak.fault": int(a.fault),
            "ak.tripped": int(self._tripped),
        }

    def apply(self, bus, op: str, args: dict) -> dict:
        a = self._ak
        try:
            if op == "estop":
                a.stop()
                self._active = None
            elif op == "set_mode":
                self._mode = args["control_mode"]
                self._tripped = False
                if self._mode == "position":
                    tgt = a.pos_out_deg
                    self._active = lambda: a.send_pos_out(tgt, self._spd, self._acc)
                elif self._mode == "velocity":
                    self._active = lambda: a.send_rpm_out(0.0)
                elif self._mode == "brake":
                    self._active = lambda: a.send_brake(0.0)
                else:  # duty
                    self._active = lambda: a.send_duty(0.0)
                self._fire()
            elif op == "set_input":
                expected = {"position": "pos_deg", "velocity": "rpm",
                            "brake": "brake_cur", "duty": "duty"}[self._mode]
                if expected not in args:
                    return {"ok": False, "target": "ak", "op": op,
                            "detail": f"'{expected}' 필요 (현재 모드 {self._mode})"}
                if "pos_deg" in args:
                    tgt = float(args["pos_deg"])
                    self._active = lambda: a.send_pos_out(tgt, self._spd, self._acc)
                elif "rpm" in args:
                    v = float(args["rpm"])
                    self._active = lambda: a.send_rpm_out(v)
                elif "brake_cur" in args:
                    v = float(args["brake_cur"])
                    self._active = lambda: a.send_brake(v)
                elif "duty" in args:
                    v = float(args["duty"])
                    self._active = lambda: a.send_duty(v)
                else:
                    return {"ok": False, "target": "ak", "op": op,
                            "detail": "no known input key"}
                self._fire()
            elif op == "set_param":
                if "spd_erpm" in args:
                    self._spd = float(args["spd_erpm"])
                if "acc_erpm_s2" in args:
                    self._acc = float(args["acc_erpm_s2"])
                if "max_cur_a" in args:
                    self._maxcur = float(args["max_cur_a"])
            elif op == "set_origin":
                a.set_origin_here()
            else:
                return {"ok": False, "target": "ak", "op": op,
                        "detail": "unsupported op"}
            return {"ok": True, "target": "ak", "op": op, "detail": "sent"}
        except Exception as e:
            return {"ok": False, "target": "ak", "op": op, "detail": str(e)}

    def close(self, bus) -> None:
        if self._ak is not None:
            try:
                self._ak.stop()
            except Exception:
                pass
