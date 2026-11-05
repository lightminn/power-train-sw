"""Read-only acceptance of board 1 (335233643235) after patch4: no save/erase/calibrate/arm."""
import json, sys, time
sys.path.insert(0, "/workspace/motor_control")
import odrive
from chassis.usb_session import motor_session, read_path
SERIAL = "335233643235"
AX = ("current_state", "requested_state", "error", "motor.error", "encoder.error", "controller.error",
      "motor.is_calibrated", "encoder.is_ready", "motor.config.pre_calibrated",
      "encoder.config.pre_calibrated", "config.can_node_id", "encoder.vel_estimate",
      "can_recovery_latched", "can_recovery_in_progress")
CAN = ("reliability_patch", "hal_error", "recovery_count", "recovery_failure_count", "tx_drop_count",
       "tx_error_count", "lec_error_count", "bus_off_count", "error_passive_count")
def snap(b):
    return {"axes": [{p: read_path(a, p) for p in AX} for a in (b.axis0, b.axis1)],
            "can": {n: read_path(b.can, n) for n in CAN}}
with motor_session("mks_board1_verify"):
    b = odrive.find_any(serial_number=SERIAL, timeout=30)
    r = {"serial": format(b.serial_number, "X"),
         "fw": [b.fw_version_major, b.fw_version_minor, b.fw_version_revision, b.fw_version_unreleased],
         "hw": [b.hw_version_major, b.hw_version_minor, b.hw_version_variant],
         "user_config_loaded": read_path(b, "user_config_loaded"), "vbus": round(b.vbus_voltage, 2),
         "baud": read_path(b, "can.config.baud_rate"), "t0": snap(b)}
    time.sleep(3)
    r["t3"] = snap(b)
    ok = (r["serial"] == SERIAL and r["user_config_loaded"] and r["t3"]["can"]["reliability_patch"] == 4
          and r["baud"] == 500000 and [a["config.can_node_id"] for a in r["t3"]["axes"]] == [11, 12]
          and all(a["current_state"] == 1 and a["error"] == 0 and a["motor.error"] == 0
                  and a["encoder.error"] == 0 and a["controller.error"] == 0
                  and a["motor.is_calibrated"] and a["encoder.is_ready"] for a in r["t3"]["axes"]))
    r["accepted"] = bool(ok)
print(json.dumps(r, indent=2, default=str))
