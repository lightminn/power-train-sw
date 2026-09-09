"""USB maintenance shares the motor owner lock with CAN; no CAN socket is opened.

Communication paths are from ODrive fw-v0.5.1 (flat axis config) and
fw-v0.5.6 (axis.config.can). Missing readbacks never certify a write/save.
"""
from __future__ import annotations

import time


def motor_session(owner):
    from chassis.runtime_lock import RealCanSession
    return RealCanSession(channel="can0", owner=owner)


def normalize_serial(serial):
    if serial is None or isinstance(serial, bool):
        raise ValueError("explicit ODrive serial is required")
    try:
        number = serial if isinstance(serial, int) else int(str(serial).strip(), 16)
    except ValueError as exc:
        raise ValueError("ODrive serial must be hexadecimal") from exc
    if number <= 0:
        raise ValueError("ODrive serial must be positive")
    return format(number, "X")


def validate_target(serial, axis, node):
    normalize_serial(serial)
    if str(axis) not in ("0", "1", "both"):
        raise ValueError("explicit axis 0, 1 or both is required")
    if isinstance(node, bool) or not isinstance(node, int) or not 0 <= node <= 62:
        raise ValueError("explicit node in 0..62 is required (63 is broadcast)")
    if str(axis) == "both" and node >= 62:
        raise ValueError("both axes require two nodes below broadcast node 63")


def read_path(obj, *paths):
    for path in paths:
        value = obj
        try:
            for part in path.split("."):
                value = getattr(value, part)
            return getattr(value, "value", value)
        except AttributeError:
            continue
    return None


def axis_communication(axis):
    result = {
        "node": read_path(axis, "config.can.node_id", "config.can_node_id"),
        "extended": read_path(axis, "config.can.is_extended", "config.can_node_id_extended", "config.can_extended_id"),
        "heartbeat_ms": read_path(axis, "config.can.heartbeat_rate_ms", "config.can_heartbeat_rate_ms"),
    }
    for name in ("encoder", "motor_error", "encoder_error", "controller_error",
                 "sensorless_error", "encoder_count", "iq", "sensorless", "bus_vi"):
        value = read_path(axis, f"config.can.{name}_rate_ms", f"config.can_{name}_rate_ms")
        if value is not None:
            result[f"{name}_rate_ms"] = value
    return result


def communication_snapshot(board, *, serial=None, strict=False):
    snapshot = {
        "serial": normalize_serial(board.serial_number),
        "firmware": [read_path(board, f"fw_version_{part}") for part in ("major", "minor", "revision")],
        "baud": read_path(board, "can.config.baud_rate", "config.can.baud_rate"),
        "protocol": read_path(board, "can.config.protocol", "config.can.protocol"),
        "axis0": axis_communication(board.axis0),
        "axis1": axis_communication(board.axis1),
    }
    if serial is not None and snapshot["serial"] != normalize_serial(serial):
        raise ValueError(f"ODrive serial mismatch: expected {serial}, got {snapshot['serial']}")
    if strict:
        required = [snapshot["baud"], snapshot["protocol"], *snapshot["firmware"]]
        for axis in ("axis0", "axis1"):
            required.extend(snapshot[axis][key] for key in ("node", "extended", "heartbeat_ms"))
        if any(value is None for value in required):
            raise ValueError(f"communication configuration readback unavailable: {snapshot}")
    return snapshot


def verify_communication(board, expected):
    actual = communication_snapshot(board, serial=expected["serial"], strict=True)
    if actual != expected:
        raise ValueError(f"communication configuration mismatch: expected={expected}, actual={actual}")
    return actual


def save_and_reconnect(board, odrive_module, expected, *, serial, timeout=20,
                       sleep_fn=time.sleep):
    """Verify immediately before save and on the exact serial after reboot."""
    verify_communication(board, expected)
    try:
        result = board.save_configuration()
        if result is False:
            raise RuntimeError("ODrive refused NVM save")
    except Exception as exc:
        # Only the SDK's documented reboot disconnect is a candidate save success.
        # Re-enumeration and complete communication readback are still mandatory.
        if type(exc).__name__ != "ObjectLostError" or not type(exc).__module__.startswith("fibre"):
            raise
    sleep_fn(8)
    reconnected = odrive_module.find_any(serial_number=serial, timeout=timeout)
    if reconnected is None:
        raise ValueError(f"ODrive serial {serial} did not re-enumerate after save")
    verify_communication(reconnected, expected)
    return reconnected
