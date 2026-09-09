"""Validate the chassis owner's hardware stop-proof payload at the ROS boundary."""
import math


def decode_hardware_stop_proof(value, *, chassis_mode, drive_enabled):
    """Return (stopped, feedback age in seconds), or (False, None) if unproven.

    Generic /wheel_states publishers are visualization/telemetry sources, not
    evidence that the process owning these six physical axes observed a stop.
    """
    if (not isinstance(value, dict) or drive_enabled is not True
            or chassis_mode not in ('IDLE', 'ARMING', 'ARMED', 'ESTOP', 'EXTRACTION')
            or value.get('source') not in ('chassis_can', 'chassis_usb')
            or value.get('valid') is not True or type(value.get('stopped')) is not bool):
        return False, None
    nodes = value.get('node_ids')
    if (not isinstance(nodes, list) or len(nodes) != 6
            or any(type(node) is not int for node in nodes)
            or sorted(nodes) != [11, 12, 13, 14, 15, 16]):
        return False, None
    age = value.get('max_feedback_age_ms')
    if type(age) not in (int, float) or not math.isfinite(age) or not 0 <= age <= 200:
        return False, None
    return value['stopped'], age / 1000.
