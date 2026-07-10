import math


_STATUS_CODES = {
    "CHECKING": 0,
    "VALID": 1,
    "INVALID_READING": 2,
    "NO_RESPONSE": 3,
}


def fill_safety_message(msg, verdict, stamp, frame_id="us100_link"):
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.status = _STATUS_CODES[verdict.status]
    msg.distance_mm = (
        float(verdict.distance_mm)
        if verdict.distance_mm is not None
        else math.nan
    )
    msg.estop_required = bool(verdict.estop_required)
    msg.consecutive_failures = int(verdict.consecutive_failures)
    msg.detail = verdict.detail
    return msg


def fill_wheel_states_message(
    msg,
    snapshot,
    stamp,
    tick_duration_ms,
    overrun_count,
    wheel_factory,
):
    msg.header.stamp = stamp
    msg.header.frame_id = "base_link"
    msg.chassis_mode = snapshot.chassis_mode
    msg.stop_state = snapshot.stop_state
    msg.healthy = snapshot.healthy
    msg.tick_duration_ms = float(tick_duration_ms)
    msg.overrun_count = int(overrun_count)
    msg.wheels = []
    for source in snapshot.wheels:
        wheel = wheel_factory()
        for field in (
            "name",
            "corner_mode",
            "drive_turns_per_s",
            "steer_deg",
            "drive_current_a",
            "steer_current_a",
            "drive_stale",
            "steer_stale",
            "drive_axis_error",
            "steer_fault",
        ):
            setattr(wheel, field, getattr(source, field))
        msg.wheels.append(wheel)
    return msg
