import math
from types import SimpleNamespace

from powertrain_ros.message_adapter import (
    fill_safety_message,
    fill_wheel_states_message,
)


def test_fill_safety_message_uses_nan_for_missing_distance():
    msg = SimpleNamespace(header=SimpleNamespace())
    verdict = SimpleNamespace(
        status="CHECKING",
        distance_mm=None,
        estop_required=False,
        consecutive_failures=1,
        detail="waiting",
    )

    fill_safety_message(msg, verdict, stamp="stamp")

    assert msg.header.stamp == "stamp"
    assert msg.header.frame_id == "us100_link"
    assert msg.status == 0
    assert math.isnan(msg.distance_mm)
    assert msg.estop_required is False
    assert msg.consecutive_failures == 1
    assert msg.detail == "waiting"


def test_fill_wheel_states_uses_actual_snapshot_values():
    msg = SimpleNamespace(header=SimpleNamespace())
    wheel = SimpleNamespace(
        name="front_left",
        corner_mode="ARMED",
        drive_turns_per_s=1.2,
        steer_deg=3.0,
        drive_current_a=0.4,
        steer_current_a=0.2,
        drive_stale=False,
        steer_stale=False,
        drive_axis_error=0,
        steer_fault=0,
    )
    snapshot = SimpleNamespace(
        chassis_mode="ARMED",
        stop_state="RUN",
        healthy=True,
        wheels=(wheel,),
    )

    fill_wheel_states_message(
        msg,
        snapshot,
        "stamp",
        4.5,
        2,
        wheel_factory=SimpleNamespace,
    )

    assert msg.header.stamp == "stamp"
    assert msg.header.frame_id == "base_link"
    assert msg.chassis_mode == "ARMED"
    assert msg.stop_state == "RUN"
    assert msg.healthy is True
    assert msg.wheels[0].name == "front_left"
    assert msg.wheels[0].drive_turns_per_s == 1.2
    assert msg.tick_duration_ms == 4.5
    assert msg.overrun_count == 2
