import json
import math

import pytest

from operator_console.rover_3d import rover_state
from operator_console.telemetry import parse_telemetry


def snapshot(**values):
    return parse_telemetry(json.dumps(dict(schema_version=1, sequence=1, **values)).encode(),
                           received_monotonic_s=100)


def test_stale_and_missing_values_cannot_animate_or_show_old_currents():
    measured = snapshot(wheel_statuses=[dict(
        name="front_left", mode="ARMED", drive_turns_per_s=0.2,
        drive_current_a=3.5, steer_deg=0.0, stale=False,
    )])
    assert rover_state(measured, now_s=100.1)["wheels"]["front_left"]["steer_deg"] == 0
    stale = rover_state(measured, now_s=101.1)
    assert stale["live"] is False and stale["wheels"] == {}
    wheel_stale = snapshot(wheel_statuses=[dict(
        name="front_left", mode="ARMED", drive_turns_per_s=0.2,
        drive_current_a=3.5, stale=True,
    )])
    assert rover_state(wheel_stale, now_s=100.1)["wheels"]["front_left"]["drive_current_a"] is None
    assert rover_state(None)["suspension"] == {}


def test_suspension_requires_its_own_fresh_measurement():
    joints = dict(ml_rocker_joint=0.2, br_bogie_joint=-0.1)
    assert rover_state(snapshot(suspension_joints_rad=joints), now_s=100.1)["suspension"] == {}
    live = snapshot(suspension_joints_rad=joints, suspension_age_ms=10)
    assert rover_state(live, now_s=100.1)["suspension"] == joints
    assert rover_state(live, now_s=100.999)["suspension"] == {}


@pytest.mark.parametrize("values", [
    {"suspension_joints_rad": {"unknown_joint": 0.1}},
    {"suspension_joints_rad": {"ml_rocker_joint": float("nan")}},
    {"suspension_joints_rad": {"ml_rocker_joint": True}},
    {"suspension_joints_rad": {"ml_rocker_joint": 4 * math.pi}},
    {"suspension_age_ms": -1},
])
def test_invalid_suspension_input_is_rejected(values):
    with pytest.raises(ValueError):
        snapshot(**values)


def test_dummy_rehearsal_returns_to_cad_reference_and_moves_one_joint_only():
    from operator_console.rover_3d_demo import dummy_payload
    for index in range(321):
        payload = dummy_payload(index, index / 20)
        angles = tuple(payload["suspension_joints_rad"].values())
        assert sum(abs(angle) > 1e-9 for angle in angles) <= 1
        assert max(map(abs, angles)) <= math.radians(8) + 1e-9
        assert payload["roll_rad"] == payload["pitch_rad"] == 0
        assert all(wheel["drive_turns_per_s"] == 0 for wheel in payload["wheel_statuses"])
    for elapsed in (0, 4, 8, 12, 16):
        assert not any(dummy_payload(1, elapsed)["suspension_joints_rad"].values())
