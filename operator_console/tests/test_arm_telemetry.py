import json
from pathlib import Path

import pytest

from operator_console import arm_telemetry
from operator_console.arm_telemetry import parse_arm_telemetry, temperature_state


def _payload(**overrides):
    payload = {
        "schema_version": 1,
        "sequence": 21,
        "stamp_s": 123.5,
        "dynamixel": [
            {
                "id": 11,
                "position_raw": 2048,
                "position_deg": 0.0,
                "velocity": 7,
                "current": -12,
                "temperature_c": 34,
            },
            {
                "id": 12,
                "position_raw": 3072,
                "position_deg": 90.0,
                "velocity": -3,
                "current": 18,
                "temperature_c": 55,
            },
        ],
        "joints": {
            "names": ["joint_a", "joint_b"],
            "position_rad": [0.25, -0.5],
            "velocity": [0.1, -0.2],
        },
        "source_age_s": {
            "dynamixel": 0.1,
            "joints": 0.2,
            "detections": 0.3,
        },
        "truncated": False,
    }
    payload.update(overrides)
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def test_parse_arm_telemetry_round_trips_motors_joints_and_ages():
    snapshot = parse_arm_telemetry(_payload(), received_monotonic_s=10.0)

    assert snapshot.sequence == 21
    assert snapshot.dynamixel is not None
    assert len(snapshot.dynamixel) == 2
    assert snapshot.dynamixel[0].id == 11
    assert snapshot.dynamixel[0].position_raw == 2048
    assert snapshot.dynamixel[0].position_deg == 0.0
    assert snapshot.dynamixel[0].velocity == 7
    assert snapshot.dynamixel[0].current == -12
    assert snapshot.dynamixel[0].temperature_c == 34
    assert snapshot.joint_names == ("joint_a", "joint_b")
    assert snapshot.joint_position_rad == (0.25, -0.5)
    assert snapshot.joint_velocity == (0.1, -0.2)
    assert snapshot.dynamixel_age_s == 0.1
    assert snapshot.joints_age_s == 0.2
    assert snapshot.detections_age_s == 0.3
    assert snapshot.received_monotonic_s == 10.0


def test_null_sources_remain_explicitly_unavailable():
    snapshot = parse_arm_telemetry(
        _payload(dynamixel=None, joints=None, source_age_s={}),
        received_monotonic_s=10.0,
    )

    assert snapshot.dynamixel is None
    assert snapshot.joint_names == ()
    assert snapshot.joint_position_rad == ()
    assert snapshot.joint_velocity == ()
    assert snapshot.dynamixel_age_s is None
    assert snapshot.joints_age_s is None
    assert snapshot.detections_age_s is None


def test_more_than_eight_dynamixel_motors_is_rejected():
    motor = {
        "id": 11,
        "position_raw": 2048,
        "position_deg": 0.0,
        "velocity": 0,
        "current": 0,
        "temperature_c": 30,
    }

    with pytest.raises(ValueError):
        parse_arm_telemetry(_payload(dynamixel=[motor] * 9))


def test_mismatched_joint_array_lengths_are_rejected():
    joints = {
        "names": ["joint_a", "joint_b"],
        "position_rad": [0.25],
        "velocity": [0.1, -0.2],
    }

    with pytest.raises(ValueError):
        parse_arm_telemetry(_payload(joints=joints))


def test_unsupported_arm_telemetry_schema_is_rejected():
    with pytest.raises(ValueError):
        parse_arm_telemetry(_payload(schema_version=2))


def test_boolean_arm_telemetry_schema_is_rejected():
    with pytest.raises(ValueError):
        parse_arm_telemetry(_payload(schema_version=True))


def test_arm_telemetry_over_4096_bytes_is_rejected():
    with pytest.raises(ValueError):
        parse_arm_telemetry(b"{" + b"x" * 4096)


def test_receiver_reads_past_limit_to_detect_oversize_datagrams():
    source = (Path(__file__).parents[1] / "arm_telemetry.py").read_text(
        encoding="utf-8",
    )

    assert "recvfrom(4097)" in source


def test_truncated_flag_round_trips():
    snapshot = parse_arm_telemetry(_payload(truncated=True))

    assert snapshot.truncated is True


@pytest.mark.parametrize(
    ("temperature_c", "expected"),
    ((54, "NORMAL"), (55, "WARN"), (64, "WARN"), (65, "CRIT")),
)
def test_temperature_state_boundaries(temperature_c, expected):
    assert temperature_state(temperature_c) == expected


def test_arm_summary_covers_normal_unavailable_and_critical_temperature():
    summary = getattr(arm_telemetry, "arm_summary", None)
    assert summary is not None
    normal = parse_arm_telemetry(_payload(
        dynamixel=[
            {
                "id": 11,
                "position_raw": 2048,
                "position_deg": 0.0,
                "velocity": 0,
                "current": 0,
                "temperature_c": 45,
            },
            {
                "id": 12,
                "position_raw": 2048,
                "position_deg": 0.0,
                "velocity": 0,
                "current": 0,
                "temperature_c": 40,
            },
        ],
    ))
    critical = parse_arm_telemetry(_payload(
        dynamixel=[
            {
                "id": 12,
                "position_raw": 2048,
                "position_deg": 0.0,
                "velocity": 0,
                "current": 0,
                "temperature_c": 65,
            },
        ],
    ))

    assert summary(normal) == "모터 2 · 최고 45 ℃ 정상"
    assert summary(None) == "미수신(UNAVAILABLE)"
    assert summary(parse_arm_telemetry(_payload(dynamixel=None))) == (
        "미수신(UNAVAILABLE)"
    )
    assert summary(critical) == "모터 1 · 최고 ⚠ 65 ℃"
