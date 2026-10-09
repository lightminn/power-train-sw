import json

import pytest

from operator_console.arm_telemetry import parse_arm_telemetry
from operator_console.demo_source import (
    DEFAULT_TEST_PORTS,
    build_payloads,
    scenario_at,
    validate_ports,
)
from operator_console.environment_telemetry import parse_environment_telemetry
from operator_console.metadata import parse_metadata
from operator_console.telemetry import parse_telemetry


def _raw(payload: dict) -> bytes:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()


def test_demo_scenarios_cover_nominal_assist_and_safe_hold():
    assert scenario_at(0.0) == "NORMAL"
    assert scenario_at(7.0) == "ROUGH_APPROACH"
    assert scenario_at(13.0) == "TRACTION_ASSIST"
    assert scenario_at(19.0) == "BLOCKED_HOLD"
    assert scenario_at(24.0) == "NORMAL"

    assist = build_payloads(3, 13.0)["chassis"]
    assert assist["controller_fsm_state"] == "TRACTION_ASSIST"
    assert assist["slip_candidate"] is True
    assert len({wheel["command_turns_per_s"] for wheel in assist["wheel_statuses"]}) > 1

    hold = build_payloads(4, 19.0)["chassis"]
    assert hold["terrain_path_available"] is False
    assert hold["stuck_candidate"] is True
    assert all(wheel["command_turns_per_s"] == 0.0 for wheel in hold["wheel_statuses"])


def test_every_demo_channel_uses_the_production_decoder_and_is_bounded():
    payloads = build_payloads(11, 13.0)
    encoded = {channel: _raw(payload) for channel, payload in payloads.items()}
    assert all(len(raw) <= 4096 for raw in encoded.values())

    assert parse_telemetry(encoded["power"], 10.0).sequence == 11
    chassis = parse_telemetry(encoded["chassis"], 10.0)
    assert chassis.controller_fsm_state == "TRACTION_ASSIST"
    assert len(chassis.wheel_statuses) == 6
    assert parse_metadata(encoded["metadata"], 10.0).detections[0].depth_m == 1.28
    assert parse_arm_telemetry(encoded["arm"], 10.0).end_effector_type == "dual_gripper"
    assert parse_environment_telemetry(encoded["environment"], 10.0).sensor_ok is True


def test_demo_source_rejects_live_or_colliding_ports():
    validate_ports(dict(DEFAULT_TEST_PORTS))
    with pytest.raises(ValueError, match="live UDP ports"):
        validate_ports(dict(DEFAULT_TEST_PORTS, chassis=5005))
    with pytest.raises(ValueError, match="distinct"):
        validate_ports(dict(DEFAULT_TEST_PORTS, chassis=15004))
