"""A2a ops 채널 와이어 계약(스펙 r6 §3.1) — 디코드 엄격성·action 표."""
import json
import math

import pytest

from powertrain_ros import ops_contract as oc


def _request(**overrides):
    payload = {
        "schema_version": 1,
        "token": "tok",
        "request_id": "r-1",
        "sequence": 0,
        "action": "authority_manual",
        "params": {},
        "stamp_s": 1.0,
    }
    payload.update(overrides)
    return json.dumps(payload)


def test_decode_accepts_exact_fields_and_optional_extras():
    decoded = oc.decode_request(_request())
    assert decoded["action"] == "authority_manual"
    decoded = oc.decode_request(
        _request(expected_state_revision=3, phase="begin")
    )
    assert decoded["expected_state_revision"] == 3
    assert decoded["phase"] == "begin"


@pytest.mark.parametrize(
    "bad",
    [
        _request(schema_version=2),
        _request(action="rm_rf"),
        _request(sequence=-1),
        _request(request_id=""),
        "not json",
        json.dumps({"schema_version": 1}),
        _request(phase="maybe"),
    ],
)
def test_decode_rejects_contract_violations(bad):
    with pytest.raises(ValueError):
        oc.decode_request(bad)


def test_decode_rejects_oversized_record():
    with pytest.raises(ValueError):
        oc.decode_request(_request(params={"x": "y" * oc.MAX_RECORD_BYTES}))


@pytest.mark.parametrize("stamp_s", [math.nan, math.inf, -math.inf])
def test_decode_rejects_nonfinite_request_stamp(stamp_s):
    with pytest.raises(ValueError, match="stamp_s must be finite"):
        oc.decode_request(_request(stamp_s=stamp_s))


@pytest.mark.parametrize("stamp_s", [None, [], {}])
def test_decode_normalizes_invalid_request_stamp_type(stamp_s):
    with pytest.raises(ValueError, match="stamp_s must be finite"):
        oc.decode_request(_request(stamp_s=stamp_s))


def test_action_table_role_bindings_match_spec():
    assert oc.ACTIONS["estop_reset"].roles == frozenset({oc.ROLE_CONSOLE})
    assert oc.ACTIONS["estop_reset"].emergency_roles == frozenset(
        {oc.ROLE_CONTROLLER}
    )
    assert oc.ACTIONS["arm"].emergency_roles == frozenset({oc.ROLE_CONTROLLER})
    assert oc.ACTIONS["authority_manual"].roles == frozenset(
        {oc.ROLE_CONSOLE, oc.ROLE_CONTROLLER}
    )
    assert oc.ACTIONS["mission_skip"].roles == frozenset({oc.ROLE_CONSOLE})
    assert oc.ACTIONS["operator_hold"].kind == "publish"
    assert oc.ACTIONS["clear_transient_hold"].kind == "composite"
    assert oc.ACTIONS["status_query"].kind == "local"


def test_arm_lock_override_is_console_only_setbool():
    spec = oc.ACTIONS["arm_lock_override"]
    assert spec.roles == frozenset({oc.ROLE_CONSOLE})
    assert spec.kind == "service_setbool"
    assert spec.target == ("/chassis_node/arm_lock_override",)


def test_estop_is_console_only_chassis_service():
    spec = oc.ACTIONS["estop"]
    assert spec.roles == frozenset({oc.ROLE_CONSOLE})
    assert spec.kind == "service"
    assert spec.target == ("/chassis_node/estop",)


@pytest.mark.parametrize(
    ("action", "target"),
    [
        ("drive_enable", "/chassis_node/component_enable_drive"),
        ("steer_enable", "/chassis_node/component_enable_steer"),
        ("us100_enable", "/chassis_node/component_enable_us100"),
        ("robot_arm_enable", "/chassis_node/component_enable_robot_arm"),
    ],
)
def test_component_enable_actions_are_console_only_setbool(action, target):
    spec = oc.ACTIONS[action]
    assert spec.roles == frozenset({oc.ROLE_CONSOLE})
    assert spec.emergency_roles == frozenset()
    assert spec.kind == "service_setbool"
    assert spec.target == (target,)


def test_extraction_grant_is_console_only_chassis_service():
    spec = oc.ACTIONS["extraction_grant"]
    assert spec.roles == frozenset({oc.ROLE_CONSOLE})
    assert spec.kind == "service"
    assert spec.target == ("/chassis_node/extraction_grant",)


def test_mission_clear_grip_lost_is_console_only_setbool_service():
    spec = oc.ACTIONS["mission_clear_grip_lost"]
    assert spec.roles == frozenset({oc.ROLE_CONSOLE})
    assert spec.kind == "service_setbool"
    assert spec.target == ("/chassis_node/mission_clear_grip_lost",)


def test_never_ready_service_uses_shorter_abandon_deadline():
    assert oc.SERVICE_CALL_TIMEOUT_S < oc.SERVICE_UNAVAILABLE_ABANDON_S
    assert oc.SERVICE_UNAVAILABLE_ABANDON_S < oc.SERVICE_ORDER_ABANDON_S
    assert oc.service_abandon_timeout_s(service_was_ready=False) == 3.0
    assert (
        oc.service_abandon_timeout_s(service_was_ready=True)
        == oc.SERVICE_ORDER_ABANDON_S
    )


def test_encode_response_is_newline_json():
    raw = oc.encode_response(
        request_id="r-1", status=oc.STATUS_PENDING, state_revision=4,
        detail="",
    )
    assert raw.endswith(b"\n")
    assert json.loads(raw)["status"] == "PENDING"
