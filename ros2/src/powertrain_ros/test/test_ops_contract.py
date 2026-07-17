"""A2a ops 채널 와이어 계약(스펙 r6 §3.1) — 디코드 엄격성·action 표."""
import json

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


def test_encode_response_is_newline_json():
    raw = oc.encode_response(
        request_id="r-1", status=oc.STATUS_PENDING, state_revision=4,
        detail="",
    )
    assert raw.endswith(b"\n")
    assert json.loads(raw)["status"] == "PENDING"
