import math

import pytest

from powertrain_observability.events import (
    KNOWN_EVENT_TYPES,
    MAX_RECORD_BYTES,
    decode_event,
    encode_event,
    validate_event,
)


EXPECTED_EVENT_TYPES = {
    "FSM_TRANSITION",
    "COMMAND_OWNER",
    "MOTION_HOLD",
    "ESTOP",
    "MISSION",
    "ARM_RESULT",
    "GRIP_LOST",
    "CONTRACT_VIOLATION",
    "OPERATOR_ACTION",
    "TERRAIN_REJECT",
    "CHANNEL_HEALTH",
    "CAN_HEALTH",
}


def _event(**overrides):
    event = {
        "schema_version": 1,
        "run_id": "run-20260715",
        "sequence": 7,
        "wall_time_ns": 1_750_000_000_000_000_000,
        "monotonic_ns": 123_456_789,
        "source": "chassis_node",
        "event_type": "MISSION",
        "severity": "INFO",
        "payload": {"mission_id": 3, "segment": "DOOR"},
    }
    event.update(overrides)
    return event


def test_known_event_types_match_the_plan_contract():
    assert set(KNOWN_EVENT_TYPES) == EXPECTED_EVENT_TYPES


@pytest.mark.parametrize(
    "missing",
    (
        "schema_version",
        "run_id",
        "sequence",
        "wall_time_ns",
        "monotonic_ns",
        "source",
        "event_type",
        "severity",
        "payload",
    ),
)
def test_validation_rejects_each_missing_required_field(missing):
    event = _event()
    del event[missing]

    with pytest.raises(ValueError, match=missing):
        validate_event(event)


def test_unknown_event_type_and_extra_payload_keys_round_trip_unchanged():
    event = _event(
        event_type="TEAM_EXTENSION_EVENT",
        payload={
            "vendor_detail": "preserved",
            "nested": {"future_key": [1, True, None]},
        },
    )

    assert decode_event(encode_event(event)) == event


def test_json_encoding_is_deterministic_for_different_insertion_orders():
    first = _event(payload={"z": 1, "a": 2})
    second = dict(reversed(list(first.items())))
    second["payload"] = {"a": 2, "z": 1}

    assert encode_event(first) == encode_event(second)
    assert encode_event(first).startswith(b'{"event_type":"MISSION"')


@pytest.mark.parametrize("non_finite", (math.nan, math.inf, -math.inf))
def test_validation_rejects_non_finite_values_anywhere_in_record(non_finite):
    event = _event(payload={"quality": {"value": non_finite}})

    with pytest.raises(ValueError, match="finite"):
        validate_event(event)


def test_encoding_rejects_oversize_record():
    event = _event(payload={"blob": "x" * MAX_RECORD_BYTES})

    with pytest.raises(ValueError, match="too large"):
        encode_event(event)
