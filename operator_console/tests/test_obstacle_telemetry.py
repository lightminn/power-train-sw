import json

import pytest

from operator_console.obstacle_telemetry import (
    obstacle_source_state,
    parse_obstacle_telemetry,
)


def payload(**updates):
    value = {
        "schema_version": 1,
        "sequence": 7,
        "capture_timestamp_s": 1234.5,
        "state": "OK",
        "zones": {
            "left": {"valid": True, "detected": False, "distance_m": None},
            "center": {"valid": True, "detected": True, "distance_m": 1.42},
            "right": {"valid": True, "detected": False, "distance_m": None},
        },
        "processing_ms": 18.4,
        "detail": "",
    }
    value.update(updates)
    return json.dumps(value).encode()


def test_parses_frame_identity_state_and_three_zones():
    snapshot = parse_obstacle_telemetry(payload(), received_monotonic_s=10.0)
    assert snapshot.sequence == 7
    assert snapshot.capture_timestamp_s == 1234.5
    assert snapshot.zones["center"].distance_m == 1.42
    assert obstacle_source_state(snapshot, now_s=10.75) == "LIVE"
    assert obstacle_source_state(snapshot, now_s=10.751) == "STALE"


@pytest.mark.parametrize("state", ["NO_GROUND", "SENSOR_ERROR"])
def test_failure_state_requires_every_zone_to_be_invalid(state):
    invalid = {name: {"valid": False, "detected": None, "distance_m": None}
               for name in ("left", "center", "right")}
    assert parse_obstacle_telemetry(payload(state=state, zones=invalid)).state == state
    with pytest.raises(ValueError):
        parse_obstacle_telemetry(payload(state=state))


def test_invalid_zone_cannot_look_like_clear_or_carry_stale_distance():
    zones = json.loads(payload())["zones"]
    zones["left"] = {"valid": False, "detected": False, "distance_m": None}
    with pytest.raises(ValueError):
        parse_obstacle_telemetry(payload(zones=zones))


def test_detected_zone_requires_distance():
    zones = json.loads(payload())["zones"]
    zones["center"]["distance_m"] = None
    with pytest.raises(ValueError):
        parse_obstacle_telemetry(payload(zones=zones))
