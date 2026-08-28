import json
import math

import pytest

from operator_console.environment_telemetry import (
    ENVIRONMENT_STALE_AFTER_S,
    environment_source_state,
    parse_environment_telemetry,
)


def _payload(**overrides):
    payload = {
        "schema_version": 1,
        "sequence": 12,
        "source": "rpi-environment-end-effector",
        "source_timestamp": "2026-08-28T22:32:11+09:00",
        "sensor_ok": True,
        "errors": [],
        "temperature_c": 29.08,
        "humidity_pct": 56.87,
        "pressure_hpa": 1003.14,
        "eco2_ppm": 401.0,
        "tvoc_ppb": 1.0,
        "sgp30_warming_up": False,
        "co_estimated_ppm": 1.0,
        "co_rs_ro": 1.019,
        "co_range": "below_detection_range",
        "co_quality": "uncalibrated_estimate",
        "lpg_estimated_ppm": 27.9,
        "lpg_rs_ro": 1.012,
        "lpg_range": "below_detection_range",
        "lpg_quality": "uncalibrated_estimate",
        "flame_detected": False,
        "flame_voltage_v": 2.754,
        "flame_threshold_v": 1.5,
    }
    payload.update(overrides)
    return json.dumps(payload, separators=(",", ":")).encode("utf-8")


def test_environment_packet_round_trips_every_operator_value():
    snapshot = parse_environment_telemetry(
        _payload(), received_monotonic_s=10.0,
    )

    assert snapshot.sequence == 12
    assert snapshot.source == "rpi-environment-end-effector"
    assert snapshot.source_timestamp == "2026-08-28T22:32:11+09:00"
    assert snapshot.sensor_ok is True
    assert snapshot.temperature_c == 29.08
    assert snapshot.humidity_pct == 56.87
    assert snapshot.pressure_hpa == 1003.14
    assert snapshot.eco2_ppm == 401.0
    assert snapshot.tvoc_ppb == 1.0
    assert snapshot.co_estimated_ppm == 1.0
    assert snapshot.co_rs_ro == 1.019
    assert snapshot.lpg_estimated_ppm == 27.9
    assert snapshot.lpg_rs_ro == 1.012
    assert snapshot.flame_detected is False
    assert snapshot.flame_voltage_v == 2.754
    assert snapshot.received_monotonic_s == 10.0


def test_environment_freshness_allows_one_hz_sender_jitter():
    snapshot = parse_environment_telemetry(
        _payload(), received_monotonic_s=10.0,
    )

    assert environment_source_state(
        snapshot, now_s=10.0 + ENVIRONMENT_STALE_AFTER_S,
    ) == "LIVE"
    assert environment_source_state(
        snapshot, now_s=10.01 + ENVIRONMENT_STALE_AFTER_S,
    ) == "STALE"


@pytest.mark.parametrize("value", (math.inf, -math.inf, math.nan))
def test_non_finite_environment_values_are_rejected(value):
    with pytest.raises(ValueError):
        parse_environment_telemetry(_payload(temperature_c=value))


def test_environment_schema_and_packet_size_are_bounded():
    with pytest.raises(ValueError):
        parse_environment_telemetry(_payload(schema_version=2))
    with pytest.raises(ValueError):
        parse_environment_telemetry(b"{" + b"x" * 4096)


def test_environment_errors_are_bounded():
    with pytest.raises(ValueError):
        parse_environment_telemetry(_payload(errors=["fault"] * 9))
    with pytest.raises(ValueError):
        parse_environment_telemetry(_payload(errors=["x" * 161]))
