from types import MappingProxyType

import pytest

from chassis.telemetry import (
    AkNodeHealth,
    CanBusHealth,
    ChassisSnapshot,
    OdriveNodeHealth,
    build_can_health_event,
)
from powertrain_ros.console_can_status import can_status_text


NOW_NS = 10_000_000_000


def _ak(can_id, *, stale=False):
    return AkNodeHealth(
        can_id=can_id,
        physical_wheel=f"ak-{can_id}",
        last_feedback_age_ms=10.0,
        feedback_rate_hz=50.0,
        steer_fault=0,
        stale=stale,
        recovery_count=0,
    )


def _odrive(node_id, *, stale=False):
    return OdriveNodeHealth(
        node_id=node_id,
        physical_wheel=f"odrive-{node_id}",
        last_heartbeat_age_ms=10.0,
        last_encoder_age_ms=10.0,
        axis_state=8,
        axis_error=0,
        stale=stale,
        recovery_count=0,
    )


def _record(
    *,
    monotonic_ns=NOW_NS,
    healthy=True,
    ak_nodes=(_ak(1), _ak(2)),
    odrive_nodes=(_odrive(11), _odrive(12), _odrive(13)),
    bus=CanBusHealth(),
):
    return build_can_health_event(
        ChassisSnapshot(
            chassis_mode="ARMED",
            stop_state="RUNNING",
            healthy=healthy,
            wheels=(),
            ak_nodes=ak_nodes,
            odrive_nodes=odrive_nodes,
            bus=bus,
        ),
        wall_time_ns=1,
        monotonic_ns=monotonic_ns,
    )


def _freeze(value):
    if isinstance(value, dict):
        return MappingProxyType(
            {key: _freeze(item) for key, item in value.items()}
        )
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


def test_can_status_reports_missing_owner_event():
    assert can_status_text(None, NOW_NS) == (
        "UNAVAILABLE · no CAN_HEALTH from chassis owner"
    )


def test_can_status_rejects_stale_owner_event():
    record = _record(monotonic_ns=NOW_NS - 3_100_000_000)

    assert can_status_text(record, NOW_NS) == (
        "UNAVAILABLE · CAN_HEALTH stale 3.1s"
    )


def test_can_status_rejects_future_monotonic_stamp():
    record = _record(monotonic_ns=NOW_NS + 1_000_000_000)

    assert can_status_text(record, NOW_NS) == (
        "UNAVAILABLE · CAN_HEALTH stale -1.0s"
    )


def test_can_status_formats_real_healthy_event_contract():
    record = _record()

    assert can_status_text(record, NOW_NS) == (
        "HEALTHY · AK 2/2 · ODrive 3/3"
    )


def test_can_status_accepts_frozen_observability_snapshot_shape():
    record = _freeze(_record())

    assert can_status_text(record, NOW_NS) == (
        "HEALTHY · AK 2/2 · ODrive 3/3"
    )


def test_can_status_uses_payload_health_for_unhealthy_label():
    record = _record(healthy=False)

    assert can_status_text(record, NOW_NS) == (
        "UNHEALTHY · AK 2/2 · ODrive 3/3"
    )


@pytest.mark.parametrize(
    ("bus", "expected_flag"),
    (
        (CanBusHealth(error_warning=True), "WARN"),
        (CanBusHealth(error_passive=True), "PASSIVE"),
        (CanBusHealth(bus_off_delta=2), "BUSOFF+2"),
        (
            CanBusHealth(
                error_warning=True,
                error_passive=True,
                bus_off_delta=2,
            ),
            "WARN,PASSIVE,BUSOFF+2",
        ),
    ),
)
def test_can_status_appends_active_bus_flags(bus, expected_flag):
    record = _record(bus=bus)

    assert can_status_text(record, NOW_NS) == (
        f"HEALTHY · AK 2/2 · ODrive 3/3 · bus {expected_flag}"
    )


def test_can_status_counts_non_stale_nodes_from_payload_lengths():
    record = _record(
        ak_nodes=(_ak(1), _ak(2, stale=True), _ak(3)),
        odrive_nodes=(
            _odrive(11, stale=True),
            _odrive(12),
            _odrive(13, stale=True),
            _odrive(14),
        ),
    )

    assert can_status_text(record, NOW_NS) == (
        "HEALTHY · AK 2/3 · ODrive 2/4"
    )


@pytest.mark.parametrize(
    "field",
    ("healthy", "ak_nodes", "odrive_nodes", "bus"),
)
def test_can_status_rejects_malformed_payload_fields(field):
    record = _record()
    replacements = {
        "healthy": "yes",
        "ak_nodes": {"not": "a list"},
        "odrive_nodes": {"not": "a list"},
        "bus": [],
    }
    record["payload"][field] = replacements[field]

    assert can_status_text(record, NOW_NS) == (
        "UNAVAILABLE · malformed CAN_HEALTH"
    )


def test_can_status_rejects_missing_required_payload_field():
    record = _record()
    del record["payload"]["healthy"]

    assert can_status_text(record, NOW_NS) == (
        "UNAVAILABLE · malformed CAN_HEALTH"
    )


def test_can_status_rejects_non_boolean_node_staleness():
    record = _record()
    record["payload"]["ak_nodes"][0]["stale"] = "false"

    assert can_status_text(record, NOW_NS) == (
        "UNAVAILABLE · malformed CAN_HEALTH"
    )
