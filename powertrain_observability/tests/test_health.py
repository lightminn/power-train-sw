from dataclasses import FrozenInstanceError

import pytest

from powertrain_observability.health import HealthState


def test_health_snapshot_is_immutable_and_detached_from_later_updates():
    health = HealthState()
    original = health.snapshot()

    with pytest.raises(FrozenInstanceError):
        original.status = "DEGRADED"

    health.record_drop()
    updated = health.snapshot()

    assert original.status == "OK"
    assert original.drop_count == 0
    assert updated.status == "OK"
    assert updated.drop_count == 1


def test_mark_degraded_preserves_diagnostic_reason():
    health = HealthState()

    health.mark_degraded("journal flush failed")

    snapshot = health.snapshot()
    assert snapshot.status == "DEGRADED"
    assert snapshot.last_error == "journal flush failed"
