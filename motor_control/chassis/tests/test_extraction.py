"""Spec r6 §6.1 reverse-only extraction contracts for ChassisManager."""

import pytest

from chassis.chassis_manager import ChassisConfig, ChassisManager
from chassis.kinematics import default_geometry, solve
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.fake import FakeDrive, FakeSteer
from corner_module.null_steer import NullSteer


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t

    def advance(self, seconds):
        self.t += seconds


def _fake_corners():
    corners = {}
    for wheel in default_geometry().wheels:
        steer = FakeSteer() if wheel.steerable else NullSteer()
        corners[wheel.name] = CornerModule(
            steer,
            FakeDrive(),
            CornerConfig(),
        )
    return corners


def _manager(**config_overrides):
    clock = FakeClock()
    cfg = ChassisConfig(extraction_enabled=True, **config_overrides)
    manager = ChassisManager(_fake_corners(), cfg=cfg, clock=clock)
    manager.connect()
    assert manager.arm() is True
    return manager, clock


def _trip_us100(manager):
    manager.update_external_safety("VALID", True, "too close")
    manager.tick()
    assert manager.mode == "ESTOP"


def _grant(manager):
    _trip_us100(manager)
    assert manager.extraction_grant() is True
    assert manager.mode == "EXTRACTION"


def _drive_targets(manager):
    return {
        name: corner.state()["drive"]["target_vel"]
        for name, corner in manager.corners.items()
    }


def test_flag_off_rejects_extraction_grant():
    manager = ChassisManager(_fake_corners(), cfg=ChassisConfig())
    manager.connect()
    assert manager.arm() is True
    _trip_us100(manager)

    assert manager.extraction_grant() is False
    assert manager.snapshot().last_extraction_reject == "disabled"


def test_extraction_grant_requires_latched_estop():
    manager, _clock = _manager()

    assert manager.extraction_grant() is False
    assert manager.snapshot().last_extraction_reject == "estop_not_latched"


def test_extraction_grant_rejects_us100_with_another_active_estop():
    manager, _clock = _manager()
    manager.update_external_safety("VALID", True, "too close")
    manager.set_safety_link_stale(True, "topic stale")
    manager.tick()

    assert manager.extraction_grant() is False
    assert manager.snapshot().last_extraction_reject == (
        "active_estop_sources_not_us100_only"
    )


def test_extraction_grant_rearms_corners_and_preserves_latch():
    manager, _clock = _manager()
    _trip_us100(manager)

    assert manager.extraction_grant() is True

    safety = manager.safety_snapshot()
    snapshot = manager.snapshot()
    assert manager.mode == "EXTRACTION"
    assert all(corner.mode == "ARMED" for corner in manager.corners.values())
    assert safety.estop_latched is True
    assert safety.active_estop_sources == ("us100",)
    assert snapshot.extraction_active is True
    assert snapshot.extraction_remaining_s == pytest.approx(3.0)
    assert snapshot.extraction_budget_left_m == pytest.approx(1.0)
    assert snapshot.extraction_grants_left == 2
    assert snapshot.last_extraction_reject == ""


def test_extraction_clamps_forward_command_to_zero():
    manager, _clock = _manager()
    _grant(manager)

    manager.set(0.5, 0.7)
    manager.tick()

    assert all(target == 0.0 for target in _drive_targets(manager).values())
    assert all(
        corner.state()["steer"]["target_deg"] == pytest.approx(0.0)
        for corner in manager.corners.values()
    )


def test_extraction_clamps_reverse_speed_and_forces_zero_yaw():
    manager, _clock = _manager()
    _grant(manager)

    manager.set(-0.5, 0.7)
    manager.tick()

    expected = solve(default_geometry(), -0.2, 0.0)
    for name, corner in manager.corners.items():
        state = corner.state()
        assert state["steer"]["target_deg"] == pytest.approx(
            expected.wheels[name].steer_deg
        )
        assert state["drive"]["target_vel"] == pytest.approx(
            expected.wheels[name].drive_turns_per_s
        )


def test_extraction_drives_backward_while_us100_remains_active():
    manager, _clock = _manager()
    _grant(manager)

    manager.set(-0.1, 0.0)
    manager.tick()

    assert manager.safety_snapshot().active_estop_sources == ("us100",)
    assert manager.mode == "EXTRACTION"
    assert all(target < 0.0 for target in _drive_targets(manager).values())


def test_extraction_ttl_expiry_returns_to_estop_with_latch_preserved():
    manager, clock = _manager(extraction_ttl_s=3.0)
    _grant(manager)
    manager.set(-0.2, 0.0)
    manager.tick()

    clock.advance(3.01)
    manager.tick()

    assert manager.mode == "ESTOP"
    assert manager.safety_snapshot().estop_latched is True
    assert manager.snapshot().extraction_budget_left_m == pytest.approx(0.4)
    assert all(corner.mode == "FAULT" for corner in manager.corners.values())
    assert all(target == 0.0 for target in _drive_targets(manager).values())
    assert manager.reset_estop() is False


def test_extraction_aborts_when_another_estop_source_appears():
    manager, _clock = _manager()
    _grant(manager)
    manager.set(-0.1, 0.0)

    manager.set_safety_link_stale(True, "topic stale")
    manager.tick()

    assert manager.mode == "ESTOP"
    assert all(corner.mode == "FAULT" for corner in manager.corners.values())
    assert all(target == 0.0 for target in _drive_targets(manager).values())


def test_extraction_cumulative_distance_budget_aborts_at_one_meter():
    manager, clock = _manager(
        extraction_ttl_s=10.0,
        watchdog_ms=10_000.0,
    )
    _grant(manager)
    manager.set(-0.2, 0.0)
    manager.tick()

    clock.advance(2.5)
    manager.tick()
    assert manager.mode == "EXTRACTION"
    assert manager.snapshot().extraction_budget_left_m == pytest.approx(0.5)

    clock.advance(2.5)
    manager.tick()
    assert manager.mode == "ESTOP"
    assert manager.snapshot().extraction_budget_left_m == pytest.approx(0.0)
    assert manager.safety_snapshot().estop_latched is True


def test_extraction_rejects_fourth_grant_in_same_episode():
    manager, clock = _manager(extraction_ttl_s=0.1)
    _trip_us100(manager)

    for _ in range(3):
        assert manager.extraction_grant() is True
        clock.advance(0.11)
        manager.tick()
        assert manager.mode == "ESTOP"

    assert manager.extraction_grant() is False
    assert manager.snapshot().extraction_grants_left == 0
    assert manager.snapshot().last_extraction_reject == "grant_limit_exhausted"


def test_successful_reset_estop_resets_extraction_episode_counters():
    manager, clock = _manager(watchdog_ms=10_000.0)
    _grant(manager)
    manager.set(-0.2, 0.0)
    manager.tick()
    clock.advance(1.0)
    manager.tick()
    assert manager.snapshot().extraction_budget_left_m == pytest.approx(0.8)

    manager.update_external_safety("VALID", False, "clear")
    assert manager.reset_estop() is True

    snapshot = manager.snapshot()
    assert manager.mode == "IDLE"
    assert snapshot.extraction_active is False
    assert snapshot.extraction_remaining_s == 0.0
    assert snapshot.extraction_budget_left_m == pytest.approx(1.0)
    assert snapshot.extraction_grants_left == 3
