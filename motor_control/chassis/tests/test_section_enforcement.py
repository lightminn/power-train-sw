"""Pure section-state enforcement and ChassisManager boundary contracts."""

import math

import pytest

from chassis.chassis_manager import ChassisConfig, ChassisManager
from chassis.kinematics import default_geometry
from chassis.section_enforcement import SectionEnforcer
from chassis.section_profiles import SectionConfig
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.fake import FakeDrive, FakeSteer
from corner_module.null_steer import NullSteer


class FakeClock:
    def __init__(self, now_s=10.0):
        self.now_s = float(now_s)

    def __call__(self):
        return self.now_s


def _payload(
    *,
    session_id="session-a",
    sequence=1,
    stamp_s=10.0,
    ttl_s=0.6,
    enabled=True,
    hold_hint=False,
    speed_hint=0.25,
):
    return {
        "schema_version": 1,
        "session_id": session_id,
        "sequence": sequence,
        "stamp_s": stamp_s,
        "ttl_s": ttl_s,
        "enabled": enabled,
        "hold_hint": hold_hint,
        "speed_hint": speed_hint,
    }


def _enforcer(now_s=10.0):
    clock = FakeClock(now_s)
    return SectionEnforcer(SectionConfig(), clock), clock


def test_fresh_positive_speed_hint_returns_velocity_cap():
    enforcer, _clock = _enforcer()
    enforcer.feed(_payload(speed_hint=0.25), received_s=10.0)

    decision = enforcer.decide(10.1)

    assert decision.v_cap == pytest.approx(0.25)
    assert decision.force_hold is False
    assert decision.reason == "speed_hint"


def test_hold_hint_forces_linear_and_angular_hold():
    enforcer, _clock = _enforcer()
    enforcer.feed(_payload(hold_hint=True), received_s=10.0)

    decision = enforcer.decide(10.1)

    assert decision.v_cap is None
    assert decision.force_hold is True
    assert decision.reason == "hold_hint"


def test_state_older_than_payload_ttl_fails_closed():
    enforcer, _clock = _enforcer()
    enforcer.feed(_payload(ttl_s=0.2), received_s=10.0)

    decision = enforcer.decide(10.200001)

    assert decision.force_hold is True
    assert decision.reason == "stale"


def test_retrograde_sequence_in_same_session_is_dropped_fail_closed():
    enforcer, _clock = _enforcer()
    enforcer.feed(_payload(sequence=2, speed_hint=0.3), received_s=10.0)
    enforcer.feed(_payload(sequence=1, speed_hint=0.1), received_s=10.1)

    decision = enforcer.decide(10.1)

    assert decision.force_hold is True
    assert decision.reason == "stale"


def test_stamp_more_than_half_second_in_future_is_dropped_fail_closed():
    enforcer, _clock = _enforcer()
    enforcer.feed(_payload(stamp_s=10.500001), received_s=10.0)

    decision = enforcer.decide(10.0)

    assert decision.force_hold is True
    assert decision.reason == "stale"


def test_new_session_accepts_sequence_restart():
    enforcer, _clock = _enforcer()
    enforcer.feed(
        _payload(session_id="session-a", sequence=10, speed_hint=0.3),
        received_s=10.0,
    )
    enforcer.feed(
        _payload(session_id="session-b", sequence=1, speed_hint=0.2),
        received_s=10.1,
    )

    decision = enforcer.decide(10.1)

    assert decision.force_hold is False
    assert decision.v_cap == pytest.approx(0.2)


def test_fresh_disabled_supervisor_remains_advisory_only():
    enforcer, _clock = _enforcer()
    enforcer.feed(
        _payload(enabled=False, hold_hint=True, speed_hint=0.1),
        received_s=10.0,
    )

    decision = enforcer.decide(10.1)

    assert decision.v_cap is None
    assert decision.force_hold is False
    assert decision.reason == "disabled"


def test_hint_below_reintroduced_drive_floor_fails_closed():
    enforcer, _clock = _enforcer()
    enforcer.feed(_payload(speed_hint=0.2), received_s=10.0)

    decision = enforcer.decide(10.1, floor_v_m_s=0.3)

    assert decision.v_cap is None
    assert decision.force_hold is True
    assert decision.reason == "hint_below_floor"


def test_cap_is_respected_at_corner_module_set_boundary():
    enforcer, _clock = _enforcer()
    enforcer.feed(_payload(speed_hint=0.2), received_s=10.0)
    decision = enforcer.decide(10.1)
    cfg = ChassisConfig()
    corners = {}
    for wheel in default_geometry().wheels:
        steer = FakeSteer() if wheel.steerable else NullSteer()
        corners[wheel.name] = CornerModule(steer, FakeDrive(), CornerConfig())
    manager = ChassisManager(corners, cfg=cfg)
    manager.connect()
    assert manager.arm() is True

    requested_v = 0.8
    final_v = max(-decision.v_cap, min(decision.v_cap, requested_v))
    manager.set(final_v, 0.0)
    manager.tick()

    circumference_m = 2.0 * math.pi * cfg.geometry.wheel_radius_m
    for corner in manager.corners.values():
        turns_per_s = corner.state()["drive"]["target_vel"]
        assert abs(turns_per_s * circumference_m) <= (
            decision.v_cap + 1e-12
        )
