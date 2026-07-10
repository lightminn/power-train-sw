from dataclasses import FrozenInstanceError

import pytest

from chassis.safety_interlock import ESTOP, MOTION_HOLD, RUN, SafetyInterlock


class FakeClock:
    def __init__(self):
        self.t = 10.0

    def __call__(self):
        return self.t


class FalseyClock(FakeClock):
    def __bool__(self):
        return False


def test_falsey_injected_clock_is_honored():
    clock = FalseyClock()
    interlock = SafetyInterlock(clock=clock)
    interlock.trip_estop("manual")
    assert interlock.snapshot().tripped_at_s == 10.0


def test_hold_auto_clears_without_latching():
    interlock = SafetyInterlock()
    interlock.set_motion_hold("cmd_timeout", True, "no command")
    assert interlock.snapshot().state == MOTION_HOLD
    interlock.set_motion_hold("cmd_timeout", False)
    assert interlock.snapshot().state == RUN


def test_estop_latches_first_cause_and_is_idempotent():
    clock = FakeClock()
    interlock = SafetyInterlock(clock=clock)
    interlock.trip_estop("manual", "circle button")
    clock.t = 20.0
    interlock.trip_estop("manual", "repeat")
    snap = interlock.snapshot()
    assert snap.state == ESTOP
    assert snap.first_source == "manual"
    assert snap.first_detail == "circle button"
    assert snap.tripped_at_s == 10.0


def test_active_condition_rejects_reset_then_allows_it_after_clear():
    interlock = SafetyInterlock()
    interlock.set_estop_condition("us100", True, "too close")
    assert interlock.reset_estop() is False
    interlock.set_estop_condition("us100", False)
    assert interlock.snapshot().state == ESTOP
    assert interlock.reset_estop() is True
    assert interlock.snapshot().state == RUN


def test_reset_waits_until_all_active_estop_sources_clear():
    interlock = SafetyInterlock()
    interlock.set_estop_condition("us100", True, "too close")
    interlock.set_estop_condition("motor_fault", True, "node 12")
    assert interlock.reset_estop() is False
    interlock.set_estop_condition("us100", False)
    assert interlock.reset_estop() is False
    interlock.set_estop_condition("motor_fault", False)
    assert interlock.reset_estop() is True


def test_snapshot_source_tuples_are_sorted_and_detached():
    interlock = SafetyInterlock()
    interlock.set_motion_hold("mission", True)
    interlock.set_motion_hold("cmd_timeout", True)
    interlock.set_estop_condition("us100", True)
    interlock.set_estop_condition("motor_fault", True)
    snap = interlock.snapshot()

    assert snap.active_estop_sources == ("motor_fault", "us100")
    assert snap.hold_sources == ("cmd_timeout", "mission")

    interlock.set_motion_hold("cmd_timeout", False)
    interlock.set_estop_condition("motor_fault", False)
    assert snap.active_estop_sources == ("motor_fault", "us100")
    assert snap.hold_sources == ("cmd_timeout", "mission")


def test_snapshot_is_frozen():
    snap = SafetyInterlock().snapshot()
    with pytest.raises(FrozenInstanceError):
        snap.state = ESTOP


def test_hold_clear_does_not_clear_estop():
    interlock = SafetyInterlock()
    interlock.set_motion_hold("mission", True)
    interlock.trip_estop("motor_fault", "node 12")
    interlock.set_motion_hold("mission", False)
    assert interlock.snapshot().state == ESTOP
