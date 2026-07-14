"""Contract-v2 tests for the ROS-independent arm interlock core.

The first 43 cases preserve the checks from the reviewed boundary-test seed.
The final two cases cover the 2026-07-14 stamp-handling corrections.
"""

import random

from powertrain_ros import contract
from powertrain_ros.arm_interlock import ArmInterlock


def _gate_with(status="STOWED_LOCKED", mission_id=0, stamp_s=10.0, now_s=10.0):
    gate = ArmInterlock()
    assert gate.update(status, mission_id, stamp_s, now_s)
    return gate


# Plan Step 1 and boundary-seed checks 1-5.
def test_contract_v2_mission_stop_is_not_a_lock_mode():
    assert contract.MODE_MISSION_STOP == "MISSION_STOP"
    assert contract.MODE_STOW_REQUEST == "STOW_REQUEST"
    assert contract.MODE_MISSION_STOP not in contract.LOCK_MODES


def test_contract_v2_driving_is_a_default_deny_lock_mode():
    assert contract.MODE_DRIVING in contract.LOCK_MODES


def test_update_accepts_fresh_stowed_locked_status():
    gate = ArmInterlock(timeout_s=0.5)
    assert gate.update("STOWED_LOCKED", 0, stamp_s=10.0, now_s=10.0)


def test_locked_status_allows_drive_just_inside_timeout():
    gate = ArmInterlock(timeout_s=0.5)
    assert gate.update("STOWED_LOCKED", 0, stamp_s=10.0, now_s=10.0)
    assert gate.drive_allowed("EMPTY_STOWED", now_s=10.49)


def test_locked_status_denies_drive_just_outside_timeout():
    gate = ArmInterlock(timeout_s=0.5)
    assert gate.update("STOWED_LOCKED", 0, stamp_s=10.0, now_s=10.0)
    assert not gate.drive_allowed("EMPTY_STOWED", now_s=10.51)


# Stamp-boundary checks 6-13.
def test_update_rejects_duplicate_stamp():
    gate = _gate_with()
    assert not gate.update("STOWED_LOCKED", 0, 10.0, 10.1)


def test_update_rejects_backward_stamp():
    gate = _gate_with()
    assert not gate.update("STOWED_LOCKED", 0, 9.9, 10.2)


def test_update_rejects_zero_stamp():
    assert not ArmInterlock().update("STOWED_LOCKED", 0, 0.0, 10.0)


def test_update_rejects_stamp_beyond_future_tolerance():
    assert not ArmInterlock().update("STOWED_LOCKED", 0, 10.2, 10.0)


def test_update_accepts_stamp_at_or_inside_future_tolerance():
    assert ArmInterlock().update("STOWED_LOCKED", 0, 10.05, 10.0)


def test_update_rejects_nan_stamp():
    assert not ArmInterlock().update("STOWED_LOCKED", 0, float("nan"), 10.0)


def test_future_tolerated_stamp_denies_drive_while_age_is_negative():
    gate = _gate_with(stamp_s=10.05, now_s=10.0)
    assert not gate.drive_allowed("EMPTY_STOWED", now_s=10.01)


def test_future_tolerated_stamp_allows_drive_once_age_is_nonnegative():
    gate = _gate_with(stamp_s=10.05, now_s=10.0)
    assert gate.drive_allowed("EMPTY_STOWED", now_s=10.06)


# Clock-rollback checks 14-17.
def test_update_rejects_local_clock_rollback():
    gate = _gate_with()
    assert not gate.update("STOWED_LOCKED", 0, 10.5, 9.0)


def test_update_clock_rollback_invalidates_previous_sample():
    gate = _gate_with()
    assert not gate.update("STOWED_LOCKED", 0, 10.5, 9.0)
    assert not gate.drive_allowed("EMPTY_STOWED", now_s=9.4)


def test_update_clock_rollback_recovers_on_new_monotonic_heartbeat():
    gate = _gate_with()
    assert not gate.update("STOWED_LOCKED", 0, 10.5, 9.0)
    assert gate.update("STOWED_LOCKED", 0, 9.5, 9.5)
    assert gate.drive_allowed("EMPTY_STOWED", now_s=9.6)


def test_fresh_clock_rollback_invalidates_previous_sample():
    gate = _gate_with()
    assert not gate.fresh(9.0)
    assert not gate.drive_allowed("EMPTY_STOWED", now_s=9.4)


# Contract and drive-profile checks 18-24.
def test_update_rejects_unrecognized_status():
    assert not ArmInterlock().update("LOCKED", 0, 10.0, 10.0)


def test_unrecognized_status_records_contract_violation():
    gate = ArmInterlock()
    assert not gate.update("LOCKED", 0, 10.0, 10.0)
    assert gate.last_contract_violation == "LOCKED"


def test_contract_violation_denies_drive():
    gate = ArmInterlock()
    assert not gate.update("LOCKED", 0, 10.0, 10.0)
    assert not gate.drive_allowed("EMPTY_STOWED", now_s=10.1)


def test_contract_violation_cannot_be_overridden():
    gate = ArmInterlock()
    assert not gate.update("LOCKED", 0, 10.0, 10.0)
    assert not gate.drive_allowed(
        "REMOTE_ARM_OVERRIDE", now_s=10.1, manual_override=True
    )


def test_carrying_status_denies_empty_stowed_profile():
    gate = _gate_with(status="CARRYING_LOCKED")
    assert not gate.drive_allowed("EMPTY_STOWED", now_s=10.1)


def test_carrying_status_allows_carrying_locked_profile():
    gate = _gate_with(status="CARRYING_LOCKED")
    assert gate.drive_allowed("CARRYING_LOCKED", now_s=10.1)


def test_unknown_drive_profile_is_denied():
    gate = _gate_with(status="CARRYING_LOCKED")
    assert not gate.drive_allowed("TURBO", now_s=10.1)


# GRIP_LOST latch checks 25-29.
def _grip_lost_gate():
    gate = _gate_with(status="CARRYING_LOCKED")
    assert gate.update("GRIP_LOST", 0, 10.1, 10.1)
    return gate


def test_grip_lost_latch_denies_drive():
    gate = _grip_lost_gate()
    assert not gate.drive_allowed("CARRYING_LOCKED", now_s=10.2)


def test_locked_heartbeat_does_not_clear_grip_lost_latch():
    gate = _grip_lost_gate()
    assert gate.update("CARRYING_LOCKED", 0, 10.3, 10.3)
    assert not gate.drive_allowed("CARRYING_LOCKED", now_s=10.35)


def test_grip_lost_latch_cannot_be_overridden():
    gate = _grip_lost_gate()
    assert not gate.drive_allowed(
        "REMOTE_ARM_OVERRIDE", now_s=20.0, manual_override=True
    )


def test_grip_lost_latch_rejects_unauthorized_clear():
    gate = _grip_lost_gate()
    assert not gate.clear_grip_lost(authorized=False)


def test_grip_lost_latch_recovers_after_authorized_clear():
    gate = _grip_lost_gate()
    assert gate.update("CARRYING_LOCKED", 0, 10.3, 10.3)
    assert gate.clear_grip_lost(authorized=True)
    assert gate.drive_allowed("CARRYING_LOCKED", now_s=10.4)


# Manual-override checks 30-36.
def test_no_status_allows_remote_arm_override():
    gate = ArmInterlock()
    assert gate.drive_allowed(
        "REMOTE_ARM_OVERRIDE", now_s=5.0, manual_override=True
    )


def test_no_status_override_rejects_normal_profile():
    gate = ArmInterlock()
    assert not gate.drive_allowed("EMPTY_STOWED", now_s=5.0, manual_override=True)


def test_fresh_status_inhibits_remote_arm_override():
    gate = _gate_with()
    assert not gate.drive_allowed(
        "REMOTE_ARM_OVERRIDE", now_s=10.2, manual_override=True
    )


def test_fresh_status_override_reports_inhibited_hold_reason():
    gate = _gate_with()
    assert (
        gate.hold_reason("REMOTE_ARM_OVERRIDE", 10.2, manual_override=True)
        == "operator_override_inhibited_by_fresh_arm"
    )


def test_stale_status_allows_remote_arm_override():
    gate = _gate_with()
    assert gate.drive_allowed(
        "REMOTE_ARM_OVERRIDE", now_s=11.0, manual_override=True
    )


def test_allowed_override_has_empty_hold_reason():
    gate = _gate_with()
    assert gate.hold_reason("REMOTE_ARM_OVERRIDE", 11.0, manual_override=True) == ""


def test_allowed_override_reports_operator_status():
    gate = _gate_with()
    assert (
        gate.operator_status("REMOTE_ARM_OVERRIDE", 11.0, manual_override=True)
        == "operator_override_active"
    )


# Mixed-clock observation check 37.
def test_mixed_clock_queries_invalidate_sample_fail_safe():
    gate = _gate_with()
    assert gate.hold_reason("EMPTY_STOWED", 10.3) == ""
    assert not gate.drive_allowed("EMPTY_STOWED", 10.2)


# Mission-scoped ACK checks 38-42.
def test_work_acknowledged_for_same_mission_id():
    gate = _gate_with(status="WORK_READY", mission_id=7)
    assert gate.work_acknowledged(7, now_s=10.2)


def test_work_acknowledgement_rejects_other_mission_id():
    gate = _gate_with(status="WORK_READY", mission_id=7)
    assert not gate.work_acknowledged(6, now_s=10.2)


def test_work_acknowledgement_rejects_stale_status():
    gate = _gate_with(status="WORK_READY", mission_id=7)
    assert not gate.work_acknowledged(7, now_s=11.0)


def test_done_is_not_a_work_acceptance_status():
    gate = _gate_with(status="WORK_READY", mission_id=7)
    assert gate.update("DONE", 7, 10.3, 10.3)
    assert not gate.work_acknowledged(7, now_s=10.4)


def test_done_is_not_a_drive_ready_status():
    gate = _gate_with(status="WORK_READY", mission_id=7)
    assert gate.update("DONE", 7, 10.3, 10.3)
    assert not gate.drive_allowed("EMPTY_STOWED", now_s=10.4)


# Randomized consistency check 43.
def test_drive_allowed_and_hold_reason_stay_consistent_for_3000_sequences():
    rng = random.Random(42)
    mismatches = []

    for _ in range(3000):
        gate = ArmInterlock()
        now_s = 10.0
        for _ in range(rng.randint(0, 6)):
            status = rng.choice(list(contract.ARM_STATUSES) + ["JUNK"])
            now_s += rng.choice([-1.0, 0.0, 0.05, 0.1, 0.3, 0.6])
            gate.update(
                status,
                rng.randint(0, 2),
                now_s + rng.choice([-0.3, 0.0, 0.05]),
                now_s,
            )

        profile = rng.choice(
            ["EMPTY_STOWED", "CARRYING_LOCKED", "REMOTE_ARM_OVERRIDE", "TURBO"]
        )
        manual_override = rng.choice([True, False])
        now_s += rng.choice([0.0, 0.2, 0.6])
        allowed = gate.drive_allowed(
            profile, now_s, manual_override=manual_override
        )
        reason = gate.hold_reason(
            profile, now_s, manual_override=manual_override
        )

        if manual_override and profile == "REMOTE_ARM_OVERRIDE":
            consistent = allowed == (reason == "")
        elif not manual_override and profile in ("EMPTY_STOWED", "CARRYING_LOCKED"):
            consistent = allowed == (reason == "")
        else:
            consistent = not allowed and reason != ""
        if not consistent:
            mismatches.append((profile, manual_override, allowed, reason))

    assert not mismatches, f"{len(mismatches)} mismatches, first={mismatches[:3]}"


# 2026-07-14 correction regressions 44-45.
def test_unrecognized_status_does_not_consume_stamp_for_valid_packet():
    gate = _gate_with()

    assert not gate.update("JUNK", 0, stamp_s=10.1, now_s=10.1)
    assert gate.update("STOWED_LOCKED", 0, stamp_s=10.1, now_s=10.1)
    assert gate.drive_allowed("EMPTY_STOWED", now_s=10.2)


def test_large_arm_stamp_regression_latches_stamp_domain_violation():
    gate = _gate_with(stamp_s=100.0, now_s=100.0)

    assert not gate.update("STOWED_LOCKED", 0, stamp_s=1.0, now_s=100.1)
    assert not gate.update("STOWED_LOCKED", 0, stamp_s=1.1, now_s=100.2)
    assert gate.last_contract_violation.startswith("stamp_domain:")
    assert not gate.drive_allowed("EMPTY_STOWED", now_s=100.2)
    assert not gate.update("STOWED_LOCKED", 0, stamp_s=100.3, now_s=100.3)

    restarted_gate = ArmInterlock()
    assert restarted_gate.update("STOWED_LOCKED", 0, stamp_s=1.2, now_s=1.2)
