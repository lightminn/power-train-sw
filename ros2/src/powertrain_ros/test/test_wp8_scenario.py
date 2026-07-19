import pytest

from powertrain_ros import contract
from powertrain_ros import wp8_scenario
from powertrain_ros.wp8_scenario import (
    Event,
    Finding,
    judge_baseline,
    judge_fault,
    judge_full_cycle,
    judge_pickup_conjunction,
    judge_resume,
    summarize,
)


def _event(t, topic, value, mission_id=None):
    return Event(t=t, topic=topic, value=value, mission_id=mission_id)


def _all_ok(findings):
    return all(finding.ok for finding in findings)


def _full_cycle_events(*, pickup_done=True, drop_id=11):
    events = [
        _event(0.0, "chassis_mode", contract.MODE_DRIVING),
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 10),
    ]
    if pickup_done:
        events.append(
            _event(0.4, "arm_status", contract.ARM_CARRYING_LOCKED, 10)
        )
    events.extend(
        [
            _event(0.5, "chassis_mode", contract.MODE_DRIVING),
            _event(1.0, "chassis_mode", contract.MODE_MISSION_STOP),
            _event(1.1, "arrival", contract.ARRIVED_DROP, drop_id),
            _event(1.3, "arm_status", contract.ARM_STOWED_LOCKED, drop_id),
            _event(1.4, "chassis_mode", contract.MODE_DRIVING),
        ]
    )
    return events


def test_baseline_accepts_heartbeat_without_work_before_arrival():
    events = [
        *[
            _event(index / 10.0, "arm_status", contract.ARM_STOWED_LOCKED, 0)
            for index in range(6)
        ],
        _event(0.55, "chassis_mode", contract.MODE_DRIVING),
    ]

    assert _all_ok(judge_baseline(events, window_s=1.0))


def test_baseline_accepts_stow_request_as_non_work_mode():
    events = [
        *[
            _event(index / 10.0, "arm_status", contract.ARM_IDLE, 0)
            for index in range(6)
        ],
        _event(0.55, "chassis_mode", contract.MODE_STOW_REQUEST),
    ]

    findings = judge_baseline(events, window_s=1.0)

    non_work_mode = next(
        finding for finding in findings if finding.check == "non_work_mode"
    )
    assert non_work_mode.ok
    assert contract.MODE_STOW_REQUEST in non_work_mode.detail


def test_baseline_rejects_slow_heartbeat():
    events = [
        _event(0.0, "arm_status", contract.ARM_IDLE, 0),
        _event(0.9, "arm_status", contract.ARM_IDLE, 0),
        _event(0.95, "chassis_mode", contract.MODE_DRIVING),
    ]

    findings = judge_baseline(events, window_s=1.0)

    assert any(f.check == "arm_heartbeat" and not f.ok for f in findings)


def test_baseline_rejects_work_accepted_before_arrival():
    events = [
        *[
            _event(index / 10.0, "arm_status", contract.ARM_IDLE, 0)
            for index in range(6)
        ],
        _event(0.45, "arm_status", contract.ARM_WORK_READY, 0),
        _event(0.55, "chassis_mode", contract.MODE_DRIVING),
    ]

    findings = judge_baseline(events, window_s=1.0)

    assert any(f.check == "no_premature_work" and not f.ok for f in findings)


def test_baseline_rejects_mission_stop_even_with_lock_mode():
    events = [
        *[
            _event(index / 10.0, "arm_status", contract.ARM_IDLE, 0)
            for index in range(6)
        ],
        _event(0.5, "chassis_mode", contract.MODE_DRIVING),
        _event(0.55, "chassis_mode", contract.MODE_MISSION_STOP),
    ]

    findings = judge_baseline(events, window_s=1.0)

    non_work_mode = next(
        finding for finding in findings if finding.check == "non_work_mode"
    )
    assert not non_work_mode.ok
    assert contract.MODE_DRIVING in non_work_mode.detail
    assert contract.MODE_MISSION_STOP in non_work_mode.detail


def test_pickup_accepts_stop_then_arrival_then_work():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 7),
        _event(0.7, "arrival", contract.ARRIVED_PICKUP, 7),
        _event(0.8, "arm_status", contract.ARM_WORK_READY, 7),
    ]

    assert _all_ok(judge_pickup_conjunction(events))
    assert wp8_scenario.pickup_branch(events) == "work_accepted"


def test_pickup_accepts_fail_closed_service_rejection():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_STOW_REQUEST),
        _event(0.2, "arm_status", contract.ARM_IDLE, 0),
        _event(0.3, "marker", "arrival_rejected:arm lock heartbeat stale"),
    ]

    findings = judge_pickup_conjunction(events)

    assert _all_ok(findings)
    assert wp8_scenario.pickup_branch(events) == "fail_closed"


def test_pickup_rejects_arrival_without_mission_stop_as_violation():
    events = [
        _event(0.1, "arrival", contract.ARRIVED_PICKUP, 7),
        _event(0.2, "marker", "arm_rejected:not ready"),
    ]

    findings = judge_pickup_conjunction(events)

    assert not _all_ok(findings)
    assert wp8_scenario.pickup_branch(events) == "violation"


def test_pickup_accepts_same_tick_skew_stop_after_arrival():
    # MISSION_STOP 모드와 arrival은 같은 supervisor tick에서 함께 발행되고
    # 토픽 간 DDS 전달 순서는 보장되지 않는다(07-19 실기 24 ms skew 실측).
    events = [
        _event(0.100, "arrival", contract.ARRIVED_PICKUP, 7),
        _event(0.124, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.8, "arm_status", contract.ARM_WORK_READY, 7),
    ]

    assert _all_ok(judge_pickup_conjunction(events))
    assert wp8_scenario.pickup_branch(events) == "work_accepted"


def test_pickup_rejects_stop_long_after_arrival():
    events = [
        _event(0.1, "arrival", contract.ARRIVED_PICKUP, 7),
        _event(2.5, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(2.8, "arm_status", contract.ARM_WORK_READY, 7),
    ]

    findings = judge_pickup_conjunction(events)

    assert any(f.check == "stop_covers_arrival" and not f.ok for f in findings)


def test_pickup_rejects_drive_mode_between_arrival_and_stop():
    events = [
        _event(0.10, "arrival", contract.ARRIVED_PICKUP, 7),
        _event(0.20, "chassis_mode", contract.MODE_DRIVING),
        _event(0.30, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.8, "arm_status", contract.ARM_WORK_READY, 7),
    ]

    findings = judge_pickup_conjunction(events)

    assert any(f.check == "stop_covers_arrival" and not f.ok for f in findings)


def test_pickup_rejects_inconsistent_arrival_mission_ids():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 7),
        _event(0.7, "arrival", contract.ARRIVED_PICKUP, 8),
        _event(0.8, "arm_status", contract.ARM_PERCEIVING, 7),
    ]

    findings = judge_pickup_conjunction(events)

    assert any(f.check == "arrival_mission_id" and not f.ok for f in findings)


def test_resume_accepts_only_work_after_resume_time():
    events = [
        _event(1.0, "arm_status", contract.ARM_WORK_READY, 7),
        _event(2.1, "arm_status", contract.ARM_PERCEIVING, 7),
    ]

    assert _all_ok(judge_resume(events, resume_t=2.0))


def test_resume_rejects_work_seen_only_before_resume_time():
    events = [_event(1.0, "arm_status", contract.ARM_WORK_READY, 7)]

    findings = judge_resume(events, resume_t=2.0)

    assert any(f.check == "work_reaccepted" and not f.ok for f in findings)


def test_full_cycle_accepts_both_authoritative_completions():
    assert _all_ok(judge_full_cycle(_full_cycle_events()))


def test_full_cycle_rejects_resume_without_carrying_locked():
    findings = judge_full_cycle(_full_cycle_events(pickup_done=False))

    assert any(f.check == "pickup_cycle" and not f.ok for f in findings)


def test_full_cycle_rejects_nonincreasing_mission_ids():
    findings = judge_full_cycle(_full_cycle_events(drop_id=10))

    assert any(f.check == "mission_id_monotonic" and not f.ok for f in findings)


def test_no_response_accepts_republishing_without_resume():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 4),
        _event(0.7, "arrival", contract.ARRIVED_PICKUP, 4),
        _event(1.2, "chassis_mode", contract.MODE_MISSION_STOP),
    ]

    assert _all_ok(judge_fault(events, scenario="no_response"))


def test_no_response_rejects_drive_resume():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 4),
        _event(0.7, "arrival", contract.ARRIVED_PICKUP, 4),
        _event(1.0, "chassis_mode", contract.MODE_DRIVING),
    ]

    assert not _all_ok(judge_fault(events, scenario="no_response"))


def test_late_done_accepts_previous_mission_completion_without_resume():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 4),
        _event(0.3, "arm_status", contract.ARM_CARRYING_LOCKED, 3),
        _event(0.4, "chassis_mode", contract.MODE_MISSION_STOP),
    ]

    assert _all_ok(judge_fault(events, scenario="late_done"))


def test_late_done_rejects_resume_after_previous_mission_completion():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 4),
        _event(0.3, "arm_status", contract.ARM_CARRYING_LOCKED, 3),
        _event(0.4, "chassis_mode", contract.MODE_DRIVING),
    ]

    assert not _all_ok(judge_fault(events, scenario="late_done"))


def test_failed_latch_accepts_failure_without_resume():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 5),
        _event(0.3, "arm_status", contract.ARM_FAILED, 5),
    ]

    assert _all_ok(judge_fault(events, scenario="failed_latch"))


def test_failed_latch_rejects_resume_after_failure():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 5),
        _event(0.3, "arm_status", contract.ARM_FAILED, 5),
        _event(0.4, "chassis_mode", contract.MODE_DRIVING),
    ]

    assert not _all_ok(judge_fault(events, scenario="failed_latch"))


def test_dup_done_accepts_one_resume_transition():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 5),
        _event(0.3, "arm_status", contract.ARM_CARRYING_LOCKED, 5),
        _event(0.4, "arm_status", contract.ARM_CARRYING_LOCKED, 5),
        _event(0.5, "chassis_mode", contract.MODE_DRIVING),
    ]

    assert _all_ok(judge_fault(events, scenario="dup_done"))


def test_dup_done_rejects_two_resume_transitions():
    events = [
        _event(0.1, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.2, "arrival", contract.ARRIVED_PICKUP, 5),
        _event(0.3, "arm_status", contract.ARM_CARRYING_LOCKED, 5),
        _event(0.35, "arm_status", contract.ARM_CARRYING_LOCKED, 5),
        _event(0.4, "chassis_mode", contract.MODE_DRIVING),
        _event(0.5, "chassis_mode", contract.MODE_MISSION_STOP),
        _event(0.6, "chassis_mode", contract.MODE_DRIVING),
    ]

    assert not _all_ok(judge_fault(events, scenario="dup_done"))


def test_fault_rejects_unknown_scenario():
    with pytest.raises(ValueError, match="scenario"):
        judge_fault([], scenario="unknown")


def test_summarize_returns_overall_result_and_human_table():
    passed, table = summarize(
        [
            Finding("heartbeat", True, "10.0 Hz"),
            Finding("ordering", False, "arrival first"),
        ]
    )

    assert passed is False
    assert "heartbeat" in table
    assert "PASS" in table
    assert "ordering" in table
    assert "FAIL" in table
