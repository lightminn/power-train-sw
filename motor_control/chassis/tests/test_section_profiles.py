"""WP8 section-profile core tests using the temporary fake event contract."""

from dataclasses import FrozenInstanceError
import ast
from pathlib import Path

import pytest

from chassis.section_profiles import (
    ARRIVAL_REACHED,
    FOLLOW_PROFILE,
    ICE_PROFILE,
    LEAD_FOUND,
    LEAD_LOST,
    LIGHT_GREEN,
    LIGHT_RED,
    MARKERS_PROFILE,
    MARKER_DETECTED,
    OPERATOR_HOLD,
    OPERATOR_RESUME,
    RELIEF_PROFILE,
    SECTION_ENTER,
    SECTION_EXIT,
    SMOG_ENTER,
    SMOG_EXIT,
    SMOG_PROFILE,
    STUCK_DETECTED,
    MarkerDedup,
    SectionConfig,
    SectionEvent,
    SectionSupervisor,
)


MODULE = Path(__file__).resolve().parents[1] / "section_profiles.py"


def _event(event_type, stamp_s, **payload):
    return SectionEvent(type=event_type, stamp_s=stamp_s, payload=payload)


def _tick(supervisor, event_type, stamp_s, **payload):
    supervisor.submit(_event(event_type, stamp_s, **payload))
    return supervisor.tick(stamp_s)


def _marker(
    dedup,
    *,
    stamp_s=1.0,
    class_id=None,
    class_name="marker_a",
    position=(0.0, 0.0, 0.0),
    confidence=0.9,
):
    return dedup.observe(
        class_id=class_id,
        class_name=class_name,
        position=position,
        confidence=confidence,
        stamp_s=stamp_s,
    )


def test_section_event_and_state_are_frozen_values():
    event = _event(SECTION_ENTER, 1.0)
    state = SectionSupervisor(RELIEF_PROFILE).tick(1.0)

    with pytest.raises(FrozenInstanceError):
        event.type = SECTION_EXIT
    with pytest.raises(FrozenInstanceError):
        state.complete = True


def test_smog_uses_speed_hint_without_hold_and_waits_for_two_arm_results():
    supervisor = SectionSupervisor(
        SMOG_PROFILE,
        SectionConfig(smog_speed_hint=0.25),
    )

    entered = _tick(supervisor, SMOG_ENTER, 1.0)
    assert entered.drive_hold_hint is False
    assert entered.speed_hint == pytest.approx(0.25)
    assert "smog_hold_policy_unconfirmed" in entered.notices

    _tick(supervisor, ARRIVAL_REACHED, 2.0, arm_result="IFF_COMPLETE")
    one_result = _tick(supervisor, SECTION_EXIT, 3.0)
    assert one_result.complete is False
    assert "smog_arm_results_required:1/2" in one_result.notices

    _tick(supervisor, ARRIVAL_REACHED, 4.0, arm_result="LED_COMPLETE")
    complete = _tick(supervisor, SECTION_EXIT, 5.0)
    assert complete.complete is True
    assert complete.phase == "COMPLETE"
    assert complete.speed_hint is None

    exited = _tick(supervisor, SMOG_EXIT, 6.0)
    assert exited.speed_hint is None


def test_relief_red_green_hold_resume_and_work_request_hint():
    supervisor = SectionSupervisor(RELIEF_PROFILE)

    red = _tick(supervisor, LIGHT_RED, 1.0)
    assert red.phase == "EVENT_HOLD"
    assert red.drive_hold_hint is True

    green = _tick(supervisor, LIGHT_GREEN, 2.0)
    assert green.phase == "DRIVE"
    assert green.drive_hold_hint is False

    arrival = _tick(supervisor, ARRIVAL_REACHED, 3.0)
    assert arrival.work_request == "ARRIVED_PICKUP"
    assert supervisor.tick(3.1).work_request is None

    _tick(supervisor, LIGHT_GREEN, 4.0)
    done = _tick(supervisor, SECTION_EXIT, 5.0)
    assert done.complete is True


def test_marker_dedup_rejects_low_confidence_and_fast_reobservation():
    dedup = MarkerDedup(min_confidence=0.5, min_reobserve_s=1.0)

    assert _marker(dedup, confidence=0.49) is False
    assert _marker(dedup, stamp_s=2.0) is True
    assert _marker(dedup, stamp_s=2.5, position=(0.2, 0.0, 0.0)) is False

    assert dedup.unique_count == 1
    assert [record.reason for record in dedup.failures] == (
        ["low_confidence", "min_reobserve"]
    )
    assert len(dedup.successes) == 1


def test_marker_dedup_clusters_position_after_minimum_reobservation():
    dedup = MarkerDedup(cluster_m=1.0, min_reobserve_s=1.0)

    assert _marker(dedup, stamp_s=1.0, position=(0.0, 0.0, 0.0)) is True
    assert _marker(dedup, stamp_s=2.0, position=(0.9, 0.0, 0.0)) is False
    assert _marker(dedup, stamp_s=3.0, position=(1.01, 0.0, 0.0)) is True

    assert dedup.unique_count == 2
    assert dedup.failures[-1].reason == "duplicate_cluster"


def test_marker_dedup_prefers_class_id_over_name_and_position():
    dedup = MarkerDedup(cluster_m=0.1, min_reobserve_s=0.0)

    assert _marker(
        dedup,
        class_id=7,
        class_name="marker_a",
        position=(0.0, 0.0, 0.0),
    ) is True
    assert _marker(
        dedup,
        stamp_s=2.0,
        class_id=7,
        class_name="renamed",
        position=(20.0, 0.0, 0.0),
    ) is False
    assert _marker(
        dedup,
        stamp_s=3.0,
        class_id=8,
        class_name="marker_a",
        position=(0.0, 0.0, 0.0),
    ) is True

    assert dedup.unique_count == 2
    assert dedup.failures[-1].reason == "duplicate_class_id"


def test_nonpositive_class_id_uses_name_and_position_fallback():
    dedup = MarkerDedup(cluster_m=0.1, min_reobserve_s=0.0)

    assert _marker(dedup, class_id=0, position=(0.0, 0.0, 0.0)) is True
    assert _marker(
        dedup,
        stamp_s=2.0,
        class_id=0,
        position=(1.0, 0.0, 0.0),
    ) is True

    assert dedup.unique_count == 2


def test_markers_complete_after_five_unique_markers_and_section_exit():
    supervisor = SectionSupervisor(MARKERS_PROFILE)

    for class_id in range(1, 6):
        state = _tick(
            supervisor,
            MARKER_DETECTED,
            float(class_id),
            class_id=class_id,
            class_name=f"marker_{class_id}",
            position=(float(class_id), 0.0, 0.0),
            confidence=0.9,
        )

    assert supervisor.unique_markers == 5
    assert state.complete is False
    complete = _tick(supervisor, SECTION_EXIT, 6.0)
    assert complete.complete is True


def test_ice_entry_sets_conservative_hint_and_stuck_only_requests_recovery():
    supervisor = SectionSupervisor(
        ICE_PROFILE,
        SectionConfig(ice_speed_hint=0.15),
    )

    entered = _tick(supervisor, SECTION_ENTER, 1.0)
    assert entered.speed_hint == pytest.approx(0.15)

    stuck = _tick(supervisor, STUCK_DETECTED, 2.0)
    assert stuck.phase == "RECOVERY_REQUESTED"
    assert stuck.drive_hold_hint is True
    assert "operator_alert:stuck_recovery_policy_pending" in stuck.notices


def test_follow_lost_and_found_only_produce_progress_hints():
    supervisor = SectionSupervisor(FOLLOW_PROFILE)

    lost = _tick(supervisor, LEAD_LOST, 1.0)
    assert lost.phase == "EVENT_HOLD"
    assert lost.drive_hold_hint is True

    found = _tick(supervisor, LEAD_FOUND, 2.0)
    assert found.phase == "DRIVE"
    assert found.drive_hold_hint is False


def test_operator_hold_has_priority_over_profile_resume_events():
    supervisor = SectionSupervisor(RELIEF_PROFILE)

    _tick(supervisor, LIGHT_RED, 1.0)
    _tick(supervisor, OPERATOR_HOLD, 2.0)
    still_held = _tick(supervisor, LIGHT_GREEN, 3.0)
    assert still_held.drive_hold_hint is True
    assert "operator_hold" in still_held.notices

    resumed = _tick(supervisor, OPERATOR_RESUME, 4.0)
    assert resumed.drive_hold_hint is False
    assert resumed.phase == "DRIVE"


def test_operator_hold_blocks_section_completion_until_operator_resume():
    supervisor = SectionSupervisor(ICE_PROFILE)

    _tick(supervisor, OPERATOR_HOLD, 1.0)
    blocked = _tick(supervisor, SECTION_EXIT, 2.0)
    assert blocked.complete is False
    assert blocked.drive_hold_hint is True
    assert blocked.journal_events[-1].reason == "operator_hold_active"

    _tick(supervisor, OPERATOR_RESUME, 3.0)
    complete = _tick(supervisor, SECTION_EXIT, 4.0)
    assert complete.complete is True


def test_retrograde_operator_resume_is_ignored_and_journaled():
    supervisor = SectionSupervisor(RELIEF_PROFILE)
    _tick(supervisor, OPERATOR_HOLD, 2.0)
    supervisor.submit(_event(OPERATOR_RESUME, 1.0))

    state = supervisor.tick(3.0)

    assert state.drive_hold_hint is True
    assert "operator_hold" in state.notices
    assert state.journal_events[-1].accepted is False
    assert state.journal_events[-1].reason == "stale_event_ignored"


def test_ice_recovery_blocks_section_exit_with_notice():
    supervisor = SectionSupervisor(ICE_PROFILE)
    _tick(supervisor, STUCK_DETECTED, 1.0)

    blocked = _tick(supervisor, SECTION_EXIT, 2.0)

    assert blocked.complete is False
    assert blocked.phase == "RECOVERY_REQUESTED"
    assert blocked.drive_hold_hint is True
    assert "section_exit_blocked:recovery_requested" in blocked.notices
    assert blocked.journal_events[-1].reason == "recovery_requested"


def test_relief_red_hold_blocks_section_exit_with_notice():
    supervisor = SectionSupervisor(RELIEF_PROFILE)
    _tick(supervisor, LIGHT_RED, 1.0)

    blocked = _tick(supervisor, SECTION_EXIT, 2.0)

    assert blocked.complete is False
    assert blocked.phase == "EVENT_HOLD"
    assert blocked.drive_hold_hint is True
    assert "section_exit_blocked:profile_hold" in blocked.notices
    assert blocked.journal_events[-1].reason == "profile_hold_active"


def test_unknown_event_is_ignored_and_recorded_in_journal():
    supervisor = SectionSupervisor(RELIEF_PROFILE)

    state = _tick(supervisor, "UNCONFIRMED_TEAM_EVENT", 1.0, vendor=3)

    assert state.phase == "READY"
    assert state.drive_hold_hint is False
    assert len(state.journal_events) == 1
    assert state.journal_events[0].accepted is False
    assert state.journal_events[0].reason == "unknown_event"
    assert state.journal_events[0].event_type == "UNCONFIRMED_TEAM_EVENT"


def test_odom_is_an_auxiliary_gate_and_never_required_for_completion():
    supervisor = SectionSupervisor(
        ICE_PROFILE,
        SectionConfig(odom_gate_m=100.0),
    )

    complete_without_odom = _tick(supervisor, SECTION_EXIT, 1.0)

    assert complete_without_odom.complete is True
    assert "odom_aux_gate_unmet" in complete_without_odom.notices


def test_same_inputs_and_injected_times_produce_identical_states():
    first = SectionSupervisor(FOLLOW_PROFILE)
    second = SectionSupervisor(FOLLOW_PROFILE)
    events = (
        _event(SECTION_ENTER, 1.0),
        _event(LEAD_LOST, 2.0),
        _event(OPERATOR_HOLD, 3.0),
        _event(LEAD_FOUND, 4.0),
        _event(OPERATOR_RESUME, 5.0),
        _event(SECTION_EXIT, 6.0),
    )

    first_states = []
    second_states = []
    for event in events:
        first.submit(event)
        second.submit(event)
        first_states.append(first.tick(event.stamp_s))
        second_states.append(second.tick(event.stamp_s))

    assert first_states == second_states


def test_every_processed_event_emits_one_journal_record_then_drains():
    supervisor = SectionSupervisor(RELIEF_PROFILE)
    supervisor.submit(_event(SECTION_ENTER, 1.0))
    supervisor.submit(_event(LIGHT_RED, 1.1))

    state = supervisor.tick(2.0)

    assert [entry.event_type for entry in state.journal_events] == [
        SECTION_ENTER,
        LIGHT_RED,
    ]
    assert supervisor.tick(2.1).journal_events == ()


def test_core_imports_only_python_standard_library():
    tree = ast.parse(MODULE.read_text(encoding="utf-8"))
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(
                alias.name.split(".")[0] for alias in node.names
            )
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".")[0])

    assert imported_roots <= {
        "__future__",
        "collections",
        "dataclasses",
        "math",
        "types",
        "typing",
    }
