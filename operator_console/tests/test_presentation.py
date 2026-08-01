from operator_console.presentation import (
    canonical_mission_state,
    mission_presentation,
    public_freshness,
    public_safety,
    step_states,
)


def test_mission_mapping_prefers_real_ops_state_then_telemetry_state():
    assert canonical_mission_state("EXECUTING", "IDLE/OK") == "EXECUTING"
    assert canonical_mission_state("UNKNOWN", "APPROACH/RUN") == "APPROACH"
    assert mission_presentation("ESTOP").title == "비상 정지"
    assert mission_presentation("DONE").step == 4


def test_unknown_mission_does_not_invent_ready_state():
    view = mission_presentation("UNKNOWN", "unavailable")
    assert view.title == "운용 준비"
    assert view.step == 0


def test_step_states_marks_only_observed_progress():
    assert step_states(2) == (
        "completed", "completed", "active", "upcoming", "upcoming",
    )
    assert step_states(-1) == (
        "active", "upcoming", "upcoming", "upcoming", "upcoming",
    )


def test_public_freshness_hides_internal_status_words():
    assert public_freshness("LIVE") == "정상"
    assert public_freshness("STALE") == "업데이트 지연"
    assert public_freshness("UNAVAILABLE") == "정보 없음"


def test_public_safety_never_claims_normal_without_a_measurement():
    assert public_safety(
        telemetry_live=False, estop_required=False, us100_enabled=True,
    ) == ("연결 확인 중", "offline")
    assert public_safety(
        telemetry_live=True, estop_required=False, us100_enabled=True,
    ) == ("정상", "success")
    assert public_safety(
        telemetry_live=True, estop_required=True, us100_enabled=True,
    ) == ("비상 정지", "danger")
