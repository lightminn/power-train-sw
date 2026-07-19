import pytest

import operator_console.labels as labels
import operator_console.ops_panel as ops_panel
from operator_console.ops_panel import (
    GESTURE_HOLD,
    GESTURE_IMMEDIATE,
    GESTURE_SPACER,
    GESTURE_STRIP,
    HOLD_CONFIRM_S,
    PANEL_ACTIONS,
    ConfirmFlow,
)


class StateSource:
    def __init__(self, state):
        self.state = state

    def latest(self):
        return self.state


def _flow(source):
    return ConfirmFlow(clock=lambda: 10.0, state_provider=source.latest)


def _action(name):
    return next(action for action in PANEL_ACTIONS if action.action == name)


def test_begin_rejects_when_no_ops_state_has_arrived():
    flow = _flow(StateSource(None))

    with pytest.raises(RuntimeError, match="ops state unavailable"):
        flow.begin("estop_reset")


def test_strip_confirmation_returns_submit_kwargs_with_snapshotted_revision():
    source = StateSource({"revision": 7, "authority_mode": "IDLE"})
    flow = _flow(source)

    pending = flow.begin("estop_reset")
    source.state["authority_mode"] = "MANUAL"

    assert pending.state_snapshot == {"revision": 7, "authority_mode": "IDLE"}
    assert flow.confirm("estop_reset") == {
        "action": "estop_reset",
        "params": {},
        "expected_state_revision": 7,
    }


def test_confirmation_resets_when_state_revision_changes():
    source = StateSource({"revision": 7, "authority_mode": "IDLE"})
    flow = _flow(source)
    flow.begin("estop_reset")
    source.state = {"revision": 8, "authority_mode": "IDLE"}

    assert flow.confirm("estop_reset") is None
    assert flow.pending is None


def test_arm_hold_rejects_short_release_and_accepts_one_point_five_seconds():
    source = StateSource({"revision": 3})
    flow = _flow(source)
    flow.begin("arm")

    assert flow.confirm("arm", held_s=HOLD_CONFIRM_S - 0.01) is None
    assert flow.confirm("arm", held_s=HOLD_CONFIRM_S) == {
        "action": "arm",
        "params": {},
        "expected_state_revision": 3,
    }


def test_estop_reset_and_arm_use_distinct_gestures_with_spacer_between():
    reset_index = next(
        index for index, action in enumerate(PANEL_ACTIONS)
        if action.action == "estop_reset"
    )
    arm_index = next(
        index for index, action in enumerate(PANEL_ACTIONS)
        if action.action == "arm"
    )

    assert _action("estop_reset").gesture == GESTURE_STRIP
    assert _action("arm").gesture == GESTURE_HOLD
    assert arm_index == reset_index + 2
    assert PANEL_ACTIONS[reset_index + 1].action is None
    assert PANEL_ACTIONS[reset_index + 1].gesture == GESTURE_SPACER


def test_panel_keeps_eight_basic_actions_and_preserves_existing_action_keys():
    actions = tuple(action for action in PANEL_ACTIONS if action.action is not None)
    basic = {action.action for action in actions if not action.advanced}
    advanced = {action.action for action in actions if action.advanced}

    assert basic == {
        "estop",
        "estop_reset",
        "arm",
        "disarm",
        "drive_enable",
        "steer_enable",
        "us100_enable",
        "robot_arm_enable",
    }
    assert {
        "authority_manual",
        "authority_auto",
        "authority_idle",
        "extraction_grant",
        "arm_lock_override",
        "clear_transient_hold",
    } <= advanced
    assert {action.action for action in actions} == {
        "estop",
        "clear_transient_hold",
        "authority_manual",
        "authority_auto",
        "authority_idle",
        "estop_reset",
        "arm",
        "extraction_grant",
        "disarm",
        "drive_enable",
        "steer_enable",
        "us100_enable",
        "robot_arm_enable",
        "arm_lock_override",
        "mission_arrive_pickup",
        "mission_arrive_drop",
        "mission_skip",
        "mission_retry",
        "mission_regrasp_confirmed",
        "mission_clear_grip_lost",
        "operator_hold",
        "operator_resume",
    }


def test_estop_confirmation_uses_snapshotted_revision_and_warning_copy():
    action = _action("estop")
    source = StateSource({"revision": 9})
    flow = _flow(source)

    flow.begin(action)

    assert action.label == "비상정지 (ESTOP)"
    assert action.gesture == GESTURE_IMMEDIATE
    assert "임무를 중단" in action.confirm_text
    assert flow.confirm(action) == {
        "action": "estop",
        "params": {},
        "expected_state_revision": 9,
    }


def test_extraction_grant_is_advanced_strip_with_korean_warning_copy():
    action = _action("extraction_grant")
    warning = "후진 0.2 m/s·3초"

    assert action.gesture == GESTURE_STRIP
    assert action.advanced is True
    assert warning in action.label
    assert warning in action.confirm_text


def test_component_mask_actions_are_four_basic_bool_strips():
    expected = (
        ("drive_enable", "구동 모터"),
        ("steer_enable", "조향 모터"),
        ("us100_enable", "US-100 안전"),
        ("robot_arm_enable", "로봇팔"),
    )
    for action_name, label in expected:
        action = _action(action_name)
        assert action.label == label
        assert action.gesture == GESTURE_STRIP
        assert action.needs_bool is True
        assert action.advanced is False


def test_us100_disable_confirmation_names_lost_automatic_stop():
    copy = _action("us100_enable").confirm_text

    assert "충돌 안전 센서" in copy
    assert "자동 정지 없음" in copy


@pytest.mark.parametrize(
    ("current_enabled", "submitted_enabled"),
    ((True, False), (False, True)),
)
def test_component_mask_action_submits_inverse_of_snapshotted_state(
    current_enabled,
    submitted_enabled,
):
    source = StateSource({
        "revision": 12,
        "component_mask": {"drive": current_enabled},
    })
    flow = _flow(source)

    flow.begin("drive_enable")

    assert flow.confirm("drive_enable") == {
        "action": "drive_enable",
        "params": {"data": submitted_enabled},
        "expected_state_revision": 12,
    }


def test_component_mask_action_cannot_begin_without_component_state():
    flow = _flow(StateSource({"revision": 13}))

    with pytest.raises(RuntimeError, match="component mask unavailable"):
        flow.begin("drive_enable")


def test_arm_lock_override_requires_bool_param_and_strong_confirmation_copy():
    action = _action("arm_lock_override")
    source = StateSource({"revision": 11})
    flow = _flow(source)

    flow.begin(action.action)

    assert action.gesture == GESTURE_STRIP
    assert action.advanced is True
    assert action.needs_bool is True
    assert "안전 잠금" in action.confirm_text
    assert flow.confirm(action.action) == {
        "action": "arm_lock_override",
        "params": {"data": True},
        "expected_state_revision": 11,
    }


def test_arm_lock_override_cancel_row_sends_false_and_string_lookup_keeps_true():
    # 콘솔에서 위험 잠금 해제를 "걸 수만 있고 풀 수 없던" 결함의 재발 방지:
    # 같은 action 이름의 취소 행이 data=False 를 보내고, 문자열 조회는
    # 기존 의미(해제 걸기 = True)를 유지해야 한다.
    rows = [
        action for action in PANEL_ACTIONS
        if action.action == "arm_lock_override"
    ]
    assert len(rows) == 2
    enable_row, cancel_row = rows
    assert enable_row.bool_value is True
    assert cancel_row.bool_value is False
    assert cancel_row.advanced is True
    assert "취소" in cancel_row.label

    flow = _flow(StateSource({"revision": 4}))
    flow.begin(cancel_row)
    assert flow.confirm(cancel_row) == {
        "action": "arm_lock_override",
        "params": {"data": False},
        "expected_state_revision": 4,
    }

    flow.begin("arm_lock_override")
    confirmed = flow.confirm("arm_lock_override")
    assert confirmed is not None
    assert confirmed["params"] == {"data": True}


def test_mission_clear_grip_lost_sends_operator_authorization_bool():
    # chassis_node 의 ~/mission_clear_grip_lost 는 SetBool(request.data=인가).
    # needs_bool 없이는 브로커가 "params.data must be bool" 로 거부한다.
    action = _action("mission_clear_grip_lost")
    assert action.needs_bool is True

    flow = _flow(StateSource({"revision": 6}))
    flow.begin(action)
    assert flow.confirm(action) == {
        "action": "mission_clear_grip_lost",
        "params": {"data": True},
        "expected_state_revision": 6,
    }


def test_primary_action_labels_are_korean():
    assert _action("arm").label == "시동 — 1.5초 홀드"
    assert _action("drive_enable").label == "구동 모터"


@pytest.mark.parametrize(
    ("action", "mode", "expected"),
    (
        ("drive_enable", "IDLE", True),
        ("drive_enable", "ARMED", False),
        ("steer_enable", "UNKNOWN", False),
        ("us100_enable", "ARMED", True),
        ("robot_arm_enable", "ESTOP", True),
    ),
)
def test_mode_gate_only_blocks_drive_and_steer_outside_idle(
    action,
    mode,
    expected,
):
    mode_allows_action = getattr(ops_panel, "mode_allows_action", None)
    assert mode_allows_action is not None
    assert mode_allows_action(action, mode) is expected


@pytest.mark.parametrize(
    ("action", "status", "detail", "expected"),
    (
        (
            "drive_enable",
            "FINAL_REJECTED",
            "not_idle",
            "last: drive_enable FINAL_REJECTED · not_idle",
        ),
        (
            "estop_reset",
            "OUTCOME_UNKNOWN",
            "no response from /chassis_node/reset_estop",
            (
                "last: estop_reset OUTCOME_UNKNOWN · "
                "no response from /chassis_node/reset_estop"
            ),
        ),
        (
            "authority_idle",
            "FINAL_SUCCESS",
            "",
            "last: authority_idle FINAL_SUCCESS",
        ),
    ),
)
def test_format_ack_line_keeps_action_status_and_detail_visible(
    action,
    status,
    detail,
    expected,
):
    assert ops_panel.format_ack_line(action, status, detail) == expected


@pytest.mark.parametrize(
    ("status", "color"),
    (
        ("FINAL_REJECTED", "#d32f2f"),
        ("OUTCOME_UNKNOWN", "#e67e22"),
    ),
)
def test_format_ack_markup_colors_non_success_final_status(status, color):
    markup = ops_panel.format_ack_markup("drive_enable", status, "not_idle")

    assert markup == (
        f'<span foreground="{color}">'
        f"last: drive_enable {status} · not_idle</span>"
    )


def test_format_ack_markup_leaves_success_plain_and_escapes_detail():
    markup = ops_panel.format_ack_markup(
        "authority_idle",
        "FINAL_SUCCESS",
        "ok <safe>",
    )

    assert markup == "last: authority_idle FINAL_SUCCESS · ok &lt;safe&gt;"


@pytest.mark.parametrize(
    ("source", "expected"),
    (
        ("us100", "US-100 안전 센서"),
        ("safety_topic_stale", "안전 센서 링크 두절"),
        ("console", "콘솔 비상정지 버튼"),
        ("manual_service", "수동 비상정지"),
        ("corner_fault", "모터(코너) 결함"),
        ("arm_failure", "시동 실패"),
        ("reset_failure", "초기화 실패"),
        ("extraction_budget_exhausted", "구조 탈출 한도 소진"),
        ("extraction_arm_failure", "구조 탈출 시동 실패"),
        ("active_estop_sources_not_us100_only", "구조 탈출 불가 조건"),
        ("extraction_complete", "구조 탈출 종료"),
        ("", ""),
        ("unknown_trip", "unknown_trip"),
    ),
)
def test_estop_source_korean_covers_known_empty_and_unknown_sources(
    source,
    expected,
):
    estop_source_korean = getattr(labels, "estop_source_korean", None)
    assert estop_source_korean is not None
    assert estop_source_korean(source) == expected


def test_estop_status_line_prioritizes_active_sources_and_details_first_source():
    assert ops_panel.format_ops_status_line(
        "ESTOP",
        "없음",
        "us100",
        "75 mm",
        ("us100", "robot_arm"),
    ) == (
        "모드: 비상정지(ESTOP) — 원인(활성): US-100 안전 센서 "
        "(75 mm) · 로봇팔 연동 정지 · 최근: 없음"
    )


def test_estop_status_line_marks_cleared_latch_as_resettable():
    assert ops_panel.format_ops_status_line(
        "ESTOP",
        "없음",
        "us100",
        "75 mm",
        (),
    ) == (
        "모드: 비상정지(ESTOP) — 원인(최초): US-100 안전 센서 "
        "(75 mm) · 활성 조건 없음 — 경고 초기화 가능 · 최근: 없음"
    )


def test_non_estop_status_line_preserves_existing_copy():
    assert ops_panel.format_ops_status_line(
        "IDLE",
        "성공",
        "console",
        "ignored outside ESTOP",
        ("robot_arm",),
    ) == "모드: 대기(IDLE) · 최근: 성공"


def test_estop_cause_event_refires_once_when_active_composition_changes():
    previous = None

    previous, event = ops_panel.next_estop_cause_event(
        previous,
        chassis_mode="ESTOP",
        estop_source="us100",
        estop_detail="75 mm",
        active_estop_sources=("us100", "robot_arm"),
    )
    assert event == "비상정지 원인: US-100 안전 센서 (75 mm)"

    previous, event = ops_panel.next_estop_cause_event(
        previous,
        chassis_mode="ESTOP",
        estop_source="us100",
        estop_detail="75 mm",
        active_estop_sources=("robot_arm", "us100"),
    )
    assert event is None

    previous, event = ops_panel.next_estop_cause_event(
        previous,
        chassis_mode="ESTOP",
        estop_source="us100",
        estop_detail="75 mm",
        active_estop_sources=("robot_arm",),
    )
    assert event == "비상정지 원인: US-100 안전 센서 (75 mm)"

    previous, event = ops_panel.next_estop_cause_event(
        previous,
        chassis_mode="ESTOP",
        estop_source="us100",
        estop_detail="75 mm",
        active_estop_sources=("robot_arm",),
    )
    assert event is None


def test_estop_cause_event_rearms_after_clear():
    previous, _event = ops_panel.next_estop_cause_event(
        None,
        chassis_mode="ESTOP",
        estop_source="us100",
        estop_detail="75 mm",
        active_estop_sources=("us100",),
    )

    previous, event = ops_panel.next_estop_cause_event(
        previous,
        chassis_mode="IDLE",
        estop_source="",
        estop_detail="",
        active_estop_sources=(),
    )
    assert previous is None
    assert event is None

    _previous, event = ops_panel.next_estop_cause_event(
        previous,
        chassis_mode="ESTOP",
        estop_source="us100",
        estop_detail="75 mm",
        active_estop_sources=("us100",),
    )
    assert event == "비상정지 원인: US-100 안전 센서 (75 mm)"
