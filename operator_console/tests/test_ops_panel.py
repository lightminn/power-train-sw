import pytest

from operator_console.ops_panel import (
    GESTURE_HOLD,
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


def test_extraction_grant_is_strip_below_arm_with_strong_warning_copy():
    action = _action("extraction_grant")
    arm_index = next(
        index for index, item in enumerate(PANEL_ACTIONS)
        if item.action == "arm"
    )
    extraction_index = next(
        index for index, item in enumerate(PANEL_ACTIONS)
        if item.action == "extraction_grant"
    )
    warning = "US-100 단독 latch에서만 · 후진 −0.2 m/s · TTL 3 s"

    assert action.gesture == GESTURE_STRIP
    assert warning in action.label
    assert warning in action.confirm_text
    assert extraction_index == arm_index + 1


def test_component_mask_actions_are_four_bool_strips_before_arm_override():
    expected = (
        ("drive_enable", "Drive motors"),
        ("steer_enable", "Steer motors"),
        ("us100_enable", "US-100 safety"),
        ("robot_arm_enable", "Robot arm"),
    )
    extraction_index = next(
        index for index, item in enumerate(PANEL_ACTIONS)
        if item.action == "extraction_grant"
    )
    override_index = next(
        index for index, item in enumerate(PANEL_ACTIONS)
        if item.action == "arm_lock_override"
    )

    for action_name, label in expected:
        action = _action(action_name)
        action_index = PANEL_ACTIONS.index(action)
        assert action.label == label
        assert action.gesture == GESTURE_STRIP
        assert action.needs_bool is True
        assert extraction_index < action_index < override_index


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
    assert action.needs_bool is True
    assert "SAFETY LOCK" in action.confirm_text
    assert flow.confirm(action.action) == {
        "action": "arm_lock_override",
        "params": {"data": True},
        "expected_state_revision": 11,
    }
