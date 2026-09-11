"""The request contract and the fact that it is not wired to anything yet.

Pure: no display, no socket, no ROS.
"""
from __future__ import annotations

import json
from pathlib import Path

import pytest

from operator_console.arm_ops import contract as K


REPO_ROOT = Path(__file__).resolve().parents[3]
OPS_CONTRACT = (
    REPO_ROOT / "ros2" / "src" / "powertrain_ros" / "powertrain_ros"
    / "ops_contract.py"
)


def test_every_action_is_prefixed_to_avoid_the_chassis_vocabulary():
    # ``arm`` already means chassis 시동 and ``robot_arm_enable`` is a component
    # toggle; an unprefixed arm action would be misread at the worst moment.
    for action in K.ARM_ACTIONS:
        assert action.startswith("robot_arm_"), action


def test_only_the_fsm_mode_and_tool_actions_are_registered_in_the_ops_contract():
    """The console cannot send any arm command yet — and that is on purpose.

    If this test starts failing, the arm actions have been registered
    server-side.  That is a real milestone, not a nuisance: update the report's
    "실연결 전제" section rather than deleting the assertion.
    """
    source = OPS_CONTRACT.read_text(encoding="utf-8")
    registered = {K.ACTION_MODE_REQUEST, K.ACTION_TOOL_COMMAND}
    for action in sorted(registered):
        assert f'"{action}"' in source
    for action in sorted(K.ARM_ACTIONS - registered):
        assert f'"{action}"' not in source


def test_unregistered_actions_reports_everything_against_an_empty_registry():
    assert set(K.unregistered_actions(set())) == K.ARM_ACTIONS
    assert K.unregistered_actions(K.ARM_ACTIONS) == ()


def test_planned_actions_cover_every_action_and_only_existing_fsm_ingress_is_resolved():
    planned = {item.action: item for item in K.PLANNED_ACTIONS}
    assert set(planned) == K.ARM_ACTIONS
    assert [item.action for item in K.PLANNED_ACTIONS if item.resolved] == []
    assert all(item.note for item in K.PLANNED_ACTIONS)


def test_backend_requirements_are_all_still_open():
    assert len(K.unresolved_requirements()) == len(K.BACKEND_REQUIREMENTS)
    keys = {item.key for item in K.BACKEND_REQUIREMENTS}
    for expected in (
        "action_registration", "grant_report", "jog_deadman",
        "tool_generation_authority", "single_ownership", "stop_semantics",
    ):
        assert expected in keys


# --- parameter validation ---------------------------------------------------
def test_missing_required_param_is_refused():
    with pytest.raises(K.ArmParamsError, match="missing param"):
        K.validate_params(K.ACTION_MODE_REQUEST, {})


def test_unknown_param_is_refused_rather_than_dropped():
    # A silently dropped typo becomes a command meaning something else.
    with pytest.raises(K.ArmParamsError, match="unknown params"):
        K.validate_params(
            K.ACTION_MODE_REQUEST, {"mode": K.MODE_MANUAL, "modee": "x"},
        )


def test_enumerated_params_reject_values_outside_the_contract():
    with pytest.raises(K.ArmParamsError):
        K.validate_params(K.ACTION_MODE_REQUEST, {"mode": "AUTO"})
    with pytest.raises(K.ArmParamsError):
        K.validate_params(
            K.ACTION_TOOL_COMMAND,
            {
                "target": "middle", "command": "open",
                "tool_id": "T1", "tool_generation": 1,
            },
        )


def test_jog_requires_a_positive_deadline_and_a_valid_direction():
    good = {
        "axis": 2, "direction": 1, "expires_at_s": 12.5, "sequence": 3,
    }
    assert K.validate_params(K.ACTION_JOG, good) == good
    with pytest.raises(K.ArmParamsError):
        K.validate_params(K.ACTION_JOG, {**good, "expires_at_s": 0.0})
    with pytest.raises(K.ArmParamsError):
        K.validate_params(K.ACTION_JOG, {**good, "direction": 0})
    with pytest.raises(K.ArmParamsError):
        K.validate_params(K.ACTION_JOG, {**good, "axis": 9})


def test_booleans_are_not_accepted_where_an_integer_is_required():
    with pytest.raises(K.ArmParamsError):
        K.validate_params(
            K.ACTION_JOG,
            {"axis": True, "direction": 1, "expires_at_s": 1.0, "sequence": 0},
        )


def test_stop_reason_must_be_one_of_the_declared_reasons():
    with pytest.raises(K.ArmParamsError):
        K.validate_params(
            K.ACTION_JOG_STOP, {"axis": 1, "reason": "because", "sequence": 1},
        )
    for reason in K.STOP_REASONS:
        K.validate_params(
            K.ACTION_JOG_STOP, {"axis": 1, "reason": reason, "sequence": 1},
        )


def test_free_text_is_length_capped_before_it_can_reach_the_record_limit():
    # Per-field caps fire first; the byte cap in _assert_size is the backstop
    # for a future field that forgets one.  Both are asserted so neither can be
    # removed without a test noticing.
    with pytest.raises(K.ArmParamsError, match="invalid param 'name'"):
        K.validate_params(K.ACTION_POSE_SAVE, {"name": "가" * 400})

    with pytest.raises(K.ArmParamsError, match="exceed"):
        K._assert_size("probe", {"blob": "가" * 400})


def _widest_value(spec: K.ParamSpec):
    """The largest value this field's own check will accept.

    Non-ASCII is the worst case: ``json.dumps`` escapes each Hangul syllable
    to a six-byte unicode escape, so a field capped at 80 *characters* can be
    480 *bytes* on the wire.
    """
    for candidate in ("가" * 80, 2 ** 53, 2 ** 53 + 0.5):
        if spec.check(candidate):
            return candidate
    for candidate in (
        K.MODE_MANUAL, K.TARGET_RIGHT, K.COMMAND_OPEN, K.SCOPE_TOOL,
        K.STOP_REASON_TOOL_CHANGED, "capture_close", 6, 10, 1,
    ):
        if spec.check(candidate):
            return candidate
    raise AssertionError(f"no widest value found for {spec.name}")


@pytest.mark.parametrize("action", sorted(K.ARM_ACTIONS))
def test_every_action_fits_the_params_budget_at_maximum_length(action):
    """The field caps and the byte cap must not contradict each other.

    If a field check accepts a value the byte cap then rejects, a legal command
    becomes unbuildable — the kind of inconsistency that only shows up with a
    long Korean tool id in the field.
    """
    params = {
        spec.name: _widest_value(spec) for spec in K.ACTION_PARAMS[action]
    }
    validated = K.validate_params(action, params)

    encoded = json.dumps(validated, allow_nan=False, separators=(",", ":"))
    assert len(encoded.encode("utf-8")) <= K.MAX_PARAMS_BYTES


def test_the_params_budget_leaves_room_for_the_ops_envelope():
    # ops_contract.MAX_RECORD_BYTES is 4 KiB and the envelope (token, uuid
    # request_id, action, stamp, sequence) costs a few hundred bytes.
    assert K.MAX_PARAMS_BYTES * 3 < 4 * 1024


def test_request_validates_on_construction():
    request = K.ArmOpsRequest(
        action=K.ACTION_STOP, params={}, expected_state_revision=7,
    )
    assert request.is_stop is True
    assert request.expected_state_revision == 7
    with pytest.raises(K.ArmParamsError):
        K.ArmOpsRequest(action=K.ACTION_MODE_REQUEST, params={"mode": "nope"})


def test_every_request_is_json_encodable_for_the_ops_envelope():
    request = K.ArmOpsRequest(
        action=K.ACTION_TOOL_JOG,
        params={
            "target": K.TARGET_LEFT, "direction": -1,
            "tool_id": "TOOL-D1", "tool_generation": 8,
            "expires_at_s": 3.5, "sequence": 1,
        },
    )
    encoded = json.dumps(request.params, allow_nan=False)
    assert json.loads(encoded) == request.params


def test_tool_scoped_actions_all_carry_the_tool_identity():
    for action in K.TOOL_SCOPED_ACTIONS:
        names = {spec.name for spec in K.ACTION_PARAMS[action]}
        assert {"tool_id", "tool_generation"} <= names, action


def test_stop_actions_are_a_subset_of_the_declared_actions():
    assert K.STOP_ACTIONS <= K.ARM_ACTIONS
    assert K.MOTION_ACTIONS <= K.ARM_ACTIONS
    assert K.CALIBRATION_ACTIONS <= K.ARM_ACTIONS
    assert K.TOOL_SCOPED_ACTIONS <= K.ARM_ACTIONS


def test_ack_status_vocabulary_matches_the_existing_ops_channel():
    """An arm ack must render through the console's existing Korean labels."""
    from operator_console.labels import ack_korean

    assert ack_korean("FINAL_SUCCESS", "") == "성공"
    assert ack_korean("OUTCOME_UNKNOWN", "") == "결과 미확정 — 재시도 가능"
    ack = K.ArmOpsAck(request_id="r1", status="OUTCOME_UNKNOWN", action=K.ACTION_STOP)
    assert ack_korean(ack.status, ack.detail) == "결과 미확정 — 재시도 가능"
