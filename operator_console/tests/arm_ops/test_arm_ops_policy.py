"""The safety conditions, asserted directly against the pure policy.

Every test here is a rule from the design that must hold no matter what the UI
does, so none of them touch a widget.
"""
from __future__ import annotations

from dataclasses import replace

import pytest

from operator_console.arm_ops import contract as K
from operator_console.arm_ops import policy as P


ALL_CAPS = frozenset({
    P.CAP_TOOL_CHANGE, P.CAP_CONTROL_MODE, P.CAP_TELEOP_JOG, P.CAP_TELEOP_POSE,
    P.CAP_GRIPPER_COMMAND, P.CAP_DUAL_SYNC, P.CAP_ARM_CALIBRATION,
    P.CAP_TOOL_CALIBRATION, P.CAP_DUAL_TOOL_CALIBRATION,
})


def granted_context(**overrides) -> P.ArmCommandContext:
    """A context in which a well-formed motion command is allowed."""
    base = dict(
        link_live=True,
        granted_mode=K.MODE_MANUAL,
        tool_id="TOOL-S1",
        tool_kind="single_gripper",
        tool_generation=7,
        actuator_ids=(5,),
        capabilities=ALL_CAPS,
        state_revision=42,
    )
    base.update(overrides)
    return P.ArmCommandContext(**base)


def tool_params(context: P.ArmCommandContext, **extra) -> dict:
    params = {
        "tool_id": context.tool_id, "tool_generation": context.tool_generation,
    }
    params.update(extra)
    return params


# --- the default is refusal -------------------------------------------------
def test_default_context_permits_nothing_except_stops():
    context = P.ArmCommandContext()

    for action in sorted(K.ARM_ACTIONS - K.STOP_ACTIONS):
        assert not P.authorize(context, action), action
    assert P.authorize(context, K.ACTION_STOP)


def test_default_context_is_not_granted_and_has_no_capabilities():
    context = P.ArmCommandContext()

    assert context.link_live is False
    assert context.granted_mode == ""
    assert context.manual_granted is False
    assert context.capabilities == frozenset()
    assert context.state_revision is None


# --- rule 1: capability -----------------------------------------------------
def test_a_missing_capability_makes_the_command_unbuildable():
    context = granted_context(capabilities=frozenset())

    decision = P.authorize(context, K.ACTION_JOG, {"axis": 1})

    assert not decision
    assert decision.reason == "capability_missing"


@pytest.mark.parametrize(
    "action,capability", sorted(P.ACTION_CAPABILITY.items()),
)
def test_every_non_stop_action_names_a_capability_it_needs(action, capability):
    without = granted_context(capabilities=ALL_CAPS - {capability})
    params = {}
    if action in K.TOOL_SCOPED_ACTIONS:
        params = tool_params(without)
    if action == K.ACTION_CALIBRATION:
        params["scope"] = K.SCOPE_ARM

    decision = P.authorize(without, action, params)

    assert not decision
    assert decision.reason == "capability_missing"


def test_capability_is_checked_before_anything_else_can_explain_the_refusal():
    # No capability *and* no grant: the operator must be told the fundamental
    # obstacle, not sent chasing a control-authority request that cannot help.
    context = granted_context(capabilities=frozenset(), granted_mode="")

    assert P.authorize(context, K.ACTION_JOG, {"axis": 1}).reason \
        == "capability_missing"


def test_dual_tool_calibration_needs_its_own_capability():
    context = granted_context(
        tool_kind="dual_gripper",
        capabilities=ALL_CAPS - {P.CAP_DUAL_TOOL_CALIBRATION},
    )

    decision = P.authorize(
        context, K.ACTION_CALIBRATION,
        {"scope": K.SCOPE_TOOL, **tool_params(context)},
    )

    assert not decision
    assert decision.reason == "capability_missing"


# --- rule 2: stops are never gated -----------------------------------------
@pytest.mark.parametrize("action", sorted(K.STOP_ACTIONS))
def test_a_stop_is_allowed_with_no_capability_no_grant_and_no_link(action):
    context = P.ArmCommandContext()
    params = {"tool_id": "TOOL-D1"} if action in K.TOOL_SCOPED_ACTIONS else {}

    assert P.authorize(context, action, params)


def test_a_tool_scoped_stop_must_still_name_a_tool():
    decision = P.authorize(
        P.ArmCommandContext(), K.ACTION_TOOL_JOG_STOP, {"target": "left"},
    )

    assert not decision
    assert decision.reason == "stop_without_tool"


def test_a_stop_for_a_superseded_tool_is_still_allowed():
    # Stopping the tool that was just swapped away is exactly what has to
    # happen; refusing it on a generation mismatch would strand the motion.
    context = granted_context(tool_generation=9)

    assert P.authorize(
        context, K.ACTION_TOOL_JOG_STOP,
        {"target": "left", "tool_id": "TOOL-OLD", "tool_generation": 7},
    )


# --- rule 3: freshness ------------------------------------------------------
def test_a_non_live_link_blocks_every_non_stop_command():
    context = granted_context(link_live=False)

    decision = P.authorize(context, K.ACTION_HOME)

    assert not decision
    assert decision.reason == "link_not_live"


def test_a_missing_ops_state_revision_blocks_commands():
    context = granted_context(state_revision=None)

    decision = P.authorize(context, K.ACTION_HOME)

    assert not decision
    assert decision.reason == "no_ops_state"


# --- rule 4: requesting is not holding --------------------------------------
@pytest.mark.parametrize("action", sorted(K.MOTION_ACTIONS - K.STOP_ACTIONS))
def test_no_motion_without_an_approved_manual_grant(action):
    context = granted_context(granted_mode="")
    params = {}
    if action in K.TOOL_SCOPED_ACTIONS:
        params = tool_params(context)
    if action == K.ACTION_JOG:
        params = {"axis": 1}

    decision = P.authorize(context, action, params)

    assert not decision
    assert decision.reason == "manual_not_granted"


def test_an_fsm_grant_does_not_unlock_manual_motion():
    context = granted_context(granted_mode=K.MODE_FSM)

    assert not P.authorize(context, K.ACTION_JOG, {"axis": 1})


def test_requesting_the_grant_stays_available_without_one():
    # Otherwise a console holding no grant could never obtain one.
    context = granted_context(granted_mode="")

    assert P.authorize(context, K.ACTION_MODE_REQUEST, {"mode": K.MODE_MANUAL})
    assert P.authorize(context, K.ACTION_MODE_RELEASE)


def test_tool_change_does_not_require_a_motion_grant():
    context = granted_context(granted_mode="")

    assert P.authorize(
        context, K.ACTION_TOOL_CHANGE,
        {"requested_kind": "dual_gripper", "observed_tool_generation": 7},
    )


# --- rule 5: calibration owns the arm ---------------------------------------
def test_a_calibration_session_blocks_general_jog_and_tool_change():
    context = granted_context(calibration_active=True)

    for action, params in (
        (K.ACTION_JOG, {"axis": 1}),
        (K.ACTION_TOOL_COMMAND, tool_params(context, target="single", command="open")),
        (K.ACTION_TOOL_CHANGE,
         {"requested_kind": "dual_gripper", "observed_tool_generation": 7}),
    ):
        decision = P.authorize(context, action, params)
        assert not decision, action
        assert decision.reason == "calibration_active"


def test_calibration_actions_stay_available_during_a_session():
    context = granted_context(calibration_active=True)

    assert P.authorize(
        context, K.ACTION_CALIBRATION,
        {"scope": K.SCOPE_ARM, "step": "gear_ratio", "action": "measure"},
    )
    assert P.authorize(
        context, K.ACTION_CALIBRATION_JOG,
        tool_params(context, target="single", direction=1),
    )


# --- rule 6: tool change in flight ------------------------------------------
def test_a_tool_change_in_flight_blocks_everything_but_the_change():
    context = granted_context(tool_change_active=True)

    assert not P.authorize(context, K.ACTION_JOG, {"axis": 1})
    assert P.authorize(
        context, K.ACTION_TOOL_CHANGE,
        {"requested_kind": "cleaner", "observed_tool_generation": 7},
    )


# --- rule 7: tool identity --------------------------------------------------
def test_a_command_for_a_different_tool_generation_is_refused():
    context = granted_context()

    decision = P.authorize(
        context, K.ACTION_TOOL_COMMAND,
        {
            "target": "single", "command": "open",
            "tool_id": context.tool_id, "tool_generation": context.tool_generation - 1,
        },
    )

    assert not decision
    assert decision.reason == "tool_generation_mismatch"


def test_a_command_for_a_different_tool_id_is_refused():
    context = granted_context()

    decision = P.authorize(
        context, K.ACTION_TOOL_COMMAND,
        {
            "target": "single", "command": "open",
            "tool_id": "TOOL-OTHER", "tool_generation": context.tool_generation,
        },
    )

    assert not decision
    assert decision.reason == "tool_id_mismatch"


def test_a_tool_command_with_no_active_tool_is_refused():
    context = granted_context(tool_id="")

    decision = P.authorize(
        context, K.ACTION_TOOL_COMMAND,
        tool_params(context, target="single", command="open"),
    )

    assert not decision
    assert decision.reason == "no_active_tool"


def test_a_matching_tool_identity_is_allowed():
    context = granted_context()

    assert P.authorize(
        context, K.ACTION_TOOL_COMMAND,
        tool_params(context, target="single", command="open"),
    )


# --- backend-reported blocks ------------------------------------------------
def test_a_backend_block_reason_is_surfaced_verbatim():
    context = granted_context(blocked_reasons=("비상정지 상태입니다.",))

    decision = P.authorize(context, K.ACTION_HOME)

    assert not decision
    assert decision.korean == "비상정지 상태입니다."


def test_a_backend_block_does_not_prevent_stopping():
    context = granted_context(blocked_reasons=("비상정지 상태입니다.",))

    assert P.authorize(context, K.ACTION_STOP)


def test_an_unknown_action_is_refused():
    assert not P.authorize(granted_context(), "robot_arm_launch_missile")


# --- the capability vocabulary is shared with the UI ------------------------
def test_capability_tokens_match_the_arm_ui_contract_exactly():
    """Two vocabularies for one fact is how a control gets enabled for a
    command that can never be built."""
    from operator_console.arm_ui import contracts as ui

    assert P.CAP_TOOL_CHANGE == ui.CAP_TOOL_CHANGE
    assert P.CAP_CONTROL_MODE == ui.CAP_CONTROL_MODE
    assert P.CAP_TELEOP_JOG == ui.CAP_TELEOP_JOG
    assert P.CAP_TELEOP_POSE == ui.CAP_TELEOP_POSE
    assert P.CAP_GRIPPER_COMMAND == ui.CAP_GRIPPER_COMMAND
    assert P.CAP_DUAL_SYNC == ui.CAP_DUAL_SYNC
    assert P.CAP_ARM_CALIBRATION == ui.CAP_ARM_CALIBRATION
    assert P.CAP_TOOL_CALIBRATION == ui.CAP_TOOL_CALIBRATION
    assert P.CAP_DUAL_TOOL_CALIBRATION == ui.CAP_DUAL_TOOL_CALIBRATION
    assert ALL_CAPS == frozenset({
        ui.CAP_TOOL_CHANGE, ui.CAP_CONTROL_MODE, ui.CAP_TELEOP_JOG,
        ui.CAP_TELEOP_POSE, ui.CAP_GRIPPER_COMMAND, ui.CAP_DUAL_SYNC,
        ui.CAP_ARM_CALIBRATION, ui.CAP_TOOL_CALIBRATION,
        ui.CAP_DUAL_TOOL_CALIBRATION,
    })


def test_mode_tokens_match_the_arm_ui_contract():
    from operator_console.arm_ui import contracts as ui

    assert K.MODE_MANUAL == ui.MODE_MANUAL
    assert K.MODE_FSM == ui.MODE_FSM


def test_tool_targets_match_the_arm_ui_contract():
    from operator_console.arm_ui import contracts as ui

    assert K.TOOL_TARGETS == frozenset({
        ui.TARGET_SINGLE, ui.TARGET_LEFT, ui.TARGET_RIGHT, ui.TARGET_BOTH,
    })
    assert K.TOOL_COMMANDS == frozenset({
        ui.COMMAND_OPEN, ui.COMMAND_CLOSE, ui.COMMAND_STOP,
    })


def test_calibration_action_tokens_match_the_arm_ui_contract():
    from operator_console.arm_ui import contracts as ui

    assert {
        ui.ACTION_MEASURE, ui.ACTION_VERIFY, ui.ACTION_APPLY_TEMPORARY,
        ui.ACTION_SAVE, ui.ACTION_START, ui.ACTION_CANCEL,
        ui.ACTION_CAPTURE_OPEN, ui.ACTION_CAPTURE_CLOSE,
    } == K.CALIBRATION_ACTION_TOKENS


def test_context_is_frozen_so_a_decision_cannot_be_edited_underneath():
    context = granted_context()
    with pytest.raises(Exception):
        context.granted_mode = K.MODE_MANUAL       # type: ignore[misc]
    assert replace(context, granted_mode="").manual_granted is False
