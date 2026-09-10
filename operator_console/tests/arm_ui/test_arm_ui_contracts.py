"""Pure-logic tests for the injected-state contract.

These need no display: they exercise the defaults, the enable/disable gate and
the active-tool filter, which is where the safety-relevant decisions live.
"""
from __future__ import annotations

from dataclasses import replace

from operator_console.arm_ui import contracts as C
from operator_console.arm_ui import fixtures


def test_production_default_is_disconnected_and_ungranted():
    state = C.default_state()

    assert state.link.state == C.LINK_UNAVAILABLE
    assert state.detected_tool.kind == C.TOOL_UNKNOWN
    assert state.detected_tool.attached is None
    assert state.authority.status == C.AUTHORITY_UNKNOWN
    assert state.authority.granted_mode == ""
    assert state.tool_change.status == C.REQUEST_IDLE
    assert state.capabilities == frozenset()
    assert state.calibration.session_active is False
    assert state.calibration.unsaved_results is False
    assert state.is_fixture is False


def test_fixtures_are_flagged_and_never_leak_into_the_default():
    for name, (_title, factory) in fixtures.SCENARIOS.items():
        assert factory().is_fixture is True, name
    assert C.default_state().is_fixture is False


def test_missing_callback_blocks_before_anything_else():
    state = fixtures.single_gripper()

    enabled, reason = C.gate(state, None, C.CAP_TELEOP_JOG)

    assert enabled is False
    assert "미연결" in reason


def test_missing_capability_blocks_even_with_a_callback():
    state = replace(fixtures.single_gripper(), capabilities=frozenset())

    enabled, reason = C.gate(state, lambda *_: None, C.CAP_TELEOP_JOG)

    assert enabled is False
    assert "지원하지 않음" in reason


def test_stale_link_is_not_treated_as_usable():
    state = fixtures.stale_single()

    enabled, reason = C.gate(state, lambda *_: None, C.CAP_TELEOP_JOG)

    assert enabled is False
    assert "STALE" in reason


def test_bridge_restart_remains_available_when_the_bridge_telemetry_is_down():
    state = C.default_state()
    state = replace(state, capabilities=frozenset({C.CAP_BRIDGE_RESTART}))

    assert C.gate(state, lambda: None, C.CAP_BRIDGE_RESTART) == (True, "")


def test_requested_mode_alone_does_not_unlock_motion():
    state = replace(
        fixtures.single_gripper(),
        authority=C.ControlAuthority(
            requested_mode=C.MODE_MANUAL,
            granted_mode="",
            status=C.AUTHORITY_REQUESTED,
        ),
    )

    enabled, reason = C.gate(state, lambda *_: None, C.CAP_TELEOP_JOG)

    assert enabled is False
    assert reason == "수동 제어권 미승인"
    # Requesting the grant itself must stay reachable, or it can never arrive.
    assert C.gate(state, lambda *_: None, C.CAP_CONTROL_MODE)[0] is True


def test_tool_change_in_flight_blocks_operation_but_not_the_change_itself():
    state = replace(
        fixtures.single_gripper(),
        tool_change=C.ToolChangeRequest(
            status=C.REQUEST_IN_PROGRESS, requested_kind=C.TOOL_DUAL_GRIPPER,
        ),
    )

    assert C.gate(state, lambda *_: None, C.CAP_GRIPPER_COMMAND)[0] is False
    assert C.gate(state, lambda *_: None, C.CAP_TOOL_CHANGE)[0] is True


def test_calibration_session_blocks_general_operation_only():
    state = replace(
        fixtures.single_gripper(),
        calibration=replace(
            fixtures.single_gripper().calibration, session_active=True,
        ),
    )

    assert C.gate(state, lambda *_: None, C.CAP_TELEOP_JOG)[0] is False
    assert C.gate(state, lambda *_: None, C.CAP_TOOL_CALIBRATION)[0] is True


def test_readings_from_another_tool_are_rejected():
    state = fixtures.single_gripper()
    leaked = replace(
        state,
        diagnostics=replace(
            state.diagnostics,
            motors=(
                C.ToolMotorReading(actuator_id=3),
                C.ToolMotorReading(actuator_id=4),
            ),
        ),
    )

    assert C.readings_match_active_tool(leaked) is False
    assert C.cleared_for_tool(leaked).diagnostics.motors == ()


def test_readings_from_a_superseded_generation_are_rejected():
    state = fixtures.single_gripper()
    late = replace(
        state, diagnostics=replace(state.diagnostics, generation=state.diagnostics.generation - 1),
    )

    assert C.readings_match_active_tool(late) is False
    assert C.cleared_for_tool(late).diagnostics.motors == ()


def test_matching_readings_pass_through_untouched():
    state = fixtures.dual_gripper()

    assert C.readings_match_active_tool(state) is True
    assert C.cleared_for_tool(state) is state


def test_unknown_tool_with_readings_is_treated_as_a_leak():
    # No actuator profile means nothing can be legitimately attributed.
    state = replace(
        fixtures.single_gripper(),
        detected_tool=C.ToolIdentity(generation=7),
    )

    assert C.readings_match_active_tool(state) is False
