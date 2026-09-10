"""Binding the command session to the arm UI's callback surface.

Pure: ``arm_ui.contracts`` has no GTK import, so these run with no display.
The GTK-side binding is covered separately in ``test_arm_ops_gtk_adapter``.
"""
from __future__ import annotations

from dataclasses import replace

from operator_console.arm_ops import contract as K
from operator_console.arm_ops.adapter import (
    UNBOUND_CALLBACKS,
    ArmOpsCallbackAdapter,
    context_from_state,
)
from operator_console.arm_ops.session import ArmCommandSession
from operator_console.arm_ops.transport import RecordingArmTransport
from operator_console.arm_ui import contracts as ui
from operator_console.arm_ui import fixtures

from test_arm_ops_session import FakeClock


def build_adapter(state=None, *, state_revision=42):
    clock = FakeClock()
    transport = RecordingArmTransport()
    session = ArmCommandSession(clock=clock, transport=transport)
    adapter = ArmOpsCallbackAdapter(session)
    adapter.update_state(
        state if state is not None else fixtures.single_gripper(),
        state_revision=state_revision,
    )
    return clock, transport, adapter


# --- state projection -------------------------------------------------------
def test_a_granted_ui_state_projects_to_a_granted_context():
    context = context_from_state(fixtures.single_gripper(), state_revision=7)

    assert context.link_live is True
    assert context.manual_granted is True
    assert context.tool_id == "TOOL-S1"
    assert context.tool_generation == 7
    assert context.capabilities == fixtures.ALL_CAPABILITIES
    assert context.state_revision == 7


def test_a_requested_but_ungranted_mode_projects_to_no_grant():
    """The single most important projection: asking is not holding."""
    state = replace(
        fixtures.single_gripper(),
        authority=ui.ControlAuthority(
            requested_mode=ui.MODE_MANUAL,
            granted_mode=ui.MODE_MANUAL,          # backend has not confirmed
            status=ui.AUTHORITY_REQUESTED,
        ),
    )

    context = context_from_state(state, state_revision=1)

    # granted_mode is only honoured when the status says GRANTED, so a state
    # that merely echoes the requested mode cannot unlock motion.
    assert context.granted_mode == ""
    assert context.manual_granted is False


def test_a_disconnected_ui_state_projects_to_a_refusing_context():
    context = context_from_state(fixtures.disconnected(), state_revision=1)

    assert context.link_live is False
    assert context.manual_granted is False
    assert context.capabilities == frozenset()
    assert context.tool_id == ""


def test_a_stale_ui_state_is_not_projected_as_live():
    context = context_from_state(fixtures.stale_single(), state_revision=1)

    assert context.link_live is False


def test_a_missing_ops_revision_projects_as_none():
    context = context_from_state(fixtures.single_gripper())

    assert context.state_revision is None


def test_an_in_flight_tool_change_projects_as_active():
    state = replace(
        fixtures.single_gripper(),
        tool_change=ui.ToolChangeRequest(status=ui.REQUEST_IN_PROGRESS),
    )

    assert context_from_state(state, state_revision=1).tool_change_active is True


def test_backend_block_reasons_are_carried_across():
    context = context_from_state(fixtures.cleaner_no_grant(), state_revision=1)

    assert context.blocked_reasons == ("수동 제어권이 승인되지 않았습니다.",)


# --- callback surface -------------------------------------------------------
def test_callbacks_returns_the_arm_ui_callback_structure():
    _clock, _transport, adapter = build_adapter()

    callbacks = adapter.callbacks()

    assert isinstance(callbacks, ui.ArmUiCallbacks)


def test_only_the_deliberately_unbound_callbacks_are_none():
    _clock, _transport, adapter = build_adapter()
    callbacks = adapter.callbacks()

    unbound = {
        name for name in ui.ArmUiCallbacks.__dataclass_fields__
        if getattr(callbacks, name) is None
    }

    assert unbound == set(UNBOUND_CALLBACKS)


def test_the_unbound_callbacks_are_the_ones_with_no_agreed_arm_meaning():
    # select_axis is a view change; change_speed has no confirmed arm-side
    # representation, and guessing one would put an unagreed command on the wire.
    assert set(UNBOUND_CALLBACKS) == {"select_axis", "change_speed"}


def test_a_bound_callback_reaches_the_transport():
    _clock, transport, adapter = build_adapter()
    callbacks = adapter.callbacks()

    outcome = callbacks.request_control_mode(ui.MODE_MANUAL)

    assert outcome
    assert transport.actions == (K.ACTION_MODE_REQUEST,)


def test_a_callback_returns_the_refusal_instead_of_silently_doing_nothing():
    _clock, transport, adapter = build_adapter(fixtures.disconnected())
    callbacks = adapter.callbacks()

    outcome = callbacks.go_home()

    assert not outcome
    assert outcome.korean
    assert transport.accepted == []


def test_every_outcome_reaches_the_sink_accepted_or_not():
    clock = FakeClock()
    transport = RecordingArmTransport()
    seen = []
    adapter = ArmOpsCallbackAdapter(
        ArmCommandSession(clock=clock, transport=transport),
        outcome_sink=seen.append,
    )
    adapter.update_state(fixtures.single_gripper(), state_revision=1)
    callbacks = adapter.callbacks()

    callbacks.go_home()                       # allowed
    callbacks.tool_command("single", "nope")  # invalid params

    assert len(seen) == 2
    assert [outcome.accepted for outcome in seen] == [True, False]


def test_calibration_callbacks_map_to_the_arm_and_tool_scopes():
    _clock, transport, adapter = build_adapter()
    callbacks = adapter.callbacks()

    callbacks.arm_calibration_command("gear_ratio", "measure")
    callbacks.tool_calibration_command("single", "start")

    scopes = [request.params["scope"] for request in transport.accepted]
    assert scopes == [K.SCOPE_ARM, K.SCOPE_TOOL]


def test_a_tool_swap_through_update_state_ends_holds():
    clock, transport, adapter = build_adapter()
    callbacks = adapter.callbacks()
    callbacks.tool_jog_start("single", 1)
    assert adapter.session.registry.active_channels != ()

    adapter.update_state(fixtures.dual_gripper(), state_revision=43)

    assert adapter.session.registry.active_channels == ()
    assert adapter.session.pending_stop_count == 1
    adapter.pump()
    assert transport.actions[-1] == K.ACTION_TOOL_JOG_STOP
    assert transport.accepted[-1].params["reason"] == K.STOP_REASON_TOOL_CHANGED


def test_blur_and_disconnect_are_exposed_for_the_tab_to_call():
    clock, _transport, adapter = build_adapter()
    adapter.callbacks().jog_start(2, 1)

    adapter.blur()
    assert adapter.session.pending_stop_count == 1

    adapter.callbacks().jog_start(2, 1)
    adapter.on_disconnect()
    assert adapter.session.pending_stop_count >= 1


def test_the_stop_for_a_swapped_tool_names_the_old_tool_not_the_new_one():
    clock, transport, adapter = build_adapter()
    adapter.callbacks().tool_jog_start("single", 1)

    adapter.update_state(fixtures.dual_gripper(), state_revision=43)
    adapter.pump()

    stop = transport.accepted[-1]
    assert stop.params["tool_id"] == "TOOL-S1"       # the tool that is moving
    assert stop.params["tool_generation"] == 7


def _run_probe(code: str):
    import subprocess
    import sys
    from pathlib import Path

    repo_root = Path(__file__).resolve().parents[3]
    return subprocess.run(
        [sys.executable, "-c", code],
        cwd=repo_root, capture_output=True, text=True,
        env={"PATH": "/usr/bin:/bin", "PYTHONPATH": str(repo_root)},
    )


def test_the_pure_core_imports_without_gtk():
    """Contract, policy, jog, session and transport must need no display.

    ``arm_ui``'s package __init__ pulls in the GTK tab modules, so the adapter
    export is lazy.  Without that, importing the command contract would drag
    GStreamer and a display requirement into every pure test.
    """
    result = _run_probe(
        "import sys;"
        "import operator_console.arm_ops as a;"
        "assert 'gi' not in sys.modules, 'gi was imported';"
        "from operator_console.arm_ops import policy, jog, session, transport;"
        "assert 'gi' not in sys.modules, 'gi was imported by a core module';"
        "print(len(a.ARM_ACTIONS))"
    )

    assert result.returncode == 0, result.stderr
    assert result.stdout.strip() == "17"


def test_the_adapter_is_still_reachable_from_the_package_root():
    # Lazy must not mean missing: the documented import has to keep working.
    from operator_console import arm_ops

    assert arm_ops.ArmOpsCallbackAdapter is ArmOpsCallbackAdapter
    assert "ArmOpsCallbackAdapter" in dir(arm_ops)


def test_an_unknown_attribute_still_raises_attribute_error():
    import pytest as _pytest

    from operator_console import arm_ops

    with _pytest.raises(AttributeError):
        arm_ops.no_such_thing
