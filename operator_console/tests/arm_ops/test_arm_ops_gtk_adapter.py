"""The real arm tabs driven through the real command path.

The pure tests prove the policy and the jog lifetime.  This one proves the two
halves actually fit: a widget the operator presses reaches the session, and a
widget the policy refuses is not pressable in the first place.

Needs a display (see ``conftest``).  Still sends nothing: the transport is the
recording double, and the contract gate is separately proven to refuse every
arm action against the real ops contract.
"""
from __future__ import annotations

from dataclasses import replace

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.arm_ops import contract as K
from operator_console.arm_ops.adapter import ArmOpsCallbackAdapter
from operator_console.arm_ops.session import ArmCommandSession
from operator_console.arm_ops.transport import (
    ContractGatedTransport,
    NullArmTransport,
    RecordingArmTransport,
)
from operator_console.arm_ui import contracts as ui
from operator_console.arm_ui import fixtures
from operator_console.arm_ui.calibration_tab import ArmCalibrationTab
from operator_console.arm_ui.manual_tab import ArmManualTab

from test_arm_ops_session import FakeClock


def build(state=None, *, transport=None):
    clock = FakeClock()
    transport = transport if transport is not None else RecordingArmTransport()
    session = ArmCommandSession(clock=clock, transport=transport)
    adapter = ArmOpsCallbackAdapter(session)
    tab = ArmManualTab(adapter.callbacks())
    live = state if state is not None else fixtures.single_gripper()
    adapter.update_state(live, state_revision=42)
    tab.update_state(live)
    return clock, transport, adapter, tab, live


def test_a_gripper_button_press_reaches_the_command_session():
    clock, transport, adapter, tab, _state = build()
    try:
        button = next(
            widget for widget in tab.tool_panels.single.gate.widgets
            if isinstance(widget, Gtk.Button) and widget.get_label() == "열기"
        )
        assert button.get_sensitive() is True

        button.clicked()

        request, = transport.accepted
        assert request.action == K.ACTION_TOOL_COMMAND
        assert request.params["command"] == K.COMMAND_OPEN
        assert request.params["tool_id"] == "TOOL-S1"
        assert request.params["tool_generation"] == 7
    finally:
        tab.dispose()


def test_a_keyboard_jog_hold_produces_a_deadman_jog_then_a_stop():
    clock, transport, adapter, tab, _state = build()
    try:
        held = tab.keyboard._jog_plus
        held.emit("pressed")
        adapter.pump()

        jog, = transport.accepted
        assert jog.action == K.ACTION_JOG
        assert jog.params["axis"] == 2
        assert jog.params["expires_at_s"] > clock.now
        transport.accepted.clear()

        held.emit("released")
        adapter.pump()

        stop, = transport.accepted
        assert stop.action == K.ACTION_JOG_STOP
        assert stop.params["reason"] == K.STOP_REASON_EXPLICIT
    finally:
        tab.dispose()


def test_choosing_a_tool_candidate_still_sends_nothing():
    clock, transport, adapter, tab, _state = build()
    try:
        tab._candidate.set_active_id(ui.TOOL_DUAL_GRIPPER)

        assert transport.accepted == []

        tab._change_button.clicked()
        request, = transport.accepted
        assert request.action == K.ACTION_TOOL_CHANGE
        assert request.params["requested_kind"] == ui.TOOL_DUAL_GRIPPER
    finally:
        tab.dispose()


def test_the_ui_disables_exactly_what_the_policy_would_refuse():
    """The greyed control and the unbuildable command must agree.

    A control the UI leaves enabled for a command the policy refuses is the
    failure this pins: the operator presses it and nothing happens.
    """
    state = replace(
        fixtures.single_gripper(),
        authority=ui.ControlAuthority(
            requested_mode=ui.MODE_MANUAL, granted_mode="",
            status=ui.AUTHORITY_REQUESTED,
        ),
    )
    clock, transport, adapter, tab, _live = build(state)
    try:
        # No grant: every gripper control is greyed…
        assert all(
            not widget.get_sensitive()
            for widget in tab.tool_panels.single.gate.widgets
        )
        # …and the command is unbuildable too.
        assert not adapter.session.tool_command("single", "open")
        assert transport.accepted == []

        # The control-mode request stays available in both places.
        assert tab._manual_button.get_sensitive() is True
        assert adapter.session.request_mode(ui.MODE_MANUAL)
    finally:
        tab.dispose()


def test_leaving_the_tab_stops_a_live_hold_through_the_session():
    clock, transport, adapter, tab, _state = build()
    try:
        tab.keyboard._jog_plus.emit("pressed")
        assert adapter.session.registry.active_channels != ()

        tab._on_unmap()          # the tab switched away
        adapter.blur()

        assert adapter.session.registry.active_channels == ()
        adapter.pump()
        stop = transport.accepted[-1]
        assert stop.action == K.ACTION_JOG_STOP
        # The UI's own release wins the race; either way a stop went out.
        assert stop.params["reason"] in (
            K.STOP_REASON_EXPLICIT, K.STOP_REASON_FOCUS_LOST,
        )
    finally:
        tab.dispose()


def test_a_calibration_tab_button_reaches_the_session_with_its_scope():
    clock = FakeClock()
    transport = RecordingArmTransport()
    session = ArmCommandSession(clock=clock, transport=transport)
    adapter = ArmOpsCallbackAdapter(session)
    tab = ArmCalibrationTab(adapter.callbacks())
    state = fixtures.single_gripper()
    adapter.update_state(state, state_revision=42)
    tab.update_state(state)
    try:
        measure = next(
            widget for widget in tab._arm_rows[0].card.box.get_children()
            if isinstance(widget, Gtk.Box)
            for child in widget.get_children()
            if isinstance(child, Gtk.Button) and child.get_label() == "측정"
        )
        buttons = [
            child for child in measure.get_children()
            if isinstance(child, Gtk.Button) and child.get_label() == "측정"
        ]
        buttons[0].clicked()

        request, = transport.accepted
        assert request.action == K.ACTION_CALIBRATION
        assert request.params["scope"] == K.SCOPE_ARM
        assert request.params["step"] == "gear_ratio"
        assert request.params["action"] == "measure"
    finally:
        tab.dispose()


def test_with_the_real_contract_gate_no_press_can_send_anything():
    """End to end against the true registry: the door is shut.

    This is the honest state of the work — the UI, the policy and the session
    all function, and nothing can reach the arm.
    """
    import importlib.util
    from pathlib import Path

    path = (
        Path(__file__).resolve().parents[3] / "ros2" / "src" / "powertrain_ros"
        / "powertrain_ros" / "ops_contract.py"
    )
    spec = importlib.util.spec_from_file_location("_ops_contract_gtk_probe", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)

    sent = []
    gate = ContractGatedTransport(
        submit_fn=lambda *args: sent.append(args) or "nope",
        registered_actions=frozenset(module.ACTIONS),
    )
    clock, _transport, adapter, tab, _state = build(transport=gate)
    try:
        button = next(
            widget for widget in tab.tool_panels.single.gate.widgets
            if isinstance(widget, Gtk.Button) and widget.get_label() == "열기"
        )
        button.clicked()

        assert sent == []
        assert gate.refusals
        assert "등록되지 않았습니다" in gate.refusals[0][1]
    finally:
        tab.dispose()


def test_a_tab_built_with_no_transport_at_all_sends_nothing():
    clock = FakeClock()
    session = ArmCommandSession(clock=clock, transport=NullArmTransport())
    adapter = ArmOpsCallbackAdapter(session)
    tab = ArmManualTab(adapter.callbacks())
    adapter.update_state(fixtures.single_gripper(), state_revision=42)
    tab.update_state(fixtures.single_gripper())
    try:
        outcome = adapter.callbacks().go_home()

        assert not outcome
        assert outcome.reason == "transport_refused"
    finally:
        tab.dispose()
