"""Widget behaviour of 로봇팔 수동조작 탭."""
from __future__ import annotations

from dataclasses import replace

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.arm_ui import contracts as C
from operator_console.arm_ui import fixtures
from operator_console.arm_ui.manual_tab import ArmManualTab

from _arm_ui_helpers import label_texts, recording_callbacks, walk


# --- panel switching --------------------------------------------------------
def test_panel_follows_the_detected_tool_kind():
    tab = ArmManualTab()
    try:
        tab.update_state(fixtures.single_gripper())
        assert tab.tool_panels.active_name == C.TOOL_SINGLE_GRIPPER

        tab.update_state(fixtures.dual_gripper())
        assert tab.tool_panels.active_name == C.TOOL_DUAL_GRIPPER

        tab.update_state(fixtures.cleaner_no_grant())
        assert tab.tool_panels.active_name == "other"

        tab.update_state(fixtures.disconnected())
        assert tab.tool_panels.active_name == "other"
    finally:
        tab.dispose()


def test_disconnected_state_does_not_render_a_detected_tool():
    tab = ArmManualTab()
    try:
        tab.update_state(fixtures.single_gripper())
        tab.update_state(fixtures.disconnected())
        texts = label_texts(tab.widget)

        assert any("감지 대기" in text for text in texts)
        # The previously seen gripper must not still be presented as detected.
        assert "TOOL-S1" not in texts
    finally:
        tab.dispose()


def test_stale_state_is_not_shown_as_normal():
    tab = ArmManualTab()
    try:
        tab.update_state(fixtures.stale_single())
        texts = " ".join(label_texts(tab.widget))

        assert "지연(STALE)" in texts
        assert "조작 가능" not in texts
    finally:
        tab.dispose()


# --- no cross-tool leakage --------------------------------------------------
def test_previous_tool_motor_rows_do_not_survive_a_tool_change():
    tab = ArmManualTab()
    try:
        tab.update_state(fixtures.dual_gripper())
        dual_texts = " ".join(label_texts(tab.widget))
        assert "ID 3" in dual_texts and "ID 4" in dual_texts

        tab.update_state(fixtures.single_gripper())
        single_texts = " ".join(label_texts(tab.widget))

        assert "ID 5" in single_texts
        assert "ID 3" not in single_texts
        assert "ID 4" not in single_texts
    finally:
        tab.dispose()


def test_readings_for_another_tool_are_dropped_before_rendering():
    tab = ArmManualTab()
    try:
        state = fixtures.single_gripper()
        leaked = replace(
            state,
            diagnostics=replace(
                state.diagnostics,
                motors=(C.ToolMotorReading(actuator_id=3, position_deg=99.0),),
            ),
        )
        tab.update_state(leaked)
        texts = " ".join(label_texts(tab.widget))

        assert "99.0" not in texts
        assert "판독 없음" in texts
    finally:
        tab.dispose()


# --- gating -----------------------------------------------------------------
def test_every_control_is_insensitive_without_callbacks():
    tab = ArmManualTab()  # no callbacks injected at all
    try:
        tab.update_state(fixtures.single_gripper())
        gated = (
            tab.gate.widgets
            + tab.keyboard.gate.widgets
            + tab.tool_panels.single.gate.widgets
        )

        assert gated
        assert all(not widget.get_sensitive() for widget in gated)
        assert all(
            "조작 불가" in (widget.get_tooltip_text() or "") for widget in gated
        )
    finally:
        tab.dispose()


def test_controls_stay_insensitive_when_capabilities_are_absent():
    callbacks, _calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        state = replace(fixtures.single_gripper(), capabilities=frozenset())
        tab.update_state(state)

        assert all(
            not widget.get_sensitive()
            for widget in tab.tool_panels.single.gate.widgets
        )
    finally:
        tab.dispose()


def test_full_capabilities_and_grant_enable_the_gripper_panel():
    callbacks, _calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())

        assert all(
            widget.get_sensitive()
            for widget in tab.tool_panels.single.gate.widgets
        )
    finally:
        tab.dispose()


# --- selection is not a command --------------------------------------------
def test_choosing_a_tool_candidate_sends_nothing():
    callbacks, calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        tab._candidate.set_active_id(C.TOOL_DUAL_GRIPPER)

        assert calls == []
    finally:
        tab.dispose()


def test_explicit_change_request_sends_the_selected_candidate():
    callbacks, calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        tab._candidate.set_active_id(C.TOOL_DUAL_GRIPPER)
        tab._change_button.clicked()

        assert calls == [("request_tool_change", (C.TOOL_DUAL_GRIPPER,))]
    finally:
        tab.dispose()


def test_change_request_is_blocked_until_a_candidate_is_chosen():
    callbacks, _calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())

        assert tab._change_button.get_sensitive() is False
        assert "변경 후보 미선택" in tab._change_button.get_tooltip_text()
    finally:
        tab.dispose()


def test_state_updates_do_not_overwrite_the_operator_candidate():
    callbacks, _calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        tab._candidate.set_active_id(C.TOOL_CLEANER)
        tab.update_state(fixtures.dual_gripper())

        assert tab._candidate.get_active_id() == C.TOOL_CLEANER
    finally:
        tab.dispose()


def test_selecting_an_axis_reports_intent_but_moves_nothing():
    callbacks, calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        calls.clear()
        tab.keyboard._axis_buttons[3].set_active(True)

        assert calls == [("select_axis", (3,))]
    finally:
        tab.dispose()


# --- hold lifecycle ---------------------------------------------------------
def test_jog_hold_reports_start_and_stop_exactly_once():
    callbacks, calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        calls.clear()
        held = tab.keyboard._jog_plus
        held.emit("pressed")
        held.emit("pressed")          # auto-repeat must not re-arm
        held.emit("released")
        held.emit("released")

        assert calls == [("jog_start", (2, 1)), ("jog_stop", (2,))]
    finally:
        tab.dispose()


def test_losing_the_manual_grant_ends_an_active_hold():
    callbacks, calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        tab.keyboard._jog_plus.emit("pressed")
        calls.clear()

        revoked = replace(
            fixtures.single_gripper(),
            authority=C.ControlAuthority(status=C.AUTHORITY_NONE),
        )
        tab.update_state(revoked)

        assert ("jog_stop", (2,)) in calls
        assert tab.keyboard._jog_plus.held is False
    finally:
        tab.dispose()


def test_losing_the_feed_ends_an_active_hold():
    callbacks, calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        tab.keyboard._jog_plus.emit("pressed")
        calls.clear()

        tab.update_state(fixtures.disconnected())

        assert ("jog_stop", (2,)) in calls
    finally:
        tab.dispose()


def test_tool_change_ends_holds_on_the_outgoing_panel():
    callbacks, calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        single_hold = tab.tool_panels.single.holds[0]
        single_hold.emit("pressed")
        calls.clear()

        tab.update_state(fixtures.dual_gripper())

        assert ("tool_jog_stop", (C.TARGET_SINGLE,)) in calls
        assert single_hold.held is False
    finally:
        tab.dispose()


def test_losing_sensitivity_mid_hold_releases_the_jog():
    callbacks, calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        held = tab.keyboard._jog_plus
        held.emit("pressed")
        calls.clear()

        held.set_sensitive(False)

        assert calls == [("jog_stop", (2,))]
    finally:
        tab.dispose()


# --- keyboard routing -------------------------------------------------------
def test_keys_are_ignored_while_the_panel_is_inactive():
    callbacks, calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        tab.keyboard.set_active(False)
        calls.clear()

        window = Gtk.Window()
        assert tab.keyboard._keys_allowed(window) is False
        window.destroy()
    finally:
        tab.dispose()


def test_text_entry_focus_suppresses_teleop_keys():
    from operator_console.arm_ui.keyboard_panel import focus_is_text_entry

    window = Gtk.Window()
    entry = Gtk.Entry()
    window.add(entry)
    window.show_all()
    entry.grab_focus()
    try:
        assert focus_is_text_entry(window) is True
    finally:
        window.destroy()


def test_axis_six_is_blocked_for_tools_without_that_joint():
    callbacks, _calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        assert tab.keyboard._axis_buttons[6].get_sensitive() is True

        cleaner = replace(
            fixtures.cleaner_no_grant(),
            authority=C.ControlAuthority(
                requested_mode=C.MODE_MANUAL,
                granted_mode=C.MODE_MANUAL,
                status=C.AUTHORITY_GRANTED,
            ),
            block_reasons=(),
        )
        tab.update_state(cleaner)

        assert tab.keyboard._axis_buttons[6].get_sensitive() is False
        assert "6번 축" in tab.keyboard._axis_buttons[6].get_tooltip_text()
    finally:
        tab.dispose()


# --- blocking reasons -------------------------------------------------------
def test_block_reasons_from_state_are_shown_verbatim():
    tab = ArmManualTab()
    try:
        tab.update_state(fixtures.cleaner_no_grant())
        texts = " ".join(label_texts(tab.widget))

        assert "수동 제어권이 승인되지 않았습니다." in texts
    finally:
        tab.dispose()


# --- teardown ---------------------------------------------------------------
def test_dispose_removes_every_handler_and_timer():
    callbacks, _calls = recording_callbacks()
    tab = ArmManualTab(callbacks)
    tab.update_state(fixtures.dual_gripper())
    assert tab.tracker.handler_count > 0

    tab.dispose()

    assert tab.tracker.handler_count == 0
    assert tab.tracker.source_count == 0
    assert tab.keyboard.tracker.handler_count == 0
    assert tab.tool_panels.single.tracker.handler_count == 0
    assert tab.tool_panels.dual.tracker.handler_count == 0


def test_dispose_is_idempotent():
    tab = ArmManualTab()
    tab.dispose()
    tab.dispose()


# --- layout resilience ------------------------------------------------------
def test_page_is_scrollable_so_a_small_window_does_not_truncate():
    tab = ArmManualTab()
    try:
        tab.update_state(fixtures.dual_gripper())
        assert isinstance(tab.widget, Gtk.ScrolledWindow)
        horizontal, vertical = tab.widget.get_policy()
        assert horizontal == Gtk.PolicyType.AUTOMATIC
        assert vertical == Gtk.PolicyType.AUTOMATIC

        # The scroller itself must be able to shrink well below the content's
        # own minimum, which is what makes scrolling (not clipping) happen.
        scroller_min, _natural = tab.widget.get_preferred_width()
        assert scroller_min <= 420
    finally:
        tab.dispose()


def test_page_fits_a_small_window_without_sideways_scrolling():
    """The page's own minimum must stay inside the narrow-window budget.

    Vertical scrolling is expected; a horizontal scrollbar on every card is
    not.  This pins the budget so a future wide control (a long button label,
    an extra column) is caught here instead of on the operator's screen.
    """
    for tab in (ArmManualTab(), ):
        try:
            tab.update_state(fixtures.dual_gripper())   # the widest scenario
            minimum, _natural = tab._page.get_preferred_width()
            assert minimum <= 430, f"page minimum {minimum}px exceeds 430px"
        finally:
            tab.dispose()


def test_long_text_wraps_or_ellipsizes_rather_than_clipping():
    import gi as _gi

    _gi.require_version("Pango", "1.0")
    from gi.repository import Pango

    tab = ArmManualTab()
    try:
        tab.update_state(fixtures.dual_gripper())
        offenders = [
            item.get_text()
            for item in walk(tab.widget)
            if isinstance(item, Gtk.Label)
            and len(item.get_text()) > 30
            and not item.get_line_wrap()
            and item.get_ellipsize() == Pango.EllipsizeMode.NONE
        ]

        assert offenders == []
    finally:
        tab.dispose()


def test_tab_renders_at_a_small_allocation_without_error():
    tab = ArmManualTab()
    window = Gtk.Window()
    window.set_default_size(420, 320)
    window.add(tab.widget)
    try:
        tab.update_state(fixtures.dual_gripper())
        window.show_all()
        while Gtk.events_pending():
            Gtk.main_iteration_do(False)
        allocation = tab.widget.get_allocation()
        assert allocation.width > 0 and allocation.height > 0
    finally:
        tab.dispose()
        window.destroy()
