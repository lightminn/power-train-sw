"""Widget behaviour of 로봇팔·도구 캘리브레이션 탭."""
from __future__ import annotations

from dataclasses import replace

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.arm_ui import contracts as C
from operator_console.arm_ui import fixtures
from operator_console.arm_ui.calibration_tab import ARM_STEP_ORDER, ArmCalibrationTab

from _arm_ui_helpers import all_text, find_buttons, label_texts, recording_callbacks, walk


def test_tab_splits_into_arm_and_current_tool_areas():
    tab = ArmCalibrationTab()
    try:
        assert tab.inner_stack.get_child_by_name("arm") is not None
        assert tab.inner_stack.get_child_by_name("tool") is not None
        assert tab.inner_stack.get_visible_child_name() == "arm"
    finally:
        tab.dispose()


def test_arm_flow_is_gear_ratio_then_zero_then_range():
    assert [key for key, _title, _desc in ARM_STEP_ORDER] == [
        "gear_ratio", "zero", "range",
    ]

    tab = ArmCalibrationTab()
    try:
        assert [row.key for row in tab._arm_rows] == ["gear_ratio", "zero", "range"]
    finally:
        tab.dispose()


def test_later_arm_steps_are_blocked_until_the_earlier_one_is_done():
    callbacks, _calls = recording_callbacks()
    tab = ArmCalibrationTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())  # gear_ratio DONE, zero IDLE
        reasons = tab.gate.reasons(fixtures.single_gripper())

        assert "이전 단계 미완료" in reasons
    finally:
        tab.dispose()


def test_measure_verify_apply_and_save_are_reported_separately():
    tab = ArmCalibrationTab()
    try:
        tab.update_state(fixtures.single_gripper())
        texts = all_text(tab.widget)

        # The gear-ratio fixture is measured, verified and temporarily applied
        # but NOT saved -- all four must be legible as distinct facts.
        assert "축1 1:36.0 / 축2 1:36.0" in texts
        assert "오차 0.3%" in texts
        assert "적용됨(임시)" in texts
        assert "미저장" in texts
        assert "임시 적용 상태입니다" in texts
    finally:
        tab.dispose()


def test_save_is_blocked_without_a_verification_result():
    callbacks, _calls = recording_callbacks()
    tab = ArmCalibrationTab(callbacks)
    try:
        state = replace(
            fixtures.single_gripper(),
            calibration=C.CalibrationState(
                arm_steps=(
                    C.CalibrationStep(
                        key="gear_ratio", title="기어비", status=C.STEP_RUNNING,
                        measured="1:36.0",
                    ),
                ),
            ),
        )
        tab.update_state(state)

        assert "검증 결과 없음" in tab.gate.reasons(state)
    finally:
        tab.dispose()


def test_unreceived_step_state_is_not_shown_as_complete():
    tab = ArmCalibrationTab()
    try:
        tab.update_state(fixtures.disconnected())
        texts = all_text(tab.widget)

        assert "미수신을 완료로 표시하지 않습니다" in texts
        assert "저장됨" not in texts
    finally:
        tab.dispose()


def test_tool_area_follows_the_attached_tool():
    tab = ArmCalibrationTab()
    try:
        tab.update_state(fixtures.single_gripper())
        assert set(tab.tool_area._target_widgets) == {C.TARGET_SINGLE}

        tab.update_state(fixtures.dual_gripper())
        assert set(tab.tool_area._target_widgets) == {C.TARGET_LEFT, C.TARGET_RIGHT}

        tab.update_state(fixtures.cleaner_no_grant())
        assert tab.tool_area._target_widgets == {}
    finally:
        tab.dispose()


def test_unsupported_tool_calibration_says_so_explicitly():
    tab = ArmCalibrationTab()
    try:
        tab.update_state(fixtures.cleaner_no_grant())
        texts = all_text(tab.widget)

        assert "지원하지 않습니다" in texts
        assert "새 보정 알고리즘 추가는 이 화면의 범위가 아닙니다" in texts
    finally:
        tab.dispose()


def test_dual_sync_result_is_shown_for_dual_tools_only():
    tab = ArmCalibrationTab()
    try:
        tab.update_state(fixtures.dual_gripper())
        assert tab.tool_area._sync_row.get_visible() is True
        assert "편차 있음" in " ".join(label_texts(tab.widget))

        tab.update_state(fixtures.single_gripper())
        assert tab.tool_area._sync_row.get_visible() is False
    finally:
        tab.dispose()


def test_active_session_and_unsaved_results_are_announced():
    tab = ArmCalibrationTab()
    try:
        tab.update_state(fixtures.dual_gripper())
        texts = all_text(tab.widget)

        assert "세션 진행 중" in texts
        assert "저장되지 않은 보정 결과가 있습니다" in texts
    finally:
        tab.dispose()


def test_communication_loss_during_a_session_is_not_shown_as_finished():
    tab = ArmCalibrationTab()
    try:
        state = replace(
            fixtures.dual_gripper(),
            link=C.SourceLink(state=C.LINK_UNAVAILABLE),
        )
        tab.update_state(state)
        texts = all_text(tab.widget)

        assert "통신이 끊긴 상태입니다" in texts
        assert "진행 상태를 완료로 표시하지 않습니다" in texts
    finally:
        tab.dispose()


def test_no_callbacks_means_no_operable_calibration_control():
    tab = ArmCalibrationTab()
    try:
        tab.update_state(fixtures.single_gripper())

        assert tab.gated_widgets
        assert all(not widget.get_sensitive() for widget in tab.gated_widgets)
    finally:
        tab.dispose()


def test_missing_dual_capability_disables_dual_endpoint_calibration():
    callbacks, _calls = recording_callbacks()
    tab = ArmCalibrationTab(callbacks)
    try:
        state = replace(
            fixtures.dual_gripper(),
            capabilities=fixtures.ALL_CAPABILITIES - {C.CAP_DUAL_TOOL_CALIBRATION},
        )
        tab.update_state(state)

        assert tab.tool_area._start.get_sensitive() is False
        assert "지원하지 않음" in tab.tool_area._start.get_tooltip_text()
    finally:
        tab.dispose()


def test_calibration_jog_hold_reports_start_and_stop_once():
    callbacks, calls = recording_callbacks()
    tab = ArmCalibrationTab(callbacks)
    try:
        tab.update_state(fixtures.dual_gripper())
        calls.clear()
        held = tab.tool_area.holds[0]
        held.emit("pressed")
        held.emit("released")

        assert calls == [
            ("tool_calibration_jog_start", (C.TARGET_LEFT, -1)),
            ("tool_calibration_jog_stop", (C.TARGET_LEFT,)),
        ]
    finally:
        tab.dispose()


def test_link_loss_releases_a_calibration_jog():
    callbacks, calls = recording_callbacks()
    tab = ArmCalibrationTab(callbacks)
    try:
        tab.update_state(fixtures.dual_gripper())
        tab.tool_area.holds[0].emit("pressed")
        calls.clear()

        tab.update_state(fixtures.disconnected())

        assert ("tool_calibration_jog_stop", (C.TARGET_LEFT,)) in calls
    finally:
        tab.dispose()


def test_arm_calibration_button_reports_step_and_action():
    callbacks, calls = recording_callbacks()
    tab = ArmCalibrationTab(callbacks)
    try:
        tab.update_state(fixtures.single_gripper())
        # Find the 측정 button inside the first (기어비) step card by label,
        # not by index, so a layout tweak does not silently retarget the test.
        buttons = find_buttons(tab._arm_rows[0].card.box, "측정")
        assert buttons
        calls.clear()
        buttons[0].clicked()

        assert calls == [("arm_calibration_command", ("gear_ratio", "measure"))]
    finally:
        tab.dispose()


def test_page_is_scrollable_and_wraps_long_text():
    import gi as _gi

    _gi.require_version("Pango", "1.0")
    from gi.repository import Pango

    tab = ArmCalibrationTab()
    try:
        tab.update_state(fixtures.dual_gripper())
        assert isinstance(tab.widget, Gtk.ScrolledWindow)
        assert tab.widget.get_policy() == (
            Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC,
        )
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


def test_page_fits_a_small_window_without_sideways_scrolling():
    tab = ArmCalibrationTab()
    try:
        tab.update_state(fixtures.dual_gripper())
        minimum, _natural = tab._page.get_preferred_width()
        assert minimum <= 430, f"page minimum {minimum}px exceeds 430px"
    finally:
        tab.dispose()


def test_dispose_removes_handlers():
    tab = ArmCalibrationTab()
    tab.update_state(fixtures.dual_gripper())
    assert tab.tracker.handler_count > 0

    tab.dispose()

    assert tab.tracker.handler_count == 0
    assert tab.tracker.source_count == 0
    assert tab.tool_area.target_tracker.handler_count == 0
