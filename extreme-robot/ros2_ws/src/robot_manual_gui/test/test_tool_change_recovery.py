"""Tool-change request recovery: every outcome releases the GUI latch.

The clock is injected, so the deadlines are exercised without sleeping and the
test asserts the same code path the 500 ms watchdog runs in production.
"""

import os
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


class FakeClock:
    """Monotonic source under the test's control."""

    def __init__(self, start=1000.0):
        self.value = start

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds
        return self.value


def _window(scope='END_EFFECTOR_ONLY'):
    from PyQt5.QtWidgets import QApplication
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.ros_interface import GuiSignals

    app = QApplication.instance() or QApplication([])
    requests = []
    node = SimpleNamespace(
        control_scope=scope, selected_tool='dual_motor_gripper', read_only=False,
        positions={}, efforts={}, gripper_busy=False, control_mode='MANUAL',
        request_mode=lambda _mode: None, jog_arm=lambda *_args: None,
        command_arm=lambda *_args: None,
        command_gripper=lambda position: True,
        command_tool_fsm=lambda command, **_values: True,
        stop_gripper=lambda: None, command_cleaner=lambda *_args: None,
        command_cleaner_direction=lambda *_args: True,
        emergency_stop=lambda: None, tool_detached=lambda: None,
        set_dual_motor_enabled=lambda *_args: True,
        set_current_tool_enabled=lambda *_args: True,
        manual_dual_recovery_jog=lambda *_args: True,
        command_dual_calibration=lambda *_args, **_kwargs: True,
        keyboard_shortcuts_enabled=True,
        request_tool_change=lambda tool: (requests.append(tool) or True))
    profile = {
        'calibrated': True, 'actuator_ids': [3, 4],
        'open_position': 1.0, 'close_position': 0.0,
        'safe_min_tick': -526, 'safe_max_tick': 2384,
        'motor_endpoints': {
            3: {'open': 1056, 'close': -526},
            4: {'open': 2384, 'close': 839}}}
    window = ManualMainWindow(node, GuiSignals(), profile, False)
    clock = FakeClock()
    window._clock = clock
    return app, window, clock, requests


def _ready_status():
    return {
        'control_scope': 'END_EFFECTOR_ONLY', 'tool_type': 'dual_motor_gripper',
        'profile_valid': True, 'calibrated': True,
        'actuators_discovered': True, 'motion_allowed': True,
        'tool_enable_allowed': True, 'tool_torque_state': 'OFF',
        'tool_preparation_state': 'READY', 'tool_preparation_error': '',
        'read_only': False, 'emergency_stop': False, 'tool_detached': False,
        'bridge_connected': True, 'fsm_state': 'READY',
        'control_mode': 'MANUAL',
        'fsm_class': 'DualMotorGripperFSM',
        'endpoint_calibration_verified': True,
        'tool_profile': {
            'calibrated': True, 'actuator_ids': [3, 4],
            'open_position': 1.0, 'close_position': 0.0,
            'safe_min_tick': -526, 'safe_max_tick': 2384,
            'motor_endpoints': {
                3: {'open': 1056, 'close': -526},
                4: {'open': 2384, 'close': 839}}},
        'synchronization': {
            'state': 'SYNCHRONIZED', 'spread': 0.0, 'limit': 0.05},
        'dual_calibration': {'state': 'READY', 'active': False},
        'actuators': [
            {'id': 3, 'online': True, 'position': 265, 'effort': 10,
             'torque_state': 'ON', 'hardware_error': 0},
            {'id': 4, 'online': True, 'position': 1612, 'effort': 10,
             'torque_state': 'ON', 'hardware_error': 0}]}


def _motion_buttons(window):
    return (window.open_button, window.close_button,
            window.hold_open_button, window.hold_close_button)


def _request(window, tool='spur_1motor_gripper'):
    window.tool_combo.setCurrentIndex(window.tool_combo.findData(tool))
    window._request_tool_change()


def _runtime_status(tool, ids, fsm='READY', torque='OFF', motion=False):
    profiles = {
        'dual_motor_gripper': {
            'calibrated': True, 'actuator_ids': [3, 4],
            'endpoint_calibration_verified': True,
            'motor_endpoints': {
                3: {'open': -675, 'close': -2211},
                4: {'open': 2559, 'close': 1017}},
            'profile_velocity': 80, 'profile_acceleration': 25},
        'spur_1motor_gripper': {
            'calibrated': True, 'actuator_ids': [5],
            'open_tick': 2945, 'close_tick': 3752,
            'safe_min_tick': 2945, 'safe_max_tick': 3752,
            'profile_velocity': 20, 'profile_acceleration': 5},
        'cleaner': {
            'calibrated': True, 'actuator_ids': [2],
            'backend': 'cleaner', 'direction': 1,
            'profile_velocity': 20, 'profile_acceleration': 5},
    }
    return {
        'control_scope': 'END_EFFECTOR_ONLY', 'tool_type': tool,
        'tool_profile': profiles[tool],
        'profile_valid': True, 'calibrated': True,
        'endpoint_calibration_verified': tool == 'dual_motor_gripper',
        'actuators_discovered': True, 'motion_allowed': motion,
        'tool_enable_allowed': True, 'tool_torque_state': torque,
        'tool_preparation_state': 'READY', 'tool_preparation_error': '',
        'tool_enable_reason': '',
        'tool_context_generation': 1,
        'read_only': False, 'emergency_stop': False, 'tool_detached': False,
        'bridge_connected': True, 'u2d2_connected': True,
        'control_mode': 'MANUAL', 'fsm_state': fsm,
        'fsm_class': {
            'dual_motor_gripper': 'DualMotorGripperFSM',
            'spur_1motor_gripper': 'SingleMotorGripperFSM',
            'cleaner': 'CleanerFSM'}[tool],
        'synchronization': {
            'state': 'SYNCHRONIZED', 'spread': 0.0, 'limit': 0.05},
        'dual_calibration': (
            {'state': 'READY', 'active': False}
            if tool == 'dual_motor_gripper' else None),
        'calibration': (
            {'active': False, 'enabled': torque == 'ON'}
            if tool == 'spur_1motor_gripper' else None),
        'actuators': [
            {'id': dxl_id, 'online': True, 'position': 1000 + dxl_id,
             'effort': 0, 'torque_state': torque, 'hardware_error': 0,
             'operating_mode': 1 if tool == 'cleaner' else 3,
             'goal_velocity': 0 if tool == 'cleaner' else None}
            for dxl_id in ids],
    }


def test_silent_bridge_times_out_and_leaves_ready_gates_untouched():
    """A silent bridge must not outlive the request latch."""
    _app, window, clock, requests = _window()
    try:
        window._update_tool_status(_ready_status())
        window._update_mode('MANUAL')
        baseline = [button.isEnabled() for button in _motion_buttons(window)]
        _request(window)
        assert requests == ['spur_1motor_gripper']
        assert window.pending_tool_change == 'spur_1motor_gripper'
        assert window.tool_change_state == 'PENDING'
        assert not window.tool_control_box.isEnabled()
        # The bridge keeps publishing status but never adopts the tool.
        for _ in range(6):
            clock.advance(0.5)
            window._update_tool_status(_ready_status())
            window._refresh_connection()
        assert window.pending_tool_change is None
        assert window.tool_change_state == 'TIMEOUT'
        assert window.tool_combo.currentData() == 'dual_motor_gripper'
        assert window.node.selected_tool == 'dual_motor_gripper'
        assert window.tool_control_box.isEnabled()
        # Recomputed, not force-enabled: identical to never having requested.
        assert [b.isEnabled() for b in _motion_buttons(window)] == baseline
    finally:
        window.close()


def test_pending_survives_until_the_deadline():
    _app, window, clock, _requests = _window()
    try:
        window._update_tool_status(_ready_status())
        _request(window)
        for _ in range(5):
            clock.advance(0.5)
            window._update_tool_status(_ready_status())
            window._refresh_connection()
            if window.pending_tool_change is None:
                break
        assert clock.value - 1000.0 == 2.5
        assert window.pending_tool_change == 'spur_1motor_gripper'
        assert window.tool_change_state == 'PENDING'
    finally:
        window.close()


def test_absent_bridge_reports_lost_link_and_keeps_motion_blocked():
    """No status at all is a lost link, not a slow bridge."""
    _app, window, clock, _requests = _window()
    try:
        _request(window)
        assert window.pending_tool_change == 'spur_1motor_gripper'
        clock.advance(1.0)
        window._refresh_connection()
        assert window.pending_tool_change is None
        assert window.tool_change_state == 'DISCONNECTED'
        assert window.tool_combo.currentData() == 'dual_motor_gripper'
        assert window.tool_control_box.isEnabled()
        # Releasing the latch must not hand out motion on a dead link.
        assert not any(b.isEnabled() for b in _motion_buttons(window))
        assert window._tool_block_reason() == '제어 노드 상태 수신 끊김'
    finally:
        window.close()


def test_switch_silence_on_a_healthy_link_waits_for_the_full_deadline():
    """The bridge stops publishing while it switches; that is not a lost link."""
    _app, window, clock, _requests = _window()
    try:
        window._update_tool_status(_ready_status())
        _request(window)
        # No further status at all, exactly like a bridge busy on the bus.
        clock.advance(1.6)
        window._refresh_connection()
        assert not window._status_fresh()
        assert window.pending_tool_change == 'spur_1motor_gripper'
        assert window.tool_change_state == 'PENDING'
        clock.advance(1.5)
        window._refresh_connection()
        assert window.tool_change_state == 'TIMEOUT'
    finally:
        window.close()


def test_late_success_after_timeout_is_still_adopted():
    """A slow switch that lands after the deadline must not desync the GUI."""
    _app, window, clock, _requests = _window()
    try:
        window._update_tool_status(_ready_status())
        _request(window)
        clock.advance(3.1)
        window._refresh_connection()
        assert window.tool_change_state == 'TIMEOUT'
        status = _ready_status()
        status.update(
            tool_type='spur_1motor_gripper', calibration_jog_enabled=True,
            calibration={'active': False, 'enabled': False},
            tool_profile={'calibrated': True, 'actuator_ids': [5],
                          'safe_min_tick': 2867, 'safe_max_tick': 3807,
                          'open_tick': 2867, 'close_tick': 3807},
            actuators=[{'id': 5, 'online': True, 'position': 3300,
                        'torque_state': 'ON', 'hardware_error': 0}])
        window._update_tool_status(status)
        assert window.node.selected_tool == 'spur_1motor_gripper'
        assert window.tool_combo.currentData() == 'spur_1motor_gripper'
        assert window.tool_control_box.isEnabled()
    finally:
        window.close()


def test_timed_out_request_can_be_retried_and_then_succeed():
    _app, window, clock, requests = _window()
    try:
        window._update_tool_status(_ready_status())
        _request(window)
        for _ in range(7):
            clock.advance(0.5)
            window._update_tool_status(_ready_status())
            window._refresh_connection()
        assert window.tool_change_state == 'TIMEOUT'
        _request(window)
        assert requests == ['spur_1motor_gripper', 'spur_1motor_gripper']
        assert window.pending_tool_change == 'spur_1motor_gripper'
        status = _ready_status()
        status.update(
            tool_type='spur_1motor_gripper', online=True, hardware_error=0,
            tool_torque_state='OFF', calibration_jog_enabled=True,
            calibration={'active': True, 'enabled': True},
            tool_profile={
                'calibrated': True, 'actuator_ids': [5],
                'safe_min_tick': 2867, 'safe_max_tick': 3807,
                'open_tick': 2867, 'close_tick': 3807},
            actuators=[{'id': 5, 'online': True, 'position': 3300,
                        'torque_state': 'OFF', 'hardware_error': 0,
                        'operating_mode': 3}])
        clock.advance(0.2)
        window._update_tool_status(status)
        assert window.pending_tool_change is None
        assert window.tool_change_state == 'DONE'
        assert window.node.selected_tool == 'spur_1motor_gripper'
        assert window.tool_combo.currentData() == 'spur_1motor_gripper'
        assert window.tool_control_box.isEnabled()
    finally:
        window.close()


def test_bridge_error_reports_rejection_and_restores_active_tool():
    _app, window, clock, _requests = _window()
    try:
        window._update_tool_status(_runtime_status(
            'dual_motor_gripper', [3, 4], torque='ON', motion=True))
        _request(window, 'cleaner')
        status = _ready_status()
        status['tool_change'] = {'error': 'motion active'}
        clock.advance(0.2)
        window._update_tool_status(status)
        assert window.pending_tool_change is None
        assert window.tool_change_state == 'REJECTED'
        assert window.tool_change_detail == 'motion active'
        assert window.tool_combo.currentData() == 'dual_motor_gripper'
        assert window.node.selected_tool == 'dual_motor_gripper'
        assert window.tool_control_box.isEnabled()
    finally:
        window.close()


def test_operator_can_cancel_a_pending_request():
    _app, window, clock, _requests = _window()
    try:
        window._update_tool_status(_ready_status())
        assert not window.cancel_tool_change.isEnabled()
        _request(window)
        assert window.cancel_tool_change.isEnabled()
        clock.advance(0.3)
        window.cancel_tool_change.click()
        assert window.pending_tool_change is None
        assert window.tool_change_state == 'CANCELED'
        assert not window.cancel_tool_change.isEnabled()
        assert window.tool_combo.currentData() == 'dual_motor_gripper'
        assert window.tool_control_box.isEnabled()
    finally:
        window.close()


def test_cancel_button_is_outside_the_latched_tool_panel():
    """The way out of a pending request must not be latched by it."""
    _app, window, _clock, _requests = _window()
    try:
        window._update_tool_status(_ready_status())
        _request(window)
        assert not window.tool_control_box.isEnabled()
        assert window.cancel_tool_change not in window.tool_control_box.findChildren(
            type(window.cancel_tool_change))
        assert window.cancel_tool_change.isEnabled()
    finally:
        window.close()


def test_request_state_and_block_reason_are_shown_in_korean():
    import re

    _app, window, clock, _requests = _window()
    try:
        window._update_tool_status(_runtime_status(
            'dual_motor_gripper', [3, 4], torque='ON', motion=True))
        window._update_mode('MANUAL')
        assert window.status_labels['tool_block'].text() == '없음'
        _request(window)
        window._refresh_tool_change_labels()
        pending_text = window.status_labels['tool_change'].text()
        assert '1모터 평기어 그리퍼' in pending_text
        assert window.status_labels['tool_block'].text() == '도구 준비 중…'
        for _ in range(7):
            clock.advance(0.5)
            window._update_tool_status(_ready_status())
            window._refresh_connection()
        assert '시간 초과' not in window.status_labels['tool_change'].text()
        assert '응답이 없습니다' in window.status_labels['tool_change'].text()
        for key in ('tool_change', 'tool_block'):
            assert window.status_labels[key].isVisible() or window.isHidden()
            assert not re.search('[A-Za-z]', window.status_labels[key].text())
            assert not re.search('[A-Za-z]', window.status_titles[key].text())
    finally:
        window.close()


def test_block_reason_names_the_active_latch():
    _app, window, _clock, _requests = _window()
    try:
        status = _ready_status()
        status['emergency_stop'] = True
        window._update_tool_status(status)
        assert window._tool_block_reason() == '비상 정지 래치'
        status = _ready_status()
        status['tool_detached'] = True
        window._update_tool_status(status)
        assert window._tool_block_reason() == '도구 분리 래치'
        status = _ready_status()
        status['actuators_discovered'] = False
        window._update_tool_status(status)
        assert window._tool_block_reason() == '도구 모터 미감지'
        status['automatic_detection'] = {'present_ids': [5]}
        status['tool_profile'] = {'actuator_ids': [2]}
        window._update_tool_status(status)
        assert window._tool_block_reason() == (
            '선택 도구와 연결 모터 번호가 다름 (감지: 5, 기대: 2)')
    finally:
        window.close()


def test_cleaner_common_buttons_send_on_press_not_release():
    from pathlib import Path

    source = (Path(__file__).parents[1] / 'robot_manual_gui/main_window.py').read_text()
    assert 'self.open_button.pressed.connect' in source
    assert 'self.close_button.pressed.connect' in source
    assert 'self.tool_stop.pressed.connect' in source
    assert 'def _common_pressed' in source


def test_spur_calibration_panel_reuses_existing_calibration_session_commands():
    from pathlib import Path

    source = (Path(__file__).parents[1] / 'robot_manual_gui/main_window.py').read_text()
    assert "그리퍼 끝점 캘리브레이션 · ID 5" in source
    assert "self._capture_spur_endpoint('open')" in source
    assert "self._capture_spur_endpoint('close')" in source
    assert "self._calibration_jog(-0.5)" in source
    assert "self._calibration_jog(-5.0)" in source
    assert "self._calibration_jog(5.0)" in source
    assert "command_calibration('jog_motor_degrees'" in source


def test_gui_rebinds_common_panel_for_dual_spur_cleaner_dual_contexts():
    _app, window, _clock, _requests = _window()
    try:
        for tool, ids, fsm in (
                ('dual_motor_gripper', [3, 4], 'READY'),
                ('spur_1motor_gripper', [5], 'READY'),
                ('cleaner', [2], 'READY'),
                ('dual_motor_gripper', [3, 4], 'READY')):
            window._update_tool_status(_runtime_status(tool, ids, fsm=fsm))
            assert window.node.selected_tool == tool
            assert window.node.actuator_ids == ids
            assert window.profile['actuator_ids'] == ids
            assert window.fsm_state == fsm
            assert not window.common_enable.isHidden()
            assert window.common_enable.isEnabled()
            assert window.tool_status['tool_torque_state'] == 'OFF'
            if tool == 'dual_motor_gripper':
                assert window.dual_start_calibration is not None
                assert window.dual_validate_calibration is not None
                assert window.spur_calibration_box is None
            elif tool == 'spur_1motor_gripper':
                assert window.spur_calibration_box is not None
                assert window.dual_validate_calibration is None
            else:
                assert window.dual_validate_calibration is None
                assert window.spur_calibration_box is None
                assert window.close_button.text().endswith('좌회전')
    finally:
        window.close()


def test_keyboard_shortcuts_can_be_disabled_from_one_flag():
    from PyQt5.QtCore import QEvent, Qt
    from PyQt5.QtGui import QKeyEvent

    _app, window, _clock, _requests = _window()
    commands = []
    window.node.command_tool_fsm = (
        lambda command, **_values: commands.append(command) or True)
    window._update_tool_status(_runtime_status(
        'dual_motor_gripper', [3, 4], torque='ON', motion=True))
    window.node.keyboard_shortcuts_enabled = False
    try:
        window.keyPressEvent(QKeyEvent(
            QEvent.KeyPress, Qt.Key_Right, Qt.NoModifier))
        assert commands == []
    finally:
        window.close()


def test_cleaner_arrow_shortcuts_use_common_button_routing():
    from PyQt5.QtCore import QEvent, Qt
    from PyQt5.QtGui import QKeyEvent

    _app, window, _clock, _requests = _window()
    commands = []
    window.node.command_tool_fsm = (
        lambda command: commands.append(command) or True)
    window._update_tool_status(_runtime_status(
        'cleaner', [2], torque='ON', motion=True))
    try:
        window.keyPressEvent(QKeyEvent(
            QEvent.KeyPress, Qt.Key_Left, Qt.NoModifier))
        window.keyPressEvent(QKeyEvent(
            QEvent.KeyPress, Qt.Key_Right, Qt.NoModifier))
        assert commands == ['LEFT', 'RIGHT']
    finally:
        window.close()


def test_common_dispatcher_uses_reported_tool_and_rejects_stale_generation():
    _app, window, _clock, _requests = _window()
    calls = []
    window.node.command_tool_fsm = (
        lambda command, **values: calls.append((command, values)) or True)
    status = _runtime_status('cleaner', [2], torque='ON', motion=True)
    try:
        window._update_tool_status(status)
        # ComboBox is only a request candidate and cannot redirect commands.
        window.tool_combo.setCurrentIndex(
            window.tool_combo.findData('spur_1motor_gripper'))
        assert window.command_current_tool('OPEN') is False
        assert calls == []
        assert window.command_current_tool('LEFT') is True
        assert calls == [('LEFT', {})]

        context = window._reported_tool_context()
        replacement = dict(status)
        replacement['tool_context_generation'] += 1
        window._update_tool_status(replacement)
        assert window.command_current_tool(
            'RIGHT', expected_context=context) is False
        assert calls == [('LEFT', {})]
    finally:
        window.close()
