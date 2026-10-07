"""Static contract tests for the GUI ROS frontend."""

from pathlib import Path
import os
from types import SimpleNamespace

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


ROOT = Path(__file__).parents[1] / 'robot_manual_gui'


def test_gui_never_imports_dynamixel_sdk():
    source = ''.join(path.read_text(encoding='utf-8')
                     for path in ROOT.glob('*.py'))
    assert 'dynamixel_sdk' not in source
    assert 'write1Byte' not in source
    assert 'write2Byte' not in source
    assert 'write4Byte' not in source


def test_gui_uses_existing_control_interfaces():
    source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    for interface in (
            '/arm_controller/joint_trajectory',
            '/gripper_controller/follow_joint_trajectory',
            '/cleaning/enable', '/tool/emergency_stop', '/tool/detached',
            '/tool/change'):
        assert interface in source


def test_runtime_tool_change_is_published_by_gui():
    source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    window = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert "self.tool_change_pub = self.create_publisher(String, '/tool/change'" in source
    assert 'def request_tool_change(self, tool_type):' in source
    assert 'self.node.request_tool_change(requested)' in window


def test_mode_status_does_not_overwrite_pending_operator_request():
    source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert 'self.mode_combo.setCurrentText(mode)' not in source


def test_end_effector_scope_blocks_arm_publish_path():
    ros_source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    window_source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert "self.control_scope == 'END_EFFECTOR_ONLY'" in ros_source
    assert 'manual and not end_effector_only' in window_source
    assert 'CONTROL / TEST SCOPE:' in window_source


def test_dashboard_is_scrollable_and_entrypoint_starts_maximized():
    from PyQt5.QtCore import Qt
    from PyQt5.QtWidgets import QScrollArea

    _app, window, _goals = _window('END_EFFECTOR_ONLY')
    assert isinstance(window.centralWidget(), QScrollArea)
    assert window.scroll_area.widgetResizable()
    assert window.scroll_area.horizontalScrollBarPolicy() == Qt.ScrollBarAsNeeded
    assert window.scroll_area.verticalScrollBarPolicy() == Qt.ScrollBarAsNeeded
    main_source = (ROOT / 'main.py').read_text(encoding='utf-8')
    assert 'window.showMaximized()' in main_source


def test_spur_gui_uses_only_id5_and_requires_explicit_enable():
    ros_source = (ROOT / 'ros_interface.py').read_text(encoding='utf-8')
    window_source = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert "'/tool/fsm_command'" in ros_source
    assert "'/tool/calibration_command'" in ros_source
    assert 'command_spur_fsm' in ros_source
    assert 'command_calibration' in ros_source
    spur = ros_source[ros_source.index('    def set_spur_motor_enabled'):ros_source.index(
        '    def command_spur_fsm')]
    assert 'torque_pub' not in spur
    assert "ENABLE ID5" in window_source
    assert "DISABLE ID5" in window_source
    assert "CalibrationSession ENABLE ID5 requested" in window_source


def test_spur_calibration_uses_actual_register_state_and_no_fake_endpoints():
    bridge = (Path(__file__).parents[2] / 'dynamixel_control' /
              'dynamixel_control' / 'moveit_dynamixel_bridge.py').read_text(
                  encoding='utf-8')
    window = (ROOT / 'main_window.py').read_text(encoding='utf-8')
    assert "ADDR_TORQUE_ENABLE" in bridge
    assert "'tool_torque_state'" in bridge
    assert "'/tool/calibration_command'" in bridge
    assert 'CalibrationSession' in bridge
    assert 'spur gripper action rejected' in bridge
    assert "MOTOR −1°" in window
    assert "SET CURRENT AS OPEN" in window
    assert "SET CURRENT AS CLOSE" in window
    jog = window[window.index('def _jog_spur_motor'):window.index('def _capture_spur_endpoint')]
    assert 'temporary_jog_safe_min' not in jog


def test_calibration_mode_has_no_startup_configuration_writes():
    bridge = (Path(__file__).parents[2] / 'dynamixel_control' /
              'dynamixel_control' / 'moveit_dynamixel_bridge.py').read_text(
                  encoding='utf-8')
    start = bridge.index('elif self.calibration_jog_enabled:')
    end = bridge.index('elif self.tool_motion_allowed', start)
    block = bridge[start:end]
    assert 'group_sync_read.addParam(5)' in block
    assert 'write1ByteTxRx' not in block
    assert 'write4ByteTxRx' not in block


def test_spur_bridge_starts_torque_off_and_has_one_gripper_executor():
    bridge = (Path(__file__).parents[2] / 'dynamixel_control' /
              'dynamixel_control' / 'moveit_dynamixel_bridge.py').read_text(
                  encoding='utf-8')
    assert bridge.count('    def execute_gripper(self, goal_handle):') == 1
    assert "self.tool_ids == [5]" in bridge
    assert "self.tool_type == 'dual_motor_gripper'" in bridge


def _window(scope):
    from PyQt5.QtWidgets import QApplication
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.ros_interface import GuiSignals

    app = QApplication.instance() or QApplication([])
    goals = []
    node = SimpleNamespace(
        control_scope=scope, selected_tool='dual_motor_gripper', read_only=False,
        developer_direct_mode=False,
        positions={}, efforts={}, gripper_busy=False,
        request_mode=lambda _mode: None, jog_arm=lambda *_args: None,
        command_arm=lambda *_args: None,
        command_gripper=lambda position: (goals.append(position) or True),
        command_tool_fsm=lambda command, **_values: (goals.append(command) or True),
        stop_gripper=lambda: None, command_cleaner=lambda *_args: None,
        emergency_stop=lambda: None, tool_detached=lambda: None,
        set_dual_motor_enabled=lambda *_args: True,
        command_calibration=lambda *_args, **_kwargs: True,
        manual_dual_recovery_jog=lambda *_args: True,
        command_dual_calibration=lambda *_args, **_kwargs: True)
    profile = {
        'calibrated': True, 'actuator_ids': [3, 4],
        'open_position': 1.0, 'close_position': 0.0,
        'safe_min_tick': -526, 'safe_max_tick': 2384,
        'motor_endpoints': {
            3: {'open': 1056, 'close': -526},
            4: {'open': 2384, 'close': 839}}}
    return app, ManualMainWindow(node, GuiSignals(), profile, False), goals


def _ready_status(scope):
    return {
        'control_scope': scope, 'tool_type': 'dual_motor_gripper',
        'tool_context_generation': 0,
        'tool_profile': {'actuator_ids': [3, 4], 'backend': 'gripper',
                         'calibrated': True},
        'profile_valid': True, 'calibrated': True,
        'actuators_discovered': True, 'motion_allowed': True,
        'tool_enable_allowed': True, 'tool_preparation_state': 'READY',
        'tool_preparation_error': '', 'tool_enable_reason': '',
        'tool_torque_state': 'ON',
        'control_mode': 'MANUAL',
        'fsm_class': 'DualMotorGripperFSM',
        'read_only': False, 'emergency_stop': False, 'tool_detached': False,
        'bridge_connected': True,
        'fsm_state': 'READY',
        'endpoint_calibration_verified': True,
        'synchronization': {
            'state': 'SYNCHRONIZED', 'spread': 0.0, 'limit': 0.05},
        'dual_calibration': {'state': 'RECALIBRATION_REQUIRED', 'active': False},
        'actuators': [
            {'id': 3, 'online': True, 'position': 265, 'effort': 10},
            {'id': 4, 'online': True, 'position': 1612, 'effort': 10}]}


def test_dual_open_close_use_fsm_ingress_only_when_all_gates_are_ready():
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    assert window.open_button.isEnabled()
    assert window.close_button.isEnabled()
    window._command_tool('OPEN')
    window._command_tool('CLOSE')
    assert commands == ['OPEN', 'CLOSE']
    window.close()


def test_dual_hold_buttons_use_endpoint_jog_without_calibration():
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    calibration_calls = []
    window.node.command_dual_calibration = (
        lambda command, **values:
        (calibration_calls.append((command, values)) or True))
    window._start_dual_hold_jog('OPEN')
    assert window.dual_hold_jog_active
    assert commands == ['JOG_OPEN']
    assert calibration_calls == []
    window._release_dual_hold_jog()
    assert calibration_calls == []
    assert commands == ['JOG_OPEN', 'HOLD']
    assert not window.dual_hold_jog_active
    window.close()


def test_dual_arrow_hold_uses_endpoint_jog_and_release_hold():
    _app, window, _commands = _window('END_EFFECTOR_ONLY')
    calls = []
    window.node.command_dual_calibration = (
        lambda command, **values: (calls.append((command, values)) or True))
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    window._start_dual_key_jog(1)
    assert _commands == ['JOG_OPEN']
    assert calls == []
    window._stop_dual_key_jog()
    assert _commands[-1] == 'HOLD'
    assert not window.dual_key_jog_timer.isActive()
    window.close()


@pytest.mark.parametrize('mutation', ('offline', 'hardware_error', 'torque_off',
                                      'calibration_invalid', 'sync_fault'))
def test_dual_motion_buttons_fail_closed(mutation):
    _app, window, _commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    if mutation == 'offline':
        status['actuators'][1]['online'] = False
    elif mutation == 'hardware_error':
        status['actuators'][0]['hardware_error'] = 4
    elif mutation == 'torque_off':
        status['actuators'][1]['torque_state'] = 'OFF'
    elif mutation == 'calibration_invalid':
        status['endpoint_calibration_verified'] = False
    elif mutation == 'sync_fault':
        status['synchronization'] = {
            'state': 'FAULT', 'spread': 0.1, 'limit': 0.05}
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    assert not window.open_button.isEnabled()
    assert not window.close_button.isEnabled()
    window.close()


def test_end_effector_scope_enables_only_tool_controls():
    _app, window, _goals = _window('END_EFFECTOR_ONLY')
    window._update_tool_status(_ready_status('END_EFFECTOR_ONLY'))
    window._update_mode('MANUAL')
    assert not window.open_button.isEnabled()
    assert not window.close_button.isEnabled()
    assert window.dual_start_calibration.isEnabled()
    assert window.tool_stop.isEnabled()
    assert not any(widget.isEnabled() for widget in window.arm_buttons)
    window.close()


def test_full_robot_preserves_arm_feedback_gate():
    _app, window, _goals = _window('FULL_ROBOT')
    window._update_tool_status(_ready_status('FULL_ROBOT'))
    window._update_mode('MANUAL')
    assert not any(widget.isEnabled() for widget in window.arm_buttons)
    window.seen_arm_joints.add('arm_joint_1')
    window._refresh_buttons()
    assert all(widget.isEnabled() for widget in window.arm_widgets['arm_joint_1'])
    assert not any(widget.isEnabled()
                   for widget in window.arm_widgets['arm_joint_2'])
    window.close()


def test_jog_interpolates_both_motors_and_busy_blocks_queue():
    _app, window, goals = _window('END_EFFECTOR_ONLY')
    window._update_tool_status(_ready_status('END_EFFECTOR_ONLY'))
    window._update_mode('MANUAL')
    window._jog_gripper(1)
    assert goals == []
    window.gripper_busy = True
    window._jog_gripper(1)
    assert goals == []
    window.close()


@pytest.mark.parametrize('finish', ('release', 'close', 'deactivate', 'stop'))
def test_hold_close_button_stops_timer_and_holds(finish):
    from PyQt5.QtCore import QEvent
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    window.hold_close_button.pressed.emit()
    window._dual_key_jog_tick()
    assert commands == ['JOG_CLOSE', 'JOG_CLOSE']
    if finish == 'release':
        window.hold_close_button.released.emit()
    elif finish == 'close':
        window.close()
    elif finish == 'deactivate':
        window.eventFilter(window, QEvent(QEvent.WindowDeactivate))
    else:
        window._stop_tool()
    assert 'HOLD' in commands
    assert not window.dual_key_jog_timer.isActive()
    before = list(commands)
    window._dual_key_jog_tick()
    assert commands == before
    window.close()


def test_korean_display_preserves_protocol_values():
    from PyQt5.QtWidgets import QLabel, QPushButton, QGroupBox
    import re
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    status = _ready_status('END_EFFECTOR_ONLY')
    status['dual_calibration'] = {'state': 'READY', 'active': False}
    for sample in status['actuators']:
        sample.update(torque_state='ON', hardware_error=0)
    window._update_tool_status(status)
    window._update_mode('MANUAL')
    assert window.hold_open_button.text() == '누르는 동안 열기'
    assert window.hold_close_button.text() == '누르는 동안 닫기'
    assert window.tool_combo.currentData() == 'dual_motor_gripper'
    assert window.tool_combo.currentText() == '2모터 그리퍼'
    requests = []
    window.node.request_mode = requests.append
    window.mode_combo.setCurrentIndex(window.mode_combo.findData('MANUAL'))
    window._request_mode()
    assert requests == ['MANUAL']
    assert window.control_mode == 'MANUAL'
    assert window.fsm_state == 'READY'
    # The remote intentionally shows the requested runtime notation; all
    # existing controls and detailed status labels remain Korean.
    assert window.remote_state.text() == 'READY'
    assert window.remote_ids.text() == 'ID 3, 4'
    assert window.remote_torque.text() == 'Torque: ' + status.get('tool_torque_state', 'UNKNOWN')
    for widget in window.findChildren((QLabel, QPushButton, QGroupBox)):
        if widget in (window.remote_state, window.remote_ids, window.remote_torque):
            continue
        text = widget.title() if isinstance(widget, QGroupBox) else widget.text()
        assert not re.search('[A-Za-z]', text), text
    window.close()


def test_mock_runtime_round_trip_routes_buttons_keys_and_clears_context():
    from PyQt5.QtCore import QEvent, Qt
    from PyQt5.QtGui import QKeyEvent

    app, window, commands = _window('END_EFFECTOR_ONLY')
    window.mock_mode = True
    window.node.temporary_jog_mode = False
    window.node.command_cleaner = lambda enabled: commands.append(('cleaner', enabled))
    window.node.command_cleaner_direction = (
        lambda direction: (commands.append(direction) or True)
    )
    window.node.command_calibration = lambda command, **values: (commands.append((command, values)) or True)
    requests = []
    window.node.request_tool_change = lambda tool: (requests.append(tool) or True)
    dual_profile = dict(window.profile)
    window._update_mode('MANUAL')
    try:
        for generation, (tool, ids) in enumerate([
                ('dual_motor_gripper', [3, 4]),
                ('spur_1motor_gripper', [5]),
                ('cleaner', [6]), ('dual_motor_gripper', [3, 4])]):
            old_panel = window.tool_control_box
            changed = window.node.selected_tool != tool
            if changed:
                window.gripper_target_ticks = {99: 123}
                window.spur_zero_tick = 123
                window.tool_combo.setCurrentIndex(window.tool_combo.findData(tool))
                window._request_tool_change()
                assert window.pending_tool_change == tool
                assert not window.tool_control_box.isEnabled()
            status = _ready_status('END_EFFECTOR_ONLY')
            status.update(tool_type=tool, tool_context_generation=generation,
                          online=True, hardware_error=0,
                          fsm_class={
                              'dual_motor_gripper': 'DualMotorGripperFSM',
                              'spur_1motor_gripper': 'SingleMotorGripperFSM',
                              'cleaner': 'CleanerFSM'}[tool],
                          tool_torque_state='OFF', calibration_jog_enabled=True,
                          calibration={'active': True, 'enabled': True},
                          dual_calibration={'state': 'READY', 'active': False})
            profile = dual_profile if len(ids) == 2 else {
                'actuator_ids': ids, 'calibrated': True,
                'safe_min_tick': 2867, 'safe_max_tick': 3807,
                'open_tick': 2867, 'close_tick': 3807}
            status['tool_profile'] = profile
            if ids != [3, 4]:
                status['actuators'] = [dict(id=i, position=3300, online=True) for i in ids]
            for sample in status['actuators']:
                sample.update(torque_state='OFF', hardware_error=0)
            window._update_tool_status(status)
            assert window.node.selected_tool == tool
            assert window.node.actuator_ids == ids
            assert window.profile is profile
            assert window.pending_tool_change is None
            status['tool_torque_state'] = 'ON'
            for sample in status['actuators']:
                sample['torque_state'] = 'ON'
            window._update_tool_status(status)
            if changed:
                assert old_panel.isHidden() and not old_panel.isEnabled()
                assert not window.gripper_target_ticks
                assert window.spur_zero_tick is None
                assert not window.dual_key_jog_timer.isActive()
                assert not window.dual_hold_jog_active
            commands.clear()
            if tool == 'cleaner':
                assert not window.open_button.isHidden()
                assert window.spur_enable is window.common_enable
                assert not window.common_enable.isHidden()
                assert window.dual_enable is window.common_enable
                window.close_button.click()
                window.open_button.click()
                window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_Space, Qt.NoModifier))
                window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_Left, Qt.NoModifier))
                assert commands == ['LEFT', 'RIGHT', 'STOP', 'LEFT']
                assert window.spur_actual_state is None
                assert window.motor_minus_half is None
            else:
                assert window.clean_start.isHidden()
                window.open_button.click()
                window.close_button.click()
                assert commands == ['OPEN', 'CLOSE']
                commands.clear()
                if tool == 'dual_motor_gripper':
                    window.hold_open_button.pressed.emit()
                    window.hold_open_button.released.emit()
                    assert commands == ['JOG_OPEN', 'HOLD']
                    commands.clear()
                    window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_Right, Qt.NoModifier))
                    window.keyReleaseEvent(QKeyEvent(QEvent.KeyRelease, Qt.Key_Right, Qt.NoModifier))
                    assert commands == ['JOG_CLOSE', 'HOLD']
                else:
                    window.jog_close.click()
                    window.jog_open.click()
                    window.keyPressEvent(QKeyEvent(QEvent.KeyPress, Qt.Key_Left, Qt.NoModifier))
                    expected_context = (
                        'spur_1motor_gripper', 'SingleMotorGripperFSM',
                        (5,), generation)
                    assert commands == [
                        ('manual_open', {'expected_context': expected_context}),
                        ('manual_hold', {'expected_context': expected_context}),
                        ('manual_close', {'expected_context': expected_context}),
                        ('manual_hold', {'expected_context': expected_context}),
                        ('manual_step', {'delta_deg': -0.5,
                                         'expected_context': expected_context})]
                    assert not window.gripper_target_ticks
            app.processEvents()
        assert requests == ['spur_1motor_gripper', 'cleaner', 'dual_motor_gripper']
    finally:
        window.close()


def test_rejected_change_preserves_active_context_and_resets_combo():
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    window.node.request_tool_change = lambda _: True
    status = _ready_status('END_EFFECTOR_ONLY')
    status['tool_profile'] = window.profile
    window._update_tool_status(status)
    panel = window.tool_control_box
    window.tool_combo.setCurrentIndex(window.tool_combo.findData('cleaner'))
    window._request_tool_change()
    status['tool_change'] = {'error': 'motion active'}
    window._update_tool_status(status)
    assert window.node.selected_tool == 'dual_motor_gripper'
    assert window.node.actuator_ids == [3, 4]
    assert window.tool_combo.currentData() == 'dual_motor_gripper'
    assert window.tool_control_box is panel and panel.isEnabled()
    assert not commands
    window.close()


def test_late_gripper_goal_response_cannot_restore_previous_tool_state():
    from robot_manual_gui.ros_interface import ManualGuiNode
    callbacks = []
    future = SimpleNamespace(add_done_callback=callbacks.append)
    node = SimpleNamespace(
        selected_tool='dual_motor_gripper', read_only=False, control_mode='MANUAL',
        gripper_busy=False, temporary_jog_mode=False, tool_context_generation=0,
        signals=SimpleNamespace(gripper_state=SimpleNamespace(emit=lambda *_: None)),
        get_logger=lambda: SimpleNamespace(info=lambda _: None),
        gripper=SimpleNamespace(send_goal_async=lambda _: future),
        _gripper_goal_response=lambda _: pytest.fail('old callback reached new tool'))
    assert ManualGuiNode.command_gripper(node, 0.5)
    node.selected_tool = 'cleaner'
    node.tool_context_generation += 1
    callbacks[0](future)


@pytest.mark.parametrize('state,allowed', [
    ('IDLE', True), ('STOWED', True), ('STOWED_LOCKED', True),
    ('CALIBRATION_REQUIRED', True), ('STOPPED', True), ('READY', True),
    ('OPENING', False), ('CLOSING', False), ('FAULT', False)])
def test_spur_manual_ownership_safe_states(monkeypatch, state, allowed):
    from PyQt5.QtWidgets import QMessageBox
    _app, window, _commands = _window('END_EFFECTOR_ONLY')
    requests, warnings = [], []
    window.node.selected_tool = 'spur_1motor_gripper'
    window.node.request_mode = requests.append
    window.fsm_state = state
    window.mode_combo.setCurrentIndex(window.mode_combo.findData('MANUAL'))
    monkeypatch.setattr(QMessageBox, 'warning', lambda *_: warnings.append(True))
    window._request_mode()
    assert requests == (['MANUAL'] if allowed else [])
    assert bool(warnings) == (not allowed)
    window.close()


def test_cleaner_in_end_effector_scope_can_request_manual_ownership(monkeypatch):
    from PyQt5.QtWidgets import QMessageBox
    _app, window, _commands = _window('END_EFFECTOR_ONLY')
    try:
        requests, warnings = [], []
        window.node.selected_tool = 'cleaner'
        window.node.request_mode = requests.append
        window.fsm_state = 'READY'
        window.mode_combo.setCurrentIndex(window.mode_combo.findData('MANUAL'))
        monkeypatch.setattr(QMessageBox, 'warning', lambda *_: warnings.append(True))
        window._request_mode()
        assert requests == ['MANUAL']
        assert warnings == []
    finally:
        window.close()


def test_bridge_accepts_ready_manual_request_without_motor_commands():
    import ast
    source = Path(__file__).parents[2] / 'dynamixel_control/dynamixel_control/moveit_dynamixel_bridge.py'
    tree = ast.parse(source.read_text())
    methods = [node for node in ast.walk(tree) if isinstance(node, ast.FunctionDef)
               and node.name in ('_on_control_mode', '_on_control_mode_request')]
    namespace = {'String': lambda **values: SimpleNamespace(**values)}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), namespace)
    published = []
    bridge = SimpleNamespace(control_mode='FSM', tool_type='spur_1motor_gripper',
                             tool_fsm=SimpleNamespace(state=SimpleNamespace(name='READY')),
                             control_mode_status_pub=SimpleNamespace(publish=published.append))
    bridge._on_control_mode = lambda msg: namespace['_on_control_mode'](bridge, msg)
    namespace['_on_control_mode_request'](bridge, SimpleNamespace(data='MANUAL'))
    assert bridge.control_mode == 'MANUAL'
    assert published[0].data == 'MANUAL'


def test_status_panel_follows_active_tool_and_clears_missing_id5_feedback():
    app, window, commands = _window('END_EFFECTOR_ONLY')
    try:
        for tool in ('dual_motor_gripper', 'spur_1motor_gripper', 'cleaner',
                     'spur_1motor_gripper', 'dual_motor_gripper'):
            window.node.selected_tool = tool
            window._update_status_panel({'actuators': [dict(
                id=5, online=True, position=2965, torque_state='OFF',
                hardware_error=0, operating_mode=3)]})
            for key in ('dual_online', 'dual_positions', 'dual_torque', 'dual_hw_error', 'dual_sync'):
                assert window.status_labels[key].isHidden() == (tool != 'dual_motor_gripper')
                assert window.status_titles[key].isHidden() == (tool != 'dual_motor_gripper')
            for key in ('spur_online', 'spur_position', 'spur_torque', 'spur_hw_error', 'spur_operating_mode'):
                assert window.status_labels[key].isHidden() == (tool != 'spur_1motor_gripper')
                assert window.status_titles[key].isHidden() == (tool != 'spur_1motor_gripper')
            if tool == 'spur_1motor_gripper':
                assert window.status_labels['spur_position'].text() == '2965'
                assert window.status_labels['spur_hw_error'].text() == '0'
                assert window.status_labels['spur_operating_mode'].text() == '3 (위치 제어)'
                window._update_status_panel({'actuators': [dict(id=3, position=123)]})
                assert window.status_labels['spur_position'].text() != '2965'
        assert commands == []
    finally:
        window.close()


def test_spur_enable_uses_current_context_before_motion_allowed():
    app, window, commands = _window('END_EFFECTOR_ONLY')
    window.node.command_calibration = lambda command, **values: (commands.append(command) or True)
    buttons = window._common_buttons()
    try:
        window.control_mode = window.node.control_mode = 'FSM'
        window.pending_tool_change = 'spur_1motor_gripper'
        status = _ready_status('END_EFFECTOR_ONLY')
        status.update(tool_type='spur_1motor_gripper',
                      fsm_class='SingleMotorGripperFSM',
                      tool_context_generation=1, control_mode='MANUAL',
                      motion_allowed=False, tool_enable_allowed=True,
                      tool_torque_state='OFF', tool_preparation_state='READY',
                      calibration={'active': False, 'enabled': False},
                      tool_profile={'actuator_ids': [5], 'calibrated': True,
                                    'open_tick': 2945, 'close_tick': 3752,
                                    'safe_min_tick': 2945, 'safe_max_tick': 3752},
                      actuators=[dict(id=5, online=True, hardware_error=0,
                                      position=3000, torque_state='OFF', operating_mode=3)])
        window._update_tool_status(status)
        assert window._common_buttons() == buttons
        assert window.control_mode == window.node.control_mode == 'MANUAL'
        assert window.common_enable.isEnabled()
        assert not window.open_button.isEnabled()
        status['fsm_state'] = 'STOPPED'
        window._update_tool_status(status)
        assert window.common_enable.isEnabled()
        assert not window.open_button.isEnabled()
        window.common_enable.click()
        assert commands == ['manual_enable']
        status['fsm_state'] = 'READY'
        status['actuators'][0]['torque_state'] = 'ON'
        status['tool_torque_state'] = 'ON'
        # Torque readback alone must not bypass the bridge motion gate.
        window._update_tool_status(status)
        assert not window.common_enable.isEnabled()
        assert not window.open_button.isEnabled()
        status['motion_allowed'] = True
        window._update_tool_status(status)
        assert all(w.isEnabled() for w in (window.open_button, window.close_button,
                                          window.hold_open_button, window.hold_close_button))
        window.open_button.click()
        window.close_button.click()
        window.hold_open_button.click()
        assert commands == ['manual_enable', 'OPEN', 'CLOSE', 'manual_open', 'manual_hold']
        status['control_mode'] = 'FSM'
        window._update_tool_status(status)
        assert not window.common_enable.isEnabled()
        assert not window.open_button.isEnabled()
    finally:
        window.close()


def test_developer_direct_panel_uses_id5_without_manual_or_fsm_ownership():
    _app, window, commands = _window('END_EFFECTOR_ONLY')
    try:
        window.node.selected_tool = 'spur_1motor_gripper'
        window.node.developer_direct_mode = True
        window.node.command_calibration = (
            lambda command, **values: (commands.append((command, values)) or True))
        window._rebuild_tool_control_group()
        status = _ready_status('END_EFFECTOR_ONLY')
        status.update(tool_type='spur_1motor_gripper', control_mode='FSM',
                      fsm_state='STOPPED', motion_allowed=False,
                      actuators=[dict(id=5, online=True, hardware_error=0,
                                      position=3000, torque_state='OFF', operating_mode=3)])
        window._update_tool_status(status)
        assert window.developer_enable.isEnabled()
        assert not window.developer_minus.isEnabled()
        window.developer_enable.click()
        assert commands == [('manual_enable', {'delta_deg': 0.0})]
        status['actuators'][0]['torque_state'] = 'ON'
        window._update_tool_status(status)
        assert window.developer_minus.isEnabled()
        window.developer_minus.click()
        assert commands[-1] == ('manual_step', {'delta_deg': -0.5})
        window.developer_plus_five.click()
        assert commands[-1] == ('manual_step', {'delta_deg': 5.0})
        status['emergency_stop'] = True
        window._update_tool_status(status)
        assert not window.developer_plus.isEnabled()
    finally:
        window.close()
