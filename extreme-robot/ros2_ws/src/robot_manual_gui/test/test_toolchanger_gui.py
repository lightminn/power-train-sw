"""잠금 패널의 게이트와 발행 페이로드. 표시 경로는 모터를 직접 건드리지 않는다."""

import os
from types import SimpleNamespace

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')


def _window(scope='END_EFFECTOR_ONLY', read_only=False):
    from PyQt5.QtWidgets import QApplication
    from robot_manual_gui.main_window import ManualMainWindow
    from robot_manual_gui.ros_interface import GuiSignals

    app = QApplication.instance() or QApplication([])
    sent = []
    node = SimpleNamespace(
        control_scope=scope, selected_tool='spur_1motor_gripper',
        read_only=read_only, positions={}, efforts={}, gripper_busy=False,
        control_mode='MANUAL',
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
        request_tool_change=lambda tool: True,
        command_toolchanger=lambda command, **values: (
            sent.append((command, values)) or True))
    profile = {'calibrated': True, 'actuator_ids': [5],
               'open_position': 1.0, 'close_position': 0.0}
    window = ManualMainWindow(node, GuiSignals(), profile, False)
    window.control_mode = 'MANUAL'
    return app, window, sent


def _status(**overrides):
    base = {'enabled': True, 'state': 'UNCALIBRATED', 'zero_tick': None,
            'zero_state': 'UNKNOWN', 'position': 2048, 'torque': 0,
            'sequence_state': 'IDLE', 'coupling_proxy': 'UNKNOWN',
            'coupling_peak_load': None, 'error': '',
            'mechanically_confirmed': False}
    base.update(overrides)
    return base


def test_panel_stays_closed_until_a_lock_status_arrives():
    _app, window, _sent = _window()
    for button in (window.lock_zero_locked, window.lock_zero_unlocked,
                   window.lock_enable, window.lock_attach,
                   window.lock_detach, window.lock_stop):
        assert not button.isEnabled()


def test_uncalibrated_parked_lock_allows_only_zero_capture_and_stop():
    _app, window, _sent = _window()
    window._update_toolchanger(_status())
    assert window.lock_zero_locked.isEnabled()
    assert window.lock_zero_unlocked.isEnabled()
    assert window.lock_stop.isEnabled()
    assert not window.lock_enable.isEnabled()
    assert not window.lock_attach.isEnabled()
    assert not window.lock_detach.isEnabled()


def test_zero_set_enables_lock_motor_enable_but_not_a_change():
    _app, window, _sent = _window()
    window._update_toolchanger(_status(zero_tick=2048, zero_state='LOCKED',
                                       state='LOCKED'))
    assert window.lock_enable.isEnabled()
    assert not window.lock_attach.isEnabled()


def test_held_and_indexed_lock_enables_change_and_blocks_zero_capture():
    _app, window, _sent = _window()
    window._update_toolchanger(_status(zero_tick=2048, zero_state='LOCKED',
                                       state='LOCKED', torque=1))
    assert window.lock_attach.isEnabled() and window.lock_detach.isEnabled()
    assert not window.lock_zero_locked.isEnabled()
    assert not window.lock_enable.isEnabled()


def test_full_robot_scope_keeps_stop_but_blocks_every_lock_change():
    _app, window, _sent = _window(scope='FULL_ROBOT')
    window._update_toolchanger(_status(zero_tick=2048, zero_state='LOCKED',
                                       state='LOCKED', torque=1))
    assert window.lock_stop.isEnabled()
    assert not window.lock_attach.isEnabled()
    assert not window.lock_zero_locked.isEnabled()


def test_read_only_gui_disables_every_lock_button_including_stop():
    _app, window, _sent = _window(read_only=True)
    window._update_toolchanger(_status(zero_tick=2048, state='LOCKED', torque=1))
    for button in (window.lock_zero_locked, window.lock_enable,
                   window.lock_attach, window.lock_detach, window.lock_stop):
        assert not button.isEnabled()


def test_change_click_sends_direction_from_the_selector():
    _app, window, sent = _window()
    window._update_toolchanger(_status(zero_tick=2048, zero_state='UNLOCKED',
                                       state='UNLOCKED', torque=1))
    window.lock_attach.click()
    assert sent[-1] == ('CHANGE', {'to': 'LOCKED', 'direction': 1})
    window.lock_direction.setCurrentIndex(1)
    window.lock_detach.click()
    assert sent[-1] == ('CHANGE', {'to': 'UNLOCKED', 'direction': -1})


def test_zero_capture_click_carries_the_physical_state_choice():
    _app, window, sent = _window()
    window._update_toolchanger(_status())
    window.lock_zero_unlocked.click()
    assert sent[-1] == ('SET_ZERO', {'state': 'UNLOCKED'})


def test_panel_always_shows_the_estimate_is_not_a_coupling_sensor():
    from robot_manual_gui.korean_text import ko
    from PyQt5.QtWidgets import QLabel
    _app, window, _sent = _window()
    window._update_toolchanger(_status(state='LOCKED', torque=1,
                                       coupling_proxy='ENGAGED'))
    texts = [label.text() for label in window.findChildren(QLabel)]
    assert ko('Estimated from the zero reference, not a coupling sensor') in texts
