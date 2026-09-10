"""Pure safety tests for the dual-gripper manual recovery path."""

import pytest

from dynamixel_control.dual_manual_recovery import (
    DualManualRecovery, DualManualRecoveryError)


class Bridge:
    def __init__(self):
        self.position = {3: -500, 4: 2400}
        self.torque = {3: 1, 4: 1}
        self.error = {3: 0, 4: 0}
        self.writes = []

    def read_manual_position(self, dxl_id): return self.position[dxl_id]
    def read_manual_torque(self, dxl_id): return self.torque[dxl_id]
    def read_manual_hardware_error(self, dxl_id): return self.error[dxl_id]
    def manual_goal_position(self, dxl_id, tick): self.writes.append((dxl_id, tick))


def test_each_click_is_one_bounded_id3_or_id4_write():
    bridge = Bridge()
    recovery = DualManualRecovery(bridge)
    assert recovery.jog(3, 0.5) == -494
    assert bridge.writes == [(3, -494)]
    assert recovery.jog(4, -0.5) == 2394
    assert bridge.writes[-1] == (4, 2394)
    assert all(dxl_id in (3, 4) for dxl_id, _tick in bridge.writes)


def test_calibration_jog_can_use_fresh_relative_goal_without_old_endpoint_gate():
    bridge = Bridge()
    writes = []
    target = DualManualRecovery(bridge).jog(
        4, 5.0, allowed_degrees=(-5.0, -2.0, -1.0, -0.5,
                                   0.5, 1.0, 2.0, 5.0),
        goal_writer=lambda dxl_id, tick: writes.append((dxl_id, tick)))
    assert target == 2457
    assert writes == [(4, 2457)]
    assert bridge.writes == []


@pytest.mark.parametrize('field,value', [('torque', 0), ('error', 4)])
def test_torque_off_or_hardware_error_blocks_without_write(field, value):
    bridge = Bridge()
    getattr(bridge, field)[3] = value
    with pytest.raises(DualManualRecoveryError):
        DualManualRecovery(bridge).jog(3, 0.5)
    assert bridge.writes == []


def test_unreadable_present_position_treats_motor_as_offline_and_blocks():
    bridge = Bridge()
    bridge.position[4] = None
    with pytest.raises(DualManualRecoveryError, match='position unavailable'):
        DualManualRecovery(bridge).jog(4, -0.5)
    assert bridge.writes == []


@pytest.mark.parametrize('dxl_id,delta', [(2, 0.5), (5, -0.5), (3, 1.0)])
def test_non_dual_or_unbounded_delta_is_rejected(dxl_id, delta):
    bridge = Bridge()
    with pytest.raises(DualManualRecoveryError):
        DualManualRecovery(bridge).jog(dxl_id, delta)
    assert bridge.writes == []


def test_bridge_callback_keeps_spread_guard_for_open_close_but_not_recovery():
    from pathlib import Path
    source = (Path(__file__).parents[1] / 'dynamixel_control' /
              'moveit_dynamixel_bridge.py').read_text(encoding='utf-8')
    assert 'manual dual recovery jog rejected by safety gate' in source
    goal_callback = source[source.index('    def gripper_goal_callback'):source.index(
        '    def gripper_cancel_callback')]
    assert "if self.tool_type == 'dual_motor_gripper':" in goal_callback
    assert 'normalized spread {spread:.4f} > 0.0500' in goal_callback
    execute = source[source.index('    def _execute_gripper'):source.index(
        '    def _read_tool_state')]
    assert 'self._dual_normalized_spread(' in execute
    window = (Path(__file__).parents[2] / 'robot_manual_gui' /
              'robot_manual_gui' / 'main_window.py').read_text(encoding='utf-8')
    assert 'max(fractions.values()) - min(fractions.values()) <= 0.05' in window
    recovery = window[window.index('def _manual_dual_recovery_jog'):]
    assert '_gripper_positions_synchronized' not in recovery.split('def ', 1)[0]


def test_manual_recovery_ingress_has_all_safety_gates_and_single_id_write():
    from pathlib import Path
    source = (Path(__file__).parents[1] / 'dynamixel_control' /
              'moveit_dynamixel_bridge.py').read_text(encoding='utf-8')
    callback = source[source.index('    def manual_recovery_callback'):source.index(
        '    def calibration_command_callback')]
    for required in (
            "self.tool_type != 'dual_motor_gripper'",
            'self.tool_ids != [3, 4]',
            "self.control_scope != 'END_EFFECTOR_ONLY'",
            "self.control_mode != 'MANUAL'",
            'self.read_only', 'self.emergency_stop_active', 'self.tool_detached'):
        assert required in callback
    write = source[source.index('    def manual_goal_position'):source.index(
        '    def _read_tool_state')]
    assert 'ADDR_GOAL_POSITION' in write
    assert 'self._manual_recovery_id_allowed(dxl_id)' in write
    assert 'GroupSyncWrite' not in write


def test_dual_profile_setup_is_explicit_enable_only_and_id_restricted():
    from pathlib import Path
    source = (Path(__file__).parents[1] / 'dynamixel_control' /
              'moveit_dynamixel_bridge.py').read_text(encoding='utf-8')
    setup = source[source.index('    def _prepare_dual_gripper_enable'):source.index(
        '    def fsm_command_callback')]
    assert "self.tool_type != 'dual_motor_gripper'" in setup
    assert "self.tool_ids != [3, 4]" in setup
    assert "actual torque must be OFF before profile setup" in setup
    assert 'profile acceleration' in setup
    assert 'profile velocity' in setup
    assert 'goal PWM' in setup


def test_estop_blocks_torque_enable_before_any_tool_write():
    from pathlib import Path
    source = (Path(__file__).parents[1] / 'dynamixel_control' /
              'moveit_dynamixel_bridge.py').read_text(encoding='utf-8')
    callback = source[source.index('    def torque_request_callback'):source.index(
        '    def _prepare_dual_gripper_enable')]
    assert 'self.emergency_stop_active or self.tool_detached' in callback
    assert 'torque enable rejected' in callback


def test_gui_recovery_buttons_are_single_click_only_and_display_actual_state():
    from pathlib import Path
    window = (Path(__file__).parents[2] / 'robot_manual_gui' /
              'robot_manual_gui' / 'main_window.py').read_text(encoding='utf-8')
    assert 'MANUAL DUAL MOTOR RECOVERY (one click only)' in window
    assert 'button.setAutoRepeat(False)' in window
    assert 'actual torque=' in window
    assert 'hardware error=' in window
    assert 'CALIBRATION JOG: spread protection bypassed' in window


def test_bridge_keeps_normal_spread_gate_but_calibration_goal_has_no_endpoint_gate():
    from pathlib import Path
    source = (Path(__file__).parents[1] / 'dynamixel_control' /
              'moveit_dynamixel_bridge.py').read_text(encoding='utf-8')
    normal = source[source.index('    def gripper_goal_callback'):source.index(
        '    def gripper_cancel_callback')]
    assert 'normalized spread {spread:.4f} > 0.0500' in normal
    calibration = source[source.index('    def dual_calibration_goal_position'):source.index(
        '    def _check_gripper_in_calibrated_range')]
    assert 'motor_endpoints' not in calibration
    assert 'ADDR_GOAL_POSITION' in calibration
