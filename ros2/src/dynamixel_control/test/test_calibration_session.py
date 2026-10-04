"""Pure calibration-session tests; no ROS, serial port, or SDK."""

import pytest
from pathlib import Path

from dynamixel_control.calibration_session import (
    CalibrationSession, CalibrationSessionError)
from dynamixel_control.tool_profiles import load_profiles
from dynamixel_control.tool_fsm.registry import create_tool_fsm
from dynamixel_control.tool_fsm.base import ToolState


class Bridge:
    def __init__(self, position=2000):
        self.position = position
        self.torque = 0
        self.hardware_error = 0
        self.allowlist = []
        self.writes = []

    def set_allowlist(self, ids): self.allowlist = list(ids)
    def read_position(self, dxl_id):
        assert dxl_id == 5
        return self.position
    def read_torque(self, dxl_id): assert dxl_id == 5; return self.torque
    def read_hardware_error(self, dxl_id): assert dxl_id == 5; return self.hardware_error
    def read_model(self, dxl_id): assert dxl_id == 5; return 1060
    def set_torque(self, dxl_id, enabled):
        self.torque = int(enabled)
        self.writes.append(('torque', dxl_id, enabled))
    def goal_position(self, dxl_id, tick): self.writes.append(('goal', dxl_id, tick))


def profile():
    # Synthetic metadata for save/reload only; it is never the real YAML.
    return {'backend': 'gripper', 'calibrated': False, 'actuator_ids': [5],
            'direction': 1, 'profile_velocity': 1, 'profile_acceleration': 1,
            'no_load_effort': 1, 'grasp_effort': 3, 'grasp_threshold': 2,
            'release_drop_threshold': 2, 'action_time': 1.0,
            'required_operating_modes': {5: 3}}


def test_start_and_capture_are_read_only():
    bridge = Bridge()
    session = CalibrationSession(bridge, profile())
    assert bridge.allowlist == [5]
    session.start()
    assert session.capture_open() == 2000
    bridge.position = 2100
    assert session.capture_close() == 2100
    assert bridge.writes == []


def test_only_one_click_jogs_write_one_id5_goal():
    bridge = Bridge()
    session = CalibrationSession(bridge, profile())
    session.start()
    session.enable()
    assert session.jog_motor_degrees(0.5) == 2006
    assert session.jog_motor_degrees(-1.0) == 1989
    assert session.jog_motor_degrees(5.0) == 2057
    assert bridge.writes == [('torque', 5, True), ('goal', 5, 2006),
                             ('goal', 5, 1989), ('goal', 5, 2057)]
    for delta in (0, 0.4, 1.5, -2, 6):
        with pytest.raises(CalibrationSessionError):
            session.jog_motor_degrees(delta)
    assert all(write[1] == 5 for write in bridge.writes)


def test_save_and_reload_use_only_explicit_temporary_path(tmp_path):
    bridge = Bridge(100)
    session = CalibrationSession(bridge, profile())
    session.start()
    session.capture_open()
    bridge.position = 600
    session.capture_close()
    assert session.validate()['motor_model'] == 1060
    output = tmp_path / 'mock_profiles.yaml'
    output.write_text('tool_profiles:\n  spur_1motor_gripper: {}\n', encoding='utf-8')
    session.save(output)
    reloaded = load_profiles(output)['spur_1motor_gripper']
    assert reloaded['calibrated'] is True
    assert reloaded['open_tick'] == 100
    assert reloaded['close_tick'] == 600
    assert reloaded['motor_model'] == 1060


def test_offline_or_hardware_error_blocks_calibration_jog_without_goal_write():
    bridge = Bridge()
    session = CalibrationSession(bridge, profile())
    session.start()
    bridge.hardware_error = 4
    with pytest.raises(CalibrationSessionError):
        session.enable()
    assert bridge.writes == []


def test_saved_witnessed_profile_reloads_to_ready_and_uses_captured_ticks(tmp_path):
    bridge = Bridge(120)
    session = CalibrationSession(bridge, profile())
    session.start()
    session.capture_open()
    bridge.position = 620
    session.capture_close()
    session.validate()
    path = tmp_path / 'profiles.yaml'
    path.write_text('tool_profiles:\n  spur_1motor_gripper: {}\n', encoding='utf-8')
    session.save(path)
    saved = load_profiles(path)['spur_1motor_gripper']
    fsm = create_tool_fsm('spur_1motor_gripper', saved, bridge)
    assert fsm.startup() == ToolState.READY
    bridge.torque = 1
    assert fsm.command('OPEN') == ToolState.OPEN
    assert fsm.command('CLOSE') == ToolState.CLOSED
    assert bridge.writes[-2:] == [('goal', 5, 120), ('goal', 5, 620)]


def test_id5_feedback_branch_cannot_enter_legacy_dual_tuple_path():
    """Static guard for the ROS-dependent feedback callback topology."""
    bridge = (Path(__file__).parents[1] / 'dynamixel_control' /
              'moveit_dynamixel_bridge.py').read_text(encoding='utf-8')
    feedback = bridge[bridge.index('    def publish_joint_states'):]
    assert "if self.tool_ids == [5]:" in feedback
    assert "if self.tool_type == 'dual_motor_gripper':" in feedback
    assert feedback.index("if self.tool_ids == [5]:") < feedback.index(
        "if self.tool_type == 'dual_motor_gripper':")
    assert bridge.count('    def gripper_goal_callback(self, goal_request):') == 1


def test_id5_read_only_mock_feedback_is_stable_for_ten_seconds():
    """200 20-Hz observation ticks stay ID5-only and write-free."""
    bridge = Bridge(3320)
    session = CalibrationSession(bridge, profile())
    session.start()
    for _ in range(200):
        assert session.snapshot()['active'] is True
        assert bridge.read_position(5) == 3320
    assert bridge.writes == []
    assert bridge.allowlist == [5]
