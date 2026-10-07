"""Exercise the production switch with memory-only adapters, never a ROS/SDK node."""
import ast
from pathlib import Path
from types import SimpleNamespace
import threading

import pytest

from dynamixel_control.tool_fsm.cleaner_fsm import CleanerFSM
from dynamixel_control.calibration_session import CalibrationSession
from dynamixel_control.dual_calibration_session import DualCalibrationSession
from dynamixel_control.dual_manual_recovery import DualManualRecovery
from dynamixel_control.tool_fsm.base import ToolState
from dynamixel_control.tool_manager import ToolManager, ParameterToolIdentityProvider
from dynamixel_control.tool_profiles import load_profiles, ToolProfileError


def test_bridge_mock_dual_spur_cleaner_dual_reuses_existing_contexts():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    tree = ast.parse(source.read_text())
    methods = [node for node in ast.walk(tree)
               if isinstance(node, ast.FunctionDef) and node.name in (
                   '_switch_tool_runtime', '_switch_tool_runtime_impl', '_required_tool_modes',
                   '_maybe_auto_enable_attached_tool',
                   '_unregister_tool_feedback_ids')]
    namespace = dict(globals())
    namespace['ARM_IDS'] = set()
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'), namespace)
    profile_path = Path(__file__).parents[1] / 'config/tool_profiles.yaml'
    bridge = SimpleNamespace(
        control_scope='FULL_ROBOT',
        tool_type='cleaner', mock_mode=True, emergency_stop_active=False,
        tool_detached=False, _gripper_goal_active=False, tool_fsm=None,
        tool_ids=[], torque_enabled_ids=set(), active_ids=set(), tool_discovered=True,
        group_sync_read=SimpleNamespace(removeParam=lambda _: None),
        _bus_lock=threading.RLock(),
        get_parameter=lambda _: SimpleNamespace(value=str(profile_path)),
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    bridge.set_allowlist = lambda ids: setattr(bridge, '_fsm_allowlist', set(ids))
    bridge._release_current_tool_for_switch = (
        lambda requested: setattr(bridge, 'tool_motion_allowed', False))
    bridge._required_tool_modes = lambda profile: namespace['_required_tool_modes'](bridge, profile)
    bridge._maybe_auto_enable_attached_tool = lambda: False
    bridge._unregister_tool_feedback_ids = (
        lambda ids: namespace['_unregister_tool_feedback_ids'](bridge, ids))
    bridge._switch_tool_runtime_impl = lambda requested: namespace['_switch_tool_runtime_impl'](bridge, requested)
    bridge.read_position = lambda i: bridge._tool_samples[i]['position']
    bridge.read_torque = lambda _: 0
    bridge.read_hardware_error = lambda _: 0
    bridge.read_model = lambda _: 1060
    switch = namespace['_switch_tool_runtime']
    for tool, ids, fsm_name in (
            ('dual_motor_gripper', [3, 4], 'DualMotorGripperFSM'),
            ('spur_1motor_gripper', [5], 'SingleMotorGripperFSM'),
            ('cleaner', [2], 'CleanerFSM'),
            ('dual_motor_gripper', [3, 4], 'DualMotorGripperFSM')):
        # The production switch requires the old tool to have stopped.
        if bridge.tool_fsm:
            bridge.tool_fsm.state = ToolState.STOPPED
        switch(bridge, tool)
        assert bridge.tool_type == tool
        assert bridge.tool_ids == bridge.tool_profile['actuator_ids'] == ids
        assert set(bridge._tool_samples) == set(ids)
        assert bridge.tool_motion_allowed is True
        assert bridge.torque_enabled_ids.isdisjoint(ids)
        assert all(bridge._tool_samples[dxl_id]['torque_state'] == 'OFF'
                   for dxl_id in ids)
        # Cleaner uses its velocity adapter rather than the gripper FSM
        # allowlist; its dedicated actuator ID is still in the active profile.
        assert bridge._fsm_allowlist == (set() if tool == 'cleaner' else set(ids))
        assert (type(bridge.tool_fsm).__name__ if bridge.tool_fsm else None) == fsm_name
        assert bool(bridge.calibration_session) == (tool == 'spur_1motor_gripper')
        assert bool(bridge.dual_calibration_session) == (tool == 'dual_motor_gripper')
        assert bool(bridge.dual_manual_recovery) == (tool == 'dual_motor_gripper')


def test_cleaner_setup_zeros_velocity_before_torque_enable():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    text = source.read_text()
    start = text.index('    def _configure_cleaning_actuator')
    end = text.index('    def _cleaner_direction_command', start)
    setup = text[start:end]
    assert "'cleaner zero goal velocity'" in setup
    assert setup.index("'cleaner zero goal velocity'") < setup.index('_enable_cleaner_torque')
    source_text = source.read_text()
    cleaner_enable = source_text[source_text.index('    def _enable_cleaner_torque'):
                                 source_text.index('    def _cleaner_direction_command')]
    assert 'ADDR_GOAL_VELOCITY' in cleaner_enable
    assert 'ADDR_GOAL_POSITION' not in cleaner_enable


def test_cleaner_feedback_falls_back_to_one_id_direct_reads_only():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    text = source.read_text()
    cleaner_block = text[text.index("if (self.tool_type == 'cleaner'"):
                         text.index("# The spur tool has exactly one feedback topology")]
    assert 'self._read_cleaner_sample(self.cleaning_actuator_id)' in cleaner_block
    method = text[text.index('    def _read_cleaner_sample'):
                  text.index('    def _read_tool_control_state')]
    assert 'ADDR_HARDWARE_ERROR_STATUS' in method
    assert 'ADDR_PRESENT_POSITION' in method
    assert 'ADDR_PRESENT_LOAD' in method


def test_cleaner_feedback_preserves_enable_gate_fields_and_refreshes_control_state():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    text = source.read_text()
    start = text.index("if (self.tool_type == 'cleaner'")
    end = text.index('# The spur tool has exactly one feedback topology', start)
    feedback = text[start:end]
    assert "dict(self._tool_samples.get(" in feedback
    assert 'self._read_tool_control_state(' in feedback
    assert 'self.cleaning_actuator_id)' in feedback
    assert "'hardware_error': int(hw_error)" in feedback
    assert "'torque_state': 'UNKNOWN'" in feedback


def test_cleaner_torque_off_sample_passes_explicit_enable_readiness_gate():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    tree = ast.parse(source.read_text())
    methods = [node for node in ast.walk(tree)
               if isinstance(node, ast.FunctionDef)
               and node.name in ('_tool_enable_allowed',
                                 '_tool_enable_block_reason')]
    namespace = {'ARM_IDS': set()}
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(source), 'exec'),
         namespace)
    bridge = SimpleNamespace(
        control_scope='END_EFFECTOR_ONLY', tool_type='cleaner',
        tool_selection=SimpleNamespace(valid=True),
        tool_profile={'calibrated': True, 'actuator_ids': [2]},
        tool_discovered=True, read_only=False, emergency_stop_active=False,
        tool_detached=False, _physical_tool_detached=False, tool_ids=[2],
        _tool_samples={2: {
            'id': 2, 'online': True, 'hardware_error': 0,
            'torque_state': 'OFF', 'position': 2855,
            'operating_mode': 1, 'goal_velocity': 0}},
        tool_fsm=SimpleNamespace(state=SimpleNamespace(name='READY')),
        _tool_actuators_online=lambda: True,
        _required_tool_modes=lambda _profile: {2: 1})
    bridge._tool_enable_block_reason = lambda: namespace[
        '_tool_enable_block_reason'](bridge)
    assert namespace['_tool_enable_allowed'](bridge) is True
    bridge._tool_samples[2]['torque_state'] = 'UNKNOWN'
    assert namespace['_tool_enable_allowed'](bridge) is False


def test_cleaner_fsm_routes_only_existing_left_right_stop_commands():
    from dynamixel_control.tool_fsm.cleaner_fsm import CleanerState

    commands = []
    bridge = SimpleNamespace(
        tool_motion_allowed=True,
        _cleaner_direction_command=lambda command: commands.append(command) or True)
    fsm = CleanerFSM({'actuator_ids': [2]}, bridge)
    assert fsm.startup() == CleanerState.READY
    assert fsm.command('LEFT') == CleanerState.CLEANING
    assert fsm.command('RIGHT') == CleanerState.CLEANING
    assert fsm.command('STOP') == CleanerState.READY
    assert commands == ['LEFT', 'RIGHT', 'STOP']
    with pytest.raises(ValueError, match='unsupported cleaner command'):
        fsm.command('OPEN')


def test_spur_feedback_falls_back_to_direct_reads_after_tool_swap():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    text = source.read_text()
    spur_block = text[text.index('if self.tool_ids == [5]:'):
                      text.index('# Legacy dual-gripper feedback',
                                 text.index('if self.tool_ids == [5]:'))]
    assert 'self._read_spur_sample(dxl_id)' in spur_block
    method = text[text.index('    def _read_spur_sample'):
                  text.index('    def _read_tool_control_state')]
    assert 'ADDR_HARDWARE_ERROR_STATUS' in method
    assert 'ADDR_PRESENT_POSITION' in method
    assert 'ADDR_PRESENT_LOAD' in method


def test_cleaner_velocity_write_uses_the_shared_bus_lock():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    text = source.read_text()
    start = text.index('    def _on_cleaning_enable')
    end = text.index('    def rad_to_tick', start)
    command = text[start:end]
    assert 'with self._bus_lock:' in command
    assert "'cleaner goal velocity'" in command


def test_developer_cleaner_command_has_its_own_executor_group_and_minimum_gate():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    text = source.read_text()
    subscription = text[text.index('Bool, "/cleaning/enable"'):
                        text.index('Bool, "/tool/emergency_stop"')]
    assert 'callback_group=self._cleaner_direct_group' in subscription
    start = text.index('    def _on_cleaning_enable')
    end = text.index('    def rad_to_tick', start)
    command = text[start:end]
    assert 'direct_bench' in command
    assert 'self.emergency_stop_active or self.tool_detached' in command
    assert 'not self.cleaning_configured or not self.tool_discovered' in command


def test_cleaner_direction_buttons_use_the_interrupt_ingress():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    text = source.read_text()
    subscription = text[text.index("String, '/cleaning/direction'"):
                        text.index('Bool, "/tool/emergency_stop"')]
    assert 'callback_group=self._cleaner_direct_group' in subscription
    assert 'def _on_cleaning_direction' in text


def test_cleaner_direction_mailbox_preempts_older_commands():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    text = source.read_text()
    start = text.index('    def _submit_cleaner_velocity')
    end = text.index('    def rad_to_tick', start)
    mailbox = text[start:end]
    assert '_cleaner_command_generation' in mailbox
    assert '_cleaner_requested_velocity' in mailbox
    assert 'generation == self._cleaner_command_generation' in mailbox
