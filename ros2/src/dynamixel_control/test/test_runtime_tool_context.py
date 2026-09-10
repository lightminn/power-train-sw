"""Exercise the production switch with memory-only adapters, never a ROS/SDK node."""
import ast
from pathlib import Path
from types import SimpleNamespace

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
    method = next(node for node in ast.walk(tree)
                  if isinstance(node, ast.FunctionDef) and node.name == '_switch_tool_runtime')
    namespace = dict(globals())
    namespace['ARM_IDS'] = set()
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), 'exec'), namespace)
    profile_path = Path(__file__).parents[1] / 'config/tool_profiles.yaml'
    bridge = SimpleNamespace(
        tool_type='cleaner', mock_mode=True, emergency_stop_active=False,
        tool_detached=False, _gripper_goal_active=False, tool_fsm=None,
        tool_ids=[], torque_enabled_ids=set(), active_ids=set(), tool_discovered=True,
        group_sync_read=SimpleNamespace(delParam=lambda _: None),
        get_parameter=lambda _: SimpleNamespace(value=str(profile_path)),
        get_logger=lambda: SimpleNamespace(info=lambda _: None))
    bridge.set_allowlist = lambda ids: setattr(bridge, '_fsm_allowlist', set(ids))
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


def test_dual_feedback_falls_back_to_direct_reads_after_tool_swap():
    source = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'
    text = source.read_text()
    start = text.index('# Legacy dual-gripper feedback')
    dual_block = text[start:text.index('        self.joint_state_pub.publish(msg)', start)]
    assert 'sample = self._read_spur_sample(gid)' in dual_block


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
