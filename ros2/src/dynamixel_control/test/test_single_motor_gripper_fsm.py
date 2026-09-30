"""Pure unit tests: no ROS node, serial port, or Dynamixel SDK access."""

import pytest

from dynamixel_control.tool_fsm.base import ToolCommandError, ToolState
from dynamixel_control.tool_fsm.registry import FSM_REGISTRY, create_tool_fsm


class MockBridge:
    def __init__(self, position=2000, torque=0, error=0, fail_read=False):
        self.position, self.torque, self.error = position, torque, error
        self.fail_read = fail_read
        self.allowlist, self.writes = (), []

    def set_allowlist(self, ids): self.allowlist = tuple(ids)
    def read_model(self, _id): return 1060
    def read_position(self, _id):
        if self.fail_read: raise RuntimeError('communication failed')
        return self.position
    def read_torque(self, _id): return self.torque
    def read_hardware_error(self, _id): return self.error
    def goal_position(self, dxl_id, tick): self.writes.append(('goal', dxl_id, tick))
    def set_torque(self, dxl_id, enabled): self.writes.append(('torque', dxl_id, enabled))


def profile(**overrides):
    data = {'actuator_ids': [5], 'calibrated': True, 'open_tick': 2100,
            'close_tick': 1900, 'safe_min_tick': 1800, 'safe_max_tick': 2200,
            'motor_model': 1060}
    data.update(overrides)
    return data


def test_registry_selects_single_motor_id5_fsm():
    assert 'spur_1motor_gripper' in FSM_REGISTRY
    bridge = MockBridge()
    fsm = create_tool_fsm('spur_1motor_gripper', profile(), bridge)
    assert fsm.startup() == ToolState.READY
    assert bridge.allowlist == (5,)


def test_uncalibrated_profile_requires_calibration_and_never_writes():
    bridge = MockBridge()
    fsm = create_tool_fsm('spur_1motor_gripper', profile(calibrated=False,
                          open_tick=None, close_tick=None, safe_min_tick=None,
                          safe_max_tick=None), bridge)
    assert fsm.startup() == ToolState.CALIBRATION_REQUIRED
    with pytest.raises(ToolCommandError): fsm.command('OPEN')
    with pytest.raises(ToolCommandError): fsm.command('CLOSE')
    assert bridge.writes == []


def test_open_close_target_only_id5_and_stop_has_no_goal():
    bridge = MockBridge(torque=1)
    fsm = create_tool_fsm('spur_1motor_gripper', profile(), bridge)
    fsm.startup()
    assert fsm.command('OPEN') == ToolState.OPEN
    assert fsm.command('CLOSE') == ToolState.CLOSED
    assert bridge.writes == [('goal', 5, 2100), ('goal', 5, 1900)]
    assert fsm.command('STOP') == ToolState.STOPPED
    assert bridge.writes[-1] == ('torque', 5, False)


def test_rejects_bad_range_and_communication_fault():
    bad = create_tool_fsm('spur_1motor_gripper', profile(open_tick=2300), MockBridge())
    assert bad.startup() == ToolState.FAULT
    failed = create_tool_fsm('spur_1motor_gripper', profile(), MockBridge(fail_read=True))
    assert failed.startup() == ToolState.FAULT


def test_id3_id4_are_not_valid_single_motor_targets():
    fsm = create_tool_fsm('spur_1motor_gripper', profile(actuator_ids=[3, 4]), MockBridge())
    assert fsm.startup() == ToolState.FAULT
