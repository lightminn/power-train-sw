"""Pure dual FSM safety tests; no ROS node, SDK, or serial hardware."""

import pytest

from dynamixel_control.tool_fsm.base import ToolCommandError, ToolState
from dynamixel_control.tool_fsm.dual_motor_gripper_fsm import DualMotorGripperFSM
from dynamixel_control.tool_fsm.registry import create_tool_fsm


class MockBridge:
    def __init__(self):
        self.allowlist = set()
        self.positions = {3: 500, 4: 1500}
        self.torque = {3: 1, 4: 1}
        self.errors = {3: 0, 4: 0}
        self.models = {3: 1060, 4: 1060}
        self.commands = []

    def set_allowlist(self, ids): self.allowlist = set(ids)
    def read_position(self, dxl_id): return self.positions[dxl_id]
    def read_torque(self, dxl_id): return self.torque[dxl_id]
    def read_hardware_error(self, dxl_id): return self.errors[dxl_id]
    def read_model(self, dxl_id): return self.models[dxl_id]
    def command_dual_targets(self, targets): self.commands.append(dict(targets))
    def start_dual_jog(self, targets): self.commands.append(('jog', dict(targets)))
    def hold_dual_position(self): self.commands.append(('hold', dict(self.positions)))
    def set_torque(self, dxl_id, enabled):
        self.torque[dxl_id] = int(enabled)
        self.commands.append(('torque', dxl_id, bool(enabled)))


def profile(**changes):
    data = {
        'backend': 'gripper', 'calibrated': True,
        'endpoint_calibration_verified': True,
        'endpoint_calibration_models': {3: 1060, 4: 1060},
        'actuator_ids': [3, 4], 'safe_min_tick': 0, 'safe_max_tick': 2500,
        'motor_endpoints': {
            3: {'open': 1000, 'close': 100},
            4: {'open': 2000, 'close': 1100}},
    }
    data.update(changes)
    return data


def ready_fsm(bridge=None, **changes):
    bridge = bridge or MockBridge()
    fsm = create_tool_fsm('dual_motor_gripper', profile(**changes), bridge)
    assert isinstance(fsm, DualMotorGripperFSM)
    assert fsm.startup() == ToolState.READY
    return fsm, bridge


def test_registry_selects_dual_fsm_and_open_close_use_dual_adapter():
    fsm, bridge = ready_fsm()
    assert fsm.command('OPEN') == ToolState.OPEN
    assert bridge.commands[-1] == {3: 1000, 4: 2000}
    assert fsm.command('CLOSE') == ToolState.CLOSED
    assert bridge.commands[-1] == {3: 100, 4: 1100}
    assert bridge.torque == {3: 1, 4: 1}


@pytest.mark.parametrize('offline_id', (3, 4))
def test_one_offline_motor_blocks_without_goal_write(offline_id):
    fsm, bridge = ready_fsm()
    bridge.positions[offline_id] = None
    with pytest.raises(ToolCommandError, match='position unavailable'):
        fsm.command('OPEN')
    assert bridge.commands == []


@pytest.mark.parametrize('fault_id', (3, 4))
def test_hardware_error_blocks_without_goal_write(fault_id):
    fsm, bridge = ready_fsm()
    bridge.errors[fault_id] = 4
    with pytest.raises(ToolCommandError, match='hardware error'):
        fsm.command('CLOSE')
    assert bridge.commands == []


def test_invalid_calibration_fails_startup_and_never_writes():
    bridge = MockBridge()
    fsm = create_tool_fsm(
        'dual_motor_gripper',
        profile(calibrated=False, endpoint_calibration_verified=False), bridge)
    assert fsm.startup() == ToolState.FAULT
    assert bridge.commands == []


def test_torque_off_blocks_without_goal_write():
    fsm, bridge = ready_fsm()
    bridge.torque[4] = 0
    with pytest.raises(ToolCommandError, match='Torque Enable is OFF'):
        fsm.command('OPEN')
    assert bridge.commands == []


def test_stop_disables_both_motors():
    fsm, bridge = ready_fsm()
    assert fsm.command('STOP') == ToolState.STOPPED
    assert bridge.commands == [
        ('torque', 3, False), ('torque', 4, False)]
    assert bridge.torque == {3: 0, 4: 0}


def test_hold_to_run_starts_endpoint_direction_and_release_holds_position():
    fsm, bridge = ready_fsm()
    assert fsm.command('JOG_OPEN') == ToolState.OPENING
    assert bridge.commands[-1] == ('jog', {3: 600, 4: 1600})
    bridge.positions.update({3: 700, 4: 1700})
    assert fsm.command('HOLD') == ToolState.READY
    assert bridge.commands[-1] == ('hold', {3: 700, 4: 1700})


def test_hold_without_active_jog_is_rejected_without_write():
    fsm, bridge = ready_fsm()
    with pytest.raises(ToolCommandError, match='HOLD unavailable'):
        fsm.command('HOLD')
    assert bridge.commands == []


@pytest.mark.parametrize('command,expected', [
    ('JOG_CLOSE', {3: -1100, 4: 800}),
    ('JOG_OPEN', {3: -900, 4: 700}),
])
def test_relative_jog_uses_signed_endpoint_span_ratio(command, expected):
    fsm, bridge = ready_fsm()
    fsm.profile.update(safe_min_tick=-2000, safe_max_tick=2000,
                      motor_endpoints={3: {'open': 500, 'close': -2000},
                                       4: {'open': 0, 'close': 1250}})
    bridge.positions = {3: -1000, 4: 750}
    original = repr(fsm.profile)
    fsm.command(command)
    assert bridge.commands[-1] == ('jog', expected)
    bridge.positions = expected
    fsm.command(command)
    assert repr(fsm.profile) == original
    assert bridge.commands[-1][1] != expected


def test_jog_clamps_both_motors_at_endpoint_and_does_not_accumulate():
    fsm, bridge = ready_fsm()
    bridge.positions = {3: 990, 4: 1990}
    for _ in range(3):
        fsm.command('JOG_OPEN')
        assert bridge.commands[-1] == ('jog', {3: 1000, 4: 2000})
    bridge.positions = {3: 1000, 4: 2000}
    fsm.command('JOG_OPEN')
    assert bridge.commands[-1] == ('jog', bridge.positions)


def test_jog_rejects_motor_outside_own_endpoints():
    fsm, bridge = ready_fsm()
    bridge.positions = {3: 50, 4: 1050}
    with pytest.raises(ToolCommandError, match='outside motor endpoints'):
        fsm.command('JOG_CLOSE')
    assert bridge.commands == []
