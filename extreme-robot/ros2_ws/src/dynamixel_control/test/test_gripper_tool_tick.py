"""Returning the rack gripper: ID3 to its remembered tick, ID4 limp, then descend."""

from types import SimpleNamespace

import pytest

from dynamixel_control.bench_tool_change import BenchToolChange, plan
from dynamixel_control.local_tool_change_bridge import LocalToolChangeBridge
from dynamixel_control.tool_fsm.base import ToolState
from dynamixel_control.tool_fsm.dual_motor_gripper_fsm import DualMotorGripperFSM

POSE = {'arm_joint_1': 0.0}
SLOTS = {'slots': {
    '1': {'tool_type': 'spur_1motor_gripper', 'seat': POSE, 'above': POSE},
    '2': {'tool_type': 'dual_motor_gripper', 'seat': POSE, 'above': POSE}}}


def names(attached, desired, **kwargs):
    steps = plan(SLOTS, attached, desired, POSE, rise_m=0.05, step_m=0.05, **kwargs)
    return [name for name, _ in steps]


def test_returning_the_gripper_sets_the_tick_before_descending():
    got = names('dual_motor_gripper', 'spur_1motor_gripper',
                tick_tools=('dual_motor_gripper',))
    assert got[:4] == ['MOVE_ABOVE', 'TOOL_TICK', 'DESCEND', 'UNLOCK']
    assert got.count('TOOL_TICK') == 1


def test_fetching_the_gripper_has_no_tick_step():
    assert 'TOOL_TICK' not in names('spur_1motor_gripper', 'dual_motor_gripper',
                                    tick_tools=('dual_motor_gripper',))


def test_the_step_is_skipped_unconfigured_and_on_dry_runs():
    assert 'TOOL_TICK' not in names('dual_motor_gripper', 'spur_1motor_gripper')
    assert 'TOOL_TICK' not in names('dual_motor_gripper', 'spur_1motor_gripper',
                                    tick_tools=('dual_motor_gripper',), dry_run=True)


class Bus:
    def __init__(self, present, mode=4, id4_torque=1):
        self.reg = {(3, 11): mode, (3, 70): 0, (3, 132): present, (4, 64): id4_torque}
        self.writes = []

    def read(self, dxl_id, addr, _length, _what, signed=False):
        return self.reg[(dxl_id, addr)]

    def write(self, dxl_id, addr, _length, value, _what):
        self.writes.append((dxl_id, addr, value))
        self.reg[(dxl_id, addr)] = value


def node(bus, tick):
    return SimpleNamespace(
        _read_register=bus.read, _write_register=bus.write, _tc_gripper_tick=tick,
        _tc_tool_goal=None,
        tool_profile=dict(goal_pwm=280, profile_acceleration=25, profile_velocity=80))


@pytest.mark.parametrize('present,tick,goal', [
    (-3970, 126, -3970), (100, 300, 300), (4000, 100, 4196), (-5000, 3000, -5192)])
def test_the_goal_is_the_nearest_position_with_that_tick(present, tick, goal):
    target = node(Bus(present), tick)
    LocalToolChangeBridge._tc_tool_tick(target, 'dual_motor_gripper')
    assert target._tc_tool_goal == goal


def test_id4_goes_limp_before_id3_gets_torque():
    bus = Bus(100)
    LocalToolChangeBridge._tc_tool_tick(node(bus, 300), 'dual_motor_gripper')
    assert bus.writes.index((4, 64, 0)) < bus.writes.index((3, 64, 1))
    assert (3, 100, 280) in bus.writes


def test_a_wrong_mode_never_moves():
    bus = Bus(100, mode=1)
    with pytest.raises(RuntimeError, match='mode 4'):
        LocalToolChangeBridge._tc_tool_tick(node(bus, 300), 'dual_motor_gripper')
    assert bus.writes == []


def test_a_stuck_motor_times_out_and_aborts():
    now = [0.0]
    held = []
    change = BenchToolChange(
        move=lambda *a, **k: None, arrived=lambda p: True, lock=lambda d: None,
        lock_state=lambda: 'DONE', attached_tool=lambda: None,
        hold=lambda: held.append(1), stop_lock=lambda: None, permission=lambda: None,
        tool_tick=lambda tool: None, tool_tick_arrived=lambda: False, clock=lambda: now[0])
    change.start([('TOOL_TICK', dict(slot='2', tool='dual_motor_gripper'))])
    now[0] = 11.0
    assert change.poll() == 'ABORTED' and held == [1]


def test_jogging_id3_alone_drops_id4_torque():
    off = []
    fsm = DualMotorGripperFSM.__new__(DualMotorGripperFSM)
    fsm.bridge = SimpleNamespace(read_torque=lambda i: 1,
                                 set_torque=lambda i, enabled: off.append((i, enabled)))
    fsm.state = ToolState.READY

    def stop_here():
        raise RuntimeError('stop here')

    fsm._validated_targets = stop_here
    with pytest.raises(RuntimeError, match='stop here'):
        fsm.command('JOG_OPEN', target='left')
    assert off == [(4, False)]
