"""Returning the rack gripper: ID3 to its remembered tick, ID4 limp, then descend."""

import math
from types import SimpleNamespace

import pytest

from dynamixel_control.bench_tool_change import BenchToolChange, plan
from dynamixel_control.local_tool_change_bridge import LocalToolChangeBridge
from dynamixel_control.tool_fsm.base import ToolState
from dynamixel_control.tool_fsm.dual_motor_gripper_fsm import DualMotorGripperFSM

POSE = {'arm_joint_1': -0.05}
SLOTS_TWO = {'slots': {'2': {
    'tool_type': 'dual_motor_gripper', 'seat': {'arm_joint_1': 0.1},
    'above': {'arm_joint_1': 0.3}}}}
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
    target = SimpleNamespace(
        _read_register=bus.read, _write_register=bus.write, _tc_gripper_tick=tick,
        _tc_tool_goal=None, _tc_tool_lead=0, _tc_tool_still=None, _tc_clock=lambda: 0.0,
        get_logger=lambda: SimpleNamespace(warn=lambda _m: None),
        tool_profile=dict(goal_pwm=280, profile_acceleration=25, profile_velocity=80))
    target._tc_tool_settled = lambda p: LocalToolChangeBridge._tc_tool_settled(target, p)
    return target


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


def test_the_goal_leads_the_motor_and_is_dropped_on_arrival():
    bus = Bus(-3914)
    target = node(bus, 126)
    LocalToolChangeBridge._tc_tool_tick(target, 'dual_motor_gripper')
    assert target._tc_tool_goal == -3970
    assert bus.writes[-1] == (3, 116, (-3970 - 256) & 0xffffffff)
    arrived = lambda: LocalToolChangeBridge._tc_tool_tick_arrived(target)
    assert arrived() is False
    bus.reg[(3, 132)] = -3955
    assert arrived() is True and bus.writes[-1] == (3, 116, (-3955) & 0xffffffff)


def test_overshooting_a_slipping_rack_still_arrives():
    bus = Bus(-3914)
    target = node(bus, 126)
    LocalToolChangeBridge._tc_tool_tick(target, 'dual_motor_gripper')
    bus.reg[(3, 132)] = -4100
    assert LocalToolChangeBridge._tc_tool_tick_arrived(target) is True


def test_a_jammed_rack_moves_on_after_settling():
    bus = Bus(-3914)
    now = [0.0]
    target = node(bus, 126)
    target._tc_clock = lambda: now[0]
    LocalToolChangeBridge._tc_tool_tick(target, 'dual_motor_gripper')
    arrived = lambda: LocalToolChangeBridge._tc_tool_tick_arrived(target)
    assert arrived() is False
    now[0] = 1.9
    assert arrived() is False
    now[0] = 2.1
    assert arrived() is True


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


def test_fitting_a_tool_ends_with_its_exercise_after_home():
    got = names('spur_1motor_gripper', 'dual_motor_gripper',
                exercise_tools=('dual_motor_gripper',))
    assert got[-2:] == ['HOME', 'TOOL_EXERCISE']


def test_a_dry_run_never_exercises():
    assert 'TOOL_EXERCISE' not in names(
        'spur_1motor_gripper', 'dual_motor_gripper', dry_run=True,
        exercise_tools=('dual_motor_gripper',))


def exercise_node(bus, motor=(3, 4)):
    target = node(bus, 0)
    target._tool_axis_motor = lambda: motor
    target._enable_tool_axis = lambda *args: bus.writes.append(('enable',) + args)
    target._tc_exercise = None
    return target


def test_the_gripper_opens_then_returns():
    bus = Bus(-3970)
    target = exercise_node(bus)
    LocalToolChangeBridge._tc_tool_exercise(target, 'dual_motor_gripper')
    assert bus.writes[:2] == [(3, 64, 0), ('enable', 3, 4, -3970)]
    done = lambda: LocalToolChangeBridge._tc_tool_exercise_done(target)
    assert done() is False and bus.writes[-1] == (3, 116, (-3370) & 0xffffffff)
    assert done() is False  # still on its way
    bus.reg[(3, 132)] = -3372
    assert done() is False and bus.writes[-1] == (3, 116, (-3970) & 0xffffffff)
    bus.reg[(3, 132)] = -3968
    assert done() is True


def test_the_excavator_stays_inside_one_turn():
    bus = Bus(3990)
    bus.reg[(5, 70)], bus.reg[(5, 132)] = 0, 3990
    target = exercise_node(bus, motor=(5, 3))
    LocalToolChangeBridge._tc_tool_exercise(target, 'spur_1motor_gripper')
    assert target._tc_exercise['goals'] == [4095, 3990]


def test_an_exercise_failure_drops_its_torque():
    bus = Bus(0)
    target = exercise_node(bus)
    target._tc_stream = None
    target._tc_tool_goal = None
    target._tc_exercise = {'id': 5, 'goals': [], 'goal': 10}
    target.active_ids = set()
    LocalToolChangeBridge._tc_hold(target)
    assert bus.writes == [(5, 64, 0)] and target._tc_exercise is None


def test_a_release_only_returns_the_tool():
    got = names('dual_motor_gripper', None, tick_tools=('dual_motor_gripper',),
                exercise_tools=('dual_motor_gripper',))
    assert got == ['MOVE_ABOVE', 'TOOL_TICK', 'DESCEND', 'UNLOCK', 'ASCEND',
                   'VERIFY_RELEASED', 'HOME']


def test_a_release_needs_a_tool():
    with pytest.raises(ValueError, match='no tool is attached'):
        names(None, None)


def test_fetching_turns_joint_1_at_the_seat_and_keeps_it_on_the_way_up():
    steps = plan(SLOTS_TWO, None, 'dual_motor_gripper', POSE, rise_m=0.05, step_m=0.05,
                 limits=False)
    got = [name for name, _ in steps]
    assert got[:5] == ['MOVE_ABOVE', 'DESCEND', 'SEAT_NUDGE', 'LOCK', 'ASCEND']
    payload = dict(steps)
    nudged = payload['SEAT_NUDGE']['poses'][0]['arm_joint_1']
    assert nudged == pytest.approx(0.1 + math.radians(5))
    assert payload['ASCEND']['poses'][-1]['arm_joint_1'] == pytest.approx(0.3 + math.radians(5))


def test_returning_a_tool_has_no_turn():
    assert 'SEAT_NUDGE' not in names('dual_motor_gripper', None)


def test_a_dry_run_rehearses_the_turn():
    assert 'SEAT_NUDGE' in names(None, 'dual_motor_gripper', dry_run=True)


def test_the_rise_continues_past_the_above_pose_and_the_descent_does_not():
    steps = plan(SLOTS_TWO, None, 'dual_motor_gripper', POSE, rise_m=0.05, step_m=0.05,
                 limits=False, extra_rise_m=0.05)
    payload = dict(steps)
    assert payload['MOVE_ABOVE']['poses'][-1]['arm_joint_1'] == pytest.approx(0.3)
    assert payload['DESCEND']['poses'][-1]['arm_joint_1'] == pytest.approx(0.1)
    assert len(payload['ASCEND']['poses']) == 2
    assert payload['ASCEND']['poses'][-1]['arm_joint_1'] == pytest.approx(0.3 + math.radians(5))


def test_a_return_comes_down_from_the_raised_pose_with_the_taught_joint_1():
    steps = plan(SLOTS_TWO, 'dual_motor_gripper', None, POSE, rise_m=0.05, step_m=0.05,
                 limits=False, extra_rise_m=0.05)
    payload = dict(steps)
    assert payload['MOVE_ABOVE']['poses'][-1]['arm_joint_1'] == pytest.approx(0.3)
    assert [p['arm_joint_1'] for p in payload['DESCEND']['poses']] == pytest.approx([0.3, 0.1])
    assert payload['ASCEND']['poses'][-1]['arm_joint_1'] == pytest.approx(0.3)


def test_the_rack_gripper_passes_the_above_pose_as_taught():
    payload = dict(plan(SLOTS_TWO, 'dual_motor_gripper', None, POSE, rise_m=0.05, step_m=0.05,
                        limits=False))
    assert payload['MOVE_ABOVE']['poses'][-1]['arm_joint_1'] == pytest.approx(0.3)


def test_the_excavator_fetch_has_no_turn():
    slots = {'slots': {'1': dict(SLOTS_TWO['slots']['2'], tool_type='spur_1motor_gripper')}}
    steps = plan(slots, None, 'spur_1motor_gripper', POSE, rise_m=0.05, step_m=0.05,
                 limits=False, extra_rise_m=0.05)
    payload = dict(steps)
    assert 'SEAT_NUDGE' not in payload
    assert payload['ASCEND']['poses'][-1]['arm_joint_1'] == pytest.approx(0.3)
