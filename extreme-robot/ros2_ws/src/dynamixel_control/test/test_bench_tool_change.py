"""Bench tool change: slot teaching, plan order and the step runner's stop paths."""

import json

import pytest

from dynamixel_control import bench_tool_change as T

NAMES = ['arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4', 'arm_joint_5']


def seat(q2=0.8, q3=1.0, j1=0.1):
    return {'arm_joint_1': j1, 'arm_joint_2': q2, 'arm_joint_3': q3,
            'arm_joint_4': 0.2, 'arm_joint_5': 0.0}


def above(base):
    """The taught pose over ``base``: same head pitch (-q2 + q3 + q4)."""
    pose = dict(base)
    pose.update(arm_joint_2=base['arm_joint_2'] - 0.1, arm_joint_3=base['arm_joint_3'] + 0.05,
                arm_joint_4=base['arm_joint_4'] - 0.15)
    return pose


def table():
    return {'version': 1, 'slots': {
        '1': dict(tool_type='spur_1motor_gripper', seat=seat(j1=0.1), above=above(seat(j1=0.1))),
        '2': dict(tool_type='dual_motor_gripper', seat=seat(j1=-0.1),
                  above=above(seat(j1=-0.1)))}}


HOME = {name: 0.0 for name in NAMES}


def test_capture_saves_atomically_and_replaces_a_tools_old_slot(tmp_path):
    path = tmp_path / 'slots.json'
    store = T.ToolSlotStore(str(path), NAMES)
    store.capture('1', 'spur_1motor_gripper', seat(), 'seat')
    store.capture('1', 'spur_1motor_gripper', above(seat()), 'above')
    assert T.slot_complete(store.data['slots']['1'])
    store.capture('2', 'spur_1motor_gripper', seat(j1=-0.1), 'seat')
    assert list(store.data['slots']) == ['2']
    assert not T.slot_complete(store.data['slots']['2'])
    assert T.ToolSlotStore(str(path), NAMES).load() == store.data
    assert [p.name for p in tmp_path.iterdir()] == ['slots.json']


@pytest.mark.parametrize('bad', ['head_turned', 'limit', 'above_first', 'other_tool'])
def test_capture_refuses_unusable_poses_without_touching_the_file(tmp_path, bad):
    path = tmp_path / 'slots.json'
    store = T.ToolSlotStore(str(path), NAMES)
    store.capture('1', 'spur_1motor_gripper', seat(), 'seat')
    before = path.read_text()
    with pytest.raises(ValueError):
        if bad == 'head_turned':
            pose = above(seat())
            pose['arm_joint_4'] += 0.5
            store.capture('1', 'spur_1motor_gripper', pose, 'above')
        elif bad == 'limit':
            store.capture('1', 'spur_1motor_gripper', above(seat(q2=1.9)), 'above')
        elif bad == 'above_first':
            store.capture('2', 'dual_motor_gripper', above(seat()), 'above')
        else:
            store.capture('1', 'dual_motor_gripper', above(seat()), 'above')
    assert path.read_text() == before


def test_reteaching_the_seat_drops_the_old_above_pose(tmp_path):
    store = T.ToolSlotStore(str(tmp_path / 'slots.json'), NAMES)
    store.capture('1', 'spur_1motor_gripper', seat(), 'seat')
    store.capture('1', 'spur_1motor_gripper', above(seat()), 'above')
    store.capture('1', 'spur_1motor_gripper', seat(q2=0.9), 'seat')
    assert 'above' not in store.data['slots']['1']
    with pytest.raises(ValueError, match='5 cm above'):
        T.plan(store.data, None, 'spur_1motor_gripper', HOME, rise_m=0.05, step_m=0.01)


def names(steps):
    return [(name, payload.get('slot')) for name, payload in steps]


def test_no_tool_attached_only_fetches_the_requested_tool():
    steps = T.plan(table(), None, 'spur_1motor_gripper', HOME, rise_m=0.05, step_m=0.01)
    assert names(steps) == [('MOVE_ABOVE', '1'), ('DESCEND', '1'), ('LOCK', '1'),
                            ('ASCEND', '1'), ('VERIFY_ATTACHED', None), ('HOME', None)]


def test_attached_tool_is_returned_to_its_slot_before_fetching():
    steps = T.plan(table(), 'spur_1motor_gripper', 'dual_motor_gripper', HOME,
                   rise_m=0.05, step_m=0.01)
    assert names(steps) == [
        ('MOVE_ABOVE', '1'), ('DESCEND', '1'), ('UNLOCK', '1'), ('ASCEND', '1'),
        ('VERIFY_RELEASED', None),
        ('MOVE_ABOVE', '2'), ('DESCEND', '2'), ('LOCK', '2'), ('ASCEND', '2'),
        ('VERIFY_ATTACHED', None), ('HOME', None)]
    descend = steps[1][1]['poses']
    assert descend[-1] == pytest.approx(table()['slots']['1']['seat'])
    top = steps[0][1]['poses'][0]
    assert top == pytest.approx(table()['slots']['1']['above'])
    assert steps[3][1]['poses'][-1] == pytest.approx(top)
    assert len(descend) == 5


@pytest.mark.parametrize('attached,desired', [
    ('spur_1motor_gripper', 'spur_1motor_gripper'), (None, 'cleaner'),
    ('cleaner', 'spur_1motor_gripper')])
def test_impossible_requests_are_refused(attached, desired):
    with pytest.raises(ValueError):
        T.plan(table(), attached, desired, HOME, rise_m=0.05, step_m=0.01)


class Rig:
    def __init__(self):
        self.now = 0.0
        self.calls = []
        self.at = None
        self.lock_status = 'MOVING'
        self.tool = None
        self.granted = True

    def runner(self):
        def permission():
            if not self.granted:
                raise RuntimeError('manual control lost')
        return T.BenchToolChange(
            move=lambda poses, vertical: self.calls.append(('move', len(poses), vertical)),
            arrived=lambda pose: pose == self.at,
            lock=lambda desired: self.calls.append(('lock', desired)),
            lock_state=lambda: self.lock_status,
            attached_tool=lambda: self.tool,
            hold=lambda: self.calls.append(('hold',)),
            stop_lock=lambda: self.calls.append(('stop_lock',)),
            permission=permission, clock=lambda: self.now)


def drive(rig, runner, steps):
    """Satisfy every step the way the hardware would, recording the run."""
    for _ in range(100):
        if not runner.running:
            break
        name, payload = steps[runner.index]
        if 'poses' in payload:
            rig.at = payload['poses'][-1]
        if name == 'LOCK':
            rig.lock_status, rig.tool = 'DONE', 'dual_motor_gripper'
        if name == 'UNLOCK':
            rig.lock_status, rig.tool = 'DONE', None
        runner.poll()
        rig.lock_status = 'MOVING'


def test_full_exchange_runs_every_step_in_order_and_ends_done():
    rig = Rig()
    rig.tool = 'spur_1motor_gripper'
    runner = rig.runner()
    steps = T.plan(table(), 'spur_1motor_gripper', 'dual_motor_gripper', HOME,
                   rise_m=0.05, step_m=0.01)
    runner.start(steps)
    drive(rig, runner, steps)
    assert runner.state == 'DONE', runner.error
    assert rig.calls == [
        ('move', 1, False), ('move', 5, True), ('lock', 'UNLOCKED'), ('move', 5, True),
        ('move', 1, False), ('move', 5, True), ('lock', 'LOCKED'), ('move', 5, True),
        ('move', 1, False)]


def test_waits_for_arrival_and_times_out_into_a_hold():
    rig = Rig()
    runner = rig.runner()
    runner.start(T.plan(table(), None, 'spur_1motor_gripper', HOME, rise_m=0.05, step_m=0.01))
    assert runner.poll() == 'MOVE_ABOVE'
    rig.now = 21.0
    assert runner.poll() == 'ABORTED'
    assert 'timed out' in runner.error
    assert rig.calls[-2:] == [('stop_lock',), ('hold',)]


def test_losing_manual_control_or_a_lock_fault_aborts_immediately():
    rig = Rig()
    runner = rig.runner()
    steps = T.plan(table(), None, 'spur_1motor_gripper', HOME, rise_m=0.05, step_m=0.01)
    runner.start(steps)
    rig.granted = False
    assert runner.poll() == 'ABORTED' and 'manual control' in runner.error
    rig.granted = True
    runner = rig.runner()
    runner.start(steps)
    rig.at = steps[0][1]['poses'][-1]
    runner.poll()
    rig.at = steps[1][1]['poses'][-1]
    runner.poll()
    assert runner.state == 'LOCK'
    rig.lock_status = 'coupling did not reach LOCKED'
    assert runner.poll() == 'ABORTED'
    assert 'did not reach' in runner.error


def test_a_tool_that_stays_on_after_unlock_stops_the_exchange():
    rig = Rig()
    rig.tool = 'spur_1motor_gripper'
    runner = rig.runner()
    steps = T.plan(table(), 'spur_1motor_gripper', 'dual_motor_gripper', HOME,
                   rise_m=0.05, step_m=0.01)
    runner.start(steps)
    for _ in range(10):
        name, payload = steps[runner.index]
        if 'poses' in payload:
            rig.at = payload['poses'][-1]
        if name == 'UNLOCK':
            rig.lock_status = 'DONE'     # lock moved, but the tool is still detected
        runner.poll()
        rig.lock_status = 'MOVING'
        if runner.state == 'VERIFY_RELEASED':
            break
    rig.now = 100.0
    assert runner.poll() == 'ABORTED'
    assert not any(call == ('lock', 'LOCKED') for call in rig.calls)


def test_slot_table_round_trips_as_json():
    assert T.validate_slots(json.loads(json.dumps(table())), NAMES)


def test_dry_run_rehearses_motion_without_coupling_or_tool_checks():
    steps = T.plan(table(), None, 'spur_1motor_gripper', HOME, rise_m=0.05,
                   step_m=0.01, dry_run=True)
    assert names(steps) == [('MOVE_ABOVE', '1'), ('DESCEND', '1'), ('ASCEND', '1'),
                            ('HOME', None)]


def test_limit_switch_off_saves_and_reloads_a_seat_beyond_the_limits(tmp_path):
    path = tmp_path / 'slots.json'
    store = T.ToolSlotStore(str(path), NAMES)
    pose = seat(q2=1.9)
    with pytest.raises(ValueError, match='joint limit'):
        store.capture('1', 'spur_1motor_gripper', pose, 'seat')
    store.capture('1', 'spur_1motor_gripper', pose, 'seat', limits=False)
    store.capture('1', 'spur_1motor_gripper', above(pose), 'above', limits=False)
    assert store.data['slots']['1']['limits_checked'] is False
    assert T.ToolSlotStore(str(path), NAMES).load() == store.data
    with pytest.raises(ValueError, match='joint limit'):
        T.plan(store.data, None, 'spur_1motor_gripper', HOME, rise_m=0.05, step_m=0.01)
    assert T.plan(store.data, None, 'spur_1motor_gripper', HOME, rise_m=0.05,
                  step_m=0.01, limits=False)
