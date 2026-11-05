"""모든 단계의 중단에서 후속 이동과 confirm이 새지 않는지 검증한다."""

import pytest

from dynamixel_control.auto_tool_change import AutoToolChange
from dynamixel_control.toolchanger_lock import ToolchangerLock
from dynamixel_control.toolchanger_sequence import ToolchangerSequence
from auto_tool_change_fixtures import dock_fixture
from test_toolchanger_lock import IO


def setup_sequence():
    clock = [10.0]
    allowed = [True]
    moves, holds, verifies = [], [], []
    io = IO()
    lock = ToolchangerLock(io, clock=lambda: clock[0])
    lock.capture_zero('LOCKED')
    lock.enable()
    lock_sequence = ToolchangerSequence(lock, lambda: None, clock=lambda: clock[0])
    def permission():
        if not allowed[0]:
            raise RuntimeError('permission lost')
    core = AutoToolChange(lock_sequence,
                         lambda names, values: moves.append(dict(zip(names, values))),
                         lambda: holds.append(True), permission,
                         verify_tool=lambda: verifies.append(True), clock=lambda: clock[0])
    docks = dock_fixture()
    for index, (slot, pose) in enumerate((('1', 'approach'), ('1', 'seat'), ('1', 'retreat'),
                                        ('2', 'approach'), ('2', 'seat'), ('2', 'retreat')), 1):
        docks['slots'][slot]['poses'][pose]['positions'][3] = index * 0.01
        docks['slots'][slot]['poses'][pose]['tolerance_rad'] = 0.001
    positions = dict.fromkeys(docks['joint_names'], 0.0)
    core.observe(positions, dict.fromkeys(positions, 0.0))
    core.start(docks, '1', '2', 'dual_motor_gripper', feedback_timeout_s=0.5,
               home_timeout_s=2.0, home_tolerance_rad=0.005,
               unlock_direction=-1, lock_direction=1)
    return core, lock, io, clock, allowed, moves, holds, verifies


def tick(core, lock, clock, moves, follow=True):
    clock[0] += 0.05
    if core.feedback is not None:
        positions = moves[-1] if moves and follow else core.feedback['positions']
        core.observe(positions, dict.fromkeys(positions, 0.0))
    lock.observe()
    core.poll()


def reach(index):
    parts = setup_sequence()
    core, lock, _, clock, _, moves, _, _ = parts
    for _ in range(80):
        if core.index == index:
            return parts
        tick(core, lock, clock, moves)
        assert core.state != 'ABORTED', core.error
    raise AssertionError(index)


def test_complete_sequence_sends_seven_single_targets_and_one_final_confirm():
    core, lock, io, clock, _, moves, holds, verifies = setup_sequence()
    for _ in range(80):
        if core.state == 'DONE':
            break
        assert core.take_confirm() is None
        tick(core, lock, clock, moves)
    assert core.state == 'DONE', core.error
    assert len(moves) == 7 and not holds and len(verifies) == 1
    assert [p['arm_joint_4'] for p in moves] == [0.01, 0.02, 0.03, 0.04, 0.05, 0.06, 0.0]
    confirm = core.take_confirm()
    assert confirm['state'] == 'LOCKED' and confirm['coupling_proxy'] == 'DISABLED'
    assert core.take_confirm() is None


@pytest.mark.parametrize('index', range(11))
@pytest.mark.parametrize('failure', ['operator', 'permission', 'stale'])
def test_abort_at_every_stage_never_emits_next_target_or_confirm(index, failure):
    core, lock, io, clock, allowed, moves, holds, _ = reach(index)
    before = len(moves)
    if failure == 'operator':
        core.abort('operator stop')
    elif failure == 'permission':
        allowed[0] = False
        core.poll()
    else:
        clock[0] += 0.51
        core.poll()
    for _ in range(3):
        core.poll()
    assert core.state == 'ABORTED'
    assert len(moves) == before and core.take_confirm() is None
    assert io.sample['torque'] == 0 and holds


def test_seat_undershoot_times_out_without_unlock():
    core, lock, io, clock, _, moves, holds, _ = reach(1)
    before = list(io.writes)
    for _ in range(60):
        tick(core, lock, clock, moves, follow=False)
        if not core.running:
            break
    assert core.state == 'ABORTED'
    assert 'timed out' in core.error
    assert not any(field == 'goal' for field, _ in io.writes[len(before):])
    assert len(moves) == 2 and holds and core.take_confirm() is None


def test_moving_at_target_is_not_arrival():
    core, lock, _, clock, _, moves, _, _ = reach(1)
    core.observe(moves[-1], dict.fromkeys(core.names, 0.01))
    core.poll()
    assert core.state == 'TO_RELEASE_SEAT'


def test_old_feedback_and_action_success_cannot_advance():
    core, lock, _, clock, _, moves, _, _ = reach(0)
    # 목표가 이미 허용오차 안이어도, 발행 전 샘플을 도착 근거로 쓰지 않는다.
    core.feedback['positions'] = moves[-1].copy()
    core.poll()
    assert core.state == 'TO_RELEASE_APPROACH' and len(moves) == 1


@pytest.mark.parametrize('index', [3, 8])
def test_post_seat_drift_prevents_lock_write(index):
    core, lock, io, _, _, moves, _, _ = reach(index)
    positions = core.feedback['positions'].copy()
    positions['arm_joint_4'] += 0.2
    core.observe(positions, dict.fromkeys(positions, 0.0))
    before = list(io.writes)
    core.poll()
    assert core.state == 'ABORTED'
    assert not any(field == 'goal' for field, _ in io.writes[len(before):])
    assert core.take_confirm() is None


def test_one_time_tool_verification_failure_stops_before_retreat():
    core, lock, _, clock, _, moves, _, _ = reach(8)
    def fail():
        raise RuntimeError('new tool preparation failed')
    core.verify_tool = fail
    before = len(moves)
    for _ in range(5):
        tick(core, lock, clock, moves)
        if core.state == 'ABORTED':
            break
    assert core.state == 'ABORTED' and len(moves) == before
    assert core.take_confirm() is None
