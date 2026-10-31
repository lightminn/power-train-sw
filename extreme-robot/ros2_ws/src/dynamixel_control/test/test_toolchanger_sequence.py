"""교체 시퀀스의 confirm 1회 발행과 모든 실패 경로의 fail-closed 중단을 검증한다."""

import pytest

from dynamixel_control.toolchanger_lock import ToolchangerLock
from dynamixel_control.toolchanger_sequence import SequenceError, ToolchangerSequence

from test_toolchanger_lock import IO


class LoadIO(IO):
    def __init__(self, position=2048, load=0):
        super().__init__(position)
        self.sample['load'] = load


def build(load=0, threshold=0, state='LOCKED', tool_ready=True):
    io = LoadIO(load=load)
    lock = ToolchangerLock(io)
    lock.capture_zero(state)
    lock.enable()
    stops = []

    def prepare_tool():
        stops.append('prepare')
        if not tool_ready:
            raise RuntimeError('selected tool torque OFF is not confirmed')

    sequence = ToolchangerSequence(lock, prepare_tool, load_threshold=threshold)
    return sequence, lock, io, stops


def tick(sequence, lock):
    """브리지 타이머와 같은 순서: observe 뒤에 poll."""
    lock.observe()
    return sequence.poll()


def test_disabled_proxy_change_confirms_once_without_mechanical_claim():
    sequence, lock, io, _ = build()
    assert sequence.start('UNLOCKED', -1) == 'MOVING'
    assert tick(sequence, lock) == 'CONFIRMED'
    assert io.sample['position'] == 1792
    confirm = sequence.take_confirm()
    assert confirm['state'] == 'UNLOCKED'
    assert confirm['coupling_proxy'] == 'DISABLED'
    assert confirm['mechanically_confirmed'] is False
    assert sequence.take_confirm() is None


def test_tool_stop_failure_rejects_before_any_lock_write():
    sequence, lock, io, _ = build(tool_ready=False)
    before = list(io.writes)
    with pytest.raises(SequenceError, match='torque OFF'):
        sequence.start('UNLOCKED', -1)
    assert io.writes == before
    assert sequence.state == 'IDLE'
    assert lock.ready


def test_lock_below_load_threshold_aborts_with_torque_off_and_no_confirm():
    sequence, lock, io, _ = build(load=30, threshold=120, state='UNLOCKED')
    assert sequence.start('LOCKED', 1) == 'MOVING'
    assert tick(sequence, lock) == 'ABORTED'
    assert sequence.coupling_proxy == 'NOT_ENGAGED'
    assert sequence.take_confirm() is None
    assert io.writes[-1] == ('torque', 0)
    assert not lock.ready


def test_lock_above_load_threshold_confirms_engaged():
    sequence, lock, io, _ = build(load=150, threshold=120, state='UNLOCKED')
    sequence.start('LOCKED', 1)
    assert tick(sequence, lock) == 'CONFIRMED'
    assert sequence.peak_load == 150
    assert sequence.take_confirm()['coupling_proxy'] == 'ENGAGED'


def test_unlock_is_not_gated_by_coupling_load():
    sequence, lock, _, _ = build(load=0, threshold=120)
    sequence.start('UNLOCKED', -1)
    assert tick(sequence, lock) == 'CONFIRMED'
    assert sequence.coupling_proxy == 'RELEASED'


def test_feedback_fault_during_move_aborts_without_confirm():
    sequence, lock, io, _ = build()
    sequence.start('UNLOCKED', -1)
    io.sample['error'] = 1
    assert tick(sequence, lock) == 'ABORTED'
    assert 'lock feedback fault' in sequence.error
    assert sequence.take_confirm() is None
    assert io.writes[-1] == ('torque', 0)


def test_already_in_desired_state_reports_unchanged_without_goal_write():
    sequence, lock, io, _ = build(state='LOCKED')
    before = list(io.writes)
    assert sequence.start('LOCKED', 1) == 'UNCHANGED'
    assert io.writes == before
    assert sequence.take_confirm() is None
    assert lock.ready


def test_second_change_is_rejected_while_moving():
    sequence, lock, _, stops = build()
    sequence.start('UNLOCKED', -1)
    with pytest.raises(SequenceError, match='already running'):
        sequence.start('LOCKED', 1)
    assert stops == ['prepare']


def test_abort_closes_an_in_flight_change_for_emergency_stop():
    sequence, lock, io, _ = build()
    sequence.start('UNLOCKED', -1)
    assert sequence.abort('emergency stop') == 'ABORTED'
    assert io.writes[-1] == ('torque', 0)
    assert sequence.abort('emergency stop') == 'ABORTED'
