"""영점 전 쓰기 금지, 양방향 인덱싱, 경계/피드백 실패를 검증한다."""

import pytest

from dynamixel_control.toolchanger_lock import LockError, ToolchangerLock


class IO:
    def __init__(self, position=2048):
        self.sample = dict(model=1060, mode=3, drive_mode=0, error=0, position=position,
                           velocity=0, torque=0, profile_velocity=5,
                           profile_acceleration=1)
        self.writes = []
        self.follow = True

    def read(self):
        return dict(self.sample)

    def write(self, field, value):
        self.writes.append((field, value))
        if field == 'goal' and self.follow:
            self.sample['position'] = value
        elif field == 'torque':
            self.sample['torque'] = value


def calibrated(position=2048, state='LOCKED'):
    io = IO(position)
    lock = ToolchangerLock(io)
    lock.capture_zero(state)
    lock.enable()
    return lock, io


def test_uncalibrated_enable_and_move_make_no_writes():
    io = IO()
    lock = ToolchangerLock(io)
    with pytest.raises(LockError):
        lock.enable()
    with pytest.raises(LockError):
        lock.move('UNLOCKED')
    assert not io.writes
    assert not lock.ready


@pytest.mark.parametrize('direction', [-1, 1])
def test_zero_and_two_steps_toggle_in_either_direction(direction):
    lock, io = calibrated()
    assert lock.ready
    lock.move('UNLOCKED', direction)
    assert not lock.ready
    assert io.sample['position'] == 2048 + direction * 256
    lock.observe()
    assert lock.state == 'UNLOCKED'
    lock.move('LOCKED', direction)
    lock.observe()
    assert lock.ready
    assert io.sample['position'] == 2048 + direction * 512


def test_unlocked_zero_is_explicit_and_capture_does_not_write():
    io = IO()
    lock = ToolchangerLock(io)
    lock.capture_zero('UNLOCKED')
    assert lock.state == 'UNLOCKED'
    assert not io.writes
    lock.enable()
    lock.move('LOCKED', -1)
    lock.observe()
    assert lock.ready


@pytest.mark.parametrize('field,value', [('torque', 1), ('velocity', 2), ('mode', 4), ('model', 1070), ('error', 32), ('drive_mode', 4), ('drive_mode', 8)])
def test_invalid_zero_capture_rejected(field, value):
    io = IO()
    io.sample[field] = value
    with pytest.raises(LockError):
        ToolchangerLock(io).capture_zero('LOCKED')
    assert not io.writes


def test_single_turn_boundary_does_not_wrap():
    lock, io = calibrated(0)
    io.writes.clear()
    with pytest.raises(LockError, match='boundary'):
        lock.move('UNLOCKED', -1)
    assert not io.writes
    lock.move('UNLOCKED', 1)
    lock.observe()
    assert io.sample['position'] == 256


def test_timeout_stops_torque_and_blocks_tool():
    lock, io = calibrated()
    clock = [0.0]
    lock.clock = lambda: clock[0]
    io.follow = False
    lock.move('UNLOCKED')
    clock[0] = 6.1
    assert lock.observe() == 'FAULT'
    assert io.sample['torque'] == 0
    assert not lock.ready


def test_unhealthy_feedback_mid_move_stops_torque():
    lock, io = calibrated()
    io.follow = False
    lock.move('UNLOCKED')
    io.sample['error'] = 32
    assert lock.observe() == 'FAULT'
    assert io.writes[-1] == ('torque', 0)
    assert not lock.ready


def test_stale_feedback_and_wrong_index_fail_closed():
    lock, io = calibrated()
    clock = [0.0]
    lock.clock = lambda: clock[0]
    lock.observe()
    assert lock.ready
    clock[0] = 0.6
    assert not lock.ready
    io.sample['position'] += 64
    lock.observe()
    assert lock.state == 'UNKNOWN'
    assert not lock.ready


def test_enabled_motor_with_unbounded_profile_cannot_move():
    lock, io = calibrated()
    io.sample['profile_velocity'] = 0
    before = list(io.writes)
    with pytest.raises(LockError):
        lock.move('UNLOCKED')
    assert io.writes == before


def test_enable_synchronizes_goal_before_torque():
    lock, io = calibrated()
    assert io.writes == [('acceleration', 1), ('velocity_profile', 5),
                         ('goal', 2048), ('torque', 1)]
    assert lock.snapshot()['mechanically_confirmed'] is False


def test_zero_file_roundtrip_and_wrong_motor_rejected(tmp_path):
    lock, _ = calibrated()
    path = tmp_path / 'zero.json'
    lock.save_zero(path)
    assert ToolchangerLock.load_zero(path) == (2048, 'LOCKED')
    path.write_text('{"actuator_id": 2}')
    with pytest.raises(LockError):
        ToolchangerLock.load_zero(path)


def test_enable_seats_only_the_current_calibrated_index():
    io = IO()
    lock = ToolchangerLock(io, zero_tick=2048, zero_state='LOCKED')
    io.sample['position'] = 2046
    lock.enable()
    assert ('goal', 2048) in io.writes
