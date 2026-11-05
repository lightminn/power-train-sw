"""A coupling that seats short of the taught tick still counts as locked."""

from dynamixel_control.toolchanger_lock import ToolchangerLock


class Io:
    def __init__(self, position):
        self.position, self.velocity = position, 0

    def read(self):
        return dict(model=ToolchangerLock.model_number, mode=3, drive_mode=0, error=0,
                    position=self.position, velocity=self.velocity, torque=1)

    def write(self, *_args):
        pass


def lock(position, tolerance=75):
    target = ToolchangerLock(Io(position), clock=lambda: 0.0)
    target.two_point = True
    target.locked_tick, target.unlocked_tick = 3064, 2837
    target.locked_tolerance = tolerance
    return target


def test_a_stall_inside_the_window_finishes_the_move_as_locked():
    target = lock(2999)
    target.target, target.deadline = 3064, 10.0
    assert target.observe() == 'LOCKED'
    assert target.target is None


def test_it_is_not_finished_while_still_moving():
    target = lock(2999)
    target.io.velocity = 3
    target.target, target.deadline = 3064, 10.0
    assert target.observe() == 'MOVING'


def test_a_relaxed_coupling_is_not_locked():
    assert lock(2941).observe() == 'BETWEEN'


def test_unlocked_keeps_the_narrow_window():
    assert lock(2880).observe() == 'BETWEEN'
    assert lock(2850).observe() == 'UNLOCKED'


def test_without_the_setting_the_window_is_unchanged():
    assert lock(2999, tolerance=None).observe() == 'BETWEEN'
