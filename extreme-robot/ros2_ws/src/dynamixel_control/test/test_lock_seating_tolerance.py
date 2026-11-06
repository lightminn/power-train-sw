"""A coupling that seats short of the taught tick still counts as locked."""

from dynamixel_control.toolchanger_lock import ToolchangerLock


class Io:
    def __init__(self, position):
        self.position, self.velocity, self.pwm, self.goal_pwm = position, 0, 0, 800
        self.torque = 1
        self.writes = []

    def read(self):
        return dict(model=ToolchangerLock.model_number, mode=3, drive_mode=0, error=0,
                    position=self.position, velocity=self.velocity, torque=self.torque,
                    pwm=self.pwm, goal_pwm=self.goal_pwm, profile_velocity=5,
                    profile_acceleration=1)

    def write(self, field, value):
        self.writes.append((field, value))
        if field == 'torque':
            self.torque = value
        elif field == 'goal_pwm':
            self.goal_pwm = value


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


def capped(position, pwm=0):
    target = lock(position)
    target.goal_pwm_limit = 800
    target.io.pwm = pwm
    return target


def test_reaching_the_cap_counts_as_locked_and_stops_pushing():
    target = capped(2920, pwm=785)
    target.io.velocity = 2
    target.target, target.deadline = 3064, 10.0
    assert target.observe() == 'LOCKED'
    assert target.seated and target.target is None
    assert target.io.writes == [('goal', 2920)]


def test_below_the_cap_it_keeps_moving():
    target = capped(2920, pwm=700)
    target.io.velocity = 2
    target.target, target.deadline = 3064, 10.0
    assert target.observe() == 'MOVING'


def test_a_seated_lock_stays_locked_while_it_settles_back():
    target = capped(2920, pwm=785)
    target.target, target.deadline = 3064, 10.0
    target.observe()
    target.io.position, target.io.pwm = 2960, 300
    assert target.observe() == 'LOCKED'


def test_a_seated_lock_without_torque_is_judged_by_position_again():
    target = capped(2920, pwm=785)
    target.target, target.deadline = 3064, 10.0
    target.observe()
    target.io.torque = 0
    assert target.observe() == 'BETWEEN'


def test_a_rebooted_servo_gets_the_cap_back_before_moving():
    target = capped(2850)
    target.io.goal_pwm = 885
    target.observe()
    assert target._pwm_cap_missing()
    target.enable()
    assert target.io.writes[:2] == [('torque', 0), ('goal_pwm', 800)]
    assert target.io.torque == 1 and not target._pwm_cap_missing()


def test_a_pwm_spike_at_the_start_of_the_travel_is_not_a_seat():
    target = capped(2850, pwm=790)
    target.target, target.deadline = 3064, 10.0
    assert target.observe() == 'MOVING'


def test_entering_the_window_finishes_at_once_without_pushing_on():
    target = capped(2971)
    target.locked_tolerance = 94
    target.io.velocity = 4
    target.target, target.deadline = 3064, 10.0
    assert target.observe() == 'LOCKED'
    assert target.io.writes == [('goal', 2971)]
