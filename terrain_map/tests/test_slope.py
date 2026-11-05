"""Ground-plane tilt display (spec r11): plane angles, still-window gravity."""
import math

import numpy as np
import pytest

from terrain_map import slope as SL
from terrain_map.config import MapConfig

CFG = MapConfig()
G = SL.G


def up_pitched(deg):
    """Gravity 'up' seen from a body whose nose is raised by ``deg``."""
    a = math.radians(deg)
    return np.array([math.sin(a), 0.0, math.cos(a)])


def normal_rising_ahead(deg):
    a = math.radians(deg)
    return np.array([-math.sin(a), 0.0, math.cos(a)])         # plane z = tan(a)·x, normal points up


def test_plane_angles_signs_and_degeneracy():
    p, r, t = SL.plane_tilt(normal_rising_ahead(10.0), np.array([0.0, 0.0, 1.0]))
    assert p == pytest.approx(10.0) and r == pytest.approx(0.0, abs=1e-9) and t == pytest.approx(10.0)
    left_high = np.array([0.0, -math.sin(math.radians(5)), math.cos(math.radians(5))])
    assert SL.plane_tilt(left_high, np.array([0.0, 0.0, 1.0]))[1] == pytest.approx(5.0)
    assert SL.plane_tilt(np.array([0.0, 0.0, -1.0]), np.array([0.0, 0.0, 1.0])) is None    # n·u ≤ 0
    assert SL.plane_tilt(np.array([0.0, 0.0, 1.0]), np.array([1.0, 0.0, 0.0])) is None     # heading ∥ up


def test_body_on_a_slope_reads_the_slope_against_gravity_and_zero_against_the_body():
    n_b = np.array([0.0, 0.0, 1.0])                            # plane parallel to the body
    up = up_pitched(10.0)                                       # body nose up 10°
    assert SL.plane_tilt(n_b, np.array([0.0, 0.0, 1.0]))[2] == pytest.approx(0.0)
    assert SL.plane_tilt(n_b, up)[0] == pytest.approx(10.0)
    pitch, roll = SL.body_tilt(up)
    assert pitch == pytest.approx(10.0) and roll == pytest.approx(0.0, abs=1e-9)


def feed(still, t0, t1, *, accel=(0.0, 0.0, G), gyro=0.0, wheels_still=True, hz=100.0, wheel_hz=50.0):
    for k in range(int(round((t1 - t0) * hz)) + 1):
        t = t0 + k / hz
        still.add_accel(t, np.array(accel))
        still.add_gyro(t, gyro)
    for k in range(int(round((t1 - t0) * wheel_hz)) + 1):
        still.add_wheels(t0 + k / wheel_hz, wheels_still)


def test_still_window_gives_gravity():
    s = SL.StillGravity(CFG)
    feed(s, 10.0, 11.0)
    up = s.up_at(10.9)
    assert up is not None and up == pytest.approx([0.0, 0.0, 1.0])


@pytest.mark.parametrize("change", [
    dict(accel=(1.0, 0.0, G), wheels_still=False),            # Codex: steady 1 m/s² while driving
    dict(wheels_still=False),                                  # cruising: IMU quiet, wheels turning
    dict(gyro=0.2),                                            # turning in place
    dict(accel=(0.0, 0.0, 0.7 * G)),                           # not 1 g
])
def test_motion_or_doubt_gives_no_gravity(change):
    s = SL.StillGravity(CFG)
    feed(s, 10.0, 11.0, **change)
    assert s.up_at(10.9) is None


def test_noisy_accel_and_uncovered_windows_give_no_gravity():
    s = SL.StillGravity(CFG)
    rng = np.random.default_rng(0)
    for k in range(101):
        t = 10.0 + k / 100
        s.add_accel(t, np.array([0.0, 0.0, G]) + rng.normal(0.0, 0.5, 3))
        s.add_gyro(t, 0.0)
    for k in range(51):
        s.add_wheels(10.0 + k / 50, True)
    assert s.up_at(10.9) is None                                # vibration
    s = SL.StillGravity(CFG)
    feed(s, 10.0, 10.2)                                         # only 0.2 s before t
    assert s.up_at(10.2) is None


def test_only_samples_up_to_the_depth_stamp_are_used():
    s = SL.StillGravity(CFG)
    feed(s, 10.0, 10.3, wheels_still=False)                     # moving up to 10.3
    feed(s, 10.31, 11.0)                                        # still afterwards
    assert s.up_at(10.4) is None                                # its window [9.9, 10.4] holds motion
    assert s.up_at(10.9) is not None


def test_wheel_gap_and_clock_reset_and_clear():
    s = SL.StillGravity(CFG)
    for k in range(101):
        t = 10.0 + k / 100
        s.add_accel(t, np.array([0.0, 0.0, G]))
        s.add_gyro(t, 0.0)
    for k in range(51):
        if not 0.5 < k / 50 < 0.7:                              # a 0.2 s hole in the wheel stream
            s.add_wheels(10.0 + k / 50, True)
    assert s.up_at(10.9) is None
    s = SL.StillGravity(CFG)
    feed(s, 100.0, 101.0)
    feed(s, 10.0, 10.3)                                          # clock restart drops the old samples
    assert s.up_at(100.9) is None and s.up_at(10.3) is None
    s = SL.StillGravity(CFG)
    feed(s, 10.0, 11.0)
    s.clear()
    assert s.up_at(10.9) is None


def test_new_parameters_are_range_checked():
    with pytest.raises(ValueError):
        MapConfig(still_window_s=5.0)
    with pytest.raises(ValueError):
        MapConfig(gravity_tolerance=0.5)
