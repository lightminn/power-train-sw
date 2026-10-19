"""HallPLL must see a continuous wheel angle although the articulation reports it wrapped.

Isaac reports a revolute wheel joint wrapped into [-2pi, 2pi) (observed +6.23 -> -6.26 rad).
A real Hall sensor never sees that jump; reading it as motion gave a false -14 m/s wheel speed.
Pure model test: numpy only, no Isaac.  Run: python -m pytest simulation/jetin_rover_description/tests
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tools'))
import manufacturer_models as mm  # noqa: E402

R, V, DT = 0.10375, 0.15, 1 / 120          # wheel radius, ground speed, PhysX step


def wrapped(theta):
    return (theta + 2 * math.pi) % (4 * math.pi) - 2 * math.pi


def pll():
    return mm.HallPLL(cpr=60, ratio=5, bandwidth=30)


def drive(model, angle, seconds=12.0, theta0=5.5):
    return [model.update(angle(theta0 + V / R * k * DT), k * DT) for k in range(int(seconds / DT))]


def test_wrapped_input_keeps_true_speed():
    out = drive(pll(), wrapped)
    speeds = [r['velocity_rad_s'] * R for r in out[240:]]
    assert sum(1 for k in range(1, len(out)) if wrapped(5.5 + V / R * k * DT) < wrapped(5.5 + V / R * (k - 1) * DT)) >= 1
    assert all(abs(v - V) < 0.03 for v in speeds), (min(speeds), max(speeds))


def test_wrapped_and_continuous_inputs_agree():
    a = drive(pll(), wrapped)
    b = drive(pll(), lambda x: x)
    assert max(abs(x['angle_rad'] - y['angle_rad']) for x, y in zip(a, b)) < 1e-9
    assert max(abs(x['velocity_rad_s'] - y['velocity_rad_s']) for x, y in zip(a, b)) < 1e-6
    assert a[-1]['angle_rad'] > 2 * math.pi            # output is the multi-turn angle, like pos_estimate


def test_wrap_in_reverse():
    out = drive(pll(), lambda x: wrapped(-x), theta0=-5.5)
    assert all(abs(r['velocity_rad_s'] * R + V) < 0.03 for r in out[240:])


def test_fast_real_motion_is_not_unwrapped():
    w = 2.5 / DT / 2                                    # 1.25 rad per step: real motion below pi
    model = pll()
    out = [model.update(wrapped(w * k * DT), k * DT) for k in range(240)]
    assert abs(out[-1]['velocity_rad_s'] - w) < 0.02 * w


def test_unwrapped_input_unchanged_from_archived_model():
    model = pll()
    out = [model.update(0.3 + V / R * k * DT + 0.02 * math.sin(k * 0.07), k * DT) for k in range(600)]
    golden = {59: (0.9935881682428301, 1.3082154482859012, 47),
              299: (3.920248423346485, 1.4418503596699448, 187),
              599: (7.498574535725919, 1.317593336804081, 358)}
    for k, (angle, vel, count) in golden.items():
        assert (out[k]['angle_rad'], out[k]['velocity_rad_s'], out[k]['hall_count']) == (angle, vel, count)


def test_restart_after_reset_and_time_regression():
    model = pll()
    drive(model, wrapped, seconds=8.0)                  # past a wrap, so an offset is held
    model.reset()
    r = model.update(wrapped(1.0), 0.0)
    assert r['velocity_rad_s'] == 0.0 and abs(r['angle_rad'] - 1.0) < 1e-9
    model.update(wrapped(1.0 + 7.0), 1.0)
    r = model.update(wrapped(-3.0), 0.5)                # clock ran back: restart at the raw angle
    assert r['velocity_rad_s'] == 0.0 and abs(r['angle_rad'] + 3.0) < 1e-9
