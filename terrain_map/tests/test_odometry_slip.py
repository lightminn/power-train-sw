"""Wheel consensus against spinning wheels (spec §4.4 r13)."""
import dataclasses
import math

import numpy as np
import pytest

from terrain_map import odometry as O
from terrain_map.config import MapConfig
from terrain_map.pose import PoseTrack
from terrain_map.tests.odometry_sim import LAYOUT, Command, feed, simulate, wheels_for

OFF = MapConfig()
ON = dataclasses.replace(OFF, max_slip_wheels=2)
START = 100.0


def readings(cmd, *, speeds=None, valid=None):
    rows, *_ = wheels_for(cmd)
    out = []
    for n, s, d, c, m in rows:
        s = speeds.get(n, s) if speeds else s
        out.append(O.WheelReading(n, s, d, c, s / (2 * math.pi * LAYOUT.wheel_radius_m), True if valid is None else n in valid))
    return out


def straight(v=0.3, **speeds):
    return readings(Command("skid", v=v), speeds=speeds)


CHAOS = dict(front_left=0.0, front_right=0.25, mid_left=0.5, mid_right=0.75, rear_left=1.0, rear_right=1.25)


# --- the solver ---------------------------------------------------------------
def test_defaults_keep_the_consensus_off():
    assert OFF.max_slip_wheels == 0 and OFF.slip_residual_m_s == 0.10 and OFF.slip_epoch_s == 1.0


def test_one_spinning_wheel_is_left_out():
    sol = O.solve_body_velocity(straight(front_left=3.0), LAYOUT, 0.0, ON)
    assert sol.ok and sol.vx == pytest.approx(0.3, abs=1e-9) and sol.excluded == ("front_left",)
    off = O.solve_body_velocity(straight(front_left=3.0), LAYOUT, 0.0, OFF)
    assert off.ok and off.vx == pytest.approx(0.75, abs=1e-9) and off.excluded == ()   # least squares: negative control


def test_two_spinning_wheels_are_left_out():
    sol = O.solve_body_velocity(straight(mid_right=2.0, rear_left=-1.0), LAYOUT, 0.0, ON)
    assert sol.ok and sol.vx == pytest.approx(0.3, abs=1e-9) and sol.excluded == ("mid_right", "rear_left")


@pytest.mark.parametrize("cmd", [Command("ackermann", v=0.5, steering=1.0), Command("ackermann", v=0.5, steering=-0.4),
                                 Command("skid", v=0.5, omega=0.5), Command("skid", v=0.4)])
def test_turns_without_slip_never_enter_the_consensus(cmd):
    _, _, _, omega = wheels_for(cmd)
    rows = readings(cmd)
    assert O.solve_body_velocity(rows, LAYOUT, omega, ON) == O.solve_body_velocity(rows, LAYOUT, omega, OFF)


def test_a_spin_that_breaks_the_lateral_limit_is_still_recovered():
    cmd = Command("ackermann", v=0.5, steering=1.0)
    _, vx, vy, omega = wheels_for(cmd)
    rows = readings(cmd, speeds={"front_left": 3.0})
    off = O.solve_body_velocity(rows, LAYOUT, omega, OFF)
    assert not off.ok and off.reason == "WHEELS_STALE"                 # |v_y| ≈ 1.61 m/s: rejected whole
    sol = O.solve_body_velocity(rows, LAYOUT, omega, ON)
    assert sol.ok and sol.excluded == ("front_left",)
    assert sol.vx == pytest.approx(vx, abs=1e-9) and sol.vy == pytest.approx(vy, abs=1e-9)
    assert vy == pytest.approx(0.0391, abs=1e-4)


def test_no_consensus_gives_a_best_effort_speed_and_an_invalid_pose():
    sol = O.solve_body_velocity(straight(**CHAOS), LAYOUT, 0.0, ON)
    assert not sol.ok and sol.best_effort and sol.reason == "WHEEL_SLIP"
    assert len(sol.excluded) == 2 and math.isfinite(sol.vx)


def test_best_effort_ties_do_not_depend_on_the_input_order():
    rows = straight(**CHAOS)
    a = O.solve_body_velocity(rows, LAYOUT, 0.0, ON)
    b = O.solve_body_velocity(list(reversed(rows)), LAYOUT, 0.0, ON)
    assert (a.vx, a.vy, a.excluded) == (b.vx, b.vy, b.excluded)
    # three candidates tie at max residual 0.375: the one nearest the all-wheel solution (0.625) wins,
    # not the alphabetically first pair (0.875)
    assert a.excluded == ("front_left", "rear_right") and a.vx == pytest.approx(0.625)


def test_consensus_ties_pick_the_alphabetically_first_wheel():
    rows = straight(v=0.3, front_left=0.5, rear_right=0.1)              # either wheel alone does not settle it
    a = O.solve_body_velocity(rows, LAYOUT, 0.0, ON)
    b = O.solve_body_velocity(list(reversed(rows)), LAYOUT, 0.0, ON)
    assert a == b and a.ok and a.excluded == ("front_left", "rear_right")


def test_never_drops_below_the_minimum_valid_wheels():
    valid = {"front_left", "front_right", "mid_left", "mid_right", "rear_left"}       # five valid, min four
    rows = readings(Command("skid", v=0.3), speeds={"front_left": 3.0, "mid_left": 2.0}, valid=valid)
    sol = O.solve_body_velocity(rows, LAYOUT, 0.0, ON)
    assert not sol.ok and sol.best_effort and len(sol.excluded) == 1


def test_structural_failure_is_not_searched():
    rows = readings(Command("skid", v=0.3), speeds={"front_left": 3.0}, valid={"front_left", "mid_left", "rear_left"})
    sol = O.solve_body_velocity(rows, LAYOUT, 0.0, ON)
    assert not sol.ok and not sol.best_effort and sol.reason == "WHEELS_STALE"


def test_condition_number_boundary_is_deterministic():
    cmd = Command("ackermann", v=0.5, steering=0.045)
    _, _, _, omega = wheels_for(cmd)
    rng = np.random.default_rng(3)
    rows = [r._replace(speed_m_s=r.speed_m_s + rng.uniform(-0.02, 0.02)) for r in readings(cmd, speeds={"mid_left": 2.0})]
    a = O.solve_body_velocity(rows, LAYOUT, omega, ON)
    b = O.solve_body_velocity(list(reversed(rows)), LAYOUT, omega, ON)
    assert a == b and a.ok and a.excluded == ("mid_left",) and a.vx == pytest.approx(0.5, abs=0.03)


def test_residual_tolerance_is_inclusive():
    rows = straight(v=0.3, front_left=0.3 + 0.12)      # mean shifts by 0.02: residual 0.10 on that wheel
    sol = O.solve_body_velocity(rows, LAYOUT, 0.0, ON)
    assert sol.ok and sol.excluded == ()
    tighter = O.solve_body_velocity(rows, LAYOUT, 0.0, dataclasses.replace(ON, slip_residual_m_s=0.099))
    assert tighter.ok and tighter.excluded == ("front_left",)


# --- dead reckoning -------------------------------------------------------------
def stop_then(motion, stop_s=1.5):
    return lambda t: Command("stop") if t < stop_s else motion


def spin(name, t0, t1, speed):
    return lambda t, n, s: speed if (n == name and t0 <= t < t1) else s


def chaos(t0, t1):
    return lambda t, n, s: CHAOS[n] if t0 <= t < t1 else s


def run(cfg, corrupt, seconds=4.0, motion=Command("skid", v=0.3), **kw):
    events, truth = simulate(stop_then(motion), seconds, gyro_bias=0.01, corrupt=corrupt, **kw)
    odo = O.PlanarOdometry(cfg, LAYOUT)
    return odo, feed(odo, events), truth


def final_error(samples, truth):
    s = [p for p in samples if p.valid][-1]
    tx, ty, _ = truth(s.stamp_s)
    return math.hypot(s.x - tx, s.y - ty)


def test_a_wheel_spin_burst_does_not_move_the_map():
    odo, out, truth = run(ON, spin("front_left", 2.5, 2.9, 3.0))
    assert final_error(out, truth) < 0.005 and odo.epoch == 0 and odo.slip_excluded > 0
    assert all(s.valid for s in out if s.stamp_s > START + 2.0)
    _, out_off, _ = run(OFF, spin("front_left", 2.5, 2.9, 3.0))
    assert final_error(out_off, truth) > 0.15                                     # 0.4 s × 2.7 m/s / 6


def test_short_disagreement_integrates_a_best_effort_and_keeps_the_epoch():
    odo, out, truth = run(ON, chaos(2.5, 2.9))
    slip = [s for s in out if s.reason == "WHEEL_SLIP"]
    assert slip and all(not s.valid for s in slip) and odo.epoch == 0 and odo.slip_guessed == len(slip)
    assert {s.stamp_s for s in slip} == {s.stamp_s for s in out if START + 2.5 <= s.stamp_s < START + 2.9}
    assert out[-1].valid and final_error(out, truth) < 0.14          # best effort 0.625 vs 0.3 for 0.4 s: 0.13


def test_a_long_disagreement_starts_a_new_epoch_once_per_slip_epoch():
    odo, out, _ = run(ON, chaos(2.0, 4.3), seconds=5.0)
    assert odo.epoch == 2                                              # 2.3 s of no consensus, 1 s limit
    assert out[-1].valid


def test_a_real_wheel_gap_inside_a_slip_still_counts():
    odo, out, _ = run(ON, chaos(2.0, 3.0), drop_wheels=lambda t: 2.4 < t < 2.8)
    assert odo.epoch == 1 and any(s.reason == "POSE_GAP" or s.epoch == 1 for s in out)


@pytest.mark.parametrize("corrupt, motion, last, n_valid, epoch", [
    # captured from the r12 module (git HEAD before r13) on the same simulated inputs
    (chaos(2.5, 2.9), Command("skid", v=0.3), (0.8650000000000005, 0.0, 0.0), 147, 0),
    (spin("front_left", 2.5, 2.9, 3.0), Command("ackermann", v=0.5, steering=0.6),
     (0.4951078450033413, 0.14894082854197874, 0.4477130263837421), 127, 1),   # r12: |v_y| rejection → gap epoch
])
def test_off_is_the_r12_pose_sequence(corrupt, motion, last, n_valid, epoch):
    odo, out, _ = run(OFF, corrupt, motion=motion)
    assert (out[-1].x, out[-1].y, out[-1].yaw) == pytest.approx(last, abs=1e-12)
    assert sum(s.valid for s in out) == n_valid and len(out) == 198 and odo.epoch == epoch
    assert odo.slip_excluded == odo.slip_guessed == 0
    explicit = dataclasses.replace(OFF, max_slip_wheels=0, slip_residual_m_s=0.5, slip_epoch_s=0.2)
    _, again, _ = run(explicit, corrupt, motion=motion)
    assert [(s.stamp_s, s.x, s.y, s.yaw, s.valid, s.reason) for s in out] == \
           [(s.stamp_s, s.x, s.y, s.yaw, s.valid, s.reason) for s in again]


def test_the_input_order_cannot_flip_the_first_check():
    """Codex r13 round 3: summing in input order put one ordering just over the tolerance."""
    names = ("front_left", "front_right", "mid_left", "mid_right")
    speeds = [0.33333333333333337, 0.2, 0.2, 0.2]
    a = [O.WheelReading(n, v, 0.0, 0.0, 0.0, True) for n, v in zip(names, speeds)]
    b = a[1:] + a[:1]
    cfg = dataclasses.replace(ON, slip_residual_m_s=0.1)
    ra, rb = O.solve_body_velocity(a, LAYOUT, 0.0, cfg), O.solve_body_velocity(b, LAYOUT, 0.0, cfg)
    assert ra == rb


@pytest.mark.parametrize("bias_s, slip_s", [(1.0, 0.5), (1.5, 1.0)])
def test_slip_epochs_wait_for_the_bias(bias_s, slip_s):
    """Codex r13 round 3: a slip epoch clears the bias window; before the bias is known none opens."""
    cfg = dataclasses.replace(ON, bias_stationary_s=bias_s, slip_epoch_s=slip_s)
    events, _ = simulate(lambda t: Command("stop"), 4.0, gyro_bias=0.4)
    odo = O.PlanarOdometry(cfg, LAYOUT)
    feed(odo, events)
    assert odo.bias_ready and odo.epoch == 0


def test_too_few_wheels_inside_a_slip_do_not_reset_its_clock():
    """Codex r13 round 3: one stale sample every 0.5 s must not keep a long no-consensus run alive."""
    stale = lambda t, n: n in ("front_left", "front_right", "mid_left") and 2.0 <= t < 5.8 and round(t * 50) % 25 == 0
    odo, out, _ = run(ON, chaos(2.0, 5.8), seconds=6.5, stale=stale)
    assert any(s.reason == "WHEELS_STALE" for s in out) and odo.epoch >= 3


def test_disagreeing_wheels_at_a_stop_are_not_stationary():
    odo, out, _ = run(ON, chaos(0.0, 4.0), motion=Command("stop"))
    assert not odo.bias_ready and all(s.reason == "BIAS_PENDING" for s in out)


@pytest.mark.parametrize("bias, tol", [(0.4, 0.10), (0.12, 0.03)])
def test_a_large_unlearnt_bias_does_not_deadlock_learning(bias, tol):
    """Codex r13 round 2: before the bias is known, omega carries it and the consensus fails at rest."""
    cfg = dataclasses.replace(ON, slip_residual_m_s=tol)
    rows = readings(Command("stop"))
    assert O.solve_body_velocity(rows, LAYOUT, bias, cfg).best_effort          # the trap is real
    events, _ = simulate(lambda t: Command("stop"), 3.0, gyro_bias=bias)
    odo = O.PlanarOdometry(cfg, LAYOUT)
    feed(odo, events)
    assert odo.bias_ready and odo.bias == pytest.approx(bias, abs=1e-6)


def test_an_excluded_wheel_that_moves_still_blocks_bias_learning():
    odo, _, _ = run(ON, spin("rear_right", 0.0, 4.0, 0.5), motion=Command("stop"))
    assert not odo.bias_ready                                          # stationary test uses every valid wheel


def test_pose_track_does_not_interpolate_across_a_slip_sample():
    track = PoseTrack(OFF)
    for k, (valid, reason) in enumerate([(True, None), (False, "WHEEL_SLIP"), (True, None)]):
        track.add(O.PoseSample(10.0 + 0.02 * k, 0.01 * k, 0.0, 0.0, valid, reason, 0))
    assert track.pose_at(10.01) == (None, "WHEEL_SLIP")
    assert track.pose_at(10.03) == (None, "WHEEL_SLIP")


def test_followed_decisions_reproduce_a_slip_epoch():
    # turning, so opening the epoch after the yaw knot instead of before it would show in the replay
    events, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.4)), 5.0, gyro_bias=0.01, corrupt=chaos(2.0, 3.6))
    records = []
    live = O.PlanarOdometry(ON, LAYOUT, record=records.append)
    live_out = feed(live, events)
    assert live.epoch == 1 and any(r["k"] == "epoch" and r["cause"] == "slip" for r in records)
    follow = O.PlanarOdometry(ON, LAYOUT, follow=O.follow_plan(records))
    stamp_ordered = sorted(events, key=lambda e: (e[1].stamp_s, 0 if isinstance(e[1], O.WheelSample) else 1))
    follow_out = feed(follow, [(s.stamp_s, s) for _, s in stamp_ordered]) + follow.drain()
    key = lambda xs: [(s.stamp_s, s.x, s.y, s.yaw, s.valid, s.reason, s.epoch) for s in xs]
    assert key(follow_out[:len(live_out)]) == key(live_out) and follow.epoch == live.epoch


def test_a_slip_that_expires_inside_a_wheel_shortage_still_starts_an_epoch():
    """Codex r13 round 4: SLIP 2.0–3.0 s, three stale wheels 3.0–3.2 s, then recovery — 1.2 s without a solution."""
    stale = lambda t, n: n in ("front_left", "front_right", "mid_left") and 3.0 <= t < 3.2
    odo, out, _ = run(ON, chaos(2.0, 3.0), seconds=4.0, stale=stale)
    assert odo.epoch == 1 and any(r.epoch == 1 and r.valid for r in out)
    short, _, _ = run(ON, chaos(2.0, 2.7), seconds=4.0,
                      stale=lambda t, n: n in ("front_left", "front_right", "mid_left") and 2.7 <= t < 2.9)
    assert short.epoch == 0                                   # 0.9 s in all: under the limit


def test_a_wrong_majority_is_a_known_limit():
    """Four wheels spinning alike outvote the two that grip: worse than least squares (spec §4.4 limits)."""
    rows = straight(v=0.3, front_left=0.8, front_right=0.8, mid_left=0.8, mid_right=0.8)
    on, off = O.solve_body_velocity(rows, LAYOUT, 0.0, ON), O.solve_body_velocity(rows, LAYOUT, 0.0, OFF)
    assert on.ok and on.excluded == ("rear_left", "rear_right") and on.vx == pytest.approx(0.8)
    assert off.vx == pytest.approx(0.6333333, abs=1e-6)


def test_a_slip_epoch_after_a_short_wheel_gap_replays_alike():
    """Codex r13 round 5: no consensus 2.0–2.88 s, wheel messages lost 2.9–3.1 s, recovery at 3.1 s."""
    events, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.4)), 4.0, gyro_bias=0.01,
                         corrupt=chaos(2.0, 2.89), drop_wheels=lambda t: 2.9 <= t < 3.1)
    records = []
    live = O.PlanarOdometry(ON, LAYOUT, record=records.append)
    live_out = feed(live, events)
    assert live.epoch == 1 and any(r["k"] == "epoch" and r["cause"] == "slip" for r in records)
    follow = O.PlanarOdometry(ON, LAYOUT, follow=O.follow_plan(records))
    stamp_ordered = sorted(events, key=lambda e: (e[1].stamp_s, 0 if isinstance(e[1], O.WheelSample) else 1))
    follow_out = feed(follow, [(s.stamp_s, s) for _, s in stamp_ordered]) + follow.drain()
    key = lambda xs: [(s.stamp_s, s.x, s.y, s.yaw, s.valid, s.reason, s.epoch) for s in xs]
    assert key(follow_out[:len(live_out)]) == key(live_out)
