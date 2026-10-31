"""Planar dead reckoning (spec §4.4) on wheel messages from the real kinematics."""
import math

import numpy as np
import pytest

from terrain_map import odometry as O
from terrain_map.config import MapConfig
from terrain_map.tests.odometry_sim import LAYOUT, Command, feed, simulate, wheels_for

CFG = MapConfig()
START = 100.0


def odo(cfg=CFG):
    return O.PlanarOdometry(cfg, LAYOUT)


def stop_then(motion, stop_s=1.5):
    return lambda t: Command("stop") if t < stop_s else motion


def last_valid(samples, epoch=None):
    good = [s for s in samples if s.valid and (epoch is None or s.epoch == epoch)]
    assert good, "no valid pose sample"
    return good[-1]


def error_at(sample, truth_at):
    tx, ty, tyaw = truth_at(sample.stamp_s)
    return math.hypot(sample.x - tx, sample.y - ty), abs(math.degrees(O.wrap(sample.yaw - tyaw)))


# --- body velocity least squares -------------------------------------------
def readings(cmd, valid=None):
    rows, *_ = wheels_for(cmd)
    return [O.WheelReading(n, s, d, c, m, True if valid is None else n in valid) for n, s, d, c, m in rows]


def test_ackermann_turn_velocity_is_exact():
    cmd = Command("ackermann", v=0.5, steering=1.0)
    _, vx, vy, omega = wheels_for(cmd)
    sol = O.solve_body_velocity(readings(cmd), LAYOUT, omega, CFG)
    assert sol.ok and sol.vx == pytest.approx(0.5, abs=1e-6) and sol.vy == pytest.approx(vy, abs=1e-6)
    mean = np.mean([r.speed_m_s for r in readings(cmd)])
    assert mean == pytest.approx(0.55458, abs=1e-4)          # what a plain average would say


def test_skid_asymmetric_subset_is_unbiased():
    cmd = Command("skid", v=0.5, omega=0.5)
    keep = {"front_right", "mid_right", "rear_left", "rear_right"}
    sol = O.solve_body_velocity(readings(cmd, keep), LAYOUT, 0.5, CFG)
    assert sol.ok and sol.vx == pytest.approx(0.5, abs=1e-6) and sol.vy == 0.0


def test_straight_line_is_rank_one_and_fixes_vy_to_zero():
    sol = O.solve_body_velocity(readings(Command("skid", v=0.4)), LAYOUT, 0.0, CFG)
    assert sol.ok and sol.vy == 0.0 and sol.vx == pytest.approx(0.4, abs=1e-6)


def test_ill_conditioned_small_steering_does_not_invent_lateral_speed():
    cmd = Command("ackermann", v=0.5, steering=0.001)
    _, _, _, omega = wheels_for(cmd)
    rng = np.random.default_rng(0)
    noisy = [r._replace(speed_m_s=r.speed_m_s + rng.uniform(-0.001, 0.001)) for r in readings(cmd)]
    sol = O.solve_body_velocity(noisy, LAYOUT, omega, CFG)
    assert sol.ok and sol.vy == 0.0 and sol.vx == pytest.approx(0.5, abs=0.002)


def test_too_few_valid_wheels_is_invalid():
    sol = O.solve_body_velocity(readings(Command("skid", v=0.4), {"front_left", "mid_left", "rear_left"}),
                                LAYOUT, 0.0, CFG)
    assert not sol.ok and sol.reason == "WHEELS_STALE"


def test_drive_fresh_but_steer_stale_wheels_count_as_invalid():
    rows = readings(Command("ackermann", v=0.5, steering=0.5))
    stale = [r._replace(valid=False) if r.name.startswith(("front", "rear")) else r for r in rows]
    assert not O.solve_body_velocity(stale, LAYOUT, 0.3, CFG).ok


# --- integration ------------------------------------------------------------
@pytest.mark.parametrize("motion, seconds", [
    (Command("ackermann", v=0.5, steering=1.0), 2.42),       # about 90 degrees
    (Command("skid", v=0.5, omega=0.5), 3.14),                # 90 degrees
    (Command("skid", v=0.4), 3.0),
])
def test_turns_track_the_truth(motion, seconds):
    events, truth = simulate(stop_then(motion), 1.5 + seconds, gyro_bias=0.01)
    sample = last_valid(feed(odo(), events))
    pos, yaw = error_at(sample, truth)
    assert pos < 0.02 and yaw < 0.5


SKID_KEEP = {"front_right", "mid_right", "rear_left", "rear_right"}


def mean_speed(wheel_readings, layout, omega, cfg):
    speeds = [r.speed_m_s for r in wheel_readings if r.valid]
    return O.BodyVelocity(float(np.mean(speeds)), 0.0, True, None)


def test_skid_turn_on_an_asymmetric_wheel_subset(monkeypatch):
    run = lambda: simulate(stop_then(Command("skid", v=0.5, omega=0.5)), 1.5 + 3.14,
                           stale=lambda t, name: t >= 1.5 and name not in SKID_KEEP)
    events, truth = run()
    pos, _ = error_at(last_valid(feed(odo(), events)), truth)
    assert pos < 0.02
    monkeypatch.setattr(O, "solve_body_velocity", mean_speed)
    pos, _ = error_at(last_valid(feed(odo(), events)), truth)
    assert pos > 0.10                                         # spec: 22.3 cm with a plain average


def test_plain_average_speed_negative_control(monkeypatch):
    monkeypatch.setattr(O, "solve_body_velocity", mean_speed)
    events, truth = simulate(stop_then(Command("ackermann", v=0.5, steering=1.0)), 1.5 + 2.42)
    pos, _ = error_at(last_valid(feed(odo(), events)), truth)
    assert pos > 0.10


def test_late_wheel_stream_still_tracks_the_turn():
    events, truth = simulate(stop_then(Command("skid", v=0.5, omega=0.5)), 3.5, wheel_delay_s=0.2)
    pos, yaw = error_at(last_valid(feed(odo(), events)), truth)
    assert pos < 0.02 and yaw < 0.5
    # Negative control: stamping each wheel message by its arrival pairs it with the wrong instant.
    by_arrival = [(rx, s._replace(stamp_s=s.rx_s) if isinstance(s, O.WheelSample) else s) for rx, s in events]
    pos, _ = error_at(last_valid(feed(odo(), by_arrival)), truth)
    assert pos > 0.05


def test_same_stamp_order_does_not_change_the_result():
    events, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.2)), 2.5)
    swapped = sorted(events, key=lambda e: (e[0], 1 if isinstance(e[1], O.WheelSample) else 0))
    a = [(s.stamp_s, s.x, s.y, s.yaw, s.valid) for s in feed(odo(), events)]
    b = [(s.stamp_s, s.x, s.y, s.yaw, s.valid) for s in feed(odo(), swapped)]
    assert a == b


def test_gyro_sign_counter_clockwise_increases_yaw_and_flip_is_caught():
    events, truth = simulate(stop_then(Command("skid", v=0.0, omega=0.4)), 3.0)
    sample = last_valid(feed(odo(), events))
    assert sample.yaw > 0.4
    flipped = [(rx, s._replace(wz=-s.wz) if isinstance(s, O.GyroSample) else s) for rx, s in events]
    assert last_valid(feed(odo(), flipped)).yaw < -0.4


# --- gaps, skew, regression -------------------------------------------------
def test_gyro_gap_with_hidden_rotation_starts_a_new_epoch():
    def rate(t):
        if 2.0 <= t <= 2.2:
            return 0.5 * (1.0 - abs(t - 2.1) / 0.1)              # triangle, zero at both ends
        return 0.0
    events, _ = simulate(stop_then(Command("stop")), 3.0, gyro_rate=rate,
                         drop_gyro=lambda t: 2.0 < t < 2.2)
    out = feed(odo(), events)
    assert {s.epoch for s in out} == {0, 1}
    assert all(abs(s.yaw) < 1e-6 for s in out if s.epoch == 1)


def test_short_gyro_jitter_keeps_the_epoch():
    events, _ = simulate(stop_then(Command("skid", v=0.3)), 3.0, drop_gyro=lambda t: 2.0 < t < 2.05)
    assert {s.epoch for s in feed(odo(), events)} == {0}


def test_short_wheel_gap_marks_samples_invalid_but_keeps_the_epoch():
    events, _ = simulate(stop_then(Command("skid", v=0.3)), 3.0, drop_wheels=lambda t: 2.0 < t < 2.1)
    out = feed(odo(), events)
    assert {s.epoch for s in out} == {0}
    after_gap = [s for s in out if 2.1 + START <= s.stamp_s <= 2.11 + START]
    assert after_gap and not after_gap[0].valid and after_gap[0].reason == "POSE_GAP"


def test_long_wheel_gap_starts_a_new_epoch():
    events, _ = simulate(stop_then(Command("skid", v=0.3)), 3.5, drop_wheels=lambda t: 2.0 < t < 2.5)
    assert {s.epoch for s in feed(odo(), events)} == {0, 1}


def test_long_run_of_too_few_valid_wheels_is_a_gap_too():
    events, _ = simulate(stop_then(Command("skid", v=0.3)), 3.5, stale=lambda t, name: 2.0 < t < 2.5)
    out = feed(odo(), events)
    assert {s.epoch for s in out} == {0, 1}
    assert all(not s.valid for s in out if 2.0 + START < s.stamp_s < 2.5 + START)


def test_wheel_gap_while_braking_matches_the_extrapolation_formula():
    def schedule(t):
        if t < 1.5:
            return Command("stop")
        if t < 2.0:
            return Command("skid", v=0.6)
        return Command("skid", v=max(0.0, 0.6 - 2.0 * (t - 2.0)))     # 2 m/s^2 braking
    gap = 0.28
    events, truth = simulate(schedule, 2.6, drop_wheels=lambda t: 2.0 < t < 2.0 + gap - 1e-9)
    out = feed(odo(), events)
    assert {s.epoch for s in out} == {0}
    end = [s for s in out if abs(s.stamp_s - (START + 2.0 + gap)) < 1e-9]
    assert end and not end[0].valid and end[0].reason == "POSE_GAP"
    pos, _ = error_at(last_valid(out), truth)
    assert pos == pytest.approx(0.5 * 2.0 * gap ** 2, abs=0.01)          # 7.84 cm, conditional on |a|


def test_stalled_wheel_stream_is_bounded_by_skew():
    o = odo()
    out = []
    for i in range(1, 600):                       # 6 s of gyro at 100 Hz, wheels fall behind
        t = START + i * 0.01
        out += o.add_gyro(O.GyroSample(t, t, 0.0))
        if i % 5 == 0:                            # wheel stamps 20 ms apart, received every 25 ms
            k = i // 5
            ws = START + k * 0.02
            rows, *_ = wheels_for(Command("stop"))
            out += o.add_wheels(O.WheelSample(ws, t, tuple(O.WheelReading(n, s, d, c, m, True) for n, s, d, c, m in rows)))
        out += o.poll(t)
        assert o.queue_span_s() <= CFG.max_stream_skew_s + 0.05
    assert o.epoch >= 1


def test_receive_stall_starts_a_new_epoch():
    events, _ = simulate(stop_then(Command("stop")), 2.0)
    o = odo()
    feed(o, events)
    before = o.epoch
    o.poll(START + 2.0 + CFG.wheel_gap_epoch_s + 0.1)
    assert o.epoch == before + 1


def test_large_stamp_regression_starts_a_new_epoch_and_small_one_is_dropped():
    events, _ = simulate(stop_then(Command("stop")), 2.0)
    o = odo()
    feed(o, events)
    rows, *_ = wheels_for(Command("stop"))
    wheels = tuple(O.WheelReading(n, s, d, c, m, True) for n, s, d, c, m in rows)
    late = o.late_dropped
    o.add_wheels(O.WheelSample(START + 1.0, START + 2.1, wheels))       # 1 s back but committed -> late
    assert o.late_dropped == late + 1
    epoch = o.epoch
    o.add_wheels(O.WheelSample(START - 5.0, START + 2.2, wheels))       # > 1 s back -> new epoch
    assert o.epoch == epoch + 1


# --- bias ----------------------------------------------------------------------
def test_bias_pending_until_a_stationary_second_then_exact_mean():
    events, _ = simulate(stop_then(Command("skid", v=0.3), stop_s=1.3), 2.0, gyro_bias=0.02)
    o = odo()
    out = feed(o, events)
    early = [s for s in out if s.stamp_s < START + 0.9]
    assert early and all(not s.valid and s.reason == "BIAS_PENDING" for s in early)
    assert o.bias_ready and o.bias == pytest.approx(0.02, abs=1e-9)


def test_slow_spin_is_not_stationary():
    events, _ = simulate(lambda t: Command("skid", v=0.0, omega=0.03), 3.0)
    o = odo()
    out = feed(o, events)
    assert not o.bias_ready
    assert out and all(not s.valid and s.reason == "BIAS_PENDING" for s in out)


def test_stale_wheels_break_the_stationary_window():
    events, _ = simulate(lambda t: Command("stop"), 1.05, gyro_rate=lambda t: 0.02 if t > 0.8 else 0.0,
                         stale=lambda t, name: t > 0.8)
    o = odo()
    feed(o, events)
    assert not o.bias_ready


# --- Codex Task 4 review ---------------------------------------------------------
def test_reordering_inside_the_margin_changes_nothing():
    """A gyro neighbour beyond the watermark must not decide a gap (Codex T4 P2-1)."""
    # Gyro 1.53-1.55 never exist (true gap 1.52 -> 1.56 = 40 ms); 1.56-1.59 arrive 45 ms late, after 1.60.
    events, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.3)), 2.5,
                         drop_gyro=lambda t: 1.525 < t < 1.555)
    held = []
    for rx, s in events:
        if isinstance(s, O.GyroSample) and START + 1.555 < s.stamp_s < START + 1.595:
            rx = rx + 0.045
            s = s._replace(rx_s=rx)
        held.append((rx, s))
    held.sort(key=lambda e: (e[0], 0 if isinstance(e[1], O.WheelSample) else 1))
    a, b = odo(), odo()
    base = [(s.stamp_s, s.x, s.y, s.yaw, s.valid, s.epoch) for s in feed(a, events)]
    late = [(s.stamp_s, s.x, s.y, s.yaw, s.valid, s.epoch) for s in feed(b, held)]
    assert b.epoch == 0 and b.late_dropped == 0
    assert late == base[:len(late)] and len(base) - len(late) <= 3


@pytest.mark.parametrize("after_stall", [False, True])
def test_one_stream_alone_keeps_the_queue_bounded(after_stall):
    """Codex T4 P2-2: a stream that never arrives must not let the other queue grow."""
    o = odo()
    t0 = START
    if after_stall:
        events, _ = simulate(stop_then(Command("stop")), 1.0)
        feed(o, events)
        t0 = START + 1.0
    for i in range(1, 2001):                                 # 20 s of gyro only
        t = t0 + i * 0.01
        o.add_gyro(O.GyroSample(t, t, 0.0))
        o.poll(t)
        assert o.queue_span_s() <= CFG.max_stream_skew_s + 0.05
    assert len(o._gyro) <= 60


def test_non_finite_inputs_never_reach_the_bias_or_the_pose():
    """Codex T4 P2-3: one NaN gyro sample during the stationary window."""
    events, truth = simulate(stop_then(Command("skid", v=0.3)), 2.5, gyro_bias=0.02)
    poisoned = []
    for rx, s in events:
        if isinstance(s, O.GyroSample) and abs(s.stamp_s - (START + 0.5)) < 1e-9:
            s = s._replace(wz=float("nan"))
        if isinstance(s, O.WheelSample) and abs(s.stamp_s - (START + 2.0)) < 1e-9:
            s = s._replace(wheels=(s.wheels[0]._replace(speed_m_s=float("inf")),) + s.wheels[1:])
        poisoned.append((rx, s))
    o = odo()
    out = feed(o, poisoned)
    assert o.bias_ready and o.bias == pytest.approx(0.02, abs=1e-9)
    assert all(math.isfinite(v) for s in out for v in (s.x, s.y, s.yaw))
    pos, yaw = error_at(last_valid(out), truth)
    assert pos < 0.02 and yaw < 0.5
    assert O.PlanarOdometry(CFG, LAYOUT).add_gyro(O.GyroSample(float("nan"), START, 0.0)) == []


# --- decisions record / follow (spec §4.4 equivalence scope) -------------------
def live_with_arrival_trouble():
    """Turning run; some wheel messages arrive too late and a 0.45 s wheel burst stalls the stream."""
    events, _ = simulate(stop_then(Command("skid", v=0.4, omega=0.4)), 4.0)
    rng = np.random.default_rng(7)
    live = []
    for rx, s in events:
        if isinstance(s, O.WheelSample):
            t = s.stamp_s - START
            if 2.0 < t < 2.2 and rng.random() < 0.3:
                rx += 0.12                                    # behind the watermark: dropped as late
            elif 2.5 <= t <= 2.9:
                rx = START + 2.95                             # burst after a silent stretch: stall epoch
            s = s._replace(rx_s=rx)
        live.append((rx, s))
    live.sort(key=lambda e: (e[0], 0 if isinstance(e[1], O.WheelSample) else 1))
    return number_by_receipt(live)


def number_by_receipt(live):
    """Give each sample its per-stream receive index (what a bag's record order preserves)."""
    seq, numbered = {"g": 0, "w": 0}, []
    for rx, s in live:
        k = "w" if isinstance(s, O.WheelSample) else "g"
        numbered.append((rx, s._replace(seq=seq[k])))
        seq[k] += 1
    return [s for _, s in sorted(numbered, key=lambda e: e[1].stamp_s)], numbered


def stamp_order(samples):
    return sorted(samples, key=lambda s: (s.stamp_s, 0 if isinstance(s, O.WheelSample) else 1, s.seq))


def key(samples):
    return [(s.stamp_s, s.x, s.y, s.yaw, s.valid, s.reason, s.epoch) for s in samples]


def test_followed_decisions_reproduce_the_live_pose_samples():
    events, live = live_with_arrival_trouble()
    records = []
    o = O.PlanarOdometry(CFG, LAYOUT, record=records.append)
    live_out = feed(o, live)
    assert o.late_dropped > 0 and o.epoch >= 1                # the scenario really exercises both
    assert any(r["k"] == "epoch" and r["cause"] == "stall" for r in records)
    plan = O.follow_plan(json_round_trip(records))
    f = O.PlanarOdometry(CFG, LAYOUT, follow=plan)
    follow_out = feed(f, [(s.stamp_s, s) for s in stamp_order(events)]) + f.drain()
    assert key(follow_out[:len(live_out)]) == key(live_out)
    assert f.epoch == o.epoch


def test_without_the_decisions_a_stamp_ordered_replay_differs():
    events, live = live_with_arrival_trouble()
    live_out = feed(odo(), live)
    plain = odo()
    replay_out = feed(plain, [(s.stamp_s, s) for s in stamp_order(events)])
    assert key(replay_out[:len(live_out)]) != key(live_out)


def json_round_trip(records):
    import json
    return [json.loads(json.dumps(r)) for r in records]


# --- Codex Task 4 re-review ---------------------------------------------------------
def stop_wheels():
    rows, *_ = wheels_for(Command("stop"))
    return tuple(O.WheelReading(n, s, d, c, m, True) for n, s, d, c, m in rows)


def test_regression_after_a_stall_flush_starts_a_new_time_base():
    o = odo()
    events, _ = simulate(stop_then(Command("stop")), 2.0)
    feed(o, events)
    o.poll(START + 2.4)                                       # stall flush keeps the committed floor
    epoch = o.epoch
    events, _ = simulate(stop_then(Command("stop")), 2.0, start_s=10.0)
    out = feed(o, events)
    assert o.epoch == epoch + 1 and len([s for s in out if s.valid]) > 50


def test_drain_does_not_emit_wheels_past_the_last_gyro():
    o = odo()
    events, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.3)), 2.2,
                         drop_gyro=lambda t: t > 2.0)
    out = feed(o, events) + o.drain()
    assert out and max(s.stamp_s for s in out) <= START + 2.0 + 1e-9


def test_a_duplicated_message_is_told_apart_from_its_twin_in_the_replay():
    events, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.3)), 3.0,
                         drop_gyro=lambda t: 2.45 < t < 2.55)           # a gyro gap epoch after the duplicate
    live = []
    for rx, s in events:
        live.append((rx, s))
        if isinstance(s, O.WheelSample) and abs(s.stamp_s - (START + 2.0)) < 1e-9:
            live.append((rx + 0.001, s._replace(rx_s=rx + 0.001)))     # the same message delivered twice
    seq = {"g": 0, "w": 0}
    numbered = []
    for rx, s in live:                                                  # receive order = bag record order
        k = "w" if isinstance(s, O.WheelSample) else "g"
        numbered.append((rx, s._replace(seq=seq[k])))
        seq[k] += 1
    records = []
    o = O.PlanarOdometry(CFG, LAYOUT, record=records.append)
    live_out = feed(o, numbered)
    assert o.epoch == 1 and any(r["k"] == "drop" and r["why"] == "duplicate" for r in records)
    f = O.PlanarOdometry(CFG, LAYOUT, follow=O.follow_plan(json_round_trip(records)))
    by_stamp = sorted(numbered, key=lambda e: (e[1].stamp_s, 0 if isinstance(e[1], O.WheelSample) else 1, e[1].seq))
    follow_out = feed(f, [(s.stamp_s, s) for _, s in by_stamp]) + f.drain()
    assert key(follow_out[:len(live_out)]) == key(live_out) and f.epoch == o.epoch


def test_follow_finishes_the_old_time_segment_before_a_regressed_one():
    """Codex T4 re-review 2: drive -> stall flush -> clock restart; segments fed back to back."""
    first, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.3)), 2.5)
    second, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.3)), 2.5, start_s=10.0)
    _, numbered = number_by_receipt(first + second)
    records = []
    o = O.PlanarOdometry(CFG, LAYOUT, record=records.append)
    live_out = []
    for rx, s in numbered[:len(first)]:
        live_out += (o.add_wheels(s) if isinstance(s, O.WheelSample) else o.add_gyro(s)) + o.poll(rx)
    live_out += o.poll(START + 2.9)                                    # stall flush
    for rx, s in numbered[len(first):]:                               # receive clock keeps running (+93 s)
        s = s._replace(rx_s=rx + 93.0)
        live_out += (o.add_wheels(s) if isinstance(s, O.WheelSample) else o.add_gyro(s)) + o.poll(s.rx_s)
    assert o.epoch == 2 and any(s.valid and s.epoch == 2 for s in live_out)
    f = O.PlanarOdometry(CFG, LAYOUT, follow=O.follow_plan(json_round_trip(records)))
    segments = [stamp_order(s for _, s in numbered[:len(first)]), stamp_order(s for _, s in numbered[len(first):])]
    follow_out = feed(f, [(s.stamp_s, s) for seg in segments for s in seg]) + f.drain()
    assert key(follow_out[:len(live_out)]) == key(live_out) and f.epoch == o.epoch


def test_identities_survive_a_clock_restart_that_repeats_old_stamps():
    """Codex T7 re-review P2-1: >512 stamps, then the sensor clock restarts and repeats them."""
    first, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.2)), 6.4)
    again, _ = simulate(stop_then(Command("skid", v=0.3, omega=0.2)), 3.0)          # same stamps from 100.0
    records = []
    o = O.PlanarOdometry(CFG, LAYOUT, record=records.append)
    live_out = []
    for rx, s in first:
        live_out += (o.add_wheels(s) if isinstance(s, O.WheelSample) else o.add_gyro(s)) + o.poll(rx)
    for rx, s in again:
        s = s._replace(rx_s=rx + 10.0)
        live_out += (o.add_wheels(s) if isinstance(s, O.WheelSample) else o.add_gyro(s)) + o.poll(s.rx_s)
    assert o.epoch >= 1 and any(s.valid and s.epoch == o.epoch for s in live_out)
    f = O.PlanarOdometry(CFG, LAYOUT, follow=O.follow_plan(json_round_trip(records)))
    feed_order = stamp_order(s for _, s in first) + stamp_order(s for _, s in again)
    follow_out = feed(f, [(s.stamp_s, s) for s in feed_order]) + f.drain()
    assert key(follow_out[:len(live_out)]) == key(live_out) and f.epoch == o.epoch
