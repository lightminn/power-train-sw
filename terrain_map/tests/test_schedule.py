"""FrameScheduler (spec §4.8): fixed target, eviction protection, skip vs loss, latency budget."""
import numpy as np
import pytest

from terrain_map import odometry as O
from terrain_map.config import MapConfig
from terrain_map.geometry import Pose2D
from terrain_map.pose import PoseTrack
from terrain_map.schedule import DepthFrame, FrameScheduler
from terrain_map.tests.odometry_sim import LAYOUT, Command, simulate

CFG = MapConfig()
START = 100.0
ARRAY = np.zeros((2, 2), np.uint16)


class FakeTrack:
    """pose_at(t) is available once t <= ready_until."""

    def __init__(self, ready_until=-1.0, reason="WAITING"):
        self.ready_until = ready_until
        self.reason = reason

    def pose_at(self, t):
        if t <= self.ready_until:
            return Pose2D(t, 0.0, 0.0), None
        return None, self.reason


def frame(t, rx=None):
    return DepthFrame(t, t if rx is None else rx, ARRAY)


def run_ticks(sched, track, frames, ticks, ready=lambda now: now):
    """Push frames by rx and select at each tick; return processed stamps and losses."""
    frames = sorted(frames, key=lambda f: f.rx_s)
    done, lost, i = [], [], 0
    for now in ticks:
        while i < len(frames) and frames[i].rx_s <= now:
            sched.push(frames[i])
            i += 1
        track.ready_until = ready(now)
        sel, gone = sched.select(track, now)
        lost += gone
        if sel is not None:
            done.append(sel.frame.stamp_s)
    return done, lost


def tick_times(t0, t1, hz=10.0, phase=0.0):
    return list(np.arange(t0 + phase, t1, 1.0 / hz))


def test_target_is_spaced_by_the_process_rate_and_older_frames_are_skipped():
    sched = FrameScheduler(CFG)
    frames = [frame(START + k * 0.1) for k in range(20)]
    done, lost = run_ticks(sched, FakeTrack(), frames, tick_times(START, START + 2.05))
    assert not lost
    assert all(b - a >= 0.9 / CFG.process_hz - 1e-9 for a, b in zip(done, done[1:]))
    assert len(done) == 10 and sched.skipped > 0 and sched.lost == 0


def test_realtime_and_stamp_order_replay_pick_the_same_frames():
    frames = [frame(START + k * 0.1, START + k * 0.1 + 0.03) for k in range(30)]   # 30 ms arrival delay
    live = run_ticks(FrameScheduler(CFG), FakeTrack(), frames, tick_times(START, START + 3.2, phase=0.037))[0]
    replay = FrameScheduler(CFG)
    picked = []
    for f in frames:                                       # replay: stamp order, select right after each push
        replay.push(f._replace(rx_s=f.stamp_s))
        sel, _ = replay.select(FakeTrack(ready_until=f.stamp_s), f.stamp_s)
        if sel is not None:
            picked.append(sel.frame.stamp_s)
    assert live == picked


def test_waiting_target_is_never_evicted_by_new_frames():
    """Codex r3 P1-5: pose 250 ms late at 10 Hz depth — the target must survive and be processed."""
    sched = FrameScheduler(CFG)
    frames = [frame(START + k * 0.1) for k in range(40)]
    done, lost = run_ticks(sched, FakeTrack(), frames, tick_times(START, START + 4.0, phase=0.01),
                           ready=lambda now: now - 0.25)
    assert not lost and len(done) >= 15
    assert sched.pending_count <= CFG.depth_pending_max


def test_a_burst_of_frames_evicts_others_but_not_the_waiting_target():
    sched = FrameScheduler(CFG)
    sched.push(frame(START))
    assert sched.select(FakeTrack(), START + 0.05) == (None, [])          # START is now the target
    for k in range(1, 8):                                                 # burst beyond depth_pending_max
        sched.push(frame(START + k * 0.1, START + 0.06))
    assert sched.pending_count == CFG.depth_pending_max
    sel, lost = sched.select(FakeTrack(ready_until=START), START + 0.1)
    assert sel is not None and sel.frame.stamp_s == START and lost == []


def test_expiry_counts_as_loss_and_other_pose_failures_too():
    sched = FrameScheduler(CFG)
    sched.push(frame(START))
    assert sched.select(FakeTrack(), START + 0.39) == (None, [])
    sel, lost = sched.select(FakeTrack(), START + 0.41)
    assert sel is None and lost == [(START, "POSE_EXPIRED")] and sched.lost == 1
    sched.push(frame(START + 0.5))
    sel, lost = sched.select(FakeTrack(reason="BIAS_PENDING"), START + 0.5)
    assert sel is None and lost == [(START + 0.5, "BIAS_PENDING")] and sched.lost == 2


def test_pose_arriving_on_the_expiry_tick_is_processed():
    sched = FrameScheduler(CFG)
    sched.push(frame(START))
    sel, lost = sched.select(FakeTrack(ready_until=START), START + 0.45)
    assert sel is not None and sel.frame.stamp_s == START and lost == []


def test_clear_drops_pending_and_the_target():
    sched = FrameScheduler(CFG)
    for k in range(3):
        sched.push(frame(START + k * 0.1))
    sched.select(FakeTrack(), START + 0.2)
    sched.clear()
    assert sched.pending_count == 0 and sched.select(FakeTrack(ready_until=START + 1), START + 0.3) == (None, [])


def test_follow_processes_exactly_the_recorded_frames():
    records = []
    live = FrameScheduler(CFG, record=records.append)
    frames = [frame(START + k * 0.1, START + k * 0.1 + 0.03) for k in range(20)]
    done, _ = run_ticks(live, FakeTrack(), frames, tick_times(START, START + 2.2, phase=0.061),
                        ready=lambda now: now - 0.1)
    follow = FrameScheduler(CFG, follow=frozenset((r["seg"], r["t"], r["d"], r["seq"]) for r in records
                                                  if r["r"] == "processed"))
    picked = []
    for f in frames:
        follow.push(f._replace(rx_s=f.stamp_s))
        sel, _ = follow.select(FakeTrack(ready_until=f.stamp_s), f.stamp_s)
        if sel is not None:
            picked.append(sel.frame.stamp_s)
    assert picked == done


# --- full path: wheel arrival delay -> watermark -> PoseTrack -> scheduler ------
def full_path(delay_s, phase):
    events, _ = simulate(lambda t: Command("stop") if t < 1.5 else Command("skid", v=0.3), 4.0,
                         wheel_delay_s=delay_s)
    odo, track, sched = O.PlanarOdometry(CFG, LAYOUT), PoseTrack(CFG), FrameScheduler(CFG)
    depth = [frame(START + 2.0 + k * 0.1) for k in range(12)]           # 2.0 .. 3.1 s, on time
    inputs = sorted([(rx, 0, s) for rx, s in events] + [(f.rx_s, 1, f) for f in depth], key=lambda e: e[:2])
    ticks = tick_times(START + 2.0, START + 4.0, hz=2 * CFG.process_hz, phase=phase)
    done, lost, i = [], [], 0
    for now in ticks:
        while i < len(inputs) and inputs[i][0] <= now:
            _, kind, item = inputs[i]
            i += 1
            if kind == 1:
                sched.push(item)
                continue
            samples = odo.add_wheels(item) if isinstance(item, O.WheelSample) else odo.add_gyro(item)
            for s in samples + odo.poll(item.rx_s):
                track.add(s)
        sel, gone = sched.select(track, now)
        lost += gone
        if sel is not None:
            done.append(sel.frame.stamp_s)
    return done, lost


@pytest.mark.parametrize("delay_s", [0.0, 0.2, 0.25])
@pytest.mark.parametrize("phase", [0.0, 0.025, 0.05, 0.075])
def test_input_delay_within_budget_is_processed_at_every_tick_phase(delay_s, phase):
    done, lost = full_path(delay_s, phase)
    assert lost == [] and len(done) == 6


def test_input_delay_300ms_is_still_processed():
    for phase in (0.0, 0.05):                              # processed, phase-dependent (not a guarantee)
        done, _ = full_path(0.30, phase)
        assert done


def test_input_delay_450ms_expires():
    done, lost = full_path(0.45, 0.0)
    assert done == [] and lost and all(why == "POSE_EXPIRED" for _, why in lost)
