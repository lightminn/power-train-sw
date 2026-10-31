"""MapRuntime (spec §4.8, §5.3): status, fusion gate, epochs, sending, decisions."""
from functools import lru_cache
import json
import math

import numpy as np
import pytest

from terrain_overlay.camera import CameraModel
from terrain_overlay.runtime import SenderSession
from terrain_map import contract as C
from terrain_map import odometry as O
from terrain_map.config import MapConfig
from terrain_map.geometry import IDENTITY, base_to_optical
from terrain_map.runtime import MapRuntime, load_decisions
from terrain_map.tests.odometry_sim import LAYOUT, Command, simulate
from terrain_map.tests.world_render import BasePose, Mount, camera_in_world, flat, render_depth, with_box, with_ditch

CAM = CameraModel(fx=115.0, fy=115.0, cx=80.0, cy=60.0, width=160, height=120)
CFG = MapConfig(pixel_stride=1)
START = 100.0
SESSION = SenderSession(boot_id="0" * 32, start_mono_ns=1, instance="a" * 16)
R = base_to_optical(CFG.camera_pitch_down_deg, CFG.camera_roll_deg, CFG.camera_yaw_deg)
SURFACE = with_box(flat(), 2.4, 2.7, -0.2, 0.2, 0.15)


class Clock:
    def __init__(self, now=START):
        self.now = now

    def __call__(self):
        return self.now


def runtime(clock, **kw):
    return MapRuntime(CFG, SESSION, LAYOUT, unix_ns=lambda: 1_700_000_000_000_000_000, clock=clock, **kw)


@lru_cache(maxsize=None)
def scenario(duration=3.2, surface_key="box"):
    surface = {"box": SURFACE, "ditch": None}[surface_key]
    events, truth = simulate(lambda t: Command("stop") if t < 1.5 else Command("skid", v=0.3, omega=0.1), duration)
    depth = []
    for k in range(16, int(duration * 10)):                  # depth from 1.6 s at 10 Hz
        x, y, yaw = truth(START + k / 10)
        depth.append((START + k / 10, render_depth(CAM, BasePose(x=x, y=y, yaw_deg=math.degrees(yaw)), Mount(), surface)))
    return tuple(events), tuple(depth)


def deliver(rt, kind, item, rx=None):
    if kind == "depth":
        stamp, array = item
        rt.on_depth(array, stamp, stamp + 0.02 if rx is None else rx)
    elif isinstance(item, O.GyroSample):
        rt.on_gyro(item.stamp_s, item.rx_s, tuple(R @ np.array([0.0, 0.0, item.wz])), item.seq)
    else:
        rt.on_wheels(item.stamp_s, item.rx_s, item.wheels, item.seq)


def drive(rt, clock, events, depth, *, until=None, depth_rx=0.02, send=True):
    """Live-like loop: inputs by receive time, process at 2*process_hz, send at status_hz."""
    inputs = sorted([(rx, 0, s) for rx, s in events] + [(st + depth_rx, 1, (st, a)) for st, a in depth],
                    key=lambda e: e[:2])
    end = until if until is not None else inputs[-1][0] + 0.1      # stop before a receive-stall epoch
    packets, i, k = [], 0, 0
    for now in np.arange(START, end, 1.0 / (2 * CFG.process_hz)):
        while i < len(inputs) and inputs[i][0] <= now:
            rx, kind, item = inputs[i]
            deliver(rt, "depth" if kind else "odo", item, rx)
            i += 1
        clock.now = now
        rt.process_tick()
        if send and k % 2 == 0:
            packets += rt.send_tick()
        k += 1
    return packets


def statuses(packets):
    return [p for p in map(C.parse_packet, packets) if isinstance(p, C.StatusPacket)]


def tiles(packets):
    return [p for p in map(C.parse_packet, packets) if isinstance(p, C.TilePacket)]


# --- status priority --------------------------------------------------------------
def status_now(rt, clock):
    return statuses(rt.send_tick())[0]


def test_status_priority_chain():
    clock = Clock()
    rt = runtime(clock)
    s = status_now(rt, clock)
    assert (s.status, s.reason, s.map_epoch, s.pose) == ("SENSOR_ERROR", "CAMERA_INFO_MISSING", 0, None)
    rt.on_camera_info(CAM)
    assert status_now(rt, clock).reason == "DEPTH_STALE"
    rt.on_bad_depth(clock.now)
    assert status_now(rt, clock).reason == "DEPTH_ENCODING"
    rt.on_depth(np.zeros((10, 10), np.uint16), START, clock.now + 0.01)
    clock.now += 0.02
    assert status_now(rt, clock).reason == "DEPTH_SHAPE_MISMATCH"
    rt.on_depth(np.zeros((CAM.height, CAM.width), np.uint16), START + 0.1, clock.now)
    rt.process_tick()
    s = status_now(rt, clock)
    assert (s.status, s.reason) == ("NO_POSE", "ODOM_MISSING")


def test_bias_pending_then_ok_then_ground_lost():
    events, depth = scenario()
    clock = Clock()
    rt = runtime(clock)
    rt.on_camera_info(CAM)
    early = [(rx, s) for rx, s in events if s.stamp_s < START + 0.95]
    packets = drive(rt, clock, early, [(START + k / 10, depth[0][1]) for k in range(2, 9)], until=START + 0.95)
    assert statuses(packets)[-1].reason == "BIAS_PENDING"
    clock = Clock()
    rt = runtime(clock)
    rt.on_camera_info(CAM)
    packets = drive(rt, clock, events, depth)
    ok = [s for s in statuses(packets) if s.status == "OK"]
    assert ok and ok[-1].pose is not None and ok[-1].distance_m > 0.2 and ok[-1].tile_count > 0
    assert ok[-1].thresholds == C.Thresholds(0.08, 0.08, 2) and ok[-1].corrector.name == "identity"
    rt.on_depth(np.zeros((CAM.height, CAM.width), np.uint16), START + 9.0, clock.now + 0.01)
    for _ in range(12):                                        # 1.2 s of floorless frames
        clock.now += 0.1
        rt.on_depth(np.zeros((CAM.height, CAM.width), np.uint16), clock.now, clock.now)
        rt.process_tick()
    assert status_now(rt, clock).status in ("NO_POSE", "NO_GROUND")


def test_not_support_frames_never_reach_the_map():
    fx = float(camera_in_world(BasePose(), Mount())[0][0])
    ditch = render_depth(CAM, BasePose(), Mount(), with_ditch(flat(), fx + 0.8, fx + 3.2, -1.2, 1.2, 0.20))   # plane = ditch bottom at 160x120
    events, _ = scenario()
    clock = Clock()
    rt = runtime(clock)
    rt.on_camera_info(CAM)
    still = [(rx, s) for rx, s in events if s.stamp_s < START + 1.5]          # parked: pose stays at the origin
    frames = [(START + k / 10, ditch) for k in range(11, 15)]
    packets = drive(rt, clock, still, frames, until=START + 1.6)
    last = statuses(packets)[-1]
    assert (last.status, last.reason) == ("NO_GROUND", "NOT_SUPPORT")
    assert last.not_support >= 2 and last.tile_count == 0 and tiles(packets) == []


# --- epochs ---------------------------------------------------------------------
def test_epoch_from_poll_alone_clears_everything_in_that_tick():
    events, depth = scenario()
    clock = Clock()
    rt = runtime(clock)
    rt.on_camera_info(CAM)
    packets = drive(rt, clock, events, depth)
    assert rt.map.tiles() and rt.odometry.bias_ready
    before = statuses(packets)[-1]
    rt.on_depth(depth[-1][1], depth[-1][0] + 0.1, clock.now)   # something pending
    clock.now += CFG.wheel_gap_epoch_s + 0.1                    # nothing arrives: receive stall
    rt.process_tick()
    assert rt.epoch == before.map_epoch + 1
    assert rt.map.tiles() == [] and rt.scheduler.pending_count == 0 and rt.track.pose_at(depth[-1][0])[0] is None
    assert rt.odometry.bias_ready and rt.delta == IDENTITY
    out = rt.send_tick()
    s = statuses(out)[0]
    assert s.map_epoch == rt.epoch and s.tile_count == 0 and s.distance_m == 0.0 and s.sequence > before.sequence
    assert before.status == "OK" and (s.status, s.reason, s.pose) == ("NO_POSE", "POSE_GAP", None)  # not OK on a wiped map


# --- sending --------------------------------------------------------------------
def test_send_order_limits_and_tile_seq():
    events, depth = scenario()
    clock = Clock()
    rt = runtime(clock)
    rt.on_camera_info(CAM)
    packets = drive(rt, clock, events, depth, send=False)
    assert len(rt.map.tiles()) >= 3
    first = rt.send_tick()
    assert all(len(p) <= C.MAX_PACKET_BYTES for p in first)
    parsed = [C.parse_packet(p) for p in first]
    assert isinstance(parsed[0], C.StatusPacket)
    sent = parsed[1:]
    changed = [t for t in sent[:CFG.max_tiles_per_tick] if t.version]
    assert [t.version for t in changed] == sorted(t.version for t in changed)
    assert len(sent) <= CFG.max_tiles_per_tick + CFG.refresh_tiles_per_tick
    seqs = [t.tile_seq for t in sent] + [t.tile_seq for t in tiles(rt.send_tick())]
    assert seqs == sorted(seqs) and len(set(seqs)) == len(seqs) and seqs[0] == 1
    s = parsed[0]
    cycle = math.ceil(s.tile_count / CFG.refresh_tiles_per_tick) / CFG.status_hz
    assert s.refresh_cycle_s == pytest.approx(cycle)


def test_refresh_eventually_resends_every_tile():
    events, depth = scenario()
    clock = Clock()
    rt = runtime(clock)
    rt.on_camera_info(CAM)
    drive(rt, clock, events, depth, send=False)
    rt.send_tick()                                            # drain the changed queue
    seen = set()
    for _ in range(math.ceil(len(rt.map.tiles()) / CFG.refresh_tiles_per_tick) + 1):
        seen |= {(t.tx, t.ty) for t in tiles(rt.send_tick())}
    assert seen == set(rt.map.tiles())


def test_processing_overrun_is_logged_separately():
    events, depth = scenario()
    clock = Clock()
    overruns, errors = [], []
    rt = runtime(clock, log_overrun=overruns.append, log_error=errors.append)
    rt.on_camera_info(CAM)
    slow = rt._detector.process

    def process(depth_array, rx_s):
        clock.now += 0.15                                     # longer than one 0.1 s processing tick
        return slow(depth_array, rx_s)
    rt._detector.process = process
    drive(rt, clock, events, depth[:3], until=START + 2.2)
    assert overruns and not errors and "exceeds" in overruns[0]
    assert "processed" in rt.processing_summary()


# --- decisions: live record -> deterministic replay ------------------------------------
def live_trouble(events):
    """Late wheel messages (dropped live) and a wheel burst after a silent stretch (stall epoch live)."""
    rng = np.random.default_rng(11)
    out = []
    for rx, s in events:
        if isinstance(s, O.WheelSample):
            t = s.stamp_s - START
            if 2.0 < t < 2.3 and rng.random() < 0.3:
                rx += 0.12
            elif 2.5 <= t <= 2.9:
                rx = START + 2.95
            s = s._replace(rx_s=rx)
        out.append((rx, s))
    return out


def replay(events, depth, follow=None):
    """Deterministic replay: every input in stamp order, the clock is the stamp."""
    clock = Clock()
    rt = runtime(clock, follow=follow)
    rt.on_camera_info(CAM)
    inputs = sorted([(s.stamp_s, 0, s) for s in events] + [(st, 1, (st, a)) for st, a in depth],
                    key=lambda e: (e[0], e[1], 0 if isinstance(e[2], O.WheelSample) else 1))
    for stamp, kind, item in inputs:
        clock.now = stamp
        deliver(rt, "depth" if kind else "odo", item, rx=stamp)
        rt.process_tick()
    rt.finish()
    return rt


def map_content(rt, tx, ty):
    """Version, heights and codes — not the age bits, which follow the receive clock."""
    heights, codes, _ = C.unpack_cells(rt.map.encode_tile(tx, ty, START)["cells"], CFG.tile_cells)
    return rt.map.version_of(tx, ty), heights, codes


def same_map(a, b):
    return (a.epoch == b.epoch and sorted(a.map.tiles()) == sorted(b.map.tiles())
            and all(map_content(a, *k) == map_content(b, *k) for k in a.map.tiles()))


def test_recorded_decisions_make_the_replay_map_identical():
    events, depth = scenario()
    live_events = live_trouble(events)
    records = []
    clock = Clock()
    live = runtime(clock, decisions=records.append)
    live.on_camera_info(CAM)
    drive(live, clock, live_events, depth)
    assert live.odometry.late_dropped > 0 and live.epoch >= 1 and live.map.tiles()
    assert {r["k"] for r in records} >= {"drop", "epoch", "frame", "result"}
    plan = load_decisions(json.loads(json.dumps(r)) for r in records)

    samples = [s for _, s in live_events]                  # identity = (stream, stamp, copy): no renumbering
    followed = replay(samples, depth, follow=plan)
    assert same_map(followed, live) and followed.scheduler.processed == live.scheduler.processed
    assert not same_map(replay(samples, depth), live)       # negative control: without the decisions


# --- Codex Task 6 review ---------------------------------------------------------------
def test_map_rebuilds_after_a_clock_restart():
    """Codex T6 P2-1: the scheduler's last attempt must not outlive the epoch."""
    events, depth = scenario()
    clock = Clock()
    rt = runtime(clock)
    rt.on_camera_info(CAM)
    drive(rt, clock, events, depth)
    processed = rt.scheduler.processed
    shift = -90.0                                              # sensor clock restarts at ~10 s
    later = [(rx + 5.0, s._replace(stamp_s=s.stamp_s + shift, rx_s=s.rx_s + 5.0)) for rx, s in events]
    moved = [(st + shift, a) for st, a in depth]
    inputs = sorted([(rx, 0, s) for rx, s in later] + [(st - shift + 5.0 + 0.02, 1, (st, a)) for st, a in moved],
                    key=lambda e: e[:2])
    i = 0
    for now in np.arange(START + 5.0, inputs[-1][0] + 0.1, 0.1):
        while i < len(inputs) and inputs[i][0] <= now:
            rx, kind, item = inputs[i]
            deliver(rt, "depth" if kind else "odo", item, rx)
            i += 1
        clock.now = now
        rt.process_tick()
    assert rt.epoch >= 1 and rt.scheduler.processed > processed and rt.map.tiles()


def test_tile_age_uses_the_receive_clock_not_the_header_stamp():
    """Codex T6 P2-2: Unix header stamps, monotonic receive clock."""
    events, depth = scenario()
    unix = 1_700_000_000.0 - START
    shifted = [(rx, s._replace(stamp_s=s.stamp_s + unix)) for rx, s in events]
    frames = [(st + unix, a) for st, a in depth]
    clock = Clock()
    rt = runtime(clock)
    rt.on_camera_info(CAM)
    inputs = sorted([(rx, 0, s) for rx, s in shifted] + [(st - unix + 0.02, 1, (st, a)) for st, a in frames],
                    key=lambda e: e[:2])
    i = 0
    for now in np.arange(START, inputs[-1][0] + 0.1, 0.1):
        while i < len(inputs) and inputs[i][0] <= now:
            rx, kind, item = inputs[i]
            deliver(rt, "depth" if kind else "odo", item, rx)
            i += 1
        clock.now = now
        rt.process_tick()
    assert rt.map.tiles()
    last_frame_rx = depth[-1][0] + 0.02
    clock.now = last_frame_rx + 9.7
    ages = []
    for tx, ty in rt.map.tiles():
        _, codes, age = C.unpack_cells(rt.map.encode_tile(tx, ty, clock.now)["cells"], CFG.tile_cells)
        ages += [a for c, a in zip(codes, age) if c != C.INSUFFICIENT]
    assert ages and min(ages) == 4                              # floor(9.7 / 2.0) for the newest cells


def test_the_replay_check_sees_a_different_camera_model():
    """Branch review 3: live processed some frames with an updated CameraInfo, the replay did not."""
    from terrain_map.runtime import unreproduced
    events, depth = scenario()
    records = []
    clock = Clock()
    live = runtime(clock, decisions=records.append)
    live.on_camera_info(CAM)
    half = len(depth) // 2
    drive(live, clock, events, depth[:half], until=depth[half][0])
    live.on_camera_info(CameraModel(fx=CAM.fx * 1.01, fy=CAM.fy * 1.01, cx=CAM.cx, cy=CAM.cy,
                                    width=CAM.width, height=CAM.height))
    rest = [(rx, s) for rx, s in events if rx >= depth[half][0]]
    drive(live, clock, rest, depth[half:])
    plan = load_decisions(json.loads(json.dumps(r)) for r in records)
    followed = replay([s for _, s in events], depth, follow=plan)        # the replay only ever saw CAM
    assert unreproduced(plan, followed.followed_results)


def test_the_decisions_log_carries_the_live_config_and_wheel_geometry():
    from dataclasses import asdict
    records = []
    runtime(Clock(), decisions=records.append)
    header = records[0]
    assert header["k"] == "session" and header["config"] == asdict(CFG)
    assert header["layout"]["radius"] == LAYOUT.wheel_radius_m and set(header["layout"]["positions"]) == set(LAYOUT.positions)
    plan = load_decisions(json.loads(json.dumps(r)) for r in records)
    assert plan.config == asdict(CFG) and plan.layout == json.loads(json.dumps(header["layout"]))


# --- ground-plane tilt (r11) ---------------------------------------------------------
@lru_cache(maxsize=None)
def still_then_drive():
    """Parked until 2.5 s, then driving; depth from 1.1 s; accelerometer = gravity in the optical frame."""
    schedule = lambda t: Command("stop") if t < 2.5 else Command("skid", v=0.3)   # noqa: E731
    events, truth = simulate(schedule, 3.6)
    depth = []
    for k in range(11, 36):
        x, y, yaw = truth(START + k / 10)
        depth.append((START + k / 10, render_depth(CAM, BasePose(x=x, y=y, yaw_deg=math.degrees(yaw)), Mount(), SURFACE)))
    up_opt = R @ np.array([0.0, 0.0, 9.80665])
    accel = [(START + k / 100, tuple(up_opt)) for k in range(0, 361)]
    return tuple(events), tuple(depth), tuple(accel)


def drive_with_accel(rt, clock, events, depth, accel, until=None):
    inputs = sorted([(rx, 0, s) for rx, s in events] + [(st + 0.02, 1, (st, a)) for st, a in depth]
                    + [(st, 2, (st, a)) for st, a in accel], key=lambda e: e[:2])
    end = until if until is not None else inputs[-1][0] + 0.1
    packets, results, i, k = [], [], 0, 0
    for now in np.arange(START, end, 0.1):
        while i < len(inputs) and inputs[i][0] <= now:
            rx, kind, item = inputs[i]
            if kind == 2:
                rt.on_accel(item[0], rx, item[1])
            else:
                deliver(rt, "depth" if kind else "odo", item, rx)
            i += 1
        clock.now = now
        rt.process_tick()
        if k % 2 == 0:
            packets += rt.send_tick()
        k += 1
    return packets


def test_tilt_from_still_frames_and_none_while_driving():
    events, depth, accel = still_then_drive()
    records = []
    clock = Clock()
    rt = runtime(clock, decisions=records.append)
    rt.on_camera_info(CAM)
    packets = drive_with_accel(rt, clock, events, depth, accel)
    results = [r for r in records if r.get("k") == "result" and r["r"] == "processed"]
    still = [r for r in results if r["t"] < START + 2.45]
    moving = [r for r in results if r["t"] > START + 2.55]
    assert still and moving
    assert all(r["gravity"] is not None for r in still if r["t"] >= START + 1.6)    # a full still window
    assert all(r["gravity"] is None for r in moving)
    for r in still:
        assert abs(r["plane_tilt"][0]) < 1.0 and abs(r["plane_tilt"][1]) < 1.0       # flat floor, body level
        if r["gravity"] is not None:
            assert abs(r["gravity"]["slope"][2]) < 1.0 and abs(r["gravity"]["body_tilt"][0]) < 0.5
    s = [p for p in statuses(packets) if p.plane_tilt is not None]
    assert s and any(p.gravity is not None for p in s) and s[-1].gravity is None    # last frames were moving


def test_tilt_expires_and_never_crosses_an_epoch():
    events, depth, accel = still_then_drive()
    clock = Clock()
    rt = runtime(clock)
    rt.on_camera_info(CAM)
    drive_with_accel(rt, clock, events, [d for d in depth if d[0] < START + 2.4], accel, until=START + 2.45)
    fresh = statuses(rt.send_tick())[0]
    assert fresh.plane_tilt is not None and fresh.gravity is not None
    clock.now += CFG.slope_max_age_s + 0.1
    assert statuses(rt.send_tick())[0].plane_tilt is None                          # expired
    rt.process_tick()                                                              # the silence is a stall epoch
    assert rt.epoch >= 1 and rt._still.up_at(START + 2.4) is None


def test_accel_does_not_touch_the_map():
    events, depth, accel = still_then_drive()
    a, b = Clock(), Clock()
    with_accel, without = runtime(a), runtime(b)
    for rt in (with_accel, without):
        rt.on_camera_info(CAM)
    drive_with_accel(with_accel, a, events, depth, accel)
    drive_with_accel(without, b, events, depth, ())
    assert same_map(with_accel, without) and with_accel.scheduler.processed == without.scheduler.processed


# --- r13 wheel consensus: distance and log ---------------------------------------
def test_distance_follows_the_best_effort_path_through_a_slip():
    """A unit circle whose middle is WHEEL_SLIP: the distance is the arc, not the chord (Codex r13 P2-6)."""
    rt = runtime(Clock())
    n = 400
    samples = [O.PoseSample(START + 0.02 * k, math.sin(2 * math.pi * k / n), 1 - math.cos(2 * math.pi * k / n), 0.0,
                            not (0 < k < n), None if not (0 < k < n) else "WHEEL_SLIP", 0) for k in range(n + 1)]
    rt._take(samples)
    assert rt._epoch.distance_m == pytest.approx(2 * math.pi, rel=1e-3)
    rt2 = runtime(Clock())
    rt2._take([s._replace(reason="POSE_GAP") if s.reason == "WHEEL_SLIP" else s for s in samples])
    assert rt2._epoch.distance_m == pytest.approx(0.0, abs=1e-9)           # other invalid stretches: unchanged


def test_processing_summary_counts_the_wheel_consensus():
    rt = runtime(Clock())
    rt.odometry.slip_excluded, rt.odometry.slip_guessed = 3, 2
    assert "slip excluded 3, guessed 2" in rt.processing_summary()


# --- world-cell aggregation (r14) -------------------------------------------------------
def test_the_decisions_log_names_the_map_algorithm():
    from terrain_map import runtime as RT
    records = []
    runtime(Clock(), decisions=records.append)
    assert records[0]["map_algorithm"] == RT.MAP_ALGORITHM == "r14-world-points"
    assert load_decisions(json.loads(json.dumps(r)) for r in records).algorithm == RT.MAP_ALGORITHM
    old = dict(records[0]); del old["map_algorithm"]                     # an r13 log carries no algorithm
    assert load_decisions([old]).algorithm is None


def test_world_binning_runs_outside_the_lock_and_its_times_are_recorded(monkeypatch):
    from terrain_map import runtime as RT
    events, depth = scenario()
    records, calls = [], []
    clock = Clock()
    rt = runtime(clock, decisions=records.append)
    real = RT.world_cells

    def checked(scan, pose, cfg):
        calls.append(rt._lock._is_owned())
        return real(scan, pose, cfg)
    monkeypatch.setattr(RT, "world_cells", checked)
    rt.on_camera_info(CAM)
    drive(rt, clock, events, depth)
    assert calls and not any(calls)
    done = [r for r in records if r.get("k") == "result" and r["r"] == "processed"]
    assert len(done) == len(calls) and rt.map.tiles()
    assert all(r["bin_ms"] >= 0 and r["lock_wait_ms"] >= 0 and r["fuse_ms"] >= 0 for r in done)


def test_an_epoch_change_while_binning_keeps_the_frame_out_of_the_new_map(monkeypatch):
    from terrain_map import runtime as RT
    events, depth = scenario()
    records = []
    clock = Clock()
    rt = runtime(clock, decisions=records.append)
    real = RT.world_cells
    hit = []

    def epoch_meanwhile(scan, pose, cfg):
        cells = real(scan, pose, cfg)
        if not hit:                                     # an input callback moves the epoch mid-frame
            with rt._lock:
                rt.odometry._new_epoch("keep_gyro", "slip")
                rt._sync_epoch()
            hit.append(rt.epoch)
        return cells
    monkeypatch.setattr(RT, "world_cells", epoch_meanwhile)
    rt.on_camera_info(CAM)
    drive(rt, clock, events, depth, until=START + 2.0)
    first = next(r for r in records if r.get("k") == "result" and r["r"] != "GROUND_NOT_FOUND")
    assert hit and first["r"] == "epoch_changed" and first["epoch"] == hit[0] - 1


def test_a_binning_error_after_an_epoch_change_does_not_flag_the_new_epoch(monkeypatch):
    """r14 Codex round 2: the old epoch's failure must not show as the new epoch's SENSOR_ERROR."""
    from terrain_map import runtime as RT
    events, depth = scenario()
    records = []
    clock = Clock()
    rt = runtime(clock, decisions=records.append)
    real = RT.world_cells
    hit = []

    def fail_after_epoch(scan, pose, cfg):
        if not hit:                                     # the epoch moves, then binning fails
            with rt._lock:
                rt.odometry._new_epoch("keep_gyro", "slip")
                rt._sync_epoch()
            hit.append(rt.epoch)
            raise MemoryError("injected")
        return real(scan, pose, cfg)
    monkeypatch.setattr(RT, "world_cells", fail_after_epoch)
    rt.on_camera_info(CAM)
    drive(rt, clock, events, depth, until=START + 2.0)
    first = next(r for r in records if r.get("k") == "result" and r["r"] != "GROUND_NOT_FOUND")
    assert hit and first["r"] == "epoch_changed" and first["epoch"] == hit[0] - 1
    assert rt._error_at is None


def test_a_slow_decisions_log_shows_in_the_next_result_line():
    """r14 Codex rounds 2–4: log writes stay where they arise but are timed, so a slow disk fails the gate."""
    events, depth = scenario()
    lines = []
    clock = Clock()

    def slow_log(entry):
        if entry.get("k") == "result":
            clock.now += 0.2                                  # a slow disk: 200 ms per result line
        lines.append(entry)
    rt = runtime(clock, decisions=slow_log)
    rt.on_camera_info(CAM)
    drive(rt, clock, events, depth)
    results = [r for r in lines if r.get("k") == "result"]
    assert len(results) >= 3
    assert results[0]["rec_ms_max"] < 200.0                  # nothing slow before the first result line
    assert all(r["rec_ms_max"] >= 200.0 for r in results[1:]) # each carries the previous line's write
    assert results[0]["hold_ms_prev"] is None
    assert all(r["hold_ms_prev"] >= 200.0 for r in results[1:])   # the whole hold includes the write


def test_the_closing_line_carries_the_last_frame_and_writes_after_it():
    """r14 Codex round 5: the last result line's write and later input writes must not go unmeasured."""
    events, depth = scenario()
    lines = []
    clock = Clock()
    slow = {"result": 0.2}

    def log(entry):
        clock.now += slow.get(entry.get("k"), 0.0)
        lines.append(entry)
    rt = runtime(clock, decisions=log)
    rt.on_camera_info(CAM)
    drive(rt, clock, events, depth)
    slow["result"], slow["rx"] = 0.0, 0.07
    rt.on_gyro(START + 9.0, START + 9.0, (0.0, 0.0, 0.0), 99)  # an input write after the last result
    rt.close_log()
    end = lines[-1]
    assert any(r.get("k") == "result" for r in lines)
    assert end["k"] == "timing_end" and end["hold_ms_prev"] >= 200.0 and end["rec_ms_max"] >= 70.0


def test_fusion_and_its_result_write_add_up_in_the_hold(monkeypatch):
    """r14 Codex round 5: 40 ms fusion + 40 ms result write is an 80 ms hold, not max(40, 40)."""
    events, depth = scenario()
    lines = []
    clock = Clock()

    def log(entry):
        if entry.get("k") == "result":
            clock.now += 0.04
        lines.append(entry)
    rt = runtime(clock, decisions=log)
    fuse = rt.map.fuse_cells

    def slow_fuse(*args, **kwargs):
        clock.now += 0.04
        return fuse(*args, **kwargs)
    rt.map.fuse_cells = slow_fuse
    rt.on_camera_info(CAM)
    drive(rt, clock, events, depth)
    rt.close_log()
    processed = [r for r in lines if r.get("k") == "result" and r["r"] == "processed"]
    holds = [r["hold_ms_prev"] for r in lines if r.get("k") in ("result", "timing_end") and r.get("hold_ms_prev") is not None]
    assert processed and max(holds) >= 80.0 and all(r["fuse_ms"] < 80.0 for r in processed)
