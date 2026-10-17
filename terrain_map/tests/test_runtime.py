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
