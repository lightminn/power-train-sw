"""terrain_map_replay — bag → same MapRuntime, with and without the live decisions (spec §6)."""
import json

import numpy as np
import pytest
import rosbag2_py
from rclpy.serialization import serialize_message

from powertrain_ros import terrain_map_node as N
from powertrain_ros import terrain_map_replay as RP
from terrain_map import contract as C
from terrain_map.config import MapConfig
from terrain_map.runtime import MapRuntime
from terrain_map.tests.odometry_sim import Command
from terrain_overlay.camera import CameraModel
from terrain_overlay.runtime import SenderSession

import terrain_map_fixture as F

CFG = MapConfig(pixel_stride=1)
SESSION = SenderSession(boot_id="0" * 32, start_mono_ns=1, instance="b" * 16)
TYPES = {F.DEPTH_TOPIC: "sensor_msgs/msg/Image", F.INFO_TOPIC: "sensor_msgs/msg/CameraInfo",
         F.GYRO_TOPIC: "sensor_msgs/msg/Imu", F.WHEEL_TOPIC: "powertrain_msgs/msg/WheelStates"}


def write_bag(path, items):
    """items: (record time s, topic, message) in record order."""
    writer = rosbag2_py.SequentialWriter()
    writer.open(rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    for name, kind in TYPES.items():
        writer.create_topic(rosbag2_py.TopicMetadata(name=name, type=kind, serialization_format="cdr"))
    for t, topic, message in items:
        writer.write(topic, serialize_message(message), int(t * 1e9))
    del writer


class Clock:
    now = 0.0

    def __call__(self):
        return self.now


def deliver(rt, topic, message, rx):
    """The node's callbacks, with an explicit receive time."""
    stamp = N.stamp_s(message)
    if topic == F.INFO_TOPIC:
        rt.on_camera_info(CameraModel.from_k(message.k, message.width, message.height))
    elif topic == F.DEPTH_TOPIC:
        rt.on_depth(N.depth_array(message), stamp, rx)
    elif topic == F.GYRO_TOPIC:
        v = message.angular_velocity
        rt.on_gyro(stamp, rx, (v.x, v.y, v.z))
    else:
        rt.on_wheels(stamp, rx, N.wheel_readings(message, CFG.wheel_radius_m))


def receipt(events):
    """Live receive times: some wheel messages late (dropped), a 0.45 s wheel silence then a burst (stall)."""
    rng = np.random.default_rng(5)
    out = []
    for e in events:
        rx = e.stamp_s + (0.02 if e.topic in (F.DEPTH_TOPIC, F.INFO_TOPIC) else 0.0)
        t = e.stamp_s - F.START
        if e.topic == F.WHEEL_TOPIC:
            if 2.0 < t < 2.3 and rng.random() < 0.3:
                rx += 0.12
            elif 2.5 <= t <= 2.9:
                rx = F.START + 2.95
        out.append((rx, e.topic, e.message))
    out.sort(key=lambda r: r[0])
    return out


def live(received, decisions, session=SESSION):
    clock = Clock()
    rt = MapRuntime(CFG, session, N.wheel_layout(), clock=clock, decisions=decisions)
    i, now = 0, F.START
    while i < len(received):
        while i < len(received) and received[i][0] <= now:
            deliver(rt, received[i][1], received[i][2], received[i][0])
            i += 1
        clock.now = now
        rt.process_tick()
        now += 0.1
    return rt


def content(rt):
    out = {}
    for tx, ty in rt.map.tiles():
        heights, codes, _ = C.unpack_cells(rt.map.encode_tile(tx, ty, 0.0)["cells"], CFG.tile_cells)
        out[(tx, ty)] = (rt.map.version_of(tx, ty), [C.UNOBSERVED_H if h is None else h for h in heights], codes)
    return rt.epoch, out


def npz_content(path):
    z = np.load(path)
    return int(z["epoch"]), {
        (int(tx), int(ty)): (int(v), [int(h) for h in hs], [int(c) for c in cs])
        for (tx, ty), v, hs, cs in zip(z["tiles"], z["versions"], z["heights_cm"], z["codes"])
    }


@pytest.fixture(scope="module")
def recorded(tmp_path_factory):
    events, _ = F.scenario(F.drive_then(Command("skid", v=0.3, omega=0.1)), 4.0)
    received = receipt(events)
    records = []
    rt = live(received, records.append)
    root = tmp_path_factory.mktemp("bag")
    write_bag(root / "run", received)
    log = root / "processed.jsonl"
    log.write_text("".join(json.dumps(r) + "\n" for r in records))
    return rt, root / "run", log


def run_cli(bag, out, *extra):
    assert RP.main([str(bag), "--param", "pixel_stride=1", "--out", str(out), *extra]) == 0
    return npz_content(out)


def test_decisions_reproduce_the_live_map(recorded, tmp_path):
    rt, bag, log = recorded
    assert rt.odometry.late_dropped > 0 and rt.epoch >= 1 and rt.map.tiles()
    assert run_cli(bag, tmp_path / "follow.npz", "--decisions-from", str(log)) == content(rt)


def test_ignoring_the_decisions_gives_a_different_map(recorded, tmp_path):
    rt, bag, _ = recorded
    assert run_cli(bag, tmp_path / "plain.npz") != content(rt)


def test_plain_replay_is_deterministic(recorded, tmp_path):
    _, bag, _ = recorded
    assert run_cli(bag, tmp_path / "a.npz") == run_cli(bag, tmp_path / "b.npz")


def test_record_order_regression_splits_the_epochs():
    gyro = lambda t: F.gyro_message(t, 0.0)
    records = [RP.Record(F.GYRO_TOPIC, t, k, gyro(t)) for k, t in enumerate([100.0, 100.02, 10.0, 10.02])]
    parts = RP.segments(records, CFG.stamp_regression_epoch_s, drop_small_regressions=True)
    assert [[r.stamp_s for r in p] for p in parts] == [[100.0, 100.02], [10.0, 10.02]]
    small = [RP.Record(F.GYRO_TOPIC, t, k, gyro(t)) for k, t in enumerate([100.0, 100.04, 100.02, 100.06])]
    assert [r.stamp_s for r in RP.segments(small, 1.0, drop_small_regressions=True)[0]] == [100.0, 100.04, 100.06]
    assert len(RP.segments(small, 1.0, drop_small_regressions=False)[0]) == 4


# --- Codex Task 7 review -----------------------------------------------------------------
@pytest.fixture(scope="module")
def drive(tmp_path_factory):
    events, _ = F.scenario(F.drive_then(Command("skid", v=0.3, omega=0.1)), 4.0)
    return receipt(events), tmp_path_factory.mktemp("cases")


def write_log(path, *runs):
    path.write_text("".join(json.dumps(r) + "\n" for run in runs for r in run))


def gyro_index(received, k):
    return [i for i, r in enumerate(received) if r[1] == F.GYRO_TOPIC][k]


def test_a_sample_only_the_recorder_lost_refuses_the_replay(drive, capsys):
    received, root = drive
    records = []
    live(received, records.append)
    lost = gyro_index(received, 50)
    write_bag(root / "recorder_lost", received[:lost] + received[lost + 1:])
    write_log(root / "a.jsonl", records)
    code = RP.main([str(root / "recorder_lost"), "--param", "pixel_stride=1", "--decisions-from", str(root / "a.jsonl")])
    assert code == 2 and "not in the bag" in capsys.readouterr().err


def test_a_sample_only_the_node_lost_is_skipped(drive, tmp_path):
    received, root = drive
    lost = gyro_index(received, 50)
    records = []
    rt = live(received[:lost] + received[lost + 1:], records.append)       # the node never got it
    write_bag(root / "node_lost", received)                                  # the recorder did
    write_log(root / "b.jsonl", records)
    assert run_cli(root / "node_lost", tmp_path / "f.npz", "--decisions-from", str(root / "b.jsonl")) == content(rt)


def test_a_log_appended_across_restarts_needs_a_session(drive, tmp_path, capsys):
    received, root = drive
    first, second = [], []
    rt = live(received, first.append)
    other = SenderSession(boot_id="0" * 32, start_mono_ns=2, instance="c" * 16)
    live(received[: len(received) // 2], second.append, session=other)      # a restart appended to the same file
    write_bag(root / "restart", received)
    write_log(root / "c.jsonl", first, second)
    args = [str(root / "restart"), "--param", "pixel_stride=1", "--decisions-from", str(root / "c.jsonl")]
    assert RP.main(args) == 2 and "choose one session" in capsys.readouterr().err
    assert run_cli(root / "restart", tmp_path / "s.npz", "--decisions-from", str(root / "c.jsonl"),
                   "--session", "b" * 16) == content(rt)


def test_a_bag_without_a_map_still_exports(tmp_path):
    events, _ = F.scenario(F.drive_then(Command("skid", v=0.3)), 2.0, surface=None)
    write_bag(tmp_path / "no_depth", [(e.stamp_s, e.topic, e.message) for e in events])
    epoch, tiles = run_cli(tmp_path / "no_depth", tmp_path / "empty.npz")
    assert tiles == {}


def test_a_clock_restart_that_repeats_old_stamps_replays_the_same_map(tmp_path):
    """Codex T7 re-review P2-1: identities must match after >512 stamps and a clock restart."""
    first, _ = F.scenario(F.drive_then(Command("skid", v=0.3, omega=0.1)), 6.4)
    again, _ = F.scenario(F.drive_then(Command("skid", v=0.3, omega=0.4), stop_s=0.0), 4.0)  # same stamps, turning
    received = [(rx, topic, m) for rx, topic, m in receipt(first)]
    # the restarted run arrives cleanly, so its first seconds (the repeated stamps) stay in the final map
    received += sorted((e.stamp_s + 20.0 + (0.02 if e.topic in (F.DEPTH_TOPIC, F.INFO_TOPIC) else 0.0), e.topic,
                        e.message) for e in again)
    records = []
    rt = live(received, records.append)
    assert rt.epoch >= 1 and rt.map.tiles()
    write_bag(tmp_path / "restart", received)
    write_log(tmp_path / "d.jsonl", records)
    assert run_cli(tmp_path / "restart", tmp_path / "r.npz", "--decisions-from", str(tmp_path / "d.jsonl")) == content(rt)


def test_identities_are_fixed_in_record_order_before_sorting():
    """Codex re-review: gyro100 → wheel100 → wheel98.8 → gyro99.5 → gyro99.5 → gyro98.9."""
    from terrain_map import odometry as O
    from terrain_map.runtime import Decisions, default_gyro_rotation, gyro_body_rate
    from terrain_map.tests.odometry_sim import wheels_for
    rows, *_ = wheels_for(Command("stop"))
    readings = tuple(O.WheelReading(n, s, d, c, m, True) for n, s, d, c, m in rows)
    order = [("gyro", 100.0), ("wheel", 100.0), ("wheel", 98.8), ("gyro", 99.5), ("gyro", 99.5), ("gyro", 98.9)]
    messages = [F.on_the_wire(F.gyro_message(t, 0.0) if src == "gyro" else F.wheel_message(O.WheelSample(t, t, readings)))
                for src, t in order]
    rotation = default_gyro_rotation(CFG)
    logged = []
    o = O.PlanarOdometry(CFG, N.wheel_layout(), record=logged.append)
    for (src, t), m in zip(order, messages):                   # live: arrival order, the node's conversions
        if src == "gyro":
            v = m.angular_velocity
            o.add_gyro(O.GyroSample(t, t, gyro_body_rate(rotation, (v.x, v.y, v.z))))
        else:
            o.add_wheels(O.WheelSample(t, t, N.wheel_readings(m, CFG.wheel_radius_m)))
    records = [RP.Record(F.GYRO_TOPIC if src == "gyro" else F.WHEEL_TOPIC, t, k, m)
               for k, ((src, t), m) in enumerate(zip(order, messages))]
    records = RP.identify(records, CFG)
    decisions = Decisions(O.follow_plan(logged), frozenset())
    assert RP.missing_from_bag(records, decisions, CFG.stamp_regression_epoch_s) == []
    assert {(RP.SOURCES[r.topic], r.seg, r.stamp_s, r.digest, r.copy) for r in records} == decisions.odometry.received


def test_a_gyro_sample_live_rejects_does_not_take_an_identity():
    """Codex T9 P2: NaN then a valid gyro with the same stamp."""
    from terrain_map import odometry as O
    from terrain_map.runtime import MapRuntime
    nan, ok = F.gyro_message(100.0, 0.0), F.gyro_message(100.0, 0.0)
    nan.angular_velocity.x = float("nan")
    nan, ok = F.on_the_wire(nan), F.on_the_wire(ok)
    logged = []
    rt = MapRuntime(CFG, SESSION, N.wheel_layout(), decisions=logged.append)
    for m in (nan, ok):
        v = m.angular_velocity
        rt.on_gyro(100.0, 100.0, (v.x, v.y, v.z))
    records = RP.identify([RP.Record(F.GYRO_TOPIC, 100.0, 0, nan), RP.Record(F.GYRO_TOPIC, 100.0, 1, ok)], CFG)
    have = {(RP.SOURCES[r.topic], r.seg, r.stamp_s, r.digest, r.copy) for r in records if r.seg >= 0}
    assert have == O.follow_plan(logged).received


def test_a_rejected_gyro_does_not_shadow_its_valid_twin_in_the_plain_replay():
    """Codex T10 P2: NaN then a valid gyro with the same stamp — the default replay keeps the valid one."""
    nan, ok = F.gyro_message(100.0, 0.0), F.gyro_message(100.0, 0.0)
    nan.angular_velocity.x = float("nan")
    records = RP.identify([RP.Record(F.GYRO_TOPIC, 100.0, 0, nan), RP.Record(F.GYRO_TOPIC, 100.0, 1, ok)], CFG)
    fed = [r for seg in RP.segments(records, CFG.stamp_regression_epoch_s, drop_small_regressions=True) for r in seg]
    assert [r.message for r in fed] == [ok]


def test_depth_decisions_tell_clock_segments_apart(tmp_path):
    """Branch review P2: after a clock restart only the node misses one depth frame."""
    first, _ = F.scenario(F.drive_then(Command("skid", v=0.3, omega=0.1)), 4.0)
    again, _ = F.scenario(F.drive_then(Command("skid", v=0.3, omega=0.4), stop_s=0.0), 4.0)
    seg1 = [(rx, t, m) for rx, t, m in receipt(first)]
    seg2 = sorted((e.stamp_s + 20.0 + (0.02 if e.topic in (F.DEPTH_TOPIC, F.INFO_TOPIC) else 0.0), e.topic, e.message)
                  for e in again)
    depth2 = [i for i, r in enumerate(seg2) if r[1] == F.DEPTH_TOPIC]
    missed = depth2[0]          # the node misses the first depth frame of segment 2 — a stamp segment 1 processed
    records = []
    rt = live(seg1 + seg2[:missed] + seg2[missed + 1:], records.append)
    assert rt.epoch >= 1 and rt.map.tiles()
    write_bag(tmp_path / "bag", seg1 + seg2)                   # the recorder got it
    write_log(tmp_path / "e.jsonl", records)
    assert run_cli(tmp_path / "bag", tmp_path / "f.npz", "--decisions-from", str(tmp_path / "e.jsonl")) == content(rt)


def test_a_different_same_stamp_copy_only_the_recorder_got_is_not_mistaken(tmp_path):
    """Branch re-review P2-1: depth A and B share a stamp; only the recorder got A."""
    events, _ = F.scenario(F.drive_then(Command("skid", v=0.3, omega=0.1)), 4.0)
    received = receipt(events)
    records = []
    rt = live(received, records.append)
    stamp = next(r["t"] for r in records if r.get("k") == "frame" and r.get("r") == "processed")
    i = next(k for k, r in enumerate(received) if r[1] == F.DEPTH_TOPIC and N.stamp_s(r[2]) == stamp)
    shadow = F.on_the_wire(F.depth_message(np.zeros((F.CAM.height, F.CAM.width), np.uint16) + 4000, stamp))
    write_bag(tmp_path / "bag", received[:i] + [(received[i][0] - 0.001, F.DEPTH_TOPIC, shadow)] + received[i:])
    write_log(tmp_path / "g.jsonl", records)
    assert run_cli(tmp_path / "bag", tmp_path / "g.npz", "--decisions-from", str(tmp_path / "g.jsonl")) == content(rt)


def test_a_replay_that_cannot_process_the_recorded_frames_refuses(tmp_path, capsys):
    """Branch re-review P2-2: the bag lost the first CameraInfo messages."""
    events, _ = F.scenario(F.drive_then(Command("skid", v=0.3, omega=0.1)), 4.0)
    received = receipt(events)
    records = []
    live(received, records.append)
    infos = [k for k, r in enumerate(received) if r[1] == F.INFO_TOPIC]
    lossy = [r for k, r in enumerate(received) if k not in infos[:4]]
    write_bag(tmp_path / "bag", lossy)
    write_log(tmp_path / "h.jsonl", records)
    out = tmp_path / "h.npz"
    code = RP.main([str(tmp_path / "bag"), "--param", "pixel_stride=1", "--decisions-from", str(tmp_path / "h.jsonl"),
                    "--out", str(out)])
    assert code == 2 and "did not reproduce" in capsys.readouterr().err and not out.exists()
