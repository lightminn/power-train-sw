"""Deterministic terrain map replay from a bag (spec §6).

    python3 -m powertrain_ros.terrain_map_replay <bag> [--decisions-from processed.jsonl]
        [--corrector identity] [--param name=value ...] [--out map.npz]
        [--send 127.0.0.1:5010 --rate 1.0]

No ROS communication: the bag is read with rosbag2_py and fed, in header-stamp
order, to the same ``MapRuntime`` the live node uses, with the stamp as the
clock.  Epoch boundaries come first, from the record order: a message whose
stamp runs more than ``stamp_regression_epoch_s`` behind its topic starts a new
time segment, and each segment is sorted on its own.  Each gyro and wheel
message carries its per-topic record index as ``seq`` — the identity that
``--decisions-from`` (the live node's ``processed_log_path``) refers to.
"""
from __future__ import annotations

import argparse
import dataclasses
import json
import math
import socket
import sys
import time
from typing import Iterable, NamedTuple

import numpy as np

from terrain_map import contract as C
from terrain_map.config import MapConfig
from terrain_map.runtime import MapRuntime, load_decisions
from terrain_overlay.camera import CameraModel
from terrain_overlay.runtime import SenderSession

from powertrain_ros.terrain_map_node import (
    DEPTH_INFO_TOPIC, DEPTH_TOPIC, GYRO_TOPIC, WHEEL_TOPIC, depth_array, stamp_s, wheel_layout, wheel_readings,
)

TOPICS = (DEPTH_TOPIC, DEPTH_INFO_TOPIC, GYRO_TOPIC, WHEEL_TOPIC)
_ORDER = {DEPTH_INFO_TOPIC: 0, WHEEL_TOPIC: 1, GYRO_TOPIC: 2, DEPTH_TOPIC: 3}   # same stamp: wheels before gyro


class Record(NamedTuple):
    topic: str
    stamp_s: float
    seq: int            # per-topic record index
    message: object


def read_bag(path: str) -> list[Record]:
    """All map topics in record order, deserialized."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in TOPICS}
    counts = {topic: 0 for topic in TOPICS}
    records = []
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in types:
            continue
        message = deserialize_message(data, types[topic])
        records.append(Record(topic, stamp_s(message), counts[topic], message))
        counts[topic] += 1
    return records


def segments(records: Iterable[Record], regression_s: float, *, drop_small_regressions: bool) -> list[list[Record]]:
    """Split at a stamp regression > ``regression_s`` within a topic; sort each segment by stamp.

    Without recorded decisions a smaller regression (or a repeat) is dropped, like a late
    sample live; with them the follower decides, so everything is kept.
    """
    out: list[list[Record]] = [[]]
    latest: dict[str, float] = {}
    for r in records:
        last = latest.get(r.topic)
        if last is not None and r.stamp_s < last - regression_s:
            out.append([])
            latest, last = {}, None
        elif last is not None and r.stamp_s <= last and drop_small_regressions and r.topic != DEPTH_INFO_TOPIC:
            continue
        latest[r.topic] = r.stamp_s if last is None else max(last, r.stamp_s)
        out[-1].append(r)
    return [sorted(seg, key=lambda r: (r.stamp_s, _ORDER[r.topic], r.seq)) for seg in out if seg]


def feed(runtime: MapRuntime, clock, record: Record, radius_m: float) -> None:
    clock.now = record.stamp_s
    m = record.message
    if record.topic == DEPTH_INFO_TOPIC:
        runtime.on_camera_info(CameraModel.from_k(m.k, m.width, m.height))
    elif record.topic == DEPTH_TOPIC:
        array = depth_array(m)
        if array is None:
            runtime.on_bad_depth(record.stamp_s)
        else:
            runtime.on_depth(array, record.stamp_s, record.stamp_s)
    elif record.topic == GYRO_TOPIC:
        v = m.angular_velocity
        runtime.on_gyro(record.stamp_s, record.stamp_s, (v.x, v.y, v.z), record.seq)
    else:
        runtime.on_wheels(record.stamp_s, record.stamp_s, wheel_readings(m, radius_m), record.seq)


class _Clock:
    now = 0.0

    def __call__(self) -> float:
        return self.now


def replay(records: list[Record], cfg: MapConfig, *, decisions=None, send=None, rate=1.0,
           session: SenderSession | None = None, log=print) -> MapRuntime:
    clock = _Clock()
    session = session or SenderSession(boot_id="0" * 32, start_mono_ns=int(time.monotonic_ns()),
                                       instance=f"{np.random.default_rng().integers(1 << 63):016x}")
    runtime = MapRuntime(cfg, session, wheel_layout(), clock=clock, follow=decisions,
                         unix_ns=lambda: int(clock.now * 1e9), log_error=log, log_overrun=lambda _m: None)
    next_send, wall0, stamp0 = None, time.monotonic(), None
    for seg in segments(records, cfg.stamp_regression_epoch_s, drop_small_regressions=decisions is None):
        for record in seg:
            feed(runtime, clock, record, cfg.wheel_radius_m)
            runtime.process_tick()
            if send is not None:
                stamp0 = record.stamp_s if stamp0 is None else stamp0
                if next_send is None or record.stamp_s >= next_send or record.stamp_s < next_send - 1.0:
                    delay = wall0 + (record.stamp_s - stamp0) / rate - time.monotonic()
                    if delay > 0:
                        time.sleep(delay)
                    for raw in runtime.send_tick():
                        send[0].sendto(raw, send[1])
                    next_send = record.stamp_s + 1.0 / cfg.status_hz
    runtime.finish()
    if send is not None:
        for raw in runtime.send_tick():
            send[0].sendto(raw, send[1])
    return runtime


def save_map(runtime: MapRuntime, path: str, now_s: float) -> None:
    keys = sorted(runtime.map.tiles())
    cells = []
    for tx, ty in keys:
        heights, codes, _ = C.unpack_cells(runtime.map.encode_tile(tx, ty, now_s)["cells"], runtime.cfg.tile_cells)
        cells.append((
            [C.UNOBSERVED_H if h is None else h for h in heights], codes,
        ))
    np.savez_compressed(
        path,
        epoch=np.int64(runtime.epoch),
        tiles=np.array(keys, np.int64).reshape(-1, 2),
        versions=np.array([runtime.map.version_of(*k) for k in keys], np.int64),
        heights_cm=np.array([c[0] for c in cells], np.int16).reshape(len(keys), -1),
        codes=np.array([c[1] for c in cells], np.uint8).reshape(len(keys), -1),
    )


def _config(params: list[str]) -> MapConfig:
    overrides = {}
    fields = {f.name: f for f in dataclasses.fields(MapConfig)}
    for item in params:
        name, _, text = item.partition("=")
        if name not in fields:
            raise SystemExit(f"unknown parameter {name!r}")
        default = fields[name].default
        if text.lower() in ("none", "nan", ""):
            overrides[name] = None
        elif isinstance(default, str):
            overrides[name] = text
        elif isinstance(default, int) and not isinstance(default, bool):
            overrides[name] = int(text)
        else:
            overrides[name] = float(text)
    return MapConfig(**overrides)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("bag")
    parser.add_argument("--decisions-from", help="the live node's processed_log_path (JSONL)")
    parser.add_argument("--corrector", default=None, choices=C.CORRECTOR_NAMES)
    parser.add_argument("--param", action="append", default=[], metavar="NAME=VALUE",
                        help="MapConfig override (repeatable)")
    parser.add_argument("--out", help="write the final map (.npz)")
    parser.add_argument("--send", metavar="HOST:PORT", help="send status/tile packets (console demo)")
    parser.add_argument("--rate", type=float, default=1.0, help="speed factor for --send")
    args = parser.parse_args(argv)
    params = list(args.param) + ([f"corrector={args.corrector}"] if args.corrector else [])
    cfg = _config(params)
    decisions = None
    if args.decisions_from:
        with open(args.decisions_from, encoding="utf-8") as fh:
            decisions = load_decisions(json.loads(line) for line in fh if line.strip())
    send = None
    if args.send:
        host, _, port = args.send.rpartition(":")
        if args.rate <= 0 or not math.isfinite(args.rate):
            raise SystemExit("--rate must be > 0")
        send = (socket.socket(socket.AF_INET, socket.SOCK_DGRAM), (host, int(port)))
    records = read_bag(args.bag)
    runtime = replay(records, cfg, decisions=decisions, send=send, rate=args.rate,
                     log=lambda m: print(m, file=sys.stderr))
    end = max((r.stamp_s for r in records), default=0.0)
    if args.out:
        save_map(runtime, args.out, end)
    s = runtime.scheduler
    print(json.dumps({
        "records": len(records), "epoch": runtime.epoch, "tiles": len(runtime.map.tiles()),
        "processed": s.processed, "lost": s.lost, "skipped": s.skipped,
        "late_dropped": runtime.odometry.late_dropped, "follow": decisions is not None,
    }))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
