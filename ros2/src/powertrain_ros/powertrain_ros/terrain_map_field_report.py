"""Field-check numbers for the terrain map (Task 12, spec §7.3) from a bag and/or the node's log.

    python3 -m powertrain_ros.terrain_map_field_report --bag <bag> [--window A:B] [--bag <bag2> --window A:B ...]
        [--reference-height H] [--param name=value ...] [--decisions processed.jsonl [--session INSTANCE]]
        [--depth-every N] [--out report.json]

- ``--bag`` is streamed (a 10-minute bag is ~3.7 GB; nothing is held in memory but numbers):
  gyro / wheel / depth header-stamp and receive intervals (gaps over 60 ms and 300 ms — the
  odometry's epoch limits), the 50 Hz chassis tick from ``/wheel_states``, and per depth frame
  the detector result, the support check and the base_link height above the fitted floor
  (n·o + d) — only from planes that pass the near-band support check — with a suggested
  ``support_base_height_m`` / ``support_base_variation_m``.  ``--window`` limits a bag to seconds
  after its first stamp (e.g. a 30 s standstill).  Procedure (spec §7.3-1): flat standstill
  first → its height H; then every rough-ground pose (one ``--bag``/``--window`` each) pooled
  with ``--reference-height H`` — only that pooled result may switch the height check on.
- ``--decisions`` summarises the live node's ``processed_log_path``: loss rate and per-frame
  processing time measured **on the robot** (the bag-side time is this machine's).
"""
from __future__ import annotations

import argparse
import json
import sys
import time

import numpy as np

from terrain_map.config import MapConfig
from terrain_map.field_report import distribution, interval_stats, summarize_decisions, support_suggestion
from terrain_map.geometry import base_to_optical
from terrain_map.scan import decimate, make_frame, support_check
from terrain_overlay.camera import CameraModel
from terrain_overlay.detector import DetectorConfig, TerrainDetector

from powertrain_ros.terrain_map_node import (
    DEPTH_INFO_TOPIC, DEPTH_TOPIC, GYRO_TOPIC, WHEEL_TOPIC, depth_array, stamp_s,
)
from powertrain_ros.terrain_map_replay import _config

STREAMS = {GYRO_TOPIC: "gyro", WHEEL_TOPIC: "wheel", DEPTH_TOPIC: "depth", DEPTH_INFO_TOPIC: "camera_info"}
CRITERIA = {"proc_ms_p95": 150.0, "loss_rate": 0.10}               # spec §7.3-3 (defaults, tune with field data)


def _messages(path: str):
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=""),
                rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"))
    types = {t.name: get_message(t.type) for t in reader.get_all_topics_and_types() if t.name in STREAMS}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic in types:
            yield topic, deserialize_message(data, types[topic]), t_ns * 1e-9


def analyze_bag(path: str, cfg: MapConfig, *, window: tuple[float, float] | None = None,
                depth_every: int = 1) -> dict:
    header = {name: [] for name in STREAMS.values()}
    receive = {name: [] for name in STREAMS.values()}
    rotation = base_to_optical(cfg.camera_pitch_down_deg, cfg.camera_roll_deg, cfg.camera_yaw_deg)
    translation = np.array([cfg.camera_x_m, cfg.camera_y_m, cfg.camera_z_m])
    camera = detector = None
    first = None
    depth_seen = 0
    counts = {"frames": 0, "ok": 0, "fresh": 0, "bad_encoding": 0, "no_camera_info": 0, "shape_mismatch": 0,
              "support_ok": 0, "near_band": 0, "base_height": 0, "status": {}}
    base_heights, rejected_heights, host_ms = [], [], []
    for topic, message, t_rx in _messages(path):
        stamp = stamp_s(message)
        first = stamp if first is None else first
        if window is not None and not window[0] <= stamp - first <= window[1]:
            continue
        name = STREAMS[topic]
        header[name].append(stamp)
        receive[name].append(t_rx)
        if topic == DEPTH_INFO_TOPIC:
            model = CameraModel.from_k(message.k, message.width, message.height)
            if model != camera:
                camera, k = model, cfg.pixel_stride
                detector = TerrainDetector(                     # the same decimated model the runtime builds
                    CameraModel(fx=camera.fx / k, fy=camera.fy / k, cx=camera.cx / k, cy=camera.cy / k,
                                width=-(-camera.width // k), height=-(-camera.height // k)),
                    DetectorConfig(min_ground_points=cfg.ground_min_points))
            continue
        if topic != DEPTH_TOPIC:
            continue
        depth_seen += 1
        if (depth_seen - 1) % depth_every:
            continue
        counts["frames"] += 1
        array = depth_array(message)
        if array is None:
            counts["bad_encoding"] += 1
            continue
        if camera is None:
            counts["no_camera_info"] += 1
            continue
        if array.shape != (camera.height, camera.width):
            counts["shape_mismatch"] += 1
            continue
        started = time.perf_counter()
        small, _ = decimate(array, camera, cfg.pixel_stride)
        result = detector.process(small, stamp)
        counts["status"][result.status] = counts["status"].get(result.status, 0) + 1
        if result.status == "OK":
            counts["ok"] += 1
        if result.status == "OK" and result.ground_state == "FRESH" and result.ground_plane is not None:
            counts["fresh"] += 1
            frame = make_frame(result, cfg)
            support = support_check(result, frame, cfg)
            counts["support_ok" if support.ok else support.reason] += 1
            normal, offset = result.ground_plane
            n = np.asarray(normal, float) / np.linalg.norm(normal)
            height = float(n @ (-(rotation @ translation)) + offset)
            # A plane the near band rejects (e.g. a ditch bottom) is not the support floor:
            # its height must not calibrate the support check (Codex field-tool review).
            (rejected_heights if support.reason == "near_band" else base_heights).append(height)
        host_ms.append((time.perf_counter() - started) * 1000.0)
    frames = counts["frames"] - counts["bad_encoding"] - counts["no_camera_info"] - counts["shape_mismatch"]
    wheel_ticks = interval_stats(header["wheel"], regression_s=cfg.stamp_regression_epoch_s, sort=True)
    return {
        "bag": path, "window_s": list(window) if window else None,
        # header: the publisher's own rhythm (sorted stamps); receive: what arrived when, in record order.
        "streams": {name: {"header": interval_stats(header[name], regression_s=cfg.stamp_regression_epoch_s,
                                                    sort=True),
                           "receive": interval_stats(receive[name]),
                           "reordered": int(sum(1 for a, b in zip(header[name], header[name][1:]) if b < a))}
                    for name in ("gyro", "wheel", "depth")},
        "chassis_tick_p99_s": wheel_ticks.get("p99_s"),
        "depth": dict(counts, fresh_ratio=(counts["fresh"] / frames) if frames else None),
        "support": support_suggestion(base_heights, cfg),
        "base_heights_m": base_heights,
        "rejected_base_heights": distribution(rejected_heights),
        "host_proc_ms": distribution(host_ms),
    }


def _summary(report: dict) -> list[str]:
    lines = []
    for bag in report.get("bag_reports", []):
        lines.append(f"— {bag['bag']} window {bag['window_s']}")
        g = bag["streams"]["gyro"]["header"]
        w = bag["streams"]["wheel"]["header"]
        d, s = bag["depth"], bag["support"]
        lines += [
            f"gyro interval p50/p99/max: {g.get('p50_s', float('nan')):.3f}/{g.get('p99_s', float('nan')):.3f}/"
            f"{g.get('max_s', float('nan')):.3f} s · gaps >60 ms: {g.get('over_s', {}).get('0.06', 0)} "
            f"(each one is a new map epoch)",
            f"wheel (chassis tick) interval p99: {w.get('p99_s', float('nan')):.4f} s · gaps >60 ms: "
            f"{w.get('over_s', {}).get('0.06', 0)} · arrived out of order: gyro "
            f"{bag['streams']['gyro']['reordered']} / wheel {bag['streams']['wheel']['reordered']}",
            f"depth frames {d['frames']} · FRESH ratio {d['fresh_ratio']} · support fail near_band {d['near_band']} "
            f"/ base_height {d['base_height']}",
        ]
        if s.get("n"):
            lines.append(f"  this recording: height {s['support_base_height_m']} · variation "
                         f"{s['support_base_variation_m']} (n={s['n']}) — not an approval on its own")
        else:
            lines.append("  support_base_height_m: no supported FRESH frame in the window")
    pooled = report.get("support_pooled")
    if pooled and pooled.get("n"):
        lines.append(f"support_base_height_m {pooled['support_base_height_m']} ({pooled['reference']}) · "
                     f"support_base_variation_m {pooled['support_base_variation_m']} · can enable: "
                     f"{pooled['can_enable']} — pooled over {len(report.get('bag_reports', []))} recordings "
                     f"(n={pooled['n']})")
    dec = report.get("decisions")
    if dec:
        loss, p95 = dec["frames"]["loss_rate"], dec["proc_ms"].get("p95")
        verdict = lambda ok: "PASS" if ok else "FAIL"  # noqa: E731
        lines.append(f"loss rate {loss if loss is None else round(loss, 4)} "
                     f"(≤ {CRITERIA['loss_rate']}: {verdict(loss is not None and loss <= CRITERIA['loss_rate'])})")
        lines.append(f"proc_ms p95 {p95} on the robot "
                     f"(≤ {CRITERIA['proc_ms_p95']}: {verdict(p95 is not None and p95 <= CRITERIA['proc_ms_p95'])})")
        lines.append(f"epochs {dec['epochs']['count']} {dec['epochs']['by_cause']} · late samples {dec['samples']['late']}")
    return lines


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("--bag", action="append", default=[], help="repeatable: one per pose/recording")
    parser.add_argument("--window", action="append", default=[],
                        help="A:B seconds after that bag's first stamp (one per --bag, or one for all)")
    parser.add_argument("--reference-height", type=float,
                        help="flat-ground base height H; rough-ground poses are pooled against it")
    parser.add_argument("--depth-every", type=int, default=1, help="analyse every N-th depth frame")
    parser.add_argument("--param", action="append", default=[], metavar="NAME=VALUE",
                        help="MapConfig override (use the mount values the node ran with)")
    parser.add_argument("--decisions", help="the node's processed_log_path (JSONL)")
    parser.add_argument("--session", help="sender instance when the log holds several runs")
    parser.add_argument("--out", help="write the full report (JSON)")
    args = parser.parse_args(argv)
    if not args.bag and not args.decisions:
        parser.error("give --bag and/or --decisions")
    if args.depth_every < 1:
        parser.error("--depth-every must be ≥ 1")
    if args.window and len(args.window) not in (1, len(args.bag)):
        parser.error("give one --window, or one per --bag")
    windows = []
    for text in (args.window * len(args.bag) if len(args.window) == 1 else args.window) or [None] * len(args.bag):
        if text is None:
            windows.append(None)
        else:
            a, _, b = text.partition(":")
            windows.append((float(a), float(b)))
    cfg = _config(args.param)
    report = {}
    if args.bag:
        report["bag_reports"] = [analyze_bag(path, cfg, window=w, depth_every=args.depth_every)
                                 for path, w in zip(args.bag, windows)]
        pooled = [h for r in report["bag_reports"] for h in r["base_heights_m"]]
        report["support_pooled"] = support_suggestion(pooled, cfg, reference_m=args.reference_height)
    if args.decisions:
        with open(args.decisions, encoding="utf-8") as fh:
            try:
                report["decisions"] = summarize_decisions((json.loads(line) for line in fh if line.strip()),
                                                          args.session)
            except ValueError as exc:
                print(f"refusing: {exc}", file=sys.stderr)
                return 2
    if args.out:
        with open(args.out, "w", encoding="utf-8") as fh:
            json.dump(report, fh, indent=1)
    print("\n".join(_summary(report)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
