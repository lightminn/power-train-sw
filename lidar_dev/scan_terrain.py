"""Summarise every recording under data/: status counts, zone medians, height.

    python -m lidar_dev.scan_terrain [data]
"""
from __future__ import annotations

from collections import Counter
from pathlib import Path
import sys

import numpy as np

from lidar_dev.recording import iter_frames, load_recording, make_detector


def summarize(folder) -> dict:
    recording = load_recording(folder)
    detector = make_detector(recording)
    statuses: Counter[str] = Counter()
    zones = {"left": [], "center": [], "right": []}
    heights = []
    for _path, depth, rx_s in iter_frames(recording):
        result = detector.process(depth, rx_s)
        statuses[f"{result.status}/{result.reason or result.ground_state}"] += 1
        for name, value in result.zones.items():
            if value is not None:
                zones[name].append(value)
        if result.camera_height_m is not None:
            heights.append(result.camera_height_m)
    return {
        "frames": sum(statuses.values()),
        "statuses": dict(statuses),
        "zones_median_m": {k: (float(np.median(v)) if v else None) for k, v in zones.items()},
        "camera_height_median_m": float(np.median(heights)) if heights else None,
    }


def main(argv=None) -> int:
    root = Path((argv or sys.argv[1:] or ["data"])[0])
    for folder in sorted(p for p in root.iterdir() if (p / "meta.json").exists()):
        print(f"{folder.name:14s} {summarize(folder)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
