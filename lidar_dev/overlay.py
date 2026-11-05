"""Render detector results over each recorded depth frame.

    python -m lidar_dev.overlay data/obj_50cm      → out_overlay/obj_50cm/*.png
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

from lidar_dev.recording import iter_frames, load_recording, make_detector


def render(depth: np.ndarray, result, depth_scale_m: float) -> np.ndarray:
    gray = (np.clip(depth.astype(np.float32) * depth_scale_m / 5.0, 0, 1) * 255).astype(np.uint8)
    image = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    if result.status == "OK":
        floor, obstacle = result.floor_mask, result.obstacle_mask
        image[floor] = (0.5 * image[floor] + 0.5 * np.array([0, 200, 0])).astype(np.uint8)
        image[obstacle] = (0, 0, 255)
    width = image.shape[1]
    ok = result.status == "OK"
    cv2.rectangle(image, (0, 0), (width, 30), (0, 120, 0) if ok else (0, 0, 200), -1)
    fmt = lambda value: "-" if value is None else f"{value:.2f}m"  # noqa: E731
    zones = result.zones
    label = (f"{result.status} {result.reason or result.ground_state or ''}  "
             f"L {fmt(zones['left'])}  C {fmt(zones['center'])}  R {fmt(zones['right'])}")
    cv2.putText(image, label, (8, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2)
    return image


def main(argv=None) -> int:
    folder = Path((argv or sys.argv[1:])[0])
    recording = load_recording(folder)
    detector = make_detector(recording)
    out = Path("out_overlay") / folder.name
    out.mkdir(parents=True, exist_ok=True)
    for path, depth, rx_s in iter_frames(recording):
        result = detector.process(depth, rx_s)
        cv2.imwrite(str(out / path.name), render(depth, result, recording.depth_scale_m))
    print("저장:", out)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
