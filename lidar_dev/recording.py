"""Offline L515 recording format shared by the lidar_dev tools.

    <folder>/meta.json        depth_scale_m, frame_period_s, depth_intrinsics,
                              color_intrinsics (optional), depth_to_color (optional)
    <folder>/depth/000000.png raw, unaligned uint16 depth (16-bit PNG)
    <folder>/color/000000.jpg optional colour frames (1280×720)

Older recordings whose ``depth_intrinsics`` is a ``[fx, fy, cx, cy]`` list
are still readable; their size comes from the first depth PNG.
"""
from __future__ import annotations

from dataclasses import dataclass
import json
from pathlib import Path

import cv2
import numpy as np

from terrain_overlay.camera import CameraModel
from terrain_overlay.detector import DetectorConfig, TerrainDetector

DEFAULT_FRAME_PERIOD_S = 0.1


@dataclass(frozen=True)
class Recording:
    folder: Path
    depth_scale_m: float
    frame_period_s: float
    depth_camera: CameraModel
    color_camera: CameraModel | None
    depth_paths: tuple[Path, ...]


def _camera(entry, fallback_size=None) -> CameraModel:
    if isinstance(entry, dict):
        width = entry.get("width", fallback_size[0] if fallback_size else None)
        height = entry.get("height", fallback_size[1] if fallback_size else None)
        return CameraModel(
            fx=float(entry["fx"]), fy=float(entry["fy"]),
            cx=float(entry.get("ppx", entry.get("cx"))),
            cy=float(entry.get("ppy", entry.get("cy"))),
            width=int(width), height=int(height),
        )
    fx, fy, cx, cy = (float(value) for value in entry[:4])
    return CameraModel(fx, fy, cx, cy, int(fallback_size[0]), int(fallback_size[1]))


def read_depth(path: Path) -> np.ndarray:
    depth = cv2.imread(str(path), cv2.IMREAD_UNCHANGED)
    if depth is None or depth.dtype != np.uint16:
        raise ValueError(f"not a 16-bit depth PNG: {path}")
    return depth


def load_recording(folder) -> Recording:
    folder = Path(folder).expanduser()
    meta = json.loads((folder / "meta.json").read_text(encoding="utf-8"))
    depth_paths = tuple(sorted((folder / "depth").glob("*.png")))
    if not depth_paths:
        raise ValueError(f"no depth/*.png under {folder}")
    first = read_depth(depth_paths[0])
    size = (first.shape[1], first.shape[0])
    color = meta.get("color_intrinsics")
    return Recording(
        folder=folder,
        depth_scale_m=float(meta.get("depth_scale_m", 0.00025)),
        frame_period_s=float(meta.get("frame_period_s", DEFAULT_FRAME_PERIOD_S)),
        depth_camera=_camera(meta["depth_intrinsics"], size),
        color_camera=None if color is None else _camera(color),
        depth_paths=depth_paths,
    )


def make_detector(recording: Recording, **config) -> TerrainDetector:
    return TerrainDetector(
        recording.depth_camera,
        DetectorConfig(depth_scale_m=recording.depth_scale_m, **config),
    )


def iter_frames(recording: Recording):
    """Yield ``(path, depth, frame_rx_s)``; time advances by the frame period."""
    for index, path in enumerate(recording.depth_paths):
        yield path, read_depth(path), index * recording.frame_period_s
