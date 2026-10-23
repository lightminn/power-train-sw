"""Depth optical points → L515 colour pixels (spec §4.3).

Pinhole only, translation only.  Distortion and the depth→colour rotation are
deliberately ignored; §7.3-3 measures whether that is good enough and the
packet's ``alignment_verified`` flag tells the operator when it is not proven.
"""
from __future__ import annotations

import numpy as np

from .camera import CameraModel

DEFAULT_COLOR_OFFSET_M = (0.0, -0.024, 0.0)


class DepthToColorProjector:
    def __init__(self, color_camera: CameraModel, color_offset_m=DEFAULT_COLOR_OFFSET_M) -> None:
        offset = np.asarray(color_offset_m, dtype=np.float64)
        if offset.shape != (3,) or not np.all(np.isfinite(offset)):
            raise ValueError("color_offset_m must be three finite numbers")
        self.camera = color_camera
        self._offset = offset.astype(np.float32)

    def project(self, points_depth: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Return ``(pixels int32[M, 2], kept bool[N])`` for depth-frame points.

        Points with z <= 0 (no depth) or outside the colour frame are dropped;
        ``kept`` marks which input rows produced the returned pixels.
        """
        points = np.asarray(points_depth, dtype=np.float32).reshape(-1, 3)
        color = points - self._offset
        z = color[:, 2]
        kept = (points[:, 2] > 0.0) & (z > 0.0)
        with np.errstate(divide="ignore", invalid="ignore"):
            u = np.floor(self.camera.fx * color[:, 0] / z + self.camera.cx)
            v = np.floor(self.camera.fy * color[:, 1] / z + self.camera.cy)
        kept &= (
            np.isfinite(u) & np.isfinite(v)
            & (u >= 0) & (u < self.camera.width)
            & (v >= 0) & (v < self.camera.height)
        )
        pixels = np.stack((u[kept], v[kept]), axis=1).astype(np.int32)
        return pixels, kept
