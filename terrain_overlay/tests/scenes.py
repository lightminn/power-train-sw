"""Synthetic raw L515 depth scenes for detector/contour tests.

Geometry is defined physically: ``normal`` is the unit up-normal of the floor
in the depth optical frame (x right, y down, z forward) and the camera sits
``height_m`` above the floor, so floor points satisfy ``P·normal = -height_m``.
Boxes are vertical faces standing on the floor, placed ``fwd_m`` ahead along
the floor-projected optical axis and ``lat_m`` to the right of it.
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from terrain_overlay.camera import CameraModel
from terrain_overlay.detector import ground_axes

DEPTH = CameraModel(fx=460.0, fy=460.0, cx=320.0, cy=240.0, width=640, height=480)
COLOR = CameraModel(fx=910.0, fy=910.0, cx=640.0, cy=360.0, width=1280, height=720)
SCALE_M = 0.00025


@dataclass(frozen=True)
class Box:
    fwd_m: float
    lat_m: float
    width_m: float
    height_m: float


def pitched_normal(tilt_deg: float) -> np.ndarray:
    """Up-normal for a camera pitched down by ``tilt_deg`` with no roll."""
    tilt = math.radians(tilt_deg)
    return np.array([0.0, -math.cos(tilt), -math.sin(tilt)])


def render_depth(
    normal, height_m: float, boxes=(), camera: CameraModel = DEPTH,
    max_range_m: float = 6.0,
) -> np.ndarray:
    normal = np.asarray(normal, dtype=np.float64)
    normal = normal / np.linalg.norm(normal)
    forward, right = ground_axes(normal)
    v, u = np.mgrid[0:camera.height, 0:camera.width].astype(np.float64)
    rays = np.stack(((u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy,
                     np.ones_like(u)), axis=-1)
    along_normal = rays @ normal
    with np.errstate(divide="ignore", invalid="ignore"):
        z = np.where(along_normal < -1e-6, -height_m / along_normal, np.inf)
        along_forward = rays @ forward
        for box in boxes:
            t = np.where(along_forward > 1e-6, box.fwd_m / along_forward, np.inf)
            lat = t * (rays @ right)
            above = t * along_normal + height_m
            face = (
                np.isfinite(t)
                & (np.abs(lat - box.lat_m) <= box.width_m / 2.0)
                & (above >= 0.0) & (above <= box.height_m)
                & (t < z)
            )
            z = np.where(face, t, z)
    z[~np.isfinite(z) | (z > max_range_m)] = 0.0
    return np.round(z / SCALE_M).astype(np.uint16)


def default_scene() -> np.ndarray:
    """Spec §7.1: camera 0.5 m high, 20° down, 0.2 m box 1.2 m ahead."""
    return render_depth(pitched_normal(20.0), 0.5, [Box(1.2, 0.0, 0.3, 0.2)])


def floor_only(tilt_deg: float = 20.0, height_m: float = 0.5) -> np.ndarray:
    return render_depth(pitched_normal(tilt_deg), height_m)
