"""World-coordinate depth renderer for tests — independent of the detector.

A height field ``surface(x, y) -> z`` (metres, world frame: x forward, y left,
z up) is ray-cast from a camera carried by a rover base.  The answers that the
map tests compare against are built from world coordinates, never from the
detector's plane or ``ground_axes``, so a shared geometry mistake cannot pass.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable

import numpy as np

from terrain_overlay.camera import CameraModel
from terrain_map.geometry import base_to_optical

DEPTH_SCALE_M = 0.00025


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1, 0, 0], [0, c, -s], [0, s, c]], float)


def rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0, s], [0, 1, 0], [-s, 0, c]], float)


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]], float)


@dataclass(frozen=True)
class BasePose:
    """Rover base_link in the world (ROS rpy: roll about x, pitch about y, yaw about z)."""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.471
    roll_deg: float = 0.0
    pitch_deg: float = 0.0
    yaw_deg: float = 0.0

    def rotation(self) -> np.ndarray:
        return (rot_z(math.radians(self.yaw_deg)) @ rot_y(math.radians(self.pitch_deg))
                @ rot_x(math.radians(self.roll_deg)))


@dataclass(frozen=True)
class Mount:
    """Depth optical origin in base_link and its mounting angles."""
    t: tuple[float, float, float] = (0.348, 0.0, 0.029)
    pitch_down_deg: float = 15.0
    roll_deg: float = 0.0
    yaw_deg: float = 0.0

    def base_to_optical(self) -> np.ndarray:
        return base_to_optical(self.pitch_down_deg, self.roll_deg, self.yaw_deg)


def camera_in_world(base: BasePose, mount: Mount) -> tuple[np.ndarray, np.ndarray]:
    """(camera origin in world, rotation optical->world)."""
    r_wb = base.rotation()
    origin = np.array([base.x, base.y, base.z]) + r_wb @ np.asarray(mount.t, float)
    r_wo = r_wb @ mount.base_to_optical().T
    return origin, r_wo


def render_depth(
    camera: CameraModel,
    base: BasePose,
    mount: Mount,
    surface: Callable[[np.ndarray, np.ndarray], np.ndarray],
    *,
    max_range_m: float = 6.0,
    step_m: float = 0.02,
) -> np.ndarray:
    """Raw uint16 depth (scale 0.00025 m) of the first hit on ``surface``."""
    origin, r_wo = camera_in_world(base, mount)
    u, v = np.meshgrid(np.arange(camera.width, dtype=float), np.arange(camera.height, dtype=float))
    rays_opt = np.stack([(u - camera.cx) / camera.fx, (v - camera.cy) / camera.fy, np.ones_like(u)], -1)
    rays = rays_opt.reshape(-1, 3) @ r_wo.T          # world direction per unit optical depth
    norm = np.linalg.norm(rays, axis=1)
    count = rays.shape[0]
    hit_t = np.zeros(count)
    lo = np.zeros(count)
    alive = np.ones(count, bool)
    t = 0.0
    steps = int(max_range_m / step_m)
    for _ in range(steps):
        t_next = t + step_m
        idx = np.nonzero(alive)[0]
        if idx.size == 0:
            break
        pts = origin + rays[idx] * (t_next / norm[idx])[:, None]
        below = pts[:, 2] <= surface(pts[:, 0], pts[:, 1])
        crossed = idx[below]
        lo[crossed] = t
        hit_t[crossed] = t_next
        alive[crossed] = False
        t = t_next
    hit = hit_t > 0
    a, b = lo[hit], hit_t[hit]
    r, n = rays[hit], norm[hit]
    for _ in range(20):
        mid = 0.5 * (a + b)
        pts = origin + r * (mid / n)[:, None]
        below = pts[:, 2] <= surface(pts[:, 0], pts[:, 1])
        b = np.where(below, mid, b)
        a = np.where(below, a, mid)
    depth = np.zeros(count)
    depth[hit] = (0.5 * (a + b)) / n                    # distance along the ray -> optical z
    raw = np.round(depth / DEPTH_SCALE_M)
    return np.clip(raw, 0, 65535).astype(np.uint16).reshape(camera.height, camera.width)


# --- surfaces ---------------------------------------------------------------
def flat(z0: float = 0.0):
    return lambda x, y: np.full(np.shape(x), z0, float)


def with_box(base_surface, x0, x1, y0, y1, height):
    def surface(x, y):
        z = base_surface(x, y)
        inside = (x >= x0) & (x <= x1) & (y >= y0) & (y <= y1)
        return np.where(inside, z + height, z)
    return surface


def with_ditch(base_surface, x0, x1, y0, y1, depth):
    return with_box(base_surface, x0, x1, y0, y1, -depth)


def stairs(x_start, tread_m, rise_m, steps):
    def surface(x, y):
        k = np.clip(np.floor((x - x_start) / tread_m) + 1, 0, steps)
        return np.where(x < x_start, 0.0, k * rise_m)
    return surface
