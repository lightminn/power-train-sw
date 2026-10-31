"""Mount rotation, base-horizontal frame on the ground plane, SE(2) (spec §4.3, §4.8).

``base_to_optical`` is the single rotation definition shared by the map
geometry and the gyro axis conversion.  Optical frame: x right, y down,
z forward.  Base frame (REP-103): x forward, y left, z up.
"""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import NamedTuple

import numpy as np

# mount (x forward, y left, z up) -> optical: x = -mount y, y = -mount z, z = mount x
_MOUNT_TO_OPTICAL = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])


def _rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def _rot_y(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def _rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def base_to_optical(pitch_down_deg: float, roll_deg: float, yaw_deg: float) -> np.ndarray:
    """R with ``v_opt = R @ v_base``: R = P·Mᵀ, M = Rz(yaw)·Ry(pitch_down)·Rx(roll) (ROS rpy)."""
    mount = (_rot_z(math.radians(yaw_deg)) @ _rot_y(math.radians(pitch_down_deg))
             @ _rot_x(math.radians(roll_deg)))
    return _MOUNT_TO_OPTICAL @ mount.T


class BaseFrame(NamedTuple):
    """Base-horizontal axes on the fitted ground plane, in optical coordinates."""
    f: np.ndarray          # base forward projected on the plane
    l: np.ndarray          # base left on the plane (n × f)
    origin_f: float        # base_link origin projected on (f, l)
    origin_l: float


def base_horizontal_frame(normal: np.ndarray, offset: float, r_base_to_opt: np.ndarray,
                          t_cam_in_base: np.ndarray) -> BaseFrame:
    """Project the body forward axis (not the optical axis) and the base origin onto the plane."""
    n = np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    forward = r_base_to_opt @ np.array([1.0, 0.0, 0.0])
    f = forward - float(forward @ n) * n
    norm = np.linalg.norm(f)
    if norm < 1e-6:
        raise ValueError("base forward axis is perpendicular to the ground plane")
    f = f / norm
    l = np.cross(n, f)
    origin = -(r_base_to_opt @ np.asarray(t_cam_in_base, float))
    return BaseFrame(f, l, float(origin @ f), float(origin @ l))


def to_base_xy(frame: BaseFrame, points_opt: np.ndarray) -> np.ndarray:
    """(N,3) optical points -> (N,2) base-horizontal (forward, left) metres."""
    pts = np.asarray(points_opt, dtype=np.float64)
    return np.stack([pts @ frame.f - frame.origin_f, pts @ frame.l - frame.origin_l], axis=-1)


def base_origin_height(normal: np.ndarray, offset: float, r_base_to_opt: np.ndarray,
                       t_cam_in_base: np.ndarray) -> float:
    """Height of base_link above the fitted plane: n·o + d (independent of body attitude)."""
    n = np.asarray(normal, float)
    n = n / np.linalg.norm(n)
    origin = -(r_base_to_opt @ np.asarray(t_cam_in_base, float))
    return float(n @ origin + offset)


def wrap_angle(angle: float) -> float:
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class Pose2D:
    x: float
    y: float
    yaw: float


IDENTITY = Pose2D(0.0, 0.0, 0.0)


def compose(a: Pose2D, b: Pose2D) -> Pose2D:
    """a ∘ b: express b (given in a's frame) in a's parent frame."""
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(a.x + c * b.x - s * b.y, a.y + s * b.x + c * b.y, wrap_angle(a.yaw + b.yaw))


def inverse(a: Pose2D) -> Pose2D:
    c, s = math.cos(a.yaw), math.sin(a.yaw)
    return Pose2D(-(c * a.x + s * a.y), s * a.x - c * a.y, wrap_angle(-a.yaw))


def transform_points(pose: Pose2D, xy: np.ndarray) -> np.ndarray:
    """(N,2) points in the pose's frame -> parent frame."""
    pts = np.asarray(xy, dtype=np.float64)
    c, s = math.cos(pose.yaw), math.sin(pose.yaw)
    return np.stack([pose.x + c * pts[:, 0] - s * pts[:, 1], pose.y + s * pts[:, 0] + c * pts[:, 1]], axis=-1)
