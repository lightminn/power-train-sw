"""Ground plane + obstacle classification from raw L515 depth (spec §4.2).

Operator visual aid only — never a control or safety input.  Every plane
lifetime decision uses ``frame_rx_s`` (the depth's receive monotonic time),
never the processing time.
"""
from __future__ import annotations

from dataclasses import dataclass, field
import math

import cv2
import numpy as np

from .camera import CameraModel

MIN_VALID_FRACTION = 0.05
VOXEL_M = 0.02
MIN_GROUND_POINTS = 200
RANSAC_SAMPLE_MAX = 3000
RANSAC_ITERATIONS = 1000
RANSAC_CHUNK = 100
RANSAC_INLIER_M = 0.02
RANSAC_SEED = 20261017
MAX_TILT_DEG = 40.0
CAMERA_HEIGHT_RANGE_M = (0.25, 0.80)  # open interval
FLOOR_BAND_M = 0.03
OBSTACLE_MIN_H_M = 0.05
OBSTACLE_MAX_H_M = 2.0
OBSTACLE_MAX_FWD_M = 3.0
OPEN_KERNEL = np.ones((5, 5), np.uint8)
ZONE_PERCENTILE = 2.0


@dataclass(frozen=True)
class DetectorConfig:
    depth_scale_m: float = 0.00025
    ground_hold_s: float = 0.5
    path_half_width_m: float = 0.4445
    front_offset_m: float = 0.0
    # Minimum floor candidates and voxels for a plane.  The terrain map runs on
    # decimated depth and lowers it (terrain_map spec §4.2); the overlay keeps 200.
    min_ground_points: int = MIN_GROUND_POINTS


@dataclass
class TerrainResult:
    status: str  # OK / NO_GROUND / SENSOR_ERROR
    reason: str | None
    ground_state: str | None = None  # FRESH / HELD when status is OK
    fresh_rx_s: float | None = None
    camera_height_m: float | None = None
    camera_tilt_deg: float | None = None
    zones: dict[str, float | None] = field(
        default_factory=lambda: {"left": None, "center": None, "right": None},
    )
    obstacle_points: np.ndarray = field(
        default_factory=lambda: np.zeros((0, 3), np.float32),
    )
    obstacle_fwd: np.ndarray = field(default_factory=lambda: np.zeros(0, np.float32))
    obstacle_lat: np.ndarray = field(default_factory=lambda: np.zeros(0, np.float32))
    floor_mask: np.ndarray | None = None
    obstacle_mask: np.ndarray | None = None
    # For the terrain map (None unless status is OK): the camera-optical points
    # (H, W, 3), their valid mask and the plane (normal, offset) with
    # normal·p + offset = height above the plane (camera side positive).
    points: np.ndarray | None = None
    valid: np.ndarray | None = None
    ground_plane: tuple[np.ndarray, float] | None = None


def tilt_deg(normal: np.ndarray) -> float:
    return math.degrees(math.acos(min(1.0, abs(float(normal[1])))))


def ground_axes(normal: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """Orthogonal forward/right axes lying in the ground plane (spec §4.2-7)."""
    z_axis = np.array([0.0, 0.0, 1.0])
    forward = z_axis - float(z_axis @ normal) * normal
    forward /= np.linalg.norm(forward)
    right = np.cross(forward, normal)
    right /= np.linalg.norm(right)
    if right[0] < 0.0:
        right = -right
    return forward, right


class TerrainDetector:
    def __init__(self, depth_camera: CameraModel, config: DetectorConfig | None = None) -> None:
        self.camera = depth_camera
        self.config = config or DetectorConfig()
        h, w = depth_camera.height, depth_camera.width
        u, v = np.meshgrid(np.arange(w, dtype=np.float32), np.arange(h, dtype=np.float32))
        self._xn = (u - depth_camera.cx) / depth_camera.fx
        self._yn = (v - depth_camera.cy) / depth_camera.fy
        self._lower_half = v > h * 0.5
        self._normal: np.ndarray | None = None
        self._offset: float | None = None
        self._fresh_rx_s: float | None = None
        self._expired = False

    def process(self, depth: np.ndarray, frame_rx_s: float) -> TerrainResult:
        if not isinstance(depth, np.ndarray) or depth.dtype != np.uint16:
            raise TypeError("depth must be a raw uint16 array (not metres)")
        if depth.shape != (self.camera.height, self.camera.width):
            raise ValueError(f"depth shape {depth.shape} does not match camera info")
        t = float(frame_rx_s)
        z = depth.astype(np.float32) * np.float32(self.config.depth_scale_m)
        valid = z > 0.0
        if valid.mean() < MIN_VALID_FRACTION:
            return TerrainResult("SENSOR_ERROR", "DEPTH_EMPTY")
        points = np.stack((self._xn * z, self._yn * z, z), axis=-1)

        if (
            self._normal is not None
            and t - self._fresh_rx_s > self.config.ground_hold_s
        ):
            self._normal = self._offset = None
            self._expired = True

        fitted = self._fit_ground(points[valid & self._lower_half])
        if fitted is not None:
            self._normal, self._offset = fitted
            self._fresh_rx_s = t
            self._expired = False
            state = "FRESH"
        elif self._normal is not None:
            state = "HELD"
        else:
            reason = "GROUND_EXPIRED" if self._expired else "GROUND_NOT_FOUND"
            return TerrainResult("NO_GROUND", reason)
        return self._classify(points, valid, state)

    def _fit_ground(self, candidates: np.ndarray) -> tuple[np.ndarray, float] | None:
        if len(candidates) < self.config.min_ground_points:
            return None
        keys = np.floor(candidates / VOXEL_M).astype(np.int64) + (1 << 20)
        packed = (keys[:, 0] << 42) | (keys[:, 1] << 21) | keys[:, 2]
        _, first = np.unique(packed, return_index=True)
        voxels = candidates[np.sort(first)].astype(np.float64)
        if len(voxels) < self.config.min_ground_points:
            return None
        rng = np.random.default_rng(RANSAC_SEED)
        if len(voxels) > RANSAC_SAMPLE_MAX:
            sample = voxels[rng.choice(len(voxels), RANSAC_SAMPLE_MAX, replace=False)]
        else:
            sample = voxels
        sample32 = sample.astype(np.float32)
        triplets = rng.integers(0, len(sample), size=(RANSAC_ITERATIONS, 3))
        best_count, best = -1, None
        for start in range(0, RANSAC_ITERATIONS, RANSAC_CHUNK):
            chosen = triplets[start:start + RANSAC_CHUNK]
            p0, p1, p2 = sample[chosen[:, 0]], sample[chosen[:, 1]], sample[chosen[:, 2]]
            normals = np.cross(p1 - p0, p2 - p0)
            norms = np.linalg.norm(normals, axis=1)
            usable = norms > 1e-9
            if not usable.any():
                continue
            normals = normals[usable] / norms[usable, None]
            offsets = -np.einsum("ij,ij->i", normals, p0[usable])
            distances = np.abs(sample32 @ normals.T.astype(np.float32) + offsets.astype(np.float32))
            counts = (distances < RANSAC_INLIER_M).sum(axis=0)
            index = int(np.argmax(counts))
            if counts[index] > best_count:
                best_count, best = int(counts[index]), (normals[index], offsets[index])
        if best is None:
            return None
        normal, offset = best
        inliers = voxels[np.abs(voxels @ normal + offset) < RANSAC_INLIER_M]
        if len(inliers) < 3:
            return None
        centroid = inliers.mean(axis=0)
        centered = inliers - centroid
        _, vectors = np.linalg.eigh(centered.T @ centered)
        normal = vectors[:, 0]
        offset = -float(normal @ centroid)
        if offset < 0.0:
            normal, offset = -normal, -offset
        low, high = CAMERA_HEIGHT_RANGE_M
        if tilt_deg(normal) > MAX_TILT_DEG or not low < offset < high:
            return None
        return normal, offset

    def _classify(self, points: np.ndarray, valid: np.ndarray, state: str) -> TerrainResult:
        normal = self._normal.astype(np.float32)
        forward, right = (axis.astype(np.float32) for axis in ground_axes(self._normal))
        height = points @ normal + np.float32(self._offset)
        fwd = points @ forward
        lat = points @ right
        floor = valid & (np.abs(height) < FLOOR_BAND_M)
        obstacle = (
            valid
            & (height > OBSTACLE_MIN_H_M) & (height < OBSTACLE_MAX_H_M)
            & (fwd > 0.0) & (fwd < OBSTACLE_MAX_FWD_M)
        )
        obstacle = cv2.morphologyEx(
            obstacle.astype(np.uint8), cv2.MORPH_OPEN, OPEN_KERNEL,
        ).astype(bool) & valid
        obstacle_fwd = fwd[obstacle]
        obstacle_lat = lat[obstacle]
        half_width = self.config.path_half_width_m
        bands = {
            "left": obstacle_lat < -half_width,
            "center": np.abs(obstacle_lat) <= half_width,
            "right": obstacle_lat > half_width,
        }
        zones: dict[str, float | None] = {}
        for name, band in bands.items():
            zones[name] = (
                max(0.0, float(np.percentile(obstacle_fwd[band], ZONE_PERCENTILE))
                    - self.config.front_offset_m)
                if band.any() else None
            )
        return TerrainResult(
            status="OK", reason=None, ground_state=state,
            fresh_rx_s=self._fresh_rx_s,
            camera_height_m=float(self._offset),
            camera_tilt_deg=tilt_deg(self._normal),
            zones=zones,
            obstacle_points=points[obstacle],
            obstacle_fwd=obstacle_fwd,
            obstacle_lat=obstacle_lat,
            floor_mask=floor,
            obstacle_mask=obstacle,
            points=points,
            valid=valid,
            ground_plane=(self._normal.copy(), float(self._offset)),
        )
