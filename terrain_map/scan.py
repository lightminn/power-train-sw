"""Support-plane check and per-frame local cells (spec §4.2, §4.3).

Heights are relative to *this frame's* fitted floor plane, so the frame's own
classification is consistent; frames are never averaged together in height
(spec §1 premises, r4).
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from terrain_overlay.camera import CameraModel
from terrain_overlay.detector import TerrainResult
from .config import MapConfig
from .contract import DITCH, FREE, OBSTACLE
from .geometry import BaseFrame, base_horizontal_frame, base_origin_height, base_to_optical, to_base_xy

__all__ = [
    "FREE", "OBSTACLE", "DITCH", "FrameGeometry", "SupportResult", "LocalScan",
    "decimate", "make_frame", "support_check", "cell_height", "classify", "build_local_scan",
]


@dataclass(frozen=True)
class FrameGeometry:
    normal: np.ndarray
    offset: float
    rotation: np.ndarray          # base -> optical
    translation: np.ndarray       # camera origin in base
    base: BaseFrame


@dataclass(frozen=True)
class SupportResult:
    ok: bool
    reason: str | None            # None / "near_band" / "base_height"
    near_count: int
    near_median_m: float
    base_height_m: float


@dataclass(frozen=True)
class LocalScan:
    ix: np.ndarray                # int32 cell index along base forward
    iy: np.ndarray                # int32 cell index along base left
    h_m: np.ndarray               # float32 cell height above this frame's floor
    code: np.ndarray              # uint8 FREE / OBSTACLE / DITCH
    count: np.ndarray             # int32 points per cell
    stamp_s: float


def decimate(depth: np.ndarray, camera: CameraModel, stride: int) -> tuple[np.ndarray, CameraModel]:
    """Keep every ``stride``-th pixel; intrinsics scale by 1/stride (pixel u' = u / s)."""
    if depth.shape != (camera.height, camera.width):
        # Checked before decimating: the small array would otherwise define the camera size.
        raise ValueError(f"depth shape {depth.shape} does not match camera info")
    if stride == 1:
        return depth, camera
    small = np.ascontiguousarray(depth[::stride, ::stride])
    cam = CameraModel(
        fx=camera.fx / stride, fy=camera.fy / stride, cx=camera.cx / stride, cy=camera.cy / stride,
        width=small.shape[1], height=small.shape[0],
    )
    return small, cam


def make_frame(result: TerrainResult, cfg: MapConfig) -> FrameGeometry:
    if result.ground_plane is None:
        raise ValueError("result has no ground plane")
    normal, offset = result.ground_plane
    rotation = base_to_optical(cfg.camera_pitch_down_deg, cfg.camera_roll_deg, cfg.camera_yaw_deg)
    translation = np.array([cfg.camera_x_m, cfg.camera_y_m, cfg.camera_z_m])
    return FrameGeometry(np.asarray(normal, float), float(offset), rotation, translation,
                         base_horizontal_frame(normal, offset, rotation, translation))


def _valid_points(result: TerrainResult) -> np.ndarray:
    return result.points.reshape(-1, 3)[result.valid.reshape(-1)].astype(np.float64)


def support_check(result: TerrainResult, frame: FrameGeometry, cfg: MapConfig) -> SupportResult:
    """Is the fitted plane the surface the rover stands on? (near band + optional base height)."""
    pts = _valid_points(result)
    heights = pts @ frame.normal + frame.offset
    forward = pts @ frame.base.f                      # from the camera foot along base forward
    near = (forward >= cfg.range_min_m) & (forward <= cfg.range_min_m + cfg.support_band_m)
    count = int(near.sum())
    median = float(np.median(heights[near])) if count else math.inf
    base_h = base_origin_height(frame.normal, frame.offset, frame.rotation, frame.translation)
    if count < cfg.support_min_points or abs(median) > cfg.support_tol_m:
        return SupportResult(False, "near_band", count, median, base_h)
    if cfg.support_height_check_enabled and abs(base_h - cfg.support_base_height_m) > cfg.support_height_tol_m:
        return SupportResult(False, "base_height", count, median, base_h)
    return SupportResult(True, None, count, median, base_h)


def severity(h: np.ndarray, cfg: MapConfig) -> np.ndarray:
    return np.maximum(h / cfg.obstacle_height_m, -h / cfg.ditch_depth_m)


def cell_height(values: np.ndarray, cfg: MapConfig) -> float:
    """Trimmed order statistic of one cell, choosing the more severe side (spec §4.3-5)."""
    ordered = np.sort(np.asarray(values, float))
    n = len(ordered)
    k = max(1, int(math.floor(cfg.cell_trim_fraction * n)))
    k = min(k, (n - 1) // 2)
    upper, lower = ordered[n - 1 - k], ordered[k]
    return float(upper if severity(np.array([upper]), cfg)[0] >= severity(np.array([lower]), cfg)[0] else lower)


def classify(h: np.ndarray, cfg: MapConfig) -> np.ndarray:
    code = np.full(np.shape(h), FREE, np.uint8)
    code[h >= cfg.obstacle_height_m] = OBSTACLE
    code[h <= -cfg.ditch_depth_m] = DITCH
    return code


def build_local_scan(result: TerrainResult, frame: FrameGeometry, stamp_s: float, cfg: MapConfig) -> LocalScan:
    pts = _valid_points(result)
    heights = pts @ frame.normal + frame.offset
    forward = pts @ frame.base.f
    keep = (forward >= cfg.range_min_m) & (forward <= cfg.range_max_m) & (np.abs(heights) <= cfg.max_abs_height_m)
    pts, heights = pts[keep], heights[keep]
    xy = to_base_xy(frame.base, pts)
    ix = np.floor(xy[:, 0] / cfg.cell_m).astype(np.int64)
    iy = np.floor(xy[:, 1] / cfg.cell_m).astype(np.int64)
    empty = LocalScan(np.zeros(0, np.int32), np.zeros(0, np.int32), np.zeros(0, np.float32),
                      np.zeros(0, np.uint8), np.zeros(0, np.int32), float(stamp_s))
    if len(heights) == 0:
        return empty
    cell_id = (ix + (1 << 20)) * (1 << 21) + (iy + (1 << 20))
    order = np.lexsort((heights, cell_id))
    ids, h_sorted = cell_id[order], heights[order]
    starts = np.flatnonzero(np.r_[True, ids[1:] != ids[:-1]])
    counts = np.diff(np.r_[starts, len(ids)])
    good = counts >= cfg.min_points_per_cell
    starts, counts = starts[good], counts[good]
    if len(starts) == 0:
        return empty
    k = np.maximum(1, np.floor(cfg.cell_trim_fraction * counts).astype(np.int64))
    k = np.minimum(k, (counts - 1) // 2)
    upper = h_sorted[starts + counts - 1 - k]
    lower = h_sorted[starts + k]
    h_cell = np.where(severity(upper, cfg) >= severity(lower, cfg), upper, lower)
    first = order[starts]
    return LocalScan(
        ix=ix[first].astype(np.int32), iy=iy[first].astype(np.int32),
        h_m=h_cell.astype(np.float32), code=classify(h_cell, cfg),
        count=counts.astype(np.int32), stamp_s=float(stamp_s),
    )
