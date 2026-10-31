"""Support-plane check and per-frame local cells (spec §4.2, §4.3).

Heights are relative to *this frame's* fitted floor plane, so the frame's own
classification is consistent; frames are never averaged together in height
(spec §1 premises, r4).  A frame keeps its filtered points so the map can bin them
straight into world cells with the final pose (r14, spec §4.5).
"""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from terrain_overlay.camera import CameraModel
from terrain_overlay.detector import TerrainResult
from .config import MapConfig
from .contract import DITCH, FREE, OBSTACLE
from .geometry import (BaseFrame, Pose2D, base_horizontal_frame, base_origin_height, base_to_optical, to_base_xy,
                       transform_points)

__all__ = [
    "FREE", "OBSTACLE", "DITCH", "FrameGeometry", "SupportResult", "LocalScan",
    "decimate", "make_frame", "support_check", "cell_height", "cell_stats", "classify", "build_local_scan",
    "world_cells",
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
    # r14: the filtered points behind the cells (base forward, base left, plane height; float32).
    # None for synthetic cell-only scans (tests, B fixtures), which the map fuses by cell centre (r13).
    px: np.ndarray | None = None
    py: np.ndarray | None = None
    ph: np.ndarray | None = None

    def __post_init__(self) -> None:
        given = [v is not None for v in (self.px, self.py, self.ph)]
        if any(given) and not (all(given) and len(self.px) == len(self.py) == len(self.ph)):
            raise ValueError("px, py and ph must be given together with one length")


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
    if cfg.cell_trim_max_points > 0:
        k = min(k, cfg.cell_trim_max_points)
    k = min(k, (n - 1) // 2)
    upper, lower = ordered[n - 1 - k], ordered[k]
    return float(upper if severity(np.array([upper]), cfg)[0] >= severity(np.array([lower]), cfg)[0] else lower)


def classify(h: np.ndarray, cfg: MapConfig) -> np.ndarray:
    code = np.full(np.shape(h), FREE, np.uint8)
    code[h >= cfg.obstacle_height_m] = OBSTACLE
    code[h <= -cfg.ditch_depth_m] = DITCH
    return code


def _ordered_float32_key(h32: np.ndarray) -> np.ndarray:
    """IEEE-754 float32 bits mapped to 0..2^32-1 with the same order as the values (exact, no quantisation)."""
    bits = h32.view(np.uint32).astype(np.int64)
    return np.where(bits >= (1 << 31), (1 << 32) - 1 - bits, bits + (1 << 31))


def cell_stats(ix: np.ndarray, iy: np.ndarray, h: np.ndarray, cfg: MapConfig):
    """Per-cell trimmed severity statistic (spec §4.3-5, r14) shared by local and world cells.

    Heights are taken as float32 (what a LocalScan stores) and ordered exactly.  Returns
    ``(ix, iy, height, code, count)`` for cells with ``count >= min_points_per_cell``, ordered by
    ``(ix, iy)``.  One int64 sort on (cell number in the frame's cell bounding box, ordered float32 bits)
    replaces the two-key lexsort; an absurd box falls back to the exact two-key sort (same result).
    """
    ix = np.asarray(ix, np.int64)
    iy = np.asarray(iy, np.int64)
    h = np.asarray(h, np.float32)
    if len(h) == 0:
        return (np.zeros(0, np.int64), np.zeros(0, np.int64), np.zeros(0, np.float64), np.zeros(0, np.uint8),
                np.zeros(0, np.int64))
    if not np.isfinite(h).all():
        raise ValueError("cell heights must be finite")
    lo_x, lo_y = int(ix.min()), int(iy.min())
    span_x, span_y = int(ix.max()) - lo_x + 1, int(iy.max()) - lo_y + 1     # Python ints: no overflow
    if span_x * span_y < (1 << 31):
        dense = (ix - lo_x) * span_y + (iy - lo_y)
        order = np.argsort(dense * (1 << 32) + _ordered_float32_key(h), kind="stable")
    else:                                   # absurd extent: the exact sort on (ix, iy, h) itself
        order = np.lexsort((h, iy, ix))
    sx, sy, hs = ix[order], iy[order], h[order].astype(np.float64)
    starts = np.flatnonzero(np.r_[True, (sx[1:] != sx[:-1]) | (sy[1:] != sy[:-1])])
    counts = np.diff(np.r_[starts, len(hs)])
    good = counts >= cfg.min_points_per_cell
    starts, counts = starts[good], counts[good]
    k = np.maximum(1, np.floor(cfg.cell_trim_fraction * counts).astype(np.int64))
    if cfg.cell_trim_max_points > 0:
        k = np.minimum(k, cfg.cell_trim_max_points)
    k = np.minimum(k, (counts - 1) // 2)
    upper, lower = hs[starts + counts - 1 - k], hs[starts + k]
    height = np.where(severity(upper, cfg) >= severity(lower, cfg), upper, lower)
    first = order[starts]
    return ix[first], iy[first], height, classify(height, cfg), counts


def build_local_scan(result: TerrainResult, frame: FrameGeometry, stamp_s: float, cfg: MapConfig) -> LocalScan:
    pts = _valid_points(result)
    heights = pts @ frame.normal + frame.offset
    forward = pts @ frame.base.f
    keep = (forward >= cfg.range_min_m) & (forward <= cfg.range_max_m) & (np.abs(heights) <= cfg.max_abs_height_m)
    pts, heights = pts[keep], heights[keep]
    xy = to_base_xy(frame.base, pts).astype(np.float32)
    ph = heights.astype(np.float32)
    ix = np.floor(xy[:, 0].astype(np.float64) / cfg.cell_m).astype(np.int64)
    iy = np.floor(xy[:, 1].astype(np.float64) / cfg.cell_m).astype(np.int64)
    cx, cy, h_cell, code, counts = cell_stats(ix, iy, ph, cfg)
    return LocalScan(
        ix=cx.astype(np.int32), iy=cy.astype(np.int32), h_m=h_cell.astype(np.float32), code=code,
        count=counts.astype(np.int32), stamp_s=float(stamp_s),
        px=np.ascontiguousarray(xy[:, 0]), py=np.ascontiguousarray(xy[:, 1]), ph=ph,
    )


def world_cells(scan: LocalScan, pose: Pose2D, cfg: MapConfig):
    """World cells one frame votes for: ``(gx, gy, height, code)``, one entry per world cell (spec §4.5).

    Points are moved by ``pose`` once and binned where they land (r14).  A cell-only scan keeps the r13 rule:
    each local cell centre is moved, and two local cells in one world cell keep the more severe one.
    """
    cell = cfg.cell_m
    if scan.px is not None:
        if len(scan.px) == 0:
            return (np.zeros(0, np.int64), np.zeros(0, np.int64), np.zeros(0, np.float64), np.zeros(0, np.uint8))
        finite = np.isfinite(scan.px) & np.isfinite(scan.py) & np.isfinite(scan.ph)
        world = transform_points(pose, np.stack([scan.px[finite], scan.py[finite]], axis=1).astype(np.float64))
        gx, gy, height, code, _ = cell_stats(np.floor(world[:, 0] / cell).astype(np.int64),
                                             np.floor(world[:, 1] / cell).astype(np.int64), scan.ph[finite], cfg)
        return gx, gy, height, code
    centres = np.stack([(scan.ix + 0.5) * cell, (scan.iy + 0.5) * cell], axis=1)
    world = np.floor(transform_points(pose, centres) / cell).astype(np.int64)
    gx, gy = world[:, 0], world[:, 1]
    h = scan.h_m.astype(np.float64)
    key = (gx + (1 << 30)) * (1 << 31) + (gy + (1 << 30))
    order = np.lexsort((-severity(h, cfg), key))
    first = order[np.r_[True, key[order][1:] != key[order][:-1]]] if len(key) else order
    return gx[first], gy[first], h[first], scan.code[first]
