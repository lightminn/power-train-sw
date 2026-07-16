"""Pure fixed-shape terrain-kernel contract and NumPy authority."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class TerrainKernelConfig:
    """Static array and metric bounds shared by terrain compute backends."""

    depth_shape_px: tuple[int, int]
    grid_shape: tuple[int, int]
    grid_resolution_m: float
    grid_x_range_m: tuple[float, float]
    grid_y_range_m: tuple[float, float]
    min_depth_m: float
    max_depth_m: float

    @classmethod
    def from_estimator_config(cls, config) -> "TerrainKernelConfig":
        sampled_shape = (
            len(range(config.roi_rows[0], config.roi_rows[1], config.stride)),
            len(range(config.roi_cols[0], config.roi_cols[1], config.stride)),
        )
        grid_shape = (
            int(
                round(
                    (config.grid_x_range_m[1] - config.grid_x_range_m[0])
                    / config.grid_resolution_m
                )
            ),
            int(
                round(
                    (config.grid_y_range_m[1] - config.grid_y_range_m[0])
                    / config.grid_resolution_m
                )
            ),
        )
        return cls(
            depth_shape_px=sampled_shape,
            grid_shape=grid_shape,
            grid_resolution_m=float(config.grid_resolution_m),
            grid_x_range_m=tuple(
                float(value) for value in config.grid_x_range_m
            ),
            grid_y_range_m=tuple(
                float(value) for value in config.grid_y_range_m
            ),
            min_depth_m=float(config.min_depth_m),
            max_depth_m=float(config.max_depth_m),
        )

    def __post_init__(self) -> None:
        for shape, name in (
            (self.depth_shape_px, "depth_shape_px"),
            (self.grid_shape, "grid_shape"),
        ):
            if (
                len(shape) != 2
                or any(
                    isinstance(value, bool)
                    or not isinstance(value, int)
                    or value < 1
                    for value in shape
                )
            ):
                raise ValueError(f"{name} must contain two positive integers")
        finite = (
            self.grid_resolution_m,
            *self.grid_x_range_m,
            *self.grid_y_range_m,
            self.min_depth_m,
            self.max_depth_m,
        )
        if not all(math.isfinite(value) for value in finite):
            raise ValueError("terrain kernel configuration must be finite")
        if self.grid_resolution_m <= 0.0:
            raise ValueError("grid_resolution_m must be positive")
        if not (
            self.grid_x_range_m[0] < self.grid_x_range_m[1]
            and self.grid_y_range_m[0] < self.grid_y_range_m[1]
            and 0.0 < self.min_depth_m < self.max_depth_m
        ):
            raise ValueError(
                "terrain kernel ranges must be positive and ordered"
            )


@dataclass(frozen=True)
class TerrainKernelResult:
    """Fixed-shape output at the JIT-to-CPU boundary."""

    points_m: np.ndarray
    height_m: np.ndarray
    observed_count: np.ndarray
    slope_x: np.ndarray
    slope_y: np.ndarray
    roughness_m: np.ndarray
    confidence: np.ndarray
    valid_mask: np.ndarray
    stamp_s: np.ndarray


def validate_kernel_inputs(
    depth_roi: np.ndarray,
    point_mask: np.ndarray,
    point_confidence: np.ndarray,
    *,
    depth_scale_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    rotation: np.ndarray,
    translation_m: np.ndarray,
    stamp_s: float,
    config: TerrainKernelConfig,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Reject shape/dtype drift before a backend can trigger compilation."""
    depth = np.asarray(depth_roi)
    mask = np.asarray(point_mask)
    confidence = np.asarray(point_confidence)
    transform = np.asarray(rotation)
    translation = np.asarray(translation_m)
    if depth.shape != config.depth_shape_px:
        raise ValueError(
            f"depth_roi must retain fixed depth shape {config.depth_shape_px}"
        )
    if depth.dtype != np.uint16:
        raise TypeError("depth_roi must retain uint16 dtype")
    if mask.shape != config.depth_shape_px:
        raise ValueError("point_mask must match the fixed depth shape")
    if mask.dtype != np.bool_:
        raise TypeError("point_mask must retain boolean dtype")
    if confidence.shape != config.depth_shape_px:
        raise ValueError("point_confidence must match the fixed depth shape")
    if confidence.dtype != np.float64:
        raise TypeError("point_confidence must retain float64 dtype")
    if transform.shape != (3, 3):
        raise ValueError("rotation must have fixed shape (3, 3)")
    if transform.dtype != np.float64:
        raise TypeError("rotation must retain float64 dtype")
    if translation.shape != (3,):
        raise ValueError("translation_m must have fixed shape (3,)")
    if translation.dtype != np.float64:
        raise TypeError("translation_m must retain float64 dtype")
    scalars = (depth_scale_m, fx, fy, cx, cy, stamp_s)
    if not all(math.isfinite(float(value)) for value in scalars):
        raise ValueError("terrain kernel scalar inputs must be finite")
    if depth_scale_m <= 0.0 or fx <= 0.0 or fy <= 0.0:
        raise ValueError("depth scale and focal lengths must be positive")
    if not np.all(np.isfinite(confidence)) or np.any(
        (confidence < 0.0) | (confidence > 1.0)
    ):
        raise ValueError("point_confidence must be finite and within [0, 1]")
    if not np.all(np.isfinite(transform)) or not np.all(
        np.isfinite(translation)
    ):
        raise ValueError("terrain kernel transform must be finite")
    if not np.allclose(
        transform @ transform.T, np.eye(3), rtol=1e-7, atol=1e-7
    ):
        raise ValueError("rotation must be orthonormal")
    return depth, mask, confidence, transform, translation


def cell_statistics_numpy(
    cell_ids: np.ndarray,
    heights_m: np.ndarray,
    point_confidence: np.ndarray,
    *,
    shape: tuple[int, int],
    stamp_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """Compute exact grouped medians using the production NumPy ordering."""
    size = shape[0] * shape[1]
    counts = np.bincount(cell_ids, minlength=size).astype(np.int32)
    occupied = np.flatnonzero(counts)

    def grouped_median(values: np.ndarray, *, fill: float) -> np.ndarray:
        output = np.full(size, fill, dtype=float)
        order = np.lexsort((values, cell_ids))
        sorted_values = values[order]
        occupied_counts = counts[occupied]
        stops = np.cumsum(occupied_counts)
        starts = stops - occupied_counts
        lower = starts + (occupied_counts - 1) // 2
        upper = starts + occupied_counts // 2
        output[occupied] = 0.5 * (sorted_values[lower] + sorted_values[upper])
        return output

    height = grouped_median(heights_m, fill=np.nan)
    absolute_deviation = np.abs(heights_m - height[cell_ids])
    roughness = grouped_median(absolute_deviation, fill=np.nan)
    confidence = grouped_median(point_confidence, fill=0.0)
    stamps = np.full(size, np.nan, dtype=float)
    stamps[occupied] = stamp_s
    return (
        height.reshape(shape),
        counts.reshape(shape),
        roughness.reshape(shape),
        confidence.reshape(shape),
        stamps.reshape(shape),
    )


def _shift(array: np.ndarray, dx: int, dy: int, fill) -> np.ndarray:
    output = np.full(array.shape, fill, dtype=array.dtype)
    source_x_start = max(0, -dx)
    source_x_stop = min(array.shape[0], array.shape[0] - dx)
    source_y_start = max(0, -dy)
    source_y_stop = min(array.shape[1], array.shape[1] - dy)
    if source_x_start >= source_x_stop or source_y_start >= source_y_stop:
        return output
    output[
        source_x_start + dx:source_x_stop + dx,
        source_y_start + dy:source_y_stop + dy,
    ] = array[source_x_start:source_x_stop, source_y_start:source_y_stop]
    return output


def finite_differences_numpy(
    height_m: np.ndarray,
    mask: np.ndarray,
    resolution_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    slope_x = np.full(height_m.shape, np.nan, dtype=float)
    slope_y = np.full(height_m.shape, np.nan, dtype=float)
    for axis, output in ((0, slope_x), (1, slope_y)):
        plus_height = _shift(
            height_m, -1 if axis == 0 else 0, -1 if axis == 1 else 0, np.nan
        )
        minus_height = _shift(
            height_m, 1 if axis == 0 else 0, 1 if axis == 1 else 0, np.nan
        )
        plus_mask = _shift(
            mask, -1 if axis == 0 else 0, -1 if axis == 1 else 0, False
        )
        minus_mask = _shift(
            mask, 1 if axis == 0 else 0, 1 if axis == 1 else 0, False
        )
        central = mask & plus_mask & minus_mask
        output[central] = (plus_height[central] - minus_height[central]) / (
            2.0 * resolution_m
        )
        forward = mask & plus_mask & ~minus_mask
        output[forward] = (
            plus_height[forward] - height_m[forward]
        ) / resolution_m
        backward = mask & minus_mask & ~plus_mask
        output[backward] = (
            height_m[backward] - minus_height[backward]
        ) / resolution_m
    return slope_x, slope_y


def build_terrain_grid_numpy(
    depth_roi: np.ndarray,
    *,
    point_mask: np.ndarray,
    point_confidence: np.ndarray,
    depth_scale_m: float,
    fx: float,
    fy: float,
    cx: float,
    cy: float,
    rotation: np.ndarray,
    translation_m: np.ndarray,
    stamp_s: float,
    config: TerrainKernelConfig,
) -> TerrainKernelResult:
    """Deproject, gravity-align, and scatter one fixed-shape ROI with NumPy."""
    validated = validate_kernel_inputs(
        depth_roi,
        point_mask,
        point_confidence,
        depth_scale_m=depth_scale_m,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        rotation=rotation,
        translation_m=translation_m,
        stamp_s=stamp_s,
        config=config,
    )
    depth, mask, confidence_in, transform, translation = validated
    rows, cols = np.indices(config.depth_shape_px, dtype=float)
    depth_m = depth.astype(float, copy=False) * depth_scale_m
    camera = np.stack(
        (
            (cols - cx) * depth_m / fx,
            (rows - cy) * depth_m / fy,
            depth_m,
        ),
        axis=-1,
    )
    points = camera @ transform.T + translation
    finite_points = np.all(np.isfinite(points), axis=2)
    valid_depth = (
        np.isfinite(depth_m)
        & (depth_m >= config.min_depth_m)
        & (depth_m <= config.max_depth_m)
    )
    safe_x = np.where(
        finite_points,
        points[..., 0],
        config.grid_x_range_m[0] - config.grid_resolution_m,
    )
    safe_y = np.where(
        finite_points,
        points[..., 1],
        config.grid_y_range_m[0] - config.grid_resolution_m,
    )
    x_index = np.floor(
        (safe_x - config.grid_x_range_m[0]) / config.grid_resolution_m
    ).astype(np.int64)
    y_index = np.floor(
        (safe_y - config.grid_y_range_m[0]) / config.grid_resolution_m
    ).astype(np.int64)
    inside = (
        mask
        & valid_depth
        & finite_points
        & (x_index >= 0)
        & (x_index < config.grid_shape[0])
        & (y_index >= 0)
        & (y_index < config.grid_shape[1])
    )
    if np.any(inside):
        cell_ids = x_index[inside] * config.grid_shape[1] + y_index[inside]
        height, counts, roughness, confidence, stamps = cell_statistics_numpy(
            cell_ids,
            points[..., 2][inside],
            confidence_in[inside],
            shape=config.grid_shape,
            stamp_s=stamp_s,
        )
    else:
        height = np.full(config.grid_shape, np.nan, dtype=float)
        counts = np.zeros(config.grid_shape, dtype=np.int32)
        roughness = np.full(config.grid_shape, np.nan, dtype=float)
        confidence = np.zeros(config.grid_shape, dtype=float)
        stamps = np.full(config.grid_shape, np.nan, dtype=float)
    valid = counts > 0
    slope_x, slope_y = finite_differences_numpy(
        height, valid, config.grid_resolution_m
    )
    return TerrainKernelResult(
        points_m=points,
        height_m=height,
        observed_count=counts,
        slope_x=slope_x,
        slope_y=slope_y,
        roughness_m=roughness,
        confidence=confidence,
        valid_mask=valid,
        stamp_s=stamps,
    )


__all__ = (
    "TerrainKernelConfig",
    "TerrainKernelResult",
    "build_terrain_grid_numpy",
    "cell_statistics_numpy",
    "finite_differences_numpy",
    "validate_kernel_inputs",
)
