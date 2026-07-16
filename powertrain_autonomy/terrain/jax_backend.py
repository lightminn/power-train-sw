"""Optional fixed-shape JAX terrain kernel.

JAX is intentionally imported only by this module.  The terrain package does
not import this backend, so systems without JAX retain the NumPy authority and
can import :mod:`powertrain_autonomy.terrain` unchanged.
"""
from __future__ import annotations

from collections.abc import Callable
import math

import jax
import jax.numpy as jnp
import numpy as np

from .kernel import (
    TerrainKernelConfig,
    TerrainKernelResult,
    validate_kernel_inputs,
)


_FIXED_DEPTH_SHAPE = (60, 80)
_compiled_config: TerrainKernelConfig | None = None
_compiled_kernel: Callable | None = None


def _normalize_config(config) -> TerrainKernelConfig:
    if isinstance(config, TerrainKernelConfig):
        return config
    return TerrainKernelConfig.from_estimator_config(config)


def _require_fixed_config(config: TerrainKernelConfig) -> None:
    if config.depth_shape_px != _FIXED_DEPTH_SHAPE:
        raise ValueError(
            f"JAX depth ROI must retain fixed depth shape {_FIXED_DEPTH_SHAPE}"
        )


def _make_kernel(config: TerrainKernelConfig) -> Callable:
    grid_rows, grid_cols = config.grid_shape
    grid_size = grid_rows * grid_cols
    point_count = config.depth_shape_px[0] * config.depth_shape_px[1]
    resolution_m = config.grid_resolution_m
    x_min, _ = config.grid_x_range_m
    y_min, _ = config.grid_y_range_m
    min_depth_m = config.min_depth_m
    max_depth_m = config.max_depth_m
    rows, cols = np.indices(config.depth_shape_px, dtype=np.float64)
    rows_jax = jnp.asarray(rows)
    cols_jax = jnp.asarray(cols)

    def shift(array, dx: int, dy: int, fill):
        padded = jnp.pad(
            array,
            ((max(dx, 0), max(-dx, 0)), (max(dy, 0), max(-dy, 0))),
            constant_values=fill,
        )
        start_x = max(-dx, 0)
        start_y = max(-dy, 0)
        return jax.lax.dynamic_slice(
            padded,
            (start_x, start_y),
            (grid_rows, grid_cols),
        )

    def finite_differences(height, valid):
        outputs = []
        for axis in (0, 1):
            plus_height = shift(
                height, -1 if axis == 0 else 0, -1 if axis == 1 else 0, jnp.nan
            )
            minus_height = shift(
                height, 1 if axis == 0 else 0, 1 if axis == 1 else 0, jnp.nan
            )
            plus_mask = shift(
                valid, -1 if axis == 0 else 0, -1 if axis == 1 else 0, False
            )
            minus_mask = shift(
                valid, 1 if axis == 0 else 0, 1 if axis == 1 else 0, False
            )
            central = valid & plus_mask & minus_mask
            forward = valid & plus_mask & ~minus_mask
            backward = valid & minus_mask & ~plus_mask
            output = jnp.full(config.grid_shape, jnp.nan, dtype=height.dtype)
            output = jnp.where(
                central,
                (plus_height - minus_height) / (2.0 * resolution_m),
                output,
            )
            output = jnp.where(
                forward, (plus_height - height) / resolution_m, output
            )
            output = jnp.where(
                backward, (height - minus_height) / resolution_m, output
            )
            outputs.append(output)
        return outputs[0], outputs[1]

    def grouped_median(cell_ids, values, counts, fill):
        safe_values = jnp.where(cell_ids < grid_size, values, jnp.inf)
        order = jnp.lexsort((safe_values, cell_ids))
        sorted_values = safe_values[order]
        stops = jnp.cumsum(counts)
        starts = stops - counts
        safe_count = jnp.maximum(counts, 1)
        lower = jnp.clip(starts + (safe_count - 1) // 2, 0, point_count - 1)
        upper = jnp.clip(starts + safe_count // 2, 0, point_count - 1)
        medians = 0.5 * (sorted_values[lower] + sorted_values[upper])
        return jnp.where(counts > 0, medians, fill)

    def kernel(
        depth,
        point_mask,
        point_confidence,
        intrinsics,
        rotation,
        translation,
        stamp_s,
    ):
        depth_scale_m, fx, fy, cx, cy = intrinsics
        depth_m = depth.astype(jnp.float64) * depth_scale_m
        camera = jnp.stack(
            (
                (cols_jax - cx) * depth_m / fx,
                (rows_jax - cy) * depth_m / fy,
                depth_m,
            ),
            axis=-1,
        )
        points = camera @ rotation.T + translation
        finite_points = jnp.all(jnp.isfinite(points), axis=2)
        valid_depth = (
            jnp.isfinite(depth_m)
            & (depth_m >= min_depth_m)
            & (depth_m <= max_depth_m)
        )
        safe_x = jnp.where(finite_points, points[..., 0], x_min - resolution_m)
        safe_y = jnp.where(finite_points, points[..., 1], y_min - resolution_m)
        x_index = jnp.floor((safe_x - x_min) / resolution_m).astype(jnp.int32)
        y_index = jnp.floor((safe_y - y_min) / resolution_m).astype(jnp.int32)
        inside = (
            point_mask
            & valid_depth
            & finite_points
            & (x_index >= 0)
            & (x_index < grid_rows)
            & (y_index >= 0)
            & (y_index < grid_cols)
        )
        flat_inside = inside.reshape(-1)
        flat_cell_ids = (x_index * grid_cols + y_index).reshape(-1)
        cell_ids = jnp.where(flat_inside, flat_cell_ids, grid_size)
        counts = jax.ops.segment_sum(
            flat_inside.astype(jnp.int32),
            cell_ids,
            num_segments=grid_size,
        )
        flat_height = points[..., 2].reshape(-1)
        height = grouped_median(cell_ids, flat_height, counts, jnp.nan)
        safe_ids = jnp.clip(cell_ids, 0, grid_size - 1)
        deviations = jnp.abs(flat_height - height[safe_ids])
        roughness = grouped_median(cell_ids, deviations, counts, jnp.nan)
        confidence = grouped_median(
            cell_ids,
            point_confidence.reshape(-1),
            counts,
            0.0,
        )
        valid = counts > 0
        stamps = jnp.where(valid, stamp_s, jnp.nan)
        height = height.reshape(config.grid_shape)
        counts = counts.reshape(config.grid_shape)
        roughness = roughness.reshape(config.grid_shape)
        confidence = confidence.reshape(config.grid_shape)
        valid = valid.reshape(config.grid_shape)
        stamps = stamps.reshape(config.grid_shape)
        slope_x, slope_y = finite_differences(height, valid)
        return (
            points,
            height,
            counts,
            slope_x,
            slope_y,
            roughness,
            confidence,
            valid,
            stamps,
        )

    return jax.jit(kernel)


def _get_compiled_kernel(config: TerrainKernelConfig) -> Callable:
    global _compiled_config, _compiled_kernel
    _require_fixed_config(config)
    if _compiled_config is None:
        with jax.enable_x64(True):
            _compiled_kernel = _make_kernel(config)
        _compiled_config = config
    elif config != _compiled_config:
        raise ValueError(
            "JAX terrain kernel configuration changed after compilation; "
            "runtime recompilation is forbidden"
        )
    assert _compiled_kernel is not None
    return _compiled_kernel


def validate_kernel_result(
    result: TerrainKernelResult,
    config,
    *,
    expected_stamp_s: float | None = None,
) -> TerrainKernelResult:
    """Validate device output after its explicit CPU NumPy conversion."""
    config = _normalize_config(config)
    array_shapes = {
        "points_m": (*config.depth_shape_px, 3),
        "height_m": config.grid_shape,
        "observed_count": config.grid_shape,
        "slope_x": config.grid_shape,
        "slope_y": config.grid_shape,
        "roughness_m": config.grid_shape,
        "confidence": config.grid_shape,
        "valid_mask": config.grid_shape,
        "stamp_s": config.grid_shape,
    }
    for name, expected_shape in array_shapes.items():
        value = np.asarray(getattr(result, name))
        if value.shape != expected_shape:
            raise ValueError(f"{name} has invalid shape {value.shape}")
    counts = np.asarray(result.observed_count)
    valid = np.asarray(result.valid_mask)
    if not np.issubdtype(counts.dtype, np.integer):
        raise TypeError("observed_count must be an integer array")
    if valid.dtype != np.bool_:
        raise TypeError("valid_mask must be a boolean array")
    if np.any(counts < 0) or int(np.sum(counts)) > math.prod(
        config.depth_shape_px
    ):
        raise ValueError("observed_count is outside the fixed input range")
    if not np.array_equal(valid, counts > 0):
        raise ValueError("valid_mask must equal observed_count > 0")
    points = np.asarray(result.points_m)
    if not np.all(np.isfinite(points)):
        raise ValueError("points_m must be finite")
    for name in ("height_m", "roughness_m", "confidence", "stamp_s"):
        values = np.asarray(getattr(result, name))
        if not np.all(np.isfinite(values[valid])):
            raise ValueError(f"{name} must be finite in valid cells")
    height = np.asarray(result.height_m)
    roughness = np.asarray(result.roughness_m)
    confidence = np.asarray(result.confidence)
    stamps = np.asarray(result.stamp_s)
    if np.any(np.isfinite(height[~valid])) or np.any(
        np.isfinite(roughness[~valid])
    ):
        raise ValueError("invalid cells must remain NaN-masked")
    if np.any(confidence[~valid] != 0.0) or np.any(
        np.isfinite(stamps[~valid])
    ):
        raise ValueError(
            "invalid confidence/stamp cells violate the mask contract"
        )
    if np.any(roughness[valid] < 0.0) or np.any(
        (confidence[valid] < 0.0) | (confidence[valid] > 1.0)
    ):
        raise ValueError("roughness or confidence is outside its valid range")
    max_abs_height = float(np.max(np.abs(points[..., 2])))
    if np.any(np.abs(height[valid]) > max_abs_height + 1e-9):
        raise ValueError("height_m is outside the deprojected point range")
    if np.any(roughness[valid] > 2.0 * max_abs_height + 1e-9):
        raise ValueError("roughness_m is outside the deprojected point range")
    slope_limit = 2.0 * max(max_abs_height, 1.0) / config.grid_resolution_m
    for name in ("slope_x", "slope_y"):
        slope = np.asarray(getattr(result, name))
        if np.any(np.isinf(slope)) or np.any(np.isfinite(slope) & ~valid):
            raise ValueError(f"{name} violates the finite mask contract")
        finite = slope[np.isfinite(slope)]
        if np.any(np.abs(finite) > slope_limit):
            raise ValueError(f"{name} is outside the grid range")
    if expected_stamp_s is not None and not np.all(
        stamps[valid] == float(expected_stamp_s)
    ):
        raise ValueError("stamp_s does not match the input frame stamp")
    return result


def _execute(
    depth: np.ndarray,
    mask: np.ndarray,
    confidence: np.ndarray,
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
) -> TerrainKernelResult:
    compiled = _get_compiled_kernel(config)
    intrinsics = np.asarray((depth_scale_m, fx, fy, cx, cy), dtype=np.float64)
    with jax.enable_x64(True):
        device_values = compiled(
            depth,
            mask,
            confidence,
            intrinsics,
            rotation,
            translation_m,
            np.asarray(stamp_s, dtype=np.float64),
        )
        cpu_values = tuple(
            np.asarray(value) for value in jax.device_get(device_values)
        )
    result = TerrainKernelResult(*cpu_values)
    return validate_kernel_result(result, config, expected_stamp_s=stamp_s)


def build_terrain_grid_jax(
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
    config,
) -> TerrainKernelResult:
    """Run the compiled kernel and return validated host NumPy arrays."""
    config = _normalize_config(config)
    depth, mask, confidence, transform, translation = validate_kernel_inputs(
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
    return _execute(
        depth,
        mask,
        confidence,
        depth_scale_m=depth_scale_m,
        fx=fx,
        fy=fy,
        cx=cx,
        cy=cy,
        rotation=transform,
        translation_m=translation,
        stamp_s=stamp_s,
        config=config,
    )


def warmup(config) -> None:
    """Compile exactly one dummy 60x80 call before an autonomy arm cycle."""
    config = _normalize_config(config)
    _require_fixed_config(config)
    depth = np.full(config.depth_shape_px, 1000, dtype=np.uint16)
    mask = np.ones(config.depth_shape_px, dtype=bool)
    confidence = np.ones(config.depth_shape_px, dtype=np.float64)
    build_terrain_grid_jax(
        depth,
        point_mask=mask,
        point_confidence=confidence,
        depth_scale_m=0.001,
        fx=60.0,
        fy=60.0,
        cx=(config.depth_shape_px[1] - 1) / 2.0,
        cy=(config.depth_shape_px[0] - 1) / 2.0,
        rotation=np.eye(3, dtype=np.float64),
        translation_m=np.zeros(3, dtype=np.float64),
        stamp_s=0.0,
        config=config,
    )


__all__ = (
    "build_terrain_grid_jax",
    "validate_kernel_result",
    "warmup",
)
