"""Fixed-shape 2.5D elevation-grid primitives for terrain estimation."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np


@dataclass(frozen=True)
class ElevationGrid:
    height_m: np.ndarray
    observed_count: np.ndarray
    slope_x: np.ndarray
    slope_y: np.ndarray
    roughness_m: np.ndarray
    confidence: np.ndarray
    valid_mask: np.ndarray
    support_mask: np.ndarray
    lower_floor_mask: np.ndarray
    obstacle_mask: np.ndarray
    stamp_s: np.ndarray


def _cell_statistics(
    cell_ids: np.ndarray,
    heights_m: np.ndarray,
    point_confidence: np.ndarray,
    *,
    shape: tuple[int, int],
    stamp_s: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
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
        source_x_start + dx : source_x_stop + dx,
        source_y_start + dy : source_y_stop + dy,
    ] = array[source_x_start:source_x_stop, source_y_start:source_y_stop]
    return output


def _support_connectivity(
    height_m: np.ndarray,
    valid_mask: np.ndarray,
    *,
    x_centres_m: np.ndarray,
    y_centres_m: np.ndarray,
    max_step_m: float,
    seed_max_x_m: float,
    seed_half_width_m: float,
) -> np.ndarray:
    seed_region = (
        valid_mask
        & (x_centres_m[:, None] <= seed_max_x_m)
        & (np.abs(y_centres_m[None, :]) <= seed_half_width_m)
    )
    if not np.any(seed_region):
        return np.zeros(valid_mask.shape, dtype=bool)
    seed_height = float(np.nanmedian(height_m[seed_region]))
    connected = seed_region & (np.abs(height_m - seed_height) <= max_step_m)
    for _ in range(sum(valid_mask.shape)):
        previous = connected
        expanded = np.zeros(valid_mask.shape, dtype=bool)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            neighbour_connected = _shift(previous, dx, dy, False)
            neighbour_height = _shift(height_m, dx, dy, np.nan)
            expanded |= (
                neighbour_connected
                & valid_mask
                & (np.abs(height_m - neighbour_height) <= max_step_m)
            )
        connected = previous | expanded
        if np.array_equal(connected, previous):
            break
    return connected


def _finite_differences(
    height_m: np.ndarray,
    mask: np.ndarray,
    resolution_m: float,
) -> tuple[np.ndarray, np.ndarray]:
    slope_x = np.full(height_m.shape, np.nan, dtype=float)
    slope_y = np.full(height_m.shape, np.nan, dtype=float)
    for axis, output in ((0, slope_x), (1, slope_y)):
        plus_height = _shift(height_m, -1 if axis == 0 else 0, -1 if axis == 1 else 0, np.nan)
        minus_height = _shift(height_m, 1 if axis == 0 else 0, 1 if axis == 1 else 0, np.nan)
        plus_mask = _shift(mask, -1 if axis == 0 else 0, -1 if axis == 1 else 0, False)
        minus_mask = _shift(mask, 1 if axis == 0 else 0, 1 if axis == 1 else 0, False)
        central = mask & plus_mask & minus_mask
        output[central] = (plus_height[central] - minus_height[central]) / (2.0 * resolution_m)
        forward = mask & plus_mask & ~minus_mask
        output[forward] = (plus_height[forward] - height_m[forward]) / resolution_m
        backward = mask & minus_mask & ~plus_mask
        output[backward] = (height_m[backward] - minus_height[backward]) / resolution_m
    return slope_x, slope_y


def _local_obstacle_mask(
    height_m: np.ndarray,
    valid_mask: np.ndarray,
    support_mask: np.ndarray,
    obstacle_height_m: float,
) -> np.ndarray:
    """Find compact high protrusions against a local support annulus."""
    neighbours = []
    for dx in range(-5, 6):
        for dy in range(-5, 6):
            radius = max(abs(dx), abs(dy))
            if not 3 <= radius <= 5:
                continue
            shifted_height = _shift(height_m, dx, dy, np.nan)
            shifted_support = _shift(support_mask, dx, dy, False)
            neighbours.append(np.where(shifted_support, shifted_height, np.nan))
    if not neighbours:
        return np.zeros(height_m.shape, dtype=bool)
    # np.nanmedian 의 3D masked 경로는 프레임당 수십 ms 를 먹는다(실측 핫스팟).
    # NaN 을 뒤로 보내는 정렬 한 번으로 동일한 per-cell nanmedian 을 계산한다.
    stacked = np.sort(np.stack(neighbours), axis=0)
    finite_count = np.sum(np.isfinite(stacked), axis=0)
    safe_count = np.maximum(finite_count, 1)
    lower = np.take_along_axis(stacked, ((safe_count - 1) // 2)[None, ...], axis=0)[0]
    upper = np.take_along_axis(stacked, (safe_count // 2)[None, ...], axis=0)[0]
    baseline = np.where(finite_count > 0, 0.5 * (lower + upper), np.nan)
    return (
        valid_mask
        & np.isfinite(baseline)
        & (height_m > baseline + obstacle_height_m)
    )


def _local_lower_floor_mask(
    height_m: np.ndarray,
    valid_mask: np.ndarray,
    support_mask: np.ndarray,
    *,
    drop_height_m: float,
    resolution_m: float,
    reference_radius_m: float,
) -> np.ndarray:
    """Compare disconnected cells with a bounded nearest local support height."""
    reference = np.where(support_mask, height_m, np.nan)
    steps = int(math.ceil(reference_radius_m / resolution_m))
    for _ in range(steps):
        previous = reference
        candidates = (
            _shift(previous, 0, 1, np.nan),
            _shift(previous, 0, -1, np.nan),
            _shift(previous, 1, 0, np.nan),
            _shift(previous, -1, 0, np.nan),
        )
        replacement = np.full(height_m.shape, np.nan, dtype=float)
        for candidate in candidates:
            replacement = np.where(
                ~np.isfinite(replacement) & np.isfinite(candidate),
                candidate,
                replacement,
            )
        fill = ~np.isfinite(previous) & np.isfinite(replacement)
        if not np.any(fill):
            break
        reference = np.where(fill, replacement, previous)
    return (
        valid_mask
        & ~support_mask
        & np.isfinite(reference)
        & (height_m < reference - drop_height_m)
    )


def build_elevation_grid(
    points_m: np.ndarray,
    point_mask: np.ndarray,
    point_confidence: np.ndarray,
    *,
    support_point_mask: np.ndarray | None = None,
    stamp_s: float,
    shape: tuple[int, int],
    resolution_m: float,
    x_range_m: tuple[float, float],
    y_range_m: tuple[float, float],
    max_support_step_m: float,
    drop_height_m: float,
    obstacle_height_m: float,
    drop_reference_radius_m: float = 1.0,
    seed_max_x_m: float,
    seed_half_width_m: float,
) -> ElevationGrid:
    """Scatter points into fixed cells and find a locally connected support surface."""
    flat_points = np.asarray(points_m, dtype=float).reshape((-1, 3))
    flat_mask = np.asarray(point_mask, dtype=bool).reshape(-1)
    flat_support_mask = (
        flat_mask
        if support_point_mask is None
        else np.asarray(support_point_mask, dtype=bool).reshape(-1)
    )
    if flat_support_mask.shape != flat_mask.shape:
        raise ValueError("support_point_mask must match point_mask")
    flat_confidence = np.asarray(point_confidence, dtype=float).reshape(-1)
    finite_points = np.all(np.isfinite(flat_points), axis=1)
    safe_x = np.where(finite_points, flat_points[:, 0], x_range_m[0] - resolution_m)
    safe_y = np.where(finite_points, flat_points[:, 1], y_range_m[0] - resolution_m)
    x_index = np.floor((safe_x - x_range_m[0]) / resolution_m).astype(np.int64)
    y_index = np.floor((safe_y - y_range_m[0]) / resolution_m).astype(np.int64)
    inside = (
        flat_mask
        & finite_points
        & (x_index >= 0)
        & (x_index < shape[0])
        & (y_index >= 0)
        & (y_index < shape[1])
    )
    support_inside = inside & flat_support_mask
    if np.any(inside):
        cell_ids = x_index[inside] * shape[1] + y_index[inside]
        height, counts, roughness, confidence, stamps = _cell_statistics(
            cell_ids,
            flat_points[inside, 2],
            flat_confidence[inside],
            shape=shape,
            stamp_s=stamp_s,
        )
    else:
        height = np.full(shape, np.nan, dtype=float)
        counts = np.zeros(shape, dtype=np.int32)
        roughness = np.full(shape, np.nan, dtype=float)
        confidence = np.zeros(shape, dtype=float)
        stamps = np.full(shape, np.nan, dtype=float)
    if np.any(support_inside):
        support_cell_ids = x_index[support_inside] * shape[1] + y_index[support_inside]
        support_height, support_counts, _, _, _ = _cell_statistics(
            support_cell_ids,
            flat_points[support_inside, 2],
            flat_confidence[support_inside],
            shape=shape,
            stamp_s=stamp_s,
        )
    else:
        support_height = np.full(shape, np.nan, dtype=float)
        support_counts = np.zeros(shape, dtype=np.int32)
    valid = counts > 0
    support_valid = support_counts > 0
    x_centres = x_range_m[0] + (np.arange(shape[0]) + 0.5) * resolution_m
    y_centres = y_range_m[0] + (np.arange(shape[1]) + 0.5) * resolution_m
    initial_support = _support_connectivity(
        support_height,
        support_valid,
        x_centres_m=x_centres,
        y_centres_m=y_centres,
        max_step_m=max_support_step_m,
        seed_max_x_m=seed_max_x_m,
        seed_half_width_m=seed_half_width_m,
    )
    obstacle = _local_obstacle_mask(height, valid, initial_support, obstacle_height_m)
    support = _support_connectivity(
        support_height,
        support_valid & ~obstacle,
        x_centres_m=x_centres,
        y_centres_m=y_centres,
        max_step_m=max_support_step_m,
        seed_max_x_m=seed_max_x_m,
        seed_half_width_m=seed_half_width_m,
    )
    lower_floor = _local_lower_floor_mask(
        height,
        valid,
        support,
        drop_height_m=drop_height_m,
        resolution_m=resolution_m,
        reference_radius_m=drop_reference_radius_m,
    )
    slope_x, slope_y = _finite_differences(support_height, support, resolution_m)
    return ElevationGrid(
        height_m=height,
        observed_count=counts,
        slope_x=slope_x,
        slope_y=slope_y,
        roughness_m=roughness,
        confidence=confidence,
        valid_mask=valid,
        support_mask=support,
        lower_floor_mask=lower_floor,
        obstacle_mask=obstacle,
        stamp_s=stamps,
    )


def empty_grid(shape: tuple[int, int]) -> ElevationGrid:
    nan = np.full(shape, np.nan, dtype=float)
    zero_float = np.zeros(shape, dtype=float)
    zero_count = np.zeros(shape, dtype=np.int32)
    false = np.zeros(shape, dtype=bool)
    return ElevationGrid(
        height_m=nan.copy(),
        observed_count=zero_count,
        slope_x=nan.copy(),
        slope_y=nan.copy(),
        roughness_m=nan.copy(),
        confidence=zero_float,
        valid_mask=false.copy(),
        support_mask=false.copy(),
        lower_floor_mask=false.copy(),
        obstacle_mask=false.copy(),
        stamp_s=nan.copy(),
    )


def warp_and_fuse_grid(
    previous: ElevationGrid,
    current: ElevationGrid,
    *,
    dx_m: float,
    dy_m: float,
    dyaw_rad: float,
    current_stamp_s: float,
    history_horizon_s: float,
    resolution_m: float,
    x_range_m: tuple[float, float],
    y_range_m: tuple[float, float],
    max_support_step_m: float,
    drop_height_m: float,
    obstacle_height_m: float,
    drop_reference_radius_m: float = 1.0,
    seed_max_x_m: float,
    seed_half_width_m: float,
) -> tuple[ElevationGrid, int, float]:
    """Re-bin recent cells through previous-to-current SE(2), then prefer current data.

    Fractional cell residual is deliberately not interpolated.  It reduces the
    carried confidence and is returned to the caller for footprint inflation.
    """
    shape = current.height_m.shape
    valid_previous = previous.valid_mask & np.isfinite(previous.stamp_s)
    ages = current_stamp_s - previous.stamp_s
    valid_previous &= (ages >= 0.0) & (ages < history_horizon_s)
    source_x, source_y = np.nonzero(valid_previous)
    warped_height = np.full(shape, np.nan, dtype=float)
    warped_count = np.zeros(shape, dtype=np.int32)
    warped_roughness = np.full(shape, np.nan, dtype=float)
    warped_confidence = np.zeros(shape, dtype=float)
    warped_stamp = np.full(shape, np.nan, dtype=float)
    residual_grid = np.full(shape, np.inf, dtype=float)
    if source_x.size:
        x = x_range_m[0] + (source_x + 0.5) * resolution_m
        y = y_range_m[0] + (source_y + 0.5) * resolution_m
        cosine, sine = math.cos(dyaw_rad), math.sin(dyaw_rad)
        translated_x = x - dx_m
        translated_y = y - dy_m
        current_x = cosine * translated_x + sine * translated_y
        current_y = -sine * translated_x + cosine * translated_y
        destination_x = np.floor((current_x - x_range_m[0]) / resolution_m).astype(int)
        destination_y = np.floor((current_y - y_range_m[0]) / resolution_m).astype(int)
        inside = (
            (destination_x >= 0)
            & (destination_x < shape[0])
            & (destination_y >= 0)
            & (destination_y < shape[1])
        )
        sx = source_x[inside]
        sy = source_y[inside]
        dx = destination_x[inside]
        dy = destination_y[inside]
        centre_x = x_range_m[0] + (dx + 0.5) * resolution_m
        centre_y = y_range_m[0] + (dy + 0.5) * resolution_m
        residual = np.hypot(current_x[inside] - centre_x, current_y[inside] - centre_y)
        age_score = np.maximum(0.0, 1.0 - ages[sx, sy] / history_horizon_s)
        residual_score = np.maximum(
            0.25, 1.0 - 0.5 * residual / (math.sqrt(0.5) * resolution_m)
        )
        candidate_confidence = previous.confidence[sx, sy] * age_score * residual_score
        # 목적지 셀당 하나만 남긴다: 신뢰도 최대, 동률이면 잔차 최소(결정적 lexsort).
        destination = dx * shape[1] + dy
        order = np.lexsort((residual, -candidate_confidence, destination))
        keep = np.ones(order.size, dtype=bool)
        keep[1:] = destination[order][1:] != destination[order][:-1]
        chosen = order[keep]
        flat_dx, flat_dy = dx[chosen], dy[chosen]
        residual_grid[flat_dx, flat_dy] = residual[chosen]
        warped_height[flat_dx, flat_dy] = previous.height_m[sx[chosen], sy[chosen]]
        warped_count[flat_dx, flat_dy] = previous.observed_count[sx[chosen], sy[chosen]]
        warped_roughness[flat_dx, flat_dy] = previous.roughness_m[sx[chosen], sy[chosen]]
        warped_confidence[flat_dx, flat_dy] = candidate_confidence[chosen]
        warped_stamp[flat_dx, flat_dy] = previous.stamp_s[sx[chosen], sy[chosen]]

    warped_count[warped_confidence <= 0.0] = 0
    carried = (warped_count > 0) & ~current.valid_mask
    residual_max = (
        float(np.max(residual_grid[carried])) if np.any(carried) else 0.0
    )
    height = np.where(current.valid_mask, current.height_m, warped_height)
    count = np.where(current.valid_mask, current.observed_count, warped_count)
    roughness = np.where(current.valid_mask, current.roughness_m, warped_roughness)
    confidence = np.where(current.valid_mask, current.confidence, warped_confidence)
    stamps = np.where(current.valid_mask, current.stamp_s, warped_stamp)
    valid = count > 0
    x_centres = x_range_m[0] + (np.arange(shape[0]) + 0.5) * resolution_m
    y_centres = y_range_m[0] + (np.arange(shape[1]) + 0.5) * resolution_m
    initial_support = _support_connectivity(
        height,
        valid,
        x_centres_m=x_centres,
        y_centres_m=y_centres,
        max_step_m=max_support_step_m,
        seed_max_x_m=seed_max_x_m,
        seed_half_width_m=seed_half_width_m,
    )
    obstacle = _local_obstacle_mask(height, valid, initial_support, obstacle_height_m)
    support = _support_connectivity(
        height,
        valid & ~obstacle,
        x_centres_m=x_centres,
        y_centres_m=y_centres,
        max_step_m=max_support_step_m,
        seed_max_x_m=seed_max_x_m,
        seed_half_width_m=seed_half_width_m,
    )
    lower_floor = _local_lower_floor_mask(
        height,
        valid,
        support,
        drop_height_m=drop_height_m,
        resolution_m=resolution_m,
        reference_radius_m=drop_reference_radius_m,
    )
    slope_x, slope_y = _finite_differences(height, support, resolution_m)
    return (
        ElevationGrid(
            height_m=height,
            observed_count=count,
            slope_x=slope_x,
            slope_y=slope_y,
            roughness_m=roughness,
            confidence=confidence,
            valid_mask=valid,
            support_mask=support,
            lower_floor_mask=lower_floor,
            obstacle_mask=obstacle,
            stamp_s=stamps,
        ),
        int(np.count_nonzero(carried)),
        residual_max,
    )


__all__ = (
    "ElevationGrid",
    "build_elevation_grid",
    "empty_grid",
    "warp_and_fuse_grid",
)
