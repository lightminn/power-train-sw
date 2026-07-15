"""Robust, ROS-independent depth ROI quality evaluation using NumPy."""
from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Iterable

import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    """Pinhole intrinsics expressed in pixels for the supplied ROI."""

    fx: float
    fy: float
    cx: float
    cy: float

    def __post_init__(self) -> None:
        values = (self.fx, self.fy, self.cx, self.cy)
        if not all(math.isfinite(value) for value in values):
            raise ValueError("camera intrinsics must be finite")
        if self.fx <= 0.0 or self.fy <= 0.0:
            raise ValueError("camera focal lengths must be positive")


@dataclass(frozen=True)
class DepthQualityConfig:
    min_depth_m: float = 0.2
    max_depth_m: float = 6.0
    lower_percentile: float = 10.0
    upper_percentile: float = 90.0
    min_valid_ratio: float = 0.75
    max_invalid_ratio: float = 0.02
    max_zero_ratio: float = 0.02
    max_out_of_range_ratio: float = 0.02
    max_mad_m: float = 0.08
    max_percentile_span_m: float = 0.25
    connectivity_delta_m: float = 0.12
    min_connected_ratio: float = 0.80
    min_normal_consistency: float = 0.85
    max_temporal_delta_m: float = 0.25
    isolated_spike_mad_multiplier: float = 6.0
    max_isolated_spike_pixels: int = 3
    min_hole_pixels: int = 4
    disconnected_floor_separation_m: float = 0.15
    min_disconnected_floor_ratio: float = 0.05
    normal_baseline_pixels: int = 5

    def __post_init__(self) -> None:
        finite_values = (
            self.min_depth_m,
            self.max_depth_m,
            self.lower_percentile,
            self.upper_percentile,
            self.min_valid_ratio,
            self.max_invalid_ratio,
            self.max_zero_ratio,
            self.max_out_of_range_ratio,
            self.max_mad_m,
            self.max_percentile_span_m,
            self.connectivity_delta_m,
            self.min_connected_ratio,
            self.min_normal_consistency,
            self.max_temporal_delta_m,
            self.isolated_spike_mad_multiplier,
            self.disconnected_floor_separation_m,
            self.min_disconnected_floor_ratio,
        )
        if not all(math.isfinite(value) for value in finite_values):
            raise ValueError("depth quality thresholds must be finite")
        if not 0.0 < self.min_depth_m < self.max_depth_m:
            raise ValueError("depth range must be positive and ordered")
        if not 0.0 <= self.lower_percentile < self.upper_percentile <= 100.0:
            raise ValueError("depth percentiles must be ordered within 0..100")
        for value in (
            self.min_valid_ratio,
            self.max_invalid_ratio,
            self.max_zero_ratio,
            self.max_out_of_range_ratio,
            self.min_connected_ratio,
            self.min_normal_consistency,
            self.min_disconnected_floor_ratio,
        ):
            if not 0.0 <= value <= 1.0:
                raise ValueError("depth quality ratios must be within 0..1")
        if min(
            self.max_mad_m,
            self.max_percentile_span_m,
            self.connectivity_delta_m,
            self.max_temporal_delta_m,
            self.isolated_spike_mad_multiplier,
            self.disconnected_floor_separation_m,
        ) <= 0.0:
            raise ValueError("depth quality distance thresholds must be positive")
        if self.max_isolated_spike_pixels < 1 or self.min_hole_pixels < 1:
            raise ValueError("depth component sizes must be positive")
        if self.normal_baseline_pixels < 1:
            raise ValueError("normal baseline must be positive")


@dataclass(frozen=True)
class DepthQualitySnapshot:
    frame_stamp_s: float
    robust_depth_m: float


@dataclass(frozen=True)
class DepthQualityResult:
    frame_stamp_s: float
    robust_depth_m: float
    valid_ratio: float
    median_m: float
    mad_m: float
    lower_percentile_m: float
    upper_percentile_m: float
    temporal_delta_m: float | None
    connected_ratio: float
    normal_consistency: float
    confidence: float
    reject_reasons: tuple[str, ...]

    @property
    def accepted(self) -> bool:
        return not self.reject_reasons

    def snapshot(self) -> DepthQualitySnapshot:
        return DepthQualitySnapshot(
            frame_stamp_s=self.frame_stamp_s,
            robust_depth_m=self.robust_depth_m,
        )


def _components(mask: np.ndarray) -> list[list[tuple[int, int, int]]]:
    """Return four-connected components as row runs, avoiding per-pixel Python BFS."""
    runs: list[tuple[int, int, int]] = []
    parents: list[int] = []
    sizes: list[int] = []

    def find(index: int) -> int:
        root = index
        while parents[root] != root:
            root = parents[root]
        while parents[index] != index:
            parent = parents[index]
            parents[index] = root
            index = parent
        return root

    def union(left: int, right: int) -> None:
        left_root = find(left)
        right_root = find(right)
        if left_root == right_root:
            return
        if sizes[left_root] < sizes[right_root]:
            left_root, right_root = right_root, left_root
        parents[right_root] = left_root
        sizes[left_root] += sizes[right_root]

    previous: list[int] = []
    for row_index, row in enumerate(mask):
        starts = np.flatnonzero(row & np.concatenate(([True], ~row[:-1])))
        stops = np.flatnonzero(row & np.concatenate((~row[1:], [True]))) + 1
        current: list[int] = []
        for start, stop in zip(starts, stops):
            run_index = len(runs)
            start = int(start)
            stop = int(stop)
            runs.append((row_index, start, stop))
            parents.append(run_index)
            sizes.append(stop - start)
            current.append(run_index)
            for previous_index in previous:
                _, previous_start, previous_stop = runs[previous_index]
                if previous_stop <= start:
                    continue
                if stop <= previous_start:
                    break
                union(run_index, previous_index)
        previous = current

    components: dict[int, list[tuple[int, int, int]]] = {}
    for run_index, run in enumerate(runs):
        components.setdefault(find(run_index), []).append(run)
    return list(components.values())


def _largest_component_mask(mask: np.ndarray) -> tuple[np.ndarray, int]:
    components = _components(mask)
    output = np.zeros(mask.shape, dtype=bool)
    if not components:
        return output, 0
    largest = max(components, key=lambda component: sum(stop - start for _, start, stop in component))
    size = 0
    for row, start, stop in largest:
        output[row, start:stop] = True
        size += stop - start
    return output, size


def _has_enclosed_hole(mask: np.ndarray, min_pixels: int) -> bool:
    height, width = mask.shape
    for component in _components(mask):
        size = sum(stop - start for _, start, stop in component)
        if size < min_pixels:
            continue
        if not any(
            row in (0, height - 1) or start == 0 or stop == width
            for row, start, stop in component
        ):
            return True
    return False


def _has_isolated_spike(mask: np.ndarray, max_pixels: int) -> bool:
    return any(
        sum(stop - start for _, start, stop in component) <= max_pixels
        for component in _components(mask)
    )


def _normal_consistency(
    depth_m: np.ndarray,
    surface_mask: np.ndarray,
    intrinsics: CameraIntrinsics,
    baseline_pixels: int,
) -> float:
    height, width = depth_m.shape
    baseline = min(baseline_pixels, (height - 1) // 2, (width - 1) // 2)
    if baseline < 1:
        return 0.0

    rows, cols = np.indices(depth_m.shape, dtype=float)
    points = np.empty((*depth_m.shape, 3), dtype=float)
    points[..., 0] = (cols - intrinsics.cx) * depth_m / intrinsics.fx
    points[..., 1] = (rows - intrinsics.cy) * depth_m / intrinsics.fy
    points[..., 2] = depth_m

    centre = surface_mask[baseline:-baseline, baseline:-baseline]
    valid_normals = (
        centre
        & surface_mask[baseline:-baseline, 2 * baseline :]
        & surface_mask[baseline:-baseline, : -2 * baseline]
        & surface_mask[2 * baseline :, baseline:-baseline]
        & surface_mask[: -2 * baseline, baseline:-baseline]
    )
    if not np.any(valid_normals):
        return 0.0

    tangent_x = (
        points[baseline:-baseline, 2 * baseline :]
        - points[baseline:-baseline, : -2 * baseline]
    )
    tangent_y = (
        points[2 * baseline :, baseline:-baseline]
        - points[: -2 * baseline, baseline:-baseline]
    )
    normals = np.cross(tangent_x, tangent_y)[valid_normals]
    lengths = np.linalg.norm(normals, axis=1)
    normals = normals[lengths > 1e-12]
    lengths = lengths[lengths > 1e-12]
    if normals.size == 0:
        return 0.0
    normals = normals / lengths[:, None]
    normals[normals[:, 2] < 0.0] *= -1.0
    reference = np.median(normals, axis=0)
    reference_length = float(np.linalg.norm(reference))
    if reference_length <= 1e-12:
        return 0.0
    reference /= reference_length
    return float(np.clip(np.median(normals @ reference), 0.0, 1.0))


def _add_reason(reasons: list[str], reason: str, condition: bool) -> None:
    if condition and reason not in reasons:
        reasons.append(reason)


def _bounded_score(value: float, limit: float) -> float:
    return float(np.clip(1.0 - value / limit, 0.0, 1.0))


def analyze_depth_quality(
    depth_roi: np.ndarray,
    *,
    depth_scale_m: float,
    intrinsics: CameraIntrinsics,
    frame_stamp_s: float,
    previous: DepthQualitySnapshot | None = None,
    config: DepthQualityConfig | None = None,
    backend: str = "numpy",
) -> DepthQualityResult:
    """Select the qualified production backend at one explicit boundary."""
    if backend != "numpy":
        raise ValueError("numpy is the only qualified backend; JAX is not implemented")
    return analyze_depth_quality_numpy(
        depth_roi,
        depth_scale_m=depth_scale_m,
        intrinsics=intrinsics,
        frame_stamp_s=frame_stamp_s,
        previous=previous,
        config=config,
    )


def analyze_depth_quality_numpy(
    depth_roi: np.ndarray,
    *,
    depth_scale_m: float,
    intrinsics: CameraIntrinsics,
    frame_stamp_s: float,
    previous: DepthQualitySnapshot | None = None,
    config: DepthQualityConfig | None = None,
) -> DepthQualityResult:
    """Evaluate a complete depth ROI; no metric depends on a single centre pixel."""
    config = config or DepthQualityConfig()
    if not math.isfinite(depth_scale_m) or depth_scale_m <= 0.0:
        raise ValueError("depth_scale_m must be finite and positive")
    if not math.isfinite(frame_stamp_s):
        raise ValueError("frame_stamp_s must be finite")
    if previous is not None and (
        not math.isfinite(previous.frame_stamp_s)
        or not math.isfinite(previous.robust_depth_m)
    ):
        raise ValueError("previous depth snapshot must be finite")

    raw = np.asarray(depth_roi)
    if raw.ndim != 2 or raw.size == 0:
        raise ValueError("depth_roi must be a non-empty two-dimensional array")
    if not np.issubdtype(raw.dtype, np.number):
        raise TypeError("depth_roi must contain numeric values")
    raw_float = raw.astype(float, copy=False)
    depth_m = raw_float * depth_scale_m
    total = raw.size

    finite = np.isfinite(raw_float)
    invalid_mask = ~finite | (raw_float < 0.0)
    zero_mask = finite & (raw_float == 0.0)
    positive = finite & (raw_float > 0.0)
    out_of_range_mask = positive & (
        (depth_m < config.min_depth_m) | (depth_m > config.max_depth_m)
    )
    valid_mask = positive & ~out_of_range_mask
    valid_values = depth_m[valid_mask]

    invalid_ratio = float(np.count_nonzero(invalid_mask) / total)
    zero_ratio = float(np.count_nonzero(zero_mask) / total)
    out_of_range_ratio = float(np.count_nonzero(out_of_range_mask) / total)
    valid_ratio = float(valid_values.size / total)
    reasons: list[str] = []
    _add_reason(reasons, "invalid_depth", invalid_ratio > config.max_invalid_ratio)
    _add_reason(reasons, "zero_depth", zero_ratio > config.max_zero_ratio)
    _add_reason(
        reasons,
        "out_of_range_depth",
        out_of_range_ratio > config.max_out_of_range_ratio,
    )
    _add_reason(reasons, "low_valid_ratio", valid_ratio < config.min_valid_ratio)
    _add_reason(
        reasons,
        "depth_hole",
        _has_enclosed_hole(invalid_mask | zero_mask, config.min_hole_pixels),
    )

    if valid_values.size == 0:
        _add_reason(reasons, "no_valid_depth", True)
        nan = float("nan")
        return DepthQualityResult(
            frame_stamp_s=frame_stamp_s,
            robust_depth_m=nan,
            valid_ratio=0.0,
            median_m=nan,
            mad_m=nan,
            lower_percentile_m=nan,
            upper_percentile_m=nan,
            temporal_delta_m=None,
            connected_ratio=0.0,
            normal_consistency=0.0,
            confidence=0.0,
            reject_reasons=tuple(reasons),
        )

    median_m = float(np.median(valid_values))
    absolute_deviation = np.abs(valid_values - median_m)
    mad_m = float(np.median(absolute_deviation))
    lower_m, upper_m = (
        float(value)
        for value in np.percentile(
            valid_values,
            (config.lower_percentile, config.upper_percentile),
        )
    )
    percentile_span_m = upper_m - lower_m

    dominant_candidates = valid_mask & (
        np.abs(depth_m - median_m) <= config.connectivity_delta_m
    )
    dominant_mask, dominant_size = _largest_component_mask(dominant_candidates)
    connected_ratio = float(dominant_size / valid_values.size)
    robust_depth_m = (
        float(np.median(depth_m[dominant_mask])) if dominant_size else median_m
    )

    spike_threshold_m = max(
        config.connectivity_delta_m,
        config.isolated_spike_mad_multiplier * mad_m,
    )
    spike_mask = valid_mask & (np.abs(depth_m - robust_depth_m) > spike_threshold_m)
    _add_reason(
        reasons,
        "isolated_spike",
        _has_isolated_spike(spike_mask, config.max_isolated_spike_pixels),
    )

    deeper_floor_mask = valid_mask & (
        depth_m > robust_depth_m + config.disconnected_floor_separation_m
    )
    deeper_floor_ratio = float(np.count_nonzero(deeper_floor_mask) / valid_values.size)
    _add_reason(
        reasons,
        "disconnected_lower_floor",
        deeper_floor_ratio >= config.min_disconnected_floor_ratio,
    )
    _add_reason(reasons, "mad_exceeded", mad_m > config.max_mad_m)
    _add_reason(
        reasons,
        "percentile_span_exceeded",
        percentile_span_m > config.max_percentile_span_m,
    )
    _add_reason(
        reasons,
        "low_connected_ratio",
        connected_ratio < config.min_connected_ratio,
    )

    normal_consistency = _normal_consistency(
        depth_m,
        dominant_mask,
        intrinsics,
        config.normal_baseline_pixels,
    )
    _add_reason(
        reasons,
        "low_normal_consistency",
        normal_consistency < config.min_normal_consistency,
    )

    temporal_delta_m: float | None = None
    temporal_score = 1.0
    if previous is not None:
        temporal_delta_m = abs(robust_depth_m - previous.robust_depth_m)
        _add_reason(
            reasons,
            "temporal_jump",
            temporal_delta_m > config.max_temporal_delta_m,
        )
        _add_reason(
            reasons,
            "regressing_frame_stamp",
            frame_stamp_s < previous.frame_stamp_s,
        )
        temporal_score = _bounded_score(
            temporal_delta_m,
            config.max_temporal_delta_m,
        )

    confidence_terms: Iterable[float] = (
        min(1.0, valid_ratio / config.min_valid_ratio),
        min(1.0, connected_ratio / config.min_connected_ratio),
        min(1.0, normal_consistency / config.min_normal_consistency),
        _bounded_score(mad_m, config.max_mad_m),
        _bounded_score(percentile_span_m, config.max_percentile_span_m),
        temporal_score,
    )
    confidence = float(np.mean(tuple(confidence_terms)))

    return DepthQualityResult(
        frame_stamp_s=frame_stamp_s,
        robust_depth_m=robust_depth_m,
        valid_ratio=valid_ratio,
        median_m=median_m,
        mad_m=mad_m,
        lower_percentile_m=lower_m,
        upper_percentile_m=upper_m,
        temporal_delta_m=temporal_delta_m,
        connected_ratio=connected_ratio,
        normal_consistency=normal_consistency,
        confidence=confidence,
        reject_reasons=tuple(reasons),
    )
