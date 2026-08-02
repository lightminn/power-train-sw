from __future__ import annotations

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from powertrain_autonomy.terrain.depth_quality import CameraIntrinsics
from powertrain_autonomy.terrain.estimator import (
    BaseToCameraExtrinsic,
    BodyTilt,
    OdometryDelta,
    TerrainEstimate,
    TerrainEstimator,
    TerrainEstimatorConfig,
    TerrainFrame,
)
from powertrain_autonomy.terrain.grid import build_elevation_grid, empty_grid


WIDE_INTRINSICS = CameraIntrinsics(fx=57.1, fy=57.6, cx=39.5, cy=29.5)
ZERO_ODOMETRY = OdometryDelta(dx_m=0.0, dy_m=0.0, dyaw_rad=0.0)


def _camera_to_base(extrinsic: BaseToCameraExtrinsic) -> np.ndarray:
    """Independent test renderer rotation: optical x=right, y=down, z=forward."""
    pitch = extrinsic.pitch_down_rad
    optical = np.array(
        [
            [0.0, -math.sin(pitch), math.cos(pitch)],
            [-1.0, 0.0, 0.0],
            [0.0, -math.cos(pitch), -math.sin(pitch)],
        ]
    )

    def axis_rotation(axis: str, angle: float) -> np.ndarray:
        c = math.cos(angle)
        s = math.sin(angle)
        if axis == "x":
            return np.array(((1.0, 0.0, 0.0), (0.0, c, -s), (0.0, s, c)))
        if axis == "y":
            return np.array(((c, 0.0, s), (0.0, 1.0, 0.0), (-s, 0.0, c)))
        return np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))

    mount = (
        axis_rotation("z", extrinsic.yaw_rad)
        @ axis_rotation("y", extrinsic.mount_pitch_rad)
        @ axis_rotation("x", extrinsic.roll_rad)
    )
    return mount @ optical


def _body_to_gravity(tilt: BodyTilt) -> np.ndarray:
    cr, sr = math.cos(tilt.roll_rad), math.sin(tilt.roll_rad)
    cp, sp = math.cos(tilt.pitch_rad), math.sin(tilt.pitch_rad)
    rotation_x = np.array(((1.0, 0.0, 0.0), (0.0, cr, -sr), (0.0, sr, cr)))
    rotation_y = np.array(((cp, 0.0, sp), (0.0, 1.0, 0.0), (-sp, 0.0, cp)))
    return rotation_y @ rotation_x


def render_track_depth(
    *,
    stamp_s: float = 1.0,
    bank_rad: float = 0.0,
    far_bank_rad: float | None = None,
    bank_transition_x_m: float = 1.4,
    longitudinal_slope_rad: float = 0.0,
    width_m: float = 1.4,
    center_offset_m: float = 0.0,
    heading_rad: float = 0.0,
    lower_floor_z_m: float = -0.45,
    noise_std_m: float = 0.0,
    extrinsic: BaseToCameraExtrinsic | None = None,
    body_tilt: BodyTilt | None = None,
    rng_seed: int = 20260716,
) -> TerrainFrame:
    """Render optical-axis Z for an elevated local track and lower floor."""
    extrinsic = extrinsic or BaseToCameraExtrinsic()
    body_tilt = body_tilt or BodyTilt(roll_rad=0.0, pitch_rad=0.0)
    height, width = 60, 80
    rows, cols = np.indices((height, width), dtype=float)
    camera_rays = np.stack(
        (
            (cols - WIDE_INTRINSICS.cx) / WIDE_INTRINSICS.fx,
            (rows - WIDE_INTRINSICS.cy) / WIDE_INTRINSICS.fy,
            np.ones((height, width), dtype=float),
        ),
        axis=-1,
    )
    body_to_gravity = _body_to_gravity(body_tilt)
    directions = camera_rays @ (body_to_gravity @ _camera_to_base(extrinsic)).T
    origin = body_to_gravity @ np.array(
        (extrinsic.x_m, extrinsic.y_m, extrinsic.z_m), dtype=float
    )

    bank = np.full((height, width), math.tan(bank_rad), dtype=float)
    far_bank = math.tan(bank_rad if far_bank_rad is None else far_bank_rad)
    heading = math.tan(heading_rad)
    longitudinal = math.tan(longitudinal_slope_rad)
    # z = longitudinal*x + bank*(y - (offset + heading*x))
    with np.errstate(divide="ignore", invalid="ignore"):
        upper_t = -origin[2] / (directions[..., 2] - longitudinal * directions[..., 0])
        for _ in range(8):
            candidate_x = origin[0] + upper_t * directions[..., 0]
            bank = np.where(candidate_x < bank_transition_x_m, math.tan(bank_rad), far_bank)
            coefficient_x = longitudinal - bank * heading
            numerator = (
                coefficient_x * origin[0]
                + bank * origin[1]
                - bank * center_offset_m
                - origin[2]
            )
            denominator = (
                directions[..., 2]
                - coefficient_x * directions[..., 0]
                - bank * directions[..., 1]
            )
            upper_t = numerator / denominator
        upper_x = origin[0] + upper_t * directions[..., 0]
        upper_y = origin[1] + upper_t * directions[..., 1]
        centre_y = center_offset_m + heading * upper_x
        lower_t = (lower_floor_z_m - origin[2]) / directions[..., 2]

    on_track = (
        np.isfinite(upper_t)
        & (upper_t > 0.0)
        & (upper_x >= 0.0)
        & (upper_x < 8.0)
        & (np.abs(upper_y - centre_y) <= width_m / 2.0)
    )
    lower_valid = np.isfinite(lower_t) & (lower_t > 0.0)
    optical_z_m = np.where(on_track, upper_t, np.where(lower_valid, lower_t, 0.0))
    if noise_std_m:
        rng = np.random.default_rng(rng_seed)
        valid = optical_z_m > 0.0
        optical_z_m[valid] += rng.normal(0.0, noise_std_m, np.count_nonzero(valid))
    raw = np.rint(np.clip(optical_z_m / 0.001, 0.0, 65535.0)).astype(np.uint16)
    raw.setflags(write=False)
    return TerrainFrame(
        depth_roi=raw,
        depth_scale_m=0.001,
        intrinsics=WIDE_INTRINSICS,
        stamp_s=stamp_s,
    )


def make_estimator(**overrides) -> TerrainEstimator:
    values = {
        "depth_shape_px": (60, 80),
        "roi_rows": (0, 60),
        "roi_cols": (0, 80),
        "stride": 1,
        "quality_tile_shape_px": (15, 20),
    }
    values.update(overrides)
    return TerrainEstimator(TerrainEstimatorConfig(**values))


def estimate(
    estimator: TerrainEstimator,
    frame: TerrainFrame,
    *,
    tilt: BodyTilt | None = None,
    extrinsic: BaseToCameraExtrinsic | None = None,
    odometry_delta: OdometryDelta = ZERO_ODOMETRY,
    now_s: float | None = None,
) -> TerrainEstimate:
    return estimator.update(
        frame,
        tilt=tilt or BodyTilt(roll_rad=0.0, pitch_rad=0.0),
        extrinsic=extrinsic or BaseToCameraExtrinsic(),
        odometry_delta=odometry_delta,
        now_s=frame.stamp_s if now_s is None else now_s,
    )


def analyze_frame_quality(
    estimator: TerrainEstimator,
    *,
    depth_m: float,
    stamp_s: float,
):
    depth = np.full(
        estimator.config.depth_shape_px,
        round(depth_m / 0.001),
        dtype=np.uint16,
    )
    return estimator._quality_and_mask(
        depth,
        depth_scale_m=0.001,
        intrinsics=WIDE_INTRINSICS,
        stamp_s=stamp_s,
    )[0]


def shift_upper_depth_rows(
    frame: TerrainFrame,
    *,
    offset_mm: int,
    stamp_s: float,
) -> TerrainFrame:
    depth = np.array(frame.depth_roi, copy=True)
    upper_rows = np.zeros(depth.shape, dtype=bool)
    upper_rows[:37] = True
    shifted = upper_rows & (depth > 0) & (depth < 6000 - offset_mm)
    depth[shifted] += offset_mm
    return TerrainFrame(
        depth_roi=depth,
        depth_scale_m=frame.depth_scale_m,
        intrinsics=frame.intrinsics,
        stamp_s=stamp_s,
    )


def summarize_centreline_frame(
    estimator: TerrainEstimator,
    *,
    center_offset_m: float,
    stamp_s: float,
) -> TerrainEstimate:
    """Build a deterministic one-metre-wide support summary frame."""
    resolution = estimator.config.grid_resolution_m
    y_min = estimator.config.grid_y_range_m[0]
    right_index = int(round((center_offset_m - 0.70 - y_min) / resolution))
    left_stop = int(round((center_offset_m + 0.70 - y_min) / resolution))
    support = np.zeros(estimator.grid_shape, dtype=bool)
    support[2:14, right_index:left_stop] = True
    valid = support.copy()
    height = np.full(estimator.grid_shape, np.nan, dtype=float)
    height[support] = 0.0
    support_values = np.where(support, 0.0, np.nan)
    grid = dataclasses.replace(
        empty_grid(estimator.grid_shape),
        height_m=height,
        observed_count=valid.astype(np.int32),
        slope_x=support_values.copy(),
        slope_y=support_values.copy(),
        roughness_m=support_values.copy(),
        confidence=support.astype(float),
        valid_mask=valid,
        support_mask=support,
        stamp_s=np.where(valid, stamp_s, np.nan),
    )
    return summarize_grid(estimator, grid, stamp_s=stamp_s)


def support_grid(estimator: TerrainEstimator, spans) -> object:
    """Build a grid from (rows, first_col, stop_col[, height_m]) spans."""
    support = np.zeros(estimator.grid_shape, dtype=bool)
    height = np.full(estimator.grid_shape, np.nan, dtype=float)
    for span in spans:
        rows, first_col, stop_col = span[:3]
        support_height_m = float(span[3]) if len(span) == 4 else 0.0
        support[rows, first_col:stop_col] = True
        height[rows, first_col:stop_col] = support_height_m
    support_values = np.where(support, 0.0, np.nan)
    return dataclasses.replace(
        empty_grid(estimator.grid_shape),
        height_m=height,
        observed_count=support.astype(np.int32),
        slope_x=support_values.copy(),
        slope_y=support_values.copy(),
        roughness_m=support_values.copy(),
        confidence=support.astype(float),
        valid_mask=support.copy(),
        support_mask=support,
        stamp_s=np.where(support, 1.0, np.nan),
    )


def with_lower_floor_evidence(grid, lower_floor: np.ndarray) -> object:
    """Add observed lower-floor cells to a support-grid fixture."""
    height = np.array(grid.height_m, copy=True)
    height[lower_floor] = -0.25
    observed_count = np.array(grid.observed_count, copy=True)
    observed_count[lower_floor] = 1
    stamp_s = np.array(grid.stamp_s, copy=True)
    stamp_s[lower_floor] = 1.0
    return dataclasses.replace(
        grid,
        height_m=height,
        observed_count=observed_count,
        valid_mask=grid.valid_mask | lower_floor,
        lower_floor_mask=lower_floor,
        stamp_s=stamp_s,
    )


def branching_surface_grid(
    estimator: TerrainEstimator,
    *,
    track_drop_rows: slice | None,
) -> object:
    """Build a narrow track beside a wider, more-overlapping branch."""
    grid = support_grid(
        estimator,
        (
            (slice(2, 3), 20, 60, 0.0),
            (slice(3, 14), 20, 39, 0.0),
            (slice(3, 14), 40, 60, 0.20),
        ),
    )
    if track_drop_rows is None:
        return grid

    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[track_drop_rows, 19] = True
    lower_floor[track_drop_rows, 39] = True
    return with_lower_floor_evidence(grid, lower_floor)


def summarize_grid(
    estimator: TerrainEstimator,
    grid,
    *,
    stamp_s: float = 1.0,
    odometry_delta: OdometryDelta | None = None,
) -> TerrainEstimate:
    kwargs = {
        "grid": grid,
        "stamp_s": stamp_s,
        "frame_confidence": 1.0,
        "reasons": (),
    }
    if odometry_delta is not None:
        kwargs["odometry_delta"] = odometry_delta
    return estimator._summarize(**kwargs)


def seed_lateral_reference(
    estimator: TerrainEstimator,
    *,
    certified: bool,
    stamp_s: float,
):
    """Seed an offset reference with or without drop-bounded evidence."""
    grid = support_grid(
        estimator,
        ((slice(2, 14), 20, 44, 0.0),),
    )
    if certified:
        lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
        lower_floor[10, (19, 44)] = True
        grid = with_lower_floor_evidence(grid, lower_floor)

    result = summarize_grid(estimator, grid, stamp_s=stamp_s)

    assert result.path_available, result.reject_reasons
    reference = estimator._lateral_reference
    assert reference is not None
    assert reference.certified is certified
    return reference


def test_public_values_are_immutable_and_grid_shape_is_fixed():
    estimator = make_estimator()
    frame = render_track_depth()

    result = estimate(estimator, frame)

    assert estimator.grid_shape == (74, 60)
    assert isinstance(result, TerrainEstimate)
    with pytest.raises(dataclasses.FrozenInstanceError):
        result.path_available = False
    with pytest.raises(dataclasses.FrozenInstanceError):
        ZERO_ODOMETRY.dx_m = 1.0

    float_frame = TerrainFrame(
        frame.depth_roi.astype(float),
        frame.depth_scale_m,
        frame.intrinsics,
        frame.stamp_s,
    )
    with pytest.raises(TypeError, match="uint16"):
        estimate(make_estimator(), float_frame)
    for invalid_value in (True, 0.0, -0.1, math.nan, math.inf):
        with pytest.raises(
            ValueError,
            match="path_estimate_tau_s must be positive",
        ):
            TerrainEstimatorConfig(path_estimate_tau_s=invalid_value)


def test_config_rejects_nonfinite_footprint_outboard_half_width():
    with pytest.raises(ValueError, match="terrain estimator thresholds must be finite"):
        TerrainEstimatorConfig(footprint_outboard_half_width_m=math.nan)


def test_config_rejects_negative_footprint_outboard_half_width():
    with pytest.raises(
        ValueError,
        match="footprint_outboard_half_width_m must be nonnegative",
    ):
        TerrainEstimatorConfig(footprint_outboard_half_width_m=-0.001)


def test_config_rejects_outboard_half_width_narrower_than_tire_tread():
    with pytest.raises(
        ValueError,
        match=(
            "footprint_outboard_half_width_m must be >= wheel_half_width_m "
            "because the hub cannot be narrower than the tire"
        ),
    ):
        TerrainEstimatorConfig(footprint_outboard_half_width_m=0.034)


def test_estimator_routes_numpy_projection_and_scatter_through_pure_kernel(monkeypatch):
    from powertrain_autonomy.terrain import estimator as estimator_module
    from powertrain_autonomy.terrain.kernel import build_terrain_grid_numpy

    calls = []

    def recording_kernel(*args, **kwargs):
        calls.append((args, kwargs))
        return build_terrain_grid_numpy(*args, **kwargs)

    monkeypatch.setattr(estimator_module, "build_terrain_grid_numpy", recording_kernel)

    result = estimate(make_estimator(), render_track_depth(width_m=1.5))

    assert result.path_available, result.reject_reasons
    assert len(calls) == 1


def test_flat_track_produces_central_available_path_and_near_zero_bank():
    estimator = make_estimator()
    frame = render_track_depth(width_m=1.4)

    result = estimate(estimator, frame)

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.0, abs=0.08)
    assert result.heading_error_rad == pytest.approx(0.0, abs=0.04)
    assert result.bank_angle_rad == pytest.approx(0.0, abs=0.03)
    assert result.longitudinal_slope_rad == pytest.approx(0.0, abs=0.03)
    expected_clearance = 1.4 / 2.0 - (
        0.3595 + estimator.config.footprint_outboard_half_width_m
    )
    assert result.left_wheel_clearance_m == pytest.approx(expected_clearance, abs=0.08)
    assert result.right_wheel_clearance_m == pytest.approx(expected_clearance, abs=0.08)
    assert result.confidence > 0.54
    assert "local_obstacle" not in result.degradation_reasons


@pytest.mark.parametrize("bank_rad", (-0.16, 0.14))
def test_constant_bank_preserves_sign_and_magnitude_without_obstacle_false_positive(bank_rad):
    estimator = make_estimator()
    frame = render_track_depth(bank_rad=bank_rad, width_m=1.5)

    result = estimate(estimator, frame)

    assert result.path_available, result.reject_reasons
    assert result.bank_angle_rad == pytest.approx(bank_rad, abs=0.035)
    assert "local_obstacle" not in result.degradation_reasons


def test_full_roll_pitch_rotation_gravity_aligns_a_flat_surface():
    tilt = BodyTilt(roll_rad=0.13, pitch_rad=-0.09)
    estimator = make_estimator()
    frame = render_track_depth(body_tilt=tilt, width_m=1.5)

    result = estimate(estimator, frame, tilt=tilt)

    assert result.path_available, result.reject_reasons
    assert result.bank_angle_rad == pytest.approx(0.0, abs=0.025)
    assert result.longitudinal_slope_rad == pytest.approx(0.0, abs=0.025)


def test_longitudinal_slope_and_track_heading_are_local_grid_outputs():
    slope_estimator = make_estimator()
    heading_estimator = make_estimator()

    slope = estimate(
        slope_estimator,
        render_track_depth(longitudinal_slope_rad=0.10, width_m=1.3),
    )
    heading = estimate(
        heading_estimator,
        render_track_depth(
            heading_rad=0.08,
            width_m=1.0,
            lower_floor_z_m=-0.19,
        ),
    )

    assert slope.path_available, slope.reject_reasons
    assert slope.longitudinal_slope_rad == pytest.approx(0.10, abs=0.035)
    assert heading.path_available, heading.reject_reasons
    assert heading.heading_error_rad == pytest.approx(0.08, abs=0.04)


def test_offcentre_track_reports_centre_and_geometry_clearance():
    estimator = make_estimator()
    frame = render_track_depth(width_m=1.5, center_offset_m=0.12)

    result = estimate(estimator, frame)

    footprint_half = 0.3595 + estimator.config.footprint_outboard_half_width_m
    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.12, abs=0.07)
    assert result.left_wheel_clearance_m == pytest.approx(0.75 + 0.12 - footprint_half, abs=0.08)
    assert result.right_wheel_clearance_m == pytest.approx(0.75 - 0.12 - footprint_half, abs=0.08)


def test_published_path_offset_follows_step_without_jumping():
    """The publication contract applies alpha to the current centre-line fit."""
    estimator = make_estimator()
    frame_interval_s = 0.1
    initial = summarize_centreline_frame(
        estimator,
        center_offset_m=0.15,
        stamp_s=1.0,
    )
    target_offset_m = -0.15
    filtered = [
        summarize_centreline_frame(
            estimator,
            center_offset_m=target_offset_m,
            stamp_s=1.0 + frame_interval_s * index,
        ).path_offset_m
        for index in range(1, 21)
    ]

    alpha = frame_interval_s / (
        estimator.config.path_estimate_tau_s + frame_interval_s
    )
    expected_first = initial.path_offset_m + alpha * (
        target_offset_m - initial.path_offset_m
    )
    assert filtered[0] == pytest.approx(expected_first, abs=1e-9)
    assert target_offset_m < filtered[0] < initial.path_offset_m
    assert all(current < previous for previous, current in zip(filtered, filtered[1:]))
    assert filtered[-1] == pytest.approx(target_offset_m, abs=0.01)


def test_alternating_path_offset_noise_is_attenuated():
    """At τ=0.5 s and Δt=0.1 s, alternating spread falls to 1/11, or 9.1%."""
    estimator = make_estimator()
    frame_interval_s = 0.1
    noise_amplitude_m = 0.20
    summarize_centreline_frame(
        estimator,
        center_offset_m=0.0,
        stamp_s=1.0,
    )
    filtered = [
        summarize_centreline_frame(
            estimator,
            center_offset_m=noise_amplitude_m if index % 2 else -noise_amplitude_m,
            stamp_s=1.0 + frame_interval_s * index,
        ).path_offset_m
        for index in range(1, 21)
    ]

    alpha = frame_interval_s / (
        estimator.config.path_estimate_tau_s + frame_interval_s
    )
    expected_spread_ratio = alpha / (2.0 - alpha)
    input_spread_m = 2.0 * noise_amplitude_m
    settled_spread_m = abs(filtered[-1] - filtered[-2])
    assert settled_spread_m < 0.15 * input_spread_m
    assert settled_spread_m == pytest.approx(
        expected_spread_ratio * input_spread_m,
        rel=0.02,
    )


def test_rejected_frame_clears_published_path_filter():
    estimator = make_estimator()
    summarize_centreline_frame(
        estimator,
        center_offset_m=0.15,
        stamp_s=1.0,
    )
    accumulated = None
    for index in range(1, 5):
        accumulated = summarize_centreline_frame(
            estimator,
            center_offset_m=0.25,
            stamp_s=1.0 + 0.1 * index,
        )

    stale_frame = render_track_depth(stamp_s=2.0, width_m=1.5)
    rejected = estimate(estimator, stale_frame, now_s=2.251)
    after_reject = summarize_centreline_frame(
        estimator,
        center_offset_m=-0.15,
        stamp_s=2.1,
    )

    assert accumulated is not None
    assert accumulated.path_offset_m != pytest.approx(0.25, abs=1e-9)
    assert rejected.reject_reasons == ("stale_frame",)
    assert after_reject.path_offset_m == pytest.approx(-0.15, abs=1e-9)


def test_path_filter_does_not_lag_wheel_clearances():
    estimator = make_estimator()
    first = summarize_centreline_frame(
        estimator,
        center_offset_m=0.10,
        stamp_s=1.0,
    )
    second = summarize_centreline_frame(
        estimator,
        center_offset_m=-0.10,
        stamp_s=1.1,
    )

    footprint_half_m = (
        0.3595 + estimator.config.footprint_outboard_half_width_m
    )
    assert second.path_offset_m != pytest.approx(-0.10, abs=1e-9)
    assert -0.10 < second.path_offset_m < first.path_offset_m
    assert second.left_wheel_clearance_m == pytest.approx(
        0.70 - 0.10 - footprint_half_m,
        abs=1e-9,
    )
    assert second.right_wheel_clearance_m == pytest.approx(
        0.70 + 0.10 - footprint_half_m,
        abs=1e-9,
    )


def test_parallel_wider_surface_is_not_adopted_mid_frame():
    estimator = make_estimator()
    spans = []
    for step, row in enumerate(range(2, 10)):
        spans.extend(
            (
                (slice(row, row + 1), 21 - 2 * step, 39 - 2 * step, 0.0),
                (slice(row, row + 1), 40 - 2 * step, 60 - 2 * step, 0.20),
            )
        )

    result = summarize_grid(estimator, support_grid(estimator, spans))

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(-0.35, abs=1e-9)
    assert result.heading_error_rad == pytest.approx(math.atan(-2.0), abs=1e-9)


def test_wider_branch_without_drop_evidence_is_not_adopted():
    """Regression for the observed ramp departure beside the real-course track."""
    estimator = make_estimator()
    grid = branching_surface_grid(
        estimator,
        track_drop_rows=slice(3, 14),
    )

    result = summarize_grid(estimator, grid)

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(-0.025, abs=1e-9)


def test_no_drop_evidence_keeps_greatest_overlap_selection():
    estimator = make_estimator()
    grid = branching_surface_grid(
        estimator,
        track_drop_rows=None,
    )

    result = summarize_grid(estimator, grid)

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(1.0, abs=1e-9)


def test_drop_evidence_from_another_row_does_not_change_current_row_selection():
    estimator = make_estimator()
    grid = branching_surface_grid(
        estimator,
        track_drop_rows=slice(14, 15),
    )

    result = summarize_grid(estimator, grid)

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(1.0, abs=1e-9)


def test_first_row_prefers_drop_bounded_run_over_wider_nearer_centreline():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 3), 20, 32, 0.20),
            (slice(2, 3), 36, 46, 0.0),
            (slice(3, 4), 16, 28, 0.20),
            (slice(3, 4), 32, 48, 0.0),
            (slice(4, 5), 12, 24, 0.20),
            (slice(4, 5), 28, 50, 0.0),
            (slice(5, 6), 8, 20, 0.20),
            (slice(5, 6), 24, 52, 0.0),
            (slice(6, 14), 4, 16, 0.20),
            (slice(6, 14), 20, 52, 0.0),
        ),
    )
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[2, (35, 46)] = True

    result = summarize_grid(
        estimator,
        with_lower_floor_evidence(grid, lower_floor),
    )

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m > 0.25


def test_first_row_transported_reference_selects_nearer_drop_bounded_run():
    estimator = make_estimator()
    initial = summarize_grid(
        estimator,
        support_grid(estimator, ((slice(2, 14), 20, 60, 0.0),)),
        stamp_s=1.0,
    )
    grid = support_grid(
        estimator,
        (
            (slice(2, 3), 20, 30, 0.20),
            (slice(2, 3), 33, 51, 0.0),
            (slice(3, 4), 16, 26, 0.20),
            (slice(3, 4), 29, 53, 0.0),
            (slice(4, 5), 12, 22, 0.20),
            (slice(4, 5), 25, 53, 0.0),
            (slice(5, 14), 8, 18, 0.20),
            (slice(5, 14), 21, 53, 0.0),
        ),
    )
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[2, (19, 31, 51)] = True

    result = summarize_grid(
        estimator,
        with_lower_floor_evidence(grid, lower_floor),
        stamp_s=1.1,
    )

    assert initial.path_offset_m == pytest.approx(0.5, abs=1e-9)
    assert result.path_available, result.reject_reasons
    assert result.path_offset_m > 0.3


def test_first_row_without_drop_bounded_runs_preserves_nearest_run_fallback():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 14), 20, 40, 0.0),
            (slice(2, 14), 44, 60, 0.20),
        ),
    )

    result = summarize_grid(estimator, grid)

    footprint_half = 0.3595 + estimator.config.footprint_outboard_half_width_m
    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.0, abs=1e-9)
    assert result.heading_error_rad == pytest.approx(0.0, abs=1e-9)
    assert result.left_wheel_clearance_m == pytest.approx(
        0.50 - footprint_half,
        abs=1e-9,
    )
    assert result.right_wheel_clearance_m == pytest.approx(
        0.50 - footprint_half,
        abs=1e-9,
    )


def test_centreline_rows_stop_when_surface_splits_without_overlap():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 6), 20, 40, 0.0),
            (slice(6, 14), 0, 18, -0.20),
            (slice(6, 14), 40, 60, 0.20),
        ),
    )

    result = summarize_grid(estimator, grid)

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.0, abs=1e-9)
    assert result.heading_error_rad == pytest.approx(0.0, abs=1e-9)


def test_previous_centre_seed_is_transported_before_nearest_row_selection():
    estimator = make_estimator(path_x_range_m=(0.40, 0.50), min_path_rows=2)
    first_grid = support_grid(estimator, ((slice(2, 4), 21, 49, 0.0),))
    first = summarize_grid(estimator, first_grid, stamp_s=1.0)
    second_grid = support_grid(
        estimator,
        (
            (slice(2, 3), 17, 34, 0.20),
            (slice(2, 3), 36, 53, 0.0),
            (slice(3, 4), 21, 49, 0.0),
        ),
    )

    second = summarize_grid(
        estimator,
        second_grid,
        stamp_s=1.1,
        odometry_delta=OdometryDelta(dx_m=0.0, dy_m=-0.20, dyaw_rad=0.0),
    )

    assert first.path_offset_m == pytest.approx(0.25, abs=1e-9)
    assert second.path_available, second.reject_reasons
    assert second.path_offset_m == pytest.approx(0.2895833333333333, abs=1e-9)


def test_drop_bounded_contributing_rows_keep_wider_adjacent_ground_out_of_fit():
    """Regression for the real-course failure beside the open ramp surface."""
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 6), 20, 40, 0.0),
            (slice(6, 10), 20, 48, 0.0),
        ),
    )
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[2:6, 19] = True
    lower_floor[2:6, 40] = True

    result = summarize_grid(
        estimator,
        with_lower_floor_evidence(grid, lower_floor),
    )

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.0, abs=1e-9)
    assert result.heading_error_rad == pytest.approx(0.0, abs=1e-9)


def test_one_drop_bounded_row_pins_offset_with_all_row_slope():
    estimator = make_estimator()
    spans = []
    for row in range(2, 10):
        shift = row - 2
        if row == 5:
            shift += 1
        spans.append(
            (slice(row, row + 1), 20 + shift, 40 + shift, 0.0)
        )
    grid = support_grid(estimator, tuple(spans))
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[5, 23] = True
    lower_floor[5, 44] = True

    result = summarize_grid(
        estimator,
        with_lower_floor_evidence(grid, lower_floor),
    )

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(
        0.224702380952381,
        abs=1e-9,
    )
    assert result.heading_error_rad == pytest.approx(
        0.7794102110135346,
        abs=1e-9,
    )


def test_held_certified_reference_replaces_uncertified_wide_ground_fit():
    estimator = make_estimator()
    certified_grid = support_grid(
        estimator,
        ((slice(2, 10), 20, 40, 0.0),),
    )
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[2:4, 19] = True
    lower_floor[2:4, 40] = True
    first = summarize_grid(
        estimator,
        with_lower_floor_evidence(certified_grid, lower_floor),
        stamp_s=1.0,
    )
    rejected = summarize_grid(
        estimator,
        empty_grid(estimator.grid_shape),
        stamp_s=1.1,
        odometry_delta=ZERO_ODOMETRY,
    )

    result = summarize_grid(
        estimator,
        support_grid(estimator, ((slice(2, 10), 20, 60, 0.0),)),
        stamp_s=1.2,
        odometry_delta=ZERO_ODOMETRY,
    )

    assert first.path_available, first.reject_reasons
    assert rejected.reject_reasons == ("no_connected_support",)
    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.0, abs=1e-9)
    assert result.heading_error_rad == pytest.approx(0.0, abs=1e-9)


def test_no_certification_preserves_9da3afc_all_row_fit_bit_identically():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        tuple(
            (slice(row, row + 1), 18 + row, 38 + row, 0.0)
            for row in range(2, 10)
        ),
    )

    result = summarize_grid(estimator, grid)

    assert result.path_available, result.reject_reasons
    # HEAD 9da3afc produced these exact literals with no certification anywhere.
    assert result.path_offset_m == 0.17500000000000038
    assert result.heading_error_rad == 0.7853981633974477


def test_certified_reference_survives_and_transports_across_uncertified_frames():
    estimator = make_estimator()
    certified_grid = support_grid(
        estimator,
        ((slice(2, 14), 20, 40, 0.0),),
    )
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[10, (19, 40)] = True
    summarize_grid(
        estimator,
        with_lower_floor_evidence(certified_grid, lower_floor),
        stamp_s=1.0,
    )
    uncertified_grid = support_grid(
        estimator,
        ((slice(2, 14), 30, 54, 0.0),),
    )

    for stamp_s in (1.1, 1.2, 1.3):
        summarize_grid(
            estimator,
            uncertified_grid,
            stamp_s=stamp_s,
            odometry_delta=OdometryDelta(
                dx_m=0.0,
                dy_m=-0.10,
                dyaw_rad=0.0,
            ),
        )

    reference = estimator._lateral_reference
    assert reference is not None
    assert reference.certified
    assert reference.offset_m == pytest.approx(0.30, abs=1e-9)
    assert reference.offset_m != pytest.approx(0.60, abs=1e-9)
    assert reference.stamp_s == pytest.approx(1.0, abs=1e-9)
    assert reference.travelled_m == pytest.approx(0.30, abs=1e-9)


def test_certified_reference_is_only_replaced_by_later_certification():
    estimator = make_estimator()
    first_grid = support_grid(
        estimator,
        ((slice(2, 14), 20, 40, 0.0),),
    )
    first_lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    first_lower_floor[10, (19, 40)] = True
    summarize_grid(
        estimator,
        with_lower_floor_evidence(first_grid, first_lower_floor),
        stamp_s=1.0,
    )

    summarize_grid(
        estimator,
        support_grid(estimator, ((slice(2, 14), 30, 54, 0.0),)),
        stamp_s=1.1,
    )

    held_reference = estimator._lateral_reference
    assert held_reference is not None
    assert held_reference.certified
    assert held_reference.offset_m == pytest.approx(0.0, abs=1e-9)
    assert held_reference.stamp_s == pytest.approx(1.0, abs=1e-9)

    replacement_grid = support_grid(
        estimator,
        ((slice(2, 14), 9, 39, 0.0),),
    )
    replacement_lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    replacement_lower_floor[8, (8, 39)] = True
    summarize_grid(
        estimator,
        with_lower_floor_evidence(replacement_grid, replacement_lower_floor),
        stamp_s=1.2,
    )

    replacement = estimator._lateral_reference
    assert replacement is not None
    assert replacement.certified
    assert replacement.offset_m == pytest.approx(-0.30, abs=1e-9)
    assert replacement.stamp_s == pytest.approx(1.2, abs=1e-9)
    assert replacement.travelled_m == pytest.approx(0.0, abs=1e-9)


def test_certified_reference_expires_by_window_travel_not_history_time():
    estimator = make_estimator()
    certified_grid = support_grid(
        estimator,
        ((slice(2, 14), 20, 40, 0.0),),
    )
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[10, (19, 40)] = True
    summarize_grid(
        estimator,
        with_lower_floor_evidence(certified_grid, lower_floor),
        stamp_s=1.0,
    )

    summarize_grid(
        estimator,
        empty_grid(estimator.grid_shape),
        stamp_s=1.0 + estimator.config.history_horizon_s + 10.0,
        odometry_delta=ZERO_ODOMETRY,
    )
    assert estimator._lateral_reference is not None
    assert estimator._lateral_reference.certified

    window_depth_m = (
        estimator.config.path_x_range_m[1]
        - estimator.config.path_x_range_m[0]
    )
    summarize_grid(
        estimator,
        empty_grid(estimator.grid_shape),
        stamp_s=20.0,
        odometry_delta=OdometryDelta(
            dx_m=window_depth_m,
            dy_m=0.0,
            dyaw_rad=0.0,
        ),
    )
    assert estimator._lateral_reference is not None
    assert estimator._lateral_reference.travelled_m == pytest.approx(2.15, abs=1e-9)

    summarize_grid(
        estimator,
        empty_grid(estimator.grid_shape),
        stamp_s=20.1,
        odometry_delta=OdometryDelta(
            dx_m=0.001,
            dy_m=0.0,
            dyaw_rad=0.0,
        ),
    )
    assert estimator._lateral_reference is None


def test_first_row_follows_carried_certified_line_without_local_drop_evidence():
    estimator = make_estimator()
    certified_grid = support_grid(
        estimator,
        tuple(
            (
                slice(row, row + 1),
                20 + 2 * (row - 2),
                40 + 2 * (row - 2),
                0.0,
            )
            for row in range(2, 11)
        ),
    )
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[10, (35, 56)] = True
    summarize_grid(
        estimator,
        with_lower_floor_evidence(certified_grid, lower_floor),
        stamp_s=1.0,
    )
    reference = estimator._lateral_reference
    assert reference is not None
    # 이 fixture의 적합선은 y=2x-0.85이므로 인증 row x=0.825의 값은 0.80 m다.
    # HEAD 9b36a80의 전체 row 중앙값 0.40 m를 재사용하면 이 계약이 깨진다.
    assert reference.offset_m == pytest.approx(0.80, abs=1e-9)
    no_evidence_grid = support_grid(
        estimator,
        (
            (slice(2, 11), 14, 21, 0.0),
            (slice(2, 11), 37, 55, 0.20),
            (slice(11, 14), 21, 55, 0.20),
        ),
    )

    result = summarize_grid(
        estimator,
        no_evidence_grid,
        stamp_s=1.0 + estimator.config.history_horizon_s + 0.1,
    )

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m > 0.40


def test_rejected_frame_preserves_surface_seed_until_history_horizon_then_expires():
    results = []
    for age_s in (1.5, 1.500001):
        estimator = make_estimator(path_x_range_m=(0.40, 0.50), min_path_rows=2)
        first_grid = support_grid(estimator, ((slice(2, 4), 21, 49, 0.0),))
        summarize_grid(estimator, first_grid, stamp_s=1.0)
        rejected = summarize_grid(
            estimator,
            empty_grid(estimator.grid_shape),
            stamp_s=1.05,
            odometry_delta=OdometryDelta(
                dx_m=0.0,
                dy_m=-0.20,
                dyaw_rad=0.0,
            ),
        )
        nearest_grid = support_grid(
            estimator,
            (
                (slice(2, 3), 17, 34, 0.20),
                (slice(2, 3), 36, 53, 0.0),
                (slice(3, 4), 19, 39, 0.20),
            ),
        )

        result = summarize_grid(
            estimator,
            nearest_grid,
            stamp_s=1.0 + age_s,
            odometry_delta=ZERO_ODOMETRY,
        )

        assert rejected.reject_reasons == ("no_connected_support",)
        assert result.path_available, result.reject_reasons
        results.append(result)

    assert results[0].path_offset_m == pytest.approx(0.3375, abs=1e-9)
    assert results[1].path_offset_m == pytest.approx(-0.1375, abs=1e-9)


def test_support_gap_wider_than_three_cells_is_not_merged_into_reported_width():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 14), 20, 39, 0.0),
            (slice(2, 14), 43, 50, 0.0),
        ),
    )

    result = summarize_grid(estimator, grid)

    footprint_width_m = 2.0 * (
        0.3595 + estimator.config.footprint_outboard_half_width_m
    )
    reported_width_m = (
        result.left_wheel_clearance_m
        + result.right_wheel_clearance_m
        + footprint_width_m
    )
    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(-0.025, abs=1e-9)
    assert reported_width_m == pytest.approx(0.95, abs=1e-9)


def test_support_gap_of_three_cells_is_still_merged():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 14), 20, 39, 0.0),
            (slice(2, 14), 42, 50, 0.0),
        ),
    )

    result = summarize_grid(estimator, grid)

    footprint_width_m = 2.0 * (
        0.3595 + estimator.config.footprint_outboard_half_width_m
    )
    reported_width_m = (
        result.left_wheel_clearance_m
        + result.right_wheel_clearance_m
        + footprint_width_m
    )
    assert result.path_available, result.reject_reasons
    assert reported_width_m == pytest.approx(1.50, abs=1e-9)


def test_path_offset_is_fit_median_at_contributing_rows_not_intercept():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        tuple(
            (slice(row, row + 1), 20 + row - 2, 40 + row - 2)
            for row in range(2, 10)
        ),
    )

    result = summarize_grid(estimator, grid)

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.175, abs=1e-9)
    assert result.heading_error_rad == pytest.approx(math.atan(1.0), abs=1e-9)


def test_centreline_refits_after_dropping_large_residual_rows():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 8), 20, 40),
            (slice(8, 10), 28, 48),
            (slice(10, 14), 20, 40),
        ),
    )

    result = summarize_grid(estimator, grid)

    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.0, abs=1e-9)
    assert result.heading_error_rad == pytest.approx(0.0, abs=1e-9)


def test_blind_rows_narrower_than_rover_do_not_move_reported_centre():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 6), 20, 40),
            (slice(6, 14), 34, 42),
        ),
    )

    result = summarize_grid(estimator, grid)

    footprint_half = 0.3595 + estimator.config.footprint_outboard_half_width_m
    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.0, abs=1e-9)
    assert result.heading_error_rad == pytest.approx(0.0, abs=1e-9)
    assert result.left_wheel_clearance_m == pytest.approx(
        0.50 - footprint_half,
        abs=1e-9,
    )
    assert result.right_wheel_clearance_m == pytest.approx(
        0.50 - footprint_half,
        abs=1e-9,
    )


def test_disjoint_support_under_every_wheel_band_passes_footprint_gate():
    """The 93% case may leave the unloaded space between wheel tracks unseen."""
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            # The 0.20 m centre hole separates the runs while each strip
            # covers all three wheel bands on its side, including the margin.
            (slice(2, 14), 21, 28),
            (slice(2, 14), 32, 39),
        ),
    )

    result = summarize_grid(estimator, grid)

    assert "unsupported_footprint" not in result.reject_reasons


def test_total_support_wider_than_rover_still_fails_when_one_band_is_missing():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            # The combined 1.20 m support exceeds the 0.815 m rover width,
            # but the right strip stops short of the outermost wheel margin.
            (slice(2, 14), 22, 28),
            (slice(2, 14), 32, 50),
        ),
    )

    result = summarize_grid(estimator, grid)

    assert not result.path_available
    assert result.reject_reasons == ("unsupported_footprint",)


def test_bare_wheel_band_coverage_without_measurement_margin_fails():
    """The bare-band gate from 20792de was too permissive without the margin."""
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 14), 22, 27),
            (slice(2, 14), 33, 38),
        ),
    )

    result = summarize_grid(estimator, grid)

    assert not result.path_available
    assert result.reject_reasons == ("unsupported_footprint",)


def test_no_lookahead_row_covering_every_wheel_band_fails_closed():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 8), 21, 28),
            (slice(8, 14), 32, 39),
        ),
    )

    result = summarize_grid(estimator, grid)

    assert not result.path_available
    assert result.reject_reasons == ("unsupported_footprint",)


def test_single_contributing_row_cannot_resolve_centreline():
    estimator = make_estimator()
    grid = support_grid(
        estimator,
        (
            (slice(2, 3), 20, 40),
            (slice(3, 14), 27, 35),
        ),
    )

    result = summarize_grid(estimator, grid)

    assert not result.path_available
    assert result.reject_reasons == ("centreline_unresolved",)


def test_disconnected_support_island_does_not_expand_reported_clearance():
    estimator = make_estimator(
        path_x_range_m=(0.40, 0.50),
        min_path_rows=2,
    )
    grid = empty_grid(estimator.grid_shape)
    support = np.zeros(estimator.grid_shape, dtype=bool)
    support[2:4, 20:40] = True
    support[2, 49] = True
    finite_support = np.where(support, 0.0, np.nan)
    support_height = finite_support.copy()
    support_height[2, 49] = -0.25
    grid = dataclasses.replace(
        grid,
        height_m=support_height,
        slope_x=finite_support.copy(),
        slope_y=finite_support.copy(),
        roughness_m=finite_support.copy(),
        confidence=support.astype(float),
        valid_mask=support.copy(),
        support_mask=support,
    )

    result = summarize_grid(estimator, grid)

    footprint_half = 0.3595 + estimator.config.footprint_outboard_half_width_m
    # 기존 최외곽 셀 규칙이면 고립 셀 때문에 좌측 경계 0.75 m, 여유 0.3425 m가 된다.
    assert result.path_available, result.reject_reasons
    assert result.left_wheel_clearance_m == pytest.approx(
        0.50 - footprint_half,
        abs=0.005,
    )


def test_no_connected_support_still_fails_closed():
    estimator = make_estimator()
    result = summarize_grid(estimator, empty_grid(estimator.grid_shape))

    assert not result.path_available
    assert result.reject_reasons == ("no_connected_support",)


@pytest.mark.parametrize(
    ("damage", "reason"),
    [
        (lambda array: array.__setitem__((slice(42, 52), slice(35, 45)), 0), "depth_hole"),
        (lambda array: array.__setitem__((50, 40), 6000), "isolated_spike"),
    ],
)
def test_hole_and_spike_are_excluded_and_quality_reasons_are_inherited(damage, reason):
    estimator = make_estimator()
    clean = render_track_depth(width_m=1.5)
    damaged = np.array(clean.depth_roi, copy=True)
    damage(damaged)
    frame = TerrainFrame(damaged, clean.depth_scale_m, clean.intrinsics, clean.stamp_s)

    result = estimate(estimator, frame)

    assert reason in (*result.degradation_reasons, *result.reject_reasons)
    if result.path_available:
        assert abs(result.path_offset_m) < 0.12
        assert min(result.left_wheel_clearance_m, result.right_wheel_clearance_m) > 0.08


def test_depth_quality_spike_is_not_admitted_to_support_points():
    estimator = make_estimator()
    frame = render_track_depth(width_m=1.5)
    depth = np.array(frame.depth_roi, copy=True)
    depth[50, 40] = 6000

    _, support_mask, classification_mask, _, reasons = estimator._quality_and_mask(
        depth,
        depth_scale_m=frame.depth_scale_m,
        intrinsics=frame.intrinsics,
        stamp_s=frame.stamp_s,
    )

    assert "isolated_spike" in reasons
    assert not classification_mask[50, 40]
    assert not support_mask[50, 40]


def test_temporal_jump_rejects_one_frame_then_accepts_the_steady_scene():
    """The measured real-course 88--98% rejection defect must not latch."""
    estimator = make_estimator()
    first_frame = render_track_depth(stamp_s=1.0, width_m=1.5)
    jumped_frame = shift_upper_depth_rows(
        first_frame,
        offset_mm=600,
        stamp_s=1.1,
    )

    first = estimate(estimator, first_frame)
    jumped = estimate(estimator, jumped_frame)
    steady = estimate(
        estimator,
        dataclasses.replace(jumped_frame, stamp_s=1.2),
    )

    assert first.path_available, first.reject_reasons
    assert jumped.reject_reasons == ("temporal_jump",)
    assert steady.path_available, steady.reject_reasons
    assert steady.reject_reasons == ()


def test_temporal_jump_keeps_certified_reference_for_the_next_normal_frame():
    """At t=7.40 one bad frame killed the measured run's remaining 53 seconds."""
    estimator = make_estimator()
    first_frame = render_track_depth(stamp_s=1.0, width_m=1.5)
    first = estimate(estimator, first_frame)
    certified_reference = seed_lateral_reference(
        estimator,
        certified=True,
        stamp_s=1.0,
    )
    jumped_frame = shift_upper_depth_rows(
        first_frame,
        offset_mm=600,
        stamp_s=1.1,
    )

    jumped = estimate(estimator, jumped_frame)
    reference_after_jump = estimator._lateral_reference
    grid_cleared_after_jump = not np.any(estimator._grid.valid_mask)
    normal = estimate(
        estimator,
        dataclasses.replace(jumped_frame, stamp_s=1.2),
    )

    assert first.path_available, first.reject_reasons
    assert jumped.reject_reasons == ("temporal_jump",)
    assert normal.path_available, normal.reject_reasons
    assert normal.path_offset_m == pytest.approx(
        certified_reference.offset_m,
        abs=1e-9,
    )
    assert normal.path_offset_m != pytest.approx(0.0, abs=1e-9)
    assert reference_after_jump == certified_reference
    assert grid_cleared_after_jump
    assert estimator._lateral_reference is not None
    assert estimator._lateral_reference.certified
    assert estimator._lateral_reference.stamp_s == pytest.approx(1.0, abs=1e-9)


def test_temporal_jump_still_clears_an_uncertified_reference():
    estimator = make_estimator()
    first_frame = render_track_depth(stamp_s=1.0, width_m=1.5)
    first = estimate(estimator, first_frame)
    reference = seed_lateral_reference(
        estimator,
        certified=False,
        stamp_s=1.0,
    )
    jumped_frame = shift_upper_depth_rows(
        first_frame,
        offset_mm=600,
        stamp_s=1.1,
    )

    jumped = estimate(estimator, jumped_frame)

    assert first.path_available, first.reject_reasons
    assert not reference.certified
    assert jumped.reject_reasons == ("temporal_jump",)
    assert estimator._lateral_reference is None


def test_clock_faults_clear_certified_and_uncertified_references():
    cases = (
        (1.999, "future_frame"),
        (2.251, "stale_frame"),
    )
    for certified in (False, True):
        for now_s, reason in cases:
            estimator = make_estimator()
            first_frame = render_track_depth(stamp_s=1.0, width_m=1.5)
            first = estimate(estimator, first_frame)
            reference = seed_lateral_reference(
                estimator,
                certified=certified,
                stamp_s=1.0,
            )
            clock_fault_frame = dataclasses.replace(first_frame, stamp_s=2.0)

            rejected = estimate(
                estimator,
                clock_fault_frame,
                now_s=now_s,
            )

            assert first.path_available, first.reject_reasons
            assert reference.certified is certified
            assert rejected.reject_reasons == (reason,), (certified, reason)
            assert estimator._lateral_reference is None, (certified, reason)
            assert not np.any(estimator._grid.valid_mask), (certified, reason)
            assert estimator._frame_quality is None, (certified, reason)
            assert estimator._tile_quality == {}, (certified, reason)


def test_certified_reference_kept_across_jump_still_expires_by_travel():
    estimator = make_estimator()
    first_frame = render_track_depth(stamp_s=1.0, width_m=1.5)
    first = estimate(estimator, first_frame)
    seed_lateral_reference(
        estimator,
        certified=True,
        stamp_s=1.0,
    )
    jumped_frame = shift_upper_depth_rows(
        first_frame,
        offset_mm=600,
        stamp_s=1.1,
    )
    jumped = estimate(estimator, jumped_frame)
    window_depth_m = (
        estimator.config.path_x_range_m[1]
        - estimator.config.path_x_range_m[0]
    )

    at_limit = estimate(
        estimator,
        dataclasses.replace(jumped_frame, stamp_s=1.2),
        odometry_delta=OdometryDelta(
            dx_m=window_depth_m,
            dy_m=0.0,
            dyaw_rad=0.0,
        ),
    )
    reference_at_limit = estimator._lateral_reference
    beyond_limit = estimate(
        estimator,
        dataclasses.replace(jumped_frame, stamp_s=1.3),
        odometry_delta=OdometryDelta(
            dx_m=0.001,
            dy_m=0.0,
            dyaw_rad=0.0,
        ),
    )

    assert first.path_available, first.reject_reasons
    assert jumped.reject_reasons == ("temporal_jump",)
    assert at_limit.path_available, at_limit.reject_reasons
    assert reference_at_limit is not None
    assert reference_at_limit.certified
    assert reference_at_limit.travelled_m == pytest.approx(
        window_depth_m,
        abs=1e-9,
    )
    assert beyond_limit.path_available, beyond_limit.reject_reasons
    assert estimator._lateral_reference is not None
    assert not estimator._lateral_reference.certified
    assert estimator._lateral_reference.stamp_s == pytest.approx(1.3, abs=1e-9)


def test_steadily_receding_scene_recovers_when_per_frame_step_drops_below_limit():
    estimator = make_estimator()
    first_frame = render_track_depth(stamp_s=1.0, width_m=1.5)
    frames = (
        first_frame,
        *(
            shift_upper_depth_rows(
                first_frame,
                offset_mm=offset_mm,
                stamp_s=stamp_s,
            )
            for offset_mm, stamp_s in (
                (260, 1.1),
                (520, 1.2),
                (780, 1.3),
                (900, 1.4),
            )
        ),
    )
    results = tuple(estimate(estimator, frame) for frame in frames)

    assert results[0].path_available, results[0].reject_reasons
    assert all(
        result.reject_reasons == ("temporal_jump",)
        for result in results[1:4]
    )
    assert results[4].path_available, results[4].reject_reasons
    assert results[4].reject_reasons == ()


def test_unusable_depth_and_regressing_stamp_do_not_advance_frame_reference():
    missing_estimator = make_estimator()
    first = analyze_frame_quality(missing_estimator, depth_m=1.0, stamp_s=1.0)
    missing_depth = np.zeros(
        missing_estimator.config.depth_shape_px,
        dtype=np.uint16,
    )
    missing = missing_estimator._quality_and_mask(
        missing_depth,
        depth_scale_m=0.001,
        intrinsics=WIDE_INTRINSICS,
        stamp_s=1.1,
    )[0]

    assert "no_valid_depth" in missing.reject_reasons
    assert missing_estimator._frame_quality == first.snapshot()

    regressing_estimator = make_estimator()
    first = analyze_frame_quality(regressing_estimator, depth_m=1.0, stamp_s=2.0)
    regressing = analyze_frame_quality(
        regressing_estimator,
        depth_m=1.20,
        stamp_s=1.9,
    )

    assert regressing.reject_reasons == ("regressing_frame_stamp",)
    assert regressing_estimator._frame_quality == first.snapshot()


def test_partial_occlusion_and_noise_reduce_confidence_in_expected_direction():
    clean_estimator = make_estimator()
    occluded_estimator = make_estimator()
    noisy_estimator = make_estimator()
    clean = estimate(clean_estimator, render_track_depth(width_m=1.5))

    frame = render_track_depth(width_m=1.5)
    occluded_depth = np.array(frame.depth_roi, copy=True)
    occluded_depth[:, :20] = 0
    occluded = estimate(
        occluded_estimator,
        TerrainFrame(occluded_depth, 0.001, WIDE_INTRINSICS, frame.stamp_s),
    )
    noisy = estimate(
        noisy_estimator,
        render_track_depth(width_m=1.5, noise_std_m=0.035),
    )

    assert occluded.confidence < clean.confidence
    assert noisy.confidence < clean.confidence
    assert noisy.roughness_m > clean.roughness_m


def test_stale_input_fails_closed():
    stale_frame = render_track_depth(stamp_s=2.0, width_m=1.5)
    stale = estimate(make_estimator(), stale_frame, now_s=2.251)

    assert not stale.path_available
    assert stale.reject_reasons == ("stale_frame",)


def test_as_built_v2_0_90_m_track_is_traversable_with_42_5_mm_clearance():
    """The 0.90 m course clears each outboard hub edge by 42.5 mm.

    The as-built v2 footprint half-width is 0.3595 + 0.048 = 0.4075 m, so
    the physical outboard clearance is 0.4500 - 0.4075 = 0.0425 m.
    The 5 mm assertion tolerance is one tenth of the estimator's 50 mm grid
    cell: tight enough to catch a one-cell boundary regression.
    """
    result = estimate(make_estimator(), render_track_depth(width_m=0.90))

    assert result.path_available, result.reject_reasons
    assert result.reject_reasons == ()
    assert result.left_wheel_clearance_m == pytest.approx(0.0425, abs=0.005)
    assert result.right_wheel_clearance_m == pytest.approx(0.0425, abs=0.005)


def test_same_input_sequence_produces_identical_outputs():
    frames = tuple(
        render_track_depth(stamp_s=1.0 + 0.1 * index, bank_rad=0.04 * index, width_m=1.5)
        for index in range(4)
    )

    def run_sequence():
        estimator = make_estimator()
        return tuple(estimate(estimator, frame) for frame in frames)

    assert run_sequence() == run_sequence()


def test_recent_grid_is_carried_into_blind_zone_with_odometry_delta():
    estimator = make_estimator(path_x_range_m=(0.70, 0.90), min_path_rows=2)
    first = render_track_depth(stamp_s=1.0, width_m=1.5)
    assert estimate(estimator, first).path_available
    partial = np.array(first.depth_roi, copy=True)
    rows, cols = np.indices(partial.shape, dtype=float)
    optical_z = partial.astype(float) * 0.001
    camera = np.stack(
        (
            (cols - WIDE_INTRINSICS.cx) * optical_z / WIDE_INTRINSICS.fx,
            (rows - WIDE_INTRINSICS.cy) * optical_z / WIDE_INTRINSICS.fy,
            optical_z,
        ),
        axis=-1,
    )
    base = camera @ _camera_to_base(BaseToCameraExtrinsic()).T
    base += np.array((0.0, 0.0, 0.60))
    blind = (base[..., 0] >= 0.70) & (base[..., 0] <= 0.90) & (np.abs(base[..., 1]) < 0.9)
    partial[blind] = 0
    second = TerrainFrame(partial, 0.001, WIDE_INTRINSICS, 1.1)

    current_only = estimate(
        make_estimator(path_x_range_m=(0.70, 0.90), min_path_rows=2), second
    )
    assert not current_only.path_available

    result = estimate(
        estimator,
        second,
        odometry_delta=OdometryDelta(dx_m=0.10, dy_m=0.0, dyaw_rad=0.0),
    )

    assert result.path_available, result.reject_reasons


def test_grid_history_expires_after_bounded_horizon():
    estimator = make_estimator(path_x_range_m=(0.70, 0.90), min_path_rows=2)
    first = render_track_depth(stamp_s=1.0, width_m=1.5)
    assert estimate(estimator, first).path_available
    depth = np.array(first.depth_roi, copy=True)
    rows, cols = np.indices(depth.shape, dtype=float)
    optical_z = depth.astype(float) * 0.001
    camera = np.stack(
        (
            (cols - WIDE_INTRINSICS.cx) * optical_z / WIDE_INTRINSICS.fx,
            (rows - WIDE_INTRINSICS.cy) * optical_z / WIDE_INTRINSICS.fy,
            optical_z,
        ),
        axis=-1,
    )
    base = camera @ _camera_to_base(BaseToCameraExtrinsic()).T
    base += np.array((0.0, 0.0, 0.60))
    blind = (base[..., 0] >= 0.70) & (base[..., 0] <= 0.90) & (np.abs(base[..., 1]) < 0.9)
    depth[blind] = 0
    expired_frame = TerrainFrame(depth, 0.001, WIDE_INTRINSICS, 2.500)

    result = estimate(
        estimator,
        expired_frame,
        odometry_delta=OdometryDelta(dx_m=0.10, dy_m=0.0, dyaw_rad=0.0),
    )

    assert not result.path_available


def test_local_drop_reference_follows_longitudinal_slope():
    resolution = 0.05
    support_points = []
    floor_points = []
    for x_m in np.arange(0.325, 3.826, resolution):
        local_height = 0.20 * x_m
        for y_m in np.arange(-0.375, 0.376, resolution):
            support_points.append((x_m, y_m, local_height))
        floor_points.extend(
            ((x_m, -0.675, local_height - 0.25), (x_m, 0.675, local_height - 0.25))
        )
    points = np.asarray((*support_points, *floor_points), dtype=float)
    grid = build_elevation_grid(
        points,
        np.ones(points.shape[0], dtype=bool),
        np.ones(points.shape[0], dtype=float),
        stamp_s=1.0,
        shape=(74, 40),
        resolution_m=resolution,
        x_range_m=(0.3, 4.0),
        y_range_m=(-1.0, 1.0),
        max_support_step_m=0.12,
        drop_height_m=0.18,
        obstacle_height_m=0.15,
        seed_max_x_m=1.2,
        seed_half_width_m=0.3,
    )

    far_x_index = int((3.525 - 0.3) / resolution)
    right_floor_index = int((-0.675 - -1.0) / resolution)
    left_floor_index = int((0.675 - -1.0) / resolution)
    assert grid.lower_floor_mask[far_x_index, right_floor_index]
    assert grid.lower_floor_mask[far_x_index, left_floor_index]


def test_local_bank_transition_is_monotonic_despite_opposite_far_surface():
    estimator = make_estimator(path_x_range_m=(0.40, 1.25))
    requested = (0.0, 0.04, 0.08, 0.12)
    outputs = []
    for index, bank in enumerate(requested):
        frame = render_track_depth(
            stamp_s=1.0 + 0.1 * index,
            bank_rad=bank,
            far_bank_rad=-0.12,
            bank_transition_x_m=1.4,
            width_m=1.5,
        )
        result = estimate(estimator, frame)
        assert result.path_available, result.reject_reasons
        outputs.append(result.bank_angle_rad)

    assert all(right > left for left, right in zip(outputs, outputs[1:]))
    assert outputs[-1] == pytest.approx(requested[-1], abs=0.04)


def test_local_high_protrusion_is_an_obstacle_candidate_not_support():
    frame = render_track_depth(width_m=1.5)
    depth = np.array(frame.depth_roi, copy=True)
    patch = depth[35:50, 30:50].astype(np.int32) - 250
    depth[35:50, 30:50] = np.clip(patch, 1, 65535).astype(np.uint16)

    estimator = make_estimator()
    estimate(
        estimator,
        TerrainFrame(depth, 0.001, WIDE_INTRINSICS, frame.stamp_s),
    )

    assert np.any(estimator._grid.obstacle_mask)
    assert not np.any(estimator._grid.obstacle_mask & estimator._grid.support_mask)


def test_mujoco_wide_fov_recording_replay_matches_drop_clearance_and_offset(tmp_path):
    # Optional MuJoCo stays isolated to this integration test; production code is
    # simulator-free.  The Jetson autonomy image ships without powertrain_sim and
    # mujoco, so skip (not error) there — the pure-core tests above still run.
    pytest.importorskip("mujoco")
    pytest.importorskip("powertrain_sim")
    from powertrain_sim.mujoco_fast.model_builder import WHEEL_HALF_WIDTH_M
    from powertrain_sim.mujoco_fast.runner import _TrackProjector, run_scenario
    from powertrain_sim.recording import RecordedRun
    from powertrain_sim.scenario import load_scenario

    scenario_path = (
        Path(__file__).resolve().parents[2]
        / "powertrain_sim/scenarios/wide_fov_drop_track.yaml"
    )
    scenario = load_scenario(scenario_path)
    run_directory = tmp_path / "wide-fov"

    def drifting_command(_elapsed_s, _snapshot):
        return 0.35, 0.015

    report = run_scenario(scenario, run_directory, command_source=drifting_command)
    recorded = RecordedRun(run_directory)
    truths = {round(frame.stamp_s, 9): frame for frame in recorded.iter_ground_truth()}
    depth_frames = [
        record.value for record in recorded.iter_records() if record.stream == "depth"
    ]
    estimator = make_estimator()
    extrinsic = BaseToCameraExtrinsic(x_m=0.30, y_m=0.0, z_m=0.18)
    projector = _TrackProjector(scenario)
    previous_truth = None
    comparisons = []

    for frame in depth_frames:
        truth = truths[round(frame.stamp_s, 9)]
        if previous_truth is None:
            delta = ZERO_ODOMETRY
        else:
            world_dx = truth.x_m - previous_truth.x_m
            world_dy = truth.y_m - previous_truth.y_m
            cosine = math.cos(previous_truth.yaw_rad)
            sine = math.sin(previous_truth.yaw_rad)
            delta = OdometryDelta(
                dx_m=cosine * world_dx + sine * world_dy,
                dy_m=-sine * world_dx + cosine * world_dy,
                dyaw_rad=math.atan2(
                    math.sin(truth.yaw_rad - previous_truth.yaw_rad),
                    math.cos(truth.yaw_rad - previous_truth.yaw_rad),
                ),
            )
        result = estimate(
            estimator,
            TerrainFrame(
                frame.depth_roi,
                frame.depth_scale_m,
                frame.intrinsics,
                frame.stamp_s,
            ),
            tilt=BodyTilt(roll_rad=truth.bank_rad, pitch_rad=0.0),
            extrinsic=extrinsic,
            odometry_delta=delta,
        )
        if result.path_available:
            lateral_truth = projector.project(
                (truth.x_m, truth.y_m, truth.z_m)
            ).lateral_m
            comparisons.append((result, lateral_truth))
        previous_truth = truth

    assert comparisons
    offset_result, lateral_truth = max(comparisons, key=lambda item: abs(item[1]))
    assert abs(lateral_truth) > 0.03
    assert offset_result.path_offset_m == pytest.approx(-lateral_truth, abs=0.12)
    estimated_min_clearance = min(
        min(result.left_wheel_clearance_m, result.right_wheel_clearance_m)
        for result, _ in comparisons
    )
    actual_min_clearance = report.min_wheel_clearance_m - WHEEL_HALF_WIDTH_M
    assert estimated_min_clearance == pytest.approx(actual_min_clearance, abs=0.12)


def test_terrain_package_exports_public_estimator_contract():
    import powertrain_autonomy.terrain as terrain

    for name in (
        "BaseToCameraExtrinsic",
        "BodyTilt",
        "OdometryDelta",
        "TerrainEstimate",
        "TerrainEstimator",
        "TerrainEstimatorConfig",
        "TerrainFrame",
    ):
        assert getattr(terrain, name) is globals()[name]


def test_autonomy_readme_records_dependency_shape_and_deferred_scope_contracts():
    readme = Path(__file__).resolve().parents[1] / "README.md"
    text = readme.read_text(encoding="utf-8")

    for phrase in (
        "powertrain_autonomy does not import powertrain_ros",
        "fixed shape",
        "invalid cells remain a mask",
        "fractional residual",
        "provisional",
        "production completion",
        "RGB",
        "JAX",
        "WP6-C",
        "The controller core lives here",
        "autonomy_controller_node",
        "fixed-shape JAX kernels and NumPy/JAX grid equivalence are implemented",
        "NumPy is the only production authority",
        "Jetson qualification and backend selection remain deferred",
    ):
        assert phrase in text
    assert "JAX kernels, NumPy/JAX equivalence" not in text
    assert (
        "ROS subscriptions, controller policy, and `/autonomy/cmd_vel` "
        "publication belong to WP6-C and are not part of this package."
        not in text
    )


def test_estimator_core_has_no_ros_or_simulator_dependency():
    terrain_dir = Path(__file__).resolve().parents[1] / "terrain"
    source = "\n".join(
        (terrain_dir / name).read_text(encoding="utf-8")
        for name in ("estimator.py", "grid.py")
    )

    for forbidden in ("powertrain_ros", "rclpy", "powertrain_sim", "mujoco"):
        assert forbidden not in source
