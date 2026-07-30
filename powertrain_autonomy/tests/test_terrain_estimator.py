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
    choke_width_m: float | None = None,
    choke_x_range_m: tuple[float, float] = (0.9, 1.3),
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

    local_width = np.full(upper_x.shape, width_m, dtype=float)
    if choke_width_m is not None:
        local_width = np.where(
            (upper_x >= choke_x_range_m[0]) & (upper_x <= choke_x_range_m[1]),
            choke_width_m,
            local_width,
        )
    on_track = (
        np.isfinite(upper_t)
        & (upper_t > 0.0)
        & (upper_x >= 0.0)
        & (upper_x < 8.0)
        & (np.abs(upper_y - centre_y) <= local_width / 2.0)
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


def summarize_lateral_reference_frame(
    estimator: TerrainEstimator,
    *,
    center_offset_m: float,
    observed_edges: bool,
    stamp_s: float,
    odometry_delta: OdometryDelta = ZERO_ODOMETRY,
) -> TerrainEstimate:
    """Build a deterministic summary frame with or without strict edge evidence."""
    resolution = estimator.config.grid_resolution_m
    y_min = estimator.config.grid_y_range_m[0]
    right_index = int(round((center_offset_m - 0.50 - y_min) / resolution))
    left_stop = int(round((center_offset_m + 0.50 - y_min) / resolution))
    support = np.zeros(estimator.grid_shape, dtype=bool)
    support[2:14, right_index:left_stop] = True
    valid = support.copy()
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    if observed_edges:
        lower_floor[2:14, right_index - 1] = True
        lower_floor[2:14, left_stop] = True
        valid |= lower_floor
    height = np.full(estimator.grid_shape, np.nan, dtype=float)
    height[support] = 0.0
    height[lower_floor] = -0.25
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
        lower_floor_mask=lower_floor,
        stamp_s=np.where(valid, stamp_s, np.nan),
    )
    return estimator._summarize(
        grid,
        stamp_s=stamp_s,
        frame_confidence=1.0,
        reasons=(),
        carried_count=0,
        odometry_residual_m=0.0,
        left_limit_y=np.full(estimator.grid_shape[0], 1.5),
        right_limit_y=np.full(estimator.grid_shape[0], -1.5),
        left_floor_seen=True,
        right_floor_seen=True,
        odometry_delta=odometry_delta,
    )


def remove_lower_floor_side(frame: TerrainFrame, *, left: bool) -> TerrainFrame:
    """Remove only one side of the independently rendered lower floor."""
    rows, cols = np.indices(frame.depth_roi.shape, dtype=float)
    optical_z = frame.depth_roi.astype(float) * frame.depth_scale_m
    camera = np.stack(
        (
            (cols - frame.intrinsics.cx) * optical_z / frame.intrinsics.fx,
            (rows - frame.intrinsics.cy) * optical_z / frame.intrinsics.fy,
            optical_z,
        ),
        axis=-1,
    )
    points = camera @ _camera_to_base(BaseToCameraExtrinsic()).T
    points += np.array((0.0, 0.0, 0.60))
    lower_floor = points[..., 2] < -0.20
    selected_side = points[..., 1] > 0.0 if left else points[..., 1] < 0.0
    depth = np.array(frame.depth_roi, copy=True)
    depth[lower_floor & selected_side] = 0
    return TerrainFrame(depth, frame.depth_scale_m, frame.intrinsics, frame.stamp_s)


def remove_planning_window_lower_floor(frame: TerrainFrame) -> TerrainFrame:
    """Leave remote side evidence, but remove lower floor from candidate edge rows."""
    rows, cols = np.indices(frame.depth_roi.shape, dtype=float)
    optical_z = frame.depth_roi.astype(float) * frame.depth_scale_m
    camera = np.stack(
        (
            (cols - frame.intrinsics.cx) * optical_z / frame.intrinsics.fx,
            (rows - frame.intrinsics.cy) * optical_z / frame.intrinsics.fy,
            optical_z,
        ),
        axis=-1,
    )
    points = camera @ _camera_to_base(BaseToCameraExtrinsic()).T
    points += np.array((0.0, 0.0, 0.60))
    lower_floor = points[..., 2] < -0.20
    planning_window = (points[..., 0] >= 0.30) & (points[..., 0] <= 2.50)
    depth = np.array(frame.depth_roi, copy=True)
    depth[lower_floor & planning_window] = 0
    return TerrainFrame(depth, frame.depth_scale_m, frame.intrinsics, frame.stamp_s)


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
    for field, invalid_values in (
        ("lateral_reference_carry_m", (True, 0.0, -0.1, math.nan, math.inf)),
        ("lateral_reference_carry_drift", (True, 0.0, -0.1, math.nan, math.inf)),
    ):
        for invalid_value in invalid_values:
            with pytest.raises(
                ValueError,
                match="lateral reference carry thresholds must be positive",
            ):
                TerrainEstimatorConfig(**{field: invalid_value})


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
    expected_clearance = 1.4 / 2.0 - (0.3595 + 0.035)
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


def test_both_drop_boundaries_report_offset_and_geometry_clearance():
    estimator = make_estimator()
    frame = render_track_depth(width_m=1.5, center_offset_m=0.12)

    result = estimate(estimator, frame)

    footprint_half = 0.3595 + 0.035
    assert result.path_available, result.reject_reasons
    assert result.path_offset_m == pytest.approx(0.12, abs=0.07)
    assert result.left_wheel_clearance_m == pytest.approx(0.75 + 0.12 - footprint_half, abs=0.08)
    assert result.right_wheel_clearance_m == pytest.approx(0.75 - 0.12 - footprint_half, abs=0.08)
    assert "drop_boundary" in result.degradation_reasons


def test_shadowed_row_local_drop_evidence_certifies_lateral_reference():
    """The old 3-cell window rejected this fixture's deeply shadowed drop floor."""
    estimator = make_estimator()
    frame = render_track_depth(
        width_m=0.9,
        center_offset_m=0.12,
        lower_floor_z_m=-0.95,
    )

    result = estimate(estimator, frame)

    assert result.path_available, result.reject_reasons
    assert "lateral_reference_unobserved" not in result.degradation_reasons
    assert result.path_offset_m == pytest.approx(0.12, abs=0.07)


def test_cross_row_drop_evidence_does_not_certify_lateral_reference():
    """Only row 4 has drop cells; the tested edge rows 2-3 have none."""
    estimator = make_estimator(
        path_x_range_m=(0.40, 0.50),
        min_path_rows=2,
    )
    support = np.zeros(estimator.grid_shape, dtype=bool)
    support[2:4, 20:40] = True
    lower_floor = np.zeros(estimator.grid_shape, dtype=bool)
    lower_floor[4, 18] = True
    lower_floor[4, 41] = True
    valid = support | lower_floor
    height = np.full(estimator.grid_shape, np.nan, dtype=float)
    height[support] = 0.0
    height[lower_floor] = -0.25
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
        lower_floor_mask=lower_floor,
        stamp_s=np.where(valid, 1.0, np.nan),
    )

    result = estimator._summarize(
        grid,
        stamp_s=1.0,
        frame_confidence=1.0,
        reasons=(),
        carried_count=0,
        odometry_residual_m=0.0,
        left_limit_y=np.full(estimator.grid_shape[0], 1.5),
        right_limit_y=np.full(estimator.grid_shape[0], -1.5),
        left_floor_seen=True,
        right_floor_seen=True,
    )

    assert result.path_available, result.reject_reasons
    assert "lateral_reference_unobserved" in result.degradation_reasons


def test_observation_limit_edges_report_unobserved_lateral_reference():
    frame = remove_planning_window_lower_floor(
        render_track_depth(width_m=1.5, center_offset_m=0.12)
    )

    result = estimate(make_estimator(), frame)

    assert result.path_available, result.reject_reasons
    assert "lateral_reference_unobserved" in result.degradation_reasons


def test_observation_limit_keeps_pre_change_corridor_bounds():
    frame = remove_planning_window_lower_floor(
        render_track_depth(width_m=1.5, center_offset_m=0.12)
    )
    result = estimate(
        make_estimator(),
        frame,
    )

    # HEAD 6cb0252 변경 전 값: provenance 분리만으로 corridor 경계는 움직이면 안 된다.
    assert result.path_available
    assert result.left_wheel_clearance_m == pytest.approx(0.5055, abs=1e-9)
    assert result.right_wheel_clearance_m == pytest.approx(0.2555, abs=1e-9)
    assert "lateral_reference_unobserved" in result.degradation_reasons


def test_unobserved_frame_uses_transported_last_observed_lateral_reference():
    estimator = make_estimator()
    first = summarize_lateral_reference_frame(
        estimator,
        center_offset_m=0.10,
        observed_edges=True,
        stamp_s=1.0,
    )
    fallback = summarize_lateral_reference_frame(
        make_estimator(),
        center_offset_m=-0.10,
        observed_edges=False,
        stamp_s=1.1,
        odometry_delta=OdometryDelta(dx_m=0.10, dy_m=0.0, dyaw_rad=0.0),
    )

    carried = summarize_lateral_reference_frame(
        estimator,
        center_offset_m=-0.10,
        observed_edges=False,
        stamp_s=1.1,
        odometry_delta=OdometryDelta(dx_m=0.10, dy_m=0.0, dyaw_rad=0.0),
    )

    assert first.path_offset_m == pytest.approx(0.10, abs=1e-9)
    assert fallback.path_offset_m == pytest.approx(-0.10, abs=1e-9)
    assert carried.path_offset_m == pytest.approx(first.path_offset_m, abs=1e-9)
    assert "lateral_reference_carried" in carried.degradation_reasons
    assert "lateral_reference_unobserved" not in carried.degradation_reasons


def test_positive_lateral_odometry_moves_carried_reference_right():
    """dy>0 means rover-left in the previous body frame, so the track offset decreases."""
    estimator = make_estimator()
    first = summarize_lateral_reference_frame(
        estimator,
        center_offset_m=0.10,
        observed_edges=True,
        stamp_s=1.0,
    )

    carried = summarize_lateral_reference_frame(
        estimator,
        center_offset_m=-0.10,
        observed_edges=False,
        stamp_s=1.1,
        odometry_delta=OdometryDelta(dx_m=0.0, dy_m=0.04, dyaw_rad=0.0),
    )

    assert carried.path_offset_m == pytest.approx(first.path_offset_m - 0.04, abs=1e-9)
    assert "lateral_reference_carried" in carried.degradation_reasons


def test_lateral_reference_carry_expires_after_accumulated_travel_budget():
    estimator = make_estimator(lateral_reference_carry_m=1.0)
    summarize_lateral_reference_frame(
        estimator,
        center_offset_m=0.10,
        observed_edges=True,
        stamp_s=1.0,
    )
    within_budget = summarize_lateral_reference_frame(
        estimator,
        center_offset_m=-0.10,
        observed_edges=False,
        stamp_s=1.1,
        odometry_delta=OdometryDelta(dx_m=0.60, dy_m=0.0, dyaw_rad=0.0),
    )

    expired = summarize_lateral_reference_frame(
        estimator,
        center_offset_m=-0.10,
        observed_edges=False,
        stamp_s=1.2,
        odometry_delta=OdometryDelta(dx_m=0.41, dy_m=0.0, dyaw_rad=0.0),
    )

    assert "lateral_reference_carried" in within_budget.degradation_reasons
    assert "lateral_reference_unobserved" not in within_budget.degradation_reasons
    assert "lateral_reference_unobserved" in expired.degradation_reasons
    assert "lateral_reference_carried" not in expired.degradation_reasons


def test_longer_lateral_reference_carry_reduces_reported_wheel_clearance():
    def carry(distance_m: float) -> TerrainEstimate:
        estimator = make_estimator(lateral_reference_carry_drift=0.03)
        summarize_lateral_reference_frame(
            estimator,
            center_offset_m=0.0,
            observed_edges=True,
            stamp_s=1.0,
        )
        return summarize_lateral_reference_frame(
            estimator,
            center_offset_m=0.0,
            observed_edges=False,
            stamp_s=1.1,
            odometry_delta=OdometryDelta(
                dx_m=distance_m,
                dy_m=0.0,
                dyaw_rad=0.0,
            ),
        )

    shorter = carry(0.20)
    longer = carry(0.60)
    expected_reduction_m = (0.60 - 0.20) * 0.03

    assert "lateral_reference_carried" in shorter.degradation_reasons
    assert "lateral_reference_carried" in longer.degradation_reasons
    assert shorter.left_wheel_clearance_m - longer.left_wheel_clearance_m == pytest.approx(
        expected_reduction_m,
        abs=1e-9,
    )
    assert shorter.right_wheel_clearance_m - longer.right_wheel_clearance_m == pytest.approx(
        expected_reduction_m,
        abs=1e-9,
    )


def test_rejected_frame_clears_carried_lateral_reference():
    control = make_estimator()
    summarize_lateral_reference_frame(
        control,
        center_offset_m=0.10,
        observed_edges=True,
        stamp_s=1.0,
    )
    still_carried = summarize_lateral_reference_frame(
        control,
        center_offset_m=-0.10,
        observed_edges=False,
        stamp_s=1.1,
        odometry_delta=OdometryDelta(dx_m=0.10, dy_m=0.0, dyaw_rad=0.0),
    )

    rejected_estimator = make_estimator()
    summarize_lateral_reference_frame(
        rejected_estimator,
        center_offset_m=0.10,
        observed_edges=True,
        stamp_s=1.0,
    )
    stale_frame = render_track_depth(stamp_s=2.0, width_m=1.5)
    rejected = estimate(rejected_estimator, stale_frame, now_s=2.251)
    after_reject = summarize_lateral_reference_frame(
        rejected_estimator,
        center_offset_m=-0.10,
        observed_edges=False,
        stamp_s=2.1,
        odometry_delta=OdometryDelta(dx_m=0.10, dy_m=0.0, dyaw_rad=0.0),
    )

    assert "lateral_reference_carried" in still_carried.degradation_reasons
    assert rejected.reject_reasons == ("stale_frame",)
    assert "lateral_reference_unobserved" in after_reject.degradation_reasons
    assert "lateral_reference_carried" not in after_reject.degradation_reasons


@pytest.mark.parametrize("missing_left", (True, False))
def test_one_sided_drop_evidence_bounds_the_corridor_by_observed_support(missing_left):
    frame = remove_lower_floor_side(
        render_track_depth(width_m=1.5),
        left=missing_left,
    )

    result = estimate(make_estimator(), frame)

    footprint_half = 0.3595 + 0.035
    missing_unverified = "left_edge_unverified" if missing_left else "right_edge_unverified"
    missing_drop = "left_drop_boundary" if missing_left else "right_drop_boundary"
    missing_clearance = (
        result.left_wheel_clearance_m
        if missing_left
        else result.right_wheel_clearance_m
    )
    assert result.path_available, result.reject_reasons
    assert missing_unverified in result.degradation_reasons
    assert missing_drop not in result.degradation_reasons
    assert missing_clearance == pytest.approx(0.75 - footprint_half, abs=0.08)


def test_fov_truncated_edges_bound_the_corridor_by_observed_support():
    estimator = make_estimator()
    result = estimate(estimator, render_track_depth(width_m=3.2))

    footprint_half = 0.3595 + 0.035
    support_columns = np.flatnonzero(np.any(estimator._grid.support_mask, axis=0))
    observed_width = support_columns.size * estimator.config.grid_resolution_m
    corridor_width = (
        result.left_wheel_clearance_m
        + result.right_wheel_clearance_m
        + 2.0 * footprint_half
    )
    assert result.path_available, result.reject_reasons
    assert "left_edge_unverified" in result.degradation_reasons
    assert "right_edge_unverified" in result.degradation_reasons
    assert corridor_width <= observed_width
    assert corridor_width < 2.5


def test_row_edge_ignores_support_island_beyond_a_no_data_gap():
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

    result = estimator._summarize(
        grid,
        stamp_s=1.0,
        frame_confidence=1.0,
        reasons=(),
        carried_count=0,
        odometry_residual_m=0.0,
        left_limit_y=np.full(estimator.grid_shape[0], 1.5),
        right_limit_y=np.full(estimator.grid_shape[0], -1.5),
        left_floor_seen=True,
        right_floor_seen=True,
    )

    # 기존 최외곽 셀 규칙이면 고립 셀 때문에 좌측 경계 0.75 m, 여유 0.3555 m가 된다.
    assert result.path_available, result.reject_reasons
    assert result.left_wheel_clearance_m == pytest.approx(0.1055, abs=0.005)


def test_no_connected_support_still_fails_closed():
    estimator = make_estimator()
    result = estimator._summarize(
        empty_grid(estimator.grid_shape),
        stamp_s=1.0,
        frame_confidence=1.0,
        reasons=(),
        carried_count=0,
        odometry_residual_m=0.0,
        left_limit_y=np.full(estimator.grid_shape[0], np.nan),
        right_limit_y=np.full(estimator.grid_shape[0], np.nan),
        left_floor_seen=False,
        right_floor_seen=False,
    )

    assert not result.path_available
    assert result.reject_reasons == ("no_connected_support",)


def test_local_choke_cannot_be_discarded_in_favour_of_wider_rows():
    frame = render_track_depth(width_m=1.5, choke_width_m=0.8)

    result = estimate(make_estimator(), frame)

    assert not result.path_available
    assert "erosion_empty" in result.reject_reasons


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


def test_temporal_jump_fails_closed_and_inherits_depth_quality_reason():
    estimator = make_estimator()
    first = render_track_depth(stamp_s=1.0, width_m=1.5)
    assert estimate(estimator, first).path_available
    jumped = np.array(first.depth_roi, copy=True)
    valid = (jumped > 0) & (jumped < 6000)
    jumped[valid] = jumped[valid] + 600
    second = TerrainFrame(jumped, 0.001, WIDE_INTRINSICS, 1.1)

    result = estimate(estimator, second)

    assert not result.path_available
    assert "temporal_jump" in result.reject_reasons

    recovered_frame = TerrainFrame(first.depth_roi, 0.001, WIDE_INTRINSICS, 1.2)
    recovered = estimate(estimator, recovered_frame)
    assert recovered.path_available, recovered.reject_reasons


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


def test_narrow_erosion_and_stale_input_fail_closed():
    narrow = estimate(make_estimator(), render_track_depth(width_m=0.75))
    stale_frame = render_track_depth(stamp_s=2.0, width_m=1.5)
    stale = estimate(make_estimator(), stale_frame, now_s=2.251)

    assert not narrow.path_available
    assert narrow.reject_reasons == ("erosion_empty",)
    assert not stale.path_available
    assert stale.reject_reasons == ("stale_frame",)


def test_as_built_v2_0_90_m_track_is_traversable_with_55_5_mm_wheel_clearance():
    """The 0.90 m course clears each outer wheel edge by 55.5 mm.

    The as-built v2 footprint half-width is 0.3595 + 0.035 = 0.3945 m, so
    the physical wheel-edge clearance is 0.4500 - 0.3945 = 0.0555 m.
    The 5 mm assertion tolerance is one tenth of the estimator's 50 mm grid
    cell: tight enough to catch a one-cell boundary regression.
    """
    result = estimate(make_estimator(), render_track_depth(width_m=0.90))

    assert result.path_available, result.reject_reasons
    assert result.reject_reasons == ()
    assert result.left_wheel_clearance_m == pytest.approx(0.0555, abs=0.005)
    assert result.right_wheel_clearance_m == pytest.approx(0.0555, abs=0.005)


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
    assert "odometry_carried" in result.degradation_reasons


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
    assert "odometry_carried" not in result.degradation_reasons


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

    result = estimate(
        make_estimator(),
        TerrainFrame(depth, 0.001, WIDE_INTRINSICS, frame.stamp_s),
    )

    assert "local_obstacle" in result.degradation_reasons
    assert not result.path_available
    assert "obstacle_blocks_path" in result.reject_reasons


@pytest.mark.parametrize(
    ("obstacle_y_m", "expect_blocked"),
    ((0.025, True), (1.425, False)),
    ids=("inside_eroded_corridor", "outside_eroded_corridor"),
)
def test_obstacle_rejection_is_scoped_to_eroded_drivable_corridor(
    obstacle_y_m,
    expect_blocked,
):
    estimator = make_estimator()
    frame = render_track_depth(width_m=1.5)
    clean = estimate(estimator, frame)
    assert clean.path_available, clean.reject_reasons

    obstacle_mask = np.zeros(estimator.grid_shape, dtype=bool)
    x_centres = (
        estimator.config.grid_x_range_m[0]
        + (np.arange(estimator.grid_shape[0]) + 0.5)
        * estimator.config.grid_resolution_m
    )
    y_centres = (
        estimator.config.grid_y_range_m[0]
        + (np.arange(estimator.grid_shape[1]) + 0.5)
        * estimator.config.grid_resolution_m
    )
    x_index = int(np.argmin(np.abs(x_centres - 1.0)))
    y_index = int(np.argmin(np.abs(y_centres - obstacle_y_m)))
    obstacle_mask[x_index, y_index] = True
    grid = dataclasses.replace(estimator._grid, obstacle_mask=obstacle_mask)
    left_limit_y, right_limit_y = estimator._fov_limits(
        grid,
        frame_intrinsics=frame.intrinsics,
        extrinsic=BaseToCameraExtrinsic(),
        tilt=BodyTilt(roll_rad=0.0, pitch_rad=0.0),
    )

    result = estimator._summarize(
        grid,
        stamp_s=frame.stamp_s,
        frame_confidence=clean.confidence,
        reasons=(),
        carried_count=0,
        odometry_residual_m=0.0,
        left_limit_y=left_limit_y,
        right_limit_y=right_limit_y,
        left_floor_seen=True,
        right_floor_seen=True,
    )

    if expect_blocked:
        assert not result.path_available
        assert "obstacle_blocks_path" in result.reject_reasons
    else:
        assert result.path_available, result.reject_reasons
        assert "obstacle_blocks_path" not in result.reject_reasons


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
    assert all(
        "left_drop_boundary" in result.degradation_reasons
        and "right_drop_boundary" in result.degradation_reasons
        for result, _ in comparisons
    )


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
