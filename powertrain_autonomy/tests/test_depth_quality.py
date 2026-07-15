import dataclasses

import numpy as np
import pytest

from powertrain_autonomy.terrain.depth_quality import (
    CameraIntrinsics,
    DepthQualityConfig,
    DepthQualitySnapshot,
    analyze_depth_quality,
    analyze_depth_quality_numpy,
)


INTRINSICS = CameraIntrinsics(fx=420.0, fy=420.0, cx=29.5, cy=19.5)


def flat_depth(*, depth_m=1.5, height=40, width=60):
    return np.full((height, width), round(depth_m * 1000), dtype=np.uint16)


def analyze(depth, **overrides):
    values = {
        "depth_scale_m": 0.001,
        "intrinsics": INTRINSICS,
        "frame_stamp_s": 10.0,
    }
    values.update(overrides)
    return analyze_depth_quality(depth, **values)


def test_flat_roi_reports_robust_metrics_from_the_roi_not_the_center_pixel():
    rng = np.random.default_rng(20260715)
    depth = np.rint((1.5 + rng.normal(0.0, 0.003, (40, 60))) * 1000).astype(np.uint16)
    depth[20, 30] = 0

    result = analyze(depth)

    assert result.accepted
    assert result.robust_depth_m == pytest.approx(1.5, abs=0.003)
    assert result.median_m == pytest.approx(1.5, abs=0.003)
    assert result.mad_m < 0.005
    assert result.lower_percentile_m < result.upper_percentile_m
    assert result.valid_ratio > 0.999
    assert result.connected_ratio > 0.999
    assert result.normal_consistency > 0.95
    assert result.confidence > 0.9
    assert result.reject_reasons == ()


def test_result_and_previous_snapshot_are_immutable():
    result = analyze(flat_depth())
    snapshot = result.snapshot()

    assert snapshot == DepthQualitySnapshot(frame_stamp_s=10.0, robust_depth_m=1.5)
    with pytest.raises(dataclasses.FrozenInstanceError):
        snapshot.robust_depth_m = 9.0


@pytest.mark.parametrize(
    ("damage", "reason"),
    [
        (lambda a: a.__setitem__((slice(0, 8), slice(0, 8)), np.nan), "invalid_depth"),
        (lambda a: a.__setitem__((slice(15, 25), slice(25, 35)), 0.0), "zero_depth"),
        (lambda a: a.__setitem__((slice(0, 8), slice(0, 8)), 9.0), "out_of_range_depth"),
        (lambda a: a.__setitem__((20, 30), 4.0), "isolated_spike"),
    ],
)
def test_invalid_zero_out_of_range_and_reflection_spike_have_explicit_reasons(
    damage, reason
):
    depth_m = np.full((40, 60), 1.5, dtype=float)
    damage(depth_m)

    result = analyze(depth_m, depth_scale_m=1.0)

    assert reason in result.reject_reasons
    assert not result.accepted


def test_nonreflective_hole_proxy_is_detected_as_an_enclosed_depth_hole():
    depth = flat_depth().astype(float)
    depth[15:25, 25:35] = 0.0

    result = analyze(depth)

    assert "depth_hole" in result.reject_reasons
    assert "zero_depth" in result.reject_reasons


def test_abrupt_temporal_jump_uses_previous_quality_snapshot():
    previous = analyze(flat_depth(depth_m=1.0), frame_stamp_s=9.9).snapshot()

    result = analyze(flat_depth(depth_m=1.55), previous=previous)

    assert result.temporal_delta_m == pytest.approx(0.55)
    assert "temporal_jump" in result.reject_reasons


def test_regressing_frame_stamp_is_rejected_separately_from_depth_change():
    previous = DepthQualitySnapshot(frame_stamp_s=10.0, robust_depth_m=1.5)

    result = analyze(flat_depth(), frame_stamp_s=9.9, previous=previous)

    assert "regressing_frame_stamp" in result.reject_reasons


def test_disconnected_deeper_floor_component_is_not_folded_into_robust_depth():
    depth_m = np.full((40, 60), np.nan, dtype=float)
    depth_m[:, :38] = 1.2
    depth_m[:, 43:] = 1.65

    result = analyze(depth_m, depth_scale_m=1.0)

    assert result.robust_depth_m == pytest.approx(1.2)
    assert result.connected_ratio == pytest.approx(38 / 55)
    assert "disconnected_lower_floor" in result.reject_reasons


def test_numpy_backend_is_an_explicit_boundary_and_jax_is_not_implemented():
    direct = analyze_depth_quality_numpy(
        flat_depth(),
        depth_scale_m=0.001,
        intrinsics=INTRINSICS,
        frame_stamp_s=10.0,
    )
    selected = analyze(
        flat_depth(),
        backend="numpy",
    )

    assert selected == direct
    with pytest.raises(ValueError, match="numpy is the only qualified backend"):
        analyze(flat_depth(), backend="jax")


def test_bad_surface_dispersion_and_normals_reduce_confidence_and_reject():
    rng = np.random.default_rng(17)
    depth_m = 1.5 + rng.normal(0.0, 0.12, (40, 60))
    config = DepthQualityConfig(
        connectivity_delta_m=0.3,
        max_mad_m=0.04,
        max_percentile_span_m=0.2,
        min_normal_consistency=0.9,
    )

    result = analyze(depth_m, depth_scale_m=1.0, config=config)

    assert "mad_exceeded" in result.reject_reasons
    assert "percentile_span_exceeded" in result.reject_reasons
    assert "low_normal_consistency" in result.reject_reasons
    assert result.confidence < 0.8
