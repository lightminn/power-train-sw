from __future__ import annotations

import dataclasses
import importlib.util

import numpy as np
import pytest


pytest.importorskip("jax")

from powertrain_autonomy.terrain.estimator import (  # noqa: E402
    BaseToCameraExtrinsic,
    BodyTilt,
    TerrainFrame,
)
from powertrain_autonomy.tests.test_terrain_estimator import (  # noqa: E402
    render_track_depth,
    make_estimator,
)


def _backend_modules():
    from powertrain_autonomy.terrain import jax_backend
    from powertrain_autonomy.terrain.kernel import (
        TerrainKernelConfig,
        build_terrain_grid_numpy,
    )

    return jax_backend, TerrainKernelConfig, build_terrain_grid_numpy


def _case(name: str) -> tuple[TerrainFrame, BodyTilt, BaseToCameraExtrinsic]:
    tilt = BodyTilt(roll_rad=0.0, pitch_rad=0.0)
    extrinsic = BaseToCameraExtrinsic()
    if name == "flat":
        frame = render_track_depth(width_m=1.5)
    elif name == "bank":
        frame = render_track_depth(bank_rad=0.13, width_m=1.5)
    elif name == "tilt":
        tilt = BodyTilt(roll_rad=0.11, pitch_rad=-0.08)
        frame = render_track_depth(body_tilt=tilt, width_m=1.5)
    elif name == "noise":
        frame = render_track_depth(noise_std_m=0.025, width_m=1.5)
    elif name == "hole":
        clean = render_track_depth(width_m=1.5)
        depth = np.array(clean.depth_roi, copy=True)
        depth[42:52, 35:45] = 0
        frame = TerrainFrame(
            depth_roi=depth,
            depth_scale_m=clean.depth_scale_m,
            intrinsics=clean.intrinsics,
            stamp_s=clean.stamp_s,
        )
    else:  # pragma: no cover - parametrization owns the values
        raise AssertionError(name)
    return frame, tilt, extrinsic


def _inputs(name: str):
    _, TerrainKernelConfig, _ = _backend_modules()
    frame, tilt, extrinsic = _case(name)
    estimator = make_estimator()
    depth = np.asarray(frame.depth_roi)
    depth_m = depth.astype(float) * frame.depth_scale_m
    point_mask = (
        np.isfinite(depth_m)
        & (depth_m >= estimator.config.min_depth_m)
        & (depth_m <= estimator.config.max_depth_m)
    )
    point_confidence = point_mask.astype(float)
    gravity_rotation = (
        estimator._rotation_y(tilt.pitch_rad)
        @ estimator._rotation_x(tilt.roll_rad)
    )
    rotation = gravity_rotation @ estimator._camera_to_base_rotation(extrinsic)
    translation_m = gravity_rotation @ np.array(
        (extrinsic.x_m, extrinsic.y_m, extrinsic.z_m), dtype=float
    )
    config = TerrainKernelConfig.from_estimator_config(estimator.config)
    kwargs = {
        "point_mask": point_mask,
        "point_confidence": point_confidence,
        "depth_scale_m": frame.depth_scale_m,
        "fx": frame.intrinsics.fx,
        "fy": frame.intrinsics.fy,
        "cx": frame.intrinsics.cx,
        "cy": frame.intrinsics.cy,
        "rotation": rotation,
        "translation_m": translation_m,
        "stamp_s": frame.stamp_s,
        "config": config,
    }
    return depth, kwargs


def _assert_grid_equal(numpy_grid, jax_grid) -> None:
    np.testing.assert_array_equal(
        jax_grid.observed_count, numpy_grid.observed_count
    )
    np.testing.assert_array_equal(jax_grid.valid_mask, numpy_grid.valid_mask)
    np.testing.assert_array_equal(
        np.isnan(jax_grid.height_m), np.isnan(numpy_grid.height_m)
    )
    np.testing.assert_array_equal(
        np.isnan(jax_grid.roughness_m), np.isnan(numpy_grid.roughness_m)
    )
    for name in (
        "points_m",
        "height_m",
        "slope_x",
        "slope_y",
        "roughness_m",
        "confidence",
        "stamp_s",
    ):
        np.testing.assert_allclose(
            getattr(jax_grid, name),
            getattr(numpy_grid, name),
            rtol=1e-5,
            atol=1e-6,
            equal_nan=True,
        )


def test_jax_backend_is_isolated_in_its_own_optional_module():
    assert (
        importlib.util.find_spec("powertrain_autonomy.terrain.jax_backend")
        is not None
    )
    assert (
        importlib.util.find_spec("powertrain_autonomy.terrain.kernel")
        is not None
    )


@pytest.mark.parametrize("name", ("flat", "bank", "tilt", "noise", "hole"))
def test_fixed_shape_numpy_and_jax_grid_kernels_are_equivalent(name):
    jax_backend, _, build_terrain_grid_numpy = _backend_modules()
    depth, kwargs = _inputs(name)

    numpy_grid = build_terrain_grid_numpy(depth, **kwargs)
    jax_grid = jax_backend.build_terrain_grid_jax(depth, **kwargs)

    _assert_grid_equal(numpy_grid, jax_grid)


def test_warmup_then_repeated_input_is_bit_deterministic():
    jax_backend, _, _ = _backend_modules()
    depth, kwargs = _inputs("noise")
    jax_backend.warmup(kwargs["config"])

    first = jax_backend.build_terrain_grid_jax(depth, **kwargs)
    second = jax_backend.build_terrain_grid_jax(depth, **kwargs)

    for field in dataclasses.fields(first):
        assert np.array_equal(
            getattr(first, field.name),
            getattr(second, field.name),
            equal_nan=True,
        )


def test_warmup_accepts_the_estimator_configuration_contract():
    jax_backend, _, _ = _backend_modules()

    jax_backend.warmup(make_estimator().config)


def test_shape_and_dtype_changes_are_rejected_before_jit_dispatch():
    jax_backend, _, _ = _backend_modules()
    depth, kwargs = _inputs("flat")

    with pytest.raises(ValueError, match="fixed depth shape"):
        jax_backend.build_terrain_grid_jax(depth[:-1], **kwargs)
    with pytest.raises(TypeError, match="uint16"):
        jax_backend.build_terrain_grid_jax(depth.astype(np.float32), **kwargs)
    with pytest.raises(TypeError, match="boolean"):
        jax_backend.build_terrain_grid_jax(
            depth,
            **{**kwargs, "point_mask": kwargs["point_mask"].astype(np.uint8)},
        )


def test_cpu_boundary_rejects_nonfinite_values_in_valid_output():
    jax_backend, _, _ = _backend_modules()
    depth, kwargs = _inputs("flat")
    result = jax_backend.build_terrain_grid_jax(depth, **kwargs)
    corrupted_points = np.array(result.points_m, copy=True)
    corrupted_points[10:50, :, :] = np.nan
    corrupted = dataclasses.replace(result, points_m=corrupted_points)

    with pytest.raises(ValueError, match="points_m must be finite"):
        jax_backend.validate_kernel_result(corrupted, kwargs["config"])

    corrupted_height = np.array(result.height_m, copy=True)
    corrupted_height[result.valid_mask] = np.nan
    corrupted = dataclasses.replace(result, height_m=corrupted_height)

    with pytest.raises(
        ValueError, match="height_m must be finite in valid cells"
    ):
        jax_backend.validate_kernel_result(corrupted, kwargs["config"])
