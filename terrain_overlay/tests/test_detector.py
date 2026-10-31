"""TerrainDetector on synthetic raw depth (spec §4.2, §7.1)."""
import tracemalloc

import numpy as np
import pytest

from terrain_overlay import detector as detector_module
from terrain_overlay.detector import DetectorConfig, TerrainDetector
from terrain_overlay.tests.scenes import (
    DEPTH, Box, default_scene, floor_only, pitched_normal, render_depth,
)

ROLL_NORMAL = (0.55, -0.779423, 0.30)  # Codex 2026-10-17 counterexample, tilt 38.79°


def wall(distance_m=1.0):
    """Fully valid depth with no floor-like plane (fit always fails)."""
    return np.full((DEPTH.height, DEPTH.width), round(distance_m / 0.00025), np.uint16)


def assert_default_scene_ok(result):
    assert result.status == "OK"
    assert result.ground_state == "FRESH"
    assert result.camera_height_m == pytest.approx(0.5, abs=0.01)
    assert result.camera_tilt_deg == pytest.approx(20.0, abs=0.5)
    assert result.zones["center"] == pytest.approx(1.20, abs=0.03)
    assert result.zones["left"] is None and result.zones["right"] is None
    assert len(result.obstacle_fwd) > 0
    assert np.all(np.abs(result.obstacle_lat) <= 0.4445)


def test_default_scene_finds_the_box_in_the_center_zone():
    assert_default_scene_ok(TerrainDetector(DEPTH).process(default_scene(), 0.0))


def test_negative_control_double_scale_bug_fails_the_scene_check():
    # Re-inject the PR #13 defect: metres already scaled, scaled again.
    buggy = TerrainDetector(DEPTH, DetectorConfig(depth_scale_m=0.00025 * 0.00025))
    with pytest.raises(AssertionError):
        assert_default_scene_ok(buggy.process(default_scene(), 0.0))


def test_metre_arrays_are_refused():
    metres = default_scene().astype(np.float32) * 0.00025
    with pytest.raises(TypeError):
        TerrainDetector(DEPTH).process(metres, 0.0)


def test_shape_mismatch_is_refused():
    with pytest.raises(ValueError):
        TerrainDetector(DEPTH).process(np.zeros((240, 320), np.uint16), 0.0)


def test_empty_depth_is_a_sensor_error():
    depth = np.zeros((DEPTH.height, DEPTH.width), np.uint16)
    depth[:10, :] = 4000  # 2 % valid
    result = TerrainDetector(DEPTH).process(depth, 0.0)
    assert (result.status, result.reason) == ("SENSOR_ERROR", "DEPTH_EMPTY")


def test_rolled_camera_keeps_a_straight_ahead_obstacle_in_the_center():
    depth = render_depth(ROLL_NORMAL, 0.5, [Box(2.7, 0.0, 0.3, 0.4)])
    result = TerrainDetector(DEPTH).process(depth, 0.0)
    assert result.status == "OK"
    assert result.camera_tilt_deg == pytest.approx(38.79, abs=0.3)
    assert result.zones["center"] == pytest.approx(2.70, abs=0.05)
    assert result.zones["left"] is None and result.zones["right"] is None


def test_plane_lifetime_follows_receive_time():
    detector = TerrainDetector(DEPTH)
    assert detector.process(floor_only(), 0.0).ground_state == "FRESH"
    held = detector.process(wall(), 0.3)
    assert (held.status, held.ground_state, held.fresh_rx_s) == ("OK", "HELD", 0.0)
    expired = detector.process(wall(), 0.6)
    assert (expired.status, expired.reason) == ("NO_GROUND", "GROUND_EXPIRED")
    still = detector.process(wall(), 0.7)
    assert (still.status, still.reason) == ("NO_GROUND", "GROUND_EXPIRED")
    assert detector.process(floor_only(), 0.8).ground_state == "FRESH"
    again = detector.process(wall(), 1.0)
    assert (again.ground_state, again.fresh_rx_s) == ("HELD", 0.8)


def test_startup_failure_is_not_found():
    result = TerrainDetector(DEPTH).process(wall(), 0.0)
    assert (result.status, result.reason) == ("NO_GROUND", "GROUND_NOT_FOUND")


def test_gap_without_process_calls_still_expires():
    detector = TerrainDetector(DEPTH)
    detector.process(floor_only(), 0.0)
    result = detector.process(wall(), 2.0)
    assert (result.status, result.reason) == ("NO_GROUND", "GROUND_EXPIRED")


def test_first_fresh_after_expiry_uses_only_the_new_plane():
    detector = TerrainDetector(DEPTH)
    assert detector.process(floor_only(tilt_deg=0.0), 0.0).ground_state == "FRESH"
    result = detector.process(floor_only(tilt_deg=30.0), 3.0)
    assert result.ground_state == "FRESH"
    assert result.camera_tilt_deg == pytest.approx(30.0, abs=0.5)
    assert result.camera_height_m == pytest.approx(0.5, abs=0.01)
    assert all(value is None for value in result.zones.values())  # floor is not an obstacle


def test_ransac_is_deterministic():
    first = TerrainDetector(DEPTH).process(default_scene(), 0.0)
    second = TerrainDetector(DEPTH).process(default_scene(), 0.0)
    assert first.camera_height_m == second.camera_height_m
    assert first.camera_tilt_deg == second.camera_tilt_deg


@pytest.mark.parametrize("depth", [floor_only(), floor_only(35.0, 0.79)],
                         ids=["lower-half-floor", "fully-valid-frame"])
def test_processing_memory_stays_bounded(depth):
    tracemalloc.start()
    try:
        TerrainDetector(DEPTH).process(depth, 0.0)
        _, peak = tracemalloc.get_traced_memory()
    finally:
        tracemalloc.stop()
    assert peak < 50 * 1024 * 1024


@pytest.mark.parametrize("height_m, tilt_deg, accepted", [
    (0.26, 20.0, True), (0.24, 20.0, False),
    (0.79, 20.0, True), (0.81, 20.0, False),
    (0.5, 39.0, True), (0.5, 41.0, False),
])
def test_plane_acceptance_bounds(height_m, tilt_deg, accepted):
    result = TerrainDetector(DEPTH).process(floor_only(tilt_deg, height_m), 0.0)
    assert (result.status == "OK") is accepted


def test_front_offset_shifts_zone_distance_and_floors_at_zero():
    shifted = TerrainDetector(DEPTH, DetectorConfig(front_offset_m=0.2)).process(default_scene(), 0.0)
    assert shifted.zones["center"] == pytest.approx(1.00, abs=0.03)
    clipped = TerrainDetector(DEPTH, DetectorConfig(front_offset_m=5.0)).process(default_scene(), 0.0)
    assert clipped.zones["center"] == 0.0


def test_axes_are_orthogonal_for_the_roll_counterexample():
    normal = np.asarray(ROLL_NORMAL) / np.linalg.norm(ROLL_NORMAL)
    forward, right = detector_module.ground_axes(normal)
    assert abs(forward @ right) < 1e-9
    assert abs(forward @ normal) < 1e-9 and abs(right @ normal) < 1e-9
    assert right[0] > 0
