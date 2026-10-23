"""Depth→colour projection and outline polygons (spec §4.3, §4.4)."""
import numpy as np
import pytest

from terrain_overlay.contract import MAX_PACKET_BYTES, encoded_size
from terrain_overlay.contours import (
    approximate_polygon, build_obstacles, fit_packet_size, outline_contours,
)
from terrain_overlay.detector import TerrainDetector
from terrain_overlay.projection import DepthToColorProjector
from terrain_overlay.tests.scenes import COLOR, DEPTH, Box, default_scene, pitched_normal, render_depth


def test_centre_point_lands_24mm_lower_in_colour():
    pixels, kept = DepthToColorProjector(COLOR).project(np.array([[0.0, 0.0, 1.0]]))
    assert kept.tolist() == [True]
    assert pixels[0, 0] == 640
    assert pixels[0, 1] == pytest.approx(360 + COLOR.fy * 0.024, abs=1.0)


def test_zero_depth_and_out_of_frame_points_are_dropped_without_error():
    points = np.array([[0.0, 0.0, 1.0], [0.0, 0.0, 0.0], [5.0, 0.0, 1.0], [0.1, 0.1, -1.0]])
    pixels, kept = DepthToColorProjector(COLOR).project(points)
    assert kept.tolist() == [True, False, False, False]
    assert len(pixels) == 1


def test_thin_long_outline_keeps_at_least_three_points():
    # 1100×8 px collapses to 2 points under approxPolyDP (2026-10-17 repro).
    mask = np.zeros((720, 1280), np.uint8)
    mask[300:308, 100:1200] = 255
    contours = outline_contours(mask)
    assert len(contours) == 1
    assert len(approximate_polygon(contours[0], 1280, 720)) == 4


def test_default_scene_produces_one_in_path_outline():
    result = TerrainDetector(DEPTH).process(default_scene(), 0.0)
    obstacles = build_obstacles(
        result.obstacle_points, result.obstacle_fwd, result.obstacle_lat,
        DepthToColorProjector(COLOR), half_width_m=0.4445, front_offset_m=0.0,
    )
    assert len(obstacles) == 1
    assert obstacles[0]["in_path"] is True
    assert obstacles[0]["dist_m"] == pytest.approx(1.20, abs=0.03)
    assert 3 <= len(obstacles[0]["poly"]) <= 16


def test_crowded_scene_is_bounded_and_nearest_first():
    boxes = [Box(f, lat, 0.15, 0.3) for f in (0.9, 1.4, 1.9, 2.4) for lat in (-1.0, -0.5, 0.0, 0.5, 1.0)]
    result = TerrainDetector(DEPTH).process(render_depth(pitched_normal(20.0), 0.5, boxes), 0.0)
    obstacles = build_obstacles(
        result.obstacle_points, result.obstacle_fwd, result.obstacle_lat,
        DepthToColorProjector(COLOR), half_width_m=0.4445, front_offset_m=0.0,
    )
    assert len(obstacles) == 4
    distances = [item["dist_m"] for item in obstacles]
    assert distances == sorted(distances)
    for item in obstacles:
        assert 3 <= len(item["poly"]) <= 16
        assert all(0 <= x < 1280 and 0 <= y < 720 for x, y in item["poly"])


def _dense_payload(count=4, points=16):
    star = [[1279 - i, 719 - i] for i in range(points)]
    return {
        "header": "x" * 400,
        "obstacles": [{"dist_m": 1.0 + i, "in_path": True, "poly": list(star)} for i in range(count)],
    }


def test_fit_packet_size_halves_dense_polygons_first():
    payload = _dense_payload()
    assert encoded_size(payload) > MAX_PACKET_BYTES
    fitted = fit_packet_size(payload)
    assert encoded_size(fitted) <= MAX_PACKET_BYTES
    assert len(fitted["obstacles"]) == 4
    assert all(len(item["poly"]) == 8 for item in fitted["obstacles"])


def test_fit_packet_size_then_drops_the_farthest():
    fitted = fit_packet_size(_dense_payload(), limit=700)
    assert encoded_size(fitted) <= 700
    assert [item["dist_m"] for item in fitted["obstacles"]] == [1.0, 2.0][: len(fitted["obstacles"])]
    assert len(fitted["obstacles"]) < 4
