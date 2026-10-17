"""Support-plane check and per-frame local cells (spec §4.2, §4.3) on independent world renders."""
import math

import numpy as np
import pytest

from terrain_overlay.camera import CameraModel
from terrain_overlay.detector import DetectorConfig, TerrainDetector
from terrain_map import scan as S
from terrain_map.config import MapConfig
from terrain_map.tests.world_render import (
    BasePose, Mount, camera_in_world, flat, render_depth, stairs, with_box, with_ditch,
)

FULL = CameraModel(fx=460.0, fy=460.0, cx=320.0, cy=240.0, width=640, height=480)
HALF = CameraModel(fx=230.0, fy=230.0, cx=160.0, cy=120.0, width=320, height=240)
MOUNT = Mount()
CFG = MapConfig()
HEIGHT_CFG = MapConfig(support_base_height_m=0.471, support_base_variation_m=0.03, support_height_tol_m=0.04)


def foot_x(base: BasePose, mount: Mount = MOUNT) -> float:
    origin, _ = camera_in_world(base, mount)
    return float(origin[0])


def run(surface, base=BasePose(), cfg=CFG, camera=HALF):
    depth = render_depth(camera, base, MOUNT, surface)
    result = TerrainDetector(camera, DetectorConfig(min_ground_points=cfg.ground_min_points)).process(depth, 0.0)
    assert result.status == "OK" and result.ground_state == "FRESH", result.status
    frame = S.make_frame(result, cfg)
    return result, frame, S.support_check(result, frame, cfg), S.build_local_scan(result, frame, 1.0, cfg)


def cells_at(scan, x0, x1, y0, y1, cell=0.05):
    cx = (scan.ix + 0.5) * cell
    cy = (scan.iy + 0.5) * cell
    return (cx >= x0) & (cx <= x1) & (cy >= y0) & (cy <= y1)


# --- detector extension (PR #15 defaults unchanged) -------------------------
def test_detector_result_carries_points_and_plane():
    result, *_ = run(flat())
    normal, offset = result.ground_plane
    assert result.points.shape == (HALF.height, HALF.width, 3) and result.valid.shape == (HALF.height, HALF.width)
    assert np.linalg.norm(normal) == pytest.approx(1.0)
    assert DetectorConfig().min_ground_points == 200


def test_sparse_floor_needs_the_lower_map_minimum():
    rng = np.random.default_rng(3)
    depth = render_depth(FULL, BasePose(), MOUNT, flat())
    sparse = np.zeros_like(depth)
    sparse[:240, :] = 12000                                   # far wall: keeps the valid fraction up
    lower = np.argwhere(depth[240:, :] > 0)
    keep = lower[rng.choice(len(lower), 600, replace=False)]
    sparse[keep[:, 0] + 240, keep[:, 1]] = depth[keep[:, 0] + 240, keep[:, 1]]
    small, cam = S.decimate(sparse, FULL, 2)
    assert TerrainDetector(cam, DetectorConfig(min_ground_points=200)).process(small, 0.0).status == "NO_GROUND"
    assert TerrainDetector(cam, DetectorConfig(min_ground_points=100)).process(small, 0.0).ground_state == "FRESH"


def test_decimation_keeps_the_plane():
    depth = render_depth(FULL, BasePose(pitch_deg=4.0), MOUNT, flat())
    full = TerrainDetector(FULL).process(depth, 0.0)
    small, cam = S.decimate(depth, FULL, 2)
    half = TerrainDetector(cam).process(small, 0.0)
    n_full, d_full = full.ground_plane
    n_half, d_half = half.ground_plane
    assert math.degrees(math.acos(min(1.0, abs(float(n_full @ n_half))))) < 0.5
    assert abs(d_full - d_half) < 0.01
    assert (cam.width, cam.height, cam.fx, cam.cx) == (320, 240, 230.0, 160.0)


@pytest.mark.parametrize("stride", [1, 2])
@pytest.mark.parametrize("shape", [(768, 1024), (480,), (480, 640, 1)])
def test_decimation_rejects_depth_that_does_not_match_camera_info(stride, shape):
    with pytest.raises(ValueError):
        S.decimate(np.zeros(shape, np.uint16), FULL, stride)


# --- support check ----------------------------------------------------------
def test_flat_floor_is_support():
    *_, support, _ = run(flat(), cfg=HEIGHT_CFG)
    assert support.ok and abs(support.base_height_m - 0.471) < 0.01


@pytest.mark.parametrize("start, depth_m, base, expect_reason", [
    (1.00, 0.20, BasePose(), "near_band"),                    # near band sees the floor 0.2 above the ditch plane
    (0.45, 0.20, BasePose(), "base_height"),                  # near band is ditch bottom; height check catches it
    (0.30, 0.10, BasePose(pitch_deg=15.0), "base_height"),    # Codex 5th-round: body pitch + 10 cm ditch
])
def test_wide_ditch_bottom_is_not_support(start, depth_m, base, expect_reason):
    fx = foot_x(base)
    surface = with_ditch(flat(), fx + start, fx + start + 2.4, -1.2, 1.2, depth_m)
    *_, support, _ = run(surface, base=base, cfg=HEIGHT_CFG)
    assert not support.ok and support.reason == expect_reason


def test_pitched_body_on_flat_floor_stays_support():
    *_, support, _ = run(flat(), base=BasePose(pitch_deg=10.0), cfg=HEIGHT_CFG)   # camera height 0.439
    assert support.ok


def test_both_checks_off_negative_control_lets_the_ditch_in():
    base = BasePose()
    fx = foot_x(base)
    surface = with_ditch(flat(), fx + 0.45, fx + 2.85, -1.2, 1.2, 0.20)
    result, frame, support, scan = run(surface, cfg=CFG)          # height check off by default
    assert support.ok                                             # near band alone accepts the ditch bottom
    inside = cells_at(scan, 0.348 + 0.6, 0.348 + 2.5, -1.0, 1.0)
    assert inside.sum() > 50 and np.all(scan.code[inside] == S.FREE)


def test_small_box_in_the_near_band_rejects_the_frame_but_not_beyond():
    fx = foot_x(BasePose())
    near = with_box(flat(), fx + 0.35, fx + 0.65, -0.15, 0.15, 0.30)
    far = with_box(flat(), fx + 1.2, fx + 1.5, -0.15, 0.15, 0.30)
    assert not run(near)[2].ok
    assert run(far)[2].ok


# --- per-frame classification ----------------------------------------------
def test_box_is_obstacle_at_its_world_position():
    fx = foot_x(BasePose())
    *_, scan = run(with_box(flat(), fx + 1.2, fx + 1.5, -0.15, 0.15, 0.20))
    box_x0, box_x1 = fx + 1.2, fx + 1.5
    obstacle = scan.code == S.OBSTACLE
    cx = (scan.ix[obstacle] + 0.5) * 0.05
    cy = (scan.iy[obstacle] + 0.5) * 0.05
    assert obstacle.sum() >= 4
    assert np.all((cx >= box_x0 - 0.05) & (cx <= box_x1 + 0.05) & (np.abs(cy) <= 0.20))
    assert np.max(scan.h_m[obstacle]) == pytest.approx(0.20, abs=0.02)


def test_trench_is_ditch():
    fx = foot_x(BasePose())
    *_, scan = run(with_ditch(flat(), fx + 1.0, fx + 1.6, -0.6, 0.6, 0.15))
    ditch = scan.code == S.DITCH
    assert ditch.sum() >= 10
    assert np.min(scan.h_m[ditch]) == pytest.approx(-0.15, abs=0.02)


@pytest.mark.parametrize("surface_name", ["stairs", "two_steps", "ridge60", "ridge80"])
def test_steps_and_ridges_are_obstacles(surface_name):
    fx = foot_x(BasePose())
    surfaces = {
        "stairs": stairs(fx + 1.0, 0.10, 0.10, 8),
        "two_steps": stairs(fx + 1.0, 0.10, 0.10, 2),
        "ridge60": with_box(flat(), fx + 1.0, fx + 1.6, -1.0, 1.0, 0.20),
        "ridge80": with_box(flat(), fx + 1.0, fx + 1.8, -1.0, 1.0, 0.20),
    }
    *_, scan = run(surfaces[surface_name])
    region = cells_at(scan, fx + 1.15, fx + 1.5, -0.3, 0.3)
    assert region.sum() >= 5 and np.all(scan.code[region] == S.OBSTACLE)


def test_wide_ditch_seen_from_the_rim_is_ditch():
    fx = foot_x(BasePose())
    # Starting 1.4 m out, the floor still dominates the lower image half (from 1.0 m the
    # detector fits the ditch bottom instead, which the support check rejects above).
    *_, support, scan = run(with_ditch(flat(), fx + 1.4, fx + 3.8, -1.2, 1.2, 0.20))
    region = cells_at(scan, fx + 1.8, fx + 2.4, -0.3, 0.3)
    assert support.ok
    assert region.sum() >= 5 and np.all(scan.code[region] == S.DITCH)


def test_floor_below_a_plateau_edge_is_a_drop():
    base = BasePose(z=0.471 + 0.20)                              # rover standing on a 20 cm plateau
    fx = foot_x(base)
    surface = with_box(flat(), -5.0, fx + 1.2, -5.0, 5.0, 0.20)  # plateau ends 1.2 m ahead of the camera foot
    *_, scan = run(surface, base=base)
    beyond = cells_at(scan, fx + 1.5, fx + 2.2, -0.3, 0.3)
    near = cells_at(scan, fx + 0.4, fx + 0.9, -0.3, 0.3)
    assert np.all(scan.code[beyond] == S.DITCH) and np.all(scan.code[near] == S.FREE)


def test_cell_value_ignores_a_single_outlier_and_prefers_the_severe_side():
    cfg = MapConfig()
    assert S.cell_height(np.array([0.0, 0.0, 0.0, 0.0, 1.0]), cfg) == pytest.approx(0.0)
    asym = MapConfig(obstacle_height_m=0.08, ditch_depth_m=0.05)
    values = np.array([-0.06, -0.06, -0.06, 0.07, 0.07, 0.07])
    h = S.cell_height(values, asym)
    assert h == pytest.approx(-0.06) and S.classify(np.array([h]), asym)[0] == S.DITCH


def test_points_outside_range_are_dropped():
    *_, scan = run(flat())
    cx = (scan.ix + 0.5) * 0.05
    assert cx.min() >= 0.348 + 0.3 - 0.05 and cx.max() <= 0.348 + 3.0 + 0.05


def test_x86_processing_time_is_recorded(capsys):
    import time
    depth = render_depth(FULL, BasePose(), MOUNT, with_box(flat(), 1.6, 1.9, -0.15, 0.15, 0.2))
    started = time.perf_counter()
    small, cam = S.decimate(depth, FULL, 2)
    result = TerrainDetector(cam, DetectorConfig(min_ground_points=100)).process(small, 0.0)
    frame = S.make_frame(result, CFG)
    S.support_check(result, frame, CFG)
    S.build_local_scan(result, frame, 1.0, CFG)
    elapsed_ms = (time.perf_counter() - started) * 1000.0
    print(f"x86 stride-2 frame: {elapsed_ms:.1f} ms")
    assert elapsed_ms < 1000.0
