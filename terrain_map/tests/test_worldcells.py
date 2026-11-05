"""World-cell aggregation of frame points (spec r14 §4.3, §4.5)."""
import math
from dataclasses import replace

import numpy as np
import pytest

from terrain_overlay.detector import DetectorConfig, TerrainDetector
from terrain_map import contract as C
from terrain_map import scan as S
from terrain_map.classmap import ClassMap
from terrain_map.config import MapConfig
from terrain_map.geometry import IDENTITY, Pose2D, transform_points
from terrain_map.tests.world_render import BasePose, Mount, flat, render_depth, with_box
from terrain_overlay.camera import CameraModel

CFG = MapConfig()
HALF = CameraModel(fx=230.0, fy=230.0, cx=160.0, cy=120.0, width=320, height=240)


def reference_cells(ix, iy, h, cfg):
    """The r13 two-key lexsort statistic (spec §4.3-5) on float32 heights, kept here as the exact reference."""
    h = np.asarray(h, np.float32).astype(np.float64)
    cell_id = (ix.astype(np.int64) + (1 << 20)) * (1 << 21) + (iy.astype(np.int64) + (1 << 20))
    order = np.lexsort((h, cell_id))
    ids, hs = cell_id[order], h[order]
    starts = np.flatnonzero(np.r_[True, ids[1:] != ids[:-1]])
    counts = np.diff(np.r_[starts, len(ids)])
    good = counts >= cfg.min_points_per_cell
    starts, counts = starts[good], counts[good]
    k = np.maximum(1, np.floor(cfg.cell_trim_fraction * counts).astype(np.int64))
    if cfg.cell_trim_max_points > 0:
        k = np.minimum(k, cfg.cell_trim_max_points)
    k = np.minimum(k, (counts - 1) // 2)
    upper, lower = hs[starts + counts - 1 - k], hs[starts + k]
    hc = np.where(S.severity(upper, cfg) >= S.severity(lower, cfg), upper, lower)
    first = order[starts]
    return ix[first], iy[first], hc, counts


def points_scan(px, py, ph, stamp=1.0, cfg=CFG):
    """A LocalScan carrying points; its local cells come from the same statistic."""
    px, py, ph = (np.asarray(v, np.float32) for v in (px, py, ph))
    cx, cy, hc, code, count = S.cell_stats(np.floor(px / cfg.cell_m).astype(np.int64),
                                           np.floor(py / cfg.cell_m).astype(np.int64), ph.astype(np.float64), cfg)
    return S.LocalScan(cx.astype(np.int32), cy.astype(np.int32), hc.astype(np.float32), code, count.astype(np.int32),
                       float(stamp), px, py, ph)


def one(m, gx, gy):
    code, h, obs, _ = m.sample(np.array([gx]), np.array([gy]))
    return int(code[0]), float(h[0]), int(obs[0])


# --- one statistic for local and world cells ---------------------------------------------
@pytest.mark.parametrize("cap", [0, 2])
def test_cell_stats_matches_the_two_key_reference(cap):
    cfg = MapConfig(cell_trim_max_points=cap)
    rng = np.random.default_rng(7)
    n = 20000
    ix = rng.integers(-40, 60, n)
    iy = rng.integers(-70, 30, n)
    h = np.where(rng.random(n) < 0.05, rng.uniform(-0.3, 0.4, n), rng.normal(0.0, 0.01, n))
    cx, cy, hc, code, count = S.cell_stats(ix, iy, h, cfg)
    rx, ry, rh, rcount = reference_cells(ix, iy, h, cfg)
    assert np.array_equal(cx, rx) and np.array_equal(cy, ry) and np.array_equal(count, rcount)
    assert np.array_equal(hc, rh)                                  # exact order: the same element
    assert np.isin(hc, h.astype(np.float32).astype(np.float64)).all()
    assert np.array_equal(code, S.classify(hc, cfg))
    assert (count >= cfg.min_points_per_cell).all()


@pytest.mark.parametrize("heights", [
    [0.08004, 0.07996, 0.08004, 0.07996, 0.07996],      # Codex r14: a 0.1 mm key made this FREE
    [0.07996, 0.08004, 0.07996, 0.07996, 0.07996],      # ... and this OBSTACLE
    [-0.08001, -0.08001, 0.08004, 0.07996, 0.08004],    # ... and flipped the severity side (160 mm)
])
def test_threshold_ties_follow_the_exact_order_for_every_input_order(heights):
    import itertools
    want = None
    for perm in itertools.permutations(heights):
        h = np.array(perm)
        got = S.cell_stats(np.zeros(5, np.int64), np.zeros(5, np.int64), h, CFG)
        ref = reference_cells(np.zeros(5, np.int64), np.zeros(5, np.int64), h, CFG)
        assert float(got[2][0]) == float(ref[2][0])
        assert got[3][0] == S.classify(ref[2][:1], CFG)[0]
        want = float(got[2][0]) if want is None else want
        assert float(got[2][0]) == want                      # independent of the input order
    assert (want >= CFG.obstacle_height_m) == (sum(v >= CFG.obstacle_height_m for v in heights) >= 2)


def test_non_finite_heights_are_refused_and_non_finite_points_dropped():
    with pytest.raises(ValueError):
        S.cell_stats(np.zeros(5, np.int64), np.zeros(5, np.int64), np.array([0, 0, np.nan, 0, 0.0]), CFG)
    px = np.r_[np.full(6, 1.01), np.nan, np.inf]
    py = np.r_[np.full(6, 0.01), 0.01, 0.01]
    ph = np.r_[np.zeros(6), 0.0, 0.0]
    scan = S.LocalScan(np.zeros(0, np.int32), np.zeros(0, np.int32), np.zeros(0, np.float32), np.zeros(0, np.uint8),
                       np.zeros(0, np.int32), 1.0, px.astype(np.float32), py.astype(np.float32), ph.astype(np.float32))
    gx, gy, h, code = S.world_cells(scan, IDENTITY, CFG)
    assert gx.tolist() == [20] and gy.tolist() == [0]


def test_an_absurd_cell_extent_keeps_cells_apart():
    """r14 Codex round 2: the fallback must not group by an overflowed cell number."""
    far = 1 << 32
    ix = np.repeat(np.array([0, far, 0, far], np.int64), 5)
    iy = np.repeat(np.array([0, 0, far, far], np.int64), 5)
    cx, cy, h, code, count = S.cell_stats(ix, iy, np.zeros(20), CFG)
    assert sorted(zip(cx.tolist(), cy.tolist())) == [(0, 0), (0, far), (far, 0), (far, far)]
    assert count.tolist() == [5, 5, 5, 5]


def test_cell_stats_on_no_points_is_empty():
    cx, cy, hc, code, count = S.cell_stats(np.zeros(0, np.int64), np.zeros(0, np.int64), np.zeros(0), CFG)
    assert len(cx) == len(cy) == len(hc) == len(code) == len(count) == 0


def test_trim_cap_keeps_a_small_obstacle_cluster_in_a_floor_cell():
    """Codex r14 counterexample: 5 points at 12 cm among 100 floor points in one cell."""
    ix = np.zeros(105, np.int64)
    iy = np.zeros(105, np.int64)
    h = np.r_[np.full(5, 0.12), np.zeros(100)]
    _, _, off, code_off, _ = S.cell_stats(ix, iy, h, MapConfig())
    _, _, on, code_on, _ = S.cell_stats(ix, iy, h, MapConfig(cell_trim_max_points=2))
    assert float(off[0]) == 0.0 and int(code_off[0]) == C.FREE            # k = 10 trims the cluster
    assert float(on[0]) == pytest.approx(0.12) and int(code_on[0]) == C.OBSTACLE   # k = 2 keeps it


def test_trim_cap_is_validated():
    with pytest.raises(ValueError):
        MapConfig(cell_trim_max_points=11)
    with pytest.raises(ValueError):
        MapConfig(cell_trim_max_points=-1)


# --- frames carry their points -------------------------------------------------------------
def test_local_scan_carries_the_filtered_points_and_its_cells_come_from_them():
    base = BasePose()
    depth = render_depth(HALF, base, Mount(), with_box(flat(), 1.45, 1.75, -0.15, 0.15, 0.15))
    result = TerrainDetector(HALF, DetectorConfig(min_ground_points=CFG.ground_min_points)).process(depth, 0.0)
    frame = S.make_frame(result, CFG)
    scan = S.build_local_scan(result, frame, 1.0, CFG)
    assert scan.px is not None and len(scan.px) == len(scan.py) == len(scan.ph) > 1000
    assert scan.px.dtype == scan.py.dtype == scan.ph.dtype == np.float32
    assert int(scan.count.sum()) <= len(scan.px)
    cx, cy, hc, code, count = S.cell_stats(np.floor(scan.px.astype(np.float64) / CFG.cell_m).astype(np.int64),
                                           np.floor(scan.py.astype(np.float64) / CFG.cell_m).astype(np.int64),
                                           scan.ph.astype(np.float64), CFG)
    assert np.array_equal(cx, scan.ix) and np.array_equal(cy, scan.iy) and np.array_equal(count, scan.count)
    assert np.allclose(hc, scan.h_m, atol=1e-6) and np.array_equal(code, scan.code)


# --- world cells from points -------------------------------------------------------------
def test_face_points_do_not_land_inside_the_box_cell():
    """Codex r14 re-binning counterexample: cell 2.5 cm, pose x 0.013 m, a box face at world x 0.999 m.

    r13 put the local cell (ix 39, centre 0.9875) at world 1.0005 -> world cell 40 = [1.000, 1.025), inside
    the box; points are binned where they are, so cell 40 gets nothing from them.
    """
    cfg = MapConfig(cell_m=0.025)
    pose = Pose2D(0.013, 0.0, 0.0)
    hz = np.linspace(0.0, 0.15, 12)
    px = np.full(len(hz), 0.999 - 0.013)
    py = np.full(len(hz), 0.0101)
    scan = points_scan(px, py, hz, cfg=cfg)
    gx, gy, h, code = S.world_cells(scan, pose, cfg)
    assert list(zip(gx.tolist(), gy.tolist())) == [(39, 0)]
    m = ClassMap(cfg)
    m.fuse(scan, pose, 1.0)
    assert one(m, 40, 0)[2] == 0 and one(m, 39, 0)[2] == 1
    # the r13 path (same local cells, no points) writes the face height into the box cell
    legacy = S.LocalScan(scan.ix, scan.iy, scan.h_m, scan.code, scan.count, scan.stamp_s)
    gx13, _, h13, _ = S.world_cells(legacy, pose, cfg)
    assert gx13.tolist() == [40] and float(h13[0]) > 0.1


def test_world_cells_equal_binning_the_world_points_directly():
    rng = np.random.default_rng(3)
    n = 30000
    px = rng.uniform(0.3, 3.0, n)
    py = rng.uniform(-1.5, 1.5, n)
    ph = np.where(np.hypot(px - 1.7, py + 0.2) < 0.3, 0.2, 0.0) + rng.normal(0, 0.003, n)
    scan = points_scan(px, py, ph)
    pose = Pose2D(1.234, -0.567, math.radians(37.0))
    gx, gy, h, code = S.world_cells(scan, pose, CFG)
    w = transform_points(pose, np.stack([scan.px, scan.py], 1).astype(np.float64))
    rx, ry, rh, rcode, _ = S.cell_stats(np.floor(w[:, 0] / CFG.cell_m).astype(np.int64),
                                        np.floor(w[:, 1] / CFG.cell_m).astype(np.int64), scan.ph.astype(np.float64), CFG)
    assert np.array_equal(gx, rx) and np.array_equal(gy, ry)
    assert np.array_equal(h, rh) and np.array_equal(code, rcode)


def test_one_vote_per_world_cell_per_frame():
    rng = np.random.default_rng(5)
    px = rng.uniform(0.5, 2.0, 8000)
    py = rng.uniform(-0.5, 0.5, 8000)
    scan = points_scan(px, py, np.zeros(8000))
    m = ClassMap(CFG)
    pose = Pose2D(0.31, 0.17, math.radians(23.0))
    m.fuse(scan, pose, 1.0)
    gx, gy, _, _ = S.world_cells(scan, pose, CFG)
    assert len(set(zip(gx.tolist(), gy.tolist()))) == len(gx)
    _, _, obs, _ = m.sample(gx, gy)
    assert (obs == 1).all()


def test_heights_stay_frame_plane_heights_not_world_z():
    scan = points_scan(np.full(6, 1.0), np.full(6, 0.01), np.full(6, 0.05))
    gx, gy, h, code = S.world_cells(scan, Pose2D(3.0, 4.0, 1.0), CFG)
    assert np.allclose(h, 0.05) and int(code[0]) == C.FREE


def test_a_scan_without_points_keeps_the_r13_centre_path():
    scan = S.LocalScan(np.array([3, 3], np.int32), np.array([4, 5], np.int32), np.array([0.0, 0.2], np.float32),
                       np.array([C.FREE, C.OBSTACLE], np.uint8), np.array([9, 9], np.int32), 1.0)
    assert scan.px is None
    gx, gy, h, code = S.world_cells(scan, IDENTITY, CFG)
    assert sorted(zip(gx.tolist(), gy.tolist())) == [(3, 4), (3, 5)]


def test_points_and_cells_must_agree_in_length():
    with pytest.raises(ValueError):
        S.LocalScan(np.zeros(0, np.int32), np.zeros(0, np.int32), np.zeros(0, np.float32), np.zeros(0, np.uint8),
                    np.zeros(0, np.int32), 1.0, np.zeros(3, np.float32), np.zeros(2, np.float32), np.zeros(3, np.float32))


def test_world_cell_gets_points_from_local_cells_that_were_each_too_thin():
    """Codex r14: 3 + 3 points in two local cells (below min 5) are 6 points in one world cell."""
    px = np.r_[np.full(3, 0.999), np.full(3, 1.001)]
    scan = points_scan(px, np.full(6, 0.01), np.full(6, 0.12))
    assert len(scan.h_m) == 0                                   # no local cell has 5 points
    m = ClassMap(CFG)
    changed = m.fuse(scan, Pose2D(0.025, 0.0, 0.0), 1.0)
    assert changed and one(m, 20, 0)[0:3:2] == (C.OBSTACLE, 1)


def test_negative_world_cells_and_tiles():
    scan = points_scan(np.r_[np.full(5, 0.999), np.full(5, 1.001)], np.full(10, -0.001), np.zeros(10))
    gx, gy, _, _ = S.world_cells(scan, Pose2D(-1.0, 0.0, 0.0), CFG)
    assert sorted(zip(gx.tolist(), gy.tolist())) == [(-1, -1), (0, -1)]
    m = ClassMap(CFG)
    m.fuse(scan, Pose2D(-1.0, 0.0, 0.0), 1.0)
    assert sorted(m.tiles()) == [(-1, -1), (0, -1)]


def test_empty_point_scan_fuses_nothing():
    scan = points_scan(np.zeros(0), np.zeros(0), np.zeros(0))
    m = ClassMap(CFG)
    assert m.fuse(scan, IDENTITY, 1.0) == set()
    assert m.tiles() == []
