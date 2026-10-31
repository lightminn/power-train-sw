"""Accumulated class map (spec §4.5)."""
import math

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


def local(cells, stamp=1.0, cfg=CFG):
    """cells: [(ix, iy, h)] -> LocalScan with codes from the real classifier."""
    ix, iy, h = (np.array(v) for v in zip(*cells))
    h = h.astype(np.float32)
    return S.LocalScan(ix.astype(np.int32), iy.astype(np.int32), h, S.classify(h, cfg),
                       np.full(len(h), 10, np.int32), float(stamp))


def one(m, gx, gy):
    code, h, obs, seen = m.sample(np.array([gx]), np.array([gy]))
    return int(code[0]), float(h[0]), int(obs[0]), float(seen[0])


def tile_cells(m, tx, ty, now):
    heights, codes, ages = C.unpack_cells(m.encode_tile(tx, ty, now)["cells"], CFG.tile_cells)
    return heights, codes, ages


# --- placement ----------------------------------------------------------------
def test_identity_pose_maps_cells_one_to_one():
    m = ClassMap(CFG)
    cells = [(0, 0, 0.0), (5, -3, 0.10), (-17, 40, -0.10)]
    changed = m.fuse(local(cells), IDENTITY, 1.0)
    for ix, iy, h in cells:
        code, height, obs, seen = one(m, ix, iy)
        assert (code, obs, seen) == (int(S.classify(np.array([h]), CFG)[0]), 1, 1.0)
        assert height == pytest.approx(h)
    assert changed == {(0, 0), (0, -1), (-2, 2)}                   # floor division into 16-cell tiles


def box_cells(base: BasePose):
    box = with_box(flat(), 2.0, 2.3, -0.15, 0.15, 0.15)
    depth = render_depth(HALF, base, Mount(), box)
    result = TerrainDetector(HALF, DetectorConfig(min_ground_points=CFG.ground_min_points)).process(depth, 0.0)
    frame = S.make_frame(result, CFG)
    m = ClassMap(CFG)
    m.fuse(S.build_local_scan(result, frame, 1.0, CFG), Pose2D(base.x, base.y, math.radians(base.yaw_deg)), 1.0)
    gx, gy = np.meshgrid(np.arange(30, 52), np.arange(-8, 8), indexing="ij")
    code, *_ = m.sample(gx.ravel(), gy.ravel())
    return {(int(a), int(b)) for a, b, c in zip(gx.ravel(), gy.ravel(), code) if c == C.OBSTACLE}


def test_same_box_lands_in_the_same_world_cells_from_two_poses():
    a = box_cells(BasePose())
    b = box_cells(BasePose(x=1.0))
    c = box_cells(BasePose(x=2.15, y=-1.5, yaw_deg=90.0))
    for other in (b, c):
        assert len(a & other) / len(a | other) > 0.6
        ca, co = np.mean(sorted(a), axis=0), np.mean(sorted(other), axis=0)
        assert np.hypot(*(ca - co)) <= 1.0                        # centroids within one cell
    assert {g for g in a if not (39 <= g[0] <= 46 and -4 <= g[1] <= 3)} == set()   # box +- one cell (face on a cell edge)


def colliding_pair(pose):
    ix, iy = np.meshgrid(np.arange(0, 20), np.arange(-10, 10), indexing="ij")
    ix, iy = ix.ravel(), iy.ravel()
    world = np.floor(transform_points(pose, np.stack([(ix + 0.5) * CFG.cell_m, (iy + 0.5) * CFG.cell_m], 1)) / CFG.cell_m)
    keys = [tuple(w) for w in world.astype(int)]
    seen = {}
    for i, k in enumerate(keys):
        if k in seen:
            return (ix[seen[k]], iy[seen[k]]), (ix[i], iy[i]), k
        seen[k] = i
    raise AssertionError("no collision")


@pytest.mark.parametrize("ditch_first", [True, False])
def test_world_cell_collision_keeps_the_more_severe_cell(ditch_first):
    pose = Pose2D(0.01, 0.02, math.radians(45.0))
    p, q, (gx, gy) = colliding_pair(pose)
    cells = [(*p, -0.09), (*q, 0.07)] if ditch_first else [(*q, 0.07), (*p, -0.09)]
    m = ClassMap(CFG)
    m.fuse(local(cells), pose, 1.0)
    code, h, obs, _ = one(m, gx, gy)
    assert (code, obs) == (C.DITCH, 1) and h == pytest.approx(-0.09)


# --- history ---------------------------------------------------------------
def votes(m, hs, stamps):
    for h, t in zip(hs, stamps):
        m.fuse(local([(3, 3, h)], t), IDENTITY, t)
    return one(m, 3, 3)


def test_majority_ignores_one_noisy_vote_and_follows_two_new_ones():
    assert votes(ClassMap(CFG), [0.0, 0.0, 0.12], [1, 2, 3])[0] == C.FREE           # noise once
    m = ClassMap(CFG)
    assert votes(m, [0.12, 0.12, 0.0], [1, 2, 3])[0] == C.OBSTACLE                 # removed once: still there
    assert votes(m, [0.0], [4])[0] == C.FREE                                       # removed twice: gone
    on_top = ClassMap(CFG)                                                         # climbed onto the top surface
    assert votes(on_top, [0.15, 0.15, 0.15, 0.0, 0.0], [1, 2, 3, 4, 5])[0] == C.FREE


def test_old_votes_expire_before_a_new_drop():
    code, h, obs, seen = votes(ClassMap(CFG), [0.0, 0.0, -0.20], [0.0, 0.2, 120.0])
    assert (code, obs, seen) == (C.DITCH, 3, 120.0) and h == pytest.approx(-0.20)


def test_display_height_is_the_last_observation_not_a_blend():
    assert votes(ClassMap(CFG), [0.03, -0.02, 0.01], [1, 2, 3])[1] == pytest.approx(0.01)


def test_history_length_one_follows_every_observation():
    cfg = MapConfig(history_length=1)
    m = ClassMap(cfg)
    for h, t, want in ((0.0, 1, C.FREE), (0.12, 2, C.OBSTACLE), (0.0, 3, C.FREE)):
        m.fuse(local([(3, 3, h)], t, cfg), IDENTITY, t)
        assert one(m, 3, 3)[0] == want


# --- transmission and version ----------------------------------------------
def test_tile_bytes_layout_heights_codes_and_unobserved():
    m = ClassMap(CFG)
    m.fuse(local([(1, 2, 0.123), (2, 1, -0.5)]), IDENTITY, 1.0)
    m.fuse(local([(1, 2, 0.123)], 2.0), IDENTITY, 2.0)
    heights, codes, ages = tile_cells(m, 0, 0, now=6.5)
    n = CFG.tile_cells
    assert (heights[2 * n + 1], codes[2 * n + 1], ages[2 * n + 1]) == (12, C.OBSTACLE, 2)   # row = gy, column = gx
    assert (heights[1 * n + 2], codes[1 * n + 2], ages[1 * n + 2]) == (-50, C.INSUFFICIENT, 2)  # one observation < 2
    assert heights[0] is None and codes[0] == C.INSUFFICIENT and ages[0] == 0


def test_tile_age_saturates_and_clips_height():
    m = ClassMap(CFG)
    m.fuse(local([(0, 0, 1.0)]), IDENTITY, 0.0)
    heights, _, ages = tile_cells(m, 0, 0, now=1000.0)
    assert heights[0] == 100 and ages[0] == C.MAX_AGE_STEPS


def test_global_version_rules():
    m = ClassMap(CFG)
    m.fuse(local([(0, 0, 0.0)], 1.0), IDENTITY, 1.0)
    v1 = m.version_of(0, 0)
    assert v1 >= 1 and m.encode_tile(0, 0, 1.0)["version"] == v1
    m.encode_tile(0, 0, 50.0)                                     # age only
    assert m.version_of(0, 0) == v1
    m.fuse(local([(0, 0, 0.0)], 2.0), IDENTITY, 2.0)              # 4 -> 0 release changes the code bits
    v2 = m.version_of(0, 0)
    assert v2 > v1
    m.fuse(local([(0, 0, 0.0)], 3.0), IDENTITY, 3.0)              # same bytes again
    assert m.version_of(0, 0) == v2
    m.fuse(local([(40, 0, 0.0)], 4.0), IDENTITY, 4.0)             # another tile takes the next counter value
    assert m.version_of(2, 0) > v2
    m.fuse(local([(0, 0, 0.004)], 5.0), IDENTITY, 5.0)            # height byte unchanged (0 cm)
    assert m.version_of(0, 0) == v2
    m.fuse(local([(0, 0, 0.02)], 6.0), IDENTITY, 6.0)             # height byte 0 -> 2 cm
    assert m.version_of(0, 0) > m.version_of(2, 0)


def test_eviction_drops_the_oldest_tiles_and_regrown_tiles_get_larger_versions():
    cfg = MapConfig(max_map_tiles=64)
    m = ClassMap(cfg)
    for k in range(70):
        m.fuse(local([(16 * k, 0, 0.0)], float(k), cfg), IDENTITY, float(k))
    old_version = m.version_of(0, 0)
    evicted = m.evict_if_needed()
    assert sorted(evicted) == [(k, 0) for k in range(6)] and len(m.tiles()) == 64
    assert m.version_of(0, 0) is None and one(m, 0, 0)[2] == 0
    m.fuse(local([(0, 0, 0.0)], 100.0, cfg), IDENTITY, 100.0)
    assert m.version_of(0, 0) > old_version and m.evict_if_needed() == [(6, 0)]


def test_sample_has_no_side_effects_and_reports_unobserved():
    m = ClassMap(CFG)
    m.fuse(local([(0, 0, 0.12)]), IDENTITY, 1.0)
    before = (m.version_of(0, 0), m.tiles(), m.encode_tile(0, 0, 1.0))
    code, h, obs, seen = m.sample(np.array([0, 500, -1]), np.array([0, 500, 0]))
    assert list(code) == [C.OBSTACLE, C.INSUFFICIENT, C.INSUFFICIENT]
    assert np.isnan(h[1:]).all() and list(obs) == [1, 0, 0] and np.isnan(seen[1:]).all()
    assert (m.version_of(0, 0), m.tiles(), m.encode_tile(0, 0, 1.0)) == before
    assert m.version_of(31, 31) is None


def test_clear_empties_the_map_and_restarts_versions():
    m = ClassMap(CFG)
    m.fuse(local([(0, 0, 0.0), (40, 0, 0.0)]), IDENTITY, 1.0)
    m.clear(epoch=1)
    assert m.tiles() == [] and m.epoch == 1
    m.fuse(local([(0, 0, 0.0)]), IDENTITY, 2.0)
    assert m.version_of(0, 0) == 1


def test_eviction_follows_the_fusion_order_not_the_receive_clock():
    """Codex T6 re-review: rx order may differ from stamp order; the replay has rx = stamp."""
    cfg = MapConfig(max_map_tiles=64)
    m = ClassMap(cfg)
    m.fuse(local([(0, 0, 0.0)], 100.0, cfg), IDENTITY, 100.0, seen_s=100.25)       # A: fused first, received last
    m.fuse(local([(16, 0, 0.0)], 100.2, cfg), IDENTITY, 100.2, seen_s=100.22)      # B
    for k in range(2, 65):
        m.fuse(local([(16 * k, 0, 0.0)], 101.0 + k, cfg), IDENTITY, 101.0 + k, seen_s=101.0 + k)
    assert m.evict_if_needed() == [(0, 0)]
