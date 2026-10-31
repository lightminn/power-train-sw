"""B (map matching) test fixtures, built during A (spec §4.7).

Synthetic maps with a known truth, local scans taken from a known pose, the
ambiguous patterns B must refuse and the 1.2 cm height-reference case.  B only
has to plug its corrector class into tests that use these.
"""
from __future__ import annotations

from dataclasses import replace
import math

import numpy as np

from terrain_map import scan as S
from terrain_map.classmap import ClassMap
from terrain_map.config import MapConfig
from terrain_map.geometry import IDENTITY, Pose2D, transform_points


# --- surfaces: world (x, y) -> height above the support floor ------------------
def boxes(*specs):
    """specs: (x0, x1, y0, y1, h); a non-repeating layout with a unique match."""
    def surface(x, y):
        z = np.zeros(np.shape(x))
        for x0, x1, y0, y1, h in specs:
            z = np.where((x >= x0) & (x < x1) & (y >= y0) & (y < y1), h, z)
        return z
    return surface


SCATTERED = boxes((1.0, 1.2, -0.4, -0.2, 0.15), (1.5, 1.6, 0.1, 0.4, -0.12),
                  (2.0, 2.3, -0.1, 0.05, 0.10), (1.2, 1.3, 0.5, 0.6, 0.20))


def wall(x0=1.5, height=0.3):
    """A straight wall along y: any shift along y matches equally (must be refused)."""
    return lambda x, y: np.where(np.asarray(x) >= x0, height, 0.0)


def stripes(period=0.3, width=0.1, height=0.12):
    """Repeating stripes across x: shifts by one period match equally (must be refused)."""
    return lambda x, y: np.where(np.mod(np.asarray(x), period) < width, height, 0.0)


# --- maps and scans -------------------------------------------------------------
def synthetic_map(cfg: MapConfig, surface, extent=((-0.5, 3.0), (-1.5, 1.5)), observations=3, stamp=1.0):
    """ClassMap holding ``surface`` sampled at cell centres, voted ``observations`` times."""
    (x0, x1), (y0, y1) = extent
    gx, gy = np.meshgrid(np.arange(math.floor(x0 / cfg.cell_m), math.ceil(x1 / cfg.cell_m)),
                         np.arange(math.floor(y0 / cfg.cell_m), math.ceil(y1 / cfg.cell_m)), indexing="ij")
    gx, gy = gx.ravel(), gy.ravel()
    h = surface((gx + 0.5) * cfg.cell_m, (gy + 0.5) * cfg.cell_m).astype(np.float32)
    scan = S.LocalScan(gx.astype(np.int32), gy.astype(np.int32), h, S.classify(h, cfg),
                       np.full(len(h), 10, np.int32), stamp)
    m = ClassMap(cfg)
    for k in range(observations):
        m.fuse(replace(scan, stamp_s=stamp + k), IDENTITY, stamp + k)
    return m


def scan_from(cfg: MapConfig, surface, true_pose: Pose2D, *, forward=(0.3, 2.5), half_width=1.0,
              height_bias_m=0.0, stamp=10.0) -> S.LocalScan:
    """What the rover at ``true_pose`` would put in its local cells (heights relative to its floor)."""
    ix, iy = np.meshgrid(np.arange(math.floor(forward[0] / cfg.cell_m), math.ceil(forward[1] / cfg.cell_m)),
                         np.arange(-math.ceil(half_width / cfg.cell_m), math.ceil(half_width / cfg.cell_m)),
                         indexing="ij")
    ix, iy = ix.ravel(), iy.ravel()
    world = transform_points(true_pose, np.stack([(ix + 0.5) * cfg.cell_m, (iy + 0.5) * cfg.cell_m], axis=1))
    h = (surface(world[:, 0], world[:, 1]) + height_bias_m).astype(np.float32)
    return S.LocalScan(ix.astype(np.int32), iy.astype(np.int32), h, S.classify(h, cfg),
                       np.full(len(h), 10, np.int32), stamp)


def agreement(cfg: MapConfig, scan: S.LocalScan, pose: Pose2D, class_map: ClassMap):
    """(code agreement ratio, overlap count, height differences) of ``scan`` placed at ``pose``."""
    world = transform_points(pose, np.stack([(scan.ix + 0.5) * cfg.cell_m, (scan.iy + 0.5) * cfg.cell_m], axis=1))
    g = np.floor(world / cfg.cell_m).astype(np.int64)
    code, h, obs, _ = class_map.sample(g[:, 0], g[:, 1])
    seen = obs >= 3
    if not seen.any():
        return 0.0, 0, np.zeros(0)
    return float(np.mean(code[seen] == scan.code[seen])), int(seen.sum()), (h[seen] - scan.h_m[seen]).astype(float)
