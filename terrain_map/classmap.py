"""Accumulated class map (spec §4.5): per-cell code history with majority vote.

Each frame's cells were already classified against that frame's own floor
(§4.3); the map only votes over the last few codes per world cell and keeps the
last height for display.  Heights are never blended across frames.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from . import contract as C
from .config import MapConfig
from .geometry import Pose2D
from .scan import LocalScan, world_cells

_CODES = np.array([C.FREE, C.OBSTACLE, C.DITCH], np.uint8)
_MAX_OBS = np.iinfo(np.uint16).max


@dataclass
class _Tile:
    hist_code: np.ndarray      # (n*n, L) uint8, oldest first
    hist_t: np.ndarray         # (n*n, L) float64, NaN = empty slot
    code: np.ndarray           # (n*n,) uint8 majority result
    display_h: np.ndarray      # (n*n,) float32, NaN = unobserved
    observations: np.ndarray   # (n*n,) uint16
    last_seen: np.ndarray      # (n*n,) float64, NaN = unobserved
    last_fused: int            # ClassMap fusion counter when last touched (eviction order)
    version: int = 0
    signature: bytes = b""


class ClassMap:
    def __init__(self, cfg: MapConfig) -> None:
        self.cfg = cfg
        self.epoch = 0
        self._tiles: dict[tuple[int, int], _Tile] = {}
        self._counter = 0
        self._fusions = 0

    # --- maintenance --------------------------------------------------------
    def clear(self, epoch: int) -> None:
        self.epoch = epoch
        self._tiles.clear()
        self._counter = 0
        self._fusions = 0

    def tiles(self) -> list[tuple[int, int]]:
        return list(self._tiles)

    def version_of(self, tx: int, ty: int) -> int | None:
        tile = self._tiles.get((tx, ty))
        return None if tile is None else tile.version

    def evict_if_needed(self) -> list[tuple[int, int]]:
        """Drop the least recently fused tiles beyond ``max_map_tiles`` (no notice is sent; §4.5).

        Fusion order, not receive time: a decisions replay fuses the same frames in the same
        order but receives them at other times (Codex T6 re-review).
        """
        evicted = []
        while len(self._tiles) > self.cfg.max_map_tiles:
            key = min(self._tiles, key=lambda k: self._tiles[k].last_fused)
            del self._tiles[key]
            evicted.append(key)
        return evicted

    # --- fusion -------------------------------------------------------------
    def fuse(self, scan: LocalScan, pose: Pose2D, stamp_s: float, seen_s: float | None = None) -> set[tuple[int, int]]:
        """Vote one frame into the map; return the tiles whose version changed.

        ``stamp_s`` (sensor clock) orders and expires votes; ``seen_s`` is when the frame was
        received on the clock ``encode_tile`` is later given, for the age bits (Codex T6 P2-2).
        """
        # One vote per world cell: the frame's points binned with the final pose (r14), or for a
        # cell-only scan the r13 centre mapping with the more severe of two local cells.
        return self.fuse_cells(*world_cells(scan, pose, self.cfg), stamp_s, seen_s)

    def fuse_cells(self, gx, gy, h, code, stamp_s: float, seen_s: float | None = None) -> set[tuple[int, int]]:
        """Vote precomputed world cells (``scan.world_cells`` output, one per cell) into the map."""
        seen_s = stamp_s if seen_s is None else seen_s
        if len(gx) == 0:
            return set()
        self._fusions += 1
        n = self.cfg.tile_cells

        tx, ty = gx // n, gy // n
        changed = set()
        tile_key = tx * (1 << 31) + ty
        for k in np.unique(tile_key):
            sel = tile_key == k
            t = (int(tx[sel][0]), int(ty[sel][0]))
            idx = (gy[sel] - t[1] * n) * n + (gx[sel] - t[0] * n)
            if self._update_tile(t, idx, h[sel], code[sel], float(stamp_s), float(seen_s)):
                changed.add(t)
        return changed

    def _new_tile(self) -> _Tile:
        size, length = self.cfg.tile_cells ** 2, self.cfg.history_length
        return _Tile(
            hist_code=np.zeros((size, length), np.uint8),
            hist_t=np.full((size, length), np.nan),
            code=np.full(size, C.INSUFFICIENT, np.uint8),
            display_h=np.full(size, np.nan, np.float32),
            observations=np.zeros(size, np.uint16),
            last_seen=np.full(size, np.nan),
            last_fused=0,
        )

    def _update_tile(self, key, idx, h, code, stamp_s, seen_s) -> bool:
        tile = self._tiles.get(key)
        created = tile is None
        if created:
            tile = self._tiles[key] = self._new_tile()
        hist_t, hist_code = tile.hist_t[idx], tile.hist_code[idx]
        # Expire votes older than history_max_age_s before the new one (spec §4.5, Codex r4 P1-2).
        hist_t[hist_t < stamp_s - self.cfg.history_max_age_s] = np.nan
        order = np.argsort(np.where(np.isnan(hist_t), -np.inf, hist_t), axis=1, kind="stable")
        hist_t = np.take_along_axis(hist_t, order, axis=1)
        hist_code = np.take_along_axis(hist_code, order, axis=1)
        hist_t = np.concatenate([hist_t[:, 1:], np.full((len(idx), 1), stamp_s)], axis=1)
        hist_code = np.concatenate([hist_code[:, 1:], code[:, None].astype(np.uint8)], axis=1)
        valid = ~np.isnan(hist_t)
        counts = np.stack([((hist_code == c) & valid).sum(axis=1) for c in _CODES], axis=1)
        best = counts.argmax(axis=1)
        majority = np.where(counts.max(axis=1) >= 2, _CODES[best], hist_code[:, -1])

        tile.hist_t[idx], tile.hist_code[idx], tile.code[idx] = hist_t, hist_code, majority
        tile.display_h[idx] = h
        tile.observations[idx] = np.minimum(tile.observations[idx].astype(np.int64) + 1, _MAX_OBS)
        tile.last_seen[idx] = seen_s
        tile.last_fused = self._fusions

        heights, codes = self._transmitted(tile)
        signature = heights.tobytes() + codes.tobytes()
        if created or signature != tile.signature:
            self._counter += 1
            tile.version, tile.signature = self._counter, signature
            return True
        return False

    # --- transmission -------------------------------------------------------
    def _transmitted(self, tile: _Tile) -> tuple[np.ndarray, np.ndarray]:
        """Height bytes (int16 holding int8 cm, -128 unobserved) and transmitted codes."""
        observed = tile.observations > 0
        cm = np.clip(np.round(np.nan_to_num(tile.display_h.astype(np.float64)) * 100.0), -127, 127)
        heights = np.where(observed, cm, C.UNOBSERVED_H).astype(np.int16)
        codes = np.where(tile.observations >= self.cfg.min_observations, tile.code, C.INSUFFICIENT).astype(np.uint8)
        return heights, codes

    def encode_tile(self, tx: int, ty: int, now_s: float) -> dict:
        """Contract-ready tile body; the runtime adds ``tile_seq`` and the common fields."""
        tile = self._tiles[(tx, ty)]
        heights, codes = self._transmitted(tile)
        observed = tile.observations > 0
        age = np.where(observed, np.floor((now_s - np.nan_to_num(tile.last_seen)) / self.cfg.age_step_s), 0)
        age = np.clip(age, 0, C.MAX_AGE_STEPS).astype(int)
        cells = C.pack_cells([None if v == C.UNOBSERVED_H else int(v) for v in heights],
                             [int(c) for c in codes], [int(a) for a in age])
        return {"tx": tx, "ty": ty, "version": tile.version, "cells": cells}

    # --- query (no side effects; B uses this) --------------------------------
    def sample(self, gx, gy) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        """Per world cell: (majority code, display height, observations, last seen). Unobserved: 4, NaN, 0, NaN.

        The code is the voted code even below ``min_observations``; callers gate on ``observations``.
        """
        gx = np.asarray(gx, np.int64)
        gy = np.asarray(gy, np.int64)
        n = self.cfg.tile_cells
        code = np.full(gx.shape, C.INSUFFICIENT, np.uint8)
        height = np.full(gx.shape, np.nan, np.float32)
        obs = np.zeros(gx.shape, np.uint16)
        seen = np.full(gx.shape, np.nan)
        tx, ty = gx // n, gy // n
        for key in set(zip(tx.tolist(), ty.tolist())):
            tile = self._tiles.get(key)
            if tile is None:
                continue
            sel = (tx == key[0]) & (ty == key[1])
            idx = (gy[sel] - key[1] * n) * n + (gx[sel] - key[0] * n)
            o = tile.observations[idx]
            code[sel] = np.where(o > 0, tile.code[idx], C.INSUFFICIENT)
            height[sel], obs[sel], seen[sel] = tile.display_h[idx], o, tile.last_seen[idx]
        return code, height, obs, seen
