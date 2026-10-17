"""L515 terrain map reception and rendering (spec §4.9, §4.10).

Pure numpy + standard library — no GTK — so ordering, TTL and the raster are
testable without a display; ``MapPanel`` in app.py only draws what
``map_rgba`` returns.  Operator visual aid only: nothing here feeds control.
"""
from __future__ import annotations

import base64
from collections import OrderedDict, deque
from dataclasses import dataclass, field
import math
import socket
import threading
import time

import numpy as np

from terrain_map import contract as C

from .session_gate import NEW, REJECT, SessionSelector

DEFAULT_FLOOR_CAPACITY = 4 * 1024           # 4 x the sender's default max_map_tiles (spec §4.9)
TRAJECTORY_MAX = 20000
TRAJECTORY_STEP_M = 0.02
STALE_DISPLAY_RANGE_S = (5.0, 60.0)


def tile_ttl_s(refresh_cycle_s: float) -> float:
    """A tile not refreshed for this long leaves the screen (3 refresh cycles + 2 s)."""
    return 3.0 * refresh_cycle_s + 2.0


@dataclass
class StoredTile:
    tx: int
    ty: int
    version: int
    tile_seq: int
    side: int
    heights: np.ndarray        # int16 cm, -128 = unobserved; row-major, row = gy, column = gx
    codes: np.ndarray          # uint8
    ages: np.ndarray           # uint8 age steps at send time
    received_s: float


def _decode(packet: C.TilePacket, received_s: float) -> StoredTile:
    raw = np.frombuffer(base64.b64decode(packet.cells), np.uint8)
    side = math.isqrt(len(raw) // 2)
    pairs = raw.reshape(-1, 2)
    return StoredTile(packet.tx, packet.ty, packet.version, packet.tile_seq, side,
                      pairs[:, 0].view(np.int8).astype(np.int16), (pairs[:, 1] >> 5).astype(np.uint8),
                      (pairs[:, 1] & 31).astype(np.uint8), received_s)


@dataclass(frozen=True)
class MapSnapshot:
    status: C.StatusPacket | None
    epoch: int | None
    session: tuple | None
    tiles: tuple[StoredTile, ...]
    last_rx_s: float | None
    trajectory: tuple[tuple[float, float], ...]
    invalid_packets: int
    rejected_packets: int


class LatestMapReceiver:
    """Daemon UDP receiver for :5010 keeping the current session/epoch's map.

    Ordering floors (status ``sequence``, per-tile ``(version, tile_seq)``) are kept apart
    from what is on screen: TTL removes tiles from the screen, never their floors, so a
    delayed copy cannot bring a faded tile back (spec §4.9, Codex r5 P2-3).
    """

    def __init__(self, port: int, *, floor_capacity: int = DEFAULT_FLOOR_CAPACITY) -> None:
        self._lock = threading.Lock()
        self._selector = SessionSelector()
        self._floor_capacity = int(floor_capacity)
        self.invalid_packet_count = 0
        self.rejected_packet_count = 0
        self.accepted_packet_count = 0
        self.last_address: str | None = None
        self._reset_session()
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", port))
        self._stopping = threading.Event()
        self._thread = threading.Thread(target=self._run, name="l515-terrain-map", daemon=True)
        self._thread.start()

    @property
    def port(self) -> int:
        return self._socket.getsockname()[1]

    @property
    def evicted_seq_floor(self) -> int:
        return self._evicted_seq_floor

    def _reset_session(self) -> None:
        self._epoch: int | None = None
        self._reset_epoch()
        self._last_rx_s: float | None = None

    def _reset_epoch(self) -> None:
        self._status: C.StatusPacket | None = None
        self._status_floor: int | None = None
        self._tiles: dict[tuple[int, int], StoredTile] = {}
        self._floors: OrderedDict[tuple[int, int], tuple[int, int]] = OrderedDict()
        self._evicted_seq_floor = 0
        self._trajectory: deque[tuple[float, float]] = deque(maxlen=TRAJECTORY_MAX)

    def _run(self) -> None:
        self._socket.settimeout(0.2)
        while not self._stopping.is_set():
            try:
                raw, address = self._socket.recvfrom(C.MAX_PACKET_BYTES + 1)
            except OSError:
                continue
            try:
                if self.ingest(raw, time.monotonic()):
                    self.last_address = f"{address[0]}:{address[1]}"
            except Exception:  # noqa: BLE001 — never let one datagram kill reception
                self.invalid_packet_count += 1

    def ingest(self, raw: bytes, received_s: float) -> bool:
        try:
            packet = C.parse_packet(raw, received_s)
        except ValueError:
            self.invalid_packet_count += 1
            return False
        with self._lock:
            accepted = self._accept(packet, received_s)
            if accepted:
                self.accepted_packet_count += 1
            else:
                self.rejected_packet_count += 1
            return accepted

    def _accept(self, packet, now_s: float) -> bool:
        key = packet.session_key
        decision = self._selector.classify(key, now_s)
        if decision == REJECT:
            return False
        if decision == NEW:
            self._reset_session()
        if self._epoch is None or packet.map_epoch > self._epoch:
            self._epoch = packet.map_epoch
            self._reset_epoch()
        elif packet.map_epoch < self._epoch:
            return False                               # a delayed packet of an older epoch
        if isinstance(packet, C.StatusPacket):
            if self._status_floor is not None and packet.sequence <= self._status_floor:
                return False
            self._status_floor = packet.sequence
            self._status = packet
            if packet.pose is not None:
                p = (packet.pose.x_m, packet.pose.y_m)
                last = self._trajectory[-1] if self._trajectory else None
                if last is None or math.hypot(p[0] - last[0], p[1] - last[1]) >= TRAJECTORY_STEP_M:
                    self._trajectory.append(p)
        elif not self._offer(packet, now_s):
            return False
        self._selector.commit(key, now_s)
        self._last_rx_s = now_s
        return True

    def _offer(self, packet: C.TilePacket, now_s: float) -> bool:
        key = (packet.tx, packet.ty)
        floor = self._floors.get(key)
        if floor is None:
            if packet.tile_seq <= self._evicted_seq_floor:
                return False
        else:
            version, seq = floor
            if packet.version < version or (packet.version == version and packet.tile_seq <= seq):
                return False
        self._tiles[key] = _decode(packet, now_s)
        self._floors[key] = (packet.version, packet.tile_seq)
        self._floors.move_to_end(key)
        while len(self._floors) > self._floor_capacity:
            _, (_, seq) = self._floors.popitem(last=False)
            self._evicted_seq_floor = max(self._evicted_seq_floor, seq)
        return True

    def clear_display(self) -> None:
        """Console-only "clear from screen": the next refresh cycle fills it again."""
        with self._lock:
            self._tiles.clear()
            self._trajectory.clear()

    def snapshot(self, now_s: float) -> MapSnapshot:
        """Status, live tiles and receive time read under one lock (TTL applied to the screen data)."""
        with self._lock:
            if self._status is not None:
                ttl = tile_ttl_s(self._status.refresh_cycle_s)
                for key in [k for k, t in self._tiles.items() if now_s - t.received_s > ttl]:
                    del self._tiles[key]
            return MapSnapshot(self._status, self._epoch, self._selector.session, tuple(self._tiles.values()),
                               self._last_rx_s,
                               tuple(self._trajectory), self.invalid_packet_count, self.rejected_packet_count)

    def close(self) -> None:
        self._stopping.set()
        self._socket.close()
        self._thread.join(timeout=1.0)


# --- rendering ---------------------------------------------------------------------
@dataclass(frozen=True)
class DisplayConfig:
    """Display-only numbers (spec §4.10). All changeable; ranges are enforced."""
    display_upsample: int = 4
    max_render_pixels: int = 1_000_000
    stale_display_s: float = 30.0
    hillshade_azimuth_deg: float = 315.0
    hillshade_altitude_deg: float = 45.0
    z_exaggeration: float = 3.0
    height_range_m: float = 0.3

    def __post_init__(self) -> None:
        checks = {
            "display_upsample": (1, 8), "max_render_pixels": (10_000, 4_000_000),
            "stale_display_s": STALE_DISPLAY_RANGE_S, "hillshade_azimuth_deg": (0.0, 360.0),
            "hillshade_altitude_deg": (5.0, 90.0), "z_exaggeration": (0.5, 20.0), "height_range_m": (0.05, 1.0),
        }
        for name, (low, high) in checks.items():
            value = getattr(self, name)
            if not (isinstance(value, (int, float)) and math.isfinite(value) and low <= value <= high):
                raise ValueError(f"{name} must be within {low}-{high}")


@dataclass(frozen=True)
class MapView:
    """World window (metres, x forward/east, y left/north) and the screen size it fills."""
    x0: float
    y0: float
    width_m: float
    height_m: float
    width_px: int
    height_px: int


_LOW = np.array([88, 104, 124], np.float32)
_MID = np.array([150, 152, 140], np.float32)
_HIGH = np.array([210, 190, 150], np.float32)
_RED = np.array([226, 46, 46], np.float32)
_BLUE = np.array([40, 96, 236], np.float32)
_GRAY = np.array([128, 128, 128], np.float32)
_OUTLINE = np.array([24, 24, 24], np.float32)


@dataclass
class _Layers:
    rgb: np.ndarray            # float32 (H, W, 3), age-independent
    alpha: np.ndarray          # uint8 (H, W)
    height_m: np.ndarray       # float32 (H, W), NaN unobserved


@dataclass
class _Grid:
    gx0: int
    gy1: int                   # one past the top row's gy (north-up raster)
    nx: int
    ny: int
    scale: int
    tiles: list = field(default_factory=list)


def _grid(snapshot: MapSnapshot, view: MapView, display: DisplayConfig) -> _Grid | None:
    status = snapshot.status
    cell = status.cell_m
    gx0, gx1 = math.floor(view.x0 / cell), math.ceil((view.x0 + view.width_m) / cell)
    gy0, gy1 = math.floor(view.y0 / cell), math.ceil((view.y0 + view.height_m) / cell)
    nx, ny = max(1, gx1 - gx0), max(1, gy1 - gy0)
    if nx * ny > display.max_render_pixels:
        return None
    scale = min(display.display_upsample, max(1, math.ceil(view.width_px / nx)))
    while scale > 1 and nx * ny * scale * scale > display.max_render_pixels:
        scale -= 1
    n = status.tile_cells
    visible = [t for t in snapshot.tiles
               if t.side == n and t.tx * n < gx1 and (t.tx + 1) * n > gx0 and t.ty * n < gy1 and (t.ty + 1) * n > gy0]
    return _Grid(gx0, gy1, nx, ny, scale, visible)


def _cells(grid: _Grid, n: int, pick, fill, dtype) -> np.ndarray:
    """(ny, nx) array; ``pick(tile) -> (n, n) array`` in tile rows = gy; ``fill`` where no tile is."""
    out = np.full((grid.ny, grid.nx), fill, dtype)
    for t in grid.tiles:
        block = pick(t)                                          # rows gy = ty*n .. +n-1
        gx_lo, gy_lo = t.tx * n, t.ty * n
        x0, x1 = max(gx_lo, grid.gx0), min(gx_lo + n, grid.gx0 + grid.nx)
        y0, y1 = max(gy_lo, grid.gy1 - grid.ny), min(gy_lo + n, grid.gy1)
        if x0 >= x1 or y0 >= y1:
            continue
        sub = block[y0 - gy_lo:y1 - gy_lo, x0 - gx_lo:x1 - gx_lo][::-1]      # flip: north up
        out[grid.gy1 - y1:grid.gy1 - y0, x0 - grid.gx0:x1 - grid.gx0] = sub
    return out


def _fill_holes(h: np.ndarray, passes: int = 2) -> np.ndarray:
    """Give unobserved cells their observed neighbours' mean so bilinear edges stay in range."""
    h = h.copy()
    for _ in range(passes):
        valid = np.isfinite(h)
        if valid.all():
            break
        padded = np.pad(np.where(valid, h, 0.0), 1)
        count = np.pad(valid.astype(np.float32), 1)
        total = sum(padded[1 + dy:padded.shape[0] - 1 + dy, 1 + dx:padded.shape[1] - 1 + dx]
                    for dy in (-1, 0, 1) for dx in (-1, 0, 1))
        number = sum(count[1 + dy:count.shape[0] - 1 + dy, 1 + dx:count.shape[1] - 1 + dx]
                     for dy in (-1, 0, 1) for dx in (-1, 0, 1))
        fill = ~valid & (number > 0)
        h[fill] = total[fill] / number[fill]
    return np.where(np.isfinite(h), h, 0.0)


def _box3(a: np.ndarray) -> np.ndarray:
    padded = np.pad(a, 1, mode="edge")
    return sum(padded[1 + dy:padded.shape[0] - 1 + dy, 1 + dx:padded.shape[1] - 1 + dx]
               for dy in (-1, 0, 1) for dx in (-1, 0, 1)) / 9.0


def _bilinear(a: np.ndarray, s: int) -> np.ndarray:
    """Upsample by ``s`` between cell centres (convex weights: no overshoot)."""
    def axis(n):
        u = np.clip((np.arange(n * s) + 0.5) / s - 0.5, 0.0, n - 1)
        i0 = np.floor(u).astype(int)
        return i0, np.minimum(i0 + 1, n - 1), (u - i0).astype(np.float32)
    i0, i1, w = axis(a.shape[1])
    x = a[:, i0] * (1.0 - w) + a[:, i1] * w
    j0, j1, v = axis(a.shape[0])
    return x[j0] * (1.0 - v)[:, None] + x[j1] * v[:, None]


def _nearest(a: np.ndarray, s: int) -> np.ndarray:
    return np.repeat(np.repeat(a, s, axis=0), s, axis=1)


def _static_layers(grid: _Grid, n: int, cell_m: float, display: DisplayConfig) -> _Layers:
    s = grid.scale
    heights = _cells(grid, n, lambda t: t.heights.reshape(n, n), C.UNOBSERVED_H, np.int16)
    codes = _cells(grid, n, lambda t: t.codes.reshape(n, n), C.INSUFFICIENT, np.uint8)
    observed = heights != C.UNOBSERVED_H
    h = np.where(observed, heights.astype(np.float32) / 100.0, np.nan)
    filled = _fill_holes(h).astype(np.float32)
    up = _bilinear(filled, s)
    # Shade a lightly smoothed surface: heights arrive in whole centimetres, and shading the
    # raw steps draws contour-like bands across gentle slopes.
    smooth = _bilinear(_box3(filled), s)
    seen = _nearest(observed, s)
    code_up = _nearest(codes, s)
    # hillshade on the exaggerated surface (north-up raster: row index grows southward)
    pixel = cell_m / s
    dz_row, dz_col = np.gradient(smooth * display.z_exaggeration, pixel)
    slope = np.arctan(np.hypot(dz_col, dz_row))
    aspect = np.arctan2(-dz_row, -dz_col)
    zenith = math.radians(90.0 - display.hillshade_altitude_deg)
    azimuth = math.radians(90.0 - display.hillshade_azimuth_deg)
    shade = np.cos(zenith) * np.cos(slope) + np.sin(zenith) * np.sin(slope) * np.cos(azimuth - aspect)
    shade = (0.35 + 0.65 * np.clip(shade, 0.0, 1.0))[..., None]
    t = np.clip((up + display.height_range_m) / (2.0 * display.height_range_m), 0.0, 1.0)[..., None]
    base = np.where(t < 0.5, _LOW + (_MID - _LOW) * (t * 2.0), _MID + (_HIGH - _MID) * ((t - 0.5) * 2.0))
    rgb = base * shade
    # classes at the raw cell boundaries (nearest), outlined (agy 11)
    for code, colour, weight in ((C.OBSTACLE, _RED, 0.65), (C.DITCH, _BLUE, 0.65), (C.INSUFFICIENT, _GRAY, 0.55)):
        mask = (code_up == code) & seen
        rgb[mask] = rgb[mask] * (1.0 - weight) + colour * weight
    hazard = np.isin(code_up, (C.OBSTACLE, C.DITCH)) & seen
    edge = np.zeros_like(hazard)
    change_x = (code_up[:, 1:] != code_up[:, :-1]) & (hazard[:, 1:] | hazard[:, :-1])
    change_y = (code_up[1:, :] != code_up[:-1, :]) & (hazard[1:, :] | hazard[:-1, :])
    edge[:, 1:] |= change_x
    edge[:, :-1] |= change_x
    edge[1:, :] |= change_y
    edge[:-1, :] |= change_y
    rgb[edge & seen] = _OUTLINE
    return _Layers(rgb.astype(np.float32), np.where(seen, 255, 0).astype(np.uint8),
                   np.where(seen, up, np.nan).astype(np.float32))


def map_rgba(snapshot: MapSnapshot, view: MapView, display: DisplayConfig, now_s: float, cache: dict | None = None):
    """``(rgba uint8 (H, W, 4), meta)`` of the visible cells, north up (row 0 = largest y).

    Only time-independent layers (interpolation, shading, class colours) are cached; the
    age fade is applied on every call (spec §4.10, Codex plan r2 P1-2).
    """
    status = snapshot.status
    if status is None:
        return np.zeros((1, 1, 4), np.uint8), {"drawn": False, "cache_hit": False}
    grid = _grid(snapshot, view, display)
    if grid is None:
        return np.zeros((1, 1, 4), np.uint8), {"drawn": False, "cache_hit": False, "reason": "view too large"}
    n, cell = status.tile_cells, status.cell_m
    key = (snapshot.session, snapshot.epoch, cell, n, grid.gx0, grid.gy1, grid.nx, grid.ny, grid.scale, display,
           tuple(sorted((t.tx, t.ty, t.version) for t in grid.tiles)))
    hit = cache is not None and cache.get("key") == key
    if hit:
        layers = cache["layers"]
    else:
        layers = _static_layers(grid, n, cell, display)
        if cache is not None:
            cache.clear()
            cache.update(key=key, layers=layers)
    # age, per draw: cell age = age bits x age_step + time since the tile arrived
    age = _cells(grid, n, lambda t: (t.ages.reshape(n, n).astype(np.float32) * status.age_step_s
                                     + np.float32(now_s - t.received_s)), np.nan, np.float32) if grid.tiles else None
    rgb = layers.rgb
    stale_cells = 0
    if age is not None:
        stale = np.isfinite(age) & (age > display.stale_display_s)
        stale_cells = int(stale.sum())
        if stale_cells:
            mask = _nearest(stale, grid.scale)
            gray = (rgb[..., 0] * 0.299 + rgb[..., 1] * 0.587 + rgb[..., 2] * 0.114)[..., None]
            rgb = np.where(mask[..., None], rgb * 0.3 + gray * 0.7, rgb)
    rgba = np.dstack([np.clip(rgb, 0, 255).astype(np.uint8), layers.alpha])
    meta = {
        "drawn": True, "cache_hit": hit, "pixels_per_cell": grid.scale, "m_per_px": cell / grid.scale,
        "origin_x_m": grid.gx0 * cell, "top_y_m": grid.gy1 * cell, "height_m": layers.height_m,
        "stale_cells": stale_cells, "tiles": len(grid.tiles),
    }
    return rgba, meta


class MapRenderer:
    """``map_rgba`` with its static-layer cache (one per panel)."""

    def __init__(self) -> None:
        self._cache: dict = {}

    def render(self, snapshot: MapSnapshot, view: MapView, display: DisplayConfig, now_s: float):
        return map_rgba(snapshot, view, display, now_s, cache=self._cache)


__all__ = ["DisplayConfig", "LatestMapReceiver", "MapRenderer", "MapSnapshot", "MapView", "StoredTile",
           "map_rgba", "tile_ttl_s"]
