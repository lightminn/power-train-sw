"""Terrain map reception and rendering (spec §4.9, §4.10) — system python, no GTK."""
import time

import numpy as np
import pytest

from operator_console.terrain_map import (
    DisplayConfig, LatestMapReceiver, MapRenderer, MapView, map_rgba, tile_ttl_s,
)
from terrain_map import contract as C

N = 16
SESSION = dict(sender_boot_id="a" * 32, sender_start_mono_ns=5, sender_instance="b" * 16)


def status(seq, epoch=0, refresh=2.0, tiles=1, pose=(0.0, 0.0, 0.0), **kw):
    fields = dict(SESSION, map_epoch=epoch, sent_unix_ns=1, sequence=seq, status="OK", reason=None,
                  cell_m=0.05, tile_cells=N, age_step_s=2.0,
                  pose=None if pose is None else C.Pose(*pose), distance_m=0.0, pose_gaps=0, not_support=0,
                  late_dropped=0, tile_count=tiles, refresh_cycle_s=refresh,
                  thresholds=C.Thresholds(0.08, 0.08, 2), corrector=C.CorrectorInfo("identity", False, None, "identity"),
                  proc_ms=None)
    fields.update(kw)
    return C.encode(C.StatusPacket(**fields))


def cells(height_cm=0, code=C.FREE, age=0, fill=None):
    n = N * N
    heights = [height_cm] * n if fill is None else fill[0]
    codes = [code] * n if fill is None else fill[1]
    return C.pack_cells(heights, codes, [age] * n)


def tile(tx, ty, version, tile_seq, epoch=0, **kw):
    return C.encode(C.TilePacket(**SESSION, map_epoch=epoch, sent_unix_ns=1, tx=tx, ty=ty, version=version,
                                 tile_seq=tile_seq, cells=kw.pop("payload", None) or cells(**kw)))


@pytest.fixture
def rx():
    r = LatestMapReceiver(0)
    yield r
    r.close()


def stored(rx, now=100.0):
    return {(t.tx, t.ty): (t.version, t.tile_seq) for t in rx.snapshot(now).tiles}


# --- ordering ---------------------------------------------------------------------
def test_epoch_rules_reject_old_and_switch_on_new_even_from_a_tile(rx):
    assert rx.ingest(status(1, epoch=2), 100.0)
    assert rx.ingest(tile(0, 0, 1, 1, epoch=2), 100.0)
    assert not rx.ingest(tile(1, 0, 1, 2, epoch=1), 100.1)            # delayed old epoch
    assert not rx.ingest(status(2, epoch=1), 100.1)
    assert stored(rx) == {(0, 0): (1, 1)}
    assert rx.ingest(tile(5, 5, 1, 1, epoch=3), 100.2)                # a tile announces epoch 3
    snap = rx.snapshot(100.2)
    assert snap.epoch == 3 and snap.status is None and stored(rx) == {(5, 5): (1, 1)}
    assert rx.ingest(status(1, epoch=3), 100.3)                        # status floor was reset


def test_same_version_newer_tile_seq_refreshes_ttl_and_ages(rx):
    rx.ingest(status(1, refresh=26.0), 0.0)
    rx.ingest(tile(0, 0, 4, 10, age=0), 0.0)
    ttl = tile_ttl_s(26.0)
    assert ttl == pytest.approx(80.0)
    assert rx.ingest(tile(0, 0, 4, 11, age=15), 70.0)                  # refresh: same version, newer seq
    snap = rx.snapshot(149.0)                                          # 79 s after the refresh
    assert [(t.version, t.tile_seq) for t in snap.tiles] == [(4, 11)]
    assert snap.tiles[0].ages[0] == 15


def test_late_reversed_same_version_and_smaller_versions_are_ignored(rx):
    rx.ingest(status(1, refresh=26.0), 0.0)
    rx.ingest(tile(0, 0, 4, 11, age=15), 10.0)
    assert not rx.ingest(tile(0, 0, 4, 10, age=0), 11.0)               # older copy of the same version
    assert not rx.ingest(tile(0, 0, 3, 12), 12.0)                      # smaller version: no TTL extension
    assert rx.snapshot(10.0 + tile_ttl_s(26.0) + 0.5).tiles == ()


def test_after_ttl_a_reversed_copy_cannot_resurrect_the_tile(rx):
    rx.ingest(status(1, refresh=2.0), 0.0)
    rx.ingest(tile(0, 0, 4, 11), 0.0)
    assert rx.snapshot(20.0).tiles == ()                               # TTL 3*2+2 = 8 s
    assert not rx.ingest(tile(0, 0, 4, 10), 20.5)
    assert rx.ingest(tile(0, 0, 4, 12), 21.0)


def test_floor_eviction_keeps_old_packets_out_and_new_ones_in():
    r = LatestMapReceiver(0, floor_capacity=4)
    try:
        r.ingest(status(1), 0.0)
        for k in range(6):                                             # 6 tiles, capacity 4
            assert r.ingest(tile(k, 0, 1, k + 1), 0.1 * k)
        assert r.evicted_seq_floor == 2
        assert not r.ingest(tile(0, 0, 1, 1), 1.0)                     # old packet of an evicted floor
        assert r.ingest(tile(0, 0, 2, 7), 1.1)                         # new one is welcome
    finally:
        r.close()


def test_tiles_before_status_are_kept_and_drawn_once_status_arrives(rx):
    rx.ingest(tile(0, 0, 1, 1, height_cm=5), 0.0)
    snap = rx.snapshot(0.1)
    assert snap.status is None and len(snap.tiles) == 1
    rgba, meta = map_rgba(snap, MapView(0.0, 0.0, 0.8, 0.8, 400, 400), DisplayConfig(), 0.1)
    assert meta["drawn"] is False and not rgba[..., 3].any()
    rx.ingest(status(1), 0.2)
    rgba, meta = map_rgba(rx.snapshot(0.3), MapView(0.0, 0.0, 0.8, 0.8, 400, 400), DisplayConfig(), 0.3)
    assert meta["drawn"] and rgba[..., 3].any()


def test_insufficient_release_out_of_order(rx):
    rx.ingest(status(1), 0.0)
    rx.ingest(tile(0, 0, 2, 5, code=C.FREE), 0.0)                      # released (4 -> 0)
    assert not rx.ingest(tile(0, 0, 1, 4, code=C.INSUFFICIENT), 0.1)   # the older, insufficient one arrives late
    assert rx.snapshot(0.2).tiles[0].codes[0] == C.FREE


def test_shuffled_and_lossy_delivery_converges():
    rng = np.random.default_rng(3)
    r = LatestMapReceiver(0, floor_capacity=4096)
    try:
        packets = [status(s + 1, tiles=1024) for s in range(30)]
        seq = 0
        final = {}
        for v in (1, 2):
            for k in range(1024):
                seq += 1
                packets.append(tile(k % 32, k // 32, v, seq, height_cm=v))
                final[(k % 32, k // 32)] = (v, seq)
        order = rng.permutation(len(packets))
        for i in order:
            if rng.random() < 0.1:
                continue                                               # lost
            r.ingest(packets[i], 0.0)
        for (tx, ty), (v, s) in final.items():                         # one refresh cycle re-sends everything
            seq += 1
            r.ingest(tile(tx, ty, v, seq, height_cm=v), 1.0)
        snap = r.snapshot(1.0)
        assert snap.status.sequence == 30 and len(snap.tiles) == 1024
        assert all(t.version == 2 and t.heights[0] == 2 for t in snap.tiles)
    finally:
        r.close()


def test_session_switch_clears_the_map(rx):
    rx.ingest(status(1), 0.0)
    rx.ingest(tile(0, 0, 1, 1), 0.0)
    newer = dict(SESSION, sender_start_mono_ns=6)
    raw = C.encode(C.TilePacket(**newer, map_epoch=0, sent_unix_ns=1, tx=3, ty=3, version=1, tile_seq=1,
                                cells=cells()))
    assert rx.ingest(raw, 0.1)
    assert stored(rx, 0.1) == {(3, 3): (1, 1)}


def test_age_ranges_refuse_to_start():
    from terrain_map.config import MapConfig
    with pytest.raises(ValueError):
        MapConfig(age_step_s=1.9)
    with pytest.raises(ValueError):
        DisplayConfig(stale_display_s=61.0)
    with pytest.raises(ValueError):
        DisplayConfig(stale_display_s=4.0)


# --- rendering --------------------------------------------------------------------
def one_tile_snapshot(fill, age=0, received=0.0):
    r = LatestMapReceiver(0)
    try:
        r.ingest(status(1, refresh=26.0), received)
        heights, codes = fill
        r.ingest(tile(0, 0, 1, 1, payload=C.pack_cells(heights, codes, [age] * (N * N))), received)
        return r.snapshot(received)
    finally:
        r.close()


def split_fill():
    """Left half (gx < 8) obstacle 12 cm, right half free 0 cm; top row unobserved."""
    heights, codes = [], []
    for gy in range(N):
        for gx in range(N):
            if gy == N - 1:
                heights.append(None)
                codes.append(C.INSUFFICIENT)
            elif gx < 8:
                heights.append(12)
                codes.append(C.OBSTACLE)
            else:
                heights.append(0)
                codes.append(C.FREE)
    return heights, codes


VIEW = MapView(0.0, 0.0, 0.8, 0.8, 320, 320)                         # one 16-cell tile, 0.8 m


def test_class_colour_changes_exactly_at_the_raw_cell_boundary():
    snap = one_tile_snapshot(split_fill())
    rgba, meta = map_rgba(snap, VIEW, DisplayConfig(display_upsample=4), 0.0)
    s = meta["pixels_per_cell"]
    row = rgba[rgba.shape[0] // 2]                                     # a middle row (gy = 7)
    left, right = row[8 * s - 2], row[8 * s + 1]
    assert left[0] > left[2] and right[0] < 200                        # red side vs not red
    border = row[8 * s - 1: 8 * s + 1]
    assert (border[:, :3] < 80).all()                                  # outline drawn on the boundary


def test_bilinear_heights_never_overshoot_and_unobserved_is_transparent():
    snap = one_tile_snapshot(split_fill())
    rgba, meta = map_rgba(snap, VIEW, DisplayConfig(display_upsample=4), 0.0)
    h = meta["height_m"]
    finite = h[np.isfinite(h)]
    assert finite.min() >= 0.0 - 1e-9 and finite.max() <= 0.12 + 1e-9
    s = meta["pixels_per_cell"]
    top = rgba[: s]                                                    # rows of gy = 15 (north is up)
    assert (top[..., 3] == 0).all()


def test_old_cells_are_desaturated():
    fresh = one_tile_snapshot(split_fill(), age=0)
    old = one_tile_snapshot(split_fill(), age=16)                       # 32 s > 30 s
    a, _ = map_rgba(fresh, VIEW, DisplayConfig(), 0.0)
    b, _ = map_rgba(old, VIEW, DisplayConfig(), 0.0)
    sat = lambda px: px[..., :3].max(-1).astype(int) - px[..., :3].min(-1).astype(int)   # noqa: E731
    mask = a[..., 3] > 0
    assert sat(b)[mask].mean() < 0.6 * sat(a)[mask].mean()


def test_pixel_cap_limits_the_raster():
    snap = one_tile_snapshot(split_fill())
    rgba, meta = map_rgba(snap, VIEW, DisplayConfig(display_upsample=8, max_render_pixels=10_000), 0.0)
    assert rgba.shape[0] * rgba.shape[1] <= 10_000


def test_cache_keeps_static_layers_but_ages_on_every_draw():
    """Age 0 tile, packets stop, view fixed, clock +31 s: the cells must fade (cache on)."""
    r = LatestMapReceiver(0)
    try:
        r.ingest(status(1, refresh=26.0), 0.0)
        r.ingest(tile(0, 0, 1, 1, payload=C.pack_cells(*split_fill(), [0] * (N * N))), 0.0)
        renderer = MapRenderer()
        first, meta1 = renderer.render(r.snapshot(0.0), VIEW, DisplayConfig(), 0.0)
        second, meta2 = renderer.render(r.snapshot(31.0), VIEW, DisplayConfig(), 31.0)
        assert meta2["cache_hit"] and not meta1["cache_hit"]
        mask = first[..., 3] > 0
        sat = lambda px: px[..., :3].max(-1).astype(int) - px[..., :3].min(-1).astype(int)   # noqa: E731
        assert sat(second)[mask].mean() < 0.6 * sat(first)[mask].mean()
    finally:
        r.close()


def test_render_speed_with_1024_tiles_on_a_large_view():
    r = LatestMapReceiver(0, floor_capacity=4096)
    try:
        r.ingest(status(1, tiles=1024), 0.0)
        rng = np.random.default_rng(1)
        seq = 0
        for k in range(1024):
            seq += 1
            heights = [int(v) for v in rng.integers(-10, 10, N * N)]
            codes = [int(c) for c in rng.choice([0, 0, 0, 1, 2], N * N)]
            r.ingest(tile(k % 32, k // 32, 1, seq, payload=C.pack_cells(heights, codes, [0] * (N * N))), 0.0)
        snap = r.snapshot(0.0)
        view = MapView(0.0, 0.0, 25.6, 16.0, 1600, 1000)
        renderer = MapRenderer()
        renderer.render(snap, view, DisplayConfig(), 0.0)              # first draw fills the cache
        started = time.perf_counter()
        _, meta = renderer.render(snap, view, DisplayConfig(), 0.5)
        assert time.perf_counter() - started < 0.05 and meta["cache_hit"]
        started = time.perf_counter()
        map_rgba(snap, view, DisplayConfig(), 0.5)                     # uncached full computation
        assert time.perf_counter() - started < 0.5
    finally:
        r.close()


def test_cells_without_any_tile_are_transparent():
    """A view larger than the received tiles: the rest is unobserved, not flat ground."""
    snap = one_tile_snapshot(split_fill())
    rgba, meta = map_rgba(snap, MapView(0.0, 0.0, 1.6, 1.6, 640, 640), DisplayConfig(display_upsample=4), 0.0)
    s = meta["pixels_per_cell"]
    half = 16 * s
    assert (rgba[:half, :, 3] == 0).all()                      # north half: no tile
    assert (rgba[half:, half:, 3] == 0).all()                  # east of the tile
    assert rgba[half:, :half, 3].any()


def test_render_cache_tells_sender_sessions_apart():
    """Codex T8 P2: a restarted sender reuses epoch 0 / version 1 with other content."""
    r = LatestMapReceiver(0)
    try:
        renderer = MapRenderer()
        r.ingest(status(1), 0.0)
        r.ingest(tile(0, 0, 1, 1, height_cm=0, code=C.FREE), 0.0)
        flat, _ = renderer.render(r.snapshot(0.0), VIEW, DisplayConfig(), 0.0)
        other = dict(SESSION, sender_start_mono_ns=99)
        r.ingest(C.encode(C.StatusPacket(**{**C.parse_packet(status(1)).__dict__, **other, "received_s": None})), 0.1)
        r.ingest(C.encode(C.TilePacket(**other, map_epoch=0, sent_unix_ns=1, tx=0, ty=0, version=1, tile_seq=1,
                                       cells=cells(height_cm=20, code=C.OBSTACLE))), 0.1)
        box, meta = renderer.render(r.snapshot(0.1), VIEW, DisplayConfig(), 0.1)
        assert not meta["cache_hit"] and not np.array_equal(flat, box)
        assert (box[..., 0] > box[..., 2]).mean() > 0.5                   # now drawn red
    finally:
        r.close()


@pytest.mark.parametrize("bad", [dict(display_upsample=1.5), dict(display_upsample=True),
                                 dict(max_render_pixels=500_000.5)])
def test_integer_display_fields_reject_other_types(bad):
    with pytest.raises(ValueError):
        DisplayConfig(**bad)


def test_display_defaults_and_ranges_follow_the_spec_table():
    """Spec §4.1 console row: 4·315·40·6·30·2,000,000 and ranges 1–8·0–360·5–85·1–20·5–60."""
    d = DisplayConfig()
    assert (d.display_upsample, d.hillshade_azimuth_deg, d.hillshade_altitude_deg, d.z_exaggeration,
            d.stale_display_s, d.max_render_pixels) == (4, 315.0, 40.0, 6.0, 30.0, 2_000_000)
    for bad in (dict(hillshade_altitude_deg=86.0), dict(z_exaggeration=0.9)):
        with pytest.raises(ValueError):
            DisplayConfig(**bad)


def test_the_readme_json_example_starts_the_console_config():
    import json
    import re
    from pathlib import Path
    text = (Path(__file__).resolve().parents[2] / "terrain_map/README.md").read_text()
    block = re.search(r"```json\n(.*?)```", text, re.S).group(1)
    DisplayConfig(**json.loads(block)["terrain_map_display"])


# --- display tidy (spec §4.10-6, r12) ------------------------------------------------
def sparse_fill(cells_at):
    """Only the listed ``{(gx, gy): (height_cm, code)}`` cells are observed."""
    heights, codes = [None] * (N * N), [C.INSUFFICIENT] * (N * N)
    for (gx, gy), (h, code) in cells_at.items():
        heights[gy * N + gx], codes[gy * N + gx] = h, code
    return heights, codes


def tidy_snapshot(tiles, thresholds=(0.08, 0.08, 2)):
    """``tiles = {(tx, ty): fill}`` in one receiver; returns its snapshot."""
    r = LatestMapReceiver(0)
    try:
        r.ingest(status(1, refresh=26.0, thresholds=C.Thresholds(*thresholds)), 0.0)
        for k, ((tx, ty), (heights, codes)) in enumerate(tiles.items()):
            r.ingest(tile(tx, ty, 1, k + 1, payload=C.pack_cells(heights, codes, [0] * (N * N))), 0.0)
        return r.snapshot(0.0)
    finally:
        r.close()


def cell_alpha(rgba, meta, gx, gy, view_y0=0.0, view_x0=0.0):
    """Alpha at the centre of raw cell (gx, gy) (cell 5 cm, north-up raster)."""
    s = meta["pixels_per_cell"]
    col = int(round((gx * 0.05 - meta["origin_x_m"]) / 0.05)) * s + s // 2
    row = int(round((meta["top_y_m"] - (gy + 1) * 0.05) / 0.05)) * s + s // 2
    return int(rgba[row, col, 3])


def block(x0, y0, w, h, height_cm, code):
    return {(gx, gy): (height_cm, code) for gx in range(x0, x0 + w) for gy in range(y0, y0 + h)}


def test_isolated_low_cells_are_hidden_but_tall_and_hazard_cells_never():
    fill = sparse_fill({(2, 2): (0, C.FREE), (5, 5): (1, C.INSUFFICIENT),       # isolated, low -> hidden
                        (8, 8): (12, C.INSUFFICIENT), (11, 11): (-2, C.DITCH),   # tall / hazard -> kept
                        (13, 2): (1, C.OBSTACLE), (2, 13): (-5, C.FREE),
                        **block(9, 1, 3, 3, 0, C.FREE)})                         # a 3x3 patch -> kept
    rgba, meta = map_rgba(tidy_snapshot({(0, 0): fill}), VIEW, DisplayConfig(display_upsample=4), 0.0)
    assert cell_alpha(rgba, meta, 2, 2) == 0 and cell_alpha(rgba, meta, 5, 5) == 0
    for gx, gy in ((8, 8), (11, 11), (13, 2), (2, 13), (10, 2), (9, 1)):
        assert cell_alpha(rgba, meta, gx, gy) > 0, (gx, gy)
    assert meta["hidden_cells"] == 2


def test_low_once_seen_cells_are_translucent_and_tall_once_seen_cells_are_not():
    fill = sparse_fill({**block(1, 1, 4, 4, 0, C.INSUFFICIENT), **block(8, 8, 4, 4, 12, C.INSUFFICIENT),
                        **block(1, 8, 4, 4, 0, C.FREE)})
    display = DisplayConfig(display_upsample=4)
    rgba, meta = map_rgba(tidy_snapshot({(0, 0): fill}), VIEW, display, 0.0)
    assert cell_alpha(rgba, meta, 2, 2) == round(255 * display.weak_alpha)
    assert cell_alpha(rgba, meta, 9, 9) == 255 and cell_alpha(rgba, meta, 2, 9) == 255
    assert meta["weak_cells"] == 16 and meta["hidden_cells"] == 0


def test_low_limit_never_reaches_half_the_sender_thresholds():
    """Sender obstacle threshold 4 cm -> low is 2 cm, so an isolated 2.5 cm cell stays visible."""
    fill = sparse_fill({(3, 3): (2.5, C.FREE), (9, 9): (1, C.FREE)})
    fill = ([None if h is None else (3 if h == 2.5 else h) for h in fill[0]], fill[1])   # int8 cm: use 3 cm
    loose = map_rgba(tidy_snapshot({(0, 0): fill}), VIEW, DisplayConfig(tidy_low_m=0.05), 0.0)
    tight = map_rgba(tidy_snapshot({(0, 0): fill}, thresholds=(0.04, 0.08, 2)), VIEW,
                     DisplayConfig(tidy_low_m=0.05), 0.0)
    assert cell_alpha(*loose, 3, 3) == 0 and cell_alpha(*tight, 3, 3) > 0
    assert cell_alpha(*tight, 9, 9) == 0


def test_neighbours_are_counted_past_the_view_and_tile_edges():
    """A low cell on the view's east edge whose neighbours sit in the next tile is not isolated."""
    west = sparse_fill({(15, 7): (0, C.FREE)})
    east = sparse_fill({(0, 6): (0, C.FREE), (0, 7): (0, C.FREE), (0, 8): (0, C.FREE)})
    snap = tidy_snapshot({(0, 0): west, (1, 0): east})
    rgba, meta = map_rgba(snap, MapView(0.0, 0.0, 0.8, 0.8, 320, 320), DisplayConfig(display_upsample=4), 0.0)
    assert cell_alpha(rgba, meta, 15, 7) > 0 and meta["hidden_cells"] == 0


def test_tidy_rules_switch_off_with_their_neutral_values():
    fill = sparse_fill({(2, 2): (0, C.FREE), **block(8, 8, 3, 3, 0, C.INSUFFICIENT)})
    off = DisplayConfig(display_upsample=4, tidy_min_neighbours=0, weak_alpha=1.0)
    rgba, meta = map_rgba(tidy_snapshot({(0, 0): fill}), VIEW, off, 0.0)
    assert cell_alpha(rgba, meta, 2, 2) == 255 and cell_alpha(rgba, meta, 9, 9) == 255
    assert meta["hidden_cells"] == 0 and meta["weak_cells"] == 0
    zero = DisplayConfig(display_upsample=4, tidy_low_m=0.0)
    rgba, meta = map_rgba(tidy_snapshot({(0, 0): fill}), VIEW, zero, 0.0)
    assert meta["hidden_cells"] == 0 and meta["weak_cells"] == 0


def test_hidden_cells_do_not_shade_their_neighbours():
    """A hidden cell must not feed the interpolation: the raster matches the map without it."""
    patch = block(4, 4, 4, 4, 0, C.FREE)
    with_speck = map_rgba(tidy_snapshot({(0, 0): sparse_fill({**patch, (12, 12): (2, C.FREE)})}), VIEW,
                          DisplayConfig(), 0.0)[0]
    without = map_rgba(tidy_snapshot({(0, 0): sparse_fill(patch)}), VIEW, DisplayConfig(), 0.0)[0]
    assert np.array_equal(with_speck, without)


def test_cache_key_follows_the_sender_thresholds():
    fill = sparse_fill({(3, 3): (3, C.FREE)})
    renderer = MapRenderer()
    _, first = renderer.render(tidy_snapshot({(0, 0): fill}), VIEW, DisplayConfig(tidy_low_m=0.05), 0.0)
    _, second = renderer.render(tidy_snapshot({(0, 0): fill}, thresholds=(0.04, 0.08, 2)), VIEW,
                                DisplayConfig(tidy_low_m=0.05), 0.0)
    assert not second["cache_hit"] and first["hidden_cells"] == 1 and second["hidden_cells"] == 0


@pytest.mark.parametrize("bad", [dict(tidy_low_m=0.051), dict(tidy_low_m=-0.01), dict(tidy_min_neighbours=5),
                                 dict(tidy_min_neighbours=1.0), dict(weak_alpha=0.19), dict(weak_alpha=1.01)])
def test_tidy_ranges_refuse_to_start(bad):
    with pytest.raises(ValueError):
        DisplayConfig(**bad)


def test_tidy_defaults_follow_the_spec_table():
    d = DisplayConfig()
    assert (d.tidy_low_m, d.tidy_min_neighbours, d.weak_alpha) == (0.03, 2, 0.45)


FREE_TILE = sparse_fill(block(0, 0, 16, 16, 0, C.FREE))


@pytest.mark.parametrize("cells, view_px, display, scale", [
    (16, 16, DisplayConfig(display_upsample=8), 1),
    (16, 32, DisplayConfig(display_upsample=8), 2),
    (16, 48, DisplayConfig(display_upsample=8), 3),
    (40, 160, DisplayConfig(display_upsample=8, max_render_pixels=10_000), 2),     # pixel budget shrinks 4 -> 2
])
@pytest.mark.parametrize("height_cm, code, colour_axis", [(10, C.OBSTACLE, 0), (-10, C.DITCH, 2)])
def test_a_one_cell_hazard_keeps_its_colour_when_cells_are_few_pixels(cells, view_px, display, scale,
                                                                     height_cm, code, colour_axis):
    """Codex r12 P2: at 1-3 px per cell the outline must not paint over a small hazard."""
    tiles = {(tx, ty): FREE_TILE for tx in range(3) for ty in range(3)}
    tiles[(0, 0)] = sparse_fill({**block(0, 0, 16, 16, 0, C.FREE), (8, 8): (height_cm, code)})
    side_m = cells * 0.05
    rgba, meta = map_rgba(tidy_snapshot(tiles), MapView(0.0, 0.0, side_m, side_m, view_px, view_px), display, 0.0)
    s = meta["pixels_per_cell"]
    assert s == scale
    row = int(round((meta["top_y_m"] - 9 * 0.05) / 0.05)) * s
    px = rgba[row:row + s, 8 * s:9 * s].reshape(-1, 4).astype(int)
    other = 2 - colour_axis
    assert (px[:, colour_axis] > px[:, other] + 60).all(), px
