"""Real GTK wiring of the terrain map page; run with xvfb-run + system python."""
import argparse
import os
import socket
import time

os.environ.setdefault("GDK_BACKEND", "x11")

import gi
import numpy as np
import pytest

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.tests.test_terrain_map_console import split_fill, status, tile  # noqa: E402
from terrain_map import contract as C  # noqa: E402


@pytest.fixture
def window(tmp_path):
    from operator_console.app import OperatorConsole, Gst
    if not Gtk.init_check()[0]:
        pytest.skip("display unavailable")
    Gst.init(None)
    console = OperatorConsole(
        "127.0.0.1", 51002, 51000, 0, 60, 0, 0,
        arm_telemetry_port=0, environment_telemetry_port=0, terrain_port=0, terrain_map_port=0,
        ops_token_file=str(tmp_path / "absent-token"),
    )
    console.show_all()
    yield console
    console.destroy()


def pump_until(predicate, timeout_s=3.0):
    from operator_console.app import GLib
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        while GLib.MainContext.default().pending():
            GLib.MainContext.default().iteration(False)
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def received(panel, tiles):
    """Reception finished (status and ``tiles`` tiles) — independent of how busy the main loop is."""
    snap = panel.receiver.snapshot(time.monotonic())
    return snap.status is not None and len(snap.tiles) == tiles


def send(window, *packets):
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        for raw in packets:
            sender.sendto(raw, ("127.0.0.1", window._map_receiver.port))
    finally:
        sender.close()


def draw(panel, now, width=800, height=600):
    import cairo
    texts = []
    original = panel._text

    def capture(cr, text, font, width=None):
        texts.append(text)
        return original(cr, text, font, width)
    panel._text = capture
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, width, height)
    panel.draw_into(cairo.Context(surface), width, height, now)
    panel._text = original
    data = np.frombuffer(surface.get_data(), np.uint8).reshape(height, surface.get_stride())[:, : width * 4]
    return data.reshape(height, width, 4), texts                 # BGRA


def test_map_page_sits_next_to_the_status_page(window):
    stack = window._stack
    assert stack.get_child_by_name("terrain_map") is window._map_panel
    assert stack.child_get_property(window._map_panel, "title") == "지형 지도"
    assert window._map_panel.probe()["terrain_map"] == "WAITING"


def test_live_packets_are_drawn_with_hangul_text_legend_and_note(window):
    from operator_console.terrain_map_panel import FIXED_NOTE, LEGEND
    send(window, status(1, pose=(0.4, 0.4, 0.0)),
         tile(0, 0, 1, 1, payload=C.pack_cells(*split_fill(), [0] * 256)))
    panel = window._map_panel
    assert pump_until(lambda: received(panel, 1))
    rx = panel.receiver.snapshot(time.monotonic()).last_rx_s
    assert panel.probe(rx + 0.05) == {"terrain_map": "LIVE", "terrain_map_tiles": 1}
    bgra, texts = draw(panel, rx + 0.05)
    assert panel.last_meta["drawn"] and panel.unknown_glyphs == 0
    red = (bgra[..., 2] > 180) & (bgra[..., 1] < 90) & (bgra[..., 0] < 90)
    assert red.sum() > 200                                     # the obstacle half of the tile
    assert FIXED_NOTE in texts and all(label in texts for label, _ in LEGEND)
    assert not any("수신 끊김" in t for t in texts)


def test_link_loss_and_bias_pending_are_shown(window):
    send(window, status(1, pose=(0.4, 0.4, 0.0), status="NO_POSE", reason="BIAS_PENDING"))
    panel = window._map_panel
    assert pump_until(lambda: received(panel, 0))
    rx = panel.receiver.snapshot(time.monotonic()).last_rx_s
    _, texts = draw(panel, rx + 0.05)
    assert "로버를 1초 정지하세요" in texts and not any("수신 끊김" in t for t in texts) and panel.unknown_glyphs == 0
    later = rx + 0.8
    _, texts = draw(panel, later)
    assert "수신 끊김" in texts and panel.probe(later)["terrain_map"] == "STALE"


def test_cli_display_numbers_reach_the_config_and_bad_ones_are_usage_errors():
    from operator_console.app import map_display_from_args
    parser = argparse.ArgumentParser()
    ok = argparse.Namespace(map_display_upsample=2, map_max_render_pixels=500_000, map_stale_s=20.0,
                            map_hillshade_azimuth=200.0, map_hillshade_altitude=30.0, map_hillshade_exaggeration=5.0)
    cfg = map_display_from_args(parser, ok)
    assert (cfg.display_upsample, cfg.max_render_pixels, cfg.stale_display_s, cfg.hillshade_azimuth_deg,
            cfg.hillshade_altitude_deg, cfg.z_exaggeration) == (2, 500_000, 20.0, 200.0, 30.0, 5.0)
    with pytest.raises(SystemExit):
        map_display_from_args(parser, argparse.Namespace(**{**vars(ok), "map_stale_s": 61.0}))
