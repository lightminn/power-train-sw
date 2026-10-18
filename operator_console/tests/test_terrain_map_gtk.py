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
                            map_hillshade_azimuth=200.0, map_hillshade_altitude=30.0, map_hillshade_exaggeration=5.0,
                            map_tidy_low_m=0.02, map_tidy_min_neighbours=3, map_weak_alpha=0.6)
    cfg = map_display_from_args(parser, ok)
    assert (cfg.display_upsample, cfg.max_render_pixels, cfg.stale_display_s, cfg.hillshade_azimuth_deg,
            cfg.hillshade_altitude_deg, cfg.z_exaggeration) == (2, 500_000, 20.0, 200.0, 30.0, 5.0)
    assert (cfg.tidy_low_m, cfg.tidy_min_neighbours, cfg.weak_alpha) == (0.02, 3, 0.6)
    for bad in ({"map_stale_s": 61.0}, {"map_tidy_min_neighbours": 5}, {"map_weak_alpha": 0.1}):
        with pytest.raises(SystemExit):
            map_display_from_args(parser, argparse.Namespace(**{**vars(ok), **bad}))


# --- Codex Task 9 review ---------------------------------------------------------
def test_tiles_alone_do_not_keep_an_old_pose_live(window):
    panel = window._map_panel
    send(window, status(1, pose=(0.4, 0.4, 0.0)))
    assert pump_until(lambda: received(panel, 0))
    status_rx = panel.receiver.snapshot(time.monotonic()).status.received_s
    time.sleep(0.7)
    send(window, tile(0, 0, 1, 1, payload=C.pack_cells(*split_fill(), [0] * 256)))   # tiles keep flowing
    assert pump_until(lambda: received(panel, 1))
    now = panel.receiver.snapshot(time.monotonic()).last_rx_s + 0.01
    assert now - status_rx > 0.5
    _, texts = draw(panel, now)
    assert any("수신 끊김" in t for t in texts) and panel.probe(now)["terrain_map"] == "STALE"


def test_turning_follow_off_keeps_the_view_where_it_was(window):
    panel = window._map_panel
    send(window, status(1, pose=(10.0, 5.0, 0.3)))
    assert pump_until(lambda: received(panel, 0))
    assert panel._view_center() == (10.0, 5.0)
    panel.follow_button.set_active(False)
    assert panel._view_center() == (10.0, 5.0)


def test_zooming_out_never_silently_drops_the_map(window):
    from operator_console.terrain_map_panel import ZOOM_RANGE
    panel = window._map_panel
    send(window, status(1, pose=(0.4, 0.4, 0.0)), tile(0, 0, 1, 1, payload=C.pack_cells(*split_fill(), [0] * 256)))
    assert pump_until(lambda: received(panel, 1))
    panel._m_per_px = ZOOM_RANGE[1]
    rx = panel.receiver.snapshot(time.monotonic()).last_rx_s
    draw(panel, rx + 0.05, 800, 600)
    assert panel.last_meta["drawn"]


# --- ground-plane tilt (r11) -----------------------------------------------------------
def test_tilt_lines_show_while_fresh_and_hide_when_old(window):
    panel = window._map_panel
    send(window, status(1, pose=(0.4, 0.4, 0.0), plane_tilt=C.PlaneTilt(4.0, -1.0, 200.0),
                        gravity=C.GravityTilt(6.5, -1.0, 6.6, 2.5, 0.0)))
    assert pump_until(lambda: received(panel, 0))
    rx = panel.receiver.snapshot(time.monotonic()).last_rx_s
    _, texts = draw(panel, rx + 0.05)
    tilt = [t for t in texts if "기준면" in t]
    assert any("차체 기준" in t and "+4.0°" in t for t in tilt)
    assert any("정지 구간 측정" in t and "+6.5°" in t and "앞 들림 +2.5°" in t for t in tilt)
    assert panel.unknown_glyphs == 0
    _, texts = draw(panel, rx + 0.85)                           # 0.2 s old at send + 0.85 s since → expired
    assert not any("기준면" in t for t in texts)


def test_no_gravity_line_without_gravity(window):
    panel = window._map_panel
    send(window, status(1, pose=(0.4, 0.4, 0.0), plane_tilt=C.PlaneTilt(0.5, 0.0, 10.0)))
    assert pump_until(lambda: received(panel, 0))
    rx = panel.receiver.snapshot(time.monotonic()).last_rx_s
    _, texts = draw(panel, rx + 0.05)
    assert any("차체 기준" in t for t in texts) and not any("정지 구간 측정" in t for t in texts)


def test_note_and_legend_describe_only_the_tidy_rules_that_are_on():
    """Codex r12 P3: switching a rule off must also drop its sentence."""
    from operator_console.terrain_map import DisplayConfig
    from operator_console.terrain_map_panel import FIXED_NOTE, LEGEND, note_and_legend
    assert note_and_legend(DisplayConfig()) == (FIXED_NOTE, LEGEND)
    for display, hidden, weak in ((DisplayConfig(tidy_min_neighbours=0), False, True),
                                  (DisplayConfig(weak_alpha=1.0), True, False),
                                  (DisplayConfig(tidy_low_m=0.0), False, False)):
        note, legend = note_and_legend(display)
        labels = [label for label, _ in legend]
        assert ("낮은 외톨이 칸은 숨김" in note) is hidden
        assert ("관측 부족(낮은 칸은 반투명)" in labels) is weak and ("관측 부족" in labels) is not weak
        assert "실시간 장애물 감지가 아님" in note and len(legend) == 5


@pytest.mark.parametrize("width, height", [(720, 540), (480, 400), (1200, 800)])
def test_scale_bar_stays_above_the_legend_when_the_note_wraps(window, width, height):
    """r12's longer note wraps on narrow pages; the legend box must not cover the scale bar."""
    send(window, status(1, pose=(0.4, 0.4, 0.0)),
         tile(0, 0, 1, 1, payload=C.pack_cells(*split_fill(), [0] * 256)))
    panel = window._map_panel
    assert pump_until(lambda: received(panel, 1))
    rx = panel.receiver.snapshot(time.monotonic()).last_rx_s
    draw(panel, rx + 0.05, width, height)
    label_top = panel.last_scale_bar_y - 18
    assert 0 < label_top and panel.last_scale_bar_y + 2 < panel.last_legend_top - 6


def test_wheel_slip_reason_has_its_own_text():
    from operator_console.terrain_map_panel import REASON_TEXT
    from terrain_map import contract as C
    assert all(r in REASON_TEXT for reasons in C.STATUS_REASONS.values() for r in reasons)
    assert REASON_TEXT["WHEEL_SLIP"] == "바퀴 합의 실패"
