"""Real GTK wiring of the L515 terrain overlay; run with xvfb-run + system python."""
import os
import socket
import time

os.environ.setdefault("GDK_BACKEND", "x11")

import gi
import pytest

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.tests.test_terrain import payload  # noqa: E402


@pytest.fixture
def window(tmp_path):
    from operator_console.app import OperatorConsole, Gst
    if not Gtk.init_check()[0]:
        pytest.skip("display unavailable")
    Gst.init(None)
    console = OperatorConsole(
        "127.0.0.1", 51002, 51000, 0, 60, 0, 0,
        arm_telemetry_port=0, environment_telemetry_port=0, terrain_port=0,
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


def test_front_panel_hands_video_size_only_to_its_own_canvases(window):
    window._l515.publish_video_size(1280, 720)
    assert window._terrain_canvas._video_size == (1280, 720)
    assert window._d435._metadata_canvas._video_width is None
    window._d435.publish_video_size(848, 480)
    assert (window._d435._metadata_canvas._video_width,
            window._d435._metadata_canvas._video_height) == (848, 480)
    assert window._terrain_canvas._video_size == (1280, 720)


def test_live_packet_reaches_the_canvas_and_draws(window):
    import cairo
    window._l515.publish_video_size(1280, 720)
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        sender.sendto(payload(sequence=1), ("127.0.0.1", window._terrain_receiver.port))
        assert pump_until(lambda: window._terrain_receiver.latest() is not None)
    finally:
        sender.close()
    # Fake a fresh decoded frame at the video-health boundary (no real SRT).
    window._l515._pipeline_live = True
    window._l515._last_frame_monotonic = time.monotonic()
    window._l515._refresh_video_health()
    canvas = window._terrain_canvas
    # Judge 50 ms after reception: a loaded main loop may be slower than the 0.5 s link limit.
    view = canvas.view(now_s=window._terrain_receiver.last_rx_monotonic + 0.05)
    assert view.probe_state == "LIVE"
    assert view.badge.startswith("지형 OK")
    assert len(view.obstacles) == 1
    allocation = canvas.get_allocation()
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, allocation.width, allocation.height)
    canvas._on_draw(canvas, cairo.Context(surface))  # exceptions surface here
    surface.flush()
    assert any(surface.get_data())  # something was painted


def test_terrain_checkbox_only_hides_the_drawing(window):
    window._show_terrain.set_active(False)
    assert window.overlay_view_state["show_terrain"] is False
    assert window._terrain_canvas._enabled is False
    assert window._show_objects.get_active() is True  # D435 options untouched
    window._show_terrain.set_active(True)
    assert window._terrain_canvas._enabled is True


def test_badge_text_has_no_missing_glyphs(window):
    import cairo
    from operator_console.app import Pango, PangoCairo
    surface = cairo.ImageSurface(cairo.FORMAT_ARGB32, 400, 60)
    for text in ("지형: 수신 없음", "정합 미검증 · 참고용 · 낮은 장애물/구덩이 미표시"):
        layout = window._terrain_canvas._layout(cairo.Context(surface), text, "Sans 14", 380)
        assert layout.get_unknown_glyphs_count() == 0, text


def test_terrain_canvas_sits_above_the_video_standby_layer(window):
    stage = window._terrain_canvas.get_parent()
    children = stage.get_children()
    standby = window._l515._placeholder
    assert stage is standby.get_parent()
    assert children.index(window._terrain_canvas) > children.index(standby)
    assert stage.get_overlay_pass_through(window._terrain_canvas) is True


def test_video_health_drives_outline_visibility(window):
    window._l515._pipeline_live = False
    window._l515._refresh_video_health()
    assert window._terrain_canvas._video_live is False


def _fake_live_video(panel):
    panel._pipeline_live = True
    panel._last_frame_monotonic = time.monotonic()
    panel._refresh_video_health()


@pytest.mark.parametrize("kind", ["error", "eos", "restart"])
def test_video_loss_drops_outlines_immediately(window, kind):
    from operator_console.app import GLib, Gst
    panel = window._l515
    _fake_live_video(panel)
    assert window._terrain_canvas._video_live is True
    if kind == "error":
        error = GLib.Error.new_literal(Gst.ResourceError.quark(), "fixture", Gst.ResourceError.READ)
        panel._on_bus_message(None, Gst.Message.new_error(panel._pipeline, error, "fixture"))
    elif kind == "eos":
        panel._on_bus_message(None, Gst.Message.new_eos(panel._pipeline))
    else:
        panel._restart_pipeline()
    # No periodic health refresh in between: the change must be immediate.
    assert window._terrain_canvas._video_live is False
