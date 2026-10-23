"""Real SRT decoding across vision-page reparenting; no robot hardware used."""
import time

import gi
import pytest

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.app import OperatorConsole, Gst, GLib  # noqa: E402
from operator_console.runtime_smoke import _free_udp_port  # noqa: E402


def pump_until(predicate, timeout_s=8):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        while GLib.MainContext.default().pending():
            GLib.MainContext.default().iteration(False)
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


@pytest.fixture
def console(tmp_path):
    if not Gtk.init_check()[0]:
        pytest.skip("needs GTK display (run under xvfb-run)")
    Gst.init(None)
    port = _free_udp_port()
    sender = Gst.parse_launch(
        "videotestsrc is-live=true ! video/x-raw,width=640,height=360,framerate=15/1 ! "
        "x264enc tune=zerolatency speed-preset=ultrafast key-int-max=15 ! "
        "h264parse config-interval=-1 ! mpegtsmux alignment=7 ! "
        f'srtsink uri="srt://127.0.0.1:{port}?mode=listener" '
        "wait-for-connection=false sync=false async=false"
    )
    sender.set_state(Gst.State.PLAYING)
    window = OperatorConsole(
        "127.0.0.1", _free_udp_port(), port, 0, 60, 0, 0,
        arm_telemetry_port=0, environment_telemetry_port=0, terrain_port=0,
        terrain_map_port=0, ops_port=1,
        ops_token_file=str(tmp_path / "absent-token"),
    )
    window.show_all()
    try:
        assert pump_until(lambda: window._l515._last_frame_monotonic is not None)
        yield window
    finally:
        window.destroy()
        sender.set_state(Gst.State.NULL)


@pytest.mark.parametrize("work_camera_main", [False, True])
def test_vision_roundtrip_keeps_single_decoder_and_restores_layout(console, monkeypatch,
                                                                 work_camera_main):
    window = console
    if work_camera_main:
        assert window.swap_camera_views(window._d435, user_initiated=True)
    main = window._main_video
    parent = window._l515.get_parent()
    pipeline = window._l515._pipeline
    sink = window._l515._sink
    video_widget = window._l515._video_widget
    overlay_parent = window._terrain_canvas.get_parent()
    receiver = window._map_receiver
    page = window._vision_panel
    assert page.map_receiver is receiver
    assert page.chassis_receiver is window._chassis_receiver

    def unexpected_pipeline(*args, **kwargs):
        pytest.fail("tab navigation must not create a decoder or SRT connection")

    monkeypatch.setattr(Gst, "parse_launch", unexpected_pipeline)
    for _ in range(3):
        before = window._l515._last_frame_monotonic
        window._stack.set_visible_child_name("vision")
        assert window._l515.get_parent() is page.camera_slot
        assert window._main_video is main  # original mission role preserved
        assert not window._swap_button.get_sensitive()
        assert not window.swap_camera_views(
            window._d435 if main is window._l515 else window._l515,
            user_initiated=True,
        )
        assert pump_until(lambda: window._l515._last_frame_monotonic > before)
        assert all(slot.get_mapped() for slot in (
            page.camera_slot, page.top_slot, page.oblique_slot, page.attitude_slot,
        ))
        before = window._l515._last_frame_monotonic
        # Leaving for a third page must restore mission even before returning to it.
        window._stack.set_visible_child_name("systems")
        assert window._l515.get_parent() is parent
        assert window._swap_button.get_sensitive()
        window._stack.set_visible_child_name("mission")
        assert pump_until(lambda: window._l515._last_frame_monotonic > before)
        assert window._main_video is main
        assert window._l515._pipeline is pipeline
        assert window._l515._sink is sink
        assert window._l515._video_widget is video_widget
        assert window._terrain_canvas.get_parent() is overlay_parent
        assert window._map_receiver is receiver
    assert window.swap_camera_views(
        window._d435 if main is window._l515 else window._l515, user_initiated=True,
    )


def test_vision_overlay_toggle_shares_mission_state(console):
    window = console
    page = window._vision_panel
    window._open_mobility_action("__terrain_preview__")
    assert window._stack.get_visible_child_name() == "vision"
    assert not hasattr(window, "_lidar_preview_window")
    assert not hasattr(window, "_obstacle_receiver")
    page.show_terrain.set_active(False)
    assert not window._show_terrain.get_active()
    assert not window.overlay_view_state["show_terrain"]
    assert not window._terrain_canvas._enabled
    window._stack.set_visible_child_name("mission")
    window._show_terrain.set_active(True)
    assert page.show_terrain.get_active()
    assert window._terrain_canvas._enabled
