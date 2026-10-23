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
        "videotestsrc is-live=true ! video/x-raw,width=1280,height=720,framerate=15/1 ! "
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
        input_source="TEST",
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


def test_vision_maps_live_stale_recovery_hidden_and_controls(console):
    import json
    import os
    from pathlib import Path
    import resource
    import socket
    from types import SimpleNamespace
    from dataclasses import replace
    from terrain_map import contract as C
    from operator_console.app import Gdk
    from operator_console.runtime_smoke import _terrain_map_packets, _chassis_payload
    from operator_console.tests.test_terrain import payload as overlay_payload

    window=console
    maps=window._vision_panel.maps
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sequence=0; last_sent=0
    def clocks():
        values=[int(file.read_text()) for file in Path('/sys/devices/system/cpu').glob('cpu*/cpufreq/scaling_cur_freq')]
        return dict(min=min(values),max=max(values)) if values else None
    clocks_before=clocks()
    power={str(p.parent.name):p.read_text().strip() for p in Path('/sys/class/power_supply').glob('*/online')}
    def map_packets(sequence):
        status=C.parse_packet(_terrain_map_packets(sequence)[0])
        yield C.encode(replace(status,tile_count=3))
        for tx in range(3):
            heights=[]; codes=[]
            for i in range(256):
                row,col=divmod(i,16)
                height=12 if tx==1 and 5<=row<10 and 3<=col<7 else -10 if tx==2 and 3<=row<7 and col<4 else 0
                heights.append(height)
                codes.append(C.OBSTACLE if height>0 else C.DITCH if height<0 else C.FREE)
            yield C.encode(C.TilePacket(sender_boot_id=status.sender_boot_id,
                sender_start_mono_ns=status.sender_start_mono_ns,sender_instance=status.sender_instance,
                map_epoch=0,sent_unix_ns=time.time_ns(),tx=tx,ty=0,version=1,tile_seq=sequence,
                cells=C.pack_cells(heights,codes,[0]*256)))
    def feed():
        nonlocal sequence,last_sent
        now=time.monotonic()
        if now-last_sent>.08:
            sequence+=1; last_sent=now
            for raw in map_packets(sequence):
                sender.sendto(raw,('127.0.0.1',window._map_receiver.port))
            sender.sendto(json.dumps(_chassis_payload(sequence)).encode(),
                ('127.0.0.1',window._chassis_receiver._socket.getsockname()[1]))
            sender.sendto(overlay_payload(sequence=sequence),('127.0.0.1',window._terrain_receiver.port))
        return False
    def live():
        feed()
        return maps.state=='LIVE' and maps.images and maps.images.top and maps.images.oblique

    beats=[]
    heartbeat=GLib.timeout_add(20,lambda:beats.append(time.monotonic()) or True)
    try:
        window._stack.set_visible_child_name('vision')
        assert pump_until(live)
        assert window._terrain_canvas.view().probe_state=='LIVE'
        assert window._terrain_canvas.view().obstacles
        initial=maps.displayed_count
        started=time.monotonic(); cpu=time.process_time()
        while time.monotonic()-started<3:
            pump_until(lambda:feed() or time.monotonic()-started>=3,timeout_s=.08)
        elapsed=time.monotonic()-started
        rate=(maps.displayed_count-initial)/elapsed
        report=dict(duration_s=elapsed,map_pair_hz=rate,
            camera_fps=window._l515.last_fps,
            cpu_percent=(time.process_time()-cpu)/elapsed*100,
            max_ui_gap_s=max(b-a for a,b in zip(beats,beats[1:])),
            rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
            render_ms=maps.images.render_ms,
            cpu_scope='test process including synthetic H264 sender encoder; not console-only',
            cpu_khz_before=clocks_before,cpu_khz_after=clocks(),power_sources_online=power,
            source='local synthetic SRT + UDP fixtures; no Gateway/L515 hardware')
        assert 1.4<=rate<=2.6,report
        assert report['max_ui_gap_s']<.5,report
        assert window._l515.last_frame_age_s<.5
        destination=os.environ.get('VISION_EVIDENCE_DIR')
        if destination:
            out=Path(destination); out.mkdir(parents=True,exist_ok=True)
            (out/'local-metrics.json').write_text(json.dumps(report,indent=2)+'\n')
            pixbuf=Gdk.pixbuf_get_from_window(window.get_window(),0,0,
                window.get_allocated_width(),window.get_allocated_height())
            pixbuf.savev(str(out/'vision-maps.png'),'png',[],[])
        assert pump_until(lambda:maps.state=='STALE',timeout_s=2)
        assert maps.oblique.raster is None
        assert maps.top.raster is not None
        assert '수신 끊김' in maps.top.message
        assert pump_until(live)
        old=maps.settings.scale
        maps._scroll(maps.top,SimpleNamespace(direction=Gdk.ScrollDirection.UP))
        assert maps.settings.scale<old
        maps._press(maps.top,SimpleNamespace(button=1,x=100,y=50))
        center=maps.settings.center
        maps._motion(maps.top,SimpleNamespace(x=125,y=60))
        assert not maps.settings.follow and maps.settings.center!=center
        maps.follow.set_active(True)
        assert pump_until(live)
        window._stack.set_visible_child_name('mission')
        count=maps.worker.render_count
        received=window._map_receiver.snapshot(time.monotonic()).status.sequence
        until=time.monotonic()+.7
        assert pump_until(lambda:feed() or time.monotonic()>until)
        assert maps.worker.render_count==count
        assert window._map_receiver.snapshot(time.monotonic()).status.sequence>received
        assert maps._timer is None and maps.worker.latest()[1] is None
        window._stack.set_visible_child_name('vision')
        assert pump_until(live)
        print('VISION_MAP_LOCAL_METRICS',json.dumps(report))
    finally:
        GLib.source_remove(heartbeat)
        sender.close()
