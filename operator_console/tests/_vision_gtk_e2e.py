"""Real SRT decoding across vision-page reparenting; no robot hardware used."""
import time

import gi
import pytest

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.app import OperatorConsole, Gst, GLib  # noqa: E402
from operator_console.runtime_smoke import _free_udp_port  # noqa: E402
from operator_console.vision_map_view import MapCanvas  # noqa: E402


def pump_until(predicate, timeout_s=8):
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        while GLib.MainContext.default().pending():
            GLib.MainContext.default().iteration(False)
        if predicate():
            return True
        time.sleep(0.01)
    return predicate()


def test_map_status_repaint_is_change_driven():
    class Canvas:
        message = "수신 중"
        draws = 0

        def queue_draw(self):
            self.draws += 1

    canvas = Canvas()
    assert MapCanvas.set_message(canvas, "수신 중") is False
    assert canvas.draws == 0
    assert MapCanvas.set_message(canvas, "수신 끊김") is True
    assert canvas.draws == 1


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
        assert maps.state_badge.get_text() == '지도 수신 중'
        assert maps.scale_label.get_text() == '범위 ×1.00'
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
        maps.reset_view()
        assert maps.settings.follow and maps.follow.get_active()
        assert maps.settings.scale == 1.0
        assert maps.scale_label.get_text() == '범위 ×1.00'
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


def test_vision_attitude_sources_missing_values_static_history_and_recovery(console):
    from dataclasses import asdict, replace
    from pathlib import Path
    import json
    import math
    import os
    import socket
    from operator_console.app import Gdk
    from operator_console.runtime_smoke import _chassis_payload, _terrain_map_packets
    from operator_console.tests.test_terrain import payload as overlay_payload
    from terrain_map import contract as C

    window=console
    panel=window._vision_panel.attitude_slot
    maps=window._vision_panel.maps
    window._stack.set_visible_child_name('vision')
    assert pump_until(lambda:panel.view is not None)
    assert panel.view.raw_pitch.value is None
    assert all(r.value is None for _,r in panel.view.suspension)
    sender=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    sequence=0; last_sent=0
    phase='missing-encoders'
    def feed():
        nonlocal sequence,last_sent
        now=time.monotonic()
        if now-last_sent>.08:
            sequence+=1; last_sent=now
            data=_chassis_payload(sequence)
            data['roll_rad']=0.0
            data['pitch_rad']=math.radians(19.5)
            data['wheel_statuses'][0].update(steer_deg=-12.5,drive_turns_per_s=.12,
                command_turns_per_s=2.0,stale=False)
            data['wheel_statuses'][2]['steer_deg']=None
            if phase in ('encoders','aged','recover'):
                data.update(suspension_joints_rad=dict(ml_rocker_joint=0,mr_rocker_joint=.02,
                    bl_bogie_joint=-.03,br_bogie_joint=None),suspension_age_ms=1800 if phase=='aged' else 100,
                    suspension_source='fixture-only',suspension_frame='urdf_joint')
            if phase=='aged':
                data['roll_rad']=None; data['pitch_rad']=None
            sender.sendto(json.dumps(data).encode(),('127.0.0.1',window._chassis_receiver.port))
            sender.sendto(overlay_payload(sequence=sequence),('127.0.0.1',window._terrain_receiver.port))
            packets=_terrain_map_packets(sequence)
            status=C.parse_packet(packets[0])
            if phase!='aged':
                status=replace(status,plane_tilt=C.PlaneTilt(4,-1,100),
                    gravity=C.GravityTilt(6,-2,6.3,3,1))
            sender.sendto(C.encode(status),('127.0.0.1',window._map_receiver.port))
            sender.sendto(packets[1],('127.0.0.1',window._map_receiver.port))
        return False
    def until(predicate):
        return pump_until(lambda:feed() or predicate())
    def save(name):
        destination=os.environ.get('VISION_ATTITUDE_EVIDENCE_DIR')
        if destination:
            out=Path(destination); out.mkdir(parents=True,exist_ok=True)
            (out/(name+'.json')).write_text(json.dumps(asdict(panel.view),indent=2,ensure_ascii=False)+'\n')
            image=Gdk.pixbuf_get_from_window(window.get_window(),0,0,
                window.get_allocated_width(),window.get_allocated_height())
            image.savev(str(out/(name+'.png')),'png',[],[])
    try:
        assert until(lambda:panel.view.raw_pitch.value is not None and panel.view.still is not None
                     and maps.images is not None and maps.images.top is not None and maps.images.oblique is not None)
        assert panel.view.raw_pitch.value==pytest.approx(19.5)
        assert panel.view.body_state=='UNVERIFIED' and '보정 미확인' in panel.body.get_text()
        assert '기울기 미적용' in panel.body.get_text()
        assert panel.wheel_labels['front_left'][1].get_text()=='-12.5'
        assert panel.wheel_labels['front_left'][2].get_text()=='+0.12'
        assert panel.wheel_labels['mid_left'][2].get_text()=='오래됨'
        assert '정지 기록' in panel.still.get_text() and '+3.0°' in panel.still.get_text()
        assert all(r.text()=='미수신' for _,r in panel.view.suspension)
        assert window.get_allocated_width()<=1100 and window.get_allocated_height()<=680
        save('missing-encoders')
        phase='encoders'
        assert until(lambda:panel.view.suspension[0][1].state=='LIVE')
        assert panel.view.suspension[0][1].value==0
        assert panel.view.suspension[3][1].value is None
        assert 'fixture-only' in panel.joint_source.get_text()
        save('fixture-encoders')
        phase='aged'
        assert until(lambda:panel.view.link_state=='LIVE' and panel.view.raw_pitch.value is None and
                     panel.view.suspension[0][1].state=='STALE')
        assert until(lambda:maps.images is not None and maps.images.status.gravity is None)
        old_age=panel.view.still_age_s
        assert until(lambda:panel.view.still_age_s>old_age+.3)
        assert panel.view.still.body_pitch_deg==3
        assert '정지 기록' in panel.still.get_text() and '비교 보류' in panel.still.get_text()
        assert pump_until(lambda:panel.view.link_state=='STALE',timeout_s=2)
        assert panel.view.raw_pitch.value is None
        assert all(w.speed.value is None for w in panel.view.wheels)
        save('stale')
        phase='recover'
        assert until(lambda:panel.view.link_state=='LIVE' and panel.view.raw_pitch.value is not None)
        assert panel.view.raw_pitch.value==pytest.approx(19.5)
        window._stack.set_visible_child_name('mission')
        count=panel.refresh_count
        received=window._chassis_receiver.latest().sequence
        deadline=time.monotonic()+.6
        assert pump_until(lambda:feed() or time.monotonic()>deadline)
        assert panel.refresh_count==count and panel._timer is None
        assert window._chassis_receiver.latest().sequence>received
        window._stack.set_visible_child_name('vision')
        assert until(lambda:panel.view.link_state=='LIVE')
        print('VISION_ATTITUDE_E2E: PASS · shared UDP · missing/stale/recovery · no mount correction · static history · hidden pause')
    finally:
        sender.close()


def test_vision_floor_and_reviewed_attitude_share_existing_sources(console):
    import json
    import math
    import socket
    from operator_console.vision_attitude import AttitudeTracker
    from operator_console.attitude_calibration import Calibration
    from terrain_overlay.tests.test_runtime import ready
    from terrain_overlay.tests.scenes import default_scene
    from operator_console.runtime_smoke import _chassis_payload
    window=console;page=window._vision_panel;panel=page.attitude_slot
    a=math.radians(19.5)
    panel.tracker=AttitudeTracker(Calibration(True,(0,math.sin(a/2),0,math.cos(a/2)),
                                             reference='SYNTHETIC calibration test only'))
    overlay=ready();overlay.on_depth(default_scene(),111,10);overlay._clock.t=10.03
    sequence=0;legacy=False
    udp=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)
    def feed():
        nonlocal sequence
        sequence+=1;now=time.monotonic()
        raw=json.loads(overlay.tick())
        if legacy:raw.pop('floor_mask',None)
        udp.sendto(json.dumps(raw).encode(),("127.0.0.1",window._terrain_receiver.port))
        raw=_chassis_payload(sequence);raw.update(roll_rad=0,pitch_rad=a)
        udp.sendto(json.dumps(raw).encode(),("127.0.0.1",window._chassis_receiver.port))
        return True
    timer=GLib.timeout_add(50,feed)
    try:
        window._stack.set_visible_child_name('vision')
        assert pump_until(lambda: window._terrain_canvas._floor_layer.prepared>0 and panel.view and panel.view.body_state=='LIVE')
        assert panel.view.body_pitch.value==pytest.approx(0,abs=1e-8)
        assert '설정 보정' in panel.body.get_text() and '19.5' in panel.raw.get_text()
        layer=window._terrain_canvas._floor_layer
        assert layer.surface and len(layer.data)==9216
        window._stack.set_visible_child_name('mission')
        assert pump_until(lambda: layer.surface is not None)
        legacy=True
        assert pump_until(lambda: layer.surface is None)
        panel.tracker=AttitudeTracker()
        window._stack.set_visible_child_name('vision')
        assert pump_until(lambda: '보정 미확인' in panel.body.get_text())
    finally:
        GLib.source_remove(timer)
        udp.close()
