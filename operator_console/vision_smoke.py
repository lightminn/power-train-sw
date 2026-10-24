"""Isolated GTK/SRT vision execution gate, launched by runtime_smoke.

Only local test patterns and UDP fixtures are used. No cameras or motor commands.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import socket
import sys
import tempfile
import time
import traceback

# Match the console's normal import order before creating any native pipelines.
from .app import OperatorConsole, Gst, Gtk, GLib
from .runtime_smoke import _free_udp_port, _terrain_map_packets, _chassis_payload, _terrain_payload
from .smoke_timing import callback_gap

FAULTS = ('map', 'telemetry', 'missing-zero', 'stale', 'recovery',
          'hidden-render', 'duplicate-camera', 'shutdown', 'drawing-stall')


def exercise(fault=None):
    if fault == 'missing-zero':
        from . import vision_attitude
        original = vision_attitude.reading
        def fabricated(value, age, source, **kwargs):
            return original(0 if value is None else value, 0 if age is None else age, source, **kwargs)
        vision_attitude.reading = fabricated
    if fault == 'stale':
        from . import vision_attitude, vision_map_view
        vision_attitude.TELEMETRY_MAX_AGE_S = 100
        vision_map_view.LINK_LOST_S = 100
    Gst.init(None)
    assert Gtk.init_check()[0], 'vision display unavailable'
    ports = [_free_udp_port(), _free_udp_port()]
    while ports[0] == ports[1]:
        ports[1] = _free_udp_port()
    sources = []
    window = None
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    errors = []
    original_hook = sys.excepthook
    original_launch = Gst.parse_launch
    def callback_error(kind, value, tb):
        errors.append(f'{kind.__name__}: {value}')
        original_hook(kind, value, tb)
    sys.excepthook = callback_error
    samples = []
    beats = []
    draws = []
    heartbeat = None
    source_id = None
    try:
        for port in ports:
            src = Gst.parse_launch(
                'videotestsrc is-live=true ! video/x-raw,width=1280,height=720,framerate=15/1 ! '
                'x264enc tune=zerolatency speed-preset=ultrafast threads=2 key-int-max=15 ! '
                'h264parse config-interval=-1 ! mpegtsmux alignment=7 ! '
                f'srtsink uri="srt://127.0.0.1:{port}?mode=listener" '
                'wait-for-connection=false sync=false async=false')
            sources.append(src)
            src.set_state(Gst.State.PLAYING)
        with tempfile.TemporaryDirectory(prefix='vision-smoke-') as directory:
            probe_file = Path(directory)/'probe.json'
            receiver_descriptions = []
            def audit_receiver(description, *args, **kwargs):
                receiver_descriptions.append(description)
                return original_launch(description, *args, **kwargs)
            Gst.parse_launch = audit_receiver
            window = OperatorConsole('127.0.0.1', ports[0], ports[1], 0, 60, 0, 0,
                arm_telemetry_port=0, environment_telemetry_port=0, terrain_port=0,
                terrain_map_port=0, ops_port=1, ops_token_file=str(Path(directory)/'absent'),
                input_source='TEST', smoke_probe_file=str(probe_file))
            assert len(receiver_descriptions)==2 and all(
                sum(f'127.0.0.1:{port}?' in description for description in receiver_descriptions)==1
                for port in ports), 'vision duplicate initial camera/decoder connection'
            maps, panel = window._vision_panel.maps, window._vision_panel.attitude_slot
            pipelines = (window._l515._pipeline, window._d435._pipeline)
            identities = (window._l515._sink, window._l515._video_widget,
                          window._chassis_receiver, window._map_receiver, window._terrain_receiver)
            original_parent = window._l515.get_parent()
            def no_new_decoder(*args, **kwargs):
                raise AssertionError('vision duplicate camera/decoder creation')
            Gst.parse_launch = no_new_decoder
            if fault == 'duplicate-camera':
                Gst.parse_launch('videotestsrc ! fakesink')
            window.show_all()
            heartbeat = GLib.timeout_add(20, lambda: beats.append(time.monotonic()) or True)
            window._l515._video_widget.connect_after('draw', lambda *_: draws.append(time.monotonic()) or False)
            sequence = 0
            phase = 'live'
            def feed():
                nonlocal sequence
                sequence += 1
                if phase == 'cut' or (fault == 'recovery' and phase == 'recover'):
                    return True
                data = _chassis_payload(sequence)
                data.update(roll_rad=0, pitch_rad=math.radians(19.5))
                data['wheel_statuses'][0].update(steer_deg=-12.5, drive_turns_per_s=.12,
                                               command_turns_per_s=2., stale=False)
                if phase in ('encoders', 'aged'):
                    data.update(suspension_joints_rad={'ml_rocker_joint':0, 'mr_rocker_joint':.02},
                        suspension_age_ms=1800 if phase=='aged' else 100,
                        suspension_source='fixture-only', suspension_frame='urdf_joint')
                if phase == 'aged':
                    data['roll_rad'] = data['pitch_rad'] = None
                if fault != 'telemetry':
                    sender.sendto(json.dumps(data).encode(), ('127.0.0.1', window._chassis_receiver.port))
                sender.sendto(json.dumps(_terrain_payload(sequence)).encode(),
                              ('127.0.0.1', window._terrain_receiver.port))
                if fault != 'map':
                    for raw in _terrain_map_packets(sequence):
                        sender.sendto(raw, ('127.0.0.1', window._map_receiver.port))
                return True
            context = GLib.MainContext.default()
            def pump_until(predicate, label, timeout=8):
                deadline = time.monotonic()+timeout
                while time.monotonic() < deadline:
                    for _ in range(32):
                        if not context.pending():
                            break
                        context.iteration(False)
                    assert not errors, f'{label}: GTK callback errors: {errors}'
                    if predicate():
                        return
                    time.sleep(.005)
                raise AssertionError(f'{label}: timed out; {snapshot()}')
            def dwell(seconds):
                end = time.monotonic()+seconds
                pump_until(lambda: time.monotonic()>=end, 'dwell', seconds+3)
            def snapshot():
                window._write_smoke_probe({})
                state = json.loads(probe_file.read_text())
                assert state['vision_camera_shared']==(state['page']=='vision'), 'vision camera sharing probe'
                return state['vision']
            def record(label):
                samples.append(dict(phase=label, **snapshot()))
            def live():
                p = snapshot()
                return (p['map_state']=='LIVE' and p['map_rasters']==[True,True]
                        and p['link']=='LIVE' and p['raw_pitch_state']=='LIVE'
                        and window._l515.last_frame_age_s is not None
                        and window._l515.last_frame_age_s < .5
                        and bool(window._terrain_canvas.view().obstacles))
            window._stack.set_visible_child_name('vision')
            pump_until(lambda: panel.view is not None and maps._timer is not None, 'vision creation')
            p = snapshot()
            assert all(p['slots_mapped']), 'vision four slots not mapped'
            assert p['map_state']=='WAITING' and p['link']=='WAITING', 'vision initial missing data state'
            assert p['raw_pitch'] is None and p['joint_values']==[None]*4, 'vision missing values fabricated'
            assert p['joints_label'].count('미수신')==4, 'vision missing encoders not labelled'
            record('missing')
            source_id = GLib.timeout_add(80, feed)
            pump_until(live, 'vision map/telemetry LIVE')
            p = snapshot()
            assert abs(p['raw_pitch']-19.5)<1e-6, 'vision applied unverified pitch correction'
            assert '보정 미확인' in p['body_label'], 'vision body correction provenance missing'
            assert p['wheel_labels']['front_left']==['-12.5','+0.12'], 'vision wheel feedback not displayed'
            assert p['joint_values']==[None]*4 and p['joint_states']==['WAITING']*4, 'vision encoder missing values'
            assert window._terrain_canvas.view().obstacles, 'vision terrain overlay not aligned/live'
            record('live')
            window._vision_panel.show_terrain.set_active(False)
            assert not window._show_terrain.get_active() and not window._terrain_canvas._enabled
            window._show_terrain.set_active(True)
            assert window._vision_panel.show_terrain.get_active() and window._terrain_canvas._enabled
            phase = 'encoders'
            pump_until(lambda: snapshot()['joint_states'][:2]==['LIVE']*2, 'vision encoder fixture')
            p = snapshot()
            assert p['joint_values']==[0., math.degrees(.02), None, None], 'vision actual zero/partial missing'
            assert 'fixture-only' in p['source_label'], 'vision encoder source absent'
            record('encoders')
            phase = 'aged'
            pump_until(lambda: snapshot()['link']=='LIVE' and snapshot()['joint_states'][:2]==['STALE']*2,
                       'vision aged measurement')
            p = snapshot()
            assert p['raw_pitch'] is None and p['raw_pitch_state']=='WAITING', 'vision null raw value retained'
            assert p['joint_values']==[None]*4, 'vision old measurement still numeric'
            assert p['joints_label'].count('오래됨')==2, 'vision old measurement labels'
            record('aged')
            phase = 'live'
            pump_until(live, 'vision before disconnect')
            phase = 'cut'
            pump_until(lambda: snapshot()['link']=='STALE' and snapshot()['map_state']=='STALE', 'vision disconnect/stale')
            p = snapshot()
            assert not p['map_rasters'][1], 'vision stale oblique remains live'
            assert 'LIVE' not in p['wheel_states'], 'vision stale wheel values remain live'
            assert p['raw_pitch_state']=='STALE' and '오래됨' in p['raw_label'], 'vision stale raw pitch not hidden'
            assert p['wheel_labels']['front_left']==['오래됨','오래됨'], 'vision stale wheel labels'
            record('disconnected')
            phase = 'recover'
            pump_until(live, 'vision recovery')
            assert snapshot()['joint_states']==['WAITING']*4, 'vision recovery resurrected missing encoder'
            record('recovered')
            # Healthy visible interval measures actual drawing, not just decoder arrival.
            dwell(.5)
            beats.clear(); draws.clear()
            watching = time.monotonic()
            if fault == 'drawing-stall':
                GLib.timeout_add(300, lambda: time.sleep(.65) or False)
            dwell(1.5)
            watched = time.monotonic()
            assert len(draws)>5, 'vision camera never drew'
            assert callback_gap(watching, watched, draws) < .5, 'vision camera draw stalled'
            assert callback_gap(watching, watched, beats) < .5, 'vision GTK stalled'
            for target in ('systems', 'mission'):
                window._stack.set_visible_child_name(target)
                assert window._l515.get_parent() is original_parent, 'vision camera parent not restored'
                assert window._swap_button.get_sensitive(), 'vision swap button not restored'
                count = maps.worker.render_count; updates = panel.refresh_count
                received = window._chassis_receiver.latest().sequence
                map_sequence = window._map_receiver.snapshot(time.monotonic()).status.sequence
                if fault == 'hidden-render':
                    maps.worker.configure(active=True)
                dwell(.8)
                p = snapshot()
                assert not p['map_timer'] and not p['attitude_timer'], 'vision hidden UI timers active'
                assert maps.worker.render_count==count and panel.refresh_count==updates, 'vision hidden render continues'
                assert window._chassis_receiver.latest().sequence>received, 'vision hidden telemetry RX stopped'
                assert window._map_receiver.snapshot(time.monotonic()).status.sequence>map_sequence, 'vision hidden map RX stopped'
                assert window._l515._pipeline is pipelines[0] and window._d435._pipeline is pipelines[1]
                assert identities==(window._l515._sink, window._l515._video_widget,
                    window._chassis_receiver, window._map_receiver, window._terrain_receiver), 'vision receiver/sink replaced'
                window._stack.set_visible_child_name('vision')
                pump_until(live, 'vision tab return')
            # Also preserve a user-selected D435 MAIN role through vision.
            window._stack.set_visible_child_name('mission')
            assert window.swap_camera_views(window._d435, user_initiated=True)
            d435_parent = window._l515.get_parent()
            window._stack.set_visible_child_name('vision')
            pump_until(live, 'vision D435-main roundtrip')
            window._stack.set_visible_child_name('mission')
            assert window._l515.get_parent() is d435_parent and window._main_video is window._d435
            window._stack.set_visible_child_name('vision')
            pump_until(live, 'vision final return')
            record('returned')
            if fault == 'shutdown':
                maps.worker.close = lambda: None
            rx = (window._metadata_receiver, window._telemetry_receiver, window._chassis_receiver,
                  window._arm_receiver, window._environment_receiver, window._terrain_receiver, window._map_receiver)
            rx_ports = [r._socket.getsockname()[1] for r in rx]
            GLib.source_remove(source_id)
            source_id = None
            window.destroy()
            # The real entrypoint exits Gtk.main after destroy. Do not dispatch
            # unrelated legacy-widget timers against destroyed GTK objects.
            deadline = time.monotonic()+2
            while maps.worker._thread.is_alive() and time.monotonic()<deadline:
                time.sleep(.01)
            assert not maps.worker._thread.is_alive(), 'vision worker shutdown: timed out'
            assert maps._timer is None and panel._timer is None and maps._closed and panel._closed
            assert all(not r._thread.is_alive() for r in rx), 'vision receiver shutdown'
            assert all(p.get_state(Gst.SECOND)[1]==Gst.State.NULL for p in pipelines), 'vision pipelines not NULL'
            for port in rx_ports:
                with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as probe:
                    probe.bind(('127.0.0.1', port))
            assert not errors, f'vision shutdown callback errors: {errors}'
            return dict(result='PASS', phases=samples, shutdown='worker/timers/receivers/pipelines/ports closed',
                        receiver_pipelines=len(receiver_descriptions),
                        source='local synthetic SRT/UDP; no Gateway/L515 hardware')
    finally:
        Gst.parse_launch = original_launch
        if source_id is not None:
            GLib.source_remove(source_id)
        if heartbeat is not None:
            GLib.source_remove(heartbeat)
        if window is not None:
            window.destroy()
        sender.close()
        for src in sources:
            src.set_state(Gst.State.NULL)
        sys.excepthook = original_hook


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--fault', choices=FAULTS)
    parser.add_argument('--report-file', type=Path)
    args = parser.parse_args()
    code = 1
    try:
        report = json.dumps(exercise(args.fault), ensure_ascii=False)
        if args.report_file:
            args.report_file.write_text(report+'\n', encoding='utf-8')
            print('VISION-CHILD: PASS', flush=True)
        else:
            print(report, flush=True)
        code = 0
    except Exception:
        traceback.print_exc()
    # Use the console entrypoint's existing libsrt destructor policy, after cleanup.
    sys.stdout.flush(); sys.stderr.flush()
    os._exit(code)


if __name__ == '__main__':
    main()
