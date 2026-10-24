"""Explicit synthetic performance experiment; never opens a camera or drives motors.

Run in an isolated display with system Python; source/encoder CPU is excluded.
This is a development measurement, not Gateway/L515 acceptance.
"""
import argparse
from dataclasses import replace
import json
import os
from pathlib import Path
import resource
import socket
import subprocess
import sys
import time
from operator_console.smoke_timing import callback_gap as heartbeat_gap


def machine():
    def reads(pattern):
        result = {}
        for path in Path('/sys').glob(pattern):
            try:
                result[str(path)] = path.read_text().strip()
            except OSError:
                pass
        return result
    return dict(power=reads('class/power_supply/*/online'),
                power_type=reads('class/power_supply/*/type'),
                frequency_khz=reads('devices/system/cpu/cpu*/cpufreq/scaling_cur_freq'),
                temperature_mdeg=reads('class/thermal/thermal_zone*/temp'),
                temperature_type=reads('class/thermal/thermal_zone*/type'))


def ac_online(state):
    return any(value == '1' and state['power_type'].get(path.replace('/online', '/type'))
               in ('Mains', 'USB', 'USB_C', 'USB_PD', 'USB_PD_DRP')
               for path, value in state['power'].items())


def packets(sequence):
    from terrain_map import contract as C
    from operator_console.runtime_smoke import _terrain_map_packets
    status = C.parse_packet(_terrain_map_packets(sequence)[0])
    yield C.encode(replace(status, tile_count=3))
    for tx in range(3):
        heights = [12 if tx == 1 and 5 <= i//16 < 10 and 3 <= i%16 < 7
                   else -10 if tx == 2 and 3 <= i//16 < 7 and i%16 < 4 else 0
                   for i in range(256)]
        codes = [C.OBSTACLE if h > 0 else C.DITCH if h < 0 else C.FREE for h in heights]
        yield C.encode(C.TilePacket(sender_boot_id=status.sender_boot_id,
            sender_start_mono_ns=status.sender_start_mono_ns, sender_instance=status.sender_instance,
            map_epoch=0, sent_unix_ns=time.time_ns(), tx=tx, ty=0, version=1,
            tile_seq=sequence, cells=C.pack_cells(heights, codes, [0]*256)))


def source(ports):
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst
    from operator_console.runtime_smoke import _chassis_payload
    from operator_console.tests.test_terrain import payload
    Gst.init(None)
    pipelines = []
    for port in ports[:2]:
        pipeline = Gst.parse_launch(
            'videotestsrc is-live=true ! video/x-raw,width=1280,height=720,framerate=15/1 ! '
            'x264enc tune=zerolatency speed-preset=ultrafast key-int-max=15 threads=2 ! '
            'h264parse config-interval=-1 ! mpegtsmux alignment=7 ! '
            f'srtsink uri="srt://127.0.0.1:{port}?mode=listener" '
            'wait-for-connection=false sync=false async=false')
        pipeline.set_state(Gst.State.PLAYING)
        pipelines.append(pipeline)
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sequence = 0
    try:
        while True:
            sequence += 1
            for raw in packets(sequence):
                udp.sendto(raw, ('127.0.0.1', ports[2]))
            udp.sendto(json.dumps(_chassis_payload(sequence)).encode(), ('127.0.0.1', ports[3]))
            udp.sendto(payload(sequence=sequence), ('127.0.0.1', ports[4]))
            time.sleep(.08)
    finally:
        udp.close()
        for pipeline in pipelines:
            pipeline.set_state(Gst.State.NULL)


def run(args):
    # Match normal console import order before creating native pipelines.
    from operator_console.app import OperatorConsole, Gst, Gtk, GLib
    from operator_console.runtime_smoke import _free_udp_port
    import numpy as np
    Gst.init(None)
    assert Gtk.init_check()[0], 'run under xvfb-run or desktop display'
    ports = [_free_udp_port() for _ in range(5)]
    while len(set(ports)) != 5:
        ports = [_free_udp_port() for _ in range(5)]
    sender = subprocess.Popen([sys.executable, '-m', 'operator_console.tests.vision_benchmark', '--source', *map(str, ports)])
    callback_errors = []
    original_hook = sys.excepthook
    def callback_error(kind, value, traceback):
        if len(callback_errors) < 16:
            callback_errors.append(f'{kind.__name__}: {value}')
        original_hook(kind, value, traceback)
    sys.excepthook = callback_error
    window = None
    observer = None
    try:
        time.sleep(1)
        window = OperatorConsole('127.0.0.1', ports[0], ports[1], 0, 60, 0, ports[3],
            arm_telemetry_port=0, environment_telemetry_port=0, terrain_port=ports[4],
            terrain_map_port=ports[2], ops_port=1, input_source='TEST',
            ops_token_file='/nonexistent/vision-benchmark-no-token')
        window.show_all()
        if args.metrics_output:
            from operator_console.performance_probe import ConsolePerformanceProbe
            observer = ConsolePerformanceProbe(window,args.metrics_output)
        maps = window._vision_panel.maps
        counters = {'frames': 0, 'last': None, 'gap': 0., 'lateness': 0.}
        draws = {'count': 0, 'last': None, 'gap': 0.}
        def drawn(*_args):
            now = time.monotonic()
            draws['count'] += 1
            if draws['last'] is not None:
                draws['gap'] = max(draws['gap'], now-draws['last'])
            draws['last'] = now
            return False
        window._l515._video_widget.connect_after('draw', drawn)
        def frame(pad, info):
            now = time.monotonic()
            counters['frames'] += 1
            if counters['last'] is not None:
                counters['gap'] = max(counters['gap'], now-counters['last'])
            counters['last'] = now
            buffer = info.get_buffer()
            clock = window._l515._pipeline.get_clock()
            if clock and buffer.pts != Gst.CLOCK_TIME_NONE:
                late = (clock.get_time()-window._l515._pipeline.get_base_time()-buffer.pts)/Gst.SECOND
                counters['lateness'] = max(counters['lateness'], late)
            return Gst.PadProbeReturn.OK
        pad = window._l515._sink.get_static_pad('sink')
        probe = pad.add_probe(Gst.PadProbeType.BUFFER, frame)
        report = dict(source='synthetic 2x1280x720@15 SRT + 3 map tiles/12.5Hz + chassis/overlay UDP',
            scope='console process; source/encoders in separate child, excluded from CPU/RSS',
            latency_scope='sink buffer running-time lateness; NOT camera-to-screen latency',
            revision=subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip(),
            openblas_num_threads=os.environ.get('OPENBLAS_NUM_THREADS'),
            numpy_runtime=str(np.__version__), machine_before=machine(), phases=[], timeline=[],
            machine_samples=[])
        context = GLib.MainContext.default()
        def pump(duration):
            end = time.monotonic()+duration
            while time.monotonic() < end:
                while context.pending():
                    context.iteration(False)
                time.sleep(.001)
        for page in ('mission', 'vision', 'mission'):
            window._stack.set_visible_child_name(page)
            pump(3)
            before = counters['frames']; count = maps.displayed_count; renders = maps.worker.render_count
            before_draws = draws['count']
            counters['gap'] = counters['lateness'] = 0.
            draws['gap'] = 0.
            beats = []; ages = []; rss = []; render_ms = []
            machine_due = 0.
            def beat():
                beats.append(time.monotonic())
                return True
            def sample():
                nonlocal machine_due
                now = time.monotonic()
                if now >= machine_due:
                    report['machine_samples'].append(dict(t=now, page=page, **machine()))
                    machine_due = now+1
                age = window._l515.last_frame_age_s
                if age is not None:
                    ages.append(age)
                rss.append(int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')/1024)
                if maps.images:
                    render_ms.append(maps.images.render_ms)
                report['timeline'].append(dict(t=time.monotonic(), page=page,
                    selected_page=window._stack.get_visible_child_name(),
                    video_mapped=window._l515._video_widget.get_mapped(),
                    map_mapped=maps.top_slot.get_mapped(), map_active=maps.worker._active,
                    map_generation=maps.worker._generation, renders=maps.worker.render_count,
                    frames=counters['frames'], draws=draws['count'],
                    size=[window.get_allocated_width(), window.get_allocated_height()]))
                return True
            heartbeat = GLib.timeout_add(20, beat)
            sampler = GLib.timeout_add(200, sample)
            if args.inject_stall and page == 'vision':
                def stall():
                    time.sleep(.65)
                    return False
                GLib.timeout_add(1000, stall)
            start = time.monotonic(); cpu = time.process_time()
            duration = args.vision_seconds if page=='vision' and args.vision_seconds is not None else args.seconds
            pump(duration)
            elapsed = time.monotonic()-start
            GLib.source_remove(heartbeat); GLib.source_remove(sampler)
            phase = dict(page=page, duration_s=elapsed,
                cpu_percent=(time.process_time()-cpu)/elapsed*100,
                rss_kib_max=max(rss, default=0), rss_kib_end=rss[-1] if rss else None,
                rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                camera_sink_fps=(counters['frames']-before)/elapsed,
                camera_draw_hz=(draws['count']-before_draws)/elapsed,
                camera_draw_gap_s_max=max(draws['gap'], time.monotonic()-(draws['last'] or start)),
                camera_sink_gap_s_max=max(counters['gap'], time.monotonic()-(counters['last'] or start)),
                sink_lateness_s_max=counters['lateness'],
                camera_frame_age_s_max=max(ages, default=0),
                ui_gap_s_max=heartbeat_gap(start, time.monotonic(), beats),
                map_pair_hz=(maps.displayed_count-count)/elapsed,
                map_worker_render_count=maps.worker.render_count-renders,
                render_ms_mean=sum(render_ms)/len(render_ms) if render_ms else None,
                overlay_state=window._terrain_canvas.view().probe_state,
                map_state=maps.state,
                window_size=[window.get_allocated_width(), window.get_allocated_height()],
                map_sizes=[maps.settings.top_size, maps.settings.oblique_size],
                map_worker_error=maps.worker.last_error,
                machine=machine())
            report['phases'].append(phase)
            print(json.dumps(phase), flush=True)
        report['machine_after'] = machine()
        report['callback_errors'] = callback_errors
        Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
        pad.remove_probe(probe)
        assert not callback_errors, callback_errors
        if args.require_ac:
            assert all(ac_online(state) for state in
                       [report['machine_before'], report['machine_after'],
                        *report['machine_samples'], *(p['machine'] for p in report['phases'])]), \
                   'AC power missing or changed during measurement'
        assert sender.poll() is None, 'source died'
        assert report['phases'][1]['overlay_state'] == 'LIVE', 'overlay not live'
        assert report['phases'][1]['map_state'] == 'LIVE', 'map not live'
        assert all(p['camera_sink_fps'] > 12 for p in report['phases']), 'video throughput failed'
        assert all(p['ui_gap_s_max'] < .5 for p in report['phases']), 'GTK stall'
        assert all(p['camera_draw_gap_s_max'] < .5 for p in report['phases']), 'video draw stall'
        assert 1.5 < report['phases'][1]['map_pair_hz'] < 2.5, 'map rate failed'
        assert report['phases'][2]['map_worker_render_count'] == 0, 'hidden render work'
    finally:
        sys.excepthook = original_hook
        if observer:
            observer.close()
        if window:
            window.destroy()
        sender.terminate()
        try:
            sender.wait(timeout=5)
        except subprocess.TimeoutExpired:
            sender.kill(); sender.wait()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', nargs=5, type=int)
    parser.add_argument('--seconds', type=float, default=20)
    parser.add_argument('--output', default='/tmp/vision-performance.json')
    parser.add_argument('--inject-stall', action='store_true',
                        help='negative control: deliberately block GTK for 650 ms')
    parser.add_argument('--require-ac', action='store_true', help='reject runs without online AC/USB power')
    parser.add_argument('--vision-seconds', type=float, help='separate visible vision duration for soak tests')
    parser.add_argument('--metrics-output', type=Path, help='observe the same events as the real-console monitor')
    args = parser.parse_args()
    if args.source:
        source(args.source)
    else:
        if not 5 <= args.seconds <= 3600:
            parser.error('--seconds must be 5..3600 per phase')
        if args.vision_seconds is not None and not 5 <= args.vision_seconds <= 3600:
            parser.error('--vision-seconds must be 5..3600')
        run(args)
