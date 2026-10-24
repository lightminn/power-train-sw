"""Explicit synthetic performance experiment; never opens a camera or drives motors.

Run in an isolated display with system Python; source/encoder CPU is excluded.
This is a development measurement, not Gateway/L515 acceptance.
"""
import argparse
from collections import deque
from dataclasses import replace
import json
import os
from pathlib import Path
import resource
import socket
import subprocess
import sys
import time
from operator_console.performance_probe import Events
from operator_console.smoke_timing import callback_gap as heartbeat_gap


class Summary:
    def __init__(self):
        self.count = 0
        self.total = 0.
        self.maximum = None
        self.last = None

    def add(self, value):
        self.count += 1
        self.total += value
        self.maximum = value if self.maximum is None else max(self.maximum, value)
        self.last = value

    @property
    def mean(self):
        return self.total/self.count if self.count else None


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
    import signal
    from operator_console.runtime_smoke import _chassis_payload
    from terrain_overlay.tests.test_runtime import ready
    from terrain_overlay.tests.scenes import default_scene
    sources = []
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    signal.signal(signal.SIGTERM,lambda *_: sys.exit(0))
    try:
        overlay = ready()
        overlay.on_depth(default_scene(),111,10.)
        overlay._clock.t = 10.03
        for port in ports[:2]:
            sources.append(subprocess.Popen([
                'gst-launch-1.0','-q','videotestsrc','is-live=true','!',
                'video/x-raw,width=1280,height=720,framerate=15/1','!',
                'x264enc','tune=zerolatency','speed-preset=ultrafast','key-int-max=15','threads=2','!',
                'h264parse','config-interval=-1','!','mpegtsmux','alignment=7','!',
                'srtsink',f'uri=srt://127.0.0.1:{port}?mode=listener',
                'wait-for-connection=false','sync=false','async=false']))
        sequence = 0
        while True:
            assert all(src.poll() is None for src in sources), 'SRT fixture source died'
            sequence += 1
            for raw in packets(sequence):
                udp.sendto(raw, ('127.0.0.1', ports[2]))
            udp.sendto(json.dumps(_chassis_payload(sequence)).encode(), ('127.0.0.1', ports[3]))
            udp.sendto(overlay.tick(), ('127.0.0.1', ports[4]))
            time.sleep(.08)
    finally:
        udp.close()
        for src in sources:
            src.terminate()
        for src in sources:
            try:
                src.wait(timeout=3)
            except subprocess.TimeoutExpired:
                src.kill();src.wait()


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
    sender = subprocess.Popen([sys.executable, '-m', 'operator_console.tests.vision_benchmark', '--source', *map(str, ports)], cwd=args.source_repo)
    callback_errors = []
    original_hook = sys.excepthook
    def callback_error(kind, value, traceback):
        if len(callback_errors) < 16:
            callback_errors.append(f'{kind.__name__}: {value}')
        original_hook(kind, value, traceback)
    sys.excepthook = callback_error
    window = None
    observer = None
    samples = None
    if args.trace_memory:
        import tracemalloc
        tracemalloc.start(3)
    try:
        samples = open(str(args.output)+'.samples.jsonl', 'x', encoding='utf-8')
        deadline=time.monotonic()+10
        while True:
            assert sender.poll() is None, f'synthetic source exited: {sender.returncode}'
            bound={int(line.split()[1].split(':')[1],16)
                   for line in Path('/proc/net/udp').read_text().splitlines()[1:]}
            if all(p in bound for p in ports[:2]):
                break
            assert time.monotonic()<deadline, 'synthetic listeners not ready'
            time.sleep(.02)
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
            numpy_runtime=str(np.__version__), machine_before=machine(), phases=[], timeline=deque(maxlen=128),
            machine_samples=deque(maxlen=32), samples_file=str(args.output)+'.samples.jsonl')
        report['ac_entire_run'] = ac_online(report['machine_before'])
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
            beats = Events(); ages = Summary(); rss = Summary(); render_ms = Summary()
            machine_due = 0.
            def beat():
                beats.add(time.monotonic())
                return True
            def sample():
                nonlocal machine_due
                now = time.monotonic()
                if now >= machine_due:
                    environment = dict(t=now, page=page, **machine())
                    report['ac_entire_run'] = report['ac_entire_run'] and ac_online(environment)
                    report['machine_samples'].append(environment)
                    samples.write(json.dumps(dict(kind='machine', **environment))+'\n')
                    if args.trace_memory and int(now-start)%30==0:
                        current, peak = tracemalloc.get_traced_memory()
                        snapshot = tracemalloc.take_snapshot()
                        samples.write(json.dumps(dict(kind='python-memory',t=now,current=current,peak=peak,
                            top=[str(item) for item in snapshot.statistics('lineno')[:15]]))+'\n')
                        del snapshot
                    machine_due = now+1
                age = window._l515.last_frame_age_s
                if age is not None:
                    ages.add(age)
                rss.add(int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')/1024)
                if maps.images:
                    render_ms.add(maps.images.render_ms)
                row = dict(t=time.monotonic(), page=page, rss_kib=rss.last,
                    selected_page=window._stack.get_visible_child_name(),
                    video_mapped=window._l515._video_widget.get_mapped(),
                    map_mapped=maps.top_slot.get_mapped(), map_active=maps.worker._active,
                    map_generation=maps.worker._generation, renders=maps.worker.render_count,
                    frames=counters['frames'], draws=draws['count'], window_mapped=window.get_mapped(),
                    backend=window.get_display().__gtype__.name,
                    size=[window.get_allocated_width(), window.get_allocated_height()])
                report['timeline'].append(row)
                samples.write(json.dumps(dict(kind='timeline', **row))+'\n')
                samples.flush()
                return True
            heartbeat = GLib.timeout_add(20, beat)
            sampler = GLib.timeout_add(200, sample)
            if args.inject_stall and page == 'vision':
                def stall():
                    time.sleep(.65)
                    return False
                GLib.timeout_add(1000, stall)
            if args.hide_seconds and page=='vision':
                def hide():
                    window.hide()
                    GLib.timeout_add(round(args.hide_seconds*1000),lambda: window.show_all() or False)
                    return False
                GLib.timeout_add(1000,hide)
            start = time.monotonic(); cpu = time.process_time()
            duration = args.vision_seconds if page=='vision' and args.vision_seconds is not None else args.seconds
            pump(duration)
            elapsed = time.monotonic()-start
            GLib.source_remove(heartbeat); GLib.source_remove(sampler)
            phase = dict(page=page, duration_s=elapsed,
                cpu_percent=(time.process_time()-cpu)/elapsed*100,
                rss_kib_max=rss.maximum, rss_kib_end=rss.last,
                rss_peak_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                camera_sink_fps=(counters['frames']-before)/elapsed,
                camera_draw_hz=(draws['count']-before_draws)/elapsed,
                camera_draw_gap_s_max=max(draws['gap'], time.monotonic()-(draws['last'] or start)),
                camera_sink_gap_s_max=max(counters['gap'], time.monotonic()-(counters['last'] or start)),
                sink_lateness_s_max=counters['lateness'],
                camera_frame_age_s_max=ages.maximum,
                ui_gap_s_max=max(beats.first-start if beats.first is not None else elapsed,
                    beats.snapshot(time.monotonic())['max_gap_s'] or elapsed),
                map_pair_hz=(maps.displayed_count-count)/elapsed,
                map_worker_render_count=maps.worker.render_count-renders,
                render_ms_mean=render_ms.mean,
                overlay_state=window._terrain_canvas.view().probe_state,
                map_state=maps.state,
                window_size=[window.get_allocated_width(), window.get_allocated_height()],
                map_sizes=[maps.settings.top_size, maps.settings.oblique_size],
                map_worker_error=maps.worker.last_error,
                machine=machine())
            report['phases'].append(phase)
            print(json.dumps(phase), flush=True)
        if args.allocator_probe:
            import ctypes
            import gc
            def rss_now():
                return int(Path('/proc/self/statm').read_text().split()[1])*os.sysconf('SC_PAGE_SIZE')/1024
            diagnostic=dict(before_gc_kib=rss_now(),scope='development-only; no GC/trim in console product')
            diagnostic['gc_unreachable']=gc.collect()
            diagnostic['after_gc_kib']=rss_now()
            trim=ctypes.CDLL(None).malloc_trim
            trim.argtypes=[ctypes.c_size_t];trim.restype=ctypes.c_int
            diagnostic['trim_result']=trim(0)
            diagnostic['after_trim_kib']=rss_now()
            report['allocator_diagnostic']=diagnostic
        report['machine_after'] = machine()
        report['callback_errors'] = callback_errors
        report['timeline'] = list(report['timeline'])
        report['machine_samples'] = list(report['machine_samples'])
        Path(args.output).write_text(json.dumps(report, indent=2)+'\n')
        pad.remove_probe(probe)
        assert not callback_errors, callback_errors
        if args.require_ac:
            assert report['ac_entire_run'] and all(ac_online(state) for state in
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
        if samples:
            samples.close()
        if args.trace_memory:
            tracemalloc.stop()
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
    parser.add_argument('--source-repo', type=Path, help='use identical source fixtures for baseline comparisons')
    parser.add_argument('--hide-seconds', type=float, default=0, help='diagnostic negative control: unmap window')
    parser.add_argument('--allocator-probe', action='store_true', help='diagnose retained malloc arenas after measurements only')
    parser.add_argument('--trace-memory', action='store_true', help='diagnostic run; tracing changes timings')
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
