"""Optional observation of existing console events; never creates a receiver/decoder."""
from collections import deque
import json
import os
from pathlib import Path
import tempfile
import threading
import time


class Events:
    def __init__(self):
        self.count = 0
        self.first = self.last = None
        self.max_gap = 0.

    def add(self, now):
        if self.last is not None:
            self.max_gap = max(self.max_gap, now-self.last)
        else:
            self.first = now
        self.last = now
        self.count += 1

    def snapshot(self, end):
        return dict(count=self.count,
            hz=(self.count-1)/(self.last-self.first) if self.count>1 and self.last>self.first else None,
            max_gap_s=max(self.max_gap,end-self.last) if self.last is not None else None)


class ConsolePerformanceProbe:
    """GTK-thread lifecycle, constant-time streaming callbacks, bounded history."""
    def __init__(self, console, output):
        from gi.repository import GLib, Gst
        self.GLib, self.Gst = GLib, Gst
        self.console, self.output = console, Path(output).resolve()
        if self.output.exists() or not self.output.parent.is_dir():
            raise ValueError('metrics output must be a new file in an existing directory')
        self.lock = threading.Lock()
        self.closed = False
        self.error = None
        self.started = time.monotonic()
        self.segments = deque(maxlen=64)
        self.dropped = 0
        self.current = None
        self.connections = []
        self.probes = []
        self.last_rasters = {}
        self.stall_events = deque(maxlen=64)
        self.visibility_events = deque(maxlen=64)
        self.visibility_dropped = 0
        self._last_visibility = None
        self._visibility()
        self._page()
        self.connections.append((console._stack,console._stack.connect('notify::visible-child-name',self._page)))
        for name, panel in (('l515',console._l515),('d435',console._d435)):
            pad = panel._sink.get_static_pad('sink')
            probe = pad.add_probe(Gst.PadProbeType.BUFFER,self._buffer,name)
            self.probes.append((pad,probe))
            widget = panel._video_widget
            self.connections.append((widget,widget.connect_after('draw',self._draw,name)))
        for name, canvas in (('top',console._vision_panel.maps.top),('oblique',console._vision_panel.maps.oblique)):
            self.connections.append((canvas,canvas.connect_after('draw',self._map_draw,name)))
        self.beat_id = GLib.timeout_add(20,self._beat)
        self.save_id = GLib.timeout_add(1000,self._save)
        self._save()

    def _page(self,*_):
        now = time.monotonic()
        with self.lock:
            if self.current is not None:
                self.current['end'] = now
            if len(self.segments)==self.segments.maxlen:
                self.dropped += 1
            self.current = dict(page=self.console._stack.get_visible_child_name(),start=now,end=None,
                events={name:Events() for name in ('heartbeat','l515_sink','d435_sink',
                                                  'l515_draw','d435_draw','top_images','oblique_images')})
            self.segments.append(self.current)

    def _event(self,name):
        with self.lock:
            if not self.closed:
                now = time.monotonic()
                events = self.current['events'][name]
                if name in ('heartbeat','l515_draw') and events.last is not None and now-events.last>.5:
                    sink_last = self.current['events']['l515_sink'].last
                    clock = self.console.get_frame_clock()
                    self.stall_events.append(dict(event=name,t=now,gap_s=now-events.last,
                        visibility=self._visibility(),sink_age_s=None if sink_last is None else now-sink_last,
                        gtk_frame_counter=None if clock is None else clock.get_frame_counter()))
                events.add(now)

    def _buffer(self,_pad,_info,name):
        self._event(name+'_sink')
        return self.Gst.PadProbeReturn.OK

    def _draw(self,_widget,_cr,name):
        self._event(name+'_draw')
        return False

    def _map_draw(self,widget,_cr,name):
        raster = widget.raster
        if raster is not None and raster is not self.last_rasters.get(name):
            self.last_rasters[name] = raster  # one retained buffer; prevents identity reuse
            self._event(name+'_images')
        return False

    def _visibility(self):
        window = self.console.get_window()
        display = self.console.get_display()
        state = dict(mapped=bool(self.console.get_mapped()), visible=bool(self.console.get_visible()),
                     gdk_window_state=None if window is None else int(window.get_state()),
                     backend=display.__gtype__.name)
        if state!=self._last_visibility:
            if len(self.visibility_events)==self.visibility_events.maxlen:
                self.visibility_dropped += 1
            self.visibility_events.append(dict(t=time.monotonic(),**state))
            self._last_visibility=state
        return state

    def _beat(self):
        self._visibility()
        self._event('heartbeat')
        return not self.closed

    def snapshot(self):
        now = time.monotonic()
        with self.lock:
            segments = [dict(page=s['page'],duration_s=(s['end'] or now)-s['start'],
                events={name:events.snapshot(s['end'] or now) for name,events in s['events'].items()})
                for s in self.segments]
        view = self.console._vision_panel.attitude_slot.view
        return dict(console_pid=os.getpid(),status='closed' if self.closed else 'running',
            observed_mono_s=now,visibility=self._visibility(),
            visibility_events=list(self.visibility_events),visibility_events_dropped=self.visibility_dropped,
            stall_events=list(self.stall_events),
            video_states={name:panel.health_state() for name,panel in
                          (('l515',self.console._l515),('d435',self.console._d435))},
            map_state=self.console._vision_panel.maps.state,
            telemetry_state=None if view is None else view.link_state,
            elapsed_s=now-self.started,hardware_verified=False,
            scope='existing sink/draw/heartbeat events; not optical camera-to-screen latency',
            missing_events='hz/max_gap are null until observed; map images count distinct painted rasters',
            segments=segments,dropped_segments=self.dropped,write_error=self.error)

    def _save(self):
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w',encoding='utf-8',dir=self.output.parent,
                prefix=self.output.name+'.',delete=False) as stream:
                temporary = Path(stream.name)
                json.dump(self.snapshot(),stream,allow_nan=False)
                stream.write('\n')
            os.replace(temporary,self.output)
            self.error = None
        except OSError as exc:
            self.error = str(exc)
        finally:
            if temporary is not None:
                temporary.unlink(missing_ok=True)
        return not self.closed

    def close(self):
        if self.closed:
            return
        with self.lock:
            self.current['end'] = time.monotonic()
            self.closed = True
        self.GLib.source_remove(self.beat_id)
        self.GLib.source_remove(self.save_id)
        for pad,probe in self.probes:
            pad.remove_probe(probe)
        for widget,connection in self.connections:
            widget.disconnect(connection)
        self.last_rasters.clear()
        self._save()
