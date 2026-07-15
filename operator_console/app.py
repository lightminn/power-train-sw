#!/usr/bin/env python3
"""GTK/GStreamer operator console — first slice: D435i raw SRT monitoring.

This program is intentionally read-only.  It does not open a camera, publish
ROS commands, or communicate with CAN.  The robot remains the SRT listener;
the operator laptop is an SRT caller.
"""
from __future__ import annotations

import argparse
import time
from collections.abc import Callable

from operator_console.pipelines import pipeline_description, srt_uri

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
gi.require_version("Gst", "1.0")
from gi.repository import Gdk, GLib, Gst, Gtk, Pango  # noqa: E402

from .metadata import LatestMetadataReceiver, MetadataFrame
from .telemetry import LatestTelemetryReceiver, TelemetrySnapshot


class MetadataCanvas(Gtk.DrawingArea):
    """Transparent D435 overlay drawn from latest-only UDP metadata."""
    def __init__(self, receiver: LatestMetadataReceiver) -> None:
        super().__init__()
        self._receiver = receiver
        self.connect("draw", self._on_draw)

    def _on_draw(self, _widget: Gtk.DrawingArea, context: object) -> bool:
        frame = self._receiver.latest()
        if frame is None or time.monotonic() - frame.received_monotonic_s > 0.25:
            return False
        allocation = self.get_allocation()
        scale = min(allocation.width / frame.width, allocation.height / frame.height)
        offset_x = (allocation.width - frame.width * scale) / 2.0
        offset_y = (allocation.height - frame.height * scale) / 2.0
        context.set_source_rgba(0.0, 1.0, 0.2, 0.95)
        context.set_line_width(2.0)
        context.select_font_face("Sans", 0, 1)
        context.set_font_size(15.0)
        for detection in frame.detections:
            x, y, width, height = detection.bbox_xywh
            context.rectangle(offset_x + x * scale, offset_y + y * scale,
                              width * scale, height * scale)
            context.stroke()
            text = f"{detection.class_name} {detection.confidence:.2f}"
            if detection.position_m is not None:
                text += f"  z={detection.position_m[2]:.2f}m"
            context.move_to(offset_x + x * scale, max(16.0, offset_y + y * scale - 4.0))
            context.show_text(text)
        return False


class EventLog(Gtk.Frame):
    """Bounded, read-only timeline for observable channel state transitions."""

    _MAX_LINES = 100

    def __init__(self) -> None:
        super().__init__(label="Event timeline (read-only)")
        self._view = Gtk.TextView(editable=False, cursor_visible=False, monospace=True)
        self._view.set_wrap_mode(Gtk.WrapMode.WORD_CHAR)
        self._buffer = self._view.get_buffer()
        scroll = Gtk.ScrolledWindow()
        scroll.set_min_content_height(90)
        scroll.add(self._view)
        self.add(scroll)

    def add_event(self, source: str, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        end = self._buffer.get_end_iter()
        self._buffer.insert(end, f"[{stamp}] {source}: {message}\n")
        line_count = self._buffer.get_line_count()
        if line_count > self._MAX_LINES:
            start = self._buffer.get_start_iter()
            trim_to = self._buffer.get_iter_at_line(line_count - self._MAX_LINES)
            self._buffer.delete(start, trim_to)
        self._view.scroll_to_iter(self._buffer.get_end_iter(), 0.0, False, 0.0, 1.0)


class VideoPanel(Gtk.Box):
    """One read-only SRT receiver panel embedded in the console."""

    def __init__(self, name: str, host: str, port: int, latency_ms: int,
                 metadata_receiver: LatestMetadataReceiver | None = None,
                 event_sink: Callable[[str, str], None] | None = None) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL)
        self._name = name
        self._event_sink = event_sink
        self._pipeline = Gst.parse_launch(pipeline_description(host, port, latency_ms))
        self._sink = self._pipeline.get_by_name("video_sink")
        self._video_widget = self._sink.get_property("widget")
        self._video_widget.set_hexpand(True)
        self._video_widget.set_vexpand(True)
        self._video_widget.connect("realize", self._on_realize)
        self._status = Gtk.Label(label=f"{name}: connecting")
        self._fps = Gtk.Label(label="Display FPS: waiting")
        self._detail = Gtk.Label(label=f"SRT caller → {host}:{port}, latency {latency_ms} ms")
        self._detail.set_xalign(0.0)
        self._detail.set_ellipsize(Pango.EllipsizeMode.END)
        header = Gtk.Box(spacing=10)
        header.set_border_width(8)
        header.pack_start(self._status, False, False, 0)
        header.pack_start(self._fps, False, False, 0)
        header.pack_start(self._detail, True, True, 0)
        self.pack_start(header, False, False, 0)
        if metadata_receiver is None:
            self.pack_start(self._video_widget, True, True, 0)
        else:
            overlay = Gtk.Overlay()
            overlay.add(self._video_widget)
            canvas = MetadataCanvas(metadata_receiver)
            canvas.set_hexpand(True)
            canvas.set_vexpand(True)
            overlay.add_overlay(canvas)
            self._metadata_canvas = canvas
            self.pack_start(overlay, True, True, 0)
            self._metadata_receiver = metadata_receiver
            self._metadata_label = Gtk.Label(label="YOLO: waiting for UDP :5003")
            self._metadata_state: tuple[str, int] | None = None
            header.pack_start(self._metadata_label, False, False, 0)
            GLib.timeout_add(100, self._refresh_metadata_status)

        self._stopped = False
        self._retry_source_id: int | None = None
        self._frames = 0
        self._sample_start = time.monotonic()
        self._last_frame_monotonic: float | None = None
        self._last_fps: float | None = None
        self._pipeline_live = False
        self._reconnects = 0
        self._freshness_state = "connecting"
        sink_pad = self._sink.get_static_pad("sink")
        sink_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_video_buffer)
        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)
        bus.enable_sync_message_emission()
        bus.connect("sync-message::element", self._on_sync_message)
        GLib.timeout_add(200, self._refresh_video_health)

    def _on_realize(self, _area: Gtk.DrawingArea) -> None:
        self._pipeline.set_state(Gst.State.PLAYING)

    def _on_sync_message(self, _bus: Gst.Bus, message: Gst.Message) -> None:
        # ``gtksink`` owns a GTK widget, so it never requests a separate
        # X11/Wayland surface through ``prepare-window-handle``.
        del message

    def _on_video_buffer(self, _pad: Gst.Pad, _info: Gst.PadProbeInfo) -> Gst.PadProbeReturn:
        self._frames += 1
        now = time.monotonic()
        self._last_frame_monotonic = now
        elapsed = now - self._sample_start
        if elapsed >= 1.0:
            fps = self._frames / elapsed
            self._frames = 0
            self._sample_start = now
            self._last_fps = fps
            GLib.idle_add(self._fps.set_text, f"Display FPS: {fps:.1f}")
        return Gst.PadProbeReturn.OK

    def _on_bus_message(self, _bus: Gst.Bus, message: Gst.Message) -> None:
        if message.type == Gst.MessageType.ERROR:
            error, detail = message.parse_error()
            self._pipeline_live = False
            self._status.set_text(f"{self._name}: reconnecting / error")
            self._detail.set_text(str(error if not detail else f"{error}: {detail}"))
            self._schedule_reconnect()
        elif message.type == Gst.MessageType.EOS:
            self._pipeline_live = False
            self._status.set_text(f"{self._name}: reconnecting / end of stream")
            self._schedule_reconnect()
        elif message.type == Gst.MessageType.WARNING:
            warning, _detail = message.parse_warning()
            self._status.set_text(f"{self._name}: reconnecting ({warning})")
        elif message.type == Gst.MessageType.STATE_CHANGED and message.src == self._pipeline:
            _old, new, _pending = message.parse_state_changed()
            if new == Gst.State.PLAYING:
                self._pipeline_live = True
                self._freshness_state = "waiting"
                self._emit_event("SRT pipeline connected; waiting for first frame")

    def _refresh_video_health(self) -> bool:
        if self._stopped:
            return False
        if not self._pipeline_live:
            return True
        if self._last_frame_monotonic is None:
            self._status.set_text(f"{self._name}: waiting for first frame")
            return True
        age_ms = (time.monotonic() - self._last_frame_monotonic) * 1000.0
        if age_ms > 1000.0:
            if self._freshness_state != "stale":
                self._emit_event("frame became stale (>1000 ms)")
                self._freshness_state = "stale"
            self._status.set_text(
                f"{self._name}: STALE ({age_ms:.0f} ms · reconnects {self._reconnects})")
        else:
            if self._freshness_state != "live":
                self._emit_event("frame flow live")
                self._freshness_state = "live"
            self._status.set_text(
                f"{self._name}: live · age {age_ms:.0f} ms · reconnects {self._reconnects}")
        return True

    def _emit_event(self, message: str) -> None:
        if self._event_sink is not None:
            self._event_sink(self._name, message)

    def health_state(self) -> str:
        if not self._pipeline_live:
            return "CONNECTING"
        if self._last_frame_monotonic is None:
            return "WAITING"
        return "STALE" if time.monotonic() - self._last_frame_monotonic > 1.0 else "LIVE"

    def _schedule_reconnect(self) -> None:
        if self._stopped or self._retry_source_id is not None:
            return
        self._reconnects += 1
        self._emit_event(f"SRT reconnect scheduled ({self._reconnects})")
        self._retry_source_id = GLib.timeout_add(1000, self._restart_pipeline)

    def _restart_pipeline(self) -> bool:
        self._retry_source_id = None
        if self._stopped:
            return False
        self._pipeline.set_state(Gst.State.NULL)
        self._pipeline.set_state(Gst.State.PLAYING)
        return False

    def _refresh_metadata_status(self) -> bool:
        receiver = getattr(self, "_metadata_receiver", None)
        if receiver is None:
            return False
        frame: MetadataFrame | None = receiver.latest()
        if frame is None:
            self._metadata_label.set_text("YOLO: waiting for UDP :5003")
            self._report_metadata_state("waiting", 0)
        else:
            age_ms = (time.monotonic() - frame.received_monotonic_s) * 1000.0
            if age_ms > 250.0:
                self._metadata_label.set_text(f"YOLO: stale ({age_ms:.0f} ms)")
                self._report_metadata_state("stale", len(frame.detections))
            else:
                summary = []
                for detection in frame.detections[:3]:
                    text = f"{detection.class_name} {detection.confidence:.2f}"
                    if detection.position_m is not None:
                        text += f" {detection.position_m[2]:.2f}m"
                    summary.append(text)
                detail = " · ".join(summary) if summary else "no detections"
                self._metadata_label.set_text(
                    f"YOLO: {len(frame.detections)} objects · {age_ms:.0f} ms · {detail}"
                )
                self._report_metadata_state("live", len(frame.detections))
        self._metadata_canvas.queue_draw()
        return True

    def _report_metadata_state(self, state: str, count: int) -> None:
        key = (state, count)
        if key == self._metadata_state:
            return
        self._metadata_state = key
        if state == "live":
            self._emit_event(f"YOLO metadata live ({count} objects)")
        elif state == "stale":
            self._emit_event(f"YOLO metadata stale ({count} objects)")
        else:
            self._emit_event("YOLO metadata waiting")

    def stop(self) -> None:
        self._stopped = True
        if self._retry_source_id is not None:
            GLib.source_remove(self._retry_source_id)
            self._retry_source_id = None
        self._pipeline.set_state(Gst.State.NULL)


class TelemetryPanel(Gtk.Frame):
    """Read-only latest snapshot display; unavailable is a valid visible state."""
    def __init__(self, receiver: LatestTelemetryReceiver, port: int,
                 event_sink: Callable[[str, str], None] | None = None) -> None:
        super().__init__(label="Robot telemetry (read-only)")
        self._receiver = receiver
        self._port = port
        self._event_sink = event_sink
        self._power_health_seen = False
        self._power_health_key: tuple[int | None, int | None] | None = None
        self._rs485_key: tuple[str, int | None, str] | None = None
        self._telemetry_link_state = "waiting"
        self._last_sequence: int | None = None
        self._labels: dict[str, Gtk.Label] = {}
        grid = Gtk.Grid(column_spacing=12, row_spacing=7, margin=10)
        rows = (("link", "Link"), ("rs485", "RS485"), ("power", "PDIST80B"))
        for row, (key, title) in enumerate(rows):
            name = Gtk.Label(label=title)
            name.set_xalign(0.0)
            value = Gtk.Label(label="UNAVAILABLE")
            value.set_xalign(0.0)
            value.set_line_wrap(True)
            grid.attach(name, 0, row, 1, 1)
            grid.attach(value, 1, row, 1, 1)
            self._labels[key] = value
        self.add(grid)
        GLib.timeout_add(200, self._refresh)

    @staticmethod
    def _number(value: float | None, suffix: str) -> str:
        return "N/A" if value is None else f"{value:.2f} {suffix}"

    @staticmethod
    def _rss(value: int | None) -> str:
        return "N/A" if value is None else f"{value / (1024 * 1024):.1f} MiB"

    @staticmethod
    def _l515_ros_rates_text(rates: tuple[tuple[str, float], ...]) -> str:
        if not rates:
            return "N/A"
        return ", ".join(
            "{} {:.1f} Hz".format(topic.rsplit("/", 1)[-1], rate)
            for topic, rate in rates
        )

    @staticmethod
    def _hex(value: int | None) -> str:
        return "N/A" if value is None else f"0x{value:02X}"

    @staticmethod
    def _power_health_text(battery_flags: int | None, protection_flags: int | None) -> str:
        if protection_flags not in (None, 0):
            return f"PROTECTION ALERT {protection_flags:#04x}"
        if battery_flags not in (None, 0):
            return f"BATTERY WARNING {battery_flags:#04x}"
        if battery_flags == 0 and protection_flags == 0:
            return "NORMAL"
        return "UNAVAILABLE"

    def _report_power_health(self, battery_flags: int | None, protection_flags: int | None) -> None:
        key = (battery_flags, protection_flags)
        if self._power_health_seen and key == self._power_health_key:
            return
        self._power_health_seen = True
        self._power_health_key = key
        if self._event_sink is not None:
            self._event_sink("PDIST80B", self._power_health_text(battery_flags, protection_flags))

    def _emit_event(self, message: str) -> None:
        if self._event_sink is not None:
            self._event_sink("Telemetry", message)

    def _report_rs485(self, snapshot: TelemetrySnapshot) -> None:
        key = (snapshot.rs485_state, snapshot.rs485_consecutive_failures,
               snapshot.rs485_detail)
        if key == self._rs485_key:
            return
        self._rs485_key = key
        if self._event_sink is not None:
            failures = ("N/A" if snapshot.rs485_consecutive_failures is None
                        else str(snapshot.rs485_consecutive_failures))
            detail = snapshot.rs485_detail or "-"
            self._event_sink("RS485", f"{snapshot.rs485_state} · failures {failures} · {detail}")

    def _refresh(self) -> bool:
        snapshot: TelemetrySnapshot | None = self._receiver.latest()
        if snapshot is None:
            self._labels["link"].set_text(f"waiting for UDP :{self._port}")
            return True
        age_ms = (time.monotonic() - snapshot.received_monotonic_s) * 1000.0
        if age_ms > 1000.0:
            self._labels["link"].set_text(f"STALE ({age_ms:.0f} ms)")
            if self._telemetry_link_state != "stale":
                self._telemetry_link_state = "stale"
                self._emit_event(f"snapshot stale ({age_ms:.0f} ms)")
            return True
        if self._telemetry_link_state != "live":
            self._telemetry_link_state = "live"
            self._emit_event("snapshot live")
        if self._last_sequence is not None and snapshot.sequence != self._last_sequence:
            expected = self._last_sequence + 1
            if snapshot.sequence != expected:
                self._emit_event(f"sequence gap {self._last_sequence} → {snapshot.sequence}")
        self._last_sequence = snapshot.sequence
        self._labels["link"].set_text(f"LIVE · seq {snapshot.sequence} · {age_ms:.0f} ms")
        failures = "N/A" if snapshot.rs485_consecutive_failures is None else str(
            snapshot.rs485_consecutive_failures)
        detail = snapshot.rs485_detail or "-"
        self._labels["rs485"].set_text(
            f"{snapshot.rs485_state} · failures {failures}\n{detail}"
        )
        self._report_rs485(snapshot)
        self._labels["power"].set_text(
            f"{self._number(snapshot.voltage_v, 'V')} · {self._number(snapshot.current_a, 'A')} · "
            f"{self._number(snapshot.power_w, 'W')}\n"
            f"SOC {'N/A' if snapshot.pdist_soc_percent is None else f'{snapshot.pdist_soc_percent}%'} · "
            f"charge {self._number(snapshot.pdist_charge_current_a, 'A')} · "
            f"battery {self._hex(snapshot.pdist_battery_flags)} · "
            f"protection {self._hex(snapshot.pdist_protection_flags)}\n"
            f"{self._power_health_text(snapshot.pdist_battery_flags, snapshot.pdist_protection_flags)}"
        )
        self._report_power_health(snapshot.pdist_battery_flags, snapshot.pdist_protection_flags)
        return True


class ChassisTelemetryPanel(Gtk.Frame):
    """Read-only chassis owner snapshot; never probes CAN directly."""
    def __init__(self, receiver: LatestTelemetryReceiver, port: int,
                 event_sink: Callable[[str, str], None] | None = None) -> None:
        super().__init__(label="Chassis telemetry (read-only)")
        self._receiver = receiver
        self._port = port
        self._event_sink = event_sink
        self._wheel_health_key: tuple[int, int, int] | None = None
        self._labels: dict[str, Gtk.Label] = {}
        grid = Gtk.Grid(column_spacing=12, row_spacing=7, margin=10)
        for row, (key, title) in enumerate((("link", "Link"), ("odom", "Odometry"),
                                             ("pose", "Pose x / y / yaw"),
                                             ("drive", "Drive"), ("safety", "Safety / US-100"),
                                             ("feedback", "Wheel feedback"), ("wheels", "Per-wheel"),
                                             ("can", "CAN"), ("l515", "L515 Gateway"))):
            name = Gtk.Label(label=title)
            name.set_xalign(0.0)
            value = Gtk.Label(label="UNAVAILABLE")
            value.set_xalign(0.0)
            value.set_line_wrap(True)
            grid.attach(name, 0, row, 1, 1)
            grid.attach(value, 1, row, 1, 1)
            self._labels[key] = value
        self.add(grid)
        GLib.timeout_add(200, self._refresh)

    @staticmethod
    def _number(value: float | None, suffix: str) -> str:
        return "N/A" if value is None else f"{value:.2f} {suffix}"

    def _refresh(self) -> bool:
        snapshot = self._receiver.latest()
        if snapshot is None:
            self._labels["link"].set_text(f"waiting for UDP :{self._port}")
            return True
        age_ms = (time.monotonic() - snapshot.received_monotonic_s) * 1000.0
        if age_ms > 1000.0:
            self._labels["link"].set_text(f"STALE ({age_ms:.0f} ms)")
            return True
        self._labels["link"].set_text(f"LIVE · seq {snapshot.sequence} · {age_ms:.0f} ms")
        self._labels["odom"].set_text(snapshot.odometry_source)
        self._labels["pose"].set_text(
            f"{self._number(snapshot.x_m, 'm')} / {self._number(snapshot.y_m, 'm')} / "
            f"{self._number(snapshot.yaw_rad, 'rad')}"
        )
        self._labels["drive"].set_text(snapshot.drive_state)
        if snapshot.safety_status == "unavailable":
            self._labels["safety"].set_text("UNAVAILABLE")
        else:
            distance = self._number(snapshot.safety_distance_mm, "mm")
            estop = "ESTOP" if snapshot.safety_estop_required else "clear"
            failures = ("N/A" if snapshot.safety_consecutive_failures is None
                        else str(snapshot.safety_consecutive_failures))
            detail = snapshot.safety_detail or "-"
            self._labels["safety"].set_text(
                f"{snapshot.safety_status} · {distance} · {estop} · failures {failures}\n{detail}"
            )
        if snapshot.wheel_count is None:
            self._labels["feedback"].set_text("UNAVAILABLE")
        else:
            self._labels["feedback"].set_text(
                f"wheels {snapshot.wheel_count} · fault {snapshot.wheel_fault_count or 0} · "
                f"stale {snapshot.wheel_stale_count or 0} · "
                f"axis {snapshot.wheel_axis_error_count or 0} · "
                f"steer {snapshot.wheel_steer_fault_count or 0}"
            )
            health_key = (snapshot.wheel_stale_count or 0,
                          snapshot.wheel_axis_error_count or 0,
                          snapshot.wheel_steer_fault_count or 0)
            if health_key != self._wheel_health_key:
                self._wheel_health_key = health_key
                if self._event_sink is not None:
                    self._event_sink(
                        "WHEELS",
                        f"stale {health_key[0]} · axis error {health_key[1]} · steer fault {health_key[2]}",
                    )
        if not snapshot.wheel_statuses:
            self._labels["wheels"].set_text("UNAVAILABLE")
        else:
            lines = []
            for wheel in snapshot.wheel_statuses:
                fault = (" fault" if wheel.drive_axis_error or wheel.steer_fault else "")
                stale = " stale" if wheel.stale else ""
                lines.append(
                    f"{wheel.name}: {wheel.mode} · {self._number(wheel.drive_turns_per_s, 'r/s')} · "
                    f"{self._number(wheel.steer_deg, 'deg')}{stale}{fault}"
                )
            self._labels["wheels"].set_text("\n".join(lines))
        self._labels["can"].set_text(snapshot.can_state)
        self._labels["l515"].set_text(
            f"{snapshot.l515_state} · {snapshot.l515_mode} · native "
            f"{self._number(snapshot.l515_color_hz, 'Hz')} / {self._number(snapshot.l515_depth_hz, 'Hz')}\n"
            f"SRT submit/sent/drop {self._number(snapshot.l515_submitted_hz, 'Hz')} / "
            f"{self._number(snapshot.l515_sent_hz, 'Hz')} / {self._number(snapshot.l515_drop_hz, 'Hz')}\n"
            f"aligned depth {self._number(snapshot.l515_aligned_depth_age_ms, 'ms')} · "
            f"process {self._number(snapshot.l515_process_cpu_percent, '% CPU')} / "
            f"{self._rss(snapshot.l515_process_rss_bytes)}\n"
            f"ROS {self._l515_ros_rates_text(snapshot.l515_ros_topic_rates_hz)}\n"
            f"{snapshot.l515_detail or '-'}"
        )
        return True


class OperatorConsole(Gtk.Window):
    def __init__(self, host: str, d435_port: int, l515_port: int, metadata_port: int,
                 latency_ms: int, telemetry_port: int, chassis_telemetry_port: int) -> None:
        super().__init__(title="Powertrain Operator Console")
        # Keep the two live video panels and safety sidebar visible without
        # covering the operator desktop.  The user can still maximize with
        # the window manager or use F11 when a larger view is useful.
        self.set_default_size(960, 650)
        self.connect("destroy", self._on_destroy)
        self.connect("key-press-event", self._on_key_press)
        self._fullscreen = False
        self._metadata_receiver = LatestMetadataReceiver(metadata_port)
        self._telemetry_receiver = LatestTelemetryReceiver(telemetry_port)
        self._chassis_receiver = LatestTelemetryReceiver(chassis_telemetry_port)
        self._events = EventLog()
        self._d435 = VideoPanel("D435i raw", host, d435_port, latency_ms,
                                metadata_receiver=self._metadata_receiver,
                                event_sink=self._events.add_event)
        self._l515 = VideoPanel("L515 driving", host, l515_port, latency_ms,
                                event_sink=self._events.add_event)
        # Stack feeds vertically to keep the safety sidebar on-screen on
        # narrow displays.
        videos = Gtk.Paned.new(Gtk.Orientation.VERTICAL)
        videos.pack1(self._l515, resize=True, shrink=False)
        videos.pack2(self._d435, resize=True, shrink=False)
        self._health = Gtk.Label()
        self._health.set_xalign(0.0)
        self._health.set_ellipsize(Pango.EllipsizeMode.END)
        self._health.set_margin_start(12)
        self._health.set_margin_top(8)
        self._health.set_margin_bottom(8)
        self._last_safety_banner: str | None = None
        self._last_l515_transport: str | None = None
        layout = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        layout.pack_start(self._health, False, False, 0)
        content = Gtk.Box(spacing=8)
        content.set_border_width(8)
        content.pack_start(videos, True, True, 0)
        telemetry = TelemetryPanel(self._telemetry_receiver, telemetry_port,
                                   event_sink=self._events.add_event)
        chassis = ChassisTelemetryPanel(self._chassis_receiver, chassis_telemetry_port,
                                        event_sink=self._events.add_event)
        side = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        side.set_size_request(280, -1)
        side.pack_start(telemetry, False, True, 0)
        side.pack_start(chassis, False, True, 0)
        # Right-side telemetry can legitimately be taller than a compact
        # operator window (six wheel rows plus Gateway detail). Keep it in
        # one narrow column with vertical scrolling instead of clipping it.
        side_scroll = Gtk.ScrolledWindow()
        side_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        side_scroll.set_size_request(296, -1)
        side_scroll.add(side)
        content.pack_start(side_scroll, False, True, 0)
        layout.pack_start(content, True, True, 0)
        layout.pack_start(self._events, False, True, 0)
        self.add(layout)
        GLib.timeout_add(250, self._refresh_health)

    @staticmethod
    def _telemetry_state(snapshot: TelemetrySnapshot | None) -> str:
        if snapshot is None:
            return "UNAVAILABLE"
        return "STALE" if time.monotonic() - snapshot.received_monotonic_s > 1.0 else "LIVE"

    def _refresh_health(self) -> bool:
        snapshot = self._telemetry_receiver.latest()
        chassis_snapshot = self._chassis_receiver.latest()
        metadata = self._metadata_receiver.latest()
        if metadata is None:
            yolo = "WAITING"
        else:
            yolo = "STALE" if time.monotonic() - metadata.received_monotonic_s > 0.25 else "LIVE"
        telemetry = self._telemetry_state(snapshot)
        chassis_state = self._telemetry_state(chassis_snapshot)
        odom = "LIVE" if chassis_snapshot is not None and chassis_snapshot.odometry_source != "unavailable" else "UNAVAILABLE"
        drive = "LIVE" if chassis_snapshot is not None and chassis_snapshot.drive_state != "unavailable" else "UNAVAILABLE"
        can = "LIVE" if chassis_snapshot is not None and chassis_snapshot.can_state != "unavailable" else "UNAVAILABLE"
        if chassis_state != "LIVE" or chassis_snapshot is None:
            safety = "SAFETY UNAVAILABLE"
            safety_color = "#d97706"
        elif chassis_snapshot.safety_estop_required:
            detail = chassis_snapshot.safety_detail or "no detail"
            safety = f"SAFETY ESTOP · {chassis_snapshot.safety_status} · {detail}"
            safety_color = "#dc2626"
        else:
            safety = f"SAFETY CLEAR · {chassis_snapshot.safety_status}"
            safety_color = "#16a34a"
        if safety != self._last_safety_banner:
            self._events.add_event("SAFETY", safety)
            self._last_safety_banner = safety
        if chassis_state != "LIVE" or chassis_snapshot is None:
            l515_transport = "L515 transport unavailable"
        elif (chassis_snapshot.l515_submitted_hz is None
              or chassis_snapshot.l515_sent_hz is None):
            l515_transport = "L515 transport waiting"
        elif (chassis_snapshot.l515_drop_hz or 0.0) > 0.1:
            l515_transport = (
                f"L515 SRT drop {chassis_snapshot.l515_drop_hz:.2f} Hz "
                f"(submit {chassis_snapshot.l515_submitted_hz:.2f}, "
                f"sent {chassis_snapshot.l515_sent_hz:.2f})"
            )
        else:
            l515_transport = "L515 SRT transport normal"
        if l515_transport != self._last_l515_transport:
            self._events.add_event("L515", l515_transport)
            self._last_l515_transport = l515_transport
        health = (
            "READ-ONLY CONSOLE  |  "
            f"L515 {self._l515.health_state()}  ·  D435i {self._d435.health_state()}  ·  "
            f"YOLO {yolo}  ·  POWER {telemetry}  ·  CHASSIS {chassis_state}  ·  "
            f"ODOM {odom}  ·  DRIVE {drive}  ·  CAN {can}"
        )
        self._health.set_markup(
            f"{GLib.markup_escape_text(health)}  |  "
            f"<span foreground='{safety_color}' weight='bold'>"
            f"{GLib.markup_escape_text(safety)}</span>"
        )
        return True

    def _on_destroy(self, *_args: object) -> None:
        self._d435.stop()
        self._l515.stop()
        self._metadata_receiver.close()
        self._telemetry_receiver.close()
        self._chassis_receiver.close()
        Gtk.main_quit()

    def _on_key_press(self, _widget: Gtk.Window, event: Gdk.EventKey) -> bool:
        if event.keyval != Gdk.KEY_F11:
            return False
        self._fullscreen = not self._fullscreen
        if self._fullscreen:
            self.fullscreen()
            self._events.add_event("CONSOLE", "fullscreen enabled")
        else:
            self.unfullscreen()
            self._events.add_event("CONSOLE", "fullscreen disabled")
        return True


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.8.106")
    parser.add_argument("--d435-port", type=int, default=5002)
    parser.add_argument("--l515-port", type=int, default=5000)
    parser.add_argument("--metadata-port", type=int, default=5003)
    parser.add_argument("--telemetry-port", type=int, default=5004)
    parser.add_argument("--chassis-telemetry-port", type=int, default=5005)
    parser.add_argument("--latency-ms", type=int, default=60)
    args = parser.parse_args()
    Gst.init(None)
    console = OperatorConsole(args.host, args.d435_port, args.l515_port,
                              args.metadata_port, args.latency_ms, args.telemetry_port,
                              args.chassis_telemetry_port)
    console.show_all()
    Gtk.main()


if __name__ == "__main__":
    main()
