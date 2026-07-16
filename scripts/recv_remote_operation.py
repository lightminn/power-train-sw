#!/usr/bin/env python3
"""Laptop dual-video receiver and remote-operation console.

The module-level API is intentionally pure and imports no cv2, numpy,
GStreamer, or pygame.  Runtime I/O dependencies are loaded only by ``main``.
"""

import argparse
from collections import deque
import json
import math
from numbers import Real

from remote_video.contract import (
    D435I_METADATA_UDP_PORT,
    D435I_RGB_HEIGHT,
    D435I_RGB_SRT_PORT,
    D435I_RGB_WIDTH,
    L515_RGB_HEIGHT,
    L515_RGB_SRT_PORT,
    L515_RGB_WIDTH,
    RECEIVER_FEEDBACK_SCHEMA_VERSION,
    RECEIVER_FEEDBACK_UDP_PORT,
)


STALL_TIMEOUT_S = 4.0


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description="WP5.3 dual-video remote-operation laptop viewer"
    )
    parser.add_argument("--host", required=True, help="Jetson IPv4 address or host name")
    parser.add_argument("--l515-port", type=int, default=L515_RGB_SRT_PORT)
    parser.add_argument("--d435i-port", type=int, default=D435I_RGB_SRT_PORT)
    parser.add_argument("--meta-port", type=int, default=D435I_METADATA_UDP_PORT)
    parser.add_argument(
        "--feedback-port", type=int, default=RECEIVER_FEEDBACK_UDP_PORT
    )
    parser.add_argument("--latency", type=int, default=60)
    parser.add_argument("--teleop-port", type=int, default=9000)
    parser.add_argument(
        "--no-teleop",
        action="store_true",
        help="receive video and metadata overlay without opening DualSense",
    )
    return parser.parse_args(argv)


def build_recv_command(host, *, port, width, height, latency):
    """Build the standard recv_yolo3d SRT-to-raw-BGR subprocess argv."""

    return [
        "gst-launch-1.0",
        "-q",
        "srtsrc",
        "uri=srt://%s:%d?mode=caller&latency=%d" % (host, port, latency),
        "!",
        "tsdemux",
        "!",
        "h264parse",
        "!",
        "avdec_h264",
        "max-threads=1",
        "!",
        "videoconvert",
        "!",
        "videoscale",
        "!",
        "video/x-raw,format=BGR,width=%d,height=%d" % (width, height),
        "!",
        "fdsink",
        "fd=1",
    ]


def build_feedback_report(
    channel,
    session_id,
    sequence,
    decode_fps,
    display_fps,
    frame_age_ms,
    sequence_gap,
    rtt_ms,
    loss_percent,
) -> bytes:
    """Encode one exact receiver-feedback v1 JSON report."""

    payload = {
        "schema_version": RECEIVER_FEEDBACK_SCHEMA_VERSION,
        "channel": channel,
        "session_id": session_id,
        "sequence": sequence,
        "decode_fps": decode_fps,
        "display_fps": display_fps,
        "frame_age_ms": frame_age_ms,
        "sequence_gap": sequence_gap,
        "rtt_ms": rtt_ms,
        "loss_percent": loss_percent,
    }
    try:
        return json.dumps(
            payload,
            allow_nan=False,
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("feedback report cannot be encoded: %s" % exc) from exc


class FpsEstimator:
    """Sliding-window FPS estimator driven only by injected monotonic time."""

    def __init__(self, window_s=2.0):
        if (
            isinstance(window_s, bool)
            or not isinstance(window_s, Real)
            or not math.isfinite(window_s)
            or window_s <= 0.0
        ):
            raise ValueError("window_s must be a finite positive number")
        self.window_s = float(window_s)
        self._timestamps = deque()

    @staticmethod
    def _timestamp(value):
        if (
            isinstance(value, bool)
            or not isinstance(value, Real)
            or not math.isfinite(value)
        ):
            raise ValueError("timestamp must be a finite number")
        return float(value)

    def _prune(self, now_s):
        cutoff = now_s - self.window_s
        while self._timestamps and self._timestamps[0] < cutoff:
            self._timestamps.popleft()

    def record(self, now_s):
        now_s = self._timestamp(now_s)
        if self._timestamps and now_s < self._timestamps[-1]:
            raise ValueError("timestamp must not go backwards")
        self._prune(now_s)
        self._timestamps.append(now_s)

    def value(self, now_s):
        now_s = self._timestamp(now_s)
        if self._timestamps and now_s < self._timestamps[-1]:
            raise ValueError("timestamp must not go backwards")
        self._prune(now_s)
        if len(self._timestamps) < 2:
            return 0.0
        elapsed_s = self._timestamps[-1] - self._timestamps[0]
        if elapsed_s <= 0.0:
            return 0.0
        return (len(self._timestamps) - 1) / elapsed_s


def should_draw_bboxes(overlay_state):
    """Return whether metadata boxes may be drawn over the raw D435i frame."""

    if overlay_state == "FRESH":
        return True
    if overlay_state == "OVERLAY_STALE":
        return False
    raise ValueError("unknown overlay_state: %r" % (overlay_state,))


class StatusPanel:
    """Convert console state into display-only text rows."""

    @staticmethod
    def _age_line(label, age_ms):
        if (
            isinstance(age_ms, bool)
            or not isinstance(age_ms, Real)
            or not math.isfinite(age_ms)
            or age_ms < 0.0
        ):
            return "%s AGE: unavailable" % label
        return "%s AGE: %.1f ms" % (label, float(age_ms))

    @staticmethod
    def render_lines(
        requested_mode,
        ack_state,
        deadman,
        hold_reason,
        assist_bypass,
        l515_age_ms,
        d435i_age_ms,
        overlay_state,
    ):
        should_draw_bboxes(overlay_state)
        return [
            "REQUESTED MODE: %s" % requested_mode,
            "JETSON ACK: %s" % ack_state,
            "DEADMAN: %s" % ("PRESSED" if deadman else "RELEASED"),
            "HOLD: %s" % (hold_reason or "none"),
            "ASSIST BYPASS: %s" % ("ON" if assist_bypass else "OFF"),
            StatusPanel._age_line("L515", l515_age_ms),
            StatusPanel._age_line("D435i", d435i_age_ms),
            "D435i OVERLAY: %s" % overlay_state,
        ]


def parse_status_line(line):
    """Parse ``S <state> <v> <omega>`` or return ``None`` when malformed."""

    if not isinstance(line, str):
        return None
    fields = line.strip().split()
    if len(fields) != 4 or fields[0] != "S" or not fields[1]:
        return None
    try:
        linear = float(fields[2])
        angular = float(fields[3])
    except ValueError:
        return None
    if not math.isfinite(linear) or not math.isfinite(angular):
        return None
    return fields[1], linear, angular


class _ChannelBuffer:
    """Thread-safe latest-only raw frame and receiver quality state."""

    def __init__(self, name, *, started_s):
        import threading

        self.name = name
        self._lock = threading.Lock()
        self._started_s = float(started_s)
        self._frame = None
        self._frame_sequence = 0
        self._last_frame_s = None
        self._last_displayed_sequence = 0
        self._last_display_s = None
        self._decode_fps = FpsEstimator()
        self._display_fps = FpsEstimator()

    def receive(self, frame, *, now_s):
        with self._lock:
            self._decode_fps.record(now_s)
            self._frame = frame
            self._frame_sequence += 1
            self._last_frame_s = float(now_s)

    def latest(self):
        with self._lock:
            return self._frame, self._frame_sequence

    def mark_displayed(self, frame_sequence, *, now_s):
        with self._lock:
            if frame_sequence <= self._last_displayed_sequence:
                return False
            self._last_displayed_sequence = frame_sequence
            self._display_fps.record(now_s)
            self._last_display_s = float(now_s)
            return True

    def metrics(self, *, now_s):
        with self._lock:
            protected_times = [float(now_s), self._started_s]
            if self._last_frame_s is not None:
                protected_times.append(self._last_frame_s)
            if self._last_display_s is not None:
                protected_times.append(self._last_display_s)
            effective_now_s = max(protected_times)
            age_from_s = (
                self._started_s
                if self._last_frame_s is None
                else self._last_frame_s
            )
            return {
                "decode_fps": self._decode_fps.value(effective_now_s),
                "display_fps": self._display_fps.value(effective_now_s),
                "frame_age_ms": max(
                    0.0,
                    (effective_now_s - age_from_s) * 1000.0,
                ),
            }


class _MetadataReceiver:
    """UDP metadata listener retaining only one validated latest packet."""

    def __init__(self, port, stop_event, errors):
        import threading

        from remote_video.metadata import MetadataTracker

        self.port = int(port)
        self.stop_event = stop_event
        self.errors = errors
        self._tracker = MetadataTracker()
        self._lock = threading.Lock()
        self._socket = None
        self._last_received_ns = None
        self.invalid_packets = 0
        self._thread = threading.Thread(
            target=self._run,
            name="d435i-metadata",
            daemon=True,
        )

    def start(self):
        self._thread.start()

    def stop(self):
        if self._socket is not None:
            try:
                self._socket.close()
            except OSError:
                pass

    def join(self, timeout=None):
        self._thread.join(timeout)

    def snapshot(self, *, now_monotonic_ns):
        with self._lock:
            effective_now_ns = int(now_monotonic_ns)
            if self._last_received_ns is not None:
                effective_now_ns = max(effective_now_ns, self._last_received_ns)
            state = self._tracker.overlay_state(effective_now_ns)
            return state, self._tracker.latest

    def _accept_packet(self, packet, *, received_monotonic_ns):
        with self._lock:
            accepted = self._tracker.update(
                packet,
                received_monotonic_ns=received_monotonic_ns,
            )
            if accepted:
                self._last_received_ns = int(received_monotonic_ns)
            return accepted

    def _run(self):
        import socket
        import time

        from remote_video.contract import MAX_METADATA_BYTES
        from remote_video.metadata import MetadataError, parse_metadata

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket = sock
        try:
            sock.bind(("0.0.0.0", self.port))
            sock.settimeout(0.5)
            while not self.stop_event.is_set():
                try:
                    data, _ = sock.recvfrom(MAX_METADATA_BYTES + 1)
                except socket.timeout:
                    continue
                except OSError as exc:
                    if not self.stop_event.is_set():
                        self.errors.put("metadata UDP receive failed: %s" % exc)
                        self.stop_event.set()
                    return
                try:
                    packet, received_ns = parse_metadata(
                        data,
                        now_monotonic_ns=time.monotonic_ns(),
                    )
                except MetadataError:
                    with self._lock:
                        self.invalid_packets += 1
                    continue
                self._accept_packet(
                    packet,
                    received_monotonic_ns=received_ns,
                )
        except OSError as exc:
            self.errors.put(
                "cannot bind metadata UDP :%d: %s" % (self.port, exc)
            )
            self.stop_event.set()
        finally:
            sock.close()
            self._socket = None


def _read_frame(fd, nbytes, stall_timeout_s):
    """Read exactly one raw frame, returning ``None`` on EOF or SRT stall."""

    import os
    import select

    chunks = []
    remaining = nbytes
    while remaining > 0:
        ready, _, _ = select.select([fd], [], [], stall_timeout_s)
        if not ready:
            return None
        chunk = os.read(fd, remaining)
        if not chunk:
            return None
        chunks.append(chunk)
        remaining -= len(chunk)
    return b"".join(chunks)


class _SrtReceiver:
    """One restartable recv_yolo3d-style GStreamer receiver thread."""

    def __init__(
        self,
        channel,
        host,
        port,
        width,
        height,
        latency,
        buffer,
        stop_event,
        errors,
    ):
        import threading

        self.channel = channel
        self.host = host
        self.port = int(port)
        self.width = int(width)
        self.height = int(height)
        self.latency = int(latency)
        self.buffer = buffer
        self.stop_event = stop_event
        self.errors = errors
        self._process = None
        self._process_lock = threading.Lock()
        self._thread = threading.Thread(
            target=self._run,
            name="srt-%s" % channel,
            daemon=True,
        )

    def start(self):
        self._thread.start()

    def stop(self):
        with self._process_lock:
            process = self._process
        if process is not None and process.poll() is None:
            process.terminate()

    def join(self, timeout=None):
        self._thread.join(timeout)

    @staticmethod
    def _terminate(process):
        import subprocess

        if process.poll() is not None:
            return
        process.terminate()
        try:
            process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=1.0)

    def _run(self):
        import subprocess
        import sys
        import time

        import numpy as np

        frame_bytes = self.width * self.height * 3
        while not self.stop_event.is_set():
            command = build_recv_command(
                self.host,
                port=self.port,
                width=self.width,
                height=self.height,
                latency=self.latency,
            )
            print(
                "[%s gst-launch] %s" % (self.channel, " ".join(command)),
                file=sys.stderr,
            )
            try:
                process = subprocess.Popen(
                    command,
                    stdout=subprocess.PIPE,
                    bufsize=0,
                )
            except OSError as exc:
                self.errors.put(
                    "%s GStreamer launch failed: %s" % (self.channel, exc)
                )
                self.stop_event.set()
                return
            with self._process_lock:
                self._process = process
            received = 0
            try:
                fd = process.stdout.fileno()
                while not self.stop_event.is_set():
                    raw = _read_frame(fd, frame_bytes, STALL_TIMEOUT_S)
                    if raw is None:
                        break
                    frame = np.frombuffer(raw, np.uint8).reshape(
                        self.height,
                        self.width,
                        3,
                    ).copy()
                    self.buffer.receive(frame, now_s=time.monotonic())
                    received += 1
            finally:
                self._terminate(process)
                with self._process_lock:
                    self._process = None
            if not self.stop_event.is_set():
                reason = "stream stalled/disconnected" if received else "waiting for sender"
                print(
                    "[%s] %s; restarting in 1 s" % (self.channel, reason),
                    file=sys.stderr,
                )
                self.stop_event.wait(1.0)


class _FeedbackSender:
    """Send one receiver report per channel each second."""

    def __init__(
        self,
        host,
        port,
        session_id,
        channels,
        stop_event,
    ):
        import threading

        self.host = host
        self.port = int(port)
        self.session_id = session_id
        self.channels = dict(channels)
        self.stop_event = stop_event
        self._thread = threading.Thread(
            target=self._run,
            name="receiver-feedback",
            daemon=True,
        )

    def start(self):
        self._thread.start()

    def join(self, timeout=None):
        self._thread.join(timeout)

    def _run(self):
        import socket
        import sys
        import time

        sequences = {channel: 0 for channel in self.channels}
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        try:
            while not self.stop_event.is_set():
                cycle_started_s = time.monotonic()
                for channel, buffer in self.channels.items():
                    metrics = buffer.metrics(now_s=cycle_started_s)
                    # SILENT CAP: raw BGR stdout carries no source sequence,
                    # RTT, or SRT transport statistics.  The wiring cycle must
                    # connect real SRT stats; 0.0 loss is not a measurement.
                    report = build_feedback_report(
                        channel,
                        self.session_id,
                        sequences[channel],
                        metrics["decode_fps"],
                        metrics["display_fps"],
                        metrics["frame_age_ms"],
                        0,
                        0.0,
                        0.0,
                    )
                    try:
                        sock.sendto(report, (self.host, self.port))
                    except OSError as exc:
                        print(
                            "[feedback] %s send failed: %s" % (channel, exc),
                            file=sys.stderr,
                        )
                    sequences[channel] += 1
                elapsed_s = time.monotonic() - cycle_started_s
                self.stop_event.wait(max(0.0, 1.0 - elapsed_s))
        finally:
            sock.close()


class _TeleopStatus:
    def __init__(self, *, enabled):
        import threading

        self._lock = threading.Lock()
        self._enabled = bool(enabled)
        self._requested_mode = "DRIVE" if enabled else "DISABLED"
        self._ack_state = "DISCONNECTED" if enabled else "DISABLED"
        self._deadman = False
        self._assist_bypass = False
        self._status_seen = False

    def update_input(self, sample):
        with self._lock:
            self._requested_mode = sample.requested_mode
            self._deadman = bool(sample.deadman)
            self._assist_bypass = bool(sample.assist_bypass)

    def update_ack(self, state):
        with self._lock:
            self._ack_state = state
            self._status_seen = True

    def snapshot(self):
        with self._lock:
            if not self._enabled:
                hold_reason = "teleop disabled"
            elif not self._status_seen:
                hold_reason = "status unavailable"
            elif self._ack_state == self._requested_mode:
                hold_reason = ""
            elif self._ack_state == "MOTION_HOLD":
                hold_reason = "Jetson MOTION_HOLD; reason unavailable in status v1"
            else:
                hold_reason = "awaiting Jetson ACK"
            return {
                "requested_mode": self._requested_mode,
                "ack_state": self._ack_state,
                "deadman": self._deadman,
                "hold_reason": hold_reason,
                "assist_bypass": self._assist_bypass,
            }


class _StopTeleop(Exception):
    pass


class _TeleopWorker:
    """Single DualSense reader, transmitter, and status receiver."""

    def __init__(
        self,
        pygame,
        joystick,
        adapter,
        transmitter,
        status,
        stop_event,
        errors,
    ):
        import threading

        self.pygame = pygame
        self.joystick = joystick
        self.adapter = adapter
        self.transmitter = transmitter
        self.status = status
        self.stop_event = stop_event
        self.errors = errors
        self._thread = threading.Thread(
            target=self._run,
            name="dualsense-teleop",
            daemon=True,
        )

    def start(self):
        self._thread.start()

    def join(self, timeout=None):
        self._thread.join(timeout)

    def _run(self):
        import time

        from motor_control.laptop.remote_operation_client import SEND_HZ

        interval_s = 1.0 / SEND_HZ
        try:
            while not self.stop_event.is_set():
                started_s = time.monotonic()
                self.pygame.event.pump()
                sample = self.adapter.sample(now_ns=time.monotonic_ns())
                self.status.update_input(sample)
                self.transmitter.send(
                    sample,
                    client_monotonic_ns=time.monotonic_ns(),
                )
                for line in self.transmitter.receive_status():
                    parsed = parse_status_line(line)
                    if parsed is not None:
                        state, _, _ = parsed
                        self.status.update_ack(state)
                remaining_s = interval_s - (time.monotonic() - started_s)
                if remaining_s > 0.0 and self.stop_event.wait(remaining_s):
                    break
        except _StopTeleop:
            pass
        except Exception as exc:
            self.errors.put("teleop failed: %s" % exc)
            self.stop_event.set()
        finally:
            self.transmitter.close()
            try:
                self.joystick.quit()
            except Exception:
                pass
            self.pygame.quit()


def _resolve_ipv4(host):
    """Force IPv4 so mDNS link-local IPv6 does not poison the SRT URI."""

    import socket

    try:
        infos = socket.getaddrinfo(host, None, socket.AF_INET)
        return infos[0][4][0]
    except (socket.gaierror, IndexError):
        return host


def _open_teleop(host, port, stop_event, status):
    try:
        import pygame
    except ImportError as exc:
        raise RuntimeError(
            "pygame is required for teleop; rerun with --no-teleop for video/overlay only"
        ) from exc

    from motor_control.laptop.remote_operation_client import (
        DualSenseInputAdapter,
        RemoteOperationTransmitter,
        mapping_for_guid,
    )

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        pygame.quit()
        raise RuntimeError(
            "DualSense controller not found; rerun with --no-teleop for video/overlay only"
        )
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    guid = (
        joystick.get_guid()
        if hasattr(joystick, "get_guid")
        else joystick.get_name()
    )
    mapping = mapping_for_guid(guid)
    adapter = DualSenseInputAdapter(joystick, mapping)

    def sample_after_reconnect():
        adapter.reset_for_new_connection()
        pygame.event.pump()
        sample = adapter.sample(now_ns=__import__("time").monotonic_ns())
        status.update_input(sample)
        return sample

    def interruptible_sleep(delay_s):
        if stop_event.wait(delay_s):
            raise _StopTeleop()

    transmitter = RemoteOperationTransmitter(
        host,
        port,
        sleep_fn=interruptible_sleep,
        on_reconnect=sample_after_reconnect,
    )
    print(
        "[teleop] controller=%s guid=%s mapping=%s gateway=%s:%d"
        % (
            joystick.get_name(),
            guid,
            mapping["config_version"],
            host,
            port,
        )
    )
    return pygame, joystick, adapter, transmitter


def _draw_metadata(cv2, frame, overlay_state, packet):
    green = (0, 255, 0)
    orange = (0, 165, 255)
    if not should_draw_bboxes(overlay_state):
        cv2.putText(
            frame,
            "OVERLAY_STALE",
            (10, 28),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            orange,
            2,
        )
        return

    height, width = frame.shape[:2]
    for detection in packet.detections if packet is not None else ():
        x1, y1, x2, y2 = detection.bbox
        p1 = (
            max(0, min(width - 1, int(round(x1)))),
            max(0, min(height - 1, int(round(y1)))),
        )
        p2 = (
            max(0, min(width - 1, int(round(x2)))),
            max(0, min(height - 1, int(round(y2)))),
        )
        cv2.rectangle(frame, p1, p2, green, 2)
        label = "%s %.2f" % (detection.class_name, detection.confidence)
        text_y = p1[1] - 8 if p1[1] >= 18 else p1[1] + 18
        cv2.putText(
            frame,
            label,
            (p1[0], text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 0, 0),
            3,
        )
        cv2.putText(
            frame,
            label,
            (p1[0], text_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            green,
            1,
        )


def _draw_status(cv2, frame, lines):
    for index, line in enumerate(lines):
        origin = (10, 54 + index * 20)
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (0, 0, 0),
            3,
        )
        cv2.putText(
            frame,
            line,
            origin,
            cv2.FONT_HERSHEY_SIMPLEX,
            0.48,
            (255, 255, 255),
            1,
        )


def _placeholder(np, cv2, width, height, label):
    frame = np.zeros((height, width, 3), dtype=np.uint8)
    cv2.putText(
        frame,
        "%s: WAITING FOR SRT" % label,
        (20, height // 2),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.8,
        (0, 165, 255),
        2,
    )
    return frame


def _pop_worker_error(errors):
    import queue

    try:
        return errors.get_nowait()
    except queue.Empty:
        return None


def run_viewer(args):
    """Run all laptop I/O until q, Ctrl-C, SIGTERM, or a fatal worker error."""

    import queue
    import signal
    import threading
    import time
    import uuid

    try:
        import cv2
        import numpy as np
    except ImportError as exc:
        raise RuntimeError("cv2 and numpy are required to display remote video") from exc

    host = _resolve_ipv4(args.host)
    if host != args.host:
        print("[network] %s resolved to IPv4 %s" % (args.host, host))
    started_s = time.monotonic()
    stop_event = threading.Event()
    errors = queue.SimpleQueue()
    channels = {
        "l515_rgb": _ChannelBuffer("l515_rgb", started_s=started_s),
        "d435i_rgb": _ChannelBuffer("d435i_rgb", started_s=started_s),
    }
    metadata = _MetadataReceiver(args.meta_port, stop_event, errors)
    receivers = [
        _SrtReceiver(
            "l515_rgb",
            host,
            args.l515_port,
            L515_RGB_WIDTH,
            L515_RGB_HEIGHT,
            args.latency,
            channels["l515_rgb"],
            stop_event,
            errors,
        ),
        _SrtReceiver(
            "d435i_rgb",
            host,
            args.d435i_port,
            D435I_RGB_WIDTH,
            D435I_RGB_HEIGHT,
            args.latency,
            channels["d435i_rgb"],
            stop_event,
            errors,
        ),
    ]
    feedback = _FeedbackSender(
        host,
        args.feedback_port,
        str(uuid.uuid4()),
        channels,
        stop_event,
    )
    teleop_status = _TeleopStatus(enabled=not args.no_teleop)
    teleop = None
    if not args.no_teleop:
        pygame, joystick, adapter, transmitter = _open_teleop(
            host,
            args.teleop_port,
            stop_event,
            teleop_status,
        )
        teleop = _TeleopWorker(
            pygame,
            joystick,
            adapter,
            transmitter,
            teleop_status,
            stop_event,
            errors,
        )

    previous_sigterm = signal.getsignal(signal.SIGTERM)
    signal.signal(signal.SIGTERM, lambda *_: stop_event.set())
    windows = {
        "l515_rgb": "L515 remote drive (SRT)",
        "d435i_rgb": "D435i remote arm (SRT + metadata)",
    }
    for window in windows.values():
        cv2.namedWindow(window, cv2.WINDOW_NORMAL)

    metadata.start()
    for receiver in receivers:
        receiver.start()
    feedback.start()
    if teleop is not None:
        teleop.start()

    fatal_error = None
    try:
        while True:
            now_s = time.monotonic()
            now_ns = time.monotonic_ns()
            fatal_error = _pop_worker_error(errors)
            if fatal_error is not None or stop_event.is_set():
                stop_event.set()
                break

            l515_metrics = channels["l515_rgb"].metrics(now_s=now_s)
            d435i_metrics = channels["d435i_rgb"].metrics(now_s=now_s)
            overlay_state, packet = metadata.snapshot(now_monotonic_ns=now_ns)
            status = teleop_status.snapshot()
            panel_lines = StatusPanel.render_lines(
                status["requested_mode"],
                status["ack_state"],
                status["deadman"],
                status["hold_reason"],
                status["assist_bypass"],
                l515_metrics["frame_age_ms"],
                d435i_metrics["frame_age_ms"],
                overlay_state,
            )

            for channel, width, height, label in (
                ("l515_rgb", L515_RGB_WIDTH, L515_RGB_HEIGHT, "L515"),
                ("d435i_rgb", D435I_RGB_WIDTH, D435I_RGB_HEIGHT, "D435i"),
            ):
                raw_frame, frame_sequence = channels[channel].latest()
                if raw_frame is None:
                    frame = _placeholder(np, cv2, width, height, label)
                else:
                    frame = raw_frame.copy()
                    channels[channel].mark_displayed(
                        frame_sequence,
                        now_s=now_s,
                    )
                if channel == "d435i_rgb":
                    _draw_metadata(cv2, frame, overlay_state, packet)
                _draw_status(cv2, frame, panel_lines)
                cv2.imshow(windows[channel], frame)

            if cv2.waitKey(1) & 0xFF == ord("q"):
                stop_event.set()
                break
            time.sleep(0.005)
    except KeyboardInterrupt:
        stop_event.set()
    finally:
        stop_event.set()
        metadata.stop()
        for receiver in receivers:
            receiver.stop()
        if teleop is not None:
            teleop.join(4.0)
        feedback.join(2.0)
        metadata.join(2.0)
        for receiver in receivers:
            receiver.join(5.0)
        cv2.destroyAllWindows()
        signal.signal(signal.SIGTERM, previous_sigterm)

    if fatal_error is None:
        fatal_error = _pop_worker_error(errors)
    if fatal_error is not None:
        raise RuntimeError(fatal_error)
    return 0


def main(argv=None):
    args = parse_args(argv)
    try:
        return run_viewer(args)
    except RuntimeError as exc:
        raise SystemExit("recv_remote_operation: %s" % exc) from exc


if __name__ == "__main__":
    main()
