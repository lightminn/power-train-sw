"""Headless L515 Gateway lifecycle and command authority."""

from enum import Enum
import resource
import threading
import time

from .control_server import DeferredResponse
from .diagnostics import DiagnosticsTracker
from .frame_modes import FrameMode
from .gateway_source import EXPECTED_L515_SERIAL


class GatewayState(str, Enum):
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    DEGRADED = "DEGRADED"
    STOPPING = "STOPPING"
    STOPPED = "STOPPED"
    FAULT = "FAULT"


def _system_snapshot():
    usage = resource.getrusage(resource.RUSAGE_SELF)
    return {"cpu_user_s": usage.ru_utime, "cpu_system_s": usage.ru_stime,
            "max_rss_kib": usage.ru_maxrss}


class Gateway:
    def __init__(self, *, guard, source, ros, streamer=None, server=None,
                 streamer_factory=None, diagnostics=None,
                 system_collector=_system_snapshot, now_ns=time.time_ns):
        self.guard = guard
        self.source = source
        self.ros = ros
        self.streamer = streamer
        self.server = server
        self.state = GatewayState.STOPPED
        self.last_error = None
        self.streaming_enabled = streamer is not None
        self._stream_failed = False
        self._streamer_factory = streamer_factory
        self._diagnostics = diagnostics if diagnostics is not None else DiagnosticsTracker()
        self._system_collector = system_collector
        self._now_ns = now_ns
        self._owned = []
        self._streamers = []
        self._lock = threading.RLock()
        self._shutdown_done = False
        self._ros_counts = {
            "/l515/color/image_raw": 0, "/l515/color/camera_info": 0,
            "/l515/depth/image_rect_raw": 0, "/l515/depth/camera_info": 0,
            "/l515/gyro/sample": 0, "/l515/accel/sample": 0,
        }

    @staticmethod
    def _start(part):
        (getattr(part, "start", None) or getattr(part, "acquire"))()

    @staticmethod
    def _stop(part):
        (getattr(part, "stop", None) or getattr(part, "shutdown", None)
         or getattr(part, "release"))()

    def _own(self, part):
        if part is not None and part not in self._owned:
            self._owned.append(part)
        if part is not None and part is self.streamer and part not in self._streamers:
            self._streamers.append(part)

    def _start_owned(self, part, *, optional=False):
        self._own(part)  # register before start so partial initialization rolls back
        try:
            self._start(part)
            return True
        except Exception as exc:
            if not optional:
                raise
            self.last_error = str(exc)
            self.streaming_enabled = False
            self._stream_failed = True
            try:
                self._stop(part)
            except Exception:
                pass
            return False

    def start(self):
        with self._lock:
            self.state = GatewayState.STARTING
            self._shutdown_done = False
            try:
                self._start_owned(self.guard)
                self._start_owned(self.source)
                self._start_owned(self.ros)
                self._start_owned(self.streamer, optional=True)
                self._start_owned(self.server)
                claim = getattr(self.guard, "claim_socket", None)
                if claim and self.server is not None:
                    claim()
                self.observe()
            except Exception as exc:
                self.last_error = str(exc)
                self._cleanup(GatewayState.FAULT)
                raise

    def _cleanup(self, final_state):
        if self._shutdown_done:
            return
        self.state = (GatewayState.STOPPING if final_state is GatewayState.STOPPED
                      else GatewayState.FAULT)
        # _lock blocks frame intake. Reap every SRT generation before SDK.
        for part in reversed(self._streamers):
            if part in self._owned:
                try:
                    self._stop(part)
                except Exception as exc:
                    self.last_error = self.last_error or str(exc)
                self._owned.remove(part)
        for part in (self.source, self.ros, self.server, self.guard):
            if part in self._owned:
                try:
                    self._stop(part)
                except Exception as exc:
                    self.last_error = self.last_error or str(exc)
                self._owned.remove(part)
        self._shutdown_done = True
        self.state = final_state

    def shutdown(self):
        with self._lock:
            self._cleanup(GatewayState.STOPPED)

    def ros_fatal(self, exc):
        with self._lock:
            self.last_error = str(exc)
            self._cleanup(GatewayState.FAULT)

    def client_disconnected(self):
        return None

    def observe(self):
        with self._lock:
            if self.state in (GatewayState.STOPPED, GatewayState.FAULT,
                              GatewayState.STOPPING):
                return
            source_state = getattr(getattr(self.source, "state", None), "value", "unknown")
            if source_state in ("connecting", "stopped"):
                self.state = GatewayState.STARTING
                return
            if source_state != "streaming":
                self.state = GatewayState.DEGRADED
                return
            if self.streamer is not None and self.streaming_enabled:
                stream = self.streamer.snapshot()
                if not stream.running:
                    self.streaming_enabled = False
                    self.last_error = stream.last_error or self.last_error
                    self._stream_failed = True
                    self.state = GatewayState.DEGRADED
                    return
            if self._stream_failed:
                self.state = GatewayState.DEGRADED
                return
            self.state = GatewayState.RUNNING

    def run_once(self):
        """Serialize drain/publish/submit against restart and cleanup."""
        with self._lock:
            if self.state in (GatewayState.STOPPED, GatewayState.FAULT,
                              GatewayState.STOPPING):
                return
            frames = self.source.poll_latest()
            if not getattr(frames, "empty", True):
                try:
                    self.ros.publish(frames)
                    self._count_ros(frames)
                except Exception as exc:
                    self.last_error = str(exc)
                    self._cleanup(GatewayState.FAULT)
                    return
                if self.streamer is not None and self.streaming_enabled:
                    import numpy as np
                    if frames.raw_color is not None:
                        self.streamer.submit_color(
                            np.asanyarray(frames.raw_color.get_data()))
                    if frames.aligned_depth is not None:
                        self.streamer.submit_depth(
                            np.asanyarray(frames.aligned_depth.get_data()))
            self.observe()

    def _count_ros(self, frames):
        now = self._now_ns()
        def observe(topic, frame):
            stamp = int(float(frame.get_timestamp()) * 1_000_000)
            self._diagnostics.observe(topic, stamp, now)
        if frames.raw_color is not None:
            self._ros_counts["/l515/color/image_raw"] += 1
            self._ros_counts["/l515/color/camera_info"] += 1
            observe("/l515/color/image_raw", frames.raw_color)
            observe("/l515/color/camera_info", frames.raw_color)
        if frames.raw_depth is not None:
            self._ros_counts["/l515/depth/image_rect_raw"] += 1
            self._ros_counts["/l515/depth/camera_info"] += 1
            observe("/l515/depth/image_rect_raw", frames.raw_depth)
            observe("/l515/depth/camera_info", frames.raw_depth)
        if frames.gyro is not None:
            self._ros_counts["/l515/gyro/sample"] += 1
            observe("/l515/gyro/sample", frames.gyro)
        if frames.accel is not None:
            self._ros_counts["/l515/accel/sample"] += 1
            observe("/l515/accel/sample", frames.accel)

    def status_snapshot(self):
        with self._lock:
            stream = self.streamer.snapshot() if self.streamer is not None else None
            snapshot = self._diagnostics.snapshot(self._now_ns())
            diagnostics = {
                topic: {"fps": metric.fps, "age_s": metric.age_s,
                        "max_gap_s": metric.max_gap_s,
                        "nonincreasing_count": metric.nonincreasing_count}
                for topic, metric in snapshot.topics.items()
            }
            config = getattr(self.source, "config", None)
            profile = None if config is None else {
                "color": [config.color_width, config.color_height, config.fps],
                "depth": [config.depth_width, config.depth_height, config.fps],
            }
            return {
                "state": self.state.value,
                "sdk": {"serial": EXPECTED_L515_SERIAL, "profile": profile,
                        "source_state": getattr(getattr(self.source, "state", None),
                                                "value", "unknown")},
                "diagnostics": diagnostics,
                "ros_publish_counts": dict(self._ros_counts),
                "srt": {"running": bool(stream and stream.running),
                        "enabled": self.streaming_enabled,
                        "mode": getattr(getattr(stream, "mode", None), "value", None),
                        "sent": getattr(stream, "sent", 0),
                        "dropped": getattr(stream, "dropped", 0),
                        "last_error": getattr(stream, "last_error", None)},
                "system": dict(self._system_collector()),
                "last_error": self.last_error,
            }

    def handle_request(self, request):
        with self._lock:
            kind, payload = request["type"], request.get("payload", {})
            if kind == "get_status":
                return self.status_snapshot()
            if kind == "set_video_mode":
                self.streamer.set_mode(FrameMode(payload["mode"]))
            elif kind == "set_streaming":
                self._set_streaming(payload["enabled"])
            elif kind == "restart_gateway":
                return DeferredResponse({"accepted": True}, self.restart_components)
            elif kind == "stop_gateway":
                return DeferredResponse({"accepted": True}, self.shutdown)
            return self.status_snapshot()

    def _set_streaming(self, enabled):
        if self.streamer is None:
            return
        if enabled and not self.streaming_enabled:
            # Reap the stopped/crashed generation before replacement.
            self._stop(self.streamer)
            if self._streamer_factory is None:
                raise RuntimeError("streamer cannot be restarted")
            self.streamer = self._streamer_factory()
            self._start_owned(self.streamer, optional=False)
            self.streaming_enabled = True
            self._stream_failed = False
        elif not enabled and self.streaming_enabled:
            self._stop(self.streamer)
            self.streaming_enabled = False
            self._stream_failed = False

    def restart_components(self):
        """Internally restart SDK, ROS and optional SRT; keep guard/socket alive."""
        with self._lock:
            if self._shutdown_done:
                return
            self.state = GatewayState.STARTING
            for streamer in reversed(self._streamers):
                self._stop(streamer)
                if streamer in self._owned:
                    self._owned.remove(streamer)
            for part in (self.source, self.ros):
                self._stop(part)
                if part in self._owned:
                    self._owned.remove(part)
            try:
                self._start_owned(self.source)
                self._start_owned(self.ros)
                if self._streamer_factory is not None:
                    self.streamer = self._streamer_factory()
                    self.streaming_enabled = self._start_owned(
                        self.streamer, optional=True)
                    if self.streaming_enabled:
                        self._stream_failed = False
                self.observe()
            except Exception as exc:
                self.last_error = str(exc)
                self._cleanup(GatewayState.FAULT)
