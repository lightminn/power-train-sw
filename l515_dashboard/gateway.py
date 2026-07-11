"""Headless L515 Gateway lifecycle and command authority."""

from enum import Enum
import threading
from .frame_modes import FrameMode


class GatewayState(str, Enum):
    STARTING="STARTING"; RUNNING="RUNNING"; DEGRADED="DEGRADED"; STOPPING="STOPPING"; STOPPED="STOPPED"; FAULT="FAULT"


class Gateway:
    def __init__(self, *, guard, source, ros, streamer=None, server=None, streamer_factory=None):
        self.guard=guard; self.source=source; self.ros=ros; self.streamer=streamer; self.server=server
        self.state=GatewayState.STOPPED; self.last_error=None; self.streaming_enabled=streamer is not None
        self._streamer_factory=streamer_factory
        self.restart_requested=False
        self._started=[]; self._lock=threading.RLock(); self._shutdown_done=False

    @staticmethod
    def _start(part):
        method=getattr(part, "start", None) or getattr(part, "acquire")
        method()

    @staticmethod
    def _stop(part):
        method=getattr(part, "stop", None) or getattr(part, "shutdown", None) or getattr(part, "release")
        method()

    def start(self):
        with self._lock:
            self.state=GatewayState.STARTING; self._shutdown_done=False
            try:
                for part in (self.guard, self.source, self.ros, self.streamer, self.server):
                    if part is None: continue
                    try:
                        self._start(part)
                    except Exception as exc:
                        if part is not self.streamer: raise
                        self.last_error=str(exc); self.streaming_enabled=False
                        try: self._stop(part)
                        except Exception: pass
                        continue
                    self._started.append(part)
                claim=getattr(self.guard, "claim_socket", None)
                if claim and self.server is not None: claim()
                self.state=GatewayState.RUNNING if self.streaming_enabled else GatewayState.DEGRADED
            except Exception as exc:
                self.last_error=str(exc); self._cleanup(GatewayState.FAULT); raise

    def _cleanup(self, final_state):
        if self._shutdown_done: return
        self.state=GatewayState.STOPPING if final_state is GatewayState.STOPPED else GatewayState.FAULT
        # Holding _lock blocks run_once (frame intake), then child precedes SDK.
        for part in (self.streamer, self.source, self.ros, self.server, self.guard):
            if part in self._started:
                try: self._stop(part)
                except Exception as exc:
                    if self.last_error is None: self.last_error=str(exc)
        self._started.clear(); self._shutdown_done=True; self.state=final_state

    def shutdown(self):
        with self._lock: self._cleanup(GatewayState.STOPPED)

    def ros_fatal(self, exc):
        with self._lock: self.last_error=str(exc); self._cleanup(GatewayState.FAULT)

    def client_disconnected(self):
        pass

    def observe(self):
        with self._lock:
            if self.state in (GatewayState.STOPPED, GatewayState.FAULT): return
            source_ok=getattr(getattr(self.source,"state",None),"value",None)=="streaming"
            stream_ok=True
            if self.streamer is not None and self.streaming_enabled:
                stream_ok=bool(self.streamer.snapshot().running)
                if not stream_ok: self.streaming_enabled=False
            self.state=GatewayState.RUNNING if source_ok and stream_ok else GatewayState.DEGRADED

    def run_once(self):
        """Drain at most one latest frameset; never replay stale input."""
        if self.state in (GatewayState.STOPPED, GatewayState.FAULT): return
        frames=self.source.poll_latest()
        if not getattr(frames, "empty", True):
            try:
                self.ros.publish(frames)
            except Exception as exc:
                self.ros_fatal(exc); return
            if self.streamer is not None and self.streaming_enabled:
                import numpy as np
                if frames.raw_color is not None:
                    self.streamer.submit_color(np.asanyarray(frames.raw_color.get_data()))
                if frames.aligned_depth is not None:
                    self.streamer.submit_depth(np.asanyarray(frames.aligned_depth.get_data()))
        self.observe()

    def status_snapshot(self):
        stream=self.streamer.snapshot() if self.streamer is not None else None
        return {"state": self.state.value, "streaming": bool(self.streaming_enabled and stream and stream.running),
                "video_mode": getattr(getattr(stream,"mode",None),"value",None), "last_error": self.last_error}

    def handle_request(self, request):
        with self._lock:
            kind=request["type"]; payload=request.get("payload",{})
            if kind=="get_status": return self.status_snapshot()
            if kind=="set_video_mode": self.streamer.set_mode(FrameMode(payload["mode"]))
            elif kind=="set_streaming": self._set_streaming(payload["enabled"])
            elif kind=="restart_gateway": self.restart_requested=True; self.shutdown()
            elif kind=="stop_gateway": self.shutdown()
            return self.status_snapshot()

    def _set_streaming(self, enabled):
        if self.streamer is None: return
        if enabled and not self.streaming_enabled:
            if self._streamer_factory is not None:
                self.streamer=self._streamer_factory()
            self.streamer.start(); self.streaming_enabled=True
        elif not enabled and self.streaming_enabled: self.streamer.stop(); self.streaming_enabled=False
