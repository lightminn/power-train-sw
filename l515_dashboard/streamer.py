"""Latest-frame ROS image handoff to the proven GStreamer SRT pipeline."""

from dataclasses import dataclass
import subprocess
from threading import Condition, Thread, current_thread
from typing import Callable, Optional

import numpy as np

from motor_control.vision.gst_stream import build_gst_command

from .config import DashboardConfig
from .frame_modes import FrameMode, LatestVideoFrames


@dataclass(frozen=True)
class StreamerSnapshot:
    running: bool
    mode: FrameMode
    sent: int
    dropped: int
    last_error: Optional[str]


class SrtStreamer:
    """Own one GStreamer child and feed it only the newest complete frame."""

    def __init__(
        self,
        config: DashboardConfig,
        mode: FrameMode = FrameMode.COLOR,
        popen: Callable[..., subprocess.Popen] = subprocess.Popen,
    ) -> None:
        self._config = config
        self._mode = FrameMode(mode)
        self._popen = popen
        self._frames = LatestVideoFrames(config.width, config.height)
        self._condition = Condition()
        self._process = None
        self._thread: Optional[Thread] = None
        self._running = False
        self._stopped = False
        self._pending = False
        self._color_ready = False
        self._depth_ready = False
        self._sent = 0
        self._dropped = 0
        self._last_error: Optional[str] = None

    def start(self) -> None:
        with self._condition:
            if self._process is not None:
                return
            width = self._config.width * (
                2 if self._mode is FrameMode.SIDE_BY_SIDE else 1
            )
            command = build_gst_command(
                self._config.port,
                width,
                self._config.height,
                self._config.fps,
                encoder=self._config.encoder,
                bitrate_kbps=self._config.bitrate_kbps,
                latency_ms=self._config.latency_ms,
            )
            self._process = self._popen(
                command, stdin=subprocess.PIPE, bufsize=0
            )
            if self._process.stdin is None:
                raise RuntimeError("GStreamer stdin pipe was not created")
            self._running = True
            self._thread = Thread(
                target=self._run, name="l515-srt-streamer", daemon=True
            )
            self._thread.start()

    def set_mode(self, mode: FrameMode) -> None:
        mode = FrameMode(mode)
        with self._condition:
            if mode is self._mode:
                return
            self._mode = mode
            self._frames.take(mode)
            self._pending = False
            self._color_ready = False
            self._depth_ready = False

    def submit_color(self, frame: np.ndarray) -> None:
        self._submit(frame, color=True)

    def submit_depth(self, frame: np.ndarray) -> None:
        self._submit(frame, color=False)

    def _submit(self, frame: np.ndarray, color: bool) -> None:
        with self._condition:
            if not self._running:
                return
            if color:
                self._frames.put_color(frame)
                selected = self._mode in (
                    FrameMode.COLOR, FrameMode.SIDE_BY_SIDE
                )
                overwrote = self._color_ready
                self._color_ready = True
            else:
                self._frames.put_depth(frame)
                selected = self._mode in (
                    FrameMode.DEPTH, FrameMode.SIDE_BY_SIDE
                )
                overwrote = self._depth_ready
                self._depth_ready = True
            if selected and overwrote:
                self._dropped += 1

            ready = (
                self._color_ready
                if self._mode is FrameMode.COLOR
                else self._depth_ready
                if self._mode is FrameMode.DEPTH
                else self._color_ready and self._depth_ready
            )
            if ready:
                self._pending = True
                self._condition.notify()

    def _run(self) -> None:
        while True:
            with self._condition:
                self._condition.wait_for(
                    lambda: self._pending or not self._running,
                    timeout=0.05,
                )
                if not self._running:
                    return
                process = self._process
                if process.poll() is not None:
                    self._fail(
                        f"GStreamer exited with code {process.returncode}"
                    )
                    return
                if not self._pending:
                    continue
                mode = self._mode
                frame = self._frames.take(mode)
                self._pending = False
                self._color_ready = False
                self._depth_ready = False

            if frame is None:
                continue
            try:
                # A single, frame-sized write is the only in-flight work. No
                # partial retry can replay stale frame bytes.
                process.stdin.write(frame.tobytes())
            except (BrokenPipeError, OSError) as exc:
                with self._condition:
                    self._fail(f"{type(exc).__name__}: {exc}")
                return
            with self._condition:
                self._sent += 1

    def _fail(self, message: str) -> None:
        self._last_error = message
        self._running = False
        self._condition.notify_all()

    def stop(self) -> None:
        with self._condition:
            if self._stopped:
                return
            self._stopped = True
            self._running = False
            self._pending = False
            process = self._process
            thread = self._thread
            self._condition.notify_all()

        if process is None:
            return
        if thread is not None and thread is not current_thread():
            thread.join(timeout=self._config.graceful_timeout_s)
        process.stdin.close()
        process.wait(timeout=self._config.graceful_timeout_s)

    def snapshot(self) -> StreamerSnapshot:
        with self._condition:
            return StreamerSnapshot(
                running=self._running,
                mode=self._mode,
                sent=self._sent,
                dropped=self._dropped,
                last_error=self._last_error,
            )
