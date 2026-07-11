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
        self._frames = LatestVideoFrames(
            config.width, config.height, config.overlay_alpha
        )
        self._condition = Condition()
        self._process = None
        self._thread: Optional[Thread] = None
        self._running = False
        self._stopped = False
        self._cleanup_in_progress = False
        self._cleanup_done = False
        self._generation = 0
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
            command = build_gst_command(
                self._config.port,
                self._config.width,
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
                process = self._process
                self._process = None
                self._reap(process, close_stdin=False)
                raise RuntimeError("GStreamer stdin pipe was not created")
            self._running = True
            self._generation += 1
            generation = self._generation
            self._thread = Thread(
                target=self._run,
                args=(generation,),
                name="l515-srt-streamer",
                daemon=True,
            )
            self._thread.start()

    def set_mode(self, mode: FrameMode) -> None:
        mode = FrameMode(mode)
        with self._condition:
            if self._stopped:
                return
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
                selected = self._mode in (FrameMode.COLOR, FrameMode.OVERLAY)
                overwrote = self._color_ready
                self._color_ready = True
            else:
                self._frames.put_depth(frame)
                selected = self._mode in (FrameMode.DEPTH, FrameMode.OVERLAY)
                overwrote = self._depth_ready
                self._depth_ready = True
            if selected and overwrote:
                self._dropped += 1

            ready = (
                self._color_ready
                if self._mode is FrameMode.COLOR
                else (
                    self._depth_ready
                    if self._mode is FrameMode.DEPTH
                    else self._color_ready and self._depth_ready
                )
            )
            if ready:
                self._pending = True
                self._condition.notify()

    def _run(self, generation) -> None:
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
                        f"GStreamer exited with code {process.returncode}",
                        generation,
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
                    self._fail(f"{type(exc).__name__}: {exc}", generation)
                return
            with self._condition:
                if self._running and generation == self._generation:
                    self._sent += 1

    def _fail(self, message: str, generation: int) -> None:
        if self._stopped or generation != self._generation:
            return
        self._last_error = message
        self._running = False
        self._condition.notify_all()

    def stop(self) -> None:
        with self._condition:
            if self._cleanup_done:
                return
            if self._cleanup_in_progress:
                self._condition.wait_for(lambda: self._cleanup_done)
                return
            self._cleanup_in_progress = True
            self._stopped = True
            self._generation += 1
            self._running = False
            self._pending = False
            process = self._process
            thread = self._thread
            self._condition.notify_all()

        try:
            if process is not None:
                if thread is not None and thread is not current_thread():
                    thread.join(timeout=self._config.graceful_timeout_s)
                self._reap(process)
        finally:
            with self._condition:
                self._cleanup_done = True
                self._cleanup_in_progress = False
                self._condition.notify_all()

    def _reap(self, process, *, close_stdin=True):
        if close_stdin and process.stdin is not None:
            process.stdin.close()
        try:
            process.wait(timeout=self._config.graceful_timeout_s)
            return
        except subprocess.TimeoutExpired:
            process.terminate()
        try:
            process.wait(timeout=self._config.termination_timeout_s)
            return
        except subprocess.TimeoutExpired:
            process.kill()
        process.wait()

    def snapshot(self) -> StreamerSnapshot:
        with self._condition:
            return StreamerSnapshot(
                running=self._running,
                mode=self._mode,
                sent=self._sent,
                dropped=self._dropped,
                last_error=self._last_error,
            )
