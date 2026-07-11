"""Latest-frame ROS image handoff to the proven GStreamer SRT pipeline."""

from dataclasses import dataclass
import subprocess
from threading import Condition, Thread, current_thread
from typing import Callable, Optional

import numpy as np

from motor_control.vision.gst_stream import build_gst_command

from .config import DashboardConfig
from .frame_modes import FrameMode, LatestVideoFrames


class StreamerStopTimeout(RuntimeError):
    """The child was reaped but the frame writer is still alive."""


@dataclass(frozen=True)
class StreamerSnapshot:
    running: bool
    mode: FrameMode
    input_color: int
    sent: int
    dropped: int
    effective_fps: float
    submitted_rate_hz: float
    sent_rate_hz: float
    drop_rate_hz: float
    depth_age_ms: Optional[float]
    pipeline_command: tuple[str, ...]
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
        self._stdin_closed = False
        self._process_reaped = False
        self._generation = 0
        self._pending = False
        self._pending_timestamp_ns: Optional[int] = None
        self._input_color = 0
        self._sent = 0
        self._dropped = 0
        self._first_sent_timestamp_ns: Optional[int] = None
        self._last_sent_timestamp_ns: Optional[int] = None
        self._first_submitted_timestamp_ns: Optional[int] = None
        self._last_submitted_timestamp_ns: Optional[int] = None
        self._latest_color_timestamp_ns: Optional[int] = None
        self._pipeline_command: tuple[str, ...] = ()
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
            self._pipeline_command = tuple(command)
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

    def submit_color(self, frame: np.ndarray, timestamp_ns: int) -> None:
        """Replace the pending RGB-paced output; never grow a queue."""
        with self._condition:
            if not self._running:
                return
            self._frames.put_color(frame, timestamp_ns)
            self._input_color += 1
            if self._first_submitted_timestamp_ns is None:
                self._first_submitted_timestamp_ns = timestamp_ns
            self._last_submitted_timestamp_ns = timestamp_ns
            self._latest_color_timestamp_ns = timestamp_ns
            if self._pending:
                self._dropped += 1
            self._pending = True
            self._pending_timestamp_ns = timestamp_ns
            self._condition.notify()

    def submit_aligned_depth(self, frame: np.ndarray, timestamp_ns: int) -> None:
        """Replace reusable overlay state without scheduling encoded output."""
        with self._condition:
            if not self._running:
                return
            self._frames.put_depth(frame, timestamp_ns)

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
                timestamp_ns = self._pending_timestamp_ns
                frame = self._frames.take(
                    mode, timestamp_ns, self._config.max_depth_age_ns
                )
                self._pending = False
                self._pending_timestamp_ns = None

            if frame is None:
                continue
            try:
                completed = self._write_all(
                    process.stdin, memoryview(frame).cast("B"), generation
                )
            except Exception as exc:
                with self._condition:
                    self._fail(f"{type(exc).__name__}: {exc}", generation)
                return
            if not completed:
                return
            with self._condition:
                if self._running and generation == self._generation:
                    self._sent += 1
                    if self._first_sent_timestamp_ns is None:
                        self._first_sent_timestamp_ns = timestamp_ns
                    self._last_sent_timestamp_ns = timestamp_ns

    def _write_all(self, stream, data: memoryview, generation: int) -> bool:
        """Write one complete frame without allowing replay or interleaving."""
        offset = 0
        while offset < len(data):
            with self._condition:
                if not self._running or generation != self._generation:
                    return False
            remaining = data[offset:]
            written = stream.write(remaining)
            if (
                isinstance(written, bool)
                or not isinstance(written, int)
                or written <= 0
                or written > len(remaining)
            ):
                raise RuntimeError(
                    f"write returned invalid byte count {written!r} "
                    f"for {len(remaining)} remaining bytes"
                )
            offset += written
        return True

    def _fail(self, message: str, generation: int) -> None:
        if self._stopped or generation != self._generation:
            return
        self._last_error = message
        self._running = False
        self._condition.notify_all()

    def stop(self) -> None:
        with self._condition:
            while self._cleanup_in_progress:
                self._condition.wait()
            if self._cleanup_done:
                return
            self._cleanup_in_progress = True
            self._stopped = True
            self._generation += 1
            self._running = False
            self._pending = False
            self._pending_timestamp_ns = None
            process = self._process
            thread = self._thread
            self._condition.notify_all()

        cleanup_error = None
        try:
            if process is not None:
                if thread is not None and thread is not current_thread():
                    thread.join(timeout=self._config.graceful_timeout_s)
                self._reap(process)
                if thread is not None and thread is not current_thread():
                    thread.join(timeout=self._config.termination_timeout_s)
                    if thread.is_alive():
                        raise StreamerStopTimeout(
                            "SRT writer thread did not stop after child cleanup"
                        )
        except Exception as exc:
            cleanup_error = exc
            raise
        finally:
            with self._condition:
                if cleanup_error is None:
                    self._cleanup_done = True
                else:
                    self._last_error = str(cleanup_error)
                self._cleanup_in_progress = False
                self._condition.notify_all()

    def _reap(self, process, *, close_stdin=True):
        if close_stdin and process.stdin is not None and not self._stdin_closed:
            self._stdin_closed = True
            process.stdin.close()
        if self._process_reaped:
            return
        try:
            process.wait(timeout=self._config.graceful_timeout_s)
            self._process_reaped = True
            return
        except subprocess.TimeoutExpired:
            process.terminate()
        try:
            process.wait(timeout=self._config.termination_timeout_s)
            self._process_reaped = True
            return
        except subprocess.TimeoutExpired:
            process.kill()
        process.wait()
        self._process_reaped = True

    def snapshot(self) -> StreamerSnapshot:
        with self._condition:
            effective_fps = 0.0
            if (
                self._sent > 1
                and self._first_sent_timestamp_ns is not None
                and self._last_sent_timestamp_ns > self._first_sent_timestamp_ns
            ):
                effective_fps = (
                    (self._sent - 1) * 1_000_000_000
                    / (self._last_sent_timestamp_ns - self._first_sent_timestamp_ns)
                )
            depth_age_ns = (
                None
                if self._latest_color_timestamp_ns is None
                else self._frames.depth_age_ns(self._latest_color_timestamp_ns)
            )
            submitted_rate_hz = 0.0
            submitted_duration_ns = 0
            if (self._input_color > 1
                    and self._first_submitted_timestamp_ns is not None
                    and self._last_submitted_timestamp_ns > self._first_submitted_timestamp_ns):
                submitted_duration_ns = (
                    self._last_submitted_timestamp_ns - self._first_submitted_timestamp_ns)
                submitted_rate_hz = ((self._input_color - 1) * 1_000_000_000
                                     / submitted_duration_ns)
            return StreamerSnapshot(
                running=self._running,
                mode=self._mode,
                input_color=self._input_color,
                sent=self._sent,
                dropped=self._dropped,
                effective_fps=effective_fps,
                submitted_rate_hz=submitted_rate_hz,
                sent_rate_hz=effective_fps,
                drop_rate_hz=(0.0 if submitted_duration_ns <= 0 else
                              self._dropped * 1_000_000_000 / submitted_duration_ns),
                depth_age_ms=(
                    None if depth_age_ns is None else depth_age_ns / 1_000_000
                ),
                pipeline_command=self._pipeline_command,
                last_error=self._last_error,
            )
