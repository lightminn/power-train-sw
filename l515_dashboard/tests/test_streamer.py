import subprocess
import threading
import time

import numpy as np
import pytest

from l515_dashboard.config import DashboardConfig
from l515_dashboard.frame_modes import FrameMode
from l515_dashboard.streamer import SrtStreamer
from motor_control.vision.gst_stream import build_gst_command


class FakeStdin:
    def __init__(self, error=None, entered=None, release=None):
        self.error = error
        self.entered = entered
        self.release = release
        self.writes = []
        self.close_calls = 0

    def write(self, data):
        if self.entered is not None:
            self.entered.set()
        if self.release is not None:
            assert self.release.wait(1.0)
        if self.error is not None:
            raise self.error
        self.writes.append(data)
        return len(data)

    def close(self):
        self.close_calls += 1


class FakeProcess:
    def __init__(self, stdin=None):
        self.stdin = stdin or FakeStdin()
        self.returncode = None
        self.wait_calls = 0

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_calls += 1
        self.returncode = 0 if self.returncode is None else self.returncode
        return self.returncode


class FakePopen:
    def __init__(self, process=None):
        self.process = process or FakeProcess()
        self.calls = []

    def __call__(self, argv, **kwargs):
        self.calls.append((argv, kwargs))
        return self.process


def wait_until(predicate, timeout=1.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition was not met")


@pytest.mark.parametrize(
    ("mode", "width"),
    [(FrameMode.COLOR, 640), (FrameMode.DEPTH, 640),
     (FrameMode.SIDE_BY_SIDE, 1280)],
)
def test_start_uses_exact_gst_command_for_mode_width(mode, width):
    config = DashboardConfig()
    popen = FakePopen()
    streamer = SrtStreamer(config, mode=mode, popen=popen)

    streamer.start()

    assert popen.calls == [(
        build_gst_command(
            config.port, width, config.height, config.fps,
            encoder=config.encoder, bitrate_kbps=config.bitrate_kbps,
            latency_ms=config.latency_ms,
        ),
        {"stdin": subprocess.PIPE, "bufsize": 0},
    )]
    streamer.stop()


def test_runtime_mode_selects_next_frame_without_restarting_child():
    popen = FakePopen()
    streamer = SrtStreamer(DashboardConfig(width=2, height=1), popen=popen)
    color = np.full((1, 2, 3), 7, dtype=np.uint8)
    depth = np.array([[0, 5000]], dtype=np.uint16)
    streamer.start()

    streamer.set_mode(FrameMode.SIDE_BY_SIDE)
    streamer.submit_color(color)
    streamer.submit_depth(depth)

    wait_until(lambda: len(popen.process.stdin.writes) == 1)
    assert len(popen.calls) == 1
    assert len(popen.process.stdin.writes[0]) == 1 * 4 * 3
    streamer.stop()


def test_worker_overwrites_pending_frame_instead_of_queueing_or_replaying():
    entered = threading.Event()
    release = threading.Event()
    process = FakeProcess(FakeStdin(entered=entered, release=release))
    streamer = SrtStreamer(
        DashboardConfig(width=2, height=1), popen=FakePopen(process)
    )
    first = np.full((1, 2, 3), 1, dtype=np.uint8)
    second = np.full((1, 2, 3), 2, dtype=np.uint8)
    third = np.full((1, 2, 3), 3, dtype=np.uint8)
    streamer.start()
    streamer.submit_color(first)
    assert entered.wait(1.0)
    streamer.submit_color(second)
    streamer.submit_color(third)
    release.set()

    wait_until(lambda: streamer.snapshot().sent == 2)
    time.sleep(0.03)
    assert process.stdin.writes == [first.tobytes(), third.tobytes()]
    assert streamer.snapshot().dropped == 1
    streamer.stop()


def test_broken_pipe_records_error_and_stops_worker():
    process = FakeProcess(FakeStdin(error=BrokenPipeError("receiver gone")))
    streamer = SrtStreamer(DashboardConfig(width=2, height=1),
                           popen=FakePopen(process))
    streamer.start()
    streamer.submit_color(np.zeros((1, 2, 3), dtype=np.uint8))

    wait_until(lambda: not streamer.snapshot().running)
    assert "BrokenPipeError" in streamer.snapshot().last_error
    assert streamer.snapshot().sent == 0
    streamer.stop()


def test_unexpected_child_exit_records_error_and_stops_worker():
    process = FakeProcess()
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    streamer.start()
    process.returncode = 17

    wait_until(lambda: not streamer.snapshot().running)
    assert streamer.snapshot().last_error == "GStreamer exited with code 17"
    streamer.stop()


def test_repeated_stop_closes_stdin_and_reaps_process_once():
    process = FakeProcess()
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    streamer.start()

    streamer.stop()
    streamer.stop()

    assert process.stdin.close_calls == 1
    assert process.wait_calls == 1
    assert not streamer.snapshot().running
