import subprocess
import threading
import time

import numpy as np
import pytest

from l515_dashboard.config import DashboardConfig
from l515_dashboard.frame_modes import FrameMode
from l515_dashboard.streamer import SrtStreamer


class FakeStdin:
    def __init__(self, error=None, entered=None, release=None,
                 write_results=None, release_on_close=False):
        self.error = error
        self.entered = entered
        self.release = release
        self.writes = []
        self.accepted = bytearray()
        self.close_calls = 0
        self.write_results = list(write_results or [])
        self.release_on_close = release_on_close

    def write(self, data):
        if self.entered is not None:
            self.entered.set()
        if self.release is not None:
            assert self.release.wait(1.0)
        if self.error is not None:
            raise self.error
        result = self.write_results.pop(0) if self.write_results else len(data)
        if isinstance(result, int) and 0 < result <= len(data):
            accepted = bytes(data[:result])
            self.writes.append(accepted)
            self.accepted.extend(accepted)
        return result

    def close(self):
        self.close_calls += 1
        if self.release_on_close and self.release is not None:
            self.release.set()


class FakeProcess:
    def __init__(self, stdin=None):
        self.stdin = stdin or FakeStdin()
        self.returncode = None
        self.wait_calls = 0
        self.terminate_calls = 0
        self.kill_calls = 0
        self.wait_results = []

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.wait_calls += 1
        if self.wait_results:
            result = self.wait_results.pop(0)
            if isinstance(result, BaseException):
                raise result
        self.returncode = 0 if self.returncode is None else self.returncode
        return self.returncode

    def terminate(self):
        self.terminate_calls += 1

    def kill(self):
        self.kill_calls += 1


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


@pytest.mark.parametrize("mode", list(FrameMode))
def test_start_uses_independently_specified_fixed_gst_command(mode):
    config = DashboardConfig()
    popen = FakePopen()
    streamer = SrtStreamer(config, mode=mode, popen=popen)

    streamer.start()

    assert popen.calls == [
        (
            [
                "gst-launch-1.0",
                "fdsrc",
                "fd=0",
                "do-timestamp=true",
                "!",
                "rawvideoparse",
                "format=bgr",
                "width=1280",
                "height=720",
                "framerate=30/1",
                "!",
                "videoconvert",
                "!",
                "video/x-raw,format=I420",
                "!",
                "x264enc",
                "tune=zerolatency",
                "speed-preset=superfast",
                "threads=2",
                "bitrate=3000",
                "key-int-max=30",
                "!",
                "h264parse",
                "config-interval=-1",
                "!",
                "mpegtsmux",
                "alignment=7",
                "!",
                "srtsink",
                "uri=srt://:5000?mode=listener&latency=60",
                "wait-for-connection=false",
                "sync=false",
                "async=false",
            ],
            {"stdin": subprocess.PIPE, "bufsize": 0},
        )
    ]
    streamer.stop()


def test_runtime_mode_selects_next_frame_without_restarting_child():
    popen = FakePopen()
    streamer = SrtStreamer(DashboardConfig(), popen=popen)
    color = np.full((720, 1280, 3), 7, dtype=np.uint8)
    depth = np.full((720, 1280), 5000, dtype=np.uint16)
    streamer.start()

    streamer.set_mode(FrameMode.OVERLAY)
    streamer.submit_aligned_depth(depth, timestamp_ns=0)
    streamer.submit_color(color, timestamp_ns=0)

    wait_until(lambda: len(popen.process.stdin.writes) == 1)
    assert len(popen.calls) == 1
    assert len(popen.process.stdin.writes[0]) == 720 * 1280 * 3
    streamer.stop()


def test_rgb_depth_overlay_each_send_one_frame_through_same_child():
    popen = FakePopen()
    streamer = SrtStreamer(DashboardConfig(), popen=popen)
    color = np.full((720, 1280, 3), 7, np.uint8)
    depth = np.full((720, 1280), 500, np.uint16)
    streamer.start()
    child = popen.process
    streamer.submit_color(color, timestamp_ns=0)
    wait_until(lambda: streamer.snapshot().sent == 1)
    streamer.set_mode(FrameMode.DEPTH)
    streamer.submit_aligned_depth(depth, timestamp_ns=33_333_333)
    streamer.submit_color(color, timestamp_ns=33_333_333)
    wait_until(lambda: streamer.snapshot().sent == 2)
    streamer.set_mode(FrameMode.OVERLAY)
    streamer.submit_aligned_depth(depth, timestamp_ns=66_666_666)
    streamer.submit_color(color, timestamp_ns=66_666_666)
    wait_until(lambda: streamer.snapshot().sent == 3)
    assert popen.process is child
    assert len(popen.calls) == 1
    assert [len(data) for data in child.stdin.writes] == [720 * 1280 * 3] * 3
    streamer.stop()


def test_worker_overwrites_pending_frame_instead_of_queueing_or_replaying():
    entered = threading.Event()
    release = threading.Event()
    process = FakeProcess(FakeStdin(entered=entered, release=release))
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    first = np.full((720, 1280, 3), 1, dtype=np.uint8)
    second = np.full((720, 1280, 3), 2, dtype=np.uint8)
    third = np.full((720, 1280, 3), 3, dtype=np.uint8)
    streamer.start()
    streamer.submit_color(first, timestamp_ns=0)
    assert entered.wait(1.0)
    streamer.submit_color(second, timestamp_ns=33_333_333)
    streamer.submit_color(third, timestamp_ns=66_666_666)
    release.set()

    wait_until(lambda: streamer.snapshot().sent == 2)
    time.sleep(0.03)
    assert process.stdin.writes == [first.tobytes(), third.tobytes()]
    assert streamer.snapshot().dropped == 1
    streamer.stop()


def test_broken_pipe_records_error_and_stops_worker():
    process = FakeProcess(FakeStdin(error=BrokenPipeError("receiver gone")))
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    streamer.start()
    streamer.submit_color(np.zeros((720, 1280, 3), dtype=np.uint8), timestamp_ns=0)

    wait_until(lambda: not streamer.snapshot().running)
    assert "BrokenPipeError" in streamer.snapshot().last_error
    assert streamer.snapshot().sent == 0
    streamer.stop()


def test_arbitrary_writer_exception_fails_stream_without_counting_sent():
    process = FakeProcess(FakeStdin(error=ValueError("bad writer")))
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    streamer.start()
    streamer.submit_color(
        np.zeros((720, 1280, 3), np.uint8), timestamp_ns=0
    )

    wait_until(lambda: not streamer.snapshot().running)
    assert streamer.snapshot().sent == 0
    assert "ValueError: bad writer" in streamer.snapshot().last_error
    streamer.stop()


def test_short_writes_complete_exactly_one_frame_before_next_frame():
    stdin = FakeStdin(write_results=[7, 13])
    process = FakeProcess(stdin)
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    first = np.full((720, 1280, 3), 4, np.uint8)
    second = np.full((720, 1280, 3), 5, np.uint8)
    streamer.start()

    streamer.submit_color(first, timestamp_ns=0)
    wait_until(lambda: streamer.snapshot().sent == 1)
    streamer.submit_color(second, timestamp_ns=33_333_333)
    wait_until(lambda: streamer.snapshot().sent == 2)

    assert bytes(stdin.accepted) == first.tobytes() + second.tobytes()
    assert streamer.snapshot().dropped == 0
    streamer.stop()


@pytest.mark.parametrize("result", [0, None, -1, 2_764_801, "bad"])
def test_invalid_write_count_fails_stream_without_counting_sent(result):
    process = FakeProcess(FakeStdin(write_results=[result]))
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    streamer.start()
    streamer.submit_color(
        np.zeros((720, 1280, 3), np.uint8), timestamp_ns=0
    )

    wait_until(lambda: not streamer.snapshot().running)
    snapshot = streamer.snapshot()
    assert snapshot.sent == 0
    assert "write returned" in snapshot.last_error
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


def test_partial_start_without_stdin_reaps_child():
    process = FakeProcess()
    process.stdin = None
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    with pytest.raises(RuntimeError, match="stdin"):
        streamer.start()
    assert process.wait_calls == 1


def test_wait_timeout_escalates_to_terminate_then_kill_and_owns_timeout():
    process = FakeProcess()
    process.wait_results = [
        subprocess.TimeoutExpired("gst", 3),
        subprocess.TimeoutExpired("gst", 2),
        0,
    ]
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    streamer.start()
    streamer.stop()
    assert (
        process.terminate_calls,
        process.kill_calls,
        process.wait_calls,
    ) == (1, 1, 3)


def test_concurrent_stop_completes_cleanup_once():
    process = FakeProcess()
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen(process))
    streamer.start()
    callers = [threading.Thread(target=streamer.stop) for _ in range(4)]
    for caller in callers:
        caller.start()
    for caller in callers:
        caller.join(1)
        assert not caller.is_alive()
    assert (process.stdin.close_calls, process.wait_calls) == (1, 1)


def test_inflight_write_cannot_increment_sent_after_stop_returns():
    entered, release = threading.Event(), threading.Event()
    process = FakeProcess(FakeStdin(entered=entered, release=release))
    streamer = SrtStreamer(
        DashboardConfig(graceful_timeout_s=0.05), popen=FakePopen(process)
    )
    streamer.start()
    streamer.submit_color(np.zeros((720, 1280, 3), np.uint8), timestamp_ns=0)
    assert entered.wait(1)
    stopper = threading.Thread(target=streamer.stop)
    stopper.start()
    time.sleep(0.02)
    release.set()
    stopper.join(1)
    after = streamer.snapshot()
    time.sleep(0.03)
    assert streamer.snapshot() == after
    assert after.sent == 0


def test_child_cleanup_unblocks_writer_and_stop_returns_only_after_join():
    entered, release = threading.Event(), threading.Event()
    process = FakeProcess(
        FakeStdin(
            error=BrokenPipeError("late failure"),
            entered=entered,
            release=release,
            release_on_close=True,
        )
    )
    streamer = SrtStreamer(
        DashboardConfig(graceful_timeout_s=0.01), popen=FakePopen(process)
    )
    streamer.start()
    streamer.submit_color(np.zeros((720, 1280, 3), np.uint8), timestamp_ns=0)
    assert entered.wait(1)

    streamer.stop()
    stopped = streamer.snapshot()

    assert not streamer._thread.is_alive()
    assert streamer.snapshot() == stopped


def test_uncooperative_writer_makes_cleanup_fail_without_false_success():
    entered, release = threading.Event(), threading.Event()
    process = FakeProcess(FakeStdin(entered=entered, release=release))
    streamer = SrtStreamer(
        DashboardConfig(graceful_timeout_s=0.01, termination_timeout_s=0.01),
        popen=FakePopen(process),
    )
    streamer.start()
    streamer.submit_color(
        np.zeros((720, 1280, 3), np.uint8), timestamp_ns=0
    )
    assert entered.wait(1)

    with pytest.raises(RuntimeError, match="writer thread did not stop"):
        streamer.stop()

    assert streamer._thread.is_alive()
    assert streamer._cleanup_done is False
    assert streamer._cleanup_in_progress is False
    release.set()
    wait_until(lambda: not streamer._thread.is_alive())
    streamer.stop()
    assert streamer._cleanup_done is True


def test_set_mode_after_stop_is_a_snapshot_preserving_noop():
    streamer = SrtStreamer(DashboardConfig(), popen=FakePopen())
    streamer.start()
    streamer.stop()
    stopped = streamer.snapshot()

    streamer.set_mode(FrameMode.DEPTH)

    assert streamer.snapshot() == stopped


def test_overlay_outputs_each_color_using_fresh_reusable_depth():
    popen = FakePopen()
    streamer = SrtStreamer(DashboardConfig(), mode=FrameMode.OVERLAY, popen=popen)
    depth = np.full((720, 1280), 500, np.uint16)
    streamer.start()
    original_process = popen.process
    streamer.submit_aligned_depth(depth, timestamp_ns=0)

    for index in range(3):
        color = np.full((720, 1280, 3), index, np.uint8)
        streamer.submit_color(color, timestamp_ns=index * 33_333_333)
        wait_until(lambda: streamer.snapshot().sent == index + 1)

    snapshot = streamer.snapshot()
    assert len(original_process.stdin.writes) == 3
    assert popen.process is original_process
    assert len(popen.calls) == 1
    assert snapshot.input_color == 3
    assert snapshot.effective_fps == pytest.approx(30.0, rel=1e-6)
    assert snapshot.depth_age_ms == pytest.approx(66.666666)
    assert snapshot.pipeline_command == tuple(popen.calls[0][0])
    streamer.stop()


def test_depth_update_never_schedules_output():
    popen = FakePopen()
    streamer = SrtStreamer(DashboardConfig(), mode=FrameMode.DEPTH, popen=popen)
    streamer.start()

    streamer.submit_aligned_depth(
        np.full((720, 1280), 500, np.uint16), timestamp_ns=0
    )
    time.sleep(0.03)

    assert streamer.snapshot().sent == 0
    assert popen.process.stdin.writes == []
    streamer.stop()
