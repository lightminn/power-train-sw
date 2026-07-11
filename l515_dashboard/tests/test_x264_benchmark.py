from scripts.benchmark_l515_x264 import (
    build_x264_benchmark_command,
    run_benchmark,
)
from motor_control.vision.gst_stream import (
    X264_CONVERSION,
    build_conversion_tokens,
    build_gst_command,
)


def test_benchmark_commands_are_software_x264_only():
    cpu = build_x264_benchmark_command(1280, 720, 30, "videoconvert", "fakesink")
    nv = build_x264_benchmark_command(1280, 720, 30, "nvvidconv", "fakesink")
    assert "x264enc" in cpu and "x264enc" in nv
    assert "nvv4l2h264enc" not in cpu + nv
    assert [token for token in nv if token == "nvvidconv"] == ["nvvidconv"]


def test_selected_stream_path_matches_videoconvert_x264_benchmark():
    command = build_gst_command(5000, 1280, 720, 30)
    assert "videoconvert" in command
    assert "nvvidconv" not in command
    assert "x264enc" in command
    assert X264_CONVERSION == "videoconvert"


def test_x264_nvvidconv_tokens_match_benchmark_path():
    assert build_conversion_tokens("x264", "nvvidconv") == [
        "videoconvert",
        "!",
        "video/x-raw,format=BGRx",
        "!",
        "nvvidconv",
        "!",
        "video/x-raw,format=I420",
    ]


def test_openh264_keeps_legacy_conversion_independent_of_x264_selection():
    assert build_conversion_tokens("openh264", "nvvidconv") == [
        "videoconvert",
        "!",
        "video/x-raw,format=I420",
    ]


class _Stdin:
    def __init__(self, close_error=None):
        self.writes = 0
        self.closed = False
        self.close_error = close_error

    def write(self, frame):
        self.writes += 1
        return len(frame)

    def close(self):
        self.closed = True
        if self.close_error is not None:
            raise self.close_error


class _Process:
    pid = 123

    def __init__(self):
        self.stdin = _Stdin()
        self.returncode = None
        self.waits = 0

    def poll(self):
        return self.returncode

    def wait(self, timeout=None):
        self.waits += 1
        self.returncode = 0
        return 0


def test_run_benchmark_writes_900_frames_and_reaps_child():
    process = _Process()
    times = iter([10.0, 40.0])
    commands = []

    def popen(command, **kwargs):
        commands.append(command)
        return process

    result = run_benchmark(
        "videoconvert",
        popen=popen,
        monotonic=lambda: next(times),
        read_cpu_ticks=lambda pid: 300.0,
        clock_ticks=100.0,
    )

    assert process.stdin.writes == 900
    assert commands[0][-2:] == ["fakesink", "sync=false"]
    assert process.stdin.closed
    assert process.waits == 1
    assert result == {
        "conversion": "videoconvert",
        "attempted": 900,
        "encoded": 900,
        "elapsed_s": 30.0,
        "fps": 30.0,
        "cpu_percent": 10.0,
        "returncode": 0,
    }


def test_stdin_close_error_still_reaps_child():
    process = _Process()
    process.stdin = _Stdin(BrokenPipeError("child closed first"))
    times = iter([10.0, 40.0])

    result = run_benchmark(
        "videoconvert",
        popen=lambda *args, **kwargs: process,
        monotonic=lambda: next(times),
        read_cpu_ticks=lambda pid: 300.0,
        clock_ticks=100.0,
    )

    assert process.stdin.closed
    assert process.waits == 1
    assert result["returncode"] == 0
