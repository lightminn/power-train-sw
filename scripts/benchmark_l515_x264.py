#!/usr/bin/env python3
"""Benchmark Orin Nano BGR conversion paths feeding software x264."""

import argparse
import json
import os
import subprocess
import time
from collections.abc import Callable
from typing import Any


WIDTH = 1280
HEIGHT = 720
FPS = 30
FRAME_COUNT = 900


def build_x264_benchmark_command(
    width: int, height: int, fps: int, conversion: str, sink: str
) -> list[str]:
    if conversion not in {"videoconvert", "nvvidconv"}:
        raise ValueError("conversion must be videoconvert or nvvidconv")
    convert = (
        ["videoconvert", "!", "video/x-raw,format=I420"]
        if conversion == "videoconvert"
        else [
            "videoconvert",
            "!",
            "video/x-raw,format=BGRx",
            "!",
            "nvvidconv",
            "!",
            "video/x-raw,format=I420",
        ]
    )
    return [
        "gst-launch-1.0",
        "fdsrc",
        "fd=0",
        "do-timestamp=true",
        "!",
        "rawvideoparse",
        "format=bgr",
        f"width={width}",
        f"height={height}",
        f"framerate={fps}/1",
        "!",
        *convert,
        "!",
        "x264enc",
        "tune=zerolatency",
        "speed-preset=superfast",
        "threads=2",
        "bitrate=3000",
        "key-int-max=30",
        "!",
        sink,
    ]


def _read_cpu_ticks(pid: int) -> float:
    with open(f"/proc/{pid}/stat", encoding="ascii") as stat_file:
        fields = stat_file.read().split()
    return float(int(fields[13]) + int(fields[14]))


def _reap(process: Any) -> int:
    try:
        return int(process.wait(timeout=10))
    except subprocess.TimeoutExpired:
        process.terminate()
    try:
        return int(process.wait(timeout=2))
    except subprocess.TimeoutExpired:
        process.kill()
        try:
            return int(process.wait(timeout=2))
        except subprocess.TimeoutExpired:
            return -1


def run_benchmark(
    conversion: str,
    *,
    sink: str = "fakesink",
    popen: Callable[..., Any] = subprocess.Popen,
    monotonic: Callable[[], float] = time.monotonic,
    read_cpu_ticks: Callable[[int], float] = _read_cpu_ticks,
    clock_ticks: float | None = None,
) -> dict[str, object]:
    """Write exactly 900 reusable BGR frames, then close and reap GStreamer."""
    command = build_x264_benchmark_command(WIDTH, HEIGHT, FPS, conversion, sink)
    if sink == "fakesink":
        command.append("sync=false")
    process = popen(command, stdin=subprocess.PIPE)
    if process.stdin is None:
        _reap(process)
        raise RuntimeError("benchmark child has no stdin")

    frame = bytes(WIDTH * HEIGHT * 3)
    attempted = 0
    encoded = 0
    start = monotonic()
    try:
        for _ in range(FRAME_COUNT):
            if process.poll() is not None:
                break
            attempted += 1
            try:
                process.stdin.write(frame)
            except (BrokenPipeError, OSError):
                break
            encoded += 1
    finally:
        try:
            cpu_ticks = read_cpu_ticks(process.pid)
        except (FileNotFoundError, ProcessLookupError):
            cpu_ticks = 0.0
        try:
            process.stdin.close()
        except (BrokenPipeError, OSError):
            pass
        finally:
            returncode = _reap(process)

    elapsed = max(monotonic() - start, 1e-9)
    ticks_per_second = clock_ticks or float(os.sysconf("SC_CLK_TCK"))
    return {
        "conversion": conversion,
        "attempted": attempted,
        "encoded": encoded,
        "elapsed_s": round(elapsed, 3),
        "fps": round(encoded / elapsed, 3),
        "cpu_percent": round(cpu_ticks / ticks_per_second / elapsed * 100.0, 3),
        "returncode": returncode,
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("conversion", choices=("videoconvert", "nvvidconv"))
    parser.add_argument("--sink", default="fakesink")
    args = parser.parse_args()
    result = run_benchmark(args.conversion, sink=args.sink)
    print(json.dumps(result, sort_keys=True))
    return 0 if (
        result["returncode"] == 0
        and result["encoded"] == FRAME_COUNT
        and result["fps"] >= 29.0
    ) else 1


if __name__ == "__main__":
    raise SystemExit(main())
