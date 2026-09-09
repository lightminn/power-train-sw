"""Actual GTK, authenticated TCP and decoded SRT; no robot or physical pad.

Run with system Python under Xvfb (GDK_BACKEND=x11, GDK_SCALE=1).
Each scenario owns a child process so GTK/GStreamer lifetime is isolated.
"""
import json
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys

import pytest


ROOT = Path(__file__).resolve().parents[2]


@pytest.fixture(scope="module", autouse=True)
def require_decoded_srt_plugins():
    if not shutil.which("gst-inspect-1.0"):
        pytest.skip("Actual decoded SRT gate requires gst-inspect-1.0")
    required = ("videotestsrc", "x264enc", "h264parse", "mpegtsmux", "srtsink",
                "srtsrc", "tsdemux", "avdec_h264", "videoconvert", "gtksink")
    missing = [name for name in required if subprocess.run(
        ["gst-inspect-1.0", name], stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL, timeout=5).returncode]
    if missing:
        pytest.skip("Actual decoded SRT gate missing plugins: " + ", ".join(missing))


@pytest.mark.parametrize("scenario", ["endpoint", "retry", "operation"])
def test_actual_console_operation_and_video_restart(scenario):
    if not os.environ.get("DISPLAY"):
        pytest.skip("Actual GTK/SRT gate requires an X11 display (use Xvfb)")
    pytest.importorskip("gi", reason="Actual GTK/SRT gate requires PyGObject")
    if not shutil.which("gst-launch-1.0"):
        pytest.skip("Actual decoded SRT gate requires gst-launch-1.0")
    child = subprocess.Popen(
        [sys.executable, "-m", "operator_console.tests._console_operation_e2e", scenario],
        cwd=ROOT,
        env=dict(os.environ, GDK_BACKEND="x11", GDK_SCALE="1", GDK_DPI_SCALE="1",
                 PYTHONPATH=os.pathsep.join((str(ROOT), str(ROOT / "motor_control"),
                                            str(ROOT / "ros2/src/powertrain_ros")))),
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, start_new_session=True,
    )
    try:
        out, err = child.communicate(timeout=75)
    except subprocess.TimeoutExpired:
        os.killpg(child.pid, signal.SIGTERM)
        try:
            out, err = child.communicate(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(child.pid, signal.SIGKILL)
            out, err = child.communicate(timeout=5)
        pytest.fail(f"{scenario} timed out (fixture group stopped):\n{out}\n{err}")
    assert child.returncode == 0, f"{scenario}:\n{out}\n{err}"
    assert "Traceback" not in err, err
    report = json.loads(out.splitlines()[-1])
    assert report["result"] == "PASS", report
    assert report["cleanup"] == "sender reaped; TCP listeners closed", report
