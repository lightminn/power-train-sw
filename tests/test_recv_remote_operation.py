import json
import math
from pathlib import Path
import subprocess
import sys
import queue
import threading
from types import SimpleNamespace

import pytest

import scripts.recv_remote_operation as viewer
from l515_dashboard.receiver_feedback import parse_report
from remote_video.contract import (
    D435I_METADATA_UDP_PORT,
    D435I_RGB_SRT_PORT,
    L515_RGB_SRT_PORT,
    RECEIVER_FEEDBACK_SCHEMA_VERSION,
    RECEIVER_FEEDBACK_UDP_PORT,
)
from scripts.recv_remote_operation import (
    FpsEstimator,
    StatusPanel,
    _ChannelBuffer,
    _MetadataReceiver,
    build_recv_command,
    build_feedback_report,
    parse_args,
    parse_status_line,
    overlay_render_decision,
    should_draw_bboxes,
)


def test_feedback_report_round_trips_through_jetson_parser():
    payload = build_feedback_report(
        "d435i_rgb",
        "receiver-session",
        17,
        29.75,
        29.5,
        41.25,
        2,
        18.0,
        0.5,
    )

    assert set(json.loads(payload)) == {
        "schema_version",
        "channel",
        "session_id",
        "sequence",
        "decode_fps",
        "display_fps",
        "frame_age_ms",
        "sequence_gap",
        "rtt_ms",
        "loss_percent",
    }
    report = parse_report(payload, now_monotonic_ns=123_456)

    assert json.loads(payload)["schema_version"] == RECEIVER_FEEDBACK_SCHEMA_VERSION
    assert report.channel == "d435i_rgb"
    assert report.session_id == "receiver-session"
    assert report.sequence == 17
    assert report.decode_fps == 29.75
    assert report.display_fps == 29.5
    assert report.frame_age_ms == 41.25
    assert report.sequence_gap == 2
    assert report.rtt_ms == 18.0
    assert report.loss_percent == 0.5
    assert report.received_monotonic_ns == 123_456


def test_fps_estimator_uses_injected_two_second_window():
    fps = FpsEstimator(window_s=2.0)

    for timestamp_s in (10.0, 10.5, 11.0, 11.5, 12.0):
        fps.record(timestamp_s)

    assert fps.value(12.0) == pytest.approx(2.0)
    assert fps.value(14.1) == 0.0


def test_channel_metrics_tolerate_receiver_update_after_caller_captured_time():
    channel = _ChannelBuffer("l515_rgb", started_s=0.0)
    channel.receive(object(), now_s=2.0)

    metrics = channel.metrics(now_s=1.0)

    assert metrics["decode_fps"] == 0.0
    assert metrics["frame_age_ms"] == 0.0


def test_metadata_snapshot_tolerates_update_after_caller_captured_time():
    from remote_video.metadata import MetadataPacket

    receiver = _MetadataReceiver(
        5003,
        threading.Event(),
        queue.SimpleQueue(),
    )
    packet = MetadataPacket(
        schema_version=1,
        session_id="session",
        sequence=1,
        source_frame_sequence=1,
        capture_stamp_ns=1,
        detections=(),
    )
    receiver._accept_packet(packet, received_monotonic_ns=2_000_000_000)

    overlay_state, latest, received_ns = receiver.snapshot(
        now_monotonic_ns=1_000_000_000
    )

    assert overlay_state == "FRESH"
    assert latest == packet
    assert received_ns == 2_000_000_000


def test_worker_error_remains_visible_after_worker_requests_stop():
    errors = queue.SimpleQueue()
    stop_event = threading.Event()
    errors.put("metadata bind failed")
    stop_event.set()

    assert viewer._pop_worker_error(errors) == "metadata bind failed"


def test_cli_requires_host_and_uses_channel_contract_defaults():
    args = parse_args(["--host", "jetson.example"])

    assert args.host == "jetson.example"
    assert args.l515_port == L515_RGB_SRT_PORT
    assert args.d435i_port == D435I_RGB_SRT_PORT
    assert args.meta_port == D435I_METADATA_UDP_PORT
    assert args.feedback_port == RECEIVER_FEEDBACK_UDP_PORT
    assert args.latency == 60
    assert args.teleop_port == 9000
    assert args.no_teleop is False

    with pytest.raises(SystemExit):
        parse_args([])


def test_gstreamer_command_matches_recv_yolo3d_raw_bgr_pattern():
    command = build_recv_command(
        "192.0.2.10",
        port=5000,
        width=1280,
        height=720,
        latency=60,
    )

    assert command == [
        "gst-launch-1.0",
        "-q",
        "srtsrc",
        "uri=srt://192.0.2.10:5000?mode=caller&latency=60",
        "!",
        "tsdemux",
        "!",
        "h264parse",
        "!",
        "avdec_h264",
        "max-threads=1",
        "!",
        "videoconvert",
        "!",
        "videoscale",
        "!",
        "video/x-raw,format=BGR,width=1280,height=720",
        "!",
        "fdsink",
        "fd=1",
    ]


def test_module_import_does_not_require_cv2_numpy_or_pygame():
    repo = Path(__file__).resolve().parents[1]
    code = """
import builtins
real_import = builtins.__import__
def guarded(name, *args, **kwargs):
    if name.split('.')[0] in {'cv2', 'numpy', 'pygame'}:
        raise AssertionError('eager runtime import: ' + name)
    return real_import(name, *args, **kwargs)
builtins.__import__ = guarded
import scripts.recv_remote_operation
"""

    result = subprocess.run(
        [sys.executable, "-c", code],
        cwd=repo,
        capture_output=True,
        text=True,
    )

    assert result.returncode == 0, result.stderr


def test_status_panel_keeps_requested_mode_and_jetson_ack_separate():
    lines = StatusPanel.render_lines(
        requested_mode="ARM",
        ack_state="DRIVE",
        deadman=True,
        hold_reason="awaiting Jetson ACK",
        assist_bypass=True,
        l515_age_ms=12.25,
        d435i_age_ms=45.5,
        overlay_state="FRESH",
    )

    assert "REQUESTED MODE: ARM" in lines
    assert "JETSON ACK: DRIVE" in lines
    assert "DEADMAN: PRESSED" in lines
    assert "HOLD: awaiting Jetson ACK" in lines
    assert "ASSIST BYPASS: ON" in lines
    assert "L515 AGE: 12.2 ms" in lines
    assert "D435i AGE: 45.5 ms" in lines
    assert "D435i OVERLAY: FRESH" in lines


def test_status_panel_formats_unavailable_frame_age():
    lines = StatusPanel.render_lines(
        requested_mode="DRIVE",
        ack_state="DISCONNECTED",
        deadman=False,
        hold_reason="status unavailable",
        assist_bypass=False,
        l515_age_ms=math.inf,
        d435i_age_ms=math.inf,
        overlay_state="OVERLAY_STALE",
    )

    assert "DEADMAN: RELEASED" in lines
    assert "ASSIST BYPASS: OFF" in lines
    assert "L515 AGE: unavailable" in lines
    assert "D435i AGE: unavailable" in lines


def test_stale_overlay_hides_bboxes_without_hiding_raw_frame():
    assert should_draw_bboxes("FRESH") is True
    assert should_draw_bboxes("OVERLAY_STALE") is False

    with pytest.raises(ValueError, match="overlay_state"):
        should_draw_bboxes("UNKNOWN")


def test_overlay_render_decision_exposes_source_sequence_and_local_age():
    from remote_video.metadata import MetadataPacket

    packet = MetadataPacket(
        schema_version=1,
        session_id="session",
        sequence=9,
        source_frame_sequence=270,
        capture_stamp_ns=123,
        detections=(),
    )

    decision = overlay_render_decision(
        "FRESH",
        packet,
        now_monotonic_ns=2_125_000_000,
        received_monotonic_ns=2_000_000_000,
    )

    assert decision.draw_bboxes is True
    assert decision.source_frame_sequence == 270
    assert decision.age_ms == pytest.approx(125.0)
    assert decision.provenance_text == "seq=270 age=125.0ms"


def test_draw_metadata_always_renders_fresh_overlay_provenance_text():
    from remote_video.metadata import MetadataPacket

    packet = MetadataPacket(
        schema_version=1,
        session_id="session",
        sequence=9,
        source_frame_sequence=270,
        capture_stamp_ns=123,
        detections=(),
    )
    decision = overlay_render_decision(
        "FRESH",
        packet,
        now_monotonic_ns=2_125_000_000,
        received_monotonic_ns=2_000_000_000,
    )

    class FakeCv2:
        FONT_HERSHEY_SIMPLEX = 0

        def __init__(self):
            self.texts = []

        def putText(self, _frame, text, *_args):
            self.texts.append(text)

        def rectangle(self, *_args):
            raise AssertionError("empty detection packet must not draw boxes")

    cv2 = FakeCv2()
    viewer._draw_metadata(
        cv2,
        SimpleNamespace(shape=(720, 1280, 3)),
        decision,
        packet,
    )

    assert "seq=270 age=125.0ms" in cv2.texts


@pytest.mark.parametrize(
    ("line", "expected"),
    [
        ("S DRIVE +1.500 -0.720", ("DRIVE", 1.5, -0.72)),
        ("  S MOTION_HOLD +0.000 +0.000\n", ("MOTION_HOLD", 0.0, 0.0)),
    ],
)
def test_parse_status_line_accepts_authoritative_status(line, expected):
    assert parse_status_line(line) == expected


@pytest.mark.parametrize(
    "line",
    [
        "",
        "DRIVE +1.0 +0.0",
        "S DRIVE +1.0",
        "S DRIVE fast +0.0",
        "S DRIVE nan +0.0",
        "S DRIVE +0.0 inf",
        "S DRIVE +0.0 +0.0 extra",
    ],
)
def test_parse_status_line_rejects_malformed_status(line):
    assert parse_status_line(line) is None
