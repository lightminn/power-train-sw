"""build_gst_command — 영상 송신 파이프라인(gst-launch argv) 조립 검증."""
import pytest

from gst_stream import ENCODERS, build_gst_command


def test_x264_pipeline_uses_srt_listener_with_requested_latency():
    cmd = build_gst_command(5000, 848, 480, 30, encoder="x264", latency_ms=45)
    assert "x264enc" in cmd
    uri = next(a for a in cmd if a.startswith("uri="))
    assert "srt://:5000" in uri
    assert "mode=listener" in uri  # 로봇이 서빙, 노트북이 접속 — 수신측 재접속 허용
    assert "latency=45" in uri


def test_openh264_is_a_valid_fallback_encoder():
    cmd = build_gst_command(5000, 848, 480, 30, encoder="openh264")
    assert "openh264enc" in cmd


def test_unknown_encoder_rejected():
    with pytest.raises(ValueError):
        build_gst_command(5000, 848, 480, 30, encoder="mpeg2")


def test_all_declared_encoders_are_buildable():
    for enc in ENCODERS:
        cmd = build_gst_command(5000, 640, 480, 30, encoder=enc)
        assert cmd[0] == "gst-launch-1.0"


def test_keyframe_interval_defaults_to_30_for_existing_callers():
    assert "key-int-max=30" in build_gst_command(5000, 640, 480, 15)
    assert "gop-size=30" in build_gst_command(5000, 640, 480, 15, encoder="openh264")


def test_keyframe_interval_can_follow_a_lower_output_rate():
    cmd = build_gst_command(5000, 1280, 720, 15, keyint=15)
    assert "framerate=15/1" in cmd and "key-int-max=15" in cmd
    cmd = build_gst_command(5000, 1280, 720, 15, encoder="openh264", keyint=15)
    assert "gop-size=15" in cmd


@pytest.mark.parametrize("keyint", [0, -1, True, 1.5])
def test_keyframe_interval_must_be_a_positive_integer(keyint):
    with pytest.raises(ValueError, match="keyint"):
        build_gst_command(5000, 640, 480, 30, keyint=keyint)
