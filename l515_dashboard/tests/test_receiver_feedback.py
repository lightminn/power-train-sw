import json
from dataclasses import FrozenInstanceError

import pytest

from l515_dashboard.receiver_feedback import (
    D435I_UNAVAILABLE_VERDICT,
    L515_UNAVAILABLE_VERDICT,
    FeedbackConfig,
    FeedbackError,
    FeedbackTracker,
    ReceiverReport,
    parse_report,
)


NS = 1_000_000_000


def report_payload(**changes):
    payload = {
        "schema_version": 1,
        "channel": "l515_rgb",
        "session_id": "session-a",
        "sequence": 7,
        "decode_fps": 30.0,
        "display_fps": 29.8,
        "frame_age_ms": 42.5,
        "sequence_gap": 1,
        "rtt_ms": 18.0,
        "loss_percent": 0.5,
    }
    payload.update(changes)
    return payload


def encode_report(**changes):
    return json.dumps(report_payload(**changes)).encode("utf-8")


def make_report(
    channel="l515_rgb",
    *,
    session_id="session-a",
    sequence=1,
    display_fps=30.0,
    loss_percent=0.0,
    received_monotonic_ns=0,
):
    return ReceiverReport(
        channel=channel,
        session_id=session_id,
        sequence=sequence,
        decode_fps=30.0,
        display_fps=display_fps,
        frame_age_ms=20.0,
        sequence_gap=0,
        rtt_ms=10.0,
        loss_percent=loss_percent,
        received_monotonic_ns=received_monotonic_ns,
    )


def test_parse_report_accepts_v1_and_stamps_local_receive_time():
    parsed = parse_report(encode_report(), now_monotonic_ns=123_456)

    assert parsed == ReceiverReport(
        channel="l515_rgb",
        session_id="session-a",
        sequence=7,
        decode_fps=30.0,
        display_fps=29.8,
        frame_age_ms=42.5,
        sequence_gap=1,
        rtt_ms=18.0,
        loss_percent=0.5,
        received_monotonic_ns=123_456,
    )
    with pytest.raises(FrozenInstanceError):
        parsed.sequence = 8


def test_parse_report_rejects_oversize_payload():
    with pytest.raises(FeedbackError, match="size"):
        parse_report(b"x" * 4097, now_monotonic_ns=0)


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"schema_version": 2}, "version"),
        ({"channel": "depth"}, "channel"),
        ({"display_fps": float("nan")}, "display_fps"),
        ({"rtt_ms": float("inf")}, "rtt_ms"),
        ({"decode_fps": -0.1}, "decode_fps"),
        ({"display_fps": -0.1}, "display_fps"),
        ({"loss_percent": -0.1}, "loss_percent"),
    ],
)
def test_parse_report_rejects_invalid_version_channel_and_quality(changes, message):
    with pytest.raises(FeedbackError, match=message):
        parse_report(encode_report(**changes), now_monotonic_ns=0)


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"session_id": ""}, "session_id"),
        ({"sequence": True}, "sequence"),
        ({"sequence": -1}, "sequence"),
        ({"sequence_gap": -1}, "sequence_gap"),
        ({"frame_age_ms": -1.0}, "frame_age_ms"),
        ({"loss_percent": 100.1}, "loss_percent"),
    ],
)
def test_parse_report_rejects_invalid_identity_counters_and_ranges(changes, message):
    with pytest.raises(FeedbackError, match=message):
        parse_report(encode_report(**changes), now_monotonic_ns=0)


def test_parse_report_rejects_invalid_json_shape_and_extra_fields():
    with pytest.raises(FeedbackError, match="JSON"):
        parse_report(b"not-json", now_monotonic_ns=0)
    with pytest.raises(FeedbackError, match="object"):
        parse_report(b"[]", now_monotonic_ns=0)
    with pytest.raises(FeedbackError, match="fields"):
        parse_report(encode_report(extra=True), now_monotonic_ns=0)


def test_tracker_rejects_nonincreasing_sequence_in_same_session():
    tracker = FeedbackTracker(FeedbackConfig(exit_dwell_s=0.0))

    assert tracker.update(make_report(sequence=4)) is True
    assert tracker.update(make_report(sequence=3, display_fps=0.0)) is False
    assert tracker.update(make_report(sequence=4, display_fps=0.0)) is False
    assert tracker.availability("l515_rgb", 0).available is True


def test_tracker_accepts_session_replacement_and_resets_sequence():
    tracker = FeedbackTracker(FeedbackConfig(exit_dwell_s=0.0))

    assert tracker.update(make_report(session_id="old", sequence=99)) is True
    assert tracker.update(make_report(session_id="new", sequence=0)) is True
    availability = tracker.availability("l515_rgb", 0)

    assert availability.available is True
    assert availability.loss_percent == 0.0


def test_tracker_marks_fresh_report_stale_after_ttl():
    tracker = FeedbackTracker(
        FeedbackConfig(report_ttl_s=2.0, exit_dwell_s=0.0)
    )
    tracker.update(make_report(received_monotonic_ns=10 * NS))

    assert tracker.availability("l515_rgb", 12 * NS).available is True
    stale = tracker.availability("l515_rgb", 12 * NS + 1)

    assert stale.available is False
    assert stale.reason == "receiver_report_stale"
    assert stale.verdict == L515_UNAVAILABLE_VERDICT


def test_tracker_applies_fps_hysteresis_and_entry_recovery_dwells():
    tracker = FeedbackTracker(
        FeedbackConfig(
            report_ttl_s=10.0,
            enter_fps=29.0,
            exit_fps=29.5,
            enter_dwell_s=1.0,
            exit_dwell_s=2.0,
        )
    )

    tracker.update(make_report(sequence=1, received_monotonic_ns=0))
    assert tracker.availability("l515_rgb", 2 * NS).available is True

    tracker.update(
        make_report(sequence=2, display_fps=28.9, received_monotonic_ns=3 * NS)
    )
    assert tracker.availability("l515_rgb", 3 * NS + 999_999_999).available is True
    tracker.update(
        make_report(sequence=3, display_fps=28.8, received_monotonic_ns=4 * NS)
    )
    assert tracker.availability("l515_rgb", 4 * NS).available is False

    tracker.update(
        make_report(sequence=4, display_fps=29.2, received_monotonic_ns=5 * NS)
    )
    assert tracker.availability("l515_rgb", 5 * NS).available is False
    tracker.update(
        make_report(sequence=5, display_fps=29.6, received_monotonic_ns=6 * NS)
    )
    assert tracker.availability("l515_rgb", 7 * NS + 999_999_999).available is False
    tracker.update(
        make_report(sequence=6, display_fps=29.7, received_monotonic_ns=8 * NS)
    )
    assert tracker.availability("l515_rgb", 8 * NS).available is True


def test_tracker_is_fail_closed_before_any_report_for_each_channel():
    tracker = FeedbackTracker(FeedbackConfig())

    drive = tracker.availability("l515_rgb", 0)
    arm = tracker.availability("d435i_rgb", 0)

    assert (drive.available, drive.verdict, drive.reason) == (
        False,
        L515_UNAVAILABLE_VERDICT,
        "no_receiver_report",
    )
    assert (arm.available, arm.verdict, arm.reason) == (
        False,
        D435I_UNAVAILABLE_VERDICT,
        "no_receiver_report",
    )


def test_tracker_rejects_unknown_channel_queries():
    tracker = FeedbackTracker(FeedbackConfig())

    with pytest.raises(ValueError, match="channel"):
        tracker.availability("unknown", 0)
