import json
from dataclasses import FrozenInstanceError

import pytest

from l515_dashboard.receiver_feedback import (
    FeedbackConfig,
    FeedbackTracker,
    ReceiverReport,
)
from remote_video.contract import MAX_METADATA_BYTES
from remote_video.metadata import (
    Detection,
    MetadataError,
    MetadataPacket,
    MetadataTracker,
    parse_metadata,
)


NS = 1_000_000_000


def metadata_payload(**changes):
    payload = {
        "schema_version": 1,
        "session_id": "session-a",
        "sequence": 9,
        "source_frame_sequence": 270,
        "capture_stamp_ns": 123_456_789,
        "detections": [
            {
                "bbox": [10.0, 20.0, 110.0, 220.0],
                "class_name": "person",
                "class_id": 0,
                "confidence": 0.875,
            }
        ],
    }
    payload.update(changes)
    return payload


def encode_metadata(**changes):
    return json.dumps(metadata_payload(**changes)).encode("utf-8")


def packet(session_id="session-a", sequence=1, capture_stamp_ns=0):
    return MetadataPacket(
        schema_version=1,
        session_id=session_id,
        sequence=sequence,
        source_frame_sequence=sequence * 15,
        capture_stamp_ns=capture_stamp_ns,
        detections=(),
    )


def test_parse_metadata_accepts_v1_and_returns_local_receive_stamp():
    parsed, received_ns = parse_metadata(
        encode_metadata(), now_monotonic_ns=987_654_321
    )

    assert parsed == MetadataPacket(
        schema_version=1,
        session_id="session-a",
        sequence=9,
        source_frame_sequence=270,
        capture_stamp_ns=123_456_789,
        detections=(
            Detection(
                bbox=(10.0, 20.0, 110.0, 220.0),
                class_name="person",
                class_id=0,
                confidence=0.875,
            ),
        ),
    )
    assert received_ns == 987_654_321
    with pytest.raises(FrozenInstanceError):
        parsed.sequence = 10
    with pytest.raises(FrozenInstanceError):
        parsed.detections[0].confidence = 0.5


def test_parse_metadata_rejects_payload_larger_than_16_kib():
    with pytest.raises(MetadataError, match="size"):
        parse_metadata(b"x" * (MAX_METADATA_BYTES + 1), now_monotonic_ns=0)


def test_parse_metadata_rejects_unknown_schema_version():
    with pytest.raises(MetadataError, match="version"):
        parse_metadata(encode_metadata(schema_version=2), now_monotonic_ns=0)


@pytest.mark.parametrize(
    "bbox",
    [
        [10.0, 20.0, 10.0, 30.0],
        [10.0, 20.0, 30.0, 20.0],
        [30.0, 20.0, 10.0, 40.0],
        [10.0, 40.0, 30.0, 20.0],
        [float("nan"), 20.0, 30.0, 40.0],
        [10.0, 20.0, float("inf"), 40.0],
    ],
)
def test_parse_metadata_rejects_nonfinite_or_inverted_bbox(bbox):
    detection = metadata_payload()["detections"][0] | {"bbox": bbox}

    with pytest.raises(MetadataError, match="bbox"):
        parse_metadata(encode_metadata(detections=[detection]), now_monotonic_ns=0)


@pytest.mark.parametrize("confidence", [-0.1, 1.1, float("nan"), float("inf")])
def test_parse_metadata_rejects_confidence_outside_unit_interval(confidence):
    detection = metadata_payload()["detections"][0] | {
        "confidence": confidence
    }

    with pytest.raises(MetadataError, match="confidence"):
        parse_metadata(encode_metadata(detections=[detection]), now_monotonic_ns=0)


@pytest.mark.parametrize(
    ("changes", "message"),
    [
        ({"session_id": ""}, "session_id"),
        ({"sequence": True}, "sequence"),
        ({"sequence": -1}, "sequence"),
        ({"source_frame_sequence": -1}, "source_frame_sequence"),
        ({"capture_stamp_ns": -1}, "capture_stamp_ns"),
        ({"detections": {}}, "detections"),
    ],
)
def test_parse_metadata_rejects_invalid_packet_fields(changes, message):
    with pytest.raises(MetadataError, match=message):
        parse_metadata(encode_metadata(**changes), now_monotonic_ns=0)


def test_parse_metadata_rejects_invalid_json_and_extra_fields():
    with pytest.raises(MetadataError, match="JSON"):
        parse_metadata(b"not-json", now_monotonic_ns=0)
    with pytest.raises(MetadataError, match="fields"):
        parse_metadata(encode_metadata(extra=True), now_monotonic_ns=0)


def test_tracker_rejects_nonincreasing_sequence_in_same_session():
    tracker = MetadataTracker()

    assert tracker.update(packet(sequence=4), received_monotonic_ns=10) is True
    assert tracker.update(packet(sequence=3), received_monotonic_ns=20) is False
    assert tracker.update(packet(sequence=4), received_monotonic_ns=30) is False
    assert tracker.latest == packet(sequence=4)


def test_tracker_accepts_new_session_with_reset_sequence():
    tracker = MetadataTracker()

    assert tracker.update(packet("old", 99), received_monotonic_ns=10) is True
    assert tracker.update(packet("new", 0), received_monotonic_ns=20) is True
    assert tracker.latest == packet("new", 0)


def test_tracker_rejects_delayed_packet_from_retired_session():
    tracker = MetadataTracker()

    assert tracker.update(packet("old", 100), received_monotonic_ns=10) is True
    assert tracker.update(packet("new", 0), received_monotonic_ns=20) is True
    assert tracker.update(packet("old", 99), received_monotonic_ns=30) is False
    assert tracker.latest == packet("new", 0)


def test_tracker_retired_session_memory_is_finite():
    tracker = MetadataTracker()

    for index in range(100):
        assert tracker.update(
            packet(f"session-{index}", 0), received_monotonic_ns=index
        ) is True

    assert len(tracker._retired_session_ids) == tracker.MAX_RETIRED_SESSIONS


def test_tracker_is_latest_only_and_has_no_backlog():
    tracker = MetadataTracker()

    for sequence in range(100):
        tracker.update(packet(sequence=sequence), received_monotonic_ns=sequence)

    assert tracker.latest == packet(sequence=99)
    assert not hasattr(tracker, "queue")


def test_overlay_state_is_stale_without_data_and_after_local_receive_ttl():
    tracker = MetadataTracker(ttl_s=0.5)
    assert tracker.overlay_state(0) == "OVERLAY_STALE"

    tracker.update(packet(), received_monotonic_ns=10 * NS)

    assert tracker.overlay_state(10 * NS + 500_000_000) == "FRESH"
    assert tracker.overlay_state(10 * NS + 500_000_001) == "OVERLAY_STALE"


def test_capture_stamp_is_not_used_for_freshness():
    tracker = MetadataTracker(ttl_s=0.5)
    tracker.update(
        packet(capture_stamp_ns=1),
        received_monotonic_ns=500 * NS,
    )

    assert tracker.overlay_state(500 * NS + 100_000_000) == "FRESH"


def test_two_hz_yolo_metadata_does_not_reduce_raw_receiver_availability():
    feedback = FeedbackTracker(
        FeedbackConfig(report_ttl_s=1.0, exit_dwell_s=0.0)
    )
    metadata = MetadataTracker(ttl_s=0.6)
    metadata_sequence = 0

    for raw_sequence in range(61):
        now_ns = raw_sequence * NS // 30
        feedback.update(
            ReceiverReport(
                channel="d435i_rgb",
                session_id="raw-session",
                sequence=raw_sequence,
                decode_fps=30.0,
                display_fps=30.0,
                frame_age_ms=25.0,
                sequence_gap=0,
                rtt_ms=10.0,
                loss_percent=0.0,
                received_monotonic_ns=now_ns,
            )
        )
        if raw_sequence % 15 == 0:
            metadata.update(
                packet(
                    session_id="yolo-session",
                    sequence=metadata_sequence,
                    capture_stamp_ns=raw_sequence,
                ),
                received_monotonic_ns=now_ns,
            )
            metadata_sequence += 1

    assert metadata_sequence == 5
    assert metadata.latest == packet(
        session_id="yolo-session",
        sequence=4,
        capture_stamp_ns=60,
    )
    assert feedback.availability("d435i_rgb", 2 * NS).available is True
