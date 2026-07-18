from operator_console.metadata import parse_metadata
from powertrain_ros.arm_console_mirror import build_detection_metadata_payload


def test_arm_detection_payload_round_trips_through_console_parser():
    encoded = build_detection_metadata_payload(
        capture_stamp_ns=987654321,
        frame_id="camera_link",
        frame_width=848,
        frame_height=480,
        detections=[
            (7, "crate", 0.875, (10, 20, 30, 40), (1.0, 2.0, 3.0), 0.5)
        ],
        pick_target=None,
    )

    frame = parse_metadata(encoded, received_monotonic_s=12.0)

    assert frame.sequence == 987654321
    assert frame.width == 848
    assert frame.height == 480
    assert len(frame.detections) == 1
    detection = frame.detections[0]
    assert detection.class_name == "crate"
    assert detection.confidence == 0.875
    assert detection.bbox_xywh == (10, 20, 30, 40)
    assert detection.position_m == (1.0, 2.0, 3.0)
