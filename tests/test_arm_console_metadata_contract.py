from operator_console.arm_telemetry import parse_arm_telemetry
from operator_console.metadata import parse_metadata
from powertrain_ros.arm_console_mirror import (
    DynamixelMotor,
    build_arm_telemetry_payload,
    build_detection_metadata_payload,
)


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


def test_arm_telemetry_payload_round_trips_through_console_parser():
    encoded = build_arm_telemetry_payload(
        sequence=42,
        stamp_s=123.5,
        motors=(
            DynamixelMotor(
                id=11,
                position_raw=2048,
                position_deg=0.0,
                velocity=7,
                current=-12,
                temperature_c=34,
            ),
        ),
        joints={
            "names": ("joint_a", "joint_b"),
            "position_rad": (0.25, -0.5),
            "velocity": (0.1, -0.2),
        },
        source_age_s={
            "dynamixel": 0.1,
            "joints": 0.2,
            "detections": 0.3,
        },
    )

    snapshot = parse_arm_telemetry(encoded, received_monotonic_s=12.0)

    assert snapshot.sequence == 42
    assert snapshot.dynamixel is not None
    assert snapshot.dynamixel[0].id == 11
    assert snapshot.dynamixel[0].current == -12
    assert snapshot.joint_names == ("joint_a", "joint_b")
    assert snapshot.joint_position_rad == (0.25, -0.5)
    assert snapshot.joint_velocity == (0.1, -0.2)
    assert snapshot.dynamixel_age_s == 0.1
    assert snapshot.joints_age_s == 0.2
    assert snapshot.detections_age_s == 0.3
