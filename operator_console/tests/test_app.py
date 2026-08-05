import importlib
import json
import math
from pathlib import Path
import sys

import pytest

from operator_console.pipelines import pipeline_description, srt_uri
from operator_console.metadata import (
    Detection,
    DisplayTargetTracker,
    MetadataFrame,
    displayable_detections,
    parse_metadata,
    pick_display_target,
    target_distance_m,
)
from operator_console import telemetry
from operator_console.telemetry import parse_telemetry


def test_srt_uri_uses_operator_caller_mode():
    assert srt_uri("192.168.8.106", 5002, 60) == (
        "srt://192.168.8.106:5002?mode=caller&latency=60"
    )


def test_d435_pipeline_is_low_latency_h264_receiver():
    pipeline = pipeline_description("robot", 5002, 60)
    assert "srtsrc" in pipeline
    assert "avdec_h264 max-threads=1" in pipeline
    assert "gtksink name=video_sink" in pipeline
    assert "xvimagesink" not in pipeline
    assert "sync=false" in pipeline


def test_metadata_contract_keeps_bbox_and_optical_position():
    frame = parse_metadata(
        b'{"schema_version":1,"capture_sequence":7,"frame_width":848,'
        b'"frame_height":480,"detections":[{"class_name":"bottle",'
        b'"confidence":0.91,"bbox_xywh":[10,20,30,40],'
        b'"position_m":[0.1,-0.2,0.8]}]}', received_monotonic_s=10.0)
    assert frame.sequence == 7
    assert frame.detections[0].bbox_xywh == (10, 20, 30, 40)
    assert frame.detections[0].position_m == (0.1, -0.2, 0.8)


def test_target_distance_matches_sdk_depth_axis():
    frame = parse_metadata(
        b'{"schema_version":1,"capture_sequence":8,"frame_width":848,'
        b'"frame_height":480,"detections":[{"class_name":"box",'
        b'"confidence":0.9,"bbox_xywh":[10,20,30,40],'
        b'"position_m":[0.3,0.4,1.2]}]}',
        received_monotonic_s=10.0,
    )

    assert target_distance_m(frame.detections[0]) == pytest.approx(1.2)
    assert frame.detections[0].yaw_rad is None
    assert frame.detections[0].is_pick_target is False


def test_distance_uses_sdk_depth_not_3d_range():
    """실기 대조값: position (0.0854,0.0704,0.2882) 의 SDK depth 는 0.2882 m,
    3D magnitude 는 0.3199 m. 콘솔 정본은 depth 다."""
    detection = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.0854, 0.0704, 0.2882),
    )
    assert target_distance_m(detection) == pytest.approx(0.2882, abs=1e-4)


def test_explicit_depth_field_overrides_position_z():
    detection = Detection(
        "box-segmentation", 0.96, (0, 0, 10, 10), (0.1, 0.1, 0.30),
        depth_m=0.2884,
    )
    assert target_distance_m(detection) == pytest.approx(0.2884, abs=1e-4)


def test_parse_metadata_reads_optional_depth_m():
    frame = parse_metadata(
        b'{"schema_version":1,"capture_sequence":1,"frame_width":848,'
        b'"frame_height":480,"detections":[{"class_name":"box-segmentation",'
        b'"confidence":0.9,"bbox_xywh":[1,2,3,4],'
        b'"position_m":[0.1,0.1,0.3],"depth_m":0.288}]}'
    )
    assert frame.detections[0].depth_m == pytest.approx(0.288)


def test_non_positive_depth_is_unavailable():
    assert target_distance_m(
        Detection("x", 0.9, (0, 0, 1, 1), (0.1, 0.1, 0.0))
    ) is None


def test_metadata_contract_keeps_yaw_and_pick_target_marker():
    frame = parse_metadata(
        b'{"schema_version":1,"capture_sequence":8,"frame_width":848,'
        b'"frame_height":480,"detections":[{"class_name":"bottle",'
        b'"confidence":0.91,"bbox_xywh":[10,20,30,40],'
        b'"position_m":null,"yaw_rad":0.75,"is_pick_target":true}]}',
        received_monotonic_s=10.0,
    )

    assert frame.detections[0].yaw_rad == 0.75
    assert frame.detections[0].is_pick_target is True


def test_metadata_skips_degenerate_bbox_without_dropping_valid_detection():
    payload = {
        "schema_version": 1,
        "capture_sequence": 9,
        "capture_stamp_ns": 123456,
        "frame_width": 848,
        "frame_height": 480,
        "frame_id": "camera_color_optical_frame",
        "detections": [
            {
                "class_name": "valid",
                "confidence": 0.9,
                "bbox_xywh": [10, 20, 30, 40],
                "position_m": None,
            },
            {
                "class_name": "degenerate",
                "confidence": 0.8,
                "bbox_xywh": [50, 60, 0, 20],
                "position_m": None,
            },
        ],
    }

    frame = parse_metadata(json.dumps(payload).encode("utf-8"))

    assert [item.class_name for item in frame.detections] == ["valid"]


def test_metadata_contract_rejects_non_finite_yaw():
    payload = {
        "schema_version": 1,
        "capture_sequence": 9,
        "capture_stamp_ns": 123456,
        "frame_width": 848,
        "frame_height": 480,
        "frame_id": "camera_color_optical_frame",
        "detections": [{
            "class_name": "bottle",
            "confidence": 0.91,
            "bbox_xywh": [10, 20, 30, 40],
            "position_m": None,
            "yaw_rad": math.inf,
        }],
    }

    with pytest.raises(ValueError):
        parse_metadata(json.dumps(payload).encode("utf-8"))


def test_metadata_invalid_depth_is_unavailable_not_a_display_number():
    for depth in (0.0, math.nan, math.inf):
        payload = {
            "schema_version": 1,
            "capture_sequence": 10,
            "frame_width": 848,
            "frame_height": 480,
            "detections": [{
                "class_name": "box",
                "confidence": 0.9,
                "bbox_xywh": [10, 20, 30, 40],
                "position_m": [0.0, 0.0, depth],
            }],
        }
        frame = parse_metadata(json.dumps(payload).encode("utf-8"))
        assert frame.detections[0].position_m is None


def test_overlay_uses_deployed_confidence_threshold_without_mutating_raw_data():
    payload = {
        "schema_version": 1,
        "capture_sequence": 11,
        "capture_stamp_ns": 123456,
        "frame_width": 848,
        "frame_height": 480,
        "frame_id": "camera_color_optical_frame",
        "detections": [
            {"class_name": "box-segmentation", "confidence": 0.49,
             "bbox_xywh": [1, 0, 320, 473], "position_m": [0, 0, 0.64]},
            {"class_name": "box-segmentation", "confidence": 0.50,
             "bbox_xywh": [10, 20, 30, 40], "position_m": [0, 0, 0.32]},
        ],
    }
    frame = parse_metadata(json.dumps(payload).encode("utf-8"))

    assert len(frame.detections) == 2
    assert [item.confidence for item in displayable_detections(frame)] == [0.50]
    assert frame.capture_stamp_ns == 123456


def test_overlay_rejects_detection_clipped_on_three_frame_borders():
    frame = parse_metadata(json.dumps({
        "schema_version": 1,
        "capture_sequence": 9,
        "frame_width": 848,
        "frame_height": 480,
        "detections": [{
            "class_name": "box-segmentation",
            "confidence": 0.86,
            "bbox_xywh": [1, 0, 320, 476],
            "position_m": [0, 0, 0.67],
        }],
    }).encode(), received_monotonic_s=10.0)

    assert displayable_detections(frame) == ()
    assert len(frame.detections) == 1
    assert frame.source_camera_id == "work"


def test_ui_never_promotes_an_unselected_background_distance_to_target():
    payload = {
        "schema_version": 1, "capture_sequence": 12,
        "frame_width": 848, "frame_height": 480,
        "detections": [
            {"class_name": "box-segmentation", "confidence": 0.9,
             "bbox_xywh": [0, 0, 300, 470],
             "position_m": [0, 0, 3.0], "is_pick_target": False},
            {"class_name": "box-segmentation", "confidence": 0.8,
             "bbox_xywh": [300, 150, 120, 100],
             "position_m": [0, 0, 0.32], "is_pick_target": True},
        ],
    }
    frame = parse_metadata(json.dumps(payload).encode("utf-8"))
    assert pick_display_target(frame).position_m[2] == 0.32

    payload["detections"][1]["is_pick_target"] = False
    frame = parse_metadata(json.dumps(payload).encode("utf-8"))
    assert pick_display_target(frame) is None


def _target_frame(
    sequence, distance, *, bbox=(100, 100, 120, 100),
    confidence=0.8, is_pick_target=True, received=10.0,
):
    payload = {
        "schema_version": 1, "capture_sequence": sequence,
        "frame_width": 848, "frame_height": 480,
        "detections": [{
            "class_name": "box-segmentation", "confidence": confidence,
            "bbox_xywh": list(bbox), "position_m": [0, 0, distance],
            "is_pick_target": is_pick_target,
        }],
    }
    return parse_metadata(
        json.dumps(payload).encode("utf-8"),
        received_monotonic_s=received,
    )


def _frame(detections, sequence=1, now_s=100.0):
    return MetadataFrame(
        sequence=sequence, width=848, height=480,
        detections=tuple(detections), received_monotonic_s=now_s,
        frame_id="camera_color_optical_frame",
    )


def test_no_pick_target_means_no_distance_number():
    """송신자가 지정하지 않은 물체를 '대상'으로 승격하지 않는다."""
    tracker = DisplayTargetTracker()
    high_conf = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.09, 0.07, 0.288), is_pick_target=False,
    )
    view = tracker.update(_frame([high_conf]), now_s=100.0)
    assert view.detection is None
    assert view.distance_m is None
    assert view.distance_state == "대상 탐지 대기"


def test_designated_pick_target_still_reports_depth():
    tracker = DisplayTargetTracker()
    target = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.09, 0.07, 0.288), is_pick_target=True,
    )
    view = tracker.update(_frame([target]), now_s=100.0)
    assert view.distance_m == pytest.approx(0.288, abs=1e-3)


def test_iou_continuity_survives_a_frame_without_the_flag():
    """한 번 지정된 대상은 다음 프레임에서 플래그가 빠져도 같은 상자면 유지된다."""
    tracker = DisplayTargetTracker()
    first = Detection(
        "box-segmentation", 0.96, (500, 290, 220, 190),
        (0.09, 0.07, 0.288), is_pick_target=True,
    )
    tracker.update(_frame([first], sequence=1), now_s=100.0)
    drifted = Detection(
        "box-segmentation", 0.95, (503, 293, 219, 187),
        (0.09, 0.07, 0.290), is_pick_target=False,
    )
    view = tracker.update(
        _frame([drifted], sequence=2, now_s=100.1), now_s=100.1,
    )
    assert view.detection is not None
    assert view.distance_m == pytest.approx(0.290, abs=2e-2)


def test_unknown_frame_id_refuses_to_report_distance():
    """좌표계가 바뀌면 depth 의 의미가 달라진다 — 숫자를 내지 않는다."""
    tracker = DisplayTargetTracker()
    target = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.09, 0.07, 0.288), is_pick_target=True,
    )
    frame = MetadataFrame(
        sequence=1, width=848, height=480, detections=(target,),
        received_monotonic_s=100.0, frame_id="base_link",
    )
    view = tracker.update(frame, now_s=100.0)
    assert view.distance_m is None
    assert view.distance_state == "거리 기준 불일치"


def test_missing_frame_id_is_accepted_for_backward_compatibility():
    tracker = DisplayTargetTracker()
    target = Detection(
        "box-segmentation", 0.96, (503, 293, 219, 187),
        (0.09, 0.07, 0.288), is_pick_target=True,
    )
    frame = MetadataFrame(
        sequence=1, width=848, height=480, detections=(target,),
        received_monotonic_s=100.0, frame_id=None,
    )
    assert tracker.update(frame, now_s=100.0).distance_m == pytest.approx(0.288)


def test_long_dropout_clears_the_distance_filter():
    """10초 공백 뒤 재획득한 첫 값이 과거 EMA 와 섞이면 안 된다."""
    tracker = DisplayTargetTracker()
    near = Detection(
        "box-segmentation", 0.96, (500, 290, 220, 190),
        (0.0, 0.0, 0.30), is_pick_target=True,
    )
    for sequence in range(1, 6):
        tracker.update(
            _frame([near], sequence=sequence, now_s=100.0 + sequence * 0.1),
            now_s=100.0 + sequence * 0.1,
        )
    tracker.update(None, now_s=110.0)
    far = Detection(
        "box-segmentation", 0.96, (500, 290, 220, 190),
        (0.0, 0.0, 1.20), is_pick_target=True,
    )
    view = tracker.update(
        _frame([far], sequence=99, now_s=110.5), now_s=110.5,
    )
    assert view.distance_m == pytest.approx(1.20, abs=1e-3)


def test_display_target_tracker_rejects_single_tenfold_outlier():
    tracker = DisplayTargetTracker()
    for sequence, distance in enumerate((0.30, 0.31, 0.32), 1):
        view = tracker.update(
            _target_frame(sequence, distance), now_s=10.0,
        )
        assert view.distance_m is not None
    outlier = tracker.update(_target_frame(4, 3.0), now_s=10.0)
    assert outlier.distance_m is None
    assert outlier.distance_state == "거리 갱신 중"
    recovered = tracker.update(_target_frame(5, 0.33), now_s=10.0)
    assert recovered.distance_m < 0.34


def test_display_target_tracker_rejects_large_same_target_jump():
    tracker = DisplayTargetTracker()
    for sequence, distance in enumerate((0.30, 0.31, 0.32), 1):
        tracker.update(_target_frame(sequence, distance), now_s=10.0)

    samples = [
        tracker.update(_target_frame(sequence, distance), now_s=10.0)
        for sequence, distance in enumerate((3.08, 3.10, 3.09), 4)
    ]

    assert all(sample.distance_m is None for sample in samples)
    assert all(sample.distance_state == "거리 갱신 중" for sample in samples)


def test_display_target_tracker_accepts_centimeter_range_depth():
    tracker = DisplayTargetTracker()

    view = tracker.update(_target_frame(1, 0.34), now_s=10.0)

    assert view.distance_m == pytest.approx(0.34)


def test_display_target_tracker_resets_on_target_geometry_change():
    tracker = DisplayTargetTracker()
    tracker.update(_target_frame(1, 0.32), now_s=10.0)
    changed = tracker.update(
        _target_frame(2, 1.2, bbox=(600, 300, 80, 60)),
        now_s=10.0,
    )
    assert changed.distance_m == pytest.approx(1.2)


def test_display_target_tracker_hides_stale_distance():
    tracker = DisplayTargetTracker()
    stale = tracker.update(
        _target_frame(1, 0.32, received=10.0), now_s=10.6,
    )
    assert stale.distance_m is None
    assert stale.distance_state == "거리 정보 지연"


def test_display_target_tracker_holds_one_short_empty_detection_frame():
    tracker = DisplayTargetTracker()
    visible = tracker.update(
        _target_frame(1, 0.32, received=10.0), now_s=10.0,
    )
    empty = parse_metadata(json.dumps({
        "schema_version": 1,
        "capture_sequence": 2,
        "frame_width": 848,
        "frame_height": 480,
        "detections": [],
    }).encode(), received_monotonic_s=10.15)

    held = tracker.update(empty, now_s=10.15)

    assert held.held is True
    assert held.detection == visible.detection
    assert held.distance_m == pytest.approx(0.32)


def test_display_target_tracker_drops_target_after_bounded_hold():
    tracker = DisplayTargetTracker()
    tracker.update(_target_frame(1, 0.32, received=10.0), now_s=10.0)
    empty = parse_metadata(json.dumps({
        "schema_version": 1,
        "capture_sequence": 2,
        "frame_width": 848,
        "frame_height": 480,
        "detections": [],
    }).encode(), received_monotonic_s=10.4)

    missing = tracker.update(empty, now_s=10.4)

    assert missing.detection is None
    assert missing.distance_m is None


def test_display_target_tracker_same_sequence_still_becomes_stale():
    tracker = DisplayTargetTracker()
    frame = _target_frame(1, 0.32, received=10.0)
    tracker.update(frame, now_s=10.0)

    stale = tracker.update(frame, now_s=10.6)

    assert stale.detection is None
    assert stale.distance_state == "거리 정보 지연"


def test_telemetry_contract_keeps_missing_sensor_values_unavailable():
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":12,"odometry_source":"wheel+imu",'
        b'"x_m":1.2,"y_m":-0.4,"yaw_rad":0.3,"voltage_v":null,'
        b'"pdist_soc_percent":80,"pdist_battery_flags":0,"pdist_protection_flags":0,'
        b'"pdist_charge_current_a":-0.1,'
        b'"drive_state":"IDLE","can_state":"ERROR-PASSIVE"}', received_monotonic_s=10.0)
    assert frame.x_m == 1.2
    assert frame.voltage_v is None
    assert frame.can_state == "ERROR-PASSIVE"
    assert frame.pdist_soc_percent == 80
    assert frame.pdist_protection_flags == 0
    assert frame.rs485_state == "unavailable"
    assert frame.safety_status == "unavailable"
    assert frame.safety_estop_required is None
    assert frame.wheel_statuses == ()


def test_telemetry_contract_keeps_us100_estop_reason():
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":13,"safety_status":"NO_RESPONSE",'
        b'"safety_distance_mm":null,"safety_estop_required":true,'
        b'"safety_consecutive_failures":3,"safety_detail":"liveness_timeout"}',
        received_monotonic_s=10.0)
    assert frame.safety_status == "NO_RESPONSE"
    assert frame.safety_distance_mm is None
    assert frame.safety_estop_required is True
    assert frame.safety_consecutive_failures == 3
    assert frame.safety_detail == "liveness_timeout"


def test_telemetry_component_mask_is_optional_for_backward_compatibility():
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":14}',
        received_monotonic_s=10.0,
    )

    assert frame.component_mask is None


def test_telemetry_component_mask_round_trips_boolean_values():
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":15,"component_mask":'
        b'{"drive":true,"steer":false,"us100":true,"robot_arm":false}}',
        received_monotonic_s=10.0,
    )

    assert frame.component_mask == {
        "drive": True,
        "steer": False,
        "us100": True,
        "robot_arm": False,
    }


@pytest.mark.parametrize("invalid_value", (0, 1, "true", None))
def test_telemetry_component_mask_rejects_non_boolean_values(invalid_value):
    payload = {
        "schema_version": 1,
        "sequence": 16,
        "component_mask": {"drive": invalid_value},
    }

    with pytest.raises(ValueError, match="component_mask"):
        parse_telemetry(json.dumps(payload).encode("utf-8"))


def test_mask_banner_lists_disabled_components_in_console_order():
    banner_text = getattr(telemetry, "mask_banner_text", None)
    assert banner_text is not None, "component mask banner helper is missing"
    assert banner_text({
        "robot_arm": True,
        "us100": False,
        "drive": False,
        "steer": True,
    }) == "꺼짐: 구동·US-100"
    assert banner_text({
        "drive": True,
        "steer": True,
        "us100": True,
        "robot_arm": True,
    }) is None


def test_us100_mask_off_safety_banner_precedes_live_estop():
    banner_state = getattr(telemetry, "safety_banner_state", None)
    assert banner_state is not None, "safety banner helper is missing"
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":17,"safety_status":"NO_RESPONSE",'
        b'"safety_estop_required":true,"safety_detail":"liveness_timeout"}',
        received_monotonic_s=10.0,
    )

    assert banner_state(
        frame,
        component_mask={"us100": False},
        telemetry_live=True,
    ) == ("안전 해제됨(US-100 꺼짐)", "#d97706")


def test_live_safety_banner_is_korean_for_clear_and_estop():
    clear = parse_telemetry(
        b'{"schema_version":1,"sequence":18,"safety_status":"VALID",'
        b'"safety_estop_required":false}',
        received_monotonic_s=10.0,
    )
    estop = parse_telemetry(
        b'{"schema_version":1,"sequence":19,"safety_status":"NO_RESPONSE",'
        b'"safety_estop_required":true,"safety_detail":"liveness_timeout"}',
        received_monotonic_s=10.0,
    )

    assert telemetry.safety_banner_state(
        clear,
        component_mask=None,
        telemetry_live=True,
    ) == ("안전 정상(CLEAR)", "#16a34a")
    assert telemetry.safety_banner_state(
        estop,
        component_mask=None,
        telemetry_live=True,
    ) == ("비상정지(ESTOP) · liveness_timeout", "#dc2626")


def test_live_safety_banner_treats_missing_estop_field_as_unavailable():
    missing = parse_telemetry(
        b'{"schema_version":1,"sequence":20,"safety_status":"VALID"}',
        received_monotonic_s=10.0,
    )

    assert telemetry.safety_banner_state(
        missing,
        component_mask=None,
        telemetry_live=True,
    ) == ("안전 미수신(UNAVAILABLE)", "#d97706")


@pytest.mark.parametrize(
    ("parser", "payload"),
    (
        (parse_telemetry, b"[]"),
        (parse_metadata, b"[]"),
    ),
)
def test_console_datagram_parsers_normalize_non_object_json_to_value_error(
    parser,
    payload,
):
    with pytest.raises(ValueError):
        parser(payload)


@pytest.mark.parametrize(
    ("parser", "payload"),
    (
        (parse_telemetry, {"schema_version": 1}),
        (
            parse_metadata,
            {
                "schema_version": 1,
                "capture_sequence": 11,
                "frame_width": 848,
                "frame_height": 480,
                "detections": [{
                    "class_name": "bottle",
                    "confidence": 0.9,
                }],
            },
        ),
    ),
)
def test_console_datagram_parsers_normalize_missing_structure_to_value_error(
    parser,
    payload,
):
    with pytest.raises(ValueError):
        parser(json.dumps(payload).encode("utf-8"))


@pytest.mark.parametrize(
    ("parser", "payload"),
    (
        (
            parse_telemetry,
            {
                "schema_version": 1,
                "sequence": 23,
                "wheel_statuses": [{"mode": "IDLE"}],
            },
        ),
        (
            parse_metadata,
            {
                "schema_version": 1,
                "capture_sequence": 12,
                "frame_width": 848,
                "frame_height": 480,
                "detections": [{
                    "class_name": "bottle",
                    "confidence": 0.9,
                    "bbox_xywh": [10, 20, 30, 40],
                    "position_m": 1,
                }],
            },
        ),
        (
            parse_metadata,
            {
                "schema_version": 1,
                "capture_sequence": 13,
                "frame_width": 848,
                "frame_height": 480,
                "detections": [{
                    "class_name": "bottle",
                    "confidence": 0.9,
                    "bbox_xywh": [10, 20, math.inf, 40],
                    "position_m": None,
                }],
            },
        ),
    ),
)
def test_console_datagram_parsers_normalize_nested_structure_to_value_error(
    parser,
    payload,
):
    with pytest.raises(ValueError):
        parser(json.dumps(payload).encode("utf-8"))


@pytest.mark.parametrize(
    ("parser", "payload"),
    (
        (
            parse_telemetry,
            {"schema_version": 1, "sequence": math.inf},
        ),
        (
            parse_metadata,
            {
                "schema_version": 1,
                "capture_sequence": math.inf,
                "frame_width": math.inf,
                "frame_height": 480,
                "detections": [],
            },
        ),
    ),
)
def test_console_datagram_parsers_normalize_overflowing_header_to_value_error(
    parser,
    payload,
):
    with pytest.raises(ValueError):
        parser(json.dumps(payload).encode("utf-8"))


def test_telemetry_rejects_string_safety_estop_boolean():
    with pytest.raises(ValueError, match="safety_estop_required"):
        parse_telemetry(
            b'{"schema_version":1,"sequence":21,'
            b'"safety_estop_required":"false"}'
        )


def test_telemetry_rejects_string_wheel_stale_boolean():
    payload = {
        "schema_version": 1,
        "sequence": 22,
        "wheel_statuses": [{
            "name": "front_left",
            "mode": "IDLE",
            "stale": "false",
        }],
    }

    with pytest.raises(ValueError, match="stale"):
        parse_telemetry(json.dumps(payload).encode("utf-8"))


def test_metadata_rejects_string_pick_target_boolean():
    payload = {
        "schema_version": 1,
        "capture_sequence": 10,
        "frame_width": 848,
        "frame_height": 480,
        "detections": [{
            "class_name": "bottle",
            "confidence": 0.9,
            "bbox_xywh": [10, 20, 30, 40],
            "position_m": None,
            "is_pick_target": "false",
        }],
    }

    with pytest.raises(ValueError, match="is_pick_target"):
        parse_metadata(json.dumps(payload).encode("utf-8"))


def test_power_summary_shows_current_based_charging_with_status_flags():
    summary = getattr(telemetry, "power_summary", None)
    assert summary is not None
    charging = parse_telemetry(
        b'{"schema_version":1,"sequence":20,"voltage_v":47.6,'
        b'"pdist_soc_percent":80,"pdist_battery_flags":2,'
        b'"pdist_protection_flags":32,"pdist_charge_current_a":2.0}',
        received_monotonic_s=10.0,
    )

    assert summary(charging) == "47.6 V · 80% · 정상 · 충전 중"


def test_power_summary_names_over_voltage_fault_before_charging_status():
    summary = getattr(telemetry, "power_summary", None)
    fault = parse_telemetry(
        b'{"schema_version":1,"sequence":21,"voltage_v":47.6,'
        b'"pdist_soc_percent":80,"pdist_battery_flags":4,'
        b'"pdist_protection_flags":32,"pdist_charge_current_a":2.0}',
        received_monotonic_s=10.0,
    )

    assert summary(fault) == "47.6 V · 80% · ⚠ 과전압 보호"


def test_power_summary_preserves_unavailable_wording_for_missing_flags():
    summary = getattr(telemetry, "power_summary", None)
    unknown = parse_telemetry(
        b'{"schema_version":1,"sequence":22,"voltage_v":47.6,'
        b'"pdist_soc_percent":80,"pdist_charge_current_a":2.0}',
        received_monotonic_s=10.0,
    )

    assert summary(None) == "미수신(UNAVAILABLE)"
    assert summary(unknown) == "47.6 V · 80% · 상태 미수신"


@pytest.mark.parametrize(
    ("battery_flags", "protection_flags"),
    ((None, 0x01), (0x04, None)),
)
def test_power_summary_treats_a_partially_missing_flag_pair_as_unavailable(
    battery_flags,
    protection_flags,
):
    summary = getattr(telemetry, "power_summary", None)
    snapshot = parse_telemetry(json.dumps({
        "schema_version": 1,
        "sequence": 23,
        "voltage_v": 47.6,
        "pdist_soc_percent": 80,
        "pdist_battery_flags": battery_flags,
        "pdist_protection_flags": protection_flags,
    }).encode("utf-8"))

    assert summary(snapshot) == "47.6 V · 80% · 상태 미수신"


@pytest.mark.parametrize("charge_current_a", (-0.1, 0.0, 0.1))
def test_power_summary_ignores_charge_current_idle_deadband(charge_current_a):
    summary = getattr(telemetry, "power_summary", None)
    snapshot = parse_telemetry(json.dumps({
        "schema_version": 1,
        "sequence": 24,
        "voltage_v": 47.6,
        "pdist_soc_percent": 80,
        "pdist_battery_flags": 0,
        "pdist_protection_flags": 0,
        "pdist_charge_current_a": charge_current_a,
    }).encode("utf-8"))

    assert summary(snapshot) == "47.6 V · 80% · 정상"


def test_power_summary_does_not_infer_charging_from_d6_bit6():
    summary = getattr(telemetry, "power_summary", None)
    snapshot = parse_telemetry(
        b'{"schema_version":1,"sequence":25,"voltage_v":47.6,'
        b'"pdist_soc_percent":80,"pdist_battery_flags":0,'
        b'"pdist_protection_flags":64,"pdist_charge_current_a":0.0}',
        received_monotonic_s=10.0,
    )

    assert summary(snapshot) == "47.6 V · 80% · 정상"


def test_power_summary_caps_multiple_fault_reasons_at_two():
    summary = getattr(telemetry, "power_summary", None)
    fault = parse_telemetry(
        b'{"schema_version":1,"sequence":26,"voltage_v":47.6,'
        b'"pdist_soc_percent":80,"pdist_battery_flags":252,'
        b'"pdist_protection_flags":15}',
        received_monotonic_s=10.0,
    )

    assert summary(fault) == (
        "47.6 V · 80% · ⚠ 과전압 보호, 저전압 보호 외 7건"
    )


def test_detailed_power_health_uses_fault_masks_and_concrete_reasons():
    from operator_console.app import TelemetryPanel

    assert TelemetryPanel._power_health_text(0x02, 0x20) == "정상"
    assert TelemetryPanel._power_health_text(0, 0x40) == "정상"
    assert TelemetryPanel._power_health_text(0x04, 0) == "⚠ 과전압 보호"
    assert TelemetryPanel._power_health_text(None, 0) == "미수신(UNAVAILABLE)"


def test_chassis_summary_covers_normal_unavailable_and_warning():
    summary = getattr(telemetry, "chassis_summary", None)
    assert summary is not None
    normal = parse_telemetry(
        b'{"schema_version":1,"sequence":22,"drive_state":"IDLE/OK",'
        b'"safety_estop_required":false,"wheel_count":6,'
        b'"wheel_fault_count":0,"wheel_stale_count":0}',
        received_monotonic_s=10.0,
    )
    warning = parse_telemetry(
        b'{"schema_version":1,"sequence":23,"drive_state":"ESTOP/LATCHED",'
        b'"safety_estop_required":true,"wheel_count":6,'
        b'"wheel_fault_count":1,"wheel_stale_count":1}',
        received_monotonic_s=10.0,
    )

    assert summary(normal) == "모드 대기(IDLE) · 안전 정상 · 바퀴 6/6"
    assert summary(None) == "미수신(UNAVAILABLE)"
    assert summary(warning) == (
        "모드 비상정지(ESTOP) · 비상정지(ESTOP) · 바퀴 5/6 ⚠"
    )


def test_telemetry_contract_keeps_rs485_failure_reason():
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":15,"rs485_state":"ERROR",'
        b'"rs485_consecutive_failures":2,"rs485_detail":"timeout"}',
        received_monotonic_s=10.0)
    assert frame.rs485_state == "ERROR"
    assert frame.rs485_consecutive_failures == 2
    assert frame.rs485_detail == "timeout"


def test_telemetry_contract_keeps_bringup_beacon_status():
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":16,'
        b'"unit_status":{"powertrain-bringup-preflight.service":"active"},'
        b'"compose_status":{"powertrain_control":"healthy"},'
        b'"journal_tail":["bring-up ready"]}',
        received_monotonic_s=10.0,
    )

    assert frame.unit_status == (
        ("powertrain-bringup-preflight.service", "active"),
    )
    assert frame.compose_status == (("powertrain_control", "healthy"),)
    assert frame.journal_tail == ("bring-up ready",)


def test_telemetry_contract_exposes_individual_wheel_statuses():
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":14,"wheel_statuses":['
        b'{"name":"front_left","mode":"IDLE","drive_turns_per_s":0.0,'
        b'"steer_deg":1.5,"stale":false,"drive_axis_error":0,"steer_fault":0}]}',
        received_monotonic_s=10.0)
    assert len(frame.wheel_statuses) == 1
    assert frame.wheel_statuses[0].name == "front_left"
    assert frame.wheel_statuses[0].steer_deg == 1.5


def test_telemetry_contract_retains_complete_l515_gateway_status():
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":16,"l515_ros_topic_rates_hz":'
        b'{"/l515/color/image_raw":30.0,"/l515/color/camera_info":30.0,'
        b'"/l515/depth/image_rect_raw":10.0,"/l515/depth/camera_info":10.0,'
        b'"/l515/gyro/sample":100.0,"/l515/accel/sample":100.0},'
        b'"l515_aligned_depth_age_ms":12.5,"l515_process_cpu_percent":77.2,'
        b'"l515_process_rss_bytes":12345678}', received_monotonic_s=10.0)
    assert len(frame.l515_ros_topic_rates_hz) == 6
    assert frame.l515_aligned_depth_age_ms == 12.5
    assert frame.l515_process_cpu_percent == 77.2
    assert frame.l515_process_rss_bytes == 12345678


def test_chassis_rows_become_stale_when_snapshot_age_exceeds_one_second():
    state_fn = getattr(telemetry, "chassis_component_states", None)
    assert state_fn is not None, "chassis component freshness helper is missing"
    snapshot = parse_telemetry(
        b'{"schema_version":1,"sequence":17,"odometry_source":"wheel+imu",'
        b'"drive_state":"ARMED/RUNNING","can_state":"HEALTHY"}',
        received_monotonic_s=10.0,
    )

    assert state_fn(snapshot, now_s=10.5) == ("LIVE", "LIVE", "LIVE")
    assert state_fn(snapshot, now_s=11.01) == ("STALE", "STALE", "STALE")


def test_chassis_components_treat_unavailable_prefix_case_insensitively():
    snapshot = parse_telemetry(
        b'{"schema_version":1,"sequence":18,'
        b'"odometry_source":"  Unavailable \xc2\xb7 no wheel odometry",'
        b'"drive_state":"UNAVAILABLE \xc2\xb7 chassis owner absent",'
        b'"can_state":"UNAVAILABLE \xc2\xb7 no CAN_HEALTH from chassis owner"}',
        received_monotonic_s=10.0,
    )

    assert telemetry.chassis_component_states(
        snapshot,
        now_s=10.5,
    ) == ("UNAVAILABLE", "UNAVAILABLE", "UNAVAILABLE")


def _payload_encoder():
    package_root = str(
        Path(__file__).resolve().parents[2] / "ros2/src/powertrain_ros"
    )
    sys.path.insert(0, package_root)
    try:
        module = importlib.import_module("powertrain_ros.chassis_telemetry")
    finally:
        sys.path.remove(package_root)
    return getattr(module, "encode_telemetry_payload", None)


def _six_wheels(*, name_size=12):
    return [
        {
            "name": (f"wheel-{index}-" + "w" * name_size),
            "mode": "IDLE",
            "drive_turns_per_s": 0.0,
            "steer_deg": 1.5,
            "stale": False,
            "drive_axis_error": 0,
            "steer_fault": 0,
        }
        for index in range(6)
    ]


def test_long_details_and_six_wheels_encode_within_console_receive_contract():
    encode = _payload_encoder()
    assert encode is not None, "bounded chassis telemetry encoder is missing"
    raw = encode({
        "schema_version": 1,
        "sequence": 18,
        "l515_detail": "g" * 5000,
        "safety_detail": "s" * 5000,
        "wheel_statuses": _six_wheels(),
    })

    assert len(raw) <= 4096
    decoded = json.loads(raw)
    assert len(decoded["l515_detail"]) == 256
    assert len(decoded["safety_detail"]) == 256
    snapshot = parse_telemetry(raw, received_monotonic_s=10.0)
    assert len(snapshot.wheel_statuses) == 6


def test_over_4096_payload_omits_wheels_and_marks_truncated_instead_of_dropping():
    encode = _payload_encoder()
    assert encode is not None, "bounded chassis telemetry encoder is missing"
    raw = encode({
        "schema_version": 1,
        "sequence": 19,
        "l515_detail": "g" * 5000,
        "safety_detail": "s" * 5000,
        "wheel_statuses": _six_wheels(name_size=1000),
    })

    assert len(raw) <= 4096
    decoded = json.loads(raw)
    assert decoded["truncated"] is True
    assert "wheel_statuses" not in decoded
    snapshot = parse_telemetry(raw, received_monotonic_s=10.0)
    assert snapshot.truncated is True
    assert snapshot.wheel_statuses == ()


def test_bounded_encoder_never_exceeds_4096_with_multibyte_free_text():
    encode = _payload_encoder()
    assert encode is not None, "bounded chassis telemetry encoder is missing"
    payload = {
        "schema_version": 1,
        "sequence": 20,
        "wheel_statuses": _six_wheels(name_size=1000),
    }
    for key in (
        "odometry_source", "drive_state", "can_state", "l515_state",
        "l515_detail", "l515_mode", "safety_status", "safety_detail",
    ):
        payload[key] = "오류🚫" * 2000

    raw = encode(payload)

    assert len(raw) <= 4096
    assert json.loads(raw)["truncated"] is True


def test_panel_formatters_are_shared_module_functions():
    # 2026-07-18 실사고: ChassisTelemetryPanel._refresh가 TelemetryPanel에만
    # 있는 staticmethod(_rss 등)를 호출해 첫 LIVE 스냅샷에서 AttributeError.
    # 포매터는 gi-무관 telemetry 모듈 함수로 공유하고, 패널 소스에 클래스
    # 헬퍼 호출이 남지 않음을 봉인한다.
    from pathlib import Path

    from operator_console.telemetry import (
        _format_hex,
        _format_number,
        _format_ros_rates,
        _format_rss,
    )

    assert _format_number(None, "Hz") == "N/A"
    assert _format_number(1.234, "Hz") == "1.23 Hz"
    assert _format_rss(None) == "N/A"
    assert _format_rss(3 * 1024 * 1024) == "3.0 MiB"
    assert _format_ros_rates(()) == "N/A"
    assert _format_ros_rates((("/l515/depth", 29.97),)) == "depth 30.0 Hz"
    assert _format_hex(None) == "N/A"
    assert _format_hex(0x1F) == "0x1F"

    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")
    for stale_call in (
        "self._number(", "self._rss(", "self._l515_ros_rates_text(",
        "self._hex(",
    ):
        assert stale_call not in source


def test_app_source_uses_korean_titles_and_collapsed_sections_without_gtk():
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    for title in (
        "로봇 상태",
        "차대",
        "로봇팔",
        "조작 (토큰 인증)",
        "이벤트 기록",
    ):
        assert title in source
    assert 'Gtk.Expander(label="고급")' in source
    assert source.count('Gtk.Expander(label="상세")') >= 3
    assert "GESTURE_IMMEDIATE" in source
    assert "모드:" in source and "최근:" in source
    assert "대기에서만" in source
    assert 'mode_allows_action(action.action, "UNKNOWN")' in source
    assert source.count('getattr(self, "_summary", None)') >= 3
    for old_visible_copy in (
        'Gtk.Label(label="Display FPS: waiting")',
        'Gtk.Label(label=f"SRT caller →',
        'self._status.set_text(f"{self._name}: waiting for first frame")',
        'f"wheels {snapshot.wheel_count}',
        'f"SRT submit/sent/drop',
    ):
        assert old_visible_copy not in source


def test_ops_panel_wires_pure_estop_cause_status_and_event_without_gtk():
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")
    compact_source = "".join(source.split())

    assert "format_ops_status_line(" in source
    assert "next_estop_cause_event(" in source
    assert 'getattr(self, "_last_estop_cause' in source
    assert 'state.get("active_estop_sources", ())' in source
    assert 'getattr(self,"_latest_active_estop_sources",())' in compact_source
    assert 'self._event_sink("안전",' in source


def test_health_banner_includes_arm_freshness_and_reuses_probe_state():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    arm_state_call = (
        "self._telemetry_state(self._arm_receiver.latest())"
    )
    assert source.count(arm_state_call) == 1
    assert "arm_state = " + arm_state_call in source
    assert '"arm": arm_state' in source
    assert '"arm": arm_state' in source


def test_immediate_estop_click_submits_directly_without_confirmation():
    import ast
    from pathlib import Path
    from types import SimpleNamespace

    source_path = Path(__file__).resolve().parents[1] / "app.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    panel = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "OpsPanel"
    )
    method = next(
        item
        for item in panel.body
        if isinstance(item, ast.FunctionDef)
        and item.name == "_on_immediate_clicked"
    )
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "Gtk": SimpleNamespace(Button=object),
        "PanelAction": object,
    }
    exec(compile(module, str(source_path), "exec"), namespace)

    submitted = []
    flow_calls = []
    flow = SimpleNamespace(
        reset=lambda: flow_calls.append("reset"),
        begin=lambda _action: flow_calls.append("begin"),
    )
    node = SimpleNamespace(
        _flow=flow,
        _emit=lambda _message: None,
        _submit=submitted.append,
    )
    action = SimpleNamespace(action="estop")

    namespace["_on_immediate_clicked"](
        node,
        None,
        action,
    )

    # 비상 조작은 state 가용성·revision 일치·확인 클릭을 전제하면 안 된다.
    # 최종 권위와 거부 판단은 브로커에 있다.
    assert flow_calls == ["reset"]
    assert submitted == [{"action": "estop", "params": {}}]


def test_ops_panel_clears_cached_component_mask_when_state_becomes_unavailable():
    import ast
    from pathlib import Path
    from types import SimpleNamespace

    source_path = Path(__file__).resolve().parents[1] / "app.py"
    tree = ast.parse(source_path.read_text(encoding="utf-8"))
    panel = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "OpsPanel"
    )
    method = next(
        item
        for item in panel.body
        if isinstance(item, ast.FunctionDef) and item.name == "_on_state"
    )
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "component_mask_from_state": lambda state: state.get("component_mask"),
        "next_estop_cause_event": lambda *_args, **_kwargs: (None, None),
        "PANEL_ACTIONS": (),
    }
    exec(compile(module, str(source_path), "exec"), namespace)

    calls = []
    node = SimpleNamespace(
        _latest_component_mask={"us100": True},
        _latest_chassis_mode="ARMED",
        _latest_estop_source="us100",
        _latest_estop_detail="near",
        _latest_active_estop_sources=("us100",),
        _last_estop_cause_key=None,
        _flow=SimpleNamespace(reset=lambda: calls.append("reset")),
        _hide_confirmation=lambda: calls.append("hide"),
        _event_sink=lambda *_args: None,
        _action_buttons={},
        _refresh_status_line=lambda: None,
    )

    namespace["_on_state"](node, None)

    assert node._latest_component_mask is None
    assert node._latest_chassis_mode == "UNKNOWN"
    assert calls == ["reset", "hide"]


def test_ops_panel_passes_the_panel_action_object_to_confirm_flow():
    # 같은 action 이름의 행이 2개(arm_lock_override 걸기/취소)라서 문자열을
    # 넘기면 _ACTION_BY_NAME 이 한 행으로 뭉개진다 — 반드시 행 객체를 넘긴다.
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")
    assert "self._flow.begin(action)" in source
    assert "self._flow.begin(action.action)" not in source
    assert "self._flow.confirm(action, held_s=held_s)" in source
    assert "self._flow.confirm(action.action" not in source


def test_video_panel_has_stale_watchdog_and_restart_resets_freshness():
    # srtsrc(auto-reconnect=true)는 리스너 사망을 bus ERROR 없이 삼킨다 —
    # 웻지된 스트림은 stale 워치독만이 복구할 수 있다(2026-07-19 리뷰).
    import ast
    from pathlib import Path

    source_path = Path(__file__).resolve().parents[1] / "app.py"
    source = source_path.read_text(encoding="utf-8")
    assert "VIDEO_STALE_RESTART_S = 5.0" in source

    tree = ast.parse(source)
    panel = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "VideoPanel"
    )

    def method_source(name):
        method = next(
            item
            for item in panel.body
            if isinstance(item, ast.FunctionDef) and item.name == name
        )
        return ast.get_source_segment(source, method)

    health = method_source("_refresh_video_health")
    assert "VIDEO_STALE_RESTART_S" in health
    assert "_schedule_reconnect()" in health

    restart = method_source("_restart_pipeline")
    for reset_line in (
        "self._pipeline_live = False",
        "self._last_frame_monotonic = None",
        "self._frames = 0",
        'self._freshness_state = "connecting"',
    ):
        assert reset_line in restart, reset_line


def test_smoke_probe_reports_video_pane_health():
    from pathlib import Path

    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")
    assert '"video_l515": self._l515.health_state()' in source
    assert '"video_d435": self._d435.health_state()' in source


def test_console_layout_separates_mission_systems_and_ops_pages():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert 'stack.add_titled(mission_page, "mission", "실시간 화면")' in source
    assert 'stack.add_titled(systems_scroll, "systems", "로봇 상태")' in source
    assert 'stack.add_titled(ops_page, "ops", "관리자 조작")' in source
    assert "Gtk.StackSwitcher()" in source
    assert "self.set_default_size(1180, 760)" in source
    assert "videos = Gtk.Overlay()" in source
    assert "int(allocation.width * 0.30)" in source
    assert "pip_width * 480 / 848" in source
    assert "event_expander = Gtk.Expander()" in source
    assert 'event_expander.add(self._events)' in source
    assert "Gtk.Revealer()" not in source
    assert '"event-expander"' in source


def test_console_visual_tokens_keep_arctic_shell_dark_video_and_safety_contrast():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert "background: rgba(10,25,40,0.88)" in source
    assert "background: {video_stage}" in source
    assert ".health-strip" in source
    assert ".video-card" in source
    assert ".danger-card" in source
    assert ".status-live" in source
    assert ".status-warn" in source
    assert ".status-bad" in source


def test_mission_preparation_keeps_dual_camera_layout_without_progress_hud():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    for copy in (
        "운용 정보",
            "현재 단계",
        "운용 준비",
        "전방 카메라",
        "작업 카메라",
        "주행 시스템",
        "안전 장치",
    ):
        assert copy in source
    assert "SYSTEM CHECK" not in source
    assert "임무 시작 전" not in source
    assert "mission_page.pack_start(mission_body" in source
    assert "mission_presentation_stack" not in source
    assert "self._readiness_count" in source
    assert 'step_names = ("준비", "탐색", "접근", "도구 작업", "완료")' not in source
    assert "videos.add_overlay(progress_hud)" not in source
    assert "rail.pack_start(display_options" in source
    assert "로봇의 주행 영상을 연결하고 있습니다" in source
    assert "로봇팔 작업 영상을 연결하고 있습니다" in source


def test_console_health_is_split_into_named_status_chips():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert "self._health = Gtk.Box(spacing=14)" in source
    for key in ("network", "power", "camera", "safety"):
        assert f'"{key}"' in source
    for title in ("로봇 연결", "전원", "카메라", "안전 장치"):
        assert f'"{title}"' in source


def test_judge_facing_mission_summary_uses_plain_language_and_live_data():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    for copy in (
        "재난 대응 로봇 관제 시스템", "현재 단계",
        "임무 정보", "안전 장치",
    ):
        assert copy in source
    assert 'self._mission_metrics["target"].set_text' in source
    assert 'self._mission_metrics["distance"].set_text' in source
    assert "self._readiness_count.set_text" in source


def test_judge_view_stays_inside_the_three_original_gui_sections():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert '"기술 소개"' not in source
    assert ".distance-card" in source
    assert ".target-card" in source
    assert ".story-phase" in source


def test_global_estop_is_always_in_topbar_and_reuses_token_gated_path():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert 'estop_title = Gtk.Label(label="긴급 정지")' in source
    assert 'estop_subtitle = Gtk.Label(label="E-STOP")' in source
    assert '"clicked", lambda _button: self._ops_panel.trigger_estop()' in source
    assert 'item for item in PANEL_ACTIONS if item.action == "estop"' in source
    assert 'self._on_immediate_clicked(' in source
    assert '"global_estop_visible": self._global_estop.get_visible()' in source


def test_explicit_controls_swap_main_and_sub_without_rebuilding_pipelines():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert 'header_click.connect("button-press-event", self._on_swap_click)' not in source
    assert "def _on_swap_click" not in source
    assert "def set_swap_handler" not in source
    assert 'Gtk.Button(label="큰 화면으로 보기")' in source
    assert "pip_overlay.add_overlay(self._swap_button)" in source
    assert "display_options.pack_start(self._swap_button" not in source
    assert "Gdk.KEY_v" in source
    assert "Gdk.KEY_V" in source
    assert 'self._l515.set_role("MAIN")' in source
    assert 'self._d435.set_role("SUB")' in source
    assert "self._videos.remove(secondary)" in source
    assert "self._pip_slot.remove(selected)" in source
    assert "self._videos.add(selected)" in source
    assert "self._pip_slot.add(secondary)" in source
    assert "클릭하여 크게 보기" not in source
    assert 'self._role.set_text(normalized)' not in source
    assert "widget.set_no_show_all(compact)" in source


def test_camera_status_is_overlaid_and_has_a_waiting_placeholder():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert '_style(header_click, "video-overlay")' in source
    assert "전방 화면 준비 중" in source
    assert "작업 화면 준비 중" in source
    assert "로봇의 주행 영상을 연결하고 있습니다" in source
    assert "RoverPlaceholder(" in source
    assert 'camera_kind=("front" if name == "전방 카메라" else "work")' in source
    assert '"arm": (72.0, 4.0, 190.0, 172.0)' in source
    assert '"front": (218.0, 27.0, 154.0, 170.0)' in source
    assert "video_stage.add_overlay(header_click)" in source


def test_event_filters_are_independent_uppercase_checkbuttons():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert 'for severity in ("ERROR", "WARNING", "INFO")' in source


def test_event_rows_render_uppercase_severity_and_tabs_auto_collapse():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")
    row_source = source[
        source.index("class EventOperationRow"):
        source.index("class EventLog")
    ]
    assert "Gtk.Label(label=severity)" in row_source
    assert '"ERROR": "오류"' not in row_source
    assert 'stack.connect("notify::visible-child-name", self._on_page_changed)' in source
    assert 'self._event_expander.set_expanded(False)' in source


def test_overlay_transform_preserves_aspect_and_letterbox_offsets():
    from operator_console.app import fit_overlay_transform

    scale, offset_x, offset_y = fit_overlay_transform(1920, 1080, 640, 480)
    assert scale == 2.25
    assert offset_x == 240
    assert offset_y == 0


def test_overlay_is_suppressed_when_metadata_size_differs_from_video():
    from operator_console.app import overlay_size_matches

    assert overlay_size_matches(848, 480, 848, 480) is True
    assert overlay_size_matches(848, 480, None, None) is True
    assert overlay_size_matches(848, 480, 1280, 720) is False


def test_set_video_size_queues_draw_only_when_value_changes():
    from operator_console.app import MetadataCanvas

    class CanvasProbe:
        _video_width = None
        _video_height = None

        def __init__(self):
            self.draw_requests = 0

        def queue_draw(self):
            self.draw_requests += 1

    canvas = CanvasProbe()
    MetadataCanvas.set_video_size(canvas, 848, 480)
    MetadataCanvas.set_video_size(canvas, 848, 480)
    MetadataCanvas.set_video_size(canvas, 1280, 720)

    assert canvas.draw_requests == 2


def test_metadata_overlay_remains_owned_by_work_camera_during_swap():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")
    assert 'self._d435 = VideoPanel("작업 카메라"' in source
    assert "metadata_receiver=self._metadata_receiver" in source
    assert 'self._l515 = VideoPanel("전방 카메라"' in source
    swap = source[
        source.index("def swap_camera_views"):
        source.index("def _on_display_option_toggled")
    ]
    assert "MetadataCanvas(" not in swap
    assert "set_metadata_display_options" not in swap
    assert "if not user_initiated:" in swap
    assert "smoke-swap-video" not in source
    assert "overlay_view_state" in source
    handler = source[
        source.index("def _on_display_option_toggled"):
        source.index("def _on_page_changed")
    ]
    assert "self._sync_overlay_rail" in handler
    assert "sendto(" not in handler
    assert "Gtk.CheckButton(label=severity)" in source
    assert "for toggle in self._filters.values():" in source
    assert "toggle.set_active(True)" in source
    assert "self._events.set_developer_visible(switch.get_active())" in source
    assert 'label="접기"' not in source


def test_standby_rover_and_watermark_use_viewport_owned_cairo_overlays():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assets = Path(__file__).resolve().parents[1] / "assets"
    rover = (assets / "mobile_robot_pictogram.svg").read_text(encoding="utf-8")
    wordmark = (assets / "jetin_wordmark.svg").read_text(encoding="utf-8")

    assert 'Rsvg.Handle.new_from_file' in source
    assert 'preserveAspectRatio="xMidYMid meet"' in rover
    assert "linearGradient" not in rover
    assert 'fill="none"' in rover
    assert 'stroke="rgba(74,116,154,.58)"' in rover
    assert rover.count("<circle") >= 10
    assert 'fill="#355B78"' in rover
    assert 'fill="#263F55"' in rover
    assert "rocker-bogie" in rover
    assert "<linearGradient" not in rover
    assert "<filter" not in rover
    assert "JET-IN" in wordmark
    assert 'font-weight="900"' in wordmark
    assert 'skewX(-10)' in wordmark
    assert "videos.add_overlay(self._watermark)" in source
    assert "videos.set_overlay_pass_through(self._watermark, True)" in source


def test_rover_placeholder_size_depends_on_slot_not_camera_name():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert "class RoverPlaceholder" in source
    assert "MAIN_WIDTH = 290" in source
    assert "PREVIEW_WIDTH = 90" in source
    assert 'width = self.MAIN_WIDTH if role == "MAIN" else self.PREVIEW_WIDTH' in source
    assert "compact_standby = name" not in source
    assert "self._rover_placeholder.set_slot(normalized)" in source


def test_event_log_uses_one_integrated_feed_with_inline_raw_rows():
    source = (
        Path(__file__).resolve().parents[1] / "app.py"
    ).read_text(encoding="utf-8")

    assert 'label="운용 기록"' not in source
    assert 'label="원본 로그"' not in source
    assert "self._log_stack = Gtk.Stack()" not in source
    assert 'Gtk.CheckButton(label="원본 기술 로그 보기")' not in source
    assert 'Gtk.ToggleButton(label="비교 보기")' not in source
    assert "event-compare-pane" not in source
    assert "class EventOperationRow(Gtk.ListBoxRow)" in source
    assert "row.toggle_detail()" in source
    assert 'label=f"원본: [{stamp}] {severity} {source}: {message}"' in source
    assert '(3, "기술 정보", 360)' in source
    assert 'technical_box.set_size_request(360, -1)' in source
    assert "self._technical_summary(message)" in source
    assert "self.set_size_request(-1, 180)" not in source
    assert "self._user_scroll.set_max_content_height(210)" in source
    assert "self._user_scroll.set_propagate_natural_height(True)" in source
    assert "event_heading.pack_end(self._events.filter_box" in source
