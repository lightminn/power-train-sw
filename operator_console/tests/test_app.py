from operator_console.pipelines import pipeline_description, srt_uri
from operator_console.metadata import parse_metadata
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


def test_telemetry_contract_keeps_rs485_failure_reason():
    frame = parse_telemetry(
        b'{"schema_version":1,"sequence":15,"rs485_state":"ERROR",'
        b'"rs485_consecutive_failures":2,"rs485_detail":"timeout"}',
        received_monotonic_s=10.0)
    assert frame.rs485_state == "ERROR"
    assert frame.rs485_consecutive_failures == 2
    assert frame.rs485_detail == "timeout"


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
