import importlib
import json
import math

import pytest

from operator_console.pipelines import pipeline_description, srt_uri
from operator_console.metadata import parse_metadata
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
    assert frame.detections[0].yaw_rad is None
    assert frame.detections[0].is_pick_target is False


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


def test_metadata_contract_rejects_non_finite_yaw():
    payload = {
        "schema_version": 1,
        "capture_sequence": 9,
        "frame_width": 848,
        "frame_height": 480,
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


def test_power_summary_covers_normal_unavailable_and_warning():
    summary = getattr(telemetry, "power_summary", None)
    assert summary is not None
    normal = parse_telemetry(
        b'{"schema_version":1,"sequence":20,"voltage_v":47.6,'
        b'"pdist_soc_percent":80,"pdist_battery_flags":0,'
        b'"pdist_protection_flags":0}',
        received_monotonic_s=10.0,
    )
    warning = parse_telemetry(
        b'{"schema_version":1,"sequence":21,"voltage_v":47.6,'
        b'"pdist_soc_percent":80,"pdist_battery_flags":0,'
        b'"pdist_protection_flags":2}',
        received_monotonic_s=10.0,
    )

    assert summary(normal) == "47.6 V · 80% · 정상"
    assert summary(None) == "미수신(UNAVAILABLE)"
    assert summary(warning) == "47.6 V · 80% · ⚠ 보호 경고"


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


def _payload_encoder():
    try:
        module = importlib.import_module("powertrain_ros.chassis_telemetry")
    except ModuleNotFoundError:
        return None
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
