"""Packet v1 contract — runs on both conda and the console's system python."""
import json

import pytest

from terrain_overlay import contract
from terrain_overlay.contract import encode_packet, parse_packet


def ok_payload(**overrides):
    payload = {
        "schema_version": 1,
        "type": "l515_terrain",
        "sender_boot_id": "0082f17fdc0a49969ff0fe1dc4979560",
        "sender_start_mono_ns": 707316067583855,
        "sender_instance": "3fa95c0e81d2b7a4",
        "sequence": 123,
        "sent_unix_ns": 1760000000000000000,
        "depth_stamp_ns": 1760000000000000000,
        "depth_age_ms": 35.2,
        "status": "OK",
        "reason": None,
        "frame_size": [1280, 720],
        "distance_origin": "l515_depth_optical",
        "alignment_verified": False,
        "ground": {
            "state": "FRESH", "age_ms": 35.2,
            "camera_height_m": 0.512, "camera_tilt_deg": 21.4,
        },
        "path_half_width_m": 0.4445,
        "zones": {"left": None, "center": 1.23, "right": 2.10},
        "obstacles": [{
            "dist_m": 1.23, "in_path": True,
            "poly": [[612, 410], [700, 410], [700, 520], [612, 520]],
        }],
        "proc_ms": 18.4,
    }
    payload.update(overrides)
    return payload


def error_payload(**overrides):
    fields = {
        "status": "SENSOR_ERROR", "reason": "DEPTH_STALE", "ground": None,
        "zones": {"left": None, "center": None, "right": None},
        "obstacles": [], "proc_ms": None,
    }
    fields.update(overrides)
    return ok_payload(**fields)


def raw(payload):
    return json.dumps(payload).encode("utf-8")


def test_spec_example_round_trips_and_fits_the_size_limit():
    encoded = encode_packet(ok_payload())
    assert len(encoded) <= contract.MAX_PACKET_BYTES
    frame = parse_packet(encoded, received_monotonic_s=12.5)
    assert frame.status == "OK"
    assert frame.ground.state == "FRESH"
    assert frame.ground.age_ms == pytest.approx(35.2)
    assert frame.zones == {"left": None, "center": 1.23, "right": 2.10}
    assert frame.obstacles[0].poly[0] == (612, 410)
    assert frame.obstacles[0].in_path is True
    assert frame.session_key == (
        "0082f17fdc0a49969ff0fe1dc4979560", 707316067583855, "3fa95c0e81d2b7a4",
    )
    assert frame.received_monotonic_s == 12.5


def test_error_packet_round_trips():
    frame = parse_packet(encode_packet(error_payload()))
    assert (frame.status, frame.reason, frame.ground, frame.obstacles) == (
        "SENSOR_ERROR", "DEPTH_STALE", None, (),
    )


def test_unknown_top_level_fields_are_ignored():
    assert parse_packet(raw(ok_payload(future_field={"x": 1}))).status == "OK"


def test_encode_refuses_nan_and_oversize():
    with pytest.raises(ValueError):
        encode_packet(ok_payload(depth_age_ms=float("nan")))
    with pytest.raises(ValueError, match="exceeds"):
        encode_packet(ok_payload(padding="x" * 1300))


@pytest.mark.parametrize("overrides", [
    {"schema_version": 2},
    {"schema_version": True},
    {"schema_version": 1.0},
    {"status": []},
    {"status": {"OK": 1}},
    {"reason": ["x"]},
    {"distance_origin": []},
    {"depth_age_ms": 10**400},
    {"path_half_width_m": 10**400},
    {"zones": {"left": None, "center": 10**400, "right": None}},
    {"type": "yolo"},
    {"sender_boot_id": "0082F17FDC0A49969FF0FE1DC4979560"},
    {"sender_boot_id": "0082f17f-dc0a-4996-9ff0-fe1dc4979560"},
    {"sender_instance": "3fa95c0e81d2b7a"},
    {"sender_start_mono_ns": -1},
    {"sender_start_mono_ns": True},
    {"sequence": 1.0},
    {"sequence": False},
    {"sent_unix_ns": None},
    {"depth_stamp_ns": -5},
    {"depth_age_ms": -0.1},
    {"status": "WARN"},
    {"reason": "DEPTH_STALE"},
    {"frame_size": [848, 480]},
    {"frame_size": [1280.0, 720]},
    {"distance_origin": "base_link"},
    {"alignment_verified": 0},
    {"path_half_width_m": 0},
    {"path_half_width_m": True},
    {"zones": {"left": None, "center": 1.0}},
    {"zones": {"left": None, "center": -1.0, "right": None}},
    {"proc_ms": -1},
    {"ground": None},
    {"ground": {"state": "STALE", "age_ms": 1.0,
                "camera_height_m": 0.5, "camera_tilt_deg": 20.0}},
    {"ground": {"state": "HELD", "age_ms": 1.0,
                "camera_height_m": 0.25, "camera_tilt_deg": 20.0}},
    {"ground": {"state": "HELD", "age_ms": 1.0,
                "camera_height_m": 0.80, "camera_tilt_deg": 20.0}},
    {"ground": {"state": "HELD", "age_ms": 1.0,
                "camera_height_m": 0.5, "camera_tilt_deg": 40.1}},
    {"ground": {"state": "HELD", "age_ms": -1.0,
                "camera_height_m": 0.5, "camera_tilt_deg": 20.0}},
    {"obstacles": [{"dist_m": 1.0, "in_path": 1, "poly": [[0, 0], [1, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1, 0]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0]] * 17}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1280, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1, 720], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1.5, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [True, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": float("inf"), "in_path": True,
                    "poly": [[0, 0], [1, 0], [1, 1]]}]},
    {"obstacles": [{"dist_m": 1.0, "in_path": True,
                    "poly": [[0, 0], [1, 0], [1, 1]]}] * 5},
])
def test_each_ok_rule_rejects_the_whole_packet(overrides):
    with pytest.raises(ValueError):
        parse_packet(raw(ok_payload(**overrides)))


@pytest.mark.parametrize("overrides", [
    {"reason": None},
    {"reason": "GROUND_EXPIRED"},
    {"ground": {"state": "FRESH", "age_ms": 1.0,
                "camera_height_m": 0.5, "camera_tilt_deg": 20.0}},
    {"zones": {"left": None, "center": 1.0, "right": None}},
    {"obstacles": [{"dist_m": 1.0, "in_path": True, "poly": [[0, 0], [1, 0], [1, 1]]}]},
])
def test_non_ok_status_requires_empty_results(overrides):
    with pytest.raises(ValueError):
        parse_packet(raw(error_payload(**overrides)))


def test_no_ground_reasons():
    for reason in ("GROUND_NOT_FOUND", "GROUND_EXPIRED"):
        payload = error_payload(reason=reason)
        payload["status"] = "NO_GROUND"
        assert parse_packet(raw(payload)).reason == reason
    payload = error_payload(reason="DEPTH_STALE")
    payload["status"] = "NO_GROUND"
    with pytest.raises(ValueError):
        parse_packet(raw(payload))


def test_garbage_and_oversize_raw_are_rejected():
    for blob in (b"\xff\xfe", b"[]", b"not json", b"{" + b" " * 1300 + b"}"):
        with pytest.raises(ValueError):
            parse_packet(blob)


def test_port_matches_the_remote_video_registry():
    from remote_video.contract import L515_TERRAIN_UDP_PORT
    assert contract.UDP_PORT == L515_TERRAIN_UDP_PORT == 5009
