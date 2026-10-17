"""terrain_map packet contract v1 (spec §5) — runs on conda and the system python."""
import base64
import json

import pytest

from terrain_map import contract as C
from terrain_map.config import MapConfig

BOOT = "0082f17fdc0a49969ff0fe1dc4979560"
INSTANCE = "3fa95c0e81d2b7a4"
MAX_I31 = 2**31 - 1


def session(**overrides):
    base = {
        "sender_boot_id": BOOT, "sender_start_mono_ns": 123_456_789,
        "sender_instance": INSTANCE, "map_epoch": 0, "sent_unix_ns": 1_760_000_000_000_000_000,
    }
    base.update(overrides)
    return base


def status(**overrides):
    packet = C.StatusPacket(
        **session(),
        sequence=7, status="OK", reason=None,
        cell_m=0.05, tile_cells=16, age_step_s=2.0,
        pose=C.Pose(1.2345, -0.5, 0.25), distance_m=12.5,
        pose_gaps=0, not_support=0, late_dropped=0,
        tile_count=12, refresh_cycle_s=25.6,
        thresholds=C.Thresholds(0.08, 0.08, 2),
        corrector=C.CorrectorInfo("identity", False, None, "identity"),
        proc_ms=42.0,
    )
    return C.replace(packet, **overrides)


def tile(**overrides):
    n = 16 * 16
    heights = [None if i % 7 == 0 else (i % 50) - 25 for i in range(n)]
    codes = [C.INSUFFICIENT if h is None else (C.OBSTACLE if (h or 0) > 8 else C.FREE) for h in heights]
    ages = [i % 32 for i in range(n)]
    packet = C.TilePacket(
        **session(), tx=-3, ty=5, version=11, tile_seq=40,
        cells=C.pack_cells(heights, codes, ages),
    )
    return C.replace(packet, **overrides)


def raw_dict(packet):
    return json.loads(C.encode(packet))


def mutate(packet, **changes):
    payload = raw_dict(packet)
    payload.update(changes)
    return json.dumps(payload, separators=(",", ":")).encode()


# --- round trips ------------------------------------------------------------
def test_status_round_trip():
    parsed = C.parse_packet(C.encode(status()), received_s=3.5)
    assert isinstance(parsed, C.StatusPacket)
    assert parsed.pose == C.Pose(1.2345, -0.5, 0.25)
    assert parsed.thresholds.min_observations == 2
    assert parsed.received_s == 3.5
    assert parsed.session_key == (BOOT, 123_456_789, INSTANCE)


def test_tile_round_trip_and_cells():
    packet = tile()
    parsed = C.parse_packet(C.encode(packet))
    assert isinstance(parsed, C.TilePacket)
    heights, codes, ages = C.unpack_cells(parsed.cells, 16)
    assert heights[0] is None and codes[0] == C.INSUFFICIENT
    assert heights[1] == -24 and ages[31] == 31


def test_floats_are_rounded_to_four_decimals():
    payload = raw_dict(status(pose=C.Pose(1.234567, 0.0, 0.123456789), proc_ms=12.345678))
    assert payload["pose"] == {"x_m": 1.2346, "y_m": 0.0, "yaw_rad": 0.1235}
    assert payload["proc_ms"] == 12.3457


def test_pack_cells_quantises_and_clips():
    cells = C.pack_cells([0, 127, -127, 200, -200, None], [0, 1, 2, 1, 2, 4], [0, 5, 31, 40, 0, 3])
    raw = base64.b64decode(cells)
    assert list(raw[0::2]) == [0, 127, 129, 127, 129, 128]  # int8 two's complement, -128 = unobserved
    assert [b >> 5 for b in raw[1::2]] == [0, 1, 2, 1, 2, 4]
    assert [b & 31 for b in raw[1::2]] == [0, 5, 31, 31, 0, 3]


# --- sizes ------------------------------------------------------------------
def test_worst_case_tile_fits():
    worst = tile(
        sender_start_mono_ns=2**63 - 1, sent_unix_ns=2**63 - 1, map_epoch=MAX_I31,
        tx=-(2**20), ty=-(2**20), version=MAX_I31, tile_seq=MAX_I31,
    )
    assert len(C.encode(worst)) <= C.MAX_PACKET_BYTES


def test_worst_case_status_fits():
    worst = status(
        sender_start_mono_ns=2**63 - 1, sent_unix_ns=2**63 - 1, map_epoch=MAX_I31,
        sequence=MAX_I31, status="SENSOR_ERROR", reason="DEPTH_SHAPE_MISMATCH",
        pose=C.Pose(-999999.9999, -999999.9999, -3.1416), distance_m=9999999.9999,
        pose_gaps=MAX_I31, not_support=MAX_I31, late_dropped=MAX_I31, tile_count=MAX_I31,
        refresh_cycle_s=99999.9999, cell_m=0.2, tile_cells=16, age_step_s=10.0,
        thresholds=C.Thresholds(0.5, 0.5, 10),
        corrector=C.CorrectorInfo("height_match", True, 999.9999, "no_overlap"),
        proc_ms=999999.9999,
        plane_tilt=C.PlaneTilt(-89.9999, -89.9999, 999999.9999),
        gravity=C.GravityTilt(-89.9999, -89.9999, 89.9999, -89.9999, -89.9999),
    )
    assert len(C.encode(worst)) <= C.MAX_PACKET_BYTES


# --- ground-plane tilt (r11, optional fields) --------------------------------
def test_tilt_fields_round_trip_and_default_to_none():
    plain = C.parse_packet(C.encode(status()))
    assert plain.plane_tilt is None and plain.gravity is None
    full = status(plane_tilt=C.PlaneTilt(5.25, -1.5, 120.0), gravity=C.GravityTilt(10.0, -1.0, 10.05, 4.75, 0.5))
    back = C.parse_packet(C.encode(full))
    assert back.plane_tilt == full.plane_tilt and back.gravity == full.gravity


def test_packets_without_tilt_keys_still_parse():
    payload = raw_dict(status())
    payload.pop("plane_tilt", None)
    payload.pop("gravity", None)
    assert C.parse_packet(json.dumps(payload).encode()).plane_tilt is None


@pytest.mark.parametrize("changes", [
    {"plane_tilt": {"pitch_deg": 91.0, "roll_deg": 0.0, "age_ms": 0.0}},
    {"plane_tilt": {"pitch_deg": 0.0, "roll_deg": 0.0}},
    {"plane_tilt": {"pitch_deg": 0.0, "roll_deg": 0.0, "age_ms": -1.0}},
    {"gravity": {"slope": {"pitch_deg": 0.0, "roll_deg": 0.0, "total_deg": -1.0},
                 "body_tilt": {"pitch_deg": 0.0, "roll_deg": 0.0}}},
    {"gravity": {"slope": {"pitch_deg": 0.0, "roll_deg": 0.0, "total_deg": 0.0}}},
    {"gravity": "flat"},
])
def test_bad_tilt_fields_reject_the_packet(changes):
    with pytest.raises(ValueError):
        C.parse_packet(mutate(status(), **changes))


def test_32_cell_tiles_are_refused_by_config():
    with pytest.raises(ValueError):
        MapConfig(tile_cells=32)


# --- rejections -------------------------------------------------------------
@pytest.mark.parametrize("changes", [
    {"schema_version": 2}, {"schema_version": True}, {"type": "l515_terrain"},
    {"sender_boot_id": "X" * 32}, {"sender_instance": "abc"},
    {"sender_start_mono_ns": -1}, {"sender_start_mono_ns": 2**63},
    {"map_epoch": MAX_I31 + 1}, {"sequence": -1}, {"sequence": True},
    {"status": "OK", "reason": "DEPTH_STALE"}, {"status": "NO_POSE", "reason": "GROUND_NOT_FOUND"},
    {"status": "MAYBE", "reason": None}, {"cell_m": float("nan")}, {"tile_cells": 32},
    {"age_step_s": 1.0}, {"pose": {"x_m": 0, "y_m": 0}},
    {"corrector": {"name": "magic", "applied": False, "score": None, "reason": "identity"}},
    {"corrector": {"name": "identity", "applied": False, "score": None, "reason": "x" * 40}},
    {"pose_gaps": MAX_I31 + 1}, {"thresholds": {"obstacle_height_m": 0.08, "ditch_depth_m": 0.08, "min_observations": 0}},
])
def test_bad_status_is_refused(changes):
    with pytest.raises(ValueError):
        C.parse_packet(mutate(status(), **changes))


@pytest.mark.parametrize("changes", [
    {"version": 0}, {"tile_seq": 0}, {"version": MAX_I31 + 1}, {"tx": 2**20 + 1},
    {"cells": "AAAA"}, {"cells": "!!!notbase64"},
])
def test_bad_tile_is_refused(changes):
    with pytest.raises(ValueError):
        C.parse_packet(mutate(tile(), **changes))


def test_unobserved_cell_must_carry_the_insufficient_code():
    n = 256
    bad = C.pack_cells([None] * n, [C.FREE] * n, [0] * n)
    with pytest.raises(ValueError):
        C.parse_packet(mutate(tile(), cells=bad))


@pytest.mark.parametrize("code", [3, 5, 6, 7])
def test_reserved_codes_are_refused(code):
    n = 256
    raw = bytearray(base64.b64decode(C.pack_cells([0] * n, [0] * n, [0] * n)))
    raw[1] = code << 5
    with pytest.raises(ValueError):
        C.parse_packet(mutate(tile(), cells=base64.b64encode(bytes(raw)).decode()))


def test_oversize_and_garbage_are_refused():
    with pytest.raises(ValueError):
        C.parse_packet(b"{" + b" " * 1300 + b"}")
    for raw in (b"", b"[]", b"\xff\xfe", b"null", b"[" * 5000):
        with pytest.raises(ValueError):
            C.parse_packet(raw)


def test_port_matches_remote_video_contract():
    from remote_video.contract import L515_TERRAIN_MAP_UDP_PORT
    assert L515_TERRAIN_MAP_UDP_PORT == C.UDP_PORT == 5010


# --- config -----------------------------------------------------------------
def test_default_config_is_valid():
    cfg = MapConfig()
    assert cfg.pose_wait_s == pytest.approx(cfg.lateness_margin_s + 1 / (2 * cfg.process_hz) + cfg.input_lag_budget_s)


@pytest.mark.parametrize("changes", [
    {"cell_m": 0.01}, {"pixel_stride": 5}, {"range_min_m": 1.2}, {"obstacle_height_m": 0.6},
    {"age_step_s": 1.9}, {"depth_pending_max": 3}, {"pose_wait_s": 0.3},
    {"history_length": 4}, {"camera_pitch_down_deg": 36.0}, {"corrector": "magic"},
    {"wheel_gap_invalid_s": 0.4}, {"refresh_tiles_per_tick": 0},
    {"support_base_height_m": 0.47},  # needs a measured variation
    {"support_base_height_m": 0.47, "support_base_variation_m": 0.05, "support_height_tol_m": 0.04},
    {"support_base_height_m": 0.47, "support_base_variation_m": 0.025, "support_height_tol_m": 0.03,
     "obstacle_height_m": 0.02},  # Codex 7th-round counterexample: tol < min(obstacle, ditch) - var fails
])
def test_inconsistent_config_is_refused(changes):
    with pytest.raises(ValueError):
        MapConfig(**changes)


def test_measured_support_height_config_is_accepted():
    cfg = MapConfig(support_base_height_m=0.47, support_base_variation_m=0.03, support_height_tol_m=0.04)
    assert cfg.support_height_check_enabled


# --- Codex Task 1 review ---------------------------------------------------
@pytest.mark.parametrize("field", ["distance_m", "proc_ms", "refresh_cycle_s", "cell_m"])
def test_huge_integers_are_value_errors(field):
    raw = mutate(status(), **{field: 0}).replace(f'"{field}":0'.encode(), f'"{field}":{10**309}'.encode())
    assert len(raw) <= C.MAX_PACKET_BYTES
    with pytest.raises(ValueError):
        C.parse_packet(raw)


@pytest.mark.parametrize("cell_m", [0.020049, 0.05000001])
def test_cell_size_must_survive_the_wire_exactly(cell_m):
    with pytest.raises(ValueError):
        MapConfig(cell_m=cell_m)


@pytest.mark.parametrize("cell_m, age_step_s", [(0.02, 2.0), (0.05, 2.5), (0.125, 7.25)])
def test_allowed_wire_values_round_trip_unchanged(cell_m, age_step_s):
    cfg = MapConfig(cell_m=cell_m, age_step_s=age_step_s, tile_cells=8)
    parsed = C.parse_packet(C.encode(status(cell_m=cfg.cell_m, age_step_s=cfg.age_step_s, tile_cells=8)))
    assert (parsed.cell_m, parsed.age_step_s) == (cfg.cell_m, cfg.age_step_s)
