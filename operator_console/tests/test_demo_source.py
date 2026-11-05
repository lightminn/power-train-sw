import json

import pytest

from operator_console.arm_telemetry import parse_arm_telemetry
from operator_console.demo_source import (
    DEFAULT_TEST_PORTS,
    build_payloads,
    scenario_at,
    validate_ports,
)
from operator_console.environment_telemetry import parse_environment_telemetry
from operator_console.metadata import parse_metadata
from operator_console.telemetry import parse_telemetry
from terrain_overlay.contract import MAX_PACKET_BYTES, parse_packet


def _raw(payload: dict) -> bytes:
    return json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()


def test_demo_scenarios_cover_nominal_assist_and_safe_hold():
    assert scenario_at(0.0) == "NORMAL"
    assert scenario_at(7.0) == "ROUGH_APPROACH"
    assert scenario_at(13.0) == "TRACTION_ASSIST"
    assert scenario_at(19.0) == "BLOCKED_HOLD"
    assert scenario_at(24.0) == "NORMAL"

    assist = build_payloads(3, 13.0)["chassis"]
    assert assist["controller_fsm_state"] == "TRACTION_ASSIST"
    assert assist["slip_candidate"] is True
    assert len({wheel["command_turns_per_s"] for wheel in assist["wheel_statuses"]}) > 1

    hold = build_payloads(4, 19.0)["chassis"]
    assert hold["terrain_path_available"] is False
    assert hold["stuck_candidate"] is True
    assert all(wheel["command_turns_per_s"] == 0.0 for wheel in hold["wheel_statuses"])


def test_every_demo_channel_uses_the_production_decoder_and_is_bounded():
    payloads = build_payloads(11, 13.0)
    encoded = {channel: _raw(payload) for channel, payload in payloads.items()}
    assert all(len(raw) <= 4096 for raw in encoded.values())

    assert parse_telemetry(encoded["power"], 10.0).sequence == 11
    chassis = parse_telemetry(encoded["chassis"], 10.0)
    assert chassis.controller_fsm_state == "TRACTION_ASSIST"
    assert len(chassis.wheel_statuses) == 6
    assert parse_metadata(encoded["metadata"], 10.0).detections[0].depth_m == 1.28
    assert parse_arm_telemetry(encoded["arm"], 10.0).end_effector_type == "dual_gripper"
    assert parse_environment_telemetry(encoded["environment"], 10.0).sensor_ok is True
    assert len(encoded["terrain"]) <= MAX_PACKET_BYTES
    assert parse_packet(encoded["terrain"]).obstacles[0].in_path is False
    blocked = parse_packet(_raw(build_payloads(12, 19.0)["terrain"]))
    assert blocked.obstacles[0].in_path is True and blocked.zones["center"] == 0.62


def test_demo_source_rejects_live_or_colliding_ports():
    validate_ports(dict(DEFAULT_TEST_PORTS))
    with pytest.raises(ValueError, match="live UDP ports"):
        validate_ports(dict(DEFAULT_TEST_PORTS, chassis=5005))
    with pytest.raises(ValueError, match="live UDP ports"):
        validate_ports(dict(DEFAULT_TEST_PORTS, terrain=5009))
    with pytest.raises(ValueError, match="distinct"):
        validate_ports(dict(DEFAULT_TEST_PORTS, chassis=15004))


def test_terrain_map_demo_packets_parse_and_reveal_the_course():
    from operator_console.demo_source import MAP_DEMO_PERIOD_S, map_packets
    from terrain_map import contract as C
    session = ("0" * 32, 7, "a" * 16)
    early = [C.parse_packet(raw) for raw in map_packets(1, 2.0, session)]
    later = [C.parse_packet(raw) for raw in map_packets(200, 20.0, session)]
    assert all(len(raw) <= C.MAX_PACKET_BYTES for raw in map_packets(200, 20.0, session))
    assert isinstance(early[0], C.StatusPacket) and early[0].status == "OK" and early[0].map_epoch == 0
    assert later[0].pose.x_m > early[0].pose.x_m and later[0].tile_count > early[0].tile_count
    tiles = [p for p in later if isinstance(p, C.TilePacket)]
    assert tiles and len({p.tile_seq for p in tiles}) == len(tiles)
    cycle = [p for seq in range(200, 220) for p in map(C.parse_packet, map_packets(seq, 20.0, session))
             if isinstance(p, C.TilePacket)]
    assert {(p.tx, p.ty) for p in cycle} and len({(p.tx, p.ty) for p in cycle}) == later[0].tile_count
    codes = {c for p in cycle for c in C.unpack_cells(p.cells, 16)[1]}
    assert {C.OBSTACLE, C.DITCH, C.FREE, C.INSUFFICIENT} <= codes
    looped = C.parse_packet(map_packets(400, MAP_DEMO_PERIOD_S + 1.0, session)[0])
    assert looped.map_epoch == 1 and looped.tile_count < later[0].tile_count


def test_terrain_map_demo_port_is_isolated_from_the_live_one():
    validate_ports(dict(DEFAULT_TEST_PORTS))
    assert DEFAULT_TEST_PORTS["terrain_map"] == 15010
    with pytest.raises(ValueError):
        validate_ports(dict(DEFAULT_TEST_PORTS, terrain_map=5010))
