#!/usr/bin/env python3
"""Continuously feed the operator console with safe, isolated demo telemetry.

This fixture exercises the same UDP decoders as the robot, but it never opens
hardware, an ops command channel, or any production port.  It is intended for
GUI work and festival rehearsals before every producer is integrated.
"""
from __future__ import annotations

import argparse
import json
import math
import secrets
import signal
import socket
import time
from collections.abc import Callable


DEFAULT_TEST_PORTS = {
    "metadata": 15003,
    "power": 15004,
    "chassis": 15005,
    "arm": 15007,
    "environment": 15008,
    "terrain": 15009,
    "terrain_map": 15010,
}
LIVE_PORTS = frozenset({5003, 5004, 5005, 5007, 5008, 5009, 5010})
MAP_DEMO_PERIOD_S = 24.0
MAP_DEMO_SPEED_M_S = 0.25
MAP_DEMO_TILES_PER_TICK = 4
# Fixed placeholder session; ``run`` gives every demo run its own session so
# the console's terrain session gate accepts a restarted demo at once.
DEMO_TERRAIN_SESSION = ("0" * 32, 0, "0" * 16)
WHEEL_NAMES = (
    "front_left", "front_right", "mid_left",
    "mid_right", "rear_left", "rear_right",
)


def scenario_at(elapsed_s: float) -> str:
    phase = elapsed_s % 24.0
    if phase < 6.0:
        return "NORMAL"
    if phase < 12.0:
        return "ROUGH_APPROACH"
    if phase < 18.0:
        return "TRACTION_ASSIST"
    return "BLOCKED_HOLD"


def _wheel_payloads(scenario: str, elapsed_s: float) -> list[dict]:
    base_speed = {
        "NORMAL": 0.72,
        "ROUGH_APPROACH": 0.46,
        "TRACTION_ASSIST": 0.34,
        "BLOCKED_HOLD": 0.0,
    }[scenario]
    payloads = []
    for index, name in enumerate(WHEEL_NAMES):
        offset = (index - 2.5) * 0.025
        measured = base_speed + math.sin(elapsed_s * 1.4 + index) * 0.025
        command = base_speed + offset
        current = 2.4 + index * 0.18
        if scenario == "ROUGH_APPROACH":
            current += 1.0 if "front" in name else 0.35
        elif scenario == "TRACTION_ASSIST":
            # The left-middle wheel loses traction; the other wheels receive
            # slightly different commands to make redistribution visible.
            if name == "mid_left":
                measured, command, current = 0.12, 0.24, 1.1
            else:
                command += 0.10
                current += 1.5
        elif scenario == "BLOCKED_HOLD":
            measured, command = 0.0, 0.0
            current = 7.8 if name in {"front_left", "front_right"} else 3.0
        payloads.append({
            "name": name,
            "mode": "HOLD" if scenario == "BLOCKED_HOLD" else "ARMED",
            "drive_turns_per_s": round(measured, 3),
            "command_turns_per_s": round(command, 3),
            "steer_deg": round(math.sin(elapsed_s * 0.35) * 8.0, 2),
            "drive_current_a": round(current, 2),
            "steer_current_a": 0.35,
            "stale": False,
            "drive_axis_error": 0,
            "steer_fault": 0,
        })
    return payloads


def _terrain_payload(
    sequence: int, scenario: str, session: tuple[str, int, str],
) -> dict:
    """L515 terrain packet v1: one outline, in the path only when blocked."""
    boot_id, start_ns, instance = session
    blocked = scenario == "BLOCKED_HOLD"
    distance = 0.62 if blocked else 2.1
    poly = (
        [[520, 330], [760, 330], [760, 520], [520, 520]] if blocked
        else [[930, 300], [1030, 300], [1030, 380], [930, 380]]
    )
    return {
        "schema_version": 1, "type": "l515_terrain",
        "sender_boot_id": boot_id, "sender_start_mono_ns": start_ns,
        "sender_instance": instance, "sequence": sequence,
        "sent_unix_ns": time.time_ns(), "depth_stamp_ns": time.time_ns(),
        "depth_age_ms": 35.0, "status": "OK", "reason": None,
        "frame_size": [1280, 720], "distance_origin": "l515_depth_optical",
        "alignment_verified": False,
        "ground": {"state": "FRESH", "age_ms": 35.0,
                   "camera_height_m": 0.5, "camera_tilt_deg": 20.0},
        "path_half_width_m": 0.4445,
        "zones": {"left": None, "center": distance if blocked else None,
                  "right": None if blocked else distance},
        "obstacles": [{"dist_m": distance, "in_path": blocked, "poly": poly}],
        "proc_ms": 20.0,
    }


def build_payloads(
    sequence: int, elapsed_s: float,
    terrain_session: tuple[str, int, str] = DEMO_TERRAIN_SESSION,
) -> dict[str, dict]:
    """Build one coherent multi-channel frame for a deterministic scenario."""
    scenario = scenario_at(elapsed_s)
    path_available = scenario != "BLOCKED_HOLD"
    slip = scenario == "TRACTION_ASSIST"
    stuck = scenario == "BLOCKED_HOLD"
    speed_scale = {
        "NORMAL": 1.0,
        "ROUGH_APPROACH": 0.7,
        "TRACTION_ASSIST": 0.55,
        "BLOCKED_HOLD": 0.0,
    }[scenario]
    roll = math.radians(2.0 if scenario == "NORMAL" else 6.5)
    pitch = math.radians({
        "NORMAL": 1.5,
        "ROUGH_APPROACH": 10.0,
        "TRACTION_ASSIST": 7.0,
        "BLOCKED_HOLD": 12.0,
    }[scenario])
    controller_state = {
        "NORMAL": "TRACKING",
        "ROUGH_APPROACH": "SPEED_LIMIT",
        "TRACTION_ASSIST": "TRACTION_ASSIST",
        "BLOCKED_HOLD": "SAFE_HOLD",
    }[scenario]
    terrain_reasons = ["forward obstacle"] if stuck else []
    wheels = _wheel_payloads(scenario, elapsed_s)
    chassis = {
        "schema_version": 1,
        "sequence": sequence,
        "odometry_source": "demo wheel+imu",
        "x_m": round(elapsed_s * 0.05, 2),
        "y_m": 0.02,
        "yaw_rad": math.radians(math.sin(elapsed_s * 0.15) * 6.0),
        "roll_rad": roll,
        "pitch_rad": pitch,
        "drive_state": "HOLD/OBSTACLE" if stuck else "DRIVING/RUN",
        "can_state": "OK · demo",
        "l515_state": "RUNNING",
        "l515_detail": "demo processed terrain",
        "l515_mode": "fixture",
        "l515_color_hz": 30.0,
        "l515_depth_hz": 30.0,
        "l515_submitted_hz": 30.0,
        "l515_sent_hz": 30.0,
        "l515_drop_hz": 0.0,
        "l515_ros_topic_rates_hz": {},
        "l515_aligned_depth_age_ms": 28.0,
        "safety_status": "VALID",
        "safety_distance_mm": 520.0 if stuck else 1450.0,
        "safety_estop_required": False,
        "safety_consecutive_failures": 0,
        "safety_detail": "",
        "component_mask": {
            "drive": True, "steer": True, "us100": True, "robot_arm": True,
        },
        "wheel_count": 6,
        "wheel_fault_count": 0,
        "wheel_stale_count": 0,
        "wheel_axis_error_count": 0,
        "wheel_steer_fault_count": 0,
        "wheel_statuses": wheels,
        "terrain_path_available": path_available,
        "terrain_path_offset_m": 0.04 if path_available else None,
        "terrain_heading_error_rad": -0.03 if path_available else None,
        "terrain_support_m": 1.8 if path_available else 0.45,
        "terrain_bank_rad": roll,
        "terrain_slope_rad": pitch,
        "terrain_roughness_m": 0.018 if scenario == "NORMAL" else 0.055,
        "terrain_confidence": 0.92 if scenario == "NORMAL" else 0.84,
        "terrain_reject_reasons": terrain_reasons,
        "controller_fsm_state": controller_state,
        "controller_fsm_reasons": terrain_reasons,
        "degradation_state": "NORMAL" if speed_scale == 1.0 else "LIMITED",
        "degradation_reasons": [] if speed_scale == 1.0 else [scenario.lower()],
        "degradation_speed_scale": speed_scale,
        "mission_fsm_state": "DRIVE" if not stuck else "HOLD",
        "mission_fsm_reason": scenario.lower(),
        "section_fsm_section": "RUBBLE_APPROACH",
        "section_fsm_phase": scenario,
        "section_fsm_notices": terrain_reasons,
        "slip_candidate": slip,
        "stuck_candidate": stuck,
        "truncated": False,
    }
    power = {
        "schema_version": 1,
        "sequence": sequence,
        "voltage_v": 47.8,
        "current_a": round(sum(wheel["drive_current_a"] for wheel in wheels), 1),
        "power_w": 520.0 if not stuck else 810.0,
        "pdist_soc_percent": 82,
        "pdist_battery_flags": 0,
        "pdist_protection_flags": 0,
        "pdist_charge_current_a": 0.0,
        "rs485_state": "OK",
        "rs485_consecutive_failures": 0,
        "rs485_detail": "demo fixture",
        "unit_status": {"demo-source": "active"},
        "compose_status": {"operator-console-demo": "healthy"},
        "journal_tail": [],
    }
    metadata = {
        "schema_version": 1,
        "capture_sequence": sequence,
        "capture_stamp_ns": time.time_ns(),
        "frame_width": 848,
        "frame_height": 480,
        "frame_id": "camera_color_optical_frame",
        "detections": [{
            "class_id": 0,
            "class_name": "구조 대상 물체",
            "confidence": 0.91,
            "bbox_xywh": [330, 155, 150, 180],
            "position_m": [0.05, -0.04, 1.28],
            "depth_m": 1.28,
            "yaw_rad": 0.04,
            "is_pick_target": True,
        }],
    }
    arm = {
        "schema_version": 1,
        "sequence": sequence,
        "dynamixel": [
            {"id": 11, "position_raw": 3072, "position_deg": 90.0,
             "velocity": 0, "current": 12, "temperature_c": 36},
            {"id": 12, "position_raw": 2048, "position_deg": 0.0,
             "velocity": 0, "current": 10, "temperature_c": 37},
        ],
        "joints": {
            "names": ["arm_joint_1", "arm_joint_2"],
            "position_rad": [0.25, -0.5],
            "velocity": [0.0, 0.0],
            "effort_raw": [12.0, 10.0],
        },
        "source_age_s": {"dynamixel": 0.05, "joints": 0.05, "detections": 0.05},
        "tool_runtime": {
            "tool": {"tool_type": "dual_motor_gripper", "actuator_ids": [3, 4],
                     "actuators": [{"id": 3, "online": True}, {"id": 4, "online": True}],
                     "control_scope": "END_EFFECTOR_ONLY", "control_mode": "MANUAL",
                     "read_only": True, "emergency_stop": False},
            "source_age_s": 0.05, "coupling_lock_age_s": 0.05,
            "coupling_lock": {"enabled": True, "state": "LOCKED", "zero_tick": 2048,
                              "zero_state": "LOCKED", "position": 2048, "torque": 0,
                              "sequence_state": "IDLE", "coupling_proxy": "DISABLED",
                              "mechanically_confirmed": False},
        },
        "arm_runtime": {"control_mode": "MANUAL", "fsm_state": "IDLE",
                        "arm_status": "READY", "source_age_s": {
                            "control_mode": 0.05, "fsm_state": 0.05, "arm_status": 0.05}},
        "end_effector_id": "demo-gripper",
        "end_effector_type": "dual_gripper",
        "end_effector_attached": True,
        "end_effector_interface": "fixture",
        "truncated": False,
    }
    environment = {
        "schema_version": 1,
        "sequence": sequence,
        "source": "demo-environment",
        "sensor_ok": True,
        "errors": [],
        "temperature_c": 29.1,
        "humidity_pct": 56.9,
        "pressure_hpa": 1003.1,
        "eco2_ppm": 410.0,
        "tvoc_ppb": 3.0,
        "sgp30_warming_up": False,
        "co_estimated_ppm": 1.0,
        "co_rs_ro": 1.019,
        "co_range": "below_detection_range",
        "co_quality": "demo_only",
        "lpg_estimated_ppm": 27.9,
        "lpg_rs_ro": 1.012,
        "lpg_range": "below_detection_range",
        "lpg_quality": "demo_only",
        "flame_detected": False,
        "flame_voltage_v": 2.75,
        "flame_threshold_v": 1.5,
    }
    return {
        "metadata": metadata,
        "power": power,
        "chassis": chassis,
        "arm": arm,
        "environment": environment,
        "terrain": _terrain_payload(sequence, scenario, terrain_session),
    }


def _map_course(x: float, y: float) -> tuple[int | None, int]:
    """Demo course height (cm) and class: gentle waves, a box and a ditch; outside the cone unobserved."""
    from terrain_map import contract as C
    if x < 0.3 or abs(y) > 0.6 + 0.45 * x:
        return None, C.INSUFFICIENT
    if 2.4 <= x <= 2.7 and 0.35 <= y <= 0.65:                  # beside the demo path, not on it
        return 15, C.OBSTACLE
    if 3.6 <= x <= 3.9 and -0.9 <= y <= -0.35:
        return -20, C.DITCH
    return int(round(3.0 * math.sin(2.0 * x) + 2.0 * y)), C.FREE


def map_packets(sequence: int, elapsed_s: float, session: tuple[str, int, str]) -> list[bytes]:
    """Terrain map contract v1 (:5010) demo: a rover driving along x reveals a course;
    every ``MAP_DEMO_PERIOD_S`` the run restarts in a new epoch."""
    from terrain_map import contract as C
    epoch, t = divmod(elapsed_s, MAP_DEMO_PERIOD_S)
    x = MAP_DEMO_SPEED_M_S * t
    reach = x + 3.0
    revealed = [(tx, ty) for tx in range(0, 8) for ty in range(-3, 3) if tx * 0.8 < reach]
    common = dict(sender_boot_id=session[0], sender_start_mono_ns=session[1], sender_instance=session[2],
                  map_epoch=int(epoch), sent_unix_ns=time.time_ns())
    packets = [C.encode(C.StatusPacket(
        **common, sequence=sequence, status="OK", reason=None, cell_m=0.05, tile_cells=16, age_step_s=2.0,
        pose=C.Pose(x, 0.05 * math.sin(t / 3.0), 0.05 * math.cos(t / 3.0)), distance_m=x, pose_gaps=0,
        not_support=0, late_dropped=0, tile_count=len(revealed),
        refresh_cycle_s=math.ceil(len(revealed) / MAP_DEMO_TILES_PER_TICK) / 10.0,
        thresholds=C.Thresholds(0.08, 0.08, 2), corrector=C.CorrectorInfo("identity", False, None, "identity"),
        proc_ms=18.0))]
    for k in range(min(MAP_DEMO_TILES_PER_TICK, len(revealed))):
        tx, ty = revealed[(sequence * MAP_DEMO_TILES_PER_TICK + k) % len(revealed)]
        heights, codes = [], []
        for gy in range(16):
            for gx in range(16):
                h, c = _map_course((tx * 16 + gx + 0.5) * 0.05, (ty * 16 + gy + 0.5) * 0.05)
                heights.append(h)
                codes.append(c)
        packets.append(C.encode(C.TilePacket(
            **common, tx=tx, ty=ty, version=1, tile_seq=sequence * MAP_DEMO_TILES_PER_TICK + k + 1,
            cells=C.pack_cells(heights, codes, [0] * 256))))
    return packets


def validate_ports(ports: dict[str, int]) -> None:
    if set(ports) != set(DEFAULT_TEST_PORTS):
        raise ValueError("all demo channels are required")
    if any(type(port) is not int or not 1 <= port <= 65535 for port in ports.values()):
        raise ValueError("ports must be integers from 1 to 65535")
    if len(set(ports.values())) != len(ports):
        raise ValueError("demo channels must use distinct ports")
    blocked = LIVE_PORTS.intersection(ports.values())
    if blocked:
        raise ValueError(f"live UDP ports are blocked: {sorted(blocked)}")


def run(
    *, host: str, ports: dict[str, int], interval_s: float,
    duration_s: float | None = None,
    stop_requested: Callable[[], bool] | None = None,
) -> None:
    validate_ports(ports)
    if interval_s <= 0:
        raise ValueError("interval_s must be greater than zero")
    if duration_s is not None and duration_s <= 0:
        raise ValueError("duration_s must be greater than zero")
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    started = time.monotonic()
    terrain_session = ("0" * 32, time.monotonic_ns(), secrets.token_hex(8))
    sequence = 0
    try:
        while (
            (duration_s is None or time.monotonic() - started < duration_s)
            and not (stop_requested is not None and stop_requested())
        ):
            elapsed = time.monotonic() - started
            sequence += 1
            payloads = build_payloads(sequence, elapsed, terrain_session)
            for channel, payload in payloads.items():
                raw = json.dumps(payload, ensure_ascii=False, separators=(",", ":")).encode()
                if len(raw) > 4096:
                    raise RuntimeError(f"{channel} demo datagram exceeds 4096 bytes")
                sender.sendto(raw, (host, ports[channel]))
            for raw in map_packets(sequence, elapsed, terrain_session):
                sender.sendto(raw, (host, ports["terrain_map"]))
            deadline = started + sequence * interval_s
            time.sleep(max(0.0, deadline - time.monotonic()))
    finally:
        sender.close()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--rate-hz", type=float, default=10.0)
    parser.add_argument("--duration", type=float, default=None)
    for channel, port in DEFAULT_TEST_PORTS.items():
        parser.add_argument(f"--{channel}-port", type=int, default=port)
    args = parser.parse_args(argv)
    if args.rate_hz <= 0:
        parser.error("--rate-hz must be greater than zero")
    if args.duration is not None and args.duration <= 0:
        parser.error("--duration must be greater than zero")
    ports = {
        channel: getattr(args, f"{channel}_port")
        for channel in DEFAULT_TEST_PORTS
    }
    try:
        validate_ports(ports)
    except ValueError as exc:
        parser.error(str(exc))
    stop = False

    def request_stop(_signum: int, _frame: object) -> None:
        nonlocal stop
        stop = True

    # Keep Ctrl+C output clean.  A short bounded chunk lets us honour the flag
    # without adding a background thread or a command surface.
    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    print(
        "TEST 전용 GUI 데이터 송신 시작 · 하드웨어/조작 채널 없음 · "
        + " · ".join(f"{name}:{port}" for name, port in ports.items()),
        flush=True,
    )
    run(
        host=args.host,
        ports=ports,
        interval_s=1.0 / args.rate_hz,
        duration_s=args.duration,
        stop_requested=lambda: stop,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
