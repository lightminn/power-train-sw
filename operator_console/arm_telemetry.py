"""Versioned robot-arm telemetry validation and latest-only UDP reception."""
from __future__ import annotations

from dataclasses import dataclass
import json
import socket
import threading
import time
from typing import Any

from .udp_source import SourceSequenceGate


@dataclass(frozen=True)
class DynamixelMotorStatus:
    id: int
    position_raw: int
    position_deg: float
    velocity: int
    current: int
    temperature_c: int


@dataclass(frozen=True)
class ArmTelemetrySnapshot:
    sequence: int
    dynamixel: tuple[DynamixelMotorStatus, ...] | None
    joint_names: tuple[str, ...]
    joint_position_rad: tuple[float, ...]
    joint_velocity: tuple[float, ...]
    dynamixel_age_s: float | None
    joints_age_s: float | None
    detections_age_s: float | None
    truncated: bool
    received_monotonic_s: float


def _required_int(payload: dict[str, Any], name: str) -> int:
    value = payload[name]
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"invalid {name}")
    return value


def _required_number(payload: dict[str, Any], name: str) -> float:
    value = payload[name]
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"invalid {name}")
    return float(value)


def _optional_number(payload: dict[str, Any], name: str) -> float | None:
    value = payload.get(name)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"invalid {name}")
    return float(value)


def _parse_dynamixel(payload: dict[str, Any]) -> tuple[DynamixelMotorStatus, ...] | None:
    raw_motors = payload.get("dynamixel")
    if raw_motors is None:
        return None
    if not isinstance(raw_motors, list) or len(raw_motors) > 8:
        raise ValueError("invalid dynamixel")
    motors = []
    for raw in raw_motors:
        if not isinstance(raw, dict):
            raise ValueError("invalid dynamixel motor")
        try:
            motors.append(DynamixelMotorStatus(
                id=_required_int(raw, "id"),
                position_raw=_required_int(raw, "position_raw"),
                position_deg=_required_number(raw, "position_deg"),
                velocity=_required_int(raw, "velocity"),
                current=_required_int(raw, "current"),
                temperature_c=_required_int(raw, "temperature_c"),
            ))
        except KeyError as exc:
            raise ValueError("invalid dynamixel motor") from exc
    return tuple(motors)


def _parse_joints(
    payload: dict[str, Any],
) -> tuple[tuple[str, ...], tuple[float, ...], tuple[float, ...]]:
    raw_joints = payload.get("joints")
    if raw_joints is None:
        return (), (), ()
    if not isinstance(raw_joints, dict):
        raise ValueError("invalid joints")
    try:
        raw_names = raw_joints["names"]
        raw_position = raw_joints["position_rad"]
        raw_velocity = raw_joints["velocity"]
    except KeyError as exc:
        raise ValueError("invalid joints") from exc
    if not all(isinstance(values, list) for values in (
        raw_names, raw_position, raw_velocity,
    )):
        raise ValueError("invalid joints")
    if len(raw_names) > 16 or not (
        len(raw_names) == len(raw_position) == len(raw_velocity)
    ):
        raise ValueError("invalid joints")
    try:
        names = tuple(str(name) for name in raw_names)
        position = tuple(float(value) for value in raw_position)
        velocity = tuple(float(value) for value in raw_velocity)
    except (TypeError, ValueError) as exc:
        raise ValueError("invalid joints") from exc
    return names, position, velocity


def parse_arm_telemetry(
    raw: bytes,
    received_monotonic_s: float | None = None,
) -> ArmTelemetrySnapshot:
    """Validate the bounded arm telemetry v1 datagram."""
    if len(raw) > 4096:
        raise ValueError("oversize arm telemetry")
    payload = json.loads(raw.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("invalid arm telemetry")
    schema_version = payload.get("schema_version")
    if type(schema_version) is not int or schema_version != 1:
        raise ValueError("unsupported schema")
    try:
        sequence = _required_int(payload, "sequence")
    except KeyError as exc:
        raise ValueError("invalid sequence") from exc
    dynamixel = _parse_dynamixel(payload)
    joint_names, joint_position_rad, joint_velocity = _parse_joints(payload)
    source_age_s = payload.get("source_age_s", {})
    if source_age_s is None:
        source_age_s = {}
    if not isinstance(source_age_s, dict):
        raise ValueError("invalid source_age_s")
    return ArmTelemetrySnapshot(
        sequence=sequence,
        dynamixel=dynamixel,
        joint_names=joint_names,
        joint_position_rad=joint_position_rad,
        joint_velocity=joint_velocity,
        dynamixel_age_s=_optional_number(source_age_s, "dynamixel"),
        joints_age_s=_optional_number(source_age_s, "joints"),
        detections_age_s=_optional_number(source_age_s, "detections"),
        truncated=payload.get("truncated") is True,
        received_monotonic_s=(
            time.monotonic() if received_monotonic_s is None else received_monotonic_s
        ),
    )


def temperature_state(temp_c: int) -> str:
    # Temporary: arm-team per-model limits are unconfirmed (2026-07-18);
    # update these thresholds after confirmation.
    if temp_c < 55:
        return "NORMAL"
    if temp_c < 65:
        return "WARN"
    return "CRIT"


def arm_summary(snapshot: ArmTelemetrySnapshot | None) -> str:
    """Return the compact robot-arm motor-temperature summary."""
    if snapshot is None or snapshot.dynamixel is None:
        return "미수신(UNAVAILABLE)"
    if not snapshot.dynamixel:
        return "모터 0 · 온도 미수신"
    highest = max(motor.temperature_c for motor in snapshot.dynamixel)
    state = temperature_state(highest)
    if state == "NORMAL":
        temperature = f"최고 {highest} ℃ 정상"
    elif state == "WARN":
        temperature = f"최고 {highest} ℃ 경고"
    else:
        temperature = f"최고 ⚠ {highest} ℃"
    return f"모터 {len(snapshot.dynamixel)} · {temperature}"


class LatestArmTelemetryReceiver:
    """Latest-only RX-bound UDP receiver for robot-arm observation."""
    def __init__(self, port: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", port))
        self._latest: ArmTelemetrySnapshot | None = None
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._source_gate = SourceSequenceGate(stale_after_s=2.0)
        self._thread = threading.Thread(target=self._run, name="arm-telemetry", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        self._socket.settimeout(0.2)
        while not self._stopping.is_set():
            try:
                raw, address = self._socket.recvfrom(4097)
                received_s = time.monotonic()
                snapshot = parse_arm_telemetry(raw, received_monotonic_s=received_s)
            except (OSError, ValueError, UnicodeDecodeError, json.JSONDecodeError):
                continue
            if not self._source_gate.accept(
                address,
                snapshot.sequence,
                now_s=received_s,
            ):
                continue
            with self._lock:
                self._latest = snapshot

    def latest(self) -> ArmTelemetrySnapshot | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stopping.set()
        self._socket.close()
        self._thread.join(timeout=1.0)
