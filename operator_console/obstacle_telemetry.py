"""Validated latest-only L515 obstacle telemetry for the operator console."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import socket
import threading
import time

from .udp_source import SourceSequenceGate


OBSTACLE_STALE_AFTER_S = 0.75
MAX_DATAGRAM_BYTES = 4096
VALID_STATES = frozenset({"OK", "NO_GROUND", "SENSOR_ERROR"})
ZONE_NAMES = ("left", "center", "right")


@dataclass(frozen=True)
class ObstacleZone:
    valid: bool
    distance_m: float | None
    detected: bool | None


@dataclass(frozen=True)
class ObstacleTelemetrySnapshot:
    sequence: int
    capture_timestamp_s: float
    state: str
    zones: dict[str, ObstacleZone]
    processing_ms: float | None
    detail: str
    received_monotonic_s: float


def _finite_number(value: object, name: str, *, minimum: float = 0.0) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{name} must be a number")
    result = float(value)
    if not math.isfinite(result) or result < minimum:
        raise ValueError(f"invalid {name}")
    return result


def parse_obstacle_telemetry(
    raw: bytes, received_monotonic_s: float | None = None,
) -> ObstacleTelemetrySnapshot:
    if len(raw) > MAX_DATAGRAM_BYTES:
        raise ValueError("oversize obstacle telemetry")
    try:
        payload = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError("invalid obstacle telemetry") from exc
    if not isinstance(payload, dict) or payload.get("schema_version") != 1:
        raise ValueError("unsupported obstacle telemetry schema")
    sequence = payload.get("sequence")
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError("invalid sequence")
    state = payload.get("state")
    if state not in VALID_STATES:
        raise ValueError("invalid state")
    capture_timestamp_s = _finite_number(
        payload.get("capture_timestamp_s"), "capture_timestamp_s",
    )
    zones_payload = payload.get("zones")
    if not isinstance(zones_payload, dict) or set(zones_payload) != set(ZONE_NAMES):
        raise ValueError("left, center and right zones are required")
    zones: dict[str, ObstacleZone] = {}
    for name in ZONE_NAMES:
        zone = zones_payload[name]
        if not isinstance(zone, dict) or type(zone.get("valid")) is not bool:
            raise ValueError(f"invalid {name} zone")
        detected = zone.get("detected")
        if detected is not None and type(detected) is not bool:
            raise ValueError(f"invalid {name}.detected")
        distance = zone.get("distance_m")
        if distance is not None:
            distance = _finite_number(distance, f"{name}.distance_m")
        if not zone["valid"] and (distance is not None or detected is not None):
            raise ValueError(f"invalid {name} zone must not carry a result")
        if detected is True and distance is None:
            raise ValueError(f"detected {name} zone requires distance")
        zones[name] = ObstacleZone(zone["valid"], distance, detected)
    if state != "OK" and any(zone.valid for zone in zones.values()):
        raise ValueError("non-OK state cannot certify valid zones")
    processing = payload.get("processing_ms")
    if processing is not None:
        processing = _finite_number(processing, "processing_ms")
    detail = payload.get("detail", "")
    if not isinstance(detail, str) or len(detail) > 160:
        raise ValueError("invalid detail")
    return ObstacleTelemetrySnapshot(
        sequence=sequence,
        capture_timestamp_s=capture_timestamp_s,
        state=state,
        zones=zones,
        processing_ms=processing,
        detail=detail,
        received_monotonic_s=(
            time.monotonic() if received_monotonic_s is None
            else float(received_monotonic_s)
        ),
    )


def obstacle_source_state(
    snapshot: ObstacleTelemetrySnapshot | None, *, now_s: float | None = None,
) -> str:
    if snapshot is None:
        return "UNAVAILABLE"
    current = time.monotonic() if now_s is None else float(now_s)
    return (
        "LIVE" if current - snapshot.received_monotonic_s <= OBSTACLE_STALE_AFTER_S
        else "STALE"
    )


class LatestObstacleTelemetryReceiver:
    """Latest-only UDP receiver; observation only and never controls motion."""

    def __init__(self, port: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", port))
        self._latest: ObstacleTelemetrySnapshot | None = None
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._source_gate = SourceSequenceGate(stale_after_s=2.0)
        self._invalid_packet_count = 0
        self._thread = threading.Thread(
            target=self._run, name="l515-obstacle-telemetry", daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        self._socket.settimeout(0.2)
        while not self._stopping.is_set():
            try:
                raw, address = self._socket.recvfrom(MAX_DATAGRAM_BYTES + 1)
            except OSError:
                continue
            received_s = time.monotonic()
            try:
                snapshot = parse_obstacle_telemetry(raw, received_s)
                accepted = self._source_gate.accept(
                    address, snapshot.sequence, now_s=received_s,
                )
            except Exception:
                self._invalid_packet_count += 1
                continue
            if accepted:
                with self._lock:
                    self._latest = snapshot

    @property
    def invalid_packet_count(self) -> int:
        return self._invalid_packet_count

    def latest(self) -> ObstacleTelemetrySnapshot | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stopping.set()
        self._socket.close()
        self._thread.join(timeout=1.0)
