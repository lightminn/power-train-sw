"""Validated latest-only UDP telemetry for the environmental end effector."""
from __future__ import annotations

from dataclasses import dataclass
import json
import math
import socket
import threading
import time
from typing import Any

from .udp_source import SourceSequenceGate


ENVIRONMENT_STALE_AFTER_S = 2.5
MAX_DATAGRAM_BYTES = 4096
MAX_ERRORS = 8
MAX_ERROR_CHARS = 160


@dataclass(frozen=True)
class EnvironmentTelemetrySnapshot:
    sequence: int
    source: str
    source_timestamp: str | None
    sensor_ok: bool
    errors: tuple[str, ...]
    temperature_c: float | None
    humidity_pct: float | None
    pressure_hpa: float | None
    eco2_ppm: float | None
    tvoc_ppb: float | None
    sgp30_warming_up: bool
    co_estimated_ppm: float | None
    co_rs_ro: float | None
    co_range: str
    co_quality: str
    lpg_estimated_ppm: float | None
    lpg_rs_ro: float | None
    lpg_range: str
    lpg_quality: str
    flame_detected: bool | None
    flame_voltage_v: float | None
    flame_threshold_v: float | None
    received_monotonic_s: float


def _required_int(payload: dict[str, Any], name: str) -> int:
    value = payload[name]
    if isinstance(value, bool) or not isinstance(value, int) or value < 0:
        raise ValueError(f"invalid {name}")
    return value


def _optional_number(payload: dict[str, Any], name: str) -> float | None:
    value = payload.get(name)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"invalid {name}")
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f"invalid {name}")
    return result


def _required_bool(payload: dict[str, Any], name: str) -> bool:
    value = payload[name]
    if not isinstance(value, bool):
        raise ValueError(f"invalid {name}")
    return value


def _optional_bool(payload: dict[str, Any], name: str) -> bool | None:
    value = payload.get(name)
    if value is None:
        return None
    if not isinstance(value, bool):
        raise ValueError(f"invalid {name}")
    return value


def _bounded_string(
    payload: dict[str, Any], name: str, *, default: str = "unknown",
) -> str:
    value = payload.get(name, default)
    if not isinstance(value, str) or not value.strip() or len(value) > 80:
        raise ValueError(f"invalid {name}")
    return value.strip()


def _optional_string(payload: dict[str, Any], name: str) -> str | None:
    value = payload.get(name)
    if value is None:
        return None
    if not isinstance(value, str) or len(value) > 80:
        raise ValueError(f"invalid {name}")
    return value.strip() or None


def _errors(payload: dict[str, Any]) -> tuple[str, ...]:
    values = payload.get("errors", [])
    if not isinstance(values, list) or len(values) > MAX_ERRORS:
        raise ValueError("invalid errors")
    if not all(isinstance(value, str) and len(value) <= MAX_ERROR_CHARS for value in values):
        raise ValueError("invalid errors")
    return tuple(value.strip() for value in values if value.strip())


def parse_environment_telemetry(
    raw: bytes,
    received_monotonic_s: float | None = None,
) -> EnvironmentTelemetrySnapshot:
    """Validate one bounded environmental telemetry v1 datagram."""
    if len(raw) > MAX_DATAGRAM_BYTES:
        raise ValueError("oversize environment telemetry")
    payload = json.loads(raw.decode("utf-8"))
    if not isinstance(payload, dict):
        raise ValueError("invalid environment telemetry")
    if type(payload.get("schema_version")) is not int or payload["schema_version"] != 1:
        raise ValueError("unsupported schema")
    try:
        sequence = _required_int(payload, "sequence")
        sensor_ok = _required_bool(payload, "sensor_ok")
    except KeyError as exc:
        raise ValueError(f"missing {exc.args[0]}") from exc

    return EnvironmentTelemetrySnapshot(
        sequence=sequence,
        source=_bounded_string(payload, "source", default="rpi-environment"),
        source_timestamp=_optional_string(payload, "source_timestamp"),
        sensor_ok=sensor_ok,
        errors=_errors(payload),
        temperature_c=_optional_number(payload, "temperature_c"),
        humidity_pct=_optional_number(payload, "humidity_pct"),
        pressure_hpa=_optional_number(payload, "pressure_hpa"),
        eco2_ppm=_optional_number(payload, "eco2_ppm"),
        tvoc_ppb=_optional_number(payload, "tvoc_ppb"),
        sgp30_warming_up=_optional_bool(payload, "sgp30_warming_up") is True,
        co_estimated_ppm=_optional_number(payload, "co_estimated_ppm"),
        co_rs_ro=_optional_number(payload, "co_rs_ro"),
        co_range=_bounded_string(payload, "co_range"),
        co_quality=_bounded_string(payload, "co_quality"),
        lpg_estimated_ppm=_optional_number(payload, "lpg_estimated_ppm"),
        lpg_rs_ro=_optional_number(payload, "lpg_rs_ro"),
        lpg_range=_bounded_string(payload, "lpg_range"),
        lpg_quality=_bounded_string(payload, "lpg_quality"),
        flame_detected=_optional_bool(payload, "flame_detected"),
        flame_voltage_v=_optional_number(payload, "flame_voltage_v"),
        flame_threshold_v=_optional_number(payload, "flame_threshold_v"),
        received_monotonic_s=(
            time.monotonic()
            if received_monotonic_s is None
            else float(received_monotonic_s)
        ),
    )


def environment_source_state(
    snapshot: EnvironmentTelemetrySnapshot | None,
    *, now_s: float | None = None,
) -> str:
    if snapshot is None:
        return "UNAVAILABLE"
    if not snapshot.sensor_ok:
        return "UNAVAILABLE"
    current = time.monotonic() if now_s is None else float(now_s)
    return (
        "LIVE"
        if current - snapshot.received_monotonic_s <= ENVIRONMENT_STALE_AFTER_S
        else "STALE"
    )


class LatestEnvironmentTelemetryReceiver:
    """RX-only receiver retaining only the newest accepted Raspberry Pi packet."""

    def __init__(self, port: int) -> None:
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._socket.bind(("0.0.0.0", port))
        self._latest: EnvironmentTelemetrySnapshot | None = None
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._source_gate = SourceSequenceGate(stale_after_s=5.0)
        self._invalid_packet_count = 0
        self._thread = threading.Thread(
            target=self._run,
            name="environment-telemetry",
            daemon=True,
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
                snapshot = parse_environment_telemetry(
                    raw, received_monotonic_s=received_s,
                )
                accepted = self._source_gate.accept(
                    address, snapshot.sequence, now_s=received_s,
                )
            except Exception:
                self._invalid_packet_count += 1
                continue
            if not accepted:
                continue
            with self._lock:
                self._latest = snapshot

    @property
    def invalid_packet_count(self) -> int:
        return self._invalid_packet_count

    def latest(self) -> EnvironmentTelemetrySnapshot | None:
        with self._lock:
            return self._latest

    def close(self) -> None:
        self._stopping.set()
        self._socket.close()
        self._thread.join(timeout=1.0)
