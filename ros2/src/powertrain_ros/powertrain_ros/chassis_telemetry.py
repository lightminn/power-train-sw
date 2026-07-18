"""ROS-free encoding and polling helpers for chassis console telemetry."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
import threading
import time
from collections.abc import Callable, Mapping


MAX_TELEMETRY_BYTES = 4096
MAX_TEXT_CHARS = 256


@dataclass(frozen=True)
class PollCache:
    value: object | None
    error: str | None
    updated_monotonic_s: float | None


class LatestPollWorker:
    """Run one possibly blocking poll in a daemon and expose only its latest result."""

    def __init__(
        self,
        poll: Callable[[], object | None],
        *,
        error: Callable[[], str | None] | None = None,
        period_s: float = 1.0,
        name: str = "telemetry-poll",
    ) -> None:
        period_s = float(period_s)
        if not math.isfinite(period_s) or period_s <= 0.0:
            raise ValueError("period_s must be finite and positive")
        self._poll = poll
        self._error = error
        self._period_s = period_s
        self._cache = PollCache(None, "not polled", None)
        self._lock = threading.Lock()
        self._stopping = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name=name,
            daemon=True,
        )
        self._thread.start()

    def _run(self) -> None:
        while not self._stopping.is_set():
            started_s = time.monotonic()
            try:
                value = self._poll()
                error = None
                if value is None:
                    error = self._error() if self._error is not None else None
                    error = error or "no response"
            except Exception as exc:
                value = None
                error = f"{type(exc).__name__}: {exc}"
            cache = PollCache(value, error, time.monotonic())
            with self._lock:
                self._cache = cache
            remaining_s = self._period_s - (time.monotonic() - started_s)
            if self._stopping.wait(max(0.0, remaining_s)):
                return

    def latest(self) -> PollCache:
        with self._lock:
            return self._cache

    def close(self, *, join_timeout_s: float = 1.0) -> bool:
        self._stopping.set()
        self._thread.join(timeout=join_timeout_s)
        return not self._thread.is_alive()


def _thawed(value: object) -> object:
    # GatewayClient/ObservabilityClient hand out read-only MappingProxyType
    # payloads; json.dumps only accepts plain containers.
    if isinstance(value, Mapping):
        return {key: _thawed(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_thawed(item) for item in value]
    return value


def _encoded(payload: dict[str, object]) -> bytes:
    return json.dumps(_thawed(payload), separators=(",", ":")).encode("utf-8")


def _bounded_top_level_strings(payload: dict[str, object]) -> dict[str, object]:
    return {
        key: value[:MAX_TEXT_CHARS] if isinstance(value, str) else value
        for key, value in payload.items()
    }


def encode_telemetry_payload(payload: dict[str, object]) -> bytes:
    """Encode a bounded datagram, retaining a summary if wheel rows overflow."""
    bounded = _bounded_top_level_strings(dict(payload))
    raw = _encoded(bounded)
    if len(raw) <= MAX_TELEMETRY_BYTES:
        return raw

    bounded.pop("wheel_statuses", None)
    bounded["truncated"] = True
    raw = _encoded(bounded)
    if len(raw) <= MAX_TELEMETRY_BYTES:
        return raw

    # Defensive final reduction for an unexpectedly large structured field.
    # Keep the console populated with the scalar operational summary.
    summary_keys = (
        "schema_version", "sequence", "odometry_source", "x_m", "y_m",
        "yaw_rad", "drive_state", "can_state", "l515_state", "l515_detail",
        "l515_mode", "safety_status", "safety_distance_mm",
        "safety_estop_required", "safety_consecutive_failures",
        "safety_detail", "wheel_count", "wheel_fault_count",
        "wheel_stale_count", "wheel_axis_error_count",
        "wheel_steer_fault_count",
    )
    summary = {key: bounded[key] for key in summary_keys if key in bounded}
    summary["truncated"] = True
    raw = _encoded(summary)
    while len(raw) > MAX_TELEMETRY_BYTES:
        text_fields = [
            (len(_encoded({key: value})), key, value)
            for key, value in summary.items()
            if isinstance(value, str) and value
        ]
        if not text_fields:
            break
        _encoded_size, key, value = max(text_fields)
        summary[key] = value[: len(value) // 2]
        raw = _encoded(summary)
    if len(raw) <= MAX_TELEMETRY_BYTES:
        return raw
    return _encoded({
        "schema_version": bounded.get("schema_version", 1),
        "sequence": bounded.get("sequence", 0),
        "truncated": True,
    })
