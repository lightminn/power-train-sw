"""Validation and deterministic JSON encoding for journal events."""
from __future__ import annotations

import json
import math
from collections.abc import Mapping
from typing import Any


MAX_RECORD_BYTES = 16 * 1024

KNOWN_EVENT_TYPES = (
    "FSM_TRANSITION",
    "COMMAND_OWNER",
    "MOTION_HOLD",
    "ESTOP",
    "MISSION",
    "ARM_RESULT",
    "GRIP_LOST",
    "CONTRACT_VIOLATION",
    "OPERATOR_ACTION",
    "TERRAIN_REJECT",
    "CHANNEL_HEALTH",
    "CAN_HEALTH",
)

REQUIRED_FIELDS = (
    "schema_version",
    "run_id",
    "sequence",
    "wall_time_ns",
    "monotonic_ns",
    "source",
    "event_type",
    "severity",
    "payload",
)


def _reject_non_finite(value: Any) -> None:
    if isinstance(value, float) and not math.isfinite(value):
        raise ValueError("event values must be finite")
    if isinstance(value, Mapping):
        for key, item in value.items():
            _reject_non_finite(key)
            _reject_non_finite(item)
    elif isinstance(value, (list, tuple)):
        for item in value:
            _reject_non_finite(item)


def _require_integer(event: Mapping[str, Any], field: str, *, minimum: int) -> None:
    value = event[field]
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise ValueError(f"{field} must be an integer >= {minimum}")


def _require_text(event: Mapping[str, Any], field: str) -> None:
    value = event[field]
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty string")


def _canonical_bytes(event: Mapping[str, Any]) -> bytes:
    try:
        text = json.dumps(
            event,
            allow_nan=False,
            ensure_ascii=False,
            separators=(",", ":"),
            sort_keys=True,
        )
    except (TypeError, ValueError) as exc:
        raise ValueError(f"event is not JSON serializable: {exc}") from exc
    return text.encode("utf-8")


def validate_event(event: Mapping[str, Any]) -> None:
    """Validate one complete event record without restricting extensions."""
    if not isinstance(event, Mapping):
        raise ValueError("event must be a JSON object")

    for field in REQUIRED_FIELDS:
        if field not in event:
            raise ValueError(f"missing required field: {field}")

    _require_integer(event, "schema_version", minimum=1)
    _require_integer(event, "sequence", minimum=0)
    _require_integer(event, "wall_time_ns", minimum=0)
    _require_integer(event, "monotonic_ns", minimum=0)
    for field in ("run_id", "source", "event_type", "severity"):
        _require_text(event, field)
    if not isinstance(event["payload"], Mapping):
        raise ValueError("payload must be a JSON object")

    _reject_non_finite(event)
    if len(_canonical_bytes(event)) > MAX_RECORD_BYTES:
        raise ValueError(f"event record is too large (maximum {MAX_RECORD_BYTES} bytes)")


def encode_event(event: Mapping[str, Any]) -> bytes:
    """Return canonical UTF-8 JSON bytes without the JSONL newline."""
    validate_event(event)
    return _canonical_bytes(event)


def decode_event(encoded: bytes | bytearray | memoryview | str) -> dict[str, Any]:
    """Decode and validate one canonical or non-canonical JSON event."""
    try:
        if isinstance(encoded, str):
            event = json.loads(encoded)
        else:
            event = json.loads(bytes(encoded).decode("utf-8"))
    except (TypeError, UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid event JSON: {exc}") from exc
    validate_event(event)
    return dict(event)
