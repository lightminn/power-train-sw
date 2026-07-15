"""Pure-stdlib wire protocol for the observability daemon."""
from __future__ import annotations

import json
import os
import socket
import struct
from collections.abc import Mapping
from typing import Any

from l515_dashboard.endpoint import abstract_address

from .events import MAX_RECORD_BYTES, validate_event


PROTOCOL_VERSION = 1
EVENT_SOCKET = "@powertrain-observability-events"
STATUS_SOCKET = "@powertrain-observability-status"
LOCK_PATH = "/run/powertrain/observability.lock"
RUN_DIRECTORY = "/var/lib/powertrain/runs"
MAX_DATAGRAM_BYTES = MAX_RECORD_BYTES
# The server exposes at most 32 recent event types, 32 channel summaries, and
# one latest-event alias. At the 16 KiB event ceiling this fixed 2 MiB frame
# remains bounded while leaving room for JSON keys and escaping.
MAX_STATUS_BYTES = 2 * 1024 * 1024
_CREDENTIAL_SIZE = struct.calcsize("3i")


def verify_credentials(
    raw: bytes | bytearray | memoryview,
    *,
    expected_uid: int | None = None,
) -> tuple[int, int, int]:
    """Validate kernel-supplied ``SCM_CREDENTIALS`` bytes."""
    encoded = bytes(raw)
    if len(encoded) != _CREDENTIAL_SIZE:
        raise PermissionError("missing or invalid kernel credentials")
    pid, uid, gid = struct.unpack("3i", encoded)
    owner_uid = os.geteuid() if expected_uid is None else int(expected_uid)
    if uid != owner_uid:
        raise PermissionError(f"peer UID {uid} does not match daemon UID {owner_uid}")
    return pid, uid, gid


def credentials_from_ancillary(
    ancillary: list[tuple[int, int, bytes]],
    *,
    expected_uid: int | None = None,
) -> tuple[int, int, int]:
    matches = [
        data
        for level, kind, data in ancillary
        if level == socket.SOL_SOCKET and kind == socket.SCM_CREDENTIALS
    ]
    if len(matches) != 1:
        raise PermissionError("packet must contain exactly one SCM_CREDENTIALS record")
    return verify_credentials(matches[0], expected_uid=expected_uid)


def _json_object(raw: bytes, *, label: str) -> dict[str, Any]:
    try:
        decoded = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f"invalid {label} JSON: {exc}") from exc
    if not isinstance(decoded, dict):
        raise ValueError(f"{label} JSON must be an object")
    return decoded


def decode_event_datagram(raw: bytes | bytearray | memoryview) -> dict[str, Any]:
    encoded = bytes(raw)
    if len(encoded) > MAX_DATAGRAM_BYTES:
        raise ValueError(f"event datagram exceeds size limit of {MAX_DATAGRAM_BYTES} bytes")
    event = _json_object(encoded, label="event")
    event.pop("run_id", None)
    event.pop("sequence", None)
    validate_event({**event, "run_id": "daemon-assigned", "sequence": 0})
    return event


def encode_event_datagram(event: Mapping[str, Any]) -> bytes:
    pending = dict(event)
    pending.pop("run_id", None)
    pending.pop("sequence", None)
    validate_event({**pending, "run_id": "daemon-assigned", "sequence": 0})
    encoded = json.dumps(
        pending,
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8")
    if len(encoded) > MAX_DATAGRAM_BYTES:
        raise ValueError(f"event datagram exceeds size limit of {MAX_DATAGRAM_BYTES} bytes")
    return encoded


def encode_status_request() -> bytes:
    return (
        json.dumps(
            {"protocol_version": PROTOCOL_VERSION, "type": "get_status"},
            separators=(",", ":"),
            sort_keys=True,
        ).encode("utf-8")
        + b"\n"
    )


def decode_status_request(raw: bytes) -> None:
    request = _json_object(raw, label="status request")
    if request.get("protocol_version", PROTOCOL_VERSION) != PROTOCOL_VERSION:
        raise ValueError("unsupported observability protocol version")
    if request.get("type") != "get_status":
        raise ValueError("unsupported observability status request")


def encode_status_response(snapshot: Mapping[str, Any]) -> bytes:
    encoded = json.dumps(
        {"protocol_version": PROTOCOL_VERSION, "status": dict(snapshot)},
        allow_nan=False,
        ensure_ascii=False,
        separators=(",", ":"),
        sort_keys=True,
    ).encode("utf-8") + b"\n"
    if len(encoded) > MAX_STATUS_BYTES:
        raise ValueError("observability status response exceeds size limit")
    return encoded


def decode_status_response(raw: bytes) -> dict[str, Any]:
    response = _json_object(raw, label="status response")
    if response.get("protocol_version") != PROTOCOL_VERSION:
        raise ValueError("unsupported observability protocol version")
    status = response.get("status")
    if not isinstance(status, dict):
        raise ValueError("observability status response has no status object")
    return status
