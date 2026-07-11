"""Bounded, versioned newline-JSON control protocol."""

import json

PROTOCOL_VERSION = 1
COMMANDS = {"get_status", "set_video_mode", "set_streaming", "restart_gateway", "stop_gateway"}


class ProtocolError(ValueError):
    pass


def encode_message(message, max_bytes):
    try:
        data = json.dumps(message, separators=(",", ":"), sort_keys=True).encode("utf-8") + b"\n"
    except (TypeError, ValueError) as exc:
        raise ProtocolError("message is not JSON serializable") from exc
    if len(data) > max_bytes:
        raise ProtocolError("message exceeds size limit")
    return data


def decode_request(line, max_bytes):
    if not line or len(line) + 1 > max_bytes:
        raise ProtocolError("empty or oversized message")
    try:
        message = json.loads(line.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ProtocolError("invalid JSON") from exc
    if not isinstance(message, dict) or set(message) != {"protocol_version", "request_id", "type", "payload"}:
        raise ProtocolError("invalid envelope")
    if message["protocol_version"] != PROTOCOL_VERSION:
        raise ProtocolError("unsupported protocol version")
    if not isinstance(message["request_id"], str) or not message["request_id"]:
        raise ProtocolError("request_id must be a non-empty string")
    kind, payload = message["type"], message["payload"]
    if kind not in COMMANDS or not isinstance(payload, dict):
        raise ProtocolError("invalid command")
    if kind == "get_status" and payload:
        raise ProtocolError("get_status payload must be empty")
    if kind == "set_video_mode" and payload not in ({"mode": "rgb"}, {"mode": "depth"}, {"mode": "overlay"}):
        raise ProtocolError("invalid video mode")
    if kind == "set_streaming" and (set(payload) != {"enabled"} or type(payload["enabled"]) is not bool):
        raise ProtocolError("enabled must be boolean")
    if kind in {"restart_gateway", "stop_gateway"} and payload:
        raise ProtocolError(f"{kind} payload must be empty")
    return message


def response(request_id, payload=None, *, error=None):
    return {"protocol_version": PROTOCOL_VERSION, "request_id": request_id,
            "type": "error" if error else "response",
            "payload": {"error": error} if error else (payload or {})}

