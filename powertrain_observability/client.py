"""Short-lived event and status clients for the independent daemon."""
from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType
import socket
import threading
import time
from typing import Any

from .protocol import (
    EVENT_SOCKET,
    MAX_STATUS_BYTES,
    STATUS_SOCKET,
    abstract_address,
    decode_status_response,
    encode_event_datagram,
    encode_status_request,
)


class ClientState(str, Enum):
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"


def _freeze(value: Any) -> Any:
    if isinstance(value, dict):
        return MappingProxyType({key: _freeze(item) for key, item in value.items()})
    if isinstance(value, list):
        return tuple(_freeze(item) for item in value)
    return value


@dataclass(frozen=True)
class StatusSnapshot:
    payload: object
    received_monotonic: float


class EventClient:
    def __init__(self, socket_path: str = EVENT_SOCKET) -> None:
        self.socket_path = str(socket_path)
        self.last_error: str | None = None

    def emit(self, event) -> bool:
        encoded = encode_event_datagram(event)
        sock = None
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
            sock.setblocking(False)
            sent = sock.sendto(encoded, abstract_address(self.socket_path))
            if sent != len(encoded):
                self.last_error = "event datagram was not sent atomically"
                return False
            self.last_error = None
            return True
        except OSError as exc:
            self.last_error = str(exc)
            return False
        finally:
            if sock is not None:
                sock.close()


class ObservabilityClient:
    def __init__(
        self,
        socket_path: str = STATUS_SOCKET,
        *,
        max_message_bytes: int = MAX_STATUS_BYTES,
        request_timeout_s: float = 2.0,
    ) -> None:
        self.socket_path = str(socket_path)
        self.max_message_bytes = int(max_message_bytes)
        self.request_timeout_s = float(request_timeout_s)
        self.state = ClientState.DISCONNECTED
        self.snapshot: StatusSnapshot | None = None
        self.last_error: str | None = None
        self._lock = threading.Lock()

    def query(self) -> StatusSnapshot:
        with self._lock:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.settimeout(self.request_timeout_s)
            reader = None
            try:
                sock.connect(abstract_address(self.socket_path))
                sock.sendall(encode_status_request())
                reader = sock.makefile("rb")
                raw = reader.readline(self.max_message_bytes + 1)
                if not raw or len(raw) > self.max_message_bytes or not raw.endswith(b"\n"):
                    raise ConnectionError(
                        "observability daemon disconnected or sent oversized status"
                    )
                payload = decode_status_response(raw[:-1])
                snapshot = StatusSnapshot(_freeze(payload), time.monotonic())
                self.snapshot = snapshot
                self.state = ClientState.CONNECTED
                self.last_error = None
                return snapshot
            except Exception as exc:
                self.state = ClientState.DISCONNECTED
                self.last_error = str(exc)
                raise
            finally:
                if reader is not None:
                    reader.close()
                sock.close()

    def poll(self) -> StatusSnapshot | None:
        try:
            return self.query()
        except Exception:
            return None
