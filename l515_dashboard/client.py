"""Socket-only client for the independently managed L515 Gateway."""

from dataclasses import dataclass
from enum import Enum
from types import MappingProxyType
import json
import socket
import threading
import time
import uuid

from .protocol import PROTOCOL_VERSION, encode_message


class ClientState(str, Enum):
    CONNECTED = "CONNECTED"
    DISCONNECTED = "DISCONNECTED"


class StaleStatusError(RuntimeError): pass


def _freeze(value):
    if isinstance(value, dict): return MappingProxyType({k:_freeze(v) for k,v in value.items()})
    if isinstance(value, list): return tuple(_freeze(v) for v in value)
    return value


@dataclass(frozen=True)
class ClientSnapshot:
    request_id: str
    payload: object
    received_monotonic: float
    acknowledged: bool = False


class GatewayClient:
    def __init__(self, socket_path, *, max_message_bytes=65536, request_timeout_s=2.0):
        self.socket_path=str(socket_path); self.max_message_bytes=int(max_message_bytes)
        self.request_timeout_s=float(request_timeout_s); self.state=ClientState.DISCONNECTED
        self.snapshot=None; self.last_error=None; self._lock=threading.Lock()

    def request(self, kind, payload=None):
        request_id=uuid.uuid4().hex
        message={"protocol_version":PROTOCOL_VERSION,"request_id":request_id,"type":kind,"payload":payload or {}}
        with self._lock:
            sock=socket.socket(socket.AF_UNIX); sock.settimeout(self.request_timeout_s)
            try:
                sock.connect(self.socket_path); sock.sendall(encode_message(message,self.max_message_bytes))
                reader=sock.makefile("rb"); raw=reader.readline(self.max_message_bytes+1)
                if not raw or len(raw)>self.max_message_bytes or not raw.endswith(b"\n"): raise ConnectionError("Gateway disconnected or sent oversized response")
                reply=json.loads(raw)
                if reply.get("protocol_version") != PROTOCOL_VERSION: raise ValueError("unsupported protocol version")
                if reply.get("request_id") != request_id: raise StaleStatusError("response request_id does not match current request")
                if reply.get("type") == "error": raise RuntimeError(reply.get("payload",{}).get("error","Gateway error"))
                if reply.get("type") != "response" or not isinstance(reply.get("payload"),dict): raise ValueError("invalid Gateway response")
                payload = reply["payload"]
                acknowledged = payload.get("accepted") is True
                snap=ClientSnapshot(request_id, _freeze(payload), time.monotonic(), acknowledged)
                self.snapshot=snap; self.state=ClientState.CONNECTED; self.last_error=None
                return snap
            except Exception as exc:
                self.state=ClientState.DISCONNECTED; self.last_error=str(exc); raise
            finally: sock.close()

    def poll(self):
        try: return self.request("get_status")
        except (OSError, ConnectionError, TimeoutError): return None
