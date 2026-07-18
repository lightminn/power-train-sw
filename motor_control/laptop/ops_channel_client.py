"""Pygame-free A2a ops client and DualSense recovery chord detector.

The constants intentionally mirror ``powertrain_ros.ops_contract`` because the
laptop client must remain importable without the ROS workspace installed.
"""
from dataclasses import dataclass
import json
import socket
import time
import uuid


OPS_SCHEMA_VERSION = 1
OPS_DEFAULT_PORT = 9001
RETRANSMIT_INTERVAL_S = 0.25
REQUEST_DEADLINE_S = 2.0
# ⚠️ recovery-v1-initial-candidate — HIL·운전자 피드백 후 변경 전제(임시).
EMERGENCY_HOLD_S = {"estop_reset": 5.0, "arm": 3.0}
CHORD_CLEAR_HOLD_NS = 2_000_000_000
CHORD_AUTHORITY_HOLD_NS = 1_000_000_000
_RECONNECT_INTERVAL_S = 1.0


class RecoveryChordDetector:
    """Translate recovery-v1-initial-candidate holds into ops actions."""

    def __init__(self):
        self._holds = {}
        self._emergency = {}

    @staticmethod
    def _button_callback(sample):
        callback = getattr(sample, "button", None)
        if callback is None:
            callback = getattr(sample, "buttons", None)
        if not callable(callback):
            raise TypeError("sample must provide button(name) callback")
        return callback

    def _held_action(self, key, active, now_ns, hold_ns, action):
        state = self._holds.setdefault(
            key, {"started_ns": None, "latched": False}
        )
        if not active:
            state["started_ns"] = None
            state["latched"] = False
            return []
        if state["started_ns"] is None:
            state["started_ns"] = now_ns
        if (
            not state["latched"]
            and now_ns - state["started_ns"] >= hold_ns
        ):
            state["latched"] = True
            return [{"action": action}]
        return []

    def _emergency_action(self, action, active, now_ns):
        state = self._emergency.setdefault(
            action, {"started_ns": None, "executed": False}
        )
        if not active:
            state["started_ns"] = None
            state["executed"] = False
            return []
        if state["started_ns"] is None:
            state["started_ns"] = now_ns
            return [{"action": action, "phase": "begin"}]
        hold_ns = int(EMERGENCY_HOLD_S[action] * 1_000_000_000)
        if (
            not state["executed"]
            and now_ns - state["started_ns"] >= hold_ns
        ):
            state["executed"] = True
            return [{"action": action, "phase": "execute"}]
        return []

    def update(self, sample, *, now_ns):
        """Return newly triggered actions for one pure button/D-pad sample."""
        now_ns = int(now_ns)
        button = self._button_callback(sample)
        dpad_y = int(sample.dpad_y)
        square = bool(button("square"))
        create = bool(button("create"))
        l1 = bool(button("l1"))
        r1 = bool(button("r1"))
        triangle = bool(button("triangle"))

        events = []
        events.extend(self._held_action(
            "clear_transient_hold",
            square and create,
            now_ns,
            CHORD_CLEAR_HOLD_NS,
            "clear_transient_hold",
        ))
        events.extend(self._held_action(
            "authority_manual",
            dpad_y == -1,
            now_ns,
            CHORD_AUTHORITY_HOLD_NS,
            "authority_manual",
        ))
        events.extend(self._held_action(
            "authority_auto",
            dpad_y == 1,
            now_ns,
            CHORD_AUTHORITY_HOLD_NS,
            "authority_auto",
        ))
        events.extend(self._emergency_action(
            "estop_reset", l1 and r1 and square, now_ns
        ))
        events.extend(self._emergency_action(
            "arm", l1 and r1 and triangle, now_ns
        ))
        return events


def open_ops_socket(host, port):
    sock = socket.create_connection((host, int(port)), timeout=0.25)
    try:
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        sock.setblocking(False)
        return sock
    except BaseException:
        sock.close()
        raise


@dataclass
class _PendingRequest:
    record: bytes
    created_s: float
    last_sent_s: float | None = None


class OpsChannelClient:
    """Correlating newline-JSON client with bounded idempotent retransmits."""

    def __init__(
        self,
        host,
        port,
        token,
        *,
        clock=time.monotonic,
        connector=open_ops_socket,
        sleep_fn=time.sleep,
        client_id=None,
    ):
        if token is None or not str(token).strip():
            raise ValueError("ops token must be non-empty")
        # 서버는 (token, client_id) 로 신원을 고정하고 그 신원에 멱등 캐시와
        # status_query 를 스코프한다. 연결마다 새 값이면 재연결 시 같은
        # request_id 가 다시 실행되므로, **재연결 간 변하지 않아야 한다**.
        # 인스턴스당 1회 생성이 그 조건을 만족한다. 프로세스 재시작 간에도
        # 유지하려면 호출자가 영속 값을 넘긴다.
        self.client_id = (
            str(uuid.uuid4()) if client_id is None else str(client_id).strip()
        )
        if not self.client_id:
            raise ValueError("client_id must be non-empty")
        self.host = str(host)
        self.port = int(port)
        self.token = str(token).strip()
        self.clock = clock
        self.connector = connector
        self.sleep_fn = sleep_fn
        self.sock = None
        self._last_connect_attempt_s = None
        self._next_sequence = 0
        self._pending = {}
        self._recv_buffer = bytearray()
        self._latest_ops_state = None
        self._closed = False
        self._connect(force=True)

    def _hello_record(self):
        return (
            json.dumps(
                {
                    "schema_version": OPS_SCHEMA_VERSION,
                    "hello": True,
                    "token": self.token,
                    "client_id": self.client_id,
                    "stamp_s": float(self.clock()),
                },
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")

    def _disconnect(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self._recv_buffer.clear()

    def _send_record(self, record):
        if self.sock is None:
            return False
        try:
            self.sock.sendall(record)
        except OSError:
            self._disconnect()
            return False
        return True

    def _connect(self, *, force=False):
        if self._closed or self.sock is not None:
            return self.sock is not None
        now_s = float(self.clock())
        if (
            not force
            and self._last_connect_attempt_s is not None
            and now_s - self._last_connect_attempt_s < _RECONNECT_INTERVAL_S
        ):
            return False
        self._last_connect_attempt_s = now_s
        try:
            sock = self.connector(self.host, self.port)
        except OSError:
            return False
        self.sock = sock
        if not self._send_record(self._hello_record()):
            return False
        for pending in self._pending.values():
            if not self._send_record(pending.record):
                return False
            pending.last_sent_s = now_s
        return True

    def submit(
        self,
        action,
        *,
        params=None,
        phase=None,
        request_id=None,
        expected_state_revision=None,
    ):
        request_id = (
            str(uuid.uuid4()) if request_id is None else str(request_id)
        )
        if not request_id:
            raise ValueError("request_id must be non-empty")
        if expected_state_revision is not None and (
            not isinstance(expected_state_revision, int)
            or isinstance(expected_state_revision, bool)
            or expected_state_revision < 0
        ):
            raise ValueError("expected_state_revision must be >= 0 int")
        now_s = float(self.clock())
        payload = {
            "schema_version": OPS_SCHEMA_VERSION,
            "token": self.token,
            "request_id": request_id,
            "sequence": self._next_sequence,
            "action": str(action),
            "params": dict(params or {}),
            "stamp_s": now_s,
        }
        if expected_state_revision is not None:
            payload["expected_state_revision"] = expected_state_revision
        if phase is not None:
            payload["phase"] = str(phase)
        self._next_sequence += 1
        record = (
            json.dumps(
                payload,
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
        pending = _PendingRequest(record=record, created_s=now_s)
        self._pending[request_id] = pending
        self._connect()
        if pending.last_sent_s is None and self._send_record(record):
            pending.last_sent_s = now_s
        return request_id

    def _receive(self):
        responses = []
        if self.sock is None or not hasattr(self.sock, "recv"):
            return responses
        while True:
            try:
                chunk = self.sock.recv(4096)
            except BlockingIOError:
                break
            except OSError:
                self._disconnect()
                break
            if not chunk:
                self._disconnect()
                break
            self._recv_buffer.extend(chunk)
            while b"\n" in self._recv_buffer:
                line, _, rest = self._recv_buffer.partition(b"\n")
                self._recv_buffer[:] = rest
                try:
                    response = json.loads(line)
                except (TypeError, ValueError):
                    continue
                if not isinstance(response, dict):
                    continue
                if response.get("push") == "ops_state":
                    self._latest_ops_state = dict(response)
                request_id = response.get("request_id")
                if request_id in self._pending and "status" in response:
                    self._pending.pop(request_id, None)
                responses.append(response)
        return responses

    def pump(self):
        """Receive replies and retry unacknowledged requests until 2 seconds."""
        if self._closed:
            return []
        self._connect()
        responses = self._receive()
        now_s = float(self.clock())
        for request_id, pending in list(self._pending.items()):
            if now_s - pending.created_s >= REQUEST_DEADLINE_S:
                self._pending.pop(request_id, None)
                continue
            if self.sock is None:
                continue
            if pending.last_sent_s is None or (
                now_s - pending.last_sent_s >= RETRANSMIT_INTERVAL_S
            ):
                if self._send_record(pending.record):
                    pending.last_sent_s = now_s
        return responses

    def latest_ops_state(self):
        if self._latest_ops_state is None:
            return None
        return dict(self._latest_ops_state)

    def close(self):
        self._closed = True
        self._pending.clear()
        self._disconnect()
