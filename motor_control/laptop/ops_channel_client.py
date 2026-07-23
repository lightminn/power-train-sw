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
MAX_RECV_BUFFER_BYTES = 4 * 1024
SERVICE_ORDER_ABANDON_S = 10.0
# 서버가 서비스 호출 결과를 포기한 뒤 마지막 status_query 왕복 여유를 둔다.
AWAIT_FINAL_ABANDON_S = SERVICE_ORDER_ABANDON_S + REQUEST_DEADLINE_S
STATUS_PENDING = "PENDING"
STATUS_FINAL_SUCCESS = "FINAL_SUCCESS"
STATUS_FINAL_REJECTED = "FINAL_REJECTED"
STATUS_OUTCOME_UNKNOWN = "OUTCOME_UNKNOWN"
_TERMINAL_STATUSES = frozenset({
    STATUS_FINAL_SUCCESS,
    STATUS_FINAL_REJECTED,
    STATUS_OUTCOME_UNKNOWN,
})
_RESPONSE_TOMBSTONE_S = AWAIT_FINAL_ABANDON_S
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


@dataclass
class _AwaitingFinal:
    started_s: float
    last_query_s: float
    state_revision: int


@dataclass
class _StatusQuery:
    target_request_id: str
    sent_s: float


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
        self._awaiting_final = {}
        self._status_queries = {}
        self._response_tombstones = {}
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
        now_s = float(self.clock())
        self._expire_pending_requests(now_s)
        for pending in self._pending.values():
            if not self._send_record(pending.record):
                return False
            pending.last_sent_s = now_s
        return True

    def _new_request_record(
        self,
        request_id,
        action,
        params,
        now_s,
        *,
        phase=None,
        expected_state_revision=None,
    ):
        payload = {
            "schema_version": OPS_SCHEMA_VERSION,
            "token": self.token,
            "request_id": request_id,
            "sequence": self._next_sequence,
            "action": str(action),
            "params": dict(params),
            "stamp_s": now_s,
        }
        if expected_state_revision is not None:
            payload["expected_state_revision"] = expected_state_revision
        if phase is not None:
            payload["phase"] = str(phase)
        self._next_sequence += 1
        return (
            json.dumps(
                payload,
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")

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
        record = self._new_request_record(
            request_id,
            action,
            params or {},
            now_s,
            phase=phase,
            expected_state_revision=expected_state_revision,
        )
        pending = _PendingRequest(record=record, created_s=now_s)
        self._pending[request_id] = pending
        self._connect()
        if (
            self._pending.get(request_id) is pending
            and pending.last_sent_s is None
            and self._send_record(record)
        ):
            pending.last_sent_s = now_s
        return request_id

    @staticmethod
    def _response_revision(response, fallback=0):
        revision = response.get("state_revision", fallback)
        if isinstance(revision, int) and not isinstance(revision, bool):
            return revision
        return fallback

    def _start_awaiting_final(self, request_id, response):
        now_s = float(self.clock())
        awaiting = self._awaiting_final.get(request_id)
        if awaiting is None:
            self._awaiting_final[request_id] = _AwaitingFinal(
                started_s=now_s,
                last_query_s=now_s,
                state_revision=self._response_revision(response),
            )
            return
        awaiting.state_revision = self._response_revision(
            response, awaiting.state_revision
        )

    def _expire_pending_requests(self, now_s):
        for request_id, pending in list(self._pending.items()):
            if now_s - pending.created_s < REQUEST_DEADLINE_S:
                continue
            self._pending.pop(request_id, None)
            # PENDING 자체가 유실됐을 수 있으므로 오래된 원 요청을 다시
            # 보내지 않고 fresh status_query 복구 상태로 전환한다.
            self._start_awaiting_final(request_id, {})

    def _resolve_awaiting_final(self, request_id):
        self._awaiting_final.pop(request_id, None)
        self._retire_response_id(request_id)

    def _retire_response_id(self, request_id, now_s=None):
        if request_id is None:
            return
        if now_s is None:
            now_s = float(self.clock())
        self._response_tombstones[request_id] = (
            now_s + _RESPONSE_TOMBSTONE_S
        )

    def _response_is_retired(self, request_id):
        expires_s = self._response_tombstones.get(request_id)
        if expires_s is None:
            return False
        if float(self.clock()) < expires_s:
            return True
        self._response_tombstones.pop(request_id, None)
        return False

    def _correlate_response(self, response):
        request_id = response.get("request_id")
        status = response.get("status")
        if status is None:
            return response
        if self._response_is_retired(request_id):
            return None

        query = self._status_queries.pop(request_id, None)
        if query is not None:
            self._retire_response_id(request_id)
            target_id = query.target_request_id
            awaiting = self._awaiting_final.get(target_id)
            if awaiting is None:
                return None
            if response.get("queried_request_id") != target_id:
                return None
            correlated = dict(response)
            correlated.pop("queried_request_id", None)
            correlated["request_id"] = target_id
            if status == STATUS_PENDING:
                awaiting.state_revision = self._response_revision(
                    response, awaiting.state_revision
                )
            elif status in _TERMINAL_STATUSES:
                self._resolve_awaiting_final(target_id)
            return correlated

        if request_id in self._pending:
            if status == STATUS_PENDING:
                self._pending.pop(request_id, None)
                self._start_awaiting_final(request_id, response)
            elif status in _TERMINAL_STATUSES:
                self._pending.pop(request_id, None)
                self._retire_response_id(request_id)
            return response

        awaiting = self._awaiting_final.get(request_id)
        if awaiting is not None:
            if status == STATUS_PENDING:
                awaiting.state_revision = self._response_revision(
                    response, awaiting.state_revision
                )
            elif status in _TERMINAL_STATUSES:
                self._resolve_awaiting_final(request_id)
        return response

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
            buffer_overflow = (
                len(self._recv_buffer) > MAX_RECV_BUFFER_BYTES
            )
            if buffer_overflow and b"\n" not in self._recv_buffer:
                self._recv_buffer.clear()
                continue
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
                response = self._correlate_response(response)
                if response is not None:
                    responses.append(response)
        return responses

    def _send_status_query(
        self, target_request_id, awaiting, now_s
    ):
        query_id = str(uuid.uuid4())
        record = self._new_request_record(
            query_id,
            "status_query",
            {"request_id": target_request_id},
            now_s,
        )
        if not self._send_record(record):
            return
        awaiting.last_query_s = now_s
        self._status_queries[query_id] = _StatusQuery(
            target_request_id=target_request_id,
            sent_s=now_s,
        )

    def _prune_status_queries(self, now_s):
        for query_id, query in list(self._status_queries.items()):
            if now_s - query.sent_s >= AWAIT_FINAL_ABANDON_S:
                self._status_queries.pop(query_id, None)
                self._retire_response_id(query_id, now_s)

    def _prune_response_tombstones(self, now_s):
        self._response_tombstones = {
            request_id: expires_s
            for request_id, expires_s in self._response_tombstones.items()
            if now_s < expires_s
        }

    @staticmethod
    def _local_outcome_unknown(request_id, awaiting):
        return {
            "request_id": request_id,
            "status": STATUS_OUTCOME_UNKNOWN,
            "state_revision": awaiting.state_revision,
            "detail": (
                "client recovery timed out after %.1fs"
                % AWAIT_FINAL_ABANDON_S
            ),
        }

    def pump(self):
        """Receive replies, retransmit requests, and recover final outcomes."""
        if self._closed:
            return []
        now_s = float(self.clock())
        self._expire_pending_requests(now_s)
        self._connect()
        now_s = float(self.clock())
        self._prune_status_queries(now_s)
        self._prune_response_tombstones(now_s)
        responses = self._receive()
        now_s = float(self.clock())
        self._expire_pending_requests(now_s)
        for request_id, pending in list(self._pending.items()):
            if self.sock is None:
                continue
            if pending.last_sent_s is None or (
                now_s - pending.last_sent_s >= RETRANSMIT_INTERVAL_S
            ):
                if self._send_record(pending.record):
                    pending.last_sent_s = now_s
        for request_id, awaiting in list(self._awaiting_final.items()):
            if now_s - awaiting.started_s >= AWAIT_FINAL_ABANDON_S:
                self._resolve_awaiting_final(request_id)
                responses.append(
                    self._local_outcome_unknown(request_id, awaiting)
                )
                continue
            if (
                now_s - awaiting.started_s < SERVICE_ORDER_ABANDON_S
                and self.sock is not None
                and
                now_s - awaiting.last_query_s >= RETRANSMIT_INTERVAL_S
            ):
                self._send_status_query(request_id, awaiting, now_s)
        return responses

    def latest_ops_state(self):
        if self._latest_ops_state is None:
            return None
        return dict(self._latest_ops_state)

    def close(self):
        self._closed = True
        self._pending.clear()
        self._awaiting_final.clear()
        self._status_queries.clear()
        self._response_tombstones.clear()
        self._disconnect()
