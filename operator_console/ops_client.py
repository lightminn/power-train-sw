"""Threaded bridge from the GTK console to the laptop ops channel."""
from collections import deque
from dataclasses import dataclass
import os
import sys
import threading
import time
import uuid


_OPERATOR_CONSOLE_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_MOTOR_CONTROL_PATH = os.path.abspath(
    os.path.join(_OPERATOR_CONSOLE_DIR, os.pardir, "motor_control")
)
_MOTOR_CONTROL_PATH = os.path.abspath(os.path.expanduser(
    os.environ.get("MOTOR_CONTROL_PATH", _DEFAULT_MOTOR_CONTROL_PATH)
))
if _MOTOR_CONTROL_PATH not in sys.path:
    sys.path.insert(0, _MOTOR_CONTROL_PATH)

from laptop import ops_channel_client  # noqa: E402


SEND_QUEUE_MAXLEN = 16
RECONNECT_BACKOFF_S = 1.0
PUMP_INTERVAL_S = 0.02


def _glib_idle_add(callback):
    from gi.repository import GLib

    return GLib.idle_add(callback)


@dataclass(frozen=True)
class _QueuedSubmit:
    request_id: str
    action: str
    params: dict
    expected_state_revision: int | None


class ConsoleOpsClient:
    """Own the ops socket on a worker and hand results to the UI scheduler."""

    def __init__(
        self,
        host,
        port,
        token,
        *,
        submit_sink,
        state_sink,
        schedule=_glib_idle_add,
        client_factory=ops_channel_client.OpsChannelClient,
    ):
        self._host = str(host)
        self._port = int(port)
        self._token = str(token)
        self._submit_sink = submit_sink
        self._state_sink = state_sink
        self._schedule = schedule
        self._client_factory = client_factory

        self._send_queue = deque(maxlen=SEND_QUEUE_MAXLEN)
        self._queue_lock = threading.Lock()
        self._dropped_send_count = 0
        self._inflight = {}
        self._latest = None
        self._state_lock = threading.Lock()
        self._state_generation = None
        self._client = None
        self._client_lock = threading.Lock()
        self._connection_generation = 0
        self._next_connect_s = 0.0
        self._stopping = threading.Event()

        self._thread = threading.Thread(
            target=self._run,
            name="console-ops-client",
            daemon=True,
        )
        self._thread.start()

    @property
    def dropped_send_count(self):
        with self._queue_lock:
            return self._dropped_send_count

    def submit(self, action, params=None, expected_state_revision=None):
        """Queue one command without blocking the GTK thread."""
        if self._stopping.is_set():
            raise RuntimeError("console ops client is closed")
        with self._client_lock:
            if self._client is None:
                raise RuntimeError("ops client is not connected")
            generation = self._connection_generation
            with self._state_lock:
                if (
                    self._latest is None
                    or self._state_generation != generation
                ):
                    raise RuntimeError("ops state unavailable for current connection")
            request_id = str(uuid.uuid4())
            queued = _QueuedSubmit(
                request_id=request_id,
                action=str(action),
                params=dict(params or {}),
                expected_state_revision=expected_state_revision,
            )
            with self._queue_lock:
                if len(self._send_queue) == self._send_queue.maxlen:
                    self._dropped_send_count += 1
                    raise RuntimeError("ops send queue full")
                self._send_queue.append(queued)
        return request_id

    def latest_state(self):
        with self._state_lock:
            return None if self._latest is None else dict(self._latest)

    def _disconnect(self, client, *, uncertain=(), unsent=()):
        disconnected = False
        with self._client_lock:
            if self._client is client:
                self._client = None
                disconnected = True
                with self._state_lock:
                    self._latest = None
                    self._state_generation = None
        if disconnected:
            queued = list(unsent) + self._take_pending()
            inflight = self._take_inflight() + list(uncertain)
            self._report_requests(
                queued,
                status="FINAL_REJECTED",
                detail="ops connection lost before command was sent",
            )
            self._report_requests(
                inflight,
                status="OUTCOME_UNKNOWN",
                detail="ops connection lost after command was sent",
            )
            self._schedule_sink(self._state_sink, None)
        try:
            client.close()
        except Exception:
            pass
        self._next_connect_s = time.monotonic() + RECONNECT_BACKOFF_S

    def _connected_client(self):
        with self._client_lock:
            client = self._client
        if client is not None or self._stopping.is_set():
            return client
        if time.monotonic() < self._next_connect_s:
            return None
        try:
            client = self._client_factory(
                self._host,
                self._port,
                self._token,
            )
        except Exception:
            self._next_connect_s = time.monotonic() + RECONNECT_BACKOFF_S
            return None
        if self._stopping.is_set():
            try:
                client.close()
            except Exception:
                pass
            return None
        with self._client_lock:
            self._client = client
            self._connection_generation += 1
            with self._state_lock:
                self._latest = None
                self._state_generation = None
        return client

    def _take_pending(self):
        with self._queue_lock:
            pending = list(self._send_queue)
            self._send_queue.clear()
        return pending

    def _take_inflight(self):
        with self._queue_lock:
            inflight = list(self._inflight.values())
            self._inflight.clear()
        return inflight

    def _report_requests(self, queued_requests, *, status, detail):
        for queued in queued_requests:
            self._schedule_sink(self._submit_sink, {
                "request_id": queued.request_id,
                "status": status,
                "detail": detail,
            })

    def _schedule_sink(self, sink, payload):
        snapshot = None if payload is None else dict(payload)

        def callback():
            sink(snapshot)
            return False

        self._schedule(callback)

    def run_once(self):
        """Run one deterministic connect/send/pump iteration."""
        if self._stopping.is_set():
            return
        client = self._connected_client()
        if client is None:
            return

        pending = self._take_pending()
        for index, queued in enumerate(pending):
            try:
                client.submit(
                    queued.action,
                    params=queued.params,
                    request_id=queued.request_id,
                    expected_state_revision=queued.expected_state_revision,
                )
                with self._queue_lock:
                    self._inflight[queued.request_id] = queued
            except Exception:
                self._disconnect(
                    client,
                    uncertain=(queued,),
                    unsent=pending[index + 1:],
                )
                return

        try:
            responses = client.pump()
        except Exception:
            self._disconnect(client)
            return
        for response in responses:
            if not isinstance(response, dict):
                continue
            if response.get("push") == "ops_state":
                snapshot = dict(response)
                with self._client_lock:
                    if self._client is not client:
                        continue
                    generation = self._connection_generation
                    with self._state_lock:
                        self._latest = snapshot
                        self._state_generation = generation
                self._schedule_sink(self._state_sink, snapshot)
            else:
                status = str(response.get("status", ""))
                if status.startswith("FINAL_") or status == "OUTCOME_UNKNOWN":
                    with self._queue_lock:
                        self._inflight.pop(str(response.get("request_id")), None)
                self._schedule_sink(self._submit_sink, response)

    def _run(self):
        while not self._stopping.is_set():
            self.run_once()
            self._stopping.wait(PUMP_INTERVAL_S)

    def close(self):
        self._stopping.set()
        with self._client_lock:
            client = self._client
            self._client = None
        if client is not None:
            try:
                client.close()
            except Exception:
                pass
        self._thread.join(timeout=RECONNECT_BACKOFF_S + 1.0)
