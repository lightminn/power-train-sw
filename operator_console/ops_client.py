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
        self._latest = None
        self._state_lock = threading.Lock()
        self._client = None
        self._client_lock = threading.Lock()
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
            self._send_queue.append(queued)
        return request_id

    def latest_state(self):
        with self._state_lock:
            return None if self._latest is None else dict(self._latest)

    def _disconnect(self, client):
        with self._client_lock:
            if self._client is client:
                self._client = None
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
        return client

    def _take_pending(self):
        with self._queue_lock:
            pending = list(self._send_queue)
            self._send_queue.clear()
        return pending

    def _restore_pending(self, pending):
        with self._queue_lock:
            combined = list(pending) + list(self._send_queue)
            overflow = max(0, len(combined) - SEND_QUEUE_MAXLEN)
            if overflow:
                self._dropped_send_count += overflow
                combined = combined[overflow:]
            self._send_queue.clear()
            self._send_queue.extend(combined)

    def _schedule_sink(self, sink, payload):
        snapshot = dict(payload)

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
            except Exception:
                self._restore_pending(pending[index:])
                self._disconnect(client)
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
                with self._state_lock:
                    self._latest = snapshot
                self._schedule_sink(self._state_sink, snapshot)
            else:
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
