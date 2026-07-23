"""Host-safe TCP listener regressions for ``teleop_command_node``."""

import ast
import errno
from pathlib import Path
import socket
import threading
import time
import uuid

import pytest


SOURCE_PATH = (
    Path(__file__).resolve().parents[1]
    / "powertrain_ros"
    / "teleop_command_node.py"
)
SOURCE = SOURCE_PATH.read_text(encoding="utf-8")


def _extract_methods(*names):
    tree = ast.parse(SOURCE)
    node_class = next(
        node
        for node in tree.body
        if isinstance(node, ast.ClassDef) and node.name == "TeleopCommandNode"
    )
    methods = [
        node
        for node in node_class.body
        if isinstance(node, ast.FunctionDef) and node.name in names
    ]
    module = ast.Module(body=methods, type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {
        "CLIENT_IDLE_TIMEOUT_S": 5.0,
        "socket": socket,
        "time": time,
        "uuid": uuid,
    }
    exec(compile(module, str(SOURCE_PATH), "exec"), namespace)
    return {name: namespace[name] for name in names}


_METHODS = _extract_methods("_serve", "_serve_client")


class _StopEvent:
    def __init__(self):
        self._event = threading.Event()
        self.wait_timeouts = []

    def is_set(self):
        return self._event.is_set()

    def set(self):
        self._event.set()

    def wait(self, timeout):
        self.wait_timeouts.append(timeout)
        return self._event.wait(timeout)


class _ServerSocket:
    def __init__(self, accept_steps):
        self._accept_steps = iter(accept_steps)
        self.closed = False
        self.accept_count = 0

    def setsockopt(self, *_args):
        pass

    def bind(self, _address):
        pass

    def listen(self, _backlog):
        pass

    def settimeout(self, _timeout):
        pass

    def accept(self):
        self.accept_count += 1
        step = next(self._accept_steps)
        if isinstance(step, BaseException):
            raise step
        return step, ("127.0.0.1", 12345)

    def close(self):
        self.closed = True


class _AcceptedConnection:
    def __init__(self, *, setup_error=None, on_recv=None):
        self.setup_error = setup_error
        self.on_recv = on_recv
        self.closed = False
        self.options = []

    def settimeout(self, timeout):
        if self.setup_error is not None:
            raise self.setup_error
        self.options.append(("timeout", timeout))

    def setsockopt(self, *option):
        self.options.append(option)

    def recv(self, _size):
        if self.on_recv is not None:
            self.on_recv()
        return b""

    def sendall(self, _data):
        pass

    def close(self):
        self.closed = True


class _AcceptHarness:
    _serve = _METHODS["_serve"]

    def __init__(self):
        self._port = 0
        self._server_socket = None
        self._stop_event = _StopEvent()
        self.served = []
        self.logged = []
        self.violations = []

    def _serve_client(self, connection):
        self.served.append(connection)
        self._stop_event.set()

    def _log_violation_throttled(self, message, now_s):
        self.logged.append((message, now_s))
        return True

    def _queue_violation(self, reason):
        self.violations.append(reason)


class _Decoder:
    def __init__(self):
        self.starts = 0
        self.ends = 0

    def start_connection(self):
        self.starts += 1

    def end_connection(self):
        self.ends += 1
        return []


class _ClientSetupHarness:
    _serve = _METHODS["_serve"]
    _serve_client = _METHODS["_serve_client"]

    def __init__(self):
        self._port = 0
        self._server_socket = None
        self._stop_event = _StopEvent()
        self._decoder = _Decoder()
        self.lifecycle = []
        self.results = []
        self.violations = []

    def _queue_lifecycle_event(self, event, session_id):
        self.lifecycle.append((event, session_id))

    def _queue_decoder_results(self, results):
        self.results.extend(results)

    @staticmethod
    def _current_status():
        return b"S DRIVE +0.000 +0.000\n"

    def _queue_violation(self, reason):
        self.violations.append(reason)


def test_transient_accept_error_keeps_listener_alive_for_later_client(
    monkeypatch,
):
    first_failure_seen = threading.Event()
    allow_later_client = threading.Event()
    later_connection = _AcceptedConnection()

    class _SequencedServer(_ServerSocket):
        def accept(self):
            self.accept_count += 1
            if self.accept_count == 1:
                first_failure_seen.set()
                raise ConnectionAbortedError(
                    errno.ECONNABORTED,
                    "handshake aborted",
                )
            assert allow_later_client.wait(1.0)
            return later_connection, ("127.0.0.1", 12345)

    server = _SequencedServer([])
    monkeypatch.setattr(socket, "socket", lambda *_args: server)
    harness = _AcceptHarness()
    thread = threading.Thread(target=harness._serve)
    thread.start()
    try:
        assert first_failure_seen.wait(1.0)
        thread.join(timeout=0.05)
        assert thread.is_alive(), (
            "listener exited after transient accept error"
        )

        allow_later_client.set()
        thread.join(timeout=1.0)
        assert not thread.is_alive()
        assert harness.served == [later_connection]
        assert harness.violations == []
        assert len(harness.logged) == 1
        assert "accept failed" in harness.logged[0][0]
        assert harness._stop_event.wait_timeouts == [pytest.approx(0.2)]
    finally:
        allow_later_client.set()
        harness._stop_event.set()
        thread.join(timeout=1.0)


def test_connection_setup_oserror_closes_only_that_client(monkeypatch):
    harness = _ClientSetupHarness()
    setup_failure = _AcceptedConnection(
        setup_error=OSError(errno.ENOBUFS, "socket setup failed")
    )
    later_connection = _AcceptedConnection(on_recv=harness._stop_event.set)
    server = _ServerSocket([setup_failure, later_connection])
    monkeypatch.setattr(socket, "socket", lambda *_args: server)

    harness._serve()

    assert server.accept_count == 2
    assert setup_failure.closed is True
    assert later_connection.closed is True
    assert harness._decoder.starts == 1
    assert harness._decoder.ends == 1
    assert [event for event, _session_id in harness.lifecycle] == [
        "connect",
        "disconnect",
    ]
    assert harness.violations == []


def test_bind_failure_remains_a_server_fatal_contract_violation(monkeypatch):
    class _BindFailureServer(_ServerSocket):
        def bind(self, _address):
            raise OSError(errno.EADDRINUSE, "address already in use")

    server = _BindFailureServer([])
    monkeypatch.setattr(socket, "socket", lambda *_args: server)
    harness = _AcceptHarness()

    harness._serve()

    assert server.closed is True
    assert len(harness.violations) == 1
    assert harness.violations[0].startswith(
        "CONTRACT_VIOLATION: TCP server failed:"
    )
