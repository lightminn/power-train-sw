"""TCP 서버 견고성 — 클라이언트 RST가 accept 루프를 죽이면 안 된다.

2026-07-17 벤치 실증: 첫 원격 클라이언트를 강제 종료하자 ConnectionResetError가
``_serve_client``의 recv에서 ``_serve``까지 전파돼 서버 스레드가 죽었고, 이후
모든 재접속이 불가능했다(노드 재시작 전까지 원격 불능).
"""
import queue
import socket
import struct
import time

import pytest
import rclpy

from powertrain_ros import teleop_command_node
from powertrain_ros.remote_input import ParseResult
from powertrain_ros.teleop_command_node import TeleopCommandNode


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _connect(port, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=1.0)
        except OSError:
            time.sleep(0.1)
    raise AssertionError("could not connect to teleop TCP server")


def _read_status(connection, node, timeout=3.0):
    connection.settimeout(0.2)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            data = connection.recv(256)
        except socket.timeout:
            continue
        if data:
            return data
    raise AssertionError("no status line from teleop TCP server")


def test_client_rst_does_not_kill_the_accept_loop():
    node = TeleopCommandNode()
    port = node._port
    try:
        first = _connect(port)
        assert _read_status(first, node).startswith(b"S ")
        # SO_LINGER 0 → close가 FIN이 아니라 RST를 보낸다(강제 종료 재현).
        first.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_LINGER,
            struct.pack("ii", 1, 0),
        )
        first.close()
        time.sleep(0.3)

        second = _connect(port)
        assert _read_status(second, node).startswith(b"S ")
        second.close()
    finally:
        node.close()
        node.destroy_node()


def test_silent_client_is_closed_and_second_client_is_accepted(monkeypatch):
    monkeypatch.setattr(teleop_command_node, "CLIENT_IDLE_TIMEOUT_S", 0.30)
    node = TeleopCommandNode()
    port = node._port
    try:
        first = _connect(port)
        assert _read_status(first, node).startswith(b"S ")

        second = _connect(port)
        assert _read_status(second, node, timeout=2.0).startswith(b"S ")
        second.close()
        first.close()
    finally:
        node.close()
        node.destroy_node()


class _GatewayRecorder:
    def __init__(self):
        self.violations = []

    def contract_violation(self, payload):
        self.violations.append(payload)

    def begin_connection(self):
        pass

    def end_connection(self):
        pass

    def submit(self, _payload):
        pass


class _LoggerRecorder:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


class _DrainHarness:
    def __init__(self):
        self._events = queue.SimpleQueue()
        self._gateway = _GatewayRecorder()
        self._logger = _LoggerRecorder()
        self._last_violation_log_s = None
        self._violation_events_suppressed = 0
        self._violation_events_reported = 0

    def get_logger(self):
        return self._logger

    def _log_violation_throttled(self, message, now_s):
        return TeleopCommandNode._log_violation_throttled(self, message, now_s)


def test_tick_drains_at_most_256_events_and_throttles_violation_logs():
    harness = _DrainHarness()
    for index in range(400):
        harness._events.put(("violation", f"violation {index}"))

    processed = TeleopCommandNode._drain_events(harness, now_s=10.0)

    assert processed == 256
    assert len(harness._gateway.violations) == 256
    assert len(harness._logger.errors) == 1
    assert harness._events.get_nowait() == ("violation", "violation 256")


def test_suppressed_violation_counter_is_monotonic_after_summary_log():
    harness = _DrainHarness()
    harness._violation_events_suppressed = 12

    assert TeleopCommandNode._drain_events(harness, now_s=10.0) == 0

    assert harness._violation_events_suppressed == 12
    assert harness._violation_events_reported == 12
    assert harness._logger.errors == [
        "CONTRACT_VIOLATION: 12 decoder violations suppressed"
    ]


def test_decoder_violation_queue_is_limited_to_fifty_per_second():
    node = TeleopCommandNode()
    try:
        node._events = queue.SimpleQueue()
        results = [ParseResult(reason=f"bad {index}") for index in range(200)]

        node._queue_decoder_results(results, now_s=1.0)

        queued = []
        while True:
            try:
                queued.append(node._events.get_nowait())
            except queue.Empty:
                break
        assert len(queued) == 50
        assert all(event == "violation" for event, _payload in queued)
        assert node._violation_events_suppressed == 150
    finally:
        node.close()
        node.destroy_node()
