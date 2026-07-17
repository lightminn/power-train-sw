"""TCP 서버 견고성 — 클라이언트 RST가 accept 루프를 죽이면 안 된다.

2026-07-17 벤치 실증: 첫 원격 클라이언트를 강제 종료하자 ConnectionResetError가
``_serve_client``의 recv에서 ``_serve``까지 전파돼 서버 스레드가 죽었고, 이후
모든 재접속이 불가능했다(노드 재시작 전까지 원격 불능).
"""
from collections import deque
import socket
import struct
import threading
import time
from types import SimpleNamespace

import pytest
import rclpy

from powertrain_ros import teleop_command_node
from powertrain_ros.remote_input import ParseResult
from powertrain_ros.remote_input_gateway import MOTION_HOLD, RemoteInputGateway
from powertrain_ros.teleop_command_node import TeleopCommandNode


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def _isolated_port(monkeypatch):
    """로봇 위에서 스위트를 돌리면 라이브 powertrain_control이 :9000을 점유해
    테스트 노드 bind가 EADDRINUSE로 죽고 테스트가 **라이브 서버**에 붙는다
    (§9-0 DDS 도메인 누수의 TCP판). 테스트마다 에페메랄 포트로 격리한다."""
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    monkeypatch.setattr(teleop_command_node, "DEFAULT_PORT", port)


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
        self.connections = []
        self.submitted = []

    def contract_violation(self, payload):
        self.violations.append(payload)

    def begin_connection(self):
        self.connections.append("connect")

    def end_connection(self):
        self.connections.append("disconnect")

    def submit(self, payload):
        self.submitted.append(payload)
        return True


class _LoggerRecorder:
    def __init__(self):
        self.errors = []

    def error(self, message):
        self.errors.append(message)


class _DrainHarness:
    def __init__(self):
        self._events_lock = threading.Lock()
        self._motion_frame = None
        self._motion_frames_dropped = 0
        self._lifecycle_events = deque(maxlen=8)
        self._violation_events = deque(maxlen=64)
        self._gateway = _GatewayRecorder()
        self._logger = _LoggerRecorder()
        self._last_violation_log_s = None
        self._violation_events_suppressed = 0
        self._violation_events_reported = 0
        self._violation_window_start_s = 0.0
        self._violation_events_in_window = 0
        self._violation_rate_lock = threading.Lock()
        self._estop_lock = threading.Lock()
        self._estop_event = None

    def get_logger(self):
        return self._logger

    def _log_violation_throttled(self, message, now_s):
        return TeleopCommandNode._log_violation_throttled(self, message, now_s)

    def _queue_motion_frame(self, frame):
        return TeleopCommandNode._queue_motion_frame(self, frame)

    def _queue_lifecycle_event(self, event, session_id):
        return TeleopCommandNode._queue_lifecycle_event(
            self,
            event,
            session_id,
        )

    def _queue_violation(self, reason, count=1):
        return TeleopCommandNode._queue_violation(self, reason, count=count)

    def _begin_estop_event(self, now_s):
        return TeleopCommandNode._begin_estop_event(self, now_s)

    def _queue_decoder_results(self, results, now_s=None):
        return TeleopCommandNode._queue_decoder_results(
            self,
            results,
            now_s=now_s,
        )


def _frame(sequence, *, estop_edge=False):
    return SimpleNamespace(sequence=sequence, estop_edge=estop_edge)


def test_motion_burst_consumes_latest_frame_and_counts_drops():
    harness = _DrainHarness()
    harness._queue_decoder_results(
        [ParseResult(frame=_frame(index)) for index in range(20)],
        now_s=1.0,
    )

    processed = TeleopCommandNode._drain_events(harness, now_s=1.0)

    assert processed == 1
    assert [frame.sequence for frame in harness._gateway.submitted] == [19]
    assert harness._motion_frames_dropped == 19


def test_estop_edge_is_durably_latched_before_event_drain():
    harness = _DrainHarness()

    harness._queue_decoder_results(
        [ParseResult(frame=_frame(1, estop_edge=True))],
        now_s=10.0,
    )

    assert harness._estop_event is not None
    assert harness._estop_event["stamp_s"] == pytest.approx(10.0)
    assert harness._gateway.submitted == []


def test_disconnect_survives_motion_burst_as_session_latest_event():
    harness = _DrainHarness()
    harness._queue_lifecycle_event("connect", "session-a")
    harness._queue_decoder_results(
        [ParseResult(frame=_frame(index)) for index in range(100)],
        now_s=1.0,
    )
    harness._queue_lifecycle_event("disconnect", "session-a")

    assert list(harness._lifecycle_events) == [
        ("disconnect", "session-a")
    ]
    TeleopCommandNode._drain_events(harness, now_s=1.0)

    assert harness._gateway.connections == ["disconnect"]
    assert harness._motion_frames_dropped == 99


def test_decoder_violations_are_coalesced_by_reason_with_counts():
    harness = _DrainHarness()
    harness._queue_decoder_results(
        [ParseResult(reason="bad frame") for _ in range(20)],
        now_s=1.0,
    )

    assert list(harness._violation_events) == [("bad frame", 20)]
    TeleopCommandNode._drain_events(harness, now_s=1.0)

    assert harness._gateway.violations == ["bad frame (20 occurrences)"]


def test_lifecycle_overflow_forces_gateway_motion_hold():
    harness = _DrainHarness()
    harness._gateway = RemoteInputGateway()
    for index in range(9):
        harness._queue_lifecycle_event("connect", f"session-{index}")

    TeleopCommandNode._drain_events(harness, now_s=1.0)

    assert harness._gateway.state == MOTION_HOLD
    assert "event overflow" in harness._gateway._last_reason


def test_tick_respects_explicit_event_limit_and_throttles_violation_logs():
    harness = _DrainHarness()
    for index in range(5):
        harness._queue_violation(f"violation {index}")

    processed = TeleopCommandNode._drain_events(
        harness,
        max_events=3,
        now_s=10.0,
    )

    assert processed == 3
    assert len(harness._gateway.violations) == 3
    assert len(harness._logger.errors) == 1
    assert list(harness._violation_events) == [
        ("violation 3", 1),
        ("violation 4", 1),
    ]


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
    harness = _DrainHarness()
    results = [ParseResult(reason=f"bad {index}") for index in range(200)]

    harness._queue_decoder_results(results, now_s=1.0)

    assert len(harness._violation_events) == 50
    assert harness._violation_events_suppressed == 150
