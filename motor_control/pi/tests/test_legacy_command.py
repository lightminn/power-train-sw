import importlib
from pathlib import Path
import queue
import socket
import threading
import time

import pytest


class FakeBlockingConnection:
    """Queue-backed recv double; avoids sandbox-restricted socket writes."""

    def __init__(self):
        self.timeout = None
        self._incoming = queue.Queue()

    def settimeout(self, value):
        self.timeout = value

    def recv(self, _size):
        try:
            return self._incoming.get(timeout=self.timeout)
        except queue.Empty:
            raise socket.timeout from None

    def feed(self, data):
        self._incoming.put(data)


def _legacy_command_module():
    try:
        return importlib.import_module("pi.legacy_command")
    except ModuleNotFoundError:
        return None


@pytest.mark.parametrize("text", ("nan", "inf", "-inf", "NaN", "+INF"))
def test_nonfinite_command_lines_are_discarded(text):
    legacy = _legacy_command_module()
    assert legacy is not None, "shared legacy command safety boundary is missing"
    assert legacy.parse_finite_command(text, max_abs=5.0) is None


def test_valid_command_is_clamped_without_changing_legacy_range_semantics():
    legacy = _legacy_command_module()
    assert legacy is not None, "shared legacy command safety boundary is missing"
    assert legacy.parse_finite_command("7.5", max_abs=5.0) == 5.0
    assert legacy.parse_finite_command("-7.5", max_abs=5.0) == -5.0


def test_command_blackhole_triggers_hold_before_freshness_deadline_margin():
    legacy = _legacy_command_module()
    assert legacy is not None, "shared legacy command safety boundary is missing"
    connection = FakeBlockingConnection()
    commands = []
    held = threading.Event()
    command_seen = threading.Event()

    def apply(value):
        commands.append(value)
        command_seen.set()

    thread = threading.Thread(
        target=legacy.serve_command_connection,
        kwargs={
            "connection": connection,
            "apply_command": apply,
            "hold_command": held.set,
            "max_abs": 5.0,
            "freshness_timeout_s": 0.05,
            "idle_timeout_s": 1.0,
        },
    )
    thread.start()
    started = time.monotonic()
    connection.feed(b"2.0\n")
    assert command_seen.wait(0.2)
    assert held.wait(0.15)
    assert time.monotonic() - started < 0.2
    assert commands == [2.0]
    connection.feed(b"")
    thread.join(timeout=1.0)
    assert not thread.is_alive()


def test_invalid_lines_never_reach_motor_callback_or_refresh_watchdog():
    legacy = _legacy_command_module()
    assert legacy is not None, "shared legacy command safety boundary is missing"
    connection = FakeBlockingConnection()
    commands = []
    held = threading.Event()
    thread = threading.Thread(
        target=legacy.serve_command_connection,
        kwargs={
            "connection": connection,
            "apply_command": commands.append,
            "hold_command": held.set,
            "max_abs": 5.0,
            "freshness_timeout_s": 0.05,
            "idle_timeout_s": 1.0,
        },
    )
    thread.start()
    connection.feed(b"nan\ninf\n-inf\n")
    time.sleep(0.08)
    assert commands == []
    assert not held.is_set()
    connection.feed(b"")
    thread.join(timeout=1.0)


def test_connection_uses_one_second_recv_timeout_and_ten_second_idle_contract():
    legacy = _legacy_command_module()
    assert legacy is not None, "shared legacy command safety boundary is missing"
    assert legacy.RECV_TIMEOUT_S == 1.0
    assert legacy.CONNECTION_IDLE_TIMEOUT_S == 10.0

    class ClosedConnection:
        timeout = None

        def settimeout(self, value):
            self.timeout = value

        def recv(self, _size):
            return b""

    connection = ClosedConnection()
    legacy.serve_command_connection(
        connection=connection,
        apply_command=lambda _value: None,
        hold_command=lambda: None,
        max_abs=5.0,
    )
    assert connection.timeout == 1.0


@pytest.mark.parametrize(
    "filename",
    (
        "pi_server_basic.py",
        "pi_server_velocity.py",
        "pi_server_position.py",
        "pi_server_video.py",
    ),
)
def test_every_legacy_pi_server_uses_shared_watchdog_and_deprecation_banner(filename):
    source = (Path(__file__).parents[1] / filename).read_text(encoding="utf-8")
    assert "DEPRECATED" in source
    assert "serve_command_connection" in source


def test_position_server_watchdog_requests_current_position_hold():
    source = (
        Path(__file__).parents[1] / "pi_server_position.py"
    ).read_text(encoding="utf-8")
    assert "def hold_current_position" in source
    assert "hold_command=hold_current_position" in source
    assert "_hold_position_requested" in source
