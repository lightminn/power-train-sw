"""Fail-closed command session shared by deprecated Raspberry Pi demos."""

from __future__ import annotations

import math
import socket
import threading
import time
from collections.abc import Callable


COMMAND_FRESHNESS_TIMEOUT_S = 0.5
RECV_TIMEOUT_S = 1.0
CONNECTION_IDLE_TIMEOUT_S = 10.0


def parse_finite_command(value: bytes | str, *, max_abs: float) -> float | None:
    """Parse and clamp one scalar command, rejecting NaN and infinities."""
    try:
        text = value.decode("utf-8", errors="strict") if isinstance(value, bytes) else value
        command = float(text.strip())
        limit = float(max_abs)
    except (AttributeError, TypeError, UnicodeDecodeError, ValueError):
        return None
    if not math.isfinite(command) or not math.isfinite(limit) or limit <= 0.0:
        return None
    return max(-limit, min(limit, command))


def serve_command_connection(
    *,
    connection,
    apply_command: Callable[[float], None],
    hold_command: Callable[[], None],
    max_abs: float,
    freshness_timeout_s: float = COMMAND_FRESHNESS_TIMEOUT_S,
    recv_timeout_s: float = RECV_TIMEOUT_S,
    idle_timeout_s: float = CONNECTION_IDLE_TIMEOUT_S,
    clock: Callable[[], float] = time.monotonic,
) -> None:
    """Receive newline commands while an independent watchdog enforces hold.

    The receive timeout exists to detect a completely idle connection.  Motor
    freshness is deliberately enforced by a separate thread, so a blackholed
    socket cannot extend the 0.5 s command lifetime to the 1 s recv timeout.
    """
    for name, value in (
        ("freshness_timeout_s", freshness_timeout_s),
        ("recv_timeout_s", recv_timeout_s),
        ("idle_timeout_s", idle_timeout_s),
    ):
        if not math.isfinite(float(value)) or float(value) <= 0.0:
            raise ValueError(f"{name} must be finite and positive")

    connection.settimeout(float(recv_timeout_s))
    state_lock = threading.Lock()
    state = {"last_valid_s": None, "hold_sent": False}
    stopping = threading.Event()

    def hold_if_stale(*, force: bool = False) -> None:
        with state_lock:
            last_valid_s = state["last_valid_s"]
            if last_valid_s is None or state["hold_sent"]:
                return
            if not force and clock() - last_valid_s <= freshness_timeout_s:
                return
            state["hold_sent"] = True
        hold_command()

    def watchdog() -> None:
        interval_s = min(0.05, freshness_timeout_s / 4.0)
        while not stopping.wait(interval_s):
            hold_if_stale()

    watchdog_thread = threading.Thread(
        target=watchdog,
        name="deprecated-pi-command-watchdog",
        daemon=True,
    )
    watchdog_thread.start()
    buffer = bytearray()
    last_data_s = clock()
    try:
        while True:
            try:
                data = connection.recv(64)
            except socket.timeout:
                if clock() - last_data_s > idle_timeout_s:
                    break
                continue
            if not data:
                break
            last_data_s = clock()
            buffer.extend(data)
            while True:
                newline = buffer.find(b"\n")
                if newline < 0:
                    break
                line = bytes(buffer[:newline])
                del buffer[: newline + 1]
                command = parse_finite_command(line, max_abs=max_abs)
                if command is None:
                    continue
                with state_lock:
                    state["last_valid_s"] = clock()
                    state["hold_sent"] = False
                apply_command(command)
    finally:
        stopping.set()
        watchdog_thread.join(timeout=1.0)
        hold_if_stale(force=True)
