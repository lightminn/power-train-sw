"""Sender pinning and sequence ordering for operator-console UDP inputs."""

from __future__ import annotations

import math


RESTART_SEQUENCE_MAX = 3


class SourceSequenceGate:
    """Accept one sender until stale and reject duplicate/reordered datagrams."""

    def __init__(self, *, stale_after_s: float = 2.0) -> None:
        stale_after_s = float(stale_after_s)
        if not math.isfinite(stale_after_s) or stale_after_s <= 0.0:
            raise ValueError("stale_after_s must be finite and positive")
        self._stale_after_s = stale_after_s
        self._sender = None
        self._last_sequence: int | None = None
        self._last_accepted_s: float | None = None

    def accept(self, address, sequence: int, *, now_s: float) -> bool:
        if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
            return False
        try:
            now_s = float(now_s)
        except (TypeError, ValueError):
            return False
        if not math.isfinite(now_s):
            return False

        sender_stale = bool(
            self._last_accepted_s is not None
            and now_s - self._last_accepted_s > self._stale_after_s
        )
        if self._sender is None or (address != self._sender and sender_stale):
            self._sender = address
            self._last_sequence = None
            self._last_accepted_s = None
        if address != self._sender:
            return False

        restarting = bool(
            sender_stale
            and self._last_sequence is not None
            and self._last_sequence > RESTART_SEQUENCE_MAX
            and sequence <= RESTART_SEQUENCE_MAX
        )
        if (
            self._last_sequence is not None
            and sequence <= self._last_sequence
            and not restarting
        ):
            return False
        self._last_sequence = sequence
        self._last_accepted_s = now_s
        return True
