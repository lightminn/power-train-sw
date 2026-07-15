"""Thread-safe mutable health state with immutable diagnostic snapshots."""
from __future__ import annotations

from dataclasses import dataclass
from threading import Lock


@dataclass(frozen=True)
class HealthSnapshot:
    status: str
    drop_count: int
    last_error: str | None


class HealthState:
    def __init__(self) -> None:
        self._status = "OK"
        self._drop_count = 0
        self._last_error: str | None = None
        self._lock = Lock()

    def record_drop(self, count: int = 1) -> None:
        if isinstance(count, bool) or not isinstance(count, int) or count < 1:
            raise ValueError("drop count increment must be a positive integer")
        with self._lock:
            self._drop_count += count

    def mark_degraded(self, reason: str) -> None:
        if not isinstance(reason, str) or not reason:
            raise ValueError("degraded reason must be a non-empty string")
        with self._lock:
            self._status = "DEGRADED"
            self._last_error = reason

    def snapshot(self) -> HealthSnapshot:
        with self._lock:
            return HealthSnapshot(
                status=self._status,
                drop_count=self._drop_count,
                last_error=self._last_error,
            )
