"""Pure lifecycle state for a broker-driven BL70200 calibration job."""

from __future__ import annotations

from dataclasses import dataclass
from enum import Enum
from typing import Callable, Iterable


class CalibrationState(str, Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    DONE = "DONE"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


@dataclass(frozen=True)
class CalibrationStatus:
    state: CalibrationState
    nodes: tuple[int, ...]
    results: tuple[tuple[int, bool], ...]
    started_at: float | None
    finished_at: float | None


class CalibrationJob:
    """Track one exclusive, externally executed calibration run."""

    def __init__(self, clock: Callable[[], float]) -> None:
        self._clock = clock
        self._state = CalibrationState.IDLE
        self._nodes: tuple[int, ...] = ()
        self._results: dict[int, bool] = {}
        self._started_at: float | None = None
        self._finished_at: float | None = None

    def start(self, nodes: Iterable[int]) -> bool:
        """Start a job, or return False when another job is still running."""

        if self._state == CalibrationState.RUNNING:
            return False
        requested = tuple(nodes)
        if not requested:
            raise ValueError("calibration nodes must not be empty")
        if len(set(requested)) != len(requested):
            raise ValueError("calibration nodes must be unique")

        self._state = CalibrationState.RUNNING
        self._nodes = requested
        self._results = {}
        self._started_at = self._clock()
        self._finished_at = None
        return True

    def on_axis_result(self, node: int, success: bool) -> bool:
        """Record one result; terminal or duplicate results are ignored."""

        if self._state != CalibrationState.RUNNING:
            return False
        if node not in self._nodes:
            raise ValueError(f"axis {node} is not part of the running job")
        if node in self._results:
            return False

        self._results[node] = bool(success)
        if len(self._results) == len(self._nodes):
            self._state = (
                CalibrationState.DONE
                if all(self._results.values())
                else CalibrationState.FAILED
            )
            self._finished_at = self._clock()
        return True

    def status(self) -> CalibrationStatus:
        """Return an immutable snapshot in requested-node order."""

        results = tuple(
            (node, self._results[node]) for node in self._nodes if node in self._results
        )
        return CalibrationStatus(
            state=self._state,
            nodes=self._nodes,
            results=results,
            started_at=self._started_at,
            finished_at=self._finished_at,
        )

    def cancel(self) -> bool:
        """Cancel a running job; return False for non-running states."""

        if self._state != CalibrationState.RUNNING:
            return False
        self._state = CalibrationState.CANCELLED
        self._finished_at = self._clock()
        return True
