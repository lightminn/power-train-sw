"""Append-only JSONL journal and nonblocking producer queue."""
from __future__ import annotations

import os
import queue
from collections.abc import Mapping
from pathlib import Path
from typing import Any, BinaryIO

from .events import decode_event, encode_event
from .health import HealthState


def _complete_prefix(data: bytes) -> bytes:
    if data.endswith(b"\n"):
        return data
    last_newline = data.rfind(b"\n")
    if last_newline < 0:
        return b""
    return data[: last_newline + 1]


def recover_records(path: str | os.PathLike[str]) -> list[dict[str, Any]]:
    """Read complete JSONL records, ignoring only an incomplete final line."""
    data = Path(path).read_bytes()
    complete = _complete_prefix(data)
    return [decode_event(line) for line in complete.splitlines() if line]


class BoundedEventQueue:
    """A bounded queue whose producer path never waits for consumers."""

    def __init__(self, capacity: int, *, health: HealthState | None = None) -> None:
        if isinstance(capacity, bool) or not isinstance(capacity, int) or capacity < 1:
            raise ValueError("capacity must be a positive integer")
        self._queue: queue.Queue[Any] = queue.Queue(maxsize=capacity)
        self._health = health if health is not None else HealthState()

    def offer(self, item: Any) -> bool:
        try:
            self._queue.put_nowait(item)
        except queue.Full:
            self._health.record_drop()
            return False
        return True

    def get_nowait(self) -> Any:
        return self._queue.get_nowait()

    @property
    def drop_count(self) -> int:
        return self._health.snapshot().drop_count


class MissionJournal:
    """Own one run's JSONL segments and assign its monotonic sequence."""

    def __init__(
        self,
        directory: str | os.PathLike[str],
        *,
        run_id: str,
        max_file_bytes: int = 64 * 1024 * 1024,
        health: HealthState | None = None,
    ) -> None:
        if not isinstance(run_id, str) or not run_id or run_id in {".", ".."}:
            raise ValueError("run_id must be a non-empty filename component")
        if "/" in run_id or "\\" in run_id or "\x00" in run_id:
            raise ValueError("run_id must not contain path separators")
        if (
            isinstance(max_file_bytes, bool)
            or not isinstance(max_file_bytes, int)
            or max_file_bytes < 1
        ):
            raise ValueError("max_file_bytes must be a positive integer")

        self._directory = Path(directory)
        self._directory.mkdir(parents=True, exist_ok=True)
        self._run_id = run_id
        self._max_file_bytes = max_file_bytes
        self._health = health if health is not None else HealthState()
        segments = self._existing_segments()
        self._segment = segments[-1][0] if segments else 0
        records = [
            record
            for _, path in segments
            for record in recover_records(path)
        ]
        for record in records:
            if record["run_id"] != self._run_id:
                raise ValueError("journal segment contains a different run_id")
        self._next_sequence = (
            max(record["sequence"] for record in records) + 1 if records else 0
        )
        self._file: BinaryIO | None = None
        self._open_current_segment()

    @property
    def path(self) -> Path:
        return self._segment_path(self._segment)

    @property
    def health(self) -> HealthState:
        return self._health

    def _segment_path(self, segment: int) -> Path:
        return self._directory / f"{self._run_id}.{segment:06d}.jsonl"

    def _existing_segments(self) -> list[tuple[int, Path]]:
        prefix = f"{self._run_id}."
        suffix = ".jsonl"
        segments = []
        for path in self._directory.iterdir():
            name = path.name
            if not name.startswith(prefix) or not name.endswith(suffix):
                continue
            number = name[len(prefix) : -len(suffix)]
            if number.isdigit():
                segments.append((int(number), path))
        return sorted(segments)

    def _open_current_segment(self) -> None:
        path = self.path
        self._file = path.open("a+b")
        self._file.seek(0)
        data = self._file.read()
        complete = _complete_prefix(data)
        if len(complete) != len(data):
            self._file.seek(len(complete))
            self._file.truncate()
        self._file.seek(0, os.SEEK_END)
        self._current_file_bytes = len(complete)

    def _rotate_if_needed(self, line_size: int) -> None:
        if self._current_file_bytes == 0:
            return
        if self._current_file_bytes + line_size <= self._max_file_bytes:
            return
        self.flush()
        assert self._file is not None
        try:
            self._file.close()
        except OSError as exc:
            self._health.mark_degraded(f"journal close failed: {exc}")
        self._segment += 1
        self._file = None
        self._open_current_segment()

    def append(self, event: Mapping[str, Any]) -> dict[str, Any] | None:
        if self._file is None:
            raise RuntimeError("journal is closed")
        record = dict(event)
        record["run_id"] = self._run_id
        record["sequence"] = self._next_sequence
        line = encode_event(record) + b"\n"
        self._rotate_if_needed(len(line))
        assert self._file is not None
        try:
            self._file.write(line)
        except OSError as exc:
            self._health.mark_degraded(f"journal write failed: {exc}")
            return None
        self._current_file_bytes += len(line)
        self._next_sequence += 1
        return record

    def flush(self) -> bool:
        if self._file is None:
            return True
        try:
            self._file.flush()
            os.fsync(self._file.fileno())
        except OSError as exc:
            self._health.mark_degraded(f"journal flush failed: {exc}")
            return False
        return True

    def close(self) -> None:
        if self._file is None:
            return
        self.flush()
        try:
            self._file.close()
        except OSError as exc:
            self._health.mark_degraded(f"journal close failed: {exc}")
        finally:
            self._file = None

    def __enter__(self) -> MissionJournal:
        return self

    def __exit__(self, _exc_type, _exc, _traceback) -> None:
        self.close()
