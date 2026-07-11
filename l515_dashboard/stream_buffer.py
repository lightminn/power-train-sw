"""Small bounded, non-waiting handoffs for RealSense callback frames."""

from collections import deque
from dataclasses import dataclass
import threading


@dataclass(frozen=True)
class StreamSample:
    stream: str
    frame_number: int
    timestamp_ms: float
    received_ns: int
    frame: object


@dataclass(frozen=True)
class RingRead:
    sequence: int
    samples: tuple


class LatestSlot:
    """A latest-one handoff whose producer never waits for a consumer."""

    def __init__(self):
        self._lock = threading.Lock()
        self._sequence = 0
        self._sample = None

    def publish(self, sample):
        with self._lock:
            self._sequence += 1
            self._sample = sample

    def read_after(self, sequence):
        with self._lock:
            if self._sample is None or self._sequence <= sequence:
                return self._sequence, None
            return self._sequence, self._sample

    def clear(self):
        with self._lock:
            self._sample = None


class BoundedRing:
    """A bounded FIFO handoff that discards its oldest item on overflow."""

    def __init__(self, capacity):
        if isinstance(capacity, bool) or int(capacity) != capacity or capacity <= 0:
            raise ValueError("capacity must be a positive integer")
        self._lock = threading.Lock()
        self._items = deque()
        self._capacity = int(capacity)
        self._sequence = 0
        self._dropped = 0

    @property
    def dropped(self):
        with self._lock:
            return self._dropped

    def publish(self, sample):
        with self._lock:
            self._sequence += 1
            if len(self._items) == self._capacity:
                self._items.popleft()
                self._dropped += 1
            self._items.append((self._sequence, sample))

    def read_after(self, sequence, limit):
        if limit < 0:
            raise ValueError("limit must not be negative")
        with self._lock:
            unread = ((seq, item) for seq, item in self._items if seq > sequence)
            selected = []
            for seq, item in unread:
                if len(selected) >= limit:
                    break
                selected.append((seq, item))
            next_sequence = selected[-1][0] if selected else max(
                sequence, self._sequence if not self._items else sequence
            )
            return RingRead(next_sequence, tuple(item for _, item in selected))

    def clear(self):
        with self._lock:
            self._items.clear()
