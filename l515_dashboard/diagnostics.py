"""Message-free rolling diagnostics for the six L515 ROS topics."""

from collections import deque
from dataclasses import dataclass
from types import MappingProxyType
from typing import Deque, Mapping, Optional


COLOR_TOPIC = "/l515/color/image_raw"
COLOR_INFO_TOPIC = "/l515/color/camera_info"
DEPTH_TOPIC = "/l515/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/l515/depth/camera_info"
GYRO_TOPIC = "/l515/gyro/sample"
ACCEL_TOPIC = "/l515/accel/sample"

ALL_TOPICS = (
    COLOR_TOPIC,
    COLOR_INFO_TOPIC,
    DEPTH_TOPIC,
    DEPTH_INFO_TOPIC,
    GYRO_TOPIC,
    ACCEL_TOPIC,
)

VIDEO_FRESHNESS_S = 0.25
CAMERA_INFO_FRESHNESS_S = 0.50
IMU_FRESHNESS_S = 0.25
DEFAULT_WINDOW_S = 5.0
DEFAULT_MAX_ARRIVALS = 512

FRESHNESS_S = MappingProxyType(
    {
        COLOR_TOPIC: VIDEO_FRESHNESS_S,
        DEPTH_TOPIC: VIDEO_FRESHNESS_S,
        COLOR_INFO_TOPIC: CAMERA_INFO_FRESHNESS_S,
        DEPTH_INFO_TOPIC: CAMERA_INFO_FRESHNESS_S,
        GYRO_TOPIC: IMU_FRESHNESS_S,
        ACCEL_TOPIC: IMU_FRESHNESS_S,
    }
)


@dataclass(frozen=True)
class TopicDiagnostics:
    """Scalar-only diagnostics for one topic at snapshot time."""

    count: int
    fps: float
    age_s: Optional[float]
    max_gap_s: float
    nonincreasing_count: int
    fresh: bool


@dataclass(frozen=True)
class DiagnosticsSnapshot:
    """Immutable point-in-time diagnostics safe for the TUI to retain."""

    topics: Mapping[str, TopicDiagnostics]
    healthy: bool


@dataclass
class _TopicState:
    arrivals: Deque[tuple[int, int]]
    last_arrival_ns: Optional[int] = None


class DiagnosticsTracker:
    """Track arrival timing and header-stamp health without retaining messages."""

    def __init__(
        self,
        *,
        window_s: float = DEFAULT_WINDOW_S,
        max_arrivals: int = DEFAULT_MAX_ARRIVALS,
    ) -> None:
        if window_s <= 0:
            raise ValueError("window_s must be positive")
        if isinstance(max_arrivals, bool) or not isinstance(max_arrivals, int):
            raise ValueError("max_arrivals must be an integer")
        if max_arrivals < 2:
            raise ValueError("max_arrivals must be at least 2")
        self._window_ns = int(window_s * 1_000_000_000)
        self._states = {
            topic: _TopicState(deque(maxlen=max_arrivals)) for topic in ALL_TOPICS
        }

    def observe(self, topic: str, stamp_ns: int, now_ns: int) -> None:
        """Record only a topic name, header stamp, and local arrival time."""

        try:
            state = self._states[topic]
        except KeyError as exc:
            raise ValueError(f"unknown L515 topic: {topic!r}") from exc

        if isinstance(stamp_ns, bool) or not isinstance(stamp_ns, int):
            raise ValueError("stamp_ns must be an integer")
        if isinstance(now_ns, bool) or not isinstance(now_ns, int):
            raise ValueError("now_ns must be an integer")

        state.last_arrival_ns = now_ns
        state.arrivals.append((now_ns, stamp_ns))
        self._prune(state, now_ns)

    def snapshot(self, now_ns: int) -> DiagnosticsSnapshot:
        """Return an immutable scalar snapshot evaluated at ``now_ns``."""

        if isinstance(now_ns, bool) or not isinstance(now_ns, int):
            raise ValueError("now_ns must be an integer")

        metrics = {}
        for topic, state in self._states.items():
            self._prune(state, now_ns)
            age_s = (
                None
                if state.last_arrival_ns is None
                else max(0, now_ns - state.last_arrival_ns) / 1_000_000_000
            )
            fresh = age_s is not None and age_s <= FRESHNESS_S[topic]
            metrics[topic] = TopicDiagnostics(
                count=len(state.arrivals),
                fps=self._fps(state.arrivals),
                age_s=age_s,
                max_gap_s=self._max_gap(state.arrivals),
                nonincreasing_count=self._nonincreasing_count(state.arrivals),
                fresh=fresh,
            )

        immutable_metrics = MappingProxyType(metrics)
        return DiagnosticsSnapshot(
            topics=immutable_metrics,
            healthy=all(metric.fresh for metric in immutable_metrics.values()),
        )

    def _prune(self, state: _TopicState, now_ns: int) -> None:
        cutoff_ns = now_ns - self._window_ns
        while state.arrivals and state.arrivals[0][0] < cutoff_ns:
            state.arrivals.popleft()

    @staticmethod
    def _fps(arrivals: Deque[tuple[int, int]]) -> float:
        if len(arrivals) < 2:
            return 0.0
        duration_ns = arrivals[-1][0] - arrivals[0][0]
        if duration_ns <= 0:
            return 0.0
        return (len(arrivals) - 1) * 1_000_000_000 / duration_ns

    @staticmethod
    def _max_gap(arrivals: Deque[tuple[int, int]]) -> float:
        stamps = [stamp_ns for _, stamp_ns in arrivals]
        positive_gaps = [
            current - previous
            for previous, current in zip(stamps, stamps[1:])
            if current > previous
        ]
        return max(positive_gaps, default=0) / 1_000_000_000

    @staticmethod
    def _nonincreasing_count(arrivals: Deque[tuple[int, int]]) -> int:
        stamps = [stamp_ns for _, stamp_ns in arrivals]
        return sum(
            current <= previous
            for previous, current in zip(stamps, stamps[1:])
        )
