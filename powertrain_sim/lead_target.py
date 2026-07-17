"""Deterministic lead-target plant and ROS-free relative detections."""
from __future__ import annotations

from dataclasses import dataclass
import hashlib
import math
from typing import Sequence

from .fixtures import GroundTruthFrame


@dataclass(frozen=True)
class LeadTargetSpec:
    path: str
    speed_m_s: float
    occlusions: tuple[tuple[float, float], ...] = ()
    dropout_ratio: float = 0.0

    def __post_init__(self) -> None:
        if self.path not in {"straight", "curve"}:
            raise ValueError("path must be 'straight' or 'curve'")
        if (
            isinstance(self.speed_m_s, bool)
            or not math.isfinite(float(self.speed_m_s))
            or float(self.speed_m_s) < 0.0
        ):
            raise ValueError("speed_m_s must be finite and nonnegative")
        if (
            isinstance(self.dropout_ratio, bool)
            or not math.isfinite(float(self.dropout_ratio))
            or not 0.0 <= float(self.dropout_ratio) <= 1.0
        ):
            raise ValueError("dropout_ratio must be within [0, 1]")

        normalized = []
        for interval in self.occlusions:
            if len(interval) != 2:
                raise ValueError("each occlusion must be (start_s, end_s)")
            start_s, end_s = (float(value) for value in interval)
            if (
                not math.isfinite(start_s)
                or not math.isfinite(end_s)
                or start_s < 0.0
                or end_s <= start_s
            ):
                raise ValueError("occlusion bounds must be finite and increasing")
            normalized.append((start_s, end_s))
        object.__setattr__(self, "speed_m_s", float(self.speed_m_s))
        object.__setattr__(self, "occlusions", tuple(normalized))
        object.__setattr__(self, "dropout_ratio", float(self.dropout_ratio))


@dataclass(frozen=True)
class LeadTargetPose:
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    station_m: float


class LeadTargetPlant:
    """Move a lead target along the scenario centerline at a fixed speed."""

    def __init__(
        self,
        spec: LeadTargetSpec,
        *,
        centerline_m: Sequence[Sequence[float]],
        seed: int,
        initial_offset_m: float = 2.0,
    ) -> None:
        if isinstance(seed, bool) or not isinstance(seed, int) or seed < 0:
            raise ValueError("seed must be a nonnegative integer")
        if (
            isinstance(initial_offset_m, bool)
            or not math.isfinite(float(initial_offset_m))
            or float(initial_offset_m) < 0.0
        ):
            raise ValueError("initial_offset_m must be finite and nonnegative")
        points = tuple(tuple(float(value) for value in point) for point in centerline_m)
        if len(points) < 2 or any(len(point) != 3 for point in points):
            raise ValueError("centerline_m must contain at least two 3D points")
        if any(not all(math.isfinite(value) for value in point) for point in points):
            raise ValueError("centerline_m values must be finite")

        lengths = tuple(
            math.hypot(right[0] - left[0], right[1] - left[1])
            for left, right in zip(points, points[1:])
        )
        if any(length <= 0.0 for length in lengths):
            raise ValueError("centerline_m XY segments must have positive length")
        stations = [0.0]
        for length in lengths:
            stations.append(stations[-1] + length)

        self.spec = spec
        self.centerline_m = points
        self.seed = seed
        self.initial_offset_m = float(initial_offset_m)
        self._lengths = lengths
        self._stations = tuple(stations)

    def pose(self, t: float) -> LeadTargetPose:
        """Return the deterministic centerline pose at elapsed time ``t``."""
        if isinstance(t, bool) or not math.isfinite(float(t)) or float(t) < 0.0:
            raise ValueError("t must be finite and nonnegative")
        requested_station = self.initial_offset_m + self.spec.speed_m_s * float(t)
        station = min(requested_station, self._stations[-1])
        segment_index = len(self._lengths) - 1
        for index, end_station in enumerate(self._stations[1:]):
            if station <= end_station:
                segment_index = index
                break

        start = self.centerline_m[segment_index]
        end = self.centerline_m[segment_index + 1]
        fraction = (station - self._stations[segment_index]) / self._lengths[segment_index]
        x_m = start[0] + fraction * (end[0] - start[0])
        y_m = start[1] + fraction * (end[1] - start[1])
        z_m = start[2] + fraction * (end[2] - start[2])
        yaw_rad = math.atan2(end[1] - start[1], end[0] - start[0])
        return LeadTargetPose(x_m, y_m, z_m, yaw_rad, station)

    def _is_dropped(self, t: float) -> bool:
        if self.spec.dropout_ratio <= 0.0:
            return False
        if self.spec.dropout_ratio >= 1.0:
            return True
        key = f"{self.seed}:{float(t).hex()}".encode("ascii")
        sample = int.from_bytes(hashlib.blake2b(key, digest_size=8).digest(), "big")
        return sample / float(2**64) < self.spec.dropout_ratio

    def detections_source(
        self,
        t: float,
        robot_pose: GroundTruthFrame,
    ) -> list[tuple[str, float, float, float, float]]:
        """Synthesize one production follow tuple in the robot body frame."""
        t = float(t)
        if any(start_s <= t < end_s for start_s, end_s in self.spec.occlusions):
            return []
        if self._is_dropped(t):
            return []

        target = self.pose(t)
        dx_m = target.x_m - float(robot_pose.x_m)
        dy_m = target.y_m - float(robot_pose.y_m)
        cosine = math.cos(float(robot_pose.yaw_rad))
        sine = math.sin(float(robot_pose.yaw_rad))
        forward_m = cosine * dx_m + sine * dy_m
        left_m = -sine * dx_m + cosine * dy_m
        if forward_m <= 0.0:
            return []
        distance_m = math.hypot(forward_m, left_m)
        # Deliberately simple simulator surrogate: apparent bbox area is
        # approximated as inversely proportional to target distance.
        bbox_area_px = 8000.0 / max(distance_m, 0.25)
        return [("robot", 0.95, forward_m, left_m, bbox_area_px)]


__all__ = (
    "LeadTargetPlant",
    "LeadTargetPose",
    "LeadTargetSpec",
)
