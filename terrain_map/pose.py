"""Pose samples -> pose at a depth stamp (spec §4.4 PoseTrack). Standard library only."""
from __future__ import annotations

import bisect
from collections import deque

from .config import MapConfig
from .geometry import Pose2D
from .odometry import PoseSample, wrap

KEEP_S = 2.0


class PoseTrack:
    def __init__(self, cfg: MapConfig) -> None:
        self.cfg = cfg
        self.epoch: int | None = None
        self._samples: deque[PoseSample] = deque()

    def clear(self) -> None:
        self._samples.clear()

    def add(self, sample: PoseSample) -> None:
        if self.epoch is not None and sample.epoch != self.epoch:
            self._samples.clear()
        self.epoch = sample.epoch
        if self._samples and sample.stamp_s <= self._samples[-1].stamp_s:
            return
        self._samples.append(sample)
        while self._samples and sample.stamp_s - self._samples[0].stamp_s > KEEP_S:
            self._samples.popleft()

    def pose_at(self, t: float) -> tuple[Pose2D | None, str | None]:
        if not self._samples:
            return None, "ODOM_MISSING"
        if t > self._samples[-1].stamp_s:
            return None, "WAITING"
        if t < self._samples[0].stamp_s:
            return None, "ODOM_MISSING"
        stamps = [s.stamp_s for s in self._samples]
        i = bisect.bisect_left(stamps, t)
        if stamps[i] == t:
            a = b = self._samples[i]
        else:
            a, b = self._samples[i - 1], self._samples[i]
            if b.stamp_s - a.stamp_s > self.cfg.pose_max_gap_s:
                return None, "POSE_GAP"
        for s in (a, b):
            if not s.valid:
                return None, s.reason or "POSE_GAP"
        if a is b:
            return Pose2D(a.x, a.y, a.yaw), None
        f = (t - a.stamp_s) / (b.stamp_s - a.stamp_s)
        return Pose2D(a.x + f * (b.x - a.x), a.y + f * (b.y - a.y), wrap(a.yaw + f * wrap(b.yaw - a.yaw))), None
