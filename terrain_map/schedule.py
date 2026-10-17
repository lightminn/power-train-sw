"""Which depth frame to process next (spec §4.8 FrameScheduler). Standard library only.

The target is the oldest pending frame at least ``0.9 / process_hz`` after the
last attempted one.  Once chosen it is fixed: new frames evict other pending
frames, never the target.  A target whose pose is still ``WAITING`` waits up to
``pose_wait_s`` (receive time); expiry is judged only after the pose check, so a
pose that appears on the expiry tick is processed.
"""
from __future__ import annotations

import bisect
from typing import Any, Callable, NamedTuple

from .config import MapConfig
from .geometry import Pose2D

POSE_EXPIRED = "POSE_EXPIRED"


class DepthFrame(NamedTuple):
    stamp_s: float
    rx_s: float
    array: Any


class Selected(NamedTuple):
    frame: DepthFrame
    pose: Pose2D


class FrameScheduler:
    def __init__(self, cfg: MapConfig, *, record: Callable[[dict], None] | None = None,
                 follow: frozenset | None = None) -> None:
        self.cfg = cfg
        self._record = record
        self._follow = follow             # stamps processed live; replay processes exactly these
        self._pending: list[DepthFrame] = []
        self._target: DepthFrame | None = None
        self._last_attempt: float | None = None
        self.skipped = 0                  # intentionally not processed (rate, eviction, too old)
        self.lost = 0                     # a target that could not be processed
        self.processed = 0

    @property
    def pending_count(self) -> int:
        return len(self._pending)

    def clear(self) -> None:
        """Epoch change: frames before it can never get a pose in the new epoch.

        The last attempt goes too: after a clock restart the new stamps are smaller (Codex T6 P2-1).
        """
        self._pending.clear()
        self._target = None
        self._last_attempt = None

    def push(self, frame: DepthFrame) -> None:
        if self._last_attempt is not None and frame.stamp_s <= self._last_attempt:
            self.skipped += 1
            return
        stamps = [f.stamp_s for f in self._pending]
        i = bisect.bisect_left(stamps, frame.stamp_s)
        if i < len(stamps) and stamps[i] == frame.stamp_s:
            return
        self._pending.insert(i, frame)
        while len(self._pending) > self.cfg.depth_pending_max:
            victim = next(f for f in self._pending if f is not self._target)
            self._pending.remove(victim)
            self.skipped += 1

    def select(self, pose_track, rx_now_s: float) -> tuple[Selected | None, list[tuple[float, str]]]:
        lost: list[tuple[float, str]] = []
        while True:
            if self._target is None:
                self._target = self._choose()
                if self._target is None:
                    return None, lost
            target = self._target
            pose, reason = pose_track.pose_at(target.stamp_s)
            if pose is not None:
                self._finish(target, "processed")
                self.processed += 1
                return Selected(target, pose), lost
            if reason == "WAITING":
                if self._follow is not None or rx_now_s - target.rx_s <= self.cfg.pose_wait_s:
                    return None, lost
                reason = POSE_EXPIRED
            self._finish(target, "lost", reason)
            self.lost += 1
            lost.append((target.stamp_s, reason))

    def _choose(self) -> DepthFrame | None:
        floor = None if self._last_attempt is None else self._last_attempt + 0.9 / self.cfg.process_hz
        while self._pending:
            head = self._pending[0]
            eligible = floor is None or head.stamp_s >= floor - 1e-9
            if eligible and (self._follow is None or head.stamp_s in self._follow):
                return head
            self._pending.pop(0)
            self.skipped += 1
        return None

    def _finish(self, target: DepthFrame, result: str, reason: str | None = None) -> None:
        self._pending.remove(target)
        self._target = None
        self._last_attempt = target.stamp_s
        if self._record is not None and self._follow is None:
            self._record({"k": "frame", "t": target.stamp_s, "r": result, "why": reason})
