"""Production lead-follower core adapted to simulator callback hooks."""
from __future__ import annotations

import math

from chassis.follow import FollowResult, LeadFollower
from powertrain_ros.state_estimation import StateSnapshot

from .fixtures import GroundTruthFrame
from .lead_target import LeadTargetPlant


class FollowDriver:
    """Cache synthesized observations, tick ``LeadFollower``, and expose hold."""

    def __init__(
        self,
        target: LeadTargetPlant,
        *,
        controller: LeadFollower | None = None,
    ) -> None:
        self.target = target
        self.controller = controller or LeadFollower()
        self._detections: list[tuple[str, float, float, float, float]] = []
        self._observation_time_s: float | None = None
        self._lead_distance_m: float | None = None
        self._result: FollowResult | None = None

    def detections_source(
        self,
        elapsed_s: float,
        robot_pose: GroundTruthFrame,
    ) -> list[tuple[str, float, float, float, float]]:
        """Synthesize and cache the observation consumed by this tick's command."""
        target_pose = self.target.pose(elapsed_s)
        self._lead_distance_m = math.hypot(
            target_pose.x_m - robot_pose.x_m,
            target_pose.y_m - robot_pose.y_m,
        )
        self._detections = self.target.detections_source(elapsed_s, robot_pose)
        self._observation_time_s = float(elapsed_s)
        return list(self._detections)

    def command(
        self,
        elapsed_s: float,
        snapshot: StateSnapshot | None,
    ) -> tuple[float, float]:
        """Tick the production follow core and return a runner command tuple."""
        del snapshot
        fresh = (
            self._observation_time_s is not None
            and math.isclose(
                self._observation_time_s,
                float(elapsed_s),
                rel_tol=0.0,
                abs_tol=1e-12,
            )
        )
        detections = self._detections if fresh else []
        self._result = self.controller.update(detections, float(elapsed_s))
        if not self._result.ok:
            return 0.0, 0.0
        return self._result.v, self._result.omega

    def hold_state(
        self,
        elapsed_s: float,
        snapshot: StateSnapshot | None,
    ) -> tuple[bool, bool]:
        """Map only production ``LOST`` to actual/required motion hold."""
        del elapsed_s, snapshot
        lost = self.follow_state == "LOST"
        return lost, lost

    @property
    def lead_distance_m(self) -> float | None:
        return self._lead_distance_m

    @property
    def follow_state(self) -> str:
        return "LOST" if self._result is None else self._result.state


__all__ = ("FollowDriver",)
