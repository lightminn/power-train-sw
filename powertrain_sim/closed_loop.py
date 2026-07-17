"""Production-core terrain autonomy wired to the MuJoCo fast plant."""
from __future__ import annotations

import math
from pathlib import Path

from chassis.kinematics import ChassisGeometry, default_geometry
from powertrain_autonomy.controller import (
    EMPTY_STOWED,
    AutonomyController,
    AutonomyControllerConfig,
    ControllerDecision,
    DriveDiagnostics,
    DriveProfile,
    MotionState,
    ProfileGate,
)
from powertrain_autonomy.terrain import (
    BaseToCameraExtrinsic,
    BodyTilt,
    OdometryDelta,
    TerrainEstimate,
    TerrainEstimator,
    TerrainEstimatorConfig,
    TerrainFrame,
)
from powertrain_ros.state_estimation import StateSnapshot

from .fixtures import DepthFrame
from .mujoco_fast.model_builder import (
    CAMERA_PITCH_DOWN_RAD,
    CAMERA_POSITION_BODY_M,
    WHEEL_CONTACT_GAP_M,
)
from .mujoco_fast.runner import MetricsReport, run_scenario
from .scenario import Scenario


def _wrap(angle_rad: float) -> float:
    return math.atan2(math.sin(angle_rad), math.cos(angle_rad))


def _tile_extent(size: int) -> int:
    target = min(size, max(3, size // 4))
    for candidate in range(target, 2, -1):
        if size % candidate == 0:
            return candidate
    return size


def _scenario_estimator_config(scenario: Scenario) -> TerrainEstimatorConfig:
    """Keep production defaults while adapting fixed array extents to the scenario."""
    height, width = (int(value) for value in scenario.sensors["depth"]["shape_px"])
    defaults = TerrainEstimatorConfig()
    if (height, width) == defaults.depth_shape_px:
        return defaults
    return TerrainEstimatorConfig(
        depth_shape_px=(height, width),
        roi_rows=(0, height),
        roi_cols=(0, width),
        quality_tile_shape_px=(_tile_extent(height), _tile_extent(width)),
    )


class TerrainAutonomyDriver:
    """Adapt fast-mode value callbacks to the production WP6-B/WP6-C cores."""

    def __init__(
        self,
        scenario: Scenario,
        *,
        geometry: ChassisGeometry | None = None,
        profile: DriveProfile = EMPTY_STOWED,
        controller_config: AutonomyControllerConfig | None = None,
        estimator_config: TerrainEstimatorConfig | None = None,
    ) -> None:
        self.scenario = scenario
        self.geometry = geometry or default_geometry()
        self.controller = AutonomyController(profile, controller_config)
        self.estimator = TerrainEstimator(
            estimator_config or _scenario_estimator_config(scenario),
            geometry=self.geometry,
        )
        # Fast-model base_link is spawned one wheel radius plus the contact gap
        # above the surface.  This is the simulated ground-referenced camera
        # height, not the production core's provisional 0.60 m default.
        camera_height_m = (
            self.geometry.wheel_radius_m
            + WHEEL_CONTACT_GAP_M
            + CAMERA_POSITION_BODY_M[2]
        )
        self.extrinsic = BaseToCameraExtrinsic(
            x_m=CAMERA_POSITION_BODY_M[0],
            y_m=CAMERA_POSITION_BODY_M[1],
            z_m=camera_height_m,
            pitch_down_rad=CAMERA_PITCH_DOWN_RAD,
        )
        self._last_snapshot: StateSnapshot | None = None
        self._previous_depth_pose: tuple[float, float, float] | None = None
        self._terrain: TerrainEstimate | None = None
        self._decision: ControllerDecision | None = None
        self._decision_terrain: TerrainEstimate | None = None

    def _odometry_delta(self, snapshot: StateSnapshot) -> OdometryDelta:
        pose = snapshot.pose
        if self._previous_depth_pose is None:
            return OdometryDelta(0.0, 0.0, 0.0)
        previous_x, previous_y, previous_yaw = self._previous_depth_pose
        global_x = pose.x_m - previous_x
        global_y = pose.y_m - previous_y
        cosine = math.cos(previous_yaw)
        sine = math.sin(previous_yaw)
        return OdometryDelta(
            dx_m=cosine * global_x + sine * global_y,
            dy_m=-sine * global_x + cosine * global_y,
            dyaw_rad=_wrap(pose.yaw_rad - previous_yaw),
        )

    def on_depth(self, frame: DepthFrame) -> None:
        """Update terrain from one recorded depth frame and cached motion."""
        snapshot = self._last_snapshot
        if snapshot is None:
            return
        terrain_frame = TerrainFrame(
            depth_roi=frame.depth_roi,
            depth_scale_m=frame.depth_scale_m,
            intrinsics=frame.intrinsics,
            stamp_s=frame.stamp_s,
        )
        try:
            estimate = self.estimator.update(
                terrain_frame,
                tilt=BodyTilt(
                    roll_rad=snapshot.tilt.roll_rad,
                    pitch_rad=snapshot.tilt.pitch_rad,
                ),
                extrinsic=self.extrinsic,
                odometry_delta=self._odometry_delta(snapshot),
                now_s=frame.stamp_s,
            )
        except ValueError:
            self._terrain = None
            return
        # Match the production adapter: a failed update must not consume the
        # accumulated SE(2) motion before the next valid depth frame.
        self._previous_depth_pose = (
            snapshot.pose.x_m,
            snapshot.pose.y_m,
            snapshot.pose.yaw_rad,
        )
        self._terrain = estimate

    def _scenario_now(self, elapsed_s: float) -> float:
        return self.scenario.clock.start_s + float(elapsed_s)

    def command(
        self,
        elapsed_s: float,
        snapshot: StateSnapshot | None,
    ) -> tuple[float, float]:
        """Return the latest production controller command in scenario time."""
        self._last_snapshot = snapshot
        if snapshot is None:
            return 0.0, 0.0
        now_s = self._scenario_now(elapsed_s)
        motion = MotionState(
            stamp_s=now_s,
            forward_m_s=snapshot.velocity.forward_m_s,
            yaw_rate_rad_s=snapshot.velocity.yaw_rate_rad_s,
            roll_rad=snapshot.tilt.roll_rad,
            pitch_rad=snapshot.tilt.pitch_rad,
        )
        # Simulation assumption: the arm lock heartbeat is fresh on every tick.
        gate = ProfileGate(stamp_s=now_s, status="STOWED_LOCKED")
        # hold_state()의 should_hold는 이 결정이 실제로 소비한 terrain 기준이어야
        # 한다 — depth_tap이 같은 tick 후반에 terrain을 갱신하므로, 갱신 후
        # 값으로 비교하면 전이 tick마다 1-tick 가짜 fail-open이 계측된다.
        self._decision_terrain = self._terrain
        state_diagnostics = snapshot.diagnostics
        self._decision = self.controller.decide(
            now_s,
            terrain=self._terrain,
            motion=motion,
            gate=gate,
            diagnostics=DriveDiagnostics(
                stamp_s=now_s,
                slip_candidate=state_diagnostics.slip_candidate,
                stuck_candidate=state_diagnostics.stuck_candidate,
                speed_cap_m_s=state_diagnostics.terrain_speed_cap,
            ),
        )
        return self._decision.v_m_s, self._decision.omega_rad_s

    def hold_state(
        self,
        elapsed_s: float,
        snapshot: StateSnapshot | None,
    ) -> tuple[bool, bool]:
        """Expose controller hold and independent path-availability policy."""
        del elapsed_s, snapshot
        actual_hold = self._decision is None or self._decision.state != "TRACKING"
        should_hold = (
            self._decision_terrain is None
            or not self._decision_terrain.path_available
        )
        return actual_hold, should_hold


def run_closed_loop(
    scenario: Scenario,
    run_directory: str | Path,
    **kwargs,
) -> MetricsReport:
    """Run one fast-mode plant with production terrain and controller cores."""
    driver = TerrainAutonomyDriver(scenario, **kwargs)
    return run_scenario(
        scenario,
        run_directory,
        command_source=driver.command,
        hold_state_source=driver.hold_state,
        depth_tap=driver.on_depth,
        geometry=driver.geometry,
    )


__all__ = ("TerrainAutonomyDriver", "run_closed_loop")
