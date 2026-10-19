"""All tunable numbers of the terrain map (spec §4.1), validated on construction.

Standard library only.  Out-of-range or mutually inconsistent values raise
``ValueError``; nothing is silently clamped.
"""
from __future__ import annotations

from dataclasses import dataclass, fields
import math

CORRECTORS = ("identity", "height_match")

# name -> (low, high) inclusive.  None-able fields are checked only when set.
_RANGES: dict[str, tuple[float, float]] = {
    "cell_m": (0.02, 0.20),
    "tile_cells": (8, 16),
    "pixel_stride": (1, 4),
    "ground_min_points": (30, 200),
    "range_min_m": (0.2, 1.0),
    "range_max_m": (1.0, 5.0),
    "max_abs_height_m": (0.3, 1.2),
    "min_points_per_cell": (3, 50),
    "cell_trim_fraction": (0.0, 0.4),
    "cell_trim_max_points": (0, 10),
    "obstacle_height_m": (0.02, 0.5),
    "ditch_depth_m": (0.02, 0.5),
    "history_length": (1, 3),
    "history_max_age_s": (1.0, 120.0),
    "support_band_m": (0.2, 1.0),
    "support_tol_m": (0.01, 0.1),
    "support_min_points": (20, 5000),
    "support_base_height_m": (0.05, 0.80),
    "support_height_tol_m": (0.01, 0.15),
    "support_base_variation_m": (0.0, 0.1),
    "min_observations": (1, 10),
    "age_step_s": (2.0, 10.0),
    "process_hz": (1.0, 10.0),
    "status_hz": (1.0, 10.0),
    "max_tiles_per_tick": (1, 64),
    "refresh_tiles_per_tick": (1, 64),
    "max_map_tiles": (64, 8192),
    "depth_pending_max": (2, 10),
    "pose_wait_s": (0.1, 0.8),
    "input_lag_budget_s": (0.0, 0.5),
    "pose_max_gap_s": (0.02, 0.2),
    "min_valid_wheels": (1, 6),
    "wheel_radius_m": (0.05, 0.2),
    "lateness_margin_s": (0.0, 0.15),
    "gyro_gap_epoch_s": (0.02, 0.2),
    "wheel_gap_invalid_s": (0.02, 0.3),
    "wheel_gap_epoch_s": (0.1, 1.0),
    "ls_max_condition": (5.0, 1000.0),
    "max_lateral_speed_m_s": (0.05, 1.0),
    "max_slip_wheels": (0, 5),
    "slip_residual_m_s": (0.03, 1.0),
    "slip_epoch_s": (0.1, 5.0),
    "max_stream_skew_s": (0.2, 2.0),
    "stationary_command_turns_per_s": (0.0, 0.1),
    "stationary_measured_turns_per_s": (0.0, 0.2),
    "bias_stationary_s": (0.3, 5.0),
    "bias_max_std_rad_s": (0.001, 0.05),
    "bias_alpha": (0.01, 1.0),
    "stamp_regression_epoch_s": (0.1, 10.0),
    "still_window_s": (0.2, 2.0),
    "still_max_gyro_rad_s": (0.005, 0.5),
    "still_max_accel_std": (0.01, 1.0),
    "gravity_tolerance": (0.02, 0.3),
    "slope_max_age_s": (0.2, 5.0),
    "camera_x_m": (-2.0, 2.0),
    "camera_y_m": (-2.0, 2.0),
    "camera_z_m": (-1.0, 2.0),
    "camera_pitch_down_deg": (0.0, 35.0),
    "camera_roll_deg": (-20.0, 20.0),
    "camera_yaw_deg": (-30.0, 30.0),
    "match_feature_m": (0.005, 0.2),
    "match_min_features": (1, 10000),
    "match_window_m": (0.0, 0.5),
    "match_window_deg": (0.0, 10.0),
    "match_yaw_step_deg": (0.1, 5.0),
    "match_min_overlap": (10, 100000),
    "match_recent_s": (1.0, 600.0),
    "match_code_weight": (0.0, 1.0),
    "match_max_step_m": (0.0, 0.5),
    "match_max_step_deg": (0.0, 10.0),
    "match_min_gain": (0.0, 0.1),
    "match_min_peak": (0.0, 0.1),
}

_INTEGERS = {
    "tile_cells", "pixel_stride", "ground_min_points", "min_points_per_cell", "history_length",
    "support_min_points", "min_observations", "max_tiles_per_tick", "refresh_tiles_per_tick",
    "max_map_tiles", "depth_pending_max", "min_valid_wheels", "match_min_features", "match_min_overlap",
    "max_slip_wheels", "cell_trim_max_points",
}


@dataclass(frozen=True)
class MapConfig:
    cell_m: float = 0.05
    tile_cells: int = 16
    pixel_stride: int = 2
    ground_min_points: int = 100
    range_min_m: float = 0.3
    range_max_m: float = 3.0
    max_abs_height_m: float = 1.0
    min_points_per_cell: int = 5
    cell_trim_fraction: float = 0.1
    cell_trim_max_points: int = 0           # r14: cap on the per-side trim count (0 = no cap)
    obstacle_height_m: float = 0.08
    ditch_depth_m: float = 0.08
    history_length: int = 3
    history_max_age_s: float = 10.0
    support_band_m: float = 0.5
    support_tol_m: float = 0.04
    support_min_points: int = 200
    support_base_height_m: float | None = None
    support_height_tol_m: float = 0.03
    support_base_variation_m: float | None = None
    min_observations: int = 2
    age_step_s: float = 2.0
    process_hz: float = 5.0
    status_hz: float = 5.0
    max_tiles_per_tick: int = 12
    refresh_tiles_per_tick: int = 8
    max_map_tiles: int = 1024
    depth_pending_max: int = 5
    pose_wait_s: float = 0.4
    input_lag_budget_s: float = 0.25
    pose_max_gap_s: float = 0.06
    min_valid_wheels: int = 4
    wheel_radius_m: float = 0.10356
    lateness_margin_s: float = 0.05
    gyro_gap_epoch_s: float = 0.06
    wheel_gap_invalid_s: float = 0.06
    wheel_gap_epoch_s: float = 0.30
    ls_max_condition: float = 50.0
    max_lateral_speed_m_s: float = 0.2
    max_slip_wheels: int = 0                 # r13 wheel consensus; 0 = off (r12)
    slip_residual_m_s: float = 0.10
    slip_epoch_s: float = 1.0
    max_stream_skew_s: float = 0.5
    stationary_command_turns_per_s: float = 0.005
    stationary_measured_turns_per_s: float = 0.02
    bias_stationary_s: float = 1.0
    bias_max_std_rad_s: float = 0.01
    bias_alpha: float = 0.2
    stamp_regression_epoch_s: float = 1.0
    # ground-plane tilt display (slope.py): gravity reference only inside a still window
    still_window_s: float = 0.5
    still_max_gyro_rad_s: float = 0.05
    still_max_accel_std: float = 0.1
    gravity_tolerance: float = 0.1
    slope_max_age_s: float = 1.0
    camera_x_m: float = 0.348
    camera_y_m: float = 0.0
    camera_z_m: float = 0.029
    camera_pitch_down_deg: float = 15.0      # L515 mount per the URDF (2026-10-19 user: as built, cannot change)
    camera_roll_deg: float = 0.0
    camera_yaw_deg: float = 0.0
    corrector: str = "identity"
    match_feature_m: float = 0.03
    match_min_features: int = 30
    match_window_m: float = 0.15
    match_window_deg: float = 3.0
    match_yaw_step_deg: float = 0.5
    match_min_overlap: int = 100
    match_recent_s: float = 30.0
    match_code_weight: float = 0.05
    match_max_step_m: float = 0.05
    match_max_step_deg: float = 1.0
    match_min_gain: float = 0.01
    match_min_peak: float = 0.005

    def __post_init__(self) -> None:
        for item in fields(self):
            name, value = item.name, getattr(self, item.name)
            if name == "corrector":
                if value not in CORRECTORS:
                    raise ValueError(f"corrector must be one of {CORRECTORS}")
                continue
            if value is None and name in ("support_base_height_m", "support_base_variation_m"):
                continue
            if isinstance(value, bool) or not isinstance(value, (int, float)):
                raise ValueError(f"{name}: number required")
            if name in _INTEGERS and not isinstance(value, int):
                raise ValueError(f"{name}: integer required")
            if not math.isfinite(value):
                raise ValueError(f"{name}: finite value required")
            low, high = _RANGES[name]
            if not low <= value <= high:
                raise ValueError(f"{name}={value} outside [{low}, {high}]")
        self._check_relations()

    def _check_relations(self) -> None:
        # Values the console reads off the wire must survive the 4-decimal encoding
        # exactly, or sender and receiver disagree on the grid (Codex Task 1 review).
        for name in ("cell_m", "age_step_s"):
            value = getattr(self, name)
            if round(value, 4) != value:
                raise ValueError(f"{name}={value} is not exact at 4 decimals (0.0001 steps)")
        if self.range_min_m >= self.range_max_m:
            raise ValueError("range_min_m must be below range_max_m")
        if self.range_min_m + self.support_band_m > self.range_max_m:
            raise ValueError("support band must lie inside the range")
        if self.wheel_gap_invalid_s >= self.wheel_gap_epoch_s:
            raise ValueError("wheel_gap_invalid_s must be below wheel_gap_epoch_s")
        needed_wait = self.lateness_margin_s + 1.0 / (2.0 * self.process_hz) + self.input_lag_budget_s
        if self.pose_wait_s + 1e-9 < needed_wait:
            raise ValueError(f"pose_wait_s {self.pose_wait_s} below the latency budget {needed_wait:.3f}")
        if self.depth_pending_max < math.ceil(self.pose_wait_s * 10.0 - 1e-9) + 1:
            raise ValueError("depth_pending_max too small for pose_wait_s at 10 Hz depth")
        if self.support_base_height_m is not None:
            var = self.support_base_variation_m
            if var is None:
                raise ValueError("support_base_height_m needs a measured support_base_variation_m")
            limit = min(self.obstacle_height_m, self.ditch_depth_m) - var
            if not var <= self.support_height_tol_m < limit:
                raise ValueError(
                    "support height check needs variation <= tol < min(obstacle, ditch) - variation "
                    f"({var} <= {self.support_height_tol_m} < {limit:.3f})"
                )

    @property
    def support_height_check_enabled(self) -> bool:
        return self.support_base_height_m is not None

    @property
    def tile_m(self) -> float:
        return self.cell_m * self.tile_cells
