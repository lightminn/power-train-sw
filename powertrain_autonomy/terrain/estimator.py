"""ROS-free, fixed-shape NumPy terrain path estimation."""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
import math

import numpy as np

from chassis.kinematics import default_geometry

from ..validation import (
    require_all_finite, require_int_at_least, require_ordered, require_positive,
)
from .depth_quality import (
    CameraIntrinsics,
    DepthQualityConfig,
    DepthQualitySnapshot,
    analyze_depth_quality,
)
from .grid import ElevationGrid, build_elevation_grid, empty_grid, warp_and_fuse_grid
from .kernel import TerrainKernelConfig, build_terrain_grid_numpy


@dataclass(frozen=True)
class TerrainFrame:
    depth_roi: np.ndarray
    depth_scale_m: float
    intrinsics: CameraIntrinsics
    stamp_s: float


@dataclass(frozen=True)
class BodyTilt:
    roll_rad: float
    pitch_rad: float


@dataclass(frozen=True)
class OdometryDelta:
    dx_m: float
    dy_m: float
    dyaw_rad: float


@dataclass(frozen=True)
class _LateralReference:
    offset_m: float
    heading_rad: float
    stamp_s: float
    certified: bool
    travelled_m: float


@dataclass(frozen=True)
class BaseToCameraExtrinsic:
    x_m: float = 0.0
    y_m: float = 0.0
    # PROVISIONAL: unmeasured 0.60 m candidate; never use for production completion.
    z_m: float = 0.60
    roll_rad: float = 0.0
    mount_pitch_rad: float = 0.0
    # PROVISIONAL: unmeasured 25 degree candidate; never use for production completion.
    pitch_down_rad: float = math.radians(25.0)
    yaw_rad: float = 0.0


@dataclass(frozen=True)
class TerrainEstimatorConfig:
    # 60x80 은 5 cm 격자를 전방 ~1.4 m 너머로 채우지 못한다. 120x160 이면
    # 중심선 적합에 기여할 수 있는 전방 support 행을 충분히 유지한다.
    # 격자가 고정 shape 라 추정기 런타임은 사실상 불변(0.83 ms, 예산 5 ms).
    depth_shape_px: tuple[int, int] = (120, 160)
    roi_rows: tuple[int, int] = (0, 120)
    roi_cols: tuple[int, int] = (0, 160)
    stride: int = 1
    quality_tile_shape_px: tuple[int, int] = (30, 40)
    grid_resolution_m: float = 0.05
    grid_x_range_m: tuple[float, float] = (0.3, 4.0)
    grid_y_range_m: tuple[float, float] = (-1.5, 1.5)
    max_frame_age_s: float = 0.25
    history_horizon_s: float = 1.5
    # 타이어 트레드 반폭 — 접지 밴드 전용.
    wheel_half_width_m: float = 0.035
    # as-built v2 인휠 허브가 타이어보다 편측 13 mm 돌출한다. 외곽 반폭은
    # 48.0 mm이며 차폭 = 2 × (0.35950 + 0.048) = 0.8150 m이다.
    # 원시 STL 실측: 허브 −48.0~+87.0 mm, 타이어 ±35.0 mm.
    footprint_outboard_half_width_m: float = 0.048
    # 같은 구간의 추정치 이동은 약 0.14 m인데 실제 횡오차 이동은 약 0.02 m였다.
    # 차량 운동을 평활화하는 게 아니라 프레임별 재구성 잡음을 거르는 필터다.
    # 0.03~0.24 m/s에서 0.5 s 지연 비용은 주행거리 0.015~0.12 m다.
    path_estimate_tau_s: float = 0.5
    min_depth_m: float = 0.2
    max_depth_m: float = 6.0
    max_support_step_m: float = 0.12
    drop_height_m: float = 0.18
    obstacle_height_m: float = 0.15
    drop_reference_radius_m: float = 1.0
    seed_max_x_m: float = 1.20
    seed_half_width_m: float = 0.30
    path_x_range_m: tuple[float, float] = (0.35, 2.50)
    min_path_rows: int = 4

    def __post_init__(self) -> None:
        depth_shape_message = "depth_shape_px must contain two integers >= 3"
        if len(self.depth_shape_px) != 2:
            raise ValueError(depth_shape_message)
        for value in self.depth_shape_px:
            require_int_at_least(value, 3, depth_shape_message)
        for bounds, size, name in (
            (self.roi_rows, self.depth_shape_px[0], "roi_rows"),
            (self.roi_cols, self.depth_shape_px[1], "roi_cols"),
        ):
            if len(bounds) != 2 or not (0 <= bounds[0] < bounds[1] <= size):
                raise ValueError(f"{name} must be ordered within depth_shape_px")
        require_int_at_least(self.stride, 1, "stride must be a positive integer")
        sampled_shape = (
            len(range(self.roi_rows[0], self.roi_rows[1], self.stride)),
            len(range(self.roi_cols[0], self.roi_cols[1], self.stride)),
        )
        if len(self.quality_tile_shape_px) != 2 or any(
            tile < 3 or sampled % tile
            for sampled, tile in zip(sampled_shape, self.quality_tile_shape_px)
        ):
            raise ValueError("quality tiles must divide the fixed sampled ROI and be >= 3")
        require_positive(self.path_estimate_tau_s, "path_estimate_tau_s must be positive")
        finite = (
            self.grid_resolution_m, *self.grid_x_range_m, *self.grid_y_range_m,
            self.max_frame_age_s, self.history_horizon_s,
            self.wheel_half_width_m, self.footprint_outboard_half_width_m,
            self.min_depth_m, self.max_depth_m, self.max_support_step_m,
            self.drop_height_m, self.obstacle_height_m, self.drop_reference_radius_m,
            self.seed_max_x_m, self.seed_half_width_m,
            *self.path_x_range_m,
        )
        require_all_finite(finite, "terrain estimator thresholds must be finite")
        range_message = "grid and path ranges must be positive and ordered"
        if self.grid_resolution_m <= 0.0:
            raise ValueError(range_message)
        require_ordered(*self.grid_x_range_m, range_message)
        require_ordered(*self.grid_y_range_m, range_message)
        require_ordered(*self.path_x_range_m, range_message)
        depth_range_message = "depth range must be positive and ordered"
        require_ordered(0.0, self.min_depth_m, depth_range_message)
        require_ordered(self.min_depth_m, self.max_depth_m, depth_range_message)
        positive = (
            self.max_frame_age_s, self.history_horizon_s, self.max_support_step_m,
            self.drop_height_m, self.obstacle_height_m, self.drop_reference_radius_m,
            self.seed_max_x_m, self.seed_half_width_m,
        )
        if min(positive) <= 0.0:
            raise ValueError("terrain time, support, and classification thresholds must be positive")
        if self.wheel_half_width_m < 0.0:
            raise ValueError("wheel_half_width_m must be nonnegative")
        if self.footprint_outboard_half_width_m < 0.0:
            raise ValueError("footprint_outboard_half_width_m must be nonnegative")
        if self.footprint_outboard_half_width_m < self.wheel_half_width_m:
            raise ValueError(
                "footprint_outboard_half_width_m must be >= wheel_half_width_m "
                "because the hub cannot be narrower than the tire"
            )
        require_int_at_least(self.min_path_rows, 2, "min_path_rows must be an integer >= 2")


@dataclass(frozen=True)
class TerrainEstimate:
    stamp_s: float
    path_offset_m: float
    heading_error_rad: float
    left_wheel_clearance_m: float
    right_wheel_clearance_m: float
    bank_angle_rad: float
    longitudinal_slope_rad: float
    roughness_m: float
    confidence: float
    degradation_reasons: tuple[str, ...]
    reject_reasons: tuple[str, ...]
    path_available: bool


class TerrainEstimator:
    def __init__(self, config: TerrainEstimatorConfig | None = None, *, geometry=None):
        self.config = config or TerrainEstimatorConfig()
        self.geometry = geometry or default_geometry()
        self.grid_shape = (
            int(round((self.config.grid_x_range_m[1] - self.config.grid_x_range_m[0]) / self.config.grid_resolution_m)),
            int(round((self.config.grid_y_range_m[1] - self.config.grid_y_range_m[0]) / self.config.grid_resolution_m)),
        )
        expected_x = (self.config.grid_x_range_m[1] - self.config.grid_x_range_m[0]) / self.config.grid_resolution_m
        expected_y = (self.config.grid_y_range_m[1] - self.config.grid_y_range_m[0]) / self.config.grid_resolution_m
        if not math.isclose(expected_x, self.grid_shape[0], abs_tol=1e-9) or not math.isclose(
            expected_y, self.grid_shape[1], abs_tol=1e-9
        ):
            raise ValueError("grid ranges must be exact multiples of grid_resolution_m")
        self._frame_quality: DepthQualitySnapshot | None = None
        self._tile_quality: dict[tuple[int, int], DepthQualitySnapshot] = {}
        self._quality_config = DepthQualityConfig(
            min_depth_m=self.config.min_depth_m,
            max_depth_m=self.config.max_depth_m,
        )
        self._grid: ElevationGrid = empty_grid(self.grid_shape)
        self._lateral_reference: _LateralReference | None = None
        self._filtered_path_estimate: tuple[float, float] | None = None
        self._path_estimate_stamp_s: float | None = None

    def _reset(self, *, clear_quality: bool) -> None:
        self._grid = empty_grid(self.grid_shape)
        self._lateral_reference = None
        self._filtered_path_estimate = None
        self._path_estimate_stamp_s = None
        if clear_quality:
            self._frame_quality = None
            self._tile_quality.clear()

    @staticmethod
    def _rotation_x(angle: float) -> np.ndarray:
        cosine, sine = math.cos(angle), math.sin(angle)
        return np.array(((1.0, 0.0, 0.0), (0.0, cosine, -sine), (0.0, sine, cosine)))

    @staticmethod
    def _rotation_y(angle: float) -> np.ndarray:
        cosine, sine = math.cos(angle), math.sin(angle)
        return np.array(((cosine, 0.0, sine), (0.0, 1.0, 0.0), (-sine, 0.0, cosine)))

    @staticmethod
    def _rotation_z(angle: float) -> np.ndarray:
        cosine, sine = math.cos(angle), math.sin(angle)
        return np.array(((cosine, -sine, 0.0), (sine, cosine, 0.0), (0.0, 0.0, 1.0)))

    @classmethod
    def _camera_to_base_rotation(cls, extrinsic: BaseToCameraExtrinsic) -> np.ndarray:
        pitch = extrinsic.pitch_down_rad
        optical_to_mount = np.array(
            (
                (0.0, -math.sin(pitch), math.cos(pitch)),
                (-1.0, 0.0, 0.0),
                (0.0, -math.cos(pitch), -math.sin(pitch)),
            )
        )
        mount_to_base = (
            cls._rotation_z(extrinsic.yaw_rad)
            @ cls._rotation_y(extrinsic.mount_pitch_rad)
            @ cls._rotation_x(extrinsic.roll_rad)
        )
        return mount_to_base @ optical_to_mount

    @staticmethod
    def _finite_values(value, label: str) -> None:
        if not all(math.isfinite(float(item)) for item in value):
            raise ValueError(f"{label} must be finite")

    def _reject(self, stamp_s: float, *reasons: str, degradation=()) -> TerrainEstimate:
        self._filtered_path_estimate = None
        self._path_estimate_stamp_s = None
        return TerrainEstimate(
            stamp_s=float(stamp_s),
            path_offset_m=0.0,
            heading_error_rad=0.0,
            left_wheel_clearance_m=0.0,
            right_wheel_clearance_m=0.0,
            bank_angle_rad=0.0,
            longitudinal_slope_rad=0.0,
            roughness_m=0.0,
            confidence=0.0,
            degradation_reasons=tuple(dict.fromkeys(degradation)),
            reject_reasons=tuple(dict.fromkeys(reasons)),
            path_available=False,
        )

    def _transport_lateral_reference(
        self,
        reference: _LateralReference,
        *,
        odometry_delta: OdometryDelta,
    ) -> _LateralReference | None:
        # 새 상태 의존성이 아니라 grid history를 운반하는 기존 OdometryDelta
        # 의존성을 연장한다. 이전 body frame의 선을
        # nₚ·pₚ=ρₚ, nₚ=(-sin(hₚ), cos(hₚ)), ρₚ=oₚ cos(hₚ)라 두면,
        # grid와 같은 pₚ=R(dyaw)p꜀+[dx,dy]에서
        # n꜀=R(-dyaw)nₚ, ρ꜀=ρₚ-nₚ·[dx,dy]다. 따라서
        # h꜀=hₚ-dyaw, o꜀=ρ꜀/cos(h꜀)가 rigid-body 운반식이다.
        previous_heading = reference.heading_rad
        previous_normal_x = -math.sin(previous_heading)
        previous_normal_y = math.cos(previous_heading)
        previous_rho = reference.offset_m * previous_normal_y
        current_heading = math.atan2(
            math.sin(previous_heading - odometry_delta.dyaw_rad),
            math.cos(previous_heading - odometry_delta.dyaw_rad),
        )
        current_cosine = math.cos(current_heading)
        if abs(current_cosine) < 1e-9:
            return None
        current_rho = previous_rho - (
            previous_normal_x * odometry_delta.dx_m
            + previous_normal_y * odometry_delta.dy_m
        )
        return _LateralReference(
            offset_m=current_rho / current_cosine,
            heading_rad=current_heading,
            stamp_s=reference.stamp_s,
            certified=reference.certified,
            travelled_m=(
                reference.travelled_m
                + math.hypot(odometry_delta.dx_m, odometry_delta.dy_m)
                if reference.certified
                else 0.0
            ),
        )

    def _quality_and_mask(
        self,
        depth: np.ndarray,
        *,
        depth_scale_m: float,
        intrinsics: CameraIntrinsics,
        stamp_s: float,
    ):
        frame_quality = analyze_depth_quality(
            depth,
            depth_scale_m=depth_scale_m,
            intrinsics=intrinsics,
            frame_stamp_s=stamp_s,
            previous=self._frame_quality,
            config=self._quality_config,
        )
        temporal_rejects = {"no_valid_depth", "temporal_jump", "regressing_frame_stamp"}
        if not {"no_valid_depth", "regressing_frame_stamp"}.intersection(
            frame_quality.reject_reasons
        ):
            self._frame_quality = frame_quality.snapshot()
        point_confidence = np.zeros(depth.shape, dtype=float)
        support_mask = np.zeros(depth.shape, dtype=bool)
        classification_mask = np.zeros(depth.shape, dtype=bool)
        tile_reasons: list[str] = []
        tile_height, tile_width = self.config.quality_tile_shape_px
        hard_tile_reasons = {
            "no_valid_depth",
            "depth_hole",
            "temporal_jump",
            "regressing_frame_stamp",
            "isolated_spike",
            "invalid_depth",
            "out_of_range_depth",
        }
        for row in range(0, depth.shape[0], tile_height):
            for col in range(0, depth.shape[1], tile_width):
                tile = depth[row : row + tile_height, col : col + tile_width]
                tile_intrinsics = CameraIntrinsics(
                    fx=intrinsics.fx,
                    fy=intrinsics.fy,
                    cx=intrinsics.cx - col,
                    cy=intrinsics.cy - row,
                )
                key = (row // tile_height, col // tile_width)
                result = analyze_depth_quality(
                    tile,
                    depth_scale_m=depth_scale_m,
                    intrinsics=tile_intrinsics,
                    frame_stamp_s=stamp_s,
                    previous=self._tile_quality.get(key),
                    config=self._quality_config,
                )
                if math.isfinite(result.robust_depth_m) and not temporal_rejects.intersection(
                    result.reject_reasons
                ):
                    self._tile_quality[key] = result.snapshot()
                tile_reasons.extend(result.reject_reasons)
                if not hard_tile_reasons.intersection(result.reject_reasons):
                    tile_depth_m = tile.astype(float, copy=False) * depth_scale_m
                    valid_tile = (
                        np.isfinite(tile_depth_m)
                        & (tile_depth_m >= self.config.min_depth_m)
                        & (tile_depth_m <= self.config.max_depth_m)
                    )
                    classification_mask[
                        row : row + tile_height, col : col + tile_width
                    ] = valid_tile
                    # support 후보를 타일 중앙 depth 근접으로 거르면 뱅크/틸트
                    # 표면(타일 내 측면 depth 변화가 큼)이 찢어진다. 스파이크·홀은
                    # 위의 타일 hard reason 이 이미 배제하므로, 표면 분리(트랙 vs
                    # 아래 바닥)는 grid 연결성(max_support_step_m)에 맡긴다.
                    support_mask[
                        row : row + tile_height, col : col + tile_width
                    ] = valid_tile
                    point_confidence[
                        row : row + tile_height, col : col + tile_width
                    ] = np.where(valid_tile, result.confidence, 0.0)
        depth_m = depth.astype(float, copy=False) * depth_scale_m
        valid_range = (
            np.isfinite(depth_m)
            & (depth_m >= self.config.min_depth_m)
            & (depth_m <= self.config.max_depth_m)
        )
        support_mask &= valid_range & (point_confidence > 0.0)
        classification_mask &= valid_range & (point_confidence > 0.0)
        reasons = tuple(dict.fromkeys((*frame_quality.reject_reasons, *tile_reasons)))
        return frame_quality, support_mask, classification_mask, point_confidence, reasons

    def _deproject(
        self,
        depth_m: np.ndarray,
        *,
        intrinsics: CameraIntrinsics,
        rows_px: np.ndarray,
        cols_px: np.ndarray,
        extrinsic: BaseToCameraExtrinsic,
        tilt: BodyTilt,
    ) -> np.ndarray:
        camera = np.empty((*depth_m.shape, 3), dtype=float)
        camera[..., 0] = (cols_px - intrinsics.cx) * depth_m / intrinsics.fx
        camera[..., 1] = (rows_px - intrinsics.cy) * depth_m / intrinsics.fy
        camera[..., 2] = depth_m
        rotation, translation = self._projection_transform(extrinsic, tilt)
        return camera @ rotation.T + translation

    def _projection_transform(
        self,
        extrinsic: BaseToCameraExtrinsic,
        tilt: BodyTilt,
    ) -> tuple[np.ndarray, np.ndarray]:
        gravity_rotation = self._rotation_y(tilt.pitch_rad) @ self._rotation_x(
            tilt.roll_rad
        )
        rotation = gravity_rotation @ self._camera_to_base_rotation(extrinsic)
        translation = gravity_rotation @ np.array(
            (extrinsic.x_m, extrinsic.y_m, extrinsic.z_m), dtype=float
        )
        return rotation, translation

    def _summarize(
        self,
        grid: ElevationGrid,
        *,
        stamp_s: float,
        frame_confidence: float,
        reasons,
        odometry_delta: OdometryDelta | None = None,
    ):
        cfg = self.config
        if odometry_delta is None:
            odometry_delta = OdometryDelta(dx_m=0.0, dy_m=0.0, dyaw_rad=0.0)
        transported_reference = None
        if self._lateral_reference is not None:
            reference = self._lateral_reference
            reference_age_s = stamp_s - reference.stamp_s
            if reference.certified or reference_age_s <= cfg.history_horizon_s:
                transported_reference = self._transport_lateral_reference(
                    reference,
                    odometry_delta=odometry_delta,
                )
                if (
                    reference.certified
                    and transported_reference is not None
                    and transported_reference.travelled_m
                    > cfg.path_x_range_m[1] - cfg.path_x_range_m[0]
                ):
                    transported_reference = None
            # 인증된 기준선은 시간이 아니라 planning window를 주행한
            # 거리까지, 미인증 기준선은 기존 history horizon까지 운반한다.
            self._lateral_reference = transported_reference
        x_centres = cfg.grid_x_range_m[0] + (np.arange(self.grid_shape[0]) + 0.5) * cfg.grid_resolution_m
        y_centres = cfg.grid_y_range_m[0] + (np.arange(self.grid_shape[1]) + 0.5) * cfg.grid_resolution_m
        lookahead = (x_centres >= cfg.path_x_range_m[0]) & (x_centres <= cfg.path_x_range_m[1])
        footprint_half = (
            max(abs(float(wheel.y)) for wheel in self.geometry.wheels)
            + cfg.footprint_outboard_half_width_m
        )
        candidate_rows = []
        drop_bounded_row_indices = set()
        previous_support_run = None

        def is_drop_bounded(x_index, run) -> bool:
            return bool(
                np.any(grid.lower_floor_mask[x_index, : int(run[0])])
                and np.any(
                    grid.lower_floor_mask[x_index, int(run[-1]) + 1 :]
                )
            )

        for x_index in np.flatnonzero(lookahead):
            support_indices = np.flatnonzero(grid.support_mask[x_index])
            if support_indices.size == 0:
                if previous_support_run is not None:
                    break
                continue
            split_points = np.flatnonzero(np.diff(support_indices) > 1) + 1
            support_runs = np.split(support_indices, split_points)
            merged_support_runs = [support_runs[0]]
            for next_run in support_runs[1:]:
                previous_run = merged_support_runs[-1]
                previous_height = grid.height_m[x_index, previous_run[-1]]
                next_height = grid.height_m[x_index, next_run[0]]
                gap_cells = int(next_run[0]) - int(previous_run[-1]) - 1
                # 미관측 gap 양 끝의 높이 차가 support flood fill의 이웃 간
                # 1-step 허용치 이내이고 gap이 grid 3칸 이하일 때만 같은
                # 지면이다. 더 넓은 미관측 구간이나 높이 점프는 경계를
                # 확장하지 않는다.
                if (
                    gap_cells <= 3
                    and np.isfinite(previous_height)
                    and np.isfinite(next_height)
                    and abs(float(next_height - previous_height))
                    <= cfg.max_support_step_m
                ):
                    merged_support_runs[-1] = np.arange(
                        int(previous_run[0]),
                        int(next_run[-1]) + 1,
                    )
                else:
                    merged_support_runs.append(next_run)
            if previous_support_run is None:
                drop_bounded_runs = [
                    run
                    for run in merged_support_runs
                    if is_drop_bounded(x_index, run)
                ]
                selection_pool = drop_bounded_runs or merged_support_runs
                if transported_reference is None:
                    support_run = min(
                        selection_pool,
                        key=lambda run: abs(float(np.mean(y_centres[run]))),
                    )
                else:
                    seed_y = transported_reference.offset_m

                    def distance_from_seed(run) -> float:
                        right_edge = y_centres[int(run[0])] - 0.5 * cfg.grid_resolution_m
                        left_edge = y_centres[int(run[-1])] + 0.5 * cfg.grid_resolution_m
                        return max(right_edge - seed_y, seed_y - left_edge, 0.0)

                    support_run = min(selection_pool, key=distance_from_seed)
            else:
                overlapping_runs = []
                previous_first = int(previous_support_run[0])
                previous_last = int(previous_support_run[-1])
                for run in merged_support_runs:
                    overlap = min(previous_last, int(run[-1])) - max(
                        previous_first,
                        int(run[0]),
                    ) + 1
                    if overlap > 0:
                        overlapping_runs.append((overlap, run))
                if not overlapping_runs:
                    break
                drop_bounded_runs = [
                    item
                    for item in overlapping_runs
                    if is_drop_bounded(x_index, item[1])
                ]
                # overlap은 연속성 제약과 동률 해소에만 쓰고, 양쪽 바깥의
                # lower-floor가 실제 관측된 후보가 있으면 그 지면을 우선한다.
                selection_pool = drop_bounded_runs or overlapping_runs
                support_run = max(selection_pool, key=lambda item: item[0])[1]
            previous_support_run = support_run
            if support_run.size < 2:
                continue
            right_index = int(support_run[0])
            left_index = int(support_run[-1])
            right_edge = y_centres[right_index] - 0.5 * cfg.grid_resolution_m
            left_edge = y_centres[left_index] + 0.5 * cfg.grid_resolution_m
            candidate_rows.append((x_index, right_edge, left_edge))
            if is_drop_bounded(x_index, support_run):
                drop_bounded_row_indices.add(int(x_index))
        if not np.any(grid.support_mask[lookahead]):
            return self._reject(
                stamp_s,
                "no_connected_support",
                degradation=reasons,
            )

        lookahead_rows = [
            row for row in candidate_rows if lookahead[int(row[0])]
        ]
        if not any(
            right_edge <= -footprint_half and left_edge >= footprint_half
            for _, right_edge, left_edge in lookahead_rows
        ):
            return self._reject(
                stamp_s,
                "unsupported_footprint",
                degradation=reasons,
            )
        contributing_rows = [
            (*row, 0.5 * (row[1] + row[2]))
            for row in lookahead_rows
            if row[2] - row[1] >= 2.0 * footprint_half
        ]
        if len(contributing_rows) < 2:
            return self._reject(
                stamp_s,
                "centreline_unresolved",
                degradation=reasons,
            )

        row_values = np.asarray(contributing_rows, dtype=float)
        row_indices = row_values[:, 0].astype(int)
        basis_x = x_centres[row_indices]
        basis_centres = row_values[:, 3]

        def fit_centreline(x_values, centre_values):
            design = np.column_stack((np.ones(x_values.shape[0]), x_values))
            intercept, slope = np.linalg.lstsq(
                design,
                centre_values,
                rcond=None,
            )[0]
            return float(intercept), float(slope)

        intercept, slope = fit_centreline(basis_x, basis_centres)
        fitted = intercept + slope * basis_x
        keep = np.abs(basis_centres - fitted) <= 2.0 * cfg.grid_resolution_m
        if 2 <= int(np.count_nonzero(keep)) < row_values.shape[0]:
            intercept, slope = fit_centreline(basis_x[keep], basis_centres[keep])

        path_offset = float(np.median(intercept + slope * basis_x))
        heading = math.atan(slope)
        certified_row_indices = np.asarray(
            [
                row_index
                for row_index in row_indices
                if row_index in drop_bounded_row_indices
            ],
            dtype=int,
        )
        if certified_row_indices.size:
            certification_x = float(np.median(x_centres[certified_row_indices]))
            self._lateral_reference = _LateralReference(
                offset_m=intercept + slope * certification_x,
                heading_rad=heading,
                stamp_s=stamp_s,
                certified=True,
                travelled_m=0.0,
            )
        elif transported_reference is None or not transported_reference.certified:
            self._lateral_reference = _LateralReference(
                offset_m=path_offset,
                heading_rad=heading,
                stamp_s=stamp_s,
                certified=False,
                travelled_m=0.0,
            )
        left_clearance = float(np.median(row_values[:, 2] - footprint_half))
        right_clearance = float(np.median(-footprint_half - row_values[:, 1]))

        selected = np.zeros(grid.support_mask.shape, dtype=bool)
        selected[row_indices, :] = grid.support_mask[row_indices, :]
        bank_values = grid.slope_y[selected & np.isfinite(grid.slope_y)]
        longitudinal_values = grid.slope_x[selected & np.isfinite(grid.slope_x)]
        roughness_values = grid.roughness_m[selected & np.isfinite(grid.roughness_m)]
        grid_confidence = grid.confidence[selected & (grid.confidence > 0.0)]
        bank = math.atan(float(np.median(bank_values))) if bank_values.size else 0.0
        longitudinal = math.atan(float(np.median(longitudinal_values))) if longitudinal_values.size else 0.0
        roughness = float(np.median(roughness_values)) if roughness_values.size else 0.0
        local_confidence = float(np.median(grid_confidence)) if grid_confidence.size else 0.0
        row_score = min(
            1.0,
            len(contributing_rows) / max(cfg.min_path_rows * 2.0, 1.0),
        )
        confidence = float(np.clip(0.55 * local_confidence + 0.20 * frame_confidence + 0.25 * row_score, 0.0, 1.0))
        degradation = list(reasons)
        if np.any(grid.obstacle_mask):
            degradation.append("local_obstacle")
        if (
            self._filtered_path_estimate is not None
            and self._path_estimate_stamp_s is not None
        ):
            dt = stamp_s - self._path_estimate_stamp_s
            alpha = dt / (cfg.path_estimate_tau_s + dt)
            previous_offset, previous_heading = self._filtered_path_estimate
            path_offset = previous_offset + alpha * (
                path_offset - previous_offset
            )
            heading = previous_heading + alpha * (heading - previous_heading)
        self._filtered_path_estimate = (path_offset, heading)
        self._path_estimate_stamp_s = stamp_s
        return TerrainEstimate(
            stamp_s=stamp_s,
            path_offset_m=path_offset,
            heading_error_rad=heading,
            left_wheel_clearance_m=left_clearance,
            right_wheel_clearance_m=right_clearance,
            bank_angle_rad=bank,
            longitudinal_slope_rad=longitudinal,
            roughness_m=roughness,
            confidence=confidence,
            degradation_reasons=tuple(dict.fromkeys(degradation)),
            reject_reasons=(),
            path_available=True,
        )

    def update(
        self,
        frame: TerrainFrame,
        *,
        tilt: BodyTilt,
        extrinsic: BaseToCameraExtrinsic,
        odometry_delta: OdometryDelta,
        now_s: float,
    ) -> TerrainEstimate:
        self._finite_values(
            (frame.depth_scale_m, frame.stamp_s, now_s),
            "frame scale and stamps",
        )
        self._finite_values((tilt.roll_rad, tilt.pitch_rad), "body tilt")
        self._finite_values(dataclasses.astuple(extrinsic), "camera extrinsic")
        self._finite_values(dataclasses.astuple(odometry_delta), "odometry delta")
        if frame.depth_scale_m <= 0.0:
            raise ValueError("depth_scale_m must be positive")
        raw = np.asarray(frame.depth_roi)
        if raw.shape != self.config.depth_shape_px:
            raise ValueError("depth_roi shape does not match fixed depth_shape_px")
        if raw.ndim != 2 or raw.dtype != np.uint16:
            raise TypeError("depth_roi must be a two-dimensional uint16 array")
        age_s = now_s - frame.stamp_s
        if age_s < 0.0:
            self._reset(clear_quality=True)
            return self._reject(frame.stamp_s, "future_frame")
        if age_s > self.config.max_frame_age_s:
            self._reset(clear_quality=True)
            return self._reject(frame.stamp_s, "stale_frame")

        row_indices = np.arange(self.config.roi_rows[0], self.config.roi_rows[1], self.config.stride)
        col_indices = np.arange(self.config.roi_cols[0], self.config.roi_cols[1], self.config.stride)
        depth = raw[np.ix_(row_indices, col_indices)]
        quality_intrinsics = CameraIntrinsics(
            fx=frame.intrinsics.fx / self.config.stride,
            fy=frame.intrinsics.fy / self.config.stride,
            cx=(frame.intrinsics.cx - self.config.roi_cols[0]) / self.config.stride,
            cy=(frame.intrinsics.cy - self.config.roi_rows[0]) / self.config.stride,
        )
        frame_quality, support_mask, classification_mask, point_confidence, reasons = self._quality_and_mask(
            depth,
            depth_scale_m=frame.depth_scale_m,
            intrinsics=quality_intrinsics,
            stamp_s=frame.stamp_s,
        )
        fatal = {"no_valid_depth", "temporal_jump", "regressing_frame_stamp"}.intersection(
            frame_quality.reject_reasons
        )
        if fatal:
            self._reset(clear_quality=False)
            return self._reject(frame.stamp_s, *sorted(fatal), degradation=reasons)

        rotation, translation = self._projection_transform(extrinsic, tilt)
        kernel_result = build_terrain_grid_numpy(
            depth,
            point_mask=classification_mask,
            point_confidence=point_confidence,
            depth_scale_m=frame.depth_scale_m,
            fx=quality_intrinsics.fx,
            fy=quality_intrinsics.fy,
            cx=quality_intrinsics.cx,
            cy=quality_intrinsics.cy,
            rotation=rotation,
            translation_m=translation,
            stamp_s=frame.stamp_s,
            config=TerrainKernelConfig.from_estimator_config(self.config),
        )
        points = kernel_result.points_m

        previous_grid = self._grid
        current_grid = build_elevation_grid(
            points,
            classification_mask,
            point_confidence,
            support_point_mask=support_mask,
            kernel_result=kernel_result,
            stamp_s=frame.stamp_s,
            shape=self.grid_shape,
            resolution_m=self.config.grid_resolution_m,
            x_range_m=self.config.grid_x_range_m,
            y_range_m=self.config.grid_y_range_m,
            max_support_step_m=self.config.max_support_step_m,
            drop_height_m=self.config.drop_height_m,
            obstacle_height_m=self.config.obstacle_height_m,
            drop_reference_radius_m=self.config.drop_reference_radius_m,
            seed_max_x_m=self.config.seed_max_x_m,
            seed_half_width_m=self.config.seed_half_width_m,
        )
        self._grid, _, _ = warp_and_fuse_grid(
            previous_grid,
            current_grid,
            dx_m=odometry_delta.dx_m,
            dy_m=odometry_delta.dy_m,
            dyaw_rad=odometry_delta.dyaw_rad,
            current_stamp_s=frame.stamp_s,
            history_horizon_s=self.config.history_horizon_s,
            resolution_m=self.config.grid_resolution_m,
            x_range_m=self.config.grid_x_range_m,
            y_range_m=self.config.grid_y_range_m,
            max_support_step_m=self.config.max_support_step_m,
            drop_height_m=self.config.drop_height_m,
            obstacle_height_m=self.config.obstacle_height_m,
            drop_reference_radius_m=self.config.drop_reference_radius_m,
            seed_max_x_m=self.config.seed_max_x_m,
            seed_half_width_m=self.config.seed_half_width_m,
        )
        return self._summarize(
            self._grid,
            stamp_s=frame.stamp_s,
            frame_confidence=frame_quality.confidence,
            reasons=reasons,
            odometry_delta=odometry_delta,
        )


__all__ = (
    "BaseToCameraExtrinsic",
    "BodyTilt",
    "OdometryDelta",
    "TerrainEstimate",
    "TerrainEstimator",
    "TerrainEstimatorConfig",
    "TerrainFrame",
)
