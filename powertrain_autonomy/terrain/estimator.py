"""ROS-free, fixed-shape NumPy terrain path estimation."""
from __future__ import annotations

import dataclasses
from dataclasses import dataclass
import math

import numpy as np

from chassis.kinematics import default_geometry

from .depth_quality import (
    CameraIntrinsics,
    DepthQualityConfig,
    DepthQualitySnapshot,
    analyze_depth_quality,
)
from .grid import ElevationGrid, build_elevation_grid, empty_grid, warp_and_fuse_grid


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
    depth_shape_px: tuple[int, int] = (60, 80)
    roi_rows: tuple[int, int] = (0, 60)
    roi_cols: tuple[int, int] = (0, 80)
    stride: int = 1
    quality_tile_shape_px: tuple[int, int] = (15, 20)
    grid_resolution_m: float = 0.05
    grid_x_range_m: tuple[float, float] = (0.3, 4.0)
    grid_y_range_m: tuple[float, float] = (-1.5, 1.5)
    max_frame_age_s: float = 0.25
    history_horizon_s: float = 1.5
    wheel_half_width_m: float = 0.035
    footprint_uncertainty_m: float = 0.05
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
        if (
            len(self.depth_shape_px) != 2
            or any(isinstance(value, bool) or not isinstance(value, int) or value < 3 for value in self.depth_shape_px)
        ):
            raise ValueError("depth_shape_px must contain two integers >= 3")
        for bounds, size, name in (
            (self.roi_rows, self.depth_shape_px[0], "roi_rows"),
            (self.roi_cols, self.depth_shape_px[1], "roi_cols"),
        ):
            if len(bounds) != 2 or not (0 <= bounds[0] < bounds[1] <= size):
                raise ValueError(f"{name} must be ordered within depth_shape_px")
        if isinstance(self.stride, bool) or not isinstance(self.stride, int) or self.stride < 1:
            raise ValueError("stride must be a positive integer")
        sampled_shape = (
            len(range(self.roi_rows[0], self.roi_rows[1], self.stride)),
            len(range(self.roi_cols[0], self.roi_cols[1], self.stride)),
        )
        if len(self.quality_tile_shape_px) != 2 or any(
            tile < 3 or sampled % tile
            for sampled, tile in zip(sampled_shape, self.quality_tile_shape_px)
        ):
            raise ValueError("quality tiles must divide the fixed sampled ROI and be >= 3")
        finite = (
            self.grid_resolution_m,
            *self.grid_x_range_m,
            *self.grid_y_range_m,
            self.max_frame_age_s,
            self.history_horizon_s,
            self.wheel_half_width_m,
            self.footprint_uncertainty_m,
            self.min_depth_m,
            self.max_depth_m,
            self.max_support_step_m,
            self.drop_height_m,
            self.obstacle_height_m,
            self.drop_reference_radius_m,
            self.seed_max_x_m,
            self.seed_half_width_m,
            *self.path_x_range_m,
        )
        if not all(math.isfinite(value) for value in finite):
            raise ValueError("terrain estimator thresholds must be finite")
        if self.grid_resolution_m <= 0.0 or not (
            self.grid_x_range_m[0] < self.grid_x_range_m[1]
            and self.grid_y_range_m[0] < self.grid_y_range_m[1]
            and self.path_x_range_m[0] < self.path_x_range_m[1]
        ):
            raise ValueError("grid and path ranges must be positive and ordered")
        if not 0.0 < self.min_depth_m < self.max_depth_m:
            raise ValueError("depth range must be positive and ordered")
        if min(
            self.max_frame_age_s,
            self.history_horizon_s,
            self.max_support_step_m,
            self.drop_height_m,
            self.obstacle_height_m,
            self.drop_reference_radius_m,
            self.seed_max_x_m,
            self.seed_half_width_m,
        ) <= 0.0:
            raise ValueError("terrain time, support, and classification thresholds must be positive")
        if min(self.wheel_half_width_m, self.footprint_uncertainty_m) < 0.0:
            raise ValueError("footprint widths must be nonnegative")
        if isinstance(self.min_path_rows, bool) or not isinstance(self.min_path_rows, int) or self.min_path_rows < 2:
            raise ValueError("min_path_rows must be an integer >= 2")


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

    def _reset(self, *, clear_quality: bool) -> None:
        self._grid = empty_grid(self.grid_shape)
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
        if not temporal_rejects.intersection(frame_quality.reject_reasons):
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
        base = camera @ self._camera_to_base_rotation(extrinsic).T
        base += np.array((extrinsic.x_m, extrinsic.y_m, extrinsic.z_m))
        gravity_rotation = self._rotation_y(tilt.pitch_rad) @ self._rotation_x(tilt.roll_rad)
        return base @ gravity_rotation.T

    def _point_side_evidence(
        self,
        points: np.ndarray,
        classification_mask: np.ndarray,
        grid: ElevationGrid,
    ) -> tuple[bool, bool]:
        cfg = self.config
        support = grid.support_mask
        row_has_support = support.any(axis=1)
        if not np.any(row_has_support):
            return False, False
        y_centres = cfg.grid_y_range_m[0] + (np.arange(self.grid_shape[1]) + 0.5) * cfg.grid_resolution_m
        first_col = np.argmax(support, axis=1)
        last_col = self.grid_shape[1] - 1 - np.argmax(support[:, ::-1], axis=1)
        masked_height = np.where(support, grid.height_m, np.nan)
        reference = np.full(self.grid_shape[0], np.nan)
        reference[row_has_support] = np.nanmedian(
            masked_height[row_has_support], axis=1
        )
        # 원거리 바닥은 support 가 없는 행(타일 품질로 제외된 원거리)에서만
        # 보이기도 한다 — 기준높이·측면 경계를 support 있는 행에서 보간·외삽한다.
        supported_indices = np.flatnonzero(row_has_support)
        all_indices = np.arange(self.grid_shape[0], dtype=float)
        reference_full = np.interp(
            all_indices,
            supported_indices.astype(float),
            reference[supported_indices],
        )
        left_bound = np.where(
            row_has_support,
            y_centres[last_col],
            float(np.max(y_centres[last_col[supported_indices]])),
        )
        right_bound = np.where(
            row_has_support,
            y_centres[first_col],
            float(np.min(y_centres[first_col[supported_indices]])),
        )
        flat_points = points.reshape((-1, 3))
        flat_mask = classification_mask.reshape(-1)
        finite = np.all(np.isfinite(flat_points), axis=1) & flat_mask
        rows = np.floor(
            (flat_points[:, 0] - cfg.grid_x_range_m[0]) / cfg.grid_resolution_m
        ).astype(int)
        usable = finite & (rows >= 0) & (rows < self.grid_shape[0])
        if not np.any(usable):
            return False, False
        rows = rows[usable]
        y = flat_points[usable, 1]
        z = flat_points[usable, 2]
        dropped = z < reference_full[rows] - cfg.drop_height_m
        left = dropped & (y > left_bound[rows] + cfg.grid_resolution_m)
        right = dropped & (y < right_bound[rows] - cfg.grid_resolution_m)
        # 스파이크 한두 픽셀이 경계 증거를 조작하지 못하게 최소 점수를 요구한다.
        minimum_points = 8
        return (
            bool(np.count_nonzero(left) >= minimum_points),
            bool(np.count_nonzero(right) >= minimum_points),
        )

    def _fov_limits(
        self,
        grid: ElevationGrid,
        *,
        frame_intrinsics: CameraIntrinsics,
        extrinsic: BaseToCameraExtrinsic,
        tilt: BodyTilt,
    ) -> tuple[np.ndarray, np.ndarray]:
        """행별 관측 한계(해석적): FOV 원뿔이 그 행의 support 높이 평면과 만나는
        측면 y 범위. 점 데이터가 아니라 카메라 기하에서 직접 계산하므로 노이즈·
        가림·틸트에 무관하게 "이 행에서 어디까지 볼 수 있었는가"를 준다."""
        cfg = self.config
        rotation = (
            self._rotation_y(tilt.pitch_rad) @ self._rotation_x(tilt.roll_rad)
        ) @ self._camera_to_base_rotation(extrinsic)
        origin = (
            self._rotation_y(tilt.pitch_rad) @ self._rotation_x(tilt.roll_rad)
        ) @ np.array((extrinsic.x_m, extrinsic.y_m, extrinsic.z_m))
        height, width = cfg.depth_shape_px
        c_min = (0.0 - frame_intrinsics.cx) / frame_intrinsics.fx
        c_max = (width - 1.0 - frame_intrinsics.cx) / frame_intrinsics.fx
        x_centres = cfg.grid_x_range_m[0] + (np.arange(self.grid_shape[0]) + 0.5) * cfg.grid_resolution_m
        masked_height = np.where(grid.support_mask, grid.height_m, np.nan)
        left_limit = np.full(self.grid_shape[0], np.nan)
        right_limit = np.full(self.grid_shape[0], np.nan)
        basis_y = rotation.T @ np.array((0.0, 1.0, 0.0))
        for x_index in range(self.grid_shape[0]):
            row_heights = masked_height[x_index]
            if not np.any(np.isfinite(row_heights)):
                continue
            row_height = float(np.nanmedian(row_heights))
            anchor = rotation.T @ (
                np.array((x_centres[x_index], 0.0, row_height)) - origin
            )
            candidates = []
            for c in (c_min, c_max):
                denominator = basis_y[0] - c * basis_y[2]
                if abs(denominator) < 1e-9:
                    continue
                y = (c * anchor[2] - anchor[0]) / denominator
                forward = anchor[2] + y * basis_y[2]
                if forward > 0.0:
                    candidates.append(float(y))
            if len(candidates) == 2:
                right_limit[x_index] = min(candidates)
                left_limit[x_index] = max(candidates)
        return left_limit, right_limit

    def _summarize(
        self,
        grid: ElevationGrid,
        *,
        stamp_s: float,
        frame_confidence: float,
        reasons,
        carried_count: int,
        odometry_residual_m: float,
        left_limit_y: np.ndarray,
        right_limit_y: np.ndarray,
        left_floor_seen: bool,
        right_floor_seen: bool,
    ):
        cfg = self.config
        x_centres = cfg.grid_x_range_m[0] + (np.arange(self.grid_shape[0]) + 0.5) * cfg.grid_resolution_m
        y_centres = cfg.grid_y_range_m[0] + (np.arange(self.grid_shape[1]) + 0.5) * cfg.grid_resolution_m
        lookahead = (x_centres >= cfg.path_x_range_m[0]) & (x_centres <= cfg.path_x_range_m[1])
        footprint_half = max(abs(float(wheel.y)) for wheel in self.geometry.wheels) + cfg.wheel_half_width_m
        erosion_half = footprint_half + cfg.footprint_uncertainty_m + odometry_residual_m
        if np.any(grid.obstacle_mask[lookahead]):
            return self._reject(
                stamp_s,
                "obstacle_blocks_path",
                degradation=(*reasons, "local_obstacle"),
            )
        # 아래 바닥 증거는 가림 기하 때문에 가장자리 행보다 계통적으로 전방에
        # 맺힌다(에지 위를 넘어간 ray 가 더 큰 x 에서 바닥에 닿음). 그래서 측면별
        # 증거는 프레임 전역으로 판정하고, 행별 support 가장자리가 "실제 트랙
        # 에지"인지는 그 행의 FOV 관측 한계(이미지 경계 열의 역투영) 안쪽인지로
        # 구분한다. FOV 로 잘린 행은 에지 판단 불가 — 실제-에지 행들에서 얻은
        # corridor 를 연속성 가정으로 상속받아 침식·커버리지 검사만 한다.
        floor_columns = np.any(grid.lower_floor_mask, axis=0)
        margin = 1.5 * cfg.grid_resolution_m
        candidate_rows = []
        for x_index in range(self.grid_shape[0]):
            support_indices = np.flatnonzero(grid.support_mask[x_index])
            if support_indices.size < 2:
                continue
            right_index = int(support_indices[0])
            left_index = int(support_indices[-1])
            right_edge = y_centres[right_index] - 0.5 * cfg.grid_resolution_m
            left_edge = y_centres[left_index] + 0.5 * cfg.grid_resolution_m
            right_limit = right_limit_y[x_index]
            left_limit = left_limit_y[x_index]
            # 행별 realness: 측면 낙하 증거(pre-grid 점 또는 in-grid 바닥 열)가
            # 있고, 이 행의 관측 범위가 support 가장자리보다 확실히 바깥까지
            # 이어질 때(그 너머가 관측됐는데 support 가 아님)만 실제 에지로
            # 인정한다. 가장자리가 관측 범위 끝과 붙어 있으면 FOV/관측 한계
            # (에지 판단 불가), 관측이 없는 행(NaN)도 판단 불가.
            right_evidence = right_floor_seen or bool(np.any(floor_columns[:right_index]))
            left_evidence = left_floor_seen or bool(np.any(floor_columns[left_index + 1 :]))
            right_real = (
                right_evidence
                and math.isfinite(right_limit)
                and right_limit < right_edge - margin
            )
            left_real = (
                left_evidence
                and math.isfinite(left_limit)
                and left_limit > left_edge + margin
            )
            candidate_rows.append(
                (x_index, right_edge, left_edge, right_real, left_real)
            )
        right_observed = any(bool(row[3]) for row in candidate_rows)
        left_observed = any(bool(row[4]) for row in candidate_rows)
        boundary_degradation = list(reasons)
        if right_observed:
            boundary_degradation.append("right_drop_boundary")
        if left_observed:
            boundary_degradation.append("left_drop_boundary")
        if not (left_observed and right_observed):
            if not np.any(grid.support_mask[lookahead]):
                reason = "no_connected_support"
            else:
                reason = "drop_boundaries_unobserved"
            return self._reject(stamp_s, reason, degradation=boundary_degradation)
        right_corridor = float(
            np.median([row[1] for row in candidate_rows if row[3]])
        )
        left_corridor = float(
            np.median([row[2] for row in candidate_rows if row[4]])
        )
        safe_right_corridor = right_corridor + erosion_half
        safe_left_corridor = left_corridor - erosion_half
        if safe_right_corridor > safe_left_corridor:
            return self._reject(
                stamp_s,
                "erosion_empty",
                degradation=(*boundary_degradation, "drop_boundary"),
            )
        # 국소 choke: corridor 안쪽에서 아래 바닥이 검출되면(가림 변위 때문에
        # choke 의 바닥은 corridor 내부 열에 맺힌다) 트랙이 국소적으로 좁아진
        # 것이다 — 넓은 행으로 우회하지 않고 프레임 전체를 기각한다. 데이터
        # 결손으로 support 가 파편화된 행은 바닥이 없으므로 여기 걸리지 않는다.
        inner_columns = (y_centres > right_corridor) & (y_centres < left_corridor)
        if np.any(grid.lower_floor_mask[:, inner_columns]):
            return self._reject(
                stamp_s,
                "erosion_empty",
                degradation=(*boundary_degradation, "drop_boundary"),
            )
        # corridor 일관성 필터: "실제 에지"가 corridor 에서 2셀 넘게 벗어나면
        # 데이터 결손 파편(관측 갭)일 가능성이 높다 — 에지 판단을 철회하고
        # corridor 를 상속시킨다(트랙 경계는 공간적으로 연속이라는 가정).
        consistency_band = 2.0 * cfg.grid_resolution_m
        candidate_rows = [
            (
                x_index,
                right_edge,
                left_edge,
                right_real and abs(right_edge - right_corridor) <= consistency_band,
                left_real and abs(left_edge - left_corridor) <= consistency_band,
            )
            for x_index, right_edge, left_edge, right_real, left_real in candidate_rows
        ]
        rows = []
        coverage_slack = cfg.grid_resolution_m
        for x_index, right_edge, left_edge, right_real, left_real in candidate_rows:
            if not lookahead[x_index]:
                continue
            effective_right = right_edge if right_real else right_corridor
            effective_left = left_edge if left_real else left_corridor
            safe_right = effective_right + erosion_half
            safe_left = effective_left - erosion_half
            covers = (
                right_edge <= safe_right + coverage_slack
                and left_edge >= safe_left - coverage_slack
            )
            if safe_right <= safe_left and covers:
                # 행 중심 추정: 양쪽 실제 에지면 그 중점, 한쪽만 실제면 corridor
                # 폭 prior 로 복원(5 cm 격자에서 heading 기울기의 x-스팬을 확보),
                # 둘 다 상속이면 corridor 중심(기울기 기여 없음 표시 direct=0).
                corridor_width = left_corridor - right_corridor
                if right_real and left_real:
                    centre, direct = 0.5 * (right_edge + left_edge), 1.0
                elif right_real:
                    centre, direct = right_edge + 0.5 * corridor_width, 1.0
                elif left_real:
                    centre, direct = left_edge - 0.5 * corridor_width, 1.0
                else:
                    centre, direct = 0.5 * (right_corridor + left_corridor), 0.0
                rows.append(
                    (x_index, effective_right, effective_left, safe_right, safe_left, centre, direct)
                )
        if rows:
            runs: list[list[tuple[float, ...]]] = [[rows[0]]]
            for row in rows[1:]:
                if int(row[0]) == int(runs[-1][-1][0]) + 1:
                    runs[-1].append(row)
                else:
                    runs.append([row])
            rows = max(runs, key=lambda run: (len(run), -int(run[0][0])))
        if len(rows) < cfg.min_path_rows:
            if not np.any(grid.support_mask[lookahead]):
                reason = "no_connected_support"
            else:
                reason = "erosion_empty"
            return self._reject(stamp_s, reason, degradation=boundary_degradation)

        row_values = np.asarray(rows, dtype=float)
        row_indices = row_values[:, 0].astype(int)
        # offset·heading 은 corridor 상속 행(상수 중심, direct=0)이 기울기를
        # 오염시키지 않도록 실측 기반 행(direct=1)만으로 계산하고, 부족하면
        # 전체 행으로 후퇴한다.
        direct_rows = row_values[row_values[:, 6] > 0.5]
        basis = direct_rows if direct_rows.shape[0] >= 2 else row_values
        basis_centres = basis[:, 5]
        path_offset = float(np.median(basis_centres))
        basis_x = x_centres[basis[:, 0].astype(int)]
        if np.unique(basis_x).size >= 2:
            slope = float(np.polyfit(basis_x, basis_centres, 1)[0])
        else:
            slope = 0.0
        heading = math.atan(slope)
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
        row_score = min(1.0, len(rows) / max(cfg.min_path_rows * 2.0, 1.0))
        confidence = float(np.clip(0.55 * local_confidence + 0.20 * frame_confidence + 0.25 * row_score, 0.0, 1.0))
        degradation = list(boundary_degradation)
        if carried_count:
            degradation.append("odometry_carried")
        if left_observed and right_observed:
            degradation.append("drop_boundary")
        if np.any(grid.obstacle_mask):
            degradation.append("local_obstacle")
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

        sampled_rows, sampled_cols = np.meshgrid(row_indices, col_indices, indexing="ij")
        depth_m = depth.astype(float, copy=False) * frame.depth_scale_m
        points = self._deproject(
            depth_m,
            intrinsics=frame.intrinsics,
            rows_px=sampled_rows,
            cols_px=sampled_cols,
            extrinsic=extrinsic,
            tilt=tilt,
        )
        depth_observable = (
            np.isfinite(depth_m)
            & (depth_m >= self.config.min_depth_m)
            & (depth_m <= self.config.max_depth_m)
        )

        previous_grid = self._grid
        current_grid = build_elevation_grid(
            points,
            classification_mask,
            point_confidence,
            support_point_mask=support_mask,
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
        # 측면 낙하 증거(pre-grid): 뱅크/오프셋 트랙에서는 가림 기하 때문에 바닥
        # 점이 grid y-범위 밖(예: y≈1.6)에 맺힐 수 있고, 원거리 바닥 타일은 품질
        # 게이트(out_of_range 비율 등)로 통째 제외되기도 한다. 그래서 증거 판정은
        # 범위 내 원시 유효 depth 점 전체에서 하되, 스파이크 오증거를 막기 위해
        # 측면당 최소 점수를 요구한다.
        left_floor_seen, right_floor_seen = self._point_side_evidence(
            points, depth_observable, current_grid
        )

        self._grid, carried_count, odometry_residual_m = warp_and_fuse_grid(
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
        left_limit_y, right_limit_y = self._fov_limits(
            self._grid,
            frame_intrinsics=frame.intrinsics,
            extrinsic=extrinsic,
            tilt=tilt,
        )
        return self._summarize(
            self._grid,
            stamp_s=frame.stamp_s,
            frame_confidence=frame_quality.confidence,
            reasons=reasons,
            carried_count=carried_count,
            odometry_residual_m=odometry_residual_m,
            left_limit_y=left_limit_y,
            right_limit_y=right_limit_y,
            left_floor_seen=left_floor_seen,
            right_floor_seen=right_floor_seen,
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
