"""Headless, deterministic value sensors for the MuJoCo fast plant."""
from __future__ import annotations

import math

import mujoco
import numpy as np

from powertrain_autonomy.terrain.depth_quality import CameraIntrinsics
from powertrain_ros.state_estimation import ImuSample, WheelSample, WheelValue

from ..fixtures import DepthFrame, GroundTruthFrame, _is_dropped
from ..scenario import PRNG_ALGORITHM, Scenario
from .model_builder import CAMERA_PITCH_DOWN_RAD
from .plant import MujocoFastPlant


MAX_VALID_DEPTH_M = 6.0
# MuJoCo mj_multiRay 는 cutoff 로 geom 을 "앵커점(geom pos)까지의 거리" 기준
# 프루닝한다(3.10.0 실측). 무한 plane 인 lower_floor 는 앵커가 월드 원점이라,
# 카메라가 원점에서 cutoff 이상 멀어지는 순간 컷오프-이내 바닥 히트까지 통째로
# 사라진다 — 훈련 트랙 ~6 m 지점 전 가족 영구 정지(6 m 벽)의 근본 원인.
# 컷오프는 프루닝이 일어나지 않는 값으로 두고, 센서 사거리는 _ray_depth_m 의
# hit 마스크(MAX_VALID_DEPTH_M)만이 정의한다. 스펙:
# docs/superpowers/specs/2026-07-21-sim-depth-floor-pruning-6m-wall-design.md
RAY_PRUNING_CUTOFF_M = 1.0e6


class FastSensorSuite:
    """Sample MuJoCo state using the part-one stream value contracts."""

    def __init__(self, scenario: Scenario, plant: MujocoFastPlant) -> None:
        if scenario.prng.algorithm != PRNG_ALGORITHM:
            raise ValueError(f"fast sensors require {PRNG_ALGORITHM}")
        self.scenario = scenario
        self.plant = plant
        self._rng = np.random.Generator(np.random.PCG64(scenario.prng.seed))
        self._depth_directions_body = self._camera_directions_body()
        # 실제 L515 depth 프레임은 ray 거리(range)가 아니라 광축 Z를 담는다.
        # 픽셀별 cos(광축, ray) 를 미리 계산해 range→Z 변환에 쓴다.
        cosine = math.cos(CAMERA_PITCH_DOWN_RAD)
        sine = math.sin(CAMERA_PITCH_DOWN_RAD)
        optical_forward = np.array((cosine, 0.0, -sine), dtype=float)
        self._depth_axial_cos = self._depth_directions_body @ optical_forward

    def _time(self, index: int) -> tuple[float, float]:
        if isinstance(index, bool) or not isinstance(index, int) or not (
            0 <= index < self.scenario.clock.sample_count
        ):
            raise ValueError("sample index is outside the scenario clock")
        elapsed_s = index * self.scenario.clock.dt_s
        return self.scenario.clock.start_s + elapsed_s, elapsed_s

    @staticmethod
    def _scheduled(index: int, config) -> bool:
        return index % int(config["sample_every_n_steps"]) == 0

    def sample_wheel(self, index: int) -> WheelSample | None:
        stamp_s, elapsed_s = self._time(index)
        config = self.scenario.sensors["wheel_states"]
        if not self._scheduled(index, config) or _is_dropped(
            self.scenario, "wheel", elapsed_s
        ):
            return None
        measured = self.plant.measured_wheel_turns_per_s()
        steering = self.plant.steering_angles_deg()
        noise_std = float(config["noise_std_turns_per_s"])
        commands = (
            self.plant.last_solve_result.wheels
            if self.plant.last_solve_result is not None
            else {}
        )
        values = []
        for wheel_name in config["wheel_names"]:
            value = measured[str(wheel_name)]
            if noise_std:
                value += float(self._rng.normal(0.0, noise_std))
            for fault in self.scenario.faults["wheel_slip"]:
                if (
                    fault["wheel"] == wheel_name
                    and fault["start_s"] <= elapsed_s < fault["end_s"]
                ):
                    value *= float(fault["measurement_scale"])
            command = commands.get(str(wheel_name))
            values.append(
                WheelValue(
                    name=str(wheel_name),
                    command_turns_per_s=(
                        0.0 if command is None else command.drive_turns_per_s
                    ),
                    measured_turns_per_s=value,
                    steer_deg=steering[str(wheel_name)],
                    stale=False,
                )
            )
        return WheelSample(stamp_s=stamp_s, wheels=tuple(values))

    def sample_imu(self, index: int) -> ImuSample | None:
        stamp_s, elapsed_s = self._time(index)
        config = self.scenario.sensors["imu"]
        if not self._scheduled(index, config) or _is_dropped(
            self.scenario, "imu", elapsed_s
        ):
            return None
        gyro, acceleration = self.plant.imu_raw()
        gyro_bias = np.asarray(config["gyro_bias_rad_s"], dtype=float)
        accel_bias = np.asarray(config["accel_bias_m_s2"], dtype=float)
        gyro_values = np.asarray(gyro, dtype=float) + gyro_bias
        accel_values = np.asarray(acceleration, dtype=float) + accel_bias
        gyro_std = float(config["gyro_noise_std_rad_s"])
        accel_std = float(config["accel_noise_std_m_s2"])
        if gyro_std:
            gyro_values += self._rng.normal(0.0, gyro_std, size=3)
        if accel_std:
            accel_values += self._rng.normal(0.0, accel_std, size=3)
        return ImuSample(
            stamp_s=stamp_s,
            gyro_x_rad_s=float(gyro_values[0]),
            gyro_y_rad_s=float(gyro_values[1]),
            gyro_z_rad_s=float(gyro_values[2]),
            accel_x_m_s2=float(accel_values[0]),
            accel_y_m_s2=float(accel_values[1]),
            accel_z_m_s2=float(accel_values[2]),
        )

    def _camera_directions_body(self) -> np.ndarray:
        config = self.scenario.sensors["depth"]
        height, width = (int(value) for value in config["shape_px"])
        intrinsics = config["intrinsics_px"]
        rows, cols = np.indices((height, width), dtype=float)
        pixel_x = (cols - float(intrinsics["cx"])) / float(intrinsics["fx"])
        pixel_y = (rows - float(intrinsics["cy"])) / float(intrinsics["fy"])

        cosine = math.cos(CAMERA_PITCH_DOWN_RAD)
        sine = math.sin(CAMERA_PITCH_DOWN_RAD)
        optical_forward = np.array((cosine, 0.0, -sine), dtype=float)
        optical_right = np.array((0.0, -1.0, 0.0), dtype=float)
        optical_down = np.cross(optical_forward, optical_right)
        directions = (
            optical_forward[None, None, :]
            + pixel_x[..., None] * optical_right[None, None, :]
            + pixel_y[..., None] * optical_down[None, None, :]
        )
        directions /= np.linalg.norm(directions, axis=2, keepdims=True)
        return directions.reshape((-1, 3))

    def _ray_depth_m(self) -> np.ndarray:
        config = self.scenario.sensors["depth"]
        height, width = (int(value) for value in config["shape_px"])
        body_matrix = np.asarray(
            self.plant.data.xmat[self.plant.base_body_id],
            dtype=float,
        ).reshape(3, 3)
        directions_world = self._depth_directions_body @ body_matrix.T
        origin = np.asarray(
            self.plant.data.site_xpos[self.plant.depth_site_id],
            dtype=float,
        )
        ray_count = int(directions_world.shape[0])
        geom_ids = np.full(ray_count, -1, dtype=np.int32)
        distances = np.full(ray_count, -1.0, dtype=float)
        geom_group = np.array((1, 0, 0, 0, 0, 0), dtype=np.uint8)
        mujoco.mj_multiRay(
            self.plant.model,
            self.plant.data,
            origin,
            np.ascontiguousarray(directions_world).ravel(),
            geom_group,
            True,
            self.plant.base_body_id,
            geom_ids,
            distances,
            None,
            ray_count,
            RAY_PRUNING_CUTOFF_M,
        )
        hit = (distances >= 0.0) & (distances <= MAX_VALID_DEPTH_M)
        axial = np.where(hit, distances * self._depth_axial_cos, 0.0)
        return axial.reshape((height, width))

    def sample_depth(self, index: int) -> DepthFrame | None:
        stamp_s, elapsed_s = self._time(index)
        config = self.scenario.sensors["depth"]
        if not self._scheduled(index, config) or _is_dropped(
            self.scenario, "depth", elapsed_s
        ):
            return None
        depth_m = self._ray_depth_m()
        valid = depth_m > 0.0
        noise_std = float(config["noise_std_m"])
        if noise_std and np.any(valid):
            depth_m[valid] += self._rng.normal(0.0, noise_std, size=np.count_nonzero(valid))

        hole_mask = np.zeros(depth_m.shape, dtype=bool)
        for fault in self.scenario.faults["depth_holes"]:
            if fault["start_s"] <= elapsed_s < fault["end_s"]:
                row_start, row_stop = (int(value) for value in fault["rows"])
                col_start, col_stop = (int(value) for value in fault["cols"])
                hole_mask[row_start:row_stop, col_start:col_stop] = True
        for fault in self.scenario.faults["depth_spikes"]:
            if fault["start_s"] <= elapsed_s < fault["end_s"]:
                row = int(fault["row"])
                col = int(fault["col"])
                depth_m[row, col] += float(fault["offset_m"])

        scale_m = float(config["depth_scale_m"])
        raw = np.rint(
            np.clip(depth_m / scale_m, 0.0, np.iinfo(np.uint16).max)
        ).astype(np.uint16)
        raw[hole_mask] = 0
        intrinsics_config = config["intrinsics_px"]
        raw.setflags(write=False)
        return DepthFrame(
            stamp_s=stamp_s,
            depth_roi=raw,
            depth_scale_m=scale_m,
            intrinsics=CameraIntrinsics(
                fx=float(intrinsics_config["fx"]),
                fy=float(intrinsics_config["fy"]),
                cx=float(intrinsics_config["cx"]),
                cy=float(intrinsics_config["cy"]),
            ),
            frame_id=self.scenario.frames["depth"],
        )

    def sample_ground_truth(self, index: int) -> GroundTruthFrame:
        stamp_s, _ = self._time(index)
        position, yaw, bank, linear_speed, yaw_rate = self.plant.ground_truth_pose()
        return GroundTruthFrame(
            stamp_s=stamp_s,
            x_m=float(position[0]),
            y_m=float(position[1]),
            z_m=float(position[2]),
            yaw_rad=yaw,
            bank_rad=bank,
            linear_speed_m_s=linear_speed,
            yaw_rate_rad_s=yaw_rate,
        )


__all__ = ("FastSensorSuite", "MAX_VALID_DEPTH_M")
