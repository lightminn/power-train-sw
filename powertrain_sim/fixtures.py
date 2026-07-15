"""Deterministic analytic sensor fixtures generated from one scenario contract."""
from __future__ import annotations

from dataclasses import dataclass
import math

import numpy as np

from chassis.kinematics import ChassisGeometry, default_geometry, solve
from powertrain_autonomy.terrain.depth_quality import CameraIntrinsics
from powertrain_ros.state_estimation import ImuSample, WheelSample, WheelValue

from .scenario import PRNG_ALGORITHM, Scenario


@dataclass(frozen=True)
class DepthFrame:
    stamp_s: float
    depth_roi: np.ndarray
    depth_scale_m: float
    intrinsics: CameraIntrinsics
    frame_id: str


@dataclass(frozen=True)
class GroundTruthFrame:
    stamp_s: float
    x_m: float
    y_m: float
    z_m: float
    yaw_rad: float
    bank_rad: float
    linear_speed_m_s: float
    yaw_rate_rad_s: float


@dataclass(frozen=True)
class FixtureStreams:
    scenario_id: str
    wheel_states: tuple[WheelSample, ...]
    imu: tuple[ImuSample, ...]
    depth: tuple[DepthFrame, ...]
    ground_truth: tuple[GroundTruthFrame, ...]


def _is_dropped(scenario: Scenario, stream: str, elapsed_s: float) -> bool:
    return any(
        fault["stream"] == stream
        and fault["start_s"] <= elapsed_s < fault["end_s"]
        for fault in scenario.faults["sensor_dropouts"]
    )


def _motion(scenario: Scenario, elapsed_s: float) -> tuple[float, float]:
    motion = scenario.motion
    profile = motion["profile"]
    if profile == "constant_speed":
        return float(motion["linear_speed_m_s"]), float(motion["yaw_rate_rad_s"])
    if profile == "pivot":
        return 0.0, float(motion["yaw_rate_rad_s"])
    remaining_s = max(0.0, scenario.clock.duration_s - elapsed_s)
    acceleration = float(motion["linear_acceleration_m_s2"])
    speed = min(
        float(motion["max_linear_speed_m_s"]),
        acceleration * elapsed_s,
        acceleration * remaining_s,
    )
    return speed, float(motion["yaw_rate_rad_s"])


def _track_profile(scenario: Scenario) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    points = np.asarray(scenario.track.centerline_m, dtype=float)
    segments = np.diff(points, axis=0)
    lengths = np.linalg.norm(segments, axis=1)
    return points, segments, np.concatenate(([0.0], np.cumsum(lengths)))


def _sample_track(
    scenario: Scenario,
    profile: tuple[np.ndarray, np.ndarray, np.ndarray],
    station_m: float,
) -> tuple[tuple[float, float, float], float, float, float]:
    points, segments, stations = profile
    station_m = min(max(float(station_m), 0.0), float(stations[-1]))
    if station_m >= stations[-1]:
        segment_index = len(segments) - 1
        fraction = 1.0
    else:
        segment_index = max(0, int(np.searchsorted(stations, station_m, side="right")) - 1)
        segment_length = stations[segment_index + 1] - stations[segment_index]
        fraction = (station_m - stations[segment_index]) / segment_length
    position = points[segment_index] + fraction * segments[segment_index]
    heading = math.atan2(segments[segment_index, 1], segments[segment_index, 0])

    def interpolate(values: tuple[float, ...]) -> float:
        return float(
            values[segment_index]
            + fraction * (values[segment_index + 1] - values[segment_index])
        )

    return (
        (float(position[0]), float(position[1]), float(position[2])),
        interpolate(scenario.track.bank_rad),
        interpolate(scenario.track.curvature_per_m),
        heading,
    )


def _yaw_offset(scenario: Scenario, elapsed_s: float) -> float:
    rate = float(scenario.motion["yaw_rate_rad_s"])
    if scenario.motion["profile"] != "pivot":
        return rate * elapsed_s
    target = float(scenario.motion["target_yaw_rad"])
    return math.copysign(min(abs(rate) * elapsed_s, abs(target)), target)


def _wheel_sample(
    scenario: Scenario,
    geometry: ChassisGeometry,
    rng: np.random.Generator,
    stamp_s: float,
    elapsed_s: float,
    linear_speed_m_s: float,
    yaw_rate_rad_s: float,
) -> WheelSample:
    commands = solve(geometry, linear_speed_m_s, yaw_rate_rad_s).wheels
    noise_std = float(
        scenario.sensors["wheel_states"]["noise_std_turns_per_s"]
    )
    values = []
    for wheel_geometry in geometry.wheels:
        command = commands[wheel_geometry.name]
        measured = command.drive_turns_per_s
        if noise_std:
            measured += float(rng.normal(0.0, noise_std))
        for fault in scenario.faults["wheel_slip"]:
            if (
                fault["wheel"] == wheel_geometry.name
                and fault["start_s"] <= elapsed_s < fault["end_s"]
            ):
                measured *= float(fault["measurement_scale"])
        values.append(
            WheelValue(
                name=wheel_geometry.name,
                command_turns_per_s=command.drive_turns_per_s,
                measured_turns_per_s=measured,
                steer_deg=command.steer_deg,
                stale=False,
            )
        )
    return WheelSample(stamp_s=stamp_s, wheels=tuple(values))


def _imu_sample(
    scenario: Scenario,
    rng: np.random.Generator,
    stamp_s: float,
    bank_rad: float,
    yaw_rate_rad_s: float,
) -> ImuSample:
    config = scenario.sensors["imu"]
    gyro_bias = tuple(float(value) for value in config["gyro_bias_rad_s"])
    accel_bias = tuple(float(value) for value in config["accel_bias_m_s2"])
    gyro_std = float(config["gyro_noise_std_rad_s"])
    accel_std = float(config["accel_noise_std_m_s2"])
    gyro_noise = (
        rng.normal(0.0, gyro_std, size=3) if gyro_std else np.zeros(3)
    )
    accel_noise = (
        rng.normal(0.0, accel_std, size=3) if accel_std else np.zeros(3)
    )
    gravity = float(config["gravity_m_s2"])
    gravity_body = (0.0, gravity * math.sin(bank_rad), gravity * math.cos(bank_rad))
    return ImuSample(
        stamp_s=stamp_s,
        gyro_x_rad_s=gyro_bias[0] + float(gyro_noise[0]),
        gyro_y_rad_s=gyro_bias[1] + float(gyro_noise[1]),
        gyro_z_rad_s=yaw_rate_rad_s + gyro_bias[2] + float(gyro_noise[2]),
        accel_x_m_s2=gravity_body[0] + accel_bias[0] + float(accel_noise[0]),
        accel_y_m_s2=gravity_body[1] + accel_bias[1] + float(accel_noise[1]),
        accel_z_m_s2=gravity_body[2] + accel_bias[2] + float(accel_noise[2]),
    )


def _depth_frame(
    scenario: Scenario,
    rng: np.random.Generator,
    stamp_s: float,
    elapsed_s: float,
    bank_rad: float,
) -> DepthFrame:
    config = scenario.sensors["depth"]
    height, width = (int(value) for value in config["shape_px"])
    depth_m = np.full((height, width), float(config["base_depth_m"]), dtype=float)

    maximum_bank = max(abs(value) for value in scenario.track.bank_rad)
    if maximum_bank > 0.0 and bank_rad != 0.0:
        span_m = float(config["bank_depth_span_m"]) * bank_rad / maximum_bank
        depth_m += np.linspace(-0.5 * span_m, 0.5 * span_m, width)[None, :]
    noise_std = float(config["noise_std_m"])
    if noise_std:
        depth_m += rng.normal(0.0, noise_std, size=depth_m.shape)

    hole_mask = np.zeros(depth_m.shape, dtype=bool)
    for fault in scenario.faults["depth_holes"]:
        if fault["start_s"] <= elapsed_s < fault["end_s"]:
            row_start, row_stop = (int(value) for value in fault["rows"])
            col_start, col_stop = (int(value) for value in fault["cols"])
            if row_stop > height or col_stop > width:
                raise ValueError("depth hole bounds exceed configured ROI shape")
            hole_mask[row_start:row_stop, col_start:col_stop] = True
    for fault in scenario.faults["depth_spikes"]:
        if fault["start_s"] <= elapsed_s < fault["end_s"]:
            row = int(fault["row"])
            col = int(fault["col"])
            if row >= height or col >= width:
                raise ValueError("depth spike coordinate exceeds configured ROI shape")
            depth_m[row, col] += float(fault["offset_m"])

    scale_m = float(config["depth_scale_m"])
    raw = np.rint(np.clip(depth_m / scale_m, 0.0, np.iinfo(np.uint16).max)).astype(
        np.uint16
    )
    raw[hole_mask] = 0
    intrinsics_config = config["intrinsics_px"]
    intrinsics = CameraIntrinsics(
        fx=float(intrinsics_config["fx"]),
        fy=float(intrinsics_config["fy"]),
        cx=float(intrinsics_config["cx"]),
        cy=float(intrinsics_config["cy"]),
    )
    raw.setflags(write=False)
    return DepthFrame(
        stamp_s=stamp_s,
        depth_roi=raw,
        depth_scale_m=scale_m,
        intrinsics=intrinsics,
        frame_id=scenario.frames["depth"],
    )


def generate_fixture(
    scenario: Scenario,
    *,
    geometry: ChassisGeometry | None = None,
) -> FixtureStreams:
    """Generate byte-stable wheel, IMU, depth, and isolated truth streams."""
    if scenario.prng.algorithm != PRNG_ALGORITHM:
        raise ValueError(f"fixture generator requires {PRNG_ALGORITHM}")
    geometry = geometry or default_geometry()
    configured_wheels = tuple(scenario.sensors["wheel_states"]["wheel_names"])
    geometry_wheels = tuple(wheel.name for wheel in geometry.wheels)
    if configured_wheels != geometry_wheels:
        raise ValueError("scenario wheel_names must match the injected geometry order")
    rng = np.random.Generator(np.random.PCG64(scenario.prng.seed))
    wheel_config = scenario.sensors["wheel_states"]
    imu_config = scenario.sensors["imu"]
    depth_config = scenario.sensors["depth"]

    wheel_samples = []
    imu_samples = []
    depth_frames = []
    ground_truth = []
    track_profile = _track_profile(scenario)
    station_m = 0.0
    previous_linear = None
    previous_yaw = None

    for index in range(scenario.clock.sample_count):
        elapsed_s = index * scenario.clock.dt_s
        stamp_s = scenario.clock.start_s + elapsed_s
        linear_speed, configured_yaw_rate = _motion(scenario, elapsed_s)
        if previous_linear is not None:
            average_linear = 0.5 * (previous_linear + linear_speed)
            station_m += abs(average_linear) * scenario.clock.dt_s
        position, bank, curvature, track_heading = _sample_track(
            scenario, track_profile, station_m
        )
        yaw_rad = math.atan2(
            math.sin(track_heading + _yaw_offset(scenario, elapsed_s)),
            math.cos(track_heading + _yaw_offset(scenario, elapsed_s)),
        )
        if previous_yaw is None:
            yaw_rate = configured_yaw_rate + curvature * linear_speed
        else:
            yaw_rate = math.atan2(
                math.sin(yaw_rad - previous_yaw),
                math.cos(yaw_rad - previous_yaw),
            ) / scenario.clock.dt_s

        if (
            index % int(wheel_config["sample_every_n_steps"]) == 0
            and not _is_dropped(scenario, "wheel", elapsed_s)
        ):
            wheel_samples.append(
                _wheel_sample(
                    scenario,
                    geometry,
                    rng,
                    stamp_s,
                    elapsed_s,
                    linear_speed,
                    yaw_rate,
                )
            )
        if (
            index % int(imu_config["sample_every_n_steps"]) == 0
            and not _is_dropped(scenario, "imu", elapsed_s)
        ):
            imu_samples.append(
                _imu_sample(scenario, rng, stamp_s, bank, yaw_rate)
            )
        if (
            index % int(depth_config["sample_every_n_steps"]) == 0
            and not _is_dropped(scenario, "depth", elapsed_s)
        ):
            depth_frames.append(
                _depth_frame(scenario, rng, stamp_s, elapsed_s, bank)
            )

        ground_truth.append(
            GroundTruthFrame(
                stamp_s=stamp_s,
                x_m=position[0],
                y_m=position[1],
                z_m=position[2],
                yaw_rad=yaw_rad,
                bank_rad=bank,
                linear_speed_m_s=linear_speed,
                yaw_rate_rad_s=yaw_rate,
            )
        )
        previous_linear = linear_speed
        previous_yaw = yaw_rad

    return FixtureStreams(
        scenario_id=scenario.scenario_id,
        wheel_states=tuple(wheel_samples),
        imu=tuple(imu_samples),
        depth=tuple(depth_frames),
        ground_truth=tuple(ground_truth),
    )
