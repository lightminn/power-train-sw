"""Build deterministic MJCF for one validated simulator-neutral scenario."""
from __future__ import annotations

import math
from xml.etree import ElementTree as ET

import numpy as np

from chassis.kinematics import ChassisGeometry, default_geometry

from ..scenario import Scenario
from .rover_model import RoverModel, load_rover_model


TRACK_THICKNESS_M = 0.04
TRACK_SEAM_OVERLAP_M = 0.012
TRACK_END_APRON_M = 0.65
WHEEL_CONTACT_GAP_M = 0.006
CAMERA_POSITION_BODY_M = (0.30, 0.0, 0.18)
CAMERA_PITCH_DOWN_RAD = math.radians(25.0)

# 바퀴 반폭. 정본은 rover_model.json (USD 타이어 STL 실측 70 mm 의 절반) 이지만,
# runner.py 와 powertrain_sim / powertrain_autonomy 의 여러 테스트가 이 이름을
# 모듈 상수로 import 하므로 여기서 다시 노출한다. 값을 손으로 적지 않고 모델에서
# 끌어와 출처를 한 곳으로 유지한다.
WHEEL_HALF_WIDTH_M = load_rover_model().wheel_half_width_m

# CAD 질량 배분. 인휠 구동모터 + 브래킷 + 타이어는 비스프렁이므로 바퀴 바디에
# 싣고, 나머지는 차체에 싣는다. 합계는 USD 총질량(66.9613 kg)과 일치해야 한다.
# USD prim 이름 예: motor_bracket_front_1_bl70200s_1_tire_1 (62개가 여기 걸린다).
_UNSPRUNG_KEYS = ("bl70200s", "motor_bracket", "AK45")


def _numbers(values) -> str:
    return " ".join(format(float(value), ".12g") for value in values)


_STEER_ACTUATOR_KEY = "AK45"
_STEERABLE_CORNERS = 4


def _mass_split(rover: RoverModel) -> tuple[float, float, float]:
    """(바퀴 1개, 조향 허브 1개, 차체) 질량. 합은 총질량과 정확히 같다.

    AK45 조향 서보는 실물에서 조향 너클 위에 얹혀 조향축과 함께 도므로 조향
    바디에 싣는다. CAD 합계 1.360 kg = 4 x 0.340 kg 이며 패키지 README 의
    "조향모터 0.34 kg x4" 와 일치한다.

    조향 바디를 질량 0 으로 두면 안 된다 — 조향 바디가 직접 담는 지오메트리는
    허브 구 하나뿐이고(바퀴는 자식 바디라 부모 질량에 안 잡힌다), MuJoCo 는
    관절 달린 바디의 질량이 0 이면 모델을 거부한다.
    """
    unsprung = sum(
        link.mass_kg
        for link in rover.links
        if any(key in link.name for key in _UNSPRUNG_KEYS)
    )
    steer = sum(
        link.mass_kg for link in rover.links if _STEER_ACTUATOR_KEY in link.name
    )
    body = rover.total_mass_kg - unsprung
    return (
        (unsprung - steer) / len(rover.wheels),
        steer / _STEERABLE_CORNERS,
        body,
    )


def _unit(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length <= 1e-12:
        raise ValueError("cannot normalize a zero vector")
    return vector / length


def _segment_axes(
    start: np.ndarray,
    stop: np.ndarray,
    bank_rad: float,
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    tangent = _unit(stop - start)
    lateral = np.array((-tangent[1], tangent[0], 0.0), dtype=float)
    if float(np.linalg.norm(lateral)) <= 1e-12:
        lateral = np.array((0.0, 1.0, 0.0), dtype=float)
    lateral = _unit(lateral)
    normal = _unit(np.cross(tangent, lateral))
    cosine = math.cos(bank_rad)
    sine = math.sin(bank_rad)
    banked_lateral = cosine * lateral + sine * normal
    banked_normal = -sine * lateral + cosine * normal
    return tangent, _unit(banked_lateral), _unit(banked_normal)


def _matrix_quaternion(matrix: np.ndarray) -> tuple[float, float, float, float]:
    """Convert a right-handed local-to-world rotation matrix to wxyz."""
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = (
            0.25 * scale,
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
        )
    else:
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = (
                (matrix[2, 1] - matrix[1, 2]) / scale,
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
            )
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = (
                (matrix[0, 2] - matrix[2, 0]) / scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
            )
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = (
                (matrix[1, 0] - matrix[0, 1]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
            )
    norm = math.sqrt(sum(value * value for value in quaternion))
    return tuple(float(value / norm) for value in quaternion)


def _track_geometries(worldbody: ET.Element, scenario: Scenario) -> tuple[np.ndarray, tuple[float, ...]]:
    points = np.asarray(scenario.track.centerline_m, dtype=float)
    first_normal: np.ndarray | None = None
    first_quaternion: tuple[float, ...] | None = None
    for index, (start, stop) in enumerate(zip(points, points[1:])):
        bank = 0.5 * (
            scenario.track.bank_rad[index] + scenario.track.bank_rad[index + 1]
        )
        tangent, lateral, normal = _segment_axes(start, stop, bank)
        if first_normal is None:
            first_normal = normal
            first_quaternion = _matrix_quaternion(
                np.column_stack((tangent, lateral, normal))
            )
        segment_length = float(np.linalg.norm(stop - start))
        start_extension = TRACK_END_APRON_M if index == 0 else 0.0
        stop_extension = TRACK_END_APRON_M if index == len(points) - 2 else 0.0
        centre = 0.5 * (start + stop)
        centre += 0.5 * (stop_extension - start_extension) * tangent
        centre -= 0.5 * TRACK_THICKNESS_M * normal
        half_length = 0.5 * (
            segment_length
            + start_extension
            + stop_extension
            + TRACK_SEAM_OVERLAP_M
        )
        half_width = 0.25 * (
            scenario.track.width_m[index] + scenario.track.width_m[index + 1]
        )
        friction = 0.5 * (
            scenario.track.friction_coefficient[index]
            + scenario.track.friction_coefficient[index + 1]
        )
        ET.SubElement(
            worldbody,
            "geom",
            {
                "name": f"track_segment_{index:03d}",
                "type": "box",
                "pos": _numbers(centre),
                "quat": _numbers(
                    _matrix_quaternion(np.column_stack((tangent, lateral, normal)))
                ),
                "size": _numbers((half_length, half_width, TRACK_THICKNESS_M / 2.0)),
                "friction": _numbers((friction, 0.01, 0.001)),
                "group": "0",
                "rgba": "0.32 0.34 0.36 1",
            },
        )
    assert first_normal is not None and first_quaternion is not None
    return first_normal, first_quaternion


def _wheel_body(
    parent: ET.Element,
    *,
    name: str,
    position: tuple[float, float, float],
    radius_m: float,
    half_width_m: float,
    mass_kg: float,
) -> None:
    wheel = ET.SubElement(parent, "body", {"name": f"wheel_{name}", "pos": _numbers(position)})
    ET.SubElement(
        wheel,
        "joint",
        {
            "name": f"drive_joint_{name}",
            "type": "hinge",
            "axis": "0 1 0",
            "damping": "0.08",
            "armature": "0.01",
        },
    )
    ET.SubElement(
        wheel,
        "geom",
        {
            "name": f"wheel_geom_{name}",
            "type": "cylinder",
            "size": _numbers((radius_m, half_width_m)),
            "quat": "0.707106781187 0.707106781187 0 0",
            "mass": _numbers((mass_kg,)),
            "friction": "1.2 0.02 0.002",
            "group": "1",
            "rgba": "0.08 0.08 0.09 1",
        },
    )


def _rover(
    worldbody: ET.Element,
    scenario: Scenario,
    geometry: ChassisGeometry,
    rover: RoverModel,
    initial_normal: np.ndarray,
    initial_quaternion: tuple[float, ...],
) -> None:
    wheel_mass_kg, hub_mass_kg, body_mass_kg = _mass_split(rover)
    start = np.asarray(scenario.track.centerline_m[0], dtype=float)
    root_position = start + (geometry.wheel_radius_m + WHEEL_CONTACT_GAP_M) * initial_normal
    base = ET.SubElement(
        worldbody,
        "body",
        {
            "name": "base_link",
            "pos": _numbers(root_position),
            "quat": _numbers(initial_quaternion),
        },
    )
    ET.SubElement(base, "freejoint", {"name": "root_free"})
    ET.SubElement(
        base,
        "geom",
        {
            "name": "chassis_geom",
            "type": "box",
            "pos": "0 0 0.16",
            "size": "0.38 0.24 0.075",
            "mass": _numbers((body_mass_kg,)),
            "group": "1",
            "rgba": "0.15 0.23 0.30 1",
        },
    )
    ET.SubElement(
        base,
        "site",
        {
            "name": "imu_site",
            "pos": "0.12 0 0.18",
            "size": "0.01",
            "group": "1",
        },
    )
    ET.SubElement(
        base,
        "site",
        {
            "name": "depth_site",
            "pos": _numbers(CAMERA_POSITION_BODY_M),
            "size": "0.01",
            "group": "1",
        },
    )

    for wheel in geometry.wheels:
        position = (wheel.x, wheel.y, 0.0)
        if wheel.steerable:
            steer = ET.SubElement(
                base,
                "body",
                {"name": f"steer_{wheel.name}", "pos": _numbers(position)},
            )
            ET.SubElement(
                steer,
                "joint",
                {
                    "name": f"steer_joint_{wheel.name}",
                    "type": "hinge",
                    "axis": "0 0 1",
                    "range": _numbers(
                        (
                            -math.radians(geometry.steer_limit_deg),
                            math.radians(geometry.steer_limit_deg),
                        )
                    ),
                    "limited": "true",
                    "damping": "1.0",
                    "armature": "0.02",
                },
            )
            ET.SubElement(
                steer,
                "geom",
                {
                    "name": f"steer_hub_{wheel.name}",
                    "type": "sphere",
                    "size": "0.025",
                    # AK45 조향 서보. 0 으로 두면 MuJoCo 가 "mass and inertia of
                    # moving bodies must be larger than mjMINVAL" 로 거부한다.
                    "mass": _numbers((hub_mass_kg,)),
                    "contype": "0",
                    "conaffinity": "0",
                    "group": "1",
                },
            )
            _wheel_body(
                steer,
                name=wheel.name,
                position=(0.0, 0.0, 0.0),
                radius_m=rover.wheel_radius_m,
                half_width_m=rover.wheel_half_width_m,
                mass_kg=wheel_mass_kg,
            )
        else:
            _wheel_body(
                base,
                name=wheel.name,
                position=position,
                radius_m=rover.wheel_radius_m,
                half_width_m=rover.wheel_half_width_m,
                mass_kg=wheel_mass_kg,
            )


def build_mjcf(
    scenario: Scenario,
    *,
    geometry: ChassisGeometry | None = None,
    suspension: bool = False,
) -> str:
    """Return a deterministic MJCF string for the scenario and production geometry."""
    geometry = geometry or default_geometry()
    rover = load_rover_model()
    configured = tuple(scenario.sensors["wheel_states"]["wheel_names"])
    actual = tuple(wheel.name for wheel in geometry.wheels)
    if configured != actual:
        raise ValueError("scenario wheel_names must match production geometry order")
    substeps = max(1, int(math.ceil(scenario.clock.dt_s / 0.005 - 1e-12)))
    physics_dt = scenario.clock.dt_s / substeps

    root = ET.Element("mujoco", {"model": f"fast_{scenario.scenario_id}"})
    ET.SubElement(
        root,
        "compiler",
        {"angle": "radian", "inertiafromgeom": "true", "coordinate": "local"},
    )
    ET.SubElement(
        root,
        "option",
        {
            "timestep": _numbers((physics_dt,)),
            "gravity": "0 0 -9.81",
            "integrator": "implicitfast",
            "cone": "elliptic",
            "iterations": "40",
        },
    )
    ET.SubElement(root, "size", {"njmax": "4000", "nconmax": "1000"})
    worldbody = ET.SubElement(root, "worldbody")
    ET.SubElement(
        worldbody,
        "geom",
        {
            "name": "lower_floor",
            "type": "plane",
            "pos": "0 0 0",
            "size": "50 50 0.05",
            "friction": "0.9 0.01 0.001",
            "group": "0",
            "rgba": "0.18 0.15 0.12 1",
        },
    )
    initial_normal, initial_quaternion = _track_geometries(worldbody, scenario)
    _rover(worldbody, scenario, geometry, rover, initial_normal, initial_quaternion)

    actuator = ET.SubElement(root, "actuator")
    maximum_angular_speed = geometry.drive_limit_mps / geometry.wheel_radius_m
    for wheel in geometry.wheels:
        if wheel.steerable:
            ET.SubElement(
                actuator,
                "position",
                {
                    "name": f"steer_{wheel.name}",
                    "joint": f"steer_joint_{wheel.name}",
                    "kp": "180",
                    "kv": "12",
                    "ctrllimited": "true",
                    "ctrlrange": _numbers(
                        (
                            -math.radians(geometry.steer_limit_deg),
                            math.radians(geometry.steer_limit_deg),
                        )
                    ),
                    "forcelimited": "true",
                    "forcerange": _numbers(
                        (-rover.steer_torque_limit_nm, rover.steer_torque_limit_nm)
                    ),
                },
            )
        ET.SubElement(
            actuator,
            "velocity",
            {
                "name": f"drive_{wheel.name}",
                "joint": f"drive_joint_{wheel.name}",
                "kv": "35",
                "ctrllimited": "true",
                "ctrlrange": _numbers((-maximum_angular_speed, maximum_angular_speed)),
                "forcelimited": "true",
                "forcerange": _numbers(
                    (-rover.drive_torque_limit_nm, rover.drive_torque_limit_nm)
                ),
            },
        )

    sensor = ET.SubElement(root, "sensor")
    ET.SubElement(sensor, "gyro", {"name": "imu_gyro", "site": "imu_site"})
    ET.SubElement(sensor, "accelerometer", {"name": "imu_accel", "site": "imu_site"})
    ET.SubElement(
        sensor,
        "framepos",
        {"name": "base_framepos", "objtype": "body", "objname": "base_link"},
    )
    ET.SubElement(
        sensor,
        "framequat",
        {"name": "base_framequat", "objtype": "body", "objname": "base_link"},
    )
    ET.indent(root, space="  ")
    return ET.tostring(root, encoding="unicode")


__all__ = (
    "CAMERA_PITCH_DOWN_RAD",
    "CAMERA_POSITION_BODY_M",
    "TRACK_END_APRON_M",
    "TRACK_THICKNESS_M",
    "build_mjcf",
)
