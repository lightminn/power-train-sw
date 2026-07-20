"""USD 에서 추출한 로버 모델 JSON 을 동결 값객체로 읽는다.

정본은 ``urdf_and_usd/rover/rover2_diff_full.usd`` 이며, 이 모듈이 읽는
``assets/rover_model.json`` 은 ``scripts/extract_rover_from_usd.py`` 가 만든
파생물이다. CAD 가 갱신되면 그 스크립트를 다시 돌려 JSON 을 교체한다.
**이 모듈은 ``usd-core`` 에 의존하지 않는다** — 런타임은 JSON 만 읽는다.

좌표 변환(USD +Y전진/+X우측 → 섀시 +X전진/+Y좌측)은 추출 시점에 이미 적용돼
있다. ``usd_to_chassis`` 는 같은 변환을 검증·재사용하기 위해 노출한다.
**변환식은 이 모듈과 추출 스크립트에만 존재한다.**
"""
from __future__ import annotations

from dataclasses import dataclass
from functools import lru_cache
import json
import math
from pathlib import Path


ASSET_PATH = Path(__file__).resolve().parents[1] / "assets" / "rover_model.json"

# 추출 스크립트와 반드시 같은 값이어야 한다.
FORWARD_OFFSET_M = 0.31785
LATERAL_OFFSET_M = 0.20
VERTICAL_OFFSET_M = 0.2235


def usd_to_chassis(x_m: float, y_m: float, z_m: float) -> tuple[float, float, float]:
    """USD 좌표(+Y 전진/+X 우측)를 섀시 좌표(+X 전진/+Y 좌측)로 옮긴다."""
    return (
        float(y_m) - FORWARD_OFFSET_M,
        -(float(x_m) - LATERAL_OFFSET_M),
        float(z_m) + VERTICAL_OFFSET_M,
    )


@dataclass(frozen=True)
class RoverWheel:
    name: str
    x_m: float
    y_m: float
    z_m: float
    steerable: bool


@dataclass(frozen=True)
class RoverLink:
    name: str
    mass_kg: float
    com_m: tuple[float, float, float]
    diagonal_inertia: tuple[float, float, float]


@dataclass(frozen=True)
class RoverModel:
    wheels: tuple[RoverWheel, ...]
    links: tuple[RoverLink, ...]
    total_mass_kg: float
    wheel_radius_m: float
    wheel_half_width_m: float
    rocker_limit_rad: float
    bogie_limit_rad: float
    drive_torque_limit_nm: float
    steer_torque_limit_nm: float


def _joint_limit_rad(joints: dict, name: str) -> float:
    """USD RevoluteJoint 의 대칭 한계를 라디안으로. 비대칭이면 거부한다."""
    joint = joints[name]
    lower = joint["lower_deg"]
    upper = joint["upper_deg"]
    if lower is None or upper is None:
        raise ValueError(f"joint {name} must declare finite limits")
    if not math.isclose(-float(lower), float(upper), rel_tol=1e-6):
        raise ValueError(f"joint {name} limits are not symmetric: {lower}..{upper}")
    return math.radians(float(upper))


def _drive_force_nm(joints: dict, names: tuple[str, ...]) -> float:
    """여러 조인트가 같은 maxForce 를 보고해야 한다. 다르면 거부한다."""
    values = {joints[name]["max_force_nm"] for name in names}
    if len(values) != 1 or None in values:
        raise ValueError(f"inconsistent drive maxForce across {names}: {values}")
    return float(values.pop())


@lru_cache(maxsize=4)
def load_rover_model(path: str | Path | None = None) -> RoverModel:
    """추출된 JSON 을 읽어 동결 값객체를 만든다."""
    asset = Path(path) if path is not None else ASSET_PATH
    document = json.loads(asset.read_text(encoding="utf-8"))
    if document.get("schema_version") != 1:
        raise ValueError(f"unsupported rover_model schema: {document.get('schema_version')}")

    joints = document["joints"]
    rocker = _joint_limit_rad(joints, "rocker_left")
    if not math.isclose(rocker, _joint_limit_rad(joints, "rocker_right")):
        raise ValueError("left and right rocker limits disagree")
    bogie = _joint_limit_rad(joints, "bogie_left")
    if not math.isclose(bogie, _joint_limit_rad(joints, "bogie_right")):
        raise ValueError("left and right bogie limits disagree")

    wheels = tuple(
        RoverWheel(
            name=entry["name"],
            x_m=float(entry["x_m"]),
            y_m=float(entry["y_m"]),
            z_m=float(entry["z_m"]),
            steerable=bool(entry["steerable"]),
        )
        for entry in document["wheels"]
    )
    if len(wheels) != 6:
        raise ValueError(f"expected six wheels, found {len(wheels)}")

    links = tuple(
        RoverLink(
            name=entry["name"],
            mass_kg=float(entry["mass_kg"]),
            com_m=tuple(float(v) for v in entry["com_m"]),
            diagonal_inertia=tuple(float(v) for v in entry["diagonal_inertia"]),
        )
        for entry in document["bodies"]
    )

    return RoverModel(
        wheels=wheels,
        links=links,
        total_mass_kg=float(document["total_mass_kg"]),
        wheel_radius_m=float(document["wheel_radius_m"]),
        wheel_half_width_m=float(document["wheel_half_width_m"]),
        rocker_limit_rad=rocker,
        bogie_limit_rad=bogie,
        drive_torque_limit_nm=_drive_force_nm(
            joints,
            (
                "wheel_front_left", "wheel_front_right",
                "wheel_center_left", "wheel_center_right",
                "wheel_rear_left", "wheel_rear_right",
            ),
        ),
        steer_torque_limit_nm=_drive_force_nm(
            joints,
            (
                "steer_front_left", "steer_front_right",
                "steer_rear_left", "steer_rear_right",
            ),
        ),
    )


__all__ = (
    "ASSET_PATH",
    "RoverLink",
    "RoverModel",
    "RoverWheel",
    "load_rover_model",
    "usd_to_chassis",
)
