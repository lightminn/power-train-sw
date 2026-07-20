#!/usr/bin/env python3
"""USD 정본 로버 모델을 시뮬용 JSON 으로 1 회 추출한다.

정본: urdf_and_usd/rover/rover2_diff_full.usd (+ configuration/ 페이로드).
USD 는 URDF 보다 최신이며 디프바 링키지·조인트 한계·모터 DriveAPI 한계를
포함한다. URDF 는 폐루프를 표현하지 못해 디프바가 빠져 있다.

사용법 (conda base 등 usd-core 가 설치된 환경에서):
    pip install usd-core
    python3 scripts/extract_rover_from_usd.py \
        --usd /path/to/urdf_and_usd/rover/rover2_diff_full.usd \
        --output powertrain_sim/assets/rover_model.json
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path

import numpy as np
from pxr import Usd, UsdGeom, UsdPhysics


# USD/URDF 는 +Y 전진 · +X 우측. 섀시(ROS REP-103)는 +X 전진 · +Y 좌측.
FORWARD_OFFSET_M = 0.31785   # 축거 중점
LATERAL_OFFSET_M = 0.20      # 차체 중심선
VERTICAL_OFFSET_M = 0.2235   # 바퀴 중심을 z=0 으로

# 타이어 STL 바운딩박스 실측 (207.1 x 207.1 x 70.0 mm).
WHEEL_RADIUS_M = 0.1035
WHEEL_HALF_WIDTH_M = 0.035

WHEEL_BRACKETS = {
    "motor_bracket_front_1": ("front_left", True),
    "motor_bracket_front_2": ("front_right", True),
    "motor_bracket_center_1": ("mid_left", False),
    "motor_bracket_center_2": ("mid_right", False),
    "motor_bracket_rear_1": ("rear_left", True),
    "motor_bracket_rear_2": ("rear_right", True),
}
FREE_JOINT_FORCE_THRESHOLD_NM = 1.0e5   # 이보다 크면 "제한 없음" 으로 본다


def to_chassis(point) -> tuple[float, float, float]:
    return (
        float(point[1]) - FORWARD_OFFSET_M,
        -(float(point[0]) - LATERAL_OFFSET_M),
        float(point[2]) + VERTICAL_OFFSET_M,
    )


def _finite(value):
    if value is None:
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def extract(usd_path: Path) -> dict:
    stage = Usd.Stage.Open(str(usd_path), Usd.Stage.LoadAll)
    cache = UsdGeom.XformCache(Usd.TimeCode.Default())

    bodies = []
    for prim in stage.Traverse():
        if not prim.HasAPI(UsdPhysics.MassAPI):
            continue
        api = UsdPhysics.MassAPI(prim)
        mass = api.GetMassAttr().Get()
        if not mass:
            continue
        matrix = np.array(cache.GetLocalToWorldTransform(prim), dtype=float)
        centre = api.GetCenterOfMassAttr().Get()
        if centre is not None:
            world = (np.array([centre[0], centre[1], centre[2], 1.0]) @ matrix)[:3]
        else:
            world = matrix[3, :3]
        inertia = api.GetDiagonalInertiaAttr().Get() or (0.0, 0.0, 0.0)
        bodies.append(
            {
                "name": prim.GetName(),
                "mass_kg": float(mass),
                "com_m": [round(v, 6) for v in to_chassis(world)],
                "diagonal_inertia": [round(float(v), 9) for v in inertia],
            }
        )

    wheels = []
    for prim in stage.Traverse():
        if "bl70200s_1_tire" not in prim.GetName():
            continue
        path = prim.GetPath().pathString
        for bracket, (name, steerable) in WHEEL_BRACKETS.items():
            if bracket + "_" not in path and not prim.GetName().startswith(bracket):
                continue
            matrix = np.array(cache.GetLocalToWorldTransform(prim), dtype=float)
            x_m, y_m, z_m = to_chassis(matrix[3, :3])
            wheels.append(
                {
                    "name": name,
                    "x_m": round(x_m, 6),
                    "y_m": round(y_m, 6),
                    "z_m": round(z_m, 6),
                    "steerable": steerable,
                }
            )
            break
    if len(wheels) != 6:
        raise SystemExit(f"expected six tyre prims, found {len(wheels)}")

    joints = {}
    for prim in stage.Traverse():
        if not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        joint = UsdPhysics.RevoluteJoint(prim)
        force = None
        if prim.HasAPI(UsdPhysics.DriveAPI, "angular"):
            raw = UsdPhysics.DriveAPI(prim, "angular").GetMaxForceAttr().Get()
            raw = _finite(raw)
            if raw is not None and raw < FREE_JOINT_FORCE_THRESHOLD_NM:
                force = raw
        joints[prim.GetName()] = {
            "axis": str(joint.GetAxisAttr().Get()),
            "lower_deg": _finite(joint.GetLowerLimitAttr().Get()),
            "upper_deg": _finite(joint.GetUpperLimitAttr().Get()),
            "max_force_nm": force,
        }

    digest = hashlib.sha256(usd_path.read_bytes()).hexdigest()
    return {
        "schema_version": 1,
        "source_usd": usd_path.name,
        "source_sha256": digest,
        "wheel_radius_m": WHEEL_RADIUS_M,
        "wheel_half_width_m": WHEEL_HALF_WIDTH_M,
        "total_mass_kg": round(sum(b["mass_kg"] for b in bodies), 6),
        "wheels": sorted(wheels, key=lambda w: w["name"]),
        "bodies": bodies,
        "joints": joints,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--usd", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    arguments = parser.parse_args()
    document = extract(arguments.usd)
    arguments.output.parent.mkdir(parents=True, exist_ok=True)
    arguments.output.write_text(
        json.dumps(document, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    print(
        f"{len(document['bodies'])} bodies, "
        f"{document['total_mass_kg']} kg -> {arguments.output}"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
