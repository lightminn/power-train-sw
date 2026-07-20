# 계획 A — CAD 실차 모델 + 훈련 트랙 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development
> (recommended) or superpowers:executing-plans to implement this plan task-by-task.
> Steps use checkbox (`- [ ]`) syntax for tracking.
>
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 3환경 green + 젯슨 parity.
> 시뮬은 dev 컨테이너 전용(실기 표면 없음).

**Goal:** 시뮬 로봇을 강체 45.5 kg 근사에서 CAD 정본(66.26 kg · 로커보기 · 디프바)으로
교체하고, 훈련 트랙을 완주율이 의미를 갖는 15 m × 1.6 m로 늘린다.

**Architecture:** USD 정본에서 1회 추출한 JSON을 좌표 변환까지 마친 동결 값객체
`RoverModel`로 읽고,
`model_builder.py`가 그것만 소비해 MJCF를 조립한다. 좌표 변환은 `rover_model.py`
단 한 곳에만 존재한다. 서스펜션은 옵션(`suspension=True` 기본)으로 넣어 강체 경로를
대조군으로 보존한다. 트랙은 신규 코드 없이 `family_scenarios.py` 파라미터만 바꾼다.

**Tech Stack:** Python 3, NumPy, MuJoCo 3.x, `xml.etree.ElementTree`, pytest.

**Spec:** `docs/superpowers/specs/2026-07-20-sim-fidelity-and-completion-design.md`

---

## Global Constraints

- **안전 불변식 우선.** 어떤 변경도 `fail_open == 0` · `edge_overrun_count == 0`을
  깨서는 안 된다. 깨지면 완주율이 올라도 되돌린다. 예외 없음.
- **hidden 시드로 튜닝 금지.** dev · regression 시드로만 작업한다.
- **USD 가 정본이다.** `rover2_diff_full.usd` 가 로버 물성의 유일한 권위이며,
  URDF(`urdf_2.urdf`)는 **쓰지 않는다** — 디프바가 빠져 있고 로커 한계가 낡았다.
- **`usd-core` 는 런타임 의존성이 아니다.** 추출 스크립트만 쓴다. dev 컨테이너와
  젯슨 이미지에 추가하지 말 것(젯슨 ARM 휠이 불확실하다).
- **좌표 변환은 `rover_model.py` 와 `scripts/extract_rover_from_usd.py` 두 곳에만
  존재한다** (같은 상수). 다른 파일에 복제하지 말 것.
- **RNG draw 순서 보존.** `procedural.py`는 수정하지 않는다. 기존 시드의 결정론이
  깨지면 안 된다.
- **`ROBOT_FOOTPRINT_WIDTH_M = 0.949`는 CAD 실측과 일치하므로 바꾸지 않는다.**
- **production `wheel_radius_m = 0.10`은 바꾸지 않는다.** 시뮬 물리만 실측
  0.1035 m를 쓴다(스펙 §5 — 의도된 불일치).
- 단위는 SI. 각도는 라디안(MJCF `compiler angle="radian"`).
- 실행 환경: dev 컨테이너. 명령 앞에 `PYTHONPATH=motor_control:ros2/src/powertrain_ros:.`
  가 필요하다.

### 정본 실측치 (모든 테스트의 기대값)

| 항목 | 값 |
|---|---|
| 축거(앞−뒤) | 0.8755 m |
| 윤거 앞 / 중간 / 뒤 | 0.7050 / 0.8790 / 0.5850 m |
| 중륜 x 오프셋 | −0.06036 m |
| **총질량** | **66.9613 kg** (강체 108개 합) |
| 타이어 반경 / 폭 | 0.1035 m / 0.070 m |
| 로커 · 보기 관절 한계 | ±45° (= ±0.785398 rad) |
| 조향 / 구동 모터 한계 | 24.0 / 39.0 N·m (DriveAPI maxForce) |
| 최대 차폭 | 0.9591 m (중륜 모터 하우징) |

> ⚠️ **총질량 주의.** URDF는 66.258 kg, 패키지 README는 "약 66.48 kg"이라고 적지만
> **USD 실측 합계는 66.9613 kg**이다. 차이는 README 계산식에 디프바 링키지(약
> 0.48 kg)가 빠졌기 때문이다. **USD가 정본이므로 66.9613 kg를 쓴다.**

### 좌표 변환 (USD +Y전진·+X우측 → 섀시 +X전진·+Y좌측)

```
chassis_x = urdf_y − 0.31785
chassis_y = −(urdf_x − 0.20)
chassis_z = urdf_z + 0.2235
```

---

## File Structure

| 파일 | 책임 |
|---|---|
| `scripts/extract_rover_from_usd.py` | **신규.** USD 정본 → `rover_model.json` 추출기. `usd-core` 필요. **개발자가 CAD 갱신 시에만 수동 실행** |
| `powertrain_sim/assets/rover_model.json` | **신규.** 추출 산물(수백 KB). 레포에 커밋되는 검증 가능한 파생물 |
| `powertrain_sim/mujoco_fast/rover_model.py` | **신규.** JSON → 동결 `RoverModel`. **`usd-core` 런타임 의존 없음.** MuJoCo·ROS 비의존 |
| `powertrain_sim/mujoco_fast/model_builder.py` | **수정.** `RoverModel`을 소비해 MJCF 조립. 서스펜션 옵션 |
| `powertrain_sim/family_scenarios.py` | **수정.** 훈련 트랙 파라미터 |
| `powertrain_sim/tests/test_rover_model.py` | **신규.** Task 1 검증 |
| `powertrain_sim/tests/test_mujoco_model.py` | **수정.** Task 2·3 검증 추가 |
| `powertrain_sim/tests/test_campaign.py` | **수정.** Task 4 검증 추가 |

---

### Task 1: USD 추출기 + `rover_model.py`

**Files:**
- Create: `scripts/extract_rover_from_usd.py`
- Create: `powertrain_sim/assets/rover_model.json` (추출 산물)
- Create: `powertrain_sim/mujoco_fast/rover_model.py`
- Test: `powertrain_sim/tests/test_rover_model.py`

**Interfaces:**
- Consumes: 없음 (첫 태스크)
- Produces:
  - `usd_to_chassis(x_m, y_m, z_m) -> tuple[float, float, float]`
  - `RoverWheel(name: str, x_m: float, y_m: float, z_m: float, steerable: bool)`
  - `RoverLink(name: str, mass_kg: float, com_m: tuple[float,float,float],
    inertia: tuple[float,float,float,float,float,float])` — `(ixx, iyy, izz, ixy, ixz, iyz)`
  - `RoverModel` (frozen dataclass) 필드: `wheels: tuple[RoverWheel, ...]`,
    `links: tuple[RoverLink, ...]`, `total_mass_kg: float`, `wheel_radius_m: float`,
    `wheel_half_width_m: float`, `rocker_limit_rad: float`, `bogie_limit_rad: float`,
    `drive_torque_limit_nm: float`, `steer_torque_limit_nm: float`
  - `ASSET_PATH: Path` — 번들 JSON 경로
  - `load_rover_model(path: str | Path | None = None) -> RoverModel` — `None`이면
    번들 JSON 사용. 결과는 `functools.lru_cache`로 1회만 로드.

- [ ] **Step 1: USD 추출기 작성**

`scripts/extract_rover_from_usd.py`. **이 스크립트는 CI 나 테스트에서 실행되지
않는다** — 개발자가 CAD 갱신 시에만 손으로 돌린다. 그래서 `usd-core` 는
개발 도구 의존성이지 런타임 의존성이 아니다.

```python
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
```

- [ ] **Step 1b: 추출 실행 — 산물 생성 및 검산**

```bash
cd /home/light/ZETIN/robotics/power-train-sw
mkdir -p /tmp/usdpkg && unzip -q -o urdf/urdf_and_usd.zip -d /tmp/usdpkg
/home/light/anaconda3/bin/pip install -q usd-core
/home/light/anaconda3/bin/python scripts/extract_rover_from_usd.py \
  --usd /tmp/usdpkg/urdf_and_usd/rover/rover2_diff_full.usd \
  --output powertrain_sim/assets/rover_model.json
```

기대 출력: `108 bodies, 66.9613 kg -> powertrain_sim/assets/rover_model.json`

⚠️ **`configuration/` 폴더가 USD 옆에 함께 있어야 한다** — 지오메트리·물리가
그 페이로드에 들어 있다. zip 을 통째로 풀면 자동으로 만족된다.

USD · STL · zip 은 **레포에 반입하지 않는다.** JSON 산물만 커밋한다.

- [ ] **Step 2: 실패하는 테스트 작성**

`powertrain_sim/tests/test_rover_model.py`:

```python
from __future__ import annotations

import math

import pytest

from powertrain_sim.mujoco_fast.rover_model import (
    RoverModel,
    load_rover_model,
    usd_to_chassis,
)


def test_coordinate_transform_maps_usd_forward_y_to_chassis_forward_x():
    # USD 앞왼쪽 타이어 중심 (x=-0.1525, y=+0.7556) → 섀시 (+0.4377, +0.3525)
    x_m, y_m, z_m = usd_to_chassis(-0.1525, 0.7556, -0.2235)

    assert x_m == pytest.approx(0.4377, abs=5e-4)
    assert y_m == pytest.approx(0.3525, abs=5e-4)
    assert z_m == pytest.approx(0.0, abs=5e-4)


def test_transform_puts_left_wheels_on_positive_y():
    # USD 는 +X 가 우측이므로 부호가 뒤집혀야 한다.
    left = usd_to_chassis(-0.2395, 0.2575, -0.2235)
    right = usd_to_chassis(0.6395, 0.2575, -0.2235)

    assert left[1] > 0.0
    assert right[1] < 0.0
    assert left[1] == pytest.approx(-right[1], abs=5e-4)


def test_wheel_layout_matches_cad_measurements():
    model = load_rover_model()
    wheels = {wheel.name: wheel for wheel in model.wheels}

    assert set(wheels) == {
        "front_left", "front_right",
        "mid_left", "mid_right",
        "rear_left", "rear_right",
    }
    # 축거 875.5 mm
    assert wheels["front_left"].x_m - wheels["rear_left"].x_m == pytest.approx(
        0.8755, abs=1e-3
    )
    # 윤거 앞 705.0 / 중간 879.0 / 뒤 585.0 mm
    assert wheels["front_left"].y_m - wheels["front_right"].y_m == pytest.approx(
        0.7050, abs=1e-3
    )
    assert wheels["mid_left"].y_m - wheels["mid_right"].y_m == pytest.approx(
        0.8790, abs=1e-3
    )
    assert wheels["rear_left"].y_m - wheels["rear_right"].y_m == pytest.approx(
        0.5850, abs=1e-3
    )
    # 중륜은 축거 중심에서 60.3 mm 뒤
    assert wheels["mid_left"].x_m == pytest.approx(-0.0603, abs=1e-3)


def test_only_front_and_rear_wheels_are_steerable():
    model = load_rover_model()
    steerable = {wheel.name for wheel in model.wheels if wheel.steerable}

    assert steerable == {
        "front_left", "front_right", "rear_left", "rear_right",
    }


def test_total_mass_matches_the_usd_source_of_truth():
    model = load_rover_model()

    # USD 실측 108 강체 합. URDF(66.258)나 README(66.48)가 아니라 이 값이 정본 —
    # 둘은 디프바 링키지를 빠뜨렸다.
    assert model.total_mass_kg == pytest.approx(66.9613, abs=0.01)
    assert sum(link.mass_kg for link in model.links) == pytest.approx(
        model.total_mass_kg, abs=1e-4
    )
    assert len(model.links) == 108


def test_tyre_dimensions_and_usd_sourced_limits():
    model = load_rover_model()

    assert model.wheel_radius_m == pytest.approx(0.1035, abs=1e-4)
    assert model.wheel_half_width_m == pytest.approx(0.035, abs=1e-4)
    # USD RevoluteJoint 한계: 로커·보기 모두 +-45 도.
    assert model.rocker_limit_rad == pytest.approx(math.radians(45.0))
    assert model.bogie_limit_rad == pytest.approx(math.radians(45.0))
    # USD DriveAPI maxForce.
    assert model.drive_torque_limit_nm == pytest.approx(39.0)
    assert model.steer_torque_limit_nm == pytest.approx(24.0)


def test_json_records_its_usd_provenance():
    """산물이 어느 USD 에서 나왔는지 추적 가능해야 한다."""
    import json
    from powertrain_sim.mujoco_fast.rover_model import ASSET_PATH

    document = json.loads(ASSET_PATH.read_text(encoding="utf-8"))

    assert document["schema_version"] == 1
    assert document["source_usd"] == "rover2_diff_full.usd"
    assert len(document["source_sha256"]) == 64


def test_model_is_frozen_and_cached():
    first = load_rover_model()
    second = load_rover_model()

    assert first is second
    assert isinstance(first, RoverModel)
    with pytest.raises(Exception):
        first.total_mass_kg = 1.0  # type: ignore[misc]


def test_wheel_positions_agree_with_production_kinematics():
    """시뮬 모델과 production 기구학이 같은 로봇이어야 한다."""
    from chassis.kinematics import default_geometry

    model = load_rover_model()
    production = {wheel.name: wheel for wheel in default_geometry().wheels}

    for wheel in model.wheels:
        assert wheel.x_m == pytest.approx(production[wheel.name].x, abs=2e-3)
        assert wheel.y_m == pytest.approx(production[wheel.name].y, abs=2e-3)
        assert wheel.steerable == production[wheel.name].steerable
```

- [ ] **Step 3: 테스트 실패 확인**

```bash
cd /home/light/ZETIN/robotics/power-train-sw
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m pytest \
  powertrain_sim/tests/test_rover_model.py -v
```

기대: `ModuleNotFoundError: No module named 'powertrain_sim.mujoco_fast.rover_model'`
로 전부 실패(collection error).

- [ ] **Step 4: 구현**

`powertrain_sim/mujoco_fast/rover_model.py`:

```python
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
```

- [ ] **Step 5: 테스트 통과 확인**

```bash
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m pytest \
  powertrain_sim/tests/test_rover_model.py -v
```

기대: 9 passed.

`test_model_is_frozen_and_cached`가 `lru_cache` 때문에 `path=None` 인자로만
캐시된다는 점에 주의 — 같은 인자면 같은 객체다.

- [ ] **Step 6: 커밋 (리뷰어가 수행)**

```bash
git add scripts/extract_rover_from_usd.py \
        powertrain_sim/assets/rover_model.json \
        powertrain_sim/mujoco_fast/rover_model.py \
        powertrain_sim/tests/test_rover_model.py
git commit -m "feat(sim): USD-sourced rover model with one-shot extraction to committed JSON"
```

---

### Task 2: `model_builder` 가 실질량·실관성·실바퀴반경을 쓴다 (강체 유지)

서스펜션은 **아직 넣지 않는다.** 질량 변화만의 영향을 분리해 관측하기 위해서다.

**Files:**
- Modify: `powertrain_sim/mujoco_fast/model_builder.py:147-286` (`_wheel_body`, `_rover`)
- Test: `powertrain_sim/tests/test_mujoco_model.py` (추가)

**Interfaces:**
- Consumes: Task 1의 `load_rover_model()`, `RoverModel`, `WHEEL_RADIUS_M`
- Produces: `build_mjcf(scenario, *, geometry=None, suspension: bool = False) -> str`
  — Task 3에서 `suspension` 기본값을 `True`로 바꾼다.

- [ ] **Step 1: 실패하는 테스트 작성**

`powertrain_sim/tests/test_mujoco_model.py` 끝에 추가:

```python
def test_mjcf_total_mass_matches_the_cad_urdf():
    scenario = _load()

    model = mujoco.MjModel.from_xml_string(build_mjcf(scenario))

    # base_link 이하 전체(월드 바디 0번 제외)
    total = float(model.body_mass[1:].sum())
    assert total == pytest.approx(66.9613, abs=0.5)


def test_wheel_geoms_use_the_measured_tyre_radius():
    scenario = _load()
    root = ET.fromstring(build_mjcf(scenario))

    geom = root.find(".//geom[@name='wheel_geom_front_left']")
    assert geom is not None
    radius, half_width = (float(value) for value in geom.attrib["size"].split())
    assert radius == pytest.approx(0.1035, abs=1e-4)
    assert half_width == pytest.approx(0.035, abs=1e-4)


def test_forward_command_moves_the_rover_along_positive_x():
    """부호 검증: v>0 이면 +X 로 간다."""
    scenario = _load()
    plant = MujocoFastPlant(scenario)
    start = plant.ground_truth_pose()[0].copy()

    plant.apply_command(0.4, 0.0)
    for _ in range(100):
        plant.step_clock_interval()

    moved = plant.ground_truth_pose()[0] - start
    assert moved[0] > 0.05
    assert abs(moved[1]) < 0.05


def test_positive_yaw_rate_turns_counter_clockwise_and_slows_the_left_wheels():
    """부호 검증: omega>0 이면 반시계, 좌측 바퀴가 더 느리다."""
    scenario = _load()
    geometry = default_geometry()
    plant = MujocoFastPlant(scenario, geometry=geometry)

    result = plant.apply_command(0.4, 0.3)

    left = result.wheels["mid_left"].drive_turns_per_s
    right = result.wheels["mid_right"].drive_turns_per_s
    assert left < right


def test_all_six_wheels_touch_the_deck_at_rest():
    scenario = _load()
    plant = MujocoFastPlant(scenario)

    for _ in range(50):
        plant.step_clock_interval()

    contacts = plant.wheel_contact_points_world()
    assert len(contacts) == 6
    heights = [point[2] for point in contacts.values()]
    assert max(heights) - min(heights) < 0.05
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m pytest \
  powertrain_sim/tests/test_mujoco_model.py -v -k "cad_urdf or tyre_radius"
```

기대: `test_mjcf_total_mass_matches_the_cad_urdf` FAIL (현재 45.5 kg),
`test_wheel_geoms_use_the_measured_tyre_radius` FAIL (현재 0.10 m).

- [ ] **Step 3: 구현 — 바퀴 반경·폭을 실측값으로**

`model_builder.py:17`을 바꾼다:

```python
from .rover_model import WHEEL_HALF_WIDTH_M, load_rover_model
```

그리고 기존 `WHEEL_HALF_WIDTH_M = 0.035` 정의를 지우고 위 import 로 대체한다
(값은 같지만 출처를 한 곳으로 모은다).

`_wheel_body()` 호출부(`model_builder.py:274-286`)에서 `radius_m=geometry.wheel_radius_m`
을 `radius_m=rover.wheel_radius_m` 으로 바꾼다. `rover` 는 `_rover()` 시그니처에
새로 추가한다:

```python
def _rover(
    worldbody: ET.Element,
    scenario: Scenario,
    geometry: ChassisGeometry,
    rover: RoverModel,
    initial_normal: np.ndarray,
    initial_quaternion: tuple[float, ...],
) -> None:
```

- [ ] **Step 4: 구현 — 질량 배분**

`_rover()` 안의 하드코딩 `"mass": "38"` 을 CAD 유래 값으로 바꾼다.
`_wheel_body()` 의 `"mass": "1.2"` 도 바꾼다.

```python
# CAD 질량 배분. 인휠 구동모터 + 브래킷 + 타이어는 비스프렁이므로 바퀴 바디에
# 싣고, 나머지는 차체에 싣는다. 합계는 USD 총질량(66.9613 kg)과 일치해야 한다.
# USD prim 이름 예: motor_bracket_front_1_bl70200s_1_tire_1 (62개가 여기 걸린다).
_UNSPRUNG_KEYS = ("bl70200s", "motor_bracket", "AK45")


def _mass_split(rover: RoverModel) -> tuple[float, float]:
    """(바퀴 1개당 비스프렁 질량, 차체 질량) 을 돌려준다."""
    unsprung = sum(
        link.mass_kg
        for link in rover.links
        if any(key in link.name for key in _UNSPRUNG_KEYS)
    )
    body = rover.total_mass_kg - unsprung
    return unsprung / len(rover.wheels), body
```

`_wheel_body()` 에 `mass_kg: float` 인자를 추가해 `"mass": _numbers((mass_kg,))` 로
쓰고, 차체 geom 의 `"mass"` 를 `_numbers((body_mass_kg,))` 로 바꾼다.
조향 허브 구(`steer_hub_*`)의 `"mass": "0.08"` 은 **0 으로 바꾼다** — 질량을
이중 계상하지 않기 위해서다. MuJoCo 는 질량 0 지오메트리를 허용하므로
`"mass": "0"` 로 두고 `contype/conaffinity` 는 그대로 0 을 유지한다.

`build_mjcf()` 안에서 `rover = load_rover_model()` 을 호출해 `_rover()` 에 넘긴다.
`build_mjcf` 시그니처에 `suspension: bool = False` 를 추가하되 이 태스크에서는
사용하지 않는다(Task 3에서 사용).

- [ ] **Step 4b: 구현 — 액추에이터 한계를 USD 실측으로**

`model_builder.py:338-373` 의 하드코딩 `forcerange` 를 `RoverModel` 값으로 바꾼다.
지금은 조향 `-160 160`, 구동 `-90 90` 인데 실물은 조향 24 N·m, 구동 39 N·m 다.

조향 액추에이터:

```python
            "forcelimited": "true",
            "forcerange": _numbers(
                (-rover.steer_torque_limit_nm, rover.steer_torque_limit_nm)
            ),
```

구동 액추에이터:

```python
            "forcelimited": "true",
            "forcerange": _numbers(
                (-rover.drive_torque_limit_nm, rover.drive_torque_limit_nm)
            ),
```

조향 속도 한계(README 의 312 °/s)는 **적용하지 않는다.** MuJoCo `position`
액추에이터는 속도 한계를 직접 받지 않고, 실측 슬루(AK `DEFAULT_SPD_ERPM=4500`
≈ 출력축 47 °/s)가 312 °/s 보다 훨씬 낮아 구속력이 없다. 추출 JSON 에도
저장하지 않는다 — USD DriveAPI 에서 나오는 값이 아니라 README 서술이다.

⚠️ 토크 한계를 낮추면(160→24) 조향이 느려지거나 실패할 수 있다. 이는 **실물
반영이므로 정상**이다. 조향이 아예 안 따라가면 보고할 것 — 실기에서도 같은
문제가 있다는 뜻이다.

테스트에 다음을 추가한다:

```python
def test_actuator_force_limits_match_the_usd_motor_specs():
    scenario = _load()
    root = ET.fromstring(build_mjcf(scenario))

    steer = root.find(".//actuator/position[@name='steer_front_left']")
    drive = root.find(".//actuator/velocity[@name='drive_mid_left']")
    assert steer is not None and drive is not None
    assert [float(v) for v in steer.attrib["forcerange"].split()] == [-24.0, 24.0]
    assert [float(v) for v in drive.attrib["forcerange"].split()] == [-39.0, 39.0]
```

- [ ] **Step 5: 테스트 통과 확인**

```bash
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m pytest \
  powertrain_sim/tests/test_mujoco_model.py -v
```

기대: 전부 PASS. 특히 `test_mjcf_total_mass_matches_the_cad_urdf` 가 66.9613 ± 0.5 kg.

- [ ] **Step 6: 캠페인 실행 — 앵커 이동 관측**

```bash
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m powertrain_sim.campaign \
  /tmp/campaign_task2
```

**이 단계에서 앵커가 움직이는 것은 정상이다.** 결과 표를 그대로 기록해 둘 것
(Task 5에서 비교한다). `passed=false` 가 나와도 여기서는 진행한다 —
`expected_metrics` 갱신은 Task 5의 일이다.

- [ ] **Step 7: 커밋 (리뷰어가 수행)**

```bash
git add powertrain_sim/mujoco_fast/model_builder.py powertrain_sim/tests/test_mujoco_model.py
git commit -m "feat(sim): CAD mass distribution and measured tyre radius in the MuJoCo plant"
```

---

### Task 3: 로커보기 서스펜션 + 디프바 등식 제약

**Files:**
- Modify: `powertrain_sim/mujoco_fast/model_builder.py` (`_rover`, `build_mjcf`)
- Test: `powertrain_sim/tests/test_mujoco_model.py` (추가)

**Interfaces:**
- Consumes: Task 2의 `build_mjcf(scenario, *, geometry, suspension)`, `RoverModel`
- Produces: `build_mjcf(..., suspension: bool = True)` — 기본값이 True 로 바뀐다.
  MJCF 조인트 이름 `rocker_left` · `rocker_right` · `bogie_left` · `bogie_right`,
  등식 제약 이름 `differential_bar`.

**바디 트리 구조** (좌측 기준, 우측 대칭):

```
base_link
 └─ rocker_left            (hinge, axis=x, +-45 deg)   ← 로커 피벗
     ├─ bogie_left         (hinge, axis=x, +-45 deg)   ← 보기 피벗
     │   ├─ steer_front_left → wheel_front_left
     │   └─ wheel_mid_left
     └─ steer_rear_left → wheel_rear_left
```

CAD 대로 **앞·중 바퀴가 보기에, 뒷바퀴가 로커에** 달린다
(`rocker_v2_L → bogie_pivot_in → bogie_v2_1` 체인, Task 1 USD 실측).

- [ ] **Step 1: 실패하는 테스트 작성**

`powertrain_sim/tests/test_mujoco_model.py` 끝에 추가:

```python
def test_suspension_model_has_rocker_and_bogie_joints_with_usd_limits():
    scenario = _load()
    root = ET.fromstring(build_mjcf(scenario))

    for name in ("rocker_left", "rocker_right", "bogie_left", "bogie_right"):
        joint = root.find(f".//joint[@name='{name}']")
        assert joint is not None, name
        assert joint.attrib["type"] == "hinge"
        lower, upper = (float(v) for v in joint.attrib["range"].split())
        assert lower == pytest.approx(-math.radians(45.0), abs=1e-6)
        assert upper == pytest.approx(math.radians(45.0), abs=1e-6)


def test_differential_bar_is_an_equality_constraint():
    scenario = _load()
    root = ET.fromstring(build_mjcf(scenario))

    equality = root.find(".//equality/joint[@name='differential_bar']")
    assert equality is not None
    assert {equality.attrib["joint1"], equality.attrib["joint2"]} == {
        "rocker_left", "rocker_right",
    }


def test_rigid_model_remains_available_as_a_control():
    scenario = _load()

    rigid = ET.fromstring(build_mjcf(scenario, suspension=False))

    assert rigid.find(".//joint[@name='rocker_left']") is None
    assert rigid.find(".//equality/joint[@name='differential_bar']") is None


def test_rocker_actually_articulates_over_a_one_sided_bump():
    """음성 대조 포함: 서스펜션이 실제로 움직여야 한다."""
    scenario = _load()
    plant = MujocoFastPlant(scenario)
    joint_id = mujoco.mj_name2id(
        plant.model, mujoco.mjtObj.mjOBJ_JOINT, "rocker_left"
    )
    assert joint_id >= 0
    address = int(plant.model.jnt_qposadr[joint_id])

    # 좌측 앞바퀴 아래에만 턱을 놓는다: base 를 롤 방향으로 기울여 접지 비대칭을 만든다.
    free_qpos = int(plant.model.jnt_qposadr[plant.root_free_joint_id])
    plant.data.qpos[free_qpos + 2] += 0.05
    plant.data.qpos[free_qpos + 4] = 0.08   # quat x 성분 = roll
    mujoco.mj_forward(plant.model, plant.data)
    for _ in range(200):
        plant.step_clock_interval()

    assert abs(float(plant.data.qpos[address])) > 1e-3


def test_differential_bar_changes_behaviour_when_removed():
    """음성 대조: 제약을 끄면 거동이 달라져야 한다. 안 달라지면 제약이 안 걸린 것."""
    scenario = _load()
    with_bar = build_mjcf(scenario)
    model_with = mujoco.MjModel.from_xml_string(with_bar)

    without_bar = with_bar.replace('name="differential_bar"', 'name="_disabled"')
    without_bar = without_bar.replace("<equality>", "<equality>").replace(
        '<joint name="_disabled"', '<joint active="false" name="_disabled"'
    )
    model_without = mujoco.MjModel.from_xml_string(without_bar)

    assert model_with.neq >= 1
    assert int(model_with.eq_active0.sum()) > int(model_without.eq_active0.sum())
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m pytest \
  powertrain_sim/tests/test_mujoco_model.py -v -k "rocker or differential or rigid_model"
```

기대: 5개 FAIL (조인트·등식 제약 없음).

- [ ] **Step 3: 구현 — 바디 트리 재구성**

`_rover()` 를 `suspension` 분기로 나눈다. 강체 경로는 **그대로 보존**한다.

```python
def _suspension_side(
    base: ET.Element,
    geometry: ChassisGeometry,
    rover: RoverModel,
    side: str,           # "left" | "right"
    wheels: dict[str, RoverWheel],
    unsprung_kg: float,
) -> None:
    """CAD 체인: base -> rocker -> (bogie -> 앞·중륜) + 뒷륜."""
    front = wheels[f"front_{side}"]
    mid = wheels[f"mid_{side}"]
    rear = wheels[f"rear_{side}"]
    # 로커 피벗은 중륜과 뒷륜 사이, 차체 중심선 쪽. CAD 의 chassis_pivot 위치.
    pivot_x = 0.5 * (mid.x_m + rear.x_m)
    pivot_y = 0.5 * front.y_m
    rocker_kg, bogie_kg = _suspension_masses(rover)
    rocker = ET.SubElement(
        base,
        "body",
        {"name": f"rocker_{side}", "pos": _numbers((pivot_x, pivot_y, 0.0))},
    )
    ET.SubElement(
        rocker,
        "joint",
        {
            "name": f"rocker_{side}",
            "type": "hinge",
            "axis": "1 0 0",
            "range": _numbers((-rover.rocker_limit_rad, rover.rocker_limit_rad)),
            "limited": "true",
            "damping": "4.0",
            "armature": "0.05",
        },
    )
    ET.SubElement(
        rocker,
        "geom",
        {
            "name": f"rocker_geom_{side}",
            "type": "capsule",
            "fromto": _numbers(
                (rear.x_m - pivot_x, 0.0, 0.0, mid.x_m - pivot_x, 0.0, 0.0)
            ),
            "size": "0.02",
            "mass": _numbers((rocker_kg,)),
            "contype": "0",
            "conaffinity": "0",
            "group": "1",
        },
    )
    bogie = ET.SubElement(
        rocker,
        "body",
        {"name": f"bogie_{side}", "pos": _numbers((mid.x_m - pivot_x, 0.0, 0.0))},
    )
    ET.SubElement(
        bogie,
        "joint",
        {
            "name": f"bogie_{side}",
            "type": "hinge",
            "axis": "1 0 0",
            "range": _numbers((-rover.bogie_limit_rad, rover.bogie_limit_rad)),
            "limited": "true",
            "damping": "2.0",
            "armature": "0.03",
        },
    )
    ET.SubElement(
        bogie,
        "geom",
        {
            "name": f"bogie_geom_{side}",
            "type": "capsule",
            "fromto": _numbers((0.0, 0.0, 0.0, front.x_m - mid.x_m, 0.0, 0.0)),
            "size": "0.02",
            "mass": _numbers((bogie_kg,)),
            "contype": "0",
            "conaffinity": "0",
            "group": "1",
        },
    )
    # 앞바퀴(조향) 는 보기에, 중륜(고정) 도 보기에, 뒷바퀴(조향) 는 로커에 붙는다.
    _attach_wheel(bogie, rover, front, (front.x_m - mid.x_m, front.y_m - pivot_y, 0.0), unsprung_kg)
    _attach_wheel(bogie, rover, mid, (0.0, mid.y_m - pivot_y, 0.0), unsprung_kg)
    _attach_wheel(rocker, rover, rear, (rear.x_m - pivot_x, rear.y_m - pivot_y, 0.0), unsprung_kg)
```

`_attach_wheel()` 은 기존 `_rover()` 의 바퀴 부착 로직(`model_builder.py:235-286`)을
부모 바디와 상대 위치를 받도록 추출한 헬퍼다. **강체 경로와 서스펜션 경로가
같은 함수를 쓰게 해서 조향 설정이 갈라지지 않게 한다.**

```python
def _attach_wheel(
    parent: ET.Element,
    geometry: ChassisGeometry,
    rover: RoverModel,
    wheel: RoverWheel,
    position: tuple[float, float, float],
    mass_kg: float,
) -> None:
    """조향 바디(필요 시)와 바퀴 바디를 parent 아래 position 에 붙인다."""
    if wheel.steerable:
        steer = ET.SubElement(
            parent,
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
                "mass": "0",
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
            mass_kg=mass_kg,
        )
    else:
        _wheel_body(
            parent,
            name=wheel.name,
            position=position,
            radius_m=rover.wheel_radius_m,
            mass_kg=mass_kg,
        )
```

**질량 정산.** Task 2의 `_mass_split()` 은 차체 질량에 로커·보기 몫이 아직
포함돼 있다. 서스펜션 경로에서는 그 몫을 캡슐로 옮기므로 차체에서 빼야 총합이
66.9613 kg 로 유지된다. Task 2의 `_mass_split()` 을 아래로 교체한다:

```python
def _suspension_masses(rover: RoverModel) -> tuple[float, float]:
    """(로커 1개, 보기 1개) 질량. CAD 링크 이름 접두사로 집계한다."""
    rocker = sum(
        link.mass_kg for link in rover.links if link.name.startswith("rocker_v2")
    )
    bogie = sum(
        link.mass_kg for link in rover.links if link.name.startswith("bogie_v2")
    )
    return rocker / 2.0, bogie / 2.0


def _mass_split(rover: RoverModel, *, suspension: bool) -> tuple[float, float]:
    """(바퀴 1개당 비스프렁 질량, 차체 질량) 을 돌려준다."""
    unsprung = sum(
        link.mass_kg
        for link in rover.links
        if any(key in link.name for key in _UNSPRUNG_KEYS)
    )
    body = rover.total_mass_kg - unsprung
    if suspension:
        rocker_kg, bogie_kg = _suspension_masses(rover)
        body -= 2.0 * (rocker_kg + bogie_kg)
    return unsprung / len(rover.wheels), body
```

`_suspension_side()` 의 캡슐 `"mass"` 하드코딩(`"1.81"` · `"1.17"`)을
`_numbers((rocker_kg,))` · `_numbers((bogie_kg,))` 로 바꾸고, 두 값은
`_suspension_masses(rover)` 에서 받아 인자로 넘긴다. **하드코딩하지 말 것** —
CAD 가 갱신되면 자동으로 따라가야 한다.

`build_mjcf()` 에서 `suspension` 기본값을 `True` 로 바꾸고, True 일 때
`root` 아래 `<equality>` 를 추가한다:

```python
if suspension:
    equality = ET.SubElement(root, "equality")
    ET.SubElement(
        equality,
        "joint",
        {
            "name": "differential_bar",
            "joint1": "rocker_left",
            "joint2": "rocker_right",
            # 디프바: 좌우 로커가 서로 반대로 움직인다 (polycoef = 0 + (-1)*q2).
            "polycoef": "0 -1 0 0 0",
            "solimp": "0.95 0.99 0.001",
            "solref": "0.02 1",
        },
    )
```

참고로 CAD 실측 로커 총 3.620 kg · 보기 총 2.338 kg 이므로 한쪽당 각각
약 1.81 kg · 1.17 kg 이 나온다 — 위 `_suspension_masses()` 결과가 이 값 근처인지
확인하면 링크 이름 접두사 집계가 맞는지 검산할 수 있다.

`_rover()` 는 `suspension` 인자를 받아 분기한다:

```python
    if suspension:
        wheels = {wheel.name: wheel for wheel in rover.wheels}
        for side in ("left", "right"):
            _suspension_side(base, geometry, rover, side, wheels, unsprung_kg)
    else:
        for wheel in rover.wheels:              # 기존 강체 경로
            _attach_wheel(
                base, geometry, rover, wheel,
                (wheel.x_m, wheel.y_m, 0.0), unsprung_kg,
            )
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m pytest \
  powertrain_sim/tests/test_mujoco_model.py -v
```

기대: 전부 PASS. 총질량 테스트가 여전히 66.9613 ± 0.5 kg 여야 한다.

물리가 불안정하면(진동·관통) `damping` · `armature` 를 키우고, 그래도 안 되면
`model_builder.py` 의 `iterations` 를 40 → 80 으로 올린다. **해결 안 되면
숨기지 말고 보고할 것.**

- [ ] **Step 5: 캠페인 실행 — 2차 앵커 이동 관측**

```bash
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m powertrain_sim.campaign \
  /tmp/campaign_task3
```

Task 2 결과와 비교해 **서스펜션만의 영향**을 기록한다. 런타임도 함께 적는다.

- [ ] **Step 6: 커밋 (리뷰어가 수행)**

```bash
git add powertrain_sim/mujoco_fast/model_builder.py powertrain_sim/tests/test_mujoco_model.py
git commit -m "feat(sim): rocker-bogie articulation and differential-bar equality constraint"
```

---

### Task 4: 훈련 트랙 — 15 m × 1.6 m + 대회 실측 기복

**Files:**
- Modify: `powertrain_sim/family_scenarios.py:22-42, 53-77, 80-106, 128-176`
- Test: `powertrain_sim/tests/test_campaign.py` (추가)

**Interfaces:**
- Consumes: 없음 (독립)
- Produces: 새 모듈 상수 `TRAINING_TRACK_LENGTH_M = 15.0`,
  `TRAINING_TRACK_WIDTH_M = 1.6`, `UNDULATION_AMPLITUDE_M = 0.15`,
  `UNDULATION_WAVELENGTH_M = 4.4`

- [ ] **Step 1: 실패하는 테스트 작성**

`powertrain_sim/tests/test_campaign.py` 끝에 추가:

```python
import pytest

from powertrain_sim.campaign import FAMILIES, build_family_document
from powertrain_sim.family_scenarios import (
    ROBOT_FOOTPRINT_WIDTH_M,
    TRAINING_TRACK_LENGTH_M,
    TRAINING_TRACK_WIDTH_M,
)


@pytest.mark.parametrize(
    "family",
    [name for name in FAMILIES if name not in ("pinch", "follow")],
)
def test_training_track_is_long_and_wide_enough_for_the_real_rover(family):
    document = build_family_document(family, seed=0, seed_class="dev")

    widths = document["track"]["width_m"]
    assert min(widths) == pytest.approx(TRAINING_TRACK_WIDTH_M, abs=1e-6)
    # 차폭 949 mm 대비 편측 여유 325 mm
    assert (min(widths) - ROBOT_FOOTPRINT_WIDTH_M) / 2.0 > 0.30

    centerline = document["track"]["centerline_m"]
    span = max(point[0] for point in centerline) - min(
        point[0] for point in centerline
    )
    assert span == pytest.approx(TRAINING_TRACK_LENGTH_M, rel=0.15)


def test_pinch_family_keeps_its_deliberate_narrowing():
    """폭 확대가 pinch 의 의도적 좁힘을 덮어쓰면 안 된다."""
    document = build_family_document("pinch", seed=0, seed_class="dev")

    widths = document["track"]["width_m"]
    assert min(widths) < ROBOT_FOOTPRINT_WIDTH_M + 0.20
    assert max(widths) > min(widths)


def test_undulating_family_matches_the_measured_course_profile():
    document = build_family_document("undulating", seed=0, seed_class="dev")

    heights = document["track"]["height_m"]
    peak_to_peak = max(heights) - min(heights)
    # 대회 코스 실측: 0.085 <-> 0.388 m (peak-to-peak 0.303 m)
    assert peak_to_peak == pytest.approx(0.30, abs=0.05)


def test_clock_duration_covers_the_longer_track():
    document = build_family_document("flat", seed=0, seed_class="dev")

    speed = document["motion"]["linear_speed_m_s"]
    duration = document["clock"]["duration_s"]
    assert speed * duration >= TRAINING_TRACK_LENGTH_M
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m pytest \
  powertrain_sim/tests/test_campaign.py -v
```

기대: `ImportError: cannot import name 'TRAINING_TRACK_LENGTH_M'` 로 collection 실패.

- [ ] **Step 3: 구현**

`family_scenarios.py:19` 아래에 상수를 추가한다:

```python
# 훈련 트랙 — 스펙 2026-07-20 §4.2.
# 길이: 2.5 m 에서는 종단 fail-closed 정지거리 0.7 m 가 전체의 28% 라
#       구조적 최대 완주율이 ~0.71 이었다. 15 m 에서는 5% 로 내려간다.
# 폭:   차폭 949 mm 대비 편측 여유 325 mm. 차폭을 진단 변수에서 제거한다.
TRAINING_TRACK_LENGTH_M = 15.0
TRAINING_TRACK_WIDTH_M = 1.6
# 대회 코스 course.stl 실측: 0.085 <-> 0.388 m (peak-to-peak 0.303 m), 주기 4.4 m.
UNDULATION_AMPLITUDE_M = 0.15
UNDULATION_WAVELENGTH_M = 4.4
# 0.45 m/s 로 15 m 를 주파하려면 33.3 s. 종단 정지 여유를 포함해 40 s.
TRAINING_DURATION_S = 40.0
```

그리고 `_terrain_document`, `friction_document`, `clothoid_document`,
`undulating_document` 의 `GenerationParameters` 에서:

```python
track_length_range_m=(TRAINING_TRACK_LENGTH_M, TRAINING_TRACK_LENGTH_M),
track_width_range_m=(TRAINING_TRACK_WIDTH_M, TRAINING_TRACK_WIDTH_M),
undulation_amplitude_m=UNDULATION_AMPLITUDE_M,
undulation_wavelength_m=UNDULATION_WAVELENGTH_M,
```

로 바꾸고, 각 함수의 `document["clock"]["duration_s"]` 를 `TRAINING_DURATION_S`
로 설정한다(`_terrain_document` 는 지금 duration 을 안 만지므로 새로 추가).

**`pinch_document` 는 다르게 다룬다** — 기본 폭만 넓히고 좁힘 구간은 유지한다:

```python
track_length_range_m=(TRAINING_TRACK_LENGTH_M, TRAINING_TRACK_LENGTH_M),
track_width_range_m=(TRAINING_TRACK_WIDTH_M, TRAINING_TRACK_WIDTH_M),
...
pinch=PinchSpec(center_ratio=0.45, length_m=0.5, width_m=width_m),
```

`width_m` 인자(호출부 `campaign.py:63-67` 에서 `ROBOT_FOOTPRINT_WIDTH_M + 0.15`)는
**그대로 둔다.**

`follow_document` 는 이미 40 m / 1.8 m 이므로 **건드리지 않는다.**

`__all__` 에 새 상수 4개를 추가한다.

- [ ] **Step 4: 테스트 통과 확인**

```bash
PYTHONPATH=motor_control:ros2/src/powertrain_ros:. python3 -m pytest \
  powertrain_sim/tests/test_campaign.py powertrain_sim/tests/test_procedural.py -v
```

기대: 전부 PASS. `test_procedural.py` 가 깨지면 RNG draw 순서를 건드린 것이니
되돌릴 것.

- [ ] **Step 5: 커밋 (리뷰어가 수행)**

```bash
git add powertrain_sim/family_scenarios.py powertrain_sim/tests/test_campaign.py
git commit -m "feat(sim): 15 m x 1.6 m training track with measured course undulation"
```

---

### Task 5: 앵커 재측정 · 성능 게이트 · 문서 — **리뷰어 주도**

**Files:**
- Modify: `powertrain_sim/README.md`
- Modify: `powertrain_sim/scenarios/*.yaml` 및 생성 문서의 `expected_metrics`
  (실측으로 갱신)
- Create: `docs/reports/2026-07-20-cad-rover-training-track.md`

- [ ] **Step 1: dev 컨테이너에서 확정 캠페인 실행**

```bash
docker compose -f docker/docker-compose.yml run --rm dev bash -lc '
  cd /workspace &&
  PYTHONPATH=motor_control:ros2/src/powertrain_ros:. \
  python3 -m powertrain_sim.campaign /tmp/campaign_final 2>&1 | tee /tmp/campaign_final.log'
```

스펙 §1.1의 호스트 잠정값을 **이 값으로 대체**한다.

- [ ] **Step 2: 성능 게이트 판정 (V-5)**

캠페인 총 런타임을 기록한다. 기준선은 2.5 m 트랙 · 강체 모델에서 59 s 였다.
15 m 트랙(6배) + 관절 4개 추가이므로 **6~10배 증가가 예상 범위**다.

- 10분 이내 → 통과. 수백 시드 스윕 가능
- 10분 초과 → 스펙 §7 대책 적용: ①스윕용 짧은 트랙 / 성적표용 15 m 이원화
  ②캠페인 병렬 실행. **어느 쪽을 골랐는지 보고서에 기록한다.**

- [ ] **Step 3: 앵커 갱신**

`expected_metrics` 를 실측값으로 갱신한다. **각 가족마다 이동폭과 사유를 적는다**
(질량 영향 = Task 2 결과, 서스펜션 영향 = Task 3 결과, 트랙 영향 = Task 4 결과).

⚠️ **`fail_open_count` 와 `edge_overrun_count` 가 0 이 아니면 갱신하지 말고
중단·보고할 것.** 안전 불변식이 깨진 것이다.

- [ ] **Step 4: 3환경 회귀**

```bash
# 호스트
python3 -m pytest operator_console -q
# dev 컨테이너
docker compose -f docker/docker-compose.yml run --rm dev bash -lc \
  'cd /workspace && python3 -m pytest -q'
# 젯슨 parity (repo 동기 후)
```

기준선(2026-07-18 `f2278d4`): 호스트 345 / dev 1192 / ros·젯슨 527.
**증감분을 보고서에 적는다.**

- [ ] **Step 5: 문서**

`docs/reports/2026-07-20-cad-rover-training-track.md` 에 기록한다:
- Task 2 / 3 / 4 각각의 캠페인 결과표 (앵커 이동 분해)
- 최종 캠페인 결과 + 런타임 + 성능 게이트 판정
- 3환경 테스트 수 증감
- **남은 병목**: `drop_boundaries_unobserved` 는 계획 B의 일이다

`powertrain_sim/README.md` 에 훈련 트랙 파라미터와 CAD 모델 출처를 추가한다.

- [ ] **Step 6: 커밋 + 핸드오프 갱신**

```bash
git add powertrain_sim/README.md docs/reports/2026-07-20-cad-rover-training-track.md \
        powertrain_sim/scenarios/
git commit -m "docs(sim): CAD rover + training track results and anchor migration"
```

`docs/reports/2026-07-16-project-state-and-handoff.md` §2 커밋 체인에 행을 추가한다.

---

## 완료 기준

- 시뮬 로봇이 USD 정본과 일치한다 — 축거 · 윤거 · 총질량 66.96 kg · 로커보기 · 디프바
- 부호 검증(V-2) 통과: `v>0 → +X`, `ω>0 → 반시계`, 좌회전 시 좌륜이 느림
- 서스펜션 음성 대조(V-3) 통과: 로커가 실제로 굴절하고, 디프바를 끄면 거동이 달라짐
- 훈련 트랙 15 m × 1.6 m, `pinch` 의 의도적 좁힘 보존, 기복 peak-to-peak 0.30 m
- **fail_open 0 · edge_overrun 0 유지** (깨지면 되돌림)
- 앵커 이동이 질량 / 서스펜션 / 트랙 셋으로 분해되어 기록됨
- 3환경 green + 캠페인 실제 실행 결과 육안 확인

## 계획 B 로 넘기는 것

- `drop_boundaries_unobserved` 병목 규명 · 수정 + 막다른 골목 탈출 경로
- 시각 리포트
- 정직한 합격선 + 대량 시드 스윕
- 품질 열화 플래그 포화 · `scenario_id` 라벨링 버그
