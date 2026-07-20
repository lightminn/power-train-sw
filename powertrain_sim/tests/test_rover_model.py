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
