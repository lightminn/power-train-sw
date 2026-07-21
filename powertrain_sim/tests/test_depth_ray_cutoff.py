"""Depth raycast cutoff regression: mj_multiRay must not prune the floor plane.

6 m 벽 근본 원인 회귀 핀 — 스펙 docs/superpowers/specs/
2026-07-21-sim-depth-floor-pruning-6m-wall-design.md §2/§4.
"""
from __future__ import annotations

import mujoco
import numpy as np
import pytest

from powertrain_sim.campaign import build_family_document
from powertrain_sim.mujoco_fast.plant import MujocoFastPlant
from powertrain_sim.mujoco_fast.sensors import FastSensorSuite, MAX_VALID_DEPTH_M
from powertrain_sim.scenario import parse_scenario


GEOM_GROUP = np.array((1, 0, 0, 0, 0, 0), dtype=np.uint8)


def _flat_suite_at(x_m: float) -> tuple[MujocoFastPlant, FastSensorSuite]:
    """Flat dev scenario, depth noise off, base teleported to x_m (no stepping)."""
    document = build_family_document("flat", seed=0, seed_class="dev")
    document["sensors"]["depth"]["noise_std_m"] = 0.0
    scenario = parse_scenario(document)
    plant = MujocoFastPlant(scenario)
    suite = FastSensorSuite(scenario, plant)
    model, data = plant.model, plant.data
    qpos_address = model.jnt_qposadr[model.body_jntadr[plant.base_body_id]]
    data.qpos[qpos_address] = x_m
    mujoco.mj_forward(model, data)
    return plant, suite


def _ground_truth_axial(
    plant: MujocoFastPlant, suite: FastSensorSuite, flat_index: int
) -> tuple[float, int]:
    """Single-ray mj_ray ground truth with the sensor's own range mask."""
    body_matrix = np.asarray(
        plant.data.xmat[plant.base_body_id], dtype=float
    ).reshape(3, 3)
    origin = np.asarray(plant.data.site_xpos[plant.depth_site_id], dtype=float)
    direction = suite._depth_directions_body[flat_index] @ body_matrix.T
    geom_id = np.zeros(1, dtype=np.int32)
    distance = mujoco.mj_ray(
        plant.model,
        plant.data,
        origin,
        np.ascontiguousarray(direction),
        GEOM_GROUP,
        True,
        plant.base_body_id,
        geom_id,
    )
    hit_geom_id = int(geom_id[0])
    if distance < 0.0 or distance > MAX_VALID_DEPTH_M:
        return 0.0, hit_geom_id
    return float(distance * suite._depth_axial_cos[flat_index]), hit_geom_id


def _frame_vs_ground_truth(
    x_m: float,
) -> tuple[MujocoFastPlant, np.ndarray, np.ndarray, np.ndarray]:
    plant, suite = _flat_suite_at(x_m)
    frame = suite.sample_depth(0)
    assert frame is not None
    height, width = frame.depth_roi.shape
    sensor = frame.depth_roi.astype(float) * frame.depth_scale_m
    sampled_sensor = []
    sampled_truth = []
    sampled_geom_ids = []
    for row in range(0, height, 8):
        for col in range(0, width, 8):
            sampled_sensor.append(sensor[row, col])
            axial, geom_id = _ground_truth_axial(
                plant, suite, row * width + col
            )
            sampled_truth.append(axial)
            sampled_geom_ids.append(geom_id)
    return (
        plant,
        np.asarray(sampled_sensor),
        np.asarray(sampled_truth),
        np.asarray(sampled_geom_ids),
    )


def test_depth_matches_single_ray_ground_truth_far_from_origin():
    """카메라가 월드 원점에서 6 m 를 넘어도 depth 는 mj_ray 와 등가여야 한다.

    수정 전에는 mj_multiRay 앵커-거리 프루닝이 lower_floor plane 을 통째로
    제외해 측면 바닥 픽셀이 전부 0 이 된다 (6 m 벽의 근본 원인)."""
    plant, sensor, truth, truth_geom_ids = _frame_vs_ground_truth(x_m=8.0)
    lower_floor_id = mujoco.mj_name2id(
        plant.model, mujoco.mjtObj.mjOBJ_GEOM, "lower_floor"
    )
    floor_visible = (truth_geom_ids == lower_floor_id) & (truth > 0.0)
    assert np.count_nonzero(floor_visible) > 0
    np.testing.assert_allclose(sensor, truth, atol=2.0e-3)


def test_depth_matches_single_ray_ground_truth_at_spawn():
    """벽 이전 포즈(스폰)에서의 등가성 — 수정이 기존 유효 depth 를 바꾸지 않음."""
    _, sensor, truth, _ = _frame_vs_ground_truth(x_m=0.0)
    np.testing.assert_allclose(sensor, truth, atol=2.0e-3)


@pytest.mark.parametrize("x_m", (0.0, 8.0))
def test_no_depth_beyond_max_valid_range(x_m):
    """수정이 센서 사거리를 확장하면 안 된다 (스펙 §4 V2)."""
    _, suite = _flat_suite_at(x_m)
    frame = suite.sample_depth(0)
    assert frame is not None
    depth_m = frame.depth_roi.astype(float) * frame.depth_scale_m
    assert float(depth_m.max()) <= MAX_VALID_DEPTH_M + 2.0e-3


def test_mujoco_multiray_anchor_pruning_quirk_is_pinned():
    """MuJoCo 3.10.0 quirk 핀: cutoff 프루닝은 geom 앵커점 거리 기준이라,
    원점-앵커 plane 은 원점에서 cutoff 밖 origin 의 컷오프-이내 히트를 잃는다.

    이 테스트가 실패하면 업스트림 의미론이 바뀐 것 — RAY_PRUNING_CUTOFF_M
    상수의 필요성을 재검토하고 스펙 §2 를 갱신할 것."""
    xml = """
    <mujoco><worldbody>
      <geom name="floor" type="plane" pos="0 0 0" size="50 50 0.05" group="0"/>
      <body name="base" pos="0 0 10"><freejoint/>
        <geom type="sphere" size="0.05" group="1" mass="1"/></body>
    </worldbody></mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    origin = np.array((8.0, 0.0, 1.0))
    down = np.array((0.0, 0.0, -1.0))
    geom_id = np.full(1, -1, dtype=np.int32)
    distance = np.full(1, -1.0)
    mujoco.mj_multiRay(
        model, data, origin, np.ascontiguousarray(down).ravel(), GEOM_GROUP,
        True, model.body("base").id, geom_id, distance, None, 1, 6.0,
    )
    single_geom = np.zeros(1, dtype=np.int32)
    single_distance = mujoco.mj_ray(
        model, data, origin, down, GEOM_GROUP, True,
        model.body("base").id, single_geom,
    )
    assert single_distance == pytest.approx(1.0, abs=1e-9)
    assert geom_id[0] == -1  # the quirk: in-cutoff hit dropped by multiRay
