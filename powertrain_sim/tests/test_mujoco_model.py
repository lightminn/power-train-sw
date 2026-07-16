from __future__ import annotations

import math
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import mujoco
import numpy as np
import pytest
import yaml

from chassis.kinematics import default_geometry, solve
from powertrain_ros.state_estimation import ImuSample, WheelSample
from powertrain_sim.fixtures import DepthFrame, GroundTruthFrame
from powertrain_sim.mujoco_fast.model_builder import build_mjcf
from powertrain_sim.mujoco_fast.plant import MujocoFastPlant
from powertrain_sim.mujoco_fast.sensors import FastSensorSuite
from powertrain_sim.scenario import load_scenario, parse_scenario


SCENARIO_DIR = Path(__file__).resolve().parents[1] / "scenarios"


def _load(name: str = "flat_straight_5m.yaml"):
    return load_scenario(SCENARIO_DIR / name)


def test_part_one_modules_import_when_third_party_mujoco_is_unavailable():
    script = """
import builtins
original_import = builtins.__import__
def guarded_import(name, *args, **kwargs):
    if name == 'mujoco' or name.startswith('mujoco.'):
        raise ImportError('mujoco intentionally blocked')
    return original_import(name, *args, **kwargs)
builtins.__import__ = guarded_import
import powertrain_sim
import powertrain_sim.scenario
import powertrain_sim.fixtures
import powertrain_sim.recording
"""

    completed = subprocess.run(
        [sys.executable, "-c", script],
        check=False,
        capture_output=True,
        text=True,
    )

    assert completed.returncode == 0, completed.stderr


def test_mjcf_contains_segment_friction_drop_floor_six_wheels_and_sensors():
    scenario = _load()

    xml = build_mjcf(scenario)
    root = ET.fromstring(xml)
    model = mujoco.MjModel.from_xml_string(xml)

    segment = root.find(".//geom[@name='track_segment_000']")
    assert segment is not None
    assert float(segment.attrib["friction"].split()[0]) == pytest.approx(0.8)
    assert root.find(".//geom[@name='lower_floor']") is not None
    assert len(root.findall(".//joint[@name][@type='hinge']")) == 10
    assert len(root.findall(".//actuator/position")) == 4
    assert len(root.findall(".//actuator/velocity")) == 6
    assert root.find(".//sensor/gyro[@name='imu_gyro']") is not None
    assert root.find(".//sensor/accelerometer[@name='imu_accel']") is not None
    assert root.find(".//sensor/framepos[@name='base_framepos']") is not None
    assert root.find(".//sensor/framequat[@name='base_framequat']") is not None
    assert model.nq > 0


def test_apply_command_is_a_direct_conversion_of_production_solve_result():
    scenario = _load()
    geometry = default_geometry()
    plant = MujocoFastPlant(scenario, geometry=geometry)

    result = plant.apply_command(0.35, 0.22)
    production = solve(geometry, 0.35, 0.22)

    assert result == production
    assert plant.last_solve_result == production
    controls = plant.actuator_controls()
    for wheel in geometry.wheels:
        command = production.wheels[wheel.name]
        assert controls[f"drive_{wheel.name}"] == pytest.approx(
            command.drive_turns_per_s * 2.0 * math.pi
        )
        if wheel.steerable:
            assert controls[f"steer_{wheel.name}"] == pytest.approx(
                math.radians(command.steer_deg)
            )


def test_physics_timestep_is_at_most_five_ms_and_divides_scenario_clock():
    scenario = _load()

    plant = MujocoFastPlant(scenario)

    assert plant.physics_dt_s <= 0.005
    assert plant.substeps_per_clock_step * plant.physics_dt_s == pytest.approx(
        scenario.clock.dt_s,
        abs=1e-12,
    )


def test_value_sensors_reuse_part_one_types_and_fault_schedule_semantics():
    scenario = _load("bank_transition.yaml")
    plant = MujocoFastPlant(scenario)
    sensors = FastSensorSuite(scenario, plant)

    wheel = sensors.sample_wheel(0)
    imu = sensors.sample_imu(0)
    depth = sensors.sample_depth(0)
    truth = sensors.sample_ground_truth(0)

    assert isinstance(wheel, WheelSample)
    assert isinstance(imu, ImuSample)
    assert isinstance(depth, DepthFrame)
    assert isinstance(truth, GroundTruthFrame)
    assert tuple(value.name for value in wheel.wheels) == tuple(
        scenario.sensors["wheel_states"]["wheel_names"]
    )

    imu_dropout_index = round(6.0 / scenario.clock.dt_s)
    assert sensors.sample_imu(imu_dropout_index) is None
    hole_index = round(2.0 / scenario.clock.dt_s)
    hole = sensors.sample_depth(hole_index)
    assert hole is not None
    assert np.all(hole.depth_roi[15:25, 25:35] == 0)


def test_headless_depth_rays_hit_track_then_lower_floor_outside_drop_boundary():
    scenario = _load()
    plant = MujocoFastPlant(scenario)
    sensors = FastSensorSuite(scenario, plant)

    on_track = sensors.sample_depth(0)
    assert on_track is not None
    centre = on_track.depth_roi[19:21, 29:31].astype(float) * on_track.depth_scale_m
    # Camera is 0.28 m above the surface and pitched down 25 degrees: the
    # centre ray hits at range 0.28/sin(25 deg) = 0.663 m, and stored depth is
    # optical-axis Z (= range at the optical centre pixel).
    assert float(np.median(centre)) == pytest.approx(0.663, abs=0.035)

    free_qpos = int(plant.model.jnt_qposadr[plant.root_free_joint_id])
    plant.data.qpos[free_qpos + 1] = 1.2
    mujoco.mj_forward(plant.model, plant.data)
    below_floor = sensors.sample_depth(0)
    assert below_floor is not None
    outside_centre = (
        below_floor.depth_roi[19:21, 29:31].astype(float)
        * below_floor.depth_scale_m
    )
    assert np.all(outside_centre > 1.4)
    assert np.all(outside_centre < 2.0)


def test_depth_spike_is_injected_over_a_ray_miss_like_part_one_fixture():
    document = yaml.safe_load(
        (SCENARIO_DIR / "flat_straight_5m.yaml").read_text(encoding="utf-8")
    )
    document["faults"]["depth_spikes"] = [
        {
            "row": 20,
            "col": 30,
            "offset_m": 2.5,
            "start_s": 0.0,
            "end_s": 0.1,
        }
    ]
    scenario = parse_scenario(document)
    plant = MujocoFastPlant(scenario)
    free_qpos = int(plant.model.jnt_qposadr[plant.root_free_joint_id])
    plant.data.qpos[free_qpos + 2] = 10.0
    mujoco.mj_forward(plant.model, plant.data)

    frame = FastSensorSuite(scenario, plant).sample_depth(0)

    assert frame is not None
    assert frame.depth_roi[20, 30] == 2500
