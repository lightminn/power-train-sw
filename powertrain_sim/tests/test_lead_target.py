from __future__ import annotations

import math

import pytest

from powertrain_sim.fixtures import GroundTruthFrame
from powertrain_sim.follow_loop import FollowDriver
from powertrain_sim.lead_target import LeadTargetPlant, LeadTargetSpec


def _robot_pose(*, x_m=0.0, y_m=0.0, yaw_rad=0.0):
    return GroundTruthFrame(
        stamp_s=1.0,
        x_m=x_m,
        y_m=y_m,
        z_m=0.5,
        yaw_rad=yaw_rad,
        bank_rad=0.0,
        linear_speed_m_s=0.0,
        yaw_rate_rad_s=0.0,
    )


def test_pose_advances_deterministically_along_track_centerline():
    plant = LeadTargetPlant(
        LeadTargetSpec(path="curve", speed_m_s=0.5),
        centerline_m=((0.0, 0.0, 0.5), (3.0, 0.0, 0.5), (3.0, 4.0, 0.5)),
        seed=17,
        initial_offset_m=2.0,
    )

    assert plant.pose(0.0) == plant.pose(0.0)
    assert plant.pose(0.0).x_m == pytest.approx(2.0)
    assert plant.pose(0.0).y_m == pytest.approx(0.0)
    assert plant.pose(4.0).x_m == pytest.approx(3.0)
    assert plant.pose(4.0).y_m == pytest.approx(1.0)
    assert plant.pose(4.0).yaw_rad == pytest.approx(math.pi / 2.0)


def test_detections_source_synthesizes_follow_tuple_in_robot_frame():
    plant = LeadTargetPlant(
        LeadTargetSpec(path="straight", speed_m_s=0.0),
        centerline_m=((0.0, 0.0, 0.5), (10.0, 0.0, 0.5)),
        seed=3,
        initial_offset_m=3.0,
    )

    detections = plant.detections_source(
        0.0,
        _robot_pose(x_m=1.0, y_m=-1.0, yaw_rad=0.0),
    )

    assert len(detections) == 1
    name, confidence, forward_m, left_m, bbox_area_px = detections[0]
    assert name == "robot"
    assert confidence == pytest.approx(0.95)
    assert forward_m == pytest.approx(2.0)
    assert left_m == pytest.approx(1.0)
    assert bbox_area_px > 0.0


def test_occlusion_and_seeded_dropout_are_deterministic():
    spec = LeadTargetSpec(
        path="straight",
        speed_m_s=0.2,
        occlusions=((1.0, 2.0),),
        dropout_ratio=0.4,
    )
    first = LeadTargetPlant(
        spec,
        centerline_m=((0.0, 0.0, 0.5), (20.0, 0.0, 0.5)),
        seed=123,
    )
    second = LeadTargetPlant(
        spec,
        centerline_m=((0.0, 0.0, 0.5), (20.0, 0.0, 0.5)),
        seed=123,
    )
    robot = _robot_pose()

    assert first.detections_source(1.5, robot) == []
    first_run = [first.detections_source(index * 0.1, robot) for index in range(40)]
    second_run = [second.detections_source(index * 0.1, robot) for index in range(40)]
    repeated = [first.detections_source(index * 0.1, robot) for index in range(40)]

    assert first_run == second_run == repeated
    assert any(not detections for detections in first_run)
    assert any(detections for detections in first_run)


@pytest.mark.parametrize(
    "spec",
    [
        LeadTargetSpec(path="straight", speed_m_s=0.5),
        LeadTargetSpec(path="curve", speed_m_s=0.5),
    ],
)
def test_supported_paths_are_explicit(spec):
    assert spec.path in {"straight", "curve"}


def test_follow_driver_ticks_production_core_from_cached_observation():
    target = LeadTargetPlant(
        LeadTargetSpec(path="straight", speed_m_s=0.5),
        centerline_m=((0.0, 0.0, 0.5), (20.0, 0.0, 0.5)),
        seed=7,
    )
    driver = FollowDriver(target)
    robot = _robot_pose()

    detections = driver.detections_source(0.0, robot)
    first_command = driver.command(0.0, None)
    driver.detections_source(0.02, robot)
    second_command = driver.command(0.02, None)

    assert detections and len(detections[0]) == 5
    assert first_command == (0.0, 0.0)
    assert second_command[0] >= 0.0
    assert driver.follow_state == "TRACKING"
    assert driver.hold_state(0.02, None) == (False, False)


def test_follow_driver_maps_lost_state_to_hold():
    target = LeadTargetPlant(
        LeadTargetSpec(
            path="straight",
            speed_m_s=0.5,
            occlusions=((0.1, 2.0),),
        ),
        centerline_m=((0.0, 0.0, 0.5), (20.0, 0.0, 0.5)),
        seed=9,
    )
    driver = FollowDriver(target)
    robot = _robot_pose()
    for t_s in (0.0, 0.02):
        driver.detections_source(t_s, robot)
        driver.command(t_s, None)

    driver.detections_source(1.0, robot)
    command = driver.command(1.0, None)

    assert command == (0.0, 0.0)
    assert driver.follow_state == "LOST"
    assert driver.hold_state(1.0, None) == (True, True)


def test_runner_calls_detection_before_command_and_records_lead_channels(tmp_path):
    pytest.importorskip("mujoco")
    from powertrain_sim.mujoco_fast.runner import run_scenario
    from powertrain_sim.procedural import GenerationParameters, generate_scenario
    from powertrain_sim.recording import DetectionFrame, RecordedRun
    from powertrain_sim.scenario import parse_scenario

    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(4.0, 4.0),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            expected_completion=False,
        ),
        seed=4,
        seed_class="dev",
    )
    document["clock"]["duration_s"] = 0.1
    document["faults"] = {name: [] for name in document["faults"]}
    scenario = parse_scenario(document)
    target = LeadTargetPlant(
        LeadTargetSpec(path="straight", speed_m_s=0.5),
        centerline_m=scenario.track.centerline_m,
        seed=scenario.prng.seed,
    )
    order = []

    class OrderedDriver(FollowDriver):
        def detections_source(self, elapsed_s, robot_pose):
            order.append((elapsed_s, "observation"))
            return super().detections_source(elapsed_s, robot_pose)

        def command(self, elapsed_s, snapshot):
            order.append((elapsed_s, "decision"))
            return super().command(elapsed_s, snapshot)

    driver = OrderedDriver(target)
    run_directory = tmp_path / "follow-hook"

    run_scenario(
        scenario,
        run_directory,
        detections_source=driver.detections_source,
        command_source=driver.command,
        hold_state_source=driver.hold_state,
    )

    assert order == [
        item
        for index in range(scenario.clock.sample_count)
        for item in (
            (index * scenario.clock.dt_s, "observation"),
            (index * scenario.clock.dt_s, "decision"),
        )
    ]
    frames = [
        record.value
        for record in RecordedRun(run_directory).iter_records()
        if isinstance(record.value, DetectionFrame)
    ]
    assert len(frames) == scenario.clock.sample_count
    assert all(frame.lead_distance_m is not None for frame in frames)
    assert {frame.follow_state for frame in frames} <= {
        "TRACKING",
        "REACQUIRING",
    }
