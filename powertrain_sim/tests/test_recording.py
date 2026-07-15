from __future__ import annotations

from dataclasses import replace
from importlib import import_module
from pathlib import Path

import numpy as np
import pytest

from chassis.kinematics import default_geometry
from powertrain_observability.events import decode_event
from powertrain_ros.state_estimation import StateEstimator, StateEstimatorConfig
from powertrain_sim.fixtures import generate_fixture
from powertrain_sim.scenario import load_scenario


SCENARIO_DIR = Path(__file__).resolve().parents[1] / "scenarios"


def _recording_module():
    return import_module("powertrain_sim.recording")


def _fixture(name="flat_straight_5m.yaml"):
    return generate_fixture(load_scenario(SCENARIO_DIR / name))


def test_stream_files_merge_by_stamp_then_global_sequence_and_replay_deterministically(
    tmp_path,
):
    module = _recording_module()
    fixture = _fixture("pivot_90deg.yaml")
    wheel = fixture.wheel_states[0]
    imu = fixture.imu[0]
    depth = fixture.depth[0]
    truth = fixture.ground_truth[0]
    detection_stamp = wheel.stamp_s + 0.01

    with module.RunWriter(tmp_path, run_id="run-order") as writer:
        writer.write_imu(imu)
        writer.write_wheel(wheel)
        writer.write_detections(
            stamp_s=detection_stamp,
            frame_id="base_link",
            detections=({"class_id": "marker", "confidence": 0.9},),
        )
        writer.write_depth(depth)
        writer.write_ground_truth(truth)

    run = module.RecordedRun(tmp_path)
    records = list(run.iter_records())
    assert [record.stream for record in records] == [
        "imu",
        "wheel",
        "depth",
        "detections",
    ]
    assert all(records[index].stamp_s <= records[index + 1].stamp_s for index in range(3))
    assert [record.sequence for record in records[:3]] == [0, 1, 3]

    def replay_trace():
        trace = []
        module.Replayer(run).replay(
            wheel=lambda sample: trace.append(("wheel", sample)),
            imu=lambda sample: trace.append(("imu", sample)),
            depth=lambda frame: trace.append(("depth", frame.depth_roi.copy())),
            detections=lambda frame: trace.append(("detections", frame)),
        )
        return trace

    first = replay_trace()
    second = replay_trace()
    assert [stream for stream, _ in first] == ["imu", "wheel", "depth", "detections"]
    assert [stream for stream, _ in second] == [stream for stream, _ in first]
    for (first_stream, first_value), (second_stream, second_value) in zip(first, second):
        assert first_stream == second_stream
        if first_stream == "depth":
            np.testing.assert_array_equal(first_value, second_value)
        else:
            assert first_value == second_value

    ground_truth = list(run.iter_ground_truth())
    assert ground_truth == [truth]
    assert all(record.stream != "ground_truth" for record in records)


def test_depth_is_npz_backed_and_jsonl_uses_existing_canonical_event_encoding(tmp_path):
    module = _recording_module()
    frame = _fixture().depth[0]

    with module.RunWriter(tmp_path, run_id="run-depth") as writer:
        writer.write_depth(frame)

    event_line = (tmp_path / "depth.jsonl").read_bytes().splitlines()[0]
    event = decode_event(event_line)
    assert event["run_id"] == "run-depth"
    assert event["source"] == "depth"
    assert event["payload"]["npz_path"].startswith("depth/")
    npz_path = tmp_path / event["payload"]["npz_path"]
    assert npz_path.suffix == ".npz"
    assert npz_path.exists()

    replayed = list(module.RecordedRun(tmp_path).iter_records())[0].value
    np.testing.assert_array_equal(replayed.depth_roi, frame.depth_roi)
    assert replayed.depth_scale_m == frame.depth_scale_m
    assert replayed.intrinsics == frame.intrinsics


def test_reader_ignores_only_an_incomplete_final_jsonl_record(tmp_path):
    module = _recording_module()
    fixture = _fixture("pivot_90deg.yaml")

    with module.RunWriter(tmp_path, run_id="run-tail") as writer:
        writer.write_wheel(fixture.wheel_states[0])
        writer.write_imu(fixture.imu[0])
    with (tmp_path / "imu.jsonl").open("ab") as stream:
        stream.write(b'{"schema_version":1,"run_id":"interrupted')

    records = list(module.RecordedRun(tmp_path).iter_records())

    assert [record.stream for record in records] == ["wheel", "imu"]
    assert [record.sequence for record in records] == [0, 1]


def _replay_estimator(module, run):
    estimator = StateEstimator(default_geometry(), StateEstimatorConfig(bias_samples=0))
    snapshots = []

    def wheel(sample):
        assert estimator.update_wheels(sample, now_s=sample.stamp_s).accepted
        snapshots.append(estimator.snapshot(now_s=sample.stamp_s))

    def imu(sample):
        assert estimator.update_imu(sample, now_s=sample.stamp_s).accepted
        snapshots.append(estimator.snapshot(now_s=sample.stamp_s))

    module.Replayer(run).replay(wheel=wheel, imu=imu)
    return snapshots


def test_same_recording_produces_identical_production_estimator_snapshots(tmp_path):
    module = _recording_module()
    fixture = _fixture()

    with module.RunWriter(tmp_path, run_id="run-estimator") as writer:
        for wheel, imu in zip(fixture.wheel_states, fixture.imu):
            writer.write_wheel(wheel)
            writer.write_imu(imu)

    run = module.RecordedRun(tmp_path)
    first = _replay_estimator(module, run)
    second = _replay_estimator(module, run)

    assert first == second
    assert len(first) == 2 * len(fixture.wheel_states)
    assert first[-1].distance_m == pytest.approx(5.0, rel=0.01)


def test_writer_rejects_nonfinite_values_before_creating_a_complete_record(tmp_path):
    module = _recording_module()

    with module.RunWriter(tmp_path, run_id="run-invalid") as writer:
        with pytest.raises(ValueError, match="finite"):
            writer.write_detections(
                stamp_s=1.0,
                frame_id="base_link",
                detections=({"confidence": float("nan")},),
            )

    path = tmp_path / "detections.jsonl"
    assert not path.exists() or path.read_bytes() == b""


def test_depth_npz_files_are_loaded_lazily_during_merged_iteration(tmp_path):
    module = _recording_module()
    first = _fixture("pivot_90deg.yaml").depth[0]
    second = replace(first, stamp_s=first.stamp_s + 0.1)

    with module.RunWriter(tmp_path, run_id="run-lazy") as writer:
        writer.write_depth(first)
        writer.write_depth(second)
    npz_paths = sorted((tmp_path / "depth").glob("*.npz"))
    assert len(npz_paths) == 2
    npz_paths[1].unlink()

    records = module.RecordedRun(tmp_path).iter_records()
    assert next(records).stamp_s == first.stamp_s
    with pytest.raises(FileNotFoundError):
        next(records)
