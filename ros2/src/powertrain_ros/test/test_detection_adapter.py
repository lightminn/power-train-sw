"""Pure perception frame and marker-dedup contract tests (WP5.2 Task 6)."""

import ast
import math
from pathlib import Path

import pytest

from powertrain_ros import detection_adapter as detection


MODULE = (
    Path(__file__).resolve().parents[1]
    / "powertrain_ros"
    / "detection_adapter.py"
)
IDENTITY_4X4 = (
    (1.0, 0.0, 0.0, 0.0),
    (0.0, 1.0, 0.0, 0.0),
    (0.0, 0.0, 1.0, 0.0),
    (0.0, 0.0, 0.0, 1.0),
)


def _bbox():
    return detection.DetectionBBox(
        x_offset=10,
        y_offset=20,
        width=30,
        height=40,
        do_rectify=False,
    )


def _object(
    *,
    class_id=7,
    class_name="marker_a",
    confidence=0.9,
    position=(1.0, 0.0, 0.0),
    yaw=0.0,
):
    return detection.DetectedObjectValue(
        class_id=class_id,
        class_name=class_name,
        confidence=confidence,
        pose=detection.DetectionPose(*position, yaw=yaw),
        bbox=_bbox(),
    )


def _array(*objects, frame_id="d435_color_optical_frame", stamp_s=10.0):
    return detection.DetectedObjectArrayValue(
        header=detection.DetectionHeader(frame_id=frame_id, stamp_s=stamp_s),
        objects=objects or (_object(),),
    )


def _adapter(
    tf_lookup=lambda _frame_id, _stamp_s: IDENTITY_4X4,
    *,
    timeout_s=0.5,
    cluster_radius_m=0.5,
    min_reobservation_s=0.2,
    confidence_min=0.6,
    unique_class_id_contract=False,
):
    return detection.DetectionAdapter(
        tf_lookup,
        detection.DetectionAdapterConfig(
            timeout_s=timeout_s,
            cluster_radius_m=cluster_radius_m,
            min_reobservation_s=min_reobservation_s,
            confidence_min=confidence_min,
            unique_class_id_contract=unique_class_id_contract,
        ),
    )


@pytest.mark.parametrize(
    "frame_id, stamp_s, now_s, expected_reason",
    (
        ("", 10.0, 10.0, "frame_id_empty"),
        ("d435_color_optical_frame", 0.0, 10.0, "stamp_zero"),
        ("d435_color_optical_frame", 10.01, 10.0, "stamp_future"),
        ("d435_color_optical_frame", 9.49, 10.0, "stamp_stale"),
    ),
)
def test_invalid_frame_and_stamp_return_zero_observations_and_hold_reason(
    frame_id,
    stamp_s,
    now_s,
    expected_reason,
):
    calls = []
    adapter = _adapter(
        lambda frame, stamp: calls.append((frame, stamp)) or IDENTITY_4X4
    )

    result = adapter.process(
        _array(frame_id=frame_id, stamp_s=stamp_s),
        now_s=now_s,
    )

    assert result.observations == ()
    assert result.hold_reason == expected_reason
    assert calls == []
    assert adapter.dedup_state().total_markers == 0


@pytest.mark.parametrize("failure", ("exception", "none"))
def test_tf_failure_returns_zero_observations_and_hold_reason(failure):
    if failure == "exception":
        def lookup(_frame_id, _stamp_s):
            raise RuntimeError("TF buffer unavailable")
    else:
        def lookup(_frame_id, _stamp_s):
            return None

    adapter = _adapter(lookup)
    result = adapter.process(_array(), now_s=10.0)

    assert result.observations == ()
    assert result.hold_reason == (
        "tf_lookup_failed" if failure == "exception" else "tf_unavailable"
    )
    assert adapter.dedup_state().total_markers == 0


def test_optical_frame_4x4_transform_rotates_position_and_yaw_into_base_link():
    calls = []
    transform = (
        (0.0, -1.0, 0.0, 1.0),
        (1.0, 0.0, 0.0, 2.0),
        (0.0, 0.0, 1.0, 3.0),
        (0.0, 0.0, 0.0, 1.0),
    )
    adapter = _adapter(
        lambda frame, stamp: calls.append((frame, stamp)) or transform
    )

    result = adapter.process(
        _array(
            _object(position=(2.0, 0.0, 1.0), yaw=0.25),
            stamp_s=9.8,
        ),
        now_s=10.0,
    )

    assert result.hold_reason == ""
    assert calls == [("d435_color_optical_frame", 9.8)]
    assert len(result.observations) == 1
    observation = result.observations[0]
    assert observation.frame_id == "base_link"
    assert observation.source_frame_id == "d435_color_optical_frame"
    assert observation.stamp_s == 9.8
    assert (observation.x, observation.y, observation.z) == pytest.approx(
        (1.0, 4.0, 4.0)
    )
    assert observation.yaw == pytest.approx(0.25 + math.pi / 2.0)
    assert observation.class_id == 7
    assert observation.class_name == "marker_a"
    assert observation.confidence == pytest.approx(0.9)
    assert observation.bbox == _bbox()
    assert observation.marker_key


def test_rotation_translation_pair_is_an_accepted_tf_callback_value():
    rotation = (
        (1.0, 0.0, 0.0),
        (0.0, 1.0, 0.0),
        (0.0, 0.0, 1.0),
    )
    adapter = _adapter(lambda _frame, _stamp: (rotation, (0.5, -0.5, 1.0)))

    result = adapter.process(
        _array(_object(position=(1.0, 2.0, 3.0), yaw=-0.4)),
        now_s=10.0,
    )

    observation = result.observations[0]
    assert (observation.x, observation.y, observation.z) == pytest.approx(
        (1.5, 1.5, 4.0)
    )
    assert observation.yaw == pytest.approx(-0.4)


def test_same_named_marker_inside_radius_reuses_one_stable_marker_key():
    adapter = _adapter(cluster_radius_m=0.5, min_reobservation_s=0.2)

    first = adapter.process(
        _array(_object(position=(1.0, 0.0, 0.0)), stamp_s=10.0),
        now_s=10.0,
    )
    second = adapter.process(
        _array(_object(position=(1.2, 0.1, 0.1)), stamp_s=10.3),
        now_s=10.3,
    )

    assert len(first.observations) == len(second.observations) == 1
    assert first.observations[0].marker_key == second.observations[0].marker_key
    state = adapter.dedup_state()
    assert state.total_markers == 1
    assert state.counts_by_class == {"marker_a": 1}
    assert state.markers[0].accepted_observations == 2
    assert state.markers[0].position == pytest.approx((1.0, 0.0, 0.0))


def test_same_named_marker_outside_radius_creates_a_new_marker():
    adapter = _adapter(cluster_radius_m=0.5, min_reobservation_s=0.0)

    first = adapter.process(
        _array(_object(position=(0.0, 0.0, 0.0)), stamp_s=10.0),
        now_s=10.0,
    )
    second = adapter.process(
        _array(_object(position=(0.51, 0.0, 0.0)), stamp_s=10.1),
        now_s=10.1,
    )

    assert first.observations[0].marker_key != second.observations[0].marker_key
    state = adapter.dedup_state()
    assert state.total_markers == 2
    assert state.counts_by_class == {"marker_a": 2}


def test_unique_class_id_contract_uses_id_before_name_or_position_cluster():
    adapter = _adapter(
        unique_class_id_contract=True,
        cluster_radius_m=0.01,
        min_reobservation_s=0.0,
    )

    first = adapter.process(
        _array(_object(class_id=7, position=(0.0, 0.0, 0.0)), stamp_s=10.0),
        now_s=10.0,
    )
    same_id_far = adapter.process(
        _array(_object(class_id=7, position=(5.0, 0.0, 0.0)), stamp_s=10.1),
        now_s=10.1,
    )
    other_id_near = adapter.process(
        _array(_object(class_id=8, position=(0.0, 0.0, 0.0)), stamp_s=10.2),
        now_s=10.2,
    )

    assert first.observations[0].marker_key == same_id_far.observations[0].marker_key
    assert first.observations[0].marker_key != other_id_near.observations[0].marker_key
    assert adapter.dedup_state().total_markers == 2


def test_reobservation_before_minimum_time_is_ignored_without_updating_state():
    adapter = _adapter(min_reobservation_s=0.5)
    first = adapter.process(
        _array(_object(position=(1.0, 0.0, 0.0)), stamp_s=10.0),
        now_s=10.0,
    )

    too_soon = adapter.process(
        _array(_object(position=(1.1, 0.0, 0.0)), stamp_s=10.49),
        now_s=10.49,
    )

    assert len(first.observations) == 1
    assert too_soon.hold_reason == ""
    assert too_soon.observations == ()
    state = adapter.dedup_state()
    assert state.total_markers == 1
    assert state.markers[0].accepted_observations == 1
    assert state.markers[0].last_seen_s == 10.0


def test_detection_below_confidence_floor_is_excluded_without_a_hold():
    adapter = _adapter(confidence_min=0.8)

    result = adapter.process(
        _array(_object(confidence=0.799)),
        now_s=10.0,
    )

    assert result.hold_reason == ""
    assert result.observations == ()
    assert adapter.dedup_state().total_markers == 0


def test_detection_adapter_core_imports_no_ros_or_ros_message_packages():
    tree = ast.parse(MODULE.read_text(encoding="utf-8"))
    imported_roots = set()
    for node in ast.walk(tree):
        if isinstance(node, ast.Import):
            imported_roots.update(alias.name.split(".")[0] for alias in node.names)
        elif isinstance(node, ast.ImportFrom) and node.module:
            imported_roots.add(node.module.split(".")[0])

    assert imported_roots.isdisjoint(
        {"rclpy", "robot_arm_msgs", "geometry_msgs", "sensor_msgs", "std_msgs"}
    )
