"""ROS adapter tests for the WP8 temporary fake section-event bridge."""

import ast
import importlib.util
import json
from pathlib import Path
import time

import pytest

from chassis.section_profiles import SectionConfig


PACKAGE = Path(__file__).resolve().parents[1]
MODULE = PACKAGE / "powertrain_ros" / "section_supervisor_node.py"
HAS_ROS = importlib.util.find_spec("rclpy") is not None

if HAS_ROS:
    import rclpy
    from geometry_msgs.msg import TransformStamped
    from rclpy.duration import Duration
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.time import Time
    from std_msgs.msg import String
    from tf2_ros import StaticTransformBroadcaster

    from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray
    from powertrain_ros.section_supervisor_node import SectionSupervisorNode


requires_ros = pytest.mark.skipif(not HAS_ROS, reason="host has no rclpy")


class EventClient:
    def __init__(self):
        self.events = []

    def emit(self, event):
        self.events.append(event)
        return True


@pytest.fixture(scope="module", autouse=True)
def ros():
    if not HAS_ROS:
        yield
        return
    rclpy.init()
    yield
    rclpy.shutdown()


class Harness:
    def __init__(self, section_node):
        self.node = rclpy.create_node("section_supervisor_test_harness")
        self.events = self.node.create_publisher(String, "/section_events", 10)
        self.detections = self.node.create_publisher(
            DetectedObjectArray,
            "/detected_objects",
            10,
        )
        self.states = []
        self.node.create_subscription(
            String,
            "/section/state",
            lambda message: self.states.append(json.loads(message.data)),
            10,
        )
        self.static_tf = StaticTransformBroadcaster(self.node)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(section_node)
        self.executor.add_node(self.node)

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    def spin_until(self, predicate, timeout=2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("condition did not become true before timeout")

    def publish_static_translation(
        self,
        section_node,
        *,
        child_frame="camera_link",
        translation_x=2.0,
    ):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = child_frame
        transform.transform.translation.x = float(translation_x)
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)
        self.spin_until(
            lambda: section_node.tf_buf.can_transform(
                "base_link",
                child_frame,
                Time(),
            )
        )

    def close(self, section_node):
        self.executor.remove_node(section_node)
        self.executor.remove_node(self.node)
        section_node.destroy_node()
        self.node.destroy_node()
        self.executor.shutdown()


def _node(section, *, enabled=True, event_client=None):
    return SectionSupervisorNode(
        event_client=event_client,
        parameter_overrides=[
            Parameter("section", value=section),
            Parameter("enabled", value=enabled),
        ],
    )


def _event(node, event_type, **payload):
    return String(
        data=json.dumps(
            {
                "type": event_type,
                "stamp_s": node.get_clock().now().nanoseconds * 1e-9,
                "payload": payload,
            },
            separators=(",", ":"),
        )
    )


def _detection(node, *, frame_id="camera_link", class_id=7):
    message = DetectedObjectArray()
    message.header.frame_id = frame_id
    message.header.stamp = node.get_clock().now().to_msg()
    detected = DetectedObject()
    detected.class_id = int(class_id)
    detected.class_name = "marker_a"
    detected.confidence = 0.9
    detected.pose.position.x = 1.0
    detected.pose.orientation.w = 1.0
    message.objects.append(detected)
    return message


@requires_ros
def test_fake_json_event_publishes_state_and_observability_journal():
    event_client = EventClient()
    section_node = _node("RELIEF", event_client=event_client)
    harness = Harness(section_node)
    try:
        harness.spin_for(0.15)
        previous = len(harness.states)
        harness.events.publish(_event(section_node, "LIGHT_RED"))
        harness.spin_until(
            lambda: len(harness.states) > previous
            and harness.states[-1]["hold_hint"] is True
        )

        state = harness.states[-1]
        assert state["section"] == "RELIEF"
        assert state["phase"] == "EVENT_HOLD"
        assert state["speed_hint"] is None
        assert state["notices"] == []
        assert state["unique_markers"] == 0
        assert state["complete"] is False
        assert event_client.events[-1]["event_type"] == "SECTION_EVENT"
        assert event_client.events[-1]["payload"]["section_event_type"] == (
            "LIGHT_RED"
        )
    finally:
        harness.close(section_node)


@requires_ros
def test_marker_detection_uses_timestamped_tf_before_dedup():
    section_node = _node("MARKERS", enabled=False, event_client=EventClient())
    harness = Harness(section_node)
    try:
        harness.spin_for(0.15)
        harness.publish_static_translation(section_node)

        harness.detections.publish(_detection(section_node))
        harness.spin_until(lambda: section_node.supervisor.unique_markers == 1)

        record = section_node.supervisor.marker_dedup.successes[-1]
        assert record.position == pytest.approx((3.0, 0.0, 0.0))
        assert record.class_id == 7
        harness.spin_until(
            lambda: harness.states
            and harness.states[-1]["unique_markers"] == 1
        )
    finally:
        harness.close(section_node)


@requires_ros
def test_empty_marker_frame_is_ignored_without_changing_count():
    section_node = _node("MARKERS", enabled=False, event_client=EventClient())
    harness = Harness(section_node)
    try:
        harness.spin_for(0.15)
        harness.detections.publish(_detection(section_node, frame_id=""))
        harness.spin_for(0.20)
        assert section_node.supervisor.unique_markers == 0
    finally:
        harness.close(section_node)


@requires_ros
def test_invalid_section_parameter_fails_node_startup():
    with pytest.raises(ValueError, match="section"):
        _node("UNCONFIRMED_SECTION")


@requires_ros
def test_disabled_node_publishes_state_without_observability_side_effect():
    event_client = EventClient()
    section_node = _node("FOLLOW", enabled=False, event_client=event_client)
    harness = Harness(section_node)
    try:
        harness.spin_for(0.15)
        previous = len(harness.states)
        harness.events.publish(_event(section_node, "LEAD_LOST"))
        harness.spin_until(
            lambda: len(harness.states) > previous
            and harness.states[-1]["hold_hint"] is True
        )

        assert event_client.events == []
        assert harness.states[-1]["enabled"] is False
        assert harness.states[-1]["phase"] == "EVENT_HOLD"
    finally:
        harness.close(section_node)


@requires_ros
def test_versioned_state_has_fixed_session_and_increasing_sequence():
    section_node = _node("FOLLOW", enabled=False, event_client=EventClient())
    harness = Harness(section_node)
    try:
        harness.spin_until(lambda: len(harness.states) >= 2)

        first, second = harness.states[-2:]
        assert first["schema_version"] == 1
        assert len(first["session_id"]) == 32
        assert int(first["session_id"], 16) >= 0
        assert second["session_id"] == first["session_id"]
        assert second["sequence"] > first["sequence"]
        assert second["stamp_s"] >= first["stamp_s"]
        assert first["ttl_s"] == pytest.approx(0.6)
        assert second["ttl_s"] == pytest.approx(0.6)
    finally:
        harness.close(section_node)


@requires_ros
def test_stale_marker_stamp_is_ignored_even_when_tf_exists():
    section_node = _node("MARKERS", enabled=False, event_client=EventClient())
    harness = Harness(section_node)
    try:
        harness.spin_for(0.15)
        harness.publish_static_translation(section_node)
        message = _detection(section_node)
        message.header.stamp = (
            section_node.get_clock().now() - Duration(seconds=0.6)
        ).to_msg()

        harness.detections.publish(message)
        harness.spin_for(0.20)
        assert section_node.supervisor.unique_markers == 0
    finally:
        harness.close(section_node)


def test_node_module_and_console_entry_are_declared():
    assert MODULE.is_file()
    setup_tree = ast.parse((PACKAGE / "setup.py").read_text(encoding="utf-8"))
    strings = {
        node.value
        for node in ast.walk(setup_tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert (
        "section_supervisor = "
        "powertrain_ros.section_supervisor_node:main"
    ) in strings

    package_xml = (PACKAGE / "package.xml").read_text(encoding="utf-8")
    assert "<exec_depend>nav_msgs</exec_depend>" in package_xml


def test_node_source_marks_fake_contract_and_has_no_chassis_control_topics():
    source = MODULE.read_text(encoding="utf-8")
    assert "fake" in source.lower()
    assert "MissionSupervisor" in source
    assert "/section_events" in source
    assert "/section/state" in source
    assert "/autonomy/cmd_vel" not in source
    assert "/cmd_vel" not in source
    assert "request_work(" not in source


def test_versioned_state_source_and_default_ttl_contract():
    source = MODULE.read_text(encoding="utf-8")
    assert SectionConfig().state_ttl_s == pytest.approx(0.6)
    assert "uuid.uuid4().hex" in source
    assert "self._section_sequence += 1" in source
    assert '"schema_version": 1' in source
    for field in ("session_id", "sequence", "stamp_s", "ttl_s"):
        assert f'"{field}"' in source
