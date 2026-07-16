"""lead_follower_node — timestamped TF gate and command publication contract."""
import math
import time

import pytest
import rclpy
from geometry_msgs.msg import Point, TransformStamped, Twist
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.time import Time
from std_msgs.msg import Float32MultiArray
from tf2_ros import StaticTransformBroadcaster

from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray
from powertrain_ros.lead_follower_node import (
    FOLLOW_STATE_LOST,
    FOLLOW_STATE_PREDICTING,
    FOLLOW_STATE_REACQUIRING,
    FOLLOW_STATE_TRACKING,
    LeadFollowerNode,
    _apply_tf,
)
from chassis.follow import FollowResult


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _detection(
    node, *, frame_id="camera_link", age_s=0.0, forward_m=3.0,
    include_object=True,
):
    message = DetectedObjectArray()
    message.header.frame_id = frame_id
    message.header.stamp = (
        node.get_clock().now() - Duration(seconds=age_s)
    ).to_msg()
    if include_object:
        detected = DetectedObject()
        detected.class_id = 1
        detected.class_name = "robot"
        detected.confidence = 0.9
        # Test TF is identity, so source +x/+y already means base_link forward/left.
        detected.pose.position.x = float(forward_m)
        detected.pose.position.y = 0.0
        detected.pose.orientation.w = 1.0
        detected.bbox.width = 40
        detected.bbox.height = 30
        message.objects.append(detected)
    return message


class Harness:
    def __init__(self, follower):
        self.node = rclpy.create_node("lead_follower_test_harness")
        self.detections = self.node.create_publisher(
            DetectedObjectArray, "/detected_objects", 10
        )
        self.commands = []
        self.states = []
        self.node.create_subscription(
            Twist, "/autonomy/cmd_vel", self.commands.append, 10
        )
        self.node.create_subscription(
            Float32MultiArray, "/follow/state", self.states.append, 10
        )
        self.static_tf = StaticTransformBroadcaster(self.node)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(follower)
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

    def publish_static_identity(self, follower, child_frame="camera_link"):
        transform = TransformStamped()
        transform.header.stamp = self.node.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = child_frame
        transform.transform.rotation.w = 1.0
        self.static_tf.sendTransform(transform)
        self.spin_until(
            lambda: follower.tf_buf.can_transform(
                "base_link", child_frame, Time()
            )
        )

    def close(self, follower):
        self.executor.remove_node(follower)
        self.executor.remove_node(self.node)
        follower.destroy_node()
        self.node.destroy_node()
        self.executor.shutdown()


def _node():
    return LeadFollowerNode(
        parameter_overrides=[Parameter("enabled", value=True)]
    )


def test_apply_tf_rotates_source_pose_into_base_link_axes():
    transform = TransformStamped()
    transform.transform.rotation.z = math.sin(math.pi / 4.0)
    transform.transform.rotation.w = math.cos(math.pi / 4.0)
    transform.transform.translation.x = 2.0
    transformed = _apply_tf(Point(x=1.0, y=0.0, z=0.0), transform)
    assert transformed == pytest.approx((2.0, 1.0, 0.0), abs=1e-6)


def test_static_tf_detection_publishes_base_link_follow_command():
    follower = _node()
    harness = Harness(follower)
    try:
        harness.spin_for(0.10)
        harness.publish_static_identity(follower)

        harness.detections.publish(_detection(follower, forward_m=3.0))
        harness.spin_until(lambda: bool(harness.states))
        assert harness.commands == []                 # 신규 target 초기화 frame
        assert harness.states[-1].data[5] == FOLLOW_STATE_REACQUIRING

        harness.detections.publish(_detection(follower, forward_m=3.0))
        harness.spin_until(lambda: bool(harness.commands))
        harness.spin_until(
            lambda: harness.states
            and harness.states[-1].data[5] == FOLLOW_STATE_TRACKING
        )

        assert harness.commands[-1].linear.x > 0.0
        assert len(harness.states[-1].data) == 6       # 기존 5필드 + 상태 코드
        assert harness.states[-1].data[5] == FOLLOW_STATE_TRACKING

        state_count = len(harness.states)
        harness.detections.publish(
            _detection(follower, include_object=False)
        )
        harness.spin_until(lambda: len(harness.states) > state_count)
        assert harness.states[-1].data[5] == FOLLOW_STATE_PREDICTING
    finally:
        harness.close(follower)


def test_empty_frame_id_publishes_lost_state_but_no_command():
    follower = _node()
    harness = Harness(follower)
    try:
        harness.spin_for(0.10)
        harness.detections.publish(_detection(follower, frame_id=""))
        harness.spin_until(lambda: bool(harness.states))
        assert harness.commands == []
        assert harness.states[-1].data[5] == FOLLOW_STATE_LOST
        assert "frame_id" in follower._last.reason
    finally:
        harness.close(follower)


def test_missing_tf_publishes_lost_state_but_no_command():
    follower = _node()
    harness = Harness(follower)
    try:
        harness.spin_for(0.10)
        harness.detections.publish(
            _detection(follower, frame_id="missing_camera_link")
        )
        harness.spin_until(lambda: bool(harness.states))
        assert harness.commands == []
        assert harness.states[-1].data[5] == FOLLOW_STATE_LOST
        assert "TF" in follower._last.reason
    finally:
        harness.close(follower)


def test_stale_detection_stamp_is_rejected_even_with_static_tf():
    follower = _node()
    harness = Harness(follower)
    try:
        harness.spin_for(0.10)
        harness.publish_static_identity(follower)
        harness.detections.publish(_detection(follower, age_s=0.6))
        harness.spin_until(lambda: bool(harness.states))
        assert harness.commands == []
        assert harness.states[-1].data[5] == FOLLOW_STATE_LOST
        assert "stale" in follower._last.reason
    finally:
        harness.close(follower)


def test_ok_to_not_ok_edge_publishes_one_zero_and_rearms_after_resume():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    follower = _node()
    follower.pub_cmd = Publisher()
    follower.pub_state = Publisher()
    follower.pub_active = Publisher()
    tracking = FollowResult(True, 0.3, 0.1, state="TRACKING")
    lost = FollowResult(False, state="LOST")
    try:
        follower._publish_result(tracking)
        follower._publish_result(lost)
        follower._publish_result(lost)
        follower._publish_result(tracking)
        follower._publish_result(lost)

        commands = follower.pub_cmd.messages
        assert [(message.linear.x, message.angular.z) for message in commands] == [
            (0.3, 0.1),
            (0.0, 0.0),
            (0.3, 0.1),
            (0.0, 0.0),
        ]
    finally:
        follower.destroy_node()


def test_frame_gate_transition_publishes_one_zero_until_command_resumes():
    class Publisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    follower = _node()
    follower.pub_cmd = Publisher()
    follower.pub_state = Publisher()
    follower.pub_active = Publisher()
    tracking = FollowResult(True, 0.3, 0.1, state="TRACKING")
    try:
        follower._publish_result(tracking)
        follower._publish_result(tracking, allow_command=False)
        follower._publish_result(tracking, allow_command=False)
        follower._publish_result(tracking)
        follower._publish_result(tracking, allow_command=False)

        commands = follower.pub_cmd.messages
        assert [(message.linear.x, message.angular.z) for message in commands] == [
            (0.3, 0.1),
            (0.0, 0.0),
            (0.3, 0.1),
            (0.0, 0.0),
        ]
    finally:
        follower.destroy_node()
