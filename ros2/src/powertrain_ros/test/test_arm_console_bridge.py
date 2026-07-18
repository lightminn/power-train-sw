"""Pure arm-console payload contracts; ROS adapter tests live below."""
import ast
import importlib.util
import json
import math
from pathlib import Path

import pytest

from powertrain_ros.arm_console_mirror import (
    DynamixelMotor,
    build_arm_telemetry_payload,
    build_detection_metadata_payload,
    parse_dynamixel_state,
    yaw_from_quaternion,
)


PACKAGE = Path(__file__).resolve().parents[1]
NODE_MODULE = PACKAGE / "powertrain_ros" / "arm_console_bridge_node.py"
HAS_ROS = importlib.util.find_spec("rclpy") is not None

if HAS_ROS:
    import socket
    import time

    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter
    from rclpy.qos import DurabilityPolicy, QoSProfile
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Int32MultiArray

    from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray
    from powertrain_ros.arm_console_bridge_node import ArmConsoleBridge


requires_ros = pytest.mark.skipif(not HAS_ROS, reason="host has no rclpy")


@pytest.fixture(scope="module", autouse=True)
def ros():
    if not HAS_ROS:
        yield
        return
    rclpy.init()
    yield
    rclpy.shutdown()


def _motor():
    return DynamixelMotor(
        id=11,
        position_raw=2048,
        position_deg=0.0,
        velocity=0,
        current=-12,
        temperature_c=34,
    )


def _detection(
    *, class_id=0, class_name="box", confidence=0.9,
    bbox=(10, 20, 30, 40), position=(1.0, 2.0, 3.0), yaw=0.52,
):
    return (class_id, class_name, confidence, bbox, position, yaw)


def test_parse_dynamixel_state_builds_two_motors_and_converts_degrees():
    motors = parse_dynamixel_state(
        [11, 2048, 0, -12, 34, 12, 3072, -3, 18, 40]
    )

    assert motors is not None
    assert len(motors) == 2
    assert motors[0] == _motor()
    assert motors[1].id == 12
    assert motors[1].position_deg == pytest.approx(90.0)


@pytest.mark.parametrize(
    "data",
    [
        pytest.param([], id="empty"),
        pytest.param([11, 2048, 0, -12], id="not-five-fields"),
        pytest.param([11, 2048, 0, 0, 30] * 9, id="nine-motors"),
        pytest.param([True, 2048, 0, 0, 30], id="bool"),
        pytest.param([253, 2048, 0, 0, 30], id="id-high"),
        pytest.param([11, 4096, 0, 0, 30], id="position-high"),
        pytest.param([11, 2048, 0, 0, 151], id="temperature-high"),
        pytest.param([11, 2048, 2**31, 0, 30], id="velocity-int32"),
        pytest.param([11, 2048, 0, -(2**31) - 1, 30], id="current-int32"),
    ],
)
def test_parse_dynamixel_state_rejects_invalid_flat_arrays(data):
    assert parse_dynamixel_state(data) is None


@pytest.mark.parametrize("theta", [0.5, -2.0])
def test_yaw_from_quaternion_recovers_signed_angle(theta):
    yaw = yaw_from_quaternion(math.sin(theta / 2.0), math.cos(theta / 2.0))

    assert yaw == pytest.approx(theta, abs=1e-9)


def test_yaw_from_quaternion_normalizes_to_pi_interval():
    yaw = yaw_from_quaternion(math.sin(2.0), math.cos(2.0))

    assert yaw == pytest.approx(4.0 - 2.0 * math.pi, abs=1e-9)


def test_arm_telemetry_payload_round_trips_null_sources_and_ages():
    encoded = build_arm_telemetry_payload(
        sequence=7,
        stamp_s=123.5,
        motors=None,
        joints=None,
        source_age_s={
            "dynamixel": None,
            "joints": None,
            "detections": 0.4,
        },
    )

    payload = json.loads(encoded)
    assert payload == {
        "schema_version": 1,
        "sequence": 7,
        "stamp_s": 123.5,
        "dynamixel": None,
        "joints": None,
        "source_age_s": {
            "dynamixel": None,
            "joints": None,
            "detections": 0.4,
        },
        "truncated": False,
    }


def test_arm_telemetry_payload_rejects_mismatched_joint_arrays():
    with pytest.raises(ValueError, match="joint"):
        build_arm_telemetry_payload(
            sequence=0,
            stamp_s=0.0,
            motors=(_motor(),),
            joints={
                "names": ["joint_1", "joint_2"],
                "position_rad": [0.1],
                "velocity": [0.2, 0.3],
            },
            source_age_s={},
        )


def test_arm_telemetry_payload_drops_oversize_joints_before_motors():
    long_names = [f"joint_{index}_" + "x" * 500 for index in range(16)]
    encoded = build_arm_telemetry_payload(
        sequence=9,
        stamp_s=10.0,
        motors=(_motor(),),
        joints={
            "names": long_names,
            "position_rad": [0.0] * 16,
            "velocity": [0.0] * 16,
        },
        source_age_s={"dynamixel": 0.1, "joints": 0.1},
    )

    payload = json.loads(encoded)
    assert len(encoded) <= 4096
    assert payload["truncated"] is True
    assert payload["joints"] is None
    assert payload["dynamixel"][0]["id"] == 11


def test_detection_metadata_is_console_schema_superset_with_pick_flag():
    encoded = build_detection_metadata_payload(
        capture_stamp_ns=123456789,
        frame_id="camera_link",
        frame_width=848,
        frame_height=480,
        detections=[_detection()],
        pick_target=(0, (10, 20, 30, 40)),
    )

    payload = json.loads(encoded)
    assert set(payload) == {
        "schema_version",
        "capture_sequence",
        "capture_stamp_ns",
        "frame_width",
        "frame_height",
        "frame_id",
        "detections",
    }
    assert payload["schema_version"] == 1
    assert payload["capture_sequence"] == 123456789
    assert payload["capture_stamp_ns"] == 123456789
    detection = payload["detections"][0]
    assert detection["yaw_rad"] == pytest.approx(0.52)
    assert detection["is_pick_target"] is True


def test_detection_metadata_requires_exact_pick_target_bbox_match():
    encoded = build_detection_metadata_payload(
        capture_stamp_ns=1,
        frame_id="camera_link",
        frame_width=848,
        frame_height=480,
        detections=[_detection()],
        pick_target=(0, (10, 20, 31, 40)),
    )

    assert json.loads(encoded)["detections"][0]["is_pick_target"] is False


def test_detection_metadata_maps_nonpositive_depth_to_null_position():
    encoded = build_detection_metadata_payload(
        capture_stamp_ns=1,
        frame_id="camera_link",
        frame_width=848,
        frame_height=480,
        detections=[_detection(position=(1.0, 2.0, 0.0))],
        pick_target=None,
    )

    assert json.loads(encoded)["detections"][0]["position_m"] is None


@pytest.mark.parametrize(
    "detection",
    [
        pytest.param(_detection(confidence=math.nan), id="nan-confidence"),
        pytest.param(
            _detection(position=(1.0, math.inf, 3.0)),
            id="inf-position",
        ),
        pytest.param(_detection(yaw=math.nan), id="nan-yaw"),
    ],
)
def test_detection_metadata_skips_nonfinite_detection(detection):
    encoded = build_detection_metadata_payload(
        capture_stamp_ns=1,
        frame_id="camera_link",
        frame_width=848,
        frame_height=480,
        detections=[detection],
        pick_target=None,
    )

    assert json.loads(encoded)["detections"] == []


def test_detection_metadata_drops_low_confidence_until_under_limit():
    detections = [
        _detection(
            class_id=index,
            class_name=f"class_{index}_" + "x" * 180,
            confidence=index / 11.0,
            bbox=(index, 20, 30, 40),
        )
        for index in range(12)
    ]

    encoded = build_detection_metadata_payload(
        capture_stamp_ns=1,
        frame_id="camera_link",
        frame_width=848,
        frame_height=480,
        detections=detections,
        pick_target=None,
    )

    payload = json.loads(encoded)
    kept_ids = [item["class_id"] for item in payload["detections"]]
    assert len(encoded) <= 2048
    assert 11 in kept_ids
    assert 0 not in kept_ids
    assert kept_ids == list(range(min(kept_ids), 12))


def test_node_module_entry_and_read_only_contract_are_declared():
    assert NODE_MODULE.is_file()
    source = NODE_MODULE.read_text(encoding="utf-8")
    assert "팔 스택 무수정 미러" in source
    assert ":5003은 단일 송신 원칙" in source
    assert "create_publisher(" not in source
    assert "create_service(" not in source
    assert "can.interface" not in source

    setup_tree = ast.parse((PACKAGE / "setup.py").read_text(encoding="utf-8"))
    strings = {
        node.value
        for node in ast.walk(setup_tree)
        if isinstance(node, ast.Constant) and isinstance(node.value, str)
    }
    assert (
        "arm_console_bridge = powertrain_ros.arm_console_bridge_node:main"
        in strings
    )


if HAS_ROS:
    class RosHarness:
        def __init__(self, bridge):
            self.node = rclpy.create_node("arm_console_bridge_test_harness")
            self.dynamixel = self.node.create_publisher(
                Int32MultiArray, "/dynamixel/state", 10
            )
            self.joints = self.node.create_publisher(
                JointState, "/joint_states", 10
            )
            self.detections = self.node.create_publisher(
                DetectedObjectArray, "/detected_objects", 1
            )
            pick_qos = QoSProfile(
                depth=1,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            )
            self.pick_target = self.node.create_publisher(
                DetectedObject, "/pick_target", pick_qos
            )
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(bridge)
            self.executor.add_node(self.node)

        def receive_json(self, receiver, predicate, publish, timeout=2.0):
            deadline = time.monotonic() + timeout
            next_publish = 0.0
            while time.monotonic() < deadline:
                now = time.monotonic()
                if now >= next_publish:
                    publish()
                    next_publish = now + 0.05
                self.executor.spin_once(timeout_sec=0.02)
                try:
                    payload = json.loads(receiver.recv(8192))
                except BlockingIOError:
                    continue
                if predicate(payload):
                    return payload
            raise AssertionError("matching UDP datagram was not received")

        def close(self, bridge):
            self.executor.remove_node(bridge)
            self.executor.remove_node(self.node)
            bridge.destroy_node()
            self.node.destroy_node()
            self.executor.shutdown()


def _udp_receiver():
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.setblocking(False)
    return receiver


def _bridge(telemetry_port, metadata_port):
    return ArmConsoleBridge(
        parameter_overrides=[
            Parameter("console_host", value="127.0.0.1"),
            Parameter("telemetry_port", value=telemetry_port),
            Parameter("metadata_port", value=metadata_port),
            Parameter("publish_hz", value=10.0),
        ]
    )


@requires_ros
def test_node_mirrors_dynamixel_and_joint_states_to_ephemeral_udp_port():
    telemetry = _udp_receiver()
    metadata = _udp_receiver()
    bridge = _bridge(telemetry.getsockname()[1], metadata.getsockname()[1])
    harness = RosHarness(bridge)
    dynamixel = Int32MultiArray(data=[11, 2048, 0, -12, 34])
    joints = JointState(
        name=["joint_1"],
        position=[0.25],
        velocity=[-0.5],
    )
    try:
        payload = harness.receive_json(
            telemetry,
            lambda item: (
                item["dynamixel"] is not None
                and item["joints"] is not None
            ),
            lambda: (
                harness.dynamixel.publish(dynamixel),
                harness.joints.publish(joints),
            ),
        )

        assert payload["dynamixel"][0]["id"] == 11
        assert payload["joints"]["names"] == ["joint_1"]
        assert payload["joints"]["position_rad"] == [0.25]
        assert payload["joints"]["velocity"] == [-0.5]
    finally:
        harness.close(bridge)
        telemetry.close()
        metadata.close()


@requires_ros
def test_node_mirrors_detection_yaw_and_exact_latched_pick_target():
    telemetry = _udp_receiver()
    metadata = _udp_receiver()
    bridge = _bridge(telemetry.getsockname()[1], metadata.getsockname()[1])
    harness = RosHarness(bridge)
    pick_target = DetectedObject()
    pick_target.class_id = 7
    pick_target.bbox.x_offset = 10
    pick_target.bbox.y_offset = 20
    pick_target.bbox.width = 30
    pick_target.bbox.height = 40
    message = DetectedObjectArray()
    message.header.frame_id = "camera_link"
    message.header.stamp = harness.node.get_clock().now().to_msg()
    detected = DetectedObject()
    detected.class_id = 7
    detected.class_name = "crate"
    detected.confidence = 0.9
    detected.bbox = pick_target.bbox
    detected.pose.position.x = 1.0
    detected.pose.position.y = 2.0
    detected.pose.position.z = 3.0
    detected.pose.orientation.z = math.sin(0.25)
    detected.pose.orientation.w = math.cos(0.25)
    message.objects.append(detected)
    try:
        payload = harness.receive_json(
            metadata,
            lambda item: (
                item["detections"]
                and item["detections"][0]["is_pick_target"] is True
            ),
            lambda: (
                harness.pick_target.publish(pick_target),
                harness.detections.publish(message),
            ),
        )

        assert payload["detections"][0]["yaw_rad"] == pytest.approx(0.5)
        assert payload["detections"][0]["is_pick_target"] is True
    finally:
        harness.close(bridge)
        telemetry.close()
        metadata.close()
