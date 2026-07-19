"""팔 스택 무수정 미러: arm ROS topics to bounded console UDP datagrams.

All four subscriptions are read-only; this node publishes no ROS topic, calls
no service, and never accesses a motor device.  :5003은 단일 송신 원칙 — 이
노드 가동 시 팔 metadata_sender_node는 미기동(배포 조율).
"""
from __future__ import annotations

import socket
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from sensor_msgs.msg import JointState
from std_msgs.msg import Int32MultiArray

from robot_arm_msgs.msg import DetectedObject, DetectedObjectArray
from powertrain_ros.arm_console_mirror import (
    build_arm_telemetry_payload,
    build_detection_metadata_payload,
    parse_dynamixel_state,
    yaw_from_quaternion,
)


class ArmConsoleBridge(Node):
    def __init__(self, *, parameter_overrides=None) -> None:
        super().__init__(
            "arm_console_bridge",
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("console_host", "")
        self.declare_parameter("telemetry_port", 5007)
        self.declare_parameter("metadata_port", 5003)
        self.declare_parameter("publish_hz", 5.0)
        self.declare_parameter("frame_width", 848)
        self.declare_parameter("frame_height", 480)
        self.declare_parameter("send_detection_metadata", True)

        host = str(self.get_parameter("console_host").value)
        telemetry_port = int(self.get_parameter("telemetry_port").value)
        metadata_port = int(self.get_parameter("metadata_port").value)
        publish_hz = float(self.get_parameter("publish_hz").value)
        self._frame_width = int(self.get_parameter("frame_width").value)
        self._frame_height = int(self.get_parameter("frame_height").value)
        self._send_detection_metadata = bool(
            self.get_parameter("send_detection_metadata").value
        )
        if (
            not host
            or not 1 <= telemetry_port <= 65535
            or not 1 <= metadata_port <= 65535
            or not 0.2 <= publish_hz <= 10.0
            or self._frame_width < 1
            or self._frame_height < 1
        ):
            raise ValueError("arm console bridge parameters are invalid")

        self._telemetry_endpoint = (host, telemetry_port)
        self._metadata_endpoint = (host, metadata_port)
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sequence = 0
        # header.stamp 를 capture_sequence 로 겸용하면 stamp=0 스택에서
        # 콘솔 게이트가 첫 프레임 이후 전부 기각한다 — 자체 단조 카운터.
        self._metadata_sequence = 0
        self._dynamixel: Int32MultiArray | None = None
        self._dynamixel_at: float | None = None
        self._joints: JointState | None = None
        self._joints_at: float | None = None
        self._detections: DetectedObjectArray | None = None
        self._detections_at: float | None = None
        self._pick_target: DetectedObject | None = None
        self._pick_target_at: float | None = None

        self.create_subscription(
            Int32MultiArray,
            "/dynamixel/state",
            self._on_dynamixel,
            10,
        )
        self.create_subscription(
            JointState,
            "/joint_states",
            self._on_joints,
            10,
        )
        self.create_subscription(
            DetectedObjectArray,
            "/detected_objects",
            self._on_detections,
            1,
        )
        pick_qos = QoSProfile(
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            DetectedObject,
            "/pick_target",
            self._on_pick_target,
            pick_qos,
        )
        self.create_timer(1.0 / publish_hz, self._send_telemetry)

    def _on_dynamixel(self, message: Int32MultiArray) -> None:
        self._dynamixel = message
        self._dynamixel_at = time.monotonic()

    def _on_joints(self, message: JointState) -> None:
        self._joints = message
        self._joints_at = time.monotonic()

    def _on_pick_target(self, message: DetectedObject) -> None:
        self._pick_target = message
        self._pick_target_at = time.monotonic()

    @staticmethod
    def _fresh(at: float | None, now: float) -> bool:
        return at is not None and now - at <= 1.0

    @staticmethod
    def _age(at: float | None, now: float) -> float | None:
        return None if at is None else now - at

    def _send_telemetry(self) -> None:
        now = time.monotonic()
        motors = None
        if self._dynamixel is not None and self._fresh(
            self._dynamixel_at, now
        ):
            motors = parse_dynamixel_state(self._dynamixel.data)
            if motors is None:
                self.get_logger().warning(
                    "invalid /dynamixel/state snapshot; source omitted",
                    throttle_duration_sec=1.0,
                )

        joints = None
        if self._joints is not None and self._fresh(self._joints_at, now):
            joints = {
                "names": self._joints.name,
                "position_rad": self._joints.position,
                "velocity": self._joints.velocity,
            }
        source_age_s = {
            "dynamixel": self._age(self._dynamixel_at, now),
            "joints": self._age(self._joints_at, now),
            "detections": self._age(self._detections_at, now),
        }
        try:
            payload = build_arm_telemetry_payload(
                sequence=self._sequence,
                stamp_s=time.time(),
                motors=motors,
                joints=joints,
                source_age_s=source_age_s,
            )
            self._udp.sendto(payload, self._telemetry_endpoint)
            self._sequence += 1
        except (ValueError, OSError) as exc:
            self.get_logger().warning(
                f"arm telemetry send failed: {exc}",
                throttle_duration_sec=5.0,
            )

    @staticmethod
    def _bbox_xywh(detected: DetectedObject) -> tuple[int, int, int, int]:
        return (
            int(detected.bbox.x_offset),
            int(detected.bbox.y_offset),
            int(detected.bbox.width),
            int(detected.bbox.height),
        )

    def _pick_comparison(self):
        if self._pick_target is None:
            return None
        return (
            int(self._pick_target.class_id),
            self._bbox_xywh(self._pick_target),
        )

    def _on_detections(self, message: DetectedObjectArray) -> None:
        self._detections = message
        self._detections_at = time.monotonic()
        if not self._send_detection_metadata:
            return

        detections = []
        for detected in message.objects:
            pose = detected.pose
            detections.append(
                (
                    int(detected.class_id),
                    str(detected.class_name),
                    float(detected.confidence),
                    self._bbox_xywh(detected),
                    (
                        float(pose.position.x),
                        float(pose.position.y),
                        float(pose.position.z),
                    ),
                    yaw_from_quaternion(
                        float(pose.orientation.z),
                        float(pose.orientation.w),
                    ),
                )
            )
        capture_stamp_ns = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        try:
            payload = build_detection_metadata_payload(
                capture_sequence=self._metadata_sequence,
                capture_stamp_ns=capture_stamp_ns,
                frame_id=message.header.frame_id,
                frame_width=self._frame_width,
                frame_height=self._frame_height,
                detections=detections,
                pick_target=self._pick_comparison(),
            )
            self._udp.sendto(payload, self._metadata_endpoint)
            self._metadata_sequence += 1
        except (ValueError, OSError) as exc:
            self.get_logger().warning(
                f"arm detection metadata send failed: {exc}",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self):
        self._udp.close()
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = ArmConsoleBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
