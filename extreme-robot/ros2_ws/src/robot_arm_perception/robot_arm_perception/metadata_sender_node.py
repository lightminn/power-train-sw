"""Best-effort D435 detection metadata sender for the operator console.

The node never opens a camera or controls the arm.  It subscribes to the
robot-arm owner's existing ``/detected_objects`` result and sends a bounded,
versioned JSON datagram to the operator laptop.  Raw RGB stays on SRT :5002.
"""
from __future__ import annotations

import json
import math
import socket

import rclpy
from rclpy.node import Node
from robot_arm_msgs.msg import DetectedObjectArray


MAX_DATAGRAM_BYTES = 2048


class MetadataSender(Node):
    def __init__(self) -> None:
        super().__init__("d435_metadata_sender")
        self.declare_parameter("operator_host", "")
        self.declare_parameter("operator_port", 5003)
        self.declare_parameter("frame_width", 848)
        self.declare_parameter("frame_height", 480)
        self._socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.create_subscription(DetectedObjectArray, "/detected_objects", self._on_detections, 1)

    def _on_detections(self, message: DetectedObjectArray) -> None:
        host = str(self.get_parameter("operator_host").value)
        port = int(self.get_parameter("operator_port").value)
        if not host or not 1 <= port <= 65535:
            self.get_logger().error("operator_host and operator_port must be configured")
            return
        stamp = message.header.stamp
        detections = []
        for item in message.objects:
            box = item.bbox
            values = (float(item.confidence), float(item.pose.position.x),
                      float(item.pose.position.y), float(item.pose.position.z))
            if not all(math.isfinite(value) for value in values):
                continue
            # z=0 is the existing perception contract for unavailable depth.
            position = None if item.pose.position.z <= 0.0 else [
                item.pose.position.x, item.pose.position.y, item.pose.position.z
            ]
            detections.append({
                "class_id": int(item.class_id),
                "class_name": str(item.class_name),
                "confidence": float(item.confidence),
                "bbox_xywh": [int(box.x_offset), int(box.y_offset), int(box.width), int(box.height)],
                "position_m": position,
            })
        payload = {
            "schema_version": 1,
            # ROS2 Image has no sequence field; the source stamp is the current correlation key.
            "capture_sequence": int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
            "capture_stamp_ns": int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
            "frame_width": int(self.get_parameter("frame_width").value),
            "frame_height": int(self.get_parameter("frame_height").value),
            "frame_id": str(message.header.frame_id),
            "detections": detections,
        }
        encoded = json.dumps(payload, allow_nan=False, separators=(",", ":")).encode("utf-8")
        if len(encoded) > MAX_DATAGRAM_BYTES:
            self.get_logger().warning("metadata datagram oversized; dropping frame")
            return
        try:
            self._socket.sendto(encoded, (host, port))
        except OSError as error:
            self.get_logger().warning(f"metadata UDP send failed: {error}")

    def destroy_node(self) -> bool:
        self._socket.close()
        return super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = MetadataSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
