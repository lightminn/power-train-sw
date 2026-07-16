"""Send read-only chassis and observability snapshots to the operator console.

This node never opens CAN or a motor device.  CAN status comes from the
observability daemon's cached chassis-owner measurements; ROS state is mirrored
to a distinct best-effort UDP channel.  Power telemetry remains on UDP :5004;
chassis telemetry uses :5005 so one source cannot erase another source's
unavailable fields.
"""
from __future__ import annotations

import json
import math
import socket
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node

from l515_dashboard.client import GatewayClient
from powertrain_msgs.msg import SafetyVerdict, WheelStates
from powertrain_observability.client import ObservabilityClient
from powertrain_ros import console_can_status


SAFETY_STATUS = {
    SafetyVerdict.CHECKING: "CHECKING",
    SafetyVerdict.VALID: "VALID",
    SafetyVerdict.INVALID_READING: "INVALID_READING",
    SafetyVerdict.NO_RESPONSE: "NO_RESPONSE",
}


class ChassisTelemetrySender(Node):
    def __init__(self) -> None:
        super().__init__("chassis_telemetry_sender")
        self.declare_parameter("operator_host", "")
        self.declare_parameter("operator_port", 5005)
        self.declare_parameter("publish_hz", 2.0)
        host = str(self.get_parameter("operator_host").value)
        port = int(self.get_parameter("operator_port").value)
        hz = float(self.get_parameter("publish_hz").value)
        if not host or not 1 <= port <= 65535 or not 0.2 <= hz <= 10.0:
            raise ValueError("operator_host, operator_port, and publish_hz are invalid")
        self._endpoint = (host, port)
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._sequence = 0
        self._wheels: WheelStates | None = None
        self._wheels_at: float | None = None
        self._odom: Odometry | None = None
        self._odom_at: float | None = None
        self._safety: SafetyVerdict | None = None
        self._safety_at: float | None = None
        self._gateway = GatewayClient("@powertrain-l515-gateway", request_timeout_s=0.5)
        self._l515: dict[str, object] = {}
        self._observability = ObservabilityClient(request_timeout_s=0.5)
        self._can_health_record: object | None = None
        self._observability_error: str | None = None
        self.create_subscription(WheelStates, "/wheel_states", self._on_wheels, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(SafetyVerdict, "/safety_verdict", self._on_safety, 10)
        self.create_timer(1.0, self._poll_l515)
        self.create_timer(1.0, self._poll_observability)
        self.create_timer(1.0 / hz, self._send)

    def _on_wheels(self, message: WheelStates) -> None:
        self._wheels = message
        self._wheels_at = time.monotonic()

    def _on_odom(self, message: Odometry) -> None:
        self._odom = message
        self._odom_at = time.monotonic()

    def _on_safety(self, message: SafetyVerdict) -> None:
        self._safety = message
        self._safety_at = time.monotonic()

    def _poll_l515(self) -> None:
        snapshot = self._gateway.poll()
        if snapshot is None:
            self._l515 = {"state": "UNAVAILABLE", "detail": self._gateway.last_error or "no response"}
            return
        payload = snapshot.payload
        sdk = payload.get("sdk", {})
        srt = payload.get("srt", {})
        system = payload.get("system", {})
        rates = sdk.get("native_callback_rates_hz", {})
        self._l515 = {
            "state": str(payload.get("state", "unknown")),
            "detail": str(payload.get("last_error") or srt.get("last_error") or ""),
            "mode": str(srt.get("mode") or "-"),
            "color_hz": rates.get("color"), "depth_hz": rates.get("depth"),
            "submitted_hz": srt.get("submitted_rate_hz"),
            "sent_hz": srt.get("sent_rate_hz"), "drop_hz": srt.get("drop_rate_hz"),
            # Preserve the complete Gateway status contract.  This observer
            # only forwards scalar status over UDP; it never opens the camera.
            "ros_topic_rates_hz": payload.get("ros_topic_rates_hz", {}),
            "aligned_depth_age_ms": srt.get("aligned_depth_age_ms"),
            "process_cpu_percent": system.get("cpu_percent"),
            "process_rss_bytes": system.get("current_rss_bytes"),
        }

    def _poll_observability(self) -> None:
        snapshot = self._observability.poll()
        if snapshot is None:
            self._observability_error = (
                self._observability.last_error or "no response"
            )
            return
        self._observability_error = None
        try:
            recent_events = snapshot.payload.get("recent_events", {})
            self._can_health_record = recent_events.get("CAN_HEALTH")
        except (AttributeError, TypeError):
            self._can_health_record = {}

    def _can_status(self) -> str:
        if self._observability_error is not None:
            return f"UNAVAILABLE · observability {self._observability_error}"
        return console_can_status.can_status_text(
            self._can_health_record,
            time.monotonic_ns(),
        )

    @staticmethod
    def _fresh(at: float | None, now: float) -> bool:
        return at is not None and now - at <= 1.0

    def _send(self) -> None:
        now = time.monotonic()
        wheels = self._wheels if self._fresh(self._wheels_at, now) else None
        odom = self._odom if self._fresh(self._odom_at, now) else None
        safety = self._safety if self._fresh(self._safety_at, now) else None
        if odom is None:
            x_m = y_m = yaw_rad = None
            odometry_source = "unavailable"
        else:
            pose = odom.pose.pose
            x_m, y_m = pose.position.x, pose.position.y
            q = pose.orientation
            yaw_rad = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            odometry_source = "wheel+imu"
        drive_state = "unavailable" if wheels is None else f"{wheels.chassis_mode}/{wheels.stop_state}"
        wheel_feedback = tuple() if wheels is None else tuple(wheels.wheels)
        wheel_statuses = [
            {
                "name": wheel.name,
                "mode": wheel.corner_mode,
                "drive_turns_per_s": wheel.drive_turns_per_s,
                "steer_deg": wheel.steer_deg,
                "stale": wheel.drive_stale or wheel.steer_stale,
                "drive_axis_error": wheel.drive_axis_error,
                "steer_fault": wheel.steer_fault,
            }
            for wheel in wheel_feedback
        ]
        payload = {
            "schema_version": 1,
            "sequence": self._sequence,
            "odometry_source": odometry_source,
            "x_m": x_m, "y_m": y_m, "yaw_rad": yaw_rad,
            "voltage_v": None, "current_a": None, "power_w": None,
            "drive_state": drive_state,
            # CAN health is measured by the flock-owning chassis process and
            # relayed here through the observability daemon.
            "can_state": self._can_status(),
            "l515_state": self._l515.get("state", "unavailable"),
            "l515_detail": self._l515.get("detail", ""),
            "l515_mode": self._l515.get("mode", "-"),
            "l515_color_hz": self._l515.get("color_hz"),
            "l515_depth_hz": self._l515.get("depth_hz"),
            "l515_submitted_hz": self._l515.get("submitted_hz"),
            "l515_sent_hz": self._l515.get("sent_hz"),
            "l515_drop_hz": self._l515.get("drop_hz"),
            "l515_ros_topic_rates_hz": self._l515.get("ros_topic_rates_hz", {}),
            "l515_aligned_depth_age_ms": self._l515.get("aligned_depth_age_ms"),
            "l515_process_cpu_percent": self._l515.get("process_cpu_percent"),
            "l515_process_rss_bytes": self._l515.get("process_rss_bytes"),
            "safety_status": "unavailable" if safety is None else SAFETY_STATUS.get(
                safety.status, f"UNKNOWN({safety.status})"),
            "safety_distance_mm": None if safety is None or not math.isfinite(
                safety.distance_mm) else safety.distance_mm,
            "safety_estop_required": None if safety is None else safety.estop_required,
            "safety_consecutive_failures": None if safety is None else safety.consecutive_failures,
            "safety_detail": "" if safety is None else safety.detail,
            "wheel_count": None if wheels is None else len(wheel_feedback),
            "wheel_fault_count": None if wheels is None else sum(
                wheel.corner_mode == "FAULT" for wheel in wheel_feedback),
            "wheel_stale_count": None if wheels is None else sum(
                wheel.drive_stale or wheel.steer_stale for wheel in wheel_feedback),
            "wheel_axis_error_count": None if wheels is None else sum(
                wheel.drive_axis_error != 0 for wheel in wheel_feedback),
            "wheel_steer_fault_count": None if wheels is None else sum(
                wheel.steer_fault != 0 for wheel in wheel_feedback),
            "wheel_statuses": wheel_statuses,
        }
        try:
            self._udp.sendto(json.dumps(payload, separators=(",", ":")).encode(), self._endpoint)
            self._sequence += 1
        except OSError as exc:
            self.get_logger().warning(
                f"operator telemetry send failed: {exc}", throttle_duration_sec=5.0
            )

    def destroy_node(self):
        self._udp.close()
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = ChassisTelemetrySender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
