"""L515 terrain obstacle outlines → operator console UDP :5009.

Operator visual aid only (2026-10-17 decision): this node never publishes to
ROS and gives nothing to control, stop, speed limits, SafetyInterlock or
US-100.  It subscribes to the Gateway's raw depth and both CameraInfo topics
and sends one packet per tick; all packet decisions live in the ROS-free
``terrain_overlay.runtime`` core.  Spec:
docs/specs/2026-10-17-l515-terrain-overlay-design.md
"""
from __future__ import annotations

import os
import socket
import time

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from powertrain_runtime.telemetry import send_datagram
from terrain_overlay.camera import CameraModel
from terrain_overlay.runtime import RuntimeConfig, SenderSession, TerrainOverlayRuntime

DEPTH_TOPIC = "/l515/depth/image_rect_raw"
DEPTH_INFO_TOPIC = "/l515/depth/camera_info"
COLOR_INFO_TOPIC = "/l515/color/camera_info"
# Gateway-compatible (best effort, volatile) but latest-only: with the default
# depth 5 a slow tick drains queued frames late and stamps old depth as freshly
# received, delaying DEPTH_STALE (2026-10-17 slow-processing test: 1.2 s).
LATEST_DEPTH_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST, depth=1,
    reliability=qos_profile_sensor_data.reliability,
    durability=qos_profile_sensor_data.durability,
)


def depth_array(message: Image) -> np.ndarray | None:
    """Return a private uint16 copy, or None if the message breaks 16UC1 LE."""
    if (
        message.encoding != "16UC1"
        or message.is_bigendian != 0
        or message.step != message.width * 2
        or len(message.data) != message.step * message.height
    ):
        return None
    return np.frombuffer(message.data, dtype=np.uint16).reshape(
        message.height, message.width,
    ).copy()


def stamp_ns(message) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(message.header.stamp.nanosec)


class TerrainOverlayNode(Node):
    def __init__(self, *, session: SenderSession | None = None, **node_kwargs) -> None:
        super().__init__("terrain_overlay", **node_kwargs)
        self.declare_parameter("operator_host", "")
        self.declare_parameter("operator_port", 5009)
        self.declare_parameter("publish_hz", 10.0)
        self.declare_parameter("depth_scale_m", 0.00025)
        self.declare_parameter("depth_stale_s", 0.5)
        self.declare_parameter("ground_hold_s", 0.5)
        self.declare_parameter("color_offset_m", [0.0, -0.024, 0.0])
        self.declare_parameter("path_half_width_m", 0.4445)
        self.declare_parameter("front_offset_m", 0.0)
        self.declare_parameter("alignment_verified", False)
        value = lambda name: self.get_parameter(name).value  # noqa: E731
        host = str(value("operator_host"))
        port = int(value("operator_port"))
        if not host and not os.environ.get("POWERTRAIN_OPERATOR_SESSION_FILE"):
            raise ValueError("operator_host is required without POWERTRAIN_OPERATOR_SESSION_FILE")
        if not 1 <= port <= 65535:
            raise ValueError("operator_port must be 1-65535")
        config = RuntimeConfig(
            publish_hz=float(value("publish_hz")),
            depth_scale_m=float(value("depth_scale_m")),
            depth_stale_s=float(value("depth_stale_s")),
            ground_hold_s=float(value("ground_hold_s")),
            color_offset_m=tuple(float(v) for v in value("color_offset_m")),
            path_half_width_m=float(value("path_half_width_m")),
            front_offset_m=float(value("front_offset_m")),
            alignment_verified=bool(value("alignment_verified")),
        )
        self._endpoint = (host, port)
        self._runtime = TerrainOverlayRuntime(
            config, session or SenderSession.create(),
            log_error=self._log_error, log_overrun=self._log_overrun,
        )
        self._udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.send_errors = 0
        self._logged_info: set[str] = set()
        # Depth arrives on its own group so its receive time is stamped while a
        # tick is still processing (needs a multi-threaded executor, see main).
        # Camera info shares the tick's group so detectors never change mid-tick.
        depth_group = MutuallyExclusiveCallbackGroup()
        tick_group = MutuallyExclusiveCallbackGroup()
        self.create_subscription(
            Image, DEPTH_TOPIC, self._on_depth, LATEST_DEPTH_QOS, callback_group=depth_group,
        )
        self.create_subscription(
            CameraInfo, DEPTH_INFO_TOPIC,
            lambda message: self._on_info("depth", message), qos_profile_sensor_data,
            callback_group=tick_group,
        )
        self.create_subscription(
            CameraInfo, COLOR_INFO_TOPIC,
            lambda message: self._on_info("color", message), qos_profile_sensor_data,
            callback_group=tick_group,
        )
        self.create_timer(1.0 / config.publish_hz, self._tick, callback_group=tick_group)
        session_file = os.environ.get("POWERTRAIN_OPERATOR_SESSION_FILE")
        target = (f"operator session file {session_file} (fallback {host or '-'})"
                  if session_file else host)
        session_ = self._runtime.session
        self.get_logger().info(
            f"terrain overlay → {target}:{port} · session "
            f"{session_.boot_id[:8]}/{session_.start_mono_ns}/{session_.instance}",
        )

    def _log_error(self, message: str) -> None:
        self.get_logger().error(message, throttle_duration_sec=5.0)

    def _log_overrun(self, message: str) -> None:
        # Own call site → own throttle state (rclpy throttles per call site).
        self.get_logger().warning(message, throttle_duration_sec=5.0)

    def processing_summary(self) -> str:
        runtime = self._runtime
        return (f"terrain processing summary: max {runtime.max_processing_s * 1000.0:.2f} ms, "
                f"overruns {runtime.processing_overruns}")

    def _on_depth(self, message: Image) -> None:
        received = time.monotonic()
        array = depth_array(message)
        if array is None:
            self._runtime.on_bad_depth(received)
            self.get_logger().warning(
                f"depth dropped: encoding={message.encoding} bigendian={message.is_bigendian} "
                f"step={message.step} width={message.width}",
                throttle_duration_sec=5.0,
            )
            return
        self._runtime.on_depth(array, stamp_ns(message), received)

    def _on_info(self, kind: str, message: CameraInfo) -> None:
        try:
            camera = CameraModel.from_k(message.k, message.width, message.height)
        except (ValueError, IndexError) as exc:
            self.get_logger().warning(f"{kind} camera_info ignored: {exc}", throttle_duration_sec=5.0)
            return
        if kind not in self._logged_info:
            self._logged_info.add(kind)
            self.get_logger().info(
                f"{kind} camera_info {message.width}x{message.height} "
                f"model={message.distortion_model} d={list(message.d)} (pinhole used)",
            )
        if kind == "depth":
            self._runtime.set_depth_info(camera)
        else:
            self._runtime.set_color_info(camera)

    def _tick(self) -> None:
        raw = self._runtime.tick()
        if raw is None:
            return
        try:
            send_datagram(self._udp, raw, self._endpoint)
        except OSError as exc:
            self.send_errors += 1
            self.get_logger().warning(
                f"terrain UDP send failed ({self.send_errors} total): {exc}",
                throttle_duration_sec=5.0,
            )

    def destroy_node(self) -> bool:
        # Unthrottled: the §7.3-2 load check reads this line after the run.
        self.get_logger().info(self.processing_summary())
        self._udp.close()
        return super().destroy_node()


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = TerrainOverlayNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
