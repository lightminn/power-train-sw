"""Thin ROS publisher shell around the threaded L515 source."""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, Imu

from .l515_adapter import (
    camera_info_from_intrinsics,
    image_from_array,
    imu_from_vector,
)
from .l515_source import L515Config, L515Source


COLOR_FRAME = "l515_color_optical_frame"
DEPTH_FRAME = "l515_depth_optical_frame"
GYRO_FRAME = "l515_gyro_frame"
ACCEL_FRAME = "l515_accel_frame"


class L515Node(Node):
    """Drain latest SDK samples without blocking the ROS executor."""

    def __init__(self, source=None):
        super().__init__("l515_camera_node")
        defaults = L515Config()
        self.declare_parameter("serial", defaults.serial)
        self.declare_parameter("width", defaults.width)
        self.declare_parameter("height", defaults.height)
        self.declare_parameter("fps", defaults.fps)
        self.declare_parameter(
            "reconnect_interval", defaults.reconnect_interval
        )

        if source is None:
            import pyrealsense2 as rs

            config = L515Config(
                serial=self.get_parameter("serial").value,
                width=self.get_parameter("width").value,
                height=self.get_parameter("height").value,
                fps=self.get_parameter("fps").value,
                reconnect_interval=self.get_parameter(
                    "reconnect_interval"
                ).value,
            )
            source = L515Source(rs, config)
        self.source = source
        self._source_stopped = False

        specs = (
            ("/l515/color/image_raw", Image),
            ("/l515/color/camera_info", CameraInfo),
            ("/l515/depth/image_rect_raw", Image),
            ("/l515/depth/camera_info", CameraInfo),
            ("/l515/gyro/sample", Imu),
            ("/l515/accel/sample", Imu),
        )
        self.stream_publishers = {
            topic: self.create_publisher(
                message_type, topic, qos_profile_sensor_data
            )
            for topic, message_type in specs
        }
        try:
            self.timer = self.create_timer(1.0 / 200.0, self._drain_source)
            self.source.start()
        except BaseException:
            try:
                self._stop_source()
            finally:
                super().destroy_node()
            raise

    def _stamp(self, frame, mapper):
        mapped_ns = mapper.map_ms(
            frame.get_timestamp(), self.get_clock().now().nanoseconds
        )
        return Time(nanoseconds=mapped_ns).to_msg()

    @staticmethod
    def _intrinsics(frame):
        return frame.profile.as_video_stream_profile().intrinsics

    def _publish_video(self, frame, mapper, *, topic, info_topic,
                       encoding, frame_id):
        stamp = self._stamp(frame, mapper)
        image = image_from_array(
            frame.get_data(), encoding, frame_id, stamp
        )
        info = camera_info_from_intrinsics(
            self._intrinsics(frame), frame_id, stamp
        )
        self.stream_publishers[topic].publish(image)
        self.stream_publishers[info_topic].publish(info)

    def _publish_motion(self, frame, mapper, *, topic, kind, frame_id):
        stamp = self._stamp(frame, mapper)
        vector = frame.as_motion_frame().get_motion_data()
        message = imu_from_vector(vector, kind, frame_id, stamp)
        self.stream_publishers[topic].publish(message)

    def _drain_source(self):
        frames = self.source.poll_latest()
        mapper = frames.timestamp_mapper
        if mapper is None:
            return
        if frames.color is not None:
            self._publish_video(
                frames.color, mapper,
                topic="/l515/color/image_raw",
                info_topic="/l515/color/camera_info",
                encoding="bgr8", frame_id=COLOR_FRAME,
            )
        if frames.depth is not None:
            self._publish_video(
                frames.depth, mapper,
                topic="/l515/depth/image_rect_raw",
                info_topic="/l515/depth/camera_info",
                encoding="16UC1", frame_id=DEPTH_FRAME,
            )
        if frames.gyro is not None:
            self._publish_motion(
                frames.gyro, mapper, topic="/l515/gyro/sample",
                kind="gyro", frame_id=GYRO_FRAME,
            )
        if frames.accel is not None:
            self._publish_motion(
                frames.accel, mapper, topic="/l515/accel/sample",
                kind="accel", frame_id=ACCEL_FRAME,
            )

    def _stop_source(self):
        if not self._source_stopped:
            self._source_stopped = True
            self.source.stop()

    def destroy_node(self):
        self._stop_source()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = L515Node()
        rclpy.spin(node)
    finally:
        try:
            if node is not None:
                node.destroy_node()
        finally:
            rclpy.shutdown()
