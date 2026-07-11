"""Nonblocking ROS publication adapter for one drained Gateway frameset."""

import threading
import time

TOPIC_SPECS = (
    "/l515/color/image_raw",
    "/l515/color/camera_info",
    "/l515/depth/image_rect_raw",
    "/l515/depth/camera_info",
    "/l515/gyro/sample",
    "/l515/accel/sample",
)

_COLOR_FRAME = "l515_color_optical_frame"
_DEPTH_FRAME = "l515_depth_optical_frame"
_GYRO_FRAME = "l515_gyro_frame"
_ACCEL_FRAME = "l515_accel_frame"


def _ros_dependencies():
    from builtin_interfaces.msg import Time
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CameraInfo, Image, Imu
    from powertrain_ros.l515_adapter import (
        camera_info_from_intrinsics,
        image_from_array,
        imu_from_vector,
    )

    kinds = (Image, CameraInfo, Image, CameraInfo, Imu, Imu)
    return (
        Time,
        qos_profile_sensor_data,
        kinds,
        image_from_array,
        camera_info_from_intrinsics,
        imu_from_vector,
    )


def _time_message(nanoseconds, message_type):
    message = message_type()
    message.sec, message.nanosec = divmod(int(nanoseconds), 1_000_000_000)
    return message


class GatewayRosPublisher:
    """Publish only the approved six topics from already-drained frames."""

    def __init__(self, node, *, now_ns=time.time_ns, dependencies=None):
        (
            self._time_type,
            qos,
            kinds,
            self._image_from_array,
            self._camera_info_from_intrinsics,
            self._imu_from_vector,
        ) = (
            dependencies or _ros_dependencies()
        )
        self._now_ns = now_ns
        self._publishers = {
            topic: node.create_publisher(kind, topic, qos)
            for topic, kind in zip(TOPIC_SPECS, kinds)
        }
        self._last_timestamps = {}
        self._mapper = None
        self._counts = {topic: 0 for topic in TOPIC_SPECS}
        self._state_lock = threading.Lock()

    @staticmethod
    def _intrinsics(frame):
        return frame.profile.as_video_stream_profile().intrinsics

    def _stamp(self, frame, mapper, stream_key):
        device_ms = float(frame.get_timestamp())
        with self._state_lock:
            if mapper is not self._mapper:
                self._mapper = mapper
                self._last_timestamps.clear()
            if self._last_timestamps.get(stream_key) == device_ms:
                return None
            self._last_timestamps[stream_key] = device_ms
        return _time_message(
            mapper.map_ms(device_ms, self._now_ns(), stream_key=stream_key),
            self._time_type,
        )

    def _video(self, frame, mapper, topic, info_topic, encoding, frame_id):
        stamp = self._stamp(frame, mapper, topic)
        if stamp is None:
            return ()
        image = self._image_from_array(
            frame.get_data(), encoding, frame_id, stamp
        )
        info = self._camera_info_from_intrinsics(
            self._intrinsics(frame), frame_id, stamp
        )
        self._publishers[topic].publish(image)
        self._publishers[info_topic].publish(info)
        self._counts[topic] += 1
        self._counts[info_topic] += 1
        return (topic, info_topic)

    def _motion(self, frame, mapper, topic, kind, frame_id):
        stamp = self._stamp(frame, mapper, topic)
        if stamp is None:
            return ()
        vector = frame.as_motion_frame().get_motion_data()
        self._publishers[topic].publish(
            self._imu_from_vector(vector, kind, frame_id, stamp)
        )
        self._counts[topic] += 1
        return (topic,)

    def publish_color(self, sample, mapper):
        return self._video(sample.frame, mapper, "/l515/color/image_raw",
                           "/l515/color/camera_info", "bgr8", _COLOR_FRAME)

    def publish_depth(self, sample, mapper):
        return self._video(sample.frame, mapper, "/l515/depth/image_rect_raw",
                           "/l515/depth/camera_info", "16UC1", _DEPTH_FRAME)

    def publish_imu(self, stream, sample, mapper):
        if stream == "gyro":
            return self._motion(sample.frame, mapper, "/l515/gyro/sample",
                                "gyro", _GYRO_FRAME)
        if stream == "accel":
            return self._motion(sample.frame, mapper, "/l515/accel/sample",
                                "accel", _ACCEL_FRAME)
        raise ValueError(f"unsupported IMU stream: {stream}")

    def publish(self, frames):
        mapper = frames.mapper
        if mapper is None:
            return ()
        published = []
        if frames.raw_color is not None:
            published.extend(self._video(
                frames.raw_color,
                mapper,
                "/l515/color/image_raw",
                "/l515/color/camera_info",
                "bgr8",
                _COLOR_FRAME,
            ))
        if frames.raw_depth is not None:
            published.extend(self._video(
                frames.raw_depth,
                mapper,
                "/l515/depth/image_rect_raw",
                "/l515/depth/camera_info",
                "16UC1",
                _DEPTH_FRAME,
            ))
        if frames.gyro is not None:
            published.extend(self._motion(
                frames.gyro, mapper, "/l515/gyro/sample", "gyro", _GYRO_FRAME
            ))
        if frames.accel is not None:
            published.extend(self._motion(
                frames.accel,
                mapper,
                "/l515/accel/sample",
                "accel",
                _ACCEL_FRAME,
            ))
        return tuple(published)

    def publish_counts(self):
        return dict(self._counts)
