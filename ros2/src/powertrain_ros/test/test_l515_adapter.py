from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time

from powertrain_ros.l515_adapter import (
    TimestampMapper,
    camera_info_from_intrinsics,
    image_from_array,
    imu_from_vector,
)


def test_timestamp_mapper_uses_one_offset_for_all_streams():
    mapper = TimestampMapper()

    color_ns = mapper.map_ms(1000.0, 10_000_000_000)
    depth_ns = mapper.map_ms(1002.5, 99_000_000_000)

    assert color_ns == 10_000_000_000
    assert depth_ns == 10_002_500_000


def test_timestamp_mapper_resets_offset_when_device_time_goes_backward():
    mapper = TimestampMapper()
    mapper.map_ms(1000.0, 10_000_000_000)

    reset_ns = mapper.map_ms(5.0, 20_000_000_000)
    following_ns = mapper.map_ms(7.0, 30_000_000_000)

    assert reset_ns == 20_000_000_000
    assert following_ns == 20_002_000_000


def test_image_from_color_array_copies_shape_step_and_bytes():
    array = np.arange(24, dtype=np.uint8).reshape(2, 4, 3)
    stamp = Time(sec=12, nanosec=34)

    msg = image_from_array(array, "bgr8", "color_frame", stamp)

    assert msg.header.stamp == stamp
    assert msg.header.frame_id == "color_frame"
    assert msg.height == 2
    assert msg.width == 4
    assert msg.encoding == "bgr8"
    assert msg.is_bigendian == 0
    assert msg.step == 12
    assert bytes(msg.data) == array.tobytes()


def test_image_from_depth_array_uses_uint16_byte_step():
    array = np.array([[1, 256, 4096], [7, 8, 9]], dtype=np.uint16)

    msg = image_from_array(array, "16UC1", "depth_frame", Time())

    assert msg.height == 2
    assert msg.width == 3
    assert msg.step == 6
    assert bytes(msg.data) == array.tobytes()


def test_image_from_array_accepts_realsense_array_compatible_buffer():
    expected = np.arange(12, dtype=np.uint8).reshape(2, 2, 3)

    class ArrayCompatible:
        def __array__(self, dtype=None, copy=None):
            return np.asarray(expected, dtype=dtype)

    msg = image_from_array(
        ArrayCompatible(), "bgr8", "color_frame", Time()
    )

    assert msg.height == 2
    assert msg.width == 2
    assert msg.step == 6
    assert bytes(msg.data) == expected.tobytes()


def test_camera_info_maps_intrinsics_and_header():
    intrinsics = SimpleNamespace(
        width=640,
        height=480,
        fx=600.0,
        fy=601.0,
        ppx=319.5,
        ppy=239.5,
        model="brown_conrady",
        coeffs=[0.1, -0.2, 0.003, 0.004, 0.05],
    )
    stamp = Time(sec=3, nanosec=4)

    msg = camera_info_from_intrinsics(intrinsics, "optical", stamp)

    assert msg.header.stamp == stamp
    assert msg.header.frame_id == "optical"
    assert msg.width == 640
    assert msg.height == 480
    assert msg.distortion_model == "plumb_bob"
    assert list(msg.d) == intrinsics.coeffs
    assert list(msg.k) == [600.0, 0.0, 319.5, 0.0, 601.0, 239.5, 0.0, 0.0, 1.0]
    assert list(msg.p) == [
        600.0, 0.0, 319.5, 0.0,
        0.0, 601.0, 239.5, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]


def test_camera_info_maps_fisheye_distortion_to_equidistant():
    intrinsics = SimpleNamespace(
        width=1, height=1, fx=1.0, fy=1.0, ppx=0.0, ppy=0.0,
        model="kannala_brandt4", coeffs=[1.0, 2.0, 3.0, 4.0],
    )

    msg = camera_info_from_intrinsics(intrinsics, "frame", Time())

    assert msg.distortion_model == "equidistant"


@pytest.mark.parametrize(
    ("kind", "field"),
    [("gyro", "angular_velocity"), ("accel", "linear_acceleration")],
)
def test_imu_from_vector_sets_only_requested_raw_vector(kind, field):
    vector = SimpleNamespace(x=1.25, y=-2.5, z=3.75)
    stamp = Time(sec=8, nanosec=9)

    msg = imu_from_vector(vector, kind, "imu_frame", stamp)

    value = getattr(msg, field)
    assert (value.x, value.y, value.z) == (1.25, -2.5, 3.75)
    assert msg.header.stamp == stamp
    assert msg.header.frame_id == "imu_frame"
    assert msg.orientation_covariance[0] == -1.0


def test_imu_from_vector_rejects_unknown_kind():
    with pytest.raises(ValueError, match="gyro.*accel"):
        imu_from_vector(
            SimpleNamespace(x=0.0, y=0.0, z=0.0),
            "mag",
            "frame",
            Time(),
        )
