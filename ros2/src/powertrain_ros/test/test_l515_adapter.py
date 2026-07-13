import array
from types import SimpleNamespace

import numpy as np
import pytest
from builtin_interfaces.msg import Time
import powertrain_ros.l515_adapter as adapter

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


def test_timestamp_mapper_does_not_reset_for_interleaved_stream_clocks():
    mapper = TimestampMapper()

    color_first = mapper.map_ms(
        1000.0, 10_000_000_000, stream_key="color"
    )
    mapper.map_ms(1005.0, 10_005_000_000, stream_key="depth")
    color_next = mapper.map_ms(
        1001.0, 99_000_000_000, stream_key="color"
    )

    assert color_first == 10_000_000_000
    assert color_next == 10_001_000_000


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


def test_image_from_array_assigns_ros_data_as_byte_array(monkeypatch):
    assigned = []

    class FakeImage:
        def __init__(self):
            self.header = SimpleNamespace(stamp=None, frame_id=None)

        @property
        def data(self):
            return assigned[-1]

        @data.setter
        def data(self, value):
            assigned.append(value)

    monkeypatch.setattr(adapter, "Image", FakeImage)

    adapter.image_from_array(
        np.zeros((2, 2, 3), dtype=np.uint8),
        "bgr8",
        "color_frame",
        Time(),
    )

    assert isinstance(assigned[0], array.array)
    assert assigned[0].typecode == "B"


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


# ── 시계 드리프트 보정 (2026-07-13) ──────────────────────────────────────
#
# ⚠️ 오프셋을 처음 한 번만 앵커하면, 장치 시계와 ROS 시계의 **주파수가 미세하게 달라**
#    시간이 갈수록 선형으로 벌어진다. 실측: depth 스탬프가 TF 대비 +1.4초 → −3.7초로
#    점프 → RViz 가 "timestamp earlier than all the data in the transform cache" 로
#    **모든 포인트클라우드를 버렸다.** TF 기반 소비자 전부가 영향을 받는다.

def _drifting_stream(mapper, ppm, seconds, hz=30, latency_ms=5.0, stream="depth"):
    """장치 시계가 ROS 대비 ppm 만큼 빠른(느린) 스트림을 흘린다. 최종 오차[ns] 반환."""
    err = 0
    n = int(seconds * hz)
    for i in range(n):
        ros_s = i / hz
        device_s = ros_s * (1.0 + ppm * 1e-6)          # 장치 시계가 다르게 흐른다
        ros_now_ns = int((ros_s + latency_ms * 1e-3) * 1e9)   # 콜백 지연
        out = mapper.map_ms(device_s * 1000.0, ros_now_ns, stream_key=stream)
        err = out - int(ros_s * 1e9)                   # 참 ROS 시각과의 차이
    return err


def test_mapper_corrects_linear_clock_drift():
    """★ 100 ppm 드리프트를 60초 흘려도 오차가 밀리초 수준에 머문다.

    보정이 없으면 100 ppm × 60 s = **6 ms**, 10분이면 60 ms, 한 시간이면 360 ms 로
    계속 벌어진다(실측에서는 초 단위까지 갔다).
    """
    mapper = TimestampMapper(window_ms=5_000.0)
    err_ns = _drifting_stream(mapper, ppm=100.0, seconds=60.0)
    assert abs(err_ns) < 20_000_000        # 20 ms 이내 (지연 + 윈도우 추종 지연)


def test_mapper_tracks_drift_in_both_directions():
    for ppm in (+200.0, -200.0):
        mapper = TimestampMapper(window_ms=5_000.0)
        err_ns = _drifting_stream(mapper, ppm=ppm, seconds=60.0)
        assert abs(err_ns) < 30_000_000, f"ppm={ppm} 에서 {err_ns/1e6:.1f} ms"


def test_mapper_is_robust_to_latency_jitter():
    """콜백 지연은 **항상 양수**다 → 최근 구간의 **최솟값**이 참 오프셋에 가깝다."""
    import random
    rnd = random.Random(0)
    mapper = TimestampMapper(window_ms=3_000.0)
    out_prev = None
    for i in range(300):
        ros_s = i / 30.0
        jitter = rnd.uniform(0.0, 0.030)               # 0~30 ms 지연 지터
        out = mapper.map_ms(ros_s * 1000.0,
                            int((ros_s + jitter) * 1e9), stream_key="color")
        err = out - int(ros_s * 1e9)
        if i > 60:
            assert abs(err) < 5_000_000                # 5 ms 이내 (지터에 안 흔들림)
        out_prev = out


def test_mapper_output_is_monotonic():
    """★ 시간이 거꾸로 가는 타임스탬프는 어떤 소비자도 못 견딘다."""
    import random
    rnd = random.Random(1)
    mapper = TimestampMapper(window_ms=2_000.0)
    prev = None
    for i in range(300):
        ros_s = i / 30.0
        out = mapper.map_ms(ros_s * 1000.0,
                            int((ros_s + rnd.uniform(0, 0.05)) * 1e9),
                            stream_key="depth")
        if prev is not None:
            assert out > prev
        prev = out


def test_streams_still_share_one_offset():
    """★ 스트림 간 **상대 타이밍**은 보존돼야 한다 — 융합(정렬·상보필터)이 그걸 믿는다."""
    mapper = TimestampMapper(window_ms=5_000.0)
    for i in range(100):
        ros_s = i / 30.0
        mapper.map_ms(ros_s * 1000.0, int(ros_s * 1e9), stream_key="color")

    # 같은 장치 시각의 두 스트림 → 같은 ROS 시각으로 매핑돼야 한다
    t_dev = 5_000.0
    a = mapper.map_ms(t_dev, int(6.0 * 1e9), stream_key="color")
    b = mapper.map_ms(t_dev + 0.001, int(9.0 * 1e9), stream_key="depth")
    assert abs(b - a) < 2_000_000        # 2 ms 이내 (단조성 보정 여유)
