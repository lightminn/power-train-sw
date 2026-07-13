"""Hardware-independent conversions from L515 samples to ROS messages."""

import array as std_array
import collections
import numpy as np
from sensor_msgs.msg import CameraInfo, Image, Imu


class TimestampMapper:
    """Map one RealSense device clock onto the ROS clock.

    ★ 오프셋을 **모든 스트림이 공유**하는 것은 의도된 설계다 — color/depth/IMU 의 **상대
      타이밍**이 보존돼야 융합(정렬·상보필터)이 맞는다. 그건 유지한다.

    ⚠️ **드리프트 보정이 없던 것이 버그였다.** 오프셋을 처음 한 번만 앵커하면, 장치 시계와
      ROS 시계의 **주파수가 미세하게 달라**(크리스털이 다르다) 시간이 갈수록 선형으로
      벌어진다. 실측: depth 스탬프가 TF 대비 **+1.4초 → −3.7초로 점프**했고, RViz 가
      "timestamp earlier than all the data in the transform cache" 로 **모든 포인트클라우드를
      버렸다**. TF 기반 소비자(우리 · 로봇팔 팀 · 모든 정합 노드)가 전부 영향을 받는다.

    **보정 방법 — 최소 오프셋 추적 (시계 동기의 표준 기법)**
      순간 오프셋 `ros_now − device` 는 **지연(latency)만큼 항상 양수로 부풀려진다**
      (프레임 도착 → 콜백 실행까지의 시간). 지연은 0 이상이므로,
          측정 오프셋 = 참 오프셋 + 지연ᵢ  (지연ᵢ ≥ 0)
      → **최근 구간의 최솟값**이 참 오프셋에 가장 가깝다. 지터에 강하고, 느린 드리프트는
        따라간다.

    **단조성 보장**: 오프셋이 줄면 출력이 뒤로 갈 수 있다 → 스트림별로 마지막 출력보다
      작아지지 않게 클램프한다. 시간이 거꾸로 가는 타임스탬프는 어떤 소비자도 못 견딘다.
    """

    #: 최소 오프셋을 찾을 구간 (장치 시계 기준). 길수록 안정적이나 드리프트 추종이 느리다.
    WINDOW_MS = 10_000.0

    def __init__(self, window_ms: float = None):
        self._offset_ns = None
        self._last_device_ms = {}
        self._last_out_ns = {}
        self._window_ms = float(
            self.WINDOW_MS if window_ms is None else window_ms)
        self._samples = collections.deque()      # (device_ms, inst_offset_ns)

    def map_ms(
        self, device_ms: float, ros_now_ns: int, stream_key=None
    ) -> int:
        device_ms = float(device_ms)
        device_ns = round(device_ms * 1_000_000)
        inst_offset_ns = int(ros_now_ns) - device_ns

        last_device_ms = self._last_device_ms.get(stream_key)
        backward = (
            last_device_ms is not None and device_ms < last_device_ms
        )
        if self._offset_ns is None or backward:
            # 최초 앵커 · 장치 재연결(시계 되감김) → 처음부터 다시 잡는다
            self._samples.clear()
            self._last_out_ns.clear()
            self._offset_ns = inst_offset_ns

        # ── 드리프트 보정: 최근 구간의 **최소** 오프셋을 따라간다 ──
        self._samples.append((device_ms, inst_offset_ns))
        cutoff = device_ms - self._window_ms
        while self._samples and self._samples[0][0] < cutoff:
            self._samples.popleft()
        self._offset_ns = min(o for _, o in self._samples)

        self._last_device_ms[stream_key] = device_ms
        out_ns = device_ns + self._offset_ns

        # ── 단조성: 시간이 거꾸로 가는 스탬프는 어떤 소비자도 못 견딘다 ──
        last_out = self._last_out_ns.get(stream_key)
        if last_out is not None and out_ns <= last_out:
            out_ns = last_out + 1
        self._last_out_ns[stream_key] = out_ns
        return out_ns


def image_from_array(array, encoding, frame_id, stamp) -> Image:
    """Copy a contiguous image array into a sensor_msgs Image."""
    array = np.asanyarray(array)
    expected = {
        "bgr8": (3, np.dtype(np.uint8)),
        "16UC1": (2, np.dtype(np.uint16)),
    }
    if encoding not in expected:
        raise ValueError(f"unsupported image encoding: {encoding}")
    ndim, dtype = expected[encoding]
    if array.ndim != ndim or array.dtype != dtype:
        raise ValueError(
            f"{encoding} requires {ndim}D {dtype}, "
            f"got {array.ndim}D {array.dtype}"
        )
    if not array.flags.c_contiguous:
        raise ValueError("image array must be C-contiguous")

    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = array.shape[0]
    msg.width = array.shape[1]
    msg.encoding = encoding
    msg.is_bigendian = 0
    msg.step = array.strides[0]
    msg.data = std_array.array("B", array.tobytes())
    return msg


def _distortion_model(model) -> str:
    name = str(model).lower().split(".")[-1]
    if name in {"ftheta", "kannala_brandt4"}:
        return "equidistant"
    return "plumb_bob"


def camera_info_from_intrinsics(intrinsics, frame_id, stamp) -> CameraInfo:
    """Convert a RealSense-like intrinsics object to CameraInfo."""
    fx = float(intrinsics.fx)
    fy = float(intrinsics.fy)
    ppx = float(intrinsics.ppx)
    ppy = float(intrinsics.ppy)

    msg = CameraInfo()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height = intrinsics.height
    msg.width = intrinsics.width
    msg.distortion_model = _distortion_model(intrinsics.model)
    msg.d = [float(value) for value in intrinsics.coeffs]
    msg.k = [fx, 0.0, ppx, 0.0, fy, ppy, 0.0, 0.0, 1.0]
    msg.p = [
        fx, 0.0, ppx, 0.0,
        0.0, fy, ppy, 0.0,
        0.0, 0.0, 1.0, 0.0,
    ]
    return msg


def imu_from_vector(vector, kind, frame_id, stamp) -> Imu:
    """Convert one raw gyro or accelerometer vector to Imu."""
    if kind not in {"gyro", "accel"}:
        raise ValueError("kind must be 'gyro' or 'accel'")

    msg = Imu()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.orientation_covariance[0] = -1.0
    target = (
        msg.angular_velocity
        if kind == "gyro"
        else msg.linear_acceleration
    )
    target.x = float(vector.x)
    target.y = float(vector.y)
    target.z = float(vector.z)
    return msg
