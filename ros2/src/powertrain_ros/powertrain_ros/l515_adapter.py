"""Hardware-independent conversions from L515 samples to ROS messages."""

from sensor_msgs.msg import CameraInfo, Image, Imu


class TimestampMapper:
    """Map one RealSense device clock onto the ROS clock."""

    def __init__(self):
        self._offset_ns = None
        self._last_device_ms = None

    def map_ms(self, device_ms: float, ros_now_ns: int) -> int:
        device_ms = float(device_ms)
        device_ns = round(device_ms * 1_000_000)
        if (
            self._offset_ns is None
            or (
                self._last_device_ms is not None
                and device_ms < self._last_device_ms
            )
        ):
            self._offset_ns = int(ros_now_ns) - device_ns
        self._last_device_ms = device_ms
        return device_ns + self._offset_ns


def image_from_array(array, encoding, frame_id, stamp) -> Image:
    """Copy a contiguous image array into a sensor_msgs Image."""
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
    msg.data = array.tobytes()
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
