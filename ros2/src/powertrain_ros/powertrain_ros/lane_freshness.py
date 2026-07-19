"""ROS-free freshness gate for lane-follower IMU samples."""

import math


class ImuFreshnessGate:
    """Require both local receipt and message-header time to be recent."""

    def __init__(self, timeout_s):
        timeout_s = float(timeout_s)
        if not math.isfinite(timeout_s) or timeout_s <= 0.0:
            raise ValueError("timeout_s must be finite and positive")
        self.timeout_s = timeout_s
        self._received_steady_s = None
        self._header_ros_s = None

    def update(self, *, received_steady_s, header_ros_s):
        self._received_steady_s = float(received_steady_s)
        self._header_ros_s = float(header_ros_s)

    def is_fresh(self, *, now_steady_s, now_ros_s):
        if self._received_steady_s is None or self._header_ros_s is None:
            return False
        receive_age_s = float(now_steady_s) - self._received_steady_s
        header_age_s = float(now_ros_s) - self._header_ros_s
        return (
            math.isfinite(receive_age_s)
            and math.isfinite(header_age_s)
            and 0.0 <= receive_age_s <= self.timeout_s
            and 0.0 <= header_age_s <= self.timeout_s
        )
