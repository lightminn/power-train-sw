import math
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from powertrain_msgs.msg import SafetyVerdict as SafetyVerdictMsg
from powertrain_ros.message_adapter import fill_safety_message


sys.path.insert(
    0,
    os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"),
)

from safety_us100.config import SafetyConfig  # noqa: E402
from safety_us100.safety_monitor import SafetyMonitor  # noqa: E402
from safety_us100.us100 import Us100Sensor  # noqa: E402


MIN_SAMPLE_HZ = 5.0
MAX_SAMPLE_HZ = 10.0


def validate_sample_hz(value):
    sample_hz = float(value)
    if (
        not math.isfinite(sample_hz)
        or not MIN_SAMPLE_HZ <= sample_hz <= MAX_SAMPLE_HZ
    ):
        raise ValueError(
            "sample_hz must be finite and within 5.0..10.0 Hz"
        )
    return sample_hz


class Us100SafetyNode(Node):
    def __init__(self):
        super().__init__("us100_safety_node")
        self.sensor = None
        try:
            self.declare_parameter("port", "/dev/ttyTHS1")
            self.declare_parameter("baud", 9600)
            self.declare_parameter("sample_hz", 5.0)
            self.declare_parameter("stop_mm", 200.0)
            self.declare_parameter("fail_stop_count", 3)

            hz = validate_sample_hz(
                self.get_parameter("sample_hz").value
            )
            cfg = SafetyConfig(
                stop_mm=float(self.get_parameter("stop_mm").value),
                fail_stop_count=int(
                    self.get_parameter("fail_stop_count").value
                ),
                port=str(self.get_parameter("port").value),
                baud=int(self.get_parameter("baud").value),
            )
            self.sensor = Us100Sensor(port=cfg.port, baud=cfg.baud)
            self.sensor.open()
            self.monitor = SafetyMonitor(self.sensor, cfg)

            safety_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
            )
            self.publisher = self.create_publisher(
                SafetyVerdictMsg,
                "/safety_verdict",
                safety_qos,
            )
            self.create_timer(1.0 / hz, self._sample)
        except BaseException:
            try:
                self.close()
            except BaseException:
                pass
            finally:
                self.destroy_node()
            raise

    def _sample(self):
        self.monitor.tick()
        msg = SafetyVerdictMsg()
        fill_safety_message(
            msg,
            self.monitor.verdict(),
            self.get_clock().now().to_msg(),
        )
        self.publisher.publish(msg)

    def close(self):
        sensor = self.sensor
        self.sensor = None
        if sensor is not None:
            sensor.close()


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = Us100SafetyNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            if node is not None:
                try:
                    node.close()
                finally:
                    node.destroy_node()
        finally:
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
