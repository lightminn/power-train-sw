import math
import os
import sys
import threading
import time

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
from safety_us100.verdict import NO_RESPONSE, Verdict  # noqa: E402


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
        self.monitor = None
        self._reader_lock = threading.Lock()
        self._reader_stop = threading.Event()
        self._reader_thread = None
        self._reader_period_s = None
        self._reader_join_timeout_s = 1.0
        self._latest_verdict = self._fail_safe_verdict(
            "reader_not_started"
        )
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
            self._reader_period_s = 1.0 / hz
            self._start_reader()
            self.create_timer(1.0 / hz, self._sample)
        except BaseException:
            try:
                self.close()
            except BaseException:
                pass
            finally:
                self.destroy_node()
            raise

    def _fail_safe_verdict(self, detail):
        return Verdict(NO_RESPONSE, None, True, 1, detail)

    def _start_reader(self):
        thread = getattr(self, "_reader_thread", None)
        if thread is not None and thread.is_alive():
            return
        stop = self._reader_stop
        stop.clear()
        thread = threading.Thread(
            target=self._reader_loop,
            name="us100-safety-reader",
            daemon=True,
        )
        self._reader_thread = thread
        thread.start()

    def _reader_loop(self):
        stop = self._reader_stop
        while not stop.is_set():
            started = time.monotonic()
            try:
                self.monitor.tick()
                verdict = self.monitor.verdict()
            except BaseException as exc:
                verdict = self._fail_safe_verdict(
                    f"reader_exception:{type(exc).__name__}"
                )
            with self._reader_lock:
                self._latest_verdict = verdict
            elapsed = time.monotonic() - started
            stop.wait(max(self._reader_period_s - elapsed, 0.0))

    def _snapshot_for_publish(self):
        thread = getattr(self, "_reader_thread", None)
        if thread is None or not thread.is_alive():
            return self._fail_safe_verdict("reader_not_running")
        lock = getattr(self, "_reader_lock", None)
        if lock is None:
            return self._fail_safe_verdict("reader_state_unavailable")
        with lock:
            verdict = getattr(self, "_latest_verdict", None)
        if not thread.is_alive():
            return self._fail_safe_verdict("reader_not_running")
        if verdict is None:
            return self._fail_safe_verdict("reader_snapshot_unavailable")
        return verdict

    def _sample(self):
        snapshot_for_publish = getattr(
            self, "_snapshot_for_publish", None
        )
        verdict = (
            snapshot_for_publish()
            if snapshot_for_publish is not None
            else Verdict(
                NO_RESPONSE,
                None,
                True,
                1,
                "reader_not_running",
            )
        )
        msg = SafetyVerdictMsg()
        fill_safety_message(
            msg,
            verdict,
            self.get_clock().now().to_msg(),
        )
        self.publisher.publish(msg)

    def close(self):
        stop = getattr(self, "_reader_stop", None)
        if stop is not None:
            stop.set()
        thread = getattr(self, "_reader_thread", None)
        if (
            thread is not None
            and thread is not threading.current_thread()
            and thread.is_alive()
        ):
            thread.join(
                timeout=getattr(self, "_reader_join_timeout_s", 1.0)
            )
        sensor = self.sensor
        self.sensor = None
        if sensor is not None:
            sensor.close()
        if thread is not None and not thread.is_alive():
            self._reader_thread = None


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
