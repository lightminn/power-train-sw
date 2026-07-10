"""ROS2 adapter for the WP5 10-motor chassis controller."""

import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Header
from std_srvs.srv import Trigger

from powertrain_msgs.msg import SafetyVerdict
from powertrain_msgs.msg import WheelState, WheelStates
from powertrain_ros import contract
from powertrain_ros.message_adapter import fill_wheel_states_message
from robot_arm_msgs.msg import ArmStatus, ArrivalStatus, ChassisMode


sys.path.insert(
    0,
    os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"),
)


class ChassisNode(Node):
    def __init__(self):
        super().__init__("chassis_node")
        self.cm = None
        try:
            self._initialize()
        except BaseException:
            self.close()
            self.destroy_node()
            raise

    def _initialize(self):
        self.declare_parameter("fake", False)
        self.declare_parameter("channel", "can0")
        self.declare_parameter("min_rev", 1.0)
        self.declare_parameter("v_max", 1.5)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("mode", contract.MODE_DRIVING)
        self.declare_parameter("safety_required", True)
        self.declare_parameter("safety_topic_timeout", 0.5)
        self.declare_parameter("safety_startup_timeout", 1.0)

        fake = bool(self.get_parameter("fake").value)
        channel = str(self.get_parameter("channel").value)
        min_rev = float(self.get_parameter("min_rev").value)
        v_max = float(self.get_parameter("v_max").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._safety_required = bool(
            self.get_parameter("safety_required").value
        )
        self._safety_topic_timeout = float(
            self.get_parameter("safety_topic_timeout").value
        )
        self._safety_startup_timeout = float(
            self.get_parameter("safety_startup_timeout").value
        )

        from chassis.chassis_manager import (
            ChassisConfig,
            ChassisManager,
            build_real_corners,
        )

        cfg = ChassisConfig(
            watchdog_ms=self._cmd_timeout * 1000.0,
            min_drive_turns_per_s=min_rev,
        )
        cfg.geometry.drive_limit_mps = max(
            v_max,
            cfg.geometry.drive_limit_mps,
        )

        if fake:
            corners = self._build_fake_corners(cfg)
            self.get_logger().warning(
                "FAKE mode: no real motors are controlled"
            )
        else:
            corners = build_real_corners(channel)
            self.get_logger().info("Real chassis on %s" % channel)

        self.cm = ChassisManager(corners, cfg)
        self.cm.connect()

        self.create_subscription(Twist, "/cmd_vel", self._on_cmd_vel, 10)
        self.create_subscription(
            ArmStatus,
            contract.TOPIC_ARM_STATUS,
            self._on_arm_status,
            10,
        )
        safety_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(
            SafetyVerdict,
            "/safety_verdict",
            self._on_safety_verdict,
            safety_qos,
        )

        self.pub_mode = self.create_publisher(
            ChassisMode,
            contract.TOPIC_CHASSIS_MODE,
            10,
        )
        self.pub_arrival = self.create_publisher(
            ArrivalStatus,
            contract.TOPIC_ARRIVAL,
            10,
        )
        self.pub_state = self.create_publisher(
            ChassisMode,
            "/chassis_state",
            10,
        )
        self.pub_wheels = self.create_publisher(
            WheelStates,
            "/wheel_states",
            10,
        )

        self.create_service(Trigger, "~/arm", self._srv_arm)
        self.create_service(Trigger, "~/disarm", self._srv_disarm)
        self.create_service(Trigger, "~/estop", self._srv_estop)
        self.create_service(
            Trigger,
            "~/reset_estop",
            self._srv_reset_estop,
        )

        self._last_arm_status = None
        self._started_ms = self._now_ms()
        self._last_safety_ms = None
        self._overrun_count = 0

        period = 1.0 / self.cm.cfg.loop_hz
        self.create_timer(period, self._tick)
        self.create_timer(0.5, self._publish_mode)
        self.create_timer(1.0, self._publish_state)

        if not self._safety_required:
            self.get_logger().warning(
                "safety_required=false: BENCH/FAKE ONLY; safety topic "
                "startup and freshness enforcement is disabled"
            )
        self.get_logger().info(
            "chassis_node started (loop %.0f Hz, min_rev %.1f, "
            "v_max %.1f, safety_required=%s)"
            % (
                self.cm.cfg.loop_hz,
                min_rev,
                v_max,
                self._safety_required,
            )
        )

    def _build_fake_corners(self, cfg):
        from corner_module.corner_module import CornerModule
        from corner_module.fake import FakeDrive, FakeSteer
        from corner_module.null_steer import NullSteer

        corners = {}
        for wheel in cfg.geometry.wheels:
            steer = FakeSteer() if wheel.steerable else NullSteer()
            corners[wheel.name] = CornerModule(
                steer,
                FakeDrive(),
                cfg.corner,
            )
        return corners

    def _now_ms(self):
        return self.get_clock().now().nanoseconds / 1e6

    def _on_cmd_vel(self, msg: Twist):
        self.cm.set(msg.linear.x, msg.angular.z)

    def _on_safety_verdict(self, msg):
        status_name = {
            SafetyVerdict.CHECKING: "CHECKING",
            SafetyVerdict.VALID: "VALID",
            SafetyVerdict.INVALID_READING: "INVALID_READING",
            SafetyVerdict.NO_RESPONSE: "NO_RESPONSE",
        }.get(msg.status, "CHECKING")
        self.cm.update_external_safety(
            status_name,
            msg.estop_required,
            msg.detail,
        )
        self.cm.set_safety_link_stale(False)
        self._last_safety_ms = self._now_ms()

    def _tick(self):
        now_ms = self._now_ms()
        if self._safety_required:
            if self._last_safety_ms is None:
                expired = (
                    now_ms - self._started_ms
                    > self._safety_startup_timeout * 1000.0
                )
                self.cm.set_safety_link_stale(
                    expired,
                    "safety_startup_timeout",
                )
                if not expired:
                    self.cm.update_external_safety(
                        "CHECKING",
                        False,
                        "startup",
                    )
            else:
                stale = (
                    now_ms - self._last_safety_ms
                    > self._safety_topic_timeout * 1000.0
                )
                self.cm.set_safety_link_stale(
                    stale,
                    "safety_topic_stale",
                )

        started = time.monotonic()
        try:
            self.cm.tick()
        except Exception as exc:
            self.cm.estop("control_exception", str(exc))
        duration_ms = (time.monotonic() - started) * 1000.0
        if duration_ms > 1000.0 / self.cm.cfg.loop_hz:
            self._overrun_count += 1
        msg = WheelStates()
        fill_wheel_states_message(
            msg,
            self.cm.snapshot(),
            self.get_clock().now().to_msg(),
            duration_ms,
            self._overrun_count,
            WheelState,
        )
        self.pub_wheels.publish(msg)

    def _header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        return header

    def _publish_mode(self):
        msg = ChassisMode()
        msg.header = self._header()
        msg.mode = self.get_parameter("mode").value
        self.pub_mode.publish(msg)

    def _publish_state(self):
        state = self.cm.state()
        msg = ChassisMode()
        msg.header = self._header()
        msg.mode = "%s v=%.2f w=%.2f" % (
            state["mode"],
            state["v"],
            state["omega"],
        )
        self.pub_state.publish(msg)

    def publish_arrival(self, mission_id: int, status: str):
        msg = ArrivalStatus()
        msg.header = self._header()
        msg.mission_id = int(mission_id)
        msg.status = status
        self.pub_arrival.publish(msg)
        self.get_logger().info(
            "arrival mission=%d status=%s" % (msg.mission_id, status)
        )

    def _on_arm_status(self, msg: ArmStatus):
        if msg.status != self._last_arm_status:
            self.get_logger().info(
                "arm status mission=%d status=%s"
                % (msg.mission_id, msg.status)
            )
            if msg.status == contract.ARM_DONE:
                self.get_logger().info(
                    "DONE received; WP8 sequencer may resume"
                )
            self._last_arm_status = msg.status

    def _srv_arm(self, _request, response):
        response.success = self.cm.arm()
        state = self.cm.state()
        safety = state["safety"]
        if response.success:
            response.message = "mode=ARMED"
        elif safety.estop_latched:
            response.message = "mode=%s estop_source=%s detail=%s" % (
                self.cm.mode,
                safety.first_source,
                safety.first_detail,
            )
        else:
            response.message = "arm rejected: mode=%s" % self.cm.mode
        self.get_logger().info("arm request: %s" % response.message)
        return response

    def _srv_disarm(self, _request, response):
        self.cm.disarm()
        response.success = True
        response.message = "mode=%s" % self.cm.mode
        return response

    def _srv_estop(self, _request, response):
        self.cm.estop("manual_service", "~/estop")
        response.success = True
        response.message = "estop latched: mode=%s" % self.cm.mode
        self.get_logger().warning("manual E-stop latched")
        return response

    def _srv_reset_estop(self, _request, response):
        response.success = self.cm.reset_estop()
        if response.success:
            response.message = "mode=IDLE; explicit arm required"
        else:
            safety = self.cm.state()["safety"]
            response.message = (
                "reset rejected: mode=%s active=%s"
                % (self.cm.mode, list(safety.active_estop_sources))
            )
        return response

    def close(self):
        manager = self.cm
        self.cm = None
        if manager is None:
            return
        try:
            manager.disarm()
        except BaseException as exc:
            self.get_logger().error("disarm during cleanup failed: %s" % exc)
        try:
            manager.close()
        except BaseException as exc:
            self.get_logger().error("close during cleanup failed: %s" % exc)


def main(argv=None):
    rclpy.init(args=argv)
    node = None
    try:
        node = ChassisNode()
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
