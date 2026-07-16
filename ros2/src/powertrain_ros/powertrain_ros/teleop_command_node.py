"""ROS wrapper and TCP owner for the WP5.2 remote-input gateway.

This process never opens CAN or imports pygame.  It owns TCP :9000, decodes
versioned frames, evaluates the pure gateway, and publishes either the drive
adapter or the arm adapter.  ARM output is intentionally hard-disabled until
the five-axis controller, Servo, video feedback, and joint HIL gates pass.
"""

import queue
import socket
import threading
import time

from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from powertrain_ros.remote_input import RemoteInputDecoder
from powertrain_ros.remote_input_gateway import (
    GatewayConfig,
    RemoteInputGateway,
    gated_arm_output,
)


DEFAULT_PORT = 9000
ARM_OUTPUT_ENABLED = False
CLIENT_IDLE_TIMEOUT_S = 5.0
MAX_EVENTS_PER_TICK = 256
MAX_VIOLATION_EVENTS_PER_S = 50
VIOLATION_LOG_PERIOD_S = 1.0


def make_status_line(output):
    """Keep the legacy ``S ...`` prefix while echoing authoritative state."""
    return "S %s %+.3f %+.3f\n" % (
        output.state,
        output.drive.linear,
        output.drive.angular,
    )


class TeleopCommandNode(Node):
    def __init__(self):
        super().__init__("teleop_command")
        self.declare_parameter("port", DEFAULT_PORT)
        self.declare_parameter("input_timeout_s", 0.20)
        self.declare_parameter("stopping_timeout_s", 2.0)
        self.declare_parameter("max_linear", 1.5)
        self.declare_parameter("max_angular", 1.2)

        self._port = int(self.get_parameter("port").value)
        input_timeout_s = float(
            self.get_parameter("input_timeout_s").value
        )
        self._decoder = RemoteInputDecoder(
            input_timeout_s=input_timeout_s
        )
        self._gateway = RemoteInputGateway(
            GatewayConfig(
                input_timeout_s=input_timeout_s,
                stopping_timeout_s=float(
                    self.get_parameter("stopping_timeout_s").value
                ),
                max_linear=float(self.get_parameter("max_linear").value),
                max_angular=float(
                    self.get_parameter("max_angular").value
                ),
            ),
            arm_output_enabled=ARM_OUTPUT_ENABLED,
            # Task 7 will inject qualified physical evidence only after the
            # arm controller/feedback path and joint HIL are complete.
            wheel_stop_qualified=lambda: False,
            wheel_stopped=lambda: False,
            arm_stationary_ack=lambda: False,
            stow_confirmed=lambda: False,
        )

        self._events = queue.SimpleQueue()
        self._status_lock = threading.Lock()
        self._status_line = b"S DISCONNECTED +0.000 +0.000\n"
        self._stop_event = threading.Event()
        self._server_socket = None
        self._closed = False
        self._input_was_fresh = False
        self._violation_rate_lock = threading.Lock()
        self._violation_window_start_s = time.monotonic()
        self._violation_events_in_window = 0
        self._violation_events_suppressed = 0
        self._violation_events_reported = 0
        self._last_violation_log_s = None

        self.pub_drive = self.create_publisher(
            Twist,
            "/teleop/cmd_vel",
            10,
        )
        self.pub_arm = self.create_publisher(
            JointJog,
            "/arm/teleop_jog",
            10,
        )
        self.pub_assist_bypass = self.create_publisher(
            Bool,
            "/teleop/assist_bypass",
            10,
        )
        self.create_service(Trigger, "~/clear_hold", self._clear_hold)
        self.create_timer(1.0 / 30.0, self._tick)

        self._server_thread = threading.Thread(
            target=self._serve,
            name="remote-input-tcp",
            daemon=True,
        )
        self._server_thread.start()
        self.get_logger().info(
            "remote input TCP :%d; ARM output enabled=%s"
            % (self._port, ARM_OUTPUT_ENABLED)
        )

    def _queue_decoder_results(self, results, now_s=None):
        event_now_s = time.monotonic() if now_s is None else float(now_s)
        for result in results:
            if result.frame is not None:
                self._events.put(("frame", result.frame))
            else:
                with self._violation_rate_lock:
                    if (
                        event_now_s < self._violation_window_start_s
                        or event_now_s - self._violation_window_start_s >= 1.0
                    ):
                        self._violation_window_start_s = event_now_s
                        self._violation_events_in_window = 0
                    if (
                        self._violation_events_in_window
                        >= MAX_VIOLATION_EVENTS_PER_S
                    ):
                        self._violation_events_suppressed += 1
                        continue
                    self._violation_events_in_window += 1
                self._events.put(("violation", result.reason))

    def _current_status(self):
        with self._status_lock:
            return self._status_line

    def _serve_client(self, connection):
        connection.settimeout(0.20)
        connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        # 상태 회신도 Nagle에 뭉치면 클라이언트 표시가 늦는다 — 양단 NODELAY.
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._decoder.start_connection()
        self._events.put(("connect", None))
        last_status_s = 0.0
        last_data_s = time.monotonic()
        try:
            while not self._stop_event.is_set():
                try:
                    data = connection.recv(4096)
                except socket.timeout:
                    data = None
                except OSError:
                    # 클라이언트 RST(예: 강제 종료·Wi-Fi 단절)는 이 연결만의
                    # 종료다 — accept 루프까지 전파되면 서버가 영구 사망해
                    # 원격이 노드 재시작 전까지 불능이 된다(2026-07-17 벤치 실증).
                    break
                if data == b"":
                    break
                now_s = time.monotonic()
                if data:
                    last_data_s = now_s
                    self._queue_decoder_results(
                        self._decoder.feed(
                            data,
                            receive_monotonic_s=now_s,
                        )
                    )
                elif now_s - last_data_s > CLIENT_IDLE_TIMEOUT_S:
                    break
                if data or now_s - last_status_s >= 0.25:
                    try:
                        connection.sendall(self._current_status())
                    except OSError:
                        break
                    last_status_s = now_s
        finally:
            self._queue_decoder_results(self._decoder.end_connection())
            self._events.put(("disconnect", None))

    def _serve(self):
        server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._server_socket = server
        try:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind(("0.0.0.0", self._port))
            server.listen(1)
            server.settimeout(0.20)
            while not self._stop_event.is_set():
                try:
                    connection, _address = server.accept()
                except socket.timeout:
                    continue
                except OSError:
                    if self._stop_event.is_set():
                        break
                    raise
                try:
                    self._serve_client(connection)
                finally:
                    connection.close()
        except BaseException as exc:
            if not self._stop_event.is_set():
                self._events.put(("server_error", repr(exc)))
        finally:
            try:
                server.close()
            except OSError:
                pass

    def _log_violation_throttled(self, message, now_s):
        last_log_s = getattr(self, "_last_violation_log_s", None)
        if (
            last_log_s is not None
            and now_s >= last_log_s
            and now_s - last_log_s < VIOLATION_LOG_PERIOD_S
        ):
            return False
        self.get_logger().error(message)
        self._last_violation_log_s = now_s
        return True

    def _drain_events(self, max_events=MAX_EVENTS_PER_TICK, now_s=None):
        drain_now_s = time.monotonic() if now_s is None else float(now_s)
        processed = 0
        while processed < max_events:
            try:
                event, payload = self._events.get_nowait()
            except queue.Empty:
                break
            processed += 1
            if event == "connect":
                self._gateway.begin_connection()
            elif event == "disconnect":
                self._gateway.end_connection()
            elif event == "frame":
                self._gateway.submit(payload)
            elif event == "violation":
                self._gateway.contract_violation(payload)
                self._log_violation_throttled(payload, drain_now_s)
            elif event == "server_error":
                self._gateway.contract_violation(
                    "CONTRACT_VIOLATION: TCP server failed: %s" % payload
                )
                self.get_logger().error("TCP server failed: %s" % payload)
        suppressed = getattr(self, "_violation_events_suppressed", 0)
        reported = getattr(self, "_violation_events_reported", 0)
        unreported = max(0, suppressed - reported)
        if unreported and self._log_violation_throttled(
            "CONTRACT_VIOLATION: %d decoder violations suppressed"
            % unreported,
            drain_now_s,
        ):
            self._violation_events_reported = suppressed
        return processed

    def _publish_drive(self, output):
        message = Twist()
        message.linear.x = float(output.drive.linear)
        message.angular.z = float(output.drive.angular)
        self.pub_drive.publish(message)

    def _publish_arm(self, output):
        arm = gated_arm_output(
            output.arm,
            enabled=ARM_OUTPUT_ENABLED,
        )
        message = JointJog()
        message.header.stamp = self.get_clock().now().to_msg()
        message.joint_names = [arm.joint_name]
        message.velocities = [float(arm.joint_velocity)]
        self.pub_arm.publish(message)

    def _publish_assist_bypass(self, output):
        message = Bool()
        message.data = bool(output.assist_bypass)
        self.pub_assist_bypass.publish(message)

    def _clear_hold(self, _request, response):
        response.success = self._gateway.clear_hold()
        response.message = (
            "MOTION_HOLD cleared; fresh neutral input required"
            if response.success
            else "clear rejected: gateway is not in MOTION_HOLD"
        )
        return response

    def _tick(self):
        self._drain_events()
        output = self._gateway.tick(time.monotonic())
        with self._status_lock:
            self._status_line = make_status_line(output).encode("utf-8")

        input_was_fresh = self._input_was_fresh
        self._input_was_fresh = bool(output.input_fresh)
        # On the fresh→stale edge, publish one explicit zero so authority stops
        # on this tick.  Silence after that preserves source-freshness expiry.
        if not output.input_fresh:
            if input_was_fresh:
                self._publish_drive(output)
                self._publish_arm(output)
            return
        self._publish_drive(output)
        self._publish_arm(output)
        self._publish_assist_bypass(output)

    def close(self):
        if self._closed:
            return
        self._closed = True
        self._stop_event.set()
        if self._server_socket is not None:
            try:
                self._server_socket.close()
            except OSError:
                pass
        if self._server_thread.ident is not None:
            self._server_thread.join(timeout=1.0)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
