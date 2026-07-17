"""ROS wrapper and TCP owner for the WP5.2 remote-input gateway.

This process never opens CAN or imports pygame.  It owns TCP :9000, decodes
versioned frames, evaluates the pure gateway, and publishes either the drive
adapter or the arm adapter.  ARM output is intentionally hard-disabled until
the five-axis controller, Servo, video feedback, and joint HIL gates pass.
"""

import json
from collections import deque
import socket
import threading
import time
import uuid

from control_msgs.msg import JointJog
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger

from powertrain_ros.remote_input import RemoteInputDecoder
from powertrain_ros.remote_input_gateway import (
    GatewayConfig,
    RemoteInputGateway,
    frame_is_neutral,
    gated_arm_output,
)


DEFAULT_PORT = 9000
ARM_OUTPUT_ENABLED = False
CLIENT_IDLE_TIMEOUT_S = 5.0
MAX_EVENTS_PER_TICK = 256
MAX_VIOLATION_EVENTS_PER_S = 50
MAX_LIFECYCLE_EVENTS = 8
MAX_VIOLATION_KINDS = 64
# ○ E-stop 전역 latch(스펙 r6 §2.1): 수신 스레드가 edge를 즉시 durable
# 슬롯에 기록하고, 다음 ROS tick부터 이 시간 동안 재발행한다. 발행 자체는
# TRANSIENT_LOCAL latched 라 재발행은 구독자 프로세스 재시작 '창'의 보험이다.
ESTOP_REBROADCAST_S = 1.0
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

        self._events_lock = threading.Lock()
        self._motion_frame = None
        self._motion_frames_dropped = 0
        self._lifecycle_events = deque(maxlen=MAX_LIFECYCLE_EVENTS)
        self._violation_events = deque(maxlen=MAX_VIOLATION_KINDS)
        self._status_lock = threading.Lock()
        self._status_line = b"S DISCONNECTED +0.000 +0.000\n"
        self._stop_event = threading.Event()
        self._server_socket = None
        self._closed = False
        self._input_was_fresh = False
        self._last_frame = None
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
        self.pub_gateway_state = self.create_publisher(
            String,
            "/teleop/gateway_state",
            10,
        )
        estop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_estop = self.create_publisher(
            String,
            "/teleop/estop",
            estop_qos,
        )
        self._estop_lock = threading.Lock()
        self._estop_event = None
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

    def _queue_motion_frame(self, frame):
        lock = getattr(self, "_events_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._events_lock = lock
        with lock:
            if getattr(self, "_motion_frame", None) is not None:
                self._motion_frames_dropped = (
                    getattr(self, "_motion_frames_dropped", 0) + 1
                )
            self._motion_frame = frame

    def _queue_lifecycle_event(self, event, session_id):
        lock = getattr(self, "_events_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._events_lock = lock
        overflow = False
        with lock:
            events = getattr(self, "_lifecycle_events", None)
            if events is None:
                events = deque(maxlen=MAX_LIFECYCLE_EVENTS)
                self._lifecycle_events = events
            retained = [
                item for item in events if item[1] != session_id
            ]
            overflow = len(retained) >= events.maxlen
            if overflow:
                retained.pop(0)
            retained.append((event, session_id))
            events.clear()
            events.extend(retained)
            if event == "disconnect":
                # A closed TCP session cannot leave its last motion frame to
                # be accepted after the next session's connect event.
                self._motion_frame = None
        if overflow:
            self._queue_violation(
                "CONTRACT_VIOLATION: event overflow"
            )

    def _queue_violation(self, reason, *, count=1):
        reason = str(reason)
        count = max(1, int(count))
        lock = getattr(self, "_events_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._events_lock = lock
        with lock:
            events = getattr(self, "_violation_events", None)
            if events is None:
                events = deque(maxlen=MAX_VIOLATION_KINDS)
                self._violation_events = events
            for index, (queued_reason, queued_count) in enumerate(events):
                if queued_reason == reason:
                    events[index] = (
                        queued_reason,
                        queued_count + count,
                    )
                    break
            else:
                events.append((reason, count))

    def _queue_decoder_results(self, results, now_s=None):
        event_now_s = time.monotonic() if now_s is None else float(now_s)
        for result in results:
            if result.frame is not None:
                if result.frame.estop_edge:
                    # Do this before the overwritable motion slot.  The TCP
                    # thread only sets a lock-protected flag; ROS publication
                    # stays on _tick and TRANSIENT_LOCAL keeps the event
                    # durable for late subscribers.
                    self._begin_estop_event(event_now_s)
                self._queue_motion_frame(result.frame)
            else:
                rate_lock = getattr(self, "_violation_rate_lock", None)
                if rate_lock is None:
                    rate_lock = threading.Lock()
                    self._violation_rate_lock = rate_lock
                with rate_lock:
                    window_start_s = getattr(
                        self,
                        "_violation_window_start_s",
                        event_now_s,
                    )
                    if (
                        event_now_s < window_start_s
                        or event_now_s - window_start_s >= 1.0
                    ):
                        self._violation_window_start_s = event_now_s
                        self._violation_events_in_window = 0
                    if (
                        getattr(self, "_violation_events_in_window", 0)
                        >= MAX_VIOLATION_EVENTS_PER_S
                    ):
                        self._violation_events_suppressed = (
                            getattr(self, "_violation_events_suppressed", 0)
                            + 1
                        )
                        continue
                    self._violation_events_in_window = (
                        getattr(self, "_violation_events_in_window", 0) + 1
                    )
                self._queue_violation(result.reason)

    def _current_status(self):
        with self._status_lock:
            return self._status_line

    def _serve_client(self, connection):
        connection.settimeout(0.20)
        connection.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        # 상태 회신도 Nagle에 뭉치면 클라이언트 표시가 늦는다 — 양단 NODELAY.
        connection.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self._decoder.start_connection()
        connection_session_id = uuid.uuid4().hex
        self._queue_lifecycle_event("connect", connection_session_id)
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
            self._queue_lifecycle_event(
                "disconnect",
                connection_session_id,
            )

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
                self._queue_violation(
                    "CONTRACT_VIOLATION: TCP server failed: %s" % repr(exc)
                )
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
            lock = getattr(self, "_events_lock", None)
            if lock is None:
                lock = threading.Lock()
                self._events_lock = lock
            with lock:
                lifecycle_events = getattr(
                    self,
                    "_lifecycle_events",
                    None,
                )
                violation_events = getattr(
                    self,
                    "_violation_events",
                    None,
                )
                if lifecycle_events:
                    lifecycle, session_id = lifecycle_events.popleft()
                    event = "lifecycle"
                    payload = (lifecycle, session_id)
                elif violation_events:
                    event = "violation"
                    payload = violation_events.popleft()
                elif getattr(self, "_motion_frame", None) is not None:
                    event = "frame"
                    payload = self._motion_frame
                    self._motion_frame = None
                else:
                    break
            processed += 1
            if event == "lifecycle":
                lifecycle, _session_id = payload
                if lifecycle == "connect":
                    self._gateway.begin_connection()
                elif lifecycle == "disconnect":
                    self._gateway.end_connection()
                    self._last_frame = None
            elif event == "frame":
                if self._gateway.submit(payload):
                    self._last_frame = payload
            elif event == "violation":
                reason, count = payload
                message = (
                    reason
                    if count == 1
                    else "%s (%d occurrences)" % (reason, count)
                )
                self._gateway.contract_violation(message)
                self._log_violation_throttled(message, drain_now_s)
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

    def _begin_estop_event(self, now_s):
        event = {
            "event_id": uuid.uuid4().hex,
            "stamp_s": float(now_s),
            "until_s": float(now_s) + ESTOP_REBROADCAST_S,
            "published": False,
        }
        lock = getattr(self, "_estop_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._estop_lock = lock
        with lock:
            self._estop_event = event

    def _publish_estop_event(self):
        lock = getattr(self, "_estop_lock", None)
        if lock is None:
            lock = threading.Lock()
            self._estop_lock = lock
        with lock:
            event = getattr(self, "_estop_event", None)
            if event is None:
                return False
            event_id = event["event_id"]
            stamp_s = event["stamp_s"]
        message = String()
        message.data = json.dumps(
            {
                "event_id": event_id,
                "stamp_s": stamp_s,
            },
            separators=(",", ":"),
        )
        self.pub_estop.publish(message)
        with lock:
            current = getattr(self, "_estop_event", None)
            if current is not None and current["event_id"] == event_id:
                current["published"] = True
        return True

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
        tick_now_s = time.monotonic()
        self._drain_events()
        estop_event = getattr(self, "_estop_event", None)
        estop_lock = getattr(self, "_estop_lock", None)
        if estop_event is None:
            estop_snapshot = None
        elif estop_lock is None:
            estop_snapshot = dict(estop_event)
        else:
            with estop_lock:
                estop_snapshot = dict(self._estop_event)
        if estop_snapshot is not None:
            if (
                not estop_snapshot["published"]
                or tick_now_s < estop_snapshot["until_s"]
            ):
                self._publish_estop_event()
            if tick_now_s >= estop_snapshot["until_s"]:
                if estop_lock is None:
                    current = getattr(self, "_estop_event", None)
                    if (
                        current is not None
                        and current["event_id"]
                        == estop_snapshot["event_id"]
                    ):
                        self._estop_event = None
                else:
                    with estop_lock:
                        current = getattr(self, "_estop_event", None)
                        if (
                            current is not None
                            and current["event_id"]
                            == estop_snapshot["event_id"]
                        ):
                            self._estop_event = None
        output = self._gateway.tick(tick_now_s)
        with self._status_lock:
            self._status_line = make_status_line(output).encode("utf-8")

        state_message = String()
        state_message.data = json.dumps(
            {
                "state": output.state,
                "input_fresh": bool(output.input_fresh),
                "neutral": bool(
                    self._last_frame is not None
                    and frame_is_neutral(self._last_frame)
                ),
                "stamp_s": tick_now_s,
            },
            separators=(",", ":"),
        )
        self.pub_gateway_state.publish(state_message)

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
