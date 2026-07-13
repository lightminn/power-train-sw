"""ROS2 adapter for the WP5 10-motor chassis controller."""

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Float32, Header, String
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


# A no-response US-100 transaction can consume two 0.2 s request paths.
# Reserve another 0.35 s for timer scheduling and DDS delivery jitter.
US100_NO_RESPONSE_WORST_CASE_S = 0.4
SAFETY_TOPIC_SCHEDULING_MARGIN_S = 0.35
MIN_SAFETY_TOPIC_TIMEOUT_S = (
    US100_NO_RESPONSE_WORST_CASE_S + SAFETY_TOPIC_SCHEDULING_MARGIN_S
)
DEFAULT_SAFETY_TOPIC_TIMEOUT_S = MIN_SAFETY_TOPIC_TIMEOUT_S


def validate_safety_topic_timeout(value):
    timeout_s = float(value)
    if (
        not math.isfinite(timeout_s)
        or timeout_s < MIN_SAFETY_TOPIC_TIMEOUT_S
    ):
        raise ValueError(
            "safety_topic_timeout must be finite and at least 0.75 s"
        )
    return timeout_s


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
        # 🛠️ 중륜 2개(ODrive node 13/14) 없이 4륜만. 중간 보드를 부하모터에 쓸 때.
        #    ⚠️ 임시 구성 — 없으면 node 13/14 stale → 코너 FAULT → 전체 estop.
        self.declare_parameter("four_wheel", False)
        self.declare_parameter("min_rev", 1.0)
        self.declare_parameter("v_max", 1.5)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("mode", contract.MODE_DRIVING)
        self.declare_parameter("safety_required", True)
        self.declare_parameter(
            "safety_topic_timeout",
            DEFAULT_SAFETY_TOPIC_TIMEOUT_S,
        )
        self.declare_parameter("safety_startup_timeout", 1.0)
        # 감속 힌트(/obstacle/speed_scale) 신선도. 넘으면 fail-open (§_expire_speed_scale)
        self.declare_parameter("speed_scale_timeout", 0.5)

        fake = bool(self.get_parameter("fake").value)
        channel = str(self.get_parameter("channel").value)
        min_rev = float(self.get_parameter("min_rev").value)
        v_max = float(self.get_parameter("v_max").value)
        self._cmd_timeout = float(self.get_parameter("cmd_timeout").value)
        self._safety_required = bool(
            self.get_parameter("safety_required").value
        )
        self._safety_topic_timeout = validate_safety_topic_timeout(
            self.get_parameter("safety_topic_timeout").value
        )
        self._safety_startup_timeout = float(
            self.get_parameter("safety_startup_timeout").value
        )
        self._scale_timeout = float(
            self.get_parameter("speed_scale_timeout").value
        )

        from chassis.chassis_manager import (
            ChassisConfig,
            ChassisManager,
            build_real_corners,
        )

        four_wheel = bool(self.get_parameter("four_wheel").value)
        cfg = ChassisConfig(
            watchdog_ms=self._cmd_timeout * 1000.0,
            min_drive_turns_per_s=min_rev,
        )
        wheel_map = None
        if four_wheel:
            from chassis.chassis_manager import FOUR_WHEEL_MAP
            from chassis.kinematics import four_wheel_geometry
            # ★ 기하와 매핑은 **반드시 짝** (이름이 어긋나면 KeyError)
            wheel_map = FOUR_WHEEL_MAP
            cfg.geometry = four_wheel_geometry()
            self.get_logger().warning(
                "🛠️ 4륜 모드 — 중륜(node 13/14) 없이 앞뒤 4륜만 구동 (임시 구성). "
                "지상 주행 시 중륜이 끌려다니며 저항·스크럽을 만든다.")
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
            # ★ CAN 단독 소유권 — teleop_server 가 이미 잡고 있으면 CanBusBusy 로 죽는다.
            #   같은 모터에 상반된 명령이 가는 것보다 안 뜨는 게 낫다.
            corners = build_real_corners(channel, owner="chassis_node",
                                         wheel_map=wheel_map)
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
        # 전방 감속 힌트 (인지 계층 · obstacle_zones). 🛑 안전 게이트가 아니다 —
        # 게이트는 US-100 + SafetyInterlock 이다. 여기서는 v 만 스케일한다.
        self.create_subscription(
            Float32,
            "/obstacle/speed_scale",
            self._on_speed_scale,
            10,
        )
        # ★ `/chassis_mode` 는 **팔과의 계약 토픽**이고 이 노드가 **단독 소유**한다.
        #   미션 시퀀서는 `/mission/chassis_mode` 로 **요청**만 하고, 운동 상태(선회·험지)는
        #   여기서 /odom·/imu 로 판단해 합친다. 두 노드가 쓰면 팔이 번갈아 받아
        #   **우리가 정차 중인데 DRIVING 을 보고 팔이 움직인다.**
        self.create_subscription(String, "/mission/chassis_mode",
                                 self._on_mission_mode, 10)
        # 앞 로봇 추종 중 → FOLLOW_LEAD (팔 자세 락. 앞 차 급정거 시 팔이 흔들린다)
        self.create_subscription(Bool, "/follow/active",
                                 lambda m: setattr(self, "_follow_lead", m.data), 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(Imu, "/imu/filtered", self._on_imu,
                                 QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=1,
                                            reliability=ReliabilityPolicy.BEST_EFFORT))

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
        self._speed_scale = 1.0
        self._last_scale_ms = None
        from chassis.chassis_mode import ChassisModeSelector
        self._mode_sel = ChassisModeSelector()
        self._mission_mode = None
        self._follow_lead = False
        self._omega = 0.0
        self._roll = 0.0
        self._pitch = 0.0
        self._scale_stale_logged = False
        self._overrun_count = 0
        self._wheel_telemetry_failed = False
        self._seed_initial_safety()

        period = 1.0 / self.cm.cfg.loop_hz
        self.create_timer(period, self._tick)
        self.create_timer(0.1, self._publish_mode)   # 팔이 코너 진입을 늦게 알면 의미 없다
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

    def _seed_initial_safety(self):
        if self._safety_required:
            self.cm.update_external_safety(
                "CHECKING",
                False,
                "startup",
            )

    def _on_cmd_vel(self, msg: Twist):
        self.cm.set(msg.linear.x, msg.angular.z)

    def _on_speed_scale(self, msg: Float32):
        self._speed_scale = min(1.0, max(0.0, float(msg.data)))
        self._last_scale_ms = self._now_ms()
        if self._scale_stale_logged:
            self.get_logger().info("감속 힌트 복구 — 다시 반영한다")
            self._scale_stale_logged = False
        self.cm.set_speed_scale(self._speed_scale)

    def _expire_speed_scale(self, now_ms):
        """감속 힌트가 끊기면 **fail-open**(제한 해제)한다.

        ★ 왜 fail-open 인가 — 이건 **감속 힌트**지 안전 게이트가 아니기 때문이다.
          실제 충돌 방지는 **US-100(독립 초음파) + SafetyInterlock** 이 하고, 그건
          끊기면 `MOTION_HOLD`/`ESTOP` 으로 **fail-closed** 한다(그대로 유지).
          인지 노드가 죽었다고 여기서 fail-closed 하면, **위험이 없는데도 로봇이
          멈춰버린다** — 게다가 이 힌트가 없던 시절(오늘 이전)이 곧 fail-open 상태이므로
          기존 동작 대비 퇴행도 없다.
          ⚠️ 바꿔 말하면 **depth 인지는 안전 책임을 지지 않는다.** 안전은 US-100 이다.
        """
        if self._last_scale_ms is None or self._speed_scale >= 1.0:
            return
        if now_ms - self._last_scale_ms <= self._scale_timeout * 1000.0:
            return
        self._speed_scale = 1.0
        self.cm.set_speed_scale(1.0)
        if not self._scale_stale_logged:
            self.get_logger().warn(
                "감속 힌트 stale — 제한 해제(fail-open). 충돌 방지는 US-100 이 담당한다.")
            self._scale_stale_logged = True

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

        self._expire_speed_scale(now_ms)

        started = time.monotonic()
        try:
            self.cm.tick()
        except Exception as exc:
            self.cm.estop("control_exception", str(exc))
        duration_ms = (time.monotonic() - started) * 1000.0
        if duration_ms > 1000.0 / self.cm.cfg.loop_hz:
            self._overrun_count += 1
        try:
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
        except Exception as exc:
            was_failed = self._wheel_telemetry_failed
            self._wheel_telemetry_failed = True
            if not was_failed:
                self.get_logger().error(
                    "wheel telemetry failed: %s" % exc
                )
        else:
            was_failed = self._wheel_telemetry_failed
            self._wheel_telemetry_failed = False
            if was_failed:
                self.get_logger().info("wheel telemetry recovered")

    def _header(self):
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "base_link"
        return header

    def _on_mission_mode(self, msg: String):
        self._mission_mode = msg.data

    def _on_odom(self, msg: Odometry):
        self._omega = msg.twist.twist.angular.z

    def _on_imu(self, msg: Imu):
        q = msg.orientation
        self._roll = math.atan2(2 * (q.w * q.x + q.y * q.z),
                                1 - 2 * (q.x * q.x + q.y * q.y))
        self._pitch = math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x))))

    def _publish_mode(self):
        """★ `/chassis_mode` 단독 발행 — 팔이 이걸 보고 자세를 락한다.

        우선순위: MISSION_STOP > FOLLOW_LEAD > ROUGH_TERRAIN > CORNERING > DRIVING.
        ⚠️ 지금까지 **파라미터 값(항상 DRIVING)만 발행**하고 있었다 — 팔은 코너·험지에서도
           DRIVING 만 받아 **자세를 락하지 않았다.** 계약은 있었지만 구현이 비어 있었다.
        """
        mode = self._mode_sel.update(
            self._now_ms() / 1000.0,
            omega=self._omega, roll=self._roll, pitch=self._pitch,
            mission_mode=self._mission_mode, follow_lead=self._follow_lead,
        )
        msg = ChassisMode()
        msg.header = self._header()
        msg.mode = mode
        self.pub_mode.publish(msg)

    def _publish_state(self):
        try:
            state = self.cm.state()
            msg = ChassisMode()
            msg.header = self._header()
            msg.mode = "%s v=%.2f w=%.2f" % (
                state["mode"],
                state["v"],
                state["omega"],
            )
            self.pub_state.publish(msg)
        except Exception as exc:
            self.get_logger().error(
                "chassis state publication failed: %s" % exc
            )

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
            manager.estop("node_shutdown", "chassis node cleanup")
        except BaseException as exc:
            self.get_logger().error(
                "E-stop during cleanup failed: %s" % exc
            )
        for name, corner in manager.corners.items():
            try:
                corner.close()
            except BaseException as exc:
                self.get_logger().error(
                    "corner %s close during cleanup failed: %s"
                    % (name, exc)
                )


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
