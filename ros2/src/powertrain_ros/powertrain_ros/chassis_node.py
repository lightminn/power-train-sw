"""ROS2 adapter for the WP5 10-motor chassis controller.

The verified WP5.1 default remains an external ``/cmd_vel`` subscription.  The
WP5.2 command-authority path is opt-in with ``authority_enabled=true``:

    /teleop/cmd_vel ───┐
                       ├─→ CommandAuthority ─→ ChassisManager.set()
    /autonomy/cmd_vel ─┘            └─→ /command_authority/state

No ROS ``/cmd_vel`` message is republished by the embedded path.

★ zero-confirmed handover: after a mode change, the selected source must send
  a neutral command once before any command is accepted.  This prevents a
  still-live full-speed command from taking effect at the handover instant.

⚠️ A stale source calls no ``set()`` at all.  That deliberate absence works
  with the existing ChassisManager command watchdog, which naturally drives
  the command to zero.  Repeatedly setting zero here would keep that watchdog
  alive and defeat the fail-silent boundary.
"""

import math
import os
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool, Header, String
from std_srvs.srv import SetBool, Trigger

from powertrain_msgs.msg import SafetyVerdict
from powertrain_msgs.msg import WheelState, WheelStates
from powertrain_ros import contract
from powertrain_ros.arm_interlock import ArmInterlock
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
ARM_GATE_PRODUCTION = "production"
ARM_GATE_ABSENT_FIELD = "arm_absent_field"
ARM_GATE_MODES = {ARM_GATE_PRODUCTION, ARM_GATE_ABSENT_FIELD}


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


def validate_arm_gate_mode(value):
    mode = str(value)
    if mode not in ARM_GATE_MODES:
        raise ValueError(
            "arm_gate_mode must be 'production' or 'arm_absent_field'"
        )
    return mode


class ChassisNode(Node):
    def __init__(self):
        super().__init__("chassis_node")
        self.cm = None
        self._can_session = None
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
        self.declare_parameter("authority_enabled", False)
        self.declare_parameter("contract_v2_verified", False)
        self.declare_parameter("arm_gate_mode", "production")

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
        self._authority_enabled = bool(
            self.get_parameter("authority_enabled").value
        )
        self._contract_v2_verified = bool(
            self.get_parameter("contract_v2_verified").value
        )
        self._arm_gate_mode = validate_arm_gate_mode(
            self.get_parameter("arm_gate_mode").value
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
            from chassis.runtime_lock import RealCanSession

            # 실물 버스/코너를 만들기 전에 lock을 잡고 node cleanup까지 유지한다.
            self._can_session = RealCanSession(
                channel=channel,
                owner="chassis_node",
            )
            self._can_session.__enter__()
            corners = build_real_corners(channel, wheel_map=wheel_map)
            self.get_logger().info("Real chassis on %s" % channel)

        self.cm = ChassisManager(corners, cfg)
        self.cm.connect()
        self._arm_interlock = ArmInterlock(timeout_s=0.5)
        # arm_absent_field의 mock은 실제 sample/latch와 분리한다. publisher가
        # 나타나는 tick부터 이 객체를 선택하지 않아 즉시 real default-deny로 복귀한다.
        self._arm_absent_interlock = ArmInterlock(timeout_s=0.5)
        self._arm_override_requested = False
        self._chassis_mode_intent = contract.MODE_STOW_REQUEST
        self._last_arm_status = None

        self._authority = None
        self.pub_authority_state = None
        if self._authority_enabled:
            from chassis.authority import (
                AUTO,
                AUTO_SOURCE,
                IDLE,
                MANUAL,
                MANUAL_SOURCE,
                CommandAuthority,
            )

            self._authority = CommandAuthority()
            self._authority.set_mode(IDLE)
            self.create_subscription(
                Twist,
                "/teleop/cmd_vel",
                lambda msg: self._on_authority_cmd(MANUAL_SOURCE, msg),
                10,
            )
            self.create_subscription(
                Twist,
                "/autonomy/cmd_vel",
                lambda msg: self._on_authority_cmd(AUTO_SOURCE, msg),
                10,
            )
            self.pub_authority_state = self.create_publisher(
                String,
                "/command_authority/state",
                10,
            )
            self.create_service(
                Trigger,
                "~/authority_manual",
                lambda _request, response: self._set_authority_mode(
                    MANUAL,
                    response,
                ),
            )
            self.create_service(
                Trigger,
                "~/authority_auto",
                lambda _request, response: self._set_authority_mode(
                    AUTO,
                    response,
                ),
            )
            self.create_service(
                Trigger,
                "~/authority_idle",
                lambda _request, response: self._set_authority_mode(
                    IDLE,
                    response,
                ),
            )
        else:
            # Deprecated: WP5.2 Task 4에서 authority 내장 경로로 대체 예정.
            self.create_subscription(
                Twist,
                "/cmd_vel",
                self._on_cmd_vel,
                10,
            )
        arm_status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            ArmStatus,
            contract.TOPIC_ARM_STATUS,
            self._on_arm_status,
            arm_status_qos,
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
            arm_status_qos,
        )
        if (
            self._contract_v2_verified
            and self._arm_gate_mode != ARM_GATE_ABSENT_FIELD
        ):
            self.pub_arrival = self.create_publisher(
                ArrivalStatus,
                contract.TOPIC_ARRIVAL,
                10,
            )
        else:
            self.pub_arrival = None
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
        self.create_service(
            SetBool,
            "~/arm_lock_override",
            self._srv_arm_lock_override,
        )

        self._started_ms = self._now_ms()
        self._last_safety_ms = None
        from chassis.chassis_mode import ChassisModeSelector
        # v1 selector 코드는 보존하지만 contract-v2 supervisor(Task 5)가
        # set_chassis_mode_intent()를 연결하기 전에는 호출하지 않는다.
        self._mode_sel = ChassisModeSelector()
        self._mission_mode = None
        self._follow_lead = False
        self._omega = 0.0
        self._roll = 0.0
        self._pitch = 0.0
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
        if self._arm_gate_mode == ARM_GATE_ABSENT_FIELD:
            self.get_logger().warning(
                "arm_gate_mode=arm_absent_field ACTIVE: "
                "/arm_status publisher가 "
                "0일 때만 내부 STOWED_LOCKED mock으로 freshness를 대체합니다. "
                "운영자는 바퀴 부양과 팔의 기계적 접힘·고정을 육안 확인하고 "
                "확인자를 journal/운영 로그에 기록해야 합니다."
            )
        self.get_logger().info(
            "chassis_node started (loop %.0f Hz, min_rev %.1f, "
            "v_max %.1f, safety_required=%s, authority_enabled=%s, "
            "contract_v2_verified=%s, arm_gate_mode=%s)"
            % (
                self.cm.cfg.loop_hz,
                min_rev,
                v_max,
                self._safety_required,
                self._authority_enabled,
                self._contract_v2_verified,
                self._arm_gate_mode,
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

    def _now_s(self):
        """The only clock domain used for every ArmInterlock call."""
        return self._now_ms() / 1000.0

    def _remote_owner_selected(self):
        return (
            self._authority_enabled
            and self._authority is not None
            and self._authority.mode == "MANUAL"
        )

    def _arm_gate_decision(self, now_s):
        """Return ``(drive_allowed, detail)`` for the manager final gate."""
        if self._arm_override_requested:
            self._chassis_mode_intent = contract.MODE_STOW_REQUEST
            if not self._remote_owner_selected():
                return False, "operator_override_remote_owner_required"
            allowed = self._arm_interlock.drive_allowed(
                "REMOTE_ARM_OVERRIDE",
                now_s,
                manual_override=True,
            )
            reason = self._arm_interlock.hold_reason(
                "REMOTE_ARM_OVERRIDE",
                now_s,
                manual_override=True,
            )
            return allowed, "" if allowed else reason

        gate = self._arm_interlock
        if (
            self._arm_gate_mode == "arm_absent_field"
            and self.count_publishers(contract.TOPIC_ARM_STATUS) == 0
        ):
            # This profile replaces freshness only. A previously observed
            # GRIP_LOST latch or contract violation must still block motion.
            real_reason = self._arm_interlock.hold_reason(
                "EMPTY_STOWED",
                now_s,
            )
            if real_reason == "grip_lost_latched" or real_reason.startswith(
                "arm_contract_violation:"
            ):
                return False, real_reason
            self._arm_absent_interlock.update(
                contract.ARM_STOWED_LOCKED,
                0,
                now_s,
                now_s,
            )
            gate = self._arm_absent_interlock

        # Task 5 will choose one qualified payload profile. Until then both
        # approved locked postures are accepted so CARRYING_LOCKED stays
        # usable.
        allowed = (
            gate.drive_allowed("EMPTY_STOWED", now_s)
            or gate.drive_allowed("CARRYING_LOCKED", now_s)
        )
        if allowed:
            return True, ""
        return False, gate.hold_reason("EMPTY_STOWED", now_s)

    def _tick_arm_gate(self, now_s):
        allowed, detail = self._arm_gate_decision(now_s)
        self.cm.set_arm_motion_hold(not allowed, detail)
        return allowed

    def _discard_pending_command(self):
        if self.cm.mode == "ARMED":
            self.cm.set(0.0, 0.0)

    def _override_activation_error(self, now_s):
        if not self._contract_v2_verified:
            return "contract_v2_verified_required"
        if not self._remote_owner_selected():
            return "remote_owner_required"

        arm_reason = self._arm_interlock.hold_reason(
            "REMOTE_ARM_OVERRIDE",
            now_s,
            manual_override=True,
        )
        if arm_reason:
            if arm_reason == "operator_override_inhibited_by_fresh_arm":
                return "arm_status_must_be_stale"
            return arm_reason

        state = self.cm.state()
        safety = state["safety"]
        if safety.estop_latched or safety.active_estop_sources:
            return "estop_or_safety_condition_active"
        other_holds = set(safety.hold_sources) - {"robot_arm"}
        if other_holds:
            return "other_motion_hold_active:%s" % ",".join(
                sorted(other_holds)
            )
        if self.cm.mode != "IDLE" and "robot_arm" not in safety.hold_sources:
            return "chassis_must_be_idle_or_arm_held"
        if not self.cm.snapshot().healthy:
            return "motor_health_not_ready"

        # Task 5 TODO: require supervisor active-mission abort plus explicit
        # ACK that ArrivalStatus publication stopped before override
        # activation.
        return ""

    def _srv_arm_lock_override(self, request, response):
        if not request.data:
            self._arm_override_requested = False
            response.success = True
            response.message = "arm lock override disabled"
            return response

        now_s = self._now_s()
        error = self._override_activation_error(now_s)
        if error:
            response.success = False
            response.message = "override rejected: %s" % error
            return response

        self._arm_override_requested = True
        self.set_chassis_mode_intent(contract.MODE_STOW_REQUEST)
        self._discard_pending_command()
        response.success = True
        response.message = (
            "override requested: REMOTE_ARM_OVERRIDE only; "
            "fresh ArmStatus immediately removes drive permission"
        )
        return response

    def _seed_initial_safety(self):
        if self._safety_required:
            self.cm.update_external_safety(
                "CHECKING",
                False,
                "startup",
            )

    def _on_cmd_vel(self, msg: Twist):
        self.cm.set(msg.linear.x, msg.angular.z)

    def _on_authority_cmd(self, source, msg: Twist):
        self._authority.submit(
            source,
            msg.linear.x,
            msg.angular.z,
            self._now_ms() / 1000.0,
        )

    def _set_authority_mode(self, mode, response):
        response.success = self._authority.set_mode(mode)
        response.message = "mode=%s" % mode
        if response.success:
            self.get_logger().warning(
                "authority mode → %s (중립 확인 전까지 명령 전달 안 함)"
                % mode
            )
        return response

    def _tick_authority(self, now_s):
        command = self._authority.select(now_s)
        self.pub_authority_state.publish(
            String(data="%s|%s" % (self._authority.mode, command.reason))
        )
        if command.ok:
            self.cm.set(command.v, command.omega)

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
        now_s = now_ms / 1000.0
        if self._authority_enabled:
            self._tick_authority(now_s)
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

        if not hasattr(self, "_tick_arm_gate"):
            # Existing ROS adapter unit tests call this unbound method with a
            # SimpleNamespace. A real manager still fails closed if a partial
            # node ever reaches this path.
            if hasattr(self.cm, "set_arm_motion_hold"):
                self.cm.set_arm_motion_hold(True, "arm_gate_uninitialized")
        else:
            try:
                self._tick_arm_gate(now_s)
            except Exception as exc:
                self.cm.set_arm_motion_hold(
                    True,
                    "arm_gate_exception:%s" % exc,
                )
                self.get_logger().error("arm gate evaluation failed: %s" % exc)

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

    def set_chassis_mode_intent(self, mode: str):
        allowed = contract.LOCK_MODES | {
            contract.MODE_MISSION_STOP,
            contract.MODE_STOW_REQUEST,
        }
        if mode not in allowed:
            raise ValueError("unsupported chassis mode intent: %s" % mode)
        if self._arm_override_requested and mode != contract.MODE_STOW_REQUEST:
            self._chassis_mode_intent = contract.MODE_STOW_REQUEST
            return False
        self._chassis_mode_intent = mode
        return True

    def _effective_chassis_mode(self):
        if not self._contract_v2_verified:
            return contract.MODE_CORNERING
        if self._arm_override_requested:
            return contract.MODE_STOW_REQUEST
        if (
            self._arm_gate_mode == "arm_absent_field"
            and self._chassis_mode_intent == contract.MODE_MISSION_STOP
        ):
            return contract.MODE_STOW_REQUEST
        return self._chassis_mode_intent

    def _publish_mode(self):
        """Publish compatibility lock or the explicit contract-v2 intent."""
        # ChassisModeSelector is v1 vocabulary. Keep its implementation for
        # history, but do not call it until Task 5 maps supervisor state into
        # set_chassis_mode_intent().
        mode = self._effective_chassis_mode()
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
        if (
            not self._contract_v2_verified
            or self._arm_gate_mode == "arm_absent_field"
            or self.pub_arrival is None
        ):
            self.get_logger().warning(
                "ArrivalStatus blocked by compatibility/arm_absent_field lock"
            )
            return False
        msg = ArrivalStatus()
        msg.header = self._header()
        msg.mission_id = int(mission_id)
        msg.status = status
        self.pub_arrival.publish(msg)
        self.get_logger().info(
            "arrival mission=%d status=%s" % (msg.mission_id, status)
        )
        return True

    def _arm_status_stamp_s(self, msg):
        return (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) / 1_000_000_000.0
        )

    def _on_arm_status(self, msg: ArmStatus):
        now_s = self._now_s()
        accepted = self._arm_interlock.update(
            msg.status,
            msg.mission_id,
            self._arm_status_stamp_s(msg),
            now_s,
        )
        if not accepted:
            self.get_logger().warning(
                "arm status rejected mission=%d status=%s"
                % (msg.mission_id, msg.status)
            )
            return
        if msg.status != self._last_arm_status:
            self.get_logger().info(
                "arm status mission=%d status=%s"
                % (msg.mission_id, msg.status)
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
        can_session = self._can_session
        self._can_session = None
        if manager is not None:
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
        # 모든 실물 bus/corner fd가 닫힌 뒤 owner flock fd를 놓는다.
        if can_session is not None:
            can_session.close()


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
