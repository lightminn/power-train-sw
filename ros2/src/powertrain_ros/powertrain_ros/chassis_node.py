"""ROS2 adapter for the WP5 10-motor chassis controller.

The verified WP5.1 default remains an external ``/cmd_vel`` subscription.  The
WP5.2 command-authority path is opt-in with ``authority_enabled=true``:

    /teleop/cmd_vel ───┐
                       ├─→ CommandAuthority ─→ ChassisManager.set()
    /autonomy/cmd_vel ─┘            └─→ /command_authority/state

No ROS ``/cmd_vel`` message is republished by the embedded path.

★ qualified handover: a moving source-to-source request first commands zero,
  waits for the qualified six-wheel stop predicate, and only then changes
  owner.  The selected source must still send a neutral command once before
  nonzero output is accepted.  An unqualified predicate rejects the request.

⚠️ The tick that detects a stale source calls no ``set()`` and enters
  ``MOTION_HOLD``.  Subsequent hold ticks explicitly command zero and require
  the ``authority_clear_hold`` acknowledgement before another owner request.
"""

import collections
import json
import math
import os
import sys
import time

import rclpy
from ament_index_python.packages import get_package_share_directory
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

from chassis import remote_assist  # noqa: E402


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
MISSION_OWNER_CHASSIS = "chassis_supervisor"
MISSION_OWNER_LEGACY = "legacy_mission_node"
MISSION_OWNERS = {MISSION_OWNER_CHASSIS, MISSION_OWNER_LEGACY}
SAFETY_STATE_PUBLISH_PERIOD_S = 0.2


def default_wheel_stop_config_path():
    return os.path.join(
        get_package_share_directory("powertrain_ros"),
        "config",
        "wheel_stop.yaml",
    )


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


def validate_mission_contract_owner(value):
    owner = str(value)
    if owner not in MISSION_OWNERS:
        raise ValueError(
            "mission_contract_owner must be 'chassis_supervisor' or "
            "'legacy_mission_node'"
        )
    return owner


class ChassisNode(Node):
    def __init__(self):
        super().__init__("chassis_node")
        self.cm = None
        self._can_session = None
        self._can_bus_sampler = None
        self._observability_event_client = None
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
        self.declare_parameter("min_rev", 0.0)
        self.declare_parameter("friction_ff", 0.0)
        self.declare_parameter("friction_v_knee", 0.5)
        self.declare_parameter("v_max", 1.5)
        self.declare_parameter("cmd_timeout", 0.5)
        self.declare_parameter("mode", contract.MODE_DRIVING)
        self.declare_parameter("safety_required", True)
        self.declare_parameter(
            "safety_topic_timeout",
            DEFAULT_SAFETY_TOPIC_TIMEOUT_S,
        )
        self.declare_parameter("safety_startup_timeout", 1.0)
        self.declare_parameter("extraction_enabled", False)
        self.declare_parameter("authority_enabled", False)
        self.declare_parameter("assist_enabled", False)
        self.declare_parameter("contract_v2_verified", False)
        self.declare_parameter("boot_qualification_enabled", False)
        self.declare_parameter("arm_gate_mode", "production")
        self.declare_parameter("arm_override_ttl_s", 30.0)
        self.declare_parameter("mission_contract_owner", MISSION_OWNER_CHASSIS)
        self.declare_parameter("mission_id_path", "/var/lib/powertrain/mission_id")

        fake = bool(self.get_parameter("fake").value)
        channel = str(self.get_parameter("channel").value)
        min_rev = float(self.get_parameter("min_rev").value)
        friction_ff = float(self.get_parameter("friction_ff").value)
        friction_v_knee = float(self.get_parameter("friction_v_knee").value)
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
        extraction_enabled = bool(
            self.get_parameter("extraction_enabled").value
        )
        self._authority_enabled = bool(
            self.get_parameter("authority_enabled").value
        )
        self._assist_enabled = bool(
            self.get_parameter("assist_enabled").value
        ) and self._authority_enabled
        self._profile_max_speed_m_s = v_max
        self._contract_v2_verified = bool(
            self.get_parameter("contract_v2_verified").value
        )
        self._boot_qualification_enabled = bool(
            self.get_parameter("boot_qualification_enabled").value
        )
        self._arm_gate_mode = validate_arm_gate_mode(
            self.get_parameter("arm_gate_mode").value
        )
        self._arm_override_ttl_s = float(
            self.get_parameter("arm_override_ttl_s").value
        )
        if (
            not math.isfinite(self._arm_override_ttl_s)
            or self._arm_override_ttl_s <= 0.0
        ):
            raise ValueError("arm_override_ttl_s must be finite and positive")
        self._mission_contract_owner = validate_mission_contract_owner(
            self.get_parameter("mission_contract_owner").value
        )
        self._mission_supervisor_enabled = self._contract_v2_verified
        if (
            self._mission_supervisor_enabled
            and self._mission_contract_owner != MISSION_OWNER_CHASSIS
        ):
            raise ValueError(
                "contract_v2_verified=true requires "
                "mission_contract_owner=chassis_supervisor"
            )

        from chassis.chassis_manager import (
            ChassisConfig,
            ChassisManager,
            DEFAULT_WHEEL_MAP,
            build_real_corners,
        )

        four_wheel = bool(self.get_parameter("four_wheel").value)
        cfg = ChassisConfig(
            watchdog_ms=self._cmd_timeout * 1000.0,
            min_drive_turns_per_s=min_rev,
            extraction_enabled=extraction_enabled,
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
            corners = build_real_corners(
                channel, wheel_map=wheel_map,
                friction_ff=friction_ff, v_knee_turns_s=friction_v_knee,
            )
            self.get_logger().info("Real chassis on %s" % channel)

        owner_snapshot = (
            None
            if self._can_session is None
            else self._can_session.owner_snapshot
        )
        self.cm = ChassisManager(
            corners,
            cfg,
            wheel_map=wheel_map,
            can_owner_snapshot=owner_snapshot,
            qualification_gate=self._build_boot_qualification_gate(
                corners,
                fake=fake,
                wheel_map=wheel_map or DEFAULT_WHEEL_MAP,
            ),
        )
        self.cm.connect()
        # ○ E-stop 전역 latch(스펙 r6 §2.1): authority 와 무관한 고정 안전 계약.
        # TRANSIENT_LOCAL — 이 노드가 발행 후 재시작해도 latch 이벤트를 놓치지 않는다.
        self._teleop_estop_seen = collections.OrderedDict()
        teleop_estop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            String,
            "/teleop/estop",
            self._on_teleop_estop,
            teleop_estop_qos,
        )
        if self._can_session is not None:
            from chassis.telemetry import CanBusStatsSampler

            self._can_bus_sampler = CanBusStatsSampler(channel)
            self._can_bus_sampler.start()
        from powertrain_observability.client import EventClient

        self._observability_event_client = EventClient()
        self._last_can_health_event_ns = 0
        self._can_health_event_period_ns = 1_000_000_000
        self._last_arm_result_event_ns = 0
        self._last_arm_result_event_key = None
        self._arm_result_event_period_ns = 1_000_000_000
        self._assist_config = remote_assist.AssistConfig()
        # TODO(WP7/WP8): lead-distance cap and work-point alignment belong to
        # their qualified target contracts, not this terrain-only first part.
        self._assist_correction = None
        self._assist_bypass_active = False
        self._assist_bypass_stamp_s = None
        self._last_assist_parse_warning_ns = 0
        self._assist_parse_warning_period_ns = 1_000_000_000
        self._last_remote_assist_event_ns = 0
        self._last_remote_assist_event_state = None
        self._remote_assist_event_period_ns = 1_000_000_000
        self._arm_interlock = ArmInterlock(timeout_s=0.5)
        # arm_absent_field의 mock은 실제 sample/latch와 분리한다. publisher가
        # 나타나는 tick부터 이 객체를 선택하지 않아 즉시 real default-deny로 복귀한다.
        self._arm_absent_interlock = ArmInterlock(timeout_s=0.5)
        self._arm_override_requested = False
        self._arm_override_activated_s = None
        self._arm_override_expired = False
        # TODO(WP5.2 Task 7/remote gate): require a hold-to-run deadman and
        # independent joint proof before production remote-arm enablement.
        self._chassis_mode_intent = contract.MODE_STOW_REQUEST
        self._last_arm_status = None
        self._last_arm_posture = ""

        self._authority = None
        self._wheel_stop = None
        self._authority_final_v = 0.0
        self._authority_final_omega = 0.0
        self.pub_authority_state = None
        if self._authority_enabled:
            from chassis.authority import (
                AUTO,
                AUTO_SOURCE,
                AuthorityConfig,
                IDLE,
                MANUAL,
                MANUAL_SOURCE,
                CommandAuthority,
            )
            from powertrain_ros.wheel_stop import (
                WheelStopPredicate,
                load_wheel_stop_config,
            )

            self.declare_parameter(
                "wheel_stop_config",
                default_wheel_stop_config_path(),
            )
            self.declare_parameter("authority_handover_timeout_s", 2.0)
            wheel_stop_config = load_wheel_stop_config(
                str(self.get_parameter("wheel_stop_config").value)
            )
            self._wheel_stop = WheelStopPredicate(wheel_stop_config)

            self._authority = CommandAuthority(
                AuthorityConfig(
                    handover_timeout_s=float(
                        self.get_parameter(
                            "authority_handover_timeout_s"
                        ).value
                    )
                ),
                wheel_stopped=lambda: self._wheel_stop.confirmed,
                wheel_stop_qualified=lambda: self._wheel_stop.qualified,
            )
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
            self.create_subscription(
                String,
                "/autonomy/assist_correction",
                self._on_assist_correction,
                10,
            )
            self.create_subscription(
                Bool,
                "/teleop/assist_bypass",
                self._on_assist_bypass,
                10,
            )
            self.create_subscription(
                WheelStates,
                "/wheel_states",
                self._on_wheel_states_for_stop,
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
            self.create_service(
                Trigger,
                "~/authority_clear_hold",
                self._clear_authority_hold,
            )
        else:
            # Deprecated: WP5.2 Task 4에서 authority 내장 경로로 대체 예정.
            self.create_subscription(
                Twist,
                "/cmd_vel",
                self._on_cmd_vel,
                10,
            )

        self._mission_supervisor = None
        if self._mission_supervisor_enabled:
            from chassis.mission import MissionSupervisor
            from chassis.mission_id_store import MissionIdStore

            self._mission_supervisor = MissionSupervisor(
                MissionIdStore(
                    str(self.get_parameter("mission_id_path").value)
                ),
                wheel_stop=self._wheel_stop,
                authority_output_zero=lambda: (
                    self._authority_final_v == 0.0
                    and self._authority_final_omega == 0.0
                ),
                clear_grip_lost=self._arm_interlock.clear_grip_lost,
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
        # contract-v2에서는 이 노드가 순수 MissionSupervisor도 직접 소유하므로
        # superseded mission_node의 v1 mode 요청을 아예 구독하지 않는다.
        if not self._mission_supervisor_enabled:
            self.create_subscription(
                String,
                "/mission/chassis_mode",
                self._on_mission_mode,
                10,
            )
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
        arrival_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        if (
            self._mission_supervisor_enabled
            and self._arm_gate_mode != ARM_GATE_ABSENT_FIELD
        ):
            self.pub_arrival = self.create_publisher(
                ArrivalStatus,
                contract.TOPIC_ARRIVAL,
                arrival_qos,
            )
        else:
            self.pub_arrival = None
        self.pub_state = self.create_publisher(
            String,
            "/chassis_state",
            10,
        )
        self.pub_safety_state = self.create_publisher(
            String,
            "/chassis/safety_state",
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
            Trigger,
            "~/extraction_grant",
            self._srv_extraction_grant,
        )
        self.create_service(
            SetBool,
            "~/arm_lock_override",
            self._srv_arm_lock_override,
        )
        if self._mission_supervisor_enabled:
            self.create_service(
                Trigger,
                "~/mission_arrive_pickup",
                lambda _request, response: self._request_supervisor_work(
                    contract.ARRIVED_PICKUP,
                    response,
                ),
            )
            self.create_service(
                Trigger,
                "~/mission_arrive_drop",
                lambda _request, response: self._request_supervisor_work(
                    contract.ARRIVED_DROP,
                    response,
                ),
            )
            self.create_service(
                Trigger,
                "~/mission_skip",
                lambda _request, response: self._resolve_mission_failure(
                    "skip",
                    response,
                ),
            )
            self.create_service(
                Trigger,
                "~/mission_retry",
                lambda _request, response: self._resolve_mission_failure(
                    "retry",
                    response,
                ),
            )
            self.create_service(
                Trigger,
                "~/mission_regrasp_confirmed",
                self._srv_mission_regrasp,
            )
            self.create_service(
                SetBool,
                "~/mission_clear_grip_lost",
                self._srv_mission_clear_grip_lost,
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
        self._last_safety_state_publish_s = None
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

    def _build_boot_qualification_gate(self, corners, *, fake, wheel_map):
        if not self._boot_qualification_enabled:
            return None

        from chassis.boot_qualification import (
            AxisReport,
            QualificationResult,
            qualify_axes,
        )

        if fake:
            def qualify_fake():
                return QualificationResult(True, (), True)

            return qualify_fake

        fallback_node_ids = {
            wheel.wheel: int(wheel.drive_node_id)
            for wheel in wheel_map
        }
        report_fields = (
            "node_id",
            "pre_calibrated",
            "is_calibrated",
            "encoder_ready",
            "axis_error",
            "board_serial",
            "config_crc",
        )

        def qualify_current_axes():
            reports = []
            collection_failures = []
            voltage_samples = []

            for wheel_name, corner in corners.items():
                axis_id = fallback_node_ids[wheel_name]
                try:
                    state = corner.drive.state()
                except Exception as exc:
                    collection_failures.append(
                        (axis_id, f"state_error={type(exc).__name__}")
                    )
                    continue

                if not isinstance(state, dict):
                    collection_failures.append((axis_id, "state_not_mapping"))
                    continue

                reasons = []
                missing = [
                    field
                    for field in report_fields
                    if field not in state or state[field] is None
                ]
                if missing:
                    reasons.append("missing=" + ",".join(missing))

                if state.get("stale") is not False:
                    reasons.append(
                        "stale=missing" if "stale" not in state else "stale=true"
                    )

                voltage = state.get("voltage_v")
                try:
                    voltage = float(voltage)
                except (TypeError, ValueError):
                    voltage = float("nan")
                if math.isfinite(voltage):
                    voltage_samples.append(voltage)
                else:
                    reasons.append("voltage_v=missing_or_nonfinite")

                report = None
                if not missing:
                    try:
                        if not all(
                            isinstance(state[field], bool)
                            for field in (
                                "pre_calibrated",
                                "is_calibrated",
                                "encoder_ready",
                            )
                        ):
                            raise ValueError("qualification flag is not bool")
                        node_id = state["node_id"]
                        if isinstance(node_id, bool) or not isinstance(node_id, int):
                            raise ValueError("node_id is not int")
                        if node_id != axis_id:
                            raise ValueError(
                                f"node_id mismatch: expected {axis_id}, "
                                f"got {node_id}"
                            )
                        axis_error = int(state["axis_error"])
                        board_serial = str(state["board_serial"])
                        config_crc = str(state["config_crc"])
                        if not board_serial or not config_crc:
                            raise ValueError("empty fingerprint field")
                        report = AxisReport(
                            node_id=node_id,
                            pre_calibrated=state["pre_calibrated"],
                            is_calibrated=state["is_calibrated"],
                            encoder_ready=state["encoder_ready"],
                            axis_error=axis_error,
                            board_serial=board_serial,
                            config_crc=config_crc,
                        )
                    except (TypeError, ValueError) as exc:
                        reasons.append(f"invalid_report={exc}")

                if report is not None:
                    reports.append(report)
                    axis_id = report.node_id
                if reasons:
                    collection_failures.append((axis_id, ";".join(reasons)))

            voltage_v = (
                min(voltage_samples)
                if len(voltage_samples) == len(corners)
                else float("-inf")
            )
            result = qualify_axes(
                reports,
                expected_fingerprints={},
                voltage_v=voltage_v,
            )
            failures = tuple(collection_failures) + result.disqualified_axes
            return QualificationResult(
                qualified=result.qualified and not collection_failures,
                disqualified_axes=failures,
                voltage_ok=result.voltage_ok,
            )

        return qualify_current_axes

    def _now_ms(self):
        return self.get_clock().now().nanoseconds / 1e6

    def _now_s(self):
        """The only clock domain used for every ArmInterlock call."""
        return self._now_ms() / 1000.0

    def _remote_owner_selected(self):
        return (
            self._authority_enabled
            and self._authority is not None
            # CommandAuthority normalizes to TELEOP.  MANUAL remains accepted
            # only at this adapter boundary for older injected/test doubles.
            and self._authority.mode in ("TELEOP", "MANUAL")
        )

    def _arm_gate_decision(self, now_s):
        """Return ``(drive_allowed, detail)`` for the manager final gate."""
        if self._arm_override_requested:
            self._chassis_mode_intent = contract.MODE_STOW_REQUEST
            activated_s = self._arm_override_activated_s
            if (
                self._arm_override_expired
                or activated_s is None
                or now_s - activated_s > self._arm_override_ttl_s
            ):
                self._arm_override_expired = True
                return False, "operator_override_expired"
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

    def _set_mission_motion_hold(self, active, detail=""):
        """Use a distinct interlock source so arm-gate clears cannot erase it."""
        active = bool(active)
        if active:
            self._authority_final_v = 0.0
            self._authority_final_omega = 0.0
        # ChassisManager.set_motion_hold routes through the command_recovery
        # hold so a pre-hold command is never replayed when mission clears.
        self.cm.set_motion_hold("mission", active, detail)

    def _apply_mission_result(self, result):
        if not self._mission_supervisor_enabled:
            return False

        if (
            self._arm_gate_mode == "arm_absent_field"
            and result.state in ("READY", "DRIVE", "RESUME", "COMPLETE")
            and result.publish_arrival is None
        ):
            self.set_chassis_mode_intent(contract.MODE_STOW_REQUEST)
            self._set_mission_motion_hold(False, "")
            return False

        if (
            self._arm_gate_mode == "arm_absent_field"
            and (
                result.mode_intent == contract.MODE_MISSION_STOP
                or result.publish_arrival is not None
            )
        ):
            self.set_chassis_mode_intent(contract.MODE_STOW_REQUEST)
            self._set_mission_motion_hold(
                True,
                "arm_absent_field_blocks_mission_work",
            )
            self.get_logger().warning(
                "arm_absent_field blocked MISSION_STOP/ArrivalStatus"
            )
            return False

        mode_applied = self.set_chassis_mode_intent(result.mode_intent)
        detail = (
            result.hold_reason
            or result.operator_notice
            or "mission_state:%s" % result.state
        )
        self._set_mission_motion_hold(not result.allow_drive, detail)

        arrival_published = True
        if result.publish_arrival is not None:
            mission_id, status = result.publish_arrival
            arrival_published = self.publish_arrival(mission_id, status)
        return bool(mode_applied and arrival_published)

    def _mission_supervisor_failure(self, detail):
        if self._mission_supervisor is not None:
            try:
                self._mission_supervisor.abort_for_override(self._now_s())
            except Exception:
                pass
        self.set_chassis_mode_intent(contract.MODE_STOW_REQUEST)
        self._set_mission_motion_hold(True, detail)
        self.get_logger().error("mission supervisor held: %s" % detail)
        return False

    def _tick_mission_supervisor(self, now_s):
        if not self._mission_supervisor_enabled or self._mission_supervisor is None:
            return False
        try:
            result = self._mission_supervisor.tick(now_s)
            return self._apply_mission_result(result)
        except Exception as exc:
            return self._mission_supervisor_failure(
                "mission_tick_exception:%s" % exc
            )

    def _request_supervisor_work(self, arrival_status, response):
        if (
            not self._mission_supervisor_enabled
            or self._mission_supervisor is None
            or self._arm_gate_mode == ARM_GATE_ABSENT_FIELD
        ):
            response.success = False
            response.message = "mission work disabled by compatibility/arm_absent_field"
            return response
        try:
            result = self._mission_supervisor.request_work(
                arrival_status,
                self._now_s(),
            )
            self._apply_mission_result(result)
        except Exception as exc:
            self._mission_supervisor_failure(
                "mission_request_exception:%s" % exc
            )
            response.success = False
            response.message = "mission request held"
            return response
        response.success = result.accepted
        response.message = "%s|%s" % (result.state, result.hold_reason)
        return response

    def _resolve_mission_failure(self, action, response):
        if not self._mission_supervisor_enabled or self._mission_supervisor is None:
            response.success = False
            response.message = "mission supervisor disabled"
            return response
        try:
            result = self._mission_supervisor.resolve_failure(
                action,
                self._now_s(),
            )
            self._apply_mission_result(result)
        except Exception as exc:
            self._mission_supervisor_failure(
                "mission_resolution_exception:%s" % exc
            )
            response.success = False
            response.message = "mission resolution held"
            return response
        response.success = result.accepted
        response.message = "%s|%s" % (result.state, result.hold_reason)
        return response

    def _srv_mission_regrasp(self, _request, response):
        try:
            result = self._mission_supervisor.confirm_regrasp(self._now_s())
            self._apply_mission_result(result)
        except Exception as exc:
            self._mission_supervisor_failure(
                "mission_regrasp_exception:%s" % exc
            )
            response.success = False
            response.message = "regrasp confirmation held"
            return response
        response.success = result.accepted
        response.message = "%s|%s" % (result.state, result.hold_reason)
        return response

    def _srv_mission_clear_grip_lost(self, request, response):
        try:
            result = self._mission_supervisor.operator_clear_grip_lost(
                bool(request.data),
                self._now_s(),
            )
            self._apply_mission_result(result)
        except Exception as exc:
            self._mission_supervisor_failure(
                "mission_grip_clear_exception:%s" % exc
            )
            response.success = False
            response.message = "grip-lost clear held"
            return response
        response.success = result.accepted
        response.message = "%s|%s" % (result.state, result.hold_reason)
        return response

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

        return ""

    def _srv_arm_lock_override(self, request, response):
        if not request.data:
            self._arm_override_requested = False
            self._arm_override_activated_s = None
            self._arm_override_expired = False
            response.success = True
            response.message = "arm lock override disabled"
            return response

        now_s = self._now_s()
        error = self._override_activation_error(now_s)
        if error:
            response.success = False
            response.message = "override rejected: %s" % error
            return response

        if getattr(self, "_mission_supervisor_enabled", False):
            try:
                aborted = self._mission_supervisor.abort_for_override(now_s)
            except Exception as exc:
                self._mission_supervisor_failure(
                    "mission_override_abort_exception:%s" % exc
                )
                response.success = False
                response.message = "override rejected: mission abort held"
                return response
            if (
                not aborted
                or self._mission_supervisor.arrival_republish_active
            ):
                response.success = False
                response.message = "override rejected: mission abort not acknowledged"
                return response
        self.set_chassis_mode_intent(contract.MODE_STOW_REQUEST)
        self._discard_pending_command()
        self._arm_override_requested = True
        self._arm_override_activated_s = now_s
        self._arm_override_expired = False
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

    def _on_assist_correction(self, msg: String):
        try:
            payload = json.loads(msg.data)
            fields = {
                "stamp_s",
                "omega_correction_rad_s",
                "speed_cap_m_s",
                "confidence",
            }
            if not isinstance(payload, dict) or set(payload) != fields:
                raise ValueError("assist correction fields do not match contract")
            if any(isinstance(payload[name], bool) for name in fields):
                raise ValueError("assist correction values must be numeric")
            correction = remote_assist.AssistCorrection(
                stamp_s=float(payload["stamp_s"]),
                omega_correction_rad_s=float(
                    payload["omega_correction_rad_s"]
                ),
                speed_cap_m_s=float(payload["speed_cap_m_s"]),
                confidence=float(payload["confidence"]),
            )
        except (
            json.JSONDecodeError,
            KeyError,
            OverflowError,
            TypeError,
            ValueError,
        ) as exc:
            now_ns = time.monotonic_ns()
            previous_ns = getattr(self, "_last_assist_parse_warning_ns", 0)
            period_ns = getattr(
                self,
                "_assist_parse_warning_period_ns",
                1_000_000_000,
            )
            if not previous_ns or now_ns - previous_ns >= period_ns:
                self._last_assist_parse_warning_ns = now_ns
                self.get_logger().warning(
                    "malformed /autonomy/assist_correction ignored: %s" % exc
                )
            return False
        self._assist_correction = correction
        return True

    def _on_assist_bypass(self, msg: Bool):
        self._assist_bypass_active = bool(msg.data)
        self._assist_bypass_stamp_s = self._now_s()

    def _on_wheel_states_for_stop(self, msg: WheelStates):
        from powertrain_ros.wheel_stop import (
            WheelStopSample,
            WheelStopWheel,
        )

        stamp_s = (
            float(msg.header.stamp.sec)
            + float(msg.header.stamp.nanosec) * 1e-9
        )
        sample = WheelStopSample(
            stamp_s=stamp_s,
            healthy=bool(msg.healthy),
            wheels=tuple(
                WheelStopWheel(
                    name=str(wheel.name),
                    drive_turns_per_s=float(wheel.drive_turns_per_s),
                    drive_stale=bool(wheel.drive_stale),
                    steer_stale=bool(wheel.steer_stale),
                    drive_axis_error=int(wheel.drive_axis_error),
                    steer_fault=int(wheel.steer_fault),
                )
                for wheel in msg.wheels
            ),
            authority_v=self._authority_final_v,
            authority_omega=self._authority_final_omega,
        )
        self._wheel_stop.update(sample, now_s=self._now_s())

    def _set_authority_mode(self, mode, response):
        result = self._authority.request_mode(mode, t=self._now_s())
        response.success = result.accepted
        response.message = "%s; state=%s" % (result.reason, result.state)
        if response.success:
            self.get_logger().warning(
                "authority request %s accepted: %s"
                % (mode, response.message)
            )
        return response

    def _clear_authority_hold(self, _request, response):
        response.success = self._authority.clear_hold()
        response.message = self._authority.last_transition_reason
        return response

    def _tick_authority(self, now_s):
        command = self._authority.select(now_s)
        final_v = command.v
        final_omega = command.omega
        assist_suffix = ""
        if (
            getattr(self, "_assist_enabled", False)
            and self._authority.mode == "TELEOP"
        ):
            assist_suffix = "|assist=off"
            if command.ok:
                result = remote_assist.compose(
                    command.v,
                    command.omega,
                    now_s=now_s,
                    correction=self._assist_correction,
                    bypass_active=self._assist_bypass_active,
                    bypass_stamp_s=self._assist_bypass_stamp_s,
                    enabled=True,
                    config=self._assist_config,
                    profile_max_speed_m_s=self._profile_max_speed_m_s,
                )
                final_v = result.v_m_s
                final_omega = result.omega_rad_s
                assist_suffix = (
                    "|assist=on" if result.applied else "|assist=off"
                )
                assist_state = (
                    "engaged"
                    if result.applied
                    else (
                        "bypassed"
                        if "assist_bypass" in result.reasons
                        else "degraded"
                    )
                )
                self._emit_remote_assist_event(assist_state, result)
        self.pub_authority_state.publish(
            String(
                data="%s|%s%s"
                % (self._authority.mode, command.reason, assist_suffix)
            )
        )
        if command.ok:
            self._authority_final_v = final_v
            self._authority_final_omega = final_omega
            self.cm.set(final_v, final_omega)

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
        now_ms = self._now_ms()
        if (
            self._last_safety_ms is not None
            and now_ms - self._last_safety_ms > 500.0
        ):
            self.get_logger().warning(
                "safety verdict rx gap: age_ms=%.0f"
                % (now_ms - self._last_safety_ms)
            )
        self._last_safety_ms = now_ms

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
                if stale and not getattr(
                    self, "_safety_stale_active", False
                ):
                    self.get_logger().error(
                        "safety topic stale latch: age_ms=%.0f "
                        "(timeout %.0f ms)"
                        % (
                            now_ms - self._last_safety_ms,
                            self._safety_topic_timeout * 1000.0,
                        )
                    )
                self._safety_stale_active = stale
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

        if getattr(self, "_mission_supervisor_enabled", False):
            self._tick_mission_supervisor(now_s)

        bus_sampler = getattr(self, "_can_bus_sampler", None)
        if bus_sampler is not None:
            self.cm.set_can_bus_health(bus_sampler.snapshot())

        started = time.monotonic()
        try:
            self.cm.tick()
        except Exception as exc:
            self.cm.estop("control_exception", str(exc))
        duration_ms = (time.monotonic() - started) * 1000.0
        if duration_ms > 1000.0 / self.cm.cfg.loop_hz:
            self._overrun_count += 1
        snapshot = None
        try:
            snapshot = self.cm.snapshot()
            msg = WheelStates()
            fill_wheel_states_message(
                msg,
                snapshot,
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
        if snapshot is not None:
            emit_can_health = getattr(self, "_emit_can_health_event", None)
            if emit_can_health is not None:
                emit_can_health(snapshot)
        publish_safety_state = getattr(self, "_publish_safety_state", None)
        if publish_safety_state is not None:
            publish_safety_state(time.monotonic())

    def _publish_safety_state(self, now_s):
        last_publish_s = self._last_safety_state_publish_s
        if (
            last_publish_s is not None
            and now_s >= last_publish_s
            and now_s - last_publish_s + 1e-12
            < SAFETY_STATE_PUBLISH_PERIOD_S
        ):
            return
        safety = self.cm.safety_snapshot()
        message = String()
        message.data = json.dumps(
            {
                "mode": self.cm.mode,
                "estop_latched": bool(safety.estop_latched),
                "active_estop_sources": list(safety.active_estop_sources),
                "stamp_s": time.monotonic(),
            },
            separators=(",", ":"),
        )
        self.pub_safety_state.publish(message)
        self._last_safety_state_publish_s = now_s

    def _emit_can_health_event(self, snapshot):
        """Best-effort datagram emission; failure never escapes the tick."""
        client = getattr(self, "_observability_event_client", None)
        if client is None:
            return False
        now_ns = time.monotonic_ns()
        previous_ns = getattr(self, "_last_can_health_event_ns", 0)
        period_ns = getattr(self, "_can_health_event_period_ns", 1_000_000_000)
        if previous_ns and now_ns - previous_ns < period_ns:
            return False
        self._last_can_health_event_ns = now_ns
        try:
            from chassis.telemetry import build_can_health_event

            return bool(client.emit(build_can_health_event(
                snapshot,
                monotonic_ns=now_ns,
            )))
        except Exception:
            return False

    def _emit_remote_assist_event(self, state, result):
        """Emit assist state transitions at most once per second."""
        if state == getattr(self, "_last_remote_assist_event_state", None):
            return False
        now_ns = time.monotonic_ns()
        previous_ns = getattr(self, "_last_remote_assist_event_ns", 0)
        period_ns = getattr(
            self,
            "_remote_assist_event_period_ns",
            1_000_000_000,
        )
        if previous_ns and now_ns - previous_ns < period_ns:
            return False
        self._last_remote_assist_event_ns = now_ns
        self._last_remote_assist_event_state = state
        client = getattr(self, "_observability_event_client", None)
        if client is None:
            return False
        try:
            return bool(
                client.emit(
                    {
                        "schema_version": 1,
                        "wall_time_ns": time.time_ns(),
                        "monotonic_ns": now_ns,
                        "source": "chassis_node",
                        "event_type": "REMOTE_ASSIST",
                        "severity": (
                            "WARN" if state == "degraded" else "INFO"
                        ),
                        "payload": {
                            "state": state,
                            "applied": bool(result.applied),
                            "reasons": list(result.reasons),
                            "v_m_s": float(result.v_m_s),
                            "omega_rad_s": float(result.omega_rad_s),
                        },
                    }
                )
            )
        except Exception:
            return False

    def _emit_arm_result_event(self, msg, accepted, result=None):
        """Best-effort adapter emission after WP5.2 has made its decision."""
        try:
            from powertrain_observability.arm_adapter import (
                ArmObservation,
                POSTURE_STATUSES,
                build_arm_events,
            )

            raw_status = str(msg.status)
            posture = str(getattr(self, "_last_arm_posture", "") or "")
            if accepted and raw_status in POSTURE_STATUSES:
                posture = raw_status
                self._last_arm_posture = raw_status

            supervisor = getattr(self, "_mission_supervisor", None)
            failure = (
                None
                if supervisor is None
                else getattr(supervisor, "failure", None)
            )
            failure_posture = getattr(failure, "last_locked_posture", "")
            if failure_posture and raw_status not in POSTURE_STATUSES:
                posture = str(failure_posture)

            current_mission_id = (
                None
                if supervisor is None
                else getattr(supervisor, "active_mission_id", None)
            )
            if current_mission_id is None:
                current_mission_id = getattr(failure, "mission_id", None)
            if current_mission_id is None:
                current_mission_id = int(msg.mission_id)

            hold_reason = str(getattr(result, "hold_reason", "") or "")
            contract_violation = False
            if not accepted and not hold_reason:
                interlock = getattr(self, "_arm_interlock", None)
                violation = getattr(
                    interlock,
                    "last_contract_violation",
                    None,
                )
                violation_text = "" if violation is None else str(violation)
                contract_violation = (
                    violation_text == raw_status
                    or violation_text.startswith("stamp_domain:")
                )
                if contract_violation:
                    hold_reason = "arm_contract_violation:%s" % violation_text

            detail_parts = []
            state = getattr(result, "state", "")
            if state:
                detail_parts.append("state=%s" % state)
            elif contract_violation:
                detail_parts.append("state=CONTRACT_VIOLATION")
            operation = getattr(failure, "operation", "")
            if operation:
                detail_parts.append("operation=%s" % operation)
            notice = getattr(result, "operator_notice", "")
            if notice:
                detail_parts.append("notice=%s" % notice)

            observation = ArmObservation(
                raw_status=raw_status,
                source_mission_id=int(msg.mission_id),
                stamp_sec=int(msg.header.stamp.sec),
                stamp_nanosec=int(msg.header.stamp.nanosec),
                accepted=bool(accepted),
                contract_violation=contract_violation,
                current_mission_id=int(current_mission_id),
                arm_posture=posture,
                hold_reason=hold_reason,
                source_detail=";".join(detail_parts),
            )
            now_ns = time.monotonic_ns()
            events = build_arm_events(
                observation,
                monotonic_ns=now_ns,
            )
            if not events:
                return False

            event_key = (
                raw_status,
                int(current_mission_id),
                events[0]["event_type"],
            )
            previous_key = getattr(self, "_last_arm_result_event_key", None)
            previous_ns = getattr(self, "_last_arm_result_event_ns", 0)
            period_ns = getattr(
                self,
                "_arm_result_event_period_ns",
                1_000_000_000,
            )
            if (
                event_key == previous_key
                and previous_ns
                and now_ns - previous_ns < period_ns
            ):
                return False

            client = getattr(self, "_observability_event_client", None)
            if client is None:
                return False
            self._last_arm_result_event_key = event_key
            self._last_arm_result_event_ns = now_ns
            emitted = False
            for event in events:
                try:
                    emitted = bool(client.emit(event)) or emitted
                except Exception:
                    pass
            return emitted
        except Exception:
            return False

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
        if hasattr(self.cm, "set_imu_yaw_rate"):
            self.cm.set_imu_yaw_rate(msg.angular_velocity.z)

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
        """Publish ``<mode> v=<m/s> w=<rad/s>`` as String diagnostics."""
        try:
            state = self.cm.state()
            msg = String(
                data="%s v=%.2f w=%.2f" % (
                    state["mode"],
                    state["v"],
                    state["omega"],
                )
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
        stamp_s = self._arm_status_stamp_s(msg)
        accepted = self._arm_interlock.update(
            msg.status,
            msg.mission_id,
            stamp_s,
            now_s,
        )
        emit_arm_result = getattr(self, "_emit_arm_result_event", None)
        if not accepted:
            if emit_arm_result is not None:
                try:
                    emit_arm_result(msg, False, None)
                except Exception:
                    pass
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
        result = None
        if getattr(self, "_mission_supervisor_enabled", False):
            try:
                result = self._mission_supervisor.on_arm_status(
                    msg.status,
                    msg.mission_id,
                    stamp_s,
                    now_s,
                )
                self._apply_mission_result(result)
            except Exception as exc:
                self._mission_supervisor_failure(
                    "mission_arm_status_exception:%s" % exc
                )
                if emit_arm_result is not None:
                    try:
                        emit_arm_result(msg, True, None)
                    except Exception:
                        pass
                return
        if emit_arm_result is not None:
            try:
                emit_arm_result(msg, True, result)
            except Exception:
                pass

    def _refresh_safety_baseline(self):
        """executor를 블로킹하는 cm 서비스(arm/disarm/estop/reset) 직후 호출.

        블로킹 동안 verdict 콜백이 큐에 밀려 freshness가 거짓 stale로
        래치된다(2026-07-16 실기: arm 후 age 783ms, disarm 후 age 1264ms).
        기준선을 서비스 종료 시점으로 당긴다 — 링크가 실제로 죽었다면
        다음 tick부터 0.75s 뒤 정상 래치된다. 최초 verdict 미수신(None)
        상태는 startup timeout 의미를 지키기 위해 그대로 둔다.
        """
        if getattr(self, "_last_safety_ms", None) is not None:
            self._last_safety_ms = self._now_ms()

    def _srv_arm(self, _request, response):
        response.success = self.cm.arm()
        self._refresh_safety_baseline()
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
        self._refresh_safety_baseline()
        response.success = True
        response.message = "mode=%s" % self.cm.mode
        return response

    def _on_teleop_estop(self, msg):
        try:
            payload = json.loads(msg.data)
            event_id = str(payload["event_id"])
            stamp_s = float(payload["stamp_s"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().error("invalid /teleop/estop payload; ignored")
            return
        if event_id in self._teleop_estop_seen:
            return
        self._teleop_estop_seen[event_id] = stamp_s
        while len(self._teleop_estop_seen) > 32:
            self._teleop_estop_seen.popitem(last=False)
        # 물리 정지까지 포함한 정본 진입점(cm.estop) — raw interlock trip 금지
        # (다음 50 Hz 틱까지 모터가 돈다). 스펙 r6 §2.1.
        self.cm.estop(
            "remote_operator",
            "teleop circle edge event_id=%s" % event_id,
        )
        self.get_logger().error(
            "REMOTE E-STOP latched (event_id=%s)" % event_id
        )

    def _srv_estop(self, _request, response):
        self.cm.estop("manual_service", "~/estop")
        self._refresh_safety_baseline()
        response.success = True
        response.message = "estop latched: mode=%s" % self.cm.mode
        self.get_logger().warning("manual E-stop latched")
        return response

    def _srv_reset_estop(self, _request, response):
        response.success = self.cm.reset_estop()
        self._refresh_safety_baseline()
        if response.success:
            response.message = "mode=IDLE; explicit arm required"
        else:
            safety = self.cm.state()["safety"]
            response.message = (
                "reset rejected: mode=%s active=%s"
                % (self.cm.mode, list(safety.active_estop_sources))
            )
        return response

    def _srv_extraction_grant(self, _request, response):
        response.success = self.cm.extraction_grant()
        snapshot = self.cm.snapshot()
        if response.success:
            response.message = "mode=%s" % snapshot.chassis_mode
        else:
            response.message = "extraction rejected: %s" % (
                snapshot.last_extraction_reject
            )
        return response

    def close(self):
        can_bus_sampler = getattr(self, "_can_bus_sampler", None)
        self._can_bus_sampler = None
        if can_bus_sampler is not None:
            can_bus_sampler.stop()
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
