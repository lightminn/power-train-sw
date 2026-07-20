"""One-process WP6-B terrain estimator and WP6-C controller ROS adapter.

This node only converts timestamped ROS messages to immutable autonomy-core
values.  Chassis command authority and final drive gating remain exclusively in
``chassis_node``.
"""
from __future__ import annotations

import json
import math
import os
import sys
import threading
import time

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

from robot_arm_msgs.msg import ArmStatus

from powertrain_ros import contract

sys.path.insert(0, os.environ.get("AUTONOMY_PATH", "/workspace"))
sys.path.insert(
    0,
    os.environ.get("MOTOR_CONTROL_PATH", "/workspace/motor_control"),
)

from powertrain_autonomy.controller import (  # noqa: E402
    AutonomyController,
    AutonomyControllerConfig,
    DriveDiagnostics,
    MotionState,
    ProfileGate,
    assist_correction_from_terrain,
    profile_by_name,
)
from powertrain_autonomy.degradation import (  # noqa: E402
    DegradationFsm,
    DegradationStage,
)
from powertrain_autonomy.terrain import (  # noqa: E402
    BaseToCameraExtrinsic,
    BodyTilt,
    OdometryDelta,
    TerrainEstimator,
    TerrainEstimatorConfig,
    TerrainFrame,
)
from powertrain_autonomy.terrain.depth_quality import (  # noqa: E402
    CameraIntrinsics,
    DepthQualityConfig,
    analyze_depth_quality,
)
from powertrain_observability.client import EventClient  # noqa: E402
from powertrain_ros.terrain_qualification import (  # noqa: E402
    load_approved_terrain_qualification,
)

# L515 depth 스케일 = 1/4000 m/unit. D400 계열의 0.001 이 **아니다**.
# Gateway 는 raw Z16 을 16UC1 로 그대로 발행하므로, 같은
# /l515/depth/image_rect_raw 를 읽는 l515_cloud_node.DEPTH_SCALE_M 과 반드시
# 같은 값이어야 한다. 틀리면 모든 지형 거리가 4배로 나온다.
# (두 값의 일치는 test_l515_depth_scale.py 가 강제한다.)
L515_DEPTH_SCALE_M = 0.00025


# TerrainEstimatorConfig.depth_shape_px 와 같은 값이어야 한다.
# 60x80 은 5 cm 격자를 전방 ~1.4 m 너머로 채우지 못해 support 성장이 끊기고,
# 로봇이 `drop_boundaries_unobserved` 로 영구 정지한다(시뮬 실측 완주율 0.00).
# 120x160 으로 완주율 0.83, fail_open 0 유지, 추정기 런타임 불변.
_TARGET_HEIGHT = 120
_TARGET_WIDTH = 160


def _stamp_s(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def _rpy(quaternion) -> tuple[float, float, float]:
    roll = math.atan2(
        2.0 * (
            quaternion.w * quaternion.x
            + quaternion.y * quaternion.z
        ),
        1.0 - 2.0 * (
            quaternion.x * quaternion.x
            + quaternion.y * quaternion.y
        ),
    )
    pitch = math.asin(
        max(
            -1.0,
            min(
                1.0,
                2.0 * (
                    quaternion.w * quaternion.y
                    - quaternion.z * quaternion.x
                ),
            ),
        )
    )
    yaw = math.atan2(
        2.0 * (
            quaternion.w * quaternion.z
            + quaternion.x * quaternion.y
        ),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y
            + quaternion.z * quaternion.z
        ),
    )
    return roll, pitch, yaw


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


class AutonomyControllerNode(Node):
    def __init__(self, *, parameter_overrides=None, event_client=None):
        super().__init__(
            "autonomy_controller",
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter("enabled", False)
        self.declare_parameter("drive_profile", "EMPTY_STOWED")
        self.declare_parameter("tick_hz", 20.0)
        default_qualification_file = os.path.join(
            get_package_share_directory("powertrain_ros"),
            "config",
            "l515_terrain.yaml",
        )
        self.declare_parameter(
            "terrain_qualification_file",
            default_qualification_file,
        )
        self.declare_parameter("min_confidence", 0.25)
        # L515 는 1/4000 m/unit. D400 계열의 0.001 이 아니다 — 틀리면 모든 지형
        # 거리가 4배로 나온다. Gateway 는 raw Z16 을 16UC1 로 그대로 발행하므로
        # 이 노드와 l515_cloud_node 는 같은 /l515/depth/image_rect_raw 를 같은
        # 스케일로 읽어야 한다 (l515_cloud_node.DEPTH_SCALE_M 과 동일 값).
        self.declare_parameter("depth_scale_m", L515_DEPTH_SCALE_M)

        depth_scale_m = float(self.get_parameter("depth_scale_m").value)
        if not math.isfinite(depth_scale_m) or depth_scale_m <= 0.0:
            raise ValueError("depth_scale_m must be finite and positive")
        self._depth_scale_m = depth_scale_m

        tick_hz = float(self.get_parameter("tick_hz").value)
        if not math.isfinite(tick_hz) or tick_hz <= 0.0:
            raise ValueError("tick_hz must be finite and positive")
        profile = profile_by_name(
            str(self.get_parameter("drive_profile").value)
        )
        controller_config = AutonomyControllerConfig(
            min_confidence=float(
                self.get_parameter("min_confidence").value
            )
        )
        self.controller = AutonomyController(profile, controller_config)
        self.degradation = DegradationFsm(clock=self._now_s)
        self._event_client = (
            event_client if event_client is not None else EventClient()
        )
        self._enabled = bool(self.get_parameter("enabled").value)
        qualification_path = str(
            self.get_parameter("terrain_qualification_file").value
        )
        try:
            qualification = load_approved_terrain_qualification(
                qualification_path
            )
        except ValueError as exc:
            if self._enabled:
                raise ValueError(
                    "terrain command production rejected: %s" % exc
                ) from exc
            self._qualification_error = str(exc)
            self._qualification = None
            self._qualified_roi = None
            self._depth_quality_config = None
            self.estimator = None
            self.extrinsic = None
        else:
            self._qualification_error = None
            self._qualification = qualification
            self._qualified_roi = qualification.roi
            self._depth_quality_config = DepthQualityConfig(
                min_depth_m=qualification.min_depth_m,
                max_depth_m=qualification.max_depth_m,
                min_valid_ratio=qualification.min_valid_ratio,
            )
            self.estimator = TerrainEstimator(
                TerrainEstimatorConfig(
                    min_depth_m=qualification.min_depth_m,
                    max_depth_m=qualification.max_depth_m,
                )
            )
            self.extrinsic = BaseToCameraExtrinsic(
                x_m=qualification.translation_m[0],
                y_m=qualification.translation_m[1],
                z_m=qualification.translation_m[2],
                roll_rad=qualification.roll_rad,
                mount_pitch_rad=qualification.pitch_rad,
                pitch_down_rad=0.0,
                yaw_rad=qualification.yaw_rad,
            )

        self._grid_source_shape: tuple[int, int] | None = None
        self._row_indices: np.ndarray | None = None
        self._col_indices: np.ndarray | None = None
        self._intrinsics: CameraIntrinsics | None = None
        self._grid_snapshot = None
        self._tilt: BodyTilt | None = None
        self._imu_stamp_s: float | None = None
        self._odom = None
        self._motion_snapshot = (None, None, None)
        self._previous_depth_pose = None
        self._terrain = None
        self._terrain_seen = False
        self._terrain_snapshot = (None, False)
        self._terrain_update_count = 0
        self._terrain_state_lock = threading.Lock()
        self._depth_condition = threading.Condition()
        self._depth_slot: tuple[Image, float] | None = None
        self._depth_stop = False
        self._depth_overwrite_count = 0
        self._depth_worker_join_timeout_s = 1.0
        self._gate: ProfileGate | None = None
        self._diagnostics: DriveDiagnostics | None = None
        self._previous_odom_xy: tuple[float, float] | None = None
        self._traveled_m = 0.0
        self._depth_quality_snapshot: tuple[
            float | None,
            float | None,
        ] = (None, None)
        self._depth_quality_seen = False
        self._degradation_output = None
        self._last_degradation_stage = DegradationStage.NORMAL

        self.pub_cmd = self.create_publisher(
            Twist,
            "/autonomy/cmd_vel",
            10,
        )
        self.pub_controller_state = self.create_publisher(
            String,
            "/autonomy/controller_state",
            10,
        )
        self.pub_terrain_state = self.create_publisher(
            String,
            "/autonomy/terrain_state",
            10,
        )
        self.pub_assist_correction = self.create_publisher(
            String,
            "/autonomy/assist_correction",
            10,
        )
        self.pub_degradation_state = self.create_publisher(
            String,
            "/autonomy/degradation_state",
            10,
        )

        self.create_subscription(
            CameraInfo,
            "/l515/depth/camera_info",
            self._on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/l515/depth/image_rect_raw",
            self._on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Imu,
            "/imu/filtered",
            self._on_imu,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Odometry,
            "/odom",
            self._on_odom,
            10,
        )
        arm_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            ArmStatus,
            contract.TOPIC_ARM_STATUS,
            self._on_arm_status,
            arm_qos,
        )
        self.create_subscription(
            String,
            "/odom_diagnostics",
            self._on_diagnostics,
            10,
        )
        self.create_timer(1.0 / tick_hz, self._tick)
        self._degradation_timer = self.create_timer(
            1.0,
            self._publish_degradation_state,
        )
        self._depth_worker_thread = threading.Thread(
            target=self._depth_worker_loop,
            name="autonomy-depth-worker",
            daemon=True,
        )
        self._depth_worker_thread.start()

        mode = "ON" if self._enabled else "OFF (diagnostics only)"
        self.get_logger().info(
            "autonomy_controller profile=%s proposal=%s"
            % (profile.name, mode)
        )

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _depth_is_stopping(self) -> bool:
        with self._depth_condition:
            return self._depth_stop

    def _publish_terrain_unavailable(self, reason: str) -> bool:
        if self._depth_is_stopping():
            return False
        self.pub_terrain_state.publish(
            String(data="False|0.000000|0.000000|0.000000|" + reason)
        )
        return True

    def _on_camera_info(self, message: CameraInfo) -> None:
        qualified_roi = getattr(self, "_qualified_roi", None)
        if qualified_roi is None:
            return
        shape = (int(message.height), int(message.width))
        grid_snapshot = self._grid_snapshot
        if grid_snapshot is not None:
            if shape != grid_snapshot[0]:
                self.get_logger().warning(
                    "CameraInfo resolution changed %s -> %s; fixed grid retained"
                    % (grid_snapshot[0], shape)
                )
            return
        height, width = shape
        roi_x, roi_y, roi_width, roi_height = qualified_roi
        if roi_x + roi_width > width or roi_y + roi_height > height:
            self.get_logger().error(
                "qualified terrain ROI exceeds depth CameraInfo %dx%d"
                % (width, height)
            )
            return
        if roi_height < _TARGET_HEIGHT or roi_width < _TARGET_WIDTH:
            # stride 가 0 이 되면 모든 인덱스가 같은 픽셀을 가리켜 지형이 조용히
            # 망가진다. fail-closed 로 거부하고 자격화를 다시 받게 한다.
            self.get_logger().error(
                "qualified terrain ROI %dx%d is smaller than the %dx%d grid"
                % (roi_width, roi_height, _TARGET_WIDTH, _TARGET_HEIGHT)
            )
            return
        row_stride = roi_height // _TARGET_HEIGHT
        col_stride = roi_width // _TARGET_WIDTH
        try:
            intrinsics = CameraIntrinsics(
                fx=float(message.k[0]) / col_stride,
                fy=float(message.k[4]) / row_stride,
                cx=(float(message.k[2]) - roi_x) / col_stride,
                cy=(float(message.k[5]) - roi_y) / row_stride,
            )
        except (IndexError, TypeError, ValueError) as exc:
            self.get_logger().error("invalid depth CameraInfo: %s" % exc)
            return
        row_indices = roi_y + row_stride * np.arange(_TARGET_HEIGHT)
        col_indices = roi_x + col_stride * np.arange(_TARGET_WIDTH)
        self._grid_source_shape = shape
        self._row_indices = row_indices
        self._col_indices = col_indices
        self._intrinsics = intrinsics
        self._grid_snapshot = (
            shape,
            row_indices,
            col_indices,
            intrinsics,
        )

    def _on_imu(self, message: Imu) -> None:
        roll, pitch, _ = _rpy(message.orientation)
        tilt = BodyTilt(roll_rad=roll, pitch_rad=pitch)
        imu_stamp_s = _stamp_s(message.header.stamp)
        self._tilt = tilt
        self._imu_stamp_s = imu_stamp_s
        self._motion_snapshot = (self._odom, tilt, imu_stamp_s)

    def _on_odom(self, message: Odometry) -> None:
        _, _, yaw = _rpy(message.pose.pose.orientation)
        odom = (
            float(message.pose.pose.position.x),
            float(message.pose.pose.position.y),
            yaw,
            float(message.twist.twist.linear.x),
            float(message.twist.twist.angular.z),
            _stamp_s(message.header.stamp),
        )
        self._odom = odom
        current_xy = odom[:2]
        if all(math.isfinite(value) for value in current_xy):
            previous_xy = getattr(self, "_previous_odom_xy", None)
            if previous_xy is not None:
                self._traveled_m = getattr(self, "_traveled_m", 0.0) \
                    + math.hypot(
                        current_xy[0] - previous_xy[0],
                        current_xy[1] - previous_xy[1],
                    )
            self._previous_odom_xy = current_xy
        self._motion_snapshot = (odom, self._tilt, self._imu_stamp_s)

    def _on_arm_status(self, message: ArmStatus) -> None:
        self._gate = ProfileGate(
            stamp_s=_stamp_s(message.header.stamp),
            status=str(message.status),
        )

    def _on_diagnostics(self, message: String) -> None:
        try:
            payload = json.loads(message.data)
            raw_cap = payload["speed_cap_m_s"]
            speed_cap = math.inf if raw_cap is None else float(raw_cap)
            self._diagnostics = DriveDiagnostics(
                stamp_s=float(payload["stamp_s"]),
                slip_candidate=bool(payload["slip_candidate"]),
                stuck_candidate=bool(payload["stuck_candidate"]),
                speed_cap_m_s=speed_cap,
            )
        except (json.JSONDecodeError, KeyError, TypeError, ValueError) as exc:
            self.get_logger().warning(
                "invalid /odom_diagnostics ignored: %s" % exc
            )

    @staticmethod
    def _decode_depth(message: Image) -> np.ndarray:
        if message.encoding not in ("16UC1", "mono16"):
            raise ValueError("depth encoding must be 16UC1 or mono16")
        if message.step < message.width * 2 or message.step % 2:
            raise ValueError("depth step is inconsistent with uint16 width")
        dtype = np.dtype(">u2" if message.is_bigendian else "<u2")
        expected = int(message.height) * int(message.step)
        if len(message.data) < expected:
            raise ValueError("depth data is shorter than height*step")
        row_width = int(message.step) // 2
        array = np.frombuffer(message.data, dtype=dtype, count=expected // 2)
        array = array.reshape(int(message.height), row_width)
        return array[:, : int(message.width)].astype(np.uint16, copy=False)

    @staticmethod
    def _odometry_delta(odom, previous_depth_pose) -> OdometryDelta:
        # 기준 pose 전진은 estimator.update 성공 후에만 한다 — 실패 프레임의
        # 이동이 grid 이력 warp에서 누락되면 캐리 셀이 조용히 어긋난다.
        x_m, y_m, yaw_rad = odom[:3]
        if previous_depth_pose is None:
            return OdometryDelta(0.0, 0.0, 0.0)
        prev_x, prev_y, prev_yaw = previous_depth_pose
        global_x = x_m - prev_x
        global_y = y_m - prev_y
        cosine = math.cos(prev_yaw)
        sine = math.sin(prev_yaw)
        return OdometryDelta(
            dx_m=cosine * global_x + sine * global_y,
            dy_m=-sine * global_x + cosine * global_y,
            dyaw_rad=_wrap(yaw_rad - prev_yaw),
        )

    def _on_depth(self, message: Image) -> None:
        received_at_s = self._now_s()
        with self._depth_condition:
            if self._depth_stop:
                return
            if self._depth_slot is not None:
                self._depth_overwrite_count += 1
            self._depth_slot = (message, received_at_s)
            self._depth_condition.notify()

    def _depth_worker_loop(self) -> None:
        while True:
            with self._depth_condition:
                while self._depth_slot is None and not self._depth_stop:
                    self._depth_condition.wait()
                if self._depth_stop:
                    return
                message, received_at_s = self._depth_slot
                self._depth_slot = None
            self._process_depth_now(message, received_at_s)

    def _process_depth_now(
        self,
        message: Image,
        received_at_s: float | None = None,
    ) -> None:
        if received_at_s is None:
            received_at_s = self._now_s()
        if self._depth_is_stopping():
            return
        if self._qualification is None:
            self._publish_terrain_unavailable("qualification_unapproved")
            return
        grid_snapshot = self._grid_snapshot
        if grid_snapshot is None:
            self._publish_terrain_unavailable("waiting_camera_info")
            return
        grid_source_shape, row_indices, col_indices, intrinsics = grid_snapshot
        shape = (int(message.height), int(message.width))
        if shape != grid_source_shape:
            if not self._depth_is_stopping():
                self.get_logger().warning(
                    "depth resolution %s ignored; fixed source is %s"
                    % (shape, grid_source_shape)
                )
            return
        odom, tilt, _ = self._motion_snapshot
        if tilt is None or odom is None:
            self._publish_terrain_unavailable("waiting_motion")
            return
        try:
            raw = self._decode_depth(message)
        except ValueError as exc:
            self._depth_quality_snapshot = (None, received_at_s)
            self._depth_quality_seen = True
            with self._depth_condition:
                if self._depth_stop:
                    return
                with self._terrain_state_lock:
                    _, terrain_seen = self._terrain_snapshot
                    self._terrain = None
                    self._terrain_snapshot = (None, terrain_seen)
            if self._publish_terrain_unavailable("invalid_depth"):
                self.get_logger().warning("depth frame ignored: %s" % exc)
            return
        sampled = raw[np.ix_(row_indices, col_indices)].copy()
        frame = TerrainFrame(
            depth_roi=sampled,
            depth_scale_m=self._depth_scale_m,
            intrinsics=intrinsics,
            stamp_s=_stamp_s(message.header.stamp),
        )
        try:
            quality = analyze_depth_quality(
                sampled,
                depth_scale_m=frame.depth_scale_m,
                intrinsics=frame.intrinsics,
                frame_stamp_s=frame.stamp_s,
                config=self._depth_quality_config,
            )
        except (TypeError, ValueError):
            self._depth_quality_snapshot = (None, received_at_s)
        else:
            self._depth_quality_snapshot = (
                1.0 - quality.valid_ratio,
                received_at_s,
            )
        self._depth_quality_seen = True
        with self._terrain_state_lock:
            previous_depth_pose = self._previous_depth_pose
        try:
            estimate = self.estimator.update(
                frame,
                tilt=tilt,
                extrinsic=self.extrinsic,
                odometry_delta=self._odometry_delta(
                    odom,
                    previous_depth_pose,
                ),
                now_s=received_at_s,
            )
        except ValueError as exc:
            with self._depth_condition:
                if self._depth_stop:
                    return
                with self._terrain_state_lock:
                    _, terrain_seen = self._terrain_snapshot
                    self._terrain = None
                    self._terrain_snapshot = (None, terrain_seen)
            if self._publish_terrain_unavailable("value_error"):
                self.get_logger().error(
                    "terrain update rejected: %s" % exc
                )
            return
        with self._depth_condition:
            if self._depth_stop:
                return
            with self._terrain_state_lock:
                self._previous_depth_pose = (
                    float(odom[0]),
                    float(odom[1]),
                    float(odom[2]),
                )
                self._terrain = estimate
                self._terrain_seen = True
                self._terrain_snapshot = (estimate, True)
                self._terrain_update_count += 1
        if self._depth_is_stopping():
            return
        reject = ",".join(estimate.reject_reasons)
        self.pub_terrain_state.publish(
            String(
                data="%s|%.6f|%.6f|%.6f|%s"
                % (
                    bool(estimate.path_available),
                    estimate.path_offset_m,
                    estimate.heading_error_rad,
                    estimate.confidence,
                    reject,
                )
            )
        )

    def _motion_state(self, now_s: float) -> MotionState | None:
        odom, tilt, imu_stamp_s = self._motion_snapshot
        if odom is None or tilt is None or imu_stamp_s is None:
            return None
        stamps = (odom[5], imu_stamp_s)
        if not all(math.isfinite(stamp) for stamp in stamps):
            stamp_s = math.nan
        elif max(stamps) > now_s + 0.1:
            stamp_s = max(stamps)
        else:
            stamp_s = min(stamps)
        return MotionState(
            stamp_s=stamp_s,
            forward_m_s=odom[3],
            yaw_rate_rad_s=odom[4],
            roll_rad=tilt.roll_rad,
            pitch_rad=tilt.pitch_rad,
        )

    def _fresh_diagnostics(self, now_s: float) -> DriveDiagnostics | None:
        diagnostics = getattr(self, "_diagnostics", None)
        if diagnostics is None:
            return None
        stamp_s = diagnostics.stamp_s
        if (
            not math.isfinite(stamp_s)
            or stamp_s > now_s + 0.1
            or now_s - stamp_s > self.controller.config.diagnostics_stale_s
        ):
            return None
        return diagnostics

    def _depth_quality(self, now_s: float) -> float | None:
        if not getattr(self, "_depth_quality_seen", False):
            # Before the first analyzable frame, terrain_missing already owns
            # fail-closed startup.  Do not poison the FSM's dropout hysteresis.
            return 0.0
        value, stamp_s = getattr(
            self,
            "_depth_quality_snapshot",
            (None, None),
        )
        if (
            stamp_s is None
            or not math.isfinite(stamp_s)
            or stamp_s > now_s + 0.1
            or now_s - stamp_s > self.controller.config.terrain_stale_s
        ):
            return None
        return value

    def _degradation_diagnostics(
        self,
        now_s: float,
    ) -> DriveDiagnostics:
        source = self._fresh_diagnostics(now_s)
        slip_candidate = bool(
            source is not None and source.slip_candidate
        )
        stuck_candidate = bool(
            source is not None and source.stuck_candidate
        )
        output = self.degradation.update(
            depth_quality=self._depth_quality(now_s),
            slip_candidate=slip_candidate,
            stuck_candidate=stuck_candidate,
            traveled_m=getattr(self, "_traveled_m", 0.0),
            now_s=now_s,
        )
        previous_stage = getattr(
            self,
            "_last_degradation_stage",
            DegradationStage.NORMAL,
        )
        self._degradation_output = output
        if output.stage is not previous_stage:
            self._emit_degradation_event(previous_stage, output, now_s)
        self._last_degradation_stage = output.stage

        degradation_cap = (
            self.controller.profile.max_speed_m_s * output.speed_scale
        )
        source_cap = math.inf if source is None else source.speed_cap_m_s
        speed_cap = (
            min(float(source_cap), degradation_cap)
            if math.isfinite(float(source_cap))
            else degradation_cap
        )
        return DriveDiagnostics(
            stamp_s=now_s,
            slip_candidate=slip_candidate,
            stuck_candidate=stuck_candidate or output.request_hold,
            speed_cap_m_s=speed_cap,
        )

    def _publish_degradation_state(self) -> None:
        output = getattr(self, "_degradation_output", None)
        publisher = getattr(self, "pub_degradation_state", None)
        if output is None or publisher is None:
            return
        publisher.publish(
            String(
                data=json.dumps(
                    {
                        "stage": output.stage.value,
                        "speed_scale": output.speed_scale,
                        "reasons": list(output.reasons),
                        "stamp_s": self._now_s(),
                    },
                    allow_nan=False,
                    separators=(",", ":"),
                    sort_keys=True,
                )
            )
        )

    def _emit_degradation_event(self, previous_stage, output, now_s) -> bool:
        client = getattr(self, "_event_client", None)
        if client is None:
            return False
        event = {
            "schema_version": 1,
            "wall_time_ns": time.time_ns(),
            "monotonic_ns": time.monotonic_ns(),
            "source": "autonomy_controller_node",
            "event_type": "DEGRADATION",
            "severity": "WARN" if output.request_hold else "INFO",
            "payload": {
                "from_stage": previous_stage.value,
                "to_stage": output.stage.value,
                "speed_scale": output.speed_scale,
                "request_hold": output.request_hold,
                "handover_wait": output.handover_wait,
                "reasons": list(output.reasons),
                "stamp_s": now_s,
            },
        }
        try:
            return bool(client.emit(event))
        except Exception:
            return False

    def _tick(self) -> None:
        now_s = self._now_s()
        terrain, terrain_seen = self._terrain_snapshot
        assist = assist_correction_from_terrain(
            terrain,
            self.controller.config,
        )
        if assist is not None:
            omega_correction, speed_cap, confidence = assist
            self.pub_assist_correction.publish(
                String(
                    data=json.dumps(
                        {
                            "stamp_s": float(terrain.stamp_s),
                            "omega_correction_rad_s": omega_correction,
                            "speed_cap_m_s": speed_cap,
                            "confidence": confidence,
                        },
                        allow_nan=False,
                        separators=(",", ":"),
                        sort_keys=True,
                    )
                )
            )
        decision = self.controller.decide(
            now_s,
            terrain=terrain,
            motion=self._motion_state(now_s),
            gate=self._gate,
            diagnostics=self._degradation_diagnostics(now_s),
        )
        self.pub_controller_state.publish(
            String(
                data="%s|%.6f|%.6f|%s"
                % (
                    decision.state,
                    decision.v_m_s,
                    decision.omega_rad_s,
                    ",".join(decision.reasons),
                )
            )
        )
        if not self._enabled or not terrain_seen:
            return
        command = Twist()
        command.linear.x = decision.v_m_s
        command.angular.z = decision.omega_rad_s
        self.pub_cmd.publish(command)

    def destroy_node(self):
        condition = getattr(self, "_depth_condition", None)
        if condition is not None:
            with condition:
                self._depth_stop = True
                self._depth_slot = None
                condition.notify_all()
        worker = getattr(self, "_depth_worker_thread", None)
        if (
            worker is not None
            and worker is not threading.current_thread()
            and worker.is_alive()
        ):
            worker.join(
                timeout=getattr(
                    self,
                    "_depth_worker_join_timeout_s",
                    1.0,
                )
            )
        return super().destroy_node()


def main():
    rclpy.init()
    node = AutonomyControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
