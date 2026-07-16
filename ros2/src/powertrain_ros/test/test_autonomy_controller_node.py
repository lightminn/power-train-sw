"""ROS adapter tests for the one-process WP6-B/WP6-C node."""
from __future__ import annotations

import json
import math
import threading
import time

import numpy as np
import pytest
import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from sensor_msgs.msg import CameraInfo, Image, Imu
from std_msgs.msg import String

from robot_arm_msgs.msg import ArmStatus
from powertrain_ros.autonomy_controller_node import AutonomyControllerNode


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _stamp(node, message):
    message.header.stamp = node.get_clock().now().to_msg()
    return message


def _camera_info(node, width=80, height=60):
    message = _stamp(node, CameraInfo())
    message.width = width
    message.height = height
    message.k = [57.1, 0.0, (width - 1) / 2.0,
                 0.0, 57.6, (height - 1) / 2.0,
                 0.0, 0.0, 1.0]
    return message


def _render_flat_track_depth():
    """Render a 1.4 m elevated flat track and the lower floor beside it."""
    height, width = 60, 80
    fx, fy, cx, cy = 57.1, 57.6, 39.5, 29.5
    rows, cols = np.indices((height, width), dtype=float)
    rays = np.stack(
        ((cols - cx) / fx, (rows - cy) / fy, np.ones((height, width))),
        axis=-1,
    )
    pitch = math.radians(25.0)
    camera_to_base = np.array(
        ((0.0, -math.sin(pitch), math.cos(pitch)),
         (-1.0, 0.0, 0.0),
         (0.0, -math.cos(pitch), -math.sin(pitch)))
    )
    directions = rays @ camera_to_base.T
    origin = np.array((0.0, 0.0, 0.60))
    with np.errstate(divide="ignore", invalid="ignore"):
        upper_t = -origin[2] / directions[..., 2]
        upper_x = origin[0] + upper_t * directions[..., 0]
        upper_y = origin[1] + upper_t * directions[..., 1]
        lower_t = (-0.45 - origin[2]) / directions[..., 2]
    on_track = (
        np.isfinite(upper_t)
        & (upper_t > 0.0)
        & (upper_x >= 0.0)
        & (upper_x < 8.0)
        & (np.abs(upper_y) <= 0.70)
    )
    lower_valid = np.isfinite(lower_t) & (lower_t > 0.0)
    depth_m = np.where(on_track, upper_t, np.where(lower_valid, lower_t, 0.0))
    return np.rint(np.clip(depth_m * 1000.0, 0.0, 65535.0)).astype(np.uint16)


def _depth(node, raw=None, width=80, height=60):
    raw = _render_flat_track_depth() if raw is None else raw
    message = _stamp(node, Image())
    message.width = width
    message.height = height
    message.encoding = "16UC1"
    message.is_bigendian = False
    message.step = width * 2
    message.data = raw.tobytes()
    return message


def _imu(node):
    message = _stamp(node, Imu())
    message.orientation.w = 1.0
    return message


def _odom(node):
    message = _stamp(node, Odometry())
    message.pose.pose.orientation.w = 1.0
    return message


def _arm(node, status="STOWED_LOCKED"):
    message = _stamp(node, ArmStatus())
    message.status = status
    return message


class Harness:
    def __init__(self, controller):
        self.node = rclpy.create_node("autonomy_controller_test_harness")
        self.info = self.node.create_publisher(CameraInfo, "/l515/depth/camera_info", 10)
        self.depth = self.node.create_publisher(Image, "/l515/depth/image_rect_raw", 10)
        self.imu = self.node.create_publisher(Imu, "/imu/filtered", 10)
        self.odom = self.node.create_publisher(Odometry, "/odom", 10)
        self.arm = self.node.create_publisher(ArmStatus, "/arm_status", 10)
        self.commands = []
        self.command_times = []
        self.controller_states = []
        self.terrain_states = []
        self.assist_corrections = []
        self.node.create_subscription(
            Twist, "/autonomy/cmd_vel", self._record_command, 10
        )
        self.node.create_subscription(
            String, "/autonomy/controller_state", self.controller_states.append, 10
        )
        self.node.create_subscription(
            String, "/autonomy/terrain_state", self.terrain_states.append, 10
        )
        self.node.create_subscription(
            String,
            "/autonomy/assist_correction",
            self.assist_corrections.append,
            10,
        )
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(controller)
        self.executor.add_node(self.node)

    def _record_command(self, message):
        self.commands.append(message)
        self.command_times.append(time.monotonic())

    def spin_for(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)

    # 젯슨이 이미지 빌드 직후 등 부하 상태면 depth→terrain→tick 체인이 2 s를
    # 넘겨 플레이크가 된다(07-16·07-17 각 1회 관측, 재실행 GREEN) — 여유 상향.
    def spin_until(self, predicate, timeout=5.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("condition did not become true before timeout")

    def publish_motion_gate(self, controller, status="STOWED_LOCKED"):
        self.imu.publish(_imu(controller))
        self.odom.publish(_odom(controller))
        self.arm.publish(_arm(controller, status))

    def publish_complete_frame(self, controller, *, status="STOWED_LOCKED"):
        self.info.publish(_camera_info(controller))
        self.publish_motion_gate(controller, status)
        self.spin_for(0.05)
        self.depth.publish(_depth(controller))

    def settle(self):
        """직전 테스트 노드의 비행 중 메시지를 흡수하고 버퍼를 비운다.

        느린 호스트(젯슨)에서는 파괴된 노드의 마지막 발행이 다음 테스트의
        구독으로 배달돼 '발행 없음' 단언을 오염시킨다(07-17 실측).
        """
        self.spin_for(0.25)
        self.commands.clear()
        self.command_times.clear()
        self.controller_states.clear()
        self.terrain_states.clear()
        self.assist_corrections.clear()

    def pump_terrain_until(
        self, controller, predicate, *, status="STOWED_LOCKED", timeout=8.0
    ):
        """실스트림처럼 입력을 반복 발행하며 predicate를 기다린다.

        워커 스레드는 executor와 독립이라, 느린 호스트에서는 단발 depth가
        camera_info/모션 콜백 처리 전에 소비돼 폐기될 수 있다(실스트림은
        다음 프레임으로 자가 치유 — 테스트도 같은 형태여야 한다).
        """
        deadline = time.monotonic() + timeout
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                self.info.publish(_camera_info(controller))
                self.publish_motion_gate(controller, status)
                self.depth.publish(_depth(controller))
                next_publish = now + 0.10
            self.executor.spin_once(timeout_sec=0.02)
            if predicate():
                return
        raise AssertionError("condition did not become true before timeout")

    def close(self, controller):
        self.executor.remove_node(controller)
        self.executor.remove_node(self.node)
        controller.destroy_node()
        self.node.destroy_node()
        self.executor.shutdown()


def _controller(enabled=True, drive_profile="EMPTY_STOWED"):
    return AutonomyControllerNode(
        parameter_overrides=[
            Parameter("enabled", value=enabled),
            Parameter("drive_profile", value=drive_profile),
        ]
    )


def test_synthetic_flat_track_inputs_publish_valid_twist_when_enabled():
    controller = _controller(enabled=True)
    harness = Harness(controller)
    try:
        harness.settle()
        harness.pump_terrain_until(
            controller,
            lambda: any(message.linear.x > 0.0 for message in harness.commands),
        )
        command = next(message for message in harness.commands if message.linear.x > 0.0)
        assert math.isfinite(command.linear.x)
        assert math.isfinite(command.angular.z)
        assert command.linear.x >= 0.0
        harness.spin_until(lambda: bool(harness.assist_corrections))
        payload = json.loads(harness.assist_corrections[-1].data)
        assert set(payload) == {
            "stamp_s",
            "omega_correction_rad_s",
            "speed_cap_m_s",
            "confidence",
        }
        assert all(math.isfinite(float(value)) for value in payload.values())
        assert abs(payload["omega_correction_rad_s"]) <= 0.4
        assert 0.0 <= payload["speed_cap_m_s"] <= 0.8
        assert 0.0 <= payload["confidence"] <= 1.0
    finally:
        harness.close(controller)


def test_no_command_is_published_before_first_terrain_estimate():
    controller = _controller(enabled=True)
    harness = Harness(controller)
    try:
        harness.settle()
        harness.publish_motion_gate(controller)
        harness.spin_for(0.20)
        assert harness.commands == []
        assert harness.assist_corrections == []
        assert harness.controller_states
    finally:
        harness.close(controller)


def test_slow_terrain_update_does_not_starve_command_timer(monkeypatch):
    controller = _controller(enabled=True)
    harness = Harness(controller)
    try:
        harness.settle()
        harness.pump_terrain_until(
            controller,
            lambda: any(message.linear.x > 0.0 for message in harness.commands),
        )

        real_update = controller.estimator.update
        slow_entered = threading.Event()
        slow_completed = threading.Event()

        def slow_update(*args, **kwargs):
            slow_entered.set()
            time.sleep(0.20)
            result = real_update(*args, **kwargs)
            slow_completed.set()
            return result

        monkeypatch.setattr(controller.estimator, "update", slow_update)
        harness.commands.clear()
        harness.command_times.clear()

        deadline = time.monotonic() + 1.40
        next_publish = 0.0
        while time.monotonic() < deadline:
            now = time.monotonic()
            if now >= next_publish:
                harness.publish_motion_gate(controller)
                harness.depth.publish(_depth(controller))
                next_publish = now + 0.03
            harness.executor.spin_once(timeout_sec=0.005)

        assert slow_entered.is_set()
        assert slow_completed.is_set()
        assert len(harness.command_times) >= 15
        intervals = np.diff(harness.command_times)
        assert np.percentile(intervals, 95) <= 2.0 / 20.0
    finally:
        harness.close(controller)


def test_depth_burst_processes_first_and_latest_frames_only(monkeypatch):
    controller = _controller(enabled=False)
    release = threading.Event()
    try:
        controller._on_camera_info(_camera_info(controller))
        controller._on_imu(_imu(controller))
        controller._on_odom(_odom(controller))

        entered = threading.Event()
        processed_stamps = []
        real_update = controller.estimator.update

        def blocking_first_update(frame, **kwargs):
            processed_stamps.append(frame.stamp_s)
            if len(processed_stamps) == 1:
                entered.set()
                assert release.wait(timeout=1.0)
            return real_update(frame, **kwargs)

        monkeypatch.setattr(
            controller.estimator, "update", blocking_first_update
        )

        base = controller.get_clock().now().nanoseconds
        frames = []
        expected_stamps = []
        for index in range(5):
            message = _depth(controller)
            stamp_ns = base + index * 1_000_000
            message.header.stamp.sec = stamp_ns // 1_000_000_000
            message.header.stamp.nanosec = stamp_ns % 1_000_000_000
            frames.append(message)
            expected_stamps.append(stamp_ns * 1e-9)

        controller._on_depth(frames[0])
        assert entered.wait(timeout=1.0)
        for message in frames[1:]:
            controller._on_depth(message)
        release.set()

        deadline = time.monotonic() + 1.0
        while (
            controller._terrain_update_count < 2
            and time.monotonic() < deadline
        ):
            time.sleep(0.01)

        assert controller._terrain_update_count == 2
        assert processed_stamps == pytest.approx(
            [expected_stamps[0], expected_stamps[-1]],
            rel=0.0,
            abs=1e-6,
        )
        assert controller._depth_overwrite_count == len(frames) - 2
    finally:
        release.set()
        controller.destroy_node()


def test_destroy_node_stops_depth_worker():
    controller = _controller(enabled=False)
    worker = controller._depth_worker_thread
    assert worker.is_alive()

    controller.destroy_node()

    assert not worker.is_alive()


def test_destroy_timeout_suppresses_late_worker_error_publish(monkeypatch):
    controller = _controller(enabled=False)
    release = threading.Event()
    entered = threading.Event()
    unavailable_reasons = []
    destroyed = False
    try:
        controller._on_camera_info(_camera_info(controller))
        controller._on_imu(_imu(controller))
        controller._on_odom(_odom(controller))
        controller._depth_worker_join_timeout_s = 0.05

        def blocked_error(*args, **kwargs):
            entered.set()
            assert release.wait(timeout=1.0)
            raise ValueError("late estimator failure")

        monkeypatch.setattr(controller.estimator, "update", blocked_error)
        monkeypatch.setattr(
            controller,
            "_publish_terrain_unavailable",
            unavailable_reasons.append,
        )
        controller._on_depth(_depth(controller))
        assert entered.wait(timeout=1.0)

        started = time.monotonic()
        controller.destroy_node()
        destroyed = True
        elapsed = time.monotonic() - started

        assert elapsed < 0.20
        assert controller._depth_worker_thread.is_alive()
        release.set()
        controller._depth_worker_thread.join(timeout=1.0)
        assert not controller._depth_worker_thread.is_alive()
        assert unavailable_reasons == []
    finally:
        release.set()
        if controller._depth_worker_thread.is_alive():
            controller._depth_worker_thread.join(timeout=1.0)
        if not destroyed:
            controller.destroy_node()


def test_disabled_node_publishes_diagnostics_but_no_command():
    controller = _controller(enabled=False)
    harness = Harness(controller)
    try:
        harness.settle()
        harness.pump_terrain_until(
            controller, lambda: bool(harness.terrain_states)
        )
        harness.spin_until(lambda: bool(harness.controller_states))
        assert harness.commands == []
        harness.spin_until(lambda: bool(harness.assist_corrections))
        payload = json.loads(harness.assist_corrections[-1].data)
        assert math.isfinite(payload["omega_correction_rad_s"])
    finally:
        harness.close(controller)


def test_depth_loss_ramps_to_zero_and_keeps_publishing_zero():
    controller = _controller(enabled=True)
    harness = Harness(controller)
    try:
        harness.settle()
        harness.pump_terrain_until(
            controller,
            lambda: any(message.linear.x > 0.0 for message in harness.commands),
        )
        deadline = time.monotonic() + 1.5
        while time.monotonic() < deadline:
            harness.publish_motion_gate(controller)
            harness.spin_for(0.08)
            if harness.commands and harness.commands[-1].linear.x == 0.0:
                break
        zero_index = next(
            index
            for index, message in enumerate(harness.commands)
            if index > 0 and message.linear.x == 0.0
        )
        harness.publish_motion_gate(controller)
        harness.spin_for(0.15)
        assert len(harness.commands) > zero_index + 1
        assert harness.commands[-1].linear.x == 0.0
    finally:
        harness.close(controller)


@pytest.mark.parametrize("mode", ("stale", "mismatch"))
def test_arm_loss_or_mismatch_blocks_immediately(mode):
    controller = _controller(enabled=True)
    harness = Harness(controller)
    try:
        harness.settle()
        harness.pump_terrain_until(
            controller,
            lambda: any(message.linear.x > 0.0 for message in harness.commands),
        )
        if mode == "mismatch":
            harness.publish_motion_gate(controller, status="EXECUTING")
            harness.depth.publish(_depth(controller))
            harness.spin_until(
                lambda: harness.commands[-1].linear.x == 0.0,
                timeout=0.25,
            )
        else:
            deadline = time.monotonic() + 0.75
            while time.monotonic() < deadline:
                harness.imu.publish(_imu(controller))
                harness.odom.publish(_odom(controller))
                harness.depth.publish(_depth(controller))
                harness.spin_for(0.08)
            assert harness.commands[-1].linear.x == 0.0
        assert harness.commands[-1].angular.z == 0.0
    finally:
        harness.close(controller)


def test_changed_camera_info_resolution_and_matching_frame_are_ignored():
    controller = _controller(enabled=False)
    try:
        controller._on_imu(_imu(controller))
        controller._on_odom(_odom(controller))
        controller._on_arm_status(_arm(controller))
        controller._on_camera_info(_camera_info(controller))
        controller._process_depth_now(_depth(controller))
        count = controller._terrain_update_count

        controller._on_camera_info(_camera_info(controller, width=160, height=120))
        resized = np.zeros((120, 160), dtype=np.uint16)
        controller._process_depth_now(
            _depth(controller, resized, width=160, height=120)
        )

        assert controller._grid_source_shape == (60, 80)
        assert controller._terrain_update_count == count
    finally:
        controller.destroy_node()


def test_first_camera_info_fixes_uniform_stride_central_crop_and_intrinsics():
    controller = _controller(enabled=False)
    try:
        info = _camera_info(controller, width=1280, height=720)
        info.k = [960.0, 0.0, 639.5,
                  0.0, 960.0, 359.5,
                  0.0, 0.0, 1.0]
        controller._on_camera_info(info)

        assert controller._grid_source_shape == (720, 1280)
        assert controller._row_indices[0] == 0
        assert controller._row_indices[-1] == 708
        assert controller._col_indices[0] == 160
        assert controller._col_indices[-1] == 1108
        assert controller._intrinsics.fx == pytest.approx(80.0)
        assert controller._intrinsics.fy == pytest.approx(80.0)
        assert controller._intrinsics.cx == pytest.approx((639.5 - 160) / 12)
        assert controller._intrinsics.cy == pytest.approx(359.5 / 12)
    finally:
        controller.destroy_node()


def test_diagnostics_null_speed_cap_maps_to_unlimited():
    controller = _controller(enabled=False)
    try:
        controller._on_diagnostics(
            String(
                data=(
                    '{"stamp_s":1.0,"slip_candidate":false,'
                    '"stuck_candidate":false,"terrain_profile":"default",'
                    '"speed_cap_m_s":null}'
                )
            )
        )
        assert math.isinf(controller._diagnostics.speed_cap_m_s)
    finally:
        controller.destroy_node()


def test_invalid_drive_profile_fails_startup():
    with pytest.raises(ValueError, match="drive_profile"):
        _controller(enabled=False, drive_profile="NOT_A_PROFILE")
