"""odometry_node · joint_state_bridge_node — ROS 래퍼.

계산 자체는 순수 코어(`motor_control/chassis/odometry.py`, pytest 22종)가 담보한다.
여기서는 **래퍼가 제대로 옮기는지**만 본다 — 메시지 → 코어 자료형 → 메시지.
"""
import dataclasses
import json
import math

import pytest
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Imu
from std_msgs.msg import String
from std_srvs.srv import Trigger

from powertrain_msgs.msg import WheelState, WheelStates
from powertrain_ros.joint_state_bridge_node import ALL_WHEELS, STEERABLE, JointStateBridge
from powertrain_ros.odometry_node import OdometryNode, _quat


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _wheel_states(geom, drive_rev=None, steer=None, stale=(), t=0.0):
    m = WheelStates()
    m.header.stamp.sec = int(t)
    m.header.stamp.nanosec = int((t % 1.0) * 1e9)
    for w in geom.wheels:
        ws = WheelState()
        ws.name = w.name
        ws.drive_turns_per_s = float((drive_rev or {}).get(w.name, 0.0))
        ws.steer_deg = float((steer or {}).get(w.name, 0.0))
        ws.drive_stale = w.name in stale
        m.wheels.append(ws)
    return m


def _straight(geom, v_mps):
    """직진: 전 바퀴 같은 rev/s, 조향 0."""
    circ = 2 * math.pi * geom.wheel_radius_m
    return _wheel_states(geom, drive_rev={w.name: v_mps / circ for w in geom.wheels})


# ── 쿼터니언 ─────────────────────────────────────────────────────────────

def test_quat_identity():
    assert _quat(0, 0, 0) == pytest.approx((0.0, 0.0, 0.0, 1.0))


def test_quat_normalized():
    q = _quat(0.2, -0.4, 2.0)
    assert sum(v * v for v in q) == pytest.approx(1.0)


# ── 오도메트리 노드 ──────────────────────────────────────────────────────

def _drive_for(n, geom, v_mps, secs, dt=0.02):
    """dt 간격 타임스탬프로 /wheel_states 를 먹인다 (첫 샘플은 dt 기준점이라 버려진다)."""
    for i in range(int(secs / dt) + 1):
        m = _straight(geom, v_mps)
        m.header.stamp.sec = 0
        m.header.stamp.nanosec = int(i * dt * 1e9)
        n._on_wheels(m)


def test_straight_drive_advances_x():
    n = OdometryNode()
    try:
        _drive_for(n, n.geom, 0.4, secs=1.0)
        x, y, _ = n.odo.pose()
        assert x == pytest.approx(0.4, rel=0.05)   # 1초에 0.4 m
        assert abs(y) < 1e-6
    finally:
        n.destroy_node()


def test_stale_wheel_is_excluded():
    """`drive_stale` 인 바퀴는 방정식에서 빼야 한다 — 죽은 센서를 믿으면 안 된다."""
    n = OdometryNode()
    try:
        geom = n.geom
        m = _straight(geom, 0.4)
        m.wheels[0].drive_stale = True
        m.wheels[0].drive_turns_per_s = 99.0      # 말도 안 되는 값
        m.header.stamp.nanosec = 0
        n._on_wheels(m)
        m2 = _straight(geom, 0.4)
        m2.wheels[0].drive_stale = True
        m2.wheels[0].drive_turns_per_s = 99.0
        m2.header.stamp.nanosec = int(0.02 * 1e9)
        n._on_wheels(m2)
        assert n._last.vx == pytest.approx(0.4, rel=0.02)   # 나머지 5개로 정답
        assert n._last.used == 5
    finally:
        n.destroy_node()


def test_imu_yaw_overrides_wheel_omega():
    """★ 원칙 '바퀴=거리, IMU=회전' — fresh IMU 스트림이 있으면 회전은 IMU 를 따른다.

    WP6-A 코어 전환으로 단일 IMU 샘플의 rate 를 무기한 유지하지 않는다(stale 재생 금지
    독트린). 대신 fresh 스트림 동안 IMU 적분이 바퀴 ω(0)를 override 함을 고정한다 —
    바퀴 정지 + 큰 각속도는 bias 가 아니라 실회전이다(빙판 슬립 시나리오)."""
    n = OdometryNode()
    try:
        geom = n.geom
        dt = 0.02
        for i in range(51):                       # 1.0 s, 바퀴·IMU 동시 스트림
            m = _straight(geom, 0.0)              # 바퀴는 "안 돈다"고 말한다
            m.header.stamp.sec = 0
            m.header.stamp.nanosec = int(i * dt * 1e9)
            n._on_wheels(m)
            imu = Imu()
            imu.orientation.w = 1.0
            imu.angular_velocity.z = 0.5          # IMU: 돌고 있다
            imu.header.stamp.sec = 0
            imu.header.stamp.nanosec = int(i * dt * 1e9)
            n._on_imu(imu)
        _, _, yaw = n.odo.pose()
        assert yaw == pytest.approx(0.5 * 1.0, rel=0.05)   # IMU 를 따라 1초에 0.5 rad
    finally:
        n.destroy_node()


def test_reset_service_zeroes_pose():
    """★ 리셋 수단이 없어 가짜 주행 후 x=77 m 로 누적됐던 부채를 막는다."""
    n = OdometryNode()
    try:
        geom = n.geom
        _drive_for(n, geom, 0.4, secs=1.0)
        assert n.odo.pose()[0] > 0.1

        n._srv_reset(Trigger.Request(), Trigger.Response())
        assert n.odo.pose() == (0.0, 0.0, 0.0)
    finally:
        n.destroy_node()


def test_odometry_publishes_controller_diagnostics_json():
    n = OdometryNode()
    snapshot = n.estimator.snapshot

    def snapshot_with_unlimited_cap(*, now_s):
        state = snapshot(now_s=now_s)
        return dataclasses.replace(
            state,
            diagnostics=dataclasses.replace(
                state.diagnostics,
                terrain_speed_cap=math.inf,
            ),
        )

    n.estimator.snapshot = snapshot_with_unlimited_cap
    observer = rclpy.create_node("odometry_diagnostics_test_observer")
    messages = []
    observer.create_subscription(
        String,
        "/odom_diagnostics",
        lambda message: messages.append(json.loads(message.data)),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(n)
    executor.add_node(observer)
    try:
        for _ in range(20):
            executor.spin_once(timeout_sec=0.02)
            if messages:
                break
        assert messages
        assert set(messages[-1]) == {
            "stamp_s",
            "slip_candidate",
            "stuck_candidate",
            "terrain_profile",
            "speed_cap_m_s",
        }
        assert isinstance(messages[-1]["slip_candidate"], bool)
        assert isinstance(messages[-1]["stuck_candidate"], bool)
        assert messages[-1]["speed_cap_m_s"] is None
    finally:
        executor.remove_node(n)
        executor.remove_node(observer)
        n.destroy_node()
        observer.destroy_node()
        executor.shutdown()


# ── joint_state_bridge ───────────────────────────────────────────────────

def test_joint_names_cover_all_joints():
    n = JointStateBridge()
    try:
        n._publish()
        assert len(STEERABLE) == 4 and len(ALL_WHEELS) == 6
    finally:
        n.destroy_node()


def test_bridge_converts_steer_deg_to_rad():
    n = JointStateBridge()
    try:
        from chassis.kinematics import default_geometry
        geom = default_geometry()
        n._on_wheels(_wheel_states(geom, steer={"front_left": 30.0}))
        assert n._steer["front_left"] == pytest.approx(math.radians(30.0))
    finally:
        n.destroy_node()


def test_bridge_publishes_zeros_without_motors():
    """모터가 꺼져 있어도 0 자세를 발행해야 RViz 에 로봇이 뜬다."""
    n = JointStateBridge()
    try:
        n._publish()
        assert all(v == 0.0 for v in n._steer.values())
        assert not n._have_data
    finally:
        n.destroy_node()
