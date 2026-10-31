"""terrain_map_node — ROS adapter around terrain_map.runtime (domain 77)."""
import json
import math
import socket
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, Imu

from powertrain_msgs.msg import WheelStates
from powertrain_ros import terrain_map_node as N
from chassis.kinematics import default_geometry
from terrain_map import contract as C
from terrain_map import runtime as runtime_module
from terrain_map.tests.odometry_sim import Command
from terrain_map.tests.world_render import rot_x, rot_y, rot_z

import terrain_map_fixture as F


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def no_operator_session(monkeypatch):
    monkeypatch.delenv("POWERTRAIN_OPERATOR_SESSION_FILE", raising=False)


@pytest.fixture
def udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.setblocking(False)
    yield sock
    sock.close()


def make_node(port=5010, **params):
    overrides = [Parameter("operator_host", value="127.0.0.1"), Parameter("operator_port", value=port)]
    overrides += [Parameter(k, value=v) for k, v in {**F.NODE_PARAMS, **params}.items()]
    return N.TerrainMapNode(parameter_overrides=overrides)


def call(node, event):
    """Deliver one fixture message straight into the node's callback (no DDS)."""
    {F.INFO_TOPIC: node._on_info, F.DEPTH_TOPIC: node._on_depth,
     F.GYRO_TOPIC: node._on_gyro, F.WHEEL_TOPIC: node._on_wheels}[event.topic](event.message)


# --- configuration --------------------------------------------------------------
def test_parameters_reach_map_config():
    node = make_node(cell_m=0.1, obstacle_height_m=0.1, support_base_height_m=0.471,
                     support_base_variation_m=0.03, camera_pitch_down_deg=25.0)
    try:
        cfg = node.config
        assert (cfg.cell_m, cfg.obstacle_height_m, cfg.support_base_height_m,
                cfg.support_base_variation_m, cfg.camera_pitch_down_deg) == (0.1, 0.1, 0.471, 0.03, 25.0)
        assert cfg.support_height_check_enabled and node._runtime.cfg is cfg
    finally:
        node.destroy_node()


@pytest.mark.parametrize("params", [{"cell_m": 0.5}, {"support_base_height_m": 0.471, "support_base_variation_m": 0.05},
                                    {"corrector": "height_match"}])
def test_invalid_parameters_refuse_to_start(params):
    with pytest.raises(ValueError):
        make_node(**params)


def test_wheel_geometry_comes_from_the_chassis_kinematics():
    geometry = default_geometry()
    layout = N.wheel_layout()
    assert layout.positions == {w.name: (w.x, w.y) for w in geometry.wheels}
    node = make_node()
    try:
        assert node.config.wheel_radius_m == pytest.approx(geometry.wheel_radius_m)
    finally:
        node.destroy_node()


def test_node_publishes_nothing():
    node = make_node()
    try:
        topics = {name for name, _ in node.get_publisher_names_and_types_by_node("terrain_map", "/")}
        assert topics <= {"/rosout", "/parameter_events"}
    finally:
        node.destroy_node()


# --- gyro: optical frame -> body z, whole node path --------------------------------
SPIN = 0.5


def spin_events(mount):
    events, truth = F.scenario(F.drive_then(Command("skid", v=0.0, omega=SPIN)), 4.5, surface=None, mount=mount)
    return events, truth


def node_yaw(events, mount):
    node = make_node(camera_pitch_down_deg=mount[0], camera_roll_deg=mount[1], camera_yaw_deg=mount[2])
    try:
        for e in events:
            call(node, e)
        node._runtime.odometry.drain()
        return node._runtime.odometry.yaw, node._runtime.odometry.epoch
    finally:
        node.destroy_node()


def test_compound_mount_gyro_integrates_the_true_yaw():
    mount = (20.0, 20.0, 30.0)
    events, truth = spin_events(mount)
    yaw, epoch = node_yaw(events, mount)
    _, _, true_yaw = truth(F.START + 4.5)
    assert epoch == 0 and abs(math.degrees(yaw - true_yaw)) < 0.5 and true_yaw > 1.4


def test_swapped_rotation_order_is_caught(monkeypatch):
    def wrong(pitch_down_deg, roll_deg, yaw_deg):          # Rx·Ry·Rz instead of Rz·Ry·Rx
        m = rot_x(math.radians(roll_deg)) @ rot_y(math.radians(pitch_down_deg)) @ rot_z(math.radians(yaw_deg))
        p = np.array([[0.0, -1.0, 0.0], [0.0, 0.0, -1.0], [1.0, 0.0, 0.0]])
        return p @ m.T
    monkeypatch.setattr(runtime_module, "base_to_optical", wrong)
    mount = (20.0, 20.0, 30.0)
    events, truth = spin_events(mount)
    yaw, _ = node_yaw(events, mount)
    _, _, true_yaw = truth(F.START + 4.5)
    assert abs(math.degrees(yaw - true_yaw)) > 0.5


# --- over DDS --------------------------------------------------------------------
class Spinning:
    def __init__(self, node):
        self.node = node
        self.executor = MultiThreadedExecutor(num_threads=3)
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        self.node.destroy_node()


class Publisher:
    def __init__(self):
        self.node = rclpy.create_node("terrain_map_fixture")
        self.pubs = {
            F.DEPTH_TOPIC: self.node.create_publisher(Image, F.DEPTH_TOPIC, qos_profile_sensor_data),
            F.INFO_TOPIC: self.node.create_publisher(CameraInfo, F.INFO_TOPIC, qos_profile_sensor_data),
            F.GYRO_TOPIC: self.node.create_publisher(Imu, F.GYRO_TOPIC, qos_profile_sensor_data),
            F.WHEEL_TOPIC: self.node.create_publisher(WheelStates, F.WHEEL_TOPIC, 10),
        }

    def play(self, events, udp, packets):
        """Publish in real time by stamp, draining UDP as we go."""
        wall0 = time.monotonic()
        for e in events:
            delay = wall0 + (e.stamp_s - F.START) - time.monotonic()
            if delay > 0:
                time.sleep(delay)
            self.pubs[e.topic].publish(e.message)
            drain(udp, packets)
        end = time.monotonic() + 0.6
        while time.monotonic() < end:
            time.sleep(0.01)
            drain(udp, packets)


def drain(udp, packets):
    while True:
        try:
            packets.append(C.parse_packet(udp.recv(2048)))
        except BlockingIOError:
            return


@pytest.fixture(scope="module")
def drive_events():
    return F.scenario(F.drive_then(Command("skid", v=0.3)), 4.5)


def test_live_drive_builds_the_box_and_tracks_the_pose(udp, drive_events):
    events, truth = drive_events
    spinning = Spinning(make_node(udp.getsockname()[1]))
    publisher = Publisher()
    packets = []
    try:
        time.sleep(0.5)                                    # discovery
        publisher.play(events, udp, packets)
    finally:
        spinning.stop()
        publisher.node.destroy_node()
    status = [p for p in packets if isinstance(p, C.StatusPacket)]
    ok = [s for s in status if s.status == "OK" and s.map_epoch == 0]   # the stall after the run starts epoch 1
    assert ok, [(s.status, s.reason) for s in status[-5:]]
    x, _, _ = truth(F.START + 4.5)
    assert ok[-1].pose is not None and abs(ok[-1].pose.x_m - x) < 0.1
    obstacle_x = []
    for t in (p for p in packets if isinstance(p, C.TilePacket) and p.map_epoch == 0):
        heights, codes, _ = C.unpack_cells(t.cells, 16)
        for i, code in enumerate(codes):
            if code == C.OBSTACLE:
                obstacle_x.append((t.tx * 16 + i % 16 + 0.5) * ok[-1].cell_m)
    assert obstacle_x and abs(np.median(obstacle_x) - F.BOX_CENTRE_X) < 0.2


def test_all_wheels_stale_reports_wheels_stale(udp):
    events, _ = F.scenario(F.drive_then(Command("skid", v=0.3)), 3.5, stale_wheels_after_s=2.0)
    spinning = Spinning(make_node(udp.getsockname()[1]))
    publisher = Publisher()
    packets = []
    try:
        time.sleep(0.5)
        publisher.play(events, udp, packets)
    finally:
        spinning.stop()
        publisher.node.destroy_node()
    seen = [(p.status, p.reason) for p in packets if isinstance(p, C.StatusPacket)]
    assert ("OK", None) in seen and ("NO_POSE", "WHEELS_STALE") in seen[seen.index(("OK", None)):]
