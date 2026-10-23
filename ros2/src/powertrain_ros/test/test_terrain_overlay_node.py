"""terrain_overlay_node — ROS adapter around terrain_overlay.runtime (domain 77).

The node spins on a 2-thread MultiThreadedExecutor in a background thread,
exactly like ``main``; the fixture publishes from the test thread, so depth
keeps arriving independently of how long a tick takes.
"""
import json
import socket
import threading
import time

import numpy as np
import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image

from powertrain_ros.terrain_overlay_node import TerrainOverlayNode, depth_array
from terrain_overlay.tests.scenes import COLOR, DEPTH, default_scene

STALE_CONTRACT_S = 0.5 + 0.1 + 0.05  # spec §4.1 at 10 Hz
SLACK_S = 0.05  # test-loop sampling


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def no_operator_session(monkeypatch):
    # Inside the production container the session file would redirect every
    # packet to the real console; tests must send to their local fixture.
    monkeypatch.delenv("POWERTRAIN_OPERATOR_SESSION_FILE", raising=False)


@pytest.fixture
def udp():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(("127.0.0.1", 0))
    sock.setblocking(False)
    yield sock
    sock.close()


def info(camera):
    message = CameraInfo()
    message.width, message.height = camera.width, camera.height
    message.k = [camera.fx, 0.0, camera.cx, 0.0, camera.fy, camera.cy, 0.0, 0.0, 1.0]
    message.distortion_model = "plumb_bob"
    message.d = [0.0] * 5
    return message


def image(array, stamp_s=1, encoding="16UC1"):
    message = Image()
    message.header.stamp.sec = stamp_s
    message.height, message.width = array.shape
    message.encoding = encoding
    message.is_bigendian = 0
    message.step = array.shape[1] * 2
    message.data = array.tobytes()
    return message


class Spinning:
    """Run a node on the production executor shape in a background thread."""

    def __init__(self, node):
        self.node = node
        self.executor = MultiThreadedExecutor(num_threads=2)
        self.executor.add_node(node)
        self.thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.thread.start()

    def stop(self):
        self.executor.shutdown()
        self.thread.join(timeout=5.0)
        self.node.destroy_node()


class Fixture:
    def __init__(self):
        self.node = rclpy.create_node("terrain_fixture")
        self.depth = self.node.create_publisher(Image, "/l515/depth/image_rect_raw", qos_profile_sensor_data)
        self.depth_info = self.node.create_publisher(CameraInfo, "/l515/depth/camera_info", qos_profile_sensor_data)
        self.color_info = self.node.create_publisher(CameraInfo, "/l515/color/camera_info", qos_profile_sensor_data)
        self.stamp = 0

    def publish(self, array=None, encoding="16UC1"):
        self.stamp += 1
        self.depth_info.publish(info(DEPTH))
        self.color_info.publish(info(COLOR))
        self.depth.publish(image(default_scene() if array is None else array,
                                 stamp_s=self.stamp, encoding=encoding))


def collect(udp, seconds, publish=None, period_s=0.1):
    """Publish at the Gateway's 10 Hz and gather (receive time, packet)."""
    packets = []
    deadline = time.monotonic() + seconds
    next_publish = time.monotonic()
    while time.monotonic() < deadline:
        if publish is not None and time.monotonic() >= next_publish:
            publish()
            next_publish += period_s
        time.sleep(0.005)
        while True:
            try:
                packets.append((time.monotonic(), json.loads(udp.recv(2048))))
            except BlockingIOError:
                break
    return packets


def make_node(port, **params):
    overrides = [Parameter("operator_host", value="127.0.0.1"),
                 Parameter("operator_port", value=port)]
    overrides += [Parameter(name, value=value) for name, value in params.items()]
    return TerrainOverlayNode(parameter_overrides=overrides)


@pytest.fixture
def rig(udp):
    spinning = Spinning(make_node(udp.getsockname()[1]))
    fixture = Fixture()
    yield spinning.node, fixture, udp
    spinning.stop()
    fixture.node.destroy_node()


def test_ok_packet_then_stale_after_depth_stops(rig):
    node, fixture, udp = rig
    packets = collect(udp, 2.0, fixture.publish)
    ok = [p for _, p in packets if p["status"] == "OK"]
    assert ok and ok[-1]["obstacles"] and ok[-1]["obstacles"][0]["in_path"] is True
    assert len(ok[-1]["sender_boot_id"]) == 32 and len(ok[-1]["sender_instance"]) == 16
    stopped = time.monotonic()
    stale_at = [t for t, p in collect(udp, 1.2) if p["reason"] == "DEPTH_STALE"]
    assert stale_at and stale_at[0] - stopped <= STALE_CONTRACT_S + SLACK_S


def test_wrong_encoding_is_reported(rig):
    node, fixture, udp = rig
    packets = collect(udp, 1.0, lambda: fixture.publish(encoding="mono16"))
    assert any(p["reason"] == "DEPTH_ENCODING" for _, p in packets)


def test_camera_info_missing_is_reported_without_info(rig):
    node, fixture, udp = rig
    packets = collect(udp, 0.5)
    assert packets and all(p["reason"] == "CAMERA_INFO_MISSING" for _, p in packets)


def test_send_failures_are_absorbed(udp):
    node = make_node(udp.getsockname()[1])
    try:
        def refuse(*_args, **_kwargs):
            raise OSError(101, "Network is unreachable")

        node._udp.close()
        node._udp = type("Sock", (), {"sendto": staticmethod(refuse), "close": lambda self: None})()
        node._tick()
        node._tick()
        assert node.send_errors == 2
    finally:
        node.destroy_node()


def test_missing_operator_host_without_session_file_refuses_to_start(monkeypatch):
    monkeypatch.delenv("POWERTRAIN_OPERATOR_SESSION_FILE", raising=False)
    with pytest.raises(ValueError):
        TerrainOverlayNode()


def test_missing_session_file_sends_nothing_and_keeps_running(udp, monkeypatch, tmp_path):
    session_file = tmp_path / "operator-session.json"  # absent = no authenticated console
    monkeypatch.setenv("POWERTRAIN_OPERATOR_SESSION_FILE", str(session_file))
    spinning = Spinning(TerrainOverlayNode(parameter_overrides=[
        Parameter("operator_port", value=udp.getsockname()[1])]))
    try:
        assert collect(udp, 0.4) == []
        session_file.write_text(json.dumps(
            {"host": "127.0.0.1", "lease_id": "x", "expires_at": time.monotonic() + 60}))
        assert collect(udp, 0.4)  # resumes once the session is valid
        assert spinning.node.send_errors == 0
    finally:
        spinning.stop()


def test_slow_processing_keeps_the_stale_contract(rig, monkeypatch):
    node, fixture, udp = rig
    collect(udp, 0.3, fixture.publish)  # camera info received, detector built
    original = node._runtime._detector.process

    def slow(*args):
        time.sleep(0.15)  # slower than the 0.1 s tick
        return original(*args)

    monkeypatch.setattr(node._runtime._detector, "process", slow)
    packets = collect(udp, 2.0, fixture.publish)
    assert len(packets) >= 8  # keeps sending while every tick processes
    stopped = time.monotonic()
    stale_at = [t for t, p in collect(udp, 1.5) if p["reason"] == "DEPTH_STALE"]
    # Depth is stamped on its own executor thread, so slow ticks do not delay it.
    assert stale_at and stale_at[0] - stopped <= STALE_CONTRACT_S + SLACK_S


def test_depth_is_stamped_while_a_tick_is_still_processing(rig, monkeypatch):
    # Deterministic form of the slow-tick case: with depth in the tick's
    # callback group (or a single-threaded executor) this fails, because the
    # frame published mid-tick is only taken — and stamped — after the tick.
    node, fixture, udp = rig
    collect(udp, 0.3, fixture.publish)
    original = node._runtime._detector.process
    processing = threading.Event()

    def slow(*args):
        processing.set()
        time.sleep(0.6)
        return original(*args)

    monkeypatch.setattr(node._runtime._detector, "process", slow)
    fixture.publish()  # new stamp: the next tick starts a 0.6 s process
    assert processing.wait(1.0)
    before = node._runtime._latest.stamp_ns
    published = time.monotonic()
    fixture.publish()
    deadline = published + 0.3
    while node._runtime._latest.stamp_ns == before and time.monotonic() < deadline:
        time.sleep(0.005)
    assert node._runtime._latest.stamp_ns != before
    assert node._runtime._latest.rx_s - published < 0.1


def test_overrun_survives_a_throttled_general_error_and_is_summarised(udp, capfd):
    node = make_node(udp.getsockname()[1])
    try:
        node._log_error("fixture general error")      # occupies the error site
        node._runtime._record_processing_time(0.6)      # within the same 5 s
        node._runtime._record_processing_time(0.02)     # recovery
        err = capfd.readouterr().err
        assert "exceeds the 100 ms tick" in err
        assert node.processing_summary() == (
            "terrain processing summary: max 600.00 ms, overruns 1")
    finally:
        node.destroy_node()
    assert "terrain processing summary: max 600.00 ms, overruns 1" in capfd.readouterr().err


def test_depth_array_checks_layout():
    good = image(np.zeros((4, 6), np.uint16))
    assert depth_array(good).shape == (4, 6)
    bad = image(np.zeros((4, 6), np.uint16))
    bad.step = 14
    assert depth_array(bad) is None
    big = image(np.zeros((4, 6), np.uint16))
    big.is_bigendian = 1
    assert depth_array(big) is None
