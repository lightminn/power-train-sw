from types import SimpleNamespace

import numpy as np
import pytest
import rclpy
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    ReliabilityPolicy,
)

from powertrain_ros.l515_source import LatestFrames


class FakeMapper:
    def map_ms(self, device_ms, ros_now_ns):
        return int(device_ms * 1_000_000) + 7_000_000_000


class FakeVideoFrame:
    def __init__(self, data, timestamp_ms):
        self._data = data
        self._timestamp_ms = timestamp_ms
        self.profile = SimpleNamespace(
            as_video_stream_profile=lambda: SimpleNamespace(
                intrinsics=SimpleNamespace(
                    width=data.shape[1], height=data.shape[0],
                    fx=100.0, fy=101.0, ppx=1.0, ppy=2.0,
                    model="brown_conrady", coeffs=[0.0] * 5,
                )
            )
        )

    def get_data(self):
        return self._data

    def get_timestamp(self):
        return self._timestamp_ms


class FakeMotionFrame:
    def __init__(self, timestamp_ms, vector):
        self._timestamp_ms = timestamp_ms
        self._vector = vector

    def get_timestamp(self):
        return self._timestamp_ms

    def as_motion_frame(self):
        return self

    def get_motion_data(self):
        return self._vector


class FakeSource:
    def __init__(self, frames):
        self.frames = frames
        self.started = False
        self.stopped = False
        self.polls = 0

    def start(self):
        self.started = True

    def stop(self):
        self.stopped = True

    def poll_latest(self):
        self.polls += 1
        frames, self.frames = self.frames, LatestFrames()
        return frames


class FailingStartSource(FakeSource):
    def start(self):
        self.started = True
        raise RuntimeError("start failed")


class FailingStopSource(FakeSource):
    def stop(self):
        self.stopped = True
        raise RuntimeError("stop failed")


@pytest.fixture
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()


def test_timer_nonblocking_drains_once_and_publishes_exact_contract(ros_context):
    from powertrain_ros.l515_node import L515Node

    mapper = FakeMapper()
    color = FakeVideoFrame(np.zeros((2, 3, 3), dtype=np.uint8), 10.0)
    depth = FakeVideoFrame(np.zeros((2, 3), dtype=np.uint16), 11.0)
    gyro = FakeMotionFrame(12.0, SimpleNamespace(x=1, y=2, z=3))
    accel = FakeMotionFrame(13.0, SimpleNamespace(x=4, y=5, z=6))
    source = FakeSource(LatestFrames(
        color=color, depth=depth, gyro=gyro, accel=accel,
        timestamp_mapper=mapper,
    ))
    node = L515Node(source=source)
    published = {topic: [] for topic in node.stream_publishers}
    for topic, publisher in node.stream_publishers.items():
        publisher.publish = published[topic].append

    node._drain_source()

    assert source.polls == 1
    assert node.timer.timer_period_ns == 5_000_000
    assert all(
        pub.qos_profile.depth == 5
        for pub in node.stream_publishers.values()
    )
    assert all(
        pub.qos_profile.reliability == ReliabilityPolicy.BEST_EFFORT
        for pub in node.stream_publishers.values()
    )
    assert all(
        pub.qos_profile.history == HistoryPolicy.KEEP_LAST
        for pub in node.stream_publishers.values()
    )
    assert all(
        pub.qos_profile.durability == DurabilityPolicy.VOLATILE
        for pub in node.stream_publishers.values()
    )
    assert all(len(messages) == 1 for messages in published.values())
    color_msg = published["/l515/color/image_raw"][0]
    color_info = published["/l515/color/camera_info"][0]
    depth_msg = published["/l515/depth/image_rect_raw"][0]
    depth_info = published["/l515/depth/camera_info"][0]
    assert color_msg.header.stamp == color_info.header.stamp
    assert depth_msg.header.stamp == depth_info.header.stamp
    assert color_msg.header.frame_id == "l515_color_optical_frame"
    assert color_info.header.frame_id == "l515_color_optical_frame"
    assert depth_msg.header.frame_id == "l515_depth_optical_frame"
    assert depth_info.header.frame_id == "l515_depth_optical_frame"
    assert published["/l515/gyro/sample"][0].header.frame_id == "l515_gyro_frame"
    assert published["/l515/accel/sample"][0].header.frame_id == "l515_accel_frame"
    node.destroy_node()


def test_empty_drain_publishes_nothing_and_shutdown_stops_source(ros_context):
    from powertrain_ros.l515_node import L515Node

    source = FakeSource(LatestFrames())
    node = L515Node(source=source)
    assert source.started

    node._drain_source()
    node.destroy_node()

    assert source.polls == 1
    assert source.stopped


def test_node_name_and_registered_timer_are_exact_poll_only_path(ros_context):
    from powertrain_ros.l515_node import L515Node

    source = FakeSource(LatestFrames())
    node = L515Node(source=source)

    assert node.get_name() == "l515_camera_node"
    assert node.timer.callback.__self__ is node
    assert node.timer.callback.__func__ is L515Node._drain_source
    assert source.started
    assert not source.stopped

    node.timer.callback()

    assert source.polls == 1
    assert source.started
    assert not source.stopped
    node.destroy_node()


def test_constructor_start_failure_stops_source_and_destroys_partial_node():
    from powertrain_ros.l515_node import L515Node

    rclpy.init()
    source = FailingStartSource(LatestFrames())
    try:
        with pytest.raises(RuntimeError, match="start failed"):
            L515Node(source=source)
        assert source.stopped
    finally:
        rclpy.shutdown()


def test_intermediate_publisher_failure_cleans_real_partial_node(
        monkeypatch):
    from rclpy.node import Node
    from powertrain_ros.l515_node import L515Node

    rclpy.init()
    source = FakeSource(LatestFrames())
    create_publisher = L515Node.create_publisher
    destroyed = []
    publisher_calls = 0

    def fail_third_publisher(self, *args, **kwargs):
        nonlocal publisher_calls
        publisher_calls += 1
        if publisher_calls == 3:
            raise RuntimeError("publisher failed")
        return create_publisher(self, *args, **kwargs)

    base_destroy = Node.destroy_node

    def record_base_destroy(self):
        destroyed.append((self, self.get_name()))
        return base_destroy(self)

    monkeypatch.setattr(L515Node, "create_publisher", fail_third_publisher)
    monkeypatch.setattr(Node, "destroy_node", record_base_destroy)
    try:
        with pytest.raises(RuntimeError, match="publisher failed"):
            L515Node(source=source)
        assert publisher_calls == 3
        assert source.stopped
        assert len(destroyed) == 1
        assert destroyed[0][1] == "l515_camera_node"
    finally:
        rclpy.shutdown()


def test_destroy_node_destroys_ros_node_when_source_stop_raises(
        ros_context, monkeypatch):
    from rclpy.node import Node
    from powertrain_ros.l515_node import L515Node

    source = FailingStopSource(LatestFrames())
    node = L515Node(source=source)
    destroyed = []
    base_destroy = Node.destroy_node

    def record_base_destroy(self):
        destroyed.append(self)
        return base_destroy(self)

    monkeypatch.setattr(Node, "destroy_node", record_base_destroy)

    with pytest.raises(RuntimeError, match="stop failed"):
        node.destroy_node()

    assert source.stopped
    assert destroyed == [node]


def test_main_constructor_exception_always_shuts_down_rclpy(monkeypatch):
    import powertrain_ros.l515_node as module

    calls = []
    monkeypatch.setattr(module.rclpy, "init", lambda args=None: calls.append("init"))
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: calls.append("shutdown"))

    def fail_constructor():
        raise RuntimeError("constructor failed")

    monkeypatch.setattr(module, "L515Node", fail_constructor)

    with pytest.raises(RuntimeError, match="constructor failed"):
        module.main()
    assert calls == ["init", "shutdown"]


def test_main_spin_exception_destroys_node_then_shuts_down(monkeypatch):
    import powertrain_ros.l515_node as module

    calls = []
    fake_node = SimpleNamespace(
        destroy_node=lambda: calls.append("destroy")
    )
    monkeypatch.setattr(module.rclpy, "init", lambda args=None: calls.append("init"))
    monkeypatch.setattr(module.rclpy, "shutdown", lambda: calls.append("shutdown"))
    monkeypatch.setattr(module, "L515Node", lambda: fake_node)

    def fail_spin(node):
        assert node is fake_node
        calls.append("spin")
        raise RuntimeError("spin failed")

    monkeypatch.setattr(module.rclpy, "spin", fail_spin)

    with pytest.raises(RuntimeError, match="spin failed"):
        module.main()
    assert calls == ["init", "spin", "destroy", "shutdown"]
