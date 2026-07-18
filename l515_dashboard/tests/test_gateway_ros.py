from types import SimpleNamespace
import threading
import time

import numpy as np

from l515_dashboard.gateway_ros import GatewayRosPublisher, TOPIC_SPECS
from l515_dashboard.gateway_source import GatewayFrames
from l515_dashboard.stream_buffer import StreamSample


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Node:
    def __init__(self):
        self.created = []

    def create_publisher(self, message_type, topic, qos):
        publisher = Publisher()
        self.created.append((message_type, topic, qos, publisher))
        return publisher


class Mapper:
    def __init__(self):
        self.calls = []

    def map_ms(self, device_ms, ros_now_ns, stream_key=None):
        self.calls.append((device_ms, ros_now_ns, stream_key))
        return int(device_ms * 1_000_000) + 9_000_000_000


class Video:
    def __init__(self, data, stamp):
        self.data, self.stamp = data, stamp
        self.profile = SimpleNamespace(
            as_video_stream_profile=lambda: SimpleNamespace(
                intrinsics=SimpleNamespace(
                    width=data.shape[1],
                    height=data.shape[0],
                    fx=10,
                    fy=11,
                    ppx=12,
                    ppy=13,
                    model="brown_conrady",
                    coeffs=[0] * 5,
                )
            )
        )

    def get_data(self):
        return self.data

    def get_timestamp(self):
        return self.stamp


class Motion:
    def __init__(self, stamp, xyz):
        self.stamp, self.xyz = stamp, xyz

    def get_timestamp(self):
        return self.stamp

    def as_motion_frame(self):
        return self

    def get_motion_data(self):
        return SimpleNamespace(x=self.xyz[0], y=self.xyz[1], z=self.xyz[2])


def dependencies():
    class Time:
        sec = 0
        nanosec = 0

    def image(data, encoding, frame_id, stamp):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=stamp, frame_id=frame_id),
            data=data,
            encoding=encoding,
        )

    def info(intrinsics, frame_id, stamp):
        return SimpleNamespace(
            header=SimpleNamespace(stamp=stamp, frame_id=frame_id),
            width=intrinsics.width,
            height=intrinsics.height,
        )

    def imu(vector, kind, frame_id, stamp):
        zero = SimpleNamespace(x=0.0, y=0.0, z=0.0)
        angular = (
            SimpleNamespace(x=vector.x, y=vector.y, z=vector.z)
            if kind == "gyro"
            else zero
        )
        accel = (
            SimpleNamespace(x=vector.x, y=vector.y, z=vector.z)
            if kind == "accel"
            else zero
        )
        return SimpleNamespace(
            header=SimpleNamespace(stamp=stamp, frame_id=frame_id),
            angular_velocity=angular,
            linear_acceleration=accel,
        )

    return Time, object(), (object,) * 6, image, info, imu


def test_publishes_only_six_topics_with_native_camera_profiles_and_imu():
    node, mapper = Node(), Mapper()
    gateway = GatewayRosPublisher(
        node, now_ns=lambda: 123, dependencies=dependencies()
    )
    frames = GatewayFrames(
        raw_color=Video(np.zeros((720, 1280, 3), np.uint8), 1),
        raw_depth=Video(np.zeros((480, 640), np.uint16), 2),
        aligned_depth=Video(np.zeros((720, 1280), np.uint16), 2),
        gyro=Motion(3, (1, 2, 3)),
        accel=Motion(4, (4, 5, 6)),
        mapper=mapper,
    )

    published = gateway.publish(frames)

    assert tuple(topic for _, topic, _, _ in node.created) == TOPIC_SPECS
    assert len(node.created) == 6
    messages = {
        topic: publisher.messages for _, topic, _, publisher in node.created
    }
    assert (
        messages["/l515/color/camera_info"][0].width,
        messages["/l515/color/camera_info"][0].height,
    ) == (1280, 720)
    assert (
        messages["/l515/depth/camera_info"][0].width,
        messages["/l515/depth/camera_info"][0].height,
    ) == (640, 480)
    assert messages["/l515/gyro/sample"][0].angular_velocity.x == 1
    assert messages["/l515/accel/sample"][0].linear_acceleration.z == 6
    assert all(len(value) == 1 for value in messages.values())
    assert set(published) == set(TOPIC_SPECS)
    assert gateway.publish_counts() == {topic: 1 for topic in TOPIC_SPECS}


def test_equal_device_timestamps_are_deduplicated_per_stream():
    node, mapper = Node(), Mapper()
    gateway = GatewayRosPublisher(
        node, now_ns=lambda: 10, dependencies=dependencies()
    )
    color = Video(np.zeros((720, 1280, 3), np.uint8), 5)
    first = gateway.publish(GatewayFrames(raw_color=color, mapper=mapper))
    second = gateway.publish(GatewayFrames(raw_color=color, mapper=mapper))

    messages = {
        topic: publisher.messages for _, topic, _, publisher in node.created
    }
    assert len(messages["/l515/color/image_raw"]) == 1
    assert len(messages["/l515/color/camera_info"]) == 1
    assert mapper.calls == [(5, 10, "/l515/color/image_raw")]
    assert first == ("/l515/color/image_raw", "/l515/color/camera_info")
    assert second == ()
    assert gateway.publish_counts()["/l515/color/image_raw"] == 1


def test_new_mapper_generation_resets_publisher_dedup():
    node = Node()
    gateway = GatewayRosPublisher(
        node, now_ns=lambda: 10, dependencies=dependencies()
    )
    color = Video(np.zeros((720, 1280, 3), np.uint8), 5)
    gateway.publish(GatewayFrames(raw_color=color, mapper=Mapper()))
    gateway.publish(GatewayFrames(raw_color=color, mapper=Mapper()))
    messages = {
        topic: publisher.messages for _, topic, _, publisher in node.created
    }
    assert len(messages["/l515/color/image_raw"]) == 2


def test_split_publish_methods_preserve_exact_topic_contract():
    node, mapper = Node(), Mapper()
    gateway = GatewayRosPublisher(node, now_ns=lambda: 10, dependencies=dependencies())
    color = StreamSample("color", 1, 1.0, 1,
                         Video(np.zeros((2, 3, 3), np.uint8), 1))
    depth = StreamSample("depth", 1, 2.0, 1,
                         Video(np.zeros((2, 3), np.uint16), 2))
    gyro = StreamSample("gyro", 1, 3.0, 1, Motion(3, (1, 2, 3)))

    assert gateway.publish_color(color, mapper) == (
        "/l515/color/image_raw", "/l515/color/camera_info")
    assert gateway.publish_depth(depth, mapper) == (
        "/l515/depth/image_rect_raw", "/l515/depth/camera_info")
    assert gateway.publish_imu("gyro", gyro, mapper) == ("/l515/gyro/sample",)


def test_timestamp_mapper_calls_are_serialized_across_stream_workers():
    active = 0; collisions = []
    class BlockingMapper(Mapper):
        def map_ms(self, *args, **kwargs):
            nonlocal active
            active += 1
            if active > 1: collisions.append(True)
            time.sleep(.02)
            active -= 1
            return 1
    gateway = GatewayRosPublisher(Node(), dependencies=dependencies())
    mapper = BlockingMapper()
    samples = [
        (gateway.publish_color, (StreamSample("color",1,1,1,Video(np.zeros((1,1,3),np.uint8),1)), mapper)),
        (gateway.publish_depth, (StreamSample("depth",1,2,1,Video(np.zeros((1,1),np.uint16),2)), mapper)),
        (gateway.publish_imu, ("gyro", StreamSample("gyro",1,3,1,Motion(3,(1,2,3))), mapper)),
        (gateway.publish_imu, ("accel", StreamSample("accel",1,4,1,Motion(4,(1,2,3))), mapper)),
    ]
    threads=[threading.Thread(target=fn,args=args) for fn,args in samples]
    for thread in threads: thread.start()
    for thread in threads: thread.join()
    assert collisions == []


# ── 정렬 depth (RGB-D SLAM 용, opt-in) ───────────────────────────────────

def test_aligned_depth_is_off_by_default(monkeypatch):
    """기본 꺼짐 — 기존 6토픽 계약도 카운트도 건드리지 않는다."""
    monkeypatch.delenv("L515_ALIGNED_DEPTH_ROS", raising=False)
    node = Node()
    gateway = GatewayRosPublisher(node, now_ns=lambda: 1, dependencies=dependencies())
    assert tuple(topic for _, topic, _, _ in node.created) == TOPIC_SPECS
    # 켜지 않았으면 발행 자체를 시도해도 아무것도 안 나간다.
    frame = Video(np.zeros((720, 1280), dtype=np.uint16), stamp=5.0)
    assert gateway.publish_aligned_depth(frame, Mapper()) == ()


def test_aligned_depth_publishes_in_color_frame(monkeypatch):
    """★ 요점은 **frame_id 와 intrinsics 가 color 것** 이라는 데 있다.

    원본 depth 는 640x480 / `l515_depth_optical_frame` 이라 color 픽셀과 대응하지 않는다.
    RTAB-Map 은 정렬된 depth 를 요구하므로, 여기서 나가는 depth 는 반드시 color 의
    광학 프레임·해상도여야 한다. 아니면 맵이 통째로 어긋난다.
    """
    monkeypatch.setenv("L515_ALIGNED_DEPTH_ROS", "1")
    node = Node()
    gateway = GatewayRosPublisher(node, now_ns=lambda: 1, dependencies=dependencies())

    topics = tuple(topic for _, topic, _, _ in node.created)
    assert topics[:len(TOPIC_SPECS)] == TOPIC_SPECS          # 기존 계약은 그대로
    assert "/l515/aligned_depth_to_color/image_raw" in topics
    assert "/l515/aligned_depth_to_color/camera_info" in topics

    frame = Video(np.zeros((720, 1280), dtype=np.uint16), stamp=5.0)
    published = gateway.publish_aligned_depth(frame, Mapper())
    assert set(published) == {"/l515/aligned_depth_to_color/image_raw",
                              "/l515/aligned_depth_to_color/camera_info"}

    image = dict((t, p) for _, t, _, p in node.created)[
        "/l515/aligned_depth_to_color/image_raw"].messages[0]
    assert image.header.frame_id == "l515_color_optical_frame"   # depth 아님!
    assert image.encoding == "16UC1"

    info = dict((t, p) for _, t, _, p in node.created)[
        "/l515/aligned_depth_to_color/camera_info"].messages[0]
    assert (info.width, info.height) == (1280, 720)              # color 해상도
    assert info.header.frame_id == "l515_color_optical_frame"
