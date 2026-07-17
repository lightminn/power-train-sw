"""○ E-stop 전역 latch 정합(스펙 r6 §2.1) — /teleop/estop latched 발행."""
import json
import time
import uuid

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from powertrain_ros import teleop_command_node
from powertrain_ros.remote_input import (
    DPad,
    NormalizedAxes,
    ParseResult,
    RemoteInputFrame,
)
from powertrain_ros.teleop_command_node import TeleopCommandNode


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture(autouse=True)
def _isolated_port(monkeypatch):
    """라이브 :9000 점유(powertrain_control)와의 충돌 격리 — 에페메랄 포트."""
    import socket

    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    monkeypatch.setattr(teleop_command_node, "DEFAULT_PORT", port)


def _estop_frame(sequence=0):
    return RemoteInputFrame(
        schema_version=2,
        session_id=str(uuid.uuid4()),
        sequence=sequence,
        client_monotonic_ns=0,
        mode="DRIVE",
        deadman=False,
        axes=NormalizedAxes(
            left_x=0.0, right_y=0.0, left_trigger=0.0, right_trigger=0.0
        ),
        dpad=DPad(x=0, y=0),
        mode_chord=False,
        estop_edge=True,
        assist_bypass=False,
        received_monotonic_s=time.monotonic(),
    )


def _latched_listener(received):
    listener = Node("estop_probe_%s" % uuid.uuid4().hex[:8])
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    listener.create_subscription(
        String, "/teleop/estop", lambda m: received.append(m.data), qos
    )
    return listener


def test_estop_edge_publishes_latched_event_visible_to_late_joiner():
    node = TeleopCommandNode()
    try:
        node._queue_decoder_results([ParseResult(frame=_estop_frame())])
        assert node._estop_event is not None
        node._tick()

        # 발행 **이후** 구독을 만들어도(late join) TRANSIENT_LOCAL 로 수신돼야 한다
        # — 구독자(chassis_node) 재시작 창에서도 latch 유실 없음(스펙 §2.1).
        received = []
        listener = _latched_listener(received)
        try:
            deadline = time.monotonic() + 3.0
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=0.05)
                rclpy.spin_once(node, timeout_sec=0.0)
            assert received, (
                "late-joining subscriber did not get latched estop"
            )
            payload = json.loads(received[0])
            assert set(payload) == {"event_id", "stamp_s"}
            assert isinstance(payload["event_id"], str) and payload["event_id"]
            assert isinstance(payload["stamp_s"], float)
        finally:
            listener.destroy_node()
    finally:
        node.close()
        node.destroy_node()


def test_rebroadcast_reuses_same_event_id_within_window():
    node = TeleopCommandNode()
    try:
        received = []
        listener = _latched_listener(received)
        try:
            node._queue_decoder_results([ParseResult(frame=_estop_frame())])
            node._tick()
            node._tick()
            deadline = time.monotonic() + 3.0
            while len(received) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=0.05)
                rclpy.spin_once(node, timeout_sec=0.0)
            assert len(received) >= 2
            ids = {json.loads(item)["event_id"] for item in received}
            assert len(ids) == 1, "rebroadcast must reuse the same event_id"
        finally:
            listener.destroy_node()
    finally:
        node.close()
        node.destroy_node()


def test_rebroadcast_stops_after_window(monkeypatch):
    from powertrain_ros import teleop_command_node as module

    monkeypatch.setattr(module, "ESTOP_REBROADCAST_S", 0.0)
    node = TeleopCommandNode()
    try:
        node._queue_decoder_results([ParseResult(frame=_estop_frame())])
        node._tick()          # edge 시점 1회 발행 후, 창(0초) 만료로 즉시 정리
        assert node._estop_event is None
    finally:
        node.close()
        node.destroy_node()
