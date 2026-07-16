"""TCP 서버 견고성 — 클라이언트 RST가 accept 루프를 죽이면 안 된다.

2026-07-17 벤치 실증: 첫 원격 클라이언트를 강제 종료하자 ConnectionResetError가
``_serve_client``의 recv에서 ``_serve``까지 전파돼 서버 스레드가 죽었고, 이후
모든 재접속이 불가능했다(노드 재시작 전까지 원격 불능).
"""
import socket
import struct
import time

import pytest
import rclpy

from powertrain_ros.teleop_command_node import TeleopCommandNode


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _connect(port, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        try:
            return socket.create_connection(("127.0.0.1", port), timeout=1.0)
        except OSError:
            time.sleep(0.1)
    raise AssertionError("could not connect to teleop TCP server")


def _read_status(connection, node, timeout=3.0):
    connection.settimeout(0.2)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        try:
            data = connection.recv(256)
        except socket.timeout:
            continue
        if data:
            return data
    raise AssertionError("no status line from teleop TCP server")


def test_client_rst_does_not_kill_the_accept_loop():
    node = TeleopCommandNode()
    port = node._port
    try:
        first = _connect(port)
        assert _read_status(first, node).startswith(b"S ")
        # SO_LINGER 0 → close가 FIN이 아니라 RST를 보낸다(강제 종료 재현).
        first.setsockopt(
            socket.SOL_SOCKET,
            socket.SO_LINGER,
            struct.pack("ii", 1, 0),
        )
        first.close()
        time.sleep(0.3)

        second = _connect(port)
        assert _read_status(second, node).startswith(b"S ")
        second.close()
    finally:
        node.close()
        node.destroy_node()
