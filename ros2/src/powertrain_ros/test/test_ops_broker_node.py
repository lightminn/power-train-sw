"""ops broker 노드 E2E — 인증·프록시·push·부분 성공 (스펙 r6 §3.1)."""
from contextlib import contextmanager
import json
import socket
import threading
import time

import pytest
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger

from powertrain_ros.ops_broker_node import OpsBrokerNode


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture()
def token_dir(tmp_path):
    (tmp_path / "ops_console.token").write_text(
        "tok-console-test\n", encoding="utf-8"
    )
    (tmp_path / "ops_controller.token").write_text(
        "tok-ctrl-test\n", encoding="utf-8"
    )
    return tmp_path


class FakeServices(Node):
    """chassis/teleop 서비스 대역 — 호출 기록 + 지연 주입."""

    def __init__(self):
        super().__init__("fake_targets")
        self.calls = []
        self.fail = set()
        self.delay_s = {}
        for name in (
            "/chassis_node/authority_manual",
            "/chassis_node/reset_estop",
            "/chassis_node/arm",
            "/teleop_command/clear_hold",
            "/chassis_node/authority_clear_hold",
        ):
            self.create_service(
                Trigger,
                name,
                lambda request, response, n=name: self._serve(n, response),
            )
        self.create_service(
            SetBool,
            "/chassis_node/arm_lock_override",
            lambda request, response: self._serve(
                "/chassis_node/arm_lock_override", response
            ),
        )

    def _serve(self, name, response):
        self.calls.append(name)
        delay_s = float(self.delay_s.get(name, 0.0))
        if delay_s > 0.0:
            time.sleep(delay_s)
        response.success = name not in self.fail
        response.message = "fake"
        return response


class StateFeeder(Node):
    """authority_manual 전이표를 만족하는 fresh 상태를 계속 공급한다."""

    def __init__(self):
        super().__init__("ops_state_feeder")
        self._authority_pub = self.create_publisher(
            String, "/command_authority/state", 10
        )
        self._gateway_pub = self.create_publisher(
            String, "/teleop/gateway_state", 10
        )
        self.create_timer(0.05, self.publish)

    def publish(self):
        self._authority_pub.publish(String(data="IDLE|ok"))
        self._gateway_pub.publish(
            String(
                data=json.dumps(
                    {
                        "state": "DRIVE",
                        "input_fresh": True,
                        "neutral": True,
                        "stamp_s": time.monotonic(),
                    },
                    separators=(",", ":"),
                )
            )
        )


class SocketReader:
    def __init__(self, sock):
        self.sock = sock
        self.buffer = b""

    def read_for(self, nodes, duration_s, keep_fresh=None):
        lines = []
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            # 느린 호스트(젯슨)에서 상태 신선도(0.5 s)가 루프 중 만료되지 않게
            # 매 반복 재발행한다 — 0cff49e "stream-shaped" 교훈과 동일.
            if keep_fresh is not None:
                keep_fresh()
            for node in nodes:
                rclpy.spin_once(node, timeout_sec=0.002)
            try:
                chunk = self.sock.recv(4096)
            except socket.timeout:
                continue
            if not chunk:
                break
            self.buffer += chunk
            while b"\n" in self.buffer:
                line, _, self.buffer = self.buffer.partition(b"\n")
                lines.append(json.loads(line))
        return lines

    def read_until(self, nodes, predicate, timeout_s=3.0, keep_fresh=None):
        lines = []
        deadline = time.monotonic() + timeout_s
        while not predicate(lines) and time.monotonic() < deadline:
            lines.extend(self.read_for(nodes, 0.05, keep_fresh=keep_fresh))
        return lines


def _free_port():
    probe = socket.socket()
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def _node(token_dir, port):
    return OpsBrokerNode(
        parameter_overrides=None,
        port_override=port,
        token_dir_override=str(token_dir),
    )


def _client(port, token):
    sock = socket.create_connection(("127.0.0.1", port), timeout=2.0)
    sock.settimeout(0.02)
    sock.sendall(
        (
            json.dumps(
                {"schema_version": 1, "hello": True, "token": token}
            )
            + "\n"
        ).encode()
    )
    return sock, SocketReader(sock)


def _request(token, action, request_id="r-1", sequence=0, **extra):
    payload = {
        "schema_version": 1,
        "token": token,
        "request_id": request_id,
        "sequence": sequence,
        "action": action,
        "params": {},
        "stamp_s": time.monotonic(),
    }
    payload.update(extra)
    return (json.dumps(payload) + "\n").encode()


def _has_request(lines, request_id, status=None):
    return any(
        item.get("request_id") == request_id
        and (status is None or item.get("status") == status)
        for item in lines
    )


def _hello(reader, nodes):
    return reader.read_until(
        nodes,
        lambda lines: _has_request(lines, "hello"),
    )


@contextmanager
def _spinning(*nodes):
    executor = MultiThreadedExecutor(num_threads=4)
    for node in nodes:
        executor.add_node(node)
    thread = threading.Thread(target=executor.spin, daemon=True)
    thread.start()
    try:
        yield
    finally:
        executor.shutdown(timeout_sec=2.0)
        thread.join(timeout=2.0)


def test_handshake_and_role_binding(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    try:
        sock, reader = _client(port, "tok-console-test")
        hello = [
            item for item in _hello(reader, [node])
            if item.get("request_id") == "hello"
        ][0]
        assert hello["status"] == "FINAL_SUCCESS"
        assert "role=console" in hello["detail"]
        sock.close()

        bad, bad_reader = _client(port, "nope")
        rejected = [
            item for item in _hello(bad_reader, [node])
            if item.get("request_id") == "hello"
        ][0]
        assert rejected["status"] == "FINAL_REJECTED"
        bad.close()
    finally:
        node.close()
        node.destroy_node()


def test_authority_manual_round_trip_calls_target_service(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    targets = FakeServices()
    feeder = StateFeeder()
    try:
        sock, reader = _client(port, "tok-console-test")
        _hello(reader, [node, targets, feeder])
        feeder.publish()
        reader.read_for([node, targets, feeder], 0.25)

        sock.sendall(_request("tok-console-test", "authority_manual"))
        replies = reader.read_until(
            [node, targets, feeder],
            lambda lines: (
                _has_request(lines, "r-1", "PENDING")
                and _has_request(lines, "r-1", "FINAL_SUCCESS")
            ),
            keep_fresh=feeder.publish,
        )
        statuses = [
            item["status"] for item in replies
            if item.get("request_id") == "r-1"
        ]
        assert "PENDING" in statuses
        assert "FINAL_SUCCESS" in statuses
        assert "/chassis_node/authority_manual" in targets.calls
        sock.close()
    finally:
        node.close()
        node.destroy_node()
        feeder.destroy_node()
        targets.destroy_node()


def test_arm_lock_override_round_trip_and_param_validation(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    targets = FakeServices()
    try:
        sock, reader = _client(port, "tok-console-test")
        _hello(reader, [node, targets])
        sock.sendall(_request(
            "tok-console-test", "arm_lock_override",
            params={"data": True},
        ))
        replies = reader.read_until(
            [node, targets],
            lambda lines: _has_request(lines, "r-1", "FINAL_SUCCESS"),
        )
        assert "/chassis_node/arm_lock_override" in targets.calls

        sock.sendall(_request(
            "tok-console-test", "arm_lock_override", request_id="r-2",
            sequence=1, params={},
        ))
        replies = reader.read_until(
            [node, targets],
            lambda lines: _has_request(lines, "r-2", "FINAL_REJECTED"),
        )
        sock.close()
    finally:
        node.close(); node.destroy_node(); targets.destroy_node()


def test_composite_clear_reports_partial_results(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    targets = FakeServices()
    targets.fail.add("/chassis_node/authority_clear_hold")
    try:
        sock, reader = _client(port, "tok-console-test")
        _hello(reader, [node, targets])
        sock.sendall(_request("tok-console-test", "clear_transient_hold"))
        replies = reader.read_until(
            [node, targets],
            lambda lines: _has_request(lines, "r-1", "FINAL_REJECTED"),
        )
        final = [
            item for item in replies
            if item.get("status") == "FINAL_REJECTED"
            and item.get("request_id") == "r-1"
        ]
        assert final and "authority_clear_hold" in final[0]["detail"]
        assert "teleop=" in final[0]["detail"]
        assert "chassis=" in final[0]["detail"]
        sock.close()
    finally:
        node.close()
        node.destroy_node()
        targets.destroy_node()


def test_ops_state_push_arrives_with_revision(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    publisher = Node("state_feeder")
    authority_pub = publisher.create_publisher(
        String, "/command_authority/state", 10
    )
    try:
        sock, reader = _client(port, "tok-ctrl-test")
        _hello(reader, [node, publisher])
        authority_pub.publish(String(data="IDLE|ok"))
        pushes = reader.read_until(
            [node, publisher],
            lambda lines: any(
                item.get("push") == "ops_state" for item in lines
            ),
        )
        pushes = [item for item in pushes if item.get("push") == "ops_state"]
        assert pushes
        assert pushes[0]["authority_mode"] in ("IDLE", "UNKNOWN")
        assert isinstance(pushes[0]["revision"], int)
        sock.close()
    finally:
        node.close()
        node.destroy_node()
        publisher.destroy_node()


def test_controller_direct_estop_reset_is_rejected(token_dir):
    port = _free_port()
    node = _node(token_dir, port)
    try:
        sock, reader = _client(port, "tok-ctrl-test")
        _hello(reader, [node])
        sock.sendall(_request("tok-ctrl-test", "estop_reset"))
        replies = reader.read_until(
            [node],
            lambda lines: _has_request(lines, "r-1"),
        )
        reply = [
            item for item in replies if item.get("request_id") == "r-1"
        ][0]
        assert reply["status"] == "FINAL_REJECTED"
        sock.close()
    finally:
        node.close()
        node.destroy_node()


def test_service_timeout_stays_pending_then_late_completion_pushes_final(
    token_dir,
):
    port = _free_port()
    node = _node(token_dir, port)
    targets = FakeServices()
    targets.delay_s["/chassis_node/reset_estop"] = 1.3
    try:
        with _spinning(node, targets):
            sock, reader = _client(port, "tok-console-test")
            _hello(reader, [])
            sock.sendall(_request("tok-console-test", "estop_reset"))

            before_late_completion = reader.read_for([], 1.1)
            assert _has_request(before_late_completion, "r-1", "PENDING")
            assert not _has_request(
                before_late_completion, "r-1", "FINAL_SUCCESS"
            )
            assert not _has_request(
                before_late_completion, "r-1", "FINAL_REJECTED"
            )

            late = reader.read_until(
                [],
                lambda lines: _has_request(lines, "r-1", "FINAL_SUCCESS"),
                timeout_s=2.0,
            )
            assert _has_request(late, "r-1", "FINAL_SUCCESS")
            assert "/chassis_node/reset_estop" in targets.calls
            sock.close()
    finally:
        node.close()
        node.destroy_node()
        targets.destroy_node()
