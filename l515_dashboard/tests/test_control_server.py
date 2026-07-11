import json
import socket
import threading

from l515_dashboard.control_server import UnixControlServer


def wire(request_id, kind="get_status", payload=None):
    return (json.dumps({"protocol_version": 1, "request_id": request_id,
                        "type": kind, "payload": payload or {}}) + "\n").encode()


def test_partial_frames_multiple_clients_and_disconnect_are_isolated(tmp_path):
    path = tmp_path / "gateway.sock"
    calls = []
    server = UnixControlServer(path, lambda message: calls.append(message) or {"ok": True}, max_message_bytes=256)
    server.start()
    clients = [socket.socket(socket.AF_UNIX) for _ in range(2)]
    for client in clients: client.connect(str(path))
    clients[0].sendall(wire("a")[:10]); clients[1].sendall(wire("b"))
    assert json.loads(clients[1].makefile().readline())["request_id"] == "b"
    clients[0].sendall(wire("a")[10:])
    assert json.loads(clients[0].makefile().readline())["request_id"] == "a"
    clients[0].close(); clients[1].close(); server.stop(); server.stop()
    assert not path.exists()
    assert {x["request_id"] for x in calls} == {"a", "b"}


def test_slow_client_does_not_block_other_client(tmp_path):
    path = tmp_path / "gateway.sock"
    server = UnixControlServer(path, lambda _: {"blob": "x" * 1000}, max_message_bytes=2048)
    server.start()
    slow = socket.socket(socket.AF_UNIX); slow.connect(str(path))
    for i in range(20): slow.sendall(wire(str(i)))
    fast = socket.socket(socket.AF_UNIX); fast.settimeout(1); fast.connect(str(path)); fast.sendall(wire("fast"))
    assert json.loads(fast.makefile().readline())["request_id"] == "fast"
    slow.close(); fast.close(); server.stop()

