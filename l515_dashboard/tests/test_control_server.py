import json
import socket
import threading
import os
import pytest

from l515_dashboard.control_server import DeferredResponse, UnixControlServer


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


def test_unknown_existing_socket_path_is_preserved(tmp_path):
    path = tmp_path / "gateway.sock"; path.write_text("unknown")
    server = UnixControlServer(path, lambda _: {})
    with pytest.raises(OSError): server.start()
    assert path.read_text() == "unknown"


def test_socket_mode_and_deferred_action_runs_after_ack(tmp_path):
    path = tmp_path / "gateway.sock"; action_read=[]
    server = UnixControlServer(path, lambda _: DeferredResponse(
        {"accepted": True}, lambda: action_read.append(True)))
    server.start()
    assert os.stat(path).st_mode & 0o777 == 0o660
    client=socket.socket(socket.AF_UNIX); client.connect(str(path)); client.sendall(wire("x"))
    response=json.loads(client.makefile().readline())
    assert response["payload"] == {"accepted": True}
    for _ in range(100):
        if action_read: break
        threading.Event().wait(.005)
    assert action_read == [True]
    client.close(); server.stop()


def test_invalid_command_response_keeps_request_id(tmp_path):
    path=tmp_path / "gateway.sock"; server=UnixControlServer(path, lambda _: {})
    server.start(); client=socket.socket(socket.AF_UNIX); client.connect(str(path))
    client.sendall(wire("known", "bogus"))
    assert json.loads(client.makefile().readline())["request_id"] == "known"
    client.close(); server.stop()


def test_client_count_idle_deadline_and_stop_join_are_bounded(tmp_path):
    path=tmp_path / "gateway.sock"
    server=UnixControlServer(path, lambda _: {}, max_clients=1, idle_timeout_s=.05)
    server.start(); idle=socket.socket(socket.AF_UNIX); idle.connect(str(path))
    for _ in range(100):
        with server._lock:
            threads=list(server._clients.values())
        if threads: break
        threading.Event().wait(.002)
    assert len(threads) == 1
    threads[0].join(.5)
    assert not threads[0].is_alive()
    server.stop()
