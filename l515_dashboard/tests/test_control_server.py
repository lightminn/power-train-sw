import json
import os
import socket
import threading
import uuid

from l515_dashboard.control_server import DeferredResponse, UnixControlServer
from l515_dashboard.endpoint import abstract_address


def endpoint():
    return "@test-l515-" + uuid.uuid4().hex


def wire(request_id, kind="get_status", payload=None):
    return (json.dumps({"protocol_version": 1, "request_id": request_id,
                        "type": kind, "payload": payload or {}}) + "\n").encode()


def connect(name):
    client = socket.socket(socket.AF_UNIX)
    client.connect(abstract_address(name))
    return client


def test_unauthorized_peer_is_rejected_before_handler():
    name = endpoint(); calls = []
    server = UnixControlServer(name, calls.append,
                               peer_authorizer=lambda _client: False)
    server.start(); client = connect(name); client.settimeout(1)
    client.sendall(wire("denied"))
    try:
        assert client.recv(1) == b""
    except ConnectionResetError:
        pass
    assert calls == []
    client.close(); server.stop()


def test_default_authorizer_accepts_same_uid():
    name = endpoint()
    server = UnixControlServer(name, lambda _: {"ok": True}); server.start()
    client = connect(name); client.sendall(wire("same-uid"))
    assert json.loads(client.makefile().readline())["request_id"] == "same-uid"
    client.close(); server.stop()


def test_abstract_server_never_unlinks(monkeypatch):
    calls = []
    monkeypatch.setattr(os, "unlink", lambda path: calls.append(path))
    server = UnixControlServer(endpoint(), lambda _: {})
    server.start(); server.stop()
    assert calls == []


def test_partial_frames_multiple_clients_are_isolated():
    name = endpoint(); calls = []
    server = UnixControlServer(name, lambda message: calls.append(message) or {"ok": True},
                               max_message_bytes=256)
    server.start(); clients = [connect(name) for _ in range(2)]
    clients[0].sendall(wire("a")[:10]); clients[1].sendall(wire("b"))
    assert json.loads(clients[1].makefile().readline())["request_id"] == "b"
    clients[0].sendall(wire("a")[10:])
    assert json.loads(clients[0].makefile().readline())["request_id"] == "a"
    [client.close() for client in clients]; server.stop(); server.stop()
    assert {item["request_id"] for item in calls} == {"a", "b"}


def test_deferred_action_runs_after_ack():
    name = endpoint(); actions = []
    server = UnixControlServer(name, lambda _: DeferredResponse(
        {"accepted": True}, lambda: actions.append(True)))
    server.start(); client = connect(name); client.sendall(wire("x"))
    assert json.loads(client.makefile().readline())["payload"] == {"accepted": True}
    for _ in range(100):
        if actions: break
        threading.Event().wait(.005)
    assert actions == [True]
    client.close(); server.stop()


def test_deferred_actions_use_fifo_worker_and_reject_full_queue():
    name = endpoint(); entered = threading.Event(); release = threading.Event(); order = []
    def handler(request):
        def action():
            order.append(request["request_id"])
            if request["request_id"] == "1": entered.set(); release.wait()
        return DeferredResponse({"accepted": True}, action)
    server = UnixControlServer(name, handler, max_clients=1); server.start()
    client = connect(name); reader = client.makefile()
    client.sendall(wire("1")); assert json.loads(reader.readline())["type"] == "response"
    assert entered.wait(1)
    client.sendall(wire("2")); assert json.loads(reader.readline())["type"] == "response"
    client.sendall(wire("3")); assert json.loads(reader.readline())["type"] == "error"
    release.set()
    for _ in range(100):
        if order == ["1", "2"]: break
        threading.Event().wait(.005)
    assert order == ["1", "2"]
    client.close(); server.stop()


def test_stop_joins_blocked_handlers_and_cancels_actions():
    name = endpoint(); entered = []; lock = threading.Lock()
    both = threading.Event(); release = threading.Event(); actions = []
    def handler(request):
        with lock:
            entered.append(request["request_id"])
            if len(entered) == 2: both.set()
        release.wait()
        return DeferredResponse({"accepted": True}, lambda: actions.append(True))
    server = UnixControlServer(name, handler, max_clients=2); server.start()
    clients = [connect(name) for _ in range(2)]
    for index, client in enumerate(clients): client.sendall(wire(str(index)))
    assert both.wait(1)
    stopper = threading.Thread(target=server.stop); stopper.start()
    threading.Event().wait(.05); assert stopper.is_alive()
    release.set(); stopper.join(1)
    assert not stopper.is_alive() and actions == [] and server._clients == {}
    [client.close() for client in clients]


def test_action_failure_is_reported():
    name = endpoint(); failures = []
    def fail(): raise RuntimeError("action failed")
    server = UnixControlServer(name, lambda _: DeferredResponse({"accepted": True}, fail),
                               on_action_error=failures.append)
    server.start(); client = connect(name); client.sendall(wire("x"))
    assert json.loads(client.makefile().readline())["type"] == "response"
    for _ in range(100):
        if failures: break
        threading.Event().wait(.005)
    assert str(failures[0]) == server.last_action_error == "action failed"
    client.close(); server.stop()
