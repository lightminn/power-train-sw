import socket
import threading
import time
import uuid

import pytest

from l515_dashboard.client import GatewayClient, ClientState, StaleStatusError
from l515_dashboard.endpoint import abstract_address
from l515_dashboard.protocol import decode_request, encode_message, response


class Server:
    def __init__(self, path, replies):
        self.path, self.replies = str(path), list(replies)
        self.seen = []
        self.stop = threading.Event()
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.ready = threading.Event()

    def start(self):
        self.thread.start()
        assert self.ready.wait(1)
    def run(self):
        sock = socket.socket(socket.AF_UNIX); sock.bind(abstract_address(self.path)); sock.listen()
        self.ready.set()
        sock.settimeout(.05)
        while not self.stop.is_set():
            try: conn, _ = sock.accept()
            except socket.timeout: continue
            with conn:
                raw = conn.makefile("rb").readline().rstrip(b"\n")
                req = decode_request(raw, 65536); self.seen.append(req)
                reply = self.replies.pop(0)(req)
                conn.sendall(encode_message(reply, 65536))
        sock.close()
    def close(self): self.stop.set(); self.thread.join()


def endpoint(): return "@test-client-" + uuid.uuid4().hex


def test_command_returns_acknowledged_immutable_snapshot(tmp_path):
    server = Server(endpoint(), [lambda r: response(r["request_id"], {"state":"RUNNING", "nested":{"x":1}})])
    server.start(); client = GatewayClient(server.path, request_timeout_s=.5)
    snap = client.request("get_status")
    assert snap.payload["state"] == "RUNNING" and not snap.acknowledged
    with pytest.raises(TypeError): snap.payload["nested"]["x"] = 2
    server.close()


@pytest.mark.parametrize("payload", [{"accepted": False}, {"accepted": 1}, {}, {"accepted": "true"}])
def test_only_literal_true_is_acknowledged(tmp_path, payload):
    server=Server(endpoint(),[lambda r: response(r["request_id"],payload)])
    server.start(); snap=GatewayClient(server.path,request_timeout_s=.5).request("stop_gateway")
    assert snap.acknowledged is False
    server.close()


def test_literal_true_is_acknowledged(tmp_path):
    server=Server(endpoint(),[lambda r: response(r["request_id"],{"accepted":True})])
    server.start(); snap=GatewayClient(server.path,request_timeout_s=.5).request("stop_gateway")
    assert snap.acknowledged is True
    server.close()


def test_version_mismatch_disconnects(tmp_path):
    server = Server(endpoint(), [lambda r: {**response(r["request_id"], {}), "protocol_version":99}])
    server.start(); client = GatewayClient(server.path, request_timeout_s=.5)
    with pytest.raises(ValueError, match="protocol version"): client.request("get_status")
    assert client.state is ClientState.DISCONNECTED
    server.close()


def test_stale_status_is_rejected(tmp_path):
    server = Server(endpoint(), [lambda r: response("wrong-id", {"state":"RUNNING"})])
    server.start(); client = GatewayClient(server.path, request_timeout_s=.5)
    with pytest.raises(StaleStatusError): client.request("get_status")
    server.close()


def test_poll_reconnects_after_server_appears(tmp_path):
    path = endpoint(); client = GatewayClient(path, request_timeout_s=.1)
    assert client.poll() is None and client.state is ClientState.DISCONNECTED
    server = Server(path, [lambda r: response(r["request_id"], {"state":"RUNNING"})]); server.start()
    deadline=time.monotonic()+1
    while client.poll() is None and time.monotonic()<deadline: time.sleep(.01)
    assert client.snapshot.payload["state"] == "RUNNING"
    server.close()


def test_disconnect_during_response_is_reported(tmp_path):
    path=endpoint(); ready=threading.Event()
    def close_once():
        sock=socket.socket(socket.AF_UNIX); sock.bind(abstract_address(path)); sock.listen(); ready.set()
        conn,_=sock.accept(); conn.recv(4096); conn.close(); sock.close()
    thread=threading.Thread(target=close_once); thread.start(); ready.wait()
    client=GatewayClient(path,request_timeout_s=.5)
    with pytest.raises(ConnectionError,match="disconnected"): client.request("get_status")
    assert client.state is ClientState.DISCONNECTED
    thread.join()
