"""Real TCP operation with production auth, ops protocol and broker policy.

Only the ROS service/hardware execution boundary and physical pad are fixtures.
"""
from dataclasses import asdict, replace
import json
import os
from pathlib import Path
import select
import signal
import socket
import socketserver
import subprocess
import sys
import threading
import time

from motor_control.laptop.ops_channel_client import OpsChannelClient
from operator_console.operation_runtime import OperationRuntime
from powertrain_ros.ops_broker_core import OpsBrokerCore, OpsState
from powertrain_runtime.drive_service import ManualDriveService
from powertrain_runtime.session import SessionServer
from powertrain_runtime.client import SessionClient


def wait_for(predicate, timeout=4):
    end = time.monotonic() + timeout
    while time.monotonic() < end:
        if predicate():
            return
        time.sleep(.01)
    assert predicate()


class FakePadBoundary:
    def __init__(self):
        self.connection = None
    def set_connection(self, value):
        self.connection = value
    def set_ops_state(self, _state):
        pass
    def snapshot(self):
        return dict(pad_connected=True, neutral=True, input_connected=self.connection is not None, age_s=0)
    def close(self):
        self.connection = None


class BrokerHandler(socketserver.BaseRequestHandler):
    def handle(self):
        server = self.server
        key, role, buffer = str(id(self)), None, bytearray()
        self.request.settimeout(.2)
        try:
            while not server.done.is_set():
                if select.select([self.request], [], [], .03)[0]:
                    data = self.request.recv(8192)
                    if not data:
                        break
                    buffer.extend(data)
                    while b"\n" in buffer:
                        line, _, rest = buffer.partition(b"\n")
                        buffer[:] = rest
                        with server.lock:
                            if role is None:
                                role, reply = server.core.handshake(key, line)
                            else:
                                decision = server.core.handle_line(key, role, line.decode())
                                reply = decision.response
                                if decision.execute:
                                    order = decision.execute
                                    server.actions.append(order.action)
                                    changes = {"clear_transient_hold": dict(authority_mode="IDLE", gateway_state="DRIVE", gateway_input_fresh=True),
                                               "authority_manual": dict(authority_mode="TELEOP"),
                                               "arm": dict(chassis_mode="ARMED"),
                                               "disarm": dict(chassis_mode="IDLE"),
                                               "authority_idle": dict(authority_mode="IDLE")}
                                    if hasattr(server, "execute"):
                                        server.execute(order.action)
                                    else:
                                        server.state = replace(server.state, revision=server.state.revision+1, **changes.get(order.action, {}))
                                    reply = server.core.complete(order.pending_key, True, "fixture executed")
                            if reply:
                                self.request.sendall(reply)
                if role:
                    with server.lock:
                        payload = dict(asdict(server.state), push="ops_state", stamp_s=time.monotonic())
                    self.request.sendall(json.dumps(payload).encode() + b"\n")
        except OSError:
            pass
        finally:
            with server.lock:
                server.core.disconnect(key)


def test_actual_session_to_ops_core_hold_start_stop_and_reconnect(tmp_path):
    token = tmp_path / "console.token"
    token.write_text("integration-private-console-token")
    broker = socketserver.ThreadingTCPServer(("127.0.0.1", 0), BrokerHandler)
    broker.daemon_threads = True
    broker.lock = threading.Lock()
    broker.done = threading.Event()
    broker.actions = []
    broker.state = OpsState(revision=1, authority_mode="MOTION_HOLD", chassis_mode="IDLE",
        gateway_state="MOTION_HOLD", gateway_input_fresh=False, gateway_neutral=True,
        estop_latched=False, active_estop_sources=(), wheels_stopped=True,
        field_age_s=dict(authority=0., gateway=0., safety=0., wheels=0.))
    broker.core = OpsBrokerCore({token.read_text(): "console"}, clock=time.monotonic, state_provider=lambda: broker.state)
    threading.Thread(target=broker.serve_forever, daemon=True).start()
    drive = ManualDriveService(lambda: OpsChannelClient("127.0.0.1", broker.server_address[1], token.read_text()), timeout_s=1)
    config = dict(robot_id="fixture", token_file=str(token), host="127.0.0.1", session_port=0,
                  input_port=0, ops_port=0, ops_target_port=broker.server_address[1], input_target_port=broker.server_address[1],
                  destination_file=str(tmp_path / "session.json"), lease_timeout_s=1.)
    server = SessionServer(config, drive.start, drive.stop, drive.snapshot).start()
    runtime = OperationRuntime(dict(robot_id="fixture", token_file=str(token), hosts=["127.0.0.1"], session_port=server.session_port), supervisor=FakePadBoundary())
    try:
        wait_for(lambda: runtime.snapshot().get("connected") and runtime.snapshot().get("start_blocker", "waiting") is None)
        assert broker.actions == []
        runtime.set_front_live(True)
        assert runtime.begin_start()
        end = time.monotonic() + 3
        while time.monotonic() < end and drive.snapshot()["status"] != "SUCCEEDED":
            runtime.set_front_live(True)
            time.sleep(.03)
        assert drive.snapshot()["status"] == "SUCCEEDED", drive.snapshot()
        assert broker.actions == ["clear_transient_hold", "authority_manual", "arm"]
        assert broker.state.chassis_mode == "ARMED"
        runtime.stop()
        wait_for(lambda: drive.snapshot()["action"] == "stop" and drive.snapshot()["status"] == "SUCCEEDED")
        assert broker.actions[-2:] == ["disarm", "authority_idle"]
        assert broker.state.chassis_mode == "IDLE"
        assert "estop_reset" not in broker.actions
        before_generation = runtime.snapshot()["session_generation"]
        # Close the actual control socket; expiry and reconnect use real I/O.
        with server._lock:
            server._revoke()
        wait_for(lambda: runtime.snapshot()["session_generation"] > before_generation)
        assert not runtime.snapshot()["ready"]
        assert broker.actions.count("arm") == 1
    finally:
        runtime.close()
        server.close()
        drive.close()
        broker.done.set()
        broker.shutdown()
        broker.server_close()


def test_robot_service_module_starts_and_stops_through_actual_entrypoint(tmp_path):
    token = tmp_path / "console.token"
    token.write_text("entrypoint-fixture-console-token")
    broker = socketserver.ThreadingTCPServer(("127.0.0.1", 0), BrokerHandler)
    broker.daemon_threads = True
    broker.lock, broker.done, broker.actions = threading.Lock(), threading.Event(), []
    broker.state = OpsState(revision=1, authority_mode="IDLE", chassis_mode="IDLE",
        gateway_state="DRIVE", gateway_input_fresh=True, gateway_neutral=True,
        estop_latched=False, active_estop_sources=(), wheels_stopped=True,
        field_age_s=dict(authority=0., gateway=0., safety=0., wheels=0.))
    broker.core = OpsBrokerCore({token.read_text(): "console"}, clock=time.monotonic, state_provider=lambda: broker.state)
    threading.Thread(target=broker.serve_forever, daemon=True).start()
    reservations = [socket.socket() for _ in range(3)]
    for reserved in reservations:
        reserved.bind(("127.0.0.1", 0))
    ports = [s.getsockname()[1] for s in reservations]
    for reserved in reservations:
        reserved.close()
    config = dict(robot_id="entrypoint", token_file=str(token), host="127.0.0.1",
        session_port=ports[0], input_port=ports[1], ops_port=ports[2],
        input_target_port=broker.server_address[1], ops_target_port=broker.server_address[1],
        destination_file=str(tmp_path / "session.json"))
    config_path = tmp_path / "robot.json"
    config_path.write_text(json.dumps(config))
    root = Path(__file__).resolve().parents[2]
    child = subprocess.Popen([sys.executable, "-m", "powertrain_runtime.robot_service", "--config", str(config_path)],
        cwd=root, env=dict(os.environ, PYTHONPATH=str(root)), stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True)
    client = SessionClient(dict(config, hosts=["127.0.0.1"], timeout_s=.2))
    try:
        def connected():
            assert child.poll() is None
            try:
                return client.connect()["connected"]
            except OSError:
                return False
        wait_for(connected)
        assert broker.actions == []
        child.send_signal(signal.SIGTERM)
        out, err = child.communicate(timeout=8)
        assert child.returncode == 0, (out, err)
        assert "Stop outcome: SUCCEEDED" in out, out
        assert "Traceback" not in err
        assert broker.actions == ["disarm", "authority_idle"]
        assert not Path(config["destination_file"]).exists()
    finally:
        client.close()
        if child.poll() is None:
            child.kill()
            child.communicate(timeout=2)
        broker.done.set()
        broker.shutdown()
        broker.server_close()
