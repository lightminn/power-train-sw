"""Per-connection ``G <input_epoch> <estop_ack_id>`` status replies."""

import json
import socket
import threading
import time
import uuid

from test_teleop_command_resilience import _StopEvent, _extract_methods

from powertrain_ros.remote_input import RemoteInputDecoder

_METHODS = _extract_methods("_serve_client", "_status_reply")


class _Harness:
    _serve_client = _METHODS["_serve_client"]
    _status_reply = _METHODS["_status_reply"]

    def __init__(self, epoch):
        self._stop_event = _StopEvent()
        self._decoder = RemoteInputDecoder()
        self._status_lock = threading.Lock()
        self._status_line = b"S DRIVE +0.000 +0.000\n"
        self._status_epoch = epoch
        self.results = []

    def _queue_lifecycle_event(self, event, session_id):
        pass

    def _queue_decoder_results(self, results):
        self.results.extend(results)


def _record(sequence, *, estop_id=0, session_id):
    payload = {
        "schema_version": 3, "session_id": session_id, "sequence": sequence,
        "client_monotonic_ns": 1, "mode": "DRIVE", "deadman": False,
        "axes": {"left_x": 0.0, "right_y": 0.0, "left_trigger": 0.0,
                 "right_trigger": 0.0},
        "dpad": {"x": 0, "y": 0}, "mode_chord": False,
        "estop_edge": bool(estop_id), "assist_bypass": False,
        "input_epoch": 0, "estop_id": estop_id,
    }
    return (json.dumps(payload) + "\n").encode()


def _g_lines(client, until):
    client.settimeout(0.05)
    data = b""
    deadline = time.monotonic() + 2.0
    while time.monotonic() < deadline:
        try:
            data += client.recv(4096)
        except socket.timeout:
            pass
        lines = [l for l in data.decode().splitlines() if l.startswith("G ")]
        if lines and until(lines):
            return lines
    raise AssertionError("no matching G line: %r" % data)


def _serve(harness):
    listener = socket.create_server(("127.0.0.1", 0))
    client = socket.create_connection(listener.getsockname())
    server, _ = listener.accept()
    listener.close()
    thread = threading.Thread(target=harness._serve_client, args=(server,), daemon=True)
    thread.start()
    return client, thread


def test_status_reply_carries_epoch_and_acks_this_connections_estop():
    harness = _Harness(epoch=4)
    client, thread = _serve(harness)
    session = str(uuid.uuid4())
    client.sendall(_record(0, session_id=session))
    assert _g_lines(client, lambda l: True)[0] == "G 4 0"
    client.sendall(_record(1, estop_id=12345, session_id=session))
    _g_lines(client, lambda l: l[-1] == "G 4 12345")
    harness._stop_event.set()
    client.close()
    thread.join(2)


def test_a_new_connection_does_not_inherit_the_previous_ack():
    harness = _Harness(epoch=1)
    client, thread = _serve(harness)
    client.sendall(_record(0, estop_id=99, session_id=str(uuid.uuid4())))
    _g_lines(client, lambda l: l[-1] == "G 1 99")
    client.close()
    thread.join(2)
    client, thread = _serve(harness)
    client.sendall(_record(0, session_id=str(uuid.uuid4())))
    assert _g_lines(client, lambda l: True)[0] == "G 1 0"
    harness._stop_event.set()
    client.close()
    thread.join(2)
