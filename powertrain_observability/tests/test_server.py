import json
import os
import struct
import time
import uuid

import pytest


def modules():
    from powertrain_observability.client import EventClient, ObservabilityClient
    from powertrain_observability.protocol import abstract_address
    from powertrain_observability.server import DaemonAlreadyRunning, ObservabilityServer

    return EventClient, ObservabilityClient, abstract_address, DaemonAlreadyRunning, ObservabilityServer


def endpoint(kind):
    return f"@test-observability-{kind}-{os.getpid()}-{uuid.uuid4().hex}"


def pending_event(event_type="MISSION", payload=None):
    return {
        "schema_version": 1,
        "wall_time_ns": time.time_ns(),
        "monotonic_ns": time.monotonic_ns(),
        "source": "test_source",
        "event_type": event_type,
        "severity": "INFO",
        "payload": payload or {"state": "RUNNING"},
    }


def wait_for(predicate, timeout=2.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.01)
    return predicate()


def make_server(tmp_path, **overrides):
    *_, ObservabilityServer = modules()
    options = {
        "event_socket": endpoint("events"),
        "status_socket": endpoint("status"),
        "lock_path": tmp_path / "observability.lock",
        "run_directory": tmp_path / "runs",
        "run_id": "test-run",
        "queue_capacity": 4,
        "socket_runtime": False,
    }
    options.update(overrides)
    return ObservabilityServer(**options)


def test_duplicate_daemon_flock_fails_and_lock_file_is_never_deleted(tmp_path):
    *_, DaemonAlreadyRunning, ObservabilityServer = modules()
    lock_path = tmp_path / "observability.lock"
    first = make_server(tmp_path, lock_path=lock_path)
    second = ObservabilityServer(
        event_socket=endpoint("events"),
        status_socket=endpoint("status"),
        lock_path=lock_path,
        run_directory=tmp_path / "other-runs",
        run_id="other-run",
    )

    first.start()
    try:
        with pytest.raises(DaemonAlreadyRunning, match="already running"):
            second.start()
    finally:
        first.stop()
        second.stop()

    assert lock_path.is_file()


def test_event_ingress_assigns_run_sequence_and_status_is_bounded_snapshot(tmp_path):
    _, _, _, _, _ = modules()
    from powertrain_observability.protocol import encode_event_datagram

    server = make_server(tmp_path)
    server.start()
    try:
        credentials = struct.pack("3i", os.getpid(), os.geteuid(), os.getegid())
        server.ingest_datagram(
            encode_event_datagram(pending_event("COMMAND_OWNER", {"owner": "teleop"})),
            credentials,
        )
        server.ingest_datagram(
            encode_event_datagram(pending_event(
                "CHANNEL_HEALTH",
                {"channel": "l515_srt", "status": "DEGRADED", "age_ms": 125},
            )),
            credentials,
        )

        snapshot = wait_for(
            lambda: (
                current if (current := server.status_snapshot())["recent_event"]
                and current["recent_event"]["sequence"] == 1 else None
            )
        )

        assert snapshot["run_id"] == "test-run"
        assert snapshot["recent_events"]["COMMAND_OWNER"]["run_id"] == "test-run"
        assert snapshot["recent_events"]["COMMAND_OWNER"]["sequence"] == 0
        assert snapshot["recent_event"]["sequence"] == 1
        assert snapshot["channel_health"]["l515_srt"]["status"] == "DEGRADED"
        assert snapshot["drop_count"] == 0
        assert "history" not in snapshot
    finally:
        server.stop()


def test_malformed_and_oversize_datagrams_are_dropped_without_stopping_server(tmp_path):
    modules()
    from powertrain_observability.protocol import MAX_DATAGRAM_BYTES

    server = make_server(tmp_path)
    server.start()
    try:
        credentials = struct.pack("3i", os.getpid(), os.geteuid(), os.getegid())
        assert server.ingest_datagram(b'{"schema_version":', credentials) is False
        assert server.ingest_datagram(
            b"x" * (MAX_DATAGRAM_BYTES + 1), credentials
        ) is False

        snapshot = wait_for(
            lambda: (
                current if (current := server.status_snapshot())["drop_count"] >= 2
                else None
            )
        )

        assert snapshot["health"]["status"] == "OK"
        assert snapshot["drop_count"] >= 2
    finally:
        server.stop()


def test_status_client_disconnect_does_not_stop_daemon(tmp_path):
    modules()

    class Reader:
        closed=False
        def readline(self, _size):
            return b'{"type":"get_status"}\n'
        def close(self):
            self.closed=True

    class DisconnectedClient:
        def __init__(self): self.reader=Reader()
        def makefile(self, _mode):
            return self.reader

        def sendall(self, _payload):
            raise BrokenPipeError("client disconnected")

        def close(self):
            pass

    server = make_server(tmp_path)
    server.start()
    try:
        client=DisconnectedClient()
        server._serve_status(client)
        assert server.is_running
        assert client.reader.closed
    finally:
        server.stop()


def test_status_socket_rejects_unit_injected_wrong_peer_uid():
    *_, ObservabilityServer = modules()

    with pytest.raises(PermissionError, match="UID"):
        ObservabilityServer.authorize_peer((123, os.geteuid() + 1, os.getegid()))


def test_event_socket_enables_kernel_credentials_before_receiving_packets():
    *_, ObservabilityServer = modules()

    class Socket:
        def __init__(self): self.calls=[]
        def setsockopt(self, *args): self.calls.append(args)

    sock=Socket()
    ObservabilityServer.configure_event_socket(sock)

    import socket
    assert sock.calls == [(socket.SOL_SOCKET, socket.SO_PASSCRED, 1)]


def test_journal_is_flushed_after_each_accepted_event(tmp_path):
    modules()
    from powertrain_observability.protocol import encode_event_datagram

    server=make_server(tmp_path)
    server.start()
    calls=[]
    original_flush=server._journal.flush
    server._journal.flush=lambda: calls.append(True) or original_flush()
    try:
        credentials=struct.pack("3i",os.getpid(),os.geteuid(),os.getegid())
        assert server.ingest_datagram(
            encode_event_datagram(pending_event()),credentials
        ) is True
        assert wait_for(lambda: calls)
    finally:
        server.stop()


def test_partial_socket_startup_closes_every_created_socket(tmp_path,monkeypatch):
    from powertrain_observability import server as server_module

    class Socket:
        def __init__(self,fail_bind=False):
            self.fail_bind=fail_bind; self.closed=False
        def setsockopt(self,*_args): pass
        def bind(self,_address):
            if self.fail_bind: raise OSError("simulated status bind failure")
        def settimeout(self,_timeout): pass
        def close(self): self.closed=True

    event_socket=Socket()
    status_socket=Socket(fail_bind=True)
    sockets=iter((event_socket,status_socket))
    monkeypatch.setattr(server_module.socket,"socket",lambda *_args: next(sockets))
    server=make_server(tmp_path,socket_runtime=True)

    with pytest.raises(OSError,match="status bind failure"):
        server.start()

    assert event_socket.closed
    assert status_socket.closed


def test_maximum_bounded_snapshot_still_fits_status_protocol(tmp_path):
    from powertrain_observability.protocol import encode_status_response

    server=make_server(tmp_path)
    blob="x" * 8_000
    sequence=0
    for index in range(server.MAX_RECENT_EVENT_TYPES):
        server._record_snapshot({
            **pending_event(f"TEAM_EVENT_{index}",{"blob":blob}),
            "run_id":"test-run","sequence":sequence,
        })
        sequence += 1
    for index in range(server.MAX_CHANNELS):
        server._record_snapshot({
            **pending_event(
                "CHANNEL_HEALTH",
                {"channel":f"channel-{index}","status":"OK","blob":blob},
            ),
            "run_id":"test-run","sequence":sequence,
        })
        sequence += 1

    encoded=encode_status_response(server.status_snapshot())

    assert encoded.endswith(b"\n")
