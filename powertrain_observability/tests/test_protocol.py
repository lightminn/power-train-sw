import json
import os
import struct

import pytest


def protocol():
    from powertrain_observability import protocol as module

    return module


def pending_event(**overrides):
    event = {
        "schema_version": 1,
        "wall_time_ns": 1_750_000_000_000_000_000,
        "monotonic_ns": 123_456_789,
        "source": "chassis_node",
        "event_type": "COMMAND_OWNER",
        "severity": "INFO",
        "payload": {"owner": "teleop"},
    }
    event.update(overrides)
    return event


def test_datagram_credentials_reject_wrong_uid_from_kernel_metadata():
    module = protocol()
    credentials = struct.pack("3i", os.getpid(), os.geteuid() + 1, os.getegid())

    with pytest.raises(PermissionError, match="UID"):
        module.verify_credentials(credentials, expected_uid=os.geteuid())


def test_datagram_credentials_accept_same_uid():
    module = protocol()
    credentials = struct.pack("3i", os.getpid(), os.geteuid(), os.getegid())

    assert module.verify_credentials(credentials, expected_uid=os.geteuid()) == (
        os.getpid(),
        os.geteuid(),
        os.getegid(),
    )


def test_oversize_datagram_is_rejected_before_json_decode():
    module = protocol()

    with pytest.raises(ValueError, match="size limit"):
        module.decode_event_datagram(b"{" + b"x" * module.MAX_DATAGRAM_BYTES)


def test_malformed_json_datagram_is_rejected():
    module = protocol()

    with pytest.raises(ValueError, match="JSON"):
        module.decode_event_datagram(b'{"schema_version":')


def test_daemon_owned_fields_are_removed_from_ingress():
    module = protocol()
    encoded = json.dumps(
        pending_event(run_id="producer-lie", sequence=999), separators=(",", ":")
    ).encode()

    decoded = module.decode_event_datagram(encoded)

    assert "run_id" not in decoded
    assert "sequence" not in decoded
    assert decoded["payload"] == {"owner": "teleop"}


def test_abstract_socket_names_use_linux_nul_prefix():
    module = protocol()

    assert module.abstract_address("@powertrain-test") == "\0powertrain-test"


def test_status_client_closes_response_reader(monkeypatch):
    from powertrain_observability import client as client_module

    response = protocol().encode_status_response(
        {
            "run_id": "run-a",
            "health": {"status": "OK", "last_error": None},
            "drop_count": 0,
            "recent_event": None,
            "recent_events": {},
            "channel_health": {},
        }
    )

    class Reader:
        closed = False
        def readline(self, _size): return response
        def close(self): self.closed = True

    class Socket:
        def __init__(self): self.reader=Reader(); self.closed=False
        def settimeout(self, _timeout): pass
        def connect(self, _address): pass
        def sendall(self, _payload): pass
        def makefile(self, _mode): return self.reader
        def close(self): self.closed=True

    sock=Socket()
    monkeypatch.setattr(client_module.socket,"socket",lambda *_args: sock)

    snapshot=client_module.ObservabilityClient().query()

    assert snapshot.payload["run_id"] == "run-a"
    assert sock.closed
    assert sock.reader.closed


def test_event_client_never_blocks_or_raises_when_daemon_is_unavailable(monkeypatch):
    from powertrain_observability import client as client_module

    class Socket:
        def __init__(self): self.blocking=None; self.closed=False
        def setblocking(self,value): self.blocking=value
        def sendto(self,_payload,_address): raise BlockingIOError("socket backlog full")
        def close(self): self.closed=True

    sock=Socket()
    monkeypatch.setattr(client_module.socket,"socket",lambda *_args: sock)
    client=client_module.EventClient()

    accepted=client.emit(pending_event())

    assert accepted is False
    assert sock.blocking is False
    assert sock.closed
    assert "backlog full" in client.last_error


def test_event_client_does_not_raise_when_socket_creation_fails(monkeypatch):
    from powertrain_observability import client as client_module

    def fail_socket(*_args):
        raise OSError("file descriptor exhaustion")

    monkeypatch.setattr(client_module.socket,"socket",fail_socket)
    client=client_module.EventClient()

    assert client.emit(pending_event()) is False
    assert "descriptor exhaustion" in client.last_error
