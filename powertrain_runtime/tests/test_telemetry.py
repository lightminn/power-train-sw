import json
import socket
import time

import pytest

from powertrain_runtime.telemetry import send_datagram


def test_live_udp_follows_session_and_suppresses_expired_record(tmp_path, monkeypatch):
    record = tmp_path / "session.json"
    monkeypatch.setenv("POWERTRAIN_OPERATOR_SESSION_FILE", str(record))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(.05)
        port = receiver.getsockname()[1]
        assert send_datagram(sender, b"absent", ("192.0.2.1", port)) is False
        for lease in ("first", "reconnected"):
            record.write_text(json.dumps(dict(host="127.0.0.1", lease_id=lease, expires_at=time.monotonic() + 10)))
            assert send_datagram(sender, lease.encode(), ("192.0.2.1", port)) is True
            assert receiver.recv(128) == lease.encode()
        record.write_text(json.dumps(dict(host="127.0.0.1", lease_id="expired", expires_at=time.monotonic() - 1)))
        assert send_datagram(sender, b"stale", ("127.0.0.1", port)) is False
        with pytest.raises(socket.timeout):
            receiver.recv(128)


def test_legacy_udp_still_works_without_session(tmp_path, monkeypatch):
    monkeypatch.delenv("POWERTRAIN_OPERATOR_SESSION_FILE", raising=False)
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
        receiver.bind(("127.0.0.1", 0))
        receiver.settimeout(.1)
        assert send_datagram(sender, b"legacy", receiver.getsockname())
        assert receiver.recv(128) == b"legacy"
