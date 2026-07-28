import json
import socket

import pytest

from tools.operator_console_replay import load_rows, replay


def test_replay_transmits_payload_unchanged_through_udp(tmp_path):
    receiver = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    receiver.bind(("127.0.0.1", 0))
    receiver.settimeout(1.0)
    port = receiver.getsockname()[1]
    payload = {"schema_version": 1, "sequence": 7, "voltage_v": 47.8}
    capture = tmp_path / "capture.jsonl"
    capture.write_text(json.dumps({
        "t": 0, "channel": "power", "payload": payload,
    }) + "\n", encoding="utf-8")

    rows = load_rows(capture)
    replay(rows, host="127.0.0.1", speed=100, ports={
        "metadata": port + 1, "power": port,
        "chassis": port + 2, "arm": port + 3,
    })

    wire, _sender = receiver.recvfrom(4096)
    receiver.close()
    assert json.loads(wire) == payload


def test_replay_rejects_unknown_channel(tmp_path):
    capture = tmp_path / "capture.jsonl"
    capture.write_text(
        '{"t":0,"channel":"lidar","payload":{}}\n', encoding="utf-8",
    )
    with pytest.raises(ValueError, match="unsupported channel"):
        load_rows(capture)
