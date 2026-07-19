import importlib
import json
from pathlib import Path
import threading

import pytest

from operator_console.metadata import LatestMetadataReceiver
from operator_console.telemetry import LatestTelemetryReceiver


def _gate_type():
    try:
        module = importlib.import_module("operator_console.udp_source")
    except ModuleNotFoundError:
        return None
    return getattr(module, "SourceSequenceGate", None)


def test_same_sender_duplicate_and_reordered_sequences_are_rejected():
    gate_type = _gate_type()
    assert gate_type is not None, "UDP source/sequence gate is missing"
    gate = gate_type(stale_after_s=2.0)
    sender = ("192.168.8.106", 5005)

    assert gate.accept(sender, 10, now_s=0.0)
    assert not gate.accept(sender, 10, now_s=0.1)
    assert not gate.accept(sender, 9, now_s=0.2)
    assert gate.accept(sender, 11, now_s=0.3)


def test_spoofed_address_is_rejected_until_pinned_sender_is_stale():
    gate_type = _gate_type()
    assert gate_type is not None, "UDP source/sequence gate is missing"
    gate = gate_type(stale_after_s=2.0)
    first = ("192.168.8.106", 5005)
    spoofed = ("192.168.8.250", 5005)

    assert gate.accept(first, 20, now_s=1.0)
    assert not gate.accept(spoofed, 21, now_s=2.9)
    assert gate.accept(spoofed, 0, now_s=3.01)


def test_same_sender_restart_near_zero_is_accepted_only_after_source_is_stale():
    gate_type = _gate_type()
    assert gate_type is not None, "UDP source/sequence gate is missing"
    gate = gate_type(stale_after_s=2.0)
    sender = ("192.168.8.106", 5005)

    assert gate.accept(sender, 100, now_s=0.0)
    assert not gate.accept(sender, 0, now_s=0.1)
    assert not gate.accept(sender, 1, now_s=0.2)
    assert gate.accept(sender, 0, now_s=2.01)
    assert gate.accept(sender, 1, now_s=2.02)
    assert not gate.accept(sender, 0, now_s=2.03)


def test_both_udp_receivers_use_the_shared_source_gate():
    root = Path(__file__).parents[1]
    for filename in ("telemetry.py", "metadata.py"):
        source = (root / filename).read_text(encoding="utf-8")
        assert "SourceSequenceGate" in source


class _StopAfterPacketSocket:
    def __init__(self, packets, stopping):
        self._packets = list(packets)
        self._stopping = stopping

    def settimeout(self, _timeout):
        pass

    def recvfrom(self, _size):
        packet = self._packets.pop(0)
        if not self._packets:
            self._stopping.set()
        return packet, ("192.168.8.106", 5005)


@pytest.mark.parametrize(
    ("receiver_type", "valid_payload", "sequence"),
    (
        (
            LatestTelemetryReceiver,
            {"schema_version": 1, "sequence": 31},
            31,
        ),
        (
            LatestMetadataReceiver,
            {
                "schema_version": 1,
                "capture_sequence": 32,
                "frame_width": 848,
                "frame_height": 480,
                "detections": [],
            },
            32,
        ),
    ),
)
def test_receiver_counts_unexpected_packet_error_and_continues_to_next_packet(
    receiver_type,
    valid_payload,
    sequence,
):
    receiver = receiver_type.__new__(receiver_type)
    receiver._latest = None
    receiver._lock = threading.Lock()
    receiver._stopping = threading.Event()
    receiver._source_gate = type(
        "AcceptAll",
        (),
        {"accept": staticmethod(lambda *_args, **_kwargs: True)},
    )()
    receiver._invalid_packet_count = 0
    receiver._socket = _StopAfterPacketSocket(
        [b"[]", json.dumps(valid_payload).encode("utf-8")],
        receiver._stopping,
    )

    receiver._run()

    assert receiver.latest().sequence == sequence
    assert receiver.invalid_packet_count == 1
