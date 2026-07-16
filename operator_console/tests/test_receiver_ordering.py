import importlib
from pathlib import Path


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


def test_same_sender_restart_near_zero_is_accepted_but_small_reorder_is_not():
    gate_type = _gate_type()
    assert gate_type is not None, "UDP source/sequence gate is missing"
    gate = gate_type(stale_after_s=2.0)
    sender = ("192.168.8.106", 5005)

    assert gate.accept(sender, 100, now_s=0.0)
    assert gate.accept(sender, 0, now_s=0.1)
    assert gate.accept(sender, 1, now_s=0.2)
    assert not gate.accept(sender, 0, now_s=0.3)


def test_both_udp_receivers_use_the_shared_source_gate():
    root = Path(__file__).parents[1]
    for filename in ("telemetry.py", "metadata.py"):
        source = (root / filename).read_text(encoding="utf-8")
        assert "SourceSequenceGate" in source
