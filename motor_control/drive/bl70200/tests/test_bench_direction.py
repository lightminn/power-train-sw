"""Bench entrypoints must use the same measured forward signs as runtime."""
from contextlib import nullcontext
import struct
import sys
from types import SimpleNamespace

import chassis.runtime_lock
from drive.bl70200 import can_drive_test, dualsense_usb_teleop


def test_can_bench_forward_uses_left_negative_right_positive(monkeypatch):
    sent = []
    bus = SimpleNamespace(send=sent.append, shutdown=lambda: None)
    monkeypatch.setattr(sys, "argv", ["can_drive_test", "--speed", "1.0"])
    monkeypatch.setattr(chassis.runtime_lock, "RealCanSession", lambda **kw: nullcontext())
    monkeypatch.setattr(can_drive_test.can, "Bus", lambda **kw: bus)
    monkeypatch.setattr(can_drive_test.time, "sleep", lambda seconds: None)
    monkeypatch.setattr(can_drive_test, "arm_check", lambda bus, node: (0, 8))
    monkeypatch.setattr(can_drive_test, "readall", lambda bus, stage: None)

    can_drive_test.main()

    first_motion = {}
    for message in sent:
        if message.arbitration_id & 31 == 0x0D:
            velocity = struct.unpack("<ff", message.data)[0]
            if velocity:
                first_motion.setdefault(message.arbitration_id >> 5, velocity)
    assert first_motion == {11: -1.0, 12: 1.0, 13: -1.0, 14: 1.0, 15: -1.0, 16: 1.0}


def test_usb_bench_mounting_sign_and_raw_override():
    board = SimpleNamespace(axis0=object(), axis1=object())
    mounted = dualsense_usb_teleop.collect_axes([("ABC", board)], "both", True)
    raw = dualsense_usb_teleop.collect_axes([("ABC", board)], "both", False)
    assert [(axis, sign) for _, axis, sign in mounted] == [(board.axis0, -1.0), (board.axis1, 1.0)]
    assert [(axis, sign) for _, axis, sign in raw] == [(board.axis0, 1.0), (board.axis1, 1.0)]
