import struct

import can
import pytest

from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.steer_ak40 import SteerAk40


class FakeClock:
    def __init__(self):
        self.seconds = 0.0

    def __call__(self):
        return self.seconds

    def advance(self, seconds):
        self.seconds += seconds


class FakeCanBus:
    def __init__(self, rx=()):
        self.rx = list(rx)
        self.sent = []

    def recv(self, timeout=0.0):
        return self.rx.pop(0) if self.rx else None

    def send(self, message):
        self.sent.append(message)


class StubAk:
    def __init__(self, poll_results):
        self.poll_results = list(poll_results)
        self.pos_out_deg = 0.0
        self.cur_a = 0.0
        self.fault = 0

    def poll(self, timeout=0.0):
        return self.poll_results.pop(0) if self.poll_results else False

    def send_pos_out(self, target_deg):
        self.pos_out_deg = target_deg


def heartbeat(node_id, *, error=0, state=8):
    return can.Message(
        arbitration_id=(node_id << 5) | 0x01,
        data=struct.pack("<I", error) + bytes([state, 0, 0, 0]),
        is_extended_id=False,
    )


def encoder(node_id, *, position=0.0, velocity=1.0):
    return can.Message(
        arbitration_id=(node_id << 5) | 0x09,
        data=struct.pack("<ff", position, velocity),
        is_extended_id=False,
    )


def test_ak_health_tracks_feedback_age_rate_and_stale_recovery():
    clock = FakeClock()
    steer = SteerAk40(motor_id=3, stale_ms=100.0, clock=clock)
    steer._ak = StubAk([True, True, False, True])

    steer.tick()
    clock.advance(0.02)
    steer.tick()
    healthy = steer.health_state()

    assert healthy["can_id"] == 3
    assert healthy["last_feedback_age_ms"] == pytest.approx(0.0)
    assert healthy["feedback_rate_hz"] == pytest.approx(50.0)
    assert healthy["rx_packets"] == 2
    assert healthy["stale"] is False
    assert healthy["recovery_count"] == 0

    clock.advance(0.101)
    steer.tick()
    assert steer.health_state()["stale"] is True

    steer.tick()
    recovered = steer.health_state()
    assert recovered["stale"] is False
    assert recovered["recovery_count"] == 1


def test_odrive_health_tracks_heartbeat_encoder_axis_and_recovery():
    clock = FakeClock()
    node_id = 15
    bus = FakeCanBus([
        heartbeat(node_id, error=0x20, state=8),
        encoder(node_id, velocity=1.25),
    ])
    drive = DriveOdriveCan(
        node_id=node_id,
        stale_ms=100.0,
        bus=bus,
        clock=clock,
    )

    drive.tick()
    healthy = drive.health_state()

    assert healthy["node_id"] == 15
    assert healthy["last_heartbeat_age_ms"] == pytest.approx(0.0)
    assert healthy["last_encoder_age_ms"] == pytest.approx(0.0)
    assert healthy["axis_state"] == 8
    assert healthy["axis_error"] == 0x20
    assert healthy["rx_packets"] == 2
    assert healthy["stale"] is False
    assert healthy["recovery_count"] == 0

    clock.advance(0.101)
    assert drive.health_state()["stale"] is True
    bus.rx.append(heartbeat(node_id, state=8))
    drive.tick()

    recovered = drive.health_state()
    assert recovered["stale"] is False
    assert recovered["recovery_count"] == 1
    assert recovered["last_heartbeat_age_ms"] == pytest.approx(0.0)
    assert recovered["last_encoder_age_ms"] == pytest.approx(101.0)
