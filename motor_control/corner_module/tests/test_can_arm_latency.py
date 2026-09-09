"""CAN activation must leave time for the existing 300 ms input watchdog."""
import struct
from types import SimpleNamespace

import can
import pytest

from chassis.authority import CommandAuthority, MANUAL_SOURCE
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.steer_ak40 import AK40, SteerAk40


class Clock:
    now = 10.0

    def __call__(self):
        return self.now


class TimedBus:
    def __init__(self, clock, frames=()):
        self.clock = clock
        self.frames = list(frames)
        self.sent = []

    def recv(self, timeout=0.0):
        if self.frames:
            return self.frames.pop(0)
        self.clock.now += timeout
        return None

    def send(self, message):
        self.sent.append(message)


@pytest.mark.parametrize('feedback', [True, False])
def test_six_drive_arm_does_not_expire_fresh_manual_input(monkeypatch, feedback):
    import corner_module.drive_odrive_can as driver_module

    clock = Clock()
    monkeypatch.setattr(driver_module, 'time', SimpleNamespace(monotonic=clock))
    authority = CommandAuthority()
    authority.request_mode('MANUAL', t=clock())
    authority.submit(MANUAL_SOURCE, 0.0, 0.0, clock())
    assert authority.select(clock()).ok
    for node in range(11, 17):
        frames = [can.Message(arbitration_id=(node << 5) | 1,
                              data=struct.pack('<IB3x', 0, 1), is_extended_id=False)] if feedback else []
        drive = DriveOdriveCan(node_id=node, bus=TimedBus(clock, frames), clock=clock)
        drive.arm()
        assert drive.health_state()['stale'] is (not feedback)
        assert drive.health_state()['target_vel'] == 0.0
    authority.select(clock())
    assert authority.mode == 'TELEOP', 'sequential CAN activation must not consume the input freshness budget'


def test_quiet_steering_arm_returns_without_fabricating_feedback():
    clock = Clock()
    bus = TimedBus(clock)
    steer = SteerAk40(clock=clock)
    steer._bus = bus
    steer._ak = AK40(bus, 1)
    before = clock()
    steer.arm()
    assert clock() == before, 'a missing steering reply must not block the chassis executor'
    assert steer.health_state()['stale'] is True
    assert steer.health_state()['rx_packets'] == 0
    assert bus.sent == []


def test_can_arm_keeps_received_fault_visible():
    clock = Clock()
    frame = can.Message(arbitration_id=(11 << 5) | 1,
                        data=struct.pack('<IB3x', 8, 1), is_extended_id=False)
    drive = DriveOdriveCan(node_id=11, bus=TimedBus(clock, [frame]), clock=clock)
    drive.arm()
    assert drive.health_state()['axis_error'] == 8
    assert drive.health_state()['stale'] is False
