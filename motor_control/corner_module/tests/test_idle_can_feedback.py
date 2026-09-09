import struct

import can
import pytest

from chassis.chassis_manager import ChassisManager
from chassis.kinematics import default_geometry
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.null_steer import NullSteer


class Bus:
    def __init__(self, node):
        self.node = node
        self.velocity = 1.0
        self.answer = True
        self.rx = [can.Message(arbitration_id=(node << 5) | 1,
                               data=struct.pack('<IB3x', 0, 1), is_extended_id=False),
                   can.Message(arbitration_id=(node << 5) | 9,
                               data=struct.pack('<ff', 0.0, 1.0), is_extended_id=False)]
        self.sent = []

    def send(self, msg):
        self.sent.append(msg)
        if msg.arbitration_id & 31 == 7 and msg.data[0] == 8:
            self.rx.append(can.Message(arbitration_id=(self.node << 5) | 1,
                                       data=struct.pack('<IB3x', 0, 8), is_extended_id=False))
        if self.answer and msg.is_remote_frame and msg.arbitration_id & 31 == 9:
            self.rx.append(can.Message(arbitration_id=(self.node << 5) | 9,
                                       data=struct.pack('<ff', 0.0, self.velocity), is_extended_id=False))

    def recv(self, timeout=0):
        return self.rx.pop(0) if self.rx else None


@pytest.mark.parametrize('estop', [False, True])
def test_inactive_chassis_refreshes_actual_speed_using_only_queries(estop):
    now = [10.0]
    buses = [Bus(n) for n in range(11, 17)]
    corners = {wheel.name: CornerModule(NullSteer(), DriveOdriveCan(node_id=bus.node, bus=bus, clock=lambda: now[0]), CornerConfig())
               for wheel, bus in zip(default_geometry().wheels, buses)}
    cm = ChassisManager(corners)
    cm.connect()
    assert cm.arm()
    cm.tick()
    assert cm.snapshot().wheels[0].drive_turns_per_s > 0
    if estop:
        cm.estop('test', 'latched for idle feedback test')
    else:
        cm.disarm()
    for bus in buses:
        bus.sent.clear()
        bus.answer = False
    cm.tick()
    assert all(w.drive_turns_per_s > 0 for w in cm.snapshot().wheels), 'no new reply must not fabricate zero'
    for bus in buses:
        bus.answer = True
        bus.velocity = 0.0
    now[0] += .06  # Polling is time-budgeted, not one query pair per tick.
    cm.tick()
    cm.tick()
    snapshot = cm.snapshot()
    assert all(w.drive_turns_per_s == 0 for w in snapshot.wheels), 'stop needs new encoder feedback after disarm'
    assert cm.mode == ('ESTOP' if estop else 'IDLE')
    for bus in buses:
        assert bus.sent
        assert all(m.is_remote_frame and m.arbitration_id & 31 in (9, 20) for m in bus.sent)


def test_heartbeat_does_not_certify_old_zero_encoder_as_fresh():
    now = [10.0]
    bus = Bus(11)
    drive = DriveOdriveCan(node_id=11, bus=bus, clock=lambda: now[0], stale_ms=300)
    encoder = can.Message(arbitration_id=(11 << 5) | 9,
                          data=struct.pack('<ff', 0.0, 0.0), is_extended_id=False)
    drive._handle_rx(encoder)
    now[0] += .4
    heartbeat = can.Message(arbitration_id=(11 << 5) | 1,
                            data=struct.pack('<IB3x', 0, 1), is_extended_id=False)
    drive._handle_rx(heartbeat)
    assert drive.health_state()['stale'] is False
    assert drive.health_state().get('encoder_stale') is True
    corners = {w.name: CornerModule(NullSteer(), drive, CornerConfig()) for w in default_geometry().wheels}
    assert ChassisManager(corners).snapshot().wheels[0].drive_stale is True


def test_closed_injected_bus_does_not_receive_idle_queries():
    bus = Bus(11)
    corner = CornerModule(NullSteer(), DriveOdriveCan(node_id=11, bus=bus), CornerConfig())
    corner.connect()
    corner.close()
    bus.sent.clear()
    corner.tick()
    assert bus.sent == []
