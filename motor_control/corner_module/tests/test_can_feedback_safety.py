"""Regression evidence for CAN audit 09/11/13/14; no hardware is opened."""
import struct
import time

import can
import pytest

from chassis.chassis_manager import ChassisManager
from chassis.kinematics import default_geometry
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.fake import FakeDrive, FakeSteer
from corner_module.null_steer import NullSteer
from corner_module.steer_ak40 import AK40, SteerAk40


def frame(node, cmd, data, timestamp=0):
    return can.Message(arbitration_id=(node << 5) | cmd, data=data,
                       is_extended_id=False, timestamp=timestamp)


class Bus:
    def __init__(self, node=11, *, answer_arm=True, fail=()):
        self.node = node
        self.answer_arm = answer_arm
        self.fail = set(fail)
        self.sent = []
        self.rx = []

    def send(self, message):
        self.sent.append(message)
        cmd = message.arbitration_id & 31
        if cmd in self.fail:
            raise can.CanOperationError('simulated ENOBUFS')
        if cmd == 7 and struct.unpack('<I', message.data[:4])[0] == 8 and self.answer_arm:
            self.rx.extend([frame(self.node, 1, struct.pack('<IB3x', 0, 8)),
                            frame(self.node, 9, struct.pack('<ff', 0, 0))])

    def recv(self, timeout=0):
        return self.rx.pop(0) if self.rx else None


def manager(*, answer_arm=True):
    corners = {}
    for index, wheel in enumerate(default_geometry().wheels, 11):
        bus = Bus(index, answer_arm=answer_arm)
        corners[wheel.name] = CornerModule(NullSteer(), DriveOdriveCan(index, bus=bus), CornerConfig())
    cm = ChassisManager(corners)
    cm.connect()
    return cm


def test_control_send_failure_is_not_hidden_and_is_counted():
    drive = DriveOdriveCan(bus=Bus(fail={13}))
    with pytest.raises(can.CanError):
        drive.tick()
    assert drive.health_state()['control_tx_failures'] == 1


def test_query_failure_is_counted_without_raising_control_failure():
    now = [0.0]
    drive = DriveOdriveCan(bus=Bus(fail={9, 20}), clock=lambda: now[0])
    drive.poll_feedback()
    now[0] = 0.020
    drive.poll_feedback()
    health = drive.health_state()
    assert health['feedback_tx_failures'] == 2
    assert health['control_tx_failures'] == 0


@pytest.mark.parametrize('method', ['disarm', 'estop'])
def test_failed_zero_still_attempts_idle(method):
    bus = Bus(fail={13})
    drive = DriveOdriveCan(bus=bus)
    drive.set_velocity(1)
    with pytest.raises(can.CanError):
        getattr(drive, method)()
    assert [m.arbitration_id & 31 for m in bus.sent] == [13, 7]
    assert drive.health_state()['target_vel'] == 0


def test_old_socket_frames_remain_old_at_drain_time():
    drive = DriveOdriveCan(bus=Bus())
    stamp = time.time() - 10
    drive._handle_rx(frame(11, 1, struct.pack('<IB3x', 0, 8), stamp))
    drive._handle_rx(frame(11, 9, struct.pack('<ff', 0, 0), stamp))
    health = drive.health_state()
    assert health['stale'] is True
    assert health['encoder_stale'] is True
    assert health['last_heartbeat_age_ms'] >= 9900


def test_encoder_replies_cannot_keep_dead_heartbeat_fresh():
    now = [10.0]
    drive = DriveOdriveCan(bus=Bus(), clock=lambda: now[0])
    drive._handle_rx(frame(11, 1, struct.pack('<IB3x', 0, 8)))
    now[0] += .3
    drive._handle_rx(frame(11, 9, struct.pack('<ff', 0, 0)))
    assert drive.health_state()['heartbeat_stale'] is True


def test_six_axis_arm_rejects_fresh_idle_feedback_with_common_deadline():
    cm = manager(answer_arm=False)
    for corner in cm.corners.values():
        bus = corner.drive._bus
        bus.rx.extend([frame(bus.node, 1, struct.pack('<IB3x', 0, 1)),
                       frame(bus.node, 9, struct.pack('<ff', 0, 0))])
    started = time.monotonic()
    assert cm.arm() is False
    assert time.monotonic() - started < .25
    assert cm.mode == 'ESTOP'


def test_pre_request_state_eight_does_not_prove_new_arm():
    cm = manager(answer_arm=False)
    for corner in cm.corners.values():
        bus = corner.drive._bus
        stamp = time.time() - .02
        bus.rx.extend([frame(bus.node, 1, struct.pack('<IB3x', 0, 8), stamp),
                       frame(bus.node, 9, struct.pack('<ff', 0, 0), stamp)])
    assert cm.arm() is False
    assert cm.mode == 'ESTOP'


def test_arm_succeeds_with_all_post_request_closed_loop_feedback():
    cm = manager()
    assert cm.arm() is True
    assert cm.mode == 'ARMED'
    assert all(c.mode == 'ARMED' for c in cm.corners.values())


@pytest.mark.parametrize('defect', ['idle', 'encoder_lost', 'heartbeat_lost'])
def test_running_corner_trips_when_required_drive_feedback_is_lost(defect):
    now = [10.0]
    bus = Bus()
    drive = DriveOdriveCan(bus=bus, clock=lambda: now[0])
    corner = CornerModule(NullSteer(), drive, CornerConfig(), clock=lambda: now[0])
    corner.connect()
    corner.arm()
    now[0] += .3
    if defect != 'heartbeat_lost':
        state = 1 if defect == 'idle' else 8
        bus.rx.append(frame(11, 1, struct.pack('<IB3x', 0, state)))
    if defect != 'encoder_lost':
        bus.rx.append(frame(11, 9, struct.pack('<ff', 0, 0)))
    corner.tick()
    assert corner.mode == 'FAULT'


def test_idle_poll_uses_same_sixty_ms_encoder_and_one_point_two_second_iq_schedule():
    now = [0.0]
    bus = Bus()
    drive = DriveOdriveCan(bus=bus, clock=lambda: now[0])
    for index in range(100):
        now[0] = index * .01
        drive.poll_feedback()
    queries = [m.arbitration_id & 31 for m in bus.sent]
    assert queries.count(9) == 17
    assert queries.count(20) == 1


def test_corner_disarm_failure_does_not_skip_other_actuator():
    class BrokenSteer(FakeSteer):
        def disarm(self):
            raise can.CanOperationError('steer stop failed')
    drive = FakeDrive()
    corner = CornerModule(BrokenSteer(), drive, CornerConfig())
    corner.connect()
    corner.arm()
    with pytest.raises(can.CanError):
        corner.disarm()
    assert drive._armed is False
    assert corner.mode == 'FAULT'


def test_manager_disarm_failure_stops_remaining_corners_and_latches():
    cm = manager()
    assert cm.arm()
    first = next(iter(cm.corners.values()))
    first.drive._bus.fail.add(13)
    cm.disarm()
    assert cm.mode == 'ESTOP'
    for corner in cm.corners.values():
        assert any((m.arbitration_id & 31) == 7 and m.data[0] == 1 for m in corner.drive._bus.sent)


def test_manager_tick_failure_stops_remaining_corners_and_latches():
    cm = manager()
    assert cm.arm()
    first = next(iter(cm.corners.values()))
    first.drive._bus.fail.add(13)
    cm.tick()
    assert cm.mode == 'ESTOP'
    assert all(c.mode == 'FAULT' for c in cm.corners.values())


def test_steering_control_send_failure_is_visible():
    class BrokenAk:
        def send_pos_out(self, angle):
            return False
        def poll(self, timeout=0):
            return False
    steer = SteerAk40()
    steer._ak = BrokenAk()
    with pytest.raises(can.CanError):
        steer.tick()


def test_old_steering_socket_frame_does_not_become_fresh():
    bus = Bus()
    bus.rx.append(can.Message(arbitration_id=(41 << 8) | 1,
                              data=struct.pack('>hhhbb', 20, 0, 0, 20, 0),
                              is_extended_id=True, timestamp=time.time() - 10))
    steer = SteerAk40()
    steer._bus = bus
    steer._ak = AK40(bus, 1)
    assert steer.state()['stale'] is True


def test_pending_arm_synchronizes_target_to_first_fresh_steering_angle():
    steer = FakeSteer(start_deg=0)
    steer.stale_flag = True
    corner = CornerModule(steer, FakeDrive(), CornerConfig())
    corner.connect()
    corner.arm()
    assert corner.mode == 'ARMING'
    steer._actual = 25
    steer.stale_flag = False
    assert corner.confirm_arm()
    corner.tick()
    assert steer.state()['target_deg'] == 25


def test_confirmed_corner_must_recheck_hardware_before_collective_success():
    bus = Bus()
    corner = CornerModule(NullSteer(), DriveOdriveCan(bus=bus), CornerConfig())
    corner.connect()
    corner.arm()
    assert corner.confirm_arm()
    bus.rx.append(frame(11, 1, struct.pack('<IB3x', 0, 1)))
    assert corner.confirm_arm() is False


def test_corner_send_failure_latches_fault_before_it_propagates():
    bus = Bus()
    corner = CornerModule(NullSteer(), DriveOdriveCan(bus=bus), CornerConfig())
    corner.connect()
    corner.arm()
    bus.fail.add(13)
    with pytest.raises(can.CanError):
        corner.tick()
    assert corner.mode == 'FAULT'
    assert any(m.arbitration_id & 31 == 7 and m.data[0] == 1 for m in bus.sent)


def test_real_bus_timestamp_zero_is_never_treated_as_test_feedback():
    with can.Bus(interface='virtual') as bus:
        drive = DriveOdriveCan(bus=bus)
        assert drive._handle_rx(frame(11, 1, struct.pack('<IB3x', 0, 8))) is False
        assert drive.health_state()['heartbeat_stale'] is True


def test_realtime_clock_correction_cannot_rejuvenate_old_frame(monkeypatch):
    import corner_module.actuator as actuator_module
    now = [10.0]
    wall = [1000.0]
    monkeypatch.setattr(actuator_module.time, 'time', lambda: wall[0])
    drive = DriveOdriveCan(bus=Bus(), clock=lambda: now[0])
    now[0] += 10
    wall[0] -= 100
    assert drive._handle_rx(frame(11, 1, struct.pack('<IB3x', 0, 8), 1000)) is False
    assert drive.health_state()['heartbeat_stale'] is True


def test_out_of_order_heartbeat_cannot_overwrite_new_fault():
    drive = DriveOdriveCan(bus=Bus())
    stamp = time.time()
    drive._handle_rx(frame(11, 1, struct.pack('<IB3x', 8, 1), stamp))
    drive._handle_rx(frame(11, 1, struct.pack('<IB3x', 0, 8), stamp - .01))
    assert drive.health_state()['axis_error'] == 8


def test_optional_extraction_cannot_bypass_real_arm_confirmation():
    cm = manager(answer_arm=False)
    cm.cfg.extraction_enabled = True
    cm.update_external_safety('VALID', True, 'too close')
    cm.tick()
    assert cm.mode == 'ESTOP'
    assert cm.extraction_grant() is False
    assert cm.mode == 'ESTOP'
