"""Six real CAN drivers, deterministic bus boundary, no hardware access."""
import struct

import can
import pytest

from chassis.chassis_manager import ChassisManager, build_corners
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.fake import FakeSteer


class AxisBus:
    def __init__(self, node):
        self.node = node
        self.state = 1
        self.error = 0
        self.marker = 0xA3
        self.flags = 0
        self.generation = 0
        self.velocity = 0.0
        self.sent = []
        self.online = True
        self.refuse_arm = False

    def recv(self, timeout=0):
        return None

    def emit(self):
        if self.online:
            self.drive._handle_rx(can.Message(is_extended_id=False, arbitration_id=self.node << 5 | 1,
                data=struct.pack("<IBBBB", self.error, self.state, self.marker, self.flags, self.generation)))
            self.drive._handle_rx(can.Message(is_extended_id=False, arbitration_id=self.node << 5 | 9,
                data=struct.pack("<ff", 0.0, self.velocity)))

    def send(self, message):
        self.sent.append(message)
        if not self.online:
            raise can.CanOperationError("injected CAN interruption")
        if message.is_remote_frame:
            self.emit()
            return
        command = message.arbitration_id & 31
        if command == 0x18:
            if bytes(message.data) == bytes(8) or (
                    bytes(message.data) == bytes([0xA3, self.generation, 0, 0, 0, 0, 0, 0])
                    and self.error == 0x4000 and self.state == 1 and self.flags == 5):
                self.error = self.flags = 0
        elif command == 0x07:
            requested = struct.unpack("<I", message.data[:4])[0]
            guarded = len(message.data) == 8 and message.data[4] == 0xA3
            if guarded:
                accepted = (requested == 8 and self.error == 0x4000 and self.state == 1
                            and self.flags == 5 and message.data[5] == self.generation
                            and bytes(message.data[6:]) == bytes(2))
                if accepted:
                    self.error = self.flags = 0
                    self.velocity = 0.
                    if not self.refuse_arm:
                        self.state = 8
            elif not (requested == 8 and (self.refuse_arm or self.error)):
                self.state = requested
        elif command == 0x0D:
            self.velocity = struct.unpack("<ff", message.data)[0]
        self.emit()

    def recover(self, *, flags=5, marker=0xA3, generation=1, error=0x4000):
        self.online = True
        self.state, self.error = 1, error
        self.flags, self.marker, self.generation = flags, marker, generation
        self.velocity = 0.0
        self.emit()


class Rig:
    def __init__(self):
        self.now = 10.0
        self.buses = {}

        def drive(node):
            bus = self.buses[node] = AxisBus(node)
            bus.drive = DriveOdriveCan(node, bus=bus, clock=lambda: self.now)
            return bus.drive

        self.cm = ChassisManager(build_corners(lambda cid: FakeSteer(), drive), clock=lambda: self.now)
        self.cm.connect()
        for bus in self.buses.values():
            bus.emit()
        assert self.cm.arm()
        self.input()
        self.cm.tick()

    def input(self, *, received_s=None, active=True):
        stamp = self.now if received_s is None else received_s
        self.cm.set_manual_input_state(active, received_s=stamp, connection_session_id="server-1")
        self.cm.set(0.4, 0.0, received_s=stamp, steering=0.0, connection_session_id="server-1")

    def step(self, dt=0.02, *, refresh_input=True, received_s=None):
        self.now += dt
        for bus in self.buses.values():
            bus.emit()
        if refresh_input:
            self.input(received_s=received_s)
        self.cm.tick()

    def wait_rearmed(self):
        for _ in range(15):
            if self.cm.state()["can_recovery"]["state"] == "WAIT_INPUT":
                return
            self.step()
        raise AssertionError(self.cm.state()["can_recovery"])


def test_eligible_fault_stops_all_six_then_requires_receipt_after_rearm():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    assert rig.cm.mode == "ARMED"
    assert "can_recovery" in rig.cm.safety_snapshot().hold_sources
    assert all(bus.velocity == 0.0 and bus.state == 1 for bus in rig.buses.values())
    assert all(not any(m.arbitration_id & 31 == 0x18 for m in bus.sent[-2:]) for bus in rig.buses.values())
    rig.wait_rearmed()
    completed_s = rig.now
    assert all(bus.state == 8 and bus.velocity == 0.0 for bus in rig.buses.values())
    for _ in range(3):
        rig.step(received_s=completed_s)
        assert "can_recovery" in rig.cm.safety_snapshot().hold_sources
        assert all(bus.velocity == 0.0 for bus in rig.buses.values())
    rig.step()
    assert "can_recovery" not in rig.cm.safety_snapshot().hold_sources
    assert all(bus.velocity != 0.0 for bus in rig.buses.values())
    assert rig.cm.state()["can_recovery"]["count"] == 1


@pytest.mark.parametrize("kind", ["disarm", "estop", "input_loss", "timeout", "other_hold", "component"])
def test_pending_recovery_cancellation_never_restarts_on_later_input(kind):
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    if kind == "disarm":
        rig.cm.disarm()
    elif kind == "estop":
        rig.cm.estop("operator")
    elif kind == "input_loss":
        rig.cm.set_manual_input_state(False)
    elif kind == "timeout":
        rig.step(0.31, refresh_input=False)
    elif kind == "other_hold":
        rig.cm.set_motion_hold("us100_checking", True)
        rig.cm.set_motion_hold("us100_checking", False)
    else:
        rig.cm.set_component_enabled("robot_arm", False)
    for _ in range(20):
        rig.step()
    assert rig.cm.mode in ("IDLE", "ESTOP")
    assert all(bus.state == 1 and bus.velocity == 0.0 for bus in rig.buses.values())
    assert rig.cm.state()["can_recovery"]["count"] == 0


@pytest.mark.parametrize("kwargs", [{"marker": 0}, {"marker": 0xA2}, {"flags": 1}, {"error": 0x4001}, {"error": 0x40, "flags": 0}])
def test_unmarked_intentional_or_mixed_fault_is_not_auto_recoverable(kwargs):
    rig = Rig()
    rig.buses[13].recover(**kwargs)
    rig.cm.tick()
    for _ in range(15):
        rig.step()
    assert rig.cm.mode == "ESTOP"
    assert rig.cm.state()["can_recovery"]["count"] == 0


def test_missing_feedback_waits_for_explicit_fresh_firmware_evidence():
    rig = Rig()
    rig.buses[13].online = False
    rig.step(0.21)
    assert rig.cm.mode == "ARMED"
    assert rig.cm.state()["can_recovery"]["state"] == "WAIT_STOP"
    rig.buses[13].recover(flags=7)
    for _ in range(12):
        rig.step()
    assert rig.cm.state()["can_recovery"]["state"] == "WAIT_STOP"
    rig.buses[13].recover()
    rig.wait_rearmed()
    rig.step()
    assert rig.cm.state()["can_recovery"]["count"] == 1


def test_generation_change_restarts_stability_without_extending_five_second_budget():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    for index in range(30):
        rig.buses[13].recover(generation=index + 2)
        rig.step(0.18)
    assert rig.cm.mode == "IDLE"
    assert rig.cm.state()["can_recovery"]["state"] == "CANCELLED"
    assert rig.cm.state()["can_recovery"]["count"] == 0


def test_failing_fresh_state8_confirmation_latches_estop():
    rig = Rig()
    rig.buses[13].recover()
    rig.buses[15].refuse_arm = True
    rig.cm.tick()
    for _ in range(15):
        rig.step()
    assert rig.cm.mode == "ESTOP"
    assert all(bus.velocity == 0.0 for bus in rig.buses.values())


def test_steering_fault_stays_strict_during_can_recovery():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    rig.cm.corners["front_left"].steer.state = lambda: {"fault": 1, "stale": False, "cur_a": 0}
    rig.step()
    assert rig.cm.mode == "ESTOP"


def test_non_manual_source_cannot_enter_auto_recovery():
    rig = Rig()
    rig.cm.set_manual_input_state(False)
    rig.buses[13].recover()
    rig.cm.tick()
    assert rig.cm.mode == "ESTOP"


def test_offline_multiple_ticks_and_buswide_steering_stale_wait_without_rearming():
    rig = Rig()
    rig.buses[13].online = False
    for corner in rig.cm.corners.values():
        if isinstance(corner.steer, FakeSteer):
            corner.steer.stale_flag = True
    rig.step(.21)
    for _ in range(15):
        rig.step()
    assert rig.cm.state()['can_recovery']['state'] == 'WAIT_STOP'
    assert rig.cm.mode == 'ARMED'
    rig.buses[13].recover()
    for _ in range(15):
        rig.step()
    assert rig.cm.state()['can_recovery']['state'] == 'WAIT_STOP'
    assert all(bus.state == 1 for bus in rig.buses.values())
    for corner in rig.cm.corners.values():
        if isinstance(corner.steer, FakeSteer):
            corner.steer.stale_flag = False
    rig.wait_rearmed()


def test_isolated_steering_stale_is_still_a_fault():
    rig = Rig()
    rig.cm.corners['front_left'].steer.stale_flag = True
    rig.step()
    assert rig.cm.mode == 'ESTOP'


def test_steering_can_send_exception_is_never_deferred_as_drive_recovery():
    rig = Rig()
    def fail():
        raise can.CanOperationError('steering TX failure')
    rig.cm.corners['front_left'].steer.tick = fail
    rig.step()
    assert rig.cm.mode == 'ESTOP'
    assert rig.cm.state()['can_recovery']['state'] != 'WAIT_STOP'


def test_fresh_drive_fault_arriving_after_preflight_does_not_get_corner_estop_race():
    rig = Rig()
    drive = rig.cm.corners['mid_left'].drive
    original_state = drive.state
    reads = []
    def state():
        reads.append(1)
        sample = original_state()
        if len(reads) == 1:
            rig.buses[13].recover()
        return sample
    drive.state = state
    rig.step()
    assert len(reads) == 1, 'corner drained again after coordinated preflight'
    assert rig.cm.mode == 'ARMED'
    rig.step()
    assert rig.cm.can_recovery_state()['state'] == 'WAIT_STOP'


def test_new_motor_failed_error_between_stable_evidence_and_clear_is_not_cleared():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    drive = rig.cm.corners['mid_left'].drive
    original_arm = drive.arm
    def arm(**kwargs):
        rig.buses[13].recover(error=0x40, flags=0)
        original_arm(**kwargs)
    drive.arm = arm
    clears_before = sum(m.arbitration_id & 31 == 0x18 for m in rig.buses[13].sent)
    for _ in range(15):
        rig.step()
    assert rig.cm.mode == 'ESTOP'
    assert rig.buses[13].error == 0x40
    assert sum(m.arbitration_id & 31 == 0x18 for m in rig.buses[13].sent) == clears_before


@pytest.mark.parametrize('marker', [0, 0xA3])
def test_stale_then_reboot_without_explicit_eligible_generation_never_arms(marker):
    rig = Rig()
    rig.buses[13].online = False
    rig.step(.21)
    rig.buses[13].recover(error=0, flags=0, marker=marker, generation=0)
    for _ in range(28):
        rig.step(.19)
    assert rig.cm.mode == 'IDLE'
    assert all(bus.state == 1 for bus in rig.buses.values())
    assert rig.cm.can_recovery_state()['count'] == 0


def test_republished_dds_receipt_does_not_refresh_original_tcp_command():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    rig.wait_rearmed()
    original_tcp_s = rig.now - .02
    for _ in range(4):
        rig.now += .02
        for bus in rig.buses.values():
            bus.emit()
        rig.cm.set_manual_input_state(True, received_s=original_tcp_s, connection_session_id="server-1")
        rig.cm.set(.4, 0., received_s=rig.now, source_received_s=original_tcp_s, steering=0., connection_session_id="server-1")
        rig.cm.tick()
        assert rig.cm.can_recovery_state()['state'] == 'WAIT_INPUT'
        assert all(bus.velocity == 0 for bus in rig.buses.values())
    rig.cm.set_manual_input_state(True, received_s=rig.now, connection_session_id="server-1")
    rig.cm.set(0., 0., received_s=rig.now, source_received_s=rig.now, steering=0., connection_session_id="server-1")
    rig.cm.tick()
    assert rig.cm.can_recovery_state()['state'] == 'RESUMED'
    assert all(bus.velocity == 0 for bus in rig.buses.values())


def test_automatic_rearm_never_uses_unconditional_clear_and_does_not_clear_error0_peers():
    rig = Rig()
    baseline = {n:len(bus.sent) for n,bus in rig.buses.items()}
    rig.buses[13].recover(generation=7)
    rig.cm.tick()
    rig.wait_rearmed()
    for n,bus in rig.buses.items():
        clears = [m for m in bus.sent[baseline[n]:] if m.arbitration_id & 31 == 24]
        assert clears == []
        arms = [m for m in bus.sent[baseline[n]:] if m.arbitration_id & 31 == 7
                and struct.unpack('<I', m.data[:4])[0] == 8]
        assert len(arms) == 1
        expected = struct.pack('<IBBBB', 8, 0xA3, 7, 0, 0) if n == 13 else struct.pack('<I', 8) + bytes(4)
        assert bytes(arms[0].data) == expected


def test_firmware_cas_refuses_fault_arriving_after_host_guard_before_clear():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    bus = rig.buses[13]
    send = bus.send
    def interrupt(message):
        if message.arbitration_id & 31 == 7 and message.data[4] == 0xA3:
            bus.error, bus.flags = 0x40, 0
        send(message)
    bus.send = interrupt
    for _ in range(15):
        rig.step()
    assert rig.cm.mode == 'ESTOP'
    assert bus.error == 0x40


def test_recovery_snapshot_is_not_healthy_until_new_input_releases_hold():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    assert rig.cm.snapshot().healthy is False
    rig.wait_rearmed()
    assert rig.cm.snapshot().healthy is False
    rig.step()
    assert rig.cm.snapshot().healthy is True


def test_external_manual_clear_between_host_guard_and_cas_cannot_become_auto_arm():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    bus = rig.buses[13]
    send = bus.send
    def revoke(message):
        if message.arbitration_id & 31 == 7 and message.data[4] == 0xA3:
            # A separate explicit clear revoked eligibility just before CAS.
            bus.error, bus.flags = 0, 0
        send(message)
    bus.send = revoke
    for _ in range(15):
        rig.step()
    assert rig.cm.mode == 'ESTOP'
    assert rig.cm.can_recovery_state()['count'] == 0


def test_wait_input_steer_stale_and_new_packet_never_sends_nonzero_to_peers():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    rig.wait_rearmed()
    baseline = {n:len(bus.sent) for n,bus in rig.buses.items()}
    rig.cm.corners['front_left'].steer.stale_flag = True
    rig.step()
    assert rig.cm.mode in ('IDLE', 'ESTOP')
    assert rig.cm.can_recovery_state()['count'] == 0
    for n,bus in rig.buses.items():
        assert all(struct.unpack('<ff', m.data)[0] == 0 for m in bus.sent[baseline[n]:]
                   if m.arbitration_id & 31 == 13)


def test_every_observed_missing_axis_needs_its_own_eligible_origin():
    rig = Rig()
    rig.buses[13].online = rig.buses[15].online = False
    rig.step(.21)
    rig.buses[13].recover()
    rig.buses[15].recover(error=0, flags=0, generation=0)
    for _ in range(20):
        rig.step()
    assert all(bus.state == 1 for bus in rig.buses.values())
    assert rig.cm.can_recovery_state()['count'] == 0
    # An actual later eligible event can provide the missing axis's proof.
    rig.buses[15].recover(generation=1)
    rig.wait_rearmed()
    rig.step()
    assert rig.cm.can_recovery_state()['count'] == 1


def test_eligible_origin_is_captured_before_the_first_stop_side_effect():
    rig = Rig()
    rig.buses[13].recover()
    rig.cm.tick()
    rig.buses[13].recover(error=0, flags=0, generation=0)
    rig.buses[15].recover()
    for _ in range(20):
        rig.step()
    assert rig.cm.mode == 'ESTOP'
    assert rig.cm.can_recovery_state()['count'] == 0


def test_new_server_connection_epoch_cancels_even_without_disconnected_snapshot():
    rig = Rig()
    rig.cm.set_manual_input_state(True, received_s=rig.now, connection_session_id='server-old')
    rig.buses[13].recover()
    rig.cm.tick()
    # Keep the same old server epoch through the successful hardware re-arm.
    for _ in range(15):
        rig.now += .02
        for bus in rig.buses.values():
            bus.emit()
        rig.cm.set_manual_input_state(True, received_s=rig.now, connection_session_id='server-old')
        rig.cm.set(.4,0,received_s=rig.now,source_received_s=rig.now,connection_session_id='server-old')
        rig.cm.tick()
        if rig.cm.can_recovery_state()['state'] == 'WAIT_INPUT':
            break
    assert rig.cm.can_recovery_state()['state'] == 'WAIT_INPUT'
    rig.now += .02
    rig.cm.set_manual_input_state(True, received_s=rig.now, connection_session_id='server-new')
    rig.cm.set(0.,0.,received_s=rig.now,source_received_s=rig.now,connection_session_id='server-new')
    rig.cm.tick()
    assert rig.cm.mode == 'IDLE'
    for _ in range(5):
        rig.step()
    assert all(bus.velocity == 0 for bus in rig.buses.values())
    assert rig.cm.can_recovery_state()['count'] == 0
