"""Host-only behavior tests for phased ODrive CAN feedback queries."""
from collections import Counter, defaultdict
import struct

import can

from chassis.chassis_manager import ChassisManager
from chassis.kinematics import default_geometry
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
import corner_module.drive_odrive_can as drive_module
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.null_steer import NullSteer


ENCODER = 0x09
SET_INPUT_VEL = 0x0D
IQ = 0x14


class ManualClock:
    def __init__(self):
        self.now_s = 0.0

    def __call__(self):
        return self.now_s


class RecordingBus:
    def __init__(self, clock):
        self.clock = clock
        self.sent = []
        self.rx = []

    def send(self, message):
        self.sent.append((self.clock.now_s, message))

    def recv(self, timeout=0.0):
        return self.rx.pop(0) if self.rx else None


class ArmReplyBus(RecordingBus):
    """Realistic fake: state request yields heartbeat; encoder needs its RTR."""

    def __init__(self, clock, node):
        super().__init__(clock)
        self.node = node

    def send(self, message):
        super().send(message)
        command = _cmd(message)
        if command == 0x07 and message.data[0] == 8:
            self.rx.append(can.Message(
                arbitration_id=(self.node << 5) | 0x01,
                data=struct.pack("<IB3x", 0, 8),
                is_extended_id=False,
            ))
        elif command == ENCODER and message.is_remote_frame:
            self.rx.append(can.Message(
                arbitration_id=(self.node << 5) | ENCODER,
                data=struct.pack("<ff", 0.0, 0.0),
                is_extended_id=False,
            ))


def _cmd(message):
    return message.arbitration_id & 0x1F


def _node(message):
    return message.arbitration_id >> 5


def _queries(bus, command):
    return [
        (stamp, _node(message))
        for stamp, message in bus.sent
        if message.is_remote_frame and _cmd(message) == command
    ]


def _drives(clock, bus):
    return [DriveOdriveCan(node_id=node, bus=bus, clock=clock) for node in range(11, 17)]


def _jitter_times(count):
    now_s = 0.0
    intervals = (0.015, 0.015, 0.030)
    for step in range(count):
        yield now_s
        now_s += intervals[step % len(intervals)]


def test_chassis_cycle_scheduler_prevents_jitter_phase_starvation():
    """Catches absolute-slot matching that permanently starves phase 2."""
    clock = ManualClock()
    bus = RecordingBus(clock)
    corners = {
        wheel.name: CornerModule(
            NullSteer(),
            DriveOdriveCan(node_id=node, bus=bus, clock=clock),
            CornerConfig(),
        )
        for wheel, node in zip(default_geometry().wheels, range(11, 17))
    }
    manager = ChassisManager(corners, clock=clock)
    manager.connect()

    for now_s in _jitter_times(100):
        clock.now_s = now_s
        manager.tick()

    encoder = _queries(bus, ENCODER)
    encoder_per_step = Counter(round(stamp, 6) for stamp, _ in encoder)
    assert max(encoder_per_step.values()) <= 2
    encoder_times = defaultdict(list)
    for stamp, node in encoder:
        encoder_times[node].append(stamp)
    assert set(encoder_times) == set(range(11, 17))
    assert all(
        later - earlier <= 0.080000001
        for times in encoder_times.values()
        for earlier, later in zip(times, times[1:])
    )


def test_arm_services_three_bounded_cycles_for_initial_encoder_coverage():
    """Catches arm confirmation waiting without scheduling remaining phases."""
    clock = ManualClock()
    buses = [ArmReplyBus(clock, node) for node in range(11, 17)]
    scheduler = drive_module.CanFeedbackScheduler()
    corners = {
        wheel.name: CornerModule(
            NullSteer(),
            DriveOdriveCan(
                node_id=bus.node,
                bus=bus,
                clock=clock,
                feedback_scheduler=scheduler,
            ),
            CornerConfig(),
        )
        for wheel, bus in zip(default_geometry().wheels, buses)
    }
    manager = ChassisManager(corners, clock=clock)
    manager.connect()

    assert manager.arm() is True
    encoder_counts = Counter(
        _node(message)
        for bus in buses
        for _, message in bus.sent
        if message.is_remote_frame and _cmd(message) == ENCODER
    )
    assert encoder_counts == Counter({node: 1 for node in range(11, 17)})


def test_explicit_shared_scheduler_bounds_extra_per_axis_calls():
    """Catches arm/tick/poll calls independently advancing the six-axis phase."""
    clock = ManualClock()
    bus = RecordingBus(clock)
    scheduler = drive_module.CanFeedbackScheduler()
    drives = [
        DriveOdriveCan(
            node_id=node,
            bus=bus,
            clock=clock,
            feedback_scheduler=scheduler,
        )
        for node in range(11, 17)
    ]

    scheduler.begin_cycle()
    for drive in drives:
        drive.poll_feedback()
        drive.tick()
        drive.poll_feedback()

    assert len(_queries(bus, ENCODER)) == 2
    assert len(_queries(bus, IQ)) <= 1


def test_standalone_driver_uses_overdue_deadlines_instead_of_exact_slots():
    """Catches phase-2 starvation when no multi-axis coordinator is supplied."""
    clock = ManualClock()
    bus = RecordingBus(clock)
    drive = DriveOdriveCan(node_id=13, bus=bus, clock=clock)

    for now_s in _jitter_times(100):
        clock.now_s = now_s
        drive.poll_feedback()

    encoder_times = [stamp for stamp, _ in _queries(bus, ENCODER)]
    assert encoder_times
    assert encoder_times[0] <= 0.080
    assert all(
        later - earlier <= 0.080000001
        for earlier, later in zip(encoder_times, encoder_times[1:])
    )

    bus.sent.clear()
    clock.now_s += 10.0
    drive.poll_feedback()
    assert len(_queries(bus, ENCODER)) == 1
    assert len(_queries(bus, IQ)) <= 1


def test_six_axis_feedback_queries_are_phased_and_bounded_at_50_hz():
    """Catches per-axis polling that emits six encoder or Iq requests in one tick."""
    clock = ManualClock()
    bus = RecordingBus(clock)
    drives = _drives(clock, bus)

    for step in range(100):
        clock.now_s = step * 0.020
        for drive in drives:
            drive.tick()

    encoder = _queries(bus, ENCODER)
    encoder_per_step = Counter(round(stamp, 3) for stamp, _ in encoder)
    assert max(encoder_per_step.values()) <= 2

    encoder_times = defaultdict(list)
    for stamp, node in encoder:
        encoder_times[node].append(stamp)
    assert set(encoder_times) == set(range(11, 17))
    assert all(times[0] <= 0.080 for times in encoder_times.values())
    assert all(
        later - earlier <= 0.080000001
        for times in encoder_times.values()
        for earlier, later in zip(times, times[1:])
    )

    iq = _queries(bus, IQ)
    iq_per_step = Counter(round(stamp, 3) for stamp, _ in iq)
    assert max(iq_per_step.values()) <= 1
    iq_per_node = Counter(node for _, node in iq)
    assert set(iq_per_node) == set(range(11, 17))
    assert all(1 <= count <= 2 for count in iq_per_node.values())

    velocity_per_node = Counter(
        _node(message)
        for _, message in bus.sent
        if not message.is_remote_frame and _cmd(message) == SET_INPUT_VEL
    )
    assert velocity_per_node == Counter({node: 100 for node in range(11, 17)})


def test_tick_always_commands_velocity_but_repeated_calls_share_one_slot():
    """Catches feedback scheduling that throttles control or repeats within a slot."""
    clock = ManualClock()
    bus = RecordingBus(clock)
    drive = DriveOdriveCan(node_id=11, bus=bus, clock=clock)
    drive.set_velocity(0.4)

    for now_s in (0.000, 0.005, 0.010, 0.019):
        clock.now_s = now_s
        drive.tick()

    commands = [
        message for _, message in bus.sent
        if not message.is_remote_frame and _cmd(message) == SET_INPUT_VEL
    ]
    assert len(commands) == 4
    assert all(struct.unpack("<ff", bytes(message.data))[0] == 2.0 for message in commands)
    assert len(_queries(bus, ENCODER)) == 1
    assert len(_queries(bus, IQ)) <= 1


def test_tick_then_poll_feedback_does_not_duplicate_same_slot_queries():
    """Catches separate tick and idle poll rate limiters issuing duplicate RTRs."""
    clock = ManualClock()
    bus = RecordingBus(clock)
    drive = DriveOdriveCan(node_id=11, bus=bus, clock=clock)

    drive.tick()
    drive.poll_feedback()

    assert len(_queries(bus, ENCODER)) == 1
    assert len(_queries(bus, IQ)) <= 1


def test_ten_second_clock_jump_never_catches_up_missed_query_slots():
    """Catches schedulers that replay every missed query after a long delay."""
    clock = ManualClock()
    bus = RecordingBus(clock)
    scheduler = drive_module.CanFeedbackScheduler()
    drives = [
        DriveOdriveCan(
            node_id=node,
            bus=bus,
            clock=clock,
            feedback_scheduler=scheduler,
        )
        for node in range(11, 17)
    ]
    scheduler.begin_cycle()
    for drive in drives:
        drive.poll_feedback()

    bus.sent.clear()
    clock.now_s = 10.0
    scheduler.begin_cycle()
    for drive in drives:
        drive.poll_feedback()

    assert len(_queries(bus, ENCODER)) <= 2
    assert len(_queries(bus, IQ)) <= 1


def test_repeated_stop_does_not_reset_all_axes_to_immediate_polling():
    """Catches estop resetting per-axis query state into a six-axis burst."""
    clock = ManualClock()
    bus = RecordingBus(clock)
    drives = _drives(clock, bus)
    for drive in drives:
        drive.poll_feedback()
    for drive in drives:
        drive.estop()
        drive.estop()

    bus.sent.clear()
    for drive in drives:
        drive.poll_feedback()

    assert _queries(bus, ENCODER) == []
    assert _queries(bus, IQ) == []


def test_unanswered_queries_do_not_refresh_feedback_freshness():
    """Catches TX query timestamps being mistaken for actual received feedback."""
    clock = ManualClock()
    bus = RecordingBus(clock)
    drive = DriveOdriveCan(node_id=11, bus=bus, clock=clock, stale_ms=200.0)
    bus.rx.extend([
        can.Message(
            arbitration_id=(11 << 5) | 0x01,
            data=struct.pack("<IB3x", 0, 8),
            is_extended_id=False,
        ),
        can.Message(
            arbitration_id=(11 << 5) | ENCODER,
            data=struct.pack("<ff", 0.0, 0.0),
            is_extended_id=False,
        ),
    ])
    drive.tick()
    assert drive.health_state()["stale"] is False

    clock.now_s = 0.220
    drive.poll_feedback()

    health = drive.health_state()
    assert health["stale"] is True
    assert health["encoder_stale"] is True
    assert health["heartbeat_stale"] is True
