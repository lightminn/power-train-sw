"""Demo mode (2026-11-06 spec D1): drive-axis faults stop instead of latching.

Six real CAN drivers on the deterministic bus from the CAN auto-recovery tests.
"""
import pytest

from chassis.chassis_manager import ChassisConfig, ChassisManager, build_corners
from chassis.tests.test_can_auto_recovery import AxisBus
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.fake import FakeSteer


class DemoRig:
    def __init__(self, demo=True):
        self.now = 10.0
        self.buses = {}

        def drive(node):
            bus = self.buses[node] = AxisBus(node)
            bus.drive = DriveOdriveCan(node, bus=bus, clock=lambda: self.now)
            return bus.drive

        self.cm = ChassisManager(build_corners(lambda cid: FakeSteer(), drive),
                                 cfg=ChassisConfig(demo_fault_stop=demo),
                                 clock=lambda: self.now)
        self.cm.connect()
        for bus in self.buses.values():
            bus.emit()
        assert self.cm.arm()
        self.input()
        self.cm.tick()

    def input(self):
        self.cm.set_manual_input_state(True, received_s=self.now, connection_session_id="server-1")
        self.cm.set(0.4, 0.0, received_s=self.now, steering=0.0, connection_session_id="server-1")

    def step(self, dt=0.02):
        self.now += dt
        for bus in self.buses.values():
            bus.emit()
        self.input()
        self.cm.tick()

    def fault(self, node, error, state=1):
        bus = self.buses[node]
        bus.state, bus.error = state, error
        bus.emit()

    def clears_sent(self):
        return sum(1 for bus in self.buses.values()
                   for m in bus.sent if m.arbitration_id & 31 == 0x18)


@pytest.mark.parametrize("error", [0x40, 0x800, 0x840, 0])
def test_demo_idle_drive_fault_stops_without_latching(error):
    rig = DemoRig()
    seq = rig.cm.state()["disarm_seq"]
    clears = rig.clears_sent()
    rig.fault(15, error)
    rig.step()
    safety = rig.cm.safety_snapshot()
    assert rig.cm.mode == "IDLE"
    assert not safety.estop_latched
    state = rig.cm.state()
    assert state["disarm_seq"] == seq + 1
    assert state["disarm_cause"] == "demo_fault"
    assert "node15" in state["disarm_detail"]
    assert all(bus.state == 1 and bus.velocity == 0.0 for bus in rig.buses.values())
    assert rig.clears_sent() == clears  # the chassis never clears the error itself


def test_demo_stop_never_rearms_by_itself():
    rig = DemoRig()
    rig.fault(15, 0x40)
    rig.step()
    rig.fault(15, 0)
    for _ in range(20):
        rig.step()
    assert rig.cm.mode == "IDLE"
    assert all(bus.state == 1 for bus in rig.buses.values())


@pytest.mark.parametrize("error,state", [(0x100, 1), (0x40 | 0x1, 1), (0x40, 8)])
def test_demo_keeps_latching_for_other_drive_faults(error, state):
    rig = DemoRig()
    rig.fault(15, error, state)
    rig.step()
    assert rig.cm.safety_snapshot().estop_latched


def test_without_demo_mode_motor_failed_still_latches():
    rig = DemoRig(demo=False)
    rig.fault(15, 0x40)
    rig.step()
    assert rig.cm.safety_snapshot().estop_latched


def test_third_demo_stop_within_window_latches():
    rig = DemoRig()
    for attempt in range(3):
        rig.fault(15, 0x40)
        rig.step()
        if attempt < 2:
            assert rig.cm.mode == "IDLE" and not rig.cm.safety_snapshot().estop_latched
            rig.fault(15, 0)
            assert rig.cm.arm()
            rig.step()
    assert rig.cm.safety_snapshot().estop_latched
    assert rig.cm.safety_snapshot().first_source == "demo_fault_stop_budget"


def test_command_disarm_counts_as_command_cause():
    rig = DemoRig()
    seq = rig.cm.state()["disarm_seq"]
    rig.cm.disarm()
    assert rig.cm.state()["disarm_seq"] == seq + 1
    assert rig.cm.state()["disarm_cause"] == "command"


def test_no_clear_arm_requires_matching_seq_and_zero_errors():
    rig = DemoRig()
    rig.cm.disarm()
    seq = rig.cm.state()["disarm_seq"]
    clears = rig.clears_sent()
    assert not rig.cm.arm_no_clear(expect_disarm_seq=seq + 1)   # someone disarmed since
    assert rig.cm.mode == "IDLE"
    rig.fault(13, 0x40)
    assert not rig.cm.arm_no_clear(expect_disarm_seq=seq)        # error still present
    assert rig.cm.mode == "IDLE" and not rig.cm.safety_snapshot().estop_latched
    rig.fault(13, 0)
    assert rig.cm.arm_no_clear(expect_disarm_seq=seq)
    assert rig.cm.mode == "ARMED"
    assert rig.clears_sent() == clears                           # armed without CLEAR_ERRORS


def test_no_clear_arm_refuses_a_drive_that_does_not_report_axis_error():
    from corner_module.fake import FakeDrive
    cm = ChassisManager(build_corners(lambda cid: FakeSteer(), lambda node: FakeDrive()),
                        cfg=ChassisConfig(demo_fault_stop=True))
    cm.connect()
    for corner in cm.corners.values():
        corner.drive.state = lambda: {"target_vel": 0.0, "actual_vel": 0.0, "cur_a": 0.0, "stale": False}
    assert not cm.arm_no_clear(expect_disarm_seq=cm.state()["disarm_seq"])
    assert cm.mode == "IDLE"


def test_demo_stop_does_not_depend_on_manual_input_freshness():
    # chassis_node compares a CLOCK_MONOTONIC gateway stamp with its
    # CLOCK_MONOTONIC_RAW steady clock, so the manual input can look stale
    # while the robot is driving. D1 must still stop without latching then.
    rig = DemoRig()
    rig.cm.set_manual_input_state(False)
    rig.cm.set(0.4, 0.0, received_s=rig.now, steering=0.0)
    rig.fault(15, 0x40)
    rig.now += 0.02
    for bus in rig.buses.values():
        bus.emit()
    rig.cm.set_manual_input_state(False)
    rig.cm.tick()
    assert not rig.cm.safety_snapshot().estop_latched
    assert rig.cm.mode == "IDLE"
    assert rig.cm.state()["disarm_cause"] == "demo_fault"


def test_stale_input_without_demo_mode_keeps_the_corner_fault_latch():
    rig = DemoRig(demo=False)
    rig.cm.set_manual_input_state(False)
    rig.fault(15, 0x40)
    rig.now += 0.02
    for bus in rig.buses.values():
        bus.emit()
    rig.cm.tick()
    assert rig.cm.safety_snapshot().estop_latched
    assert rig.cm.safety_snapshot().first_source == "corner_fault"


def _idle_rig():
    rig = DemoRig()
    rig.cm.disarm()
    for bus in rig.buses.values():
        bus.emit()
    return rig


def test_demo_manual_arm_never_clears_and_refuses_an_existing_error():
    rig = _idle_rig()
    rig.fault(13, 0x100)              # outside the demo mask: needs a human
    clears = rig.clears_sent()
    assert not rig.cm.arm()
    assert rig.clears_sent() == clears
    assert rig.cm.mode == "IDLE" and not rig.cm.safety_snapshot().estop_latched
    rig.fault(13, 0)
    assert rig.cm.arm()
    assert rig.clears_sent() == clears


def test_no_clear_arm_rechecks_each_axis_just_before_arming_it():
    rig = _idle_rig()
    seq = rig.cm.state()["disarm_seq"]
    names = list(rig.cm.corners)
    last = rig.cm.corners[names[-1]].drive
    first_steer = rig.cm.corners[names[0]].steer
    original = first_steer.arm

    def arm_and_fault():
        original()
        rig.fault(last.health_state()["node_id"], 0x40)   # appears after the pre-check
    first_steer.arm = arm_and_fault
    assert not rig.cm.arm_no_clear(expect_disarm_seq=seq)
    assert not rig.cm.safety_snapshot().estop_latched
    assert rig.cm.mode == "IDLE"
    assert all(bus.state == 1 for bus in rig.buses.values())


def test_no_clear_arm_is_refused_for_non_can_drives():
    from corner_module.fake import FakeDrive
    cm = ChassisManager(build_corners(lambda cid: FakeSteer(), lambda node: FakeDrive()),
                        cfg=ChassisConfig(demo_fault_stop=True))
    cm.connect()
    assert not cm.arm_no_clear(expect_disarm_seq=cm.state()["disarm_seq"])
    assert cm.mode == "IDLE" and not cm.safety_snapshot().estop_latched
    assert cm.arm()                  # demo manual start still works on non-CAN drives


def test_demo_stop_is_not_used_when_another_axis_feedback_is_stale():
    rig = DemoRig()
    rig.buses[11].emit = lambda: None          # node11 goes silent but still looks ARMED
    rig.now += 0.25
    for node, bus in rig.buses.items():
        bus.emit()
    rig.fault(15, 0x40)
    rig.input()
    rig.cm.tick()
    assert rig.cm.safety_snapshot().estop_latched
    assert rig.cm.state()["disarm_cause"] != "demo_fault"


def test_estop_and_reset_invalidate_an_old_compare_and_arm():
    rig = DemoRig()
    rig.fault(15, 0x40)
    rig.step()
    old = rig.cm.state()["disarm_seq"]
    rig.fault(15, 0)
    rig.cm.estop("console", "operator")
    assert rig.cm.state()["disarm_seq"] > old
    rig.cm.reset_estop()
    for bus in rig.buses.values():
        bus.emit()
    assert not rig.cm.arm_no_clear(expect_disarm_seq=old)
    assert rig.cm.mode != "ARMED"
    assert rig.cm.state()["disarm_cause"] in ("estop", "estop_reset")
