"""Origin registration exercises real drivers with an in-memory CAN bus."""
import struct
import time

import can
import pytest

from chassis.chassis_manager import ChassisManager, build_corners
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.null_steer import NullSteer
from corner_module.steer_ak40 import AK40, SteerAk40


class Bus:
    def __init__(self, motor_id=None):
        self.motor_id = motor_id
        self.sent = []
        self.pending = False
        self.confirm = True
        self.fail = False
        self.old_stamp = None

    def send(self, message, timeout=None):
        if self.fail:
            raise can.CanOperationError("injected send failure")
        self.sent.append(message)
        self.pending = True

    def recv(self, timeout=0):
        if not self.pending or self.motor_id is None or not self.confirm:
            return None
        self.pending = False
        return can.Message(
            arbitration_id=(41 << 8) | self.motor_id,
            is_extended_id=True,
            timestamp=self.old_stamp or time.time(),
            data=struct.pack(">hhhbb", 0, 0, 0, 25, 0))


def manager():
    def steer_factory(motor_id):
        if motor_id is None:
            return NullSteer()
        steer = SteerAk40(motor_id, invert=True)
        steer._bus = Bus(motor_id)
        steer._ak = AK40(steer._bus, motor_id)
        steer._ak.pos_out_deg = 12
        steer._target_deg = -12
        steer._record_feedback()
        return steer

    corners = build_corners(steer_factory,
                            lambda nid: DriveOdriveCan(nid, bus=Bus()))
    cm = ChassisManager(corners)
    cm.mode = "IDLE"
    for corner in corners.values():
        corner.mode = "IDLE"
        drive = corner.drive
        for cmd, payload in [(1, struct.pack("<IB3x", 0, 1)),
                             (9, struct.pack("<ff", 0, 0))]:
            drive._handle_rx(can.Message(
                arbitration_id=(drive._node_id << 5) | cmd,
                data=payload, is_extended_id=False))
    return cm


def steering(cm):
    return [corner.steer for corner in cm.corners.values()
            if isinstance(corner.steer, SteerAk40)]


def test_origin_with_command_recovery_preserves_drive_hold():
    cm = manager()
    cm.set_motion_hold("us100_checking", True)
    cm.set_motion_hold("us100_checking", False)
    assert "command_recovery" in cm.safety_snapshot().hold_sources
    accepted, reason = cm.register_steering_origin()
    assert accepted, reason
    assert "command_recovery" in cm.safety_snapshot().hold_sources
    assert cm.mode == "IDLE"
    assert all(not corner.drive._bus.sent for corner in cm.corners.values())


def test_origin_registers_all_four_without_motion_or_drive_commands():
    cm = manager()
    accepted, reason = cm.register_steering_origin()
    assert accepted, reason
    assert "changed_ids=1,2,3,4" in reason
    assert cm.mode == "IDLE"
    for steer in steering(cm):
        assert len(steer._bus.sent) == 1
        sent = steer._bus.sent[0]
        assert sent.arbitration_id == (5 << 8) | steer._motor_id
        assert bytes(sent.data) == b"\x01"
        assert steer.health_state()["target_deg"] == 0
        assert steer.health_state()["actual_deg"] == 0
    assert all(not corner.drive._bus.sent for corner in cm.corners.values())


@pytest.mark.parametrize("defect", ["armed", "disabled", "transition", "recovery",
                                   "drive_closed_loop", "drive_stale", "steer_stale",
                                   "steer_speed", "steer_current", "steer_fault"])
def test_preflight_rejects_before_any_origin_write(defect):
    cm = manager()
    steer = steering(cm)[-1]  # Late failure must not have written earlier peers.
    drive = next(iter(cm.corners.values())).drive
    if defect == "armed":
        cm.mode = "ARMED"
    elif defect == "disabled":
        cm._component_mask["steer"] = False
    elif defect == "transition":
        cm._pending_steering_mode = "skid"
    elif defect == "recovery":
        cm._can_recovery.begin(time.monotonic(), "test")
    elif defect == "drive_closed_loop":
        drive._axis_state = 8
    elif defect == "drive_stale":
        drive._last_encoder_ms -= 201
    elif defect == "steer_stale":
        steer._last_rx_ms -= 201
    elif defect == "steer_speed":
        steer._ak.spd_erpm = 201
    elif defect == "steer_current":
        steer._ak.cur_a = 5
    else:
        steer._ak.fault = 1
    assert cm.register_steering_origin()[0] is False
    assert all(not steer._bus.sent for steer in steering(cm))


@pytest.mark.parametrize("old_queued", [False, True])
def test_cached_or_pre_send_zero_never_confirms_registration(old_queued):
    cm = manager()
    for steer in steering(cm):
        steer._ak.pos_out_deg = 0
        steer._bus.confirm = old_queued
        steer._bus.old_stamp = time.time() - .01
    started = time.monotonic()
    accepted, reason = cm.register_steering_origin()
    assert accepted is False
    assert "unconfirmed_ids=1,2,3,4" in reason
    assert time.monotonic() - started < .3


def test_partial_send_stops_and_preserves_failed_driver_target():
    cm = manager()
    steers = steering(cm)
    steers[2]._bus.fail = True
    accepted, reason = cm.register_steering_origin()
    assert accepted is False
    assert "partial_send" in reason
    assert "possibly_changed_ids=1,2" in reason
    assert steers[2].health_state()["target_deg"] == -12
    assert not steers[3]._bus.sent
    assert cm.mode == "IDLE"
