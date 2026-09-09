import struct

import can
import pytest

from chassis.chassis_manager import ChassisManager, build_corners
from corner_module.drive_odrive_can import DriveOdriveCan
from corner_module.fake import FakeDrive, FakeSteer
from corner_module.null_steer import NullSteer


class Bus:
    def recv(self, timeout=0):
        return None
    def send(self, message):
        pass


def hardware_manager():
    now = [10.0]
    corners = build_corners(lambda cid: NullSteer(),
                            lambda nid: DriveOdriveCan(nid, bus=Bus(), clock=lambda: now[0]))
    cm = ChassisManager(corners, clock=lambda: now[0])
    cm.connect()
    for corner in corners.values():
        node = corner.drive._node_id
        for cmd, payload in [(1, struct.pack('<IB3x', 0, 1)), (9, struct.pack('<ff', 0, 0))]:
            corner.drive._handle_rx(can.Message(arbitration_id=(node << 5) | cmd,
                                                data=payload, is_extended_id=False))
    return cm, now


def test_fake_drivers_never_certify_hardware_stop():
    cm = ChassisManager(build_corners(lambda cid: FakeSteer(), lambda nid: FakeDrive()))
    assert cm.hardware_stop_proof('can')['valid'] is False


def test_hardware_stop_proof_uses_exact_nodes_and_actual_feedback_age():
    cm, now = hardware_manager()
    now[0] += .05
    proof = cm.hardware_stop_proof('can')
    assert proof == {'source': 'chassis_can', 'valid': True, 'stopped': True,
                     'node_ids': [11, 12, 13, 14, 15, 16],
                     'max_feedback_age_ms': pytest.approx(50)}


@pytest.mark.parametrize('defect', ['stale', 'missing_encoder', 'moving', 'error', 'calibrating', 'disabled', 'duplicate_id'])
def test_hardware_stop_proof_rejects_missing_or_unsafe_evidence(defect):
    cm, now = hardware_manager()
    drive = next(iter(cm.corners.values())).drive
    if defect == 'stale':
        now[0] += .21
    elif defect == 'missing_encoder':
        drive._last_encoder_ms = None
    elif defect == 'moving':
        drive._actual_vel = 5.0
    elif defect == 'error':
        drive._axis_error = 1
    elif defect == 'calibrating':
        drive._axis_state = 3
    elif defect == 'disabled':
        cm.set_component_enabled('drive', False)
    elif defect == 'duplicate_id':
        drive._node_id = 12
    proof = cm.hardware_stop_proof('can')
    assert proof['stopped'] is False
    if defect != 'moving':
        assert proof['valid'] is False


def test_external_command_receipt_age_is_preserved_by_manager_watchdog():
    now = [10.0]
    cm = ChassisManager(build_corners(lambda cid: FakeSteer(), lambda nid: FakeDrive()),
                        clock=lambda: now[0])
    cm.connect()
    assert cm.arm()
    cm.set(.5, 0, received_s=9.8)
    now[0] = 10.101
    cm.tick()
    assert 'cmd_watchdog' in cm.safety_snapshot().hold_sources
