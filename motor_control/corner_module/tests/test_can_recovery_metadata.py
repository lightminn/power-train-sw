import struct

import can
import pytest

from corner_module.drive_odrive_can import DriveOdriveCan


class Bus:
    def recv(self, timeout=0):
        return None


def heartbeat(drive, *, marker=0xA3, flags=5, generation=255, state=1, error=0x4000, short=False):
    data = struct.pack("<IBBBB", error, state, marker, flags, generation)
    return drive._handle_rx(can.Message(arbitration_id=(13 << 5) | 1,
                                       data=data[:5] if short else data, is_extended_id=False))


def test_patch3_metadata_is_decoded_without_changing_standard_heartbeat_fields():
    drive = DriveOdriveCan(13, bus=Bus(), clock=lambda: 10.0)
    assert heartbeat(drive)
    state = drive.health_state()
    assert state["axis_state"] == 1 and state["axis_error"] == 0x4000
    assert state["can_recovery_supported"] is True
    assert state["can_recovery_latched"] is True
    assert state["can_recovery_in_progress"] is False
    assert state["can_auto_resume_eligible"] is True
    assert state["can_recovery_generation"] == 255
    assert heartbeat(drive, flags=7, generation=0)
    assert drive.health_state()["can_recovery_in_progress"] is True
    assert drive.health_state()["can_recovery_generation"] == 0


@pytest.mark.parametrize("kwargs", [{"marker": 0}, {"marker": 0xA2}, {"short": True}, {"flags": 0x85}])
def test_unmarked_or_unknown_heartbeat_revokes_cached_eligibility(kwargs):
    drive = DriveOdriveCan(13, bus=Bus(), clock=lambda: 10.0)
    heartbeat(drive)
    heartbeat(drive, **kwargs)
    state = drive.health_state()
    assert state["can_recovery_supported"] is False
    assert state["can_auto_resume_eligible"] is False
    assert state["can_recovery_generation"] is None
