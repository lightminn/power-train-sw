"""Maintenance uses actual kernel locks across process boundaries."""
import os
from pathlib import Path
import subprocess
import sys

import pytest

from chassis.runtime_lock import CanMaintenanceSession, CanOwnershipError, RealCanSession


def test_maintenance_excludes_other_owners_and_other_resetters(tmp_path):
    path = str(tmp_path / "can0.lock")
    with CanMaintenanceSession(path=path) as maintenance:
        assert maintenance.mark_reset() == 1
        code = '''from chassis.runtime_lock import RealCanSession, CanMaintenanceSession, CanOwnershipError
import sys
for factory in (RealCanSession, CanMaintenanceSession):
    try:
        with factory(path=sys.argv[1]):
            raise AssertionError("competing ownership was granted")
    except CanOwnershipError:
        pass
print("both refused")
'''
        result = subprocess.run([sys.executable, "-c", code, path], capture_output=True, text=True)
        assert result.returncode == 0, result.stderr
        assert result.stdout.strip() == "both refused"
    with RealCanSession(path=path):
        pass
    assert Path(path).exists() and Path(path + ".reset").exists()
    with CanMaintenanceSession(path=path) as maintenance:
        assert maintenance.mark_reset() == 2


def test_mismatched_owner_cannot_authorize_different_link_reset(tmp_path):
    with RealCanSession(channel="can0", path=str(tmp_path / "can0.lock")) as owner:
        with pytest.raises(CanOwnershipError):
            with CanMaintenanceSession(channel="can1", owner_session=owner):
                pytest.fail("different CAN link authorized")


def test_maintenance_releases_lock_after_failed_mutation(tmp_path):
    path = str(tmp_path / "can0.lock")
    with pytest.raises(OSError):
        with CanMaintenanceSession(path=path) as maintenance:
            maintenance.mark_reset()
            raise OSError("link up failed")
    with CanMaintenanceSession(path=path) as maintenance:
        assert maintenance.generation == 1
