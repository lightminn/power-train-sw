import importlib
from pathlib import Path
import runpy
import sys
from types import SimpleNamespace

import pytest

from drive.bl70200 import bl70200_setup as setup
from drive.bl70200.tests.test_bl70200_setup import _board, _patch_apply_runtime


def board(serial=0x3352, nodes=(13, 14)):
    b = _board()
    b.serial_number = serial
    b.can.config = SimpleNamespace(baud_rate=500000, protocol=0)
    b.can.set_baud_rate = lambda value: setattr(b.can.config, "baud_rate", value)
    b.saves = 0
    def save():
        b.saves += 1
    b.save_configuration = save
    for axis, node in zip((b.axis0, b.axis1), nodes):
        axis.config.can_node_id = node
        axis.config.can_extended_id = False
        axis.motor.config.pre_calibrated = True
        axis.encoder.config.pre_calibrated = True
    return b


class Boards:
    def __init__(self, *boards):
        self.boards = list(boards)
        self.calls = []

    def find_any(self, **kwargs):
        self.calls.append(kwargs)
        return self.boards.pop(0) if len(self.boards) > 1 else self.boards[0]


@pytest.fixture
def lock_path(monkeypatch, tmp_path):
    from chassis import runtime_lock
    cls = runtime_lock.RealCanSession
    path = str(tmp_path / "can0.lock")
    monkeypatch.setattr(runtime_lock, "RealCanSession",
                        lambda **kw: cls(path=path, **kw))
    return path


@pytest.mark.parametrize("argv", [
    ["--apply"], ["--calibrate"], ["--apply", "--serial", "3352"],
    ["--apply", "--serial", "3352", "--axis", "1"],
    ["--persist-calibration", "--serial", "3352"],
    ["--apply", "--serial", "3352", "--axis", "both", "--node", "63"],
    ["--apply", "--serial", "3352", "--axis", "0", "--node", "-1"],
])
def test_writing_cli_rejected_before_any_usb_discovery(argv):
    class ForbiddenDiscovery:
        def find_any(self, **kwargs):
            pytest.fail("USB discovery reached with an incomplete/invalid target")
    with pytest.raises(SystemExit):
        setup.main(argv, odrive_module=ForbiddenDiscovery())


def test_setup_refuses_busy_can_owner_before_find(lock_path):
    from chassis.runtime_lock import RealCanSession, CanOwnershipError
    devices = Boards(board())
    with RealCanSession(owner="can owner"):
        with pytest.raises(CanOwnershipError):
            setup.main(["--calibrate", "--serial", "3352", "--axis", "both", "--node", "13"],
                       odrive_module=devices)
    assert not devices.calls


def test_read_reports_serial_both_axes_and_can_configuration(capsys):
    setup.main(["--read"], odrive_module=Boards(board()))
    text = capsys.readouterr().out
    for expected in ("3352", "axis0", "axis1", "node=13", "node=14",
                     "baud=500000", "protocol=0", "extended=False", "heartbeat_ms=100"):
        assert expected in text


@pytest.mark.parametrize("changed", ["serial", "sibling_node", "baud", "protocol", "extended", "heartbeat"])
def test_apply_rejects_reenumeration_identity_or_comms_drift(monkeypatch, lock_path, changed):
    original, rebooted = board(), board()
    if changed == "serial":
        rebooted.serial_number = 0x9999
    elif changed == "sibling_node":
        rebooted.axis0.config.can_node_id = 15
    elif changed == "baud":
        rebooted.can.config.baud_rate = 250000
    elif changed == "protocol":
        rebooted.can.config.protocol = 42
    elif changed == "extended":
        rebooted.axis0.config.can_extended_id = True
    elif changed == "heartbeat":
        rebooted.axis0.config.can_heartbeat_rate_ms = 0
    rebooted.axis1.config.can_heartbeat_rate_ms = 20
    _patch_apply_runtime(monkeypatch, setup)
    devices = Boards(original, rebooted)
    with pytest.raises(ValueError, match="serial|communication|configuration"):
        setup.main(["--apply", "--serial", "3352", "--axis", "1", "--node", "14"],
                   odrive_module=devices)
    assert original.saves == 1
    assert all(c["serial_number"] == "3352" for c in devices.calls)


def test_apply_refuses_node_collision_before_write(monkeypatch, lock_path):
    b = board()
    _patch_apply_runtime(monkeypatch, setup)
    with pytest.raises(ValueError, match="duplicate|collision"):
        setup.main(["--apply", "--serial", "3352", "--axis", "1", "--node", "13"],
                   odrive_module=Boards(b))
    assert b.saves == 0
    assert b.axis1.config.can_node_id == 14


def test_apply_supports_fw_056_nested_config_and_baud_property(monkeypatch, lock_path):
    b = board()
    b.fw_version_revision = 6
    b.can.config.protocol = 1
    del b.can.set_baud_rate
    for ax in (b.axis0, b.axis1):
        old = ax.config
        ax.config = SimpleNamespace(can=SimpleNamespace(
            node_id=old.can_node_id, is_extended=False,
            heartbeat_rate_ms=100, encoder_rate_ms=10, iq_rate_ms=0))
    _patch_apply_runtime(monkeypatch, setup)
    assert setup.main(["--apply", "--serial", "3352", "--axis", "both", "--node", "13"],
                      odrive_module=Boards(b)) == 0
    assert b.axis0.config.can.heartbeat_rate_ms == 20
    assert b.axis1.config.can.node_id == 14


def test_unknown_comms_blocks_save_before_any_motor_tuning(monkeypatch, lock_path):
    b = board()
    del b.can.config.protocol
    _patch_apply_runtime(monkeypatch, setup)
    with pytest.raises(ValueError, match="unavailable"):
        setup.main(["--apply", "--serial", "3352", "--axis", "both", "--node", "13"],
                   odrive_module=Boards(b))
    assert b.saves == 0
    assert b.axis0.config.can_heartbeat_rate_ms == 100


def test_fw051_official_extended_id_field_is_read():
    from chassis.usb_session import communication_snapshot
    b = board()
    for axis in (b.axis0, b.axis1):
        del axis.config.can_extended_id
        axis.config.can_node_id_extended = False
    snap = communication_snapshot(b, strict=True)
    assert snap["axis0"]["extended"] is False
    assert snap["axis1"]["extended"] is False


def test_fw051_simple_protocol_zero_is_preserved(monkeypatch, lock_path):
    b = board()
    b.can.config.protocol = 0
    _patch_apply_runtime(monkeypatch, setup)
    assert setup.main(["--apply", "--serial", "3352", "--axis", "both", "--node", "13"],
                      odrive_module=Boards(b)) == 0
    assert b.can.config.protocol == 0


def test_explicit_both_axis_setup_can_repair_factory_duplicate_nodes(monkeypatch, lock_path):
    b = board(nodes=(0, 0))
    _patch_apply_runtime(monkeypatch, setup)
    assert setup.main(["--apply", "--serial", "3352", "--axis", "both", "--node", "13"],
                      odrive_module=Boards(b)) == 0
    assert (b.axis0.config.can_node_id, b.axis1.config.can_node_id) == (13, 14)


def test_dual_axis_demo_rejects_unnamed_board_before_discovery(monkeypatch):
    monkeypatch.setattr(sys, "argv", ["dual", "--demo"])
    class NoHardware:
        def find_spec(self, fullname, *args):
            if fullname == "odrive":
                pytest.fail("hardware imported before CLI target validation")
    monkeypatch.delitem(sys.modules, "odrive", raising=False)
    finder = NoHardware()
    sys.meta_path.insert(0, finder)
    try:
        with pytest.raises(SystemExit) as error:
            runpy.run_path("motor_control/drive/bl70200/bl70200_dual_axis.py", run_name="__main__")
        assert error.value.code == 2
    finally:
        sys.meta_path.remove(finder)


def test_no_auto_arm_does_not_arm_hardware(monkeypatch, lock_path):
    from drive.bl70200 import dualsense_usb_teleop as m
    arms, idles = [], []
    monkeypatch.setattr(sys, "argv", ["usb", "--serial", "3352", "--axis", "both", "--node", "13", "--no-auto-arm"])
    monkeypatch.setattr(m, "connect", lambda _: [("3352", board())])
    monkeypatch.setattr(m, "arm", lambda *args: arms.append(args))
    monkeypatch.setattr(m, "idle_all", lambda axes: idles.append(list(axes)))
    monkeypatch.setattr(m, "serve", lambda axes, args: None)
    m.main()
    assert not arms
    assert len(idles) >= 1


def test_partial_usb_arm_failure_idles_every_selected_axis(monkeypatch, lock_path):
    from drive.bl70200 import dualsense_usb_teleop as m
    idles = []
    monkeypatch.setattr(sys, "argv", ["usb", "--serial", "3352", "--axis", "both", "--node", "13"])
    monkeypatch.setattr(m, "connect", lambda _: [("3352", board())])
    def fail(*_):
        raise SystemExit("injected arm failure")
    monkeypatch.setattr(m, "arm", fail)
    monkeypatch.setattr(m, "idle_all", lambda axes: idles.extend(axes))
    with pytest.raises(SystemExit, match="arm failure"):
        m.main()
    assert len(idles) == 2


LEGACY = (
    list(Path("motor_control/drive/x2212_test").glob("*.py"))
    + list(Path("motor_control/pi").glob("pi_server_*.py"))
    + list(Path("motor_control/drive/bl70200").glob("odrive_*_test.py"))
)


@pytest.mark.parametrize("path", LEGACY, ids=str)
def test_legacy_tools_fail_before_importing_hardware(monkeypatch, path):
    class ForbidHardware:
        def find_spec(self, fullname, *args):
            if fullname.split(".")[0] in {"odrive", "can", "pygame", "cv2"}:
                pytest.fail(f"legacy tool imported hardware before hard stop: {fullname}")
    for name in list(sys.modules):
        if name.split(".")[0] in {"odrive", "can", "pygame", "cv2"}:
            monkeypatch.delitem(sys.modules, name)
    finder = ForbidHardware()
    sys.meta_path.insert(0, finder)
    try:
        with pytest.raises(SystemExit, match="DEPRECATED.*bl70200_setup"):
            runpy.run_path(str(path), run_name="__main__")
    finally:
        sys.meta_path.remove(finder)
