import sys
from types import SimpleNamespace

import pytest

from chassis import runtime_lock
from corner_module.drive_odrive_usb_axis import UsbBoardPool, DriveOdriveUsbAxis


@pytest.fixture
def usb(monkeypatch, tmp_path):
    from drive.bl70200.tests.test_usb_remediation import board
    b = board()
    for ax in (b.axis0, b.axis1):
        ax.current_state = 1
        ax.error = 0
        ax.encoder.vel_estimate = 0.0
        ax.motor.current_control = SimpleNamespace(Iq_measured=0.0)
    cls = runtime_lock.RealCanSession
    path = str(tmp_path / "can0.lock")
    monkeypatch.setattr(runtime_lock, "RealCanSession", lambda **kw: cls(path=path, **kw))
    calls = []
    def find(**kw):
        calls.append(kw)
        return b
    monkeypatch.setitem(sys.modules, "odrive", SimpleNamespace(find_any=find))
    return b, calls


def test_usb_pool_cannot_open_board_while_can_runtime_owns_motors(usb):
    _, calls = usb
    with runtime_lock.RealCanSession(owner="CAN"):
        with pytest.raises(runtime_lock.CanOwnershipError):
            UsbBoardPool().board("3352")
    assert calls == []


def test_usb_pool_keeps_lock_until_last_axis_has_stopped(usb):
    b, calls = usb
    pool = UsbBoardPool()
    a = DriveOdriveUsbAxis(pool, "3352", 0, node_id=13)
    c = DriveOdriveUsbAxis(pool, "3352", 1, node_id=14)
    a.connect()
    c.connect()
    assert len(calls) == 1
    a.close()
    with pytest.raises(runtime_lock.CanOwnershipError):
        with runtime_lock.RealCanSession(owner="CAN"):
            pass
    c.close()
    with runtime_lock.RealCanSession(owner="CAN"):
        assert b.axis0.requested_state == 1
        assert b.axis1.requested_state == 1


def test_usb_pool_rejects_wrong_serial_before_using_axis_and_releases_lock(usb):
    b, _ = usb
    b.serial_number = 0x9999
    with pytest.raises(ValueError, match="serial"):
        UsbBoardPool().board("3352")
    with runtime_lock.RealCanSession(owner="CAN"):
        pass


def test_usb_runtime_reports_age_of_complete_axis_read(usb):
    b, _ = usb
    clock = [1.0]
    driver = DriveOdriveUsbAxis(UsbBoardPool(), "3352", 0, node_id=13, clock=lambda: clock[0])
    try:
        driver.connect()
        driver._poll_now()
        clock[0] += .1
        assert driver.state()["last_feedback_age_ms"] == pytest.approx(100)
        assert driver.health_state()["last_feedback_age_ms"] == pytest.approx(100)
    finally:
        driver.close()


def test_idle_usb_feedback_refresh_never_writes_motor_commands(usb):
    b, _ = usb
    driver = DriveOdriveUsbAxis(UsbBoardPool(), "3352", 0, node_id=13, poll_period_ticks=1)
    driver.connect()
    b.axis0.controller.input_vel = 99.0
    b.axis0.requested_state = 77
    try:
        driver.poll_feedback()
        assert driver.state()["last_feedback_age_ms"] is not None
        assert b.axis0.controller.input_vel == 99.0
        assert b.axis0.requested_state == 77
    finally:
        driver.close()


def test_usb_arm_requires_observed_closed_loop(usb):
    b, _ = usb
    driver = DriveOdriveUsbAxis(UsbBoardPool(), "3352", 0, node_id=13, poll_period_ticks=1)
    driver.connect()
    try:
        driver.arm()
        assert hasattr(driver, "arm_confirmed"), "USB arm needs real state confirmation"
        assert driver.arm_confirmed() is False
        b.axis0.current_state = 8
        driver.poll_feedback()
        assert driver.arm_confirmed() is True
        b.axis0.error = 1
        driver.poll_feedback()
        assert driver.arm_confirmed() is False
    finally:
        driver.close()


def test_pending_usb_arm_refreshes_state_without_a_chassis_tick(usb):
    b, _ = usb
    now = [10.0]
    driver = DriveOdriveUsbAxis(UsbBoardPool(), "3352", 0, node_id=13, clock=lambda: now[0])
    driver.connect()
    try:
        driver.arm()
        assert not driver.arm_confirmed()
        b.axis0.current_state = 8
        now[0] += .021
        assert driver.arm_confirmed()
    finally:
        driver.close()


def test_usb_write_failure_propagates_while_receive_still_refreshes(usb):
    b, _ = usb
    driver = DriveOdriveUsbAxis(UsbBoardPool(), "3352", 0, node_id=13, poll_period_ticks=1)
    driver.connect()
    class BrokenInput:
        @property
        def input_vel(self):
            return 0
        @input_vel.setter
        def input_vel(self, value):
            raise OSError("injected USB write failure")
    original = b.axis0.controller
    b.axis0.controller = BrokenInput()
    try:
        with pytest.raises(RuntimeError, match="USB.*write"):
            driver.tick()
        assert driver.state()["last_feedback_age_ms"] is not None
    finally:
        b.axis0.controller = original
        driver.close()


def test_slow_usb_property_read_cannot_certify_old_encoder_as_fresh():
    from chassis.chassis_manager import ChassisManager, build_corners
    from corner_module.null_steer import NullSteer
    from corner_module.tests.test_drive_odrive_usb_axis import FakeAxis, FakeBoard

    now = [10.0]

    class SlowStateAxis(FakeAxis):
        def __getattribute__(self, name):
            if name == "current_state":
                now[0] += .25
            return super().__getattribute__(name)

    axes = {node: SlowStateAxis() if node == 11 else FakeAxis()
            for node in range(11, 17)}
    pool = UsbBoardPool(finder=lambda serial: FakeBoard(axis0=axes[int(serial)]))
    corners = build_corners(lambda cid: NullSteer(), lambda node: DriveOdriveUsbAxis(
        pool, str(node), 0, node_id=node, clock=lambda: now[0]))
    manager = ChassisManager(corners, clock=lambda: now[0])
    manager.connect()
    try:
        for corner in corners.values():
            corner.drive._poll_now()
        first = next(c.drive for c in corners.values() if c.drive._node_id == 11)
        proof = manager.hardware_stop_proof("usb")
        assert proof["valid"] is False, proof
        assert proof["stopped"] is False
        assert first.health_state()["last_feedback_age_ms"] == pytest.approx(250)
    finally:
        manager.close()
