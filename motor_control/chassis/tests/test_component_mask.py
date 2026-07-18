"""컴포넌트 마스크 순수 코어 계약 검증."""
from unittest.mock import Mock

import chassis.chassis_manager as chassis_manager
from chassis.chassis_manager import ChassisManager
from chassis.kinematics import default_geometry
from chassis.safety_interlock import ESTOP, RUN
from corner_module.config import CornerConfig
from corner_module.corner_module import CornerModule
from corner_module.fake import FakeDrive, FakeSteer
from corner_module.null_steer import NullSteer


def _fake_corners():
    """기존 chassis 테스트와 같은 4조향+2고정, 6구동 구성."""
    corners = {}
    for wheel in default_geometry().wheels:
        steer = FakeSteer() if wheel.steerable else NullSteer()
        corners[wheel.name] = CornerModule(
            steer,
            FakeDrive(),
            CornerConfig(),
        )
    return corners


def _idle_manager():
    manager = ChassisManager(_fake_corners())
    manager.connect()
    return manager


def _armed_manager():
    manager = _idle_manager()
    assert manager.arm() is True
    return manager


def test_component_mask_defaults_on_and_is_exposed_as_detached_snapshots():
    manager = _idle_manager()
    expected = {name: True for name in chassis_manager.COMPONENTS}

    assert manager.component_mask == expected
    assert manager.snapshot().component_mask == expected
    assert manager.safety_snapshot().component_mask == expected

    detached = manager.component_mask
    detached["drive"] = False
    snapshot_mask = manager.snapshot().component_mask
    snapshot_mask["steer"] = False
    assert manager.component_mask == expected


def test_unknown_component_is_rejected_without_mutating_mask():
    manager = _idle_manager()
    before = manager.component_mask

    assert manager.set_component_enabled("camera", False) == (
        False,
        "unknown_component",
    )
    assert manager.component_mask == before


def test_motor_mask_changes_require_idle_but_external_components_do_not():
    manager = _armed_manager()

    assert manager.set_component_enabled("drive", False) == (False, "not_idle")
    assert manager.set_component_enabled("steer", False) == (False, "not_idle")
    assert manager.set_component_enabled("us100", False) == (True, "")
    assert manager.set_component_enabled("robot_arm", False) == (True, "")
    assert manager.component_mask == {
        "drive": True,
        "steer": True,
        "us100": False,
        "robot_arm": False,
    }


def test_drive_off_skips_commands_and_fault_monitoring_while_steer_runs():
    manager = _idle_manager()
    for corner in manager.corners.values():
        corner.drive.arm = Mock(wraps=corner.drive.arm)
        corner.drive.set_velocity = Mock(wraps=corner.drive.set_velocity)
        corner.drive.tick = Mock(wraps=corner.drive.tick)
        corner.steer.set_angle = Mock(wraps=corner.steer.set_angle)
        corner.steer.tick = Mock(wraps=corner.steer.tick)

    assert manager.set_component_enabled("drive", False) == (True, "")
    assert manager.arm() is True
    manager.set(0.4, 0.2)
    manager.corners["front_left"].drive.stale_flag = True
    manager.corners["front_left"].drive.axis_error = 0x10

    for _ in range(3):
        manager.tick()

    assert manager.mode == "ARMED"
    assert manager.safety_snapshot().estop_latched is False
    for corner in manager.corners.values():
        corner.drive.arm.assert_not_called()
        corner.drive.set_velocity.assert_not_called()
        corner.drive.tick.assert_not_called()
    front_left = manager.corners["front_left"]
    assert front_left.steer.set_angle.call_count > 0
    assert front_left.steer.tick.call_count > 0


def test_steer_off_skips_commands_and_fault_monitoring_while_drive_runs():
    manager = _idle_manager()
    corner = manager.corners["front_left"]
    corner.steer.arm = Mock(wraps=corner.steer.arm)
    corner.steer.set_angle = Mock(wraps=corner.steer.set_angle)
    corner.steer.tick = Mock(wraps=corner.steer.tick)
    corner.drive.set_velocity = Mock(wraps=corner.drive.set_velocity)
    corner.drive.tick = Mock(wraps=corner.drive.tick)

    assert manager.set_component_enabled("steer", False) == (True, "")
    assert manager.arm() is True
    manager.set(0.4, 0.2)
    corner.steer.fault = 5
    corner.steer.stale_flag = True

    for _ in range(3):
        manager.tick()

    assert manager.mode == "ARMED"
    assert manager.safety_snapshot().estop_latched is False
    corner.steer.arm.assert_not_called()
    corner.steer.set_angle.assert_not_called()
    corner.steer.tick.assert_not_called()
    assert corner.drive.set_velocity.call_count > 0
    assert corner.drive.tick.call_count > 0


def test_us100_off_clears_active_sources_but_preserves_estop_latch():
    manager = _armed_manager()
    manager._interlock.set_estop_condition("us100", True, "near")
    manager._interlock.set_estop_condition("us100_link", True, "stale")
    manager.tick()
    assert manager.mode == "ESTOP"

    assert manager.set_component_enabled("us100", False) == (True, "")

    safety = manager.safety_snapshot()
    assert safety.active_estop_sources == ()
    assert safety.estop_latched is True
    assert safety.state == ESTOP
    assert manager.mode == "ESTOP"
    assert manager.reset_estop() is True
    assert manager.mode == "IDLE"


def test_robot_arm_off_clears_prefixed_holds_but_still_wants_fresh_command():
    manager = _armed_manager()
    manager.set_motion_hold("robot_arm", True, "stale")
    manager.set_motion_hold("robot_arm_link", True, "link stale")
    assert manager.safety_snapshot().state != RUN

    assert manager.set_component_enabled("robot_arm", False) == (True, "")

    # 컴포넌트 hold는 사라지지만 command_recovery는 남는다: hold 중 저장된
    # 이전 명령의 재생을 막고, 오직 새 set()(fresh command)만이 해제한다.
    safety = manager.safety_snapshot()
    assert safety.hold_sources == ("command_recovery",)
    assert safety.state != RUN

    manager.set(0.0, 0.0)
    safety = manager.safety_snapshot()
    assert safety.hold_sources == ()
    assert safety.state == RUN


def test_setting_the_existing_value_is_an_idempotent_noop():
    manager = _idle_manager()
    for corner in manager.corners.values():
        corner.set_drive_enabled = Mock(wraps=corner.set_drive_enabled)

    assert manager.set_component_enabled("drive", True) == (True, "")
    for corner in manager.corners.values():
        corner.set_drive_enabled.assert_not_called()


def test_drive_is_reincluded_by_the_next_arm_after_reenable():
    manager = _idle_manager()
    for corner in manager.corners.values():
        corner.drive.arm = Mock(wraps=corner.drive.arm)

    assert manager.set_component_enabled("drive", False) == (True, "")
    assert manager.arm() is True
    for corner in manager.corners.values():
        corner.drive.arm.assert_not_called()

    manager.disarm()
    assert manager.set_component_enabled("drive", True) == (True, "")
    assert manager.arm() is True
    for corner in manager.corners.values():
        corner.drive.arm.assert_called_once_with()
