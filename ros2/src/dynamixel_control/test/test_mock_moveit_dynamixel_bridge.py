"""하드웨어 없는 MoveIt 브리지의 안전 및 인터페이스 테스트."""

from pathlib import Path

from dynamixel_control.mock_moveit_dynamixel_bridge import (
    ALL_JOINTS, ARM_JOINTS, DEFAULT_POSITIONS, GRIPPER_JOINT,
)


def test_mock_defaults_cover_arm_and_virtual_gripper_geometry():
    assert ARM_JOINTS == (
        'arm_joint_1', 'arm_joint_2', 'arm_joint_3',
        'arm_joint_4', 'arm_joint_5')
    assert GRIPPER_JOINT == 'gripper_left_pinion_joint'
    assert ALL_JOINTS[-1] == GRIPPER_JOINT
    assert len(DEFAULT_POSITIONS) == len(ALL_JOINTS)


def test_mock_module_has_no_serial_or_dynamixel_dependency():
    source = Path(__file__).parents[1] / 'dynamixel_control' \
        / 'mock_moveit_dynamixel_bridge.py'
    text = source.read_text(encoding='utf-8')
    forbidden = ('dynamixel_sdk', 'PortHandler', 'PacketHandler',
                 'GroupSyncWrite', 'GroupSyncRead', '/dev/ttyUSB')
    for token in forbidden:
        assert token not in text
