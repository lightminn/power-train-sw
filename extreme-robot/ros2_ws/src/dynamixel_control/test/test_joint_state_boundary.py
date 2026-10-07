"""MoveIt 고정 모델과 런타임 도구 피드백의 토픽 경계 테스트."""

from pathlib import Path
from types import SimpleNamespace

from builtin_interfaces.msg import Time
from dynamixel_control.moveit_dynamixel_bridge import (
    JOINT_CONFIG, MoveItDynamixelBridge)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def _mock_bridge(tool_type, joint_names, actuator_ids):
    bridge = object.__new__(MoveItDynamixelBridge)
    bridge.mock_mode = True
    bridge.control_scope = 'FULL_ROBOT'
    bridge._mock_arm_positions = {
        name: index * 0.1 for index, name in enumerate(JOINT_CONFIG, start=1)}
    bridge.tool_type = tool_type
    bridge.tool_profile = {
        'joint_names': list(joint_names),
        'actuator_ids': list(actuator_ids),
    }
    bridge._tool_samples = {
        dxl_id: {'position': 2048, 'effort': 17}
        for dxl_id in actuator_ids}
    bridge.joint_state_pub = Publisher()
    bridge.tool_joint_state_pub = Publisher()
    bridge.fault_pub = Publisher()
    bridge.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: Time()))
    return bridge


def test_spur_joint_never_enters_moveit_joint_states():
    bridge = _mock_bridge(
        'spur_1motor_gripper', ['gripper_drive_joint'], [5])

    bridge.publish_joint_states()

    arm = bridge.joint_state_pub.messages[-1]
    tool = bridge.tool_joint_state_pub.messages[-1]
    assert arm.name == list(JOINT_CONFIG)
    assert 'gripper_drive_joint' not in arm.name
    assert tool.name == ['gripper_drive_joint']


def test_dual_tool_feedback_is_preserved_outside_moveit_stream():
    bridge = _mock_bridge(
        'dual_motor_gripper', ['gripper_left_pinion_joint'], [3, 4])

    bridge.publish_joint_states()

    arm = bridge.joint_state_pub.messages[-1]
    tool = bridge.tool_joint_state_pub.messages[-1]
    assert set(arm.name) == set(JOINT_CONFIG)
    assert tool.name == ['gripper_left_pinion_joint']
    assert list(tool.effort) == [17.0]


def test_arm_fsm_subscribes_to_both_feedback_streams():
    source = (Path(__file__).parents[1] / 'dynamixel_control/arm_fsm_node.py'
              ).read_text(encoding='utf-8')
    assert "JointState, '/joint_states', self._on_joint_states" in source
    assert "JointState, '/tool/joint_states', self._on_joint_states" in source


def test_feedback_transport_exception_is_contained_and_fails_closed():
    source = (Path(__file__).parents[1] / 'dynamixel_control' /
              'moveit_dynamixel_bridge.py').read_text(encoding='utf-8')
    feedback = source[source.index('    def publish_joint_states'):]
    assert 'except Exception as exc:' in feedback
    assert 'joint feedback transport failed' in feedback
    assert 'self._mark_tool_feedback_offline()' in feedback
    assert 'self.fault_pub.publish(Bool(data=True))' in feedback


def test_id5_control_table_short_response_is_recoverable():
    class Packet:
        short_response = True

        def read1ByteTxRx(self, *_args):
            return 0, 0, 0

        def read4ByteTxRx(self, *_args):
            if self.short_response:
                raise IndexError('short response')
            return 20, 0, 0

    bridge = object.__new__(MoveItDynamixelBridge)
    bridge.tool_ids = [5]
    bridge.tool_type = 'spur_1motor_gripper'
    bridge.control_scope = 'END_EFFECTOR_ONLY'
    bridge.packet_handler = Packet()
    bridge.port_handler = object()
    bridge._bus_lock = __import__('threading').RLock()
    bridge.get_logger = lambda: SimpleNamespace(warn=lambda _message: None)

    sample = bridge._read_tool_control_state(5)
    assert sample['torque_state'] == 'UNKNOWN'
    assert sample['operating_mode'] is None
    assert sample['profile_velocity'] is None
    bridge.packet_handler.short_response = False
    recovered = bridge._read_tool_control_state(5)
    assert recovered['torque_state'] == 'OFF'
    assert recovered['operating_mode'] == 0
    assert recovered['profile_velocity'] == 20
