"""Explicit registry keeps mechanism choice out of the hardware bridge."""

from dynamixel_control.tool_fsm.dual_motor_gripper_fsm import DualMotorGripperFSM
from dynamixel_control.tool_fsm.single_motor_gripper_fsm import SingleMotorGripperFSM

FSM_REGISTRY = {
    'spur_1motor_gripper': SingleMotorGripperFSM,
    'dual_motor_gripper': DualMotorGripperFSM,
}


def create_tool_fsm(tool_type, profile, bridge):
    try:
        return FSM_REGISTRY[tool_type](profile, bridge)
    except KeyError as exc:
        raise ValueError(f'no tool FSM registered for {tool_type!r}') from exc
