"""명시적으로 선택한 엔드이펙터 하나와 단일 버스 소유자를 실행한다."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from dynamixel_control.gripper_presets import GRIPPER_PRESETS


def _configured_nodes(context):
    preset_name = LaunchConfiguration('end_effector_preset').perform(context)
    mission = LaunchConfiguration('mission_type').perform(context)
    preset = GRIPPER_PRESETS.get(preset_name)
    if preset is None:
        raise RuntimeError(
            f"unknown end_effector_preset {preset_name!r}; "
            f"choose one of {sorted(GRIPPER_PRESETS)}")
    if mission != preset['allowed_mission']:
        raise RuntimeError(
            f"mission_type {mission!r} is incompatible with "
            f"end_effector_preset {preset_name!r}; "
            f"expected {preset['allowed_mission']!r}")

    common = {'end_effector_preset': preset_name}
    return [
        # 이 launch 파일에서 /dev/ttyUSB0을 여는 유일한 노드다.
        Node(
            package='dynamixel_control',
            executable='moveit_dynamixel_bridge',
            name='moveit_dynamixel_bridge',
            output='screen',
            parameters=[common, {
                'gripper_change_mode': ParameterValue(
                    LaunchConfiguration('gripper_change_mode'), value_type=bool),
                'gripper_disabled': ParameterValue(
                    LaunchConfiguration('gripper_disabled'), value_type=bool),
                'read_only': ParameterValue(
                    LaunchConfiguration('read_only'), value_type=bool),
                'gripper_only_mode': ParameterValue(
                    LaunchConfiguration('end_effector_only'), value_type=bool),
                'integrated_test_mode': ParameterValue(
                    LaunchConfiguration('integrated_test_mode'),
                    value_type=bool),
                'random_demo_enabled': ParameterValue(
                    LaunchConfiguration('random_demo_enabled'),
                    value_type=bool),
                'arm_test_goal_tolerance_ticks': ParameterValue(
                    LaunchConfiguration('arm_test_goal_tolerance_ticks'),
                    value_type=int),
            }],
        ),
        Node(
            package='dynamixel_control',
            executable='arm_fsm',
            name='arm_fsm_node',
            output='screen',
            parameters=[common, {
                'mission_type': mission,
                'ik_mode': LaunchConfiguration('ik_mode'),
                'gripper_change_mode': ParameterValue(
                    LaunchConfiguration('gripper_change_mode'), value_type=bool),
                'gripper_disabled': ParameterValue(
                    LaunchConfiguration('gripper_disabled'), value_type=bool),
                'stop_after_descend': ParameterValue(
                    LaunchConfiguration('stop_after_descend'), value_type=bool),
                'rotary_relative': ParameterValue(
                    LaunchConfiguration('rotary_relative'), value_type=bool),
                'rotary_ticks': ParameterValue(
                    LaunchConfiguration('rotary_ticks'), value_type=int),
                'integrated_test_mode': ParameterValue(
                    LaunchConfiguration('integrated_test_mode'),
                    value_type=bool),
                'random_demo_enabled': ParameterValue(
                    LaunchConfiguration('random_demo_enabled'),
                    value_type=bool),
                'random_seed': ParameterValue(
                    LaunchConfiguration('random_seed'), value_type=int),
                'random_pose_count': ParameterValue(
                    LaunchConfiguration('random_pose_count'), value_type=int),
            }],
            condition=IfCondition(LaunchConfiguration('start_fsm')),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'end_effector_preset', default_value='dual_motor_gripper'),
        DeclareLaunchArgument(
            'mission_type', default_value='PICK_PLACE'),
        DeclareLaunchArgument(
            'ik_mode', default_value='analytic',
            description='Arm IK backend: analytic or moveit'),
        DeclareLaunchArgument(
            'gripper_change_mode', default_value='false',
            description='Arm-only PICK_PLACE mode latched after DESCEND'),
        DeclareLaunchArgument(
            'gripper_disabled', default_value='false',
            description='Exclude all physical gripper IDs from bus access'),
        DeclareLaunchArgument(
            'stop_after_descend', default_value='false',
            description='Latch before GRASP after successful DESCEND'),
        DeclareLaunchArgument(
            'read_only', default_value='true',
            description='Safe default: bridge performs no register writes'),
        DeclareLaunchArgument(
            'end_effector_only', default_value='false',
            description='Do not activate or command arm motors'),
        DeclareLaunchArgument(
            'start_fsm', default_value='false',
            description='Start the mission FSM after preset validation'),
        DeclareLaunchArgument(
            'integrated_test_mode', default_value='false',
            description='Enable the guarded four-axis tick test before rotate'),
        DeclareLaunchArgument(
            'random_demo_enabled', default_value='false',
            description='Run bounded deterministic arm poses with rotations'),
        DeclareLaunchArgument(
            'arm_test_goal_tolerance_ticks', default_value='10',
            description='Arm tick goal tolerance for guarded test actions'),
        DeclareLaunchArgument('random_seed', default_value='42'),
        DeclareLaunchArgument('random_pose_count', default_value='3'),
        DeclareLaunchArgument('rotary_relative', default_value='true'),
        DeclareLaunchArgument('rotary_ticks', default_value='0'),
        OpaqueFunction(function=_configured_nodes),
    ])
