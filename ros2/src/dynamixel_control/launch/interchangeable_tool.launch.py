"""Manual interchangeable-tool selection for hardware and mock validation."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    tool_type = LaunchConfiguration('tool_type')
    profile_file = LaunchConfiguration('tool_profile_file')
    mock_mode = LaunchConfiguration('mock_mode')
    start_fsm = LaunchConfiguration('start_fsm')
    read_only = LaunchConfiguration('read_only')
    control_scope = LaunchConfiguration('control_scope')
    developer_direct_mode = LaunchConfiguration('developer_direct_mode')
    gripper_tolerance = LaunchConfiguration('gripper_target_tolerance_ticks')
    temporary_jog_mode = LaunchConfiguration('temporary_jog_mode')
    dual_single_motor_test = LaunchConfiguration('dual_single_motor_test_mode')
    dual_manual_test = LaunchConfiguration('dual_manual_test_mode')
    validation_mode = LaunchConfiguration('validation_mode')
    temporary_safe_min = LaunchConfiguration('temporary_jog_safe_min_tick')
    temporary_safe_max = LaunchConfiguration('temporary_jog_safe_max_tick')
    mechanical_open = LaunchConfiguration('temporary_jog_mechanical_open_tick')
    mechanical_close = LaunchConfiguration('temporary_jog_mechanical_close_tick')
    temporary_velocity = LaunchConfiguration('temporary_jog_profile_velocity')
    temporary_acceleration = LaunchConfiguration(
        'temporary_jog_profile_acceleration')
    calibration_jog_mode = LaunchConfiguration('calibration_jog_mode')
    cleaner_joint = LaunchConfiguration('cleaning_actuator_joint')
    cleaner_id = LaunchConfiguration('cleaning_actuator_id')
    cleaner_direction = LaunchConfiguration('cleaning_direction')
    cleaner_velocity = LaunchConfiguration('cleaning_velocity_raw')
    common = {
        'tool_type': tool_type,
        'tool_profile_file': profile_file,
    }
    return LaunchDescription([
        DeclareLaunchArgument('tool_type', default_value='spur_1motor_gripper'),
        DeclareLaunchArgument(
            'tool_profile_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('dynamixel_control'), 'config',
                'tool_profiles.yaml'])),
        DeclareLaunchArgument('mock_mode', default_value='false'),
        DeclareLaunchArgument('start_fsm', default_value='true'),
        DeclareLaunchArgument('read_only', default_value='false'),
        DeclareLaunchArgument('control_scope', default_value='END_EFFECTOR_ONLY'),
        DeclareLaunchArgument(
            'developer_direct_mode', default_value='false',
            description='ID5-only direct bench controls; retains physical safety gates.'),
        DeclareLaunchArgument(
            'gripper_target_tolerance_ticks', default_value='20'),
        DeclareLaunchArgument('temporary_jog_mode', default_value='false'),
        DeclareLaunchArgument('dual_single_motor_test_mode', default_value='false'),
        DeclareLaunchArgument('dual_manual_test_mode', default_value='false'),
        DeclareLaunchArgument(
            'validation_mode', default_value='false',
            description='Explicit endpoint/mode/HW validation; effort is simulated.'),
        DeclareLaunchArgument('temporary_jog_safe_min_tick', default_value='2867'),
        DeclareLaunchArgument('temporary_jog_safe_max_tick', default_value='3807'),
        DeclareLaunchArgument(
            'temporary_jog_mechanical_open_tick', default_value='2817'),
        DeclareLaunchArgument(
            'temporary_jog_mechanical_close_tick', default_value='3857'),
        DeclareLaunchArgument('temporary_jog_profile_velocity', default_value='5'),
        DeclareLaunchArgument(
            'temporary_jog_profile_acceleration', default_value='1'),
        DeclareLaunchArgument('calibration_jog_mode', default_value='false'),
        DeclareLaunchArgument('cleaning_actuator_joint', default_value=''),
        DeclareLaunchArgument('cleaning_actuator_id', default_value='-1'),
        DeclareLaunchArgument('cleaning_direction', default_value='0'),
        DeclareLaunchArgument('cleaning_velocity_raw', default_value='0'),
        Node(
            package='dynamixel_control',
            executable='moveit_dynamixel_bridge', output='screen',
            parameters=[common, {
                'mock_mode': ParameterValue(mock_mode, value_type=bool),
                'read_only': ParameterValue(read_only, value_type=bool),
                'control_scope': control_scope,
                'developer_direct_mode': ParameterValue(
                    developer_direct_mode, value_type=bool),
                'gripper_target_tolerance_ticks': ParameterValue(
                    gripper_tolerance, value_type=int),
                'temporary_jog_mode': ParameterValue(
                    temporary_jog_mode, value_type=bool),
                'dual_single_motor_test_mode': ParameterValue(
                    dual_single_motor_test, value_type=bool),
                'dual_manual_test_mode': ParameterValue(
                    dual_manual_test, value_type=bool),
                'validation_mode': ParameterValue(
                    validation_mode, value_type=bool),
                'temporary_jog_safe_min_tick': ParameterValue(
                    temporary_safe_min, value_type=int),
                'temporary_jog_safe_max_tick': ParameterValue(
                    temporary_safe_max, value_type=int),
                'temporary_jog_mechanical_open_tick': ParameterValue(
                    mechanical_open, value_type=int),
                'temporary_jog_mechanical_close_tick': ParameterValue(
                    mechanical_close, value_type=int),
                'temporary_jog_profile_velocity': ParameterValue(
                    temporary_velocity, value_type=int),
                'temporary_jog_profile_acceleration': ParameterValue(
                    temporary_acceleration, value_type=int),
                'calibration_jog_mode': ParameterValue(
                    calibration_jog_mode, value_type=bool),
                'cleaning_actuator_joint': cleaner_joint,
                'cleaning_actuator_id': ParameterValue(cleaner_id, value_type=int),
                'cleaning_direction': ParameterValue(
                    cleaner_direction, value_type=int),
                'cleaning_velocity_raw': ParameterValue(
                    cleaner_velocity, value_type=int),
            }],
        ),
        Node(
            package='dynamixel_control', executable='arm_fsm', output='screen',
            parameters=[common, {
                'dry_run_mode': ParameterValue(mock_mode, value_type=bool),
                'validation_mode': ParameterValue(
                    validation_mode, value_type=bool),
                'sensor_mock_mode': ParameterValue(mock_mode, value_type=bool),
                'vla_standalone_mode': ParameterValue(mock_mode, value_type=bool),
                'mock_contact': True, 'mock_distance': 1.0,
                'mock_lock_confirmed': True,
                'cleaning_actuator_joint': cleaner_joint,
                'cleaning_start_time': 0.05,
                'clean_duration': 0.1,
                'locked_dwell': 0.0,
            }],
            condition=IfCondition(start_fsm),
        ),
    ])
