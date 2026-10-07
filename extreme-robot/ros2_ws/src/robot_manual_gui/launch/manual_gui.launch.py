"""Launch the manual GUI, optionally with exactly one bridge/FSM stack."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    mock_mode = LaunchConfiguration('mock_mode')
    start_moveit = LaunchConfiguration('start_moveit')
    profile_file = LaunchConfiguration('tool_profile_file')
    read_only = LaunchConfiguration('read_only')
    start_stack = LaunchConfiguration('start_stack')
    start_fsm = LaunchConfiguration('start_fsm')
    tool_type = LaunchConfiguration('tool_type')
    control_scope = LaunchConfiguration('control_scope')
    auto_enable = LaunchConfiguration('auto_enable_on_attach')
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
    stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare('dynamixel_control'), 'launch',
            'interchangeable_tool.launch.py'])),
        launch_arguments={
            'mock_mode': mock_mode,
            'tool_profile_file': profile_file,
            'start_fsm': start_fsm,
            'read_only': read_only,
            'tool_type': tool_type,
            'control_scope': control_scope,
            'auto_enable_on_attach': auto_enable,
            'developer_direct_mode': developer_direct_mode,
            'gripper_target_tolerance_ticks': gripper_tolerance,
            'temporary_jog_mode': temporary_jog_mode,
            'dual_single_motor_test_mode': dual_single_motor_test,
            'dual_manual_test_mode': dual_manual_test,
            'validation_mode': validation_mode,
            'temporary_jog_safe_min_tick': temporary_safe_min,
            'temporary_jog_safe_max_tick': temporary_safe_max,
            'temporary_jog_mechanical_open_tick': mechanical_open,
            'temporary_jog_mechanical_close_tick': mechanical_close,
            'temporary_jog_profile_velocity': temporary_velocity,
            'temporary_jog_profile_acceleration': temporary_acceleration,
            'calibration_jog_mode': calibration_jog_mode,
        }.items(),
        condition=IfCondition(start_stack),
    )
    return LaunchDescription([
        DeclareLaunchArgument('mock_mode', default_value='true'),
        DeclareLaunchArgument(
            'tool_profile_file', default_value=PathJoinSubstitution([
                FindPackageShare('dynamixel_control'), 'config', PythonExpression([
                    "'tool_profiles.mock.yaml' if '", mock_mode,
                    "'.lower() == 'true' else 'tool_profiles.yaml'"])])),
        DeclareLaunchArgument(
            'start_moveit', default_value='true',
            description='Start the persistent arm MoveIt/TF stack; false when already running.'),
        *[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('robot_arm_moveit_config'), 'launch', filename])),
            condition=IfCondition(start_moveit))
          for filename in ('rsp.launch.py', 'move_group.launch.py')],
        DeclareLaunchArgument(
            'read_only', default_value='true',
            description='Hardware launch defaults to no actuator writes.'),
        DeclareLaunchArgument(
            'start_stack', default_value='true',
            description='Set false when bridge/FSM are already running.'),
        DeclareLaunchArgument(
            'start_fsm', default_value='true',
            description='Keep the arm mission FSM connected to the selected tool context.'),
        DeclareLaunchArgument(
            'tool_type', default_value='dual_motor_gripper'),
        DeclareLaunchArgument(
            'control_scope', default_value='FULL_ROBOT',
            description='Arm joints 1–5 and the interchangeable tool share one bridge.'),
        DeclareLaunchArgument(
            'auto_enable_on_attach', default_value='false',
            description='Enable a validated runtime attachment in END_EFFECTOR_ONLY scope.'),
        DeclareLaunchArgument(
            'developer_direct_mode', default_value='false',
            description='ID5-only direct bench controls; retains physical safety gates.'),
        DeclareLaunchArgument(
            'gripper_target_tolerance_ticks', default_value='20'),
        DeclareLaunchArgument('temporary_jog_mode', default_value='false'),
        DeclareLaunchArgument(
            'dual_single_motor_test_mode', default_value='false',
            description='Dual profile test: torque/control ID3 only; ID4 stays free.'),
        DeclareLaunchArgument(
            'dual_manual_test_mode', default_value='false',
            description='Dual profile Q/W manual synchronized jog mode.'),
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
        stack,
        Node(
            package='robot_manual_gui', executable='manual_gui', output='screen',
            parameters=[{
                'mock_mode': ParameterValue(mock_mode, value_type=bool),
                'read_only': ParameterValue(read_only, value_type=bool),
                'tool_type': tool_type,
                'control_scope': control_scope,
                'developer_direct_mode': ParameterValue(
                    developer_direct_mode, value_type=bool),
                'temporary_jog_mode': ParameterValue(
                    temporary_jog_mode, value_type=bool),
                'dual_single_motor_test_mode': ParameterValue(
                    dual_single_motor_test, value_type=bool),
                'dual_manual_test_mode': ParameterValue(
                    dual_manual_test, value_type=bool),
                'temporary_jog_safe_min_tick': ParameterValue(
                    temporary_safe_min, value_type=int),
                'temporary_jog_safe_max_tick': ParameterValue(
                    temporary_safe_max, value_type=int),
                'temporary_jog_mechanical_open_tick': ParameterValue(
                    mechanical_open, value_type=int),
                'temporary_jog_mechanical_close_tick': ParameterValue(
                    mechanical_close, value_type=int),
                'calibration_jog_mode': ParameterValue(
                    calibration_jog_mode, value_type=bool),
            }],
        ),
    ])
