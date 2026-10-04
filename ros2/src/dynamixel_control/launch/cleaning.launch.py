"""털털이 actuator bridge와 청소 FSM 실행.

ZIP에 빠진 하드웨어 값은 안전 비활성 기본값이다. 실제 로봇에서는 네 개의
cleaning_* 인자를 모두 명시해야 회전 명령이 허용된다.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    joint = LaunchConfiguration('cleaning_actuator_joint')
    dxl_id = LaunchConfiguration('cleaning_actuator_id')
    direction = LaunchConfiguration('cleaning_direction')
    velocity = LaunchConfiguration('cleaning_velocity_raw')
    contact = LaunchConfiguration('joint_effort_contact_threshold')
    duration = LaunchConfiguration('clean_duration')
    mock_mode = LaunchConfiguration('sensor_mock_mode')
    mock_contact = LaunchConfiguration('mock_contact')
    mock_distance = LaunchConfiguration('mock_distance')
    mock_lock = LaunchConfiguration('mock_lock_confirmed')

    shared = {
        'cleaning_actuator_joint': joint,
    }
    return LaunchDescription([
        DeclareLaunchArgument('cleaning_actuator_joint', default_value=''),
        DeclareLaunchArgument('cleaning_actuator_id', default_value='-1'),
        DeclareLaunchArgument('cleaning_direction', default_value='0'),
        DeclareLaunchArgument('cleaning_velocity_raw', default_value='0'),
        DeclareLaunchArgument('joint_effort_contact_threshold', default_value='0.0'),
        DeclareLaunchArgument('clean_duration', default_value='5.0'),
        DeclareLaunchArgument('sensor_mock_mode', default_value='false'),
        DeclareLaunchArgument('mock_contact', default_value='false'),
        DeclareLaunchArgument('mock_distance', default_value='0.0'),
        DeclareLaunchArgument('mock_lock_confirmed', default_value='false'),
        Node(
            package='dynamixel_control',
            executable='moveit_dynamixel_bridge',
            output='screen',
            parameters=[shared, {
                'tool_type': 'cleaner',
                'cleaning_actuator_id': ParameterValue(dxl_id, value_type=int),
                'cleaning_direction': ParameterValue(direction, value_type=int),
                'cleaning_velocity_raw': ParameterValue(velocity, value_type=int),
            }],
        ),
        Node(
            package='dynamixel_control',
            executable='arm_fsm',
            output='screen',
            parameters=[shared, {
                'tool_type': 'cleaner',
                'joint_effort_sensor_joint': joint,
                'joint_effort_contact_threshold': ParameterValue(
                    contact, value_type=float),
                'clean_duration': ParameterValue(duration, value_type=float),
                'sensor_mock_mode': ParameterValue(mock_mode, value_type=bool),
                'mock_contact': ParameterValue(mock_contact, value_type=bool),
                'mock_distance': ParameterValue(mock_distance, value_type=float),
                'mock_lock_confirmed': ParameterValue(mock_lock, value_type=bool),
            }],
        ),
    ])
