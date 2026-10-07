"""Hardware-free VLA -> FSM -> mock sensors dry run."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    tool_type = LaunchConfiguration('tool_type')
    return LaunchDescription([
        DeclareLaunchArgument('tool_type', default_value='cleaner'),
        Node(package='robot_vla', executable='vla_node', output='screen'),
        Node(
            package='dynamixel_control', executable='arm_fsm', output='screen',
            parameters=[{
                'vla_standalone_mode': True,
                'tool_type': tool_type,
                'dry_run_mode': True,
                'sensor_mock_mode': True,
                'mock_contact': True,
                'mock_distance': 1.0,
                'mock_lock_confirmed': True,
                'cleaning_start_time': 0.05,
                'clean_duration': 0.1,
                'locked_dwell': 0.0,
            }],
        ),
    ])
