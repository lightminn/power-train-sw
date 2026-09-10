"""Power-train-only end-effector runtime.

Starts the vendored Dynamixel tool bridge and the existing console mirror;
no checkout or source path from the robot-arm repository is required.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('tool_type', default_value='dual_motor_gripper'),
        DeclareLaunchArgument('dual_single_axis_id', default_value='3'),
        DeclareLaunchArgument('console_host', default_value='127.0.0.1'),
        Node(
            package='dynamixel_control', executable='moveit_dynamixel_bridge',
            name='moveit_dynamixel_bridge', output='screen',
            parameters=[{
                'tool_type': LaunchConfiguration('tool_type'),
                'dual_single_axis_id': LaunchConfiguration('dual_single_axis_id'),
                'control_scope': 'END_EFFECTOR_ONLY',
                'auto_tool_detection': True,
                'read_only': False,
            }],
        ),
        Node(
            package='powertrain_ros', executable='arm_console_bridge',
            name='arm_console_bridge', output='screen',
            parameters=[{'console_host': LaunchConfiguration('console_host')}],
        ),
    ])
