"""powertrain_control 서비스의 PID 1 — teleop_command + ops_broker (D5)."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("ops_port", default_value="9001"),
        DeclareLaunchArgument(
            "ops_token_dir",
            default_value="/etc/powertrain",
        ),
        Node(
            package="powertrain_ros",
            executable="teleop_command",
            name="teleop_command",
            output="screen",
        ),
        Node(
            package="powertrain_ros",
            executable="ops_broker",
            name="ops_broker",
            output="screen",
            parameters=[{
                "port": ParameterValue(
                    LaunchConfiguration("ops_port"),
                    value_type=int,
                ),
                "token_dir": LaunchConfiguration("ops_token_dir"),
            }],
        ),
    ])
