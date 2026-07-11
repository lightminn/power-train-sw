from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "stop_mm",
            description="HIL-approved US-100 emergency-stop distance (mm)",
        ),
        Node(
            package="powertrain_ros",
            executable="us100_safety",
            output="screen",
            parameters=[{
                "stop_mm": ParameterValue(
                    LaunchConfiguration("stop_mm"),
                    value_type=float,
                ),
            }],
        ),
        Node(
            package="powertrain_ros",
            executable="chassis",
            output="screen",
        ),
    ])
