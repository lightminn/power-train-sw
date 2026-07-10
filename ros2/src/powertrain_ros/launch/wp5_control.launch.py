from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="powertrain_ros",
            executable="us100_safety",
            output="screen",
        ),
        Node(
            package="powertrain_ros",
            executable="chassis",
            output="screen",
        ),
    ])
