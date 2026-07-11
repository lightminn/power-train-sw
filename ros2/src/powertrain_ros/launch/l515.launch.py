from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    config = PathJoinSubstitution([
        FindPackageShare("powertrain_ros"), "config", "l515.yaml"
    ])
    return LaunchDescription([
        Node(
            package="powertrain_ros",
            executable="l515_camera",
            name="l515_camera",
            parameters=[str(config)],
            output="screen",
        )
    ])
