from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    raise RuntimeError(
        "legacy l515.launch.py is retired; start the singleton Gateway with "
        "`python3 -m l515_dashboard.gateway_main`"
    )
    config = PathJoinSubstitution([
        FindPackageShare("powertrain_ros"), "config", "l515.yaml"
    ])
    return LaunchDescription([
        Node(
            package="powertrain_ros",
            executable="l515_camera",
            name="l515_camera_node",
            parameters=[config],
            output="screen",
        )
    ])
