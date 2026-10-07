import os
import xacro
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('robot_arm_description'),
        'urdf', 'robot_arm.urdf.xacro'
    )

    robot_description = xacro.process_file(xacro_file).toxml()

    # 저장된 RViz 설정이 없으면 Fixed Frame/RobotModel/Durability를 매번 손으로 잡아야 해서
    # rviz/robot_arm.rviz를 기본으로 넘긴다.
    rviz_config = os.path.join(
        get_package_share_directory('robot_arm_description'),
        'rviz', 'robot_arm.rviz'
    )

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
        ),
    ])
