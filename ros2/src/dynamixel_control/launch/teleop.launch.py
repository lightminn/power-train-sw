"""원격조종 스택 런치.

기동 대상:
  - teleop_core (항상)  : /arm/teleop_jog → /dynamixel/goal_position
  - position_node       : use_hardware:=true 일 때만 (실서보, /dev/ttyUSB0)
  - robot_state_publisher + rviz2 : rviz:=true 일 때만 (RViz 디버그 뷰)

keyboard_teleop 은 stdin 포커스가 필요해 여기 넣지 않는다.
별도 터미널에서 실행:  ros2 run dynamixel_control keyboard_teleop

/joint_states 소스 (충돌 방지 위해 하나만):
  - use_hardware:=false → teleop_core 가 목표값을 sim 으로 발행(publish_sim_joint_states)
  - use_hardware:=true  → position_node 가 실서보 엔코더로 발행 (teleop_core sim 은 off)
  * joint_state_publisher 는 teleop 값을 덮으므로 일부러 넣지 않는다.

사용 예)
  # 하드웨어 없이 RViz 로 확인 (디버그 모드)
  ros2 launch dynamixel_control teleop.launch.py rviz:=true
  # 실서보까지
  ros2 launch dynamixel_control teleop.launch.py use_hardware:=true rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_hardware = LaunchConfiguration("use_hardware")
    rviz = LaunchConfiguration("rviz")

    urdf_path = os.path.join(
        get_package_share_directory("robot_arm_description"),
        "urdf", "robot_arm.urdf",
    )
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    rviz_config = os.path.join(
        get_package_share_directory("dynamixel_control"),
        "rviz", "teleop.rviz",
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_hardware", default_value="false",
            description="true 면 position_node(실서보, /dev/ttyUSB0)도 함께 기동",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="false",
            description="true 면 robot_state_publisher + rviz2(디버그 뷰) 기동",
        ),

        # teleop_core — 하드웨어 없을 때: sim /joint_states 발행 ON
        Node(
            package="dynamixel_control", executable="teleop_core", name="teleop_core",
            output="screen",
            parameters=[{"publish_sim_joint_states": True}],
            condition=UnlessCondition(use_hardware),
        ),
        # teleop_core — 실서보일 때: sim OFF (position_node 가 /joint_states 담당)
        Node(
            package="dynamixel_control", executable="teleop_core", name="teleop_core",
            output="screen",
            parameters=[{"publish_sim_joint_states": False}],
            condition=IfCondition(use_hardware),
        ),

        # 실서보 드라이버
        Node(
            package="dynamixel_control", executable="position_node",
            name="dynamixel_position_node", output="screen",
            condition=IfCondition(use_hardware),
        ),

        # RViz 디버그 뷰 (robot_state_publisher + rviz2). joint_state_publisher 는 넣지 않음.
        Node(
            package="robot_state_publisher", executable="robot_state_publisher",
            name="robot_state_publisher", output="screen",
            parameters=[{"robot_description": robot_description}],
            condition=IfCondition(rviz),
        ),
        Node(
            package="rviz2", executable="rviz2", name="rviz2",
            arguments=["-d", rviz_config], output="screen",
            condition=IfCondition(rviz),
        ),
    ])
