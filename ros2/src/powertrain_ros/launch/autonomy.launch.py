"""자율주행 전체 스택 — 한 줄로 띄운다.

    # 인지·시각화만 (모터 안 돌림. 기본값)
    ros2 launch powertrain_ros autonomy.launch.py

    # 실차 주행까지 (🛑 아래 안전 조건 확인 후)
    ros2 launch powertrain_ros autonomy.launch.py chassis:=true guidance:=lane

    노트북:  rviz2 -d <repo>/ros2/src/powertrain_ros/config/robot_viz.rviz

전제: **L515 Gateway 가 이미 떠 있어야 한다** (이 launch 는 Gateway 를 띄우지 않는다).
    docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros

────────────────────────────────────────────────────────────────────────
계층 (아래로 갈수록 모터에 가깝다)
────────────────────────────────────────────────────────────────────────
    [인지]  l515_cloud · lane_follower / wall_follower
                              │ 제안
                              ▼
    [권한]  /autonomy/cmd_vel ─→ chassis_node 내장 authority
                                         │
    [차체]  ChassisManager.set() ◀───────┘   (US-100 + SafetyInterlock 이 최종 게이트)
    [진단]  obstacle_zones → /diagnostics/obstacle/speed_scale (production 미연결)
    [상태]  odometry · imu_tilt · robot_state_publisher · mission

🛑 **주행(`chassis:=true`)은 기본 꺼짐이다.** 켜기 전에:
   · 바퀴 6개를 완전히 띄웠거나, 주행 가능한 안전한 공간인가
   · 48 V 물리 E-stop 에 손이 닿는가
   · `teleop_server` 가 안 떠 있는가 (can0 락이 막지만, 확인이 먼저다)
   · ODrive 재캘리를 했는가 (전원 사이클마다 필요 — 안 하면 arm 은 되는데 안 돈다)

🛑 **유도 소스는 동시에 켜지 않는다** — 모두 `/autonomy/cmd_vel` 를 쓴다.
   `guidance:=lane|wall|follow|terrain` 중 하나만 선택한다.

⚠️ 주행을 켜도 **바로 안 움직인다.** `chassis_node` 내장 authority가 기본 IDLE이다:
       ros2 service call /chassis_node/authority_auto std_srvs/srv/Trigger
   그리고 **중립 확인**(자율이 0 을 한 번 보냄) 후에야 권한이 넘어간다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("powertrain_ros")
    xacro_path = os.path.join(share, "urdf", "jetin_rover.urdf.xacro")
    robot_description = ParameterValue(Command(["xacro ", xacro_path]), value_type=str)

    guidance = LaunchConfiguration("guidance")
    chassis = LaunchConfiguration("chassis")
    lane_on = PythonExpression(["'", guidance, "' == 'lane'"])
    wall_on = PythonExpression(["'", guidance, "' == 'wall'"])
    follow_on = PythonExpression(["'", guidance, "' == 'follow'"])
    terrain_on = PythonExpression(["'", guidance, "' == 'terrain'"])

    args = [
        DeclareLaunchArgument("stride", default_value="2",
                              description="점군 픽셀 간격 (2=45k점, 4=9k점)"),
        DeclareLaunchArgument(
            "guidance", default_value="none",
            description="유도 방식 택일: none | lane | wall | follow | terrain. "
                        "🛑 동시에 켜지 않는다 (전부 /autonomy/cmd_vel 를 쓴다)"),
        DeclareLaunchArgument(
            "chassis", default_value="false",
            description="🛑 실차 모터 제어. 바퀴 상태·E-stop·재캘리를 확인하고 켤 것"),
        DeclareLaunchArgument("fake_chassis", default_value="false",
                              description="가짜 모터로 chassis_node 를 띄운다 (벤치)"),
        DeclareLaunchArgument("min_rev", default_value="0.0",
                              description="코깅존 플로어 — 폐지(기본 0, D3). 재도입은 커미셔닝 재량. "
                                          "정본: docs/superpowers/specs/2026-07-17-abc-program-design.md §2.2"),
        DeclareLaunchArgument("propose", default_value="false",
                              description="선택한 유도 노드가 /autonomy/cmd_vel 로 제안한다"),
    ]

    # ── 상태 (항상) ──
    state = [
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             name="robot_state_publisher", output="screen",
             parameters=[{"robot_description": robot_description}]),
        Node(package="powertrain_ros", executable="joint_state_bridge",
             name="joint_state_bridge", output="screen"),
        Node(package="powertrain_ros", executable="imu_tilt", name="imu_tilt",
             output="screen",
             parameters=[{"publish_static_tf": False, "publish_odom_tf": False}]),
        Node(package="powertrain_ros", executable="odometry", name="odometry",
             output="screen"),
    ]

    # ── 인지 (항상) ──
    perception = [
        Node(package="powertrain_ros", executable="l515_cloud", name="l515_cloud",
             output="screen",
             parameters=[{"stride": LaunchConfiguration("stride")}]),
        Node(package="powertrain_ros", executable="obstacle_zones",
             name="obstacle_zones", output="screen"),
    ]

    # ── 유도 (택일) ──
    guidance_nodes = [
        Node(package="powertrain_ros", executable="lane_follower",
             name="lane_follower", output="screen",
             condition=IfCondition(lane_on),
             parameters=[{"enabled": LaunchConfiguration("propose")}]),
        Node(package="powertrain_ros", executable="wall_follower",
             name="wall_follower", output="screen",
             condition=IfCondition(wall_on),
             parameters=[{"enabled": LaunchConfiguration("propose")}]),
        # WP9 앞 로봇 추종 — 추종 중에는 /chassis_mode 가 FOLLOW_LEAD 가 된다(팔 자세 락)
        Node(package="powertrain_ros", executable="lead_follower",
             name="lead_follower", output="screen",
             condition=IfCondition(follow_on),
             parameters=[{"enabled": LaunchConfiguration("propose")}]),
        Node(package="powertrain_ros", executable="autonomy_controller",
             name="autonomy_controller", output="screen",
             condition=IfCondition(terrain_on),
             parameters=[{"enabled": LaunchConfiguration("propose")}]),
    ]

    # ── 미션 (항상 — 계약 출력은 기본 false라 실물 팔에 도착 신호를 보내지 않는다) ──
    control = [
        Node(package="powertrain_ros", executable="mission", name="mission",
             output="screen"),
    ]

    # ── 차체 (명시적으로 켜야 함) ──
    body = [
        Node(package="powertrain_ros", executable="chassis", name="chassis_node",
             output="screen",
             condition=IfCondition(chassis),
             parameters=[{"fake": LaunchConfiguration("fake_chassis"),
                          "min_rev": LaunchConfiguration("min_rev"),
                          "authority_enabled": True}]),
    ]

    return LaunchDescription(args + state + perception + guidance_nodes + control + body)
