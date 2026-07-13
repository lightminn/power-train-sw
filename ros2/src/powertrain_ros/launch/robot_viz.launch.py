"""로봇 모델 + 라이다 + IMU 기울임 통합 시각화 (WP6 Step 1·3·4).

    젯슨 (powertrain_ros 컨테이너 안, Gateway 가 이미 떠 있어야 함):
        ros2 launch powertrain_ros robot_viz.launch.py

    노트북 (같은 네트워크):
        rviz2 -d <repo>/ros2/src/powertrain_ros/config/robot_viz.rviz

`l515_viz.launch.py`(클라우드 + IMU TF)에 **URDF 로봇 모델**을 얹은 것이다.

    odom ──(IMU 자세)── base_link ──(URDF)── 바퀴 6 · 조향 4 · 센서 3
                                      ↑
                    robot_state_publisher ← /joint_states ← /wheel_states

모터가 꺼져 있어도 `joint_state_bridge` 가 0 자세를 계속 발행하므로 **로봇은 화면에 뜬다.**
모터를 돌리면(ChassisManager → `/wheel_states`) 바퀴와 조향이 화면에서 실제로 움직인다.

⚠️ URDF 치수 중 **윤거·차체·센서 마운트는 미실측 플레이스홀더**다. 설계팀 정본이 오면
   `urdf/jetin_rover.urdf.xacro` 의 property 만 갈아끼운다. 파이프라인은 그대로 산다.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, NotSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory("powertrain_ros")
    xacro_path = os.path.join(share, "urdf", "jetin_rover.urdf.xacro")

    robot_description = ParameterValue(
        Command(["xacro ", xacro_path]), value_type=str)

    fake = LaunchConfiguration("fake_wheels")
    args = [
        DeclareLaunchArgument("stride", default_value="4",
                              description="클라우드 픽셀 간격 (클수록 가벼움)"),
        DeclareLaunchArgument("alpha", default_value="0.98",
                              description="상보 필터 자이로 가중"),
        DeclareLaunchArgument(
            "fake_wheels", default_value="false",
            description="🛑 벤치 전용 — 가짜 /wheel_states 로 주행을 흉내낸다. 실차 금지."),
        DeclareLaunchArgument("course", default_value="square",
                              description="가짜 주행 코스 (straight/circle/square/figure8/pivot)"),
        DeclareLaunchArgument(
            "lane", default_value="false",
            description="레인 추종 노드를 띄운다 (인식만 — 주행은 chassis_node authority가 결정)"),
    ]

    return LaunchDescription(args + [
        # 오도메트리 — /wheel_states + IMU → /odom + TF(odom→base_link)
        # 원칙상 회전은 IMU 에서 가져온다("바퀴=거리, IMU=회전"). 단 **가짜 휠 모드에서는
        # IMU 가 그 가상 로봇에 붙어 있지 않다** — 책상 위 젯슨은 "안 돈다"고 말하므로
        # 로봇이 직진만 하게 된다. 그래서 fake_wheels 일 때만 휠 ω 로 되돌린다.
        Node(
            package="powertrain_ros",
            executable="odometry",
            name="odometry",
            output="screen",
            parameters=[{
                "use_imu_yaw": ParameterValue(NotSubstitution(fake), value_type=bool),
            }],
        ),
        # 🛑 벤치 전용 가짜 휠 (fake_wheels:=true 일 때만)
        Node(
            package="powertrain_ros",
            executable="fake_wheels",
            name="fake_wheels",
            output="screen",
            condition=IfCondition(fake),
            parameters=[{"course": LaunchConfiguration("course")}],
        ),
        # URDF → base_link 아래 모든 TF 를 자동 생성
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        # /wheel_states → /joint_states (모터 꺼져 있으면 0 발행)
        Node(
            package="powertrain_ros",
            executable="joint_state_bridge",
            name="joint_state_bridge",
            output="screen",
        ),
        # L515 depth → PointCloud2
        Node(
            package="powertrain_ros",
            executable="l515_cloud",
            name="l515_cloud",
            output="screen",
            parameters=[{"stride": LaunchConfiguration("stride")}],
        ),
        # 점구름 → 바닥/장애물 분리 → 좌·중·우 판정 → speed_scale (WP7)
        # 🛑 감속 힌트일 뿐 안전 최종 게이트가 아니다 (US-100 + SafetyInterlock 이 게이트).
        Node(
            package="powertrain_ros",
            executable="obstacle_zones",
            name="obstacle_zones",
            output="screen",
        ),
        # 레인 추종 (인식만). /autonomy/cmd_vel 로 **제안**하고, 실제 주행 여부는
        # chassis_node authority가 AUTO 모드 + 중립 확인 후에 정한다.
        Node(
            package="powertrain_ros",
            executable="lane_follower",
            name="lane_follower",
            output="screen",
            condition=IfCondition(LaunchConfiguration("lane")),
        ),
        # IMU → /imu/filtered (자세 + 편향보정 각속도).
        # 정적 TF 는 URDF(robot_state_publisher)가, odom→base_link 는 odometry 가 소유
        # → 여기서는 둘 다 끈다. **같은 TF 를 두 곳에서 쏘면 충돌한다.**
        Node(
            package="powertrain_ros",
            executable="imu_tilt",
            name="imu_tilt",
            output="screen",
            parameters=[{
                "alpha": LaunchConfiguration("alpha"),
                "publish_static_tf": False,
                "publish_odom_tf": False,
            }],
        ),
    ])
