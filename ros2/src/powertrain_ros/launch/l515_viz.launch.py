"""BENCH/RViz 전용 L515 시각화 — 포인트클라우드 + IMU 기울임 TF.

    젯슨 (powertrain_ros 컨테이너 안):
        ros2 launch powertrain_ros l515_viz.launch.py

    노트북 (같은 네트워크, ROS_DOMAIN_ID 동일):
        rviz2 -d <repo>/ros2/src/powertrain_ros/config/l515_tilt.rviz

전제: **L515 Gateway 가 이미 떠 있어야 한다** (이 launch 는 Gateway 를 띄우지 않는다).
    docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros
Gateway 가 `/l515/depth/...`, `/l515/accel|gyro/...` 를 발행하고, 여기 두 노드가 그것을
포인트클라우드와 TF 로 바꾼다.

⚠️ `mount_*` = base_link→l515_link 마운트. **아직 실측 전인 플레이스홀더**다
   (WP6 남은 커미셔닝). 마운트 조립 후 실측값으로 교체:
       ros2 launch powertrain_ros l515_viz.launch.py mount_x:=0.31 mount_z:=0.42
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ⚠️ 미실측 플레이스홀더 — 마운트 조립 후 CAD 대조 + 실측으로 확정할 것
MOUNT_DEFAULTS = {"mount_x": "0.30", "mount_y": "0.0", "mount_z": "0.35"}


def generate_launch_description():
    args = [
        DeclareLaunchArgument("stride", default_value="4",
                              description="클라우드 픽셀 간격 (4 = 160x120, 클수록 가벼움)"),
        DeclareLaunchArgument("alpha", default_value="0.98",
                              description="상보 필터 자이로 가중"),
    ] + [
        DeclareLaunchArgument(k, default_value=v,
                              description="⚠️ base_link→l515_link 마운트 (미실측)")
        for k, v in MOUNT_DEFAULTS.items()
    ]

    cloud = Node(
        package="powertrain_ros",
        executable="l515_cloud",
        name="l515_cloud",
        output="screen",
        parameters=[{"stride": LaunchConfiguration("stride")}],
    )
    tilt = Node(
        package="powertrain_ros",
        executable="imu_tilt",
        name="imu_tilt",
        output="screen",
        parameters=[{
            "alpha": LaunchConfiguration("alpha"),
            "mount_x": LaunchConfiguration("mount_x"),
            "mount_y": LaunchConfiguration("mount_y"),
            "mount_z": LaunchConfiguration("mount_z"),
        }],
    )
    return LaunchDescription(args + [cloud, tilt])
