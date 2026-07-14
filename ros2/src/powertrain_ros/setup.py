from setuptools import setup

package_name = "powertrain_ros"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            "share/" + package_name + "/launch",
            [
                "launch/wp5_control.launch.py",
                "launch/l515_viz.launch.py",
                "launch/robot_viz.launch.py",
                "launch/autonomy.launch.py",
            ],
        ),
        (
            "share/" + package_name + "/config",
            [
                "config/l515.yaml",
                "config/l515_tilt.rviz",
                "config/robot_viz.rviz",
                "config/wheel_stop.yaml",
            ],
        ),
        (
            "share/" + package_name + "/urdf",
            ["urdf/jetin_rover.urdf.xacro"],
        ),
        (
            # 설계팀 CAD 형상 (메시 없이 관성텐서로 역산한 상자·원통 — cad:=true 로 쓴다)
            "share/" + package_name + "/urdf/cad",
            ["urdf/cad/rover_cad_boxes.urdf"],
        ),
    ],
    install_requires=["setuptools"],
    tests_require=["pytest"],
    zip_safe=True,
    maintainer="ZETIN Powertrain",
    maintainer_email="nitez0423@gmail.com",
    description="파워트레인 ROS2 브릿지 — 로봇팔 FSM 연동 (ROS 는 껍데기)",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "bringup = powertrain_ros.bringup_node:main",
            "chassis = powertrain_ros.chassis_node:main",
            "us100_safety = powertrain_ros.us100_safety_node:main",
            "l515_cloud = powertrain_ros.l515_cloud_node:main",
            "imu_tilt = powertrain_ros.imu_tilt_node:main",
            "joint_state_bridge = powertrain_ros.joint_state_bridge_node:main",
            "odometry = powertrain_ros.odometry_node:main",
            "fake_wheels = powertrain_ros.fake_wheels_node:main",
            "obstacle_zones = powertrain_ros.obstacle_zones_node:main",
            "lane_follower = powertrain_ros.lane_follower_node:main",
            "mission = powertrain_ros.mission_node:main",
            "wall_follower = powertrain_ros.wall_follower_node:main",
            "lead_follower = powertrain_ros.lead_follower_node:main",
        ],
    },
)
