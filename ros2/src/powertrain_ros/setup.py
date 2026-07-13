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
            ],
        ),
        (
            "share/" + package_name + "/config",
            ["config/l515.yaml", "config/l515_tilt.rviz", "config/robot_viz.rviz"],
        ),
        (
            "share/" + package_name + "/urdf",
            ["urdf/jetin_rover.urdf.xacro"],
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
        ],
    },
)
