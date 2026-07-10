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
            ["launch/wp5_control.launch.py"],
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
        ],
    },
)
