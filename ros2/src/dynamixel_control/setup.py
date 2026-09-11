from glob import glob

from setuptools import find_packages, setup

package_name = 'dynamixel_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'MASTER_SLAVE_BENCH.md']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config', glob('config/*.json')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'position_node = dynamixel_control.dynamixel_position_node:main',
            'dynamixel_position_node = dynamixel_control.dynamixel_position_node:main',
            'moveit_dynamixel_bridge = dynamixel_control.moveit_dynamixel_bridge:main',
            'mock_moveit_dynamixel_bridge = '
            'dynamixel_control.mock_moveit_dynamixel_bridge:main',
            'gripper_calibration = dynamixel_control.gripper_calibration:main',
            'recorded_path_replay = '
            'dynamixel_control.recorded_path_replay:main',
            'gripper_load_calibration = dynamixel_control.gripper_load_calibration:main',
            'spur_gripper_calibration = dynamixel_control.spur_gripper_calibration:main',
            'yolo_bridge = dynamixel_control.yolo_to_dynamixel_bridge:main',
            'yolo_detection = dynamixel_control.yolo_detection_node:main',
            'arm_fsm = dynamixel_control.arm_fsm_node:main',
            'mission_console = dynamixel_control.mission_console_node:main',
            'teleop_core = dynamixel_control.teleop_core_node:main',
            'keyboard_teleop = dynamixel_control.keyboard_teleop_node:main',
            'joystick_teleop = dynamixel_control.joystick_teleop_node:main',
            'master_slave_master = '
            'dynamixel_control.master_slave_master_client:main',
            'master_slave_slave = '
            'dynamixel_control.master_slave_slave_server:main',
        ],
    },
)
