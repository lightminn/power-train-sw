import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_arm_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jo',
    maintainer_email='ddkk0714@naver.com',
    description='RealSense + YOLO perception node',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'perception_node = robot_arm_perception.perception_node:main',
            'stream_node = robot_arm_perception.stream_node:main',
            'metadata_sender_node = robot_arm_perception.metadata_sender_node:main',
            'vision_test = robot_arm_perception.vision_test_node:main',
            'detection_markers = robot_arm_perception.detection_markers:main',
            'camera_tf_tuner = robot_arm_perception.camera_tf_tuner:main',
            'calib_status_view = robot_arm_perception.calib_status_view:main',
            'wrist_camera = robot_arm_perception.wrist_camera_node:main',
            'ground_truth_markers = robot_arm_perception.ground_truth_markers:main',
            'depth_method_compare = robot_arm_perception.depth_method_compare:main',
            'mask_pca_explain = robot_arm_perception.mask_pca_explain:main',
            'align_cost_explain = robot_arm_perception.align_cost_explain:main',
        ],
    },
)
