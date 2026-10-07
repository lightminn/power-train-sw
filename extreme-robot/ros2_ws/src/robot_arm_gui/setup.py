import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_arm_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # ⚠️ glob('web/*') 는 하위 디렉터리를 담지 못한다 — web/ 은 평면으로 유지할 것.
        (os.path.join('share', package_name, 'web'),
            [p for p in glob('web/*') if os.path.isfile(p)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ksp118',
    maintainer_email='koreanmcpe@gmail.com',
    description='브라우저 관제 GUI (읽기 전용) — stdlib HTTP + SSE + MJPEG',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'monitor = robot_arm_gui.telemetry_node:main',
            'fake_publisher = robot_arm_gui.fake_publisher:main',
        ],
    },
)
