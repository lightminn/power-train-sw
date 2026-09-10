"""조이스틱 벤치 텔레옵 — 파워트레인 없이 팔만 구동한다.

기동 대상:
  - joy_node        : 게임패드 → /joy
  - joystick_teleop : /joy → /arm/teleop_jog (5축 직접 매핑)
  - teleop_core     : /arm/teleop_jog → /dynamixel/goal_position
  - position_node   : use_hardware:=true 일 때만 (실서보, /dev/ttyUSB0)
  - robot_state_publisher + rviz2 : rviz:=true 일 때만

**격리는 이 launch 파일이 전부다.** `arm_fsm` 을 띄우지 않으므로 계약 토픽
(/arm_status, /chassis_mode, /arrival_status)이 **아예 생기지 않는다.**
노드 코드에 "테스트 모드면 건너뛰기" 같은 분기는 넣지 않았다 — 안전 게이트에
스킵 분기가 있으면 실기에서 켜진 채 도는 사고가 언젠가 난다.

/joint_states 소스 (충돌 방지 위해 하나만):
  - use_hardware:=false → teleop_core 가 목표값을 sim 으로 발행
  - use_hardware:=true  → position_node 가 실서보 엔코더로 발행 (teleop_core sim 은 off)

⚠️ 이 경로(direct dynamixel goal publisher)는 파워트레인 계약상 **production 금지**다.
   벤치/개발 전용이며, 대회 launch 에 넣지 않는다.

사용 예)
  # 하드웨어 없이 RViz 로 확인
  ros2 launch dynamixel_control bench.launch.py rviz:=true
  # 실서보까지
  ros2 launch dynamixel_control bench.launch.py use_hardware:=true rviz:=true
  # 패드 없이 가짜 /joy 로 검증할 때 (joy_node 를 끈다)
  ros2 launch dynamixel_control bench.launch.py joy_node:=false rviz:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    rviz = LaunchConfiguration('rviz')
    joy_node = LaunchConfiguration('joy_node')
    joy_device = LaunchConfiguration('joy_device')
    port = LaunchConfiguration('port')

    urdf_path = os.path.join(
        get_package_share_directory('robot_arm_description'),
        'urdf', 'robot_arm.urdf',
    )
    with open(urdf_path, 'r') as f:
        robot_description = f.read()

    rviz_config = os.path.join(
        get_package_share_directory('dynamixel_control'),
        'rviz', 'teleop.rviz',
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_hardware', default_value='false',
            description='true 면 position_node(실서보, /dev/ttyUSB0)도 함께 기동',
        ),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='true 면 robot_state_publisher + rviz2 기동',
        ),
        DeclareLaunchArgument(
            'joy_node', default_value='true',
            description='false 면 joy_node 를 안 띄운다 (가짜 /joy 로 검증할 때)',
        ),
        DeclareLaunchArgument(
            'joy_device', default_value='0',
            description='joy_node 의 device_id (/dev/input/js<N>)',
        ),
        DeclareLaunchArgument(
            'port', default_value='/dev/ttyUSB0',
            description='position_node 가 열 시리얼 포트. USB 재연결로 번호가 '
                        '밀리면(/dev/ttyUSB1 등) 바꿔서 넘길 것',
        ),

        # 게임패드 드라이버.
        # autorepeat_rate 를 명시하는 이유: 스틱을 가만히 붙들고 있어도 /joy 가 계속
        # 흐르게 해서, joystick_teleop 의 신선도 감시가 "패드 사망"만 잡도록 한다.
        Node(
            package='joy', executable='joy_node', name='joy_node', output='screen',
            parameters=[{
                'device_id': joy_device,
                'autorepeat_rate': 50.0,
                'deadzone': 0.0,     # 데드존은 joystick_teleop 에서 관절별로 적용
            }],
            condition=IfCondition(joy_node),
        ),

        Node(
            package='dynamixel_control', executable='joystick_teleop',
            name='joystick_teleop', output='screen',
        ),

        # teleop_core — 하드웨어 없을 때: sim /joint_states 발행 ON
        Node(
            package='dynamixel_control', executable='teleop_core', name='teleop_core',
            output='screen',
            parameters=[{'publish_sim_joint_states': True}],
            condition=UnlessCondition(use_hardware),
        ),
        # teleop_core — 실서보일 때: sim OFF (position_node 가 /joint_states 담당)
        Node(
            package='dynamixel_control', executable='teleop_core', name='teleop_core',
            output='screen',
            parameters=[{'publish_sim_joint_states': False}],
            condition=IfCondition(use_hardware),
        ),

        Node(
            package='dynamixel_control', executable='position_node',
            name='dynamixel_position_node', output='screen',
            parameters=[{'port': port}],
            condition=IfCondition(use_hardware),
        ),

        Node(
            package='robot_state_publisher', executable='robot_state_publisher',
            name='robot_state_publisher', output='screen',
            parameters=[{'robot_description': robot_description}],
            condition=IfCondition(rviz),
        ),
        Node(
            package='rviz2', executable='rviz2', name='rviz2',
            arguments=['-d', rviz_config], output='screen',
            condition=IfCondition(rviz),
        ),
    ])
