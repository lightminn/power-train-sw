"""비전 → 픽 스택 (구간2 pick 경로) 일괄 기동.

여태 이 경로에는 launch 파일이 없어서 노드를 하나씩 손으로 띄워야 했고, 순서를
틀리면 조용히 실패했다. 이 파일이 그 순서를 고정한다.

기동 대상:
  - moveit_dynamixel_bridge : 실서보(/dev/ttyUSB0) + /joint_states + /arm_controller
  - robot_state_publisher   : URDF → TF (base_link↔link_043)
  - camera_tf               : base_link → camera_link → camera_color_optical_frame
  - move_group              : ⚠️ ik_mode=analytic 이어도 **필수**다 (아래 참고)
  - perception_node         : RealSense + YOLO seg → /detected_objects, /pick_target
  - arm_fsm                 : 픽 FSM. /pick_target + 계약 게이트 → 팔 구동

선택 기동(기본 off — 현장에서는 디스플레이가 없다):
  - wrist_camera (wrist:=true) : 손목 USB 캠 → /wrist/{raw,debug}_image, /wrist/metrics
  - rviz2        (rviz:=true)  : 로봇 모델(실제 관절값) + depth 포인트클라우드 +
                                 검출 마커 + 손목캠 영상을 한 화면에서 본다
                                 (config: robot_arm_description/rviz/live_view.rviz)

게이트를 여는 것(=미션 지시)은 여기 없다. 파워트레인이 없는 벤치에서는
**별도 터미널**에서 운영자 콘솔을 띄운다(stdin 이 필요해 launch 에 못 넣는다):

    ros2 run dynamixel_control mission_console

## ⚠️ 버스 독점

`moveit_dynamixel_bridge` 가 `/dev/ttyUSB0` 를 잡는다. `position_node`/`teleop_core`
(벤치 텔레옵)와 **절대 같이 띄우지 말 것** — 같은 시리얼 포트를 둘이 쓰면 패킷이
섞여 원인 불명의 무응답이 된다. 텔레옵 스택을 먼저 내리고 이걸 띄운다.

## ⚠️ move_group 이 왜 필요한가

`arm_fsm` 의 기본값 `ik_mode='analytic'` 은 MoveGroup 의 **계획**을 우회하지만
FK 서비스 `/compute_fk` 는 그대로 쓴다. 그 서비스를 제공하는 게 move_group 이라,
안 띄우면 IK 반복이 매번 타임아웃으로 죽는다. "analytic 은 MoveGroup 을
우회한다"는 서술과 헷갈리지 말 것 — 우회하는 것은 planning 이지 FK 가 아니다.

## ⚠️ 해상도 기본값

RealSense 가 **USB 2.1** 로 물려 있으면 848×480@30 은 스트림이 열리지도 않는다.
그래서 기본값을 640×480@15 로 둔다. USB 3 로 확인됐다면 인자로 올릴 것.

사용 예)
  ros2 launch dynamixel_control pick.launch.py
  ros2 launch dynamixel_control pick.launch.py width:=848 height:=480 fps:=30
  ros2 launch dynamixel_control pick.launch.py model_name:=box backend:=trt
  # 서보 없이 인식·계획만 확인 (팔은 안 붙는다)
  ros2 launch dynamixel_control pick.launch.py use_hardware:=false
  # 손목캠 + 뎁스캠 + 실제로 움직이는 로봇을 RViz 한 화면에서 보기
  ros2 launch dynamixel_control pick.launch.py rviz:=true wrist:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_hardware = LaunchConfiguration('use_hardware')
    ik_mode = LaunchConfiguration('ik_mode')

    args = [
        DeclareLaunchArgument(
            'use_hardware', default_value='true',
            description='실서보(moveit_dynamixel_bridge)를 띄운다. false 면 인식/계획만.'),
        DeclareLaunchArgument(
            'ik_mode', default_value='analytic',
            description="arm_fsm 의 IK 경로. 'analytic'(기본) | 'moveit'"),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='15'),
        DeclareLaunchArgument(
            'model_name', default_value='box',
            description='perception_node 의 model_presets 프리셋 이름'),
        DeclareLaunchArgument(
            'backend', default_value='pt',
            description="'pt'(즉시) | 'trt'(첫 실행 시 엔진 빌드 ~8분, 이후 빠름)"),
        # ⚠️ 기본값이 **두 곳**에 있다 — 여기(launch)와 arm_fsm_node 의 declare_parameter.
        # launch 가 항상 파라미터를 넘기므로 **여기가 이긴다.** 2026-08-19 노드 쪽만
        # true 로 바꿨다가 실기에서 접이 모션이 조용히 안 돌았다(로그에 carry_home
        # 메시지가 아예 없고 LIFT→CARRY 로 직행). 한쪽만 고치지 말 것.
        DeclareLaunchArgument(
            'carry_home', default_value='true',
            description='파지 후 물건을 문 채 home(접힘) 자세로 돌아간 뒤 CARRY 대기'),
        DeclareLaunchArgument(
            'rviz', default_value='false',
            description='관측용 RViz(live_view.rviz)를 같이 띄운다. 디스플레이 필요'),
        DeclareLaunchArgument(
            'wrist', default_value='false',
            description='손목 USB 캠 노드도 띄운다. /pick_target 은 발행하지 않아 '
                        '픽 경로와 간섭하지 않는다(wrist_camera.launch.py 참고)'),
        # 발행 자체가 **구독자 게이트** 안에 있다(perception_node._maybe_publish_cloud
        # 가 subscription_count==0 이면 즉시 리턴) — 아무도 안 보면 비용이 0 이라
        # 기본으로 켜 둔다. RViz 를 열자마자 클라우드가 나오는 이유가 이것이다.
        DeclareLaunchArgument(
            'cloud', default_value='true',
            description='depth 포인트클라우드 발행(구독자가 있을 때만 실제로 그린다)'),
    ]

    moveit_share = get_package_share_directory('robot_arm_moveit_config')
    desc_share = get_package_share_directory('robot_arm_description')

    rsp = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'rsp.launch.py')))

    move_group = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(moveit_share, 'launch', 'move_group.launch.py')))

    camera_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(desc_share, 'launch', 'camera_tf.launch.py')))

    bridge = Node(
        package='dynamixel_control',
        executable='moveit_dynamixel_bridge',
        name='moveit_dynamixel_bridge',
        output='screen',
        condition=IfCondition(use_hardware),
    )

    perception = Node(
        package='robot_arm_perception',
        executable='perception_node',
        name='perception_node',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'),
            'fps': LaunchConfiguration('fps'),
            'model_name': LaunchConfiguration('model_name'),
            'backend': LaunchConfiguration('backend'),
            'publish_cloud': LaunchConfiguration('cloud'),
        }],
    )

    fsm = Node(
        package='dynamixel_control',
        executable='arm_fsm',
        name='arm_fsm_node',
        output='screen',
        parameters=[{
            'ik_mode': ik_mode,
            'carry_home': LaunchConfiguration('carry_home'),
        }],
    )

    perception_share = get_package_share_directory('robot_arm_perception')
    wrist = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(perception_share, 'launch', 'wrist_camera.launch.py')),
        condition=IfCondition(LaunchConfiguration('wrist')),
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(desc_share, 'rviz', 'live_view.rviz')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # /detected_objects → RViz 큐브. 읽기 전용이고 RViz 없이는 볼 사람이 없어서
    # rviz 인자에 묶는다(camera_calib.launch.py 도 같은 노드를 쓴다).
    detection_markers = Node(
        package='robot_arm_perception',
        executable='detection_markers',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(
        args + [rsp, camera_tf, move_group, bridge, perception, fsm, wrist,
                rviz, detection_markers])
