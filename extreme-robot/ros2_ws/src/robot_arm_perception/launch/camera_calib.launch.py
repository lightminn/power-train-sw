"""카메라 TF 캘리브레이션 한 방 실행 (2026-08-07 추가).

RViz에서 **카메라 프레임을 드래그해가며** 검출된 박스가 실제 박스 위치(와이어프레임)에
겹치도록 맞추는 작업 일습을 띄운다:

    robot_state_publisher   URDF 로봇 모델 (맞았는지 판단할 시각 기준)
    joint_state_publisher_gui   서보 없이 팔 자세를 바꿔볼 수 있게
    camera_tf_tuner         base_link→camera_link 를 드래그로 조정 + 값 출력
    calib_status_view       그 안내를 한국어로 크게 띄우는 별도 창 (status_view:=false 로 끔)
    detection_markers       /detected_objects → RViz 큐브
    ground_truth_markers    줄자로 잰 박스 위치 → 흰 와이어프레임
    perception_node         RealSense + YOLO (+ depth 포인트클라우드)
    rviz2                   위 전부가 세팅된 config

사용:

    ros2 launch robot_arm_perception camera_calib.launch.py \\
        gt_points:="30,0,3; 35,15,3; 38,-15,8"
    # 숫자 입력 GUI(rqt_reconfigure)까지 원하면 rqt:=true

`gt_points` 는 **줄자로 잰 박스 위치(cm, base_link 기준)** 다. 흰 와이어프레임으로 그려지는
동시에 `camera_tf_tuner` 의 우클릭 메뉴 항목이 된다 — 박스를 그 자리에 놓고 노란 카메라
몸통을 우클릭 → "측정 ▶ N번 측정" 을 누르면 여러 프레임 평균으로 관측을 기록하고,
3점 이상 모인 뒤 "해 계산" 을 누르면 최소자승으로 풀어 **카메라를 그 자리로 옮긴다.**
캘리브 중에 숫자를 타이핑할 일이 없다.

⚠️ `camera_tf.launch.py`(확정값 static TF)와 **같이 띄우지 말 것** — 같은 TF를 둘이
발행하면 트리가 깨진다. `camera_tf_tuner`가 그 역할을 대신하며, 기동 시 충돌을 감지해
에러 로그를 찍는다.

⚠️ RealSense가 USB 2.1로 붙어 있으면 기본 848x480@30이 **열리지도 않는다**
(2026-08-07 실측: 640x480@30은 열리지만 프레임이 안 오고, 640x480@15가 유일하게
정상 동작). 그래서 이 launch의 기본값은 640x480@15다. USB 3 포트로 옮겼다면
`width:=848 height:=480 fps:=30`으로 올려 쓰면 된다.
"""
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _workspace_root(share_dir):
    """perception_node 의 `model_path` 기본값이 상대경로라 CWD=워크스페이스 루트를 가정한다.

    `ros2 launch` 는 사용자가 있던 디렉터리를 그대로 물려주기 때문에, 루트가 아닌 곳에서
    띄우면 모델을 못 찾고 죽는다(run_vision_test.sh 가 cd 를 해주는 것과 같은 이유).
    설치 경로에서 워크스페이스 루트를 역산해 cwd 로 넘긴다.
    """
    candidate = os.path.abspath(os.path.join(share_dir, *(['..'] * 4)))
    if os.path.isdir(os.path.join(candidate, 'src', 'robot_arm_perception')):
        return candidate
    return os.getcwd()


def generate_launch_description():
    description_share = get_package_share_directory('robot_arm_description')
    robot_description = xacro.process_file(
        os.path.join(description_share, 'urdf', 'robot_arm.urdf.xacro')).toxml()
    rviz_config = os.path.join(description_share, 'rviz', 'camera_calib.rviz')

    args = [
        # USB 2.1 실측 기본값 — 모듈 docstring 참고
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        DeclareLaunchArgument('fps', default_value='15'),
        DeclareLaunchArgument('model_name', default_value='box'),
        DeclareLaunchArgument(
            'conf_threshold', default_value='0.5',
            description='검출이 안 잡히면 0.3 정도까지 낮춰볼 것'),
        DeclareLaunchArgument(
            'gt_points', default_value='',
            description='줄자로 잰 박스 위치, cm, base_link 기준. 예: "30,0,3; 35,15,3"'),
        # camera_tf.launch.py 의 현재 기본값에서 이어서 조정한다 — 두 파일을 항상
        # 같이 갱신할 것(여기서 맞춘 값을 그쪽 기본값으로 옮기는 게 이 도구의 출구다).
        DeclareLaunchArgument('cam_x', default_value='-0.0178'),
        DeclareLaunchArgument('cam_y', default_value='-0.8008'),
        DeclareLaunchArgument('cam_z', default_value='0.1645'),
        DeclareLaunchArgument('cam_roll', default_value='-0.0302'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0937'),
        DeclareLaunchArgument('cam_yaw', default_value='1.5108'),
        DeclareLaunchArgument(
            'perception', default_value='true',
            description='false 면 perception_node 를 안 띄운다(이미 따로 돌릴 때)'),
        DeclareLaunchArgument('cloud', default_value='true',
                              description='depth 포인트클라우드 발행'),
        DeclareLaunchArgument('jsp_gui', default_value='true'),
        DeclareLaunchArgument('rviz', default_value='true'),
        DeclareLaunchArgument(
            'status_view', default_value='true',
            description='RViz 3D 텍스트와 같은 안내를 한국어로 크게 띄우는 별도 창'),
        DeclareLaunchArgument('status_font_size', default_value='14'),
        DeclareLaunchArgument(
            'rqt', default_value='false',
            description='cam_x/y/z 를 숫자로 입력하는 GUI(rqt_reconfigure)도 같이 띄운다'),
    ]

    nodes = [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_description}],
        ),
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            condition=IfCondition(LaunchConfiguration('jsp_gui')),
        ),
        Node(
            package='robot_arm_perception',
            executable='camera_tf_tuner',
            output='screen',           # 드래그할 때마다 찍히는 값을 봐야 한다
            parameters=[{
                'cam_x': LaunchConfiguration('cam_x'),
                'cam_y': LaunchConfiguration('cam_y'),
                'cam_z': LaunchConfiguration('cam_z'),
                'cam_roll': LaunchConfiguration('cam_roll'),
                'cam_pitch': LaunchConfiguration('cam_pitch'),
                'cam_yaw': LaunchConfiguration('cam_yaw'),
                # 와이어프레임과 우클릭 메뉴가 같은 목록을 보게 한다
                'gt_points': LaunchConfiguration('gt_points'),
            }],
        ),
        Node(
            package='robot_arm_perception',
            executable='detection_markers',
        ),
        Node(
            package='robot_arm_perception',
            executable='ground_truth_markers',
            output='screen',
            parameters=[{'points': LaunchConfiguration('gt_points')}],
        ),
        Node(
            package='robot_arm_perception',
            executable='perception_node',
            output='screen',
            condition=IfCondition(LaunchConfiguration('perception')),
            cwd=_workspace_root(get_package_share_directory('robot_arm_perception')),
            parameters=[{
                'model_name': LaunchConfiguration('model_name'),
                'width': LaunchConfiguration('width'),
                'height': LaunchConfiguration('height'),
                'fps': LaunchConfiguration('fps'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'publish_cloud': LaunchConfiguration('cloud'),
            }],
        ),
        # 3D 뷰의 텍스트 마커는 작고·가려지고·한글을 못 그린다(Ogre 폰트). 같은 안내를
        # 한국어로 크게 띄우는 창을 옆에 둔다 — 읽기 전용이라 캘리브 동작에 관여하지 않는다.
        Node(
            package='robot_arm_perception',
            executable='calib_status_view',
            output='screen',
            condition=IfCondition(LaunchConfiguration('status_view')),
            parameters=[{'font_size': LaunchConfiguration('status_font_size')}],
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', rviz_config],
            condition=IfCondition(LaunchConfiguration('rviz')),
        ),
        # RViz2 는 파이썬 패널 플러그인을 지원하지 않는다(C++ 컴파일 필요). 대신 튜너의
        # 자세가 전부 ROS 파라미터라, rqt_reconfigure 가 그대로 숫자 입력 GUI가 된다.
        Node(
            package='rqt_reconfigure',
            executable='rqt_reconfigure',
            condition=IfCondition(LaunchConfiguration('rqt')),
        ),
    ]
    return LaunchDescription(args + nodes)
