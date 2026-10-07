"""손목 카메라 단독 기동 (2026-08-13 추가).

    ros2 launch robot_arm_perception wrist_camera.launch.py
    ros2 launch robot_arm_perception wrist_camera.launch.py mask_source:=yolo backend:=pt
    ros2 launch robot_arm_perception wrist_camera.launch.py f_px:=412.0 box_size_m:=0.095

영상 확인은 관제 GUI 의 영상 소스 `wrist`(원본) / `wrist_debug`(오버레이) 또는
RViz Image 디스플레이(`/wrist/debug_image`)로 한다.

⚠️ **`/pick_target` 을 발행하지 않는다** — 이 노드는 관측 전용이고, 팔의 목표 좌표는
여전히 전방 D435i(`perception_node`) 한 곳에서만 나온다. 픽 경로와 같이 띄워도 안전한
이유가 이것이다(`pick.launch.py` 는 손대지 않았다).

## optical frame static TF

`wrist_camera_link`(URDF 고정관절, `robot_state_publisher` 가 팔 자세 따라 갱신) 아래에
**REP-103 optical 회전**을 얹어 `wrist_camera_optical_frame` 을 만든다 — 노드가 발행하는
모든 메시지의 `frame_id` 가 이것이고, 없으면 RViz 도 TF 변환도 안 붙는다. 전방 캠의
`camera_link → camera_color_optical_frame`(`robot_arm_description/camera_tf.launch.py`)과
같은 회전·같은 이유다.

⚠️ **장착 오차를 여기서 보정하지 말 것.** 이 TF 는 "광학축 규약 회전"만 담당하고,
카메라가 실제로 어느 방향을 보는가는 **URDF `fixed_joint_035`** 한 곳이 정한다(현재
`rpy="0 0 0"` — CAD 자동생성분이라 **아직 실물 검증 안 됨**). 여기서 각도를 만지면 URDF
와 두 곳이 진실을 나눠 갖게 되고, 다음 CAD 재export 때 조용히 어긋난다.
"""
import math
import os
from typing import List

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

#: body(X전방·Y좌·Z상) → optical(Z전방·X우·Y하). 전방 캠 launch 와 같은 상수.
OPTICAL_ROLL = -math.pi / 2.0
OPTICAL_PITCH = 0.0
OPTICAL_YAW = -math.pi / 2.0

#: `ls /dev/v4l/by-id/` 실측값(2026-08-13). 번호(/dev/video2)는 부팅·핫플러그마다 바뀌므로
#: 기본값으로 쓰지 않는다 — RealSense 가 video4~9 를 한꺼번에 잡아 순서가 잘 밀린다.
DEFAULT_DEVICE = '/dev/v4l/by-id/usb-Generic_USB_camera_200901010001-video-index0'


def _workspace_root(share_dir):
    """`model_path` 기본값이 상대경로라 CWD=워크스페이스 루트를 가정한다.

    `camera_calib.launch.py` 와 같은 이유·같은 방식(설치 경로에서 역산).
    """
    candidate = os.path.abspath(os.path.join(share_dir, *(['..'] * 4)))
    if os.path.isdir(os.path.join(candidate, 'src', 'robot_arm_perception')):
        return candidate
    return os.getcwd()


def generate_launch_description():
    share = get_package_share_directory('robot_arm_perception')
    args = [
        DeclareLaunchArgument('device', default_value=DEFAULT_DEVICE),
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480',
                              description='전방 캠과 같은 640x480 이면 TensorRT 엔진 캐시를 '
                                          '재사용한다(엔진 재빌드 8분 회피)'),
        DeclareLaunchArgument('mask_source', default_value='color',
                              description='color=HSV 색상 마스크(기본, GPU 미사용) | yolo=seg 모델. '
                                          '파지 거리에서 YOLO 가 대상을 못 봐 color 가 기본이다'),
        DeclareLaunchArgument('roi', default_value='[0.10, 0.90, 0.45, 1.00]',
                              description='그리퍼가 보이는 화면 영역 (x0,x1,y0,y1) 비율 — '
                                          '배경 택배 상자 배제용. 카메라 재장착 시 다시 잡을 것'),
        DeclareLaunchArgument('thin_reject_px', default_value='15',
                              description='이보다 가는 구조물(빨간 케이블)은 지운다. 0 이면 끈다'),
        DeclareLaunchArgument('backend', default_value='trt'),
        DeclareLaunchArgument('inference_rate_hz', default_value='5.0',
                              description='전방 캠 추론과 GPU 를 나눠 쓴다 — 낮게 유지할 것'),
        DeclareLaunchArgument('conf_threshold', default_value='0.5'),
        DeclareLaunchArgument('frame_id', default_value='wrist_camera_optical_frame'),
        # 아래 둘은 실측 전까지 0 — 그러면 metrics 의 distance_m 만 null 이 되고
        # fill/u/v 등 나머지 지표는 그대로 나온다.
        # ⚠️ f_px 는 **가로 기준**이다(세로는 단축돼 2.7배 다르다).
        #    실측: scripts/measure_wrist_proximity.py
        #
        # 2026-08-14 실측 확정: **360.7** (160/200/260mm 3점, 잔차 최대 1.4mm,
        # 기준점 오프셋 -0.2mm). 같은 프레임의 독립적인 세 특징으로 교차검증됨 —
        # 큐브 실루엣 95mm(+0.2%), 빨간 원판 50mm(-0.5%), 원판 구멍 25mm(-4.0%).
        # ⚠️ 옛 값 412 는 폐기: 케이블이 붙어 부푼 마스크로 잰 단일점이라 14% 컸다.
        DeclareLaunchArgument('f_px', default_value='360.7',
                              description='가로 기준 초점거리(px). 2026-08-14 실측 확정'),
        DeclareLaunchArgument('box_size_m', default_value='0.095',
                              description='대상의 **가로로 보이는 변** 실치수(m). 95mm 큐브'),
        DeclareLaunchArgument('optical_tf', default_value='true',
                              description='wrist_camera_link → wrist_camera_optical_frame '
                                          'static TF 를 같이 띄운다(모듈 docstring 참고)'),
    ]
    return LaunchDescription(args + [
        Node(
            package='robot_arm_perception',
            executable='wrist_camera',
            output='screen',
            cwd=_workspace_root(share),
            parameters=[{
                'camera_device': LaunchConfiguration('device'),
                'image_width': LaunchConfiguration('width'),
                'image_height': LaunchConfiguration('height'),
                'mask_source': LaunchConfiguration('mask_source'),
                # ⚠️ 리스트 파라미터는 반드시 value_type 을 명시한다 — 그냥 넘기면 문자열
                #    '[0.1, ...]' 로 들어가 declare 한 double_array 와 안 맞아
                #    InvalidParameterTypeException 으로 노드가 죽는다(관제 GUI 의
                #    video_default_source YAML 함정과 같은 계열의 사고다).
                'roi': ParameterValue(LaunchConfiguration('roi'), value_type=List[float]),
                'thin_reject_px': LaunchConfiguration('thin_reject_px'),
                'backend': LaunchConfiguration('backend'),
                'inference_rate_hz': LaunchConfiguration('inference_rate_hz'),
                'conf_threshold': LaunchConfiguration('conf_threshold'),
                'frame_id': LaunchConfiguration('frame_id'),
                'f_px': LaunchConfiguration('f_px'),
                'box_size_m': LaunchConfiguration('box_size_m'),
            }],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='wrist_camera_link_to_optical',
            condition=IfCondition(LaunchConfiguration('optical_tf')),
            arguments=[
                '--x', '0.0', '--y', '0.0', '--z', '0.0',
                '--roll', str(OPTICAL_ROLL),
                '--pitch', str(OPTICAL_PITCH),
                '--yaw', str(OPTICAL_YAW),
                '--frame-id', 'wrist_camera_link',
                '--child-frame-id', 'wrist_camera_optical_frame',
            ],
        ),
    ])
