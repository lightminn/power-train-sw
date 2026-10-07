"""카메라 static TF 발행 — 전방 RGB-D(RealSense D435i).

perception_node 는 pyrealsense2 를 직접 쓰므로(realsense-ros 드라이버 미사용) 아무도
TF 를 내지 않는다. /pick_target 의 frame_id='camera_color_optical_frame' 을 MoveIt 이
base_link(planning frame)로 변환하려면 이 체인이 TF 트리에 있어야 한다 (Phase3 §6-E).

전방 RGB-D (차체 고정):
  base_link ──(CAD 오프셋)──▶ camera_link
  camera_link ──(REP-103 optical 회전, 고정)──▶ camera_color_optical_frame

손목 RGB (그리퍼 위, 팔에 장착):
  2026-07-31부로 URDF 관절(robot_arm.urdf의 link_036→link_051/052→wrist_camera_link)로
  통합됨 — robot_state_publisher가 팔 자세에 따라 동적으로 발행하므로 여기서 static TF를
  더 이상 내지 않는다(같은 프레임을 두 곳에서 발행하면 TF 트리 충돌). robot_state_publisher가
  뜬 launch(display.launch.py 등)를 같이 켜야 wrist_camera_link가 TF에 나타난다.
"""

import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# REP-103 optical frame: body(X전방·Y좌·Z상) → optical(Z전방·X우·Y하)
OPTICAL_ROLL = -math.pi / 2.0
OPTICAL_PITCH = 0.0
OPTICAL_YAW = -math.pi / 2.0


def generate_launch_description():
    args = [
        # ── 전방 RGB-D 카메라 ──
        # 2026-08-12 실측값 (camera_tf_tuner 로 RViz 에서 **depth 포인트클라우드**를
        # 로봇 모델·바닥 그리드에 맞춰 정렬).
        #   앞으로 49.7cm, 오른쪽 50.4cm, 높이 12.9cm / yaw +161.6°
        # ⚠️ 이번 세션에서 **roll/pitch 는 건드리지 않았다** — 08-09 값을 그대로
        #    물려받았다(드래그로 위치와 yaw 만 맞춤). 카메라를 물리적으로 다시
        #    장착했다면 roll/pitch 도 다시 볼 것.
        # ⚠️ 다점 최소자승은 이번에 돌리지 않았다 — 눈 정렬만으로 확정한 값이라
        #    좌우(y) 정밀도가 특히 검증되지 않았다. 파지 전에 `/pick_target` 방위각
        #    (아래 품질 판정 기준)으로 확인할 것.
        # ⚠️ 이건 **카메라를 책상에 올려둔 벤치 배치**다 — 차체에 정식 장착하면
        #    반드시 다시 재야 한다.
        # 근거: RViz 에서 depth 포인트클라우드와 YOLO 검출 큐브가 실제 상자 위치에
        #    겹치는 것을 눈으로 확인(전 6-DOF 를 한 번에 검증하는 방식). 다점
        #    최소자승(calibrate_camera_pose.py / 튜너의 Solve)은 이 배치에서
        #    오히려 부정확해 채택하지 않았다.
        #
        # ⚠️ **책상 위 카메라는 세션 사이에 밀린다 — 매번 재캘리브할 것.**
        #    2026-08-09 에 08-07 값을 그대로 믿고 픽을 시도했다가, 검출된 박스가
        #    base_link 기준 방위각 -122° / 반경 0.52m 라는 **도달 불가 좌표**로
        #    찍혔다. 박스를 잘못 놓은 게 아니라 그 사이 카메라가 움직인 것이었고
        #    (x 가 +75.5cm, yaw 가 +66° 이동), 재캘리브 후 같은 박스가 방위각
        #    -6.8° / 반경 0.37m 로 정상 복귀했다(YOLO confidence 도 0.81→0.94).
        #    "인식은 되는데 IK 가 이상한 데를 가리킨다" 면 여기부터 의심할 것.
        #
        # 📌 **캘리브 품질 판정 기준(2026-08-09 확립):** 숫자만 보고는 잘 맞췄는지 알 수 없다.
        #    `/pick_target` 을 base_link 로 변환해 **방위각**을 보라 — `arm_joint_1`(베이스
        #    요축)에 모터가 없어 팔이 +x 평면에서만 움직이므로, 방위각 차이가 그대로 파지
        #    오차로 남는다. 실측 대응:
        #      -10.3° → analytic IK 잔차 2.13cm (ik_tol 미수렴, 3cm 로 겨우 수용)
        #       -4.4° → analytic IK 잔차 0.96cm (수렴)
        #    ±5° 안쪽을 목표로 할 것. 그래도 안 줄면 캘리브가 아니라 **박스가 실제로 팔
        #    정면에서 벗어나 있는** 것이므로 박스를 옮겨야 한다(둘을 혼동하지 말 것).
        #
        # 🔧 **2026-08-12 2차 — 그리퍼 대응점 3개로 평행이동만 보정.**
        #    `campose --from-gripper` 로 (optical 관측, 그리퍼 FK 실측) 대응점을 모았는데
        #    쓸 만한 게 3점(A/B/D)뿐이라 회전은 풀지 못했다(비공선 3점 이상 필요, 게다가
        #    수집점이 거의 한 직선 위였다). 대신 **현재 TF 의 잔차가 전 점에서 13~16cm**
        #    로 크고 방향이 일관돼, 그 평균만큼 camera_link 원점을 옮겼다:
        #      Δ(예측−실측) 평균 = (+0.0485, -0.0224, -0.1347) m  → 보정은 부호 반대
        #    ⚠️ 이 보정은 순수 카메라 오차가 아니라 **tip_link(link_043)↔실제 파지점
        #       오프셋까지 함께 흡수**한다 — 실측 '진짜 좌표'가 그리퍼 FK 이기 때문이고,
        #       FSM 이 겨냥하는 것도 같은 tip_link 라 그게 오히려 맞다.
        #    ⚠️ **z(-13cm)와 x(+5cm)는 3점이 일관**했지만 **y 는 -5.7~+1.9cm 로 흩어졌다**
        #       (편차 7.7cm). 손목 각도에 따라 tip↔접촉점 오프셋이 평면 안에서 회전하는
        #       탓으로 보인다 — 평행이동 한 벌로는 y 를 못 잡는다. y 가 계속 빗나가면
        #       이 값이 아니라 **회전까지 푸는 다점 캘리브**가 답이다.
        # 이전 값(2026-08-19 이전): x=0.4483 y=-0.4817 z=0.2634
        #    roll=-0.0619 pitch=0.0404 yaw=2.8198
        # 이전 값(2026-08-12 1차): x=0.4968 y=-0.5041 z=0.1287
        # 이전 값(2026-08-09 2차): x=0.2022 y=-0.5957 z=0.1272
        #    roll=-0.0619 pitch=0.0404 yaw=1.6265
        # 이전 값(2026-08-09 1차): x=0.2022 y=-0.6269 z=0.1402
        # 이전 값(2026-08-07 벤치): x=-0.5526 y=-0.4469 z=0.1654
        #    roll=0.0267 pitch=0.0210 yaw=0.4718
        # 이전 값(차체 장착 CAD 추정, 실기 검증 전): x=0.123 y=0.0 z=0.082
        #    roll=0.0 pitch=-0.26 yaw=0.0 — 정식 장착 시 출발점으로 참고.
        # 이전 값(2026-08-19 1차): x=0.6936 y=-0.4817 z=0.3638
        #    roll=-0.1068 pitch=0.2407 yaw=2.8157
        # 🔁 2026-08-19 2차 재캘리브 — camera_tf_tuner 로 RViz 에서 depth 포인트클라우드가
        #    로봇 모델에 겹치도록 드래그해 맞춘 값(사용자 확정, camera_tf_calib.txt).
        #    앞 -1.8, 좌우 -80.1, 높이 16.5 cm / roll -1.7° pitch +5.4° yaw +86.6°
        # ⚠️ 같은 날 1차 값에서 **yaw 가 161.3° → 86.6° (-74.7°)**, x 가 -71cm,
        #    z 가 -20cm 움직였다. 미세조정 폭이 아니라 **카메라를 물리적으로 다시
        #    놓은 수준**이다 — 책상 위 벤치 배치라 세션 중에도 밀린다는 기존 경고
        #    그대로다. 이 값은 그 배치에 한정되며, 카메라를 건드렸으면 다시 잴 것.
        DeclareLaunchArgument('cam_x',     default_value='-0.0178'),
        DeclareLaunchArgument('cam_y',     default_value='-0.8008'),
        DeclareLaunchArgument('cam_z',     default_value='0.1645'),
        DeclareLaunchArgument('cam_roll',  default_value='-0.0302'),
        DeclareLaunchArgument('cam_pitch', default_value='0.0937'),
        DeclareLaunchArgument('cam_yaw',   default_value='1.5108'),
    ]

    # ── 전방 RGB-D: base_link → camera_link ──
    front_mount_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_camera_link',
        arguments=[
            '--x', LaunchConfiguration('cam_x'),
            '--y', LaunchConfiguration('cam_y'),
            '--z', LaunchConfiguration('cam_z'),
            '--roll', LaunchConfiguration('cam_roll'),
            '--pitch', LaunchConfiguration('cam_pitch'),
            '--yaw', LaunchConfiguration('cam_yaw'),
            '--frame-id', 'base_link',
            '--child-frame-id', 'camera_link',
        ],
    )

    # ── 전방 RGB-D: camera_link → camera_color_optical_frame (REP-103 고정) ──
    front_optical_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_to_optical',
        arguments=[
            '--x', '0.0', '--y', '0.0', '--z', '0.0',
            '--roll',  str(OPTICAL_ROLL),
            '--pitch', str(OPTICAL_PITCH),
            '--yaw',   str(OPTICAL_YAW),
            '--frame-id', 'camera_link',
            '--child-frame-id', 'camera_color_optical_frame',
        ],
    )

    return LaunchDescription(args + [front_mount_tf, front_optical_tf])
