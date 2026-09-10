"""로봇팔 미션 FSM 노드 (Phase 3, 구간2 박스 운반 중심).

설계 문서: `project_docs/PHASE3_FSM_설계.md` §4 상태표 / §5 핸드셰이크.
구현 방식(결정 '가', 2026-06-29): **MoveIt 단일 경로**.
  - 팔 모션: MoveIt `move_action`(MoveGroup)에 목표 pose 전송 → IK·경로계획은
    MoveIt이 수행 → MoveIt이 `arm_controller` FollowJointTrajectory로 실행 →
    upstream `moveit_dynamixel_bridge`가 실제 다이나믹셀 구동.
  - 털털이: `/cleaning/enable`로 start/stop 의도를 bridge에 전달하고, bridge가 별도
    Dynamixel velocity mode actuator를 구동한다. MoveIt planning joint에서는 제외한다.
  - 접촉 피드백: `/joint_states` effort의 cleaning_actuator_joint 값을 사용한다.

  ⚠️ 전제(브릿지 측 선행 작업, PHASE3_FSM_설계.md §6 → 결정 '가'로 이관):
    1) `moveit_dynamixel_bridge`가 `/joint_states`에 **effort(전류)** 를 채워야 함
       (현재는 position만 발행) — 안 그러면 파지/DROP 판정 불가.
    2) 털털이 ID/방향/속도와 contact effort 임계값을 실기에서 확인해야 함.
    3) 카메라 frame → planning frame(base_link) **TF**가 있어야 MoveIt이 목표를 변환.

2026-07-13 파워트레인 "계약 v2"(Notion `자율주행 SW 개발계획` §5.1/5.2)
반영 — status/mode 문자열과 트리거 조건을 아래처럼 갱신:
  - 작업 개시/하역 모두 **`/chassis_mode == MISSION_STOP` AND 같은 mission_id의
    `/arrival_status`** 를 순서 무관하게 둘 다 받아야 전이(`_try_advance`). 미인식
    mode·stale/미래/역행 stamp는 default-deny(상태 변경 없음).
  - `DONE`은 더 이상 완료 권위가 아님 — 픽업 완료는 `CARRYING_LOCKED`, 하역 완료는
    `STOWED_LOCKED`가 최종 권위. `RELEASE` 뒤 `STOWING` 경유해서 `STOWED_LOCKED` 도달.
  - `CARRY` 중 DROP 감지 시 기존 자동 재파지 루프(REGRASP↔PERCEIVE) 대신 `GRIP_LOST`로
    **완전 래치**(자동 재시도 없음) — 새 MISSION_STOP+ArrivalStatus(ARRIVED_PICKUP,
    같은 mission_id 재발행 가능) conjunction이 다시 와야만 PERCEIVE 재진입.
  - `STOW_REQUEST` mode 수신 시 진행 중인 작업을 강제로 RELEASE→STOWING 경로로 유도.
  - mission_id 멱등성: 이미 `STOWED_LOCKED`까지 완료한 mission_id의 ArrivalStatus
    재수신은 무시(중복 재실행 방지).

2026-07-14 자체 결정(우리 쪽에서 결정 가능한 항목은 회의 전에 확정·구현):
  - **`STOW_REQUEST` 범위 확장**: `GRIP_LOST` 전용이었던 것을 `STOW_ABORTABLE_STATES`
    (진행 중인 모든 작업 상태 + `CARRY` + `LOCKED`)로 확장.
  - **`WORK_READY` vs `STOWED_LOCKED` 역할 확정**: `WORK_READY`=MISSION_STOP+ArrivalStatus
    conjunction 수락 순간의 1회성 ack, `STOWED_LOCKED`=그 외 평상시(빈손) 상시 하트비트.
  - **`_is_settled()`**: locked 하트비트(`CARRYING_LOCKED`/`STOWED_LOCKED`) 발행 전 실제
    확인 — TF(`base_frame`←`tip_link`) tip 위치가 `locked_pos_tol` 이내로 안정되고,
    관절각 유한차분 속도가 `locked_vel_tol` 이내인 상태가 `locked_dwell`초 이상
    지속돼야 True. 브릿지가 `/joint_states`에 velocity를 안 실어도 위치 유한차분으로
    자체 계산하므로 문제 없음.

2026-07-15 — origin/main 재합류 + 실제 STOWING 모션 구현:
  - PR #17("파워트레인 DDS 통신 복구 + arm_status 10Hz heartbeat")이 이 파일을 이
    브랜치와 무관하게 독립적으로 다시 손대(계약 v2 상태/게이트 로직 없는 이전 버전 위에
    `contract.py`/`qos_profiles.py` 단일 출처 + heartbeat 전용 타이머·MultiThreadedExecutor
    를 추가) `main`에 먼저 병합됨. 이 세션에서 그 인프라(heartbeat 아키텍처·QoS·contract
    상수 단일 출처) 위에 위 계약 v2 FSM 로직(conjunction 게이트·GRIP_LOST 래치·
    STOW_ABORTABLE_STATES·`_is_settled()`)을 재적용.
  - **LOCK_MODES를 `contract.py` 것으로 통일**(기존엔 이 파일이 로컬로 `DRIVING`을 제외한
    부분집합을 따로 들고 있었음) — `contract.py`(파워트레인 contract.py와 짝, 단일 출처)는
    `DRIVING`도 LOCK_MODES에 포함한다. 즉 PERCEIVE~LIFT 중 `DRIVING` 수신 시에도 이제
    `_enter_locked()`가 걸린다("MISSION_STOP만 허가, 나머지 전부 잠금"을 문자 그대로 적용).
    LOCKED 상태에서 `DRIVING`으로 자동 언락되는 옛 버그(PR #17이 미수정으로 지적)는
    애초에 이 파일에 그런 분기가 없으므로 해당 없음 — `_try_advance()`의
    MISSION_STOP+ArrivalStatus conjunction으로만 탈출.
  - **`STOWING` 실제 접이 모션 구현**(`_run_stow_sequence`) — 이전까지는 스켈레톤이라 현재
    자세 그대로 `_is_settled()`만 확인했음(모션 자체가 없어 접힘 자세 검증이 아니라
    "멈춰있나" 검증에 불과했음). 이제 `stow_joint_positions` 파라미터가 정의하는 목표
    관절각으로 `/arm_controller/joint_trajectory`에 직접 궤적을 발행 → 완료 후 `_is_settled()`
    게이트를 거쳐 `STOWED_LOCKED`.
    `stow_joint_positions` 기본값은 2026-07-29 팀 결정으로 **all-zero**다(주행 안정성 —
    도달 가능한 자세 중 CG가 가장 낮음). 아래 파라미터 선언부의 근거 주석 참고.
    ⚠️ **파워트레인 문서 §6의 all-zero home 금지와 정면 충돌한다 — 양 팀 합의 전까지
    실차 연동 금지.** 또한 URDF 상으로만 검증됐고 실기 검증은 아직이다.

2026-07-15 — 파워트레인 §5.1 잔여 합의 2건 해결(`project_docs/파워트레인_계약_충돌점검.md`
항목 1·2 대응):
  - **`_near_stow_posture()` 추가** — `LOCKED`(지형/주행 이벤트로 작업 중단) 경유로 도달한
    임의 자세를 예전엔 정지만 확인되면 바로 `STOWED_LOCKED`로 근사했음. 이제 관절각이
    `stow_joint_positions` 근처(`stow_pos_tol_rad`)인지 확인한 경우에만 `STOWED_LOCKED`를
    발행하고, 아니면 `EXECUTING`을 유지해 파워트레인 쪽 motion hold를 받는다(거짓 주행
    허가 방지).
  - **`LOWER_RELEASE` 상태 신설** — `PAYLOAD_ALOFT_STATES`(`LIFT`/`CARRY`, 화물을 든 채
    공중일 수 있는 상태)에서 `STOW_REQUEST`로 중단되면, 예전엔 바로 `RELEASE`(그리퍼
    오픈)로 갔는데 이는 화물이 공중에서 그대로 낙하하는 경로였음. 이제 `RELEASE` 전에
    `lift_height`만큼 먼저 내려(`_lower_pose`/`_lower_target_xyz`, `_carry_pose`/
    `_lift_target_xyz`의 역) grasp 당시 높이 근처로 되돌아간 뒤에만 그리퍼를 연다.
  - **controller fault 게이트 추가** — `moveit_dynamixel_bridge`가 Hardware Error
    Status(주소 70)를 기존 current/position SyncRead 블록에 합쳐 읽어
    `/dynamixel/controller_fault`(Bool, 내부용 — DDS 경계 안 넘음)로 발행하도록
    확장. `_is_settled()`가 이 값이 True(등록된 서보 중 하나라도 에러 또는 무응답)
    이면 즉시 미확인 처리 — 이전엔 이 필드 자체가 없어 검사 불가였던 항목(§5.1
    잔여 항목 3).

2026-08-19 — 실기 픽 테스트 피드백 3건 반영:
  - **DESCEND→GRASP 게이트를 실측 정지 확인으로 강화**: 기존엔 모션 완료 추정 시각
    (`duration+0.5s`)만 보고 바로 그리퍼를 닫았는데, 실기에서 하중/마찰로 이동이
    추정보다 길어지면 팔이 아직 내려가는 중에 닫힘 명령이 나갈 수 있었다. `_is_settled()`
    (LOCKED 하트비트에도 쓰는 tip TF + 관절속도 유한차분 실측)를 추가 게이트로 걸어
    **팔이 완전히 멈춘 뒤에만** GRASP 로 넘어가도록 `_do_descend()` 수정.
  - **그리퍼 전류 스파이크 조기 감지**: `_do_grasp()`가 이제 닫는 도중 매 tick effort 를
    감시한다 — 완전닫힘 전에 전류가 `grasp_effort_thresh` 를 넘으면(=물체에 걸려 막힘)
    남은 시간(`gripper_action_time`)을 기다리지 않고 그 순간 파지 성공으로 간주해 바로
    LIFT 로 전이한다. `_do_grasp_check()`가 이미 문서화한 함정(완전닫힘 근처 전류 상승은
    기구적 끝단을 미는 것이지 파지가 아님)을 피하려고, 위치가 아직 완전닫힘 근처가
    아닐 때만 스파이크를 성공 신호로 인정한다. 스파이크가 끝내 없으면 기존 동작(시간
    경과 후 GRASP_CHECK 에서 위치+전류 재확인)으로 폴백 — 빈손 판정 로직은 그대로.
  - **`carry_home` 기본값 false→true**: 파지 성공 후 물건을 문 채로 접힘(home) 자세로
    돌아가 CARRY 에 들어간다(그리퍼는 건드리지 않아 계속 물고 있음). 필요하면
    `carry_home:=false` 로 되돌려 예전 동작(들어올린 자리에서 대기)을 쓸 수 있다.

2026-08-19(2차) — "옆에서 대각선으로 문다 / 파지 순간 arm_joint_4 가 혼자 들린다" 두
증상의 원인을 잡음. 둘 다 뿌리가 같다 — **analytic IK 에 자유도가 남아돌았다**:
  - **손목 자세 잠금 신설**(모듈 상단 `WRIST_PITCH_COEFFS` 블록에 기하 유도 전문).
    관절 5개로 위치 3개만 풀어 2자유도가 남았고, 남는 자유도를 댐핑 최소자승이
    "가장 적게 움직이는 해"로 아무렇게나 채우는 바람에 손목이 매번 다른 각도로 잡혔다.
    URDF 실측상 j2(+X)·j3(-X)·j4(-X)가 전부 평행이라 그리퍼가 향하는 방향은
    `j2-j3-j4` 합 하나로만 정해지므로, 이 식을 j4 에 대해 풀어 **종속 관절**로 만들고
    j5(접근축 둘레 롤)는 0 고정 → 자유변수가 j1·j2·j3 셋으로 줄어 위치 3개와 정확히
    맞아떨어진다(해가 유일 = 매번 같은 자세). `tool_pitch=pi/2` 가 수직 아래(top-down).
  - **IK 반복에 관절 리밋 clamp 추가**(`_clamp_arm_joints`). 예전엔 IK 가 리밋 밖으로
    자유롭게 반복하고 브릿지 `rad_to_tick` 이 마지막에 clamp 해서 **IK 가 푼 자세와 팔이
    실제로 가는 자세가 달랐다.** URDF FK 재현: `arm_joint_2` 가 리밋 [0, 1.4276] 밖
    -1.968 로 수렴 → 손목 종속각이 리밋에 걸려 접근축이 수직에서 최대 88° 기울었다.
    reproduce 후 clamp 를 반복 안에 넣으니 접근축이 정확히 90.00°(수직)로 유지된다.
  ⚠️ **top-down 잠금 시 도달 가능 범위가 좁다**(URDF 전수 스캔, tip_link 기준):
     x [-0.052, +0.101] / y [-0.310, -0.030] / z [-0.022, +0.191] m.
     팔은 base_link **-Y 방향으로만** 뻗는다 — `arm_joint_1` 이 ±14.3° 뿐이라 방위
     회전이 사실상 불가능하다. 목표가 이 밖이면 이제 IK 가 정직하게 실패를 보고한다
     (예전엔 조용히 기울어진 자세로 갔다).

상태 흐름: STOWED_LOCKED → PERCEIVE → PLAN → APPROACH → DESCEND → GRASP
  → GRASP_CHECK → LIFT → CARRY → (ARRIVED_DROP) RELEASE → DONE → STOWING
  → STOWED_LOCKED(빈손 대기)
  계획/접근/하강/리프트 또는 파지 판정 실패 → FAILED(래치)
  CARRY 중 DROP 감지 → GRIP_LOST(래치) → (재발행 conjunction) PERCEIVE
  PERCEIVE~LIFT 중 지형/주행 이벤트 → LOCKED(래치, 하트비트 유지) → (재발행 conjunction) PERCEIVE
  LIFT/CARRY(또는 LIFT 중 LOCKED) 중 STOW_REQUEST → LOWER_RELEASE → RELEASE → STOWING
  그 외 상태 중 STOW_REQUEST → RELEASE → STOWING (이미 grasp 높이 근처라 낙하 낙차 없음)
  LOCKED/GRIP_LOST 모두: 진행 중 모션 취소 + 현재 자세 홀드, MISSION_STOP conjunction으로만 탈출.

⚠️ 스켈레톤: MoveGroup pose goal / 그리퍼 액션 / effort 판정 / FSM 골격은 구현.
   LIFT·CARRY 목표 pose, 임계값 캘리브, TF 연결은 구현됨. STOWING 모션은 위 참고
   (목표 관절각 실측 필요). CARRYING_LOCKED/STOWED_LOCKED 발행 전 controller fault
   확인은 `moveit_dynamixel_bridge`의 `/dynamixel/controller_fault`(Hardware Error
   Status 집계)를 `_is_settled()`에서 게이트로 사용해 구현 완료(2026-07-15).
"""
import math
from copy import deepcopy
from enum import Enum, auto
import random

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy
from rclpy.time import Time
from rclpy.duration import Duration as RclpyDuration
from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose

from builtin_interfaces.msg import Duration
from std_msgs.msg import Bool, String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from moveit_msgs.action import MoveGroup
from control_msgs.action import FollowJointTrajectory
from moveit_msgs.msg import (MotionPlanRequest, Constraints, PositionConstraint,
                             OrientationConstraint, BoundingVolume, RobotState)
from moveit_msgs.srv import GetPositionFK
from shape_msgs.msg import SolidPrimitive
from robot_arm_msgs.msg import ArrivalStatus, ChassisMode, ArmStatus, DetectedObject
from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset, trip_seconds_for
# 관절 이름·순서의 단일 출처(ARM_JOINT_NAMES 주석 참고). 이 import 는 상수만 읽으며
# 서보 포트를 열지 않는다 — 포트는 브릿지 노드 인스턴스가 생성될 때만 열린다.
from dynamixel_control.moveit_dynamixel_bridge import JOINT_CONFIG
from dynamixel_control import joint_limits


# 2026-07-15 Isaac Sim 기반 재export(robotarm_urdf_20260711.urdf) 기준 — URDF 자체는
# 팔 5축(arm_joint_1~5)을 전부 반영하지만, analytic IK(FK+수치 자코비안)는 아직 앞의
# 3관절만 풀도록 남겨둠(HW-7 당시 6DOF pose goal이 NO_IK_SOLUTION이던 문제 회피용으로
# 도입된 3DOF 위치전용 IK — URDF가 3축만 있어서가 아니라 solver를 아직 5DOF로 확장 안
# 해서임, 방향은 여전히 무시). MoveGroup 경로(§6 결정 '가')는 남겨두되 ik_mode:='moveit'로
# 전환 가능하게만 유지.
# 팔 관절 이름·순서는 **moveit_dynamixel_bridge.JOINT_CONFIG 가 단일 출처**다.
# 여기에 리터럴로 다시 적지 말 것 — 2026-08-07 이전에는 `['arm_joint_1','arm_joint_2',
# 'arm_joint_3']` 로 하드코딩돼 있었고, 브릿지가 실기 버스(arm_joint_2~5)에 맞춰
# 갱신된 뒤에도 그대로 남아 **모터가 없는 arm_joint_1 을 명령하고 실제로 달려 있는
# arm_joint_4/5 는 건드리지도 않는** 상태였다. dict 는 삽입 순서를 보존하므로
# 궤적 메시지의 관절 순서도 브릿지와 자동으로 일치한다.
ARM_JOINT_NAMES = list(JOINT_CONFIG)


# ──────────────────────────────────────────────
# 손목 자세 잠금 — "위에서 내려다보는 파지"(top-down)를 강제한다 (2026-08-19 신설).
#
# ## 왜 필요한가
#
# analytic IK 는 **위치 3개만** 맞추고 방향을 통째로 버린다. 그런데 팔은 관절이
# 5개라 **2자유도가 남아돈다** — 남는 자유도를 댐핑 최소자승이 "현재 자세에서 가장
# 적게 움직이는 해"로 아무렇게나 채우기 때문에, 같은 목표 좌표라도 손목이 매번 다른
# 각도로 잡힌다. 실기 증상이 정확히 이것이었다(2026-08-19):
#   - 상자를 **옆에서 대각선으로** 물려 든다 (접근축이 수직이 아님)
#   - APPROACH → DESCEND 사이에 **arm_joint_4 가 혼자 들려서** 손가락이 상자를 빗나감
#     (두 목표는 x·y 가 같고 z 만 다른데, 해가 재분배되며 손목만 따로 움직인다)
#
# ## 기하 (urdf/robot_arm.urdf 실측, 2026-08-19)
#
# `arm_joint_2` 축 `+X` / `arm_joint_3` 축 `-X` / `arm_joint_4` 축 `-X` — **셋이 전부
# 평행한 피치축**이다. 따라서 그리퍼가 향하는 방향은 세 각도의 **부호합 하나로만**
# 정해진다(개별 값과는 무관하다 — FK 로 교차검증함):
#
#       tool_pitch = j2 - j3 - j4        [rad, base_link +X 축 둘레 회전량]
#
# 그리퍼 접근축(손가락이 뻗어나가는 방향)은 `link_043` 로컬 `-Y` 이고, all-zero 에서
# base_link `-Y`(수평)를 본다. `+X` 둘레로 `+pi/2` 돌리면 base_link `-Z`(수직 아래) —
# 즉 **`j2 - j3 - j4 = pi/2` 가 top-down 파지**다.
# 리밋상 도달 범위는 [-4.35, +2.67] 이라 pi/2 는 여유 있게 들어간다.
#
# 손가락은 로컬 `X` 축을 따라 닫힌다(랙 조인트 축이 ±X). `arm_joint_5`(축 `-Y`)는
# 접근축 **자신을 축으로 한 롤**이라 향하는 방향은 안 바꾸고 손가락 닫히는 방향만
# 돌린다 — 그래서 위치 IK 에는 기여가 거의 없으면서 해만 흔든다. 0 으로 고정한다.
# (CLAUDE.md 의 손목카메라 파지 검증도 `arm_joint_5=0` 고정을 전제로 캘리브돼 있다 —
#  롤을 안 고정하면 fill 지표가 2.5배 흔들리고 검출률이 53%까지 떨어진다.)
#
# ## 어떻게 거는가
#
# 위 식을 `arm_joint_4` 에 대해 풀어 **종속 관절**로 만든다(`_apply_wrist_lock`):
#
#       j4 = j2 - j3 - tool_pitch
#
# 그러면 IK 의 자유변수는 j1·j2·j3 셋만 남아 **위치 3개와 정확히 맞아떨어진다**
# (남는 자유도 0 = 해가 유일 → 매번 같은 자세). 제약을 반복 **안쪽**에서 걸기 때문에
# (자코비안도 자유변수에 대해서만 잡는다) 수렴한 해는 위치와 손목 자세를 동시에
# 만족한다 — 풀고 나서 j4 를 덮어쓰는 방식이면 그만큼 위치가 어긋난다.
WRIST_PITCH_COEFFS = {'arm_joint_2': +1.0, 'arm_joint_3': -1.0, 'arm_joint_4': -1.0}
WRIST_PITCH_DEPENDENT = 'arm_joint_4'   # 위 식을 풀어 종속시킬 관절
WRIST_ROLL_JOINT = 'arm_joint_5'        # 접근축 둘레 롤 — 고정
TOOL_PITCH_DOWN = math.pi / 2           # 접근축이 수직 아래(base -Z)를 보는 값


# ──────────────────────────────────────────────
# status / mode 문자열 — 단일 출처는 contract.py (파워트레인 contract.py 와 짝).
# 여기서 상수를 새로 정의하지 말 것. 어휘 변경은 양 팀 합의 사항이다.
# ──────────────────────────────────────────────
from dynamixel_control.contract import (       # noqa: E402
    ARRIVED_PICKUP, ARRIVED_DROP,
    ARM_PERCEIVING, ARM_PLANNING, ARM_EXECUTING, ARM_DONE, ARM_FAILED,
    ARM_WORK_READY, ARM_STOWING, ARM_STOWED_LOCKED, ARM_CARRYING_LOCKED, ARM_GRIP_LOST,
    LOCK_MODES, MODE_MISSION_STOP, MODE_STOW_REQUEST, HEARTBEAT_RATE_HZ,
)
from dynamixel_control.qos_profiles import HEARTBEAT_QOS, ARRIVAL_QOS   # noqa: E402
from dynamixel_control.sensor_manager import SensorManager              # noqa: E402
from robot_arm_msgs.msg import TaskCommand, TaskResult
from dynamixel_control.tool_manager import (                            # noqa: E402
    ParameterToolIdentityProvider, ToolManager)
from dynamixel_control.tool_profiles import (                           # noqa: E402
    load_profiles, ToolProfileError)
from ament_index_python.packages import get_package_share_directory     # noqa: E402
from pathlib import Path                                                # noqa: E402
import json                                                             # noqa: E402

# contract.py의 LOCK_MODES는 DRIVING을 포함한다("MISSION_STOP만 허가, 나머지 전부 잠금").
RECOGNIZED_MODES = LOCK_MODES | {MODE_MISSION_STOP, MODE_STOW_REQUEST}

# stamp freshness — 미래/역행 판정 허용오차 [s] (계약 §5.1 age 0~0.5s 기준)
STAMP_FUTURE_TOL = 0.5

# moveit_msgs/MoveItErrorCodes.SUCCESS
MOVEIT_SUCCESS = 1
MISSION_PICK_PLACE = "PICK_PLACE"
MISSION_ROTARY_TOOL = "ROTARY_TOOL"


class State(Enum):
    IDLE = auto()
    PERCEIVE = auto()
    PLAN = auto()
    APPROACH = auto()
    # Pick-place flow compatibility states.  The live handlers still use these
    # names, and external tests/clients observe the enum as part of the FSM API.
    DESCEND = auto()
    DESCEND_STOPPED = auto()
    TOOL_ACTION = auto()
    GRASP = auto()
    GRASP_CHECK = auto()
    LIFT = auto()
    CLEAN_START = auto()
    CONTACT_CHECK = auto()
    CLEAN = auto()
    CLEAN_STOP = auto()
    RETRACT = auto()
    LOCK_CHECK = auto()
    CARRY = auto()
    RELEASE = auto()
    ARM_TEST_MOVE = auto()
    RANDOM_ARM_DEMO = auto()
    END_EFFECTOR_ROTATE = auto()
    DONE = auto()
    FAILED = auto()
    # 기존 파워트레인 계약에서 유지하는 감독/안전 상태.
    GRIP_LOST = auto()
    LOWER_RELEASE = auto()
    STOWING = auto()
    STOWED_LOCKED = auto()
    LOCKED = auto()


# 지형/주행 이벤트(LOCK_MODES)로 preempt 대상이 되는 상태 — 실제 모션/그리퍼 동작이 진행
# 중일 수 있는 상태만. CARRY는 이미 정지-유지 상태(그리퍼 effort 감시만)라 preempt
# 불필요 — 계약 v2 하트비트를 CARRY 자체 루프가 계속 발행해야 하므로 굳이 LOCKED로 빼지 않음.
PREEMPTIBLE_STATES = (
    State.PERCEIVE, State.PLAN, State.APPROACH, State.TOOL_ACTION, State.CLEAN_START,
    State.CONTACT_CHECK, State.CLEAN, State.CLEAN_STOP, State.RETRACT,
    State.LOCK_CHECK,
)

# STOW_REQUEST(운영자 포기·재정렬 유도)로 즉시 RELEASE(or LOWER_RELEASE)→STOWING 강제
# 진입 가능한 상태 — 작업이 진행 중이거나 래치된 모든 상태(2026-07-14 결정: GRIP_LOST
# 전용이었던 것을 확장). IDLE/LOWER_RELEASE/RELEASE/STOWING/STOWED_LOCKED는 이미
# 정지/포기 진행 중이라 대상에서 제외.
STOW_ABORTABLE_STATES = PREEMPTIBLE_STATES + (
    State.CARRY, State.FAILED, State.GRIP_LOST, State.LOCKED,
)

# STOW_REQUEST로 중단될 때 화물을 든 채 공중에 있을 수 있는 상태 — 그리퍼를 바로 열면
# 낙하 위험(파워트레인 §5.1 잔여 합의 ①). 이 상태들에서만 RELEASE 전에 파지 높이까지
# 먼저 내리는 LOWER_RELEASE를 경유한다. 그 외(PERCEIVE/PLAN/DESCEND/청소 상태)는 이미
# grasp 높이 근처라 낙하 낙차가 없어 바로 RELEASE해도 안전.
PAYLOAD_ALOFT_STATES = (State.RETRACT, State.CARRY)


class ArmFsmNode(Node):
    def __init__(self):
        super().__init__('arm_fsm_node')

        # ── 파라미터 ──────────────────────────────
        # 형상과 tip_link를 포함한 모든 엔드이펙터 기본값을 같은 preset에서 고른다.
        self.declare_parameter('end_effector_preset', DEFAULT_GRIPPER)
        self.declare_parameter('cleaning_actuator_joint', '')
        self.cleaning_actuator_joint = self.get_parameter('cleaning_actuator_joint').value
        self.declare_parameter('dry_run_mode', False)
        self.declare_parameter('vla_standalone_mode', False)
        self.dry_run_mode = bool(self.get_parameter('dry_run_mode').value)
        self.vla_standalone_mode = bool(self.get_parameter('vla_standalone_mode').value)
        self.declare_parameter('vla_command_topic', '/vla/task_command')
        self.declare_parameter('vla_result_topic', '/vla/task_result')
        self.declare_parameter('tool_type', 'dual_motor_gripper')
        self.declare_parameter('tool_profile_file', str(Path(
            get_package_share_directory('dynamixel_control')) / 'config/tool_profiles.yaml'))
        gripper_type = self.get_parameter('end_effector_preset').value
        gpreset = get_preset(gripper_type, self.get_logger())

        # MoveIt
        self.declare_parameter('planning_group', 'arm')          # SRDF group
        # tip_link: arm_joint_5 이후 고정 조인트 체인의 마지막 링크(link_051).
        # 털털이 cleaner_base_link는 이 링크 아래 wrist_to_cleaner fixed joint로 부착된다.
        # 2026-07-15 Isaac Sim 재export(robotarm_urdf_20260711.urdf) 기준.
        self.declare_parameter('tip_link', 'link_051')            # 그리퍼 부모 링크
        self.declare_parameter('base_frame', 'base_link')        # planning frame (리프트 기준)
        self.declare_parameter('lift_height', 0.10)              # LIFT 시 base_link +Z [m]
        self.declare_parameter('approach_height', 0.08)          # target 위 접근 오프셋 [m]
        self.declare_parameter('pick_frame_id', 'camera_color_optical_frame')
        self.declare_parameter('pos_tolerance', 0.01)            # [m]
        self.declare_parameter('orient_tolerance', 0.1)          # [rad]
        self.declare_parameter('planning_time', 5.0)
        self.declare_parameter('vel_scale', 0.1)                 # 저속(파지 안전)
        self.declare_parameter('acc_scale', 0.1)
        # 'analytic'(기본, URDF 3관절 한정 수치 IK) | 'moveit'(URDF 5축 완성 후 전환)
        self.declare_parameter('ik_mode', 'analytic')
        self.declare_parameter('ik_max_iters', 8)
        self.declare_parameter('ik_tol', 0.01)          # [m] 위치 수렴 허용오차
        # [m] 최종 실패 판정 기준 — 이 안이면 "덜 수렴했지만 그 자리에서 집어본다".
        #
        # 2026-08-19 0.03 → 0.05 로 완화(사용자 지시: "상자에 거의 위치하면 왠만하면
        # 거기에서 집어라 — 그리퍼가 커서 집힌다"). 실기에서 3.4cm 부족으로 IK 가
        # 실패하면 FAILED 가 **래치**돼 미션이 통째로 중단됐는데(자동 재시도 없음,
        # pick 을 다시 눌러야 한다), 그 정도 오차는 그리퍼가 흡수한다.
        #
        # 0.05 의 근거는 **대상 상자 반폭**이다 — 95mm 큐브라 중심에서 47.5mm 를 넘게
        # 빗나가면 그리퍼가 상자 **바깥**에서 닫혀 아예 못 문다. 즉 이 값을 더 키우면
        # "집으려 시도했지만 허공을 쥔다" 가 늘 뿐이다. 대상 크기가 바뀌면 같이 바꿀 것
        # (손목카메라 캘리브의 box_size_m=0.095 와 같은 대상이다).
        self.declare_parameter('ik_accept_tol', 0.05)
        # 손목 자세 잠금 (위 WRIST_PITCH_COEFFS 블록의 근거 참고).
        # false 로 두면 2026-08-19 이전 동작(손목 자유 = 대각선 파지)으로 돌아간다.
        self.declare_parameter('lock_tool_pitch', True)
        # 접근축 기울기 [rad]. pi/2 = 수직 아래(top-down). 실기에서 상자를 살짝
        # 비스듬히 물어야 하면 여기서 몇 도만 빼면 된다(예: 1.40 ≈ 아래에서 10° 젖힘).
        self.declare_parameter('tool_pitch', TOOL_PITCH_DOWN)
        # 손목 롤 [rad]. 0 = 손가락이 base X 방향으로 닫힘.
        self.declare_parameter('wrist_roll', 0.0)
        # 재시도(FAILED→PERCEIVE→PLAN) 때 타겟을 다시 인식하지 않고 처음 얼린 값을 재사용한다.
        #
        # ⚠️ 왜 필요한가 (2026-08-09 실기): APPROACH/DESCEND 로 팔이 움직이면 **팔 자신이
        #    카메라 시야에 들어온다.** 그 상태에서 재시도가 PERCEIVE 를 다시 돌면 YOLO 가
        #    팔/그리퍼를 박스로 잡거나 진짜 박스가 가려져서 타겟이 통째로 튄다 — 실측으로
        #    base_link (0.348,-0.053,0.056) → (0.603,0.481,0.067) 로 53cm 점프했고, 그
        #    뒤로는 IK 실패만 반복하는 루프에 빠졌다.
        #
        # 기본값 False(=매번 다시 인식)는 기존 동작이다. 현장에서는 박스가 실제로 움직였을
        # 수 있어 다시 보는 게 맞을 때도 있기 때문에 옵트인으로 둔다.
        #
        # 🔧 근본 해결은 따로다: 재인식 전에 팔을 시야 밖(관측 자세)으로 물린 뒤 보는 것.
        #    그게 들어가기 전까지는 이 플래그가 실용적인 우회다.
        self.declare_parameter('freeze_target_on_retry', False)
        self.declare_parameter('gripper_change_mode', False)
        self.declare_parameter('gripper_disabled', False)
        self.declare_parameter(
            'gripper_command_calibrated', gpreset.get('command_calibrated', False))
        self.declare_parameter('stop_after_descend', False)
        self.declare_parameter('arm_move_speed', 0.5)   # [rad/s] 직접명령 시 소요시간 추정용
        # 그리퍼 — gripper_type 이 gripper_presets.GRIPPER_PRESETS 의 기본값을 고르고,
        # 아래 개별 파라미터는 필요 시 CLI/런치로 여전히 개별 오버라이드 가능.
        self.declare_parameter('gripper_type', DEFAULT_GRIPPER)
        gripper_type = self.get_parameter('gripper_type').value
        gpreset = get_preset(gripper_type, self.get_logger())

        self.declare_parameter('gripper_joints', gpreset['gripper_joints'])
        self.declare_parameter('gripper_open', gpreset['gripper_open_rad'])
        self.declare_parameter('gripper_close', gpreset['gripper_close_rad'])
        # 전류(effort) 임계 — moveit_dynamixel_bridge 가 /joint_states.effort 에
        # raw signed PRESENT_CURRENT(XL430 기준 1단위≈2.69mA)를 발행. preset 값은 placeholder,
        # 실측 캘리브 필요(TODO): 무부하 파지 전류/낙하 시 전류를 측정해 임계값 설정.
        # 파지 시 완전닫힘보다 더 깊이 밀 양 [rad]. 0 이면 예전 동작(살짝 쥠).
        # 힘의 상한은 gripper_goal_pwm(280)이 잡으므로 Overload 트립은 안 난다
        # (PWM 280 은 40초+ 유지 무트립 실측). 세게 쥐려면 이 값을 올린다.
        self.declare_parameter('gripper_squeeze_rad', 0.25)
        # 빈손 판정 — 그리퍼가 완전닫힘에서 이 이내면 손가락 사이가 비어 있다.
        # 얇은 물체를 쥐면 이 값보다 작게 벌어질 수 있으니 그때는 줄일 것.
        self.declare_parameter('gripper_empty_pos_tol', 0.06)
        self.declare_parameter('grasp_effort_thresh', gpreset['grasp_effort_thresh'])
        self.declare_parameter('drop_effort_thresh', gpreset['drop_effort_thresh'])
        # 동작 제어
        # max_regrasp(origin/main의 옛 자동 재파지 루프 파라미터)는 도입 안 함 — 계약 v2는
        # CARRY 중 DROP 감지 시 REGRASP↔PERCEIVE 루프 대신 GRIP_LOST 완전 래치로 대체함
        # (자동 재시도 없음, 새 MISSION_STOP+ArrivalStatus conjunction으로만 재개).
        self.declare_parameter('gripper_action_time', gpreset['gripper_action_time'])  # [s]
        # 파지 유지 시간 경고 [s]. XL430 은 전류 제어가 없어 파지력이 Goal PWM 으로만
        # 정해지는데, Overload 는 부하를 **시간에 누적**해 판정한다 — 힘을 올리면
        # 무한정 버티던 게 유한 시간 트립으로 바뀐다(gripper_presets 실측 스윕:
        # PWM 280→40초+ 무트립, PWM 400→17초, PWM 885→3.5초).
        # 트립하면 토크가 끊겨 **화물을 떨어뜨리고 REBOOT 전까지 무응답**이다.
        # 2026-08-19 PWM 을 400 으로 올렸으므로(사용자 지시) 17초가 한계인데, 실측
        # 미션에서 파지~해제가 23초 걸렸다 — 대부분 CARRY 에서 운영자의 drop/stow
        # 입력을 기다린 시간이라 코드로는 줄일 수 없다. 그래서 **경고로 알린다.**
        # PWM 을 되돌리면(280) 이 경고도 같이 꺼도 된다(0 이하면 비활성).
        self.declare_parameter('grip_hold_warn_s', 12.0)
        # 파지 유지 중 닫힘 명령을 재발행하는 주기 [s]. 0 이하면 끔(예전 동작 = 1회 발행).
        # 근거는 `_maintain_grip()` docstring 참고.
        self.declare_parameter('grip_refresh_s', 1.0)
        self.declare_parameter('tick_rate', 10.0)
        # chassis_mode 수신 끊김 워치독 — §5.1 "수신 끊김 = default-deny(잠금 유지)"
        self.declare_parameter('chassis_mode_timeout', 1.0)      # [s]
        # locked 하트비트(CARRYING_LOCKED/STOWED_LOCKED) 발행 전 실제 확인 조건 — §5.1
        # "문자열만 바꾸는 게 아니라 자세 오차·관절 속도·유지 시간을 확인한 뒤 발행".
        # 수치는 실측 캘리브 전 placeholder(TODO). 컨트롤러 fault 확인은 브릿지에 아직
        # 해당 필드가 없어 미포함(별도 후속 작업).
        self.declare_parameter('locked_pos_tol', 0.005)   # [m] tip 위치 흔들림 허용치
        self.declare_parameter('locked_vel_tol', 0.05)    # [rad/s] 관절 속도(유한차분) 허용치
        self.declare_parameter('locked_dwell', 0.5)       # [s] 안정 유지 시간
        # STOWING 목표 관절각(ARM_JOINT_NAMES 순서 — 2026-08-19 기준 5축:
        # arm_joint_1..5. ID 11 서보 실장으로 4축에서 5축이 됐다).
        # **팀이 주행 안정성 기준으로 all-zero 를 접힘 자세로 확정**(사용자 지시, 2026-07-29).
        # 2026-07-29 랙피니언 그리퍼 URDF 실측 지표:
        #   점유 bbox x 224mm × y 606mm   높이 285mm   최저점 +50mm
        #   CG 높이 160mm (도달 가능한 자세 중 최저)   j2+j3 중력토크 13.13N·m   자기충돌 0
        # 근거: j2·j3 하한이 0이라 팔은 [수평 → 수직 → 반대쪽 수평]만 훑고 아래로는 못 내려간다.
        # 그래서 도달 가능한 자세 중 CG 가 가장 낮은 것이 all-zero 이고, 경사·요철에서 전복
        # 여유가 가장 크다. (대안이던 '수직 마스트' [0, 1.40, 2.85] 는 bbox 213×235mm 로
        # 발자국은 훨씬 작고 중력토크도 0.79N·m 로 20배 낮지만, 높이 978mm·CG 400mm 라
        # 전복 여유가 나쁘다. 차체가 짧아 y 606mm 가 안 들어가면 그쪽으로 되돌릴 것.)
        #
        # ⚠️ **파워트레인 계약 위반 상태다 — 양 팀 합의 전까지 실차 연동 금지.**
        # 파워트레인 문서 §6 "all-zero home 과 direct dynamixel goal publisher 는 production
        # 에서 금지한다" (project_docs/파워트레인_계약_충돌점검.md:110).
        # 실질 위험: `_near_stow_posture()` 가 무력화된다 — 전원만 들어오고 초기화 안 된 팔도
        # 관절각이 ~0 이라 이 검사를 그냥 통과해서, 실제로 접히지 않았는데 STOWED_LOCKED 를
        # 발행 → 파워트레인이 주행 허가로 받는다. 금지 조항의 이유가 정확히 이것이다.
        # 회피책: 물리적으로 거의 같으면서 0 과 구분되는 값(예: [0.0, 0.15, 0.15] — bbox
        # 224×606mm 로 all-zero 와 동일, 높이 328mm, 토크 12.61N·m)을 쓰면 stow_pos_tol_rad
        # (0.1) 밖이라 위 검사가 되살아난다. 주행 안정성은 사실상 그대로다.
        #
        # 이전 기본값 [0.0, -0.6, 1.2]은 j2=-0.6이 URDF 하한(0) 밖이라 애초에 도달 불가였다.
        # ⚠️ URDF 검증만 끝났고 실기 검증은 아직 — 실물 구동 전 서보 tick 대응 확인 필요.
        # 길이는 ARM_JOINT_NAMES 를 따라간다 — 리터럴로 박아두면 축 수가 바뀔 때
        # 길이 검증(아래)에 걸려 STOWING 모션이 **조용히 비활성**된다.
        # 파지 후 **물건을 문 채** 접힘(=URDF home) 자세로 돌아갈지. true 면 LIFT 완료 후
        # stow_joint_positions 로 접고 나서 CARRY 로 들어간다(그리퍼는 건드리지 않으므로
        # 계속 물고 있음) — 화물을 든 채 주행하려면 팔이 펴져 있는 것보다 접혀 있는 쪽이
        # 안전하다는 판단. 2026-08-19: 실기 픽 테스트 요청으로 기본값을 true 로 전환
        # (예전 기본 false 는 들어올린 자리에서 CARRY 대기 — 필요하면 CLI/launch 에서
        # carry_home:=false 로 되돌릴 것).
        # ⚠️ 접는 경로에 충돌 검사가 없다(_run_stow_sequence 는 known-safe 가정으로
        #    직접 궤적을 쏜다). 화물이 큰 경우 차체·자기 링크와 부딪히는지 눈으로
        #    확인하고 켤 것.
        self.declare_parameter('carry_home', True)
        self.declare_parameter('stow_joint_positions', [0.0] * len(ARM_JOINT_NAMES))
        # 접힘(home 복귀)을 **단계로 나눠** 이 관절들을 먼저 보낸다. 나머지 축은 그동안
        # 현재 자세를 유지하고, 선행축이 도착한 뒤에 함께 접힌다.
        #
        # 왜 (2026-08-19 사용자 지시): 전 축을 동시에 접으면 팔이 뻗은 채로 호를 그리며
        # 돌아와 화물이 차체·주변에 쓸린다. 어깨축(arm_joint_2)을 먼저 세워 팔을 몸쪽으로
        # 당긴 뒤 나머지를 접으면 훨씬 안정적이다.
        #
        # ⚠️ **다점 궤적 하나로는 이걸 못 만든다.** 브릿지의 `/arm_controller/joint_trajectory`
        # 구독부는 `msg.points[-1]` **만** 읽고 중간 점을 전부 버린다
        # (moveit_dynamixel_bridge.py 의 `point = msg.points[-1]`). 그래서 단계마다
        # **별도의 궤적을 시간차로 발행**한다(`_run_stow_sequence`).
        #
        # 빈 리스트면 단계 없이 예전처럼 전 축 동시 접힘.
        self.declare_parameter('stow_lead_joints', ['arm_joint_2'])
        # STOWED_LOCKED 발행 전 "실제로 접힌 자세인지" 확인용 관절각 허용오차 — §5.1 잔여
        # 합의 ②(정지 안정성만 검사하고 접힘 자세 근접은 미확인) 대응. LOCKED 경유(지형/주행
        # 이벤트로 작업 중단)로 도달한 임의 자세를 STOWED_LOCKED로 착칭하지 않기 위함.
        self.declare_parameter('stow_pos_tol_rad', 0.1)   # [rad] ≈5.7도, placeholder

        g = self.get_parameter
        self.planning_group = g('planning_group').value
        self.tip_link = g('tip_link').value
        self.base_frame = g('base_frame').value
        self.lift_height = g('lift_height').value
        self.approach_height = g('approach_height').value
        self.pick_frame_id = g('pick_frame_id').value
        self.pos_tol = g('pos_tolerance').value
        self.orient_tol = g('orient_tolerance').value
        self.planning_time = g('planning_time').value
        self.vel_scale = g('vel_scale').value
        self.acc_scale = g('acc_scale').value
        self.ik_mode = g('ik_mode').value
        self.ik_max_iters = int(g('ik_max_iters').value)
        self.ik_tol = g('ik_tol').value
        self.ik_accept_tol = g('ik_accept_tol').value
        self.lock_tool_pitch = bool(g('lock_tool_pitch').value)
        self.tool_pitch = float(g('tool_pitch').value)
        self.wrist_roll = float(g('wrist_roll').value)
        self._setup_wrist_lock()
        self.freeze_target_on_retry = bool(g('freeze_target_on_retry').value)
        self.gripper_change_mode = bool(g('gripper_change_mode').value)
        self.gripper_command_calibrated = bool(
            g('gripper_command_calibrated').value)
        # An uncalibrated preset must not even emit a gripper action goal.  The
        # bridge has the same independent guard, so ID5 remains blocked if a
        # caller bypasses this FSM.
        self.gripper_disabled = (
            self.gripper_change_mode
            or bool(g('gripper_disabled').value)
            or not self.gripper_command_calibrated)
        self.stop_after_descend = (
            self.gripper_change_mode or bool(g('stop_after_descend').value))
        self.arm_move_speed = g('arm_move_speed').value
        self.gripper_type = gripper_type
        self.gripper_joints = list(g('gripper_joints').value)
        self.gripper_open = g('gripper_open').value
        self.gripper_close = g('gripper_close').value
        self.gripper_squeeze_rad = float(g('gripper_squeeze_rad').value)
        self.gripper_empty_pos_tol = float(g('gripper_empty_pos_tol').value)
        self.grasp_thresh = g('grasp_effort_thresh').value
        self.drop_thresh = g('drop_effort_thresh').value
        self.gripper_action_time = g('gripper_action_time').value
        # 경고 문구가 실제 설정을 따라가게 preset 에서 PWM·트립시간을 읽어둔다.
        # 하드코딩하면 PWM 을 바꿨을 때 경고만 옛 숫자로 남아 운영자를 오도한다.
        self._grip_pwm = gpreset.get('gripper_goal_pwm')
        # (트립시간, 실측여부) — 의미 구분은 gripper_presets.trip_seconds_for 참고.
        # "재봤는데 무트립"과 "안 재봄"을 반드시 갈라야 한다(뭉개면 가장 위험한 경우에
        # 경고가 꺼진다).
        self._grip_trip_seconds, self._grip_trip_measured = trip_seconds_for(
            self._grip_pwm, gripper_type)
        self.grip_refresh_s = float(g('grip_refresh_s').value)
        self.grip_hold_warn_s = float(g('grip_hold_warn_s').value)
        if self.grip_hold_warn_s <= 0.0:
            pass
        elif self._grip_trip_seconds is None:
            if self._grip_trip_measured:
                # 실측으로 무트립이 확인된 PWM — 경고를 끈다. 안 끄면 무해한 유지에서도
                # 계속 경고가 떠 진짜 위험할 때 무시하게 된다.
                self.get_logger().info(
                    f'그리퍼 Goal PWM {self._grip_pwm} 은 실측상 무트립 구간이라 '
                    '파지 유지 경고를 끕니다.')
            else:
                self.get_logger().warn(
                    f'그리퍼 Goal PWM {self._grip_pwm} 의 트립 시간을 추정할 근거가 '
                    '전혀 없습니다 — 파지 유지 경고를 끕니다. '
                    'scripts/measure_gripper_pwm_limit.py 로 재서 '
                    "gripper_presets 의 'gripper_pwm_trip_seconds' 에 넣으세요.")
            self.grip_hold_warn_s = 0.0
        else:
            # 트립의 70% 지점에서 경고 — 운영자가 drop/stow 할 시간이 남게.
            self.grip_hold_warn_s = min(self.grip_hold_warn_s,
                                        self._grip_trip_seconds * 0.7)
            if self._grip_trip_measured:
                self.get_logger().warn(
                    f'그리퍼 Goal PWM {self._grip_pwm} — 실측상 약 '
                    f'{self._grip_trip_seconds:.0f}초에 Overload 트립합니다. '
                    f'파지 유지 {self.grip_hold_warn_s:.1f}초에 경고합니다.')
            else:
                self.get_logger().error(
                    f'⚠️ 그리퍼 Goal PWM {self._grip_pwm} 은 **트립 시간을 안 재본 값**입니다. '
                    f'더 높은 PWM 의 실측값에서 가져온 보수적 하한 '
                    f'{self._grip_trip_seconds:.1f}초를 기준으로 '
                    f'{self.grip_hold_warn_s:.1f}초에 경고합니다(실제로는 더 오래 버팁니다). '
                    'scripts/measure_gripper_pwm_limit.py 로 재서 표에 넣으면 경고 시점이 '
                    '정확해집니다.')
        self.chassis_mode_timeout = g('chassis_mode_timeout').value
        self.locked_pos_tol = g('locked_pos_tol').value
        self.locked_vel_tol = g('locked_vel_tol').value
        self.locked_dwell = g('locked_dwell').value
        self.stow_pos_tol_rad = g('stow_pos_tol_rad').value
        self.carry_home = bool(g('carry_home').value)
        self.stow_lead_joints = [j for j in g('stow_lead_joints').value
                                 if j in ARM_JOINT_NAMES]
        unknown_lead = [j for j in g('stow_lead_joints').value
                        if j not in ARM_JOINT_NAMES]
        if unknown_lead:
            self.get_logger().warn(
                f'stow_lead_joints 의 {unknown_lead} 은 ARM_JOINT_NAMES 에 없어 무시합니다 '
                f'(유효: {ARM_JOINT_NAMES})')
        self.stow_joint_positions = list(g('stow_joint_positions').value)
        if len(self.stow_joint_positions) != len(ARM_JOINT_NAMES):
            self.get_logger().error(
                f'stow_joint_positions 길이({len(self.stow_joint_positions)})가 '
                f'ARM_JOINT_NAMES({len(ARM_JOINT_NAMES)})와 다름 — STOWING 모션 비활성')
            self.stow_joint_positions = None
        else:
            self.get_logger().warn(
                f'stow_joint_positions={self.stow_joint_positions} — URDF 상으로만 검증된 '
                '값이다(실기 미검증). 실물 구동 전 서보 tick 대응을 확인할 것.')

        # 모든 개별 센서 topic과 threshold는 SensorManager가 소유한다. FSM은 아래
        # backend-independent 판정 메서드만 사용한다.
        self.sensors = SensorManager(self)

        # ── 토픽/액션 I/O ─────────────────────────
        # QoS 는 계약(contract.py/qos_profiles.py) 기준. heartbeat 계열을 depth 10 으로
        # 두면 낡은 샘플이 큐에 쌓여 파워트레인의 age(신선도) 판정이 어긋난다.
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(DetectedObject, '/pick_target', self._on_pick_target, latched)
        self.create_subscription(ArrivalStatus, '/arrival_status', self._on_arrival, ARRIVAL_QOS)
        self.create_subscription(
            ChassisMode, '/chassis_mode', self._on_chassis_mode, HEARTBEAT_QOS)
        self.create_subscription(JointState, '/joint_states', self._on_joint_states, 10)
        self.create_subscription(
            JointState, '/tool/joint_states', self._on_joint_states, 10)
        self.create_subscription(
            TaskCommand, g('vla_command_topic').value, self._on_task_command, 10)
        # 계약 §5.1 "locked heartbeat는 ... controller fault 0 ... 을 실제 확인한다" —
        # moveit_dynamixel_bridge가 Hardware Error Status를 집계해 발행(내부용 토픽,
        # 파워트레인 DDS 경계를 넘지 않음). _is_settled()에서 게이트로 사용.
        self.create_subscription(Bool, '/dynamixel/controller_fault',
                                  self._on_controller_fault, 10)
        # 브릿지가 그리퍼 Overload 를 REBOOT 로 복구하는 동안 True. 그 구간엔 토크가
        # 끊겨 effort 가 0 으로 떨어지는데, 그걸 DROP 으로 읽으면 복구가 끝나기도 전에
        # GRIP_LOST 가 **래치**돼(자동 재시도 없음) 복구가 무의미해진다.
        self.create_subscription(Bool, '/dynamixel/gripper_recovering',
                                  self._on_gripper_recovering, 10)

        self.pub_status = self.create_publisher(ArmStatus, '/arm_status', HEARTBEAT_QOS)
        self.pub_task_result = self.create_publisher(
            TaskResult, g('vla_result_topic').value, 10)
        self.pub_fsm_state = self.create_publisher(String, '/fsm/state', 10)
        self.pub_control_mode = self.create_publisher(
            String, '/control/mode_status', 10)
        self.pub_contact_status = self.create_publisher(
            Bool, '/sensors/contact_status', 10)

        # TF: base_link ← tip_link 조회용 (LIFT 시 현재 TCP 기준 수직 리프트 계산)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # MoveIt action is used when ik_mode == 'moveit'.
        self._move = ActionClient(self, MoveGroup, 'move_action')
        self._cleaning_pub = self.create_publisher(Bool, '/cleaning/enable', 10)
        self._tool_stop_pub = self.create_publisher(Bool, '/tool/emergency_stop', 10)
        self._gripper = ActionClient(
            self, FollowJointTrajectory,
            '/gripper_controller/follow_joint_trajectory')

        # analytic IK 경로 (ik_mode=='analytic', 기본): FK 서비스 + 직접 관절궤적 publish
        # ⚠️ FK 호출은 _tick(타이머 콜백) 안에서 블로킹 대기함 — self 를 spin하면 이미
        # 실행 중인 콜백을 재진입 spin 하게 되어 응답을 못 받고 타임아웃(실측 확인:
        # 독립 스크립트로는 2회 반복만에 수렴하는데 노드 내부에서는 즉시 실패).
        # 별도 헬퍼 노드/이그제큐터로 분리해서 우회.
        self._fk_node = rclpy.create_node('arm_fsm_fk_client')
        self._fk_client = self._fk_node.create_client(GetPositionFK, '/compute_fk')
        self._arm_traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)
        self._joint_position = {}          # joint_name -> position(rad), /joint_states 에서 갱신
        self._joint_effort = {}
        self._arm_move_deadline = None      # analytic 이동 완료 예상 시각

        # ── 내부 상태 ─────────────────────────────
        # 빈손으로 접혀 잠긴 평상시 상태. 외부 heartbeat의 STOWED_LOCKED와 내부 FSM
        # 상태를 일치시켜, 시작 직후에도 새 pickup conjunction을 받을 수 있게 한다.
        self.state = State.STOWED_LOCKED
        self._state_enter_t = self.get_clock().now()
        self._prev_state = None
        self.locked = False
        self.pick_target = None
        self._planned_grasp_pose = None
        self._planned_approach_pose = None
        self._planned_grasp_xyz = None
        self._planned_approach_xyz = None
        self.mission_id = 0
        self.task_command = ''
        self.tool_type = ''
        self._tool_status = None
        self._tool_status_stamp = None
        self.selected_tool_type = self.get_parameter('tool_type').value
        manager = ToolManager(
            load_profiles(self.get_parameter('tool_profile_file').value),
            ParameterToolIdentityProvider(self.selected_tool_type),
            mock_mode=self.dry_run_mode)
        selection = manager.refresh('IDLE')
        self.tool_profile = selection.profile
        self.tool_profile_valid = selection.valid
        self.create_subscription(String, '/tool/status', self._on_runtime_tool_status, 10)
        self.create_subscription(String, '/control/mode', self._on_mode_request, 10)

        self._gripper_command_state = 'idle'
        self._gripper_command_ok = False
        self.control_mode = 'FSM'
        self._task_result_sent = False
        # 브릿지 controller fault 게이트 — 첫 샘플 받기 전엔 알 수 없으니 보수적으로 True
        # (TF 미가용 시 _is_settled()가 False를 리턴하는 것과 같은 안전 측 기본값).
        self._controller_fault = True
        # 브릿지가 그리퍼 Overload 를 재부팅 복구 중인가 (DROP 오판 방지 게이트).
        self._gripper_recovering = False
        # 계약 v2 — MISSION_STOP + ArrivalStatus conjunction 게이트 (순서 무관)
        self._mission_stop_active = False
        self._chassis_mode = None
        self._pending_arrival = None        # 아직 소비 안 한 최신 ArrivalStatus
        self._last_arrival_stamp = None
        self._last_chassis_stamp = None
        self._last_chassis_recv_wall = None     # chassis_mode 마지막 수신 시각(워치독용)
        self._last_completed_mission_id = None  # STOWED_LOCKED 도달한 mission_id (중복 재실행 방지)
        # locked-heartbeat 실측 확인용 (_is_settled)
        self._settle_start = None          # 안정 유지 시작 시각(불안정 감지 시 리셋)
        self._last_settle_pos = None       # 직전 tick의 tip 위치(np.array)
        self._last_settle_time = None
        self._last_settle_joints = {}      # 직전 tick의 관절각 스냅샷(유한차분 속도용)
        # 팔 모션(MoveIt/직접궤적) 진행 추적
        self._motion_state = 'idle'        # 'idle' | 'active' | 'done'
        self._motion_ok = False
        self._arm_goal_handle = None
        self._grip_sent = False            # 상태 진입 시 _transition에서 리셋
        # 접힘 시퀀스 진행 단계(선행축 → 전체). 상태 진입 시 _transition 에서 리셋.
        self._stow_stage = 0
        self._stow_plan = None             # 시작 시 한 번 세워 고정하는 단계 계획
        # LOWER_RELEASE 되돌림 단계(원래 집었던 지점 위 → 지점). _transition 에서 리셋.
        self._return_stage = 0
        # LIFT 안에서 carry_home 접이 단계로 넘어갔는가 (리프트 모션 처리와 분리).
        self._carry_home_active = False
        # 파지를 시작한 시각 — Overload 트립 경고용(grip_hold_warn_s 참고).
        # 상태 전환마다 리셋하면 안 된다: 파지는 GRASP 에서 시작해 RELEASE 까지
        # 여러 상태에 걸쳐 **연속으로** 유지되고, 트립은 그 누적 시간으로 결정된다.
        self._grip_hold_start = None
        self._grip_hold_warned = False
        # 파지 유지 재발행용 — _send_gripper 가 clamp 후 값을 저장한다(_maintain_grip).
        self._grip_hold_target = None
        self._last_grip_refresh = None

        # ── heartbeat ─────────────────────────────
        # 계약: 현재 상태를 10Hz 로 끊임없이 발행한다. 0.5초 넘게 끊기면 파워트레인이
        # arm_status_stale 로 차를 세운다.
        #
        # 발행 경로는 **반드시 이 타이머 하나뿐**이어야 한다. 상태 핸들러가 각자
        # publish 하면 stamp 가 뒤섞여 나갈 수 있는데, 파워트레인은 stamp 가 0.5초 이상
        # 역행하면 **영구 latch**(프로세스 재시작 전까지 해제 불가)를 건다.
        # 그래서 핸들러는 _set_status() 로 값만 바꾸고, 실제 발행은 여기서만 한다.
        #
        # 별도 콜백그룹인 이유: _tick 은 analytic IK 의 FK 호출에서 블로킹 대기한다.
        # 같은 그룹이면 IK 도는 동안 heartbeat 가 굶어 stale 판정을 맞는다.
        # → main() 이 MultiThreadedExecutor 로 띄운다.
        #
        # ⚠️ 기동 직후엔 실제 접힘 자세를 아직 확인 못 했으므로 STOWED_LOCKED(주행 허가)로
        # 시작하지 않는다 — 첫 _tick() 이전 짧은 순간(재시작 시 팔이 임의 자세여도) 파워
        # 트레인이 즉시 주행 허가로 오인할 수 있었다. _do_stowed_locked()가 실제 검증 후
        # 승격하도록 아래에서 함께 수정 — _do_locked()가 이미 쓰는 것과 같은 안전 측
        # 기본값(ARM_EXECUTING, 미확인 상태 표시)으로 시작한다.
        self._status = ARM_EXECUTING
        self._hb_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(1.0 / HEARTBEAT_RATE_HZ, self._publish_heartbeat,
                          callback_group=self._hb_group)

        period = 1.0 / g('tick_rate').value
        self._tick_group = MutuallyExclusiveCallbackGroup()
        self.create_timer(period, self._tick, callback_group=self._tick_group)
        self.get_logger().info(
            f'arm_fsm_node started (MoveIt 경로, state=IDLE, cleaning_actuator='
            f'{self.cleaning_actuator_joint or "UNCONFIGURED"}, '
            f'tool_type={self.selected_tool_type}, '
            f'tool_profile_valid={self.tool_profile_valid}, '
            f'heartbeat={HEARTBEAT_RATE_HZ}Hz)'
        )

    def _on_mode_request(self, msg):
        mode = msg.data.strip().upper()
        if mode in ('MANUAL', 'FSM'):
            self.control_mode = mode
            self.pub_control_mode.publish(String(data=mode))

    def _on_runtime_tool_status(self, msg):
        try:
            status = json.loads(msg.data)
            tool = status['tool_type']
            profile = status['tool_profile']
            if tool not in ('dual_motor_gripper', 'spur_1motor_gripper', 'cleaner'):
                return
        except (ValueError, KeyError, TypeError):
            return
        # Only the tool context changes. Arm state, joint feedback and MoveIt
        # clients remain owned by this same node for its entire lifetime.
        changed = tool != self.selected_tool_type
        self.selected_tool_type = tool
        self.tool_profile = dict(profile)
        self.tool_profile_valid = bool(status.get('profile_valid'))
        self._tool_status = status
        self._tool_status_stamp = self.get_clock().now()
        if changed:
            self._gripper_command_state = 'idle'
            self._gripper_command_ok = False
            self._grip_hold_target = None
            self.gripper_joints = list(profile.get('joint_names', []))
            self.gripper_open = float(profile.get('open_position', 1.0))
            self.gripper_close = float(profile.get('close_position', 0.0))
            if profile.get('action_time') is not None:
                self.gripper_action_time = float(profile['action_time'])
            self.cleaning_actuator_joint = (
                (profile.get('joint_names') or [''])[0] if tool == 'cleaner' else '')

    def _tool_ready(self):
        if self._tool_status_stamp is None:
            return False
        age = (self.get_clock().now() - self._tool_status_stamp).nanoseconds / 1e9
        return bool(age < 1.5 and self.tool_profile_valid
                    and self._tool_status.get('motion_allowed')
                    and not self._tool_status.get('emergency_stop')
                    and not self._tool_status.get('tool_detached'))

    # ── 콜백 ───────────────────────────────────

    def _on_pick_target(self, msg):
        self.pick_target = msg

    def _on_task_command(self, msg):
        """Accept only mission-level VLA commands; hardware remains behind this FSM."""
        command = msg.command.upper()
        if self.control_mode != 'FSM':
            self._publish_task_result(
                msg.mission_id, False, self.state.name,
                'control ownership is MANUAL')
            return
        if command not in {'CLEAN', 'PICK', 'MOVE', 'STOP', 'STOW'}:
            self._publish_task_result(msg.mission_id, False, 'REJECTED', 'unknown command')
            return
        if command == 'STOP':
            self._cancel_arm_motion()
            self._set_cleaning(False)
            self._tool_stop_pub.publish(Bool(data=True))
            self._publish_task_result(msg.mission_id, True, 'IDLE', 'stopped')
            self._transition(State.IDLE)
            return
        if command == 'STOW':
            self.mission_id = msg.mission_id
            self._transition(State.STOWING)
            return
        if self.state != State.IDLE:
            self._publish_task_result(msg.mission_id, False, self.state.name, 'FSM busy')
            return
        requested_tool = msg.tool_type.strip()
        accepted_aliases = {
            'gripper': ('spur_1motor_gripper', 'dual_motor_gripper'),
            'cleaner': ('cleaner',),
        }
        if (requested_tool and requested_tool != self.selected_tool_type
                and self.selected_tool_type not in accepted_aliases.get(
                    requested_tool, ())):
            self._publish_task_result(
                msg.mission_id, False, 'REJECTED',
                f'command requests {requested_tool}, selected tool is '
                f'{self.selected_tool_type}')
            return
        if not self.vla_standalone_mode and not self._mission_stop_active:
            self._publish_task_result(
                msg.mission_id, False, 'LOCKED', 'MISSION_STOP interlock not active')
            return
        self.mission_id = msg.mission_id
        self.task_command = command
        self.tool_type = self.selected_tool_type
        self._task_result_sent = False
        self._clear_frozen_target()
        target = DetectedObject()
        target.class_name = msg.target_object
        target.confidence = msg.confidence
        target.pose = msg.target_pose.pose
        self.pick_target = target
        self._transition(State.PERCEIVE)

    def _on_arrival(self, msg):
        if not self._stamp_is_fresh(msg.header.stamp, self._last_arrival_stamp):
            self.get_logger().warn('ArrivalStatus stamp 무효(0/미래/역행) — 무시')
            return
        self._last_arrival_stamp = msg.header.stamp
        self._pending_arrival = msg
        self._try_advance()

    def _on_chassis_mode(self, msg):
        if not self._stamp_is_fresh(msg.header.stamp, self._last_chassis_stamp):
            return
        self._last_chassis_stamp = msg.header.stamp
        self._last_chassis_recv_wall = self.get_clock().now()
        self._chassis_mode = msg.mode

        if msg.mode not in RECOGNIZED_MODES:
            # 미인식 mode — default-deny, 상태 변경 없음(락 유지)
            self.get_logger().warn(f'미인식 chassis_mode={msg.mode!r} — 무시')
            return

        # 계약 v2: MISSION_STOP만이 유일한 언락·작업 허가. DRIVING 포함 나머지는 전부
        # 잠금 유지 — LOCKED 탈출은 _try_advance()의 MISSION_STOP+ArrivalStatus
        # conjunction으로만 가능(자동 언락 분기 없음).
        self._mission_stop_active = (msg.mode == MODE_MISSION_STOP)

        if msg.mode in LOCK_MODES:      # contract.py 기준 DRIVING 포함
            self._enter_locked()
        elif msg.mode == MODE_STOW_REQUEST and self.state in STOW_ABORTABLE_STATES:
            # 운영자 포기/재정렬 유도 — 진행 중인 작업(또는 GRIP_LOST/LOCKED 래치) 중단하고
            # 접어 잠금. 2026-07-14: GRIP_LOST 전용이었던 범위를 작업 중 모든 상태로 확장.
            # 화물을 든 채 공중일 수 있으면(PAYLOAD_ALOFT_STATES, 또는 LIFT 중 LOCKED로
            # 중단된 경우) 그리퍼를 바로 열지 않고 LOWER_RELEASE로 먼저 내린다 — §5.1
            # 잔여 합의 ①(무조건 RELEASE 전이 = 화물 낙하 경로) 대응.
            aloft = (self.state in PAYLOAD_ALOFT_STATES
                     or (self.state == State.LOCKED and self._prev_state == State.RETRACT))
            self._cancel_arm_motion()
            self._set_cleaning(False)
            self.locked = False
            if self.mission_type == MISSION_ROTARY_TOOL:
                self._transition(State.STOWING)
            else:
                self._transition(State.LOWER_RELEASE if aloft else State.RELEASE)

        self._try_advance()

    def _enter_locked(self):
        """PREEMPTIBLE_STATES 중이면 모션 취소 후 LOCKED 진입. 이미 락이면 no-op."""
        self.locked = True
        if self.state in PREEMPTIBLE_STATES:
            self._prev_state = self.state
            self._cancel_arm_motion()
            self._set_cleaning(False)
            self._transition(State.LOCKED)

    def _try_advance(self):
        """MISSION_STOP + ArrivalStatus conjunction(순서 무관) 충족 시에만 전이.

        픽업 개시: IDLE/STOWED_LOCKED/GRIP_LOST/FAILED 에서 ARRIVED_PICKUP 수신 시
        PERCEIVE.
        지형 중단 복귀: LOCKED(같은 mission_id) 에서도 동일 conjunction으로 PERCEIVE 재진입
        (중단 시점 재개 대신 PERCEIVE부터 다시 — 중단 중 타겟이 변했을 수 있어 더 안전).
        하역: CARRY 에서 ARRIVED_DROP(같은 mission_id) 수신 시 RELEASE.
        이미 STOWED_LOCKED 까지 끝난 mission_id 의 재발행(중복)은 무시.
        """
        msg = self._pending_arrival
        if msg is None or not self._mission_stop_active:
            return

        pickup_states = (State.IDLE, State.STOWED_LOCKED, State.GRIP_LOST, State.FAILED)
        if self.state in pickup_states and msg.status == ARRIVED_PICKUP:
            if msg.mission_id == self._last_completed_mission_id:
                return  # 이미 완료된 mission_id 재발행 — 재실행 금지
            if msg.mission_id != self.mission_id:
                self._clear_frozen_target()
            self.mission_id = msg.mission_id
            if not self.task_command:
                self.task_command = (
                    'CLEAN' if self.selected_tool_type == 'cleaner' else 'PICK')
                self.tool_type = self.selected_tool_type
            self._pending_arrival = None
            self._set_status(ARM_WORK_READY)
            self._transition(
                self._rotary_entry_state()
                if self.mission_type == MISSION_ROTARY_TOOL
                else State.PERCEIVE)
            self._pending_arrival = None
        elif (self.state == State.LOCKED and msg.status == ARRIVED_PICKUP
                and msg.mission_id == self.mission_id):
            self.locked = False
            self._set_status(ARM_WORK_READY)
            self._transition(
                self._rotary_entry_state()
                if self.mission_type == MISSION_ROTARY_TOOL
                else State.PERCEIVE)
            self._pending_arrival = None
        elif (self.state == State.CARRY and msg.status == ARRIVED_DROP
                and msg.mission_id == self.mission_id):
            self._transition(State.RELEASE)
            self._pending_arrival = None

    def _rotary_entry_state(self):
        if not self.integrated_test_mode:
            return State.END_EFFECTOR_ROTATE
        return (State.RANDOM_ARM_DEMO if self.random_demo_enabled
                else State.ARM_TEST_MOVE)

    def _stamp_is_fresh(self, stamp, prev_stamp):
        """0/미래/동일·역행 stamp 거부 (계약 §5.1 heartbeat freshness 기준)."""
        t = stamp.sec + stamp.nanosec * 1e-9
        if t <= 0.0:
            return False
        now = self.get_clock().now().nanoseconds * 1e-9
        if t > now + STAMP_FUTURE_TOL:
            return False
        if prev_stamp is not None:
            pt = prev_stamp.sec + prev_stamp.nanosec * 1e-9
            if t <= pt:
                return False
        return True

    def _on_joint_states(self, msg):
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self._joint_position[name] = msg.position[i]
            if i < len(msg.effort):
                self._joint_effort[name] = abs(float(msg.effort[i]))

    def _on_controller_fault(self, msg):
        self._controller_fault = bool(msg.data)

    def _on_gripper_recovering(self, msg):
        recovering = bool(msg.data)
        if recovering and not self._gripper_recovering:
            self.get_logger().warn(
                '브릿지가 그리퍼 Overload 를 재부팅으로 복구 중입니다 — 그동안 DROP '
                '감지를 멈춥니다(토크가 끊겨 effort 가 0 이라 그대로 두면 GRIP_LOST 가 '
                '래치됩니다). ⚠️ 화물은 이미 놓쳤을 가능성이 큽니다 — 눈으로 확인하세요.')
        self._gripper_recovering = recovering

    # ── FSM tick ───────────────────────────────

    def _tick(self):
        self.pub_fsm_state.publish(String(data=self.state.name))
        self.pub_control_mode.publish(String(data=self.control_mode))
        self.pub_contact_status.publish(Bool(data=self.sensors.contact_confirmed()))
        if self.control_mode == 'MANUAL':
            return
        self._check_chassis_mode_watchdog()
        if (self._motion_state == 'active' and self._arm_move_deadline is not None
                and self.get_clock().now() >= self._arm_move_deadline):
            self._motion_state = 'done'
            self._arm_move_deadline = None
        handler = getattr(self, f'_do_{self.state.name.lower()}', None)
        if handler:
            handler()

    def _check_chassis_mode_watchdog(self):
        """chassis_mode 수신 끊김 = default-deny(잠금 유지, §5.1)."""
        if self._last_chassis_recv_wall is None:
            return  # 아직 한 번도 못 받음 — IDLE 기본값(안 움직임)으로 이미 안전
        age = (self.get_clock().now() - self._last_chassis_recv_wall).nanoseconds * 1e-9
        if age > self.chassis_mode_timeout:
            self._mission_stop_active = False
            self._enter_locked()

    def _is_settled(self):
        """locked 하트비트(CARRYING_LOCKED/STOWED_LOCKED) 발행 전 실제 확인 — §5.1.

        tip pose(TF, base_frame←tip_link)가 연속 tick 사이 `locked_pos_tol` 이내로
        유지되고, 관절각 유한차분 속도가 `locked_vel_tol` 이내인 상태가 `locked_dwell`
        초 이상 지속돼야 True. TF 조회 실패·불안정 감지 시 dwell 타이머 리셋(안전 측
        기본값 = 미확인). controller fault(`/dynamixel/controller_fault`, 브릿지가
        Hardware Error Status 집계) 가 True 이면 즉시 미확인 처리(2026-07-15 추가 —
        이전엔 브릿지에 해당 필드가 없어 미포함이었음).
        """
        if self.dry_run_mode:
            return True
        if self._controller_fault:
            self._settle_start = None
            return False
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException:
            self._settle_start = None
            return False
        t = tf.transform.translation
        pos = np.array([t.x, t.y, t.z])
        now = self.get_clock().now()

        stable = False
        if self._last_settle_pos is not None and self._last_settle_time is not None:
            dt = (now - self._last_settle_time).nanoseconds * 1e-9
            pos_delta = float(np.linalg.norm(pos - self._last_settle_pos))
            max_joint_vel = 0.0
            if dt > 1e-6:
                for name in ARM_JOINT_NAMES:
                    prev = self._last_settle_joints.get(name)
                    cur = self._joint_position.get(name)
                    if prev is not None and cur is not None:
                        max_joint_vel = max(max_joint_vel, abs(cur - prev) / dt)
            stable = (pos_delta <= self.locked_pos_tol
                      and max_joint_vel <= self.locked_vel_tol)

        self._last_settle_pos = pos
        self._last_settle_time = now
        self._last_settle_joints = dict(self._joint_position)

        if not stable:
            self._settle_start = None
            return False
        if self._settle_start is None:
            self._settle_start = now
        return (now - self._settle_start).nanoseconds * 1e-9 >= self.locked_dwell

    def _do_idle(self):
        # 계약 v2: 평상시(빈손) 주행 중 상시 하트비트 — WORK_READY 아님, STOWED_LOCKED.
        # WORK_READY는 MISSION_STOP+ArrivalStatus conjunction 수락 순간(_try_advance)의
        # 1회성 ack로 재배치됨. (STOWING에서 settle 확인 후 넘어온 상태라 여기선 재확인 안 함)
        self._set_status(ARM_STOWED_LOCKED)

    def _do_perceive(self):
        self._set_status(ARM_PERCEIVING)
        if self.pick_target is None:
            return
        if self.pick_target.pose.position.z == 0.0:   # depth 무효 (Phase 2 require_depth 기준)
            self.get_logger().warn('pick_target depth 무효 — 대기')
            return
        self._transition(State.PLAN)

    def _do_plan(self):
        """안전 게이트를 통과한 검출 하나를 접근/작업 목표로 고정한다."""
        self._set_status(ARM_PLANNING)
        if self.task_command in ('PICK', 'CLEAN') and not self._tool_ready():
            self.get_logger().error(
                f'{self.selected_tool_type} profile/backend not ready; arm motion blocked')
            self._publish_task_result(
                self.mission_id, False, 'PLAN', 'tool backend not ready')
            self._set_status(ARM_FAILED)
            self._transition(State.IDLE)
            return
        if not self.sensors.obstacle_clear():
            return
        if self.dry_run_mode:
            self._motion_ok = True
            self._motion_state = 'done'
            self._transition(State.APPROACH)
            return
        if self.freeze_target_on_retry and self._has_frozen_target():
            # 같은 mission 안의 재시도 — 팔이 시야에 들어와 오염됐을 수 있는 새 관측 대신
            # 처음 얼린 타겟을 그대로 쓴다(파라미터 주석 참고).
            self.get_logger().info('freeze_target_on_retry: 기존 타겟 재사용 (재인식 생략)')
            self._transition(State.APPROACH)
            return
        if self.ik_mode == 'moveit':
            self._planned_grasp_pose = self._grasp_pose_in_base()
            if self._planned_grasp_pose is None:
                self._fail('target pose transform failed')
                return
            self._planned_approach_pose = self._offset_pose_z(
                self._planned_grasp_pose, self.approach_height)
        else:
            self._planned_grasp_xyz = self._grasp_target_xyz()
            if self._planned_grasp_xyz is None:
                self._fail('target position transform failed')
                return
            x, y, z = self._planned_grasp_xyz
            self._planned_approach_xyz = (x, y, z + self.approach_height)
        self._transition(State.APPROACH)

    def _has_frozen_target(self):
        """이번 mission에서 이미 고정한 목표가 있는지 IK 경로별로 확인한다."""
        if self.ik_mode == 'moveit':
            return (self._planned_grasp_pose is not None
                    and self._planned_approach_pose is not None)
        return (self._planned_grasp_xyz is not None
                and self._planned_approach_xyz is not None)

    def _clear_frozen_target(self):
        """얼린 타겟을 버린다 — 새 mission 진입 시 호출."""
        self._planned_grasp_pose = None
        self._planned_approach_pose = None
        self._planned_grasp_xyz = None
        self._planned_approach_xyz = None

    def _do_approach(self):
        """고정한 목표 위의 clearance 자세로 이동한다."""
        self._set_status(ARM_EXECUTING)
        if not self.sensors.obstacle_clear():
            self.get_logger().warn('obstacle sensor blocked/stale → motion cancel')
            self._cancel_arm_motion()
            self._set_status(ARM_FAILED)
            self._transition(State.IDLE)
            return
        if self._motion_state == 'active':
            return
        if self._motion_state == 'done':
            ok = self._motion_ok
            self._motion_state = 'idle'
            if ok:
                self._transition(
                    State.TOOL_ACTION if self.dry_run_mode else State.DESCEND)
            else:
                self._fail('approach motion failed')
            return
        if self.ik_mode == 'moveit':
            self._begin_arm_move(self._planned_approach_pose)
        elif not self._move_to_xyz(self._planned_approach_xyz):
            self._fail('approach IK failed')

    def _do_descend(self):
        """Descend from the clearance pose to the frozen grasp pose.

        2026-08-19: 그리퍼는 팔이 **실제로 멈춘 뒤**에만 닫는다. `_motion_state=='done'`
        은 추정 소요시간(`duration+0.5s`, `_publish_joint_trajectory`) 경과일 뿐 실측
        정지 확인이 아니라서, 하중·마찰로 실기 이동이 추정보다 길어지면 그리퍼가 아직
        내려가는 중에 닫히기 시작할 수 있다. `_is_settled()`(tip TF + 관절속도 유한차분,
        LOCKED 하트비트에도 쓰는 실측 정지 판정)를 추가로 통과해야 GRASP 로 넘어간다.
        """
        self._set_status(ARM_EXECUTING)
        if self._motion_state == 'active':
            return
        if self._motion_state == 'done':
            if not self._motion_ok:
                self._motion_state = 'idle'
                self._fail('descend motion failed')
                return
            if not self._is_settled():
                return  # 모션은 끝났다고 보고됐지만 아직 실측 정지 확인 전 — 대기
            self._motion_state = 'idle'
            self._transition(State.TOOL_ACTION)
            return
        if self.ik_mode == 'moveit':
            self._begin_arm_move(self._planned_grasp_pose)
        elif not self._move_to_xyz(self._planned_grasp_xyz):
            self._fail('descend IK failed')

    def _do_tool_action(self):
        """선택된 backend에 맞는 기존 작업 상태로만 분기한다."""
        backend = self.tool_profile.get('backend')
        if self.task_command == 'PICK' and backend == 'gripper':
            self._transition(State.GRASP)
        elif self.task_command == 'CLEAN' and backend == 'cleaner':
            self._transition(State.CLEAN_START)
        elif self.task_command == 'MOVE':
            self._transition(State.RETRACT)
        else:
            self._fail_tool_action(
                f'unsupported task/tool pair: {self.task_command}/{backend}')

    def _do_grasp(self):
        """Close the gripper; success evaluation normally belongs to GRASP_CHECK.

        2026-08-19: 닫는 도중 전류(effort)를 계속 감시한다 — 완전닫힘 전에 급전류가
        튀면 손가락이 물체에 걸려 막힌 것이므로, 끝까지(gripper_action_time) 기다리지
        않고 **그 순간 파지 성공으로 간주**해 바로 LIFT 로 넘어간다(더 조이지 않음).
        ⚠️ `_do_grasp_check()` 상단 주석과 같은 함정을 피해야 한다 — 완전닫힘 근처의
        전류 상승은 기구적 끝단을 미는 것이지 파지가 아니다. 그래서 위치가 아직
        `gripper_empty_pos_tol` 밖(=완전닫힘까지 안 갔음)일 때만 전류 스파이크를
        성공 신호로 인정한다. 스파이크가 없으면 기존과 동일하게 시간 경과 후
        GRASP_CHECK 에서 위치+전류를 함께 재확인한다(빈손 폴백 유지).
        """
        self._set_status(ARM_EXECUTING)
        if not self._tool_ready():
            self._fail_tool_action('gripper backend became unavailable')
            return
        if not self._grip_sent:
            self._send_gripper(self.gripper_close - self.gripper_squeeze_rad)
            self._grip_sent = True
            return
        pos = self._joint_position.get(self.gripper_joints[0])
        near_closed = (pos is not None
                       and abs(pos - self.gripper_close) <= self.gripper_empty_pos_tol)
        if not near_closed and self._gripper_effort() >= self.grasp_thresh:
            self.get_logger().info(
                f'그리퍼 전류 스파이크 감지 (effort {self._gripper_effort():.1f} '
                f'≥ {self.grasp_thresh:.1f}, pos={pos if pos is not None else float("nan"):.3f}) '
                '— 파지 성공으로 간주, 더 조이지 않고 LIFT로 진행')
            self._begin_grip_hold()
            self._transition(State.LIFT)
            return
        if self._elapsed() >= self.gripper_action_time:
            self._transition(State.GRASP_CHECK)

    def _do_grasp_check(self):
        self._set_status(ARM_EXECUTING)
        self._maintain_grip()
        pos = self._joint_position.get(self.gripper_joints[0])
        if pos is not None and abs(pos - self.gripper_close) <= self.gripper_empty_pos_tol:
            self.get_logger().error(
                f'그리퍼가 완전닫힘({self.gripper_close:.3f})까지 닫혔습니다 '
                f'(위치 {pos:.3f}, 허용 {self.gripper_empty_pos_tol:.3f}) — '
                '손가락 사이에 아무것도 없습니다(빈손). effort 는 끝단을 미는 힘입니다.')
            self._on_grasp_failure()
            return
        if self._gripper_effort() >= self.grasp_thresh:
            self._begin_grip_hold()
            self._transition(State.LIFT)
        else:
            self._on_grasp_failure()

    def _fail_tool_action(self, reason):
        self.get_logger().error(reason)
        self._tool_stop_pub.publish(Bool(data=True))
        self._publish_task_result(
            self.mission_id, False, self.state.name, reason)
        self._set_status(ARM_FAILED)
        self._transition(State.STOWING)

    def _do_clean_start(self):
        """털털이 회전을 시작한다. 미설정 actuator에서는 안전하게 실패한다."""
        self._set_status(ARM_EXECUTING)
        if not self.cleaning_actuator_joint and not self.dry_run_mode:
            self.get_logger().error('cleaning_actuator_joint 미설정 — 털털이 구동 금지')
            self._set_status(ARM_FAILED)
            self._transition(State.RELEASE)
            return
        if not self._cleaning_command_sent:
            self._set_cleaning(True)
            self._cleaning_command_sent = True
            return
        if self._elapsed() < self.cleaning_start_time:
            return
        self._transition(State.CONTACT_CHECK)

    def _do_contact_check(self):
        """SensorManager의 contact backend fusion 결과를 확인한다."""
        self._set_status(ARM_EXECUTING)
        if self.sensors.contact_confirmed():
            self._transition(State.CLEAN)
        elif self._elapsed() >= self.contact_timeout:
            self.get_logger().warn('털털이 접촉 확인 실패 → CLEAN_STOP')
            self._set_status(ARM_FAILED)
            self._transition(State.RELEASE)

    def _do_clean(self):
        self._set_status(ARM_EXECUTING)
        if self._elapsed() >= self.clean_duration:
            self._transition(State.CLEAN_STOP)

    def _do_clean_stop(self):
        self._set_status(ARM_EXECUTING)
        if not self._cleaning_command_sent:
            self._set_cleaning(False)
            self._cleaning_command_sent = True
            return
        if self._elapsed() >= 0.2:
            self._transition(State.RETRACT)

    def _do_retract(self):
        """수직 리프트 → 운반 자세 (base_link +Z, 현재 tip TF 기준)."""
        self._set_status(ARM_EXECUTING)
        self._check_grip_hold()
        self._maintain_grip()

        # carry_home: 들어올린 뒤 **물건을 문 채로** 접힘(=URDF home) 자세까지 접고 나서
        # CARRY 로 간다. 그리퍼는 건드리지 않으므로 계속 물고 있다.
        #
        # 왜 CARRY 가 아니라 여기서 하나: CARRY 는 DROP 감시(effort 급감 → GRIP_LOST)를
        # 도는 상태라, 그 안에서 큰 모션을 시작하면 이동 중 흔들림을 낙하로 오탐할 수
        # 있다. 이동을 먼저 끝내고 CARRY 에 들어가면 감시 대상이 "정지해서 물고 있는
        # 상태" 하나로 유지된다.
        #
        # ⚠️ 접힘이 **여러 단계**(선행축 먼저)라 리프트 모션 처리와 분리해야 한다.
        # 예전 구조는 `_carry_home_sent` 하나로 "보냈다" 만 표시해서, 1단계가 끝나
        # `_motion_state=='done'` 이 되면 플래그가 이미 True 라 **나머지 단계를 건너뛰고**
        # 곧장 CARRY 로 갔다. 접이 단계로 들어갔는지를 별도 플래그로 들고 간다.
        if self._carry_home_active:
            if not self._run_stow_sequence():
                return
            self._transition(State.CARRY)
            return

        if self._motion_state == 'active':
            return
        if self._motion_state == 'done':
            ok = self._motion_ok
            self._motion_state = 'idle'
            if not ok:
                self._fail('lift motion failed')
                return
            if self.carry_home:
                if self.stow_joint_positions is None:
                    self.get_logger().warn(
                        'carry_home=true 이지만 stow_joint_positions 가 없어 건너뜁니다')
                else:
                    self.get_logger().info('carry_home: 물건을 문 채 home 자세로 접습니다')
                    self._carry_home_active = True
                    return
            self._transition(State.CARRY)
            return

        # 'idle' — 리프트 모션 시작
        if self.ik_mode == 'moveit':
            lift_pose = self._carry_pose()
            if lift_pose is None:
                self.get_logger().warn('carry pose 미구현/TF 실패 — 스킵하고 CARRY 진입')
                self._transition(State.LOCK_CHECK)
                return
            self._begin_arm_move(lift_pose)
            return

        target = self._lift_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self.get_logger().warn('LIFT 목표 계산/이동 실패 — 스킵하고 CARRY 진입')
            self._transition(State.LOCK_CHECK)

    def _do_lock_check(self):
        """리프트 후 기계식/전기식 lock confirmation을 fail-closed 확인한다."""
        self._set_status(ARM_EXECUTING)
        if self.sensors.lock_confirmed():
            self._transition(State.DONE if self.task_command else State.CARRY)
        elif self._elapsed() >= self.lock_check_timeout:
            self.get_logger().warn('lock confirmation missing/stale → RELEASE')
            self._set_status(ARM_FAILED)
            self._transition(State.RELEASE)

    def _do_carry(self):
        """청소 후 이동 가능한 안정 자세를 유지한다.

        `_is_settled()`(pose·관절속도·dwell 실측) 충족 전엔 `CARRYING_LOCKED` 대신
        `EXECUTING`을 발행 — 문자열만 바꾸지 않고 실제 정지 확인 후에만 locked 하트비트.
        """
        self._set_status(ARM_CARRYING_LOCKED if self._is_settled() else ARM_EXECUTING)
        self._check_grip_hold()
        self._maintain_grip()
        if self._elapsed() < 0.5:          # 진입 직후 dwell (DROP 오탐 방지, settle과 별개)
            return
        if self._gripper_recovering:
            return          # 복구 중 — effort 0 은 트립 때문이지 낙하 판정 근거가 아니다
        if self._gripper_effort() < self.drop_thresh:
            self.get_logger().warn('DROP 감지 (effort 급감) → GRIP_LOST 래치')
            self._cancel_arm_motion()
            self._transition(State.GRIP_LOST)
        # ARRIVED_DROP 은 _try_advance(MISSION_STOP conjunction)에서 RELEASE로 전이

    def _do_grip_lost(self):
        """supervisor-latched hold — 자동 재시도 없음(§5.1).

        새 MISSION_STOP+ArrivalStatus(ARRIVED_PICKUP) conjunction이 다시 와야만
        `_try_advance()`가 PERCEIVE로 재진입시킴. STOW_REQUEST 수신 시
        `_on_chassis_mode`에서 RELEASE로 강제 전이(운영자 포기, `STOW_ABORTABLE_STATES`).
        """
        self._set_status(ARM_GRIP_LOST)

    def _do_lower_release(self):
        """RELEASE 전에 **원래 집었던 자리로 화물을 되돌려 놓는다** (픽 순서의 역재생).

        `PAYLOAD_ALOFT_STATES`(LIFT/CARRY)에서 STOW_REQUEST로 중단됐을 때만 경유한다.
        원래 취지는 "그리퍼를 열기 전에 파지 높이까지 내려 낙하를 막는다"(§5.1 잔여
        합의 ①)였고, 구현은 **현재 tip 위치에서 `lift_height` 만큼 아래**로 내리는
        것이었다.

        ⚠️ **`carry_home` 을 켜면서 그 계산이 깨졌다** (2026-08-19). 이제 STOW_REQUEST
        시점에는 팔이 이미 home 으로 접혀 있어서, "현재 위치에서 조금 아래" 는 원래
        상자가 있던 곳이 아니라 **베이스 근처 엉뚱한 자리**다. 거기서 그리퍼를 열면
        화물을 아무 데나 떨궈 놓는다.

        그래서 PLAN 이 얼려둔 목표를 그대로 되짚어 간다 — APPROACH → DESCEND → GRASP
        의 **역순**이다:
          1) 원래 집었던 지점 **위**(`_planned_approach_*`) — 접혀 있으면 여기서 펴진다.
             바로 파지점으로 가지 않는 이유: 접힌 자세에서 직선으로 내려가면 화물이
             차체나 주변을 쓸고 지나간다.
          2) 원래 집었던 지점(`_planned_grasp_*`)으로 하강 → 상자가 원래 높이로 돌아온다.
          3) `_do_release` 가 그리퍼를 연다.

        얼린 목표가 없으면(플랜 전에 중단 등) 예전 동작(현재 위치에서 `lift_height`
        하강)으로 폴백한다. 이동/IK 실패 시에도 무한정 대기하지 않고 바로 RELEASE 로
        간다 — 높이를 못 낮추는 것보다 그리퍼를 닫은 채 매달아두는 쪽이 더 위험할 수 있다.
        """
        self._set_status(ARM_EXECUTING)
        self._check_grip_hold()
        self._maintain_grip()
        if self._motion_state == 'active':
            return
        if self._motion_state == 'done':
            ok = self._motion_ok
            self._motion_state = 'idle'
            if not ok:
                self.get_logger().warn(
                    '되돌림 이동 실패 — 여기서 RELEASE 합니다(낙하 위험 감수)')
                self._transition(State.RELEASE)
                return

        targets = self._return_targets()
        if targets is not None:
            if self._return_stage >= len(targets):
                self._transition(State.RELEASE)
                return
            target = targets[self._return_stage]
            self._return_stage += 1
            self.get_logger().info(
                f'화물 되돌림 {self._return_stage}/{len(targets)} 단계 — '
                + ('원래 집었던 지점 위로' if self._return_stage == 1
                   else '원래 집었던 지점으로 하강'))
            if self.ik_mode == 'moveit':
                self._begin_arm_move(target)
            elif not self._move_to_xyz(target):
                self.get_logger().warn(
                    '되돌림 IK 실패 — 여기서 RELEASE 합니다(낙하 위험 감수)')
                self._transition(State.RELEASE)
            return

        # ── 폴백: 얼린 목표가 없다(플랜 전에 중단됨). 예전 동작 — 제자리에서 하강. ──
        if self._return_stage > 0:
            self._transition(State.RELEASE)
            return
        self._return_stage = 1

        if self.ik_mode == 'moveit':
            pose = self._lower_pose()
            if pose is None:
                self.get_logger().warn('lower pose 계산 실패 — 바로 RELEASE(낙하 위험 감수)')
                self._transition(State.RELEASE)
                return
            self._begin_arm_move(pose)
            return

        target = self._lower_target_xyz()
        if target is None or not self._move_to_xyz(target):
            self.get_logger().warn('내림 이동 실패 — 바로 RELEASE(낙하 위험 감수)')
            self._transition(State.RELEASE)

    def _return_targets(self):
        """화물을 원래 자리로 되돌리는 경로 = 픽 목표의 역재생 [approach, grasp].

        PLAN 이 얼려둔 값을 그대로 쓴다 — 다시 인식하지 않는다. 재인식하면 팔 자신이
        시야에 들어와 타겟이 통째로 튀는 문제가 있다(`freeze_target_on_retry` 주석의
        실측: 53cm 점프). 어차피 "원래 있던 자리" 가 목적이라 새 관측은 필요 없다.

        얼린 목표가 없으면(플랜 전에 중단) None → 호출부가 예전 동작으로 폴백.
        """
        if self.ik_mode == 'moveit':
            if self._planned_approach_pose is None or self._planned_grasp_pose is None:
                return None
            return [self._planned_approach_pose, self._planned_grasp_pose]
        if self._planned_approach_xyz is None or self._planned_grasp_xyz is None:
            return None
        return [self._planned_approach_xyz, self._planned_grasp_xyz]

    def _lower_pose(self):
        """현재 TCP(tip_link)를 base_link 기준 -Z(lift_height만큼) 내린 자세. `_carry_pose` 역."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'lower_pose TF 조회 실패 ({self.base_frame} <- {self.tip_link}): {e}')
            return None
        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        t = tf.transform.translation
        ps.pose.position.x = t.x
        ps.pose.position.y = t.y
        ps.pose.position.z = max(0.0, t.z - self.lift_height)
        ps.pose.orientation = tf.transform.rotation
        return ps

    def _lower_target_xyz(self):
        """현재 tip 위치(base_frame)에서 -Z lift_height 만큼 내린 목표. `_lift_target_xyz` 역."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(f'lower target TF 조회 실패: {e}')
            return None
        t = tf.transform.translation
        return (t.x, t.y, max(0.0, t.z - self.lift_height))

    def _do_release(self):
        if getattr(self, 'gripper_change_mode', False):
            self._transition(State.DESCEND_STOPPED)
            return
        if (self.mission_type != MISSION_PICK_PLACE
                or self.end_effector_kind != 'gripper'):
            self._fail('RELEASE is disabled for the selected end effector')
            return
        self._set_status(ARM_EXECUTING)
        self._set_cleaning(False)
        if self.tool_profile.get('backend') != 'gripper':
            self._transition(State.DONE)
            return
        if not self._grip_sent:
            if self.dry_run_mode or self._tool_ready():
                self._send_gripper(
                    float(self.tool_profile.get('open_position', 1.0)))
            else:
                # Never invent a motor command after a detach/fault.
                self._transition(State.DONE)
                return
            self._grip_sent = True
            return
        if self._elapsed() >= self.gripper_action_time:
            self._transition(State.DONE)

    def _do_done(self):
        self._set_status(ARM_EXECUTING)
        if self.task_command and not self._task_result_sent:
            self._publish_task_result(self.mission_id, True, 'DONE', '')
            self._task_result_sent = True
        self._transition(State.STOWING)

    def _do_stowing(self):
        """접힘 자세로 이동 → `_is_settled()` 확인 후 `STOWED_LOCKED`.

        `stow_joint_positions`(파라미터)로 직접 관절궤적을 발행하되, 2026-08-19부터
        **단계로 나눠** 보낸다(선행축 `stow_lead_joints` 먼저 → 나머지, `_run_stow_sequence`).
        모션 완료 전에 `_is_settled()`를 확인하면 CARRY 종료 시점의 자세가 우연히
        안정적이어서 실제로 접히기도 전에 통과해버릴 수 있으므로, 반드시 전 단계가
        끝난 뒤에만 settle 게이트를 본다. ⚠️ `_motion_state`를 'idle'로 되돌리면 다음
        tick에 재발행 분기를 다시 타 dwell 누적 없이 궤적을 계속 재전송하는 버그가
        있었음(2026-07-15 발견·수정) — 이제 단계 카운터 `_stow_stage`가 그 역할을 하고,
        `_transition()`이 상태를 벗어날 때만 0으로 되돌리므로 매 tick 재발행되지 않는다.
        """
        self._set_status(ARM_STOWING)
        # 🆕 2026-08-19: 접힘 자세에는 **그리퍼 열림도 포함**이다.
        #
        # `_run_stow_sequence` 는 ARM_JOINT_NAMES(팔 축)만 궤적으로 내보내고 그리퍼는
        # 건드리지 않는다. 그리퍼를 여는 곳은 `_do_release()` 하나뿐이라, RELEASE 를
        # **거치지 않고** STOWING 에 들어온 경로(실패/중단 등)에서는 그리퍼가 닫힌 채
        # 접힌다 — 실기에서 그 증상이 나왔다.
        #
        # `_grip_sent` 는 `_transition()` 이 상태 전환마다 False 로 되돌리므로 이
        # 분기는 STOWING 진입당 딱 한 번만 탄다(매 tick 재전송 아님). 아래
        # stow_joint_positions 가 None 인 폴백 경로에도 똑같이 적용되도록 **분기보다
        # 앞에** 둔다.
        if not self._grip_sent:
            self._send_gripper(self.gripper_open)
            self._grip_sent = True
        if self.stow_joint_positions is None:
            # 목표 미설정(파라미터 길이 오류) — 접이 모션 없이 현재 자세 유지로 폴백.
            if self._is_settled():
                self._transition(State.STOWED_LOCKED)
            return
        # 접힘은 여러 단계로 나뉜다(선행축 먼저) — 전 단계가 끝나야 settle 게이트를 본다.
        # `_run_stow_sequence` 가 단계별 재발행을 스스로 막으므로 예전의 `_stow_move_sent`
        # 1회 발행 가드는 필요 없다(매 tick 재발행 버그도 그쪽에서 함께 방지된다).
        if not self._run_stow_sequence():
            return
        if self._is_settled():
            self._transition(State.STOWED_LOCKED)

    def _do_stowed_locked(self):
        """빈손으로 접혀 잠긴 평상시 상태이자 하역 완료 최종 권위.

        ⚠️ 이 상태 진입 경로는 `_do_stowing()`의 settle 확인된 전이뿐 아니라 **기동 시
        기본값**(__init__의 `self.state = State.STOWED_LOCKED`)도 있다 — 재시작 시 팔이
        실제로 어떤 자세든 있을 수 있는데 그 경우를 검증 없이 신뢰하면 안 된다. `_do_locked()`
        와 동일하게 매 tick `_near_stow_posture()`+`_is_settled()`로 실측 확인하고, 통과
        못 하면 `STOWED_LOCKED`(주행 허가) 대신 `EXECUTING`(미확인)을 발행한다.
        """
        self.pick_target = None
        self.task_command = ''
        self.tool_type = ''
        self._transition(State.IDLE)

    def _do_locked(self):
        """현재 자세 홀드: MoveIt에 새 goal 안 보냄 → 브릿지가 torque로 마지막 위치 유지.

        계약 v2 10Hz 하트비트 요구 — LOCKED 중에도 발행을 멈추면 안 됨. `_is_settled()`
        충족 전엔 `EXECUTING`(아직 정지 미확인). 충족 후 `_prev_state==LIFT`(파지 확정 뒤
        중단, 화물을 든 상태)면 `CARRYING_LOCKED`.

        그 외(PERCEIVE/PLAN/DESCEND/청소 중단, 빈손)는 예전엔 정지만 확인되면
        바로 `STOWED_LOCKED`로 근사했으나, LOCKED는 새 goal을 안 보내(모션 없이 그 자리에서
        홀드) 실제로는 임의의 안 접힌 자세일 수 있다 — 파워트레인은 `STOWED_LOCKED`를
        "차가 출발해도 되는" 근거로 그대로 신뢰하므로 이는 안전 gap이었다(§5.1 잔여 합의 ②).
        `_near_stow_posture()`로 관절각이 실제 `stow_joint_positions` 근처인지 확인한
        경우에만 `STOWED_LOCKED`를 발행하고, 아니면 정직하게 `EXECUTING`을 유지해 파워트레인
        쪽 motion hold를 받는다(거짓 주행 허가보다 안전).
        """
        if not self._is_settled():
            self._set_status(ARM_EXECUTING)
            return
        if self._prev_state == State.RETRACT:
            self._set_status(ARM_CARRYING_LOCKED)
            return
        self._set_status(ARM_STOWED_LOCKED if self._near_stow_posture() else ARM_EXECUTING)

    def _near_stow_posture(self):
        """현재 관절각이 `stow_joint_positions` 근처(±`stow_pos_tol_rad`)인지 확인.

        `stow_joint_positions`가 비활성(파라미터 길이 오류)이면 검증 불가하므로 기존
        동작(정지만 확인)으로 폴백한다 — 이미 그 경우엔 시작 시점에 에러 로그를 남긴다.
        """
        if self.stow_joint_positions is None:
            return True
        for name, target in zip(ARM_JOINT_NAMES, self.stow_joint_positions):
            cur = self._joint_position.get(name)
            if cur is None or abs(cur - target) > self.stow_pos_tol_rad:
                return False
        return True

    # ── MoveIt 팔 모션 ─────────────────────────

    def _begin_arm_move(self, pose_stamped):
        self._motion_state = 'active'
        self._motion_ok = False
        if not self._move.server_is_ready():
            self.get_logger().warn('move_action 서버 미준비 — MoveIt(move_group) 실행 확인')
        goal = self._build_move_group_goal(pose_stamped)
        self._move.send_goal_async(goal).add_done_callback(self._on_goal_response)

    def _on_goal_response(self, future):
        gh = future.result()
        if not gh.accepted:
            self.get_logger().warn('MoveGroup goal 거부됨')
            self._motion_state = 'done'
            self._motion_ok = False
            return
        self._arm_goal_handle = gh
        gh.get_result_async().add_done_callback(self._on_arm_result)

    def _on_arm_result(self, future):
        result = future.result().result
        self._motion_ok = (result.error_code.val == MOVEIT_SUCCESS)
        self._motion_state = 'done'
        self._arm_goal_handle = None

    def _cancel_arm_motion(self):
        if self._arm_goal_handle is not None:
            self._arm_goal_handle.cancel_goal_async()
            self._arm_goal_handle = None
        self._arm_move_deadline = None
        self._motion_state = 'idle'

    def _cancel_end_effector_motion(self):
        if self._rotate_goal_handle is not None:
            self._rotate_goal_handle.cancel_goal_async()
            self._rotate_goal_handle = None
        self._rotate_state = 'idle'

    def _cancel_arm_test_motion(self):
        if self._arm_test_goal_handle is not None:
            self._arm_test_goal_handle.cancel_goal_async()
            self._arm_test_goal_handle = None
        self._arm_test_state = 'idle'

    def _build_move_group_goal(self, pose_stamped):
        """목표 pose → MoveGroup goal (plan & execute). tip_link를 pose로 이동."""
        req = MotionPlanRequest()
        req.group_name = self.planning_group
        req.num_planning_attempts = 5
        req.allowed_planning_time = self.planning_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        pc = PositionConstraint()
        pc.header = pose_stamped.header
        pc.link_name = self.tip_link
        region = BoundingVolume()
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [self.pos_tol]
        region.primitives.append(sphere)
        region.primitive_poses.append(pose_stamped.pose)
        pc.constraint_region = region
        pc.weight = 1.0

        oc = OrientationConstraint()
        oc.header = pose_stamped.header
        oc.link_name = self.tip_link
        oc.orientation = pose_stamped.pose.orientation
        oc.absolute_x_axis_tolerance = self.orient_tol
        oc.absolute_y_axis_tolerance = self.orient_tol
        oc.absolute_z_axis_tolerance = self.orient_tol
        oc.weight = 1.0

        constraints = Constraints()
        constraints.position_constraints.append(pc)
        # KDL/OMPL은 off-axis TCP의 position-only goal에서 유효한 goal state를
        # 샘플하지 못한다. single TCP에는 인식/TF로 변환된 자세를
        # 함께 제약해 IK sampler가 실제 TCP chain을 풀게 한다. dual은
        # 기존 4축 position-only 계약을 그대로 유지한다.
        if self.tip_link == 'single_gripper_grasp_frame':
            constraints.orientation_constraints.append(oc)
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        # planning_options 기본값: plan_only=False → 계획 후 컨트롤러로 실행
        return goal

    def _grasp_pose(self):
        """/pick_target(DetectedObject, frame 정보 없음) → PoseStamped(pick_frame_id)."""
        if self.pick_target is None:
            return None
        ps = PoseStamped()
        ps.header.frame_id = self.pick_frame_id    # DetectedObject엔 header 없음 → 파라미터 사용
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.pose = self.pick_target.pose
        return ps

    def _grasp_pose_in_base(self):
        """선택한 인식 자세를 MoveIt 계획 프레임으로 변환한다."""
        source = self._grasp_pose()
        if source is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, source.header.frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'grasp pose TF lookup failed: {e}')
            return None
        result = PoseStamped()
        result.header.frame_id = self.base_frame
        result.header.stamp = self.get_clock().now().to_msg()
        result.pose = do_transform_pose(source.pose, tf)
        return result

    @staticmethod
    def _offset_pose_z(pose, offset):
        # ROS 메시지는 변경 가능하므로, 접근 오프셋이 DESCEND에서 사용할 고정 파지
        # 목표를 바꾸지 않도록 복사한다.
        result = deepcopy(pose)
        result.pose.position.z += offset
        return result

    def _carry_pose(self):
        """현재 TCP(tip_link)를 base_link 기준 +Z 로 들어올린 운반 자세.

        파지 직후의 실제 말단 자세를 TF(base_frame←tip_link)로 조회 → z 에 lift_height
        를 더하고 orientation 은 유지(박스 자세 보존). base_frame 이 planning frame 이라
        MoveIt 이 바로 계획 가능. TF 미가용 시 None → 호출부(_do_lift)가 LIFT 스킵.
        """
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(
                f'carry_pose TF 조회 실패 ({self.base_frame} <- {self.tip_link}): {e}')
            return None

        ps = PoseStamped()
        ps.header.frame_id = self.base_frame
        ps.header.stamp = self.get_clock().now().to_msg()
        t = tf.transform.translation
        ps.pose.position.x = t.x
        ps.pose.position.y = t.y
        ps.pose.position.z = t.z + self.lift_height
        ps.pose.orientation = tf.transform.rotation
        return ps

    # ── analytic IK (ik_mode=='analytic', URDF 3관절 한정) ─────

    def _current_arm_joint_positions(self):
        return [self._joint_position.get(j, 0.0) for j in ARM_JOINT_NAMES]

    # ── 손목 자세 잠금 ─────────────────────────

    def _setup_wrist_lock(self):
        """손목 잠금에 쓸 자유/종속 관절 인덱스를 준비한다 (모듈 상단 블록 참고).

        필요한 관절이 `ARM_JOINT_NAMES` 에 하나라도 없으면(팔 축 구성이 바뀐 경우)
        경고만 남기고 조용히 잠금을 끈다 — 여기서 죽이면 축 구성을 바꿀 때마다
        원인 모를 정지가 난다. 관절 수를 상수로 박지 않는다는 이 저장소 규약대로
        전부 이름으로 조회한다.
        """
        self._pitch_dep_idx = None
        self._roll_idx = None
        self._ik_free_idx = list(range(len(ARM_JOINT_NAMES)))
        if not self.lock_tool_pitch:
            self.get_logger().warn(
                'lock_tool_pitch=false — 손목이 자유롭습니다. IK 에 자유도가 남아돌아 '
                '상자를 대각선으로 물거나 APPROACH→DESCEND 사이에 손목만 따로 움직일 수 '
                '있습니다(2026-08-19 실기 증상).')
            return
        missing = [n for n in WRIST_PITCH_COEFFS if n not in ARM_JOINT_NAMES]
        if missing:
            self.get_logger().warn(
                f'손목 피치 잠금에 필요한 관절 {missing} 이 ARM_JOINT_NAMES 에 없어 '
                '잠금을 끕니다 — 팔 축 구성이 바뀌었다면 WRIST_PITCH_COEFFS 를 '
                '새 URDF 축으로 다시 유도할 것.')
            self.lock_tool_pitch = False
            return

        self._pitch_dep_idx = ARM_JOINT_NAMES.index(WRIST_PITCH_DEPENDENT)
        locked = {self._pitch_dep_idx}
        if WRIST_ROLL_JOINT in ARM_JOINT_NAMES:
            self._roll_idx = ARM_JOINT_NAMES.index(WRIST_ROLL_JOINT)
            locked.add(self._roll_idx)
        self._ik_free_idx = [i for i in range(len(ARM_JOINT_NAMES)) if i not in locked]

        free_names = [ARM_JOINT_NAMES[i] for i in self._ik_free_idx]
        self.get_logger().info(
            f'손목 자세 잠금 ON — tool_pitch={self.tool_pitch:.4f}rad '
            f'({math.degrees(self.tool_pitch):.1f}°, pi/2=수직 아래), '
            f'wrist_roll={self.wrist_roll:.4f}rad. '
            f'IK 자유관절={free_names}, 종속={WRIST_PITCH_DEPENDENT}'
            f'(=j2-j3-tool_pitch)')
        if len(self._ik_free_idx) != 3:
            self.get_logger().warn(
                f'IK 자유관절이 {len(self._ik_free_idx)}개입니다(위치 제약은 3개) — '
                '3개가 아니면 해가 유일하지 않거나(남는 자유도) 과결정입니다.')

    @staticmethod
    def _clamp_arm_joints(q):
        """전체 관절각 벡터를 `joint_limits` 안전범위로 제한한다.

        ⚠️ **IK 반복 안에서 매번 걸어야 한다** (2026-08-19 추가). 예전에는 이게 없어
        댐핑 최소자승이 리밋 **밖으로** 자유롭게 반복했고, 그 해가 그대로 발행된 뒤
        브릿지 `rad_to_tick` 이 마지막에 clamp 했다 — 즉 **IK 가 푼 자세와 팔이 실제로
        가는 자세가 달랐다.** URDF FK 로 재현해 보니 `arm_joint_2` 가 리밋 [0, 1.4276]
        을 한참 벗어난 -1.968 로 수렴하고, 그 상태에서 손목 종속각이 리밋에 걸려
        접근축이 수직에서 최대 88° 까지 기울었다(=상자를 옆에서 대각선으로 무는 증상).
        반복 안에서 clamp 하면 IK 는 **실제 도달 가능한 집합 안에서만** 풀고, 목표가
        정말 못 닿으면 잔차로 정직하게 실패를 보고한다.
        """
        out = []
        for name, val in zip(ARM_JOINT_NAMES, q):
            clamped, _ = joint_limits.clamp(name, float(val))
            out.append(clamped)
        return out

    def _apply_wrist_lock(self, q):
        """전체 관절각 벡터에 리밋 clamp + 손목 잠금(피치 종속 + 롤 고정)을 적용한다.

        `j4 = j2 - j3 - tool_pitch` 를 계수 dict 에서 일반적으로 풀어낸다. 순서가 중요하다 —
        **자유관절을 먼저 clamp 한 뒤** 그 값으로 종속 관절을 계산해야 피치 식이 실제
        관절각과 맞는다. 종속 관절도 `joint_limits` 로 clamp 한다: clamp 되면 요청한
        tool_pitch 가 그만큼 안 나오지만, 여기서 clamp 해야 **IK 반복이 실제로 도달 가능한
        손목 자세를 전제로** 위치를 풀어준다(풀고 나서 브릿지가 clamp 하면 그만큼 위치가
        조용히 어긋난다). 미달성은 `_move_to_xyz` 가 경고로 보고한다.
        """
        q = self._clamp_arm_joints(q)
        if not self.lock_tool_pitch:
            return q
        dep_coeff = WRIST_PITCH_COEFFS[WRIST_PITCH_DEPENDENT]
        residual = self.tool_pitch
        for name, coeff in WRIST_PITCH_COEFFS.items():
            if name == WRIST_PITCH_DEPENDENT:
                continue
            residual -= coeff * q[ARM_JOINT_NAMES.index(name)]
        dep, clamped = joint_limits.clamp(
            WRIST_PITCH_DEPENDENT, residual / dep_coeff)
        self._wrist_pitch_clamped = clamped
        q[self._pitch_dep_idx] = dep
        if self._roll_idx is not None:
            roll, _ = joint_limits.clamp(WRIST_ROLL_JOINT, self.wrist_roll)
            q[self._roll_idx] = roll
        return q

    def _tool_pitch_of(self, q):
        """관절각 벡터의 실제 tool_pitch [rad] — 로그·검증용."""
        return sum(coeff * q[ARM_JOINT_NAMES.index(name)]
                   for name, coeff in WRIST_PITCH_COEFFS.items())

    def _grasp_target_xyz(self):
        """/pick_target(카메라 프레임) → base_frame 기준 (x,y,z). 방향은 무시(3DOF 한계)."""
        if self.pick_target is None:
            return None
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, self.pick_frame_id, Time())
        except TransformException as e:
            self.get_logger().warn(f'grasp target TF 조회 실패: {e}')
            return None
        out = do_transform_pose(self.pick_target.pose, tf)
        return (out.position.x, out.position.y, out.position.z)

    def _lift_target_xyz(self):
        """현재 tip 위치(base_frame)에서 +Z lift_height 만큼 든 목표."""
        try:
            tf = self.tf_buffer.lookup_transform(self.base_frame, self.tip_link, Time())
        except TransformException as e:
            self.get_logger().warn(f'lift target TF 조회 실패: {e}')
            return None
        t = tf.transform.translation
        return (t.x, t.y, t.z + self.lift_height)

    def _fk_tip(self, q):
        """3관절 각도 q=[j1,j2,j3] → tip_link 위치(base_frame) np.array, 실패 시 None."""
        req = GetPositionFK.Request()
        req.header.frame_id = self.base_frame
        req.fk_link_names = [self.tip_link]
        req.robot_state = RobotState()
        req.robot_state.joint_state.name = list(ARM_JOINT_NAMES)
        req.robot_state.joint_state.position = [float(v) for v in q]
        if not self._fk_client.service_is_ready():
            return None
        future = self._fk_client.call_async(req)
        rclpy.spin_until_future_complete(self._fk_node, future, timeout_sec=1.0)
        res = future.result()
        if res is None or not res.pose_stamped:
            return None
        p = res.pose_stamped[0].pose.position
        return np.array([p.x, p.y, p.z])

    def _solve_position_ik(self, target_xyz, q_init):
        """FK + 수치 자코비안(finite-difference) 로 위치만 맞추는 IK.

        MoveIt 6DOF pose IK 대신 이 방식을 기본으로 씀(HW-7 실측 확인, compute_ik 가 현재
        실제 tip pose 에도 NO_IK_SOLUTION 반환하던 문제 회피). 방향은 포기하고 위치만
        댐핑 최소자승(Levenberg-Marquardt 유사)으로 반복 수렴.

        ⚠️ 관절 수는 `ARM_JOINT_NAMES`(=JOINT_CONFIG) 에서 온다 — **여기에 상수로 박지 말 것.**
        2026-08-08 수정: 예전엔 3 이 하드코딩돼 있었는데 실기 배선 확정으로 구동 관절이
        4개(arm_joint_2~5, id 14/13/12/16)가 되면서 `q + dq` 가 shape (4,)+(3,) 로 깨져
        APPROACH 에서 무조건 ValueError 로 죽었다(실기 dry-run 에서 발견).
        자코비안은 3×n(작업공간 3차원 × **자유관절** n)이고, 감쇠항 `np.eye(3)` 은
        작업공간 쪽이라 관절 수와 무관하게 3 이 맞다.

        2026-08-19 — **손목 자세 잠금**(모듈 상단 WRIST_PITCH_COEFFS 블록) 적용. 반복은
        자유관절(`_ik_free_idx`, 기본 j1·j2·j3)에 대해서만 돌고, 종속 관절(j4)·롤(j5)은
        매 평가마다 `_apply_wrist_lock` 이 채운다. 제약을 반복 **안쪽**에 두는 게 핵심 —
        풀고 나서 j4 를 덮어쓰면 그만큼 tip 위치가 어긋난다.
        """
        target = np.array(target_xyz, dtype=float)
        eps = 0.05
        lam = 0.01
        max_step = 0.4

        # 마지막 해의 위치 잔차 [m] — 호출부가 로그에 실어 "수렴(ik_tol)" 과 "겨우 수용
        # (ik_accept_tol)" 을 구분할 수 있게 한다. 이 둘은 3배 차이(1cm vs 3cm)인데
        # 로그가 같으면 파지가 빗나갈 때 IK 를 의심할지 캘리브를 의심할지 못 가린다.
        self._last_ik_residual = None
        self._wrist_pitch_clamped = False

        # 자유관절 집합은 **호출 시점의** lock_tool_pitch 로 정한다 — `_ik_free_idx` 를
        # 그대로 쓰면 `_report_ik_failure` 가 잠금을 잠깐 끄고 재진단할 때 j4/j5 가
        # 초기화 때 정해진 축소 집합에 갇혀 **얼어붙은 채로** 풀리고, "손목을 풀어도
        # 안 된다" 는 틀린 진단이 나온다.
        free_idx = (self._ik_free_idx if self.lock_tool_pitch
                    else list(range(len(ARM_JOINT_NAMES))))
        q_base = self._apply_wrist_lock(q_init)

        def expand(x):
            """자유관절 벡터 → 손목 잠금이 적용된 전체 관절각 벡터."""
            q_full = list(q_base)
            for slot, val in zip(free_idx, x):
                q_full[slot] = float(val)
            return self._apply_wrist_lock(q_full)

        x = np.array([q_base[i] for i in free_idx], dtype=float)
        p = self._fk_tip(expand(x))
        if p is None:
            return None

        for _ in range(self.ik_max_iters):
            err = target - p
            if np.linalg.norm(err) < self.ik_tol:
                self._last_ik_residual = float(np.linalg.norm(err))
                return expand(x)
            J = np.zeros((3, x.size))
            for i in range(x.size):
                dx = np.zeros(x.size)
                dx[i] = eps
                # 종속 관절이 자유관절을 따라 함께 움직이므로, 유한차분도 반드시
                # expand() 를 거쳐야 한다 — 그래야 자코비안이 "손목을 잠근 채로 이
                # 관절을 움직이면 tip 이 어디로 가는가" 를 반영한다.
                p2 = self._fk_tip(expand(x + dx))
                if p2 is None:
                    return None
                J[:, i] = (p2 - p) / eps
            delta = J.T @ np.linalg.solve(J @ J.T + lam * np.eye(3), err)
            norm = np.linalg.norm(delta)
            if norm > max_step:
                delta *= max_step / norm
            x = x + delta
            # 자유관절도 매 스텝 리밋 안으로 되돌린다 — clamp 된 값이 다음 반복의
            # 출발점이 돼야 IK 가 도달 가능 집합 안에서만 움직인다
            # (`_clamp_arm_joints` docstring 의 실측 근거 참고).
            for k, slot in enumerate(free_idx):
                x[k], _ = joint_limits.clamp(ARM_JOINT_NAMES[slot], float(x[k]))
            p = self._fk_tip(expand(x))
            if p is None:
                return None

        residual = float(np.linalg.norm(target - p))
        # 실패해도 잔차를 남긴다 — 호출부(`_report_ik_failure`)가 "몇 cm 부족한가" 를
        # 보고해야 사용자가 상자를 얼마나 당겨야 하는지 바로 안다. 예전엔 그냥 None 만
        # 돌려줘서 "도달 불가" 가 1cm 부족인지 10cm 부족인지 구분이 안 됐다.
        self._last_ik_residual = residual
        if residual < self.ik_accept_tol:
            return expand(x)
        return None

    def _move_to_xyz(self, target_xyz):
        """target_xyz(base_frame) 로 analytic IK 계산 → /arm_controller/joint_trajectory 직접 발행."""
        q_current = self._current_arm_joint_positions()
        solution = self._solve_position_ik(target_xyz, q_current)
        if solution is None:
            self._report_ik_failure(target_xyz, q_current)
            return False
        # 손목 잠금이 실제로 지켜졌는지 확인한다. 종속 관절(j4)이 리밋에 걸리면
        # 요청한 tool_pitch 가 안 나오고 — 그게 바로 "상자를 대각선으로 문다" 증상이다.
        # 조용히 넘어가면 원인을 IK/캘리브에서 찾게 되므로 눈에 띄게 남긴다.
        if self.lock_tool_pitch:
            achieved = self._tool_pitch_of(solution)
            if abs(achieved - self.tool_pitch) > 1e-3:
                lo, hi = joint_limits.get_limits(WRIST_PITCH_DEPENDENT) or (float('nan'),) * 2
                self.get_logger().warn(
                    f'손목 피치 미달성: 요청 {math.degrees(self.tool_pitch):.1f}° → '
                    f'실제 {math.degrees(achieved):.1f}° '
                    f'({WRIST_PITCH_DEPENDENT} 이 리밋 [{lo:+.3f}, {hi:+.3f}] 에 걸림). '
                    '그리퍼가 그만큼 기울어 물게 됩니다 — 목표가 팔에 너무 가깝거나 '
                    '멀지 않은지, joint_limits 의 이 축 범위가 맞는지 확인하세요.')

        self._publish_joint_trajectory(solution, q_current)
        residual = getattr(self, '_last_ik_residual', None)
        # 잔차가 ik_tol 을 넘으면 "수렴 실패했지만 수용 범위" 라는 뜻 — 파지가 그만큼
        # 빗나가므로 눈에 띄게 남긴다.
        if residual is not None and residual >= self.ik_tol:
            self.get_logger().warn(
                f'analytic IK: {[round(v, 3) for v in solution]} rad 로 이동 '
                f'(잔차 {residual * 100:.1f}cm — ik_tol {self.ik_tol * 100:.0f}cm 미수렴, '
                f'ik_accept_tol {self.ik_accept_tol * 100:.0f}cm 로 수용)')
        else:
            self.get_logger().info(
                f'analytic IK: {[round(v, 3) for v in solution]} rad 로 이동 '
                f'(잔차 {residual * 100:.1f}cm)' if residual is not None
                else f'analytic IK: {[round(v, 3) for v in solution]} rad 로 이동')
        return True

    def _report_ik_failure(self, target_xyz, q_current):
        """IK 실패 원인을 실제로 **구분해서** 보고한다 — "왜 팔이 안 움직이지" 를 로그만으로 판정.

        ⚠️ **이 진단이 거짓말을 한 적이 있다 (2026-08-19 실기).** 예전 문구는 출발 자세가
        관절 리밋 밖이기만 하면 무조건
            "목표가 먼 게 아니라 여기서 IK 가 막힌다 — stow 로 home 복귀하라"
        고 단정했다. 그런데 그날 실패의 진짜 원인은 **정반대**였다: 목표
        `(0.045, -0.412, 0.136)` 이 팔의 최대 도달 반경 밖이었다(URDF 전수 스캔 결과
        손목을 완전히 풀어도 3.5cm, top-down 잠금에서는 10.3cm 부족). 단정적인 오진이
        원인 추적을 엉뚱한 곳으로 몰았다.
        게다가 IK 반복에 리밋 clamp 가 들어간 뒤로(`_clamp_arm_joints`) 출발 자세가 리밋
        밖이어도 즉시 clamp 되므로, 그건 이제 **막히는 원인 자체가 아니다.**

        그래서 단정하지 않고 세 가지를 실제로 갈라본다:
          (1) **손목 잠금이 원인인가** — 잠금을 잠깐 풀고 다시 풀어 본다(진단 전용, 실행
              하지 않는다). 그때 풀리면 목표는 닿는데 top-down 자세로만 못 닿는 것이다.
          (2) **순수 도달 불가인가** — 잠금 없이도 실패하면 작업공간 밖이다. 최근접 잔차를
              "몇 cm 부족" 으로 보고해 상자를 얼마나 당겨야 하는지 바로 알 수 있게 한다.
          (3) **출발 자세 이상** — 이제 blocking 원인은 아니지만 팔이 처졌다는 신호이므로
              참고 정보로 같이 남긴다(단정하지 않는다).
        """
        gap = getattr(self, '_last_ik_residual', None)
        gap_s = f'{gap * 100:.1f}cm' if gap is not None else '알 수 없음'
        self.get_logger().warn(
            f'analytic IK 실패 — 목표 {tuple(round(v, 3) for v in target_xyz)} '
            f'(최근접 도달점까지 {gap_s} 부족)')

        # (1) 손목 잠금이 binding constraint 인지 — 잠금만 빼고 같은 목표를 다시 풀어본다.
        free_gap = None
        if self.lock_tool_pitch:
            saved = self.lock_tool_pitch
            try:
                self.lock_tool_pitch = False
                free_solution = self._solve_position_ik(target_xyz, q_current)
                free_gap = getattr(self, '_last_ik_residual', None)
            finally:
                self.lock_tool_pitch = saved
            if free_solution is not None:
                self.get_logger().error(
                    f'원인: **손목 자세 잠금**입니다 — 손목을 풀면 닿는 목표입니다'
                    f'(tool_pitch={math.degrees(self.tool_pitch):.0f}° 고정 때문). '
                    '상자를 팔 쪽으로 더 당기거나, 비스듬한 파지를 허용하려면 '
                    'tool_pitch 를 낮추세요(예: -p tool_pitch:=1.2 ≈ 21° 젖힘). '
                    'lock_tool_pitch:=false 로 아예 끄면 2026-08-19 이전의 '
                    '대각선 파지 동작으로 돌아갑니다.')
                return

        # (2) 손목을 풀어도 안 되면 순수 도달 불가.
        if free_gap is not None:
            gap_s = f'{free_gap * 100:.1f}cm'
        self.get_logger().error(
            f'원인: **목표가 팔의 도달 범위 밖**입니다 — 손목을 완전히 풀어도 '
            f'{gap_s} 부족합니다. 손목 잠금 문제가 아니니 tool_pitch 를 만져도 소용없습니다. '
            '상자를 팔 쪽으로 당기거나 차체를 더 붙여야 합니다. '
            '(참고: 이 팔은 base_link -Y 방향으로만 뻗고 arm_joint_1 이 ±14.3° 뿐이라 '
            '방위 회전으로 거리를 벌 수 없습니다.) 목표 좌표 자체가 의심되면 '
            'camera_tf.launch.py 의 카메라 외부파라미터를 다시 재세요 — 캘리브가 틀리면 '
            '상자는 코앞에 있는데 좌표만 멀리 나옵니다.')

        # (3) 참고: 출발 자세가 리밋 밖이면 팔이 처졌다는 신호. 이제 IK 를 막지는 않는다
        #     (반복 안에서 clamp 된다) — 그래서 '원인' 이 아니라 '참고' 로만 남긴다.
        out = []
        for name, q in zip(ARM_JOINT_NAMES, q_current):
            bounds = joint_limits.get_limits(name)
            if bounds is None:
                continue
            lo, hi = bounds
            if q < lo - 1e-3:
                out.append(f'{name}={q:+.3f}<하한{lo:+.3f}')
            elif q > hi + 1e-3:
                out.append(f'{name}={q:+.3f}>상한{hi:+.3f}')
        if out:
            self.get_logger().warn(
                '참고 — 출발 자세가 관절 리밋 밖입니다: ' + ', '.join(out)
                + ' (토크가 꺼진 동안 팔이 처졌을 때 이렇게 됩니다). IK 는 반복 안에서 '
                  'clamp 하므로 이것 때문에 막히지는 않지만, 자세를 믿을 수 없으니 '
                  'stow(STOW_REQUEST)로 home 복귀 후 다시 시도하는 게 안전합니다.')

    def _stow_stages(self):
        """접힘 목표를 단계 리스트로 만든다 (선행축 먼저 → 그다음 전체).

        `stow_lead_joints` 가 비었거나 이미 목표에 있으면 단계 하나(전 축 동시)로
        떨어져 예전 동작과 같아진다.
        """
        full = list(self.stow_joint_positions)
        if not self.stow_lead_joints:
            return [full]
        # 1단계: 선행축만 목표로, 나머지는 **현재 자세 유지**.
        q_now = self._current_arm_joint_positions()
        lead = list(q_now)
        for name in self.stow_lead_joints:
            i = ARM_JOINT_NAMES.index(name)
            lead[i] = full[i]
        # 선행축이 이미 목표 근처면 1단계는 의미 없는 재발행이라 건너뛴다.
        if all(abs(lead[i] - q_now[i]) < 1e-3 for i in range(len(lead))):
            return [full]
        return [lead, full]

    def _run_stow_sequence(self):
        """접힘 시퀀스를 한 단계씩 굴린다. **완료되면 True**, 진행 중이면 False.

        접힘 자세는 충돌 회피가 필요 없는 known-safe 설정값이라 가정하므로(실측 후
        캘리브 전제), ik_mode와 무관하게 항상 `_arm_traj_pub`로 직접 명령한다 —
        analytic IK를 거칠 필요가 없다(목표가 이미 관절각이지 xyz가 아님).

        ⚠️ **단계를 다점 궤적 하나로 합치면 안 된다.** 브릿지의 트래젝토리 토픽
        구독부는 `msg.points[-1]` 만 읽고 중간 점을 버리므로(`moveit_dynamixel_bridge`),
        한 메시지에 담으면 선행축 단계가 통째로 무시되고 곧장 최종 자세로 간다.
        그래서 단계마다 **별도 메시지를 시간차로** 발행하고, 각 단계의 완료를
        `_motion_state` 로 기다린다.

        호출부는 매 tick `if not self._run_stow_sequence(): return` 형태로 쓴다.
        단계 카운터·계획은 `_transition()` 이 상태 전환마다 리셋한다.

        ⚠️ **계획(`_stow_plan`)은 시작할 때 딱 한 번 세우고 고정한다.** 매 tick
        `_stow_stages()` 를 다시 부르면 안 된다 — 1단계가 끝난 뒤에는 선행축이 이미
        목표에 있어 계획이 2단계에서 1단계로 **줄어들고**, 카운터(이미 1)가 곧바로
        끝으로 판정돼 **나머지 축이 통째로 안 접힌다.** 시퀀스를 시뮬레이션해서 잡은
        실제 버그다. 1단계의 "나머지 축은 현재 자세 유지" 스냅샷도 시작 시점 값이어야
        의미가 있으므로, 어느 쪽으로 보든 계획은 얼려야 맞다.
        """
        if self._motion_state == 'active':
            return False
        if self._motion_state == 'done':
            self._motion_state = 'idle'
        if self._stow_plan is None:
            self._stow_plan = self._stow_stages()
        stages = self._stow_plan
        if self._stow_stage >= len(stages):
            return True
        target = stages[self._stow_stage]
        self._stow_stage += 1
        if len(stages) > 1:
            self.get_logger().info(
                f'접힘 {self._stow_stage}/{len(stages)} 단계 발행'
                + (f' (선행축 {self.stow_lead_joints} 먼저)'
                   if self._stow_stage == 1 else ' (나머지 축)'))
        self._publish_joint_trajectory(target, self._current_arm_joint_positions())
        return False

    def _publish_joint_trajectory(self, target_positions, q_current):
        """목표 관절각으로 단일 포인트 궤적 발행 + 모션 진행 상태 갱신 (공통 헬퍼)."""
        delta = max(abs(a - b) for a, b in zip(target_positions, q_current))
        duration = max(1.0, min(5.0, delta / max(self.arm_move_speed, 0.05)))

        traj = JointTrajectory()
        traj.joint_names = list(ARM_JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(v) for v in target_positions]
        pt.time_from_start = Duration(sec=int(duration))
        traj.points.append(pt)
        self._arm_traj_pub.publish(traj)

        self._motion_state = 'active'
        self._motion_ok = True
        self._arm_move_deadline = self.get_clock().now() + RclpyDuration(
            seconds=duration + 0.5)

    # ── 털털이 actuator ─────────────────────────

    def _set_cleaning(self, enabled):
        """별도 Dynamixel velocity actuator에 start/stop 의도를 전달한다."""
        self._cleaning_pub.publish(Bool(data=bool(enabled)))

    def _on_arm_test_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._arm_test_ok = False
            self._arm_test_state = 'done'
            return
        self._arm_test_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(
            self._on_arm_test_result)

    def _on_arm_test_result(self, future):
        wrapped = future.result()
        self._arm_test_ok = bool(wrapped.result.success)
        self._arm_test_state = 'done'
        self._arm_test_goal_handle = None

    def _on_rotate_goal_response(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self._rotate_ok = False
            self._rotate_state = 'done'
            return
        self._rotate_goal_handle = goal_handle
        goal_handle.get_result_async().add_done_callback(self._on_rotate_result)

    def _on_rotate_result(self, future):
        wrapped = future.result()
        self._rotate_ok = bool(wrapped.result.success)
        self._rotate_state = 'done'
        self._rotate_goal_handle = None

    def _check_grip_hold(self):
        """파지 유지 시간이 Overload 트립 구간에 들어가면 **한 번** 경고한다.

        XL430 은 전류 제어가 없어 파지력이 Goal PWM 으로만 정해지고, Overload 는 부하를
        시간에 누적해 판정한다 — 힘을 올리면 "무한정 버팀" 이 "유한 시간 트립" 으로
        바뀐다(gripper_presets 실측: PWM 280→40초+ 무트립 / 400→17초 / 885→3.5초).
        2026-08-19 PWM 400 채택으로 이제 **17초가 한계**인데, 남은 시간의 대부분은
        CARRY 에서 운영자의 drop/stow 입력을 기다리는 시간이라 코드로 줄일 수 없다.
        트립하면 토크가 끊겨 화물을 떨어뜨리고 REBOOT 전까지 응답하지 않으므로,
        적어도 **일어나기 전에 알린다.**
        """
        if self.grip_hold_warn_s <= 0.0 or self._grip_hold_start is None:
            return
        if self._grip_hold_warned:
            return
        held = (self.get_clock().now() - self._grip_hold_start).nanoseconds * 1e-9
        if held >= self.grip_hold_warn_s:
            self._grip_hold_warned = True
            trip = self._grip_trip_seconds
            when = ('측정된 트립 시간 없음' if trip is None
                    else f'약 {trip:.1f}초'
                    + ('' if self._grip_trip_measured else ' 이후(보수적 하한, 미실측)'))
            self.get_logger().warn(
                f'⚠️ 파지 유지 {held:.0f}초 — 그리퍼 Goal PWM {self._grip_pwm} 기준 '
                f'{when}에 Overload 트립합니다(토크 차단 → 화물 낙하, REBOOT 전까지 '
                '무응답). 지금 바로 drop 또는 stow 하세요. 더 오래 들어야 하는 운용이면 '
                'gripper_goal_pwm 을 낮추고 손가락 마찰 패드로 보강하세요 — 미끄럼 힘은 '
                'μ×법선력인데 PWM 은 법선력만 건드립니다.')

    def _begin_grip_hold(self):
        """파지 유지 시계 시작 (GRASP 성공 시점). 이미 재고 있으면 건드리지 않는다."""
        if self._grip_hold_start is None:
            self._grip_hold_start = self.get_clock().now()
            self._grip_hold_warned = False

    def _end_grip_hold(self):
        """파지 유지 시계 정지 — 그리퍼를 여는 모든 경로에서 부른다."""
        self._grip_hold_start = None
        self._grip_hold_warned = False
        self._grip_hold_target = None
        self._last_grip_refresh = None

    def _maintain_grip(self):
        """파지 중 닫힘 명령을 주기적으로 재발행해 **조이는 토크를 계속 유지**한다.

        다이나믹셀 위치제어는 Goal Position 이 남아 있는 동안 계속 미므로, 원리적으로는
        한 번만 보내도 토크가 유지된다. 문제는 그 전제("Goal Position 이 그대로 남아
        있다")가 실기에서 깨질 수 있다는 것이다:

          · 브릿지의 그리퍼 자동복구(`_recover_gripper_range`)는 Goal Position 을
            **열림 쪽으로 덮어쓴다.** 기동 시에만 돌지만, 파지 중에 조건이 걸리면
            조이던 목표가 통째로 사라진다.
          · 물체가 눌리거나 살짝 미끄러져 손가락이 더 들어갈 여지가 생겼을 때, 목표를
            다시 못 박아 주면 **그 여지만큼 느슨해진 채로** 남는다.
          · 액션 서버 경로라 goal 이 유실돼도(통신 실패 등) 아무도 재시도하지 않았다 —
            `_write_gripper` 의 write 실패는 warn 만 찍고 넘어간다.

        재발행 비용은 사실상 0 이고(같은 목표를 다시 쓰는 것뿐), **부하가 늘지 않으므로
        Overload 트립 위험도 그대로다** — 서보는 이미 그 목표를 향해 밀고 있었다.
        실패 모드를 여럿 지우면서 위험은 안 늘리므로 기본으로 켠다.

        ⚠️ 목표는 `_do_grasp` 가 실제로 보낸 값(`_grip_hold_target`)을 그대로 다시 쓴다.
        여기서 새로 계산하면 `_send_gripper` 의 끝단 clamp 와 어긋날 수 있고, 끝단
        너머로 밀면 하드스톱에 물려 되열리지도 못한다(그 실기 사고가 clamp 를 만든 이유다).
        """
        if self.grip_refresh_s <= 0.0 or self._grip_hold_target is None:
            return
        now = self.get_clock().now()
        if self._last_grip_refresh is not None:
            age = (now - self._last_grip_refresh).nanoseconds * 1e-9
            if age < self.grip_refresh_s:
                return
        self._last_grip_refresh = now
        self._send_gripper(self._grip_hold_target, refresh=True)

    def _send_gripper(self, position, refresh=False):
        """gripper_controller에 FollowJointTrajectory 단일 점 전송 (fire-and-forget).

        ⚠️ **캘리브된 개폐 끝단 밖으로는 명령하지 않는다** (2026-08-19 추가).
        `gripper_close - gripper_squeeze_rad` 는 완전닫힘(`gripper_close`=0.0)보다
        `gripper_squeeze_rad`(0.25rad) 만큼 **더 닫으라는** 값이라, tick 으로는
        -2036 이 나간다 — 캘리브된 완전닫힘 -1895 는 물론 손으로 잰 기구 한계
        -1933 도 넘는다. 실기에서 그리퍼가 하드스톱에 물려 멈췄고, Goal PWM 이
        280 으로 제한돼 있어 **되열리지도 못했다**(effort 316·velocity 0 으로 정지,
        하드웨어 에러는 안 뜸 → 원인을 못 찾기 쉽다).

        squeeze 는 원래 "접촉점보다 조금 더 눌러 힘을 준다" 는 뜻인데, 이 그리퍼는
        파지력이 Goal PWM 으로 정해지므로 목표를 더 밀어넣어도 힘이 늘지 않는다 —
        잃는 것 없이 끝단으로 자른다. squeeze 를 진짜로 쓰려면 `gripper_close` 를
        기구 끝단이 아니라 **접촉 위치**로 잡아야 한다.
        """
        lo, hi = sorted((self.gripper_close, self.gripper_open))
        clamped = max(lo, min(hi, float(position)))
        if abs(clamped - float(position)) > 1e-9:
            self.get_logger().warn(
                f'그리퍼 목표 {position:+.4f} rad 가 캘리브 범위 '
                f'[{lo:+.4f}, {hi:+.4f}] 밖이라 {clamped:+.4f} 로 자름')
        position = clamped
        if not refresh:
            # 이 값이 파지 유지 재발행(_maintain_grip)의 기준이 된다. clamp 를 **거친 뒤**
            # 저장해야 재발행이 끝단 밖으로 나가지 않는다.
            self._grip_hold_target = position
            self._last_grip_refresh = self.get_clock().now()
        # 여는 명령이면 파지 유지 시계를 멈춘다. 호출부(_do_release/_do_stowing)마다
        # 따로 부르지 않고 **이 funnel 한 곳**에서 처리한다 — 그리퍼를 여는 경로가
        # 나중에 늘어나도 시계가 안 멈춘 채 남는 일이 없다(멈추지 않으면 다음 파지가
        # 시작하자마자 경고를 띄운다).
        if abs(position - self.gripper_open) < abs(position - self.gripper_close):
            self._end_grip_hold()
        traj = JointTrajectory()
        traj.joint_names = self.gripper_joints
        pt = JointTrajectoryPoint()
        pt.positions = [float(position)] * len(self.gripper_joints)
        pt.time_from_start = Duration(sec=int(self.gripper_action_time))
        traj.points.append(pt)
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = traj
        if not self._gripper.server_is_ready():
            # 재발행 경로는 조용히 넘어간다 — 1초마다 같은 경고가 찍히면 진짜 문제가 묻힌다.
            if not refresh:
                self.get_logger().warn('gripper_controller 액션 서버 미준비')
            return
        self._gripper.send_goal_async(goal)

    def _on_gripper_goal_response(self, future):
        try:
            handle = future.result()
            if not handle.accepted:
                self._gripper_command_state = 'done'
                return
            handle.get_result_async().add_done_callback(self._on_gripper_result)
        except Exception:
            self._gripper_command_state = 'done'

    def _on_gripper_result(self, future):
        try:
            wrapped = future.result()
            self._gripper_command_ok = (
                wrapped.result.error_code ==
                FollowJointTrajectory.Result.SUCCESSFUL)
        except Exception:
            self._gripper_command_ok = False
        self._gripper_command_state = 'done'

    def _tool_effort(self):
        joints = self.tool_profile.get('joint_names', [])
        return max((self._joint_effort.get(name, 0.0) for name in joints),
                   default=0.0)

    def _publish_task_result(self, mission_id, success, state, reason):
        msg = TaskResult()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = mission_id
        msg.success = success
        msg.state = state
        msg.reason = reason
        self.pub_task_result.publish(msg)

    # ── 공통 ───────────────────────────────────

    def _elapsed(self):
        return (self.get_clock().now() - self._state_enter_t).nanoseconds * 1e-9

    def _gripper_reported(self):
        """그리퍼 관절이 /joint_states 에 **존재하는가**.

        존재하지 않는 것과 effort 가 0 인 것은 원인이 완전히 다른데 `_gripper_effort()`
        는 둘 다 0.0 으로 뭉갠다. 2026-08-12 실기에서 브릿지 둘이 같은 시리얼 포트를
        두드려 그리퍼 초기화만 `result=-3002` 로 실패했고(팔 축은 성공), 그리퍼가
        SyncRead 에서 빠져 /joint_states 에 아예 안 실렸다. 그때 FSM 메시지가
        "grasp effort 0.0 below threshold" 라 **약하게 물었다**로 읽혀서 파지력·
        캘리브를 한참 의심했다. 둘을 갈라서 보고한다.
        """
        return self.gripper_joints[0] in self._joint_effort

    def _on_grasp_failure(self):
        """향후 REGRASP 정책/상태를 추가하기 위한 단일 확장 지점."""
        self._fail(
            f'grasp effort {self._gripper_effort():.1f} below threshold '
            f'{self.grasp_thresh:.1f}')

    def _fail(self, reason):
        self.get_logger().error(reason)
        self._cancel_arm_motion()
        self._cancel_arm_test_motion()
        self._cancel_end_effector_motion()
        self._transition(State.FAILED)

    def _transition(self, new_state):
        pending_status = (self._pending_arrival.status
                          if self._pending_arrival is not None else None)
        arrival_mission_id = (self._pending_arrival.mission_id
                              if self._pending_arrival is not None else None)
        self.get_logger().info(
            f'{self.state.name} → {new_state.name}; '
            f'current_state={self.state.name}, chassis_mode={self._chassis_mode!r}, '
            f'pending_arrival_status={pending_status!r}, '
            f'arrival_mission_id={arrival_mission_id!r}, '
            f'current_mission_id={self.mission_id}, '
            f'last_completed_mission_id={self._last_completed_mission_id!r}')
        if new_state == State.STOWED_LOCKED and self.state != State.STOWED_LOCKED:
            self._last_completed_mission_id = self.mission_id
        self.state = new_state
        self._state_enter_t = self.get_clock().now()
        self._grip_sent = False
        self._stow_stage = 0
        self._stow_plan = None
        self._return_stage = 0
        self._carry_home_active = False

    def _set_status(self, status):
        """발행할 현재 상태를 갱신한다. 실제 발행은 _publish_heartbeat 가 전담한다.

        여기서 직접 publish 하지 말 것 — 발행 경로가 둘이 되면 stamp 순서가 뒤집힐 수
        있고, 파워트레인은 stamp 역행을 영구 latch 로 처벌한다(contract.py 참고).
        """
        self._status = status

    def _publish_heartbeat(self):
        """계약 heartbeat — 현재 상태를 10Hz 로 발행. **유일한 /arm_status 발행 지점.**"""
        msg = ArmStatus()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.mission_id = self.mission_id
        msg.status = self._status
        self.pub_status.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArmFsmNode()
    # heartbeat 타이머가 _tick(analytic IK 의 FK 블로킹 대기)에 굶지 않도록 멀티스레드.
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
