#!/usr/bin/env python3
"""원격조종 공통 코어 노드.

입력 장치(키보드/게임패드/네트워크 등)와 무관하게 동작하는 '두뇌'.
프론트엔드는 표준 control_msgs/JointJog 로 '어느 관절을 얼마나' 만 알려주고,
여기서 현재값 기준 적분·소프트리밋·데드맨·rad↔tick 변환을 모두 처리한 뒤
position_node 가 이해하는 /dynamixel/goal_position ([id, tick]) 로 내보낸다.

토픽
  구독 /arm/teleop_jog   (control_msgs/JointJog)   : 연속 이동 의도
         - displacements : 즉시 위치 증분 (단위: jog_step_rad 배수). 키보드가 쓴다.
         - velocities    : **rad/s**. 타이머가 적분한다. 조이스틱 같은 아날로그 입력이 쓴다.
  구독 /arm/teleop_cmd   (std_msgs/String)         : 이산 명령
         - "stop"            : 전 관절(그리퍼 포함) **즉시** 정지 + 토크 즉시 차단
                                (2026-08-02 변경 — 지연 없는 E-stop 성격). 재개는 "resume".
         - "resume"           : stop 으로 끊은 전 관절 토크를 그 자리에서(현재 위치
                                홀드) 복귀시킨다.
         - "home"             : 저장된 자세 "home" 으로 이동 (= "goto home" 의 축약형)
         - "save <이름>"      : 팔 관절(그리퍼+arm_joint_5 제외, _pose_joint_names 참고)
                                현재 측정 각도를 <이름> 으로 저장
         - "goto <이름>"      : 저장된 자세로 팔 관절만 이동 (그리퍼·arm_joint_5 는 안 건드림)
         - "poses"            : 저장된 자세 이름 목록을 /arm/teleop_poses 로 재발행
         - "reboot <이름|all>": 과부하 등으로 트립된 서보 reboot. <이름> 은 관절 하나
                                (예 arm_joint_2), "all" 은 현재 하드웨어 에러가 latch 된
                                서보 전체 — 관절 이름→모터 ID 변환만 여기서 하고 실제
                                reboot 는 position_node(/dynamixel/reboot_request) 가 수행.
         - "freedrive"        : 팔 관절(그리퍼+arm_joint_5 제외) 토크 OFF — 손으로 자세를 잡을 수 있게.
         - "freedrive_cancel" : freedrive 를 저장 없이 취소 — 팔 관절 토크 복귀(현재 자세 홀드).
         - "calib_start"      : 리미트 측정 모드 시작 — 팔 관절 전체 토크 OFF, 첫 축의
                                하한선 측정 대기 상태로 진입.
         - "calib_mark"       : 현재 측정 각도를 "현재 축의 현재 단계(하한/상한)" 값으로
                                기록하고 다음 단계(하한→상한→다음 축의 하한...)로 진행.
                                모든 축의 상/하한을 다 기록하면 자동으로 적용+저장하고
                                토크를 복귀시킨다.
         - "calib_cancel"     : 리미트 측정 모드를 저장 없이 취소 — 토크 복귀(현재 자세 홀드).
         - "spike on"/"spike off"/"spike <threshold>" : position_node 의 전류 급변
                                감지(비상정지) on/off·threshold 런타임 조정(2026-08-02
                                추가). /dynamixel/current_spike_config 로 그대로 전달.
         - "trip on"/"trip off"/"trip <threshold>" : position_node 의 절대 과전류
                                트립 on/off·threshold 런타임 조정(2026-08-02 추가,
                                spike 와 같은 형식). /dynamixel/current_trip_config 로 전달.
         - "limit on"/"limit off" : 관절 리밋(joint_limits) 마스터 스위치(2026-08-02
                                추가). off 는 소프트 clamp 와 position_node 의
                                tick_limits 방어선을 동시에 푼다(단, 다회전 축은
                                tick_limits 없인 안 움직여 여전히 안전 — _cmd_limit 참고).
  구독 /joint_states     (sensor_msgs/JointState)  : 현재 각도(시작 튐 방지·stop 기준·자세 저장 소스)
  발행 /dynamixel/goal_position (std_msgs/Int32MultiArray) [id, tick]
  발행 /arm/teleop_poses (std_msgs/String, transient_local) : 저장된 자세 이름 콤마 목록.
         프론트엔드가 몰라도 되지만(키보드 TUI는 화면 표시에 씀) 값의 단일 출처는 이 노드.
  발행 /arm/calib_status (std_msgs/String, transient_local) : 리미트 측정 모드 진행 상황.
         "idle" | "active,<축이름>,<lower|upper>,<진행축1based>,<전체축수>" | "done" | "cancelled".
         키보드 TUI가 그대로 파싱해서 배너로 보여준다(자세히는 아래 "리미트 측정 모드" 참고).
  발행 /dynamixel/reboot_request (std_msgs/Int32MultiArray) : reboot 대상 모터 ID 목록
         (빈 배열 = "all"). position_node 가 구독해서 실제로 reboot 한다.
         하드웨어 에러 자체(/dynamixel/hardware_error)는 position_node 가 직접 발행하며
         이 노드를 거치지 않는다 — 프론트엔드가 바로 구독한다.
  발행 /dynamixel/tick_limits (std_msgs/Int32MultiArray, transient_local) [id, min_tick,
         max_tick, id, min_tick, max_tick, ...] : joint_limits(rad)를 tick 으로 변환해
         position_node 에 전달(2026-08-02 추가) — position_node.goal_callback 의 최종
         방어선. 이 노드를 거치지 않고 /dynamixel/goal_position 에 직접 쏜 값도 막는다.
         boot 하한 캡처·리미트 측정 완료·기동 시마다 "지금 유효한 전체 목록"을 통째로
         재발행한다(증분 아님).
  발행 /dynamixel/torque_request (std_msgs/Int32MultiArray) [enable(0|1), id, id, ...] :
         position_node 가 구독해서 실제 토크 on/off 를 수행(2026-08-02 추가). "freedrive"/
         "freedrive_cancel"/저장 성공/"calib_*" 흐름이 이 노드가 팔 관절 ID로 채워서 내보낸다.
  발행 /dynamixel/profile_velocity_request (std_msgs/Int32MultiArray) [velocity, id, id, ...] :
         position_node 가 구독해서 Profile Velocity(112) 를 즉시 바꾼다(2026-08-02 추가) —
         조그는 jog_profile_velocity(빠름), 자세 불러오기(goto)는 goto_profile_velocity
         (느림)를 쓴다. self.profile_mode 로 관절별 "지금 어느 속도가 걸려있다고 보는지"를
         추적해 전환되는 순간에만 보낸다(매 조그 틱마다 다시 쓰지 않음).
  발행 /dynamixel/current_spike_config (std_msgs/Int32MultiArray) [enabled(0|1), threshold] :
         position_node 가 구독해서 전류 급변 감지 on/off·threshold 를 즉시 바꾼다
         (2026-08-02 추가). "spike on"/"off"/"<threshold>" 명령이 그대로 전달한다.
  발행 /dynamixel/current_trip_config (std_msgs/Int32MultiArray) [enabled(0|1), threshold] :
         position_node 가 구독해서 절대 과전류 트립 on/off·threshold 를 즉시 바꾼다
         (2026-08-02 추가, 형식은 위 current_spike_config 와 동일). "trip on"/"off"/
         "<threshold>" 명령이 그대로 전달한다.

프론트엔드가 무엇이든 이 노드는 그대로 재사용된다.

⚠️ **velocity 프론트엔드가 지켜야 할 것** — on_jog 는 **메시지에 실린 관절만** velocity 를
   갱신한다. 움직이는 관절만 골라 보내면 **놓은 관절이 마지막 속도로 계속 돈다.**
   매 발행마다 **전 관절을 joint_names 에 싣고 velocity 를 0 포함해 채울 것.**
   (deadman_timeout_s 가 최후의 안전망이지만, 그건 입력이 완전히 끊겼을 때만 작동한다.)

자세 저장/이동 (2026-08-01 추가, 2026-08-02 arm_joint_5 제외 추가)
  "save"/"goto" 는 `_pose_joint_names()`(그리퍼 + `POSE_EXCLUDED_NAMES`)를 제외한
  joint_names 만 다룬다 — 그리퍼는 항상 별도 조작이라는 요구사항 그대로. arm_joint_5
  는 자세 저장/불러오기 시 오작동이 보고돼(2026-08-02) 그리퍼와 동일하게 제외했다
  (jog 로 개별 조작하는 건 그대로 된다 — freedrive/save/goto/calib_* 에서만 빠진다).
  저장된 자세는 poses_file 파라미터 경로(JSON)에 영속화되어 노드를 재기동해도 남는다.

프리드라이브 저장 (2026-08-02 추가)
  "save"만으로는 팔이 토크 ON 상태라 손으로 자세를 잡을 수 없었다. 이제
  프론트엔드가 "save"를 보내기 *전에* "freedrive"를 먼저 보내 팔 관절 토크를
  끄고, 사용자가 손으로 자세를 잡은 뒤 "save <이름>"을 보내면 이 노드가
  현재 측정 각도를 저장함과 동시에 그 자세 그대로 토크를 다시 켠다(position_node
  가 켜기 직전 현재 위치를 goal_position 에 되써서 이전 목표로 튀지 않게 홀드—
  자세한 건 dynamixel_position_node.torque_request_callback 참고). 사용자가
  프롬프트를 취소하면 "freedrive_cancel"로 저장 없이 토크만 복귀시킨다.

리미트 측정 모드 (2026-08-02 추가)
  DEFAULT_FLOOR_FROM_BOOT(켰을 때 각도 하한)만으로는 상한이 실제로 있는 축(예:
  arm_joint_4)을 완전히 보호할 수 없다 — 위쪽은 여전히 무제한이었다. 그래서
  `_pose_joint_names()` 대상 축(그리퍼+arm_joint_5 제외)의 상/하한을 실측으로 직접
  캘리브레이션하는 모드를 추가했다:

    calib_start → (팔 관절 전체 토크 OFF) → [축0: 손으로 하한까지 이동 → calib_mark
    → 손으로 상한까지 이동 → calib_mark] → [축1: 반복] → ... → 마지막 축까지 끝나면
    자동으로 결과를 joint_limits(활성화, 실측 lower/upper)에 적용 + joint_limits_file
    에 영속화 + 토크 복귀(그 자리 홀드).

  중간에 calib_cancel 을 보내면 그때까지 기록한 값은 버리고 토크만 복귀한다(재시작은
  처음부터). joint_limits_file 에 저장된 축은 DEFAULT_FLOOR_FROM_BOOT 캡처보다
  우선한다 — 재기동해도 실측 상/하한이 그대로 유지된다(floor_from_boot_names 에서
  제외해서 boot 캡처가 덮어쓰지 못하게 함, __init__ 참고).

즉시 정지(E-stop) + resume (2026-08-02 변경)
  예전 "stop"은 goal 을 현재 위치로 맞추기만 해서, 서보의 Profile Accel/Velocity
  감속 시간만큼(수백 ms) 밀리는 게 "즉시"가 아니었다 — 사용자 보고로 진짜
  지연 없는 정지로 바꿨다: **토크부터 먼저** 끊고(물리적으로 그 순간 멈춤),
  그 다음에 goal=현재위치/velocity=0 을 맞춘다(내부 상태 일관성 + 나중에 resume
  할 때 안 튀게 하기 위함일 뿐). 전 관절(그리퍼 포함) 대상이다 — 산업용 로봇의
  E-stop 과 같은 원리라 팔이 힘을 잃고 중력으로 처질 수 있다. 다시 움직이려면
  "resume"을 보내야 한다(그 자리에서 토크 복귀 — freedrive_cancel 과 같은 원리지만
  대상이 전 관절이라는 점이 다르다).

키보드 jog 는 매 발행마다 전 관절을 메시지에 채워야 한다 (2026-08-02 버그 수정)
  keyboard_teleop_node 가 예전엔 선택된 관절 하나만 담아서 jog 를 보냈다 — 위
  "velocity 프론트엔드가 지켜야 할 것" 경고를 몰랐거나 displacement 만 쓰니
  상관없다고 생각한 것. 그런데 다른 경로(조이스틱 세션 등)로 어쩌다 다른 관절에
  nonzero velocity 가 걸리면, 키보드로 그 관절을 한 번도 안 건드리는 한 **영원히
  0 으로 안 돌아와서** "명령도 안 냈는데 축이 계속 움직인다"/"키를 떼도 계속
  움직인다" 는 사고가 났다(2026-08-02 실사용 보고로 발견). keyboard_teleop_node
  가 이제 joystick_teleop_node 와 동일하게 매 jog 발행마다 **전 관절**을 싣고
  선택 안 된 관절은 displacement=0/velocity=0 으로 명시한다 — 이 노드가 뭔가를
  보낼 때마다 stray velocity 를 스스로 확실히 0 으로 되돌린다.
"""

import json
import math
import os
import re

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Int32MultiArray, String
from sensor_msgs.msg import JointState
from control_msgs.msg import JointJog

from dynamixel_control.gripper_presets import DEFAULT_GRIPPER, get_preset


#: 저장된 자세 목록 — 늦게 붙는 구독자(키보드 TUI 재시작 등)도 마지막 값을 바로 받도록
#: transient_local. 발행 빈도가 낮아(자세 변경 시에만) depth 1로 충분하다.
POSES_LIST_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)


TICKS_PER_RAD = 4096.0 / (2.0 * math.pi)
DXL_MIN_TICK = 0
DXL_MAX_TICK = 4095
# X 시리즈 Extended Position Control Mode 의 raw 범위(대략 ±256회전) — 감속기
# 달린 축(EXTENDED_POSITION_NAMES) 전용. 단일회전 축의 0~4095 캡과 달리 이건
# "하드웨어가 낼 수 있는 절대 바깥 한계" 일 뿐이고, 실제 안전 범위는 언제나
# joint_limits(_clamp_rad)/tick_limits 가 담당한다 — 이 상수 자체는 안전장치가
# 아니다(dynamixel_position_node.py 의 동일 개념 주석 참고).
DXL_EXTENDED_MIN_TICK = -1_048_575
DXL_EXTENDED_MAX_TICK = 1_048_575

# 감속기(기어박스)가 달려서 서보 자신은 여러 바퀴 돌아야 관절이 그 비율만큼만
# 도는 축. **2026-08-07 실측(scripts/measure_gear_ratio.py): arm_joint_2 = 9.034:1,
# arm_joint_3 = 4.040:1 — 두 축의 감속비는 서로 다르다**(그 전까지 이 주석들은 둘 다
# "약 10:1" 로 추정했으나 실측으로 뒤집혔다). 이 노드 자체는 기어비를 쓰지 않는다 —
# 아래 rad 값은 전부 **서보축 각도**다. 관절 각도로 환산하는 건 moveit_dynamixel_bridge
# 의 JOINT_CONFIG[*]["gear_ratio"] 쪽이고, 값의 단일 출처도 거기다. dynamixel_position_node
# 의 extended_position_ids 파라미터(기본 [14,13,3], 아래 참고)와 짝 — 바뀌면 여기도
# 같이 바꿀 것.
#
# ⚠️ 그리퍼(gripper_left_pinion_joint, ID3)도 2026-08-02 실기로 추가: 완전 오픈
# (~+178.7°)과 완전 클로즈(~-178°대, 사용자 확인상 더 닫힘) 사이 실제 필요
# 스트로크가 거의 360°(랙피니언 스트로크가 피니언 한 바퀴에 거의 맞먹음)라,
# 단일회전(0~4095 tick)의 wrap 경계(±π)가 실사용 범위 한가운데 걸려 양쪽 끝이
# 소프트웨어 tick 한계(0 또는 4095)에 막혀버렸다(토크를 올려도 goal 자체가
# 그 이상 안 나가서 해결 안 됨 — 실기로 확인, current 는 평평하게 유지되고
# 계속 오르지 않았다). 다회전으로 바꾸면 이 wrap 경계 문제 자체가 사라진다
# — recenter 로는 못 고친다(양쪽에 남는 여유가 총 3°대뿐이라 재배치해도 어차피
# 어느 한쪽은 다시 타이트해짐).
EXTENDED_POSITION_NAMES = {"arm_joint_2", "arm_joint_3", "gripper_left_pinion_joint"}

# joint_names/motor_ids/centers/directions/limit* 는 전부 병렬 배열이다.
#
# ⚠️ 2026-08-02 그리퍼 단일 모터로 전환: 원래 그리퍼는 물리 서보 2개(id 3,4)를
# 랙피니언 한 레일에 물려 "이름을 두 번 등록 + 항상 같은 tick 동시 발행"으로
# 구동했는데(position_node 와 동일 전략), 실기 테스트 결과 두 독립 위치제어
# 루프가 같은 강체 레일을 밀다 보니 미세하게 어긋나며 서로 힘을 겨뤄(전류가
# 계속 상승) current_trip_threshold 를 트립시키고, 트립 후엔 goal_position 은
# 계속 갱신되는데도 두 모터 다 velocity/current=0(하드웨어 에러 없음)으로 응답을
# 멈추는 증상이 재현됨 — 손으로 돌려보면 걸리는 곳 없이 정상이라 기구적 문제가
# 아니라 "두 서보가 서로 버티는" 문제로 확인(사용자 확인). 레일에는 피니언이
# 2개 물려있어도 하나만 구동해도 나머지는 자유롭게 딸려 돈다 — 그래서 ID4는
# 아예 명령 대상에서 뺐다(토크도 안 걸림 → 자유회전, 레일에 저항 안 준다).
# 다시 2모터 동시구동을 시도한다면 마스터-팔로워 방식(하나는 위치제어, 하나는
# 그 위치를 실측 추종하는 토크/전류 제어)처럼 서로 안 겨루는 구조가 필요하다.
DEFAULT_JOINT_NAMES = [
    "arm_joint_1", "arm_joint_2", "arm_joint_3", "arm_joint_4", "arm_joint_5",
    "gripper_left_pinion_joint",
]

# 실기 버스 스캔값 (2026-08-01, 사용자 재확인). arm_joint_1(ID 11)은 현재 미연결.
# 그리퍼는 ID 3 하나만 구동(위 주석 참고, ID 4는 미등록 → 자유회전).
DEFAULT_MOTOR_IDS = [11, 14, 13, 12, 16, 3]
# ⚠️ **2048 을 실측 center 로 바꾸지 말 것** — 2026-08-19 에 시도했다가 되돌렸다.
# 이 노드의 measured_rad 는 position_node 가 발행한 /joint_states 를 그대로 받는데,
# position_node 는 `rad = (tick - 2048)*2π/4096` 으로 **2048 을 하드코딩**한다
# (dynamixel_position_node.py 의 "2048을 중앙, 한 바퀴를 2pi로 가정" 참고).
# 여기 center 만 실측값으로 바꾸면 **읽기는 2048 도메인, 쓰기(_rad_to_tick)는 실측
# 도메인**이 되어 왕복이 깨진다 — goal 을 측정값으로 시드하는 _ensure_goal 이
# 곧바로 1500 tick 넘게 어긋난 tick 을 내보낸다(arm_joint_2 기준 ≈19° 관절 점프).
# 바꾸려면 position_node 의 환산까지 같이 고쳐야 한다.
DEFAULT_CENTERS = [2048] * 6
# arm_joint_2 는 2026-08-01 사용자 요청으로 방향 반전(-1). direction 은 tick 변환
# (_rad_to_tick)에만 쓰이던 값이라, 읽기 쪽(on_joint_states/_publish_sim_joint_states)도
# 같은 direction 을 곱해 goal_rad 와 같은 '명령 도메인'으로 맞춰줬다 — 안 그러면
# stop/자세 저장·이동이 center(2048) 밖에서 순간적으로 튄다.
DEFAULT_DIRECTIONS = [1, -1, 1, 1, 1, 1]

# 그리퍼(마지막 축)의 리밋은 gripper_presets 의 **실측 tick** 에서 유도한다 —
# 숫자를 여기 따로 적어두면 캘리브를 다시 했을 때 조용히 어긋난다.
# 이 노드의 rad 도메인은 center=2048·direction=+1 기준이라 변환은 단순하다.
_GRIPPER_PRESET = get_preset(DEFAULT_GRIPPER)
_GRIPPER_MIN_RAD, _GRIPPER_MAX_RAD = sorted(
    (tick - 2048) / TICKS_PER_RAD
    for tick in (_GRIPPER_PRESET["gripper_open_tick"],
                 _GRIPPER_PRESET["gripper_close_tick"])
)

# 소프트리밋 기본 OFF — 모터가 아직 본체에 장착되지 않았다(통신 파이프라인 점검용).
# 장착 후에는 URDF(robot_arm.urdf)의 관절 리밋에 맞춰 반드시 다시 켤 것.
# 리밋을 꺼도 tick 은 0~4095(단일회전)/±약256회전(다회전, EXTENDED_POSITION_NAMES)
# 로 clamp 되므로 서보 자체 범위는 넘지 않는다.
#
# arm_joint_2/3/4 는 예외 — 2026-08-02 실기 리미트 측정 모드로 실측한 양쪽 상/하한을
# 코드 기본값으로 반영했다(joint_limits_file 이 없는 첫 기동에도 안전하게).
# arm_joint_4 측정값은 URDF(robot_arm.urdf) 의 CAD 리밋(lower=-0.610865,
# upper=0.698132)과도 근사해 신뢰할 만하다. arm_joint_2/3 는 처음엔 ±π 를 훨씬
# 넘는 값(단일회전으로는 불가능)이라 거부됐었는데, 사용자 확인 결과 이 두 축엔
# 감속기(2026-08-07 실측 9.034:1 / 4.040:1)가 달려있어 서보 자신이 여러 바퀴 도는 게 정상 설계였다
# — EXTENDED_POSITION_NAMES/OP_MODE_EXTENDED_POSITION 로 다회전을 정식 지원하고
# CALIB_SANE_RAD_LIMIT_EXTENDED 로 재검증을 통과해 반영했다(같은 세션에 함께
# 측정된 arm_joint_5 는 폭 0(하한/상한을 실제로 안 움직임)이라 여전히 반영 안 함
# — CALIB_MIN_RANGE_RAD 주석 참고).
#
# 🔁 **2026-08-19 재조립 후 전면 교체.** 위 DEFAULT_CENTERS 를 실측값으로 바꿨으므로
# 이 값들도 같은 도메인으로 다시 표현해야 한다. 옛 값은 center=2048 가정 위에서
# 2026-08-02 에 잰 것이라 그대로 두면 도메인이 어긋난다.
#
# ⚠️ joint_limits.py 에서 **복사한 게 아니라 환산**한 값이다(그쪽은 관절 도메인,
#    여기는 center=2048 기준 서보축 도메인). 변환식:
#        tick       = JOINT_CONFIG.center + direction_jc × 관절rad × TPR × gear_ratio
#        서보축 rad = (tick - 2048) / (direction_tc × TPR)
#    joint_limits.py 나 JOINT_CONFIG 의 center 가 바뀌면 **여기도 다시 환산**할 것.
#    숫자를 그냥 옮겨 적으면 감속기 축에서 9배, 영점 차이만큼 또 한 번 틀린다.
#
# ⚠️ arm_joint_5 의 하한은 환산값 -3.6605 가 아니라 **-π 로 잘랐다.** 원래 값은
#    tick -338 로 단일회전 범위(0~4095) 밖이고, CALIB_SANE_RAD_LIMIT(π·1.05)도
#    넘어 _validate_calib_bounds 에 거부된다. -π = tick 0 이 이 축이 Position
#    모드에서 실제로 갈 수 있는 끝이다 — 실측 범위의 아래쪽 일부(약 30%)는
#    도달 불가다. 다 쓰려면 이 축도 Extended Position 으로 돌려야 한다.
#
# 🔁 **2026-08-19: 그리퍼 리밋을 켰다(구 False).** 끄면 안 움직인다 —
# _publish_tick_limits 가 enabled=False 인 축을 빼는데, position_node 는
# **다회전(Extended) 축에 tick_limits 가 없으면 그 목표를 통째로 거부**한다
# ("ID 3 는 다회전 축인데 tick_limits 가 아직 없습니다 — 이 목표는 무시합니다").
# 즉 리밋을 끄는 것이 곧 그리퍼를 못 쓰게 만드는 교착이었다. 실기에서 GUI 로
# 그리퍼만 안 움직이는 증상으로 나타났다(2026-08-19).
# 리밋 값은 gripper_presets 의 실측 tick 에서 유도하므로 캘리브와 항상 붙어 있다
# — 덤으로 하드스톱을 밀어 과부하 트립 나는 것도 막는다.
DEFAULT_LIMIT_ENABLED = [True, True, True, True, True, True]
DEFAULT_MIN_RADS = [-0.199779, 2.365398, -0.593806, -1.893259, -math.pi,
                    _GRIPPER_MIN_RAD]
DEFAULT_MAX_RADS = [0.301021, 15.262337, 11.240970, 0.762741, -0.617856,
                    _GRIPPER_MAX_RAD]

# 팔 관절(arm_joint_1..5) 전부 "켰을 때 각도보다 아래(키보드 ↓/s 방향)로 안 가게"
# 처리한다 — 처음엔 arm_joint_2/3만 예외 처리했다가(2026-08-02), 사용자 요청으로
# 나머지 축까지 전부 확장(2026-08-02). rad=0 을 하한으로 박아두면 안 된다 — 실기
# 로그(2026-08-01 21:02 KST 세션, /root/.ros/log/python3_1100_1785585746362.log)를
# 보면 전원 인가 직후(런치 3초 후, 아직 아무도 조그하지 않은 시점) 측정값이
# arm_joint_2=-0.331, arm_joint_3=-0.259 로 이미 0이 아니었다 — 즉 이 관절들은
# rad=0 이 기구적 home과 안 맞는다(장착 오프셋). min_rad=0 으로 고정했다면 첫
# 조그 명령에서 곧장 0 으로 튀는 사고가 났을 것이다. 그래서 정적 상수 대신,
# 매 세션 **첫 /joint_states 로 실측된 값**을 하한으로 그때그때 캡처한다
# (on_joint_states 의 캡처 로직 참고) — 위(↑/w, 양의 방향)는 그대로 자유.
#
# ⚠️ 이건 "아래로만" 막는 한쪽 리밋이다. arm_joint_1/5처럼 상한이 아직 실측 안 된
# 축은 이걸로 완전히 안전하지 않다 — 위쪽은 여전히 무제한으로 열려있다. 상한까지
# 막으려면 joint_min_rads/joint_max_rads/joint_limit_enabled 로 그 축의 실측
# 상/하한값(리미트 측정 모드로 캡처)을 직접 채워야 한다.
#
# 그리퍼는 열림/닫힘 tick 캘리브레이션(gripper_presets.py)이 따로 있고 "home 아래
# 금지" 개념 자체가 안 맞아서 제외한다. arm_joint_2/3/4 도 제외 — 위 DEFAULT_MIN_RADS/
# MAX_RADS 의 실측 양쪽 리밋을 쓰므로 boot 편측 캡처가 필요 없다(오히려 캡처가
# 실행되면 하한이 boot 값으로 덮어써져 실측 하한을 잃는다 — __init__ 의
# floor_from_boot_names 처리 참고).
# joint_names 파라미터와 병렬 배열.
DEFAULT_FLOOR_FROM_BOOT = [True, False, False, False, True, False]

# 리미트 측정 모드(calib_*) 결과 안전성 검사 (2026-08-02 추가, 같은 날 기어비
# 확인으로 갱신) — 실사용 중 arm_joint_2/3 가 ±π 를 훨씬 넘는 값(예: 9.2rad)으로,
# arm_joint_5 가 사실상 폭 0(0.005rad)으로 기록되는 사고가 있었다. 처음엔 전자를
# "단일회전 joint 가 다회전 모드로 잘못 남은 사고"로 의심했는데, 사용자 확인
# 결과 arm_joint_2/3 에는 실제로 감속기(2026-08-07 실측 9.034:1 / 4.040:1)가 달려있어서 서보
# 자신이 여러 바퀴 돌아야 관절이 그 비율만큼만 도는 게 **정상 설계**였다 —
# 그래서 이 두 축은 EXTENDED_POSITION_NAMES 로 등록해 다회전을 정식 지원하고
# (dynamixel_position_node.py 의 OP_MODE_EXTENDED_POSITION 참고), 이 축들만 훨씬
# 넓은 sane 범위(기어비 불확실성 감안한 여유 포함)를 쓴다. arm_joint_5 의 폭 0
# 사고는 기어비와 무관하게 여전히 유효한 문제(하한/상한을 실제로 안 움직이고
# 거의 같은 자리에서 두 번 마크한 경우)라 그대로 둔다. 그대로 적용하면 goto/
# 조그가 엉뚱한 tick 으로 clamp 되거나 그 축이 사실상 못 움직이는 결과로 이어진다
# — "자세 저장/불러오기 시 일부 관절만 이상하게 움직이는 것처럼 보이는 버그"의
# 실제 원인이었다(코드 버그가 아니라 잘못 기록된 캘리브레이션 데이터가 안전장치를
# 통해 정직하게 반영된 것). calib_mark 로 최종 확정하는 시점과 joint_limits_file
# 을 로드하는 시점 둘 다에서 이 조건들을 검사해 이상하면 적용을 거부하고 로그로
# 알린다.
CALIB_SANE_RAD_LIMIT = math.pi * 1.05        # 단일회전 축 — 이보다 크면 물리적으로 불가능
# 기어비가 축마다 다르고(2026-08-07 실측 9.034:1 / 4.040:1) 앞으로 또 바뀔 수 있어
# 여유를 넉넉히 뒀다 — 기어비가 달라져도 정상 측정을 오탐 거부하지 않도록.
CALIB_SANE_RAD_LIMIT_EXTENDED = math.pi * 15  # 감속기 축(EXTENDED_POSITION_NAMES)
CALIB_MIN_RANGE_RAD = 0.05              # 이보다 좁으면 하한/상한을 실제로 안 움직인 것으로 간주


def _validate_calib_bounds(name, lower, upper):
    """(ok, reason) — 리미트가 물리적으로 말이 되는지 검사. ok=False 면 적용 거부."""
    sane_limit = (
        CALIB_SANE_RAD_LIMIT_EXTENDED if name in EXTENDED_POSITION_NAMES
        else CALIB_SANE_RAD_LIMIT
    )
    if abs(lower) > sane_limit or abs(upper) > sane_limit:
        reason_extra = (
            "감속기 축인데도 이 범위를 넘습니다 — 기어비가 실측값(arm_joint_2=9.034:1, "
            "arm_joint_3=4.040:1)과 많이 다르거나 캘리브레이션이 잘못됐을 수 있습니다."
            if name in EXTENDED_POSITION_NAMES else
            "서보가 Extended Position Control Mode(다회전)로 남아있을 가능성이 "
            "높습니다. operating mode(force_position_mode) 확인 후 재측정하세요."
        )
        return False, (
            f"[{lower:+.3f},{upper:+.3f}] rad 가 sane 범위(±{sane_limit:.2f}) 밖입니다 — "
            f"{reason_extra}"
        )
    if (upper - lower) < CALIB_MIN_RANGE_RAD:
        return False, (
            f"[{lower:+.3f},{upper:+.3f}] rad 폭이 {upper - lower:.4f}rad 밖에 안 됩니다 — "
            "하한/상한 측정 시 실제로 손으로 움직이지 않은 것 같습니다. 재측정하세요."
        )
    return True, ""


class TeleopCore(Node):
    def __init__(self):
        super().__init__("teleop_core")

        # --- 관절 ↔ 모터 매핑 ---
        self.declare_parameter("joint_names", DEFAULT_JOINT_NAMES)
        self.declare_parameter("motor_ids", DEFAULT_MOTOR_IDS)
        self.declare_parameter("centers", DEFAULT_CENTERS)
        self.declare_parameter("directions", DEFAULT_DIRECTIONS)

        # --- 동작 파라미터 ---
        self.declare_parameter("jog_step_rad", 0.05)      # displacement=±1 당 이동량
        self.declare_parameter("joint_min_rad", -math.pi)  # 호환용 fallback 공통 리밋
        self.declare_parameter("joint_max_rad", math.pi)
        self.declare_parameter("joint_limit_enabled", DEFAULT_LIMIT_ENABLED)
        self.declare_parameter("joint_min_rads", DEFAULT_MIN_RADS)
        self.declare_parameter("joint_max_rads", DEFAULT_MAX_RADS)
        # 켜진 관절은 min_rads 대신 "첫 /joint_states 로 실측된 각도"를 하한으로
        # 그때그때 캡처해서 쓴다(DEFAULT_FLOOR_FROM_BOOT 위 주석 참고).
        self.declare_parameter("joint_limit_floor_from_boot", DEFAULT_FLOOR_FROM_BOOT)
        self.declare_parameter("deadman_timeout_s", 0.5)  # 이 시간 넘게 입력 없으면 velocity 적분 정지
        # velocity 적분 주기 = 목표 갱신 주기. 낮으면 한 번에 크게 뛰어 서보가
        # 매번 프로파일을 새로 그리며 틱틱 끊긴다 — 잘게 자주 보내는 편이 매끄럽다.
        # 2026-08-01: 지연 최소화 요청으로 50→100Hz 상향(적분 주기가 짧아져 조이스틱
        # 연속 입력의 목표 갱신이 더 촘촘해짐).
        self.declare_parameter("publish_rate_hz", 100.0)
        # JointJog.velocities 의 안전 상한 [rad/s]. 프론트엔드 버그로 큰 값이 들어와도
        # 팔이 튀지 않도록 on_jog 에서 clamp 한다.
        # 2026-08-01: 사용자 요청으로 속도 절반(1.0→0.5) — 조이스틱 연속 조그 상한.
        # 2026-08-02: "위자드처럼 지연 없이" 요청 + 이후 추가된 안전장치(관절
        # 리밋 검증, tick_limits 최종 방어선, 전류 트립, 즉시정지 E-stop)를
        # 감안해 다시 1.0으로 복귀 — 처음 절반으로 줄였을 때는 이런 안전장치가
        # 없었다. 프론트엔드가 실제로 요청하는 속도(키보드는
        # jog_velocity_rad_s)는 여전히 이보다 낮게 잡혀있으니 이건 "상한"일 뿐,
        # 기본 체감 속도가 갑자기 2배 되는 게 아니다.
        self.declare_parameter("max_vel_rad_s", 1.0)
        # Profile Velocity(raw, position_node 의 동일 파라미터와 같은 단위) —
        # 조그와 자세 불러오기(goto)가 서로 다른 값을 쓴다(2026-08-02 추가).
        # jog_profile_velocity 는 position_node 의 기본값(50)과 맞춰뒀다 —
        # 이게 달라지면 조그 시작할 때 첫 틱만 속도가 안 맞는 걸로 보일 수 있다.
        # goto_profile_velocity 는 "자세 불러오기가 너무 빠르다" 는 요청으로
        # 그보다 훨씬 느리게(사용자가 이전에 괜찮다고 했던 값 20) 잡았다.
        self.declare_parameter("jog_profile_velocity", 50)
        self.declare_parameter("goto_profile_velocity", 20)
        # 하드웨어 없이 RViz 디버그: 목표값을 그대로 /joint_states 로도 발행(open-loop).
        # 실서보 사용 시엔 position_node 가 실제 /joint_states 를 내므로 반드시 false.
        self.declare_parameter("publish_sim_joint_states", False)
        # 저장 자세 영속화 파일. 기본값은 bind-mount 된 ws 루트(호스트에서도 보임,
        # src/ 밖이라 git 추적 대상 아님 — .gitignore 에 패턴 추가돼 있다).
        self.declare_parameter(
            "poses_file",
            os.path.join(os.path.expanduser("~"), "ros2_ws", "dynamixel_arm_poses.json"),
        )
        # 리미트 측정 모드(calib_*)로 실측한 축별 상/하한 영속화 파일. poses_file 과
        # 같은 이유로 src/ 밖에 둔다 — .gitignore 패턴에 포함.
        self.declare_parameter(
            "joint_limits_file",
            os.path.join(os.path.expanduser("~"), "ros2_ws", "dynamixel_arm_joint_limits.json"),
        )

        # raw_* 는 파라미터 그대로(그리퍼처럼 이름이 두 번 나올 수 있는 병렬 배열).
        # self.joint_names 는 이걸 이름 기준으로 dedupe 한 것 — 이후 모든 로직
        # (goal_rad/measured_rad/velocity/발행 루프)은 이 유일 목록만 쓴다.
        raw_joint_names = list(self.get_parameter("joint_names").value)
        motor_ids = list(self.get_parameter("motor_ids").value)
        centers = list(self.get_parameter("centers").value)
        directions = list(self.get_parameter("directions").value)
        limit_enabled = list(self.get_parameter("joint_limit_enabled").value)
        min_rads = list(self.get_parameter("joint_min_rads").value)
        max_rads = list(self.get_parameter("joint_max_rads").value)
        floor_from_boot = list(self.get_parameter("joint_limit_floor_from_boot").value)

        if not (len(raw_joint_names) == len(motor_ids) == len(centers) == len(directions)):
            raise RuntimeError(
                "joint_names/motor_ids/centers/directions 길이가 서로 다릅니다"
            )

        self.cfg = {}
        for i, name in enumerate(raw_joint_names):
            entry = self.cfg.setdefault(
                name, {"ids": [], "center": int(centers[i]), "direction": int(directions[i])})
            entry["ids"].append(int(motor_ids[i]))

        self.joint_names = list(dict.fromkeys(raw_joint_names))

        self.jog_step_rad = float(self.get_parameter("jog_step_rad").value)
        self.joint_min_rad = float(self.get_parameter("joint_min_rad").value)
        self.joint_max_rad = float(self.get_parameter("joint_max_rad").value)
        self.joint_limits = self._build_joint_limits(
            raw_joint_names, limit_enabled, min_rads, max_rads)

        # floor_from_boot 관절: 캡처 전까지는 하한을 강제하지 않는다(첫
        # /joint_states 가 아직 없어 "켰을 때 각도"를 모르므로) — 캡처되면
        # on_joint_states 가 self.joint_limits[name] 을 (True, 실측값, upper) 로
        # 덮어쓴다. 상한은 min_rads/joint_limit_enabled 설정과 무관하게 그대로 둔다.
        if len(floor_from_boot) != len(raw_joint_names):
            self.get_logger().warn(
                "joint_limit_floor_from_boot 길이가 joint_names와 맞지 않아 무시합니다")
            floor_from_boot = [False] * len(raw_joint_names)
        self.floor_from_boot_names = {
            name for name, flag in zip(raw_joint_names, floor_from_boot) if bool(flag)
        }
        self.boot_floor_captured = set()
        for name in self.floor_from_boot_names:
            _enabled, _lower, upper = self.joint_limits[name]
            self.joint_limits[name] = (False, -math.pi, upper)

        # calib_* (리미트 측정 모드)로 실측한 상/하한을 로드한다 — boot_floor 보다
        # 우선한다(정밀한 양쪽 실측값이 있는데 편측 boot 캡처가 나중에 덮어쓰면 안 됨).
        # 그래서 로드된 축은 floor_from_boot_names 에서 아예 제외한다.
        self.joint_limits_file = os.path.expanduser(
            str(self.get_parameter("joint_limits_file").value))
        loaded_limits = self._load_joint_limits_file()
        self.calibrated_joints = set()
        for name, bounds in loaded_limits.items():
            if name not in self.joint_names:
                continue  # 파일에 남은 옛 관절 이름(구성 변경) — 무시
            lower, upper = bounds["lower"], bounds["upper"]
            if lower > upper:
                lower, upper = upper, lower
            # 파일에 저장된 값도 검증한다 — 코드가 바뀌기 전에 이미 이상한 값이
            # 저장된 파일일 수 있다(실사용 중 실제로 이런 파일이 있었다).
            ok, reason = _validate_calib_bounds(name, lower, upper)
            if not ok:
                self.get_logger().error(
                    f"joint_limits_file 의 {name} 값을 거부합니다 — {reason} "
                    "(무시하고 이 축은 boot 캡처/기본값으로 진행합니다)"
                )
                continue
            self.joint_limits[name] = (True, lower, upper)
            self.floor_from_boot_names.discard(name)
            self.calibrated_joints.add(name)

        self.deadman_timeout_s = float(self.get_parameter("deadman_timeout_s").value)
        self.publish_sim = bool(self.get_parameter("publish_sim_joint_states").value)
        self.max_vel_rad_s = abs(float(self.get_parameter("max_vel_rad_s").value))
        self.jog_profile_velocity = int(self.get_parameter("jog_profile_velocity").value)
        self.goto_profile_velocity = int(self.get_parameter("goto_profile_velocity").value)
        rate = float(self.get_parameter("publish_rate_hz").value)

        # 관절별 목표 각도(rad). None = 아직 초기화 안 됨(joint_states 대기).
        self.goal_rad = {name: None for name in self.joint_names}
        self.measured_rad = {name: None for name in self.joint_names}
        # velocity 모드용 상태
        self.velocity = {name: 0.0 for name in self.joint_names}
        self.last_input_time = self.get_clock().now()
        # 관절별 "지금 서보에 걸려있는(것으로 추정하는) profile velocity 모드".
        # position_node 기동 시 기본값이 jog_profile_velocity 이므로 "jog"로
        # 시작한다 — on_jog 는 "goto" 상태인 축만 골라 jog 속도로 되돌린다
        # (매 틱 다시 쓰면 트래픽 낭비라 전환 시점에만 쓴다, 아래 on_jog 참고).
        self.profile_mode = {name: "jog" for name in self.joint_names}

        # 관절 리밋(joint_limits) 전체를 켜고 끄는 마스터 스위치(2026-08-02 추가
        # — "리밋을 거는 모드/푸는 모드" 요청). False 면 각 관절의 개별
        # enabled 값과 무관하게 _clamp_rad 가 clamp 하지 않고, _publish_tick_limits
        # 도 position_node 에 빈 목록을 보낸다(방어선까지 같이 풀어야 실제로
        # 리밋 밖으로 움직인다 — 안 그러면 소프트만 풀리고 position_node 의
        # tick_limits 최종 방어선에 그대로 다시 clamp 당해 아무 effect 가 없다).
        # ⚠️ 다회전(Extended Position) 축은 position_node.goal_callback 이
        # tick_limits 가 아예 없으면 그 축의 목표를 통째로 거부하도록 설계돼
        # 있다(감속기 축을 무제한으로 돌리는 게 더 위험하다는 기존 판단) — 즉
        # "리밋 해제"는 단일회전 축(arm_joint_1/4/5, 그리퍼)만 실제로 자유로워
        # 지고, arm_joint_2/3 은 해제 중엔 오히려 안 움직인다(안전한 방향의
        # 부작용, 의도된 것).
        self.limits_master_enabled = True

        # 리미트 측정 모드(calib_*) 진행 상태. calib_active=False 면 나머지는 무의미.
        self.calib_active = False
        self.calib_joint_names = []   # 이번 세션에서 순서대로 측정할 축 목록
        self.calib_index = 0          # 현재 측정 중인 축의 인덱스
        self.calib_step = "lower"     # "lower" | "upper"
        self.calib_results = {}       # name -> {"lower": rad, "upper": rad} (진행 중 누적)

        self.sub_jog = self.create_subscription(
            JointJog, "/arm/teleop_jog", self.on_jog, 10)
        self.sub_cmd = self.create_subscription(
            String, "/arm/teleop_cmd", self.on_cmd, 10)
        self.sub_js = self.create_subscription(
            JointState, "/joint_states", self.on_joint_states, 10)
        # 2026-08-02 추가 — 과전류 트립(절대/급변) 직후 재개했더니 그리퍼가 트립
        # 전의 무리한 옛 목표로 다시 밀고 들어가 재트립되는 문제(사용자 실기
        # 재현) 대응. position_node 는 트립된 서보의 GOAL_POSITION 레지스터
        # 자체는 트립 지점으로 되돌리지만(dynamixel_position_node.py 참고),
        # 이 노드가 들고 있는 goal_rad 캐시는 별개라 안 고치면 다음 조그/타이머
        # 틱에 그 캐시값을 다시 내보내 버린다 — 그래서 하드웨어 에러 텍스트에
        # **새로** 나타나는 관절을 감지해 그 즉시 goal_rad 를 실측값으로 맞춘다
        # (resume 을 기다리지 않고 트립 나는 순간 바로).
        self.sub_hw_error = self.create_subscription(
            String, "/dynamixel/hardware_error", self.on_hardware_error, 10)
        self._last_hw_error_names = set()

        self.pub = self.create_publisher(
            Int32MultiArray, "/dynamixel/goal_position", 10)
        self.poses_pub = self.create_publisher(
            String, "/arm/teleop_poses", POSES_LIST_QOS)
        # 리미트 측정 모드 진행 상황 — 늦게 붙는 구독자도 마지막 상태를 바로 받도록
        # poses 와 같은 QoS(transient_local, depth 1).
        self.calib_status_pub = self.create_publisher(
            String, "/arm/calib_status", POSES_LIST_QOS)
        # joint_limits(rad) 를 tick 으로 변환해서 position_node 에 최종 방어선으로
        # 전달한다(2026-08-02 추가, "하드웨어 안 망가지게 최대한 안전하게" 요청) —
        # 이 노드를 거치지 않고 누가 /dynamixel/goal_position 에 직접 쏴도 position_node
        # 가 스스로 막게 하기 위함. transient_local: position_node 가 이 노드보다
        # 늦게/다시 뜨거나 재기동해도 마지막 리밋을 놓치지 않고 받는다.
        self.tick_limits_pub = self.create_publisher(
            Int32MultiArray, "/dynamixel/tick_limits", POSES_LIST_QOS)
        # "reboot <joint_name>" 요청을 관절 이름→모터 ID(self.cfg)로 풀어서
        # position_node 에 전달한다. 하드웨어 에러 표시 자체는 position_node 가
        # /dynamixel/hardware_error 로 직접 내보내므로 여기서 중계하지 않는다.
        self.reboot_pub = self.create_publisher(
            Int32MultiArray, "/dynamixel/reboot_request", 10)
        # "freedrive"/"freedrive_cancel"/저장 성공 시 팔 관절 토크 on/off 요청.
        # 실제 토크 제어는 position_node.torque_request_callback 이 수행한다.
        self.torque_pub = self.create_publisher(
            Int32MultiArray, "/dynamixel/torque_request", 10)
        # 조그/자세 불러오기(goto)가 서로 다른 Profile Velocity 를 쓰게 하는
        # 런타임 요청(2026-08-02 추가 — "조그는 위자드처럼 빠르게, goto 는
        # 너무 빠르니 느리게" 두 요구가 같은 레지스터 하나를 두고 충돌해서
        # 분리했다). on_jog/_cmd_goto_pose 에서 상황에 맞게 이 토픽으로
        # position_node 에 속도를 바꿔달라고 요청한다.
        self.profile_velocity_pub = self.create_publisher(
            Int32MultiArray, "/dynamixel/profile_velocity_request", 10)
        # 전류 급변 감지 on/off·threshold 런타임 조정(2026-08-02 추가 — 실서보
        # 전환 후 기동 가속 전류로 오탐이 나서, threshold 를 실기로 다시 잡는
        # 동안 기능을 잠깐 끌 수 있게 해달라는 요청). on_cmd 의 "spike" 액션
        # 참고. position_node 의 current_spike_config_callback 이 구독한다.
        self.spike_config_pub = self.create_publisher(
            Int32MultiArray, "/dynamixel/current_spike_config", 10)
        # 절대 과전류 트립 on/off·threshold 런타임 조정(2026-08-02 추가 — 그리퍼
        # 정상 파지 중에도 트립되는 게 실기로 재현돼, 급변 감지와 같은 방식으로
        # 만듦). on_cmd 의 "trip" 액션 참고. position_node 의
        # current_trip_config_callback 이 구독한다.
        self.trip_config_pub = self.create_publisher(
            Int32MultiArray, "/dynamixel/current_trip_config", 10)

        # 하드웨어 없이 RViz 디버그용 sim /joint_states 발행자
        self.js_pub = None
        if self.publish_sim:
            self.js_pub = self.create_publisher(JointState, "/joint_states", 10)

        self.timer = self.create_timer(1.0 / rate, self.on_timer)

        self.poses_file = os.path.expanduser(str(self.get_parameter("poses_file").value))
        self.poses = self._load_poses_file()
        self._publish_poses_list()
        self._publish_calib_status("idle")
        # joint_limits_file 에서 로드한(있다면) 리밋을 기동 즉시 position_node 에
        # 전달 — position_node 가 이 노드보다 먼저 뜨는 launch 순서라도 transient_local
        # QoS 라 놓치지 않는다.
        self._publish_tick_limits()

        self.get_logger().info(
            f"teleop_core started (joints={self.joint_names}, "
            f"jog_step={self.jog_step_rad} rad, limits={self._limit_summary()}, "
            f"sim_joint_states={self.publish_sim}, "
            f"poses_file={self.poses_file}, poses={sorted(self.poses)}, "
            f"joint_limits_file={self.joint_limits_file}, "
            f"calibrated_joints={sorted(self.calibrated_joints)})"
        )

    # ------------------------------------------------------------------ helpers
    def _build_joint_limits(self, raw_names, enabled, min_rads, max_rads):
        n = len(raw_names)
        if not (len(enabled) == len(min_rads) == len(max_rads) == n):
            self.get_logger().warn(
                "joint_limit_enabled/joint_min_rads/joint_max_rads 길이가 "
                "joint_names와 맞지 않아 공통 fallback 리밋을 사용합니다"
            )
            return {
                name: (True, self.joint_min_rad, self.joint_max_rad)
                for name in self.joint_names
            }

        # raw_names 에 그리퍼처럼 중복 이름이 있어도 값이 동일하므로 마지막 값으로
        # 덮어써도 무방하다 — 결과 딕셔너리는 self.joint_names(유일 목록) 기준으로 쓰인다.
        limits = {}
        for i, name in enumerate(raw_names):
            lower = float(min_rads[i])
            upper = float(max_rads[i])
            if lower > upper:
                lower, upper = upper, lower
            limits[name] = (bool(enabled[i]), lower, upper)
        return limits

    def _limit_summary(self):
        parts = []
        for name in self.joint_names:
            enabled, lower, upper = self.joint_limits[name]
            if name in self.floor_from_boot_names and name not in self.boot_floor_captured:
                parts.append(f"{name}=[boot시 캡처 예정,{upper:.3f}]")
            elif enabled:
                parts.append(f"{name}=[{lower:.3f},{upper:.3f}]")
            else:
                parts.append(f"{name}=off")
        return ", ".join(parts)

    def _physical_rad_range(self, name):
        """이 관절이 tick 으로 표현 가능한 rad 범위(물리적 한계) — center/direction
        기준으로 _rad_to_tick 의 tick clamp 를 rad 도메인으로 거꾸로 계산한다.
        """
        c = self.cfg[name]
        if name in EXTENDED_POSITION_NAMES:
            tick_lo, tick_hi = DXL_EXTENDED_MIN_TICK, DXL_EXTENDED_MAX_TICK
        else:
            tick_lo, tick_hi = DXL_MIN_TICK, DXL_MAX_TICK
        rad_a = (tick_lo - c["center"]) / (c["direction"] * TICKS_PER_RAD)
        rad_b = (tick_hi - c["center"]) / (c["direction"] * TICKS_PER_RAD)
        return min(rad_a, rad_b), max(rad_a, rad_b)

    def _clamp_rad(self, name, rad):
        # ⚠️ 2026-08-02 추가: 소프트리밋(joint_limits)/마스터 스위치와 무관하게
        # **항상** 적용한다 — 이건 "안전 리밋"이 아니라 "tick 으로 표현 가능한
        # 물리적 범위"라서 끌 수 있는 대상이 아니다. 이게 없으면(리밋이 꺼져있는
        # 그리퍼 등에서) 조그를 오래 누르고 있을 때 goal_rad 가 tick 표현 범위를
        # 한참 벗어나 누적되고, _rad_to_tick 은 tick 만 0/4095 로 clamp 할 뿐
        # goal_rad 자체는 안 줄어서 "명령은 계속 나가는데 화면상 전혀 안 움직이는"
        # 것처럼 보이는 버그가 났다(실기로 확인 — 벗어난 만큼 반대 방향으로 다시
        # 조그해야 반응이 왔다). 벗어났다면 여기서 즉시 물리 범위 안으로 되돌린다.
        phys_lo, phys_hi = self._physical_rad_range(name)
        rad = max(phys_lo, min(phys_hi, rad))

        if not self.limits_master_enabled:
            return rad
        enabled, lower, upper = self.joint_limits[name]
        if not enabled:
            return rad
        return max(lower, min(upper, rad))

    def _publish_tick_limits(self):
        """joint_limits(rad) 를 tick 으로 변환해 position_node 에 발행한다.

        position_node.goal_callback 의 최종 방어선(이 노드를 거치지 않은
        /dynamixel/goal_position 직접 발행도 막기 위함)의 입력값이다. 리밋이
        꺼져있는(enabled=False, 아직 boot 캡처 전 등) 관절은 여기서 빠진다 —
        position_node 는 그런 ID 에 대해 기존처럼 0~4095 전체 범위만 적용한다.
        매번 "지금 유효한 전체 목록"을 통째로 보낸다(증분 아님).

        limits_master_enabled=False 면 빈 목록을 보낸다 — position_node 쪽
        방어선도 같이 풀어야 소프트 clamp 해제가 실제로 의미가 있다(위
        limits_master_enabled 선언부 주석 참고).
        """
        if not self.limits_master_enabled:
            self.tick_limits_pub.publish(Int32MultiArray(data=[]))
            return
        flat = []
        for name in self.joint_names:
            enabled, lower, upper = self.joint_limits[name]
            if not enabled:
                continue
            tick_a = self._rad_to_tick(name, lower)
            tick_b = self._rad_to_tick(name, upper)
            lo, hi = min(tick_a, tick_b), max(tick_a, tick_b)
            for motor_id in self.cfg[name]["ids"]:
                flat.extend([motor_id, lo, hi])
        self.tick_limits_pub.publish(Int32MultiArray(data=flat))

    def _capture_boot_floor(self, name, rad):
        """전원 인가 후 처음 들어온 실측 각도를 그 관절의 하한으로 고정한다.

        이후 이 하한 밑(키보드 ↓/s 방향)으로는 goal_rad 가 못 내려간다 — 위(↑/w)는
        그대로 자유. 이미 잡혀있던 goal_rad(드물게 첫 joint_states 보다 먼저 조그가
        들어온 경우)도 새 하한으로 다시 clamp 한다.
        """
        _enabled, _lower, upper = self.joint_limits[name]
        self.joint_limits[name] = (True, rad, upper)
        self.boot_floor_captured.add(name)
        self.get_logger().info(
            f"{name} 하한 고정(전원 인가 시 각도): {rad:+.3f} rad — 이 밑으로는 이동 금지")
        if self.goal_rad[name] is not None:
            self.goal_rad[name] = self._clamp_rad(name, self.goal_rad[name])
        self._publish_tick_limits()

    def _rad_to_tick(self, name, rad):
        c = self.cfg[name]
        tick = int(round(c["center"] + c["direction"] * rad * TICKS_PER_RAD))
        if name in EXTENDED_POSITION_NAMES:
            return max(DXL_EXTENDED_MIN_TICK, min(DXL_EXTENDED_MAX_TICK, tick))
        return max(DXL_MIN_TICK, min(DXL_MAX_TICK, tick))

    def _ensure_goal(self, name):
        """목표가 초기화 안 됐으면 측정값(없으면 0.0=center)으로 채운다 — 시작 튐 방지."""
        if self.goal_rad[name] is None:
            base = self.measured_rad[name]
            self.goal_rad[name] = self._clamp_rad(name, 0.0 if base is None else base)

    def _publish_goals(self, names):
        """지정 관절들의 현재 목표를 [id, tick] 로 발행.

        그리퍼처럼 이름 하나에 ID가 여럿(3,4) 매핑돼 있으면 같은 tick 을
        ID마다 한 번씩 따로 발행한다(position_node.goal_callback 은 ID 단위라
        여기서 미러링해서 보내야 두 서보가 같이 움직인다).
        """
        for name in names:
            if self.goal_rad[name] is None:
                continue
            tick = self._rad_to_tick(name, self.goal_rad[name])
            for motor_id in self.cfg[name]["ids"]:
                msg = Int32MultiArray()
                msg.data = [motor_id, tick]
                self.pub.publish(msg)

    # ------------------------------------------------------------------ callbacks
    def on_joint_states(self, msg):
        """/joint_states 는 direction 을 모르는 raw 값이다(position_node 가 tick 을
        (tick-2048)*2π/4096 로 그대로 변환) — direction=-1 인 관절은 여기서 다시
        곱해 goal_rad 와 같은 '명령 도메인'으로 되돌린다(direction²=1이라 자기 역변환)."""
        for name, pos in zip(msg.name, msg.position):
            if name in self.measured_rad:
                direction = self.cfg[name]["direction"]
                rad = float(pos) * direction
                self.measured_rad[name] = rad
                if name in self.floor_from_boot_names and name not in self.boot_floor_captured:
                    self._capture_boot_floor(name, rad)

    def on_hardware_error(self, msg):
        """position_node 의 하드웨어 에러 텍스트에서 **새로** 나타난 관절을 감지해
        그 즉시 goal_rad 를 실측값으로 되돌린다(2026-08-02 추가 — 위
        sub_hw_error 선언부 주석의 재트립 문제 대응).

        형식은 position_node._publish_hardware_errors() 가 만드는
        "이름(ID숫자):라벨,이름(ID숫자):라벨,..." — 이 정확한 포맷에 의존한다.
        resume 을 기다리지 않고 트립되는 순간 바로 고쳐서, 그 사이에 조그/타이머
        틱이 옛(무리한) 목표를 다시 내보내는 걸 막는다. "새로 나타난" 것만
        다루고 이미 떠 있던/사라진 건 건드리지 않는다 — resume/reboot 흐름이
        각자 알아서 처리한다.
        """
        current_names = set(re.findall(r"([A-Za-z0-9_]+)\(ID\d+\):", msg.data))
        newly_appeared = current_names - self._last_hw_error_names
        self._last_hw_error_names = current_names
        for name in newly_appeared:
            if name not in self.goal_rad:
                continue
            if self.measured_rad.get(name) is not None:
                self.goal_rad[name] = self._clamp_rad(name, self.measured_rad[name])
                self.velocity[name] = 0.0
                self.get_logger().warn(
                    f"{name}: 새 하드웨어 에러 감지 — 목표를 트립 지점으로 즉시 되돌림")

    def on_jog(self, msg):
        """연속 이동 의도. displacement 는 즉시 위치 증분, velocity 는 타이머가 적분."""
        self.last_input_time = self.get_clock().now()
        changed = []
        restore_jog_speed_ids = []

        disp = list(msg.displacements)
        vel = list(msg.velocities)
        for i, name in enumerate(msg.joint_names):
            if name not in self.cfg:
                self.get_logger().warn(f"알 수 없는 관절: {name}")
                continue
            self._ensure_goal(name)
            d = disp[i] if i < len(disp) else 0.0
            if d != 0.0:
                self.goal_rad[name] = self._clamp_rad(
                    name, self.goal_rad[name] + d * self.jog_step_rad)
                changed.append(name)
            # velocity 는 타이머에서 적분 (deadman 적용). 단위는 **rad/s** (JointJog 표준).
            v = vel[i] if i < len(vel) else 0.0
            self.velocity[name] = max(-self.max_vel_rad_s, min(self.max_vel_rad_s, v))

            # 실제로 움직이려는 입력(displacement 또는 velocity)이 있는데 이
            # 축이 아직 "goto 속도(느림)" 로 남아있으면 조그 속도로 되돌린다 —
            # 매 틱 다시 쓰지 않고 전환되는 순간에만 써서 버스 트래픽을 아낀다
            # (self.profile_mode, __init__ 근처 주석 참고).
            if (d != 0.0 or v != 0.0) and self.profile_mode.get(name) != "jog":
                self.profile_mode[name] = "jog"
                restore_jog_speed_ids.extend(self.cfg[name]["ids"])

        if restore_jog_speed_ids:
            self._set_profile_velocity(restore_jog_speed_ids, self.jog_profile_velocity)
        if changed:
            self._publish_goals(changed)

    def on_cmd(self, msg):
        raw = msg.data.strip()
        if not raw:
            return
        # action 만 소문자로; 자세 이름의 대소문자는 원래대로 두되 조회는
        # _norm_pose_name 에서 소문자로 통일해 대소문자 오타를 흡수한다.
        action, _, arg = raw.partition(" ")
        action = action.lower()
        arg = arg.strip()

        if action == "stop":
            # 지연 없는 즉시 정지 + 토크 차단(2026-08-02 사용자 요청: "정지를 누르면
            # 지연 없이 바로 그 상태에서 모든 명령을 멈추고 토크를 정지"). goal 을
            # 현재 위치로 맞추는 것만으로는 서보의 Profile Accel/Velocity 감속
            # 시간만큼 밀리는데(진짜 "즉시"가 아님), 토크를 직접 끊으면 그 즉시
            # 물리적으로 멈춘다 — 산업용 로봇 E-stop 과 같은 원리. 그래서 순서가
            # 중요하다: **토크 차단부터 맨 먼저** 보낸다(다른 계산보다 먼저). velocity
            # 0/goal=현재위치는 그 다음(내부 상태 일관성 + resume 시 안 튀게 하기
            # 위함일 뿐, 토크가 이미 꺼졌으니 물리적 정지 자체엔 영향 없다).
            # ⚠️ 팔이 힘을 잃고 중력으로 처질 수 있다 — 재개하려면 "resume".
            self._torque_request(self._all_motor_ids(), enable=False)
            self.velocity = {n: 0.0 for n in self.joint_names}
            for name in self.joint_names:
                if self.measured_rad[name] is not None:
                    self.goal_rad[name] = self._clamp_rad(name, self.measured_rad[name])
                else:
                    self._ensure_goal(name)
            self._publish_goals(self.joint_names)
            self.get_logger().info(
                "stop: 즉시 정지 + 전 관절(그리퍼 포함) 토크 차단 — "
                "재개하려면 'resume' 명령")
        elif action == "resume":
            # stop 으로 끊은 토크를 그 자리에서(현재 위치 홀드) 되살린다.
            self._torque_request(self._all_motor_ids(), enable=True)
            self._sync_goal_to_measured(self.joint_names)
            self.get_logger().info("resume: 전 관절 토크 복귀(현재 자세 홀드)")
        elif action == "home":
            # "home" 단독 = 저장된 자세 "home" 으로 이동. all-zero 로 임의 낙하하지
            # 않는다 — 대회 코드베이스 전반에서 all-zero 를 미검증 placeholder 로
            # 취급하는 것과 같은 이유(CLAUDE.md 참고). 먼저 'save home' 으로 저장할 것.
            self._cmd_goto_pose("home")
        elif action == "save":
            self._cmd_save_pose(arg)
        elif action == "goto":
            self._cmd_goto_pose(arg)
        elif action == "delete":
            self._cmd_delete_pose(arg)
        elif action == "poses":
            self._publish_poses_list()
            self.get_logger().info(f"저장된 자세: {sorted(self.poses) or '(없음)'}")
        elif action == "reboot":
            self._cmd_reboot(arg)
        elif action == "freedrive":
            self._torque_request(self._pose_motor_ids(), enable=False)
            self.get_logger().info("freedrive: 팔 관절 토크 OFF — 손으로 자세를 잡으세요")
        elif action == "freedrive_cancel":
            self._torque_request(self._pose_motor_ids(), enable=True)
            self._sync_goal_to_measured(self._pose_joint_names())
            self.get_logger().info("freedrive 취소: 팔 관절 토크 복귀(현재 자세 홀드)")
        elif action == "calib_start":
            self._cmd_calib_start()
        elif action == "calib_mark":
            self._cmd_calib_mark()
        elif action == "calib_cancel":
            self._cmd_calib_cancel()
        elif action == "spike":
            self._cmd_threshold_toggle(
                arg, self.spike_config_pub, "전류 급변 감지", "spike")
        elif action == "trip":
            self._cmd_threshold_toggle(
                arg, self.trip_config_pub, "절대 과전류 트립", "trip")
        elif action == "limit":
            self._cmd_limit(arg)
        else:
            self.get_logger().warn(f"알 수 없는 명령: {msg.data}")

    def _cmd_limit(self, arg):
        """관절 리밋(joint_limits) 마스터 스위치 — 'limit on'/'limit off'.

        off 는 소프트 clamp(_clamp_rad) 뿐 아니라 position_node 의 tick_limits
        방어선까지 같이 풀어야 실제로 리밋 밖으로 움직인다(위
        limits_master_enabled 선언부 주석 참고) — 그래서 매번
        _publish_tick_limits() 를 다시 부른다.
        """
        arg = arg.strip().lower()
        if arg == "on":
            self.limits_master_enabled = True
            self._publish_tick_limits()
            self.get_logger().info(
                "관절 리밋 ON — 소프트 clamp + tick_limits 방어선 복귀")
        elif arg == "off":
            self.limits_master_enabled = False
            self._publish_tick_limits()
            self.get_logger().warn(
                "관절 리밋 OFF — 단일회전 축은 리밋 없이 움직입니다("
                "다회전 축(arm_joint_2/3)은 tick_limits 없인 아예 안 움직여 "
                "안전 — position_node 설계). 다시 켜려면 'limit on'")
        else:
            self.get_logger().warn("limit 명령 형식: 'limit on' / 'limit off'")

    def _cmd_threshold_toggle(self, arg, pub, label, cmd_name):
        """전류 관련 on/off·threshold 런타임 조정 — position_node 로
        [enabled(0|1), threshold] 를 보낸다(threshold=0 은 "지금 값 유지").
        current_spike_config/current_trip_config 콜백이 이 형식을 공유한다
        (2026-08-02 spike 에서 시작해 trip 도 같은 방식으로 추가하며 공통화).

        '<cmd_name> on'/'<cmd_name> off' 는 threshold 를 안 건드리고 켜고 끄기만
        한다. '<cmd_name> <숫자>' 는 threshold 를 그 값으로 바꾸면서 자동으로
        켠다(끈 상태에서 새 값을 시험해보려는 게 보통이라서).
        """
        arg = arg.strip().lower()
        msg = Int32MultiArray()
        if arg == "on":
            msg.data = [1, 0]
        elif arg == "off":
            msg.data = [0, 0]
        else:
            try:
                threshold = int(arg)
            except ValueError:
                self.get_logger().warn(
                    f"{cmd_name} 명령 형식: '{cmd_name} on' / '{cmd_name} off' / "
                    f"'{cmd_name} <threshold>'")
                return
            if threshold <= 0:
                self.get_logger().warn("threshold 는 0보다 커야 합니다")
                return
            msg.data = [1, threshold]
        pub.publish(msg)
        self.get_logger().info(f"{label} 설정 요청: {arg}")

    def _cmd_reboot(self, arg):
        name = arg.strip().lower()
        if not name:
            self.get_logger().warn(
                "reboot 대상이 없습니다 (예: 'reboot all' 또는 'reboot arm_joint_2')")
            return
        msg = Int32MultiArray()
        if name == "all":
            msg.data = []  # position_node 가 "현재 에러 latch 된 서보 전체" 로 해석
        else:
            entry = self.cfg.get(name)
            if entry is None:
                self.get_logger().warn(f"알 수 없는 관절: '{name}'")
                return
            msg.data = list(entry["ids"])
        self.reboot_pub.publish(msg)
        self.get_logger().info(f"reboot 요청 전송: '{name}' (ID {list(msg.data) or '전체(latch)'})")

    # ------------------------------------------------------------------ 자세 저장/이동
    #: 자세 저장/불러오기(freedrive/save/goto/calib_*) 대상에서 빼는 관절 이름.
    #: arm_joint_5 는 2026-08-02 사용자 보고로 자세 저장/불러오기 시 오작동이
    #: 있어 그리퍼와 동일하게(그리퍼도 원래 이 요구사항으로 빠져있었다) 제외했다
    #: — jog(1-6 키 개별 조작)는 그대로 된다, _pose_joint_names() 를 쓰는 기능만
    #: (freedrive/save/goto/calib_*) 건드리지 않는다.
    POSE_EXCLUDED_NAMES = {"arm_joint_5"}

    def _pose_joint_names(self):
        """자세 저장/이동 대상 — 그리퍼 + POSE_EXCLUDED_NAMES 제외."""
        return [
            n for n in self.joint_names
            if "gripper" not in n and n not in self.POSE_EXCLUDED_NAMES
        ]

    def _pose_motor_ids(self):
        """_pose_joint_names() 를 모터 ID로 풀어준다(freedrive 토크 요청용)."""
        ids = []
        for name in self._pose_joint_names():
            ids.extend(self.cfg[name]["ids"])
        return ids

    def _calib_target_names(self):
        """리미트 측정(calib_*) 대상 축 — _pose_joint_names() 기반이지만, 다회전
        (EXTENDED_POSITION_NAMES) 축은 여기 반드시 포함시킨다(2026-08-02 그리퍼
        다회전 전환과 함께 추가). position_node.goal_callback 은 다회전 축을
        tick_limits 없이는 이동 자체를 거부하도록 설계돼 있어서(위 tick_limits
        선언부 주석 참고) — 그리퍼처럼 원래 _pose_joint_names() 에서 빠지던
        축이라도 다회전으로 바뀌면 실측 리밋이 없으면 아예 못 움직인다. 즉
        "다회전 축이 됐다" = "반드시 리미트 측정을 거쳐야 쓸 수 있다".
        """
        names = list(self._pose_joint_names())
        for name in self.joint_names:
            if name in EXTENDED_POSITION_NAMES and name not in names:
                names.append(name)
        return names

    def _calib_target_motor_ids(self):
        """_calib_target_names() 를 모터 ID로 풀어준다(calib 진행 중 토크 요청용)."""
        ids = []
        for name in self._calib_target_names():
            ids.extend(self.cfg[name]["ids"])
        return ids

    def _all_motor_ids(self):
        """전 관절(그리퍼 포함, POSE_EXCLUDED_NAMES 도 포함) 모터 ID — stop/resume 전용.

        _pose_motor_ids() 와 달리 그리퍼·arm_joint_5 도 뺴지 않는다 — "정지" 는
        예외 없이 전부 멈추고 토크를 끊어야 하는 명령이라서다.
        """
        ids = []
        for name in self.joint_names:
            ids.extend(self.cfg[name]["ids"])
        return ids

    def _torque_request(self, ids, enable):
        """position_node 에 토크 on/off 를 요청한다. ids 가 비면 아무것도 안 보낸다."""
        if not ids:
            return
        msg = Int32MultiArray()
        msg.data = [1 if enable else 0] + list(ids)
        self.torque_pub.publish(msg)

    def _set_profile_velocity(self, ids, velocity):
        """position_node 에 Profile Velocity 변경을 요청한다. ids 가 비면 무시."""
        if not ids:
            return
        msg = Int32MultiArray()
        msg.data = [int(velocity)] + list(ids)
        self.profile_velocity_pub.publish(msg)

    def _sync_goal_to_measured(self, names):
        """토크 재활성화 직후 goal_rad 를 실측값으로 맞춘다.

        freedrive 중엔 손으로 움직이므로 goal_rad(teleop_core 내부 목표)가
        낡은 값 그대로 남아있다 — 안 맞춰주면 재활성화 후 첫 조그에서 그 낡은
        값을 기준으로 움직여 갑자기 튄다(position_node 는 자체적으로 현재
        위치를 홀드하지만, 그건 goal_position 레지스터 얘기고 teleop_core 의
        goal_rad 캐시는 별개라 따로 동기화해야 한다).
        """
        for name in names:
            if self.measured_rad.get(name) is not None:
                self.goal_rad[name] = self._clamp_rad(name, self.measured_rad[name])
                self.velocity[name] = 0.0

    # ------------------------------------------------------------------ 리미트 측정 모드
    def _cmd_calib_start(self):
        if self.calib_active:
            self.get_logger().warn(
                "이미 리미트 측정 모드입니다 — calib_mark 로 계속하거나 calib_cancel 로 취소하세요")
            return
        calib_joints = self._calib_target_names()
        # save 와 동일한 이유로 "지금 측정값이 있는" 축만 대상으로 한다 — 안 그러면
        # 미연결 축(arm_joint_1 등)에서 영원히 calib_mark 를 기다리며 멈춘다.
        available = [n for n in calib_joints if self.measured_rad.get(n) is not None]
        missing = [n for n in calib_joints if n not in available]
        if not available:
            self.get_logger().warn(
                "리미트 측정 시작 실패: /joint_states 를 받은 관절이 하나도 없습니다")
            return
        if missing:
            self.get_logger().warn(
                f"리미트 측정: {missing} 는 /joint_states 미수신이라 이번 세션에서 제외합니다")

        self.calib_active = True
        self.calib_joint_names = available
        self.calib_index = 0
        self.calib_step = "lower"
        self.calib_results = {}

        self._torque_request(self._calib_target_motor_ids(), enable=False)
        self.get_logger().info(
            f"리미트 측정 모드 시작: {self.calib_joint_names} — 팔 관절 토크 OFF, "
            f"'{self.calib_joint_names[0]}' 하한선부터"
        )
        self._publish_calib_progress()

    def _cmd_calib_mark(self):
        if not self.calib_active:
            self.get_logger().warn("리미트 측정 모드가 아닙니다 (calib_start 먼저)")
            return
        name = self.calib_joint_names[self.calib_index]
        rad = self.measured_rad.get(name)
        if rad is None:
            self.get_logger().warn(f"{name} 측정값이 아직 없어 기록할 수 없습니다")
            return

        self.calib_results.setdefault(name, {})[self.calib_step] = rad
        self.get_logger().info(f"{name} {self.calib_step} 기록: {rad:+.3f} rad")

        if self.calib_step == "lower":
            self.calib_step = "upper"
            self._publish_calib_progress()
            return

        # upper 까지 끝났으면 다음 축으로 진행
        self.calib_index += 1
        self.calib_step = "lower"
        if self.calib_index < len(self.calib_joint_names):
            self._publish_calib_progress()
        else:
            self._finish_calibration()

    def _cmd_calib_cancel(self):
        if not self.calib_active:
            self.get_logger().warn("리미트 측정 모드가 아닙니다")
            return
        ids = self._calib_target_motor_ids()
        self.calib_active = False
        self.calib_joint_names = []
        self.calib_results = {}
        self._torque_request(ids, enable=True)
        self._sync_goal_to_measured(self._calib_target_names())
        self.get_logger().info("리미트 측정 취소 — 팔 관절 토크 복귀(현재 자세 홀드), 기록값 폐기")
        self._publish_calib_status("cancelled")

    def _finish_calibration(self):
        """모든 축의 상/하한을 다 기록했다 — 검증 후 joint_limits 에 적용 + 파일
        영속화 + 토크 복귀. 값이 이상하면(단일회전 범위 밖, 폭 0 등) 그 축은
        적용을 거부한다 — _validate_calib_bounds 위 주석의 실사용 중 사고 참고.
        """
        applied, rejected = [], []
        for name, bounds in self.calib_results.items():
            lower, upper = bounds["lower"], bounds["upper"]
            if lower > upper:
                lower, upper = upper, lower
            ok, reason = _validate_calib_bounds(name, lower, upper)
            if not ok:
                self.get_logger().error(f"{name} 리미트 측정 거부 — {reason}")
                rejected.append(name)
                continue
            self.joint_limits[name] = (True, lower, upper)
            # boot_floor 캡처가 나중에 이 정밀한 양쪽 실측값을 편측 값으로 덮어쓰지
            # 못하게 제외한다.
            self.floor_from_boot_names.discard(name)
            self.calibrated_joints.add(name)
            applied.append(name)

        # 토크를 다시 켜기 *전에* 먼저 position_node 의 최종 방어선부터 갱신한다.
        self._publish_tick_limits()
        self._save_joint_limits_file()

        ids = self._calib_target_motor_ids()
        self.calib_active = False
        self.calib_joint_names = []
        self.calib_results = {}
        self._torque_request(ids, enable=True)
        self._sync_goal_to_measured(self._calib_target_names())

        self.get_logger().info(
            f"리미트 측정 완료 — 적용: {applied or '없음'}"
            + (f", 거부(재측정 필요): {rejected}" if rejected else "")
            + f" — {self._limit_summary()}"
        )
        self._publish_calib_status(f"done,{len(applied)},{len(rejected)}")

    def _publish_calib_progress(self):
        name = self.calib_joint_names[self.calib_index]
        total = len(self.calib_joint_names)
        self._publish_calib_status(
            f"active,{name},{self.calib_step},{self.calib_index + 1},{total}")

    def _publish_calib_status(self, text):
        self.calib_status_pub.publish(String(data=text))

    def _load_joint_limits_file(self):
        if not os.path.isfile(self.joint_limits_file):
            return {}
        try:
            with open(self.joint_limits_file, "r") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                raise ValueError("최상위가 객체가 아님")
            return {
                str(name): {"lower": float(v["lower"]), "upper": float(v["upper"])}
                for name, v in data.items()
            }
        except (OSError, ValueError, TypeError, KeyError) as exc:
            self.get_logger().error(
                f"리미트 파일 로드 실패({self.joint_limits_file}): {exc} — "
                "boot 캡처(있는 축) 로 대체합니다"
            )
            return {}

    def _save_joint_limits_file(self):
        try:
            os.makedirs(os.path.dirname(self.joint_limits_file), exist_ok=True)
            data = {
                name: {"lower": lower, "upper": upper}
                for name, (enabled, lower, upper) in self.joint_limits.items()
                if enabled and name in self.calibrated_joints
            }
            tmp_path = self.joint_limits_file + ".tmp"
            with open(tmp_path, "w") as f:
                json.dump(data, f, indent=2, ensure_ascii=False, sort_keys=True)
            os.replace(tmp_path, self.joint_limits_file)
        except OSError as exc:
            self.get_logger().error(f"리미트 파일 저장 실패({self.joint_limits_file}): {exc}")

    @staticmethod
    def _norm_pose_name(name):
        return name.strip().lower()

    def _cmd_save_pose(self, name):
        name = self._norm_pose_name(name)
        if not name:
            self.get_logger().warn("저장할 자세 이름이 없습니다 (예: 'save home')")
            return
        pose_joints = self._pose_joint_names()
        # 관절 전부가 아니라 "지금 /joint_states 를 받은 것"만 저장한다 — arm_joint_1
        # 처럼 미연결 서보가 하나만 있어도 저장 자체가 영구히 막히던 버그(2026-08-01
        # 발견: '아직 /joint_states 미수신' 경고만 계속 찍히고 저장이 한 번도 안 됨).
        available = [n for n in pose_joints if self.measured_rad.get(n) is not None]
        missing = [n for n in pose_joints if n not in available]
        if not available:
            self.get_logger().warn(
                "저장 실패: /joint_states 를 받은 관절이 하나도 없습니다 — "
                "position_node(use_hardware:=true) 연결을 확인하세요")
            return
        if missing:
            self.get_logger().warn(
                f"자세 저장: {missing} 는 /joint_states 미수신(미연결 서보 등)이라 "
                "제외하고 저장합니다")
        self.poses[name] = {n: self.measured_rad[n] for n in available}
        self._save_poses_file()
        self._publish_poses_list()
        summary = ", ".join(f"{n}={self.poses[name][n]:+.3f}" for n in available)
        self.get_logger().info(f"자세 저장: '{name}' = {summary}")

        # freedrive 로 손으로 잡은 자세를 그대로 홀드하도록 토크 복귀
        # (2026-08-02 추가). position_node 가 켜기 직전 현재 위치를 goal_position
        # 에 되써서 이전 목표로 안 튄다 — 여기서도 teleop_core 쪽 goal_rad 캐시를
        # 같은 값으로 맞춰준다(안 그러면 다음 조그가 낡은 목표 기준으로 튐).
        self._torque_request(self._pose_motor_ids(), enable=True)
        self._sync_goal_to_measured(available)

    def _cmd_delete_pose(self, name):
        """저장된 자세를 지운다 (2026-08-19 추가).

        지우기 전에는 GUI 에서 잘못 저장된 이름을 없앨 방법이 없었다 — 실제로
        붙여넣기 사고로 만들어진 이름(제어문자 포함)이 목록에 남아 있었다.
        **모션을 만들지 않는다**(파일과 목록만 바꾼다).
        """
        name = self._norm_pose_name(name)
        if not name:
            self.get_logger().warn("지울 자세 이름이 없습니다 (예: 'delete home')")
            return
        if name not in self.poses:
            self.get_logger().warn(
                f"자세 '{name}' 이(가) 없습니다 — 저장된 자세: "
                f"{sorted(self.poses) or '(없음)'}")
            return
        del self.poses[name]
        self._save_poses_file()
        self._publish_poses_list()
        self.get_logger().info(f"자세 삭제: '{name}'")

    def _cmd_goto_pose(self, name):
        name = self._norm_pose_name(name)
        if not name:
            self.get_logger().warn("이동할 자세 이름이 없습니다 (예: 'goto home')")
            return
        pose = self.poses.get(name)
        if pose is None:
            self.get_logger().warn(
                f"알 수 없는 자세: '{name}' — 저장된 자세: {sorted(self.poses) or '(없음)'}")
            return
        moving_names = []
        goto_speed_ids = []
        for joint_name, rad in pose.items():
            if joint_name not in self.cfg:
                continue  # 파일에 남은 옛 관절 이름(구성 변경) — 무시
            self._ensure_goal(joint_name)
            self.goal_rad[joint_name] = self._clamp_rad(joint_name, rad)
            self.velocity[joint_name] = 0.0
            moving_names.append(joint_name)
            if self.profile_mode.get(joint_name) != "goto":
                self.profile_mode[joint_name] = "goto"
                goto_speed_ids.extend(self.cfg[joint_name]["ids"])

        # goal_position 을 쏘기 *전에* 먼저 느린 속도로 바꿔둔다 — 순서가
        # 바뀌면 첫 이동은 아직 빠른(조그) 속도로 시작해버린다.
        if goto_speed_ids:
            self._set_profile_velocity(goto_speed_ids, self.goto_profile_velocity)
        self._publish_goals(moving_names)
        self.get_logger().info(f"자세 이동: '{name}' (그리퍼는 영향 없음)")

    def _load_poses_file(self):
        if not os.path.isfile(self.poses_file):
            return {}
        try:
            with open(self.poses_file, "r") as f:
                data = json.load(f)
            if not isinstance(data, dict):
                raise ValueError("최상위가 객체가 아님")
            return {
                str(name): {str(j): float(v) for j, v in joints.items()}
                for name, joints in data.items()
            }
        except (OSError, ValueError, TypeError) as exc:
            self.get_logger().error(
                f"자세 파일 로드 실패({self.poses_file}): {exc} — 빈 목록으로 시작합니다")
            return {}

    def _save_poses_file(self):
        try:
            os.makedirs(os.path.dirname(self.poses_file), exist_ok=True)
            tmp_path = self.poses_file + ".tmp"
            with open(tmp_path, "w") as f:
                json.dump(self.poses, f, indent=2, ensure_ascii=False, sort_keys=True)
            os.replace(tmp_path, self.poses_file)  # 원자적 교체 — 쓰다 중단돼도 원본 보존
        except OSError as exc:
            self.get_logger().error(f"자세 파일 저장 실패({self.poses_file}): {exc}")

    def _publish_poses_list(self):
        self.poses_pub.publish(String(data=",".join(sorted(self.poses))))

    def _publish_sim_joint_states(self):
        """목표 각도를 raw 도메인으로 되돌려 /joint_states 로 발행(open-loop, RViz 디버그 전용).

        on_joint_states 가 raw→명령 도메인으로 direction 을 곱해 되돌리므로, 여기서도
        같은 곱을 적용해야 sim 모드에서 자기 자신을 구독하는 루프가 일관된다.
        """
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        for name in self.joint_names:
            rad = self.goal_rad[name]
            direction = self.cfg[name]["direction"]
            msg.name.append(name)
            msg.position.append(0.0 if rad is None else rad * direction)
        self.js_pub.publish(msg)

    def on_timer(self):
        """sim joint_states 발행(옵션) + velocity 모드 적분 + 데드맨."""
        # RViz 디버그: 목표 포즈를 항상 내보내 TF 가 갱신되게 한다(입력 없어도).
        if self.js_pub is not None:
            self._publish_sim_joint_states()

        now = self.get_clock().now()
        dt = 1.0 / max(1e-3, float(self.get_parameter("publish_rate_hz").value))
        elapsed = (now - self.last_input_time).nanoseconds * 1e-9
        if elapsed > self.deadman_timeout_s:
            return  # 입력 끊김 → velocity 적분 중단(현재 목표 유지)

        moving = []
        for name in self.joint_names:
            v = self.velocity.get(name, 0.0)
            if v != 0.0:
                self._ensure_goal(name)
                # velocities 는 rad/s → dt 만 곱한다. (jog_step_rad 는 displacement 전용)
                self.goal_rad[name] = self._clamp_rad(name, self.goal_rad[name] + v * dt)
                moving.append(name)
        if moving:
            self._publish_goals(moving)


def main(args=None):
    rclpy.init(args=args)
    node = TeleopCore()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
