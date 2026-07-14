"""파워트레인 ↔ 로봇팔 ROS2 계약 문자열 — 단일 출처.

로봇팔 팀 `arm_fsm_node.py` 가 쓰는 값과 합의한 어휘. 그쪽 주석은 "⚠️ 잠정값 —
파워트레인 팀과 합의 후 확정" 이며, 우리가 여기서 확정한다. 메시지 타입 정의는
벤더링(`ros2/src/robot_arm_msgs`), **값 어휘는 이 파일** 이 기준.

계약 v1의 상수 이름은 기존 노드 호환을 위해 보존한다. 계약 v2에서는
`MISSION_STOP`만 팔 작업을 허가하고, `DRIVING`을 포함한 나머지 주행 mode는 모두
default-deny 잠금 명령이다. 상세 정본은 WP5.2 협업 안전 계획을 따른다.
"""

# ── 우리 → 팔: ArrivalStatus.status ──
ARRIVED_PICKUP = "ARRIVED_PICKUP"     # 박스 정렬 완료 → 팔이 집기 시작(팔 IDLE 조건)
ARRIVED_DROP = "ARRIVED_DROP"         # 하역 지점 도착 → 팔이 내려놓기(팔 CARRY 조건)

# ── 우리 → 팔: ChassisMode.mode ──
MODE_DRIVING = "DRIVING"              # v1 이름 보존, v2에서는 default-deny 잠금
MODE_CORNERING = "CORNERING"          # ┐
MODE_ROUGH_TERRAIN = "ROUGH_TERRAIN"  # ├ 팔 락(자세 고정) — 그들 LOCK_MODES
MODE_FOLLOW_LEAD = "FOLLOW_LEAD"      # ┘
MODE_MISSION_STOP = "MISSION_STOP"    # 계약 v2: 유일한 팔 작업 허가 mode
MODE_STOW_REQUEST = "STOW_REQUEST"    # 계약 v2: release 없는 접힘·잠금 요청
LOCK_MODES = {
    MODE_DRIVING,
    MODE_CORNERING,
    MODE_ROUGH_TERRAIN,
    MODE_FOLLOW_LEAD,
}

# ── 팔 → 우리: ArmStatus.status ──
ARM_IDLE = "IDLE"
ARM_PERCEIVING = "PERCEIVING"
ARM_PLANNING = "PLANNING"
ARM_EXECUTING = "EXECUTING"
ARM_CARRYING = "CARRYING"
ARM_DONE = "DONE"                     # v1 이름 보존, v2에서는 ACK·주행허가로 쓰지 않음
ARM_FAILED = "FAILED"
ARM_WORK_READY = "WORK_READY"
ARM_STOWING = "STOWING"
ARM_STOWED_LOCKED = "STOWED_LOCKED"
ARM_CARRYING_LOCKED = "CARRYING_LOCKED"
ARM_GRIP_LOST = "GRIP_LOST"

ARM_DIAGNOSTIC_FAILURES = {
    "IK_FAILURE",
    "TRAJECTORY_FAILURE",
    "SELF_COLLISION",
    "BASE_COLLISION",
    "JOINT_OVERCURRENT",
    "GRIP_UNCERTAIN",
    "STOW_FAILURE",
    "ACTION_TIMEOUT",
}
DRIVE_READY_STATUSES = {ARM_STOWED_LOCKED, ARM_CARRYING_LOCKED}
ARM_STATUSES = {
    ARM_IDLE,
    ARM_PERCEIVING,
    ARM_PLANNING,
    ARM_EXECUTING,
    ARM_CARRYING,
    ARM_DONE,
    ARM_FAILED,
    ARM_WORK_READY,
    ARM_STOWING,
    ARM_STOWED_LOCKED,
    ARM_CARRYING_LOCKED,
    ARM_GRIP_LOST,
} | ARM_DIAGNOSTIC_FAILURES
WORK_ACCEPTED_STATUSES = {
    ARM_WORK_READY,
    ARM_PERCEIVING,
    ARM_PLANNING,
    ARM_EXECUTING,
}

# ── 토픽명 ──
TOPIC_ARM_STATUS = "/arm_status"              # 구독
TOPIC_DETECTED = "/detected_objects"          # 구독
TOPIC_CHASSIS_MODE = "/chassis_mode"          # 발행
TOPIC_ARRIVAL = "/arrival_status"             # 발행
