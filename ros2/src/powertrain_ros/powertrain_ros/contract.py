"""파워트레인 ↔ 로봇팔 ROS2 계약 문자열 — 단일 출처.

로봇팔 팀 `arm_fsm_node.py` 가 쓰는 값과 합의한 어휘. 그쪽 주석은 "⚠️ 잠정값 —
파워트레인 팀과 합의 후 확정" 이며, 우리가 여기서 확정한다. 메시지 타입 정의는
벤더링(`ros2/src/robot_arm_msgs`), **값 어휘는 이 파일** 이 기준.

⚠️ 미결 2건 (계약 확정 시 팀 합의 필요 — docs/plans/2026-07-07-wp4-ros2-roundtrip.md §계약):
  (1) MISSION_STOP: 우리 vocab 에 있으나 그들 LOCK_MODES 에 없음(팔이 무시).
  (2) 락 해제 순서: 코너(락)→미션 정차 시, DRIVING 을 먼저 보내 언락 후 ARRIVED_* 발행.
"""

# ── 우리 → 팔: ArrivalStatus.status ──
ARRIVED_PICKUP = "ARRIVED_PICKUP"     # 박스 정렬 완료 → 팔이 집기 시작(팔 IDLE 조건)
ARRIVED_DROP = "ARRIVED_DROP"         # 하역 지점 도착 → 팔이 내려놓기(팔 CARRY 조건)

# ── 우리 → 팔: ChassisMode.mode ──
MODE_DRIVING = "DRIVING"              # 정상 주행 = 팔 언락
MODE_CORNERING = "CORNERING"          # ┐
MODE_ROUGH_TERRAIN = "ROUGH_TERRAIN"  # ├ 팔 락(자세 고정) — 그들 LOCK_MODES
MODE_FOLLOW_LEAD = "FOLLOW_LEAD"      # ┘
MODE_MISSION_STOP = "MISSION_STOP"    # ⚠️ 미결: 그들 코드에 없음(팔 무시). 계약 확정 대상.
LOCK_MODES = {MODE_CORNERING, MODE_ROUGH_TERRAIN, MODE_FOLLOW_LEAD}

# ── 팔 → 우리: ArmStatus.status ──
ARM_IDLE = "IDLE"
ARM_PERCEIVING = "PERCEIVING"
ARM_PLANNING = "PLANNING"
ARM_EXECUTING = "EXECUTING"
ARM_CARRYING = "CARRYING"
ARM_DONE = "DONE"                     # 팔 작업 완료 → 우리 재출발 신호
ARM_FAILED = "FAILED"

# ── 토픽명 ──
TOPIC_ARM_STATUS = "/arm_status"              # 구독
TOPIC_DETECTED = "/detected_objects"          # 구독
TOPIC_CHASSIS_MODE = "/chassis_mode"          # 발행
TOPIC_ARRIVAL = "/arrival_status"             # 발행
