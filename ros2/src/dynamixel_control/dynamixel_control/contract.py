"""로봇팔 ↔ 파워트레인 ROS2 계약 — 값 어휘·QoS 단일 출처.

⚠️ **이 파일은 파워트레인의 `powertrain_ros/contract.py` 와 짝이다.**
    github.com/lightminn/power-train-sw : ros2/src/powertrain_ros/powertrain_ros/contract.py

메시지 **타입**은 `robot_arm_msgs`(우리 소유, 파워트레인이 벤더링해 각자 빌드)가 정의하고,
**값 어휘**는 양쪽 contract.py 가 정의한다. 한쪽만 바꾸면 통신이 조용히 깨지므로
**어휘 변경은 양 팀 합의 사항**이다.

## 통신 구조
같은 Jetson의 서로 다른 컨테이너에서 각자 ROS2 노드를 돌리고 DDS 로만 통신한다
(워크스페이스 오버레이 없음). 양쪽 컨테이너가 `network_mode: host` + **`ipc: host`** 여야
한다 — `ipc: host` 가 없으면 Fast-DDS 공유메모리 전송이 컨테이너 경계를 못 넘어
discovery 는 되는데 데이터가 안 온다.

| 방향 | 토픽 | 타입 |
| --- | --- | --- |
| 팔 → 파워트레인 | `/arm_status` | ArmStatus |
| 팔 → 파워트레인 | `/detected_objects` | DetectedObjectArray |
| 파워트레인 → 팔 | `/chassis_mode` | ChassisMode |
| 파워트레인 → 팔 | `/arrival_status` | ArrivalStatus |

## 계약 v2 요약 (파워트레인 `arm_interlock.py` 기준)
- `MISSION_STOP` 만이 팔 작업을 허가한다. **`DRIVING` 을 포함한 나머지 mode 는 전부 잠금**이다.
- 팔은 **10 Hz 로 현재 상태를 계속 발행**해야 한다. 0.5초 넘게 끊기면 파워트레인이 차를 세운다.
- header.stamp 는 **단조 증가**해야 한다. 0.5초 이상 역행하면 파워트레인이 계약 위반으로
  **영구 latch** 를 걸고 프로세스 재시작 전까지 안 풀린다 → 발행 경로는 **반드시 하나**여야 한다.
- 차가 움직이려면 팔이 `STOWED_LOCKED`(빈 손) 또는 `CARRYING_LOCKED`(운반 중) 중
  하나를 신선하게 발행해야 한다. 그 외 status 는 전부 주행 불가다.

⚠️ **이 모듈은 ROS 비의존(순수 파이썬)으로 유지한다.** 파워트레인 contract.py 와 1:1로
   대조할 수 있어야 하고, ROS 없이 테스트할 수 있어야 한다.
   QoS 프로파일은 rclpy 가 필요하므로 `qos_profiles.py` 로 분리했다.
"""

# ── 파워트레인 → 팔 : ArrivalStatus.status ──
ARRIVED_PICKUP = 'ARRIVED_PICKUP'
ARRIVED_DROP = 'ARRIVED_DROP'

# ── 파워트레인 → 팔 : ChassisMode.mode ──
MODE_DRIVING = 'DRIVING'
MODE_CORNERING = 'CORNERING'
MODE_ROUGH_TERRAIN = 'ROUGH_TERRAIN'
MODE_FOLLOW_LEAD = 'FOLLOW_LEAD'
MODE_MISSION_STOP = 'MISSION_STOP'      # 계약 v2: 유일한 작업 허가
MODE_STOW_REQUEST = 'STOW_REQUEST'      # 계약 v2: 접고 잠그라는 요청

#: 계약 v2 — 이 mode 들은 전부 잠금. 모르는 mode·stale 도 잠금(default-deny).
LOCK_MODES = {
    MODE_DRIVING,
    MODE_CORNERING,
    MODE_ROUGH_TERRAIN,
    MODE_FOLLOW_LEAD,
}

# ── 팔 → 파워트레인 : ArmStatus.status (계약 v1 어휘) ──
ARM_IDLE = 'IDLE'
ARM_PERCEIVING = 'PERCEIVING'
ARM_PLANNING = 'PLANNING'
ARM_EXECUTING = 'EXECUTING'
ARM_CARRYING = 'CARRYING'
ARM_DONE = 'DONE'          # 계약 v2 에서 진단용 — 파워트레인은 ACK·주행허가로 쓰지 않는다
ARM_FAILED = 'FAILED'

# ── 계약 v2 신설 status ──
# ⚠️ 아직 **의미가 구현되지 않았다** — 접힘 자세(stow posture) 정의가 선행이다.
#    상수만 정의해 두고, FSM 은 아직 발행하지 않는다.
ARM_WORK_READY = 'WORK_READY'
ARM_STOWING = 'STOWING'
ARM_STOWED_LOCKED = 'STOWED_LOCKED'
ARM_CARRYING_LOCKED = 'CARRYING_LOCKED'
ARM_GRIP_LOST = 'GRIP_LOST'

#: 파워트레인이 "차를 움직여도 된다"고 판단하는 유일한 두 status.
#: 우리가 이걸 발행하기 전까지 **차는 절대 출발하지 못한다.**
DRIVE_READY_STATUSES = {ARM_STOWED_LOCKED, ARM_CARRYING_LOCKED}

#: 파워트레인이 ArrivalStatus 의 ACK 로 인정하는 status (mission_id 일치 조건 추가).
WORK_ACCEPTED_STATUSES = {
    ARM_WORK_READY,
    ARM_PERCEIVING,
    ARM_PLANNING,
    ARM_EXECUTING,
}

#: 선택 진단 실패 코드. 지원하지 않으면 FAILED 로 대체해도 안전 전이는 유지된다.
ARM_DIAGNOSTIC_FAILURES = {
    'IK_FAILURE',
    'TRAJECTORY_FAILURE',
    'SELF_COLLISION',
    'BASE_COLLISION',
    'JOINT_OVERCURRENT',
    'GRIP_UNCERTAIN',
    'STOW_FAILURE',
    'ACTION_TIMEOUT',
}

#: 파워트레인이 받아주는 status 전체 집합(closed set). 이 밖의 값을 보내면
#: 즉시 motion hold + CONTRACT_VIOLATION 이다.
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

# ── 토픽명 ──
TOPIC_ARM_STATUS = '/arm_status'
TOPIC_DETECTED = '/detected_objects'
TOPIC_CHASSIS_MODE = '/chassis_mode'
TOPIC_ARRIVAL = '/arrival_status'

# ── 타이밍 (계약 §5.1) ──
#: heartbeat 발행 주기 [Hz]. 파워트레인 timeout 이 0.5초라 5배 여유.
HEARTBEAT_RATE_HZ = 10.0

#: 파워트레인이 팔 heartbeat 를 stale 로 판정하는 나이 [s]. 넘기면 차가 선다.
HEARTBEAT_TIMEOUT_S = 0.5
