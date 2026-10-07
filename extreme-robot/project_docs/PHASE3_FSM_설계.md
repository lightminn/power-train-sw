# Phase 3 — 로봇팔 FSM 설계 (drawio 미션 FSM 다이어그램 기반)

> 작성: 2026-06-29 / 기준: 미션 FSM 다이어그램(5구간 미션 FSM) + `CLAUDE_Plan.md` §1·§2·§3 인터페이스
> 갱신: 2026-07-05 — HW-7 실측으로 §6-A-1(analytic IK 우회)·§7-8 추가.
> 범위: **설계 문서만** (구현 전 단계). 코딩은 이 문서 합의 후 진행.

---

## 1. 핵심 발견 — 팔이 실제로 일하는 구간은 "구간2" 하나

drawio는 **전체 로봇 5구간 미션 FSM**(주행+인식+팔이 한 흐름에 섞임). 구간별 로봇팔 관여도:

| 구간 | 미션 | 팔 역할 |
|------|------|---------|
| 1 대상 식별 | 저시정 주행 → 대상 분류 → LED | ❌ 자세 락만 |
| **2 박스 운반** | 박스 파지 → 운반 → 하역 | ✅ **핵심 픽 작업 (ARM_GRASP_BOX)** |
| 3 마커 식별 | 비전마커 5개 식별 | ❌ 자세 락만 |
| 4 노면 주행 | 빙판/제설 주행 | ❌ 자세 락만 (제설 방식 미정 — 팔로 치울지 논의 필요) |
| 5 선도차 추종 | 선도차 PID 추종 | ❌ 자세 락만 |

→ **Phase 3 로봇팔 구현의 90%는 구간2의 `ARM_GRASP_BOX` 서브루틴.** 나머지 구간은 팔 FSM 입장에서 "CARRY/LOCK 상태 유지"로 동일하게 처리.

> ⚠️ 구간4 제설: drawio 논의사항에 "치우며 갈지 / 밟고 갈지" 미정. 팔로 제설하면 두 번째 팔 작업이 생김 → **선결 결정 필요**(아래 §6-D).

---

## 2. 노드 책임 분리 (drawio는 전체 로봇 → 우리는 팔 노드만)

`CLAUDE_Plan.md`는 **팔 FSM 노드 / 파워트레인 노드**를 토픽으로 분리하는 구조. drawio의 한 흐름을 둘로 자른다:

```
[파워트레인 소유]                          [로봇팔 FSM 소유]
DETECT_BOX, ALIGN_BOX, 주행/지형/신호등 ──/arrival_status──▶ ARM_GRASP_BOX 전체
SAND/GRAVEL/WATER/ICE, FOLLOW_LEAD     ──/chassis_mode───▶ 자세 락 트리거
                                       ◀──/arm_status────  파지 완료/실패 보고
```

- **파워트레인 소유**: 박스 탐지(`DETECT_BOX`)·정렬(`ALIGN_BOX`)·모든 주행/지형/신호등/추종 상태.
  - 단, `DETECT_BOX`의 YOLO는 인식 노드(`robot_arm_perception`)가 단일 소스로 publish → 파워트레인은 `/detected_objects` 구독, 팔은 `/pick_target` 구독 (Phase 2에서 구현됨).
- **로봇팔 FSM 소유**: `ALIGN_BOX` 완료 신호를 받은 뒤의 파지 시퀀스 전체 + 운반 중 전류 감시 + DROP 재파지.

---

## 3. 토픽 인터페이스 (Phase 2에서 일부 확정)

| 토픽 | 타입 | 방향 | 팔 FSM | 상태 |
|------|------|------|--------|------|
| `/pick_target` | DetectedObject | 인식→팔 | 파지 타겟 pose (transient_local) | ✅ Phase 2 구현 |
| `/arrival_status` | ArrivalStatus | 파워트레인→팔 | `ARRIVED_PICKUP`/`ARRIVED_DROP`로 FSM 트리거 | ⚠️ enum 미합의 |
| `/chassis_mode` | ChassisMode | 파워트레인→팔 | 지형/추종 모드 → 자세 락 | ⚠️ enum 미합의 |
| `/arm_status` | ArmStatus | 팔→파워트레인 | `PERCEIVING/PLANNING/EXECUTING/DONE/FAILED` | ⚠️ enum 미합의 |
| `/detected_objects` | DetectedObjectArray | 인식→파워트레인 | (팔 미사용) | ✅ Phase 2 |

**구간2용 status enum 제안 (파워트레인 팀 합의 필요):**
- `ArrivalStatus.status`: `ARRIVED_PICKUP`(박스 정렬 완료, 집어), `ARRIVED_DROP`(하역 지점 도착, 내려)
- `ArmStatus.status`: `PERCEIVING` → `PLANNING`(IK) → `EXECUTING`(하강/파지/리프트) → `CARRYING` → `DONE` / `FAILED`(재시도 한계 초과)
- `ChassisMode.mode`: `DRIVING`/`CORNERING`/`ROUGH_TERRAIN`(SAND·GRAVEL·WATER·ICE 통합)/`MISSION_STOP`/`FOLLOW_LEAD`

---

## 4. 로봇팔 FSM 상태표 (구간2 ARM_GRASP_BOX 중심)

drawio 구간2의 파지 서브루틴 + DROP 루프를 팔 노드 상태로 정리. 전류 임계 기반 파지/낙하 판정이 핵심.

| 상태 | 진입 조건 | 동작 | 전이 |
|------|-----------|------|------|
| `IDLE` | 초기 / DONE 후 | 홈 자세 대기, `/arm_status=IDLE` | `/arrival_status=ARRIVED_PICKUP` → `PERCEIVE` |
| `PERCEIVE` | 트리거 수신 | `/pick_target` 최신값 읽기(latched), pose 유효성 체크 | pose 유효 → `PLAN` / depth 무효 → 대기·재시도 |
| `PLAN` | 타겟 pose 확보 | MoveIt IK: 박스 상단 파지 포인트(454~754g 하중 고려) | 해 존재 → `DESCEND` / 해 없음 → `FAILED`(파워트레인에 재정렬 요청) |
| `DESCEND` | IK 성공 | 저속 하강 0.02 m/s, **그리퍼 전류 감시** | 전류 ≥ 임계 → `GRASP_CHECK` |
| `GRASP_CHECK` | 하강 정지 | CLOSE_GRIPPER, 전류 ≥ 임계 유지 확인 | 성공 → `LIFT` / 실패 → `DESCEND` 재시도(횟수 제한) |
| `LIFT` | 파지 성공 | 수직 리프트 → `CARRY_POSE` 고정 | 완료 → `CARRY` |
| `CARRY` | 운반 자세 | 자세 락 유지, **그리퍼 전류 주기 체크**, `/arm_status=CARRYING` | 전류 급감(DROP) → `REGRASP` / `ARRIVED_DROP` → `RELEASE` |
| `REGRASP` | DROP 감지(−2점) | `/arm_status=FAILED`(파워트레인 재정렬 유도) → `PERCEIVE` | 재시도 한계 초과 → `ABORT` |
| `RELEASE` | 하역 지점 도착 | 그리퍼 개방, 박스 내려놓기 | 완료 → `DONE` |
| `DONE` | 하역 완료 | `/arm_status=DONE`(파워트레인 재출발) → `IDLE` | → `IDLE` |
| `LOCKED` | `/chassis_mode∈{ROUGH_TERRAIN,CORNERING,FOLLOW_LEAD}` | 현재 관절각 홀드(진동 보호) | `DRIVING` 복귀 → 직전 상태 |

**자세 락(`LOCKED`)은 모든 상태에 우선하는 인터럽트.** CARRY 중 지형 모드면 CARRY_POSE 그대로 유지 + 전류 감시 계속. 구간1·3·4·5는 팔이 사실상 `IDLE`+`LOCKED` 조합으로만 동작.

---

## 5. 핸드셰이크 시퀀스 (구간2)

```
파워트레인                 인식노드(Phase2)          로봇팔 FSM
  │  DETECT_BOX(주행중)  ◀── /pick_target(latched) ──┐(상시 publish)
  │  ALIGN_BOX 정렬완료                               │
  ├── /arrival_status=ARRIVED_PICKUP ───────────────▶ IDLE→PERCEIVE→PLAN→DESCEND→GRASP→LIFT→CARRY
  │                                                   ├── /arm_status=EXECUTING/CARRYING ──▶ (대기)
  │  CARRY_MODE 주행 시작 ◀── (DONE 아님, 파지유지) ──┤
  │  지형 진입 → /chassis_mode=ROUGH_TERRAIN ────────▶ LOCKED(CARRY_POSE 유지)
  │  하역지점 → /arrival_status=ARRIVED_DROP ────────▶ RELEASE→DONE
  ├── /arm_status=DONE ◀──────────────────────────────┘
  │  재출발(레인 추종 재개)
```

DROP 발생 시: 팔이 `/arm_status=FAILED` → 파워트레인이 다시 `ALIGN_BOX`로 복귀 → `ARRIVED_PICKUP` 재발행 → 재파지 (drawio의 `DROP! → ALIGN_BOX` 루프와 일치).

---

## 6. 구현 방식 결정 (가) + 남은 선결 과제

### 결정: '가' — MoveIt 단일 경로 (2026-06-29)

> ⚠️ 처음엔 "MoveIt 실하드웨어 경로 부재 → position_node 직접 제어(A)"로 갔으나,
> **upstream/main에 이미 `moveit_dynamixel_bridge`(PR #9, commit 08ac318)가 있음**을
> PR 점검 중 발견. 이 브릿지가 `/arm_controller/follow_joint_trajectory` 액션을 구현해
> **MoveIt → 실제 다이나믹셀** 경로를 이미 뚫어놓음 → 전제가 바뀌어 **가로 전환**.

- **A. 모션 경로**: MoveIt 단일. FSM은 `move_action`(MoveGroup)에 목표 pose만 던지고
  IK·경로계획은 MoveIt이 수행 → `arm_controller` → `moveit_dynamixel_bridge` → 서보.
  IK 직접 구현(`_solve_ik`) 숙제 제거됨. position_node 직접 제어는 폐기(포트 경합 회피).
- **A-1. IK 방식 — HW-7 실측 반영 (2026-07-05)**: 위 A는 URDF가 팔 전체 자유도(5축)를
  반영했다는 전제였으나, 실측 결과 URDF/SRDF가 아직 `joint_1~3` **3축만** 반영(CAD 미완성)
  → MoveIt 6DOF pose IK가 **현재 실제 tip pose에도 `NO_IK_SOLUTION`** 반환(자유도 3 < 목표
  6이라 원천 불가). **임시 우회**: `arm_fsm_node`의 `ik_mode` 파라미터(기본 `'analytic'`)로
  FK(`/compute_fk`)+수치 자코비안 위치-only 3DOF IK를 사용해 `/arm_controller/joint_trajectory`에
  직접 명령. A의 MoveGroup 경로는 폐기 아님 — URDF가 5축으로 확장되면 `ik_mode:='moveit'`로
  전환해 그대로 재사용. 상세: `WORK_STATUS.md` HW-7 섹션.

### 남은 선결 과제 (대부분 **브릿지 측** 작업으로 이관)

**B. 브릿지 effort(전류) 발행** ✅ 완료 (2026-06-29), **부재 서보 방어 강화** (2026-07-04, 커밋 `3bed8bd`)
브릿지가 PRESENT_CURRENT(126,2 signed)~PRESENT_POSITION(132,4)을 연속 10바이트
SyncRead 블록으로 한 번에 읽어 `/joint_states`에 position+effort(**raw signed current**)
동시 발행. FSM은 effort에서 읽도록 이미 구현. (임계값 실측 캘리브만 남음)
**신규**: `_enable_torque()`가 성공 여부를 반환하도록 바꿔 **토크 활성화에 성공한 ID만**
SyncRead에 등록 — 실하드웨어에서 일부 관절 서보가 미연결/무응답이어도 나머지 서보는
정상 구동(이전엔 하나라도 없으면 SyncRead 전체가 얽힘). `publish_joint_states()`도
`txRxPacket()` 반환값을 더 이상 체크하지 않고 응답 온 ID만 처리.

**C. 그리퍼 실행 경로** ✅ 완료 (2026-06-29) — 같은 브릿지 노드에
`/gripper_controller/follow_joint_trajectory` 액션 서버 추가(단일 서보 양 핑거 미러링,
결정 B). 그리퍼 ID·미터↔틱 매핑·열림/닫힘 전부 파라미터화(`gripper_ids` 기본 [5]).
남은 것: `gripper_open/close_tick`·전류 임계값 실측 캘리브, `gripper_ids` 실제 ID 확정.

**D. status enum 합의** (파워트레인 팀) — §3 잠정값(`ARRIVED_PICKUP`/`ARRIVED_DROP`/`DONE`...)
확정. drawio엔 핸드셰이크가 암묵적이라 명시 합의 필요.

**E. TF 연결** ✅ 완료 (2026-06-29), **CAD 오프셋 반영 + 손목 카메라 추가** (2026-07-04, 커밋 `3bed8bd`) —
`robot_arm_description/launch/camera_tf.launch.py`.
perception 이 pyrealsense2 직접 사용(드라이버 미사용)이라 아무도 TF 를 안 냄 → 전방 RGB-D
(차체 고정) static TF 2단 발행: `base_link→camera_link`(장착 오프셋, **CAD 실측값** `x=0.123,
z=0.082, pitch=-0.26` — 2026-06-29엔 placeholder=0이었음) + `camera_link→camera_color_optical_frame`
(REP-103 optical 회전 고정). tf2_echo 검증 완료.
**신규**: 손목 RGB 카메라(그리퍼 위) `base_link→wrist_camera_link` static TF 추가(CAD 실측값
`x=0.040, z=0.295`) — 단, **홈 포즈 기준 static placeholder**라 팔이 움직이면 실제 위치와
어긋남. **URDF 관절 통합(eye-in-hand)은 여전히 후속 과제.**

**F. 구간4 제설 주체** — 팔로 치울지/밟고 갈지 미정(D 보류).

**G. 박스 파지 stroke** — 95mm 큐브, 454~754g. 그리퍼가 95mm를 잡는지 URDF 핑거 범위 확인(E 가능 확정).

---

## 7. 다음 단계

1. ✅ `arm_fsm_node.py` 스켈레톤(가 방향) — `dynamixel_control`, 빌드+mock 스모크테스트 통과.
2. ✅ **브릿지 확장**(§6-B effort + §6-C 그리퍼) — 완료(2026-06-29). 전류 임계 실측 캘리브만 남음.
3. ✅ **TF**(§6-E) 카메라→base_link — `camera_tf.launch.py` 완료(2026-06-29). 장착 오프셋 실측만 남음.
4. ✅ `_carry_pose()` 구현 — 완료(2026-06-29). TF로 현재 TCP +Z(`lift_height`) 리프트.
5. status enum 합의(§6-D) 후 `arm_fsm_node.py` 상단 상수 교체. **(미완)**
6. 구간2 단독 통합 테스트 (MoveIt+브릿지+`camera_tf`+perception 기동 → `/arrival_status` mock → 파지 → `/arm_status=DONE`). **(다음 단계)**
7. ✅ **HW-2~6 실하드웨어 테스트** (2026-07-04, 커밋 `3bed8bd`) — 젯슨 ARM64 이미지 전환,
   브릿지 부재 서보 방어, 손목 카메라 TF, `/perception/debug_image`+`stream_node`(SRT 원격
   모니터링). 커밋 시점 미커밋 상태로 서보 ID 0 이상 디버깅(`ros2_ws/*_servo*.py`) 진행 중
   이었음 — 다음 세션에서 원인 확인 필요.
8. ✅ **HW-7 실하드웨어 픽 시퀀스 검증 + analytic IK 우회** (2026-07-05, 커밋 `3048f02`) —
   URDF 3축 한정으로 MoveIt 6DOF IK 불가 실측 확인(§6-A-1), `ik_mode='analytic'`(FK+수치
   자코비안 위치 IK) 구현·전환. 실기 검증: bottle 인식 → analytic IK → 팔 하강 → 그리퍼
   닫힘 → effort 판정까지 실제 모터로 end-to-end 확인(방향까지 맞춘 정밀 파지는 URDF 5축
   확장 후). FK 서비스 재귀 spin 타임아웃 버그를 헬퍼 노드로 분리해 수정. 서보 디버깅
   스크립트(`check_servo.py` 등) 정식 커밋(ID 0 이상 근본 원인은 미확정 — 다음 세션 확인).
