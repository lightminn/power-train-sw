# 비전 자동 도착 판정 + 능동 정렬 접근 (Vision Arrival & Approach) 설계

- 상태: 설계 확정 (2026-07-27)
- 트랙: 파워트레인 자율주행 (WP6/WP7 연계, WP8 핸드셰이크 상류)
- 관련 계획: `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`,
  `docs/plans/2026-07-13-wp5.2-arm-collaboration-safety-plan.md`
- 메모리: `robot-arm-team-resources` (ARRIVED = 완전자율 결정, 2026-07-27)

---

## 1. 배경 & 문제

대회 채점이 **완전 자율**이므로, 로봇이 픽업/드롭 지점에 스스로 도착·정렬하고
로봇팔에게 `ArrivalStatus = ARRIVED_PICKUP/ARRIVED_DROP`를 **자동 발행**해야 한다.

현재 프로덕션 경로(`chassis_node`가 소유한 `MissionSupervisor`)의 도착 트리거는
**수동 서비스 `~/mission_arrive_pickup` / `~/mission_arrive_drop` (std_srvs/Trigger)뿐**이다.
비전 자동 트리거 코어(`MissionTrigger`)는 존재하나 **SUPERSEDED 레거시 `mission_node.py`에만
배선**돼 있어 프로덕션에서 쓰이지 않는다.

로봇팔은 3축이며, 팔이 물체에 닿으려면 차체가 **팔 작업영역 안(전방 거리 + 횡방향 정렬)**에
멈춰야 한다. 거리만 보는 기존 트리거로는 불충분하다 — **능동 최종접근(정렬)**이 필요하다.

### 확정된 제품 결정 (2026-07-27 사용자)

1. **정렬 범위 = 능동 최종접근**: 대상 감지 후 파워트레인이 마지막 구간을 폐루프로
   조향·크립해 박스를 팔 작업영역 중앙에 맞추고, 정렬 완료 시 ARRIVED 발행.
2. **측정 신호 = `DetectedObject.pose.position` + 프레임 확정**: 팔팀과 pose 좌표
   프레임을 계약으로 확정하고 TF로 base_link에 투영. x=전방거리, y=횡오차.
3. **실패 처리 = 재시도 후 정지 + 콘솔 알림**: 정렬 타임아웃 내 재시도 N회, 소진 시
   소프트 정지(cmd 0) + 콘솔 경고 배너, 운용자 ops 개입 대기.

---

## 2. 범위 (Scope)

### 2.1 이 WP가 만드는 것

- **`approach_controller_node`** (신규 behavior 노드 1개): `/detected_objects`를 구독해
  대상을 감지·lock하고, 폐루프 정렬 접근을 `/autonomy/cmd_vel`로 제안하며, 정렬 완료 시
  `chassis_node`의 기존 `~/mission_arrive_pickup/_drop` 서비스를 **자동 호출**한다.
- **`approach.py`** (신규 순수 코어, `motor_control/chassis/`): 하드웨어·ROS 의존 없는
  정렬 상태기계 + 제어법. pytest로 전수 검증.
- **`lane_follower_node` 소규모 수정**: `/approach/active` 구독 → 접근 활성 동안
  `/autonomy/cmd_vel` 제안 억제(협조 양보). `lead_follower`의 `/follow/active` 패턴 계승.

### 2.2 재사용 (무변경 또는 최소)

- `MissionTrigger` (`chassis/mission_trigger.py`): 클래스·거리·디바운스·쿨다운 게이트.
  **대상 감지·lock 판정**에 그대로 사용.
- `MissionSupervisor.request_work()` (`chassis/mission.py`): 자동 경로가 **수동 서비스와
  동일 진입점**을 호출. `chassis_node`는 무변경(서비스 이미 존재).
- MISSION_STOP 발행 → 실제 정지 확인 → `ArrivalStatus` 발행 → 팔 DONE → 재출발 →
  쿨다운: **전부 기존 하류 로직 재사용**.

### 2.3 YAGNI (명시적 제외)

- 다중 동시 대상 추적 — 가장 가까운 유효 대상 1개만.
- orientation 기반 yaw 정렬 — 3축 팔이 흡수, 파워트레인은 횡오차+거리만.
- `section_supervisor` 통합 arbiter — 후속 WP. 이번엔 협조 양보 래치로 충분.
- 레거시 `mission_node.py` 자동트리거 삭제 — SUPERSEDED라 프로덕션 무영향. 후속 정리.

---

## 3. 아키텍처 & 데이터 흐름

```
                          ┌──────────── 재사용(기존, 무변경) ─────────────┐
/detected_objects ─┐      │  chassis_node.MissionSupervisor               │
   (팔 YOLO, 3D)   ├─▶ [approach_controller_node] ──정렬완료──▶ ~/mission_arrive_pickup/_drop (Trigger 서비스)
/odom ─────────────┤      │        │                                      │  → MISSION_STOP(/chassis_mode)
TF(base_link) ─────┘      │        ├─▶ /autonomy/cmd_vel (접근 크립)      │  → 정지 확인
                          │        ├─▶ /approach/active (Bool 래치)       │  → ArrivalStatus = ARRIVED_*
                          │        └─▶ /approach/state (String 진단)      │  → 팔 DONE → 재출발
                          └───────────────────────────────────────────────┘
                                       │ /approach/active
                          lane_follower_node: active 동안 /autonomy/cmd_vel 제안 억제(양보)
```

**단일 writer 규율**: `/autonomy/cmd_vel`은 여러 behavior 노드가 공유하되 **한 번에 하나만**
제안한다(기존 원칙). 접근 활성 시 lane_follower가 `/approach/active`를 보고 물러난다.

**authority 미소유**: 접근 노드는 `/cmd_vel`을 직접 쓰지 않고 authority를 잡지 않는다.
`/autonomy/cmd_vel`로 **제안**만 하며, 실제 구동 전달은 `chassis_node` command authority가
결정한다(기존 계약 준수).

---

## 4. 접근 제어 상태기계 (순수 코어 `approach.py`)

### 4.1 상태

```
SEARCHING ──대상 lock(MissionTrigger: 거리<engage_m·연속N·conf≥min)──▶ APPROACHING
   ▲                                                                      │
   │◀── 쿨다운(미션 완료) ── DONE ◀── 서비스 ACK success ── ARRIVED_FIRED  │
   │                                                                      │
   └── 재시도 소진 / align_timeout ── FAILED_HOLD                          │
APPROACHING ── |y|<lat_tol & |x−stop_m|<dist_tol & 정지확인 ──▶ ALIGNED ──서비스 호출──▶ ARRIVED_FIRED
   │  ├ 대상 lost(연속 lost_frames) → BACKOFF(뒤로 크립) → 재획득 or 재시도++
   │  ├ align_timeout_s 경과 & 재시도<max_retries → BACKOFF 후 재접근
   │  └ 재시도≥max_retries → FAILED_HOLD
```

- `SEARCHING`: `/autonomy/cmd_vel` 미제안(`/approach/active=false`). lane_follower가 주행.
- `APPROACHING`: `/approach/active=true`. 폐루프 크립 제안.
- `ALIGNED`: 정렬·정지 확인 완료. `~/mission_arrive_*` 서비스 호출.
- `ARRIVED_FIRED`: 서비스 ACK 성공. 하류 MissionSupervisor가 미션 수행. `/approach/active`는
  유지하되 `/autonomy/cmd_vel`=0 (MissionSupervisor가 allow_drive=False).
- `DONE`: 팔 DONE → 재출발. 해당 클래스 `cooldown_s` 무시 설정 후 SEARCHING 복귀
  (`/approach/active=false`, lane 재개).
- `FAILED_HOLD`: `/approach/active` 유지 + `/autonomy/cmd_vel`=0 + `/approach/state=FAILED`.
  콘솔 경고. 운용자 ops 개입(수동 `~/mission_arrive_*` 또는 리셋) 대기.
  ⚠️ 이것은 접근 노드의 **소프트 정지**(0 제안)이지 안전-latched `MOTION_HOLD`가 아니다 —
  접근 노드는 authority/hold_source를 소유하지 않는다(§3 계약). 실제 정지는 authority가 0을
  전달해 달성. 운용자가 별도 E-stop을 원하면 기존 ops 채널로 명령한다.

### 4.2 제어법 (APPROACHING)

입력: 최신 `pose.position` (TF → base_link). `x`=전방거리, `y`=횡오차.

```
ω = clamp(−k_yaw · y, ±ω_max)                    # 박스를 정면에 두게 조향
v = clamp(k_dist · (x − stop_m), 0, v_approach_max)  # stop_m까지 전진, 후진 없음(접근 중)
```

- 크립 상한 `v_approach_max`는 낮게(예: 0.15 m/s) — 저속 정밀 접근.
- **정렬 완료 판정**: `|y| < lat_tol` AND `|x − stop_m| < dist_tol` AND `|odom.v| < v_settle`
  (실제 정지 확인). 3조건 동시 충족 시 ALIGNED.
- **BACKOFF**: 대상 lost 시 `v = −backoff_creep`를 짧게(뒤로 물러나 재획득 시야 확보),
  `backoff_time_s` 후 SEARCHING/재접근.

### 4.3 강건·안전 게이트

- **정지 후 발행**: ARRIVED 서비스 호출은 실제 정지(`v_settle`) 확인 후에만. "한 프레임
  깜빡임 급정거 금지" 원칙 계승. MissionSupervisor도 wheel_stop 재확인 → 이중 방어.
- **쿨다운**: DONE 후 같은 클래스 `cooldown_s` 무시(재출발 시 눈앞 물체 무한루프 방지).
  `MissionTrigger.mission_finished()` 재사용.
- **stale-pose 감속**: `/detected_objects` freshness(`pose_stale_s`) 초과 시 크립 금지 →
  감속·정지. 오래된 pose로 전진하지 않는다.
- **프레임 미해석 안전 실패**: `header.frame_id` 미제공 또는 TF 미해석 시 APPROACHING
  진입 거부 → SEARCHING 유지 + 경고(엉뚱한 거리 크립 금지).

### 4.4 튜닝 파라미터 (전부 ROS declare, HIL 캘리브)

`engage_m`, `stop_m`, `lat_tol`, `dist_tol`, `k_yaw`, `k_dist`, `omega_max`,
`v_approach_max`, `v_settle`, `backoff_creep`, `backoff_time_s`, `align_timeout_s`,
`max_retries`, `consecutive`, `cooldown_s`, `min_confidence`, `lost_frames`,
`pose_stale_s`, `pickup_class`, `drop_class`.

기본값은 벤치용 보수값. `stop_m`·`lat_tol`은 3축 팔 리치 확정 후 실서보 HIL에서 캘리브.

---

## 5. Cross-team 의존성 (팔팀 relay 필요)

1. **`DetectedObject.pose` 좌표 프레임 확정 — 유일한 하드 blocker**:
   - `DetectedObjectArray.header.frame_id`에 실제 카메라 프레임 명시(예:
     `d435i_color_optical_frame`).
   - 그 프레임이 **우리 TF 트리에 존재**(정적 extrinsic: 팔 카메라 ↔ base_link).
   - `pose.position`이 그 프레임 기준 미터 단위임을 확인. `orientation`은 불필요.
   - **완충**: 미제공/미해석 시 노드는 SEARCHING 유지 + 경고(§4.3) — 계약 확정 전 안전 실패.
2. **팔 작업영역 수치(`stop_m`, `lat_tol`)**: 3축 팔 리치 확정 후 실서보 HIL에서 캘리브
   (WP5.2 Task 7 잔여 서보 HIL과 같은 세션 후보).
3. **`class_name` 어휘 + pickup/drop 매핑**: 팔팀 YOLO 클래스명 확정
   (기본 후보 `box`→ARRIVED_PICKUP, `dropzone`→ARRIVED_DROP).

---

## 6. 테스트 전략 (완료선언 규칙 준수 — 실행·E2E 필수)

- **순수 pytest** (`test_approach.py`): 상태 전이(SEARCHING→APPROACHING→ALIGNED→FIRED),
  제어법 클램프, BACKOFF, 재시도 소진→FAILED_HOLD, 쿨다운, stale-pose 감속, lost→backoff,
  프레임 미해석 시 SEARCHING 유지. `mission_trigger` 14 테스트 패턴 계승.
- **rclpy 노드 스모크** (젯슨, 도메인77 격리): fixture로 `/detected_objects`+`/odom`+TF
  발행 → `/autonomy/cmd_vel` 크립·`/approach/active`·정렬 완료 시 `~/mission_arrive_pickup`
  서비스 호출을 실응답으로 단언 → killpg. ARM-CON/CMASK 스모크 패턴.
- **E2E** (모터 무전원): 합성 대상 pose 스트림 → 접근 크립 관측 → 정렬 완료 →
  MissionSupervisor MISSION_STOP → ArrivalStatus=ARRIVED_PICKUP까지 전 체인 검증.
  `scratchpad/e2e_drive.py` 패턴 계승. CAN 바퀴출력만 미관측.
- **음성대조**: 정렬 tol 게이트를 제거하면 급발진/오정지가 재현되는지로 게이트 자체를 증명.
- **콘솔 runtime_smoke**: FAILED_HOLD 배너·`/approach/state` 렌더 확인.

---

## 7. 구현 분담

- 스펙·계획·검증 = Claude. 실제 구현 = Codex 위임(맥락·파일경로·완료조건 명확 전달).
- 검증 = Claude(diff 정독 + 순수 pytest 재실행 + 젯슨 rclpy 스모크 + E2E + 음성대조).

---

## 8. 완료 기준 (Acceptance)

1. `approach.py` 순수 pytest 전수 green + 음성대조 실증.
2. 젯슨 rclpy 스모크: 합성 입력 → 정렬 크립 → `~/mission_arrive_pickup` 자동 호출 관측.
3. E2E(무전원): 자동 도착 → MISSION_STOP → ArrivalStatus=ARRIVED_PICKUP 전 체인 PASS.
4. lane_follower 양보 검증: `/approach/active=true` 동안 lane 제안 억제 확인.
5. 프레임 미확정 시 안전 실패(SEARCHING 유지) 확인.
6. (HW 게이트, 후속) 실서보 HIL에서 `stop_m`·`lat_tol` 캘리브 + 실주행 정렬 검증.
