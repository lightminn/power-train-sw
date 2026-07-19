# WP8 미션 핸드셰이크 — 실팔 E2E 하네스 + 크로스 감사 + 활성화 게이트 (설계)

2026-07-19. 승인: 사용자 (A안 — "실팔 E2E 하네스 + 크로스 감사 + 활성화 게이트 문서").

## 0. 배경 — 탐색으로 확정한 현재 상태

핸드셰이크는 **양쪽 모두 계약 v2로 이미 구현돼 있다**. 남은 것은 구현이 아니라
검증·활성화다.

- 팔 (`~/extreme-robot`, `arm_fsm_node.py`, PR #17 이후): `MISSION_STOP` + 같은
  mission_id `ArrivalStatus` **conjunction**(순서 무관)만이 작업 개시/재개 권위.
  완료 권위는 픽업=`CARRYING_LOCKED`, 하역=`STOWED_LOCKED`(팁 정착 실측 후 발행).
  `DONE`은 권위 아님. `chassis_mode` 1.0 s 워치독 = 끊기면 default-deny 잠금.
  실패는 완전 래치. stamp 신선도 검사(0·미래·동일·역행 거부). 10 Hz heartbeat.
- 우리: `chassis/mission.py`의 v2 `MissionSupervisor`(830줄 중 232행~)가
  `chassis_node`에 배선 완료 — `/chassis_mode`·`/arrival_status` 단독 소유,
  wheel_stop·authority-zero·grip_lost 연동, `~/mission_arrive_pickup`/`_drop`
  Trigger 서비스, 재발행(`arrival_republish`), FAILED 대기. 전부
  **`contract_v2_verified=false`(읽기 전용 안전 파라미터) 뒤에 잠김**.
- 어휘 정본 = 우리 `powertrain_ros/contract.py` (팔 코드와 값 일치 확인됨).

**갭**: ① 실팔 노드를 상대로 맞물려 돈 적이 0회(우리 테스트는 자작 fake 상대)
② `contract_v2_verified`를 true로 켜는 기준이 미정의 ③ 팔은 headless로는
`DESCEND`에서 멈춤(MoveIt/실서보 필요) → 풀사이클은 협조 세션 몫.

## 1. 산출물

1. **E2E 하네스** `scripts/wp8_handshake_e2e.sh` (젯슨 호스트에서 실행) +
   probe `ros2/src/powertrain_ros/powertrain_ros/wp8_handshake_probe.py`
   (rclpy 시나리오 드라이버, 콘솔 엔트리포인트 등록).
2. **크로스 감사 결과** — 발견 불일치는 **우리 쪽만** 수정(팔 레포 무접촉).
3. **활성화 게이트 문서** — 이 스펙 §5가 정본, 핸드오프 보고서에 링크.
4. **협조 세션 런북** — 실서보로 Phase 1·2를 재실행하는 절차(하네스 재사용).

## 2. 하네스 구조

```
scripts/wp8_handshake_e2e.sh  (젯슨 호스트)
 ├─ docker exec powertrain_ros:  chassis_node  (도메인 77, fake:=true,
 │     contract_v2_verified:=true, mission_contract_owner:=chassis_supervisor,
 │     safety_required:=false — BENCH/FAKE 전용 조합, 프로세스 그룹 신규)
 ├─ docker exec ros2_humble:     arm_fsm_node  (도메인 77, 팔 레포 설치본 그대로,
 │     파라미터 기본값 — 수정·주입 없음)
 └─ docker exec powertrain_ros:  wp8_handshake_probe (도메인 77)
        · /arm_status·/chassis_mode·/arrival_status 구독 + 전이 기록
        · ~/mission_arrive_pickup 등 서비스 콜, fake /pick_target 발행
        · 시나리오 스텝 실행 → 단언 → PASS/FAIL 요약(JSON 한 줄 + 사람용 표)
 정리: 각 기동을 setsid로 묶고 종료 시 killpg (§9-5 좀비 방지).
```

- 도메인 77로 운용 스택(도메인 0)과 완전 격리 — 라이브 perception·콘솔 무영향.
- probe는 **관측+자극만** 한다. 판정 로직은 순수 함수
  (`powertrain_ros/wp8_scenario.py`)로 분리해 x86 pytest로도 검증.
- 팔 컨테이너가 없으면 하네스는 Phase 2만 실행하고 Phase 1을 SKIP으로 보고
  (exit 3). 운용 스택 간섭 금지: 도메인 0 프로세스는 건드리지 않는다.

## 3. 시나리오 (Phase 1 — 실팔, headless 한계까지)

> **개정(07-19 구현 리뷰)**: v2 supervisor는 READY→DRIVE 전이에 팔의 신선한
> 잠금자세 heartbeat(`STOWED_LOCKED`/`CARRYING_LOCKED`, 0.5 s)를 요구한다
> (`mission.py` READY 분기). 실팔 headless는 TF 부재로 `STOWED_LOCKED`에 도달
> 못 할 수 있고, 그 경우 chassis는 `STOW_REQUEST` 유지 + DRIVING 미발행 +
> 서비스 거부 = **fail-closed가 정답 동작**이다. 판정은 2분기: ⓐ잠금 heartbeat
> 도달 시 아래 표의 conjunction 경로 ⓑ미도달 시 fail-closed 경로(작업 수락·
> ArrivalStatus·MISSION_STOP 전무 + 거부 근거 marker)를 PASS로 인정하고, 표
> 4~6(SIGSTOP/재개)은 ⓐ에서만 수행한다. ⓑ가 나오면 표 2~6의 실증은 협조
> 세션(실서보, TF 가용)으로 이월된다. baseline의 "DRIVING 발행" 기대도 같은
> 이유로 "작업 불허 모드(LOCK_MODES ∪ STOW_REQUEST) 발행"으로 정정한다.

| # | 자극 | 기대(단언) |
|---|---|---|
| 1 | chassis FAKE 시동, DRIVING 발행 시작 | 팔 heartbeat 수신(10 Hz), 팔 상태 IDLE/LOCKED — 작업 미개시 |
| 2 | `~/mission_arrive_pickup` 호출 | `/chassis_mode`=MISSION_STOP 전환 → **실정지 settle 후** `/arrival_status`(ARRIVED_PICKUP, mission_id=N) — 이 순서 역전 없음 |
| 3 | (fake `/pick_target` 발행) | 팔 `WORK_READY`→`PERCEIVING`(→PLANNING) 전이 = conjunction 수락 실증. stamp·QoS 상성 실증 |
| 4 | chassis_mode 발행 1.5 s 중단(프로세스 SIGSTOP) | 팔 워치독 default-deny → LOCKED 전이 |
| 5 | 발행 재개(SIGCONT) | 같은 mission_id conjunction으로 팔 재진입(LOCKED→PERCEIVE) |
| 6 | ArrivalStatus 재발행 관측 | 우리 재전송이 팔에서 중복 부작용 없음(이미 진행 중이면 무시) |
| 7 | headless 한계 도달(DESCEND 정지 or move_action 미준비 경고) | 기록 후 Phase 1 종료 — FAIL 아님(문서화된 한계) |

## 4. 시나리오 (Phase 2 — 계약 충실 fake-arm, 풀사이클+오류 경로)

fake-arm은 probe 내장 rclpy 노드로, 팔 v2 의미론을 재현(conjunction 대기,
지연 후 `CARRYING_LOCKED`/`STOWED_LOCKED` 발행, stamp 규칙 준수, FAILED 래치).

| # | 시나리오 | 기대 |
|---|---|---|
| 1 | 정상 풀사이클 | DRIVE→MISSION_STOP→ARRIVED_PICKUP→CARRYING_LOCKED→재출발(DRIVING 발행 재개+allow)→ARRIVED_DROP→STOWED_LOCKED→재출발. mission_id 단조 증가 |
| 2 | 완료 무응답 | 재전송 N회 → FAILED, **재출발 금지 유지**(모드 MISSION_STOP 고정) |
| 3 | 이전 mission_id의 뒤늦은 완료 | 무시(재출발 없음) |
| 4 | 팔 FAILED 래치 | 우리도 FAILED 대기(사람 개입 경로), `resolve_failure` 흐름 확인 |
| 5 | 재출발 직후 완료 중복 수신 | 무시(이중 재출발 없음) |

## 5. `contract_v2_verified` 활성화 게이트 (정본)

기본값 false 유지. **다음 3단 전부 통과 후에만** launch에서 명시 활성:
1. 이 하네스 Phase 1+2 PASS (젯슨, 실팔 headless).
2. 팔팀 협조 세션에서 실서보 풀사이클 1회 PASS (같은 하네스 Phase 1 재실행 +
   실제 CARRYING_LOCKED/STOWED_LOCKED 도달, 양팀 입회).
3. 활성 커밋에 게이트 증적(하네스 출력) 링크.
활성 방식: `wp5_control.launch.py`/FULL 런치의 명시 인자만(코드 기본값 변경 금지).

## 6. 오류 처리·안전

- 하네스는 도메인 77 밖(운용 스택)을 절대 건드리지 않는다. 모터 명령 경로는
  fake:=true라 실 CAN 송신 없음(FAKE 코너).
- 팔 노드는 그들 레포 설치본을 파라미터 기본값으로만 실행 — 팔 레포 파일 수정
  금지, 실서보 없는 환경에서 토크 인가 없음(arm_fsm은 MoveIt 미준비 시 경고만).
- 타임아웃마다 상태 덤프 후 FAIL — 무한 대기 없음. 종료 시 killpg + 잔류
  프로세스 검사(§9-5).

## 7. 검증 계획 (완료 선언 규칙 준수)

- 순수 판정기(`wp8_scenario.py`) pytest — 전이 시퀀스 수락/거부 케이스.
- 하네스 **음성 대조**: 시나리오 4(워치독)에서 기대 전이를 일부러 뒤집은
  변형 실행으로 하네스가 실제 FAIL을 내는지 1회 증명.
- 젯슨 실행 2회(멱등) → PASS 출력 보관, 핸드오프 §2 기록.

## 8. 비범위 (YAGNI)

- 팔 레포 수정·파라미터 튜닝 일절 없음.
- section_supervisor의 실 인식 이벤트 토픽 계약(마커 등) — 크로스팀 미확정,
  별도 사이클.
- `contract_v2_verified` 실제 활성(= 게이트 2단, 협조 세션 이후).
- mission_trigger(/detected_objects 자동 도착 판정) 튜닝 — 수동 서비스 트리거만
  사용(자동 트리거는 대회 코스 세팅에서).
