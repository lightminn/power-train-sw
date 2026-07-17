# A/B/C 개선 프로그램 통합 설계 (2026-07-17)

아이디어 세션(Codex + agy×2 + Claude 서브에이전트×3, 6소스)의 수렴 결과를
계획 정본(§10 중단 기준·§11 금지 목록)·트랙 정책·크로스팀 경계로 필터링해 확정한
통합 설계다. 실행 구조는 **통합 스펙 1건 + 배치 6개 순차**(A1→A2→A3→B1→B2→C)이며,
배치마다 Codex 위임 → 3환경 검증 → 커밋 체인을 따른다.

정본 관계: 이 문서는 `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`
(마스터플랜)의 하위 실행 설계다. 충돌 시 마스터플랜이 우선하되, 본 프로그램이
명시적으로 개정하는 항목(§3.5 콘솔 헌장)은 개정 커밋 후 본 문서가 근거가 된다.

## 0. 운용 원칙 (사용자 지시, 2026-07-17)

1. **실전(대회 당일)의 모든 사용자 조작은 두 표면으로만 한다:
   ①operator_console GUI ②DualSense.** SSH·추가 창·별도 툴은 실전 조작 경로에서
   제외하고 정비·비상 폴백으로만 남긴다.
2. 콘솔 안에서도 팝업/별도 창을 띄우지 않는다 — 기존 창 안 **인라인 2단 확인**만 쓴다.
3. **모든 키매핑·chord는 임시(versioned initial candidate)다.** HIL·운전자 피드백 후
   변경을 전제하며, 코드(단일 mapping 모듈)·스펙·문서 세 곳에 임시임을 명시한다.
4. latched E-stop의 reset→arm은 **콘솔 패널 전용**이다(원인을 보며 판단 후 해제).
   조종기 chord에는 E-stop reset을 두지 않는다.

## 1. 배치 개요

| 배치 | 내용 | 규모 |
|---|---|---|
| A1 | 의미론 정합 2건 (○ E-stop 전역 latch, min_rev 정합 검사) | S |
| A2 | 현장 복구·운용 표면 (ops broker + chord + 콘솔 패널 + 브링업 무SSH + 햅틱) | L |
| A3 | 실행 배선 (WP8 힌트 집행 + 마커 ledger + 큐/dwell 강화) | M |
| B1 | 시뮬 S급 (핀치포인트·클로소이드 곡률·기복 지형·fixture 계약화) | S~M |
| B2 | 시뮬 M/L급 (물리 마찰·스모그 열화·동적 선도 표적·hidden 캠페인) | L |
| C | WP9 환경 degradation 정책 + 문서 정합 | M |

## 2. A1 — 의미론 정합

### 2.1 ○ E-stop 전역 latch 정합

- 현재: ○(`estop_edge`)는 gateway MOTION_HOLD의 한 원인일 뿐이고 `~/clear_hold`로
  풀린다(`remote_input_gateway.py:347-348`). 계획 정본은 "○ = 전역 수동 latched
  E-stop"을 규정한다(마스터플랜 §5 매핑표 :508, 고정 안전 계약 :498).
- 변경: `teleop_command_node`가 `estop_edge` 수신 시 `/teleop/estop`(Bool, edge)을
  발행하고, `chassis_node`가 구독해 `SafetyInterlock.trip_estop("remote_operator")`로
  latch한다. gateway의 기존 hold 진입은 유지한다(이중 방어).
- 해제 경로는 기존 `~/reset_estop`(활성 E-stop 원인 잔존 시 거부) → `~/arm` 분리
  흐름 그대로이며, 표면만 콘솔 패널로 확장된다(§3).
- §11 준수: 통신 단절은 E-stop으로 승격하지 않는다 — 명시적 ○ edge만 trip.

### 2.2 min_rev 정합 검사

- 현재: `min_drive_turns_per_s` 플로어(기본 1.0 turns/s = 0.628 m/s,
  `chassis_manager.py:404-410`, 휠반경 0.10 m)가 어떤 속도 캡과도 대조되지 않는다.
  CARRYING_LOCKED 캡 0.5 m/s(`powertrain_autonomy/controller/profiles.py:39-51`),
  smog 힌트 0.25/ice 0.15(`section_profiles.py:370-371`)와 결정론적으로 충돌한다.
- 변경: ①`ChassisManager`에 플로어 발동 시 바퀴별 카운트 텔레메트리 추가
  ②`chassis_node`가 활성 프로파일 캡·수신 speed_hint가 플로어 환산속도보다 낮으면
  기동 시·변경 시 WARN 이벤트를 journal에 발행.
- **하지 않음**: 플로어 수치 하향(지상 재자격화 필요 — 실차 커미셔닝 몫).

### 2.3 폐기된 후보 (근거 기록)

- ~~gateway stale-hold 자동복구~~, ~~핸드오버 타임아웃 시 이전 소유자 복귀~~:
  WP5.2 계획이 타임아웃→MOTION_HOLD 전이와 "MOTION_HOLD 이후 전환은 **운영자 확인
  뒤에만** 수락"을 명시(wp5.2 plan :708-711)하고, "이전 명령·모드 자동 복원 금지"
  조항이 양 계획 6곳에 있다. 두 후보 모두 위반이므로 폐기한다. 프리모템 시나리오
  1·4의 실제 문제는 latch가 아니라 "SSH 없이는 못 푸는 것"이며, §3의 chord
  (=운영자 확인, 2초)가 계획 철학 그대로 해결한다.

## 3. A2 — 현장 복구·운용 표면

### 3.1 Ops broker 노드 (신규, powertrain_ros)

- TCP **:9001** (레포 전체 미사용 확인). 주행 채널(:9000, 프레임 v2)은 **불변**.
- newline-JSON request/ACK. 요청 = `{schema_version, client_id, nonce, action,
  params, stamp_s}`; 응답 = `{nonce, accepted, state, detail}`. nonce 멱등(중복
  재전송 안전), 순서 역전 거부, 전 요청·결과 journal(EventClient) 기록.
- 다중 클라이언트 허용(조종기 클라이언트 + 콘솔). 접속 핸드셰이크에서
  `client_type ∈ {controller, console}`을 선언하고 broker가 유형별 인가를 강제한다:
  controller는 `clear_transient_hold`·`authority_manual/auto`만, 나머지는 console
  전용(§0-4의 "E-stop reset은 콘솔 전용"을 broker 레벨에서 봉인). 인가 action
  화이트리스트:
  - `clear_transient_hold` — gateway `~/clear_hold` + authority
    `~/authority_clear_hold`를 **한 요청으로 동시** 호출(이중 latch 시나리오 대응)
  - `authority_manual` / `authority_auto` / `authority_idle`
  - `estop_reset`, `arm`, `disarm`, `arm_lock_override` (모두 console 전용)
  - `mission_arrive_pickup/arrive_drop/skip/retry/regrasp_confirmed/clear_grip_lost`
  - `operator_hold` / `operator_resume` — `/section_events` JSON 발행(현행 유일
    주입 경로 유지)
  - `calibration_start` / `calibration_status`(§3.4, IDLE·disarm 상태에서만 수락)
- **ops-state push**: 연결 클라이언트에 5 Hz로 `{authority_mode, gateway_state,
  safety_status, safety_distance_mm, estop_latched, wheel_fault_count, ...}` 송신.
  클라이언트 햅틱·콘솔 패널 표시의 데이터원이다(:5005 텔레메트리는 콘솔이 점유
  중이므로 UDP 경합을 피한다).
- 구현: rclpy 서비스 클라이언트 프록시. 서비스 콜은 타임아웃(1 s)과 실패 ACK로
  감싸 TCP 스레드가 executor를 블로킹하지 않게 한다.

### 3.2 DualSense chord (mapping `recovery-v1-initial-candidate`, ⚠️ 전부 임시)

- **□+CREATE 2 s hold** = `clear_transient_hold`. (기존 모드 chord CREATE+OPTIONS
  1 s와 조합 상이. □=button 3, CREATE=button 8.)
- **D-pad ↓ 1 s** = `authority_manual`(TELEOP 인수), **D-pad ↑ 1 s** =
  `authority_auto`(AUTONOMY 복귀). dpad.y는 현재 완전 미사용 확인
  (dpad.x만 ARM 관절 선택에 사용 — 충돌 없음).
- chord 감지는 클라이언트 측. 요청은 ops 채널로만 전송하고 주행 프레임에는 넣지
  않는다. E-stop reset chord는 **의도적으로 없다**(§0-4).

### 3.3 콘솔 운용 패널 (operator_console 헌장 개정)

- 기존 창 안 패널 추가. 모든 명령 버튼은 **인라인 2단 확인**(1단: 현재 원인·상태
  표시 — 예: E-stop 원인, US-100 거리; 2단: 확인 스트립)을 거친다. 팝업 금지.
- 범위: `clear_transient_hold`, 권한 3종, **estop_reset→arm(콘솔 전용)**, disarm,
  `arm_lock_override`(더 강한 확인 문구), 미션 6종, OPERATOR_HOLD/RESUME,
  브링업(§3.4) 상태·트리거.
- **헌장 개정**: "read-only" → "관측 수신 전용 + 조작은 게이트된 ops 채널 경유만".
  개정 대상: 관측성 계획 :329-334(유일한 계획 문서 조항), `operator_console/`
  독스트링·배너(`app.py:627`)·README·`telemetry.py`, 프로젝트 CLAUDE.md.
- 계획이 주장하나 실존하지 않는 "no-send 계약 테스트"를 실물로 대체: **ops 채널
  클라이언트 외 어떤 송신 경로도 없음**을 봉인하는 계약 테스트를 추가한다
  (텔레메트리·SRT 경로는 수신 전용 불변).

### 3.4 당일 브링업 무SSH화

- **systemd 스택 유닛 신설**(현재 스택 자동기동 유닛 부재 확인 — 기존 유닛은
  텔레메트리·콘솔뿐): 부팅 시 docker compose 스택 기동 + 컨테이너 내
  `wp5_control.launch.py` 실행. `stop_mm`은 launch 필수 인자(기본값 없음)이므로
  `/etc/powertrain/powertrain.env`(EnvironmentFile)에서 명시 주입 — "명시적
  stop_mm" 정책 유지, 커미셔닝 때 파일만 갱신.
- **캘리브레이션 NVM 영속화**: `bl70200_setup.py`에 `--persist-calibration` 추가 —
  풀캘리 성공 확인 후 `motor.config.pre_calibrated=True` +
  `encoder.config.pre_calibrated=True` + `save_configuration`(리부팅 수반) →
  재열거 후 검증. fw-v0.5.6 HALL 경로 지원 확인됨(HALL_POLARITY/PHASE_CALIBRATION
  상태 존재). NVM 쓰기는 `bl70200_setup.py` 단일 경로 원칙 유지, 영속화 후
  `--read` 전수 대조 포함.
- **부팅 자격화(무회전)**: preflight가 축별 `pre_calibrated`·에러 플래그·HALL
  상태를 모터 회전 없이 판독 → broker 경유 콘솔 표시. 자격화 실패 축이 있으면
  arm 거부. 전압 판독 포함(프리모템 시나리오 5 겸용).
- **캘리 서비스 = 폴백**: 자격화 실패 시에만 콘솔에서 트리거. "바퀴 전부 리프트
  확인" 인라인 확인 필수(모터 회전 수반 — FULL HIL 규율).
- **조립 전 벤치 게이트(신설, 핸드오프에 기록)**: wheels-up에서 캘리→영속→
  **전원 사이클 3회 × 6축 직진입 closed-loop 재현** 통과가 차체 조립 선행조건.
- 완료 후 CLAUDE.md·Notion의 "캘리 RAM-only, 전원 사이클마다 재캘리" 서술 갱신.

### 3.5 햅틱·LED (입력=pygame, 출력 전부=pydualsense — BT 동시 사용 실증 완료)

- 실증(2026-07-17, BT): rumble·라이트바·플레이어 LED·트리거 Rigid/Pulse 전부 동작,
  pygame 20 Hz 입력과 pydualsense 출력 동시 사용 12 s 무충돌.
- **Tier 1 (rumble, A2 포함)**: ①chord 진행(점증→완료 스냅→ACK 더블/거부 트리플)
  ②US-100 근접 빌드업(warn 400 mm부터 거리 반비례; 데이터 = ops-state push)
  ③권한 전이(인수 확인 펄스·STOPPING 틱·hold/E-stop 진입 패턴 구분)
  ④링크 상실 하트비트 ⑤assist bypass 상시 미세 진동.
- **Tier 2 (feature-flag, 실패해도 배치 통과 무관)**: 어댑티브 트리거 —
  **무권한 잠김**(미arm·데드맨 미유지·hold·E-stop 시 풀 강성)과 **슬립 플러터**
  (slip 후보 시 고속 미세떨림 — C배치의 지령속 배선 후 유효). 라이트바 상태색
  (AUTONOMY 파랑/TELEOP 흰색/hold 노랑/E-stop 빨강).
- 클라이언트 출력은 pydualsense로 단일화(pygame rumble과의 출력 리포트 경합 회피).
  pydualsense 미설치·미지원 환경에서는 자동 비활성(입력 경로 무영향).

## 4. A3 — 실행 배선

1. **WP8 힌트→집행 어댑터**: `chassis_node`에 `section_enforcement` 파라미터
   (기본 off, `authority_enabled` 필요). `/section/state` 구독 → `speed_hint`를
   최종 선택 명령 v 클램프, `hold_hint`를 0-클램프로 적용(모두 journal).
   `work_request`·마커 진행은 **자동 집행 없이 콘솔 표시만**(팔 크로스팀 계약 대기).
2. **마커 ledger 강화**: `MarkerDedup`에 후보→확정 2단계(재관측 N=2 tracklet,
   `min_reobserve_s` 유지) 추가. `unique_markers`는 확정만 카운트.
3. **teleop 이벤트 큐 bound**: `teleop_command_node._events`(무한 SimpleQueue)에
   상한(기본 1024) — 초과 시 최고(古) drop + 카운터 journal.
4. **복귀 dwell 강화**: `recovery_ticks`(3틱)에 최소 경과 시간(기본 0.15 s)·fresh
   샘플 수 조건 결합(`AutonomyControllerConfig` 확장) — 부하 시 틱 주기 변동에도
   dwell 의미 보존.

## 5. B — 시뮬 포트폴리오 (트랙 영구 부재 → 시뮬이 실기 검증 대체)

### B1 (S급)

1. **핀치포인트 폭 협착**: 트랙 폭 국소 축소 가족 — 통과/hold 판단 검증.
2. **곡률 연속 변화**: 상수 `curvature_profile`(`procedural.py:286-302`)을
   클로소이드/구간 변화 가족으로 확장.
3. **기복(undulating) 지형**: `TERRAIN_FAMILIES`에 추가.
4. **fixture_class 계약화**: 라벨을 실행 가능한 검증 계약으로 전환.

### B2 (M/L급)

5. **물리 마찰 패치**: 측정-only `wheel_slip`과 구분되는 실제 μ 패치 지형 +
   `closed_loop.py:183`의 `diagnostics=None` 하드코딩 해제(DriveDiagnostics 배선)
   — C의 시뮬 검증 기반.
6. **스모그 depth 열화 램프**: 새 fault 그룹 `depth_degradation`
   (스키마 `scenario.py:462-524` + 파서 + 러너).
7. **동적 선도 표적**: `run_scenario`에 `detections_source` 훅 신설(현재 perception
   주입 훅 없음 — `command_source`/`hold_state_source`/`depth_tap`뿐) +
   `follow.py` 코어 폐루프 드라이버. **5구간 60점(간격 35+재추종 25)의 유일한
   검증 경로.**
8. **hidden-seed 캠페인 러너**: 가족×시드 매트릭스 일괄 실행·리포트.

## 6. C — WP9 환경 degradation 정책 (마스터플랜 §6 :848-857 구현)

1. **지령 휠속 배선**: `odometry_node`에 지령 휠속이 없어 slip 감지가 무력화된
   상태("cannot invent a slip warning") 해소 — 지령속 피드 추가로
   `slip_candidate`/`stuck_candidate` 실질화.
2. **단계 정책**: slip/stuck/depth 열화 → 감속 → CONTROLLED_HOLD → **bounded**
   auto-recovery(계획이 직접 규정; §11의 "무제한 재시도 금지"와 구분) → 실패 시
   원격 핸드오버 대기. 컨트롤러의 기존 seam(`core.py` `decide(..., diagnostics)`,
   소비 코드 :354/:431 실존) 활성화.
3. **스모그 저속 통과**: depth 품질 열화 시 SMOG 힌트와 결합한 저속 정책.
4. **문서**: 핸드오프 WP표에 WP9 행 추가(현재 누락 확인) + 프로그램 전체 반영,
   마스터플랜·Notion 동기(3문서 acceptance 동기 규칙).

## 7. 검증 전략

- 배치마다: Codex 위임 → 3환경 회귀(호스트 240 / dev 979+2skip / ros 410 기준선
  + 신규) → 커밋 체인. 테스트 배치는 pure-core(`motor_control/*/tests/`,
  `powertrain_sim/tests/`) + 노드(`ros2/src/powertrain_ros/test/`) 관례를 따른다.
- 시뮬 배치는 dev-seed 앵커(0.805, recovery 0.04 s) 재확인.
- **A배치는 Jetson 벤치 스모크 필수**: broker·콘솔 패널·systemd 유닛·부팅 자격화 +
  햅틱/트리거 체감 + chord 실감 — 배치 말 사용자 물리 확인 세션 1회.
  캘리 영속화·캘리 서비스는 모터 회전 수반(FULL HIL 규율, 사전 물리 확인).
- §11 금지 목록 15항 전수 대조 완료 — 본 프로그램 저촉 없음.

## 8. 하지 않는 것 (본 프로그램 범위 밖)

- min_rev 플로어 수치 하향, `stop_mm` 구간별 값(지상 커미셔닝 몫).
- gateway/authority hold의 자동복구 의미론 변경(§2.3 폐기 근거).
- `work_request` 자동 집행, 팔 협업 계약, 팀원 gateway WIP(l515_dashboard) 접촉.
- 크랩 조향, 서멀 쿨다운, 자동 타임박싱 강등(안건 보존만).
- pygame rumble 경로 사용(출력은 pydualsense 단일화).

## 9. 리스크

- pydualsense BT 안정성(장시간): A2 벤치 스모크에 20분 연속 출력 확인 포함.
- broker의 rclpy 서비스 콜 데드락: 타임아웃+전용 callback group으로 격리, 노드
  테스트에 지연 서비스 fixture 포함.
- 콘솔 헌장 개정의 안전 퇴행: 게이트(인라인 2단 확인) + ops 채널 단일 송신 계약
  테스트로 봉인. E-stop **발동**은 콘솔에 두지 않는다(발동은 조종기 ○·US-100·
  워치독 몫, 콘솔은 해제·전환·미션만).
- NVM 영속화 후 stale 캘리 런어웨이: 부팅 자격화 실패 시 arm 거부 + 조립 전
  3회 전원사이클 게이트.
