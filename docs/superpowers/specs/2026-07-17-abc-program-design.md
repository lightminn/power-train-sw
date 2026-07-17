# A/B/C 개선 프로그램 통합 설계 (2026-07-17, r2)

아이디어 세션(Codex + agy×2 + Claude 서브에이전트×3, 6소스)의 수렴 결과를
계획 정본(§10 중단 기준·§11 금지 목록)·트랙 정책·크로스팀 경계로 필터링해 확정한
통합 설계다. 실행 구조는 **통합 스펙 1건 + 배치 6개 순차**(A1→A2→A3→B1→B2→C)이며,
배치마다 Codex 위임 → 3환경 검증 → 커밋 체인을 따른다.

r2 개정: 초판(fdbed5a)을 6개 검증원(Codex 심층 25건, 레드팀 프리모템, 코드 실현성,
계획 정합, agy Pro/Flash 세컨드 오피니언)에 돌린 결과를 전량 반영했다.
사용자 결정 2건 포함: **D1 = E-stop reset 비상 chord 허용, D2 = 후진 한정
extraction 상태 신설.**

정본 관계: 이 문서는 `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`
(마스터플랜)의 하위 실행 설계다. 본 프로그램이 **명시적으로 개정하는 정본 항목**은
다음 넷이며, 해당 개정 커밋 후 본 문서가 근거가 된다:

1. 콘솔 read-only 헌장 (관측성 계획 :329-334 + 마스터플랜의 복구 표면·service
   ownership·DoD 관련 서술 + 코드 독스트링·배너·README).
2. §10 완료 정의 4항의 TRACKING류 구간 HIL → 시뮬레이터 영구 대체
   (근거: 핸드오프 보고서 :167-168 "실전/모의 트랙은 앞으로도 없음 — TRACKING류
   검증은 시뮬레이터가 영구 정본", 2026-07-17 사용자 확정).
3. WP5.1 US-100 의미론에 후진 한정 extraction 상태 추가 (§6.3, 벤치 HIL 선행).
4. "캘리 RAM-only, 전원 사이클마다 재캘리" 서술 → NVM 영속화 (§3.4).

## 0. 운용 원칙 (사용자 지시, 2026-07-17)

1. **실전(대회 당일)의 모든 사용자 조작은 두 표면으로만 한다:
   ①operator_console GUI ②DualSense.** SSH·추가 창·별도 툴은 실전 조작 경로에서
   제외하고 정비·비상 폴백으로만 남긴다.
2. 콘솔 안에서도 팝업/별도 창을 띄우지 않는다 — 기존 창 안 **인라인 2단 확인**만 쓴다.
3. **모든 키매핑·chord는 임시(versioned initial candidate)다.** HIL·운전자 피드백 후
   변경을 전제하며, 코드(단일 mapping 모듈)·스펙·문서 세 곳에 임시임을 명시한다.
4. latched E-stop의 reset→arm은 **평시 콘솔 패널 전용**이다(원인을 보며 판단 후
   해제). 단 콘솔 사망 대비 **의도적으로 어려운 비상 chord**(§3.2)를 조종기에
   허용한다(D1) — "의도적 수동 reset" 철학은 chord 난이도·햅틱 확인으로 유지.

## 1. 배치 개요

| 배치 | 내용 | 규모 |
|---|---|---|
| A1 | ○ E-stop 전역 latch 정합 + min_rev 플로어 발동 텔레메트리 | S |
| A2 | 현장 복구·운용 표면 (ops broker + chord + 콘솔 패널 + 브링업 무SSH + 햅틱) | L |
| A3 | 실행 배선 (WP8 힌트 집행[플로어 정합 포함] + 마커 ledger + 큐/dwell 강화) | M |
| B1 | 시뮬 S급 (핀치포인트·클로소이드 곡률·기복 지형·fixture 계약화) | S~M |
| B2 | 시뮬 M/L급 (마찰 fixture 계약화·스모그 열화·동적 선도 표적·hidden 캠페인) | L |
| C | WP9 degradation FSM + 지령속 계약 + extraction 상태 + 문서 정합 | M~L |

## 2. A1 — 의미론 정합

### 2.1 ○ E-stop 전역 latch 정합

- 현재: ○(`estop_edge`)는 gateway MOTION_HOLD의 한 원인일 뿐이고 `~/clear_hold`로
  풀린다(`remote_input_gateway.py:347-348`). 계획 정본은 "○ = 전역 수동 latched
  E-stop"을 규정한다(마스터플랜 §5 :508, 고정 안전 계약 :498).
- 변경: `teleop_command_node`가 `estop_edge` 수신 시 `/teleop/estop`
  (String JSON `{event_id, stamp_s}`)을 **1초간 매 틱 재발행**(단발 Bool edge는
  전달 미보장)하고, `chassis_node`가 구독해 `event_id`로 멱등 dedup 후
  **`cm.estop("remote_operator", ...)`** 호출 — raw `SafetyInterlock.trip_estop`이
  아니라 코너 6개 물리 정지까지 포함한 기존 진입점(`chassis_manager.py:271-283`)을
  쓴다. gateway의 기존 hold 진입은 유지한다(이중 방어).
- 해제 경로는 기존 `~/reset_estop`(활성 E-stop 원인 잔존 시 거부) → `~/arm` 분리
  흐름 그대로이며, 표면은 콘솔 패널(평시) + 비상 chord(§3.2). E-stop 구독자
  (chassis_node) 재시작 시 재발행 창과의 상호작용을 수용 시험에 포함.
- §11 준수: 통신 단절은 E-stop으로 승격하지 않는다 — 명시적 ○ edge만 trip.

### 2.2 min_rev 플로어 발동 텔레메트리 (A1 몫)

- 현재: `min_drive_turns_per_s` 플로어(기본 1.0 turns/s = 0.628 m/s,
  `chassis_manager.py:404-410`, 휠반경 0.10 m)가 어떤 속도 캡과도 대조되지 않는다.
  CARRYING_LOCKED 캡 0.5 m/s(`powertrain_autonomy/controller/profiles.py:39-51`),
  smog 힌트 0.25/ice 0.15(`section_profiles.py:370-371`)와 결정론적으로 충돌한다.
- A1 변경: `ChassisManager`에 플로어 발동 시 바퀴별 카운트를 `WheelSnapshot` 확장
  으로 노출하고(`snapshot()` 경유), chassis_node가 journal에 기록. **캡과의 정합
  검사·집행 규칙은 A3(§4.1)에서 `/section/state` 구독과 함께 배선**한다 — 현재
  chassis_node에는 프로파일 캡 데이터 경로 자체가 없어 A1 단독으로는 불가.
- **하지 않음**: 플로어 수치 하향(지상 재자격화 필요 — 실차 커미셔닝 몫).
  extraction 상태(§6.3)만 플로어 적용 예외.

### 2.3 폐기된 후보 (근거 기록)

- ~~gateway stale-hold 자동복구~~, ~~핸드오버 타임아웃 시 이전 소유자 복귀~~:
  WP5.2 계획이 타임아웃→MOTION_HOLD 전이와 "MOTION_HOLD 이후 전환은 **운영자 확인
  뒤에만** 수락"을 명시(wp5.2 plan :708-711)하고, "이전 명령·모드 자동 복원 금지"
  조항이 양 계획에 다수(≥12곳) 있다. 두 후보 모두 위반이므로 폐기한다. 프리모템
  시나리오 1·4의 실제 문제는 latch가 아니라 "SSH 없이는 못 푸는 것"이며, §3의
  chord(=운영자 확인, 2초)가 계획 철학 그대로 해결한다.
- `clear_transient_hold`의 정밀 의미(Codex 13 반영): clear 가능 cause whitelist를
  정의하고(E-stop 원인 hold는 제외), gateway `~/clear_hold`·authority
  `~/authority_clear_hold` 두 하위 동작 각각의 멱등 postcondition과 **부분 성공**
  (한쪽만 성공) 시 ACK에 하위 결과를 분리 보고, 이후 fresh neutral 세션 요구를
  명문화한다.

## 3. A2 — 현장 복구·운용 표면

### 3.1 Ops broker 노드 (신규, powertrain_ros)

- TCP **:9001** (레포 전체 미사용 확인). 주행 채널(:9000, 프레임 v2)은 **불변**.
- **인증(신규)**: 장치별 프로비저닝 토큰 — 로봇 `/etc/powertrain/ops_token`(부팅
  불변, 레포 미커밋)과 운용 노트북 설정에 동일 값. 핸드셰이크
  `{client_type, token}` 불일치·부재 시 즉시 거부. 무인증 접속·역할 위조·replay·
  rate-limit 초과 거부를 **프로토콜 수용 기준**으로 명시(Codex 4). 폐쇄 AP(WPA2)
  전제의 경량 방어이며 mTLS는 §8 범위 밖.
- **프로토콜(정밀 정의, Codex 5)**: newline-JSON. 요청 =
  `{schema_version, client_type, token, request_id(불투명), sequence(세션별 단조),
  action, params, expected_state_revision, stamp_s}`; 응답 =
  `{request_id, accepted, final(bool), state_revision, detail}`.
  broker는 `broker_boot_id`를 핸드셰이크에 노출, **bounded pending/final 캐시**로
  재전송 중복을 흡수(멱등), mutation은 **단일 큐 직렬화**. 서비스 콜 1 s 타임아웃
  초과 시 `accepted=false, final=false`로 응답하되 **late-completion 정책**:
  타임아웃 뒤 실제 완료가 도착하면 final ACK를 추가 push하고 journal에 기록.
  **클라이언트는 동일 request_id를 250 ms 간격으로 ACK 또는 2 s까지 재전송**
  (레드팀 3b — 멱등은 재전송이 있어야 의미가 있다).
- **유형별 인가**: controller = `clear_transient_hold`·`authority_manual/auto` +
  **비상 chord 전용** `estop_reset`·`arm`(§3.2, D1). console = 전체 화이트리스트:
  - `clear_transient_hold` — gateway+authority hold 동시 해제(§2.3 의미론)
  - `authority_manual` / `authority_auto` / `authority_idle` — **gateway FSM과
    통합된 전이표**(Codex 6): gateway가 ARM/STOPPING 상태거나 fresh DRIVE ACK·
    wheel-stop 조건 미충족이면 broker가 거부. 전이표는 구현 계획에서 확정.
  - `estop_reset`, `arm`, `disarm`, `arm_lock_override`(console은 평시 경로)
  - `mission_arrive_pickup/arrive_drop/skip/retry/regrasp_confirmed/clear_grip_lost`
  - `operator_hold` / `operator_resume` — `/section_events` JSON 발행. ⚠️ 이 fake
    브릿지 토픽을 production 운용 API로 승격하지 않기 위해, broker → supervisor
    전용 입력은 구현 계획에서 서비스화 여부를 결정(Codex 16).
  - `calibration_*` — 1 s RPC가 아닌 **lifecycle job**(§3.4): `calibration_start /
    status / cancel`. start는 IDLE·disarm + **콘솔 확인 스트립이 발급한 broker
    1회용 wheels-up 토큰** 동반 시에만 수락(UI 확인의 서버측 이중화, 레드팀 4d).
- **action 계약표**: action별 대상 서비스 FQN·타입(Trigger/SetBool)·params 스키마·
  필요 launch 게이트·pre/postcondition을 스펙 부속표로 구현 계획에 포함(Codex 16).
- **ops-state push (versioned)**: 연결 클라이언트에 5 Hz. 각 필드에 **권위 소스·
  source stamp·local age**를 부여하고 전체에 **단조 revision** —
  `{revision, authority_mode, gateway_state, safety_status, safety_distance_mm,
  estop_latched, active_estop_sources, wheel_fault_count, boot_qualification, ...}`.
  명령 요청은 `expected_state_revision`을 실어 2단 확인 시점의 상태와 실행 시점
  상태의 불일치(TOCTOU)를 broker가 거부로 방어(Codex 14/15).
- 구현: rclpy **`call_async` + future 타임아웃**(Humble `Client.call()`은 무한
  블로킹). TCP 스레드에서 콜하고 노드 spin은 별도 — ops-state push 타이머와 TCP
  I/O는 서비스 콜과 **분리된 callback group**으로, 콜 지연이 push를 절대 못 막게
  한다(레드팀 2b). broker는 **전용 compose 서비스**로 `restart: unless-stopped` +
  :9001 TCP-connect healthcheck(`on-failure:N` 금지, bash 래퍼 cmdline-grep
  healthcheck 함정 금지 — compose 기존 사례 참조).

### 3.2 DualSense chord (mapping `recovery-v1-initial-candidate`, ⚠️ 전부 임시)

- **□+CREATE 2 s hold** = `clear_transient_hold`. (기존 모드 chord CREATE+OPTIONS
  1 s와 조합 상이. □=button 3, CREATE=button 8.)
- **D-pad ↓ 1 s** = `authority_manual`(TELEOP 인수), **D-pad ↑ 1 s** =
  `authority_auto`(AUTONOMY 복귀). ⚠️ dpad.y는 동작 미할당이나 **gateway neutral
  판정에는 포함**(`remote_input_gateway.py:182`) — 접속/재접속/clear 직후 DRIVE
  ACK는 chord를 뗀 뒤에만 성립한다. "chord 유지 중 연결·재연결" 시나리오를 수용
  시험에 포함(계획정합 1·Codex 25).
- **비상 chord (D1, 콘솔 사망 대비)**: **L1+R1+□ 5 s hold** = `estop_reset`,
  이어서 **L1+R1+△ 3 s hold** = `arm`. 햅틱 점증→완료 스냅→ACK 더블 펄스 확인
  필수. 평시엔 콘솔 경로를 쓰고 비상 chord는 훈련·문서에 비상용으로 명시.
- chord 감지는 클라이언트 측, 부분 입력(조합 일부 해제)은 즉시 무효·카운트
  리셋. 요청은 ops 채널로만 전송하고 주행 프레임 형식은 불변.

### 3.3 콘솔 운용 패널 (operator_console 헌장 개정)

- 기존 창 안 패널 추가. 모든 명령 버튼은 **인라인 2단 확인**(1단: 현재 원인·상태
  표시; 2단: 확인 스트립)을 거치고, 확인 스트립은 실행 직전 **ops-state revision
  재검증**(불일치 시 1단으로 복귀 — TOCTOU 방어). `estop_reset`과 `arm`은 **서로
  다른 확인 제스처**(reset=확인 스트립, arm=1.5 s hold-to-confirm)와 비명령
  스페이서로 공간 분리(레드팀 5a). 두 동작은 각각 독립 버튼·독립 확인이다
  (reset→IDLE, no implicit arm 유지).
- 범위: `clear_transient_hold`, 권한 3종, `estop_reset`·`arm`·`disarm`,
  `arm_lock_override`(더 강한 확인 문구), 미션 6종, OPERATOR_HOLD/RESUME,
  브링업(§3.4) 상태·트리거.
- **GTK 동시성 설계**(Codex 15): ops 클라이언트는 전용 스레드 — nonblocking
  reconnect 워커, bounded send queue, request_id↔ACK correlation, UI 반영은
  `GLib.idle_add`, 종료 시 join. GStreamer 백프레셔가 명령 경로를 얼릴 수 있는
  구조적 한계는 D1 비상 chord가 최종 방어선.
- **헌장 개정 범위**: 관측성 계획 :329-334 **및 Task 6 acceptance**, 마스터플랜의
  복구 표면·service ownership·DoD 관련 서술, `operator_console/` 독스트링·배너
  (`app.py:627`)·README·`telemetry.py`, 프로젝트 CLAUDE.md(Codex 21).
- 계획이 주장하나 실존하지 않는 "no-send 계약 테스트"를 실물로 대체: **"ops
  채널 클라이언트 외 어떤 제어 송신 경로도 없음"** 계약 테스트 추가(텔레메트리·
  SRT 수신 경로 불변).

### 3.4 당일 브링업 무SSH화

- **스택 감독 구조**(Codex 7): systemd→compose→`docker exec` 3단은 실제 제어
  프로세스를 감독하지 못한다. `wp5_control.launch.py`(+teleop, broker)를 **compose
  서비스의 PID 1**로 실행하는 launch 서비스로 재구성하고, systemd 유닛은 compose
  스택 기동만 담당: `Requires/After=docker.service`, `ExecStartPre` ①
  `scripts/can_setup.sh`(호스트 root) ② **env 검증** — `/etc/powertrain/
  powertrain.env` 존재 + `stop_mm` 수치 sane-band + `STOP_MM_PROVENANCE=BENCH|
  COMMISSIONED` 필수(BENCH이면 콘솔 브링업 패널에 경고 배지). `stop_mm` 자체는
  launch 필수 인자 정책 유지. DDS 도메인·RMW·`authority_enabled=true`·
  `use_sim_time=false`를 유닛/compose에 고정. **커미셔닝 전에는 유닛을 disabled로
  설치**(벤치에서만 수동 start — provisional stop_mm이 사실상의 기본값이 되는 것
  방지, 계획정합 5).
- **crash-loop 비콘(신규 S)**: 메인 스택과 독립된 초경량 systemd 서비스가 유닛
  상태·journal 꼬리를 UDP로 콘솔에 push — 스택이 못 뜨는 이유를 SSH 없이 콘솔
  에서 판독(agy×2 블랙아웃 지적). 부팅 후 모터 열거 지연 대비 preflight 재시도
  창(기본 30 s)을 둔다.
- **캘리브레이션 NVM 영속화(재설계, Codex 8·실현성 10)**: `bl70200_setup.py`는
  단일 보드·axis1 전용이라 그대로는 불가. 신규 `bl70200_persist_calibration.py`:
  ①**보드 시리얼→CAN 노드쌍(11/12·13/14·15/16) 레지스트리**(설정 파일) ②보드별
  `find_any(serial_number=...)` 접속 ③axis0+axis1 **양축** 캘리 성공 확인 후
  보드당 1회 `motor/encoder.config.pre_calibrated=True`+`save_configuration`
  (리부팅 수반) ④재열거→`--read` 전수 대조 — 를 보드 단위 트랜잭션으로. NVM
  쓰기는 정본 CFG(bl70200_setup.py의 값) 준수.
  ⚠️ **보드 fw = 0.5.1**(사용자 확인; 0.5.6은 파이썬 라이브러리) — HALL polarity
  캘리 상태는 0.5.2+ 전용이므로 근거로 쓰지 않는다. 영속화는 0.5.1 시절 공식
  hoverboard(HALL) 가이드 경로이며, **실증 정본은 벤치 게이트**다.
- **부팅 자격화 = chassis 소유 게이트**(Codex 9): 무회전 판독(축별
  `pre_calibrated`·`motor.is_calibrated`·`encoder.is_ready`·에러 플래그) + **보드
  지문 대조**(시리얼 + 설정 CRC — 보드/모터 교체 맹검 방어, 레드팀 4c)를
  ChassisManager 권위 게이트로 두고, **broker·`~/arm`·chord 등 모든 arm 경로가
  이 게이트를 통과**해야 한다(우회 불가). 실패 축 존재 시 arm 거부 + 콘솔 표시.
  전압 판독 포함(프리모템 시나리오 5).
- **캘리 폴백 = lifecycle job**: `calibration_start/status/cancel`(축당 ~55 s,
  exclusive — 실행 중 arm 요청 거부). 콘솔 확인("바퀴 전부 리프트") + broker
  wheels-up 토큰 이중 게이트. 조립 후에는 사실상 정비 절차임을 문서화.
- **조립 전 벤치 게이트(신설, 핸드오프 기록)**: wheels-up에서 캘리→영속→
  **전원 사이클 3회 × 6축 직진입 closed-loop 재현** 통과가 차체 조립 선행조건.
  실패 시 fw 업그레이드 여부를 사용자 결정 안건으로 회부.
- 완료 후 CLAUDE.md·Notion의 "캘리 RAM-only" 서술 갱신.

### 3.5 햅틱·LED (입력=pygame, 출력 전부=pydualsense — BT 동시 사용 실증 완료)

- 실증(2026-07-17, BT): rumble·라이트바·플레이어 LED·트리거 Rigid/Pulse 전부 동작,
  pygame 20 Hz 입력과 pydualsense 출력 동시 사용 12 s 무충돌. pydualsense는 레포
  신규 의존성(노트북 환경 전용) — 배치에서 명시 추가.
- **햅틱 arbiter(신규)**: 우선순위 단일 중재기 — E-stop > 권한 전이 > 링크 상실 >
  US-100 근접 > bypass 상시 — 로 **한 번에 한 패턴만** 재생(마스킹 방지, agy×2).
  연속 진동 duty 상한(배터리) + 조종기 충전을 구간별 체크리스트에 명시.
- **Tier 1 (rumble)**: ①chord 진행(점증→완료 스냅→ACK 더블/거부 트리플)
  ②US-100 근접 빌드업(warn 400 mm부터; 데이터 = ops-state push) ③권한 전이
  ④링크 상실 하트비트 ⑤assist bypass 상시 미세 진동.
- **stale 방어(레드팀 3c)**: ops-state age > 0.5 s면 **링크 상실 패턴 강제 +
  라이트바 중립** — stale push로부터 긍정 상태를 절대 표시하지 않는다.
- **장애 격리(레드팀 3d)**: pydualsense 출력은 격리 스레드 + 예외 시 출력만 자동
  비활성 — pygame 입력 루프·프로세스로 전파 금지. BT 단절 시 입력 경로는 gateway
  stale→hold(fail-safe)로 이미 방어됨; **예비 조종기 사전 페어링 + 클라이언트
  joystick 재열거 지원**을 운용 절차에 추가(핫스왑, agy Pro).
- **Tier 2 (feature-flag, 실패해도 배치 통과 무관)**: 어댑티브 트리거 —
  **무권한 잠김**(강한 저항 + 펄스 프로파일, 풀 강성 잠금은 패닉·파손 우려로
  배제 — agy Pro)과 **슬립 플러터**(C의 지령속 배선 후 유효 — **체감 검증은 C
  배치로 이월**). 라이트바 상태색(AUTONOMY 파랑/TELEOP 흰색/hold 노랑/E-stop 빨강).

## 4. A3 — 실행 배선

1. **WP8 힌트→집행 어댑터**: `/section/state`를 **versioned SectionState**로 강화
   (session/sequence/source stamp/TTL — Codex 10) 후 chassis_node가 구독
   (`section_enforcement` 파라미터, 기본 off, `authority_enabled` 필요).
   집행 규칙: `speed_hint` → 최종 선택 명령 v 클램프; `hold_hint` → **v·ω 모두 0**;
   enforcement 활성 중 stale·미래·sequence 역행 힌트 → hold(fail-close).
   **플로어 정합(Codex 1)**: `speed_hint < 2πr×min_rev`(플로어 환산속도)이면
   플로어로 끌어올리지 않고 **0/hold로 fail-close** + WARN journal —
   `CornerModule.set()` 입력 기준 캡 준수 테스트 포함. `work_request`·마커 진행은
   자동 집행 없이 콘솔 표시만(팔 크로스팀 계약 대기).
2. **마커 ledger 강화**: `MarkerDedup` 후보→확정 2단계(재관측 N=2 tracklet) +
   **후보 TTL·최대 후보 수·공간/ID 연속성·stamp 역행 거부**(Codex 18).
   `unique_markers`는 확정만 카운트. positive `class_id`=instance-ID 협업 계약이
   확정되기 전에는 위치 클러스터 경로 유지.
3. **teleop 이벤트 큐 재설계(Codex 3)**: 모션 프레임은 **latest-only 병합**,
   E-stop·connect/disconnect·contract violation·server_error는 **비유실 우선
   큐**로 분리(단순 drop-oldest 금지 — E-stop 이벤트 유실 위험). 기존
   suppressed-violation 카운터와 합성.
4. **복귀 dwell 강화**: `recovery_ticks`(3틱)에 최소 경과 시간(기본 0.15 s)·fresh
   샘플 수 조건 결합(`AutonomyControllerConfig` 확장).

## 5. B — 시뮬 포트폴리오 (트랙 영구 부재 → §10-4 TRACKING류 검증의 명시 개정)

### B1 (S급)

1. **핀치포인트 폭 협착**: 트랙 폭 국소 축소 가족(per-station 폭은 이미 존재 —
   `procedural.py:318` → `model_builder.py:120-122`). 수용 기준(폭 여유 마진별
   통과/hold 판정)은 구현 계획에서 수치 고정.
2. **곡률 연속 변화**: 상수 `curvature_profile`(`procedural.py:286-302`)을
   클로소이드/구간 변화 가족으로 확장. 곡률 연속성 상한 명시.
3. **기복(undulating) 지형**: heightfield가 아니라 **per-station `elevation_m`
   변주**로 구현(`_segment_axes`가 3-D 지원 — 실현성 검증 확인).
4. **fixture_class 계약화**: 라벨을 실행 가능한 검증 계약으로 전환.

### B2 (M/L급)

5. **마찰 fixture 계약화(재정의, Codex 19)**: per-station `friction_coefficient`는
   **이미 물리 μ로 배선돼 있다**(`model_builder.py:123-126`) — 신규 스키마가 아니라
   기존 경로를 저마찰 patch fixture로 계약화·시나리오 가족화한다. 동적(시간 변화)
   패치가 필요해질 때만 별도 스키마 추가. + `closed_loop.py:183`의
   `diagnostics=None` 하드코딩 해제(DriveDiagnostics 배선 — 시뮬에 지령 휠속·
   슬립 재료 실존 확인) — C의 시뮬 검증 기반.
6. **스모그 depth 열화 램프**: 새 fault 그룹 `depth_degradation`
   (스키마 `scenario.py:462-524` + 파서 + 러너).
7. **동적 선도 표적(산출물 전체 명시, Codex 20)**: `run_scenario`에
   `detections_source` 훅 신설 + follow.py 코어 폐루프 드라이버 + **표적 plant
   궤적 모델·상대 pose/bbox 합성·가림/dropout 주입·틱 순서 정의·간격 유지/재추종
   점수화·recording 스키마**까지가 한 세트다. 5구간 추종 배점(간격 35+재추종 25,
   완주 10 별도)의 **유일한 폐루프(plant-급) 검증 경로**(fake-target 단위시험은
   기존재).
8. **hidden-seed 캠페인 러너**: 가족×시드 매트릭스 일괄 실행·리포트.
   dev/regression/hidden 시드 분리 정책과 가족별 수치 합격 기준(핀치 여유·곡률
   연속성·기복 범위·slip 발생/진단률·depth 램프 반응·추종 점수)을 구현 계획에서
   숫자로 고정(Codex 23).

## 6. C — WP9 + 지령속 계약 + extraction (마스터플랜 §6 :848-857 구현)

1. **지령 휠속 = 계약 변경 전체**(Codex 11): post-kinematics·post-floor 지령을
   `WheelSnapshot`(chassis/telemetry.py) → `ChassisManager.snapshot()` →
   `powertrain_msgs/WheelState.msg` **additive 필드** → `message_adapter` →
   `odometry_node`(기존 `getattr` seam이 자동 수용, `odometry_node.py:141-150`)
   체인으로 공개. **msg 변경 = 양 환경(dev/ros)+Jetson 재빌드·type-hash 배포**를
   배치 범위에 명시.
2. **WP9 degradation FSM(전용 순수 모듈, Codex 12)**: 기존 diagnostics seam
   (`core.py:354/:431`)은 소비 지점일 뿐이다. slip×stuck×depth 조건별 단계·
   진입/해제 hysteresis·시도/거리/시간 budget·복구 명령·**원격 핸드오버 대기
   상태**를 가진 degradation FSM을 순수 모듈로 신설하고 출력 계약(속도 스케일·
   hold 요청·handover 요청)을 정의. bounded auto-recovery는 계획 §6 :855-856이
   직접 규정(§11 "무제한 재시도 금지"와 구분).
3. **후진 한정 extraction 상태(D2, WP5.1 의미론 개정)**: US-100 근접 latch 중
   탈출 수단 부재는 현행 결함(장애물 <stop_mm이면 reset 거부 + arm 불가 + 이동
   불가). **extraction 상태**: 콘솔 강확인으로만 진입, **후진 전용 v ≤ 0.2 m/s·
   조향 0·grant당 TTL 3 s**, 전방 US-100이 막지 않는 유일한 상태, min_rev 플로어
   적용 예외(코깅 감수). **벤치 HIL 통과가 활성화 선행조건**이며 그 전까지
   feature-flag off. 문 PUSH 구간 생존성 확보.
4. **스모그 저속 통과**: depth 품질 열화 시 SMOG 힌트와 결합한 저속 정책 —
   집행은 §4.1 규칙(힌트<플로어 → hold)과 정합.
5. **문서**: 핸드오프 WP표에 WP9 행 추가(현재 누락 확인) + 프로그램 전체 반영,
   마스터플랜·관측성·wp5.2 문서의 §0 명시 개정 4건 동기, Notion 동기.

## 7. 검증 전략

- 배치마다: Codex 위임 → 3환경 회귀(호스트 240 / dev 979+2skip / ros 410 기준선
  + 신규) → 커밋 체인. pure-core(`motor_control/*/tests/`, `powertrain_sim/tests/`)
  + 노드(`ros2/src/powertrain_ros/test/`) 관례.
- **A2 안전 프로토콜 독립 수용 기준**(Codex 22): 인증 위조/무토큰 거부·replay/
  재접속·타임아웃 후 late-completion·부분 clear·E-stop 구독자 재시작·큐 overflow·
  broker 프로세스 kill/재시작·cold boot·chord 유지 중 재연결.
- **C 검증**(Codex 24): degradation 조합(slip×stuck×depth)·hysteresis·budget
  소진·원격 핸드오버·min_rev 충돌·diagnostics 생산자 사망 — 순수 코어 + full-stack
  양쪽. 슬립 플러터 체감 확인 포함(A2에서 이월).
- 시뮬 배치: dev-seed 앵커(0.805, recovery 0.04 s) 재확인 + §5-8 수치 기준.
- **A배치 Jetson 벤치 스모크 필수**: broker·콘솔 패널·systemd 유닛·비콘·부팅
  자격화 + 햅틱/트리거 체감 + chord 실감(비상 chord 포함) + **20분 연속 출력
  soak**(pydualsense BT 안정성) — 배치 말 사용자 물리 확인 세션 1회.
  캘리 영속화·캘리 잡은 모터 회전 수반(FULL HIL 규율, 사전 물리 확인).
- §11 금지 목록 15항 전수 대조 완료(독립 검증 포함) — 본 프로그램 저촉 없음.
- **r2 재검증**: 본 개정본을 Codex 적대 재검토에 1회 회부 후 구현 계획으로 내린다.

## 8. 하지 않는 것 (본 프로그램 범위 밖)

- min_rev 플로어 수치 하향, `stop_mm` 구간별 값(지상 커미셔닝 몫).
- gateway/authority hold의 자동복구 의미론 변경(§2.3 폐기 근거).
- `work_request` 자동 집행, 팔 협업 계약, 팀원 gateway WIP(l515_dashboard) 접촉.
- 크랩 조향, 서멀 쿨다운, 자동 타임박싱 강등(안건 보존만).
- pygame rumble 경로 사용(출력은 pydualsense 단일화), mTLS/PKI(토큰으로 충분).
- US-100 tilt 보상(IMU 연동 게이팅) — WP9 후속 백로그로 기록만.

## 9. 리스크

- pydualsense BT 안정성(장시간): A2 벤치 20분 soak로 확인, Tier 2는 feature-flag.
- broker rclpy 콜 데드락: `call_async`+타임아웃+분리 callback group, 지연 서비스
  fixture 테스트.
- 콘솔 헌장 개정의 안전 퇴행: 토큰 인증 + revision 재검증 + "ops 클라이언트 외
  제어 송신 금지" 계약 테스트 + E-stop **발동**은 콘솔에 두지 않음(발동은 조종기
  ○·US-100·워치독 몫).
- NVM stale 캘리 런어웨이: 지문 대조 + 자격화 실패 시 전 경로 arm 거부 + 조립 전
  3회 전원사이클 게이트. fw 0.5.1 영속화 미지원 판명 시 fw 업그레이드 결정 회부.
- extraction 오남용: 콘솔 강확인 + TTL 3 s + 후진·저속 한정 + 벤치 HIL 선행 +
  feature-flag.
- 조종기/콘솔 명령 경합: broker 단일 mutation 직렬화 + revision 조건부 실행 +
  journal로 사후 추적(선점 lock은 §8 — 과설계 판단).
