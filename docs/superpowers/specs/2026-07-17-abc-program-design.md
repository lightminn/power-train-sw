# A/B/C 개선 프로그램 통합 설계 (2026-07-17, r5)

아이디어 세션(Codex + agy×2 + Claude 서브에이전트×3, 6소스)의 수렴 결과를
계획 정본(§10 중단 기준·§11 금지 목록)·트랙 정책·크로스팀 경계로 필터링해 확정한
통합 설계다. 배치마다 Codex 위임 → 3환경 검증 → 커밋 체인을 따른다.

개정 이력: r1(fdbed5a) → 6개 검증원(Codex 25건·레드팀·실현성·계획정합·agy Pro/
Flash) → r2(29db46e, 사용자 결정 D1 비상 chord·D2 extraction) → Codex 재검토
(해소16/부분9/신규9) → r3(a74dad9, 신규 9건 전량 반영 + 배치 재배열 + 계약표
내재화) → r4(3892820, 사용자 결정 D3: min_rev 플로어 폐지 — 기본값 전면 0,
Codex #28 저속 구간 차단 모순 소멸) → **r5(본 판, 사용자 결정 D4): 플로어 대체
메커니즘으로 저속 마찰/코깅 보상 torque_ff(최소 전류) 노브 신설.**

정본 관계: 이 문서는 `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`
(마스터플랜)의 하위 실행 설계다. 본 프로그램이 **명시적으로 개정하는 정본 항목**:

1. 콘솔 read-only 헌장 (관측성 계획 :329-334 + Task 6 acceptance + 마스터플랜의
   복구 표면·service ownership·DoD 서술 + 코드 독스트링·배너·README).
2. §10 완료 정의 4항의 TRACKING류 구간 HIL → 시뮬레이터 영구 대체
   (근거: 핸드오프 :167-168, 2026-07-17 사용자 확정).
3. WP5.1 US-100 의미론에 후진 한정 extraction 상태 추가 (§6.1, 벤치 HIL 선행).
4. "캘리 RAM-only, 전원 사이클마다 재캘리" 서술 → NVM 영속화 (§3.4).

## 0. 운용 원칙 (사용자 지시, 2026-07-17)

1. **실전(대회 당일)의 모든 사용자 조작은 두 표면으로만 한다:
   ①operator_console GUI ②DualSense.** SSH·추가 창·별도 툴은 실전 조작 경로에서
   제외하고 정비·비상 폴백으로만 남긴다.
2. 콘솔 안에서도 팝업/별도 창을 띄우지 않는다 — 기존 창 안 **인라인 2단 확인**만.
3. **모든 키매핑·chord는 임시(versioned initial candidate)다.** HIL·운전자 피드백
   후 변경 전제, 코드(단일 mapping 모듈)·스펙·문서 세 곳에 임시 명시.
4. latched E-stop의 reset→arm은 **평시 콘솔 패널 전용**. 콘솔 사망 대비
   **의도적으로 어려운 비상 chord**(§3.2)를 허용하되(D1), chord 시간·순서는
   **로봇 측(broker)이 권위 검증**한다(§3.1 2단계 프로토콜) — 클라이언트 자기
   신고만으로 reset이 실행되지 않는다.

## 1. 배치 개요 (r3 재배열 — 의존성·롤백 단위 기준)

| 배치 | 내용 | 규모 |
|---|---|---|
| A1 | ○ E-stop 전역 latch 정합 + 플로어 폐지(기본 0) + 저속 마찰보상 ff 노브 | S~M |
| A2a | ops broker(:9001) + 인증 + chord (조종기 클라이언트) | M |
| A2b | 콘솔 운용 패널 + 햅틱/LED | M |
| A2c | 브링업 무SSH (systemd·비콘·캘리 영속화·부팅 자격화) | M |
| C0 | 후진 한정 extraction 상태 (A1·A2 의존, 대회 생존성 — 앞당김) | S~M |
| A3 | 실행 배선 (WP8 힌트 집행·마커 ledger·큐/dwell) — enforcement는 §4.1 게이트 | M |
| B1 | 시뮬 S급 (핀치포인트·클로소이드·기복·fixture 계약화) | S~M |
| B2 | 시뮬 M/L급 (마찰 fixture·스모그 열화·동적 선도 표적·hidden 캠페인) | L |
| C1 | WP9 degradation FSM + 지령속 계약 + 문서 정합 | M~L |

A2a/A2b/A2c는 독립 롤백 단위(Codex 34). C0은 A2 완료 즉시 착수(벤치 HIL 선행
조건은 §6.1). A3의 section enforcement는 파라미터 기본 off로 출하하고 벤치
검증 후 활성한다(r4: 플로어 폐지로 r3의 "재자격화까지 강제 off" 게이트는 소멸).

## 2. A1 — 의미론 정합

### 2.1 ○ E-stop 전역 latch 정합

- 현재: ○(`estop_edge`)는 gateway MOTION_HOLD의 한 원인일 뿐(`remote_input_
  gateway.py:347-348`). 계획 정본은 "○ = 전역 수동 latched E-stop"(마스터플랜
  §5 :508, :498).
- 변경: `teleop_command_node`가 `estop_edge` 수신 시 `/teleop/estop`(String JSON
  `{event_id, stamp_s}`)을 **TRANSIENT_LOCAL(latched) QoS + 1초 재발행**으로
  발행 — 구독자(chassis_node)가 1초 넘게 재시작 중이어도 latched 메시지로 수신
  보장(Codex r2-2). chassis_node는 `event_id` 멱등 dedup 후
  **`cm.estop("remote_operator", ...)`**(코너 6개 물리 정지 포함,
  `chassis_manager.py:271-283`) 호출. gateway hold 진입은 유지(이중 방어).
- 해제: 기존 `~/reset_estop`(활성 원인 잔존 시 거부) → `~/arm` 분리 흐름 그대로.
  표면은 콘솔(평시) + 비상 chord(§3.2, broker 권위 검증).
- §11 준수: 통신 단절은 E-stop 승격 금지 — 명시적 ○ edge만 trip.

### 2.2 min_rev 플로어 폐지 (r4, 사용자 결정 D3)

- 배경: 플로어(1.0 turns/s = 0.628 m/s, `chassis_manager.py:404-410`)가
  CARRYING_LOCKED 0.5 m/s(`profiles.py:39-51`)·smog 0.25/ice 0.15
  (`section_profiles.py:370-371`)와 결정론적 충돌(Codex 28, 프리모템 8).
  사용자 판정: 1.0은 애초에 임의값이고 실차 조립 후 저속 특성은 달라진다 —
  **폐지한다.**
- A1 변경: **기본값을 전면 0(off)으로** — `chassis_node` param `min_rev`
  (`:134`), `teleop_server.py --min-rev`(`:279`), `teleop_dualsense.py
  --min-rev`(`:146`), `autonomy.launch.py`(`:74`) 및 문서(프로젝트 CLAUDE.md·
  핸드오프·Notion)의 1.0 기본 서술 동기 갱신. **메커니즘
  (`min_drive_turns_per_s`, 0=off)은 삭제하지 않고** 커미셔닝 때 실측으로
  재도입 가능한 opt-in 노브로 보존한다.
- 결과: SMOG/ICE/CARRYING 저속 힌트가 더 이상 차단되지 않는다(Codex 28 모순
  소멸). §4.1의 "힌트<플로어 → fail-close" 규칙은 **플로어가 재도입된 경우에만
  작동하는 휴면 정합 가드**로 유지.
- ⚠️ 잔여 위험 재배치: HALL 코깅존(<0.3 rev/s)에서 "바퀴 정지 + 텔레메트리
  정상"(2026-07-05 HIL 실측) 위험이 플로어 대신 §2.2b 마찰보상 ff + **C1 WP9
  stuck 감지**와 실차 커미셔닝 튜닝의 몫이 된다 — §9 리스크에 명시.

### 2.2b 저속 마찰/코깅 보상 torque_ff — "최소 전류" 노브 (r5, 사용자 결정 D4)

- 원리: 코깅존 문제의 뿌리는 저속 지령 시 정지마찰·코깅 디텐트를 깰 토크가
  늦게 형성되는 것(적분기 wind-up 대기). 속도 플로어는 이 영역을 회피하는
  우회였고 저속 프로파일 차단·애커만 차동 붕괴를 낳았다. **마찰 보상
  피드포워드는 지령 속도를 보존한 채 토크만 보탠다** — 표준적인 Coulomb/
  stiction 보상 기법.
- 구현 이음새(실측 확인): CAN `Set_Input_Vel(0x0D)` = `<ff`(vel, torque_ff)
  2필드 — 2026-06-24 본 보드에서 검증된 기존 명령. 프로토콜·NVM 변경 없이
  `DriveOdriveCan`이 지령마다 ff를 싣는다. ⚠️ fw 0.5.1에서 2번째 필드 단위
  (A vs N·m)는 벤치 1회 확정.
- 규칙: `friction_ff`(기본 **0=off**), `v_knee`(적용 상한, 초기 0.5 rev/s).
  `0 < |v_cmd| < v_knee`일 때만 `sign(v_cmd) × friction_ff` 인가, 지령 0에선
  정확히 0(크리프·발열 방지), 방향 전환 시 부호 즉시 추종. interlock hold/
  E-stop 중에는 지령 자체가 0이므로 자동 무효.
- 초기 후보값: Iq 0.2~0.4 A급(브링업 실측 — 마찰 돌파 0.2~0.5 A, 1바퀴 이동
  최대 0.84 A). **값 튜닝 = wheels-up 벤치**(A배치 스모크에 저속 추종
  0.15/0.25 m/s 검증 추가), 최종 자격화 = 지상 커미셔닝.
- 알려진 리스크(벤치 확인 항목): ①경부하 서징(디텐트 돌파 후 과속↔재정지
  반복 — ff·v_knee 튜닝으로 완화) ②장애물에 눌린 정지 시 지속 전류 발열
  (1차 = ODrive current_lim, 2차 = C1 stuck 감지) ③HALL 저속 피드백 지연과의
  상호작용.

### 2.3 폐기된 후보 (근거 기록)

- ~~gateway stale-hold 자동복구~~, ~~핸드오버 타임아웃 시 이전 소유자 복귀~~:
  wp5.2 :708-711("MOTION_HOLD 이후 전환은 운영자 확인 뒤에만") + "이전 명령·모드
  자동 복원 금지" 조항 다수(≥12곳) 위반이라 폐기. 프리모템 시나리오 1·4의 실제
  문제는 "SSH 없이 못 푸는 것" — §3 chord(=운영자 확인)가 계획 철학대로 해결.
- `clear_transient_hold` 정밀 의미: clear 가능 cause whitelist(E-stop 원인 hold
  제외), gateway/authority 두 하위 동작 각각의 멱등 postcondition, **부분 성공**
  시 ACK에 하위 결과 분리 보고, 이후 fresh neutral 세션 요구.

## 3. A2 — 현장 복구·운용 표면

### 3.1 Ops broker 노드 (A2a, 신규, powertrain_ros)

- TCP **:9001**. 주행 채널(:9000, 프레임 v2) 불변. **broker는 단일 소유의 전용
  compose 서비스(PID 1)로만 존재**하며 control launch에 포함하지 않는다(Codex 30
  — r2의 이중 정의 정정). `restart: unless-stopped` + :9001 TCP-connect
  healthcheck(cmdline-grep 함정 금지). `/etc/powertrain`은 read-only mount,
  `/run/powertrain`·`/var/lib/powertrain`은 기동 전 존재·권한 검증.
- **인증(역할 결박, Codex 29)**: 역할별 별도 시크릿 —
  `/etc/powertrain/ops_console.token`·`ops_controller.token`(root:service 0640
  ro-mount, 레포 미커밋). **서버가 토큰→역할을 매핑**하고 `client_type`은 표시용
  일 뿐 권한 근거가 아니다. journal·비콘에서 토큰 강제 redaction, 부팅 preflight
  에 토큰 파일 검사 포함, 교체는 파일 재생성+서비스 재시작(수동 rotation)으로
  문서화. 무토큰·불일치·rate-limit 초과·replay 거부를 수용 기준으로 명시.
  (challenge-HMAC·mTLS는 §8 — 폐쇄 WPA2 AP + 물리 보안 전제의 잔여 위험 수용.)
- **프로토콜**: newline-JSON. 요청 = `{schema_version, token, request_id(불투명),
  sequence(세션별 단조), action, params, expected_state_revision, stamp_s}`.
  응답 상태는 **4값**(Codex 31): `PENDING / FINAL_SUCCESS / FINAL_REJECTED /
  OUTCOME_UNKNOWN`. 서비스 콜 1 s 타임아웃 시 `{accepted:true, final:false,
  status:PENDING}` — "거부"가 아니다. 실제 완료 도착 시 final ACK push.
  pending/final 캐시 키 = **인증된 client_id + request_id**(bounded), 재접속
  클라이언트용 `status_query` action 제공. mutation은 단일 큐 직렬화, 클라이언트
  는 동일 request_id를 250 ms 간격 재전송(ACK 또는 2 s까지).
- **비상 action 2단계 서버 검증(Codex 26)**: controller 역할의 `estop_reset`·
  `arm`은 직접 호출 불가 — ①`emergency_begin{action}` 수신 시 broker가 타이머
  시작·햅틱 진행 push ②**서버 측 경과 시간**(reset 5 s / arm 3 s) 충족 후 동일
  request_id의 `emergency_execute`만 수락. arm은 추가로 **reset 완료 후 전 버튼
  release + fresh neutral + wheel-stop 확인**(gateway/authority 상태로 검증)을
  선행조건으로 한다. 클라이언트 chord 감지는 UX일 뿐 권위가 아니다.
- **유형별 인가**: controller = `clear_transient_hold`·`authority_manual/auto`·
  비상 2단계(`estop_reset`→`arm`). console = 전체(아래 계약표).
- **action 계약표** (권위 정의 — 구현 계획은 이 표를 그대로 구현):

  | action | 대상 | 타입 | 선행조건 | 사후조건 |
  |---|---|---|---|---|
  | clear_transient_hold | teleop `~/clear_hold` + chassis `~/authority_clear_hold` | Trigger×2 | 각 hold가 clear-가능 cause | 부분 성공 분리 보고, fresh neutral 요구 |
  | authority_manual/auto/idle | chassis `~/authority_*` | Trigger | 전이표(하단) | authority.mode 전이 |
  | estop_reset | chassis `~/reset_estop` | Trigger | 활성 원인 없음(§6.1 예외) · console 또는 비상 2단계 | mode=IDLE, no implicit arm |
  | arm / disarm | chassis `~/arm`/`~/disarm` | Trigger | 부팅 자격화 게이트(§3.4) 통과 · arm은 reset과 별개 확인 | mode=ARMED/IDLE |
  | arm_lock_override | chassis `~/arm_lock_override` | SetBool | console 전용 · 강확인 | override 플래그 |
  | mission_* (6종) | chassis `~/mission_*` | Trigger | console 전용 · mission_supervisor_enabled | 미션 상태 전이 |
  | operator_hold/resume | `/section_events` 발행(잠정) | String pub | console 전용 | supervisor notice. ⚠️ production 승격 여부는 A3에서 서비스화로 결정 |
  | calibration_start/status/cancel | 신규 lifecycle job | 신규 srv | §3.4 (wheels-up 토큰 + IDLE·disarm + exclusive) | 진행/완료/중단 상태 |
  | extraction_grant | 신규 (C0) | 신규 srv | §6.1 상태표 | grant TTL 시작 |
  | status_query | broker 내부 | — | 인증 | pending/final 캐시 조회 |

- **authority 전이표**(Codex 6 — gateway FSM 통합):

  | 요청 | 허용 | 거부 |
  |---|---|---|
  | authority_manual | authority ∈ {IDLE, AUTONOMY} ∧ gateway=DRIVE ∧ input fresh ∧ E-stop 미latch | gateway ∈ {ARM, STOPPING_*} · MOTION_HOLD(clear 선행) · stale |
  | authority_auto | authority ∈ {IDLE, TELEOP} ∧ autonomy 소스 fresh ∧ E-stop 미latch | 동일 + autonomy stale |
  | authority_idle | MOTION_HOLD 외 전 상태 | MOTION_HOLD(clear 선행) |

- **ops-state push (versioned)**: 5 Hz. 필드별 권위 소스·source stamp·local age +
  전체 단조 **revision — 의미 있는 상태 전이에서만 증가(age·주기 push 제외,
  Codex 32)**. `expected_state_revision` 불일치 시 거부하되, broker는 실행 시점에
  안전 predicate(E-stop·자격화·전이표)를 **직접 재평가**한다 — revision은 UI
  TOCTOU 방어, predicate가 최종 권위.
- 구현: rclpy **`call_async`+future 타임아웃**(Humble `call()`은 무한 블로킹),
  ops-state push 타이머·TCP I/O는 서비스 콜과 분리된 callback group — 콜 지연이
  push를 못 막게(레드팀 2b).

### 3.2 DualSense chord (mapping `recovery-v1-initial-candidate`, ⚠️ 전부 임시)

- **□+CREATE 2 s** = `clear_transient_hold`. (□=3, CREATE=8; 기존 모드 chord
  CREATE+OPTIONS 1 s와 상이.)
- **D-pad ↓/↑ 1 s** = `authority_manual`/`authority_auto`. ⚠️ dpad.y는 동작
  미할당이나 **gateway neutral 판정에 포함**(`remote_input_gateway.py:182`) —
  DRIVE ACK는 chord를 뗀 뒤 성립. "chord 유지 중 재연결" 수용 시험 포함.
- **비상 chord(D1)**: **L1+R1+□ 5 s** = `estop_reset`, 이후 **L1+R1+△ 3 s** =
  `arm`. 클라이언트 chord는 `emergency_begin/execute` 2단계 요청으로 변환되고
  **시간 검증은 broker가 한다**(§3.1). 햅틱 점증→완료 스냅→ACK 더블 펄스.
  평시 콘솔 경로 우선, 비상용임을 훈련·문서 명시.
- 부분 입력(조합 일부 해제)은 즉시 무효·타이머 리셋. 주행 프레임 형식 불변.

### 3.3 콘솔 운용 패널 (A2b, operator_console 헌장 개정)

- 기존 창 안 패널. 모든 버튼 = 인라인 2단 확인(1단 원인·상태, 2단 확인 스트립) +
  실행 직전 **revision 재검증**(불일치 시 1단 복귀). `estop_reset`(확인 스트립)과
  `arm`(1.5 s hold-to-confirm)은 **서로 다른 제스처 + 비명령 스페이서 분리**.
  reset→IDLE, no implicit arm — 각각 독립 버튼·독립 확인.
- 범위: `clear_transient_hold`, 권한 3종, `estop_reset`·`arm`·`disarm`,
  `arm_lock_override`(강확인), 미션 6종, OPERATOR_HOLD/RESUME, 브링업 상태·트리거,
  extraction_grant(C0 이후).
- GTK 동시성: ops 클라이언트 전용 스레드 — nonblocking reconnect, bounded send
  queue, request_id↔ACK correlation, `GLib.idle_add` UI 반영, 종료 join.
  GStreamer 백프레셔로 UI가 얼 수 있는 구조적 한계는 D1 비상 chord가 최종 방어선.
- 헌장 개정 범위: 관측성 계획 :329-334 + Task 6 acceptance, 마스터플랜 복구 표면·
  service ownership·DoD 서술, 코드 독스트링·배너(`app.py:627`)·README·
  `telemetry.py`, 프로젝트 CLAUDE.md.
- "ops 채널 클라이언트 외 어떤 제어 송신 경로도 없음" 계약 테스트 신설(수신 경로
  불변; 계획이 주장하던 미실존 no-send 테스트의 실물 대체).

### 3.4 당일 브링업 무SSH화 (A2c)

- **감독 구조**: `wp5_control.launch.py`(+teleop; **broker 제외** — §3.1 단일
  소유)를 compose 서비스 PID 1로 실행. systemd 유닛은 compose 스택 기동 담당:
  `Requires/After=docker.service`, `ExecStartPre` ①`scripts/can_setup.sh`(호스트
  root) ②env 검증 — `/etc/powertrain/powertrain.env` 존재 + `stop_mm` sane-band +
  `STOP_MM_PROVENANCE=BENCH|COMMISSIONED`(BENCH → 콘솔 경고 배지) ③runtime-dir·
  토큰 파일 검증. DDS 도메인·RMW·`authority_enabled=true`·`use_sim_time=false`
  고정. **커미셔닝 전 유닛 disabled 설치**(벤치 수동 start).
- **crash-loop 비콘**: 독립 초경량 systemd 서비스 — 유닛 상태·journal 꼬리를
  UDP로 콘솔 push(토큰 redaction 적용). 모터 열거 지연 대비 preflight 재시도 창
  (기본 30 s).
- **캘리 NVM 영속화**: 신규 `bl70200_persist_calibration.py` — ①보드 시리얼→CAN
  노드쌍(11/12·13/14·15/16) 레지스트리 ②`find_any(serial_number=...)` 보드별
  접속 ③axis0+axis1 양축 캘리 성공 확인 후 보드당 1회
  `pre_calibrated=True`+`save_configuration`(리부팅) ④재열거→`--read` 전수 대조 —
  보드 단위 트랜잭션. NVM 쓰기는 정본 CFG 준수.
  ⚠️ **보드 fw = 0.5.1**(사용자 확인; 0.5.6은 라이브러리) — HALL polarity 상태
  (0.5.2+)는 근거로 쓰지 않는다. 실증 정본 = 벤치 게이트.
- **부팅 자격화 = chassis 소유 게이트**: 무회전 판독(`pre_calibrated`·
  `motor.is_calibrated`·`encoder.is_ready`·에러 플래그) + **보드 지문**(시리얼+
  설정 CRC — 교체 맹검 방어) + 전압. **모든 arm 경로(broker·`~/arm`·chord)가
  통과 필수.** **power-session 무효화(Codex r2-9)**: Jetson 생존 중 ODrive만
  재부팅(전압 sag 등)한 경우를 heartbeat 상태 리셋으로 감지 → 자격화 즉시 무효 →
  재판독 통과 전 arm 거부.
- **캘리 폴백 = lifecycle job**: `calibration_start/status/cancel`(축당 ~55 s,
  exclusive — 실행 중 arm 거부). 콘솔 확인("바퀴 전부 리프트") + broker 1회용
  wheels-up 토큰 이중 게이트. 조립 후엔 정비 절차임을 문서화.
- **조립 전 벤치 게이트**: wheels-up 캘리→영속→**전원 사이클 3회 × 6축 직진입
  closed-loop 재현** = 차체 조립 선행조건. 실패 시 fw 업그레이드 결정 회부.
- 완료 후 CLAUDE.md·Notion "캘리 RAM-only" 서술 갱신.

### 3.5 햅틱·LED (A2b; 입력=pygame, 출력 전부=pydualsense — BT 동시 실증 완료)

- 실증(2026-07-17, BT): rumble·라이트바·플레이어 LED·트리거 Rigid/Pulse 동작,
  pygame 20 Hz 입력과 동시 12 s 무충돌. pydualsense = 노트북 신규 의존성.
- **햅틱 arbiter**: 우선순위(E-stop > 권한 전이 > 링크 상실 > US-100 근접 >
  bypass) 단일 중재, 한 번에 한 패턴. 진동 duty 상한 + 조종기 충전 체크리스트.
- Tier 1(rumble): ①chord/비상 2단계 진행 ②US-100 근접 빌드업 ③권한 전이
  ④링크 상실 하트비트 ⑤bypass 상시 미세.
- **stale 방어**: ops-state age > 0.5 s → 링크 상실 패턴 강제 + 라이트바 중립 —
  stale push에서 긍정 상태 표시 금지.
- **장애 격리**: pydualsense 출력 격리 스레드 + 예외 시 출력만 자동 비활성(입력
  경로·프로세스 전파 금지). 예비 조종기 사전 페어링 + joystick 재열거(핫스왑).
- Tier 2(feature-flag): 무권한 잠김(강한 저항+펄스 — 풀 강성 배제), 슬립 플러터
  (**체감 검증 C1로 이월**), 라이트바 상태색.

## 4. A3 — 실행 배선

1. **WP8 힌트→집행**: `/section/state` → versioned SectionState(session/sequence/
   stamp/TTL). chassis_node `section_enforcement` 파라미터(기본 off,
   `authority_enabled` 필요).
   집행: `speed_hint` → v 클램프; `hold_hint` → v·ω=0; stale·미래·역행 → hold;
   **휴면 정합 가드**: 플로어가 재도입(>0)됐는데 힌트<플로어 환산속도면
   0/hold fail-close + WARN(플로어로 승격 금지, §2.2) —
   `CornerModule.set()` 입력 기준 캡 준수 테스트. `work_request`·마커는 콘솔
   표시만.
2. **마커 ledger**: 후보→확정 2단계(N=2 tracklet) + 후보 TTL·최대 수·공간/ID
   연속성·stamp 역행 거부. `unique_markers`는 확정만. class_id=instance-ID 계약
   확정 전 위치 클러스터 유지.
3. **teleop 이벤트 큐 재설계 + 상한(Codex 33)**: 모션 프레임 latest-only.
   중요 이벤트는 종류별 계약 — **E-stop = ACK까지 durable 단일 슬롯**,
   connect/disconnect = 세션별 latest, violation/error = 종류별 count-coalescing.
   overflow 시 MOTION_HOLD + journal(무한 큐 금지).
4. **복귀 dwell**: `recovery_ticks`(3틱) + 최소 경과 시간(0.15 s) + fresh 샘플 수.

## 5. B — 시뮬 포트폴리오 (§10-4 TRACKING류 검증의 명시 개정)

수치 수용 기준은 **초기값**으로 아래에 고정하고, dev-seed 보정 시 스펙 개정으로만
변경한다(Codex 23).

### B1 (S급)

1. **핀치포인트**: per-station 폭 축소(기존 경로 `procedural.py:318`→
   `model_builder.py:120-122`). 초기 기준: 폭 ≥ 차폭+0.15 m → 통과율 ≥95%,
   폭 < 차폭+0.05 m → hold 판정 100%(fail-open 0).
2. **클로소이드 곡률**: 상수 `curvature_profile`(:286-302) → 구간 선형/클로소이드.
   초기 기준: |dκ/ds| ≤ 0.08 m⁻² 가족에서 이탈 없이 완주, fail_open 0.
3. **기복 지형**: per-station `elevation_m` 변주(heightfield 불필요 — 3-D
   `_segment_axes` 확인). 초기 기준: ±0.05 m/2 m 파장에서 pitch 추정 안정·완주.
4. **fixture_class 계약화**: 라벨→실행 가능 검증(위반 시 테스트 실패).

### B2 (M/L급)

5. **마찰 fixture 계약화**: 기존 per-station `friction_coefficient`
   (`model_builder.py:123-126`, 이미 물리 μ)를 저마찰 patch fixture로 계약화.
   동적 패치는 필요 시 별도 스키마. + `closed_loop.py:183` `diagnostics=None`
   해제(DriveDiagnostics 배선). 초기 기준: μ=0.3 patch에서 slip_candidate 검출률
   ≥80%, 오검출(정상 구간) ≤5%.
6. **스모그 depth 열화**: 새 fault 그룹 `depth_degradation`(스키마+파서+러너).
   초기 기준: 열화 램프에서 fail-open 0, CONTROLLED_HOLD 진입 후 §6 정책 연동.
7. **동적 선도 표적**: `detections_source` 훅 + follow 폐루프 드라이버 + 표적
   plant 궤적·상대 pose/bbox 합성·가림/dropout·틱 순서·간격/재추종 점수화·
   recording 스키마. 초기 기준: 정속 0.5 m/s 표적 60 s 추종 — 간격 2.0±0.5 m
   체류율 ≥90%, 5 s 가림 후 재획득 ≤3 s, min 1.5 m 침범 0.
   (5구간 추종 배점: 간격 35+재추종 25, 완주 10 별도 — **유일한 폐루프(plant-급)
   검증 경로**; fake-target 단위시험 기존재.)
8. **hidden-seed 캠페인**: 가족×시드 매트릭스 러너. dev/regression/hidden 시드
   분리, hidden은 결과 해시만 기록·수동 열람.

## 6. C0/C1

### 6.1 C0 — 후진 한정 extraction 상태 (D2, WP5.1 의미론 개정, 상태표 완성판)

| 항목 | 정의 |
|---|---|
| 진입 | **`latched_sources == {us100}` 단독일 때만** 콘솔 `extraction_grant`(강확인) 수락. 다른 E-stop 원인 공존 시 거부 |
| 명령 소유 | DualSense **deadman 유지 + fresh TELEOP 프레임**만. autonomy·기타 소스 무시 |
| 최종 clamp | **chassis 최종단**에서 −0.2 m/s ≤ v ≤ 0, ω = 0 강제(kinematics 이전이 아니라 CornerModule 입력 기준) |
| 플로어 | 폐지(기본 0, §2.2)로 무관 — 재도입되더라도 extraction은 적용 예외 |
| budget | grant당 TTL 3 s(단조 시계) + latch당 누적 후진 ≤1.0 m·grant ≤3회 |
| 중단 | TTL 만료·deadman 해제·타 fault 발생·budget 소진 → 즉시 정지 + **원래 US-100 ESTOP latch 유지**(자동 reset 아님) |
| 종료 | 후진으로 거리 확보 → US-100 active 해제 → 통상 reset→arm 흐름 |
| 게이트 | **벤치 HIL 통과 전 feature-flag off** |

문 PUSH 구간 생존성 확보. WP5.1 canon 개정으로 문서화(§0 정본 개정 3).

### 6.2 C1 — WP9 degradation FSM + 지령속 계약

1. **지령 휠속 계약 변경 전체**: post-kinematics·post-floor 지령을
   `WheelSnapshot` → `snapshot()` → `WheelState.msg` additive 필드 →
   `message_adapter` → `odometry_node`(기존 getattr seam) 체인으로 공개.
   msg 변경 = dev/ros/Jetson 재빌드·type-hash 배포 포함.
2. **degradation FSM(순수 모듈)**: slip×stuck×depth 조건별 단계·진입/해제
   hysteresis·시도/거리/시간 budget·복구 명령·원격 핸드오버 대기 상태 + 출력
   계약(속도 스케일·hold 요청·handover 요청). bounded auto-recovery는 계획 §6
   :855-856 직접 규정.
3. **스모그 저속 통과**: 플로어 폐지(§2.2)로 즉시 설계 가능 — SMOG 힌트와
   결합한 저속 정책. ICE 0.15 m/s(0.24 rev/s)는 HALL 코깅존 내부 — §2.2b
   마찰보상 ff가 1차 수단이고, 잔여 정지는 FSM stuck 감지가 잡으며, 실주행
   신뢰성 확정은 커미셔닝 실측 몫.
4. **문서**: 핸드오프 WP표 WP9 행 추가, §0 명시 개정 4건 동기, Notion 동기.

## 7. 검증 전략

- 배치마다: Codex 위임 → 3환경 회귀(호스트 240 / dev 979+2skip / ros 410 + 신규)
  → 커밋 체인. pure-core + 노드 테스트 관례.
- **A2 안전 프로토콜 독립 수용 기준**: 역할별 토큰 위조/무토큰/스왑 거부·replay·
  재접속 status_query·PENDING→FINAL 후행 ACK·부분 clear·E-stop 구독자 재시작
  (TRANSIENT_LOCAL 수신)·큐 overflow→MOTION_HOLD·broker kill/재시작·cold boot·
  chord 유지 중 재연결·**비상 2단계 서버 시간 검증(조기 execute 거부)**.
- **C0**: 상태표 전 행 단위시험 + 벤치 HIL(실모터, FULL HIL 규율).
- **C1**: slip×stuck×depth 조합·hysteresis·budget 소진·핸드오버·min_rev 상호작용·
  diagnostics 생산자 사망 — 순수 코어 + full-stack. 슬립 플러터 체감(A2 이월).
- 시뮬: dev-seed 앵커(0.805, 0.04 s) + §5 수치 기준.
- **A배치 Jetson 벤치 스모크**: broker·패널·유닛·비콘·자격화 + 햅틱/트리거 체감 +
  chord(비상 포함) + **20분 연속 출력 soak** + **저속 추종 검증(§2.2b —
  wheels-up 0.15/0.25 m/s 상당 지령에서 ff 튜닝·서징 확인·단위 확정)** —
  배치 말 사용자 물리 확인 1세션. 캘리 영속화·캘리 잡·C0·저속 추종은 모터 회전
  수반(FULL HIL 규율).
- §11 금지 15항 전수 대조(독립 검증 포함) — 저촉 없음.
- r2 Codex 재검토 완료(해소16/부분9/신규9 → r3 반영). r3은 구현 계획 단계에서
  배치별 스펙으로 분해하며 추가 적대 검토는 배치 리뷰로 대체.

## 8. 하지 않는 것 (범위 밖)

- min_rev 플로어 재도입 여부·수치(커미셔닝 실측 재량 — 기본 0=off 유지, §2.2),
  `stop_mm` 구간별 값(커미셔닝).
- gateway/authority hold 자동복구 의미론 변경(§2.3).
- `work_request` 자동 집행, 팔 협업 계약, 팀원 gateway WIP 접촉.
- 크랩 조향, 서멀 쿨다운, 자동 타임박싱 강등(안건 보존).
- pygame rumble 출력 경로, challenge-HMAC/mTLS/PKI(역할별 정적 토큰으로 충분 —
  물리 보안 전제 잔여 위험 수용 명시), US-100 tilt 보상(WP9 후속 백로그).

## 9. 리스크

- pydualsense BT 장시간: 20분 soak, Tier 2 feature-flag.
- broker 콜 데드락: call_async+타임아웃+분리 callback group, 지연 서비스 fixture.
- 콘솔 개정 안전 퇴행: 역할 토큰 + revision·predicate 이중 검증 + 송신 계약
  테스트 + E-stop 발동은 콘솔 비탑재.
- 토큰 파일 자체가 새 SPOF: preflight 검사 + 비콘 경고 + 수동 rotation 절차.
- NVM stale 캘리: 지문 + power-session 무효화 + 전 경로 arm 거부 + 3회
  전원사이클 게이트. fw 0.5.1 미지원 판명 시 fw 업그레이드 결정 회부.
- extraction 오남용: §6.1 상태표(단독 원인·deadman·budget·타 fault 즉시 차단) +
  벤치 HIL 선행 + feature-flag.
- 명령 경합: 단일 mutation 직렬화 + revision/predicate 이중 검증 + journal.
- **플로어 폐지의 잔여 위험(D3/D4)**: 초저속 지령(코깅존 <0.3 rev/s)에서 바퀴가
  정지한 채 텔레메트리는 정상으로 보일 수 있음(2026-07-05 HIL 실측) — §2.2b
  마찰보상 ff가 1차 방어, C1 WP9 stuck 감지가 2차, 실차 저속 튜닝은 커미셔닝
  재량. ff 벤치 튜닝 전·C1 전까지 저속 자율 프로파일 운용은 벤치 검증 범위로
  한정.
