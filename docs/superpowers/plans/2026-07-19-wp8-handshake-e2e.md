# WP8 실팔 핸드셰이크 E2E 하네스 Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **이 레포 운용 현실**: 구현은 Codex 위임(git 금지), 커밋·젯슨 실행·검증은 리뷰어 몫.

**Goal:** 실제 팔 `arm_fsm_node`(headless)와 우리 chassis v2를 도메인 77에서 맞붙여
계약 표면을 검증하고, 계약 충실 fake-arm으로 풀사이클·오류 경로를 검증하는
재실행 가능한 E2E 하네스를 만든다 (스펙
`docs/superpowers/specs/2026-07-19-wp8-handshake-e2e-design.md`).

**Architecture:** 순수 판정기(`wp8_scenario.py`, 이벤트 리스트 → Finding) ↔ rclpy
probe(`wp8_handshake_probe.py`, 관측·자극·fake-arm — 판정은 순수 모듈 호출) ↔ bash
오케스트레이터(`scripts/wp8_handshake_e2e.sh`, 컨테이너 기동·SIGSTOP·페이즈 순서·
killpg). 판정 로직은 하드웨어 없이 x86 pytest로 검증한다.

**Tech Stack:** bash, rclpy(ROS2 Humble), pytest(fake-PATH 관례 = `scripts/tests/`).

## Global Constraints (스펙 §2·§6·§8 그대로)

- ROS_DOMAIN_ID=77 격리. 도메인 0(운용 스택)은 절대 건드리지 않는다.
- chassis는 `fake:=true` + `safety_required:=false`(BENCH/FAKE 전용) + 
  `contract_v2_verified:=true` + `mission_contract_owner:=chassis_supervisor` + 
  `mission_id_path:=/tmp/wp8_mission_id`(운용 store 오염 금지)로만 기동.
- 팔 레포 파일 수정·파라미터 주입 금지 — `arm_fsm` 설치본을 기본값 그대로 실행.
- 팔팀 프로세스 kill 대상은 우리가 이 하네스에서 띄운 `arm_fsm`뿐. perception·
  stream·metadata_sender는 무접촉.
- 모든 기동은 setsid 프로세스 그룹 + 종료 시 killpg(§9-5).
- 어휘·토픽은 `powertrain_ros/contract.py`만 참조(문자열 하드코딩 금지).
- 코드 기본값 변경 금지 — `contract_v2_verified` 기본 false 유지(스펙 §5).

## 확정 사실 (구현자가 재조사할 필요 없음)

- chassis 노드명 = `chassis_node` → private 서비스는
  `/chassis_node/mission_arrive_pickup`, `/chassis_node/mission_arrive_drop`,
  `/chassis_node/arm`, `/chassis_node/reset_estop` (Trigger).
- 실행 파일: `ros2 run powertrain_ros chassis` / probe는 setup.py
  `console_scripts`에 `wp8_handshake_probe = powertrain_ros.wp8_handshake_probe:main`
  추가(`ros2/src/powertrain_ros/setup.py:53` 목록).
- 팔: 컨테이너 `ros2_humble`, `ros2 run dynamixel_control arm_fsm`(엔트리명은
  Task 3 Step 2에서 `ros2 pkg executables dynamixel_control`로 실행 전 확인).
- v2 `SupervisorConfig`(`motor_control/chassis/mission.py` 232행~): 
  `arrival_period_s=0.5`, `arrival_window_s=2.0`, `arm_status_timeout_s=0.5` 등 —
  판정 타임아웃 산정에 이 값 사용. 완료 권위 = `CARRYING_LOCKED`(픽업)/
  `STOWED_LOCKED`(하역), 작업 수락 = `WORK_ACCEPTED_STATUSES`.
- 팔 워치독: `chassis_mode` 1.0 s 미수신 → default-deny 잠금. 팔 heartbeat 10 Hz.
  stamp 신선도: 0·미래·동일·역행 거부 → 발행마다 now() 재스탬프 필수.
- `/arrival_status` 발행자는 `arm_gate_mode != absent_field`일 때만 생성됨 —
  하네스가 기동 후 `ros2 topic info /arrival_status`로 발행자 1 확인.

---

### Task 1: 순수 판정기 `wp8_scenario.py`

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/wp8_scenario.py`
- Test: `ros2/src/powertrain_ros/test/test_wp8_scenario.py`

**Interfaces (Produces):**
```python
@dataclass(frozen=True)
class Event:
    t: float                 # monotonic 초 (수신 시각)
    topic: str               # "chassis_mode" | "arrival" | "arm_status" | "marker"
    value: str               # mode/status 문자열, marker는 "sigstop"/"sigcont" 등
    mission_id: int | None = None

@dataclass(frozen=True)
class Finding:
    check: str
    ok: bool
    detail: str

def judge_baseline(events, *, window_s, min_heartbeat_hz=5.0) -> list[Finding]
def judge_pickup_conjunction(events) -> list[Finding]
def judge_resume(events, *, resume_t) -> list[Finding]
def judge_full_cycle(events) -> list[Finding]
def judge_fault(events, *, scenario) -> list[Finding]   # scenario ∈ {"no_response","late_done","failed_latch","dup_done"}
def summarize(findings) -> tuple[bool, str]              # (전체 PASS, 사람용 표)
```

판정 내용(스펙 §3·§4의 관측 가능 계약면만):
- baseline: `arm_status` 수신률 ≥ min_heartbeat_hz AND 창 내 `WORK_ACCEPTED_STATUSES`
  없음(도착 전 작업 금지) AND `chassis_mode`가 LOCK 모드(DRIVING 등)로 수신됨.
- pickup_conjunction: `chassis_mode`가 `MISSION_STOP`이 된 시각 t₁ < 첫
  `arrival(ARRIVED_PICKUP)` 시각 t₂ (순서 역전 없음) AND 이후
  `WORK_ACCEPTED_STATUSES` 등장 AND arrival mission_id 일관.
- resume: `resume_t` 이후에 `WORK_ACCEPTED_STATUSES` 재등장(재전송 conjunction
  재수락) — resume_t 이전 이벤트는 근거로 쓰지 않음.
- full_cycle: MISSION_STOP→ARRIVED_PICKUP→`CARRYING_LOCKED`→LOCK 모드 복귀(재출발)
  →MISSION_STOP→ARRIVED_DROP→`STOWED_LOCKED`→LOCK 모드 복귀, mission_id 단조 증가.
- fault: `no_response`=arrival 재전송 ≥2회 관측 AND 마지막까지 LOCK 모드 복귀
  없음(MISSION_STOP 유지); `late_done`=이전 mission_id 완료 수신 후에도 재출발
  없음; `failed_latch`=팔 FAILED 후 재출발 없음; `dup_done`=재출발 1회만(중복
  완료 무시).

- [ ] **Step 1: RED** — `test_wp8_scenario.py`에 판정별 수락 1 + 거부 1 케이스
  (합성 Event 리스트; 예: pickup 순서 역전 t₂<t₁ → ok=False가 반드시 있어야 함,
  full_cycle에서 CARRYING_LOCKED 없이 재출발 → FAIL). 총 ≥12케이스.
- [ ] **Step 2:** 실행해 전부 FAIL(모듈 부재) 확인:
  `python -m pytest ros2/src/powertrain_ros/test/test_wp8_scenario.py -q`
- [ ] **Step 3: GREEN** — 판정기 구현. contract 상수는
  `from powertrain_ros import contract` 재사용. 이벤트 스캔은 단순 선형 루프.
- [ ] **Step 4:** 같은 명령 PASS 확인.

### Task 2: probe `wp8_handshake_probe.py` (관측·자극·fake-arm)

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/wp8_handshake_probe.py`
- Modify: `ros2/src/powertrain_ros/setup.py:53` (console_scripts 1줄 추가)
- Test: `ros2/src/powertrain_ros/test/test_wp8_handshake_probe.py`

**Interfaces:**
- Consumes: Task 1 판정기 전부, `contract.py` 어휘.
- Produces: CLI `wp8_handshake_probe <subcommand> [--timeout-s F] [--node-ns /chassis_node] [--json PATH]`
  - subcommands: `baseline` / `pickup` / `resume --resume-t-file PATH` /
    `full-cycle` / `fault --scenario X`. exit 0=PASS, 1=FAIL, 2=실행 오류.
  - 각 subcommand: rclpy 노드 1개로 `/chassis_mode`·`/arrival_status`·`/arm_status`
    구독→Event 축적, 필요한 자극(서비스 콜) 수행, 종료 시 판정기 호출 →
    사람용 표 + JSON 한 줄(`{"subcommand":..., "pass":..., "findings":[...]}`).
- `pickup`은 시작 시 `/chassis_node/arm` 호출(ARMED로) 뒤
  `/chassis_node/mission_arrive_pickup` 호출. `resume`은 관측만.
- `full-cycle`/`fault`는 **fake-arm을 프로브 프로세스 안 별도 노드로 내장**:
  10 Hz `/arm_status` heartbeat(매 발행 now() 재스탬프 — 팔 stamp 규칙 §확정
  사실과 동일), conjunction(MISSION_STOP AND 자기 미완료 mission_id arrival)
  충족 시 0.5 s 후 `WORK_READY`→1.0 s 후 완료 상태(`CARRYING_LOCKED` 또는
  `STOWED_LOCKED`, arrival status로 픽업/하역 판별). fault 시나리오는 완료를
  안 보냄(no_response) / 이전 mission_id로 보냄(late_done) / `FAILED` 발행 후
  침묵(failed_latch) / 완료를 2회 보냄(dup_done).
- 실팔과의 이중 응답 방지: fake-arm subcommand는 하네스가 arm_fsm을 내린 뒤에만
  실행된다(Task 3 순서 책임) — probe는 시작 시 `/arm_status` 발행자 수를 세어
  fake-arm 켜기 전 발행자 ≥1이면 exit 2(가드).

- [ ] **Step 1: RED** — 순수 헬퍼 테스트: CLI 파싱(서브커맨드·기본값·미지 인자
  거부), `events_from_records()`(rclpy 메시지 튜플→Event 변환), JSON 요약 직렬화.
  rclpy 실기동은 젯슨 몫이라 테스트 대상 아님(순수 부분만).
- [ ] **Step 2:** FAIL 확인 (위와 같은 pytest 경로).
- [ ] **Step 3: GREEN** — 구현. rclpy import는 `main()` 안에서만(순수 헬퍼는
  모듈 최상위, x86 테스트가 rclpy 없이 import 가능해야 함 — 기존
  `chassis_telemetry` 계열 관례 동일).
- [ ] **Step 4:** PASS + `python -c "import powertrain_ros.wp8_handshake_probe"`가
  rclpy 없는 환경에서도 성공하는지 확인.

### Task 3: 하네스 `scripts/wp8_handshake_e2e.sh`

**Files:**
- Create: `scripts/wp8_handshake_e2e.sh` (실행권한, 상단 사용법·런북 주석 —
  협조 세션 실서보 재실행 절차 포함)
- Test: `scripts/tests/test_wp8_handshake_e2e.py` (fake-PATH 관례 =
  `test_jetson_gui_up.py`와 동일 패턴)

**Interfaces:**
- Consumes: Task 2 CLI 계약 전부.
- Produces: `bash scripts/wp8_handshake_e2e.sh [--phase1-only|--phase2-only]
  [--negative-control] [--timeout SEC]`, exit 0=전부 PASS / 1=FAIL /
  3=Phase1 SKIP(ros2_humble 부재)+Phase2 PASS.

동작(스펙 §2·§3·§4 순서 그대로):
1. 전제: `powertrain_ros` 컨테이너 running(아니면 즉시 실패 안내), 로그 디렉터리
   `/tmp/wp8_e2e_<UTC타임스탬프>/`.
2. 헬퍼 `start_chassis()`: `docker exec -d powertrain_ros bash -lc 'source
   /opt/ros/humble/setup.bash && source /workspace/ros2/install/setup.bash &&
   ROS_DOMAIN_ID=77 exec setsid ros2 run powertrain_ros chassis --ros-args
   -p fake:=true -p safety_required:=false -p contract_v2_verified:=true
   -p mission_contract_owner:=chassis_supervisor
   -p mission_id_path:=/tmp/wp8_mission_id'` + 기동 후
   `ros2 topic info /arrival_status`(도메인 77)로 발행자 1 확인.
   `stop_chassis()`: 컨테이너 내 `pkill -g <pgid>`(기록해 둔 setsid 그룹만).
3. Phase 1 (ros2_humble 있을 때만): `start_arm()`(도메인 77 arm_fsm, setsid) →
   probe `baseline` → probe `pickup` → chassis SIGSTOP:
   `docker exec powertrain_ros pkill -STOP -f 'lib/powertrain_ros/chassis'` →
   1.5 s → `-CONT` → resume 시각 기록 → probe `resume` → `stop_arm()`
   (우리가 띄운 arm_fsm 그룹만 kill — perception/stream 무접촉).
4. Phase 2: chassis 재기동(fresh) 후 probe `full-cycle` → fault 4종을
   각각 chassis 재기동 후 probe `fault --scenario X`.
5. `--negative-control`: pickup 자극 없이 `pickup` 판정만 실행 → **FAIL(exit 1)이
   나와야 하네스 게이트 자체가 증명됨**. 이 모드의 성공 조건은 "probe가 FAIL을
   보고"이며 하네스는 그때 exit 0으로 "음성 대조 OK"를 출력.
6. 각 단계 로그·JSON을 로그 디렉터리에 저장, 마지막에 한국어 요약 표.
7. trap으로 모든 setsid 그룹 killpg + 도메인 77 잔류 프로세스 검사 출력.

- [ ] **Step 1: RED** — fake-PATH pytest: ①`bash -n` ②해피패스(가짜 docker가
  probe 호출 기록) exit 0 + probe 서브커맨드 순서(baseline→pickup→resume→
  full-cycle→fault×4) ③`--negative-control`이 pickup 자극 경로를 안 태우고
  probe FAIL 시 exit 0 ④pkill 대상이 chassis 패턴·arm_fsm 그룹뿐(부정 단언:
  perception/stream/metadata_sender 문자열 없음) ⑤ros2_humble 부재 시 exit 3.
- [ ] **Step 2:** FAIL 확인: `python -m pytest scripts/tests/test_wp8_handshake_e2e.py -q`
- [ ] **Step 3: GREEN** — 구현(출력 한국어, `jetson_gui_up.sh`의 수집·요약 관례).
- [ ] **Step 4:** PASS + `scripts/tests` 전체 회귀.

### Task 4 (리뷰어 전용 — Codex 범위 아님): 젯슨 검증·활성 게이트 기록

- [ ] ros 컨테이너 colcon 재빌드(probe 엔트리 반영) 후 젯슨에서
  `bash scripts/wp8_handshake_e2e.sh` 2회(멱등) + `--negative-control` 1회.
- [ ] Phase 1 실팔 결과·headless 한계 지점(스펙 §3-7)·불일치 발견을 스펙 §5
  게이트 1단 증적으로 핸드오프 §2에 기록. 불일치 수정은 우리 쪽만.
- [ ] 3환경 회귀(호스트/dev/ros) + 커밋·푸시.

## Self-Review 결과

- 스펙 §1~§8 전 항목이 Task 1~4에 매핑됨(§5 활성화는 Task 4 기록 + 비범위 유지).
- placeholder 없음. Task 간 시그니처 일치(Event/Finding/CLI, 재확인함).
- 스코프: 단일 계획 적정. fake-arm을 probe에 내장해 파일 수 최소화(YAGNI).
