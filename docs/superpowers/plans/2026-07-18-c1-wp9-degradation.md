# C1 배치 구현 계획 — WP9 degradation FSM + 지령 휠속 계약 + 문서 정합 (최종 배치)

> **For agentic workers:** superpowers:subagent-driven-development 또는 executing-plans.
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 3환경(+pipefail) + 젯슨 실기(비회전).

**Goal:** 스펙 r6 §6.2 — 마스터플랜 §6 :848-857의 WP9(환경 degradation 정책)를
구현하고, 지령 휠속 계약 체인을 열어 slip 진단을 실질화하며, 프로그램의 정본
개정 4건(§0)을 문서에 이행한다. **A/B/C 프로그램 최종 배치.**

**Spec:** r6 §6.2(+§0 정본 개정 목록). 기준선(B2 후): 호스트 302 / dev 1131+2skip / ros 480 / 젯슨 480.

## 전제 사실 (B2에서 확정)

- **slip 검출 실한계**: production 추정기(휠+IMU)는 균일 저마찰 slip을 **0%**
  검출(절대 종방향 지면속도 불가시). FSM의 slip 단계는 검출 가능한 모드
  (stall형 — 휠·IMU 불일치)만 정직하게 다루고, 균일 slip 맹점은 문서화한다
  (시각 odometry 등은 범위 밖 후보).
- 지령 휠속 체인의 seam: `odometry_node.py:141-150` `getattr(wheel,
  "command_turns_per_s", ...)` — msg 필드만 추가되면 자동 수용.
- 지령값 원천: 코너 드라이브 `target_vel`(`chassis_manager.py` snapshot의
  wheel-consistency 경로에서 이미 사용).

## Global Constraints

- 이전 배치 승계. **msg 변경 = dev/ros/젯슨 3환경 재빌드 필수**(type-hash) —
  젯슨 동기 시 colcon 재빌드 포함.
- §11 준수: 자동 재시도는 **bounded**(§6 :855-856이 직접 규정), 핸드오버는
  **대기 상태 + 운영자 조작**(자동 전환 금지), run 중 온라인 튜닝 없음.
- FSM은 순수 모듈(시계·입력 주입) — production 노드와 sim이 동일 코어 소비.

---

### Task 1: 지령 휠속 계약 체인

**Files:**
- Modify: `motor_control/chassis/telemetry.py` (`WheelSnapshot.command_turns_per_s: float = 0.0`), `motor_control/chassis/chassis_manager.py` (`snapshot()`에서 코너 target_vel 채움), `ros2/src/powertrain_msgs/msg/WheelState.msg` (additive `float32 command_turns_per_s`), `ros2/src/powertrain_ros/powertrain_ros/message_adapter.py` (필드 매핑)
- Test: `motor_control/chassis/tests/test_chassis_manager.py`·`ros2/src/powertrain_ros/test/`의 message_adapter/odometry 테스트 확장

**Interfaces:**
- post-kinematics·post-floor(현재 플로어 0이므로 = kinematics 출력) 지령을
  바퀴별로 노출. `odometry_node`는 기존 getattr seam으로 자동 수용 —
  **slip_candidate가 지령 vs 실측 불일치로 실질화**되는지 노드 테스트로 확인
  (stall형: 지령>0·실측≈0 → slip/stuck 후보).
- [ ] RED→구현→GREEN(3환경 재빌드) → 커밋
  `feat: command wheel-speed contract chain (WheelState additive field)`

---

### Task 2: WP9 degradation FSM (순수)

**Files:**
- Create: `powertrain_autonomy/degradation.py`
- Test: `powertrain_autonomy/tests/test_degradation.py`

**Interfaces:**

```python
class DegradationStage(Enum): NORMAL, SLOWDOWN, HOLD_RECOVERY, HANDOVER_WAIT
@dataclass(frozen=True)
class DegradationConfig:
    slowdown_scale: float = 0.5          # SLOWDOWN 단계 속도 스케일
    enter_depth_dropout: float = 0.35    # 열화 진입(신뢰도 저하 비율)
    exit_depth_dropout: float = 0.20     # 해제(hysteresis)
    stuck_enter_ticks: int = 5
    recovery_attempts_max: int = 3       # bounded(§11) — 소진 시 HANDOVER_WAIT
    recovery_time_budget_s: float = 8.0
    recovery_distance_budget_m: float = 1.5
@dataclass(frozen=True)
class DegradationOutput:
    stage: DegradationStage
    speed_scale: float                   # 1.0 | slowdown_scale | 0.0
    request_hold: bool
    handover_wait: bool                  # 운영자 개입 대기(자동 전환 없음)
    reasons: tuple
class DegradationFsm:
    def __init__(self, config=None, *, clock): ...
    def update(self, *, depth_quality: float|None, slip_candidate: bool,
               stuck_candidate: bool, traveled_m: float, now_s: float
               ) -> DegradationOutput
    def operator_reset(self)             # HANDOVER_WAIT 해제는 운영자 조작만
```

- 전이: NORMAL→SLOWDOWN(depth 열화 or slip 후보, hysteresis 진입/해제)·
  SLOWDOWN→HOLD_RECOVERY(stuck 연속 or 열화 심화)·HOLD_RECOVERY는 bounded
  재시도(시도/시간/거리 budget — 하나라도 소진 시 HANDOVER_WAIT)·
  HANDOVER_WAIT는 `operator_reset()`만 탈출(§11 자동 재개 금지).
- [ ] 테스트 10케이스: 각 전이·hysteresis 미스침·budget 3종 소진·operator_reset·
  reasons 누적·시계 주입 결정론.
- [ ] RED→구현→GREEN → 커밋 `feat: WP9 pure degradation FSM (bounded recovery, handover wait)`

---

### Task 3: 노드 배선 + 시뮬 통합

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/autonomy_controller_node.py` (FSM 인스턴스·`/autonomy/degradation_state` String JSON 발행·journal `DEGRADATION`), `powertrain_sim/closed_loop.py` (driver에 FSM 통합 — speed_scale을 DriveDiagnostics.speed_cap_m_s로 반영)
- Test: `ros2/src/powertrain_ros/test/test_autonomy_controller_node.py` 케이스 + `powertrain_sim/tests/test_closed_loop.py` smog+FSM 케이스

**Interfaces:**
- 노드: `/odom_diagnostics`(slip/stuck)·depth 품질(기존 depth_quality 경로)을
  FSM에 공급, 출력 speed_scale → 컨트롤러 diagnostics.speed_cap 경로(기존
  seam), request_hold → CONTROLLED_HOLD 사유 추가, handover_wait → 상태 토픽
  +journal(운영자 콘솔이 보게 — 자동 전환 없음). `operator_reset`은 ops 채널
  후속 배선 후보로 주석만(§8 범위 밖 — 현재는 노드 재기동/파라미터).
- 시뮬: smog 가족 dev seed에서 FSM 경유 시 SLOWDOWN 관측 + fail_open 0 유지
  (앵커 변동 시 정직 갱신·사유 주석).
- [ ] RED→구현→GREEN → 커밋 `feat: degradation FSM wiring (node + sim loop)`

---

### Task 4: 문서 정합 — 정본 개정 4건 이행 + WP9 행 (리뷰어 주도)

- [ ] **핸드오프 §1 WP표에 WP9 행 추가**(아이디어 세션 지적 갭 해소) + §2 C1 행.
- [ ] 마스터플랜 개정: ①콘솔 헌장(§5 원격운용 절 — A2b 반영) ②§10-4 TRACKING류
  구간 HIL의 시뮬 영구 대체 ③WP5.1 US-100 의미론에 extraction 추가(§6 or 안전
  절) ④"캘리 RAM-only" 서술 → NVM 영속화 경로(벤치 게이트 전제) — 각 개정에
  스펙 r6 §0 근거 각주.
- [ ] 관측성 계획 :329-334 + Task 6 acceptance 개정, wp5.2 계획의 콘솔 언급 확인.
- [ ] 프로젝트 CLAUDE.md 상단 배치(§WP5.1 Override 등)와 충돌 서술 정리.
- [ ] **Notion 동기**: 전체계획 페이지에 A/B/C 프로그램 요약 콜아웃(+개정 4건),
  쓰기 후 재조회(레포 규칙).
- [ ] 커밋 `docs: canonical amendments (console charter, sim substitution, extraction, NVM cal) + WP9 row`

---

### Task 5: 3환경·젯슨 실기·프로그램 마감 — 리뷰어 주도

- [ ] 3환경 green(**msg 변경 재빌드 포함**) + 캠페인 dev 매트릭스 재실행 exit 0.
- [ ] 젯슨: pull + **colcon 재빌드**(msg!) + parity ×2 + FAKE autonomy
  degradation_state 발행 스모크(process-group kill).
- [ ] **프로그램 종합 보고**: 스펙 r6 배치표 전체 ✅, 벤치 이월 통합 목록
  (A1 ff 튜닝·A2 chord/햅틱 체감·A2c NVM/캘리/전원사이클·C0 후진 HIL·
  WP7 feedforward 후보·slip 실측정) — 핸드오프 §6 갱신 + 메모리.
- [ ] 커밋·push·젯슨 동기.

## 완료 기준

- WP9 FSM 10계약 + 노드/시뮬 배선 green, 지령속 체인으로 stall형 slip 실질화.
- 정본 개정 4건이 실제 문서에 반영(스펙 §0 목록 소진), 핸드오프 WP9 행 존재.
- **A/B/C 프로그램 9배치 전부 완료** — 벤치 이월 목록으로 실기 게이트 인계.
