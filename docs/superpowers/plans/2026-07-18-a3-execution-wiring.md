# A3 배치 구현 계획 — 실행 배선 (WP8 힌트 집행 · 마커 ledger · 큐/dwell 강화)

> **For agentic workers:** superpowers:subagent-driven-development 또는 executing-plans.
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 3환경 + 젯슨 실기.

**Goal:** 스펙 r6 §4 — ①`/section/state`를 versioned 계약으로 강화하고 chassis가
힌트를 **집행**(신호 20점 실패조건 폐쇄) ②마커 ledger 후보→확정 2단계(3구간 50점
실전 준비) ③teleop 이벤트 큐 종류별 계약(무한 큐 제거) ④복귀 dwell 시간·샘플 보강.

**Spec:** r6 §4 (+§2.2 휴면 플로어 가드). 기준선(C0 후): 호스트 259 / dev 1063+2skip(→C0 후 재측정치 기준) / ros 470 / 젯슨 470.

## Global Constraints

- A2a~C0 계획의 Global Constraints 승계.
- `section_enforcement` 기본 **off** — 벤치 검증 후 활성(§1 배치표 주석).
- work_request·마커 진행은 자동 집행 금지(콘솔 표시만 — 팔 크로스팀 대기).
- 젯슨 실기(비회전): FAKE chassis + fake `/section/state` 발행으로 클램프·stale
  fail-close 동작 확인, parity. **프로브는 process-group kill(§9-5 함정) 필수.**

---

### Task 1: versioned SectionState — supervisor 발행 강화

**Files:**
- Modify: `motor_control/chassis/section_profiles.py` (SectionConfig에 `state_ttl_s: float = 0.6`), `ros2/src/powertrain_ros/powertrain_ros/section_supervisor_node.py` (payload 확장)
- Test: `ros2/src/powertrain_ros/test/test_section_supervisor_node.py` 케이스 추가

**Interfaces:**
- `/section/state` JSON에 추가: `"schema_version": 1`, `"session_id"`(노드 기동 시
  uuid4 hex), `"sequence"`(단조 증가), `"stamp_s"`(publish 시각 monotonic),
  `"ttl_s"`(config `state_ttl_s`). 기존 필드 불변(additive).
- [ ] RED(기존 테스트에 스키마 케이스 추가: 필드 존재·sequence 단조·session 고정)
  → 구현 → GREEN(ros 컨테이너) → 커밋
  `feat: versioned /section/state (session, sequence, stamp, ttl)`

---

### Task 2: chassis_node section enforcement — 클램프·fail-close

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py` (구독+집행), `motor_control/chassis/` 신규 `section_enforcement.py` (순수 판정기)
- Test: `motor_control/chassis/tests/test_section_enforcement.py` (신규) + `ros2/src/powertrain_ros/test/test_section_enforcement_wiring.py` (AST/소스 계약)

**Interfaces:**
- 순수 `SectionEnforcer(config, clock)`:

```python
@dataclass(frozen=True)
class EnforcementDecision:
    v_cap: float | None      # None = 제한 없음
    force_hold: bool         # v·ω 모두 0
    reason: str
class SectionEnforcer:
    def feed(self, payload: dict, received_s: float)   # 검증: schema 1·같은
        # session에서 sequence 역행 drop·stamp 미래(>0.5 s) drop
    def decide(self, now_s: float, *, floor_v_m_s: float = 0.0) -> EnforcementDecision
        # 수신 없음/ttl 초과(received_s 기준) → force_hold("stale")
        # hold_hint → force_hold("hold_hint")
        # speed_hint>0 → v_cap; 단 §2.2 휴면 가드: 0<speed_hint<floor_v_m_s
        #   (플로어 재도입 시) → force_hold("hint_below_floor")
        # supervisor disabled(payload enabled=False) → 제한 없음(advisory 유지)
```

- chassis_node: `section_enforcement` 파라미터(기본 False, `authority_enabled`
  필요). `_tick_authority`의 최종 `cm.set(final_v, final_omega)` 직전에 decide
  적용: force_hold → `cm.set(0,0)`+1 Hz journal(`SECTION_ENFORCEMENT`), v_cap →
  `final_v = clamp(final_v, ±v_cap)`(부호 보존). 플로어 환산 `floor_v_m_s =
  cfg.min_drive_turns_per_s × 2πr`.
- 테스트(순수 9): 정상 캡·hold_hint·stale ttl·sequence 역행 drop·미래 stamp
  drop·세션 교체 수용·disabled 무제한·hint<floor(플로어>0 주입 시) hold·
  `CornerModule.set()` 기준 캡 준수(FakeCorner로 chassis 통합 1케이스 —
  test_chassis_manager 스타일).
- [ ] RED→구현→GREEN→커밋 `feat: section hint enforcement with versioned-state fail-close (default off)`

---

### Task 3: 마커 ledger 후보→확정 2단계

**Files:**
- Modify: `motor_control/chassis/section_profiles.py` (`MarkerDedup`)
- Test: `motor_control/chassis/tests/test_section_profiles.py` 케이스 추가

**Interfaces:**
- `SectionConfig` 추가: `marker_confirm_observations: int = 2`,
  `marker_candidate_ttl_s: float = 5.0`, `marker_max_candidates: int = 16`.
- `MarkerDedup.observe(...)`: 첫 수락 → **후보**(카운트 0 유지). 같은 실체
  (class_id 우선/공간 클러스터)의 `min_reobserve_s` 지난 재관측 → 관측수 증가,
  `marker_confirm_observations` 도달 시 **확정**(unique_count 증가는 이때만).
  후보 TTL 초과 → 소멸. 후보 수 상한 초과 → 최고(古) 제거. stamp 역행 관측
  거부(기존). `unique_markers`는 확정만. 기존 `MarkerObservationRecord`에
  `stage: "candidate"|"confirmed"` 필드.
- 테스트(7): 1회 관측=후보(카운트 0)·2회 확정(+1)·TTL 소멸 후 재관측=새 후보·
  상한 초과 古제거·min_reobserve 내 중복은 무진전·class_id 무시 시 공간 클러스터
  동작 유지·기존 supervisor 목표(5종) 흐름 회귀.
- [ ] RED→구현→GREEN(supervisor 노드 테스트 회귀 포함)→커밋
  `feat: two-stage marker ledger (candidate TTL -> confirmed count)`

---

### Task 4: teleop 이벤트 큐 종류별 계약

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/teleop_command_node.py`
- Test: `ros2/src/powertrain_ros/test/test_teleop_command_node.py` 케이스 추가

**Interfaces:**
- `_events` SimpleQueue 대체(스펙 §4.3, Codex 33): ①**모션 프레임 latest-only
  슬롯**(락 하 교체 — 이전 프레임 drop 카운터) ②**수명 이벤트**(connect/
  disconnect) 세션별 latest(순서 보존 소형 deque maxlen 8) ③violation은 기존
  rate-cap 유지 + 종류별 카운트 병합(deque maxlen 64) ④`estop_edge`는 이미
  `/teleop/estop` latched로 별도 경로(A1) — 프레임 슬롯 drop과 무관하게
  `_begin_estop_event`는 **수신 즉시**(drain 아님, `_queue_decoder_results`
  경로에서 플래그) 처리해 유실 불가. drain은 슬롯·deque 소비로 재작성,
  `MAX_EVENTS_PER_TICK` 의미 유지. overflow(수명 deque 포화) → gateway
  `contract_violation`("event overflow") → MOTION_HOLD 경로.
- 테스트(5): 모션 폭주 시 최신만 소비+drop 카운트·estop_edge가 drain 전에
  latched 발행·disconnect 유실 없음·violation 병합 카운트·overflow→hold.
- [ ] RED→구현→GREEN(기존 teleop·estop·gateway 테스트 전부 회귀)→커밋
  `feat: typed teleop event queues - latest-only motion, lossless lifecycle, coalesced violations`

---

### Task 5: 복귀 dwell 시간·샘플 보강

**Files:**
- Modify: `powertrain_autonomy/controller/core.py`
- Test: `powertrain_autonomy/tests/test_autonomy_controller.py` 케이스 추가

**Interfaces:**
- `AutonomyControllerConfig` 추가: `recovery_min_elapsed_s: float = 0.15`,
  `recovery_min_samples: int = 3`(기존 recovery_ticks와 AND 결합 — 셋 다
  충족해야 복귀). hold 진입 시 exit-후보 시각·표본 카운트 리셋(기존
  `_controlled_hold(reset_recovery=True)` 지점).
- 테스트(4): 틱만 충족·시간 미달 → 유지, 시간 충족·틱 미달 → 유지, 셋 충족 →
  복귀, 시뮬 dev-seed 앵커 영향 확인(폐루프 recovery 0.04 s가 dt 0.02×3틱 =
  0.04 s ≥ 0.15? **주의**: 0.15 s 기본이면 폐루프 anchor가 0.04→0.16으로 변함 —
  `test_closed_loop.py`의 `pytest.approx(0.04)` 핀을 **의도 변경으로 갱신**
  (사유 주석: dwell 시간 조건 추가, hold 의미 불변·복귀만 지연). fail_open 0
  불변 확인.
- [ ] RED→구현→GREEN(host powertrain_autonomy + powertrain_sim 폐루프)→커밋
  `feat: recovery dwell requires ticks AND min-elapsed AND min-samples`

---

### Task 6: 문서·3환경·젯슨 실기 — 리뷰어 주도

- [ ] 핸드오프 §2 A3 행+기준선, CLAUDE.md(§chassis·§WP8 줄 갱신 — enforcement
  기본 off), 스펙 §4와 대조 확인.
- [ ] 3환경 green(폐루프 앵커 갱신 포함).
- [ ] **젯슨 실기(비회전)**: parity + FAKE chassis(`section_enforcement:=true`·
  `authority_enabled:=true`) + fake `/section/state` 발행 프로브(process-group
  kill)로 ①hold_hint 시 authority state가 hold 반영 ②stale(발행 중단 0.6 s+)
  fail-close ③speed_hint 캡이 `/wheel_states` 지령에 반영 — rclpy 관찰.
- [ ] 커밋 `docs: A3 chain` + push + 젯슨 pull.

## 완료 기준

- SectionState versioned + 집행 fail-close 9계약, 마커 2단계 7계약, 큐 5계약,
  dwell 4계약 — 전부 테스트. 기존 의미론 회귀 0(플로어 폐지 상태에서 hint 캡이
  즉시 유효, 휴면 가드는 플로어>0일 때만).
- 신호 20점 실패조건(§스펙: 힌트 미집행) 폐쇄 — enforcement on 경로 실증(FAKE).
