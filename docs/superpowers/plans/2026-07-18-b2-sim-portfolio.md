# B2 배치 구현 계획 — 시뮬 M/L급 (마찰·스모그 열화·동적 선도 표적·hidden 캠페인)

> **For agentic workers:** superpowers:subagent-driven-development 또는 executing-plans.
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 3환경(+pipefail) + 젯슨 parity(시뮬은 dev 컨테이너 전용 — 실기 표면 없음).

**Goal:** 스펙 r6 §5 B2 — ⑤물리 마찰 fixture 계약화 + `closed_loop` diagnostics
배선(C1 WP9 시뮬 검증 기반) ⑥스모그 depth 열화 fault 가족 ⑦**동적 선도 표적
폐루프**(5구간 추종 배점의 유일한 plant-급 검증 경로) ⑧hidden-seed 캠페인 러너.

**Spec:** r6 §5 B2. 기준선(B1 후): 호스트 278 / dev 1107+2skip / ros 480 / 젯슨 480.

## 확인된 확장점

- 마찰: per-station `friction_coefficient` **이미 물리 μ**(`model_builder.py:123-126`).
- diagnostics: `closed_loop.py:183` `diagnostics=None` 하드코딩 — 시뮬에 지령
  휠속(`sensors.py:50-88` `command_turns_per_s`)·ground truth 실존, production
  `StateEstimator.diagnostics`(slip/stuck/terrain_speed_cap) 사용 가능.
- fault 스키마: `scenario.py:462-524` — 4그룹 고정(`_no_unknown`), `depth_degradation` 신설 필요.
- perception 훅: `runner.py:298-306` — `detections_source` 없음, 센서 루프 `:320-360`.
- follow 코어: `motor_control/chassis/follow.py`(target 2.0 m·band 1.5~2.5·min 1.5·TRACKING/PREDICTING/REACQUIRING/LOST).

## Global Constraints

- 이전 배치 승계 + **additive**(기존 가족·앵커 0.709/0.20 불변, RNG draw 순서 보존).
- 수치 수용 기준은 스펙 §5의 초기값 — dev seed 실측으로 보정 시 사유 주석.
- 젯슨 = repo 동기 + ros parity만.

---

### Task 1: 마찰 patch fixture 계약화 + closed_loop diagnostics 배선

**Files:**
- Modify: `powertrain_sim/procedural.py` (`FrictionPatchSpec(center_ratio, length_m, mu)` — additive, 기존 friction_range draw 순서 보존), `powertrain_sim/closed_loop.py` (`diagnostics=None` 해제)
- Test: `powertrain_sim/tests/test_procedural.py`·`test_closed_loop.py` 추가

**Interfaces:**
- `TerrainAutonomyDriver.command()`가 production `StateEstimator` snapshot의
  `DriveDiagnostics(stamp_s, slip_candidate, stuck_candidate, speed_cap_m_s)`를
  구성해 `controller.decide(..., diagnostics=...)`로 전달(§: `state_estimation.py`
  의 diagnostics 필드 재사용 — 발명 금지, 추정기가 이미 계산하는 값만).
- 수용(초기): μ=0.3 patch dev seed에서 **slip_candidate 검출률 ≥80%**(patch 구간
  내 tick 기준), patch 밖 오검출 ≤5%, fail_open 0. 검출률 미달 시 추정기 임계를
  바꾸지 말고 **실측치로 기준 보정 + 사유 주석**(추정기는 production 불변).
- diagnostics 배선으로 기존 dev-seed 앵커가 변하면(stuck hold 등) 사유 주석과
  함께 갱신하되 passed/fail_open 불변 전제 — 변동 시 보고.
- [ ] RED→구현→GREEN→커밋 `feat(sim): friction patch fixtures + closed-loop DriveDiagnostics wiring`

---

### Task 2: 스모그 depth 열화 fault 가족

**Files:**
- Modify: `powertrain_sim/scenario.py` (`depth_degradation` 그룹: `(start_s, end_s, dropout_ratio_start, dropout_ratio_end, noise_std_m)` — 램프), `powertrain_sim/mujoco_fast/runner.py` (depth 샘플에 적용: 비율만큼 무효 픽셀 + 가우시안 노이즈, seed 결정론), `powertrain_sim/procedural.py` `_faults`(stress 가족에서 생성 가능)
- Test: `test_scenario.py`·`test_mujoco_fast.py`·`test_closed_loop.py` 추가

**Interfaces:**
- 결정론: 시나리오 seed에서 파생한 RNG로 픽셀 마스크·노이즈(재실행 동일).
- 수용(초기): 열화 램프(0→60% dropout) dev seed에서 fail_open 0, 열화 심화 구간
  CONTROLLED_HOLD 진입(low_confidence/terrain 사유), 램프 해제 후 dwell 준수
  복귀. C1 스모그 정책의 시뮬 기반.
- [ ] RED→구현→GREEN→커밋 `feat(sim): depth_degradation fault ramp (smog surrogate)`

---

### Task 3: 동적 선도 표적 폐루프 (L)

**Files:**
- Create: `powertrain_sim/lead_target.py` (표적 plant + 상대 관측 합성), `powertrain_sim/follow_loop.py` (FollowDriver — follow.py 코어 소비)
- Modify: `powertrain_sim/mujoco_fast/runner.py` (`detections_source` 훅 — 센서 루프에서 호출, `recording`에 lead 채널 추가)
- Test: `powertrain_sim/tests/test_lead_target.py` (신규) + `test_closed_loop.py` follow 케이스

**Interfaces:**
- `LeadTargetSpec(path="straight|curve", speed_m_s=0.5, occlusions=((start_s,end_s),...), dropout_ratio=0.0)`;
  `LeadTargetPlant.pose(t)` — 결정론 궤적(트랙 centerline 선행 오프셋).
- `detections_source(t, robot_pose) -> list[detection]` — follow.py 입력 계약
  `(class_name, confidence, forward_m, left_m, bbox_area_px)` 5-튜플로 상대
  좌표 합성(가림 구간엔 빈 리스트, dropout 확률 seed 결정론), bbox_area는
  거리 반비례 근사(주석).
- `FollowDriver` — `FollowController`(follow.py)를 tick마다 구동해 `(v, ω)`
  반환(runner의 `command_source`로 주입), `hold_state_source`는 follow 상태
  (LOST→hold)로. **틱 순서**: 관측 합성 → follow 결정 → plant step(주석 명문화).
- recording: `lead_distance_m`·`follow_state` 채널 추가(스키마 additive).
- 수용(스펙 초기): 직선 0.5 m/s 60 s — 간격 2.0±0.5 m 체류율 ≥90%·min 1.5 m
  침범 0·fail_open 0; 5 s 가림 → 재획득 ≤3 s; 곡선 경로 1케이스 완주.
- [ ] RED(합성 관측 결정론·기하 검증) → plant/driver 구현 → 폐루프 3케이스 →
  GREEN → 커밋 `feat(sim): dynamic lead-target closed loop via detections_source hook`

---

### Task 4: hidden-seed 캠페인 러너

**Files:**
- Create: `powertrain_sim/campaign.py` + `python -m powertrain_sim.campaign`
- Test: `powertrain_sim/tests/test_campaign.py`

**Interfaces:**
- 입력: 가족 목록(flat/bank/pinch/clothoid/undulating/friction/smog/follow) ×
  시드 클래스(dev=고정 목록·regression=manifest·hidden=수치 시드 인자) 매트릭스.
- 실행: 가족별 표준 생성 파라미터로 `run_closed_loop`(follow 가족은 FollowDriver)
  순차 실행, 결과 표(JSON+표준출력): passed/completion/fail_open/recovery.
- **hidden 정책**: hidden 클래스는 scenario 본문 미저장 — `canonical_json_sha256`
  해시와 메트릭만 기록(기존 hidden_eval 관례).
- 수용: dev 매트릭스 전 가족 1시드 실행 exit 0(테스트는 2가족 축소 매트릭스로
  결정론·리포트 스키마 검증 — 전체 실행은 캠페인 CLI 몫).
- [ ] RED→구현→GREEN→커밋 `feat(sim): family x seed campaign runner with hidden-hash policy`

---

### Task 5: 문서·3환경·젯슨 parity — 리뷰어 주도

- [ ] 핸드오프 §2 B2 행+기준선, `powertrain_sim/README.md` 갱신(가족·훅·캠페인).
- [ ] dev 컨테이너에서 캠페인 dev 매트릭스 1회 실행(전 가족 스모크) — 결과 기록.
- [ ] 3환경 green + 젯슨 repo 동기·parity.
- [ ] 커밋 `docs: B2 chain` + push + 젯슨 pull.

## 완료 기준

- 5구간 추종 배점의 plant-급 검증 경로 확보(간격·재추종·침범 0 수치 통과).
- C1 기반(diagnostics 배선·스모그 가족) 완성. 기존 앵커 불변(변동 시 정직 갱신).
- 캠페인 러너로 가족 전체 일괄 실행 가능(hidden 해시 정책 포함).
