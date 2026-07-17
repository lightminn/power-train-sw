# B1 배치 구현 계획 — 시뮬 S급 가족 (핀치포인트·클로소이드·기복·fixture 계약화)

> **For agentic workers:** superpowers:subagent-driven-development 또는 executing-plans.
> **레포 관례:** Codex 위임(git 금지 — 커밋은 리뷰어) + 3환경(+pipefail) + 젯슨 parity.

**Goal:** 스펙 r6 §5 B1 — 트랙 영구 부재를 대체하는 시뮬 검증 가족 4종.
실전 조건(폭 협착·곡률 연속 변화·기복)을 폐루프가 덮고, fixture_class 라벨을
실행 가능한 계약으로 만든다.

**Spec:** r6 §5 B1(수치 초기 기준 포함). 기준선(A3 후): 호스트 264 / dev 1093+2skip / ros 480 / 젯슨 480.

## 확인된 확장점 (실현성 검증 완료)

- 폭: per-station `width` 이미 흐름(`procedural.py:318` → `model_builder.py:120-122`).
- 곡률: 상수 `curvature_profile`(`procedural.py:286-302`) — 리스트 구성만 바꾸면 됨.
- 고도: per-station `elevation_m` 상수(`:294-295, :302`) — `_segment_axes` 3-D 지원.
- fixture_class: `scripts/run_autonomy_regression.py` `FIXTURE_CLASSES` 7종 라벨 +
  `tests/fixtures/environment/manifest.yaml` 9엔트리 — 현재 라벨은 검증 없는 문자열.

## Global Constraints

- 이전 배치 Global Constraints 승계. dev 컨테이너(mujoco)가 시뮬 실행 환경 —
  호스트는 mujoco importorskip 관례 유지.
- **결정론**: 생성 파라미터는 seed 기반(`GenerationParameters` 관례), 앵커는
  dev seed 고정 + 사유 주석. `Date.now` 류 금지(기존 관례).
- 기존 dev-seed 앵커(완주 0.709 근방·recovery 0.20 s) 회귀 0 — 신규 가족은
  **기존 flat 가족 기본값을 바꾸지 않는 additive 확장**이어야 한다.
- 젯슨 실기 = repo 동기 + ros parity(시뮬은 dev 컨테이너 전용) — 이 배치의 실기
  표면은 없음(문서에 명기).

---

### Task 1: 핀치포인트 폭 협착 가족

**Files:**
- Modify: `powertrain_sim/procedural.py` (`GenerationParameters.pinch: PinchSpec | None = None` — `PinchSpec(center_ratio, length_m, width_m)`; width_profile 생성 시 적용)
- Test: `powertrain_sim/tests/test_procedural.py` + `tests/test_closed_loop.py`(신규 케이스)

**Interfaces:**
- `PinchSpec` 적용: centerline 스테이션 중 `center_ratio±length/2` 구간의 폭을
  `width_m`로 좁힘(나머지는 기존 랜덤 폭). 스키마/기존 시나리오 불변(additive).
- 폐루프 수용(스펙 초기 기준): 로봇 footprint 폭 `W`(model_builder/geometry에서
  실측해 테스트 상수로 고정 — 구현 시 값 주석) 기준,
  `width_m ≥ W+0.15` 핀치 dev seed → 완주(fail_open 0, 통과),
  `width_m < W+0.05` → **낙하 경계 fail-closed 정지**(통과율 0이 정답 — 기존
  P1 의미론: `expected_completion=False` + 정지 위치가 핀치 앞).
- [ ] RED(생성 테스트: 핀치 구간 폭 반영·경계 밖 불변) → 구현 → 폐루프 2케이스
  → GREEN → 커밋 `feat(sim): pinch-point width family with closed-loop acceptance`

---

### Task 2: 클로소이드(곡률 연속 변화) 가족

**Files:**
- Modify: `powertrain_sim/procedural.py` (`curvature_mode: str = "constant"` — `"clothoid"`면 시작 κ₀→끝 κ₁ 선형 보간, `|dκ/ds| ≤ 0.08` 클램프)
- Test: `powertrain_sim/tests/test_procedural.py` + 폐루프 1케이스

**Interfaces:**
- `curvature_profile`이 스테이션별 선형 변화 리스트가 됨(centerline 적분은 기존
  코드 그대로 — per-station 값 사용 확인, 아니면 적분 루프에서 리스트 인덱싱).
- 폐루프 수용: clothoid dev seed(κ −0.08→+0.08)에서 이탈 없이 완주 또는
  fail-closed 정지(경계 의미론 준수), fail_open 0, 코스 중앙 오프셋 유계.
- [ ] RED→구현→GREEN→커밋 `feat(sim): clothoid curvature family`

---

### Task 3: 기복(undulating) 지형 가족

**Files:**
- Modify: `powertrain_sim/procedural.py` (`TERRAIN_FAMILIES`에 `"undulating"` 추가 — per-station `elevation_m = amp·sin(2πs/λ)`, 기본 amp 0.05 m·λ 2.0 m 파라미터화)
- Test: `powertrain_sim/tests/test_procedural.py` + 폐루프 1케이스

**Interfaces:**
- bank 가족과 독립(`elevation`만 변조, bank 0). 폐루프 수용: undulating dev seed
  완주·fail_open 0·pitch 추정 유한(recording의 pitch 채널 폭주 없음).
- [ ] RED→구현→GREEN→커밋 `feat(sim): undulating elevation family`

---

### Task 4: fixture_class 실행 계약화

**Files:**
- Modify: `scripts/run_autonomy_regression.py` (클래스별 검증 훅), `tests/fixtures/environment/manifest.yaml`(엔트리에 `contract` 필드 — additive)
- Test: `tests/test_fixture_class_contracts.py` (신규)

**Interfaces:**
- `FIXTURE_CLASS_CONTRACTS: dict[str, callable]` — 각 클래스가 "이 fixture가
  실제로 그 조건을 담고 있는지"를 데이터로 검증(예: `fog_smoke`/`depth_hole_jump`
  → depth 채널에 결손/스파이크 비율 임계 이상, `below_floor` → 음의 고도 셀 존재,
  `occlusion`/`lead_occlusion` → 시야 결손 구간 존재, `shadow_backlight`/
  `reflective_surface` → 해당 노이즈 모델 필드 존재). 데이터에 없는 주장 발명
  금지 — 검증 불가능한 클래스는 `contract: "declared-only"`로 명시 강등(라벨
  과장 제거가 목적).
- regression 러너: manifest 로드 시 계약 실행, 위반 → ManifestError(명시 실패).
- [ ] RED→구현→GREEN→커밋 `feat(sim): executable fixture_class contracts (declared-only made explicit)`

---

### Task 5: 문서·3환경·젯슨 parity — 리뷰어 주도

- [ ] 핸드오프 §2 B1 행+기준선, `powertrain_sim/README.md` 가족 표 갱신.
- [ ] 3환경(+pipefail) green — dev 컨테이너가 시뮬 정본.
- [ ] 젯슨: repo 동기 + ros parity(변화 없음 기대). 실기 표면 없음 명기.
- [ ] 커밋 `docs: B1 chain` + push + 젯슨 pull.

## 완료 기준

- 4가족 전부: 생성 결정론 테스트 + 폐루프 수용(fail_open 0·경계 의미론) green.
- fixture 라벨 과장 제거(실행 계약 또는 declared-only 명시).
- 기존 앵커(0.709/0.20 s) 불변.
