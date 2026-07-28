# WP6 실코스 자율주행 개발 계획 (PK-3A course.stl)

- **작성**: 2026-07-28
- **목표**: v2 로버가 실제 대회 코스(`course.stl`)를 **자율주행으로 완주**한다.
- **근거**: 2026-07-28 실코스 주행성 규명(`power-train-sim/docs/reports/2026-07-28-v2-rover-real-course-driveability.md`).
  블로커를 순차로 실증·규명했고, 본 계획은 그 블로커들을 구조적 개발 태스크로 전환한다.
- **성격**: 반응적 box-hack이 아니라 **소관별·단계별 개발 계획**. 각 단계는 독립 검증 가능.

## 0-bis. 2026-07-28 후속 규명 — 아래 §0 의 충돌 서술을 대체한다

측정 근거로 블로커가 재정의됐다. §0 이하의 상충하는 진단(특히 "비대칭 우측 에지 검출 버그",
"혼합셀 roughness 계단")은 **오진으로 확인**됐으므로 이력으로만 읽는다.

- **★근본원인 = 하네스 스폰 12.05 cm 편심.** as-built v2 URDF 에서 앞·중·뒤 바퀴쌍 중점이 전부
  정확히 +0.1200 → **base_link 가 로버 중심선에서 벗어남**. 시뮬은 이를
  `m3a.CERTIFIED_ROVER_CENTRE_LOCAL_X_M = 0.1205` 로 보정하며 **m3a 헬퍼 `centred_rover_root_x()`
  와 m4 `initial_robot_spawn_pose`(m4_campaign.py:1357)는 적용**하는데, 이를 오버라이드한
  **m5 만 누락**했다. ⇒ 모든 실코스 런이 편측 여유 6 cm 코스에서 12 cm 치우쳐 스폰됐다.
  8패밀리(m4)는 보정을 적용하므로 무관·회귀 없음.
- **estimator 무죄.** 로버 중심 기준 실제 능선 `[-0.330, +0.571]` vs 추정 corridor `[-0.350, +0.500]`
  — **양쪽 다 보수적이고 편향 없음**. 중심정렬만으로 `erosion_empty` 999/1000 → **14/1000(1.4%)**.
- **valley 누출의 실제 기전**: 파도 valley 에서 주행면이 바닥 위 **8.4 cm** 뿐 →
  `max_support_step 0.12` 미만이라 flood-fill 이 아레나 바닥으로 샌다. 바닥이 support 가 되면
  `_local_lower_floor_mask` 의 `~support_mask` 요건 때문에 **crest 의 0.39 m 진짜 절벽까지 전역
  미검출**된다(국소 누출의 전역 증폭). 누출 셀 roughness 는 전부 0.00 → **roughness-gate 가설 기각**.
- **실측 기하(로컬 STL ray-cast)**: 능선 폭 median **0.910 m**, 로버 0.789 m → 편측 실여유 6.05 cm.
  요구 corridor = 2×(0.3945+0.05) = **0.889 m** → **총 여유 2.1 cm**. estimator 는 corridor 를
  약 1격자(5 cm) 좁게 본다 ⇒ 마지막 관문은 **sub-cell 에지 정밀화**.
- **pitch**: 축거 현경사 max 14.24° 이지만 **실측 차체 pitch 16.2°** > `EMPTY_STOWED` 15° 한계.
  상향은 안전 판단이라 사용자 결정 사항.
- **아레나 바닥 깊이 미해결**: "옆은 진짜 낭떠러지" 확정과 현행 z≈0 fixture 가 배치된다.
- **커밋**: `3df0114` production `kinematics.py` 횡방향 기하 → as-built v2(편측 8 cm 과대 수정).
  부수효과로 제자리 피벗 충실도 실제 악화(ω 복원 −1.10%→−3.91%) — 실기 벤치 재확인 권장.

## 0. 요약 (현 시점 정직한 상태 — 0-bis 로 일부 대체됨)

- **자율주행 loop 자체는 작동한다**: 무수정 프로덕션 depth→TerrainEstimator→controller가 실코스에서
  지형을 수용(100%)하고 TRACKING(주행)한다 — 단, 아래 블로커들이 순차로 완주를 막는다.
- **실증된 진전**: 100% 얼음(frozen) → 지형 수용·추종·전진 0.47 m. 근본 파이프라인은 건강.
- **막는 것**: 코스가 진짜 어려운 장애물 코스(좁은 통로 0.83~2.1 m · 램프 · 벽 · 헤어핀 · undulating)라서
  블로커가 층층이 있다. 각 층은 아래 Phase로 분해된다.
- **핵심 판단**: 완주 가능성은 "자율 무능"이 아니라 ①코스/로봇 모델 정합 ②perception 보정
  ③controller 튜닝 ④통합의 문제. 단 최협 핀치(0.83 m vs 로봇 0.79 m = 편측 2 cm)는 근본 리스크로 남는다.

## 1. 범위 · 성공기준 · 비범위

- **성공기준(단계적)**:
  - S1: 아레나 바닥 모델 하에 넓은 구간(통로 ≥1.0 m·완만 램프)에서 **연속 자율주행 ≥0.3 m/s**.
  - S2: 코스 1/3(한 직선+한 완만 코너) 무정지 자율 완주.
  - S3: 전 코스 자율 완주(핀치·헤어핀·램프 포함), 이탈 0·전복 0.
- **비범위**: 로봇팔 조작(별 트랙), teleop(별 방식), 코스 물리 확폭(기구 소관 — 단 요구는 제기).

## 2. 검증 방법론 (모든 Phase 공통)

- **하니스**: `power-train-sim/m5_real_course.py`(무수정 프로덕션 폐루프 재사용, box Isaac 5.1).
  스폰 실측 수정·아레나 바닥 모델 포함(이미 반영, 미커밋).
- **지표**: `distance_m`·`avg_speed`·`controller_states{TRACKING/HOLD}`·`estimator acceptance_rate`·
  `reject_reasons`·`hold_reasons`·`departed`·`max_pitch/roll`. 리포트의 계량 표와 동일 포맷.
- **회귀 안전바**: 전 변경에서 `departed=0`·전복 0·낙하 이탈 0 유지. 완화 시 음성대조로 재확인.
- **⚠️ 프로덕션 안전코드(estimator 마진·controller clearance_hold) 수정은 하네스 classifier가 차단** →
  해당 실험은 사용자 `!` 실행 또는 perception/controller 소관 정식 변경으로 진행(급조 patch 금지).

## 3. Phase별 개발 (블로커 스택 → 태스크)

### Phase 1 — 코스·로봇 모델 정합 (크로스팀, 최우선·최대효과)

블로커: `drop_boundaries_unobserved`(course.stl 아레나 바닥 누락) + stale geometry.

- **T1.1 [✅ 사용자 확정 2026-07-28] 아레나 바닥 = sim 스탠드인 그대로 사용.** m5 default-on 슬래브
  (base−0.20 m, `a98ad53` 커밋)로 확정 운용. dohyun의 course.stl 수정 대기 불필요. (실증: 게이트 A가
  전 코스에서 열림.)
- **T1.2 [✅ 사용자 확정: v2 CAD = as-built 정본] v2 geometry 반영.** footprint lateral 확정값 mid ±0.360
  (외곽 0.79 m; front ±0.273·rear ±0.213). forward(x)는 v2 wheelbase가 구값과 동일(0.875 m)이라 구
  ±0.4377/−0.0603 유지가 정확. **남은 실행**: 프로덕션 `default_geometry()`에 넣을 정밀값은 v2 URDF
  `2026_07_24_URDF.urdf` FK 추출로 확정(내 Isaac 측정은 base_link 재센터링 ~19 mm 오차 → 하드코딩 금지).
  ⚠️실물 4WS 애커만 제어도 이 track을 공유하므로(구 0.4395로 10모터 HIL 검증 이력) **트랙 축소 반영 시
  4WS HIL 재검증 권고**. **완료조건**: default_geometry == v2 URDF 실측 + 4WS 재검증.
- **T1.3 [기구/코스, R1 게이트] 최협 통로 실측 확정**: `course.stl` 0.83 m vs 로봇 0.79 m(편측 2 cm).
  이 핀치가 최종인지, 확폭 가능한지 확인. **완료조건**: 핀치 폭 확정값 + 확폭 가부 결정(S3 완주 전제).

### Phase 2 — Perception 보정 (perception 소관)

블로커: `erosion_empty`(과보수 footprint 마진) + 드롭/노치 관측한계.

- **T2.1** `footprint_uncertainty_m`(현재 0.05 임의값)를 **실 L515 depth + 오도메트리 노이즈 실측**으로
  재보정. 그리드 양자화·오도 드리프트 기반 원리값 도출(0 hack 금지, 실질 버퍼 유지). **완료조건**:
  마진 근거 문서 + 넓은 구간 erosion 통과.
- **T2.2** 드롭경계 게이트를 실코스 전 지형(램프정점 crest·노치·핀치)에서 검증. undulating crest 관측한계
  (07-22)와 실코스 험프의 관계 재확인(실코스 험프는 benign 판정 이력 있음 — sim-6m-wall 메모 참조).
  **완료조건**: 각 지형 유형별 수용/거부 근거표.

### Phase 3 — Controller 튜닝 (controller 소관)

블로커: `clearance_slow`/`curvature_slow`(좁은 통로 크롤) + `pitch_limit`(램프/junction).

- **T3.1** 속도 프로파일(`controller/core.py` `clearance_full_m`·`curvature_slow_k`·프로파일 `max_speed`)을
  실코스 통로폭 분포에 맞게 튜닝. **안전 정지 임계 `clearance_hold_m`(에지 정지 마진)는 데이터 근거 없이
  낮추지 말 것.** **완료조건**: 넓은 구간 ≥0.3 m/s·좁은 구간 저속주행·에지 정지 유지(S1).
- **T3.2 [규명완료] 램프/pitch**: `core.py:399` 로봇 pitch > 프로파일 `max_slope_rad`(15°)면 하드
  `pitch_limit` 정지. 실측 코스 램프 grade **max 16.8°**(median 9.5·p90 14.4) — **램프가 15° 초과**.
  로커보기는 물리적으로 16.8°+를 넘음(v4가 15°/30° 경사 최적화). 15°는 프로파일 "PROVISIONAL" 임시값.
  **수정: `profiles.py` EMPTY_STOWED `max_slope_rad` 15°→~22°(램프16.8+마진, 30° 설계한계 내)·
  `soft_slope_rad` 상향.** ⚠️좁은 도로서 바퀴가 아레나 바닥(스탠드인 −0.20 m)으로 빠져 기우는 tilt와
  램프 pitch를 구분해야 함(아레나 바닥 깊이 vs 게이트A 드롭요건 vs tilt 안전의 tension). **완료조건**:
  램프 무정지 통과 + 이탈/전복 0.
- **T3.3 [규명완료·프론티어] 코너 선회 항법**: Phase 3 튜닝(pitch 포함) 후 v2가 직선+램프
  ~0.45 m 자율주행해 **첫 코너 벽에 도달** → 코리도어가 꺾이는데 직진 heading만 따라 벽 정면 정지
  (`obstacle_blocks_path`). 로컬 추정 lookahead(0.3~4 m)로 급코너를 조향해 따라가는 능력이 필요 —
  `max_yaw_rate`·애커만 클램프·경로추종(heading_error/path_offset 게인)·코너 예측. **파라미터 아니라
  실질 자율 능력 갭.** **완료조건**: 첫 코너 무정지 선회 통과(S2 관문).

### Phase 4 — 장애물·통합·완주

- **T4.1** `obstacle_blocks_path`(벽) 처리: 통로 경계 벽을 장애물로 정지 vs 경로 경계로 추종하는 로직 검증.
- **T4.2** 전 코스 통합 주행: 실제 시작점 스폰(현 스폰은 규명용 지점 — 대회 시작점으로 교체) →
  서펀타인 전 구간 자율 주행, `completion` 측정. **완료조건**: S2→S3 단계적.

### Phase 5 — HIL · 대회 준비

- **T5.1** 실물 로버 HIL: 물리 테스트 코스 구간에서 자율 주행 검증(sim↔실차 갭).
- **T5.2** 열화/폴백: 자율 불가 구간의 안전 처리(정지·경보·권한 이양) — 대회 리스크 관리.

## 4. 의존 순서 · 결정 게이트

```
T1.1(아레나바닥) ─┐
T1.2(geometry)  ─┼─→ Phase2(perception 보정) ─→ Phase3(controller) ─→ Phase4(통합/완주) ─→ Phase5(HIL)
T1.3(핀치 실측)  ─┘                                    │
                                                       └─ 게이트 G1: 완주 가능성 판정
```

- **게이트 G1 (Phase 3 후)**: 넓은 구간+램프+한 코너가 자율로 되면 → 전 코스 완주 추진(Phase 4).
  안 되면 → 병목 지형(핀치/헤어핀) 재설계 or 코스 확폭 요구(T1.3) or 부분자율+구간별 처리 결정.
- **게이트 G2 (Phase 4 후)**: 대회 일정 내 전 코스 완주 실현성 판정. 미달 시 우선순위 재조정.

## 5. 리스크

- **R1 (높음) 최협 핀치 0.83 m**: 로봇 0.79 m라 편측 2 cm. erosion 마진 0 이어도 통로 ~0.11 m로 자율
  경로계획 확신 어려움(실증). **코스 확폭(T1.3) 없으면 S3 완주 불투명** — 근본 리스크.
- **R2 (중) undulating + 램프 pitch**: 험프/램프에서 지지·pitch 게이트. 로커보기 물리력 vs 컨트롤러 보수성.
- **R3 (중) sim↔실차 갭**: sim 아레나바닥·클린 depth 가정. 실 L515 노이즈·실 아레나 바닥 형상에서 재검증 필요(Phase 5).
- **R4 (조직) 크로스팀 의존**: T1.1(dohyun)·T1.2/T1.3(기구) 미완이면 Phase 2+ 차단.

## 6. 일정 프레이밍 (사용자 확정 필요)

- 대회: 국방 9월 · 극한 10월(메모리 기준). **본 계획은 전 코스 자율 완주 목표이나, Phase별 게이트에서
  실현성을 재판정**한다. 최소 목표(S1: 넓은구간 자율)와 완주 목표(S3) 중 어디를 대회 커밋 라인으로 둘지,
  자율 불가 구간의 폴백을 무엇으로 할지는 **사용자·팀 결정 사항**.

## 7. 즉시 착수 가능 (크로스팀 대기 없이)

- ✅ m5 스폰 수정·아레나 바닥 스탠드인(미커밋) 커밋 → Phase 검증 기반 확정.
- T1.1 dohyun 요청(course.stl 아레나 바닥) 발송.
- T1.2/T1.3 기구팀에 as-built 트랙·핀치 실측 요청.
- Phase 2/3는 위 3개 인계 확정 후 perception/controller 소관에서 구조적으로(정식 config·근거문서).
