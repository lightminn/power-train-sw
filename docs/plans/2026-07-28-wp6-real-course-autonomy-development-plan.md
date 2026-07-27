# WP6 실코스 자율주행 개발 계획 (PK-3A course.stl)

- **작성**: 2026-07-28
- **목표**: v2 로버가 실제 대회 코스(`course.stl`)를 **자율주행으로 완주**한다.
- **근거**: 2026-07-28 실코스 주행성 규명(`power-train-sim/docs/reports/2026-07-28-v2-rover-real-course-driveability.md`).
  블로커를 순차로 실증·규명했고, 본 계획은 그 블로커들을 구조적 개발 태스크로 전환한다.
- **성격**: 반응적 box-hack이 아니라 **소관별·단계별 개발 계획**. 각 단계는 독립 검증 가능.

## 0. 요약 (현 시점 정직한 상태)

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

- **T1.1 [dohyun/코스]** `course.stl`에 **도로 아래 아레나 바닥** 모델링. 도로면보다 >0.18 m 낮게,
  카메라 FOV·6 m 사거리 안, 측당 ≥8 관측점, 지지 에지 FOV한계 안쪽 75 mm. (실증: 이것만으로 게이트 A가
  전 코스에서 열림. sim 임시 스탠드인 = m5 `M5_ARENA_FLOOR_Z_M`/default −0.20.) **완료조건**: m5에서
  전 코스 `drop_boundaries_unobserved` 소멸.
- **T1.2 [기구+SW]** `motor_control/chassis/kinematics.py` `default_geometry()`를 **as-built v2 트랙**으로
  갱신(현재 stale 0.949 m → 실측 0.79 m; mid ±0.360 등). ⚠️실물 4WS 제어에도 영향 → as-built 실측 확인
  후. **완료조건**: 추정 footprint == 실측, erosion이 실제 폭 반영.
- **T1.3 [기구/코스]** 최협 통로 실측 확정: `course.stl` 0.83 m vs 로봇 0.79 m(편측 2 cm). 이 핀치가
  최종인지, 확폭 가능한지 확인. **완료조건**: 핀치 폭 확정값 + 확폭 가부 결정(S3 완주 가능성의 전제).

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
- **T3.2** 램프/pitch 처리: 실코스 램프에서 `pitch_limit`(현 15°) 발동 규명 — 램프 경사 실측 vs 프로파일
  `max_slope_rad`/`soft_slope_rad`. 로커보기는 물리적으로 램프를 넘으나 컨트롤러 pitch 게이트가 막는지,
  아니면 좁은 도로서 바퀴가 아레나 바닥으로 빠져 기우는지 분리. **완료조건**: 램프 무정지 통과.
- **T3.3** 헤어핀/급선회: 서펀타인 급코너에서 `max_yaw_rate`·애커만 클램프·경로추종 게인 검증.
  **완료조건**: 한 코너 무정지 선회(S2).

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
