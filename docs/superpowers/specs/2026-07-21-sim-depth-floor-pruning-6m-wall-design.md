# 6 m 벽 근본 원인 확정 + 시뮬 depth 렌더러 수정 설계 (2026-07-21)

> 한 문장: **"6 m 데드락"의 근본 원인은 production 지형 추정기가 아니라
> powertrain_sim depth 렌더러다 — MuJoCo `mj_multiRay` 가 cutoff 프루닝을 geom
> 앵커점 거리로 수행해, 카메라가 월드 원점(바닥 plane 의 `pos`)에서 6 m 를
> 벗어나는 순간 무한 바닥 plane 전체를 후보에서 제외한다.** 바닥이 depth 에서
> 통째로 사라지니 낙하 증거가 소실되고, 추정기는 설계대로 fail-closed hold 한다.
> 수정은 시뮬 센서 한 곳(컷오프 비프루닝화)이며 production 추정기는 손대지 않는다.

배경 정본: `docs/reports/2026-07-21-sim-fidelity-completion-progress.md` §3·§3-B,
실패한 1차 시도 `docs/superpowers/specs/2026-07-21-terrain-corridor-carry-deadlock-design.md`.
이 문서는 그 두 문서의 **원인 진단(§3-B "품질 타일 게이팅 / 단일프레임 사거리
한계")을 실측으로 반증·대체**한다.

## 1. 재조사 실측 — 기존 진단의 반증

집중 과제 프롬프트의 "확정 메커니즘"을 신뢰하되, 설계 전에 게이트 지점을
정밀 계측한 결과 핵심 전제 두 개가 실측과 어긋났다(전부 flat 15 m dev seed 0,
frame = depth 샘플 인덱스):

1. **"far 타일이 품질 게이트로 통째 배제" — 반증.** 프레임 0(정지 단일 시점)에서
   hard-reject 타일 0개. 타일 사유는 전부 soft(`low_valid_ratio` 등, 픽셀 배제
   없음)이고 단일프레임 **유효 셀은 전방 3.93 m 까지 존재**한다.
2. **"확인에는 support 가 ~4 m 필요, 단일프레임 2.3 m 로는 불가" — 반증.**
   프레임 0은 support 최원 2.32 m 상태에서 낙하 경계 확인에 성공,
   `path_available=True`. 실제-에지 인정은 x≈1.6 m 행부터 시작된다(FOV 한계가
   에지+margin 을 넘는 지점). 단일프레임 support 가 2.3 m 에서 끊기는 것 자체는
   사실이나(그레이징 각 샘플링 희소로 5 cm 행이 ~2.4 m 부터 비고 4-연결 성장이
   끊김) **6 m 벽의 원인이 아니다**.
3. **실제 트리거: 측면 바닥의 물리적 소실.** 프레임 85(x=5.35)→86(x=5.41) 사이
   측면 바닥 픽셀 812개가 전 열에 걸쳐 **정확히 0**이 되고, 이후 영구히
   돌아오지 않는다(floor_hits 797→0). `_point_side_evidence` 의 기준높이−0.18 m
   아래 점이 ~800→**0** — 8개 미만이 아니라 완전 소실. 증거가 없으니
   `drop_boundaries_unobserved` 는 **올바른 fail-closed** 다.

## 2. 근본 원인 — MuJoCo `mj_multiRay` plane 프루닝 결함

`powertrain_sim/mujoco_fast/sensors.py` `_ray_depth_m` 은
`mj_multiRay(..., cutoff=MAX_VALID_DEPTH_M=6.0)` 으로 depth 를 만든다. 실측:

- 소실 시점의 레이를 같은 모델에서 재캐스트하면, 해석적 바닥 히트가
  **3.3~5.9 m(컷오프 이내)** 인 레이 768개가 `mj_multiRay` 에서 NO_HIT.
  같은 레이를 `mj_ray` 로 쏘면 정확히 해석값 거리에서 `lower_floor` 를 맞춘다.
- flip 지점은 |카메라 − 월드원점| = 6.0 (ray86: 5.93 m 유지, ray87: 6.00 m 소실).
- 최소 재현(MuJoCo 3.10.0): plane(pos 0 0 0) 위 (x, 0, 1) 에서 수직 아래 레이,
  cutoff 6.0 → x=5.90 은 dist 1.00 보고, **x=5.99 부터 NO_HIT** (mj_ray 는 1.00).
  즉 cutoff 프루닝이 geom 앵커점까지의 거리로 수행되고, 무한 plane 은 앵커가
  원점이라 **카메라가 원점에서 cutoff 이상 떨어지면 plane 전체가 제외**된다.

이 하나로 기존 관측 전부가 설명된다: 모든 트랙·시드에서 ~6 m(정확히는 원점
기준 유클리드 6 m, 카메라 높이 0.88 → 전방 ≈5.94 m), 해상도·서스펜션·바퀴반경·
오도메트리 무관, corridor-carry 5 m 게이트에서 10.5 m(맹목 creep 소진뿐, 증거는
영영 안 돌아옴), hold 후 자기잠금(융합 시간만료는 증상이지 원인 아님).

**실기 함의(중요)**: 실차 L515 에는 이 프루닝이 없다. "실차도 6 m 마다 멈출 수
있는 결함"이라는 우려는 **이 메커니즘에 관한 한 해당 없음**. production
추정기·안전 논리는 이 사건에서 결함이 발견되지 않았다.

## 3. 수정 설계

### 채택 — A. 레이캐스트 컷오프 비프루닝화 (sensors.py 한 곳)

`mj_multiRay` 에 넘기는 cutoff 를 센서 사거리와 분리한다:

```python
# MuJoCo mj_multiRay 는 cutoff 를 "geom 앵커점 거리" 로 프루닝한다. 무한
# plane(lower_floor) 은 앵커가 월드 원점이라, 카메라가 원점에서 cutoff 이상
# 멀어지면 plane 전체가 레이 후보에서 빠져 바닥이 depth 에서 소실된다
# (3.10.0 실측, docs/superpowers/specs/2026-07-21-...-6m-wall-design.md §2).
# 컷오프는 프루닝 없는 값으로 두고, 센서 사거리는 아래 hit 마스크가 정의한다.
RAY_PRUNING_CUTOFF_M = 1.0e6
...
mujoco.mj_multiRay(..., ray_count, RAY_PRUNING_CUTOFF_M)
hit = (distances >= 0.0) & (distances <= MAX_VALID_DEPTH_M)   # 기존 그대로
```

- 사거리 의미론은 기존과 동일: 6 m 초과 히트는 지금도 `hit` 마스크가 0 으로
  만든다(컷오프를 넘겨 보고되던 히트가 이미 있었고 동일하게 마스킹돼 왔다).
- 수정이 **바꾸는 것은 "프루닝으로 부당하게 누락되던 히트의 복원" 뿐**이다.
  유효 depth 를 지우는 경로는 없다. (2026-07-21 리뷰 보정: 앵커가 6 m 밖인
  가림 geom 이 복원되면 기존 픽셀이 *더 가까운* 올바른 값으로 바뀔 수는 있다 —
  mj_ray ground-truth 와 일치하는 방향이며 V3 등가성 테스트가 이를 검증한다.)
- 실측 성능: flat 15 m 40 s 폐루프 41 s — 기존과 동급(프루닝 이득 무의미).

### 기각 대안

- **B. plane 을 매 스텝 카메라 아래로 재앵커**: 물리 모델을 런타임 변경(오염
  위험), 트랙 끝 box 앵커 등 다른 잠재 사례를 못 덮음.
- **C. plane → 대형 box 치환**: 모델·충돌 여백 변경, 근본 교훈(컷오프 프루닝
  불신) 미반영.
- **D. 추정기 far-field 게이팅 완화(원래 과제 방향)**: §1 반증으로 불필요.
  안전 최민감 코드를 무근거로 완화하게 되므로 **하지 않는다**.

## 4. 안전 논거 + 음성 대조 (필수 검증)

추정기(fail_open 방어선)는 변경하지 않는다. 시뮬 수정의 위험은 "가짜 depth 로
위험을 가리는 것"이며 아래로 봉인한다:

- **V1 결함 재현 회귀(게이트 자체 증명)**: 카메라를 원점에서 6 m 초과 지점에
  둔 최소 장면에서 (a) 기존 컷오프(6.0)로는 컷오프-이내 바닥 히트가 누락됨을,
  (b) 수정 컷오프로는 `mj_ray` 와 동일 거리로 복원됨을 단언. MuJoCo 업그레이드로
  프루닝 의미론이 바뀌어도 여기서 잡힌다.
- **V2 사거리 마스크 불변**: 수정 후에도 depth 프레임에 6.0 m 초과 유효값이
  나타나지 않음을 단언(센서 사거리 확장 금지).
- **V3 서브-6 m 등가성**: 벽 이전 포즈에서 수정 전/후 depth 프레임이 동일함을
  단언(수정은 히트를 복원만 하고 기존 유효값을 바꾸지 않는다). 샘플 레이는
  `mj_ray` 를 ground truth 로 대조.
- **V4 캠페인 절대 게이트**: 전 가족 dev 캠페인 `fail_open_count == 0` **AND**
  `edge_overrun_count == 0`. 하나라도 깨지면 즉시 되돌림. 예외 없음.
- **V5 회귀**: `powertrain_autonomy` 134 passed 불변(코드 무변경이므로 기계적
  확인). `powertrain_sim` 스위트에서 6 m 벽 증상이던
  `too_narrow_pinch`·`clothoid` false_hold 실패의 재평가 — 벽 해소로 green 이
  되는지 확인하고, 완주/false_hold 앵커 변화는 **정직 재기준선**(실측값 명기).
  기존 무관 3건(follow 간격 0.9 mm, three_percent 의도 노출, depth-shape 핀)은
  이 작업 범위 밖(별도 기계적 재핀 대상 유지).

사전 실측(모끼패치 프로브, 참고용 — 정본 수치는 구현 후 캠페인으로 확정):
flat 15 m 40 s 완주율 0.396 → **0.942**, fail_open 0, edge_overrun 0.

## 5. 잔여 이슈 · 비목표

- **비목표: 추정기 far-field/타일 게이팅 변경, corridor-carry 재시도.**
- 단일프레임 support 4-연결이 ~2.4 m 의 빈 행에서 끊기는 현상은 실재한다
  (그레이징 샘플링 희소). 정지 확인은 성공하므로(§1-2) 당장 결함은 아니나,
  실기에서 "긴 hold 후 재확인 강건성" 여지로 **backlog 에만 기록**한다. 손대는
  경우 원거리 노이즈 → 가짜 support → fail-open 위험을 §4 수준의 음성 대조로
  다뤄야 한다.
- flat 0.94 에서 남는 잔여(종단 근처 시간 소진·false_hold 13)는 Task 5 정직
  재기준선의 재료다(별건, 계획 A 재개).
- MuJoCo 업스트림 리포트는 선택 사항(재현 스크립트 보존됨).

## 6. 완료 기준 (과제 원문 대비)

1. 폐루프 flat 15 m 가 6 m 벽을 넘어 완주율 유의미 상승 — 사전 실측 0.94.
2. 전 가족 캠페인 fail_open 0 · edge_overrun 0 (V4) + V1~V3 음성 대조.
3. 추정기 런타임 예산 — 추정기 무변경, 해당 없음. 시뮬 벽시계 동급 확인.
4. powertrain_autonomy green 유지, powertrain_sim 재기준선은 정직 실측으로.
5. 훈련 트랙 15 m 확정 + Task 5 재개는 이 수정 랜딩 후 별건.
