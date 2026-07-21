# 지형 추정기 corridor-carry 데드락 수정 설계 (2026-07-21)

> ⛔ **결과: 실증 실패(불충분). 구현은 되돌림.** 이 문서는 시도·근거의 기록으로
> 보존한다. 상세는 `docs/reports/2026-07-21-sim-fidelity-completion-progress.md`
> §3-B. 요약: 설계대로 구현·게이트 검증(fail_open 0)까지 됐으나, **안전한
> 게이트값(0.75 s / 0.15 m)에서는 데드락을 뚫지 못했다**. 로봇이 안전한 맹목
> creep(0.15 m) 안에 무너진 모션-융합 사거리를 회복하지 못하기 때문이다. 게이트를
> 5 m 로 풀면 10.5 m 까지 가지만 그건 5 m 맹목 주행 = 실장애물 트랙에서 fail-open.
> 근본 원인은 **단일프레임 사거리(~2.3 m)가 낙하경계 확인 거리(~4 m)보다 짧은 것**
> 이며, 카메라 장착·해상도로도 안 풀린다(별도 far-field 게이팅 재설계 필요).

> (원안) 한 문장: **정지 중 방금 확인한 낙하 corridor 를 짧고 좁게 유지해, 로봇이
> 한 발 더 내딛어 모션 융합을 되살리게 한다 — fail-closed 안전은 그대로 두고.**

배경 정본: `docs/reports/2026-07-21-sim-fidelity-completion-progress.md` §3.
대상 코드: `powertrain_autonomy/terrain/estimator.py` (production, 안전 민감).

## 1. 고치려는 결함 — 모션 의존 데드락

`docs/reports/2026-07-21-...progress.md` §3 에서 규명한 근본 원인:

- 낙하 경계 확인에는 연결 support 가 전방 ~4 m 까지 뻗어야 한다(측면 FOV 가 트랙
  에지를 75 mm margin 만큼 넘겨야 함).
- **단일 depth 프레임의 유효 셀은 전방 ~2.3 m 까지만**이다(해상도 2배로도 안 늘어남).
- 주행 중에는 **그리드 융합이 연속 시점을 누적**해 유효/support 영역을 ~4 m 로
  늘려, 딱 확인 가능한 수준을 만든다.
- 그래서 전진이 **연속 모션에 임계 의존**한다. 지속 hold(≥ `history_horizon_s`
  =1.5 s)가 한 번 나면 먼 융합 셀이 시간만료 → support 가 2.3 m 로 붕괴 →
  `drop_boundaries_unobserved` → hold. hold 가 모션을 없애므로 **자기잠금**.
- 서스펜션·해상도·바퀴반경·오도메트리·트랙 기하 전부 배제됨(실측).

증상: 15 m 트랙에서 전 가족이 주행 ~6 m 에서 영구 정지(완주율 ~0.40),
`false_hold_count 1`. **fail_open 은 0**(정지는 안전 방향).

## 2. 수정 원리

`estimator.py:544` 의 `if not (left_observed and right_observed): reject` 지점에서,
프레임이 방금 낙하 경계를 확인하지 못했더라도 **직전에 확인한 corridor 를 캐시에서
꺼내 대입하고 나머지 안전 검사를 그대로 통과**시킨다. 로봇이 한 발 내딛으면 새
시점이 생겨 융합 사거리가 회복되고 즉시 fresh 확인으로 캐시가 갱신된다.

**핵심 안전 논거**: corridor 를 유지해도 아래 검사는 **현재 프레임 데이터로 그대로
실행**된다 — 장애물(`obstacle_blocks_path`, :498), 침식(`erosion_empty`, :564),
국소 choke(inner-floor, :575), 행별 커버리지(:604). 즉 **새 위험은 여전히 잡힌다.**
유지되는 것은 "양쪽 낙하 경계의 측면 위치"뿐이고, 그것도 로봇이 그 corridor 를
확인한 자리에서 **거의 안 움직였을 때만** 물리적으로 유효하다.

## 3. 안전 게이트 (fail-open 방지의 전부)

캐시된 corridor 를 대입하려면 **세 게이트를 모두** 통과해야 한다:

| 게이트 | 조건 | 목적 |
|---|---|---|
| G1 시간 | `now_s − 확인시각 ≤ corridor_carry_horizon_s` (0.75 s) | 긴 맹목 구간 차단(움직이는 장애물 등 세계 변화) |
| G2 이동 | 확인 이후 누적 이동 `≤ corridor_carry_max_distance_m` (0.15 m) | 확인한 자리에서만 유지 — 미관측 영역으로 맹목 전진 금지 |
| G3 존재 | 캐시에 확인된 corridor 가 있음 | 콜드 스타트 유지 금지 |

세 게이트 중 하나라도 실패하면 **원래대로 `drop_boundaries_unobserved` 기각**.

누적 이동은 `update()` 가 매 프레임 `hypot(dx, dy)` 를 더하고, **fresh 확인 때 0
으로 리셋**한다. 따라서 데드락 중 여러 번 유지하더라도 **누적 0.15 m 를 넘으면
강제 재확인** — 맹목 전진은 항상 0.15 m 이내로 유계.

## 4. 유지 프레임의 출력

- `degradation_reasons` 에 **`corridor_carried`** 추가 → 관측 가능.
- confidence 를 **`corridor_carry_confidence_cap` (0.4) 로 상한** → 하류 속도
  스케일이 보수적으로 유지(느리게 전진).
- `path_available=True` 로 corridor 를 내되, 위 §2 안전 검사가 현재 프레임에서
  통과할 때만. 침식·장애물·choke 에 걸리면 여전히 기각.

## 5. 상태 · 설정

`TerrainEstimatorConfig` 신규 필드(기본값):
```
corridor_carry_horizon_s: float = 0.75
corridor_carry_max_distance_m: float = 0.15
corridor_carry_confidence_cap: float = 0.4
```

`TerrainEstimator` 신규 상태:
```
self._confirmed_corridor: tuple[float, float, float] | None = None  # (right_m, left_m, stamp_s)
self._corridor_carry_distance_m: float = 0.0
```

- `update()`: 프레임마다 `self._corridor_carry_distance_m += hypot(dx_m, dy_m)`
  (summarize 호출 전).
- `_summarize()` fresh 성공(양쪽 확인 → corridor 계산 → 유효 estimate 반환) 시:
  `self._confirmed_corridor = (right_corridor, left_corridor, stamp_s)`,
  `self._corridor_carry_distance_m = 0.0`.
- `_reset()`(future/stale 프레임 등): `self._confirmed_corridor = None`,
  `self._corridor_carry_distance_m = 0.0` — 시간 불연속 후 유지 금지.

## 6. 검증 (음성 대조 필수)

**V1 데드락 해소**: 폐루프 flat 15 m 가 주행 6 m 벽을 넘어 완주율 상승.

**V2 안전 불변식 (절대)**: 전 가족 캠페인 `fail_open_count == 0` **AND**
`edge_overrun_count == 0`. 하나라도 깨지면 **즉시 중단·되돌림**.

**V3 이동 게이트 (fail-open 방지 핵심)**: corridor 확인 후 로봇을 0.15 m 초과
이동(odometry 주입)시키면 **유지 거부 → 기각**. 음성 대조: 이 게이트를 끄면
데드락은 풀리지만 로봇이 미관측 영역으로 전진함을 보여 게이트의 필요성 증명.

**V4 시간 게이트**: 확인 후 0.75 s 초과 정지면 **유지 거부 → 기각**.

**V5 콜드 스타트**: 확인 이력 없이 첫 프레임이 `drop_boundaries_unobserved` 면
**유지 없이 기각**(G3).

**V6 실장애물 우선**: 유지 자격이 있어도 corridor 안에 장애물/바닥이 현재
프레임에 있으면 여전히 기각(§2 검사 우선).

**V7 회귀**: `powertrain_autonomy` 134 + `powertrain_sim` 스위트. 유지로 인해
기존 `false_hold`/완주 앵커가 개선되면 정직 재기준선(별건).

## 7. 비목표

- 단일프레임 사거리 자체를 늘리는 것(옵션 a) — 원거리 노이즈 위험, 별도.
- 양쪽 동시확인 요구 완화(옵션 c) — 이미 부분 구현(FOV-잘린 행 상속), 확대 안 함.
- 6 m 트리거 위치의 정확한 규명 — 데드락만 끊으면 실용상 무의미해짐.
- 훈련 트랙 길이(15 m 유지) · 앵커 재기준선 — 이 수정 후 별건(Task 5 재개).

## 8. 위험

| 위험 | 대처 |
|---|---|
| 유지가 fail-open 유발 | G1·G2·G3 + §2 현재프레임 안전검사 전부 유지 + V3 음성대조 |
| 0.15 m 창 안에 실낙하 | 로봇은 solid 확인 자리에서만 유지 → 그 자리 트랙은 solid. 창 밖 낙하는 유계 creep 내 재확인/기각 |
| 유지가 무한 반복(느린 crawl) | 누적 이동 0.15 m 상한 → 강제 재확인 |
| 기존 안전 테스트 회귀 | V2 + V7, 실패 시 되돌림 |
