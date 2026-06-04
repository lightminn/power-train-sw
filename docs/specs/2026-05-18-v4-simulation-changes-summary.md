# v4 시뮬레이션 개선 요약 (현재 상태 스냅샷)

**작성일**: 2026-05-18
**대상 코드**: `parameter_calc/python_gpu_triangle/`
**기준선**: v3 (`parameter_calc/python_gpu/`) — 동결 유지, 비교용으로만 사용

---

## 1. 개요

v4는 ZETIN 6륜 로커-보기 로봇의 기하 파라미터 최적화 파이프라인이다.
v3 대비 다음의 큰 흐름으로 진화했다.

| 구간 | 핵심 작업 | 비고 |
|------|----------|------|
| Phase 0 | v3→v4 fork (삼각형/사각형 모드 동시 탐색, `brk_v` 추가) | 검색 차원 14→15 |
| Phase 1 | 사다리꼴 속도 프로파일 + 모터 포화 + 동적 TOI + 비대칭 CG + 링크 관성 | 정적 → 동적 |
| Phase 2 | 곡면 ramp + 경사 슬로프 + 지형별 마찰 + 모터 토크-속도 곡선 + 슬립 | 지형 4→7종 |
| Phase 2b | 모터 변경: D6374+5:1 → **BL70200 8" 인휠 BLDC** | 토크/RPM/질량 재교정 |
| Phase 3+ | 연속 토크(RMS), 배터리 전류, 시스템 견인, 적응 샘플링, 패치 폭 | 6개 메트릭 추가 |
| 가속 | N_PTS 160→100, DE workers 환경변수, JIT 워밍업 4-mode 일괄 | 평가당 5x 단축 |
| 검증 | `validate_mujoco.py`, `cross_validate.py`, `analyze_v4_result.py` | 3rd-party 검증기 |
| 인프라 | `watch.sh` (DE 진행 + GPU 상태 통합 모니터링) | 본 실행 가시성 |
| 차세대 | `docs/specs/2026-05-18-v5-architecture-design.md` | v5 시간영역 ODE 설계 |

---

## 2. 파라미터 공간 (15차원)

```
x[0]   rocker_mode     ∈ {1: triangle, 2: frame}
x[1]   bogie_mode      ∈ {1: triangle, 2: frame}
x[2:8] rocker 파라미터   (모드별 의미 다름)
x[8:14] bogie 파라미터  (모드별 의미 다름)
x[14]  brk_v           ∈ [0.345, 0.375] m  ← v4 신규 (브래킷 피벗 → 휠 축 V 오프셋)
```

v3는 `bogie_mode=2`(frame) 고정 + `rocker_mode=2` 고정이었음.
v4는 4가지 모드 조합 (frame-frame / frame-tri / tri-frame / tri-tri)을 단일 DE 실행에서 탐색.
모드 분기는 JAX `lax.switch`로 단일 컴파일에 처리.

---

## 3. 기본 파라미터(`p0`) 변경 사항

`ZETIN_JointOptSearch_v4_gpu.py:59-89`:

```python
p0 = {
    'R_w': 0.100, 'h_body': 0.300,
    'mass': 50,                      # v3: 30  (실 BOM 반영, 배터리 7kg 포함)
    'mu': 0.70, 'g': 9.81, 'obs_h': 0.150,
    # ─── 모터 (v3: D6374 4.95Nm × 5:1 = 21Nm @ wheel) ───
    'gear_ratio': 1.0,               # 인휠 BLDC, 내장 1:5 (사양에 이미 반영)
    'eta_gear': 1.0,
    'motor_tau_peak': 39.0,          # 휠 측 피크 (Nm)
    'motor_tau_cont': 22.0,          # 휠 측 정격
    'omega_no_load_rpm': 240.0,      # 휠 측 무부하 RPM @ 48V
    'V_bus': 48.0,
    'v_min_advisable': 0.14,         # 0.5 km/h
    'Kt_eff': 22.0/9.0,              # ≈ 2.44 Nm/A
    'battery_max_current': 30.0,
    # ─── 질량 / 관성 ───
    'n_wheel_total': 6,
    'm_wheel': 4.5,                  # 휠+허브모터 단위 (v3: 3.5)
    'm_rocker_link': 2.5,            # (v3: 1.5)
    'm_bogie_link': 1.5,             # (v3: 0.8)
    'I_rocker_add': 0.15, 'I_bogie_add': 0.08,
    'e_restitution': 0.3,
    # ─── 운동 프로파일 (Phase 1) ───
    'v_robot': 0.8, 'v_max': 0.8, 'a_lim': 1.5, 'v_max_flat': 2.0,
    'step_thresh': 5.0,
    'phi_r0': 0, 'delta_pb': 0,
    # ─── 비대칭 CG (Phase 1) ───
    'CG_offset': 0.050,              # 전방 50mm 편위
    # ─── 접촉 패치 (Phase 3+ Tier C-2) ───
    'patch_width': 0.030,            # 8" 70mm 폭 타이어
}
p0['h_CG'] = p0['h_body'] * 0.55     # 0.165m
```

---

## 4. 목적함수 가중치(`W`) 및 지형 가중치(`W_terrain`)

`ZETIN_JointOptSearch_v4_gpu.py:109-116`:

```python
W = {
    'tau':   0.12,   # 피크 토크 (Phase 1)
    'imbal': 0.08,   # 좌우 균형
    'stab':  0.18,   # TOI / 들림
    'sn':    0.06,   # 진동
    'fail':  0.10,   # 수렴 실패율
    'sat':   0.12,   # 속도 인식 모터 포화 (Phase 2d)
    'slip':  0.12,   # 휠별 슬립 (Phase 2c)
    'cont':  0.10,   # 연속 정격 토크 RMS (Phase 3+ A-1)
    'batt':  0.06,   # 배터리 전류 한계 (Phase 3+ B-2)
    'stuck': 0.06,   # 시스템 견인 부족 (Phase 3+ B-1)
}
# 합 = 1.00

W_terrain = {
    'stairs':      0.45,
    'wood':        0.12,
    'rough':       0.13,
    'step':        0.10,
    'curved_ramp': 0.10,   # v4 신규 (Phase 2)
    'incline_15':  0.05,   # v4 신규 (Phase 2)
    'incline_30':  0.05,   # v4 신규 (Phase 2)
}

MU_TERRAIN = {
    'flat': 0.70, 'step': 0.65, 'stairs': 0.60, 'real_stairs': 0.60,
    'wood_block': 0.70, 'rough': 0.55, 'curved_ramp': 0.65,
    'incline_15': 0.65, 'incline_30': 0.60,
}
```

`TAU_REF = 15.0` Nm (휠 측. v3: 1.85 Nm 모터 측)
`P0_HEIGHT_MAX = 0.900` (v3: 0.700. `brk_v` ≤ 0.375 추가로 상한 확대)
`N_PTS = 100` (v3: 160. envelope grid 8000은 그대로, 평가점만 축소)
`EDGE_BOOST = 3.0` (단차 모서리 근처 4x 밀도)

---

## 5. Phase별 상세 변경 사항

### Phase 0 — v3 → v4 fork (삼각형 모드 + brk_v)

* `functions/wpos_jax.py`: `p_arr` 20 → 21 (slot[20] = `brk_v`).
  Wf/Wm/Wr 위치에서 `y`에 `brk_v`를 차감 (링크 끝점이 브래킷 피벗, 휠은 그 아래).
* `decode_x(x)`: `x[14]`을 `p['brk_v']`로 매핑.
* `calc_P0_height_flat(p)`: 기본 높이를 `R_w + brk_v`로 산정.
* 4-mode 워밍업 (`_make_warmup_p` × 4): 별도 컴파일 회피.

### Phase 1 — 동역학 현실화

`functions/calc_dynamics_jax.py`:

* **사다리꼴 속도 프로파일** (`trap_velocity_profile`, line 129):
  0 → v_max(가속) → cruise → 0(감속). 삼각형 분기 포함 (단거리).
  반환: `(v_arr, a_arr)`. `v_arr ≥ 0.05` (시간 미분 발산 방지).
* **종방향 가속도 반영**:
  `F_drv = |W·sin(θ) + 0.02·W·cos(θ) + mass·a_long| / 2`
* **링크 관성**:
  `I_rocker = (1/3)·m_r1·L_r1² + (1/3)·m_r2·L_r2² + I_rocker_add`
  `I_bogie  = m_wheel·c_b² + m_wheel·d_b² + (1/12)·m_bogie·(c_b+d_b)² + I_bogie_add`
* **모터 포화 페널티** (`sat` weight 추가).

`functions/calc_stability_jax.py:23-30`:

* **동적 TOI 보정** — 가속 시 의사-힘 `m·a_long` 이 `h_CG` 높이에 작용 →
  x_cg 가 `-(a_long/g)·h_CG` 만큼 이동한 것과 등가.
  급발진/급제동 시 전후 안정성을 즉시 반영.

* **비대칭 CG** (`CG_offset = 0.050 m`):
  `a_eff = a_h + CG_offset·cos(ar)`, `b_eff = b_h - CG_offset·cos(ar)`.
  전방 50mm 편위로 모든 법선력 분배에 반영.

### Phase 2 — 지형 다양화 + 마찰 모델

`functions/gen_terrain.py`:

* **`curved_ramp`** (cosine bump):
  `y = (h/2)·(1 + cos(2π·u))`, h=150mm, W=1.5m at x=3.0m.
  곡률반경 R = W²/(2π²·h) ≈ 0.38m — 휠 100mm 대비 적당한 챌린지.
* **`incline_15`, `incline_30`** (일정 경사):
  진입 평지 1.5m → 경사 2.0m → 정점 평지.

`functions/calc_dynamics_jax.py`:

* **지형별 μ** (`MU_TERRAIN`): step/stairs/rough/curved_ramp/incline 표면 특성 반영.
* **휠별 슬립** (Phase 2c):
  `slip_r/m/f = |Fd_r/m/f| / (μ · max(N_r/m/f, 0.5))`.
  `slip_violation_rate`, `slip_peak` 메트릭 (`slip` weight).
* **모터 토크-속도 곡선** (Phase 2d, `motor_tau_max`):
  `τ_avail(ω) = τ_peak · clip(1 - |ω|/ω_no_load, 0.1, 1.0)`.
  `sat_peak_speed_aware`, `sat_violation_rate` (점별 τ_demand > τ_avail 비율).

### Phase 2b — 모터 변경 (D6374 → BL70200)

이전: D6374 외장 BLDC 4.95Nm × 5:1 기어 = **21 Nm @ 휠**, 4500 RPM → 휠 900 RPM.
변경: **BL70200 8" 인휠 BLDC** (내장 1:5), 휠 측 정격 22Nm / 피크 39Nm, 무부하 240 RPM @ 48V.

| 항목 | v3 (D6374) | v4 (BL70200) |
|------|-----------|--------------|
| 피크 토크 (휠) | 21 Nm | **39 Nm** |
| 정격 토크 (휠) | (미반영) | **22 Nm** |
| 무부하 회전 | 900 RPM | **240 RPM** (≈ 2.51 m/s) |
| 휠+모터 질량 | 3.5 kg | **4.5 kg** |
| 외부 기어 | 1:5 (별도) | 없음 (내장) |
| Kt_eff | n/a | **2.44 Nm/A** |
| 한계 속도 | 4.5 m/s | **2.5 m/s** (이론) |

연쇄 갱신: `p0`, `TAU_REF` (1.85 → 15.0), `TAU_MOTOR_SAT`, `validate_mujoco.py`,
`analyze_v4_result.py`, `calc_dynamics_jax.motor_tau_max` 기본값.

### Phase 3+ — 열적/전기적 한계 + 시스템 견인 + 적응 샘플링

`functions/calc_dynamics_jax.py:385-460`:

* **Tier A-1** — **연속 정격 토크 (RMS)** [열적 한계]:
  `tau_rms_per_wheel = sqrt(mean(τ²))`. `tau_rms_worst > motor_tau_cont(22Nm)` 시 페널티.
  `cont_violation_rate` = 피크 토크가 정격 초과한 시점 비율. `cont` weight.

* **Tier A-2** — **에너지 메트릭** (보고용, objective 미포함):
  `P(t) = (|τ_f| + |τ_m| + |τ_r|) · ω_wheel · 2` (좌우 대칭).
  `energy_J`, `energy_Wh`, `avg_power_W`, `total_time_s`.
  → 배터리 35Ah × 48V = 1680 Wh 대비 운행 가능 시간 계산 가능.

* **Tier B-1** — **시스템 견인력 부족 (stuck)**:
  `F_traction_cap = μ·ΣN`, `F_demand_long = |mass·a_long + W·sin(θ)|`.
  `traction_util = F_demand_long / F_traction_cap`.
  `system_stuck_rate = mean(traction_util > 1.0)`. `stuck` weight.
  휠 개별 슬립과 독립 — 전체가 못 굴러가는 케이스 감지.

* **Tier B-2** — **배터리 전류 한계**:
  `I_motor = |τ| / Kt_eff`, 합산 6륜.
  `battery_current_peak`, `battery_violation_rate > battery_max_current(30A)`. `batt` weight.

* **Tier C-1** — **적응적 샘플링** (`adaptive_xa`, line 128):
  지형 그라디언트 기반 인버스 CDF 변형.
  `density = 1 + EDGE_BOOST·|∇y|`. JIT shape 안정성 위해 `n_pts` 고정.
  단차 모서리 부근 ~4배 밀도, 평지는 균등.

* **Tier C-2** — **휠 접촉 패치 폭**:
  `calc_envelope_gpu(..., patch_width=0.030)`.
  Minkowski 팽창 후 box-car smoothing (3cm = 8" 타이어). 점접촉의 비현실성 완화.

---

## 6. 신규 도구 및 검증 인프라

| 파일 | 역할 | 라인 수 |
|------|------|--------|
| `analyze_v4_result.py` | 다중 속도 스윕 + 몬테카를로 강건성 + 7신호 판정 | 383 |
| `cross_validate.py` | 옵티마이저 예측 vs MuJoCo 실측 (지형별 토크 오차) | 278 |
| `validate_mujoco.py` | 6륜 절반 MJCF + heightfield 7종 + PD 제어 + 휠당 접촉력 | 466 |
| `design_review.py` | 정성적 디자인 리뷰 + 권고 사항 자동 생성 | 237 |
| `plot_diagnostics.py` | 토크/슬립/포화/배터리 다이얼 (matplotlib) | 217 |
| `plot_geometry.py` | 최적해 지오메트리 시각화 (Wf/Wm/Wr/Pb/CG) | 249 |
| `test_v4.py` | 옵티마이저 컴포넌트 유닛 테스트 | 299 |
| `scripts/watch.sh` | DE 진행 + GPU mem/temp/pwr/util 통합 모니터 | — |
| `scripts/run_v4_smoke_local.sh` | 단축 스모크 (DE_MAXITER=200, popsize=15) | — |

`validate_mujoco.py` 주요 사양:
* 6륜 절반 로봇 (좌측 ×3, 우측 ×3 대칭) MJCF
* 7종 지형 heightfield (flat/step/stairs/wood/rough/curved_ramp/incline_15/30)
* PD 제어 (Kp=1.2, Kd=0) — 토크 포화 회피
* `ctrlrange = ±motor_tau_peak (39 Nm)`
* 휠당 접촉력 로깅 → cross_validate에서 옵티마이저 예측치와 비교

---

## 7. 가속 전략 (실측)

`bsc74cwku.output` 측정:

| 설정 | iter당 시간 |
|------|------------|
| baseline (workers=1, GPU, N_PTS=160) | ~30초 |
| workers=1, GPU, **N_PTS=100** | **3초** (~10배) |
| workers=8, CPU JAX, N_PTS=100 | (CPU bottleneck) |

채택: **N_PTS=100 + workers=1 + GPU**.
원인: GPU util 30% / CPU util 114% → Python 단일 스레드가 병목.
N_PTS 축소가 가장 효과적.
JIT 워밍업 (4-mode 일괄 컴파일): 2-3분 → 약 3초 (재실행 시 캐시 활용).

`DE_MAXITER`, `DE_POPSIZE`, `DE_WORKERS` 환경변수로 런타임 조절 가능.

---

## 8. 메트릭 인덱스 (objective 항목별 매핑)

| objective 항 | weight | 소스 | 단위/정의 |
|-------------|--------|------|----------|
| `tau_norm` | 0.12 | `stair_torque_peak` / TAU_REF | 95% 분위 휠 토크 [Nm/15] |
| `imbal_norm` | 0.08 | (max-min)/mean Nr/Nm/Nf | % |
| `stab_penalty` | 0.18 | TOI_min, liftoff_ratio | clip-scale |
| `sn_norm` | 0.06 | 1/(1+SN/35) | dB |
| `fail_norm` | 0.10 | 수렴 실패 / 총 평가점 | × 10 |
| `sat_norm` | 0.12 | sat_peak_speed_aware, sat_violation_rate | Phase 2d |
| `slip_norm` | 0.12 | slip_violation_rate, slip_peak | Phase 2c |
| `cont_norm` | 0.10 | tau_rms_worst / 22Nm, cont_violation_rate | Phase 3+ A-1 |
| `stuck_norm` | 0.06 | system_stuck_rate | Phase 3+ B-1 |
| `batt_norm` | 0.06 | battery_current_peak / 30A | Phase 3+ B-2 |
| (보고용) `energy_Wh_all` | — | ∫P dt | 누적 에너지 |

---

## 9. 파일별 변경 요약

| 파일 | v3 → v4 변경 핵심 |
|------|------------------|
| `ZETIN_JointOptSearch_v4_gpu.py` | x[0]/x[1] 자유 탐색, x[14]=brk_v, 4-mode warmup, p0 BL70200, W 10개 (cont/batt/stuck 추가), W_terrain 7종, adaptive_xa, DE_WORKERS, polish=False |
| `functions/wpos_jax.py` | p_arr 20→21 (brk_v), Wf/Wm/Wr y에서 brk_v 차감 |
| `functions/calc_dynamics_jax.py` | trap_velocity_profile, motor_tau_max, mass·a_long, 슬립률, 모터 곡선, RMS 토크, 에너지, 시스템 견인, 배터리 전류 |
| `functions/calc_stability_jax.py` | 동적 TOI (a_long pseudo-force), CG_offset |
| `functions/calc_envelope_jax.py` | patch_width 인자 (box-car smoothing) |
| `functions/gen_terrain.py` | curved_ramp, incline_15, incline_30 추가 |
| `functions/newton_solver.py`, `ceq_jax.py`, `calc_metrics_jax.py` | 미세 조정만 |

---

## 10. 알려진 한계 (v5 설계 사유)

* **준정적 모델**: 시간이 명시적이지 않음 — 충격, 동적 추종(PID), 진동 동역학 미반영.
* **측면 절반 2D**: 차체 롤(roll), 좌우 비대칭 지형(한쪽 휠만 단차) 미반영.
* **wheel/link inertia 후처리**: kin_sim은 정역학, 관성/가속도는 후처리 보정 → 자기일관성 한계.
* **MuJoCo 검증에서 큰 brk_v 케이스 텀블 발생** (Test brk_v=347.9mm) — 동적 안정성이 준정적 추정과 발산.
  → v5 (시간영역 ODE)에서만 정확히 잡힘. 자세한 설계는 `docs/specs/2026-05-18-v5-architecture-design.md`.

---

## 11. 현재 산출물

```
parameter_calc/python_gpu_triangle/
├── zetin_optimal_params_v4.pkl              # 최신 (가장 최근 본 실행 결과)
├── zetin_optimal_params_v4_smoke.pkl        # 스모크 (200 iter)
├── zetin_optimal_params_v4_phase2_3iter.pkl
├── zetin_optimal_params_v4_phase3_3iter.pkl
└── zetin_optimal_params_v4_phase3c_smoke.pkl
```

각 `.pkl`는 `{p_opt, x_opt, f_opt, elapsed, lb, ub, W, W_terrain, version}`.

---

## 12. 다음 단계 (제안)

1. **v4 본 실행 결과 분석** — `analyze_v4_result.py`, `cross_validate.py`, `design_review.py` 순차 실행.
2. **MuJoCo 풀 검증** — 7종 지형 + 모터 사양 + heightfield로 정밀 비교.
3. **v5 구현 결정** — `docs/specs/2026-05-18-v5-architecture-design.md` 검토 후 Phase 5a/5b 진입 여부 결정.

---

## 13. 관련 문서

* `parameter_calc/CLAUDE.md` — 시뮬레이션 파이프라인 전반
* `docs/specs/2026-05-18-v5-architecture-design.md` — 시간영역 ODE 시뮬레이터 v5 설계
* `.claude/CLAUDE.md` — 프로젝트 전체 개요 (motor_control 포함)
