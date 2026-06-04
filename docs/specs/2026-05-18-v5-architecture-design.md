# v5 아키텍처 설계 — 시간 영역 동역학 검증/최적화

**작성일**: 2026-05-18
**상태**: 설계 (구현 미시작)
**대상**: v4 (준정적 옵티마이저)와 병행 운영되는 v5 (시간 영역 검증기/2차 최적기)

---

## 1. 배경 & 동기

### v4의 한계

현재 `parameter_calc/python_gpu_triangle/` 의 v4 옵티마이저:

- **준정적**: 각 x 위치에서 Newton constraint solver로 *순간* 자세 결정. 시간 진화 없음.
- **세 휠 항상 접지 가정**: kin_sim의 ceq가 3-point contact를 강제. 일시적 들림 모델 불가.
- **단일 사다리꼴 v(x) 프로파일**: 실제 v는 시뮬 결과로 *변경되지 않음*.
- **측면뷰 2D 가정**: 좌/우 대칭, yaw=0.

이러한 가정 덕에 매우 빠른 옵티마이저(N=160점 평가에 ~0.06초/eval, 1만 evals/min)가 가능했음.

### v4의 실제 검증 결과 (MuJoCo)

`validate_mujoco.py` 결과: 비-flat heightfield에서 v4 최적해가 *동역학적으로 텀블*. f_opt 0.20 이라는 "좋은" 점수에도 실제 거동에선 피크 ±90°+ 회전 발생. 이는 *준정적 가정의 본질적 한계*.

### v5의 목적

> 준정적 v4의 한계를 *정량화*하고, 동역학적으로 *재현 가능*한 디자인을 *최종 선정*하기 위한 high-fidelity 검증/2차 최적기.

---

## 2. 결정 사항

| 결정 | 선택 | 근거 |
|------|------|------|
| **스코프** | D-1만 (시간 영역 ODE, 2D 유지) | 동역학 효과가 대부분 실패 원인. 좌/우 비대칭(3D)은 별도 v6 분리. 2D 유지로 v4와 좌표 호환. |
| **엔진** | 하이브리드 (단계적) | v5a (MuJoCo 확장 검증) 단기, v5b (JAX ODE) 중기. 인프라 재사용 최대화. |
| **역할** | Secondary 검증기 + 비교기 (B+C) | v4를 primary 옵티마이저로 유지. v5는 v4 최종 후보의 *깊이 검증* + *fidelity 정량화*. |

---

## 3. 두 단계 아키텍처

### 3.1 v5a — MuJoCo 기반 깊이 검증 (단기, 1~2주)

**무엇**: 기존 `validate_mujoco.py` + `cross_validate.py`를 *프로덕션 등급*으로 확장.

**무엇이 추가되나** (현 도구 대비):

| 항목 | 현재 (Phase 3) | v5a (목표) |
|------|---------------|-----------|
| 모델 | 측면뷰 절반 (3륜) | **6륜 전체 + 좌/우 동기화 (differential 메커니즘)** |
| 제어 | P 게인 1.2 (정착 0.5초) | **PID + feedforward + 정착 1.0초** |
| 지형 | heightfield 메커니즘 OK, 조정 필요 | **각 지형별 패치 폭 = 70mm 타이어 반영** |
| 컨트롤러 한계 | ±39Nm clip만 | **모터 토크-속도 곡선 + 전류 한계 실제 모델** |
| 자세 초기화 | 평지 기준 chassis_z0 | **각 지형 시작점 자동 정렬 (rocker/bogie 각도 평형)** |
| 출력 | 단일 텍스트 리포트 | **CSV, plot PNG, 동영상 (mujoco-py renderer)** |
| 검증 메트릭 | 4종 (속도/토크/자세/접지) | **9종 (위 + 슬립률/연속토크/배터리/에너지/추진 효율)** |

**산출물**:
- `parameter_calc/python_mujoco_v5a/`
- `mjcf_builder_v5a.py` — 풀 6륜 MJCF 생성
- `simulate_v5a.py` — 정착 + PID + 다중 지형 자동 평가
- `validate_v5a.py` — v4 pkl → 자동 검증 리포트 (markdown + PNG + mp4)
- `cross_validate_v4_v5a.py` — fidelity 점수 정밀화

### 3.2 v5b — 커스텀 JAX ODE 옵티마이저 (중기, 4~5주)

**무엇**: 시간 영역 multibody 동역학 *직접* 구현. 빠른 GPU 최적화 가능.

#### 시스템 모델 (2D 측면뷰 절반 로봇)

자유도 (Generalized Coordinates, **q** ∈ ℝ⁶):
```
q[0] = x_chassis    (월드 x)
q[1] = z_chassis    (월드 z)
q[2] = θ_chassis    (피치)
q[3] = φ_rocker     (chassis 기준 rocker 회전)
q[4] = φ_bogie      (rocker 기준 bogie 회전)
q[5..7] = ψ_wheel   (3개 휠 회전: front, mid, rear)  ← 사실 8개 변수
```

정확하게는 **q ∈ ℝ⁸** (chassis 3 + rocker/bogie 2 + wheels 3).

**상태 벡터**: `(q, q̇) ∈ ℝ¹⁶`

#### 운동 방정식 (Newton-Euler)

```
M(q) q̈ + C(q, q̇) q̇ + G(q) = τ_actuator + τ_contact
```

- `M(q)`: 8×8 generalized mass matrix
- `C(q, q̇) q̇`: 코리올리/원심력
- `G(q)`: 중력항
- `τ_actuator`: 모터 토크 (휠 회전축 3개에만 적용)
- `τ_contact`: 지면 접촉 force/moment

**유도 방법**: Lagrange (T - V) 또는 Newton-Euler 재귀.
- Lagrange: 대칭적, 자동 미분 가능 (JAX). 추천.
- Newton-Euler: 더 빠르지만 EOM 정리가 복잡.

`L = T - V` 형태로 Lagrangian 정의 후 JAX autodiff:
```python
@jax.jit
def lagrangian(q, q_dot, p_arr):
    T = kinetic_energy(q, q_dot, p_arr)   # 본체+링크+휠 운동E
    V = potential_energy(q, p_arr)        # 중력 위치E
    return T - V

# Euler-Lagrange equations via autodiff
dL_dq = jax.grad(lagrangian, argnums=0)
dL_dqdot = jax.grad(lagrangian, argnums=1)
# Solve M q_ddot = F where F = τ - C q_dot - dV/dq + ...
```

#### 접촉 모델

각 휠 i에 대해:
```
정상력 N_i = max(0, k·δ_i + c·δ̇_i)   # spring-damper
                                     where δ_i = max(0, R_w - dist_to_ground)
마찰력 F_i = min(μ·N_i, F_demanded)  # Coulomb with saturation
```

- `k`: 1e5 N/m (단단한 노면)
- `c`: 1e3 N·s/m (감쇠)
- 슬립 발생 시 미분기능 약화 (smooth max로 ODE 안정성 확보)

#### 모터 모델

```
τ_motor(ω, V_bus, τ_cmd) = clip(τ_cmd, ±τ_max(ω))
where τ_max(ω) = τ_peak · (1 - |ω|/ω_no_load)
```

- BL70200: τ_peak=39, ω_no_load=25.1 rad/s
- 전류: I = |τ| / Kt_eff (Kt_eff=2.44 Nm/A)
- 6모터 합 전류 ≤ 30A (배터리 한계) → 초과 시 비례 축소

#### 제어기

```
v_target(t) = trapezoidal_profile(t)    # v4와 동일
ω_target(t) = v_target / R_w
e(t) = ω_target - ω_wheel_avg
τ_cmd(t) = Kp·e + Ki·∫e dt + Kd·ė       # PID
```

게인: 실제 ODrive controller 유사 (Kp=2~5, Ki=0.5, Kd=0.05). 튜닝 필요.

#### ODE 적분

- **선택 1**: scipy.integrate.solve_ivp (RK45) — 가변 스텝, 정확. JAX 호환 안 됨 (CPU only).
- **선택 2**: 커스텀 RK4 in JAX — 고정 스텝 dt=1ms. JIT 가능. GPU 가속.
- **선택 3**: jax.experimental.ode.odeint — JAX 내장. RK45 변형. JIT 가능.

**추천: 선택 3 (jax.experimental.ode)** — 검증된 구현 + GPU 가속.

#### 모듈 구조

```
parameter_calc/python_gpu_v5/
├── README_v5.md                       # 사용법 + v4와의 차이
├── ZETIN_TimeOpt_v5_gpu.py            # 메인 옵티마이저 (DE + objective_v5)
├── functions_v5/
│   ├── __init__.py
│   ├── lagrangian_jax.py              # T, V, L = T - V
│   ├── eom_jax.py                     # Euler-Lagrange → q_ddot
│   ├── contact_jax.py                 # 휠별 N, F (spring-damper + Coulomb)
│   ├── motor_jax.py                   # τ_max(ω), 전류 계산
│   ├── controller_jax.py              # PID + 사다리꼴 ref
│   ├── trajectory_sim_jax.py          # 전체 ODE 롤아웃 (시간 적분)
│   ├── metrics_v5_jax.py              # 시간 영역 메트릭 (peak τ, energy, tipover, etc)
│   ├── gen_terrain.py                 # ← v4에서 import 또는 sym-link
│   └── calc_envelope_jax.py           # ← v4에서 import (휠 envelope 그대로 사용)
├── analyze_v5_result.py               # v5 pkl 분석 (v4 analyze와 유사 형식)
├── cross_validate_v4_v5.py            # v4 vs v5 결과 비교 (핵심 산출물)
└── scripts/
    ├── run_v5_smoke.sh
    └── run_v5.sh
```

#### 데이터 흐름

```
[1] 초기화
    q0 = 평지 정착 자세 (rocker/bogie 평형각 계산)
    state0 = (q0, 0)

[2] 시간 적분 (t = 0 → T_max, dt ~ 1ms)
    매 step:
      a. terrain_height(q.position) → ground contact
      b. contact_forces(q, q_dot, terrain) → τ_contact
      c. motor_torques = controller(t, ω_wheels)
      d. q_ddot = eom(q, q_dot, τ_actuator, τ_contact)
      e. state = integrator_step(state, q_ddot, dt)
      f. log: (t, q, q_dot, τ, N, ...)

[3] 메트릭 계산
    τ_peak_motor (each wheel) — vs v4 stair_torque_max
    τ_rms_motor (each wheel) — vs v4 tau_rms_worst
    tipover_event (any |θ_chassis| > 60°)
    success (x_chassis 도달)
    energy_J (∫ τ·ω dt)
    슬립 발생 시간 비율
    배터리 전류 피크

[4] objective_v5(metrics) → f
    v4와 유사한 가중합. 다만 동역학 메트릭 추가.
```

---

## 4. 호환성 & 마이그레이션

### 4.1 파라미터 호환

**중요**: v5는 v4와 *완전히 동일한* 15-차원 파라미터 공간 사용.

```
x = [rocker_mode, bogie_mode, T_r/L_r1, S_r1/L_r2, S_r2 또는 미사용,
     th_r1, th_r2, j_r, T_b/L_b1, S_b1/L_b2, S_b2 또는 미사용,
     th_b1, th_b2, j_b, brk_v]
```

→ v4 pkl을 그대로 v5에 입력 가능. 직접 비교.

### 4.2 메트릭 비교

| v4 메트릭 (준정적) | v5 메트릭 (시간 영역) | 비교 방법 |
|-------------------|---------------------|----------|
| `stair_torque_max` | `tau_peak_motor` | 직접 비교 |
| `min_TOI` | `tipover_event_count` + `min_pitch_margin` | 정성적 |
| `liftoff_ratio` | `contact_loss_time_ratio` | 직접 비교 |
| `energy_Wh` | `∫τω dt` (적분) | 직접 비교 |
| `slip_violation_rate` | `slip_time_ratio` | 직접 비교 |

### 4.3 cross_validate_v4_v5.py 산출물

```
=== v4 vs v5 비교 ===
지형        τ peak (v4)  τ peak (v5)  차이%   성공
real_stairs    18.5 Nm       28.3 Nm    +53%   v4성공/v5실패
wood_block      9.2 Nm       11.8 Nm    +28%   둘 다 성공
curved_ramp     6.1 Nm        7.4 Nm    +21%   둘 다 성공
...
Fidelity 점수: 35% 평균 오차 → v4 신뢰도 "중"
```

이 표가 *진짜 가치*. 어디서 v4가 과소평가하는지 명확.

---

## 5. 단계별 개발 계획

### Phase 5a-1: MuJoCo 검증기 확장 (1~2주)

| 주차 | 작업 |
|------|------|
| W1 | 6륜 풀 모델 MJCF 생성, differential 메커니즘 (`<equality>` 또는 weld constraint) |
| W2 | PID 튜닝, 다중 지형 검증, 자동 리포트 |

**산출물**: v4 결과 → 정량적 fidelity 점수. v5b 필요성 확인.

### Phase 5b-1: EOM 도출 + 단위 테스트 (1주)

| 작업 | 검증 방법 |
|------|----------|
| Lagrangian T, V 도출 | 분석해 분리/심볼릭 sympy 비교 |
| Euler-Lagrange EOM | 평지에서 자유 낙하 → 단순 단진자 거동 비교 |
| 한 휠만 활성 | 평지에서 전진 → F=ma 일치 확인 |

### Phase 5b-2: 접촉 + 모터 + 제어기 (1주)

| 작업 | 검증 방법 |
|------|----------|
| Spring-damper 접촉 | 평지에서 정상력 = 무게/3 (3륜 측면 반쪽) |
| 모터 토크-속도 곡선 | step τ_cmd → ω → τ 실제 곡선 |
| PID 속도 제어 | 평지 v_target 추종 (오차 < 5%) |

### Phase 5b-3: 시간 적분 + 단일 지형 (1주)

| 작업 | 검증 방법 |
|------|----------|
| odeint 통합 | 평지 5초 주행: 최종 x = 4m (= 0.8 m/s × 5s) |
| 계단 단일 (real_stairs) | 통과 가능 디자인 → v4 결과 재현 |
| 텀블 케이스 | brk_v=350 디자인 → 텀블 발생 확인 (MuJoCo와 일치) |

### Phase 5b-4: 최적화 통합 + 비교 (1주)

| 작업 | 결과 |
|------|------|
| DE 통합 | maxiter=200, popsize=15 (v5는 평가 비용 크므로 budget 줄임) |
| cross_validate_v4_v5.py | 5~10개 v4 후보 디자인을 v5로 재평가 |
| 디자인 결정 보고서 | 최종 디자인 1개 선정 (v4 + v5 합의) |

**총 소요**: v5a 2주 + v5b 4주 = **6주**

---

## 6. 리스크 & 완화 전략

| 리스크 | 영향 | 완화 |
|--------|------|------|
| **EOM 유도 버그** | 모든 결과 무효 | 단위 테스트 + 분석해 비교. JAX autodiff로 도함수 자동 검증. |
| **접촉 stiffness 진동** | ODE 발산 | 임플리시트 적분 또는 smooth-max로 비선형성 완화. 또는 MuJoCo로 폴백. |
| **v5 평가 비용 폭증** | DE 수렴 불가 | (1) v4 결과를 v5 초기 population으로 사용 (warm start) (2) v5는 후보 5~10개만 평가 (full search 안 함) |
| **v4 vs v5 결과 크게 다름** | 어느 쪽을 믿어야? | MuJoCo (외부 검증)와 비교하여 *제3자 판정* |
| **6륜 differential 모델링 복잡** | v5a 일정 초과 | 우선 좌/우 동기화 (rigid constraint)로 단순화. 진짜 differential은 v5a-2로 분리. |

---

## 7. 미해결 결정

다음 항목들은 *개발 시작 시* 추가 결정 필요:

1. **JAX 버전 호환**: jax.experimental.ode가 stable인가? 대안 backup 필요.
2. **EOM 코드 생성**: 손으로 유도 vs sympy 자동 생성? 후자 권장하지만 sympy 종속성 추가.
3. **시간 영역 가시화**: 동영상 (matplotlib animation) 또는 MuJoCo viewer 차용?
4. **v4 polish**: v5에서 발견한 *동역학적 우수 디자인*을 v4로 *역검증* 가능한가? (재실행)
5. **v6 (3D) 분리 시점**: v5b 완료 후 즉시 vs v5 결과 충분히 신뢰 가능 후?

---

## 8. v4와의 관계 (서로 죽이지 않기)

- v4는 *동결되지 않음*. 계속 빠른 mid-fidelity 탐색기로 사용.
- v5는 v4 결과에 *피드백*만 제공 (수정 안 함).
- 개발 중 v4 결과 손상되지 않도록 `python_gpu_triangle/`과 `python_gpu_v5/`는 *완전 독립* (functions 디렉토리 fork, v3→v4 분기와 같은 패턴).
- gen_terrain, calc_envelope만 v4에서 *import* (이건 둘 다 같은 지형 모델 써야 비교 가능하므로 공유).

---

## 9. 즉시 시작 가능한 작업 (v4 본 실행 중에도)

본 실행이 도는 동안 가능한 v5a/v5b 준비:

1. **v5b EOM 도출 (종이/sympy)** — JAX 코딩 전 분석해 분리.
2. **현 validate_mujoco를 v5a 골격으로 리네임/이동** — `python_mujoco_v5a/` 디렉토리 생성.
3. **단위 테스트 인프라** — `tests/test_eom_consistency.py` 등.
4. **이 설계 문서 리뷰/수정** — 결정 누락 사항 발견 시 갱신.

---

## 부록 A: 8-DOF 시스템의 EOM 유도 개요

```
변수: q = (x, z, θ, φ_r, φ_b, ψ_f, ψ_m, ψ_r)

T_chassis = (1/2) m_c (ẋ² + ż²) + (1/2) I_c θ̇²
T_rocker = (1/2) m_r |v_cm_r|² + (1/2) I_r (θ̇ + φ̇_r)²
T_bogie = (1/2) m_b |v_cm_b|² + (1/2) I_b (θ̇ + φ̇_r + φ̇_b)²
T_wheels = Σ_i (1/2) m_w |v_w_i|² + (1/2) I_w (관성 추가 항)
T = T_chassis + T_rocker + T_bogie + T_wheels

V = m_c g z_c + m_r g z_cm_r + m_b g z_cm_b + Σ_i m_w g z_w_i

L = T - V

EOM: d/dt(∂L/∂q̇_i) - ∂L/∂q_i = Q_i  (i=1..8)
    Q_i: generalized force (motor τ, contact F)
```

질량 중심 위치들은 `wpos_jax`의 forward kinematics와 동일 (v4 재사용 가능!).

JAX 구현 outline:
```python
from jax import grad, jit
import jax.numpy as jnp

def forward_kin(q, p):
    """위치 (chassis CG, rocker CG, bogie CG, 각 휠 CG)"""
    ...

def T_total(q, q_dot, p):
    positions = forward_kin(q, p)
    velocities = jax.jacobian(forward_kin)(q, p) @ q_dot   # chain rule
    # 운동 에너지 합산
    ...

def V_total(q, p):
    positions = forward_kin(q, p)
    return sum(m_i * g * z_i for ...)

@jit
def lagrangian(q, q_dot, p):
    return T_total(q, q_dot, p) - V_total(q, p)

# Euler-Lagrange (autodiff)
@jit
def q_ddot(q, q_dot, tau, F_contact, p):
    M = jax.hessian(lagrangian, argnums=1)(q, q_dot, p)   # ∂²L/∂q̇²
    # ... 해석적 또는 수치적으로 M q̈ = F 풀기
    ...
```

---

## 부록 B: 시간 영역 메트릭 vs v4 메트릭 매핑

| 디자인 관심사 | v4 (준정적) | v5 (시간 영역) | 어느 게 더 정확? |
|--------------|------------|--------------|----------------|
| 모터 토크 한계 | 95% percentile peak | 시간 누적 peak | **v5** (충격 토크 포함) |
| 열적 한계 | RMS 토크 추정 | 실제 적분 RMS | **v5** |
| 전복 안정성 | TOI (정적) | 실제 피치 시계열 max | **v5** |
| 가속/감속 토크 | 사다리꼴 가정 | 실제 PID 추종 결과 | **v5** |
| 정적 무게 분포 | 정확 | (포함됨) | 동등 |
| 휠 들림 비율 | binary (Nf or Nr < 0) | 실제 contact 시간 | **v5** |
| 슬립 | 점접촉 ratio | 시간 영역 슬립 누적 | **v5** |
| 디자인 공간 탐색 속도 | 매우 빠름 | 느림 | **v4** |

→ **v4 = 빠른 탐색, v5 = 정확한 검증**. 합쳐서 사용.
