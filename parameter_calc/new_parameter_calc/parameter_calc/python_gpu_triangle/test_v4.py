"""test_v4.py — v4 옵티마이저 함수 단위 테스트 (실제 assertion 기반)

이전 버전의 문제(전면 재작성 사유):
  - `sys.path.insert(0, '../python_gpu')` → v3 functions(20요소 p_arr, brk_v 없음)를 import.
  - 14차원 bounds + decode_x가 x[14](brk_v) 미사용.
  - p0 상수가 Phase 0 시절(mass 30kg, tau_peak 4.95, gear 5, TAU_REF 1.85, W 5키, 4지형)로 노후.
  - `assert`가 하나도 없어 무조건 "통과" 출력 → 어떤 회귀도 못 잡음.

이 버전:
  - 로컬 v4 functions만 사용(script_dir), 15차원·brk_v 반영.
  - 면-기준 정적평형(평지 ΣN ≈ 0.5·mass·g) 등 물리 불변식을 실제로 검증.
    → calc_dynamics의 무게 좌표계 회귀(전체 W로 되돌림)를 즉시 검출.

실행: python test_v4.py   (실패 시 AssertionError로 비정상 종료, 종료코드 ≠ 0)
"""
import os
import sys

# 백엔드는 강제하지 않는다(자동 선택) — CPU dev 컨테이너/ GPU 양쪽에서 실행 가능.
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')

import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)  # 로컬 v4 functions (v3 아님 — 핵심 회귀 사유였음)

from functions.gen_terrain import gen_terrain
from functions.calc_envelope_jax import calc_envelope_gpu
from functions.newton_solver import kin_sim_gpu
from functions.calc_dynamics_jax import (
    calc_dynamics_gpu, trap_velocity_profile, motor_tau_max,
)
from functions.calc_stability_jax import calc_stability_gpu
from functions.calc_metrics_jax import calc_metrics_gpu
from functions.wpos_jax import pack_params_auto

import jax
print(f'JAX backend: {jax.default_backend()}\n')

# ═══════════════════════════════════════
# v4 p0 — ZETIN_JointOptSearch_v4_gpu.py 와 일치 (BL70200 50kg)
# ═══════════════════════════════════════
p0 = {
    'R_w': 0.100, 'h_body': 0.300, 'mass': 50, 'g': 9.81,
    'obs_h': 0.150, 'mu': 0.70,
    'gear_ratio': 1.0, 'eta_gear': 1.0,
    'motor_tau_peak': 39.0, 'motor_tau_cont': 22.0,
    'omega_no_load_rpm': 240.0, 'V_bus': 48.0, 'v_min_advisable': 0.14,
    'Kt_eff': 22.0 / 9.0, 'battery_max_current': 30.0,
    'n_wheel_total': 6,
    'm_wheel': 4.5, 'm_rocker_link': 2.5, 'm_bogie_link': 1.5,
    'I_rocker_add': 0.15, 'I_bogie_add': 0.08, 'e_restitution': 0.3,
    'v_robot': 0.8, 'v_max': 0.8, 'a_lim': 1.5, 'v_max_flat': 2.0,
    'step_thresh': 5.0, 'phi_r0': 0, 'delta_pb': 0,
    'CG_offset': 0.050, 'patch_width': 0.030,
}
p0['h_CG'] = p0['h_body'] * 0.55

# 옵티마이저의 objective 가중치 (합 == 1.0 검증용)
W = {'tau': 0.12, 'imbal': 0.08, 'stab': 0.18, 'sn': 0.06, 'fail': 0.10,
     'sat': 0.12, 'slip': 0.12, 'cont': 0.10, 'batt': 0.06, 'stuck': 0.06}
W_terrain = {'stairs': 0.45, 'wood': 0.12, 'rough': 0.13, 'step': 0.10,
             'curved_ramp': 0.10, 'incline_15': 0.05, 'incline_30': 0.05}

# 옵티마이저의 실제 15차원 탐색 공간 (Phase 0 14차원이 아님)
LB = [1, 1, 0.20, 0.15, 60,  0,  0,  0.30, 0.15, 0.15, 60,  0,  0,  0.30, 0.345]
UB = [2, 2, 0.45, 0.35, 160, 35, 35, 0.70, 0.35, 0.35, 160, 40, 40, 0.70, 0.375]


# ═══════════════════════════════════════
# 테스트 하니스
# ═══════════════════════════════════════
_PASS = 0


def check(cond, msg):
    global _PASS
    if not cond:
        raise AssertionError(f'FAIL: {msg}')
    _PASS += 1
    print(f'  ✓ {msg}')


def make_p(rm, bm, brk_v=0.360):
    """검증용 기하 파라미터 dict 생성 (워밍업 기하와 동일 계열)."""
    p = dict(p0)
    p['brk_v'] = brk_v
    if rm == 'triangle':
        p['rocker_mode'] = 'triangle'
        p['L_r1'] = 0.30; p['L_r2'] = 0.25; p['alpha_r'] = np.deg2rad(120)
    else:
        p['rocker_mode'] = 'frame'
        p['T_r'] = 0.30; p['S_r1'] = 0.20; p['S_r2'] = 0.15
        p['th_r1'] = 0.3; p['th_r2'] = 0.3; p['j_r'] = 0.5
    if bm == 'triangle':
        p['bogie_mode'] = 'triangle'
        p['L_b1'] = 0.22; p['L_b2'] = 0.20; p['beta_b'] = np.deg2rad(120)
        p['c_b'] = p['L_b1'] * abs(np.sin(p['beta_b'] / 2))
        p['d_b'] = p['L_b2'] * abs(np.sin(p['beta_b'] / 2))
    else:
        p['bogie_mode'] = 'frame'
        p['T_b'] = 0.20; p['S_b1'] = 0.18; p['S_b2'] = 0.12
        p['th_b1'] = 0.3; p['th_b2'] = 0.3; p['j_b'] = 0.5
        p['c_b'] = 0.14; p['d_b'] = 0.14
    return p


def flat_sim(p, n=60):
    """평지에서 kin_sim 실행 → (xa, x_t, y_raw, y_env, R).

    옵티마이저와 동일하게 raw 지형(간섭검사용)과 envelope(역기구학·동역학용)를 분리 반환.
    """
    x_t, y_raw = gen_terrain('flat', p)
    y_env = calc_envelope_gpu(x_t, y_raw, p['R_w'], patch_width=p.get('patch_width', 0.030))
    xa = np.linspace(1.0, 9.0, n)
    p_arr = pack_params_auto(p)
    R = kin_sim_gpu(xa, x_t, y_env, p_arr, p)
    return xa, x_t, y_raw, y_env, R


# ═══════════════════════════════════════
# TEST 1: objective 가중치 합 == 1.0
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 1: 가중치 무결성')
print('=' * 55)
check(abs(sum(W.values()) - 1.0) < 1e-9, f'W 합 == 1.0 (실제 {sum(W.values()):.4f})')
check(abs(sum(W_terrain.values()) - 1.0) < 1e-9,
      f'W_terrain 합 == 1.0 (실제 {sum(W_terrain.values()):.4f})')
check(len(W) == 10, 'W 항목 10개 (Phase 3+: cont/batt/stuck 포함)')
check(len(W_terrain) == 7, 'W_terrain 지형 7종')
print()

# ═══════════════════════════════════════
# TEST 2: 탐색 공간 15차원 + brk_v
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 2: 15차원 탐색 공간 + brk_v 운동학 배선')
print('=' * 55)
check(len(LB) == 15 and len(UB) == 15, 'bounds 15차원 (Phase 0의 14차원 아님)')
check(LB[14] == 0.345 and UB[14] == 0.375, 'x[14] = brk_v ∈ [0.345, 0.375]')

p_ff = make_p('frame', 'frame', brk_v=0.360)
arr = np.array(pack_params_auto(p_ff))
check(arr.shape[0] == 21, f'pack_params_auto → 21요소 p_arr (v3의 20 아님), 실제 {arr.shape[0]}')
check(abs(float(arr[20]) - 0.360) < 1e-6, f'p_arr[20] == brk_v (실제 {float(arr[20]):.4f})')
check(int(round(float(arr[5]))) == 2 and int(round(float(arr[6]))) == 2,
      'frame-frame 모드 코드 == (2, 2)')
arr_tt = np.array(pack_params_auto(make_p('triangle', 'triangle')))
check(int(round(float(arr_tt[5]))) == 1 and int(round(float(arr_tt[6]))) == 1,
      'triangle-triangle 모드 코드 == (1, 1)')
print()

# ═══════════════════════════════════════
# TEST 3: 모터 토크-속도 곡선
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 3: motor_tau_max(ω) 곡선')
print('=' * 55)
tau0 = float(motor_tau_max(0.0, motor_tau_peak=39.0, omega_no_load_rpm=240.0))
check(abs(tau0 - 39.0) < 1e-6, f'ω=0 → τ_peak (39.0), 실제 {tau0:.3f}')
tau_hi = float(motor_tau_max(1e4, motor_tau_peak=39.0, omega_no_load_rpm=240.0))
check(abs(tau_hi - 3.9) < 1e-3, f'ω≫ω_no_load → 0.1·τ_peak clip (3.9), 실제 {tau_hi:.3f}')
om = np.linspace(0, 25, 50)
tau_curve = np.asarray(motor_tau_max(om, motor_tau_peak=39.0, omega_no_load_rpm=240.0))
check(np.all(np.diff(tau_curve) <= 1e-9), 'τ_max(ω) 단조 비증가')
print()

# ═══════════════════════════════════════
# TEST 4: 사다리꼴 속도 프로파일
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 4: trap_velocity_profile')
print('=' * 55)
xa_t = np.linspace(0.0, 5.0, 200)
v_arr, a_arr = trap_velocity_profile(xa_t, v_max=0.8, a_lim=1.5)
check(v_arr.max() <= 0.8 + 1e-6, f'v ≤ v_max (peak {v_arr.max():.3f})')
check(v_arr.min() >= 0.05 - 1e-9, f'v ≥ 0.05 (시간미분 발산 방지, min {v_arr.min():.3f})')
check(abs(a_arr.max() - 1.5) < 1e-6 and abs(a_arr.min() + 1.5) < 1e-6,
      'a_arr 가속 +a_lim / 감속 -a_lim 포함')
check(np.any(np.abs(a_arr) < 1e-9), 'cruise 구간(a=0) 존재')
print()

# ═══════════════════════════════════════
# TEST 5: Envelope 팽창 단조성 (Minkowski ≥ 원지형)
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 5: calc_envelope_gpu 팽창')
print('=' * 55)
x_st, y_st = gen_terrain('real_stairs', p0)
y_env_pure = calc_envelope_gpu(x_st, y_st, p0['R_w'], patch_width=0.0)
check(np.all(y_env_pure >= y_st - 1e-6),
      '순수 Minkowski 팽창 ≥ 원지형 (모든 점)')
check(y_env_pure.shape == y_st.shape, 'envelope shape == 지형 shape')
print()

# ═══════════════════════════════════════
# TEST 6: 평지 kin_sim 수렴
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 6: 평지 역기구학 수렴 (frame-frame)')
print('=' * 55)
xa, x_t, y_raw, y_env, R = flat_sim(p_ff)
check(R['fail_rate'] == 0.0, f'평지 수렴 실패율 0% (실제 {R["fail_rate"]*100:.1f}%)')
check(np.all(R['ok']), '모든 평가점 수렴(ok)')
# 평지에서는 지형이 모든 곳에서 동일하므로 자세(ar)가 위치에 무관하게 일정해야 한다.
# (값 자체는 기하에 따라 0이 아닐 수 있음 — 일정함이 올바른 불변식.)
check(np.nanstd(R['ar']) < np.deg2rad(1), '평지 rocker 각 일정 (std < 1°)')
check(np.nanstd(R['bb']) < np.deg2rad(1), '평지 bogie 각 일정 (std < 1°)')
print()

# ═══════════════════════════════════════
# TEST 7: 면-기준 정적 평형 — ΣN ≈ 0.5·mass·g  (★ 무게 좌표계 회귀 검출)
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 7: 측면-절반 정적평형 ΣN ≈ 0.5·mg  (무게 좌표계 가드)')
print('=' * 55)
D = calc_dynamics_gpu(R, xa, x_t, y_env, dict(p_ff))
sumN = D['Nr'] + D['Nm'] + D['Nf']
cruise = (np.abs(D['a_long']) < 1e-9) & R['ok']
check(np.sum(cruise) > 5, f'cruise(a_long=0) 구간 충분 ({int(np.sum(cruise))}점)')
mean_sumN = float(np.nanmean(sumN[cruise]))
target_side = 0.5 * p0['mass'] * p0['g']    # 면-기준: 한쪽이 절반 지지
target_full = p0['mass'] * p0['g']
check(abs(mean_sumN - target_side) / target_side < 0.02,
      f'평지 cruise ΣN ≈ 0.5·mg ({mean_sumN:.1f}N vs {target_side:.1f}N, 오차<2%)')
check(abs(mean_sumN - target_full) / target_full > 0.10,
      f'ΣN 이 전체 mg({target_full:.1f}N)와 명확히 다름 → 면-기준 확인')
print()

# ═══════════════════════════════════════
# TEST 8: 토크 유한·양수, 평지 cruise는 모터 피크 미만
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 8: 모터 토크 위생')
print('=' * 55)
for k in ('tau_motor_r', 'tau_motor_m', 'tau_motor_f'):
    valid = D[k][R['ok']]
    check(np.all(np.isfinite(valid)) and np.all(valid >= -1e-9),
          f'{k} 유한 & ≥ 0 (평지 유효점)')
tau_peak_flat = float(np.nanmax(D['tau_max_arr'][R['ok']]))
check(tau_peak_flat < p0['motor_tau_peak'],
      f'평지 최대 토크 < τ_peak ({tau_peak_flat:.2f} < {p0["motor_tau_peak"]:.0f} Nm)')
print()

# ═══════════════════════════════════════
# TEST 9: 평지 안정성 SAFE
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 9: 평지 전복 안정성')
print('=' * 55)
S = calc_stability_gpu(R, xa, x_t, y_raw, y_env, dict(p_ff))
check(S['min_TOI'] > 0, f'min_TOI > 0 (실제 {S["min_TOI"]:.3f})')
check(S['liftoff_ratio'] == 0.0, f'평지 들림 0% (실제 {S["liftoff_ratio"]*100:.1f}%)')
check(not S['is_collision'], '차체-지형 간섭 없음')
# 'safe' vs 'warning'은 TOI 마진(기하 의존)에 좌우됨 — 평지의 핵심 불변식은
# danger(TOI<0·간섭·들림 초과)가 아니라는 것.
check(S['risk_level'] != 'danger', f"위험도 != danger (실제 {S['risk_level']})")
print()

# ═══════════════════════════════════════
# TEST 10: 지형 생성 형상/높이 sanity
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 10: gen_terrain 형상')
print('=' * 55)
for t in ['flat', 'step', 'stairs', 'rough', 'real_stairs', 'wood_block',
          'curved_ramp', 'incline_15', 'incline_30']:
    xt, yt = gen_terrain(t, p0)
    check(xt.shape == (8000,) and yt.shape == (8000,), f'{t}: 8000점 격자')
rs_x, rs_y = gen_terrain('real_stairs', p0)
check(abs(rs_y.max() - 3 * 0.080) < 1e-6, f'real_stairs 정점 = 3×80mm (실제 {rs_y.max()*1000:.0f}mm)')
i30_x, i30_y = gen_terrain('incline_30', p0)
check(abs(i30_y.max() - 2.0 * np.sin(np.deg2rad(30))) < 1e-3,
      f'incline_30 정점 = 2·sin30 = 1.0m (실제 {i30_y.max():.3f}m)')
print()

# ═══════════════════════════════════════
# TEST 11: triangle-triangle 경로도 동작 (모드 분기 회귀 방지)
# ═══════════════════════════════════════
print('=' * 55)
print('TEST 11: triangle-triangle 평지 수렴 + 면-기준 평형')
print('=' * 55)
p_tt = make_p('triangle', 'triangle', brk_v=0.360)
xa2, x_t2, y_raw2, y_env2, R2 = flat_sim(p_tt)
check(R2['fail_rate'] < 0.05, f'tri-tri 평지 수렴 (실패율 {R2["fail_rate"]*100:.1f}%)')
D2 = calc_dynamics_gpu(R2, xa2, x_t2, y_env2, dict(p_tt))
sumN2 = (D2['Nr'] + D2['Nm'] + D2['Nf'])
cruise2 = (np.abs(D2['a_long']) < 1e-9) & R2['ok']
mean2 = float(np.nanmean(sumN2[cruise2]))
# 삼각형 경로는 평지 CG 경로의 소량 수직관성으로 frame보다 편차가 큼(기존 동작).
# 핵심 불변식은 "전체 mg가 아니라 면-기준(절반) 스케일"임 — 전체-W 회귀를 검출.
check(abs(mean2 - target_side) / target_side < 0.20,
      f'tri-tri ΣN ~ 0.5·mg ({mean2:.1f}N vs {target_side:.1f}N, 오차<20%)')
check(mean2 < 0.70 * target_full,
      f'tri-tri ΣN 이 전체 mg({target_full:.1f}N) 대비 명백히 낮음 → 면-기준 확인')
print()

print('=' * 55)
print(f'=== 전체 {_PASS}개 검증 통과 ===')
print('=' * 55)
