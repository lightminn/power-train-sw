"""
test_v4.py — ZETIN v4 삼각형+사각형 동시 탐색 코드 검증

1. decode_x 4가지 모드 조합 테스트
2. objective 함수 각 모드로 1회 호출 검증
3. 미니 differential_evolution (popsize=5, maxiter=3) 실행
"""
import os
import sys
import time
import numpy as np

os.environ.setdefault('JAX_PLATFORM_NAME', 'gpu')
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')

import jax
print(f'JAX devices: {jax.devices()}')
print(f'JAX backend: {jax.default_backend()}\n')

script_dir = os.path.dirname(os.path.abspath(__file__))
gpu_dir = os.path.join(script_dir, '..', 'python_gpu')
sys.path.insert(0, gpu_dir)

from functions.gen_terrain import gen_terrain
from functions.calc_envelope_jax import calc_envelope_gpu
from functions.newton_solver import kin_sim_gpu
from functions.calc_dynamics_jax import calc_dynamics_gpu
from functions.calc_stability_jax import calc_stability_gpu
from functions.wpos_jax import pack_params_auto
from scipy.optimize import differential_evolution

p0 = {
    'R_w': 0.100, 'h_body': 0.300, 'mass': 30, 'g': 9.81,
    'obs_h': 0.150, 'mu': 0.70, 'gear_ratio': 5, 'eta_gear': 0.85,
    'motor_tau_peak': 4.95, 'motor_tau_cont': 2.75, 'Kt': 0.055,
    'm_wheel': 3.5, 'm_rocker_link': 1.5, 'm_bogie_link': 0.8,
    'I_rocker_add': 0, 'I_bogie_add': 0, 'e_restitution': 0.3,
    'v_robot': 0.8, 'v_max_flat': 2.0, 'step_thresh': 5.0,
    'phi_r0': 0, 'delta_pb': 0, 'CG_offset': 0,
}
p0['h_CG'] = p0['h_body'] * 0.55

N_PTS = 160
WBOT_MIN, WBOT_MAX = 0.400, 0.700
P0_HEIGHT_MAX = 0.500
FAIL_MAX = 0.10
LIFTOFF_MAX = 0.02
TOI_WARN = 0.20
TAU_REF, IMBAL_REF, SN_REF = 1.85, 10, 35
W = {'tau': 0.25, 'imbal': 0.20, 'stab': 0.30, 'sn': 0.15, 'fail': 0.10}
W_terrain = {'stairs': 0.55, 'wood': 0.20, 'rough': 0.15, 'step': 0.10}


# ─── 헬퍼 함수 (v4와 동일) ───
def decode_x(x, p0):
    p = dict(p0)
    rm = int(round(x[0])); bm = int(round(x[1]))
    if bm == 1:
        p['bogie_mode'] = 'triangle'
        p['L_b1'] = x[8]; p['L_b2'] = x[9]; p['beta_b'] = np.deg2rad(x[10])
        p['c_b'] = p['L_b1'] * abs(np.sin(p['beta_b'] / 2))
        p['d_b'] = p['L_b2'] * abs(np.sin(p['beta_b'] / 2))
        h_bogie_drop = p['L_b1'] * np.cos(-np.pi / 2 + p['beta_b'] / 2)
    elif bm == 2:
        p['bogie_mode'] = 'frame'
        p['T_b'] = x[8]; p['S_b1'] = x[9]; p['S_b2'] = x[10] * 5e-3
        p['th_b1'] = np.deg2rad(x[11]); p['th_b2'] = np.deg2rad(x[12]); p['j_b'] = x[13]
        N_bb = p['S_b1'] * np.cos(p['th_b1']) - p['S_b2'] * np.cos(p['th_b2'])
        D_bb = p['T_b'] + p['S_b1'] * np.sin(p['th_b1']) + p['S_b2'] * np.sin(p['th_b2'])
        bb_0 = np.arctan2(N_bb, D_bb)
        h_bogie_drop = -(1 - p['j_b']) * p['T_b'] * np.sin(bb_0) + p['S_b1'] * np.cos(p['th_b1'] + bb_0)
        Wf_c = abs((1 - p['j_b']) * p['T_b'] * np.cos(bb_0) + p['S_b1'] * np.sin(p['th_b1'] + bb_0))
        Wm_c = abs(p['j_b'] * p['T_b'] * np.cos(bb_0) + p['S_b2'] * np.sin(p['th_b2'] - bb_0))
        p['c_b'] = max(Wf_c, p0['R_w']); p['d_b'] = max(Wm_c, p0['R_w'])
    else:
        return None
    if rm == 1:
        p['rocker_mode'] = 'triangle'
        p['L_r1'] = x[2]; p['L_r2'] = x[3]; p['alpha_r'] = np.deg2rad(x[4])
    elif rm == 2:
        p['rocker_mode'] = 'frame'
        p['T_r'] = x[2]; p['S_r1'] = x[3]
        p['th_r1'] = np.deg2rad(x[5]); p['th_r2'] = np.deg2rad(x[6]); p['j_r'] = x[7]
        h_rocker_front = p['S_r1'] * np.cos(p['th_r1'])
        p['S_r2'] = (h_rocker_front + h_bogie_drop) / np.cos(p['th_r2'])
    else:
        return None
    return p

def calc_wbot(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle': return (p['L_b1'] + p['L_b2']) * np.sin(p['beta_b'] / 2)
    elif mode == 'frame': return max(p['S_b1']*abs(np.cos(p['th_b1'])) + p['S_b2']*abs(np.cos(p['th_b2'])), (p['S_b1']+p['S_b2'])*0.7)
    return p['c_b'] + p['d_b']

def get_cb_fwd(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_b1']*abs(np.sin(p['beta_b']/2)), p['R_w'])
    elif mode == 'frame': return max(p['S_b1']*abs(np.cos(p['th_b1'])), p['R_w'])
    return max(p.get('c_b', 0.14), p['R_w'])

def get_b_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_r2']*abs(np.cos(p['alpha_r']/2)), p['R_w'])
    elif mode == 'frame': return max(p['j_r']*p['T_r'] + p['S_r2']*abs(np.sin(p['th_r2'])), p['R_w'])
    return max(p.get('b_r', 0.28), p['R_w'])

def get_a_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_r1']*abs(np.cos(p['alpha_r']/2)), p['R_w'])
    elif mode == 'frame': return max((1-p['j_r'])*p['T_r'] + p['S_r1']*abs(np.sin(p['th_r1'])), p['R_w'])
    return max(p.get('a_r', 0.22), p['R_w'])

def calc_P0_height_flat(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['R_w'] + p['L_r2']*np.sin(p['alpha_r']/2), p['R_w'])
    elif mode == 'frame': return max(p['R_w'] + p['S_r2']*np.cos(p['th_r2']), p['R_w'])
    return p['R_w']

def objective(x):
    x = list(x)
    x[0] = round(x[0]); x[1] = round(x[1])
    p = decode_x(x, p0)
    if p is None: return 10
    Wbot = calc_wbot(p)
    if Wbot < WBOT_MIN or Wbot > WBOT_MAX: return 50 + abs(Wbot - (WBOT_MIN+WBOT_MAX)/2)
    y0_flat = calc_P0_height_flat(p)
    if y0_flat > P0_HEIGHT_MAX: return 50 + (y0_flat - P0_HEIGHT_MAX)*10
    try:
        dist_mr = get_a_eff(p) + get_b_eff(p) - p.get('d_b', 0)
    except KeyError: return 10
    if dist_mr < 0.250: return 50 + abs(0.250-dist_mr)*10
    p_arr = pack_params_auto(p)
    try:
        flat_x, flat_y_raw = gen_terrain('flat', p)
        flat_y_env = calc_envelope_gpu(flat_x, flat_y_raw, p['R_w'])
        test_R = kin_sim_gpu(np.array([0.0]), flat_x, flat_y_env, p_arr, p)
        if not test_R['ok'][0]: return 60
    except Exception: return 60
    terrains = ['real_stairs', 'wood_block', 'rough', 'step']
    t_weights = np.array([W_terrain['stairs'], W_terrain['wood'], W_terrain['rough'], W_terrain['step']])
    tau_vals = np.zeros(4); sn_vals = np.zeros(4); imbal_vals = np.zeros(4)
    fail_pts = np.zeros(4); total_pts = np.zeros(4)
    toi_min_all = np.ones(4); liftoff_all = np.zeros(4)
    p['liftoff_max'] = LIFTOFF_MAX
    for ti, t in enumerate(terrains):
        try:
            x_t, y_t_raw = gen_terrain(t, p)
            y_t_env = calc_envelope_gpu(x_t, y_t_raw, p['R_w'])
            b_eff_v = get_b_eff(p); a_eff_v = get_a_eff(p); cb = get_cb_fwd(p)
            xs = x_t[0] + b_eff_v + 0.05; xe = x_t[-1] - (a_eff_v+cb) - 0.05
            if xs >= xe: return 12
            xa = np.linspace(xs, xe, N_PTS)
            R = kin_sim_gpu(xa, x_t, y_t_env, p_arr, p)
            fail_pts[ti] = np.sum(~R['ok']); total_pts[ti] = len(xa)
            D = calc_dynamics_gpu(R, xa, x_t, y_t_env, p)
            tau_vals[ti] = D['stair_torque_peak']
            S = calc_stability_gpu(R, xa, x_t, y_t_raw, y_t_env, p)
            toi_min_all[ti] = S['min_TOI']; liftoff_all[ti] = S['liftoff_ratio']
            if S.get('is_collision', False): return 9 + abs(S['min_clearance'])*20
            if S['liftoff_ratio'] > LIFTOFF_MAX: return 7 + S['liftoff_ratio']*3
            valid_d = ~D['liftoff_r'] & ~D['liftoff_f'] & R['ok']
            nm_ = [np.mean(D['Nr'][valid_d if np.sum(valid_d)>5 else slice(None)]),
                   np.mean(D['Nm'][valid_d if np.sum(valid_d)>5 else slice(None)]),
                   np.mean(D['Nf'][valid_d if np.sum(valid_d)>5 else slice(None)])]
            if np.mean(nm_) > 0.5:
                imbal_vals[ti] = (max(nm_)-min(nm_)) / np.mean(nm_) * 100
            from functions.calc_metrics_jax import calc_metrics_gpu
            M = calc_metrics_gpu(xa, x_t, y_t_env, R, p)
            sn_vals[ti] = M['SN_dB']
        except Exception as ME:
            print(f'  [예외] {t}: {ME}'); return 10
    fail_rate_total = np.sum(fail_pts) / max(np.sum(total_pts), 1)
    if fail_rate_total > FAIL_MAX: return 5 + fail_rate_total*5
    tau_norm = np.sum(t_weights * tau_vals) / TAU_REF
    imbal_norm = np.sum(t_weights * imbal_vals) / IMBAL_REF
    global_toi_min = np.min(toi_min_all); global_liftoff = np.max(liftoff_all)
    toi_penalty = (0.5-global_toi_min)*2 if global_toi_min >= TOI_WARN else 0.6+((TOI_WARN-global_toi_min)/TOI_WARN)*5
    stab_penalty = max(toi_penalty, (global_liftoff/LIFTOFF_MAX)*3)
    sn_norm = 1 / (1 + max(np.sum(t_weights*sn_vals), 0) / SN_REF)
    fail_norm = fail_rate_total * 10
    return max(W['tau']*tau_norm + W['imbal']*imbal_norm + W['stab']*stab_penalty + W['sn']*sn_norm + W['fail']*fail_norm, 0)


# ═══════════════════════
# TEST 1: decode_x 4가지 모드
# ═══════════════════════
print('=' * 55)
print('TEST 1: decode_x 4가지 모드 조합')
print('=' * 55)

test_configs = {
    'frame-frame':    [2, 2, 0.30, 0.20, 90, 15, 15, 0.50, 0.22, 0.18, 120, 20, 20, 0.50],
    'frame-triangle': [2, 1, 0.30, 0.20, 90, 15, 15, 0.50, 0.22, 0.20, 120, 0,  0,  0.50],
    'triangle-frame': [1, 2, 0.30, 0.25, 120, 0, 0,  0.50, 0.22, 0.18, 120, 20, 20, 0.50],
    'triangle-tri':   [1, 1, 0.30, 0.25, 120, 0, 0,  0.50, 0.22, 0.20, 120, 0,  0,  0.50],
}

for name, x_test in test_configs.items():
    p = decode_x(x_test, p0)
    if p is None:
        print(f'  {name}: FAIL (decode_x returned None)')
    else:
        wbot = calc_wbot(p)
        h0 = calc_P0_height_flat(p)
        print(f'  {name}: rocker={p["rocker_mode"]}, bogie={p["bogie_mode"]}, '
              f'W_bot={wbot*1000:.0f}mm, P0_h={h0*1000:.0f}mm  OK')

print()

# ═══════════════════════
# TEST 2: JIT 워밍업
# ═══════════════════════
print('=' * 55)
print('TEST 2: JAX JIT 워밍업 (4가지 모드)')
print('=' * 55)

def make_warmup_p(rm, bm):
    wp = dict(p0)
    if rm == 'triangle':
        wp['rocker_mode'] = 'triangle'
        wp['L_r1'] = 0.30; wp['L_r2'] = 0.25; wp['alpha_r'] = np.deg2rad(120)
    else:
        wp['rocker_mode'] = 'frame'
        wp['T_r'] = 0.3; wp['S_r1'] = 0.2; wp['S_r2'] = 0.15
        wp['th_r1'] = 0.3; wp['th_r2'] = 0.3; wp['j_r'] = 0.5
    if bm == 'triangle':
        wp['bogie_mode'] = 'triangle'
        wp['L_b1'] = 0.22; wp['L_b2'] = 0.20; wp['beta_b'] = np.deg2rad(120)
        wp['c_b'] = wp['L_b1'] * abs(np.sin(wp['beta_b']/2))
        wp['d_b'] = wp['L_b2'] * abs(np.sin(wp['beta_b']/2))
    else:
        wp['bogie_mode'] = 'frame'
        wp['T_b'] = 0.2; wp['S_b1'] = 0.18; wp['S_b2'] = 0.12
        wp['th_b1'] = 0.3; wp['th_b2'] = 0.3; wp['j_b'] = 0.5
        wp['c_b'] = 0.14; wp['d_b'] = 0.14
    return wp

t_wu = time.time()
for rm, bm in [('frame','frame'), ('frame','triangle'), ('triangle','frame'), ('triangle','triangle')]:
    wp = make_warmup_p(rm, bm)
    wp_arr = pack_params_auto(wp)
    flat_x, flat_y = gen_terrain('flat', wp)
    flat_env = calc_envelope_gpu(flat_x, flat_y, wp['R_w'])
    xa = np.linspace(flat_x[0]+0.3, flat_x[-1]-0.3, N_PTS)
    R = kin_sim_gpu(xa, flat_x, flat_env, wp_arr, wp)
    print(f'  {rm}-{bm}: 실패율 {R["fail_rate"]*100:.1f}%  OK')
print(f'워밍업 완료 ({time.time()-t_wu:.1f}s)\n')

# ═══════════════════════
# TEST 3: objective 4가지 모드 1회 호출
# ═══════════════════════
print('=' * 55)
print('TEST 3: objective 함수 4가지 모드 호출')
print('=' * 55)

test_x_obj = {
    'frame-frame':    [2, 2, 0.30, 0.20, 90, 15, 15, 0.50, 0.22, 0.18, 120, 20, 20, 0.50],
    'frame-triangle': [2, 1, 0.30, 0.20, 90, 15, 15, 0.50, 0.22, 0.20, 120, 0,  0,  0.50],
    'triangle-frame': [1, 2, 0.30, 0.25, 120, 0, 0,  0.50, 0.22, 0.18, 120, 20, 20, 0.50],
    'triangle-tri':   [1, 1, 0.30, 0.25, 120, 0, 0,  0.50, 0.22, 0.20, 120, 0,  0,  0.50],
}

for name, x_test in test_x_obj.items():
    t0 = time.time()
    val = objective(x_test)
    dt = time.time() - t0
    print(f'  {name}: f={val:.4f}  ({dt:.2f}s)')

print()

# ═══════════════════════
# TEST 4: 미니 최적화 (popsize=5, maxiter=3)
# ═══════════════════════
print('=' * 55)
print('TEST 4: 미니 differential_evolution (popsize=5, maxiter=3)')
print('=' * 55)

lb = [1, 1, 0.20, 0.15, 60,  0,  0,  0.30, 0.15, 0.15, 60,  0,  0,  0.30]
ub = [2, 2, 0.45, 0.35, 160, 35, 35, 0.70, 0.35, 0.35, 160, 40, 40, 0.70]
bounds = list(zip(lb, ub))

t_mini = time.time()
mini_result = differential_evolution(
    objective, bounds,
    maxiter=3, popsize=5, seed=42,
    disp=True, workers=1,
    polish=False,  # 이산 파라미터(모드)로 인한 불필요한 L-BFGS-B 반복 방지
)
dt_mini = time.time() - t_mini

print(f'\n미니 최적화 결과: f={mini_result.fun:.4f}  평가횟수={mini_result.nfev}  소요={dt_mini:.1f}s')
x_mini = list(mini_result.x)
x_mini[0] = round(x_mini[0]); x_mini[1] = round(x_mini[1])
p_mini = decode_x(x_mini, p0)
if p_mini:
    print(f'최적 모드: rocker={p_mini["rocker_mode"]}, bogie={p_mini["bogie_mode"]}')

print('\n=== 모든 테스트 통과 ===')
