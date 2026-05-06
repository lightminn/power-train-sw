"""
ZETIN_JointOptSearch_v4_gpu.py
Rocker x Bogie 전체 파라미터 최적 탐색 — 삼각형/사각형 동시 탐색 버전

v3 대비 변경사항:
  - x[0] (rocker_mode): 1=triangle, 2=frame  (기존: 2로 고정 → 이제 자유 탐색)
  - x[1] (bogie_mode):  1=triangle, 2=frame  (기존: 2로 고정 → 이제 자유 탐색)
  - 4가지 모드 조합 (frame-frame, frame-tri, tri-frame, tri-tri) 동시 탐색
  - JAX lax.switch가 모든 모드 분기를 단일 컴파일로 처리

가속 전략 (v3과 동일):
  1. kin_sim: JAX vmap + 배치 Newton 솔버 (GPU 병렬 160포인트 동시)
  2. calc_dynamics: numpy 벡터 연산
  3. calc_stability: 완전 벡터화
  4. 최적화: scipy differential_evolution
"""
import os
import sys
import time
import pickle
import numpy as np

# JAX GPU 설정 (반드시 import 전에)
os.environ.setdefault('JAX_PLATFORM_NAME', 'gpu')
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')

import jax
import jax.numpy as jnp

print(f'JAX devices: {jax.devices()}')
print(f'JAX default backend: {jax.default_backend()}')

# python_gpu/functions/ 공유 사용 (코드 중복 없이)
script_dir = os.path.dirname(os.path.abspath(__file__))
gpu_dir = os.path.join(script_dir, '..', 'python_gpu')
sys.path.insert(0, gpu_dir)

from functions.gen_terrain import gen_terrain
from functions.calc_envelope_jax import calc_envelope_gpu
from functions.newton_solver import kin_sim_gpu
from functions.calc_dynamics_jax import calc_dynamics_gpu
from functions.calc_stability_jax import calc_stability_gpu
from functions.calc_metrics_jax import calc_metrics_gpu
from functions.wpos_jax import pack_params_auto, wpos_jax

from scipy.optimize import differential_evolution

# ═══════════════════════════════════════
np.random.seed(2026)

# ═══════════════════════════════════════
# [SECTION 1] 공통 파라미터 (v3과 동일)
# ═══════════════════════════════════════
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

# ═══════════════════════════════════════
# [SECTION 2] 목적함수 설계 상수 (v3과 동일)
# ═══════════════════════════════════════
TAU_REF = 1.85
IMBAL_REF = 10
SN_REF = 35
WBOT_MIN = 0.400
WBOT_MAX = 0.700
FAIL_MAX = 0.10
LIFTOFF_MAX = 0.02
TOI_WARN = 0.20
P0_HEIGHT_MAX = 0.500

W = {'tau': 0.25, 'imbal': 0.20, 'stab': 0.30, 'sn': 0.15, 'fail': 0.10}
W_terrain = {'stairs': 0.55, 'wood': 0.20, 'rough': 0.15, 'step': 0.10}
N_PTS = 160

print('=== ZETIN GPU 가속 최적화 v4 (삼각형+사각형 동시 탐색) ===')
print(f'JAX backend: {jax.default_backend()}')
print(f'스펙: R_w={p0["R_w"]*1000:.0f}mm  mass={p0["mass"]:.0f}kg')
print(f'TAU_REF={TAU_REF:.2f}Nm  WBOT=[{WBOT_MIN*1000:.0f}~{WBOT_MAX*1000:.0f}]mm')
print(f'탐색 모드: Rocker={{triangle, frame}}  Bogie={{triangle, frame}}\n')


# ═══════════════════════════════════════
# [SECTION 5] 헬퍼 함수 (v3과 동일)
# ═══════════════════════════════════════

def decode_x(x, p0):
    p = dict(p0)
    rm = int(round(x[0]))
    bm = int(round(x[1]))

    if bm == 1:
        p['bogie_mode'] = 'triangle'
        p['L_b1'] = x[8]; p['L_b2'] = x[9]
        p['beta_b'] = np.deg2rad(x[10])
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
    if mode == 'triangle':
        return (p['L_b1'] + p['L_b2']) * np.sin(p['beta_b'] / 2)
    elif mode == 'frame':
        return max(p['S_b1'] * abs(np.cos(p['th_b1'])) + p['S_b2'] * abs(np.cos(p['th_b2'])),
                   (p['S_b1'] + p['S_b2']) * 0.7)
    return p['c_b'] + p['d_b']


def get_cb_fwd(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_b1'] * abs(np.sin(p['beta_b'] / 2)), p['R_w'])
    elif mode == 'frame': return max(p['S_b1'] * abs(np.cos(p['th_b1'])), p['R_w'])
    return max(p.get('c_b', 0.14), p['R_w'])


def get_b_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_r2'] * abs(np.cos(p['alpha_r'] / 2)), p['R_w'])
    elif mode == 'frame': return max(p['j_r'] * p['T_r'] + p['S_r2'] * abs(np.sin(p['th_r2'])), p['R_w'])
    return max(p.get('b_r', 0.28), p['R_w'])


def get_a_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_r1'] * abs(np.cos(p['alpha_r'] / 2)), p['R_w'])
    elif mode == 'frame': return max((1 - p['j_r']) * p['T_r'] + p['S_r1'] * abs(np.sin(p['th_r1'])), p['R_w'])
    return max(p.get('a_r', 0.22), p['R_w'])


def calc_P0_height_flat(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['R_w'] + p['L_r2'] * np.sin(p['alpha_r'] / 2), p['R_w'])
    elif mode == 'frame': return max(p['R_w'] + p['S_r2'] * np.cos(p['th_r2']), p['R_w'])
    return p['R_w']


# ═══════════════════════════════════════
# JIT 워밍업 — 4가지 모드 조합 모두 컴파일
# lax.switch는 단일 컴파일로 모든 분기를 처리하지만
# newton_solver의 초기값 계산 경로 확인을 위해 전체 워밍업 수행
# ═══════════════════════════════════════
def _make_warmup_p(rocker_mode, bogie_mode):
    wp = dict(p0)
    if rocker_mode == 'triangle':
        wp['rocker_mode'] = 'triangle'
        wp['L_r1'] = 0.30; wp['L_r2'] = 0.25; wp['alpha_r'] = np.deg2rad(120)
    else:
        wp['rocker_mode'] = 'frame'
        wp['T_r'] = 0.3; wp['S_r1'] = 0.2; wp['S_r2'] = 0.15
        wp['th_r1'] = 0.3; wp['th_r2'] = 0.3; wp['j_r'] = 0.5
    if bogie_mode == 'triangle':
        wp['bogie_mode'] = 'triangle'
        wp['L_b1'] = 0.22; wp['L_b2'] = 0.20; wp['beta_b'] = np.deg2rad(120)
        wp['c_b'] = wp['L_b1'] * abs(np.sin(wp['beta_b'] / 2))
        wp['d_b'] = wp['L_b2'] * abs(np.sin(wp['beta_b'] / 2))
    else:
        wp['bogie_mode'] = 'frame'
        wp['T_b'] = 0.2; wp['S_b1'] = 0.18; wp['S_b2'] = 0.12
        wp['th_b1'] = 0.3; wp['th_b2'] = 0.3; wp['j_b'] = 0.5
        wp['c_b'] = 0.14; wp['d_b'] = 0.14
    return wp


print('JAX JIT 워밍업 중 (4가지 모드 조합 컴파일, 2-3분 소요)...')
_t_warmup = time.time()

_mode_combos = [
    ('frame', 'frame'),
    ('frame', 'triangle'),
    ('triangle', 'frame'),
    ('triangle', 'triangle'),
]

for _rm, _bm in _mode_combos:
    _wp = _make_warmup_p(_rm, _bm)
    _wp_arr = pack_params_auto(_wp)
    _flat_x, _flat_y = gen_terrain('flat', _wp)
    _flat_env = calc_envelope_gpu(_flat_x, _flat_y, _wp['R_w'])
    _warmup_xa = np.linspace(_flat_x[0] + 0.3, _flat_x[-1] - 0.3, N_PTS)
    _R_w = kin_sim_gpu(_warmup_xa, _flat_x, _flat_env, _wp_arr, _wp)
    print(f'  {_rm}-{_bm}: 실패율 {_R_w["fail_rate"]*100:.1f}%')

print(f'JAX 워밍업 완료! ({time.time()-_t_warmup:.1f}초)\n')


# ═══════════════════════════════════════
# [SECTION 4] 목적함수 (v3과 동일, 모드 변수화)
# ═══════════════════════════════════════

def objective(x):
    x = list(x)
    x[0] = round(x[0]); x[1] = round(x[1])

    p = decode_x(x, p0)
    if p is None: return 10

    Wbot = calc_wbot(p)
    if Wbot < WBOT_MIN or Wbot > WBOT_MAX:
        return 50 + abs(Wbot - (WBOT_MIN + WBOT_MAX) / 2)

    y0_flat = calc_P0_height_flat(p)
    if y0_flat > P0_HEIGHT_MAX:
        return 50 + (y0_flat - P0_HEIGHT_MAX) * 10

    try:
        dist_mr = get_a_eff(p) + get_b_eff(p) - p.get('d_b', 0)
    except KeyError:
        return 10
    if dist_mr < 0.250:
        return 50 + abs(0.250 - dist_mr) * 10

    p_arr = pack_params_auto(p)

    # 평지 테스트 (GPU)
    try:
        flat_x, flat_y_raw = gen_terrain('flat', p)
        flat_y_env = calc_envelope_gpu(flat_x, flat_y_raw, p['R_w'])
        test_R = kin_sim_gpu(np.array([0.0]), flat_x, flat_y_env, p_arr, p)
        if not test_R['ok'][0]:
            return 60
    except Exception:
        return 60

    # 4종 지형 평가 (GPU 가속)
    terrains = ['real_stairs', 'wood_block', 'rough', 'step']
    t_weights = np.array([W_terrain['stairs'], W_terrain['wood'], W_terrain['rough'], W_terrain['step']])

    tau_vals = np.zeros(4)
    sn_vals = np.zeros(4)
    imbal_vals = np.zeros(4)
    fail_pts = np.zeros(4)
    total_pts = np.zeros(4)
    toi_min_all = np.ones(4)
    liftoff_all = np.zeros(4)

    p['liftoff_max'] = LIFTOFF_MAX

    for ti, t in enumerate(terrains):
        try:
            x_t, y_t_raw = gen_terrain(t, p)
            y_t_env = calc_envelope_gpu(x_t, y_t_raw, p['R_w'])

            b_eff_v = get_b_eff(p); a_eff_v = get_a_eff(p); cb = get_cb_fwd(p)
            xs = x_t[0] + b_eff_v + 0.05
            xe = x_t[-1] - (a_eff_v + cb) - 0.05
            if xs >= xe: return 12
            xa = np.linspace(xs, xe, N_PTS)

            R = kin_sim_gpu(xa, x_t, y_t_env, p_arr, p)
            fail_pts[ti] = np.sum(~R['ok'])
            total_pts[ti] = len(xa)

            D = calc_dynamics_gpu(R, xa, x_t, y_t_env, p)
            tau_vals[ti] = D['stair_torque_peak']

            S = calc_stability_gpu(R, xa, x_t, y_t_raw, y_t_env, p)
            toi_min_all[ti] = S['min_TOI']
            liftoff_all[ti] = S['liftoff_ratio']

            if S.get('is_collision', False):
                return 9 + abs(S['min_clearance']) * 20
            if S['liftoff_ratio'] > LIFTOFF_MAX:
                return 7 + S['liftoff_ratio'] * 3

            valid_d = ~D['liftoff_r'] & ~D['liftoff_f'] & R['ok']
            if np.sum(valid_d) > 5:
                nm_ = [np.mean(D['Nr'][valid_d]), np.mean(D['Nm'][valid_d]), np.mean(D['Nf'][valid_d])]
            else:
                nm_ = [np.mean(D['Nr']), np.mean(D['Nm']), np.mean(D['Nf'])]
            if np.mean(nm_) > 0.5:
                imbal_vals[ti] = (max(nm_) - min(nm_)) / np.mean(nm_) * 100

            M = calc_metrics_gpu(xa, x_t, y_t_env, R, p)
            sn_vals[ti] = M['SN_dB']

        except Exception as ME:
            if '수렴 실패' not in str(ME):
                print(f'  [예외] {t}: {ME}')
            return 10

    fail_rate_total = np.sum(fail_pts) / max(np.sum(total_pts), 1)
    if fail_rate_total > FAIL_MAX:
        return 5 + fail_rate_total * 5

    tau_norm = np.sum(t_weights * tau_vals) / TAU_REF
    imbal_norm = np.sum(t_weights * imbal_vals) / IMBAL_REF

    global_toi_min = np.min(toi_min_all)
    global_liftoff = np.max(liftoff_all)

    if global_toi_min >= TOI_WARN:
        toi_penalty = (0.5 - global_toi_min) * 2
    else:
        toi_penalty = 0.6 + ((TOI_WARN - global_toi_min) / TOI_WARN) * 5

    stab_penalty = max(toi_penalty, (global_liftoff / LIFTOFF_MAX) * 3)
    sn_norm = 1 / (1 + max(np.sum(t_weights * sn_vals), 0) / SN_REF)
    fail_norm = fail_rate_total * 10

    f = (W['tau'] * tau_norm + W['imbal'] * imbal_norm
         + W['stab'] * stab_penalty + W['sn'] * sn_norm + W['fail'] * fail_norm)
    return max(f, 0)


# ═══════════════════════════════════════
# [SECTION 3] 탐색 공간
# v4 변경사항: x[0], x[1] 범위를 [1,2]로 개방
#
# x[0] = rocker_mode: 1=triangle, 2=frame
# x[1] = bogie_mode:  1=triangle, 2=frame
#
# 파라미터 슬롯 (모드 무관 공유):
#   x[2:5]  → triangle rocker: L_r1, L_r2, alpha_r(deg)
#           → frame rocker:    T_r,   S_r1, (unused)
#   x[5:8]  → triangle rocker: (unused)
#           → frame rocker:    th_r1(deg), th_r2(deg), j_r
#   x[8:11] → triangle bogie:  L_b1, L_b2, beta_b(deg)
#           → frame bogie:     T_b,  S_b1, S_b2×5mm
#   x[11:14]→ triangle bogie:  (unused)
#           → frame bogie:     th_b1(deg), th_b2(deg), j_b
# ═══════════════════════════════════════
lb = [1, 1, 0.20, 0.15, 60,  0,  0,  0.30, 0.15, 0.15, 60,  0,  0,  0.30]
ub = [2, 2, 0.45, 0.35, 160, 35, 35, 0.70, 0.35, 0.35, 160, 40, 40, 0.70]
bounds = list(zip(lb, ub))

# ═══════════════════════════════════════
# [SECTION 7] 최적화 실행
# ═══════════════════════════════════════
print('differential_evolution 실행 중 (삼각형+사각형 동시 탐색, maxiter=2000)...\n')
tic = time.time()

result = differential_evolution(
    objective, bounds,
    maxiter=2000, popsize=30, tol=1e-4, seed=2026,
    disp=True, workers=1,
    polish=False,  # x[0], x[1]이 이산 파라미터이므로 L-BFGS-B 폴리싱 비활성화
)

x_opt = result.x
f_opt = result.fun
elapsed = time.time() - tic

print(f'\n[최적화 완료] 평가: {result.nfev}  f_opt: {f_opt:.4f}')
print(f'총 소요 시간: {elapsed/60:.1f}분\n')

# ═══════════════════════════════════════
# [SECTION 8] 결과 출력
# ═══════════════════════════════════════
x_opt[0] = round(x_opt[0]); x_opt[1] = round(x_opt[1])
p_opt = decode_x(x_opt, p0)

print('=' * 60)
print('  최적 구조 (v4: 삼각형/사각형 동시 탐색)')
print('-' * 60)
print(f'Rocker mode : {p_opt["rocker_mode"]}')
if p_opt['rocker_mode'] == 'triangle':
    print(f'  L_r1={p_opt["L_r1"]*1000:.1f}mm  L_r2={p_opt["L_r2"]*1000:.1f}mm  '
          f'alpha_r={np.rad2deg(p_opt["alpha_r"]):.1f}deg')
elif p_opt['rocker_mode'] == 'frame':
    print(f'  T_r={p_opt["T_r"]*1000:.1f}mm  S_r1={p_opt["S_r1"]*1000:.1f}mm  S_r2={p_opt["S_r2"]*1000:.1f}mm')
    print(f'  th_r1={np.rad2deg(p_opt["th_r1"]):.1f}deg  th_r2={np.rad2deg(p_opt["th_r2"]):.1f}deg  j_r={p_opt["j_r"]:.2f}')

print(f'Bogie mode  : {p_opt["bogie_mode"]}')
if p_opt['bogie_mode'] == 'triangle':
    Wb = (p_opt['L_b1'] + p_opt['L_b2']) * np.sin(p_opt['beta_b'] / 2)
    print(f'  L_b1={p_opt["L_b1"]*1000:.1f}mm  L_b2={p_opt["L_b2"]*1000:.1f}mm  '
          f'beta_b={np.rad2deg(p_opt["beta_b"]):.1f}deg  W_bot={Wb*1000:.1f}mm')
elif p_opt['bogie_mode'] == 'frame':
    print(f'  T_b={p_opt["T_b"]*1000:.1f}mm  S_b1={p_opt["S_b1"]*1000:.1f}mm  S_b2={p_opt["S_b2"]*1000:.1f}mm')
    print(f'  th_b1={np.rad2deg(p_opt["th_b1"]):.1f}deg  th_b2={np.rad2deg(p_opt["th_b2"]):.1f}deg  j_b={p_opt["j_b"]:.2f}')

print(f'P0 평지높이 : {calc_P0_height_flat(p_opt)*1000:.1f}mm')
print(f'목적함수 값 : {f_opt:.4f}')
print('=' * 60)

# ═══════════════════════════════════════
# [SECTION 10] 저장
# ═══════════════════════════════════════
save_path = os.path.join(script_dir, 'zetin_optimal_params_v4.pkl')
with open(save_path, 'wb') as fh:
    pickle.dump({
        'p_opt': p_opt, 'x_opt': x_opt, 'f_opt': f_opt, 'elapsed': elapsed,
        'lb': lb, 'ub': ub, 'W': W, 'W_terrain': W_terrain,
        'version': 'v4_triangle_search',
    }, fh)
print(f'\n저장: {save_path}')
print(f'=== 완료 ({elapsed/60:.1f}분) ===')
