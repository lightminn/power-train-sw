"""
ZETIN_JointOptSearch_v3.py
Rocker x Bogie 전체 파라미터 최적 탐색 — scipy 기반 v3

MATLAB의 surrogateopt/ga를 scipy.optimize.differential_evolution으로 대체
"""
import os
import sys
import time
import pickle
import numpy as np
from scipy.optimize import differential_evolution

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from functions.gen_terrain import gen_terrain
from functions.calc_envelope import calc_envelope
from functions.kin_sim import kin_sim
from functions.calc_dynamics import calc_dynamics
from functions.calc_stability import calc_stability
from functions.calc_metrics import calc_metrics
from functions.wpos import wpos

# ═══════════════════════════════════════
# 난수 시드 고정
# ═══════════════════════════════════════
np.random.seed(2026)


# ═══════════════════════════════════════
# [SECTION 1] 공통 파라미터
# ═══════════════════════════════════════
p0 = {
    'R_w': 0.100,
    'h_body': 0.300,
    'mass': 30,
    'g': 9.81,
    'obs_h': 0.150,
    'mu': 0.70,
    'gear_ratio': 5,
    'eta_gear': 0.85,
    'motor_tau_peak': 4.95,
    'motor_tau_cont': 2.75,
    'Kt': 0.055,
    'm_wheel': 3.5,
    'm_rocker_link': 1.5,
    'm_bogie_link': 0.8,
    'I_rocker_add': 0,
    'I_bogie_add': 0,
    'e_restitution': 0.3,
    'v_robot': 0.8,
    'v_max_flat': 2.0,
    'step_thresh': 5.0,
    'phi_r0': 0,
    'delta_pb': 0,
    'CG_offset': 0,
}
p0['h_CG'] = p0['h_body'] * 0.55

# ═══════════════════════════════════════
# [SECTION 2] 목적함수 설계 상수
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
RUN_SENSITIVITY = True
N_SENS_PERTURB = 5

print('=== ZETIN Rocker×Bogie differential_evolution v3 (100mm/30kg/D6374) ===')
print(f'스펙: R_w={p0["R_w"]*1000:.0f}mm  mass={p0["mass"]:.0f}kg  v_stair={p0["v_robot"]:.1f}m/s')
print(f'TAU_REF={TAU_REF:.2f}Nm  WBOT=[{WBOT_MIN*1000:.0f}~{WBOT_MAX*1000:.0f}]mm  TOI_warn={TOI_WARN:.2f}')
print(f'가중치: tau={W["tau"]:.2f}  imbal={W["imbal"]:.2f}  stab={W["stab"]:.2f}  SN={W["sn"]:.2f}  fail={W["fail"]:.2f}\n')


# ═══════════════════════════════════════
# [SECTION 5] 헬퍼 함수
# ═══════════════════════════════════════

def decode_x(x, p0):
    p = dict(p0)
    rm = int(round(x[0]))
    bm = int(round(x[1]))

    # Bogie
    if bm == 1:  # triangle
        p['bogie_mode'] = 'triangle'
        p['L_b1'] = x[8]
        p['L_b2'] = x[9]
        p['beta_b'] = np.deg2rad(x[10])
        p['c_b'] = p['L_b1'] * abs(np.sin(p['beta_b'] / 2))
        p['d_b'] = p['L_b2'] * abs(np.sin(p['beta_b'] / 2))
        h_bogie_drop = p['L_b1'] * np.cos(-np.pi / 2 + p['beta_b'] / 2)
    elif bm == 2:  # frame
        p['bogie_mode'] = 'frame'
        p['T_b'] = x[8]
        p['S_b1'] = x[9]
        p['S_b2'] = x[10] * 5e-3
        p['th_b1'] = np.deg2rad(x[11])
        p['th_b2'] = np.deg2rad(x[12])
        p['j_b'] = x[13]

        N_bb = p['S_b1'] * np.cos(p['th_b1']) - p['S_b2'] * np.cos(p['th_b2'])
        D_bb = p['T_b'] + p['S_b1'] * np.sin(p['th_b1']) + p['S_b2'] * np.sin(p['th_b2'])
        bb_0 = np.arctan2(N_bb, D_bb)

        h_bogie_drop = -(1 - p['j_b']) * p['T_b'] * np.sin(bb_0) + p['S_b1'] * np.cos(p['th_b1'] + bb_0)

        Wf_c = abs((1 - p['j_b']) * p['T_b'] * np.cos(bb_0) + p['S_b1'] * np.sin(p['th_b1'] + bb_0))
        Wm_c = abs(p['j_b'] * p['T_b'] * np.cos(bb_0) + p['S_b2'] * np.sin(p['th_b2'] - bb_0))
        p['c_b'] = max(Wf_c, p0['R_w'])
        p['d_b'] = max(Wm_c, p0['R_w'])
    else:
        return None

    # Rocker
    if rm == 1:  # triangle
        p['rocker_mode'] = 'triangle'
        p['L_r1'] = x[2]
        p['L_r2'] = x[3]
        p['alpha_r'] = np.deg2rad(x[4])
    elif rm == 2:  # frame
        p['rocker_mode'] = 'frame'
        p['T_r'] = x[2]
        p['S_r1'] = x[3]
        p['th_r1'] = np.deg2rad(x[5])
        p['th_r2'] = np.deg2rad(x[6])
        p['j_r'] = x[7]

        h_rocker_front = p['S_r1'] * np.cos(p['th_r1'])
        h_total_drop = h_rocker_front + h_bogie_drop
        p['S_r2'] = h_total_drop / np.cos(p['th_r2'])
    else:
        return None

    return p


def calc_wbot(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle':
        return (p['L_b1'] + p['L_b2']) * np.sin(p['beta_b'] / 2)
    elif mode == 'frame':
        Wbot_proj = p['S_b1'] * abs(np.cos(p['th_b1'])) + p['S_b2'] * abs(np.cos(p['th_b2']))
        Wbot_raw = p['S_b1'] + p['S_b2']
        return max(Wbot_proj, Wbot_raw * 0.7)
    else:
        return p['c_b'] + p['d_b']


def get_cb_fwd(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle':
        cb = p['L_b1'] * abs(np.sin(p['beta_b'] / 2))
    elif mode == 'frame':
        cb = p['S_b1'] * abs(np.cos(p['th_b1']))
    else:
        cb = p['c_b']
    return max(cb, p['R_w'])


def get_b_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'linear':
        b = p['b_r']
    elif mode == 'triangle':
        b = p['L_r2'] * abs(np.cos(p['alpha_r'] / 2))
    elif mode == 'frame':
        b = p['j_r'] * p['T_r'] + p['S_r2'] * abs(np.sin(p['th_r2']))
    else:
        b = 0.40
    return max(b, p['R_w'])


def get_a_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'linear':
        a = p['a_r']
    elif mode == 'triangle':
        a = p['L_r1'] * abs(np.cos(p['alpha_r'] / 2))
    elif mode == 'frame':
        a = (1 - p['j_r']) * p['T_r'] + p['S_r1'] * abs(np.sin(p['th_r1']))
    else:
        a = 0.35
    return max(a, p['R_w'])


def calc_P0_height_flat(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle':
        y0 = p['R_w'] + p['L_r2'] * np.sin(p['alpha_r'] / 2)
    elif mode == 'frame':
        y0 = p['R_w'] + p['S_r2'] * np.cos(p['th_r2'])
    else:
        y0 = p['R_w']
    return max(y0, p['R_w'])


# ═══════════════════════════════════════
# [SECTION 4] 목적함수
# ═══════════════════════════════════════

def objective(x):
    # 정수 변수 반올림
    x = list(x)
    x[0] = round(x[0])
    x[1] = round(x[1])

    p = decode_x(x, p0)
    if p is None:
        return 10

    # W_bot 제약
    Wbot = calc_wbot(p)
    if Wbot < WBOT_MIN or Wbot > WBOT_MAX:
        return 50 + abs(Wbot - (WBOT_MIN + WBOT_MAX) / 2)

    # P0 높이 제약
    y0_flat = calc_P0_height_flat(p)
    if y0_flat > P0_HEIGHT_MAX:
        return 50 + (y0_flat - P0_HEIGHT_MAX) * 10

    # 중간-뒷바퀴 간섭 제약
    try:
        dist_mr = get_a_eff(p) + get_b_eff(p) - p.get('d_b', 0)
    except KeyError:
        return 10
    if dist_mr < 0.250:
        return 50 + abs(0.250 - dist_mr) * 10

    # 평지 단위 테스트
    try:
        flat_x, flat_y_raw = gen_terrain('flat', p)
        flat_y_env = calc_envelope(flat_x, flat_y_raw, p['R_w'])
        test_R = kin_sim(np.array([0.0]), flat_x, flat_y_env, p)
        if not test_R['ok'][0]:
            return 60
    except Exception:
        return 60

    # 4종 지형 평가
    terrains = ['real_stairs', 'wood_block', 'rough', 'step']
    t_weights = [W_terrain['stairs'], W_terrain['wood'], W_terrain['rough'], W_terrain['step']]
    n_t = len(terrains)

    tau_vals = np.zeros(n_t)
    sn_vals = np.zeros(n_t)
    imbal_vals = np.zeros(n_t)
    fail_pts = np.zeros(n_t)
    total_pts = np.zeros(n_t)
    toi_min_all = np.ones(n_t)
    liftoff_all = np.zeros(n_t)

    p['liftoff_max'] = LIFTOFF_MAX

    for ti in range(n_t):
        t = terrains[ti]
        try:
            x_t, y_t_raw = gen_terrain(t, p)
            y_t_env = calc_envelope(x_t, y_t_raw, p['R_w'])

            b_eff_v = get_b_eff(p)
            a_eff_v = get_a_eff(p)
            cb = get_cb_fwd(p)
            xs = x_t[0] + b_eff_v + 0.05
            xe = x_t[-1] - (a_eff_v + cb) - 0.05
            if xs >= xe:
                return 12
            xa = np.linspace(xs, xe, N_PTS)

            R = kin_sim(xa, x_t, y_t_env, p)
            fail_pts[ti] = np.sum(~R['ok'])
            total_pts[ti] = len(xa)

            D = calc_dynamics(R, xa, x_t, y_t_env, p)
            tau_vals[ti] = D['stair_torque_peak']

            S = calc_stability(R, xa, x_t, y_t_raw, y_t_env, p)
            toi_min_all[ti] = S['min_TOI']
            liftoff_all[ti] = S['liftoff_ratio']

            if S.get('is_collision', False):
                return 9 + abs(S['min_clearance']) * 20

            if S['liftoff_ratio'] > LIFTOFF_MAX:
                return 7 + S['liftoff_ratio'] * 3

            # 편중도
            valid_d = ~D['liftoff_r'] & ~D['liftoff_f'] & R['ok']
            if np.sum(valid_d) > 5:
                nm_ = [np.mean(D['Nr'][valid_d]), np.mean(D['Nm'][valid_d]), np.mean(D['Nf'][valid_d])]
            else:
                nm_ = [np.mean(D['Nr']), np.mean(D['Nm']), np.mean(D['Nf'])]
            if np.mean(nm_) > 0.5:
                imbal_vals[ti] = (max(nm_) - min(nm_)) / np.mean(nm_) * 100

            M = calc_metrics(xa, x_t, y_t_env, R, p)
            sn_vals[ti] = M['SN_dB']

        except Exception as ME:
            if '수렴 실패' not in str(ME):
                print(f'  [예외] {t}: {ME}')
            return 10

    # 전체 지형 통합 fail_rate
    fail_rate_total = np.sum(fail_pts) / max(np.sum(total_pts), 1)
    if fail_rate_total > FAIL_MAX:
        return 5 + fail_rate_total * 5

    # 각 항 정규화
    tau_weighted = np.sum(np.array(t_weights) * tau_vals)
    tau_norm = tau_weighted / TAU_REF

    imbal_weighted = np.sum(np.array(t_weights) * imbal_vals)
    imbal_norm = imbal_weighted / IMBAL_REF

    global_toi_min = np.min(toi_min_all)
    global_liftoff = np.max(liftoff_all)

    if global_toi_min >= TOI_WARN:
        toi_penalty = (0.5 - global_toi_min) * 2
    else:
        toi_penalty = 0.6 + ((TOI_WARN - global_toi_min) / TOI_WARN) * 5

    liftoff_penalty = (global_liftoff / LIFTOFF_MAX) * 3
    stab_penalty = max(toi_penalty, liftoff_penalty)

    sn_mean = np.sum(np.array(t_weights) * sn_vals)
    sn_norm = 1 / (1 + max(sn_mean, 0) / SN_REF)

    fail_norm = fail_rate_total * 10

    f = (W['tau'] * tau_norm
         + W['imbal'] * imbal_norm
         + W['stab'] * stab_penalty
         + W['sn'] * sn_norm
         + W['fail'] * fail_norm)

    return max(f, 0)


# ═══════════════════════════════════════
# [SECTION 3] 탐색 공간 경계 (14차원)
# ═══════════════════════════════════════
lb = [2, 2,
      0.20, 0.15, 60, 0, 0, 0.30,
      0.15, 0.15, 60, 0, 0, 0.30]
ub = [2, 2,
      0.45, 0.35, 160, 35, 35, 0.70,
      0.35, 0.25, 160, 40, 40, 0.70]

bounds = list(zip(lb, ub))

# ═══════════════════════════════════════
# [SECTION 7] 최적화 실행
# ═══════════════════════════════════════
print('differential_evolution 실행 중 (maxiter=200, popsize=30)...\n')
tic = time.time()

result = differential_evolution(
    objective,
    bounds,
    maxiter=200,
    popsize=30,
    tol=1e-4,
    seed=2026,
    disp=True,
    workers=1,  # set to -1 for parallel (requires __main__ guard)
)

x_opt = result.x
f_opt = result.fun
elapsed = time.time() - tic

print(f'\n[최적화 완료] 평가: {result.nfev}  f_opt: {f_opt:.4f}')
print(f'총 소요 시간: {elapsed/60:.1f}분\n')

# ═══════════════════════════════════════
# [SECTION 8] 최적 파라미터 복원 및 출력
# ═══════════════════════════════════════
x_opt[0] = round(x_opt[0])
x_opt[1] = round(x_opt[1])
p_opt = decode_x(x_opt, p0)

print('=' * 60)
print('  최적 구조')
print('-' * 60)
print(f'Rocker mode : {p_opt["rocker_mode"]}')
if p_opt['rocker_mode'] == 'triangle':
    print(f'  L_r1={p_opt["L_r1"]*1000:.1f}mm  L_r2={p_opt["L_r2"]*1000:.1f}mm  '
          f'alpha_r={np.rad2deg(p_opt["alpha_r"]):.1f}deg')
elif p_opt['rocker_mode'] == 'frame':
    print(f'  T_r={p_opt["T_r"]*1000:.1f}mm  S_r1={p_opt["S_r1"]*1000:.1f}mm  '
          f'S_r2={p_opt["S_r2"]*1000:.1f}mm')
    print(f'  th_r1={np.rad2deg(p_opt["th_r1"]):.1f}deg  '
          f'th_r2={np.rad2deg(p_opt["th_r2"]):.1f}deg  j_r={p_opt["j_r"]:.2f}')

print(f'Bogie mode  : {p_opt["bogie_mode"]}')
if p_opt['bogie_mode'] == 'triangle':
    Wb = (p_opt['L_b1'] + p_opt['L_b2']) * np.sin(p_opt['beta_b'] / 2)
    print(f'  L_b1={p_opt["L_b1"]*1000:.1f}mm  L_b2={p_opt["L_b2"]*1000:.1f}mm  '
          f'beta_b={np.rad2deg(p_opt["beta_b"]):.1f}deg  W_bot={Wb*1000:.1f}mm')
elif p_opt['bogie_mode'] == 'frame':
    Wf_ = p_opt['S_b1'] * abs(np.cos(p_opt['th_b1']))
    Wm_ = p_opt['S_b2'] * abs(np.cos(p_opt['th_b2']))
    print(f'  T_b={p_opt["T_b"]*1000:.1f}mm  S_b1={p_opt["S_b1"]*1000:.1f}mm  '
          f'S_b2={p_opt["S_b2"]*1000:.1f}mm')
    print(f'  th_b1={np.rad2deg(p_opt["th_b1"]):.1f}deg  '
          f'th_b2={np.rad2deg(p_opt["th_b2"]):.1f}deg  j_b={p_opt["j_b"]:.2f}  '
          f'W_bot={(Wf_+Wm_)*1000:.1f}mm')

y0_opt = calc_P0_height_flat(p_opt)
print(f'P0 평지높이 : {y0_opt*1000:.1f}mm  (제약: <={P0_HEIGHT_MAX*1000:.0f}mm)')
print(f'목적함수 값 : {f_opt:.4f}')
print('=' * 60 + '\n')

# ═══════════════════════════════════════
# [SECTION 9] 최종 검증 시뮬레이션
# ═══════════════════════════════════════
terrains_v = ['real_stairs', 'wood_block', 'rough', 'step']
terrain_nm = ['실제 계단', '목재 블록', '불규칙', '단차']
results_v = []

print('최종 검증 시뮬레이션 (calc_stability + calc_metrics v3)...')
fail_pts_total = 0
pts_total = 0
p_opt['liftoff_max'] = LIFTOFF_MAX

for ti in range(4):
    t = terrains_v[ti]
    x_t, y_t_raw = gen_terrain(t, p_opt)
    y_t_env = calc_envelope(x_t, y_t_raw, p_opt['R_w'])

    b_eff_v = get_b_eff(p_opt)
    a_eff_v = get_a_eff(p_opt)
    cb = get_cb_fwd(p_opt)
    xs = x_t[0] + b_eff_v + 0.05
    xe = x_t[-1] - (a_eff_v + cb) - 0.05
    xa = np.linspace(xs, xe, 200)

    R = kin_sim(xa, x_t, y_t_env, p_opt)
    D = calc_dynamics(R, xa, x_t, y_t_env, p_opt)
    S = calc_stability(R, xa, x_t, y_t_raw, y_t_env, p_opt)
    M = calc_metrics(xa, x_t, y_t_env, R, p_opt)
    fail_pts_total += np.sum(~R['ok'])
    pts_total += len(xa)

    valid_v = ~D['liftoff_r'] & ~D['liftoff_f']
    if np.sum(valid_v) > 5:
        nm_ = [np.mean(D['Nr'][valid_v]), np.mean(D['Nm'][valid_v]), np.mean(D['Nf'][valid_v])]
    else:
        nm_ = [np.mean(D['Nr']), np.mean(D['Nm']), np.mean(D['Nf'])]
    im_ = 0
    if np.mean(nm_) > 0.5:
        im_ = (max(nm_) - min(nm_)) / np.mean(nm_) * 100

    rv = {
        'terrain': t, 'tau_peak': D['stair_torque_peak'],
        'tau_link': D['tau_link_ratio'], 'imbal': im_,
        'fail_rate': R['fail_rate'], 'toi_min': S['min_TOI'],
        'risk_level': S['risk_level'], 'sn_dB': M['SN_dB'],
        'R': R, 'D': D, 'S': S, 'M': M,
        'x_arr': xa, 'x_t': x_t, 'y_t': y_t_raw, 'y_t_env': y_t_env,
    }
    if 'min_clearance' in S:
        rv['clearance'] = S['min_clearance']
    results_v.append(rv)

    sf_cont = p_opt['motor_tau_cont'] / max(D['stair_torque_peak'], 1e-9)
    sf_peak = p_opt['motor_tau_peak'] / max(D['stair_torque_peak'], 1e-9)
    peak_warn = ' PEAK!' if sf_peak < 1.2 else ''
    col_warn = ' (간섭!)' if S.get('is_collision', False) else ''

    print(f'  [{terrain_nm[ti]}]  tau={D["stair_torque_peak"]:.3f}Nm'
          f'(SF_cont=x{sf_cont:.2f} SF_peak=x{sf_peak:.2f}{peak_warn} '
          f'링크{D["tau_link_ratio"]*100:.0f}%)  imbal={im_:.1f}%  '
          f'TOI={S["min_TOI"]:.3f}[{S["risk_level"]}]  '
          f'SN={M["SN_dB"]:.1f}dB  fail={R["fail_rate"]*100:.1f}%{col_warn}')

print(f'\n  전 지형 통합 fail_rate: {fail_pts_total/pts_total*100:.2f}%\n')

# ═══════════════════════════════════════
# [SECTION 10] 파라미터 저장
# ═══════════════════════════════════════
save_path = os.path.join(script_dir, 'zetin_optimal_params_v3.pkl')
save_data = {
    'p_opt': p_opt, 'x_opt': x_opt, 'f_opt': f_opt,
    'results_v': results_v, 'elapsed': elapsed,
    'lb': lb, 'ub': ub,
    'W': W, 'W_terrain': W_terrain,
    'TAU_REF': TAU_REF, 'IMBAL_REF': IMBAL_REF,
    'SN_REF': SN_REF, 'TOI_WARN': TOI_WARN,
    'FAIL_MAX': FAIL_MAX, 'LIFTOFF_MAX': LIFTOFF_MAX,
}
with open(save_path, 'wb') as f:
    pickle.dump(save_data, f)
print(f'최적 파라미터 저장: {save_path}\n')

# ═══════════════════════════════════════
# [SECTION 11] 시각화
# ═══════════════════════════════════════
import matplotlib.pyplot as plt

# --- 한글 폰트 및 마이너스 깨짐 방지 세팅 ---
plt.rc('font', family='NanumGothic')
plt.rc('axes', unicode_minus=False)

t_cols = [[0.20, 0.55, 0.35], [0.80, 0.50, 0.15],
          [0.25, 0.40, 0.80], [0.70, 0.25, 0.20]]
W_tot = p_opt['mass'] * p_opt['g']

# Figure 1: 법선력 + TOI
fig1, axes1 = plt.subplots(2, 4, figsize=(15, 8.4))
fig1.patch.set_facecolor('w')

for ti in range(4):
    rv = results_v[ti]
    D = rv['D']
    S = rv['S']
    x = rv['x_arr']
    col = t_cols[ti]

    ax = axes1[0, ti]
    ax.plot(x, D['Nr'], '-', color=[0.10, 0.45, 0.75], linewidth=1.5, label='Nr')
    ax.plot(x, D['Nm'], '-', color=[0.10, 0.60, 0.25], linewidth=1.5, label='Nm')
    ax.plot(x, D['Nf'], '-', color=[0.85, 0.40, 0.10], linewidth=1.5, label='Nf')
    ax.axhline(W_tot / 3, linestyle='--', color='k', linewidth=0.8)
    lft = D['liftoff_r'] | D['liftoff_f']
    ax.set_title(f'{terrain_nm[ti]}\nimbal={rv["imbal"]:.1f}%  liftoff={np.sum(lft)}  [{rv["risk_level"]}]',
                 fontsize=8, fontweight='bold')
    if ti == 0:
        ax.legend(fontsize=6, loc='upper right')
    ax.set_xlabel('X [m]', fontsize=8)
    ax.set_ylabel('N [N]', fontsize=8)
    ax.grid(True)

    ax = axes1[1, ti]
    ax.plot(x, S['TOI'], '-', color=col, linewidth=2.0, label='TOI')
    ax.plot(x, S['TOI_front'], '--', color=[c * 0.7 for c in col], linewidth=0.9, label='TOI_fwd')
    ax.plot(x, S['TOI_rear'], ':', color=[c * 0.7 for c in col], linewidth=0.9, label='TOI_rr')
    ax.axhline(0, color='r', linewidth=1.5)
    ax.axhline(TOI_WARN, color='r', linestyle='--', linewidth=1.0)
    ax.set_ylim([-0.3, 1.15])
    ax.set_title(f'{terrain_nm[ti]}  TOI_min={S["min_TOI"]:.3f}', fontsize=8, fontweight='bold')
    if ti == 0:
        ax.legend(fontsize=6, loc='lower left')
    ax.set_xlabel('X [m]', fontsize=8)
    ax.set_ylabel('TOI [-]', fontsize=8)
    ax.grid(True)

fig1.suptitle(f'최적 구조 검증 — 법선력 & TOI 안정성  [Rocker:{p_opt["rocker_mode"]} / '
              f'Bogie:{p_opt["bogie_mode"]}  f={f_opt:.4f}]', fontsize=10, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'fig1_normal_force_toi.png'), dpi=150)
print('Figure 1 저장: fig1_normal_force_toi.png')

# Figure 2: 모터 토크
fig2, axes2 = plt.subplots(2, 4, figsize=(15, 7))
fig2.patch.set_facecolor('w')

for ti in range(4):
    rv = results_v[ti]
    D = rv['D']
    x = rv['x_arr']
    col = t_cols[ti]

    ax = axes2[0, ti]
    ax.plot(x, D['tau_max_arr'] * 1000, '-', color=col, linewidth=2.0, label='총 토크')
    ax.fill_between(x, (D['tau_rocker_inertia'] + D['tau_bogie_inertia']) * 1000,
                    color=col, alpha=0.22, label='링크 관성분')
    ax.axhline(D['stair_torque_peak'] * 1000, linestyle='--', color='r', linewidth=1.2)
    ax.set_title(f'{terrain_nm[ti]}\n링크관성 {D["tau_link_ratio"]*100:.1f}%',
                 fontsize=8, fontweight='bold')
    if ti == 0:
        ax.legend(fontsize=6, loc='upper left')
    ax.set_xlabel('X [m]', fontsize=8)
    ax.set_ylabel('토크 [mNm]', fontsize=8)
    ax.grid(True)

    ax = axes2[1, ti]
    ax.plot(x, D['alpha_rocker'], '-', color=[0.2, 0.4, 0.8], linewidth=1.5, label='alpha_Rocker')
    ax.plot(x, D['alpha_bogie'], '-', color=[0.8, 0.4, 0.2], linewidth=1.5, label='alpha_Bogie')
    ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
    ax.set_title(f'{terrain_nm[ti]} 링크 각가속도', fontsize=8, fontweight='bold')
    if ti == 0:
        ax.legend(fontsize=6)
    ax.set_xlabel('X [m]', fontsize=8)
    ax.set_ylabel('rad/s^2', fontsize=8)
    ax.grid(True)

fig2.suptitle('모터 토크 분해 — D6374 150KV 기준 (링크 관성 포함)', fontsize=10, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'fig2_motor_torque.png'), dpi=150)
print('Figure 2 저장: fig2_motor_torque.png')

# Figure 3: S/N비 이상 궤적
fig3, axes3 = plt.subplots(2, 4, figsize=(15, 7))
fig3.patch.set_facecolor('w')

for ti in range(4):
    rv = results_v[ti]
    M = rv['M']
    R = rv['R']
    x = rv['x_arr']
    x_t_v = rv['x_t']
    y_t_v = rv['y_t']
    col = t_cols[ti]

    x_valid = x[R['ok']] if np.any(~R['ok']) else x
    n_v = min(len(M['y0_valid']), len(x_valid))
    x_v = x_valid[:n_v]
    y0_v = M['y0_valid'][:n_v]
    y_ideal_v = M['y_ideal'][:n_v]

    ax = axes3[0, ti]
    ax.plot(x_t_v, y_t_v, 'k-', linewidth=1.2)
    ax.plot(x_v, y0_v, '-', color=col, linewidth=2.0, label='차체 높이')
    ax.plot(x_v, y_ideal_v, '--', color=[0.2, 0.7, 0.3], linewidth=1.5, label='이상 (y_t+R_w)')
    ax.set_title(f'{terrain_nm[ti]}\nSN_v3={M["SN_dB"]:.1f}dB  SN_v2={M["SN_dB_v2"]:.1f}dB',
                 fontsize=8, fontweight='bold')
    if ti == 0:
        ax.legend(fontsize=6, loc='upper left')
    ax.set_xlabel('X [m]', fontsize=8)
    ax.set_ylabel('Y [m]', fontsize=8)
    ax.grid(True)
    ax.set_xlim([x_t_v[0], x_t_v[-1]])

    ax = axes3[1, ti]
    err_v3 = y0_v - y_ideal_v
    err_v3_dm = err_v3 - np.nanmean(err_v3)
    win = max(3, min(40, n_v // 4))
    err_v2 = y0_v - np.convolve(y0_v, np.ones(win) / win, mode='same')
    ax.plot(x_v, err_v3_dm * 1000, '-', color=[0.2, 0.7, 0.3], linewidth=1.5, label='오차 v3')
    ax.plot(x_v, err_v2 * 1000, '-', color=[0.7, 0.3, 0.8], linewidth=1.0, label='오차 v2')
    ax.axhline(0, color='k', linestyle='--', linewidth=0.5)
    ax.set_title(f'{terrain_nm[ti]}  sigma_v3={np.std(err_v3_dm)*1000:.2f}mm  '
                 f'sigma_v2={np.std(err_v2)*1000:.2f}mm',
                 fontsize=8, fontweight='bold')
    if ti == 0:
        ax.legend(fontsize=6)
    ax.set_xlabel('X [m]', fontsize=8)
    ax.set_ylabel('오차 [mm]', fontsize=8)
    ax.grid(True)
    ax.set_xlim([x_t_v[0], x_t_v[-1]])

fig3.suptitle('S/N비 이상 궤적 비교 — v3(지형기반) vs v2(movmean)', fontsize=10, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'fig3_sn_ratio.png'), dpi=150)
print('Figure 3 저장: fig3_sn_ratio.png')

# Figure 4: 로봇 형상 스케치
fig4, ax4 = plt.subplots(figsize=(9, 5.2))
rv_s = results_v[0]
ax4.fill_between(rv_s['x_t'], rv_s['y_t'], -0.06, color=[0.78, 0.68, 0.58], alpha=0.5)
ax4.plot(rv_s['x_t'], rv_s['y_t'], 'k-', linewidth=1.2)

snap_idx = np.round(np.linspace(20, len(rv_s['x_arr']) - 20, 7)).astype(int)
th_w = np.linspace(0, 2 * np.pi, 24)

for fi in snap_idx:
    R_s = rv_s['R']
    y0_ = R_s['y0'][fi]
    ar_ = R_s['ar'][fi]
    bb_ = R_s['bb'][fi]
    Wf_, Wm_, Wr_, Pb_ = wpos(np.array([y0_, ar_, bb_]), rv_s['x_arr'][fi], p_opt)[:4]
    P0_ = np.array([rv_s['x_arr'][fi], y0_])

    mode_r = p_opt.get('rocker_mode', 'linear').lower()
    if mode_r in ('linear', 'triangle'):
        ax4.plot([Wr_[0], P0_[0], Pb_[0]], [Wr_[1], P0_[1], Pb_[1]], 'b-', linewidth=2.5)
    elif mode_r == 'frame':
        ar_e = ar_ + p_opt.get('phi_r0', 0)
        ur = np.array([np.cos(ar_e), np.sin(ar_e)])
        Ptr = P0_ - p_opt['j_r'] * p_opt['T_r'] * ur
        Ptf = P0_ + (1 - p_opt['j_r']) * p_opt['T_r'] * ur
        ax4.plot([Ptr[0], Ptf[0]], [Ptr[1], Ptf[1]], 'b-', linewidth=2.5)
        ax4.plot([Ptf[0], Pb_[0]], [Ptf[1], Pb_[1]], 'b-', linewidth=2)
        ax4.plot([Ptr[0], Wr_[0]], [Ptr[1], Wr_[1]], 'b-', linewidth=2)

    mode_b = p_opt.get('bogie_mode', 'linear').lower()
    if mode_b in ('linear', 'triangle'):
        ax4.plot([Wm_[0], Pb_[0], Wf_[0]], [Wm_[1], Pb_[1], Wf_[1]], 'g-', linewidth=2.5)
    elif mode_b == 'frame':
        ubb = np.array([np.cos(bb_), np.sin(bb_)])
        Pbm = Pb_ - p_opt['j_b'] * p_opt['T_b'] * ubb
        Pbf = Pb_ + (1 - p_opt['j_b']) * p_opt['T_b'] * ubb
        ax4.plot([Pbm[0], Pbf[0]], [Pbm[1], Pbf[1]], 'g-', linewidth=2.5)
        ax4.plot([Pbf[0], Wf_[0]], [Pbf[1], Wf_[1]], 'g-', linewidth=2)
        ax4.plot([Pbm[0], Wm_[0]], [Pbm[1], Wm_[1]], 'g-', linewidth=2)

    # 차체
    ar_e = ar_ + p_opt.get('phi_r0', 0)
    ur = np.array([np.cos(ar_e), np.sin(ar_e)])
    nr = np.array([-np.sin(ar_e), np.cos(ar_e)])
    bw = 0.08
    ax4.fill([P0_[0] - bw * ur[0], P0_[0] + bw * ur[0],
              P0_[0] + bw * ur[0] + p_opt['h_body'] * nr[0],
              P0_[0] - bw * ur[0] + p_opt['h_body'] * nr[0]],
             [P0_[1] - bw * ur[1], P0_[1] + bw * ur[1],
              P0_[1] + bw * ur[1] + p_opt['h_body'] * nr[1],
              P0_[1] - bw * ur[1] + p_opt['h_body'] * nr[1]],
             color=[0.4, 0.6, 0.85], alpha=0.5, edgecolor=[0.1, 0.3, 0.7])

    for Ww in [Wf_, Wm_, Wr_]:
        ax4.fill(Ww[0] + p_opt['R_w'] * np.cos(th_w),
                 Ww[1] + p_opt['R_w'] * np.sin(th_w),
                 color=[0.2, 0.2, 0.2], alpha=0.8)

ax4.plot(rv_s['x_arr'], rv_s['R']['y0'], '-', color=[0.20, 0.55, 0.35],
         linewidth=2.2, label='차체 궤적')
ax4.legend(loc='upper left', fontsize=8)
ax4.set_xlim([rv_s['x_t'][0], rv_s['x_t'][-1]])
ax4.set_ylim([-0.05, np.max(rv_s['y_t']) + p_opt['R_w'] * 4 + p_opt['h_body'] + 0.15])
ax4.grid(True)
ax4.set_xlabel('X [m]', fontsize=10)
ax4.set_ylabel('Y [m]', fontsize=10)
ax4.set_title(f'최적 구조 — 실제 계단 주행\n'
              f'Rocker:{p_opt["rocker_mode"]}  Bogie:{p_opt["bogie_mode"]}  f={f_opt:.4f}',
              fontsize=10, fontweight='bold')
plt.tight_layout()
plt.savefig(os.path.join(script_dir, 'fig4_robot_shape.png'), dpi=150)
print('Figure 4 저장: fig4_robot_shape.png')

# ═══════════════════════════════════════
# [SECTION 12] 가중치 민감도 분석
# ═══════════════════════════════════════
if RUN_SENSITIVITY:
    print('\n' + '=' * 60)
    print('  가중치 민감도 분석 시작')
    print('-' * 60)

    w_fields = ['tau', 'imbal', 'stab', 'sn', 'fail']
    n_fields = len(w_fields)

    sens_f_vals = np.zeros((n_fields, N_SENS_PERTURB))
    sens_grid = np.linspace(-0.10, +0.10, N_SENS_PERTURB)

    for wi, fname in enumerate(w_fields):
        w_nom = W[fname]
        for p_idx in range(N_SENS_PERTURB):
            W_perturb = dict(W)
            delta = w_nom * sens_grid[p_idx]
            W_perturb[fname] = max(w_nom + delta, 0)

            other_fields = [f for f in w_fields if f != fname]
            w_sum_other = sum(W[f] for f in other_fields)
            scale = (1 - W_perturb[fname]) / max(w_sum_other, 1e-6)
            for of in other_fields:
                W_perturb[of] = W[of] * scale

            # Temporarily override W for objective
            W_backup = dict(W)
            W.update(W_perturb)
            f_perturb = objective(x_opt)
            W.update(W_backup)

            sens_f_vals[wi, p_idx] = f_perturb

        df_dw = (sens_f_vals[wi, -1] - sens_f_vals[wi, 0]) / (0.20 * w_nom + 1e-9)
        print(f'  W.{fname:<6s}  명목={w_nom:.2f}  df/dw={df_dw:.4f}  '
              f'f 범위=[{np.min(sens_f_vals[wi,:]):.4f}, {np.max(sens_f_vals[wi,:]):.4f}]')

    # 민감도 시각화
    fig5, axes5 = plt.subplots(1, n_fields, figsize=(10, 5))
    w_labels = ['W_tau', 'W_imbal', 'W_stab', 'W_SN', 'W_fail']
    w_colors = [[0.7, 0.2, 0.2], [0.2, 0.5, 0.8], [0.2, 0.7, 0.4],
                [0.8, 0.6, 0.1], [0.5, 0.3, 0.7]]

    for wi in range(n_fields):
        ax = axes5[wi]
        w_nom = W[w_fields[wi]]
        x_axis = w_nom * (1 + sens_grid)
        ax.plot(x_axis, sens_f_vals[wi, :], '-o', color=w_colors[wi], linewidth=2.0,
                markersize=5, markerfacecolor=w_colors[wi])
        ax.axvline(w_nom, color='k', linestyle='--')
        ax.set_xlabel(w_labels[wi], fontsize=9)
        ax.set_ylabel('목적함수 f', fontsize=8)
        ax.set_title(f'민감도: {w_labels[wi]}', fontsize=9, fontweight='bold')
        ax.grid(True)

    fig5.suptitle('가중치 민감도 분석 — 현재 최적해 기준 +-10% 섭동', fontsize=10, fontweight='bold')
    plt.tight_layout()
    plt.savefig(os.path.join(script_dir, 'fig5_sensitivity.png'), dpi=150)
    print('Figure 5 저장: fig5_sensitivity.png')
    print('=' * 60)
    print('민감도 분석 완료.\n')

print('=== 완료 ===')
print(f'저장: {save_path}')
plt.show()
