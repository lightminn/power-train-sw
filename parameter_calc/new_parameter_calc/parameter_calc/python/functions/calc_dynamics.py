"""
calc_dynamics: 동역학 계산 — 전 모드 통합판 [v4]
"""
import numpy as np
from scipy.signal import filtfilt
from .wpos import wpos


def _rocker_arm_len(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'linear':
        L1 = p.get('a_r', 0.22)
        L2 = p.get('b_r', 0.28)
    elif mode == 'triangle':
        L1 = p['L_r1']
        L2 = p['L_r2']
    elif mode == 'frame':
        L1 = np.sqrt(((1 - p['j_r']) * p['T_r'])**2 + p['S_r1']**2)
        L2 = np.sqrt((p['j_r'] * p['T_r'])**2 + p['S_r2']**2)
    else:
        L1 = p.get('a_r', 0.22)
        L2 = p.get('b_r', 0.28)
    return max(L1, 0.05), max(L2, 0.05)


def _rocker_arm_h(ar, p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'linear':
        a_h = p['a_r'] * abs(np.cos(ar))
        b_h = p['b_r'] * abs(np.cos(ar))
    elif mode == 'triangle':
        ang_pb = ar - p['alpha_r'] / 2
        ang_wr = ar - np.pi + p['alpha_r'] / 2
        a_h = max(p['L_r1'] * abs(np.cos(ang_pb)), 0.01)
        b_h = max(p['L_r2'] * abs(np.cos(ang_wr)), 0.01)
    elif mode == 'frame':
        a_h = max((1 - p['j_r']) * p['T_r'] * abs(np.cos(ar)) + p['S_r1'] * abs(np.sin(p['th_r1'])), 0.01)
        b_h = max(p['j_r'] * p['T_r'] * abs(np.cos(ar)) + p['S_r2'] * abs(np.sin(p['th_r2'])), 0.01)
    else:
        a_h = p.get('a_r', 0.22) * abs(np.cos(ar))
        b_h = p.get('b_r', 0.28) * abs(np.cos(ar))
    return max(a_h, 0.01), max(b_h, 0.01)


def _bogie_arm_h(bb, p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'linear':
        c_h = max(p['c_b'] * abs(np.cos(bb)), 0.01)
        d_h = max(p['d_b'] * abs(np.cos(bb)), 0.01)
    elif mode == 'triangle':
        ang_vert = -np.pi / 2 + bb
        ang_wf = ang_vert + p['beta_b'] / 2
        ang_wm = ang_vert - p['beta_b'] / 2
        c_h = max(p['L_b1'] * abs(np.cos(ang_wf)), 0.01)
        d_h = max(p['L_b2'] * abs(np.cos(ang_wm)), 0.01)
    elif mode == 'frame':
        c_h = max(abs((1 - p['j_b']) * p['T_b'] * np.cos(bb) + p['S_b1'] * np.sin(p['th_b1'] + bb)), 0.01)
        d_h = max(abs(p['j_b'] * p['T_b'] * np.cos(bb) + p['S_b2'] * np.sin(p['th_b2'] - bb)), 0.01)
    else:
        c_h = max(p.get('c_b', 0.14) * abs(np.cos(bb)), 0.01)
        d_h = max(p.get('d_b', 0.14) * abs(np.cos(bb)), 0.01)
    return max(c_h, 0.01), max(d_h, 0.01)


def _bogie_cb0(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'linear':
        cb = p.get('c_b', 0.14)
    elif mode == 'triangle':
        cb = p['L_b1'] * abs(np.sin(p['beta_b'] / 2))
    elif mode == 'frame':
        cb = p['S_b1'] * abs(np.cos(p['th_b1']))
    else:
        cb = p.get('c_b', 0.14)
    return max(cb, 0.04)


def _bogie_db0(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'linear':
        db = p.get('d_b', 0.14)
    elif mode == 'triangle':
        db = p['L_b2'] * abs(np.sin(p['beta_b'] / 2))
    elif mode == 'frame':
        db = p['S_b2'] * abs(np.cos(p['th_b2']))
    else:
        db = p.get('d_b', 0.14)
    return max(db, 0.04)


def calc_dynamics(R, x_arr, x_t, y_t, p):
    import warnings
    warnings.filterwarnings('ignore', category=RuntimeWarning)

    # 기본값
    mu = p.get('mu', 0.70)
    gear_ratio = p.get('gear_ratio', 5)
    eta_gear = p.get('eta_gear', 0.85)
    motor_tau_peak = p.get('motor_tau_peak', 4.0)
    m_wheel = p.get('m_wheel', 1.2)
    m_rocker_link = p.get('m_rocker_link', 0.8)
    m_bogie_link = p.get('m_bogie_link', 0.5)
    I_rocker_add = p.get('I_rocker_add', 0)
    I_bogie_add = p.get('I_bogie_add', 0)
    e_restitution = p.get('e_restitution', 0.3)
    v_robot = p.get('v_robot', 1.0)
    step_thresh = p.get('step_thresh', 5.0)
    CG_offset = p.get('CG_offset', 0)
    h_CG = p.get('h_CG', p.get('h_body', 0.3) * 0.5)
    R_w = p['R_w']
    mass = p['mass']
    g = p['g']

    N = len(x_arr)
    W = mass * g

    # ① 바퀴 회전 관성
    I_wheel = 0.5 * m_wheel * R_w**2

    # ② Rocker 링크 관성 모멘트
    L_r1_eff, L_r2_eff = _rocker_arm_len(p)
    L_r_tot = max(L_r1_eff + L_r2_eff, 1e-4)
    m_r1 = m_rocker_link * L_r1_eff / L_r_tot
    m_r2 = m_rocker_link * L_r2_eff / L_r_tot
    I_rocker = (1 / 3) * m_r1 * L_r1_eff**2 + (1 / 3) * m_r2 * L_r2_eff**2 + I_rocker_add

    # ③ Bogie 링크 관성 모멘트
    cb0 = _bogie_cb0(p)
    db0 = _bogie_db0(p)
    I_bogie_wheel = m_wheel * cb0**2 + m_wheel * db0**2
    I_bogie_link = (1 / 12) * m_bogie_link * (cb0 + db0)**2 + I_bogie_add
    I_bogie = I_bogie_wheel + I_bogie_link
    m_bogie_eff = I_bogie / (cb0**2 + 1e-9)

    # ④ CG 가속도
    ycg = R.get('ycg', R['y0'] + p.get('h_body', 0.3) * 0.5)
    if ycg is None or np.all(np.isnan(ycg)):
        ycg = R['y0'] + p.get('h_body', 0.3) * 0.5

    sm_win_cg = max(5, N // 50)
    kernel_cg = np.ones(sm_win_cg) / sm_win_cg

    if len(ycg) > 3 * sm_win_cg:
        ycg_smooth = filtfilt(kernel_cg, 1, ycg)
    else:
        ycg_smooth = ycg.copy()

    vy_cg = np.gradient(ycg_smooth, x_arr) * v_robot
    ay_cg_raw = np.gradient(vy_cg, x_arr) * v_robot
    if len(ay_cg_raw) > 3 * sm_win_cg:
        ay_cg = filtfilt(kernel_cg, 1, ay_cg_raw)
    else:
        ay_cg = ay_cg_raw.copy()
    ay_cg = np.clip(ay_cg, -3 * g, 3 * g)

    # ⑤ 링크 각가속도
    sm_win = max(5, N // 40)
    kernel = np.ones(sm_win) / sm_win

    if len(R['ar']) > 3 * sm_win:
        ar_smooth = filtfilt(kernel, 1, R['ar'])
    else:
        ar_smooth = R['ar'].copy()
    dar_dt = np.gradient(ar_smooth, x_arr) * v_robot
    alpha_rocker_raw = np.gradient(dar_dt, x_arr) * v_robot
    if len(alpha_rocker_raw) > 3 * sm_win:
        alpha_rocker = filtfilt(kernel, 1, alpha_rocker_raw)
    else:
        alpha_rocker = alpha_rocker_raw.copy()
    alpha_rocker = np.clip(alpha_rocker, -50, 50)

    if len(R['bb']) > 3 * sm_win:
        bb_smooth = filtfilt(kernel, 1, R['bb'])
    else:
        bb_smooth = R['bb'].copy()
    dbb_dt = np.gradient(bb_smooth, x_arr) * v_robot
    alpha_bogie_raw = np.gradient(dbb_dt, x_arr) * v_robot
    if len(alpha_bogie_raw) > 3 * sm_win:
        alpha_bogie = filtfilt(kernel, 1, alpha_bogie_raw)
    else:
        alpha_bogie = alpha_bogie_raw.copy()
    alpha_bogie = np.clip(alpha_bogie, -50, 50)

    # ⑥ 계단 에지 감지
    theta_arr = np.zeros(N)
    for i in range(N):
        hf_ = np.interp(R['xwf'][i], x_t, y_t)
        hr_ = np.interp(R['xwr'][i], x_t, y_t)
        sp = R['xwf'][i] - R['xwr'][i]
        if abs(sp) > 1e-6:
            theta_arr[i] = np.arctan2(hf_ - hr_, sp)

    dth_dx = np.abs(np.gradient(theta_arr, x_arr))
    edge_mask = dth_dx > np.deg2rad(step_thresh) / 0.30

    # 출력 초기화
    D = {
        'Nr': np.zeros(N), 'Nm': np.zeros(N), 'Nf': np.zeros(N),
        'Nr_raw': np.zeros(N), 'Nf_raw': np.zeros(N),
        'liftoff_r': np.zeros(N, dtype=bool), 'liftoff_f': np.zeros(N, dtype=bool),
        'Fdr': np.zeros(N), 'Fdm': np.zeros(N), 'Fdf': np.zeros(N),
        'tau_wheel_r': np.zeros(N), 'tau_wheel_m': np.zeros(N), 'tau_wheel_f': np.zeros(N),
        'tau_motor_r': np.zeros(N), 'tau_motor_m': np.zeros(N), 'tau_motor_f': np.zeros(N),
        'tau_inertia': np.zeros(N),
        'tau_rocker_inertia': np.zeros(N), 'tau_bogie_inertia': np.zeros(N),
        'alpha_rocker': alpha_rocker, 'alpha_bogie': alpha_bogie,
        'tau_impact': np.zeros(N), 'tau_impact2': np.zeros(N),
        'slip_r': np.zeros(N), 'slip_m': np.zeros(N), 'slip_f': np.zeros(N),
        'power_total': np.zeros(N), 'traction_ok': np.ones(N, dtype=bool),
        'theta_local': theta_arr, 'edge_mask': edge_mask,
        'F_inertia': np.zeros(N), 'F_impact': np.zeros(N), 'F_impact2': np.zeros(N),
        'ay_cg': ay_cg,
    }

    n_liftoff_r = 0
    n_liftoff_f = 0

    # 메인 루프
    for i in range(N):
        ar = R['ar'][i]
        bb = R['bb'][i]
        theta = theta_arr[i]

        # 관성력
        F_in_y = mass * ay_cg[i]
        D['F_inertia'][i] = F_in_y

        # 2단계 연쇄 충격
        if edge_mask[i]:
            v_app = min(max(abs(vy_cg[i]), 0.01), v_robot)
            dv = (1 + e_restitution) * v_app
            m_eff_c = m_wheel + m_bogie_eff
            k_contact = 1e5
            t_c = np.pi * np.sqrt(m_eff_c / k_contact)
            t_c = max(t_c, 0.002)
            F_imp1 = (m_wheel + m_bogie_eff) * dv / (t_c + 1e-9)
            a_bog = F_imp1 * cb0 / (I_bogie + 1e-9)
            t_del = np.sqrt(np.pi / max(a_bog, 0.1))
            F_imp2 = F_imp1 * e_restitution
            dx_arr = np.mean(np.diff(x_arr)) if N > 1 else 1.0
            i2 = min(i + round(t_del * v_robot / dx_arr), N - 1)
            D['F_impact'][i] = F_imp1
            D['F_impact2'][i2] = D['F_impact2'][i2] + F_imp2

        F_imp_tot = D['F_impact'][i] + D['F_impact2'][i]

        # 유효 중력 성분
        W_eff_raw = W * np.cos(theta) + F_in_y + F_imp_tot
        W_eff_raw = min(W_eff_raw, 5 * W)

        # 법선력 분배
        c_h, d_h = _bogie_arm_h(bb, p)
        a_h, b_h = _rocker_arm_h(ar, p)

        a_eff = a_h + CG_offset * np.cos(ar)
        b_eff = b_h - CG_offset * np.cos(ar)

        ratio_fm = d_h / max(c_h, 1e-3)
        if abs(b_eff) < 1e-3:
            b_eff_safe = np.sign(b_eff + 1e-9) * 1e-3
        else:
            b_eff_safe = b_eff
        ratio_rb = a_eff / b_eff_safe

        Nb_raw = W_eff_raw / (ratio_rb + 1)
        Nr_raw = Nb_raw * ratio_rb
        Nm_raw = Nb_raw / (1 + ratio_fm)
        Nf_raw = Nm_raw * ratio_fm

        D['Nr_raw'][i] = Nr_raw
        D['Nf_raw'][i] = Nf_raw

        # 들림 감지
        if Nr_raw < 0:
            D['liftoff_r'][i] = True
            n_liftoff_r += 1
        if Nf_raw < 0:
            D['liftoff_f'][i] = True
            n_liftoff_f += 1

        Nr = max(Nr_raw, 0)
        Nm = max(Nm_raw, 0)
        Nf = max(Nf_raw, 0)
        D['Nr'][i] = Nr
        D['Nm'][i] = Nm
        D['Nf'][i] = Nf

        # 구동력
        F_req_total = W * np.sin(theta) + 0.02 * W * np.cos(theta)
        F_drv = abs(F_req_total) / 2

        N_tot = Nr + Nm + Nf
        if N_tot > 1e-6:
            Fdr = F_drv * (Nr / N_tot)
            Fdm = F_drv * (Nm / N_tot)
            Fdf = F_drv * (Nf / N_tot)
        else:
            Fdr = Fdm = Fdf = 0
        D['Fdr'][i] = Fdr
        D['Fdm'][i] = Fdm
        D['Fdf'][i] = Fdf

        # 바퀴 토크
        tau_wr = Fdr * R_w
        tau_wm = Fdm * R_w
        tau_wf = Fdf * R_w
        D['tau_wheel_r'][i] = tau_wr
        D['tau_wheel_m'][i] = tau_wm
        D['tau_wheel_f'][i] = tau_wf

        # 바퀴 회전관성
        tau_in_total = I_wheel * abs(ay_cg[i]) / R_w
        D['tau_inertia'][i] = tau_in_total
        if N_tot > 1e-6:
            tau_in_r = tau_in_total * (Nr / N_tot)
            tau_in_m = tau_in_total * (Nm / N_tot)
            tau_in_f = tau_in_total * (Nf / N_tot)
        else:
            tau_in_r = tau_in_m = tau_in_f = tau_in_total / 3

        # Rocker 링크 관성
        tau_rocker_lnk = I_rocker * abs(alpha_rocker[i])
        ge = gear_ratio * eta_gear
        D['tau_rocker_inertia'][i] = tau_rocker_lnk / ge
        if N_tot > 1e-6:
            tau_rk_r = D['tau_rocker_inertia'][i] * (Nr / N_tot)
            tau_rk_m = D['tau_rocker_inertia'][i] * (Nm / N_tot)
            tau_rk_f = D['tau_rocker_inertia'][i] * (Nf / N_tot)
        else:
            tau_rk_r = tau_rk_m = tau_rk_f = D['tau_rocker_inertia'][i] / 3

        # Bogie 링크 관성
        tau_bogie_lnk = I_bogie * abs(alpha_bogie[i])
        D['tau_bogie_inertia'][i] = tau_bogie_lnk / ge
        N_bogie = Nm + Nf
        if N_bogie > 1e-6:
            tau_bg_m = D['tau_bogie_inertia'][i] * (Nm / N_bogie)
            tau_bg_f = D['tau_bogie_inertia'][i] * (Nf / N_bogie)
        else:
            tau_bg_m = tau_bg_f = D['tau_bogie_inertia'][i] / 2

        # 충격 토크
        ti1 = D['F_impact'][i] * R_w
        ti2 = D['F_impact2'][i] * R_w
        D['tau_impact'][i] = ti1
        D['tau_impact2'][i] = ti2

        # 모터 토크 합산
        D['tau_motor_r'][i] = (tau_wr + tau_in_r + tau_rk_r) / ge
        D['tau_motor_m'][i] = (tau_wm + tau_in_m + tau_rk_m + tau_bg_m + ti2) / ge
        D['tau_motor_f'][i] = (tau_wf + tau_in_f + tau_rk_f + tau_bg_f + ti1) / ge

        # 마찰 여유도
        MIN_N = 0.5
        D['slip_r'][i] = abs(Fdr) / (mu * max(Nr, MIN_N))
        D['slip_m'][i] = abs(Fdm) / (mu * max(Nm, MIN_N))
        D['slip_f'][i] = abs(Fdf) / (mu * max(Nf, MIN_N))

        # 견인력 판정
        motor_ok = max(D['tau_motor_r'][i], D['tau_motor_m'][i], D['tau_motor_f'][i]) <= motor_tau_peak
        slip_ok = max(D['slip_r'][i], D['slip_m'][i], D['slip_f'][i]) < 1.0
        no_liftoff = not D['liftoff_r'][i] and not D['liftoff_f'][i]
        D['traction_ok'][i] = slip_ok and motor_ok and no_liftoff

        # 소비 전력
        om = (v_robot / R_w) * gear_ratio
        D['power_total'][i] = (D['tau_motor_r'][i] + D['tau_motor_m'][i] + D['tau_motor_f'][i]) * om * 2

    # 들림 요약
    if n_liftoff_r > 0 or n_liftoff_f > 0:
        print(f'  [법선력 경고] 뒷바퀴 들림: {n_liftoff_r} pts / 앞바퀴 들림: {n_liftoff_f} pts')

    # 실패 포인트 NaN 처리
    if 'ok' in R:
        invalid = ~R['ok']
        for key in ['tau_motor_r', 'tau_motor_m', 'tau_motor_f', 'slip_r', 'slip_m', 'slip_f']:
            D[key][invalid] = np.nan

    # 계단 피크 토크
    D['tau_max_arr'] = np.maximum(np.maximum(D['tau_motor_r'], D['tau_motor_m']), D['tau_motor_f'])
    stair_zone = np.convolve(edge_mask.astype(float), np.ones(11) / 11, mode='same') > 0

    valid_mask = R['ok']
    valid_stair = stair_zone & valid_mask

    if np.any(valid_stair):
        D['stair_torque_peak'] = np.nanpercentile(D['tau_max_arr'][valid_stair], 95)
    elif np.any(valid_mask):
        D['stair_torque_peak'] = np.nanpercentile(D['tau_max_arr'][valid_mask], 95)
    else:
        D['stair_torque_peak'] = 100

    valid_tau = D['tau_max_arr'][valid_mask]
    D['stair_torque_max'] = np.nanmax(valid_tau) if len(valid_tau) > 0 else 100

    # 링크 관성 기여
    D['tau_link_max'] = np.max(D['tau_rocker_inertia'] + D['tau_bogie_inertia'])
    D['tau_link_ratio'] = D['tau_link_max'] / (D['stair_torque_max'] + 1e-9)

    return D
