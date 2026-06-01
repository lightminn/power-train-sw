"""
calc_stability: 전복 안정성 분석 — ZMP + Tip-over Index (TOI) [v3]
"""
import numpy as np
from .wpos import wpos


def _rocker_arm_h_local(ar, p):
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
        a_h = p.get('a_r', 0.22) * abs(np.cos(ar)) if 'a_r' in p else 0.22
        b_h = p.get('b_r', 0.28) * abs(np.cos(ar)) if 'b_r' in p else 0.28
    return max(a_h, 0.01), max(b_h, 0.01)


def _bogie_arm_h_local(bb, p):
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


def calc_stability(R, x_arr, x_t, y_t_raw, y_t_env, p):
    CG_offset = p.get('CG_offset', 0)
    liftoff_max = p.get('liftoff_max', 0.02)
    phi_r0 = p.get('phi_r0', 0)

    N = len(x_arr)
    W = p['mass'] * p['g']

    # CG 위치
    if 'xcg' in R and not np.all(np.isnan(R['xcg'])):
        x_cg = R['xcg']
    else:
        x_cg = x_arr + CG_offset * np.cos(R['ar'])

    # 초기화
    S = {
        'x_zmp': np.full(N, np.nan),
        'x_sp_min': np.full(N, np.nan),
        'x_sp_max': np.full(N, np.nan),
        'zmp_margin': np.full(N, np.nan),
        'zmp_ok': np.zeros(N, dtype=bool),
        'TOI_front': np.full(N, np.nan),
        'TOI_rear': np.full(N, np.nan),
        'TOI': np.full(N, np.nan),
        'Nr_raw': np.full(N, np.nan),
        'Nf_raw': np.full(N, np.nan),
        'liftoff_r': np.zeros(N, dtype=bool),
        'liftoff_f': np.zeros(N, dtype=bool),
    }

    for i in range(N):
        if 'ok' in R and not R['ok'][i]:
            continue

        xwf = R['xwf'][i]
        xwm = R['xwm'][i]
        xwr = R['xwr'][i]
        if any(np.isnan([xwf, xwm, xwr])):
            continue

        # ① 법선력 재계산
        sp = xwf - xwr
        if abs(sp) < 1e-6:
            sp = 1e-6

        hf_ = np.interp(xwf, x_t, y_t_env)
        hr_ = np.interp(xwr, x_t, y_t_env)
        theta = np.arctan2(hf_ - hr_, sp)

        a_h, b_h = _rocker_arm_h_local(R['ar'][i], p)
        c_h, d_h = _bogie_arm_h_local(R['bb'][i], p)

        W_cos = W * np.cos(theta)
        a_eff = a_h + CG_offset * np.cos(R['ar'][i])
        b_eff = b_h - CG_offset * np.cos(R['ar'][i])

        ratio_fm = d_h / max(c_h, 1e-3)

        if abs(b_eff) < 1e-3:
            b_eff_safe = np.sign(b_eff + 1e-9) * 1e-3
        else:
            b_eff_safe = b_eff

        ratio_rb = a_eff / b_eff_safe

        Nb_raw = W_cos / (ratio_rb + 1)
        Nr_raw = Nb_raw * ratio_rb
        Nm_raw = Nb_raw / (1 + ratio_fm)
        Nf_raw = Nm_raw * ratio_fm

        S['Nr_raw'][i] = Nr_raw
        S['Nf_raw'][i] = Nf_raw
        S['liftoff_r'][i] = Nr_raw < 0
        S['liftoff_f'][i] = Nf_raw < 0

        # ② ZMP 계산
        N_total = Nr_raw + Nm_raw + Nf_raw
        if N_total > W * 0.05:
            x_zmp_i = (Nr_raw * xwr + Nm_raw * xwm + Nf_raw * xwf) / N_total
        else:
            x_zmp_i = x_cg[i]
        S['x_zmp'][i] = x_zmp_i

        sp_min = min(xwf, xwr)
        sp_max = max(xwf, xwr)
        S['x_sp_min'][i] = sp_min
        S['x_sp_max'][i] = sp_max

        S['zmp_margin'][i] = min(x_zmp_i - sp_min, sp_max - x_zmp_i)
        S['zmp_ok'][i] = (x_zmp_i >= sp_min) and (x_zmp_i <= sp_max)

        # ③ TOI 계산
        sp_width = sp_max - sp_min
        if sp_width < 0.05:
            S['TOI_front'][i] = 0.5
            S['TOI_rear'][i] = 0.5
            S['TOI'][i] = 0.5
            continue

        TOI_f = max(min((sp_max - x_cg[i]) / sp_width, 2.0), -2.0)
        TOI_r = max(min((x_cg[i] - sp_min) / sp_width, 2.0), -2.0)

        S['TOI_front'][i] = TOI_f
        S['TOI_rear'][i] = TOI_r
        S['TOI'][i] = min(TOI_f, TOI_r)

    # ④ 차체 간섭 검사
    N_div = 20
    rocker_mode = p.get('rocker_mode', 'linear').lower()
    bogie_mode = p.get('bogie_mode', 'linear').lower()

    t_div = np.linspace(0, 1, N_div)
    S['clearance'] = np.full(N, np.nan)

    for i in range(N):
        if 'ok' in R and not R['ok'][i]:
            continue

        X = np.array([R['y0'][i], R['ar'][i], R['bb'][i]])
        Wf, Wm, Wr, Pb, _ = wpos(X, x_arr[i], p)
        P0 = np.array([x_arr[i], R['y0'][i]])

        pts_x = []
        pts_y = []

        # Rocker 선분
        if rocker_mode == 'frame':
            ar_e = R['ar'][i] + phi_r0
            ur = np.array([np.cos(ar_e), np.sin(ar_e)])
            Ptr = P0 - p['j_r'] * p['T_r'] * ur
            Ptf = P0 + (1 - p['j_r']) * p['T_r'] * ur
            for A, B in [(Ptr, Ptf), (Ptf, Pb), (Ptr, Wr)]:
                pts_x.extend(A[0] + (B[0] - A[0]) * t_div)
                pts_y.extend(A[1] + (B[1] - A[1]) * t_div)
        else:
            for A, B in [(P0, Pb), (P0, Wr)]:
                pts_x.extend(A[0] + (B[0] - A[0]) * t_div)
                pts_y.extend(A[1] + (B[1] - A[1]) * t_div)

        # Bogie 선분
        if bogie_mode == 'frame':
            ubb = np.array([np.cos(R['bb'][i]), np.sin(R['bb'][i])])
            Pbm = Pb - p['j_b'] * p['T_b'] * ubb
            Pbf = Pb + (1 - p['j_b']) * p['T_b'] * ubb
            for A, B in [(Pbm, Pbf), (Pbf, Wf), (Pbm, Wm)]:
                pts_x.extend(A[0] + (B[0] - A[0]) * t_div)
                pts_y.extend(A[1] + (B[1] - A[1]) * t_div)
        else:
            for A, B in [(Pb, Wf), (Pb, Wm)]:
                pts_x.extend(A[0] + (B[0] - A[0]) * t_div)
                pts_y.extend(A[1] + (B[1] - A[1]) * t_div)

        pts_x = np.array(pts_x)
        pts_y = np.array(pts_y)

        terr_y = np.interp(pts_x, x_t, y_t_raw)
        clearances = pts_y - terr_y
        S['clearance'][i] = np.min(clearances)

    S['min_clearance'] = np.nanmin(S['clearance'])
    S['is_collision'] = S['min_clearance'] < 0.01

    # 전체 통계
    S['min_TOI'] = np.nanmin(S['TOI'])
    S['min_zmp_margin'] = np.nanmin(S['zmp_margin'])

    S['n_liftoff'] = np.sum(S['liftoff_r'] | S['liftoff_f'])
    S['liftoff_ratio'] = S['n_liftoff'] / N

    S['n_zmp_out'] = np.sum(~S['zmp_ok'])
    pct_zmpout = S['n_zmp_out'] / N

    # 위험 등급 판정
    if S['min_TOI'] < 0 or S['liftoff_ratio'] > liftoff_max or pct_zmpout > 0.50 or S['is_collision']:
        S['risk_level'] = 'danger'
    elif S['min_TOI'] < 0.15 or S['liftoff_ratio'] > liftoff_max * 0.5 or pct_zmpout > 0.20:
        S['risk_level'] = 'warning'
    else:
        S['risk_level'] = 'safe'

    # 콘솔 출력
    badges = {'safe': 'SAFE', 'warning': 'WARNING', 'danger': 'DANGER'}
    badge = badges.get(S['risk_level'], '?')
    col_warn = ' (간섭!)' if S['is_collision'] else ''
    print(f'  [안정성] {badge}  TOI_min={S["min_TOI"]:.3f}  '
          f'ZMP이탈={pct_zmpout*100:.1f}%  들림={S["liftoff_ratio"]*100:.1f}%  '
          f'여유공간={S["min_clearance"]:.3f}m{col_warn}')

    return S
