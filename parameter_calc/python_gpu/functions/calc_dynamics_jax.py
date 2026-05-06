"""
calc_dynamics_jax: 동역학 계산 — JAX GPU 가속 버전

가속 전략:
  1. 포인트별 for 루프 → jnp 벡터 연산으로 전환 (한 번에 N개 처리)
  2. filtfilt → jax 호환 이동평균 (jnp.convolve)
  3. 법선력/토크 분배 → 배열 연산 벡터화
"""
import jax
import jax.numpy as jnp
import numpy as np
from .wpos_jax import pack_params_auto


# ═══════════════════════════════════════
# 헬퍼: 팔길이 계산 (벡터화)
# ═══════════════════════════════════════

def _rocker_arm_h_vec(ar_arr, p):
    """Rocker 수평 투영 팔길이 — 배열 입력"""
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'linear':
        a_h = p['a_r'] * np.abs(np.cos(ar_arr))
        b_h = p['b_r'] * np.abs(np.cos(ar_arr))
    elif mode == 'triangle':
        ang_pb = ar_arr - p['alpha_r'] / 2
        ang_wr = ar_arr - np.pi + p['alpha_r'] / 2
        a_h = np.maximum(p['L_r1'] * np.abs(np.cos(ang_pb)), 0.01)
        b_h = np.maximum(p['L_r2'] * np.abs(np.cos(ang_wr)), 0.01)
    elif mode == 'frame':
        a_h = np.maximum((1 - p['j_r']) * p['T_r'] * np.abs(np.cos(ar_arr))
                         + p['S_r1'] * np.abs(np.sin(p['th_r1'])), 0.01)
        b_h = np.maximum(p['j_r'] * p['T_r'] * np.abs(np.cos(ar_arr))
                         + p['S_r2'] * np.abs(np.sin(p['th_r2'])), 0.01)
    else:
        a_h = np.full_like(ar_arr, 0.22)
        b_h = np.full_like(ar_arr, 0.28)
    return np.maximum(a_h, 0.01), np.maximum(b_h, 0.01)


def _bogie_arm_h_vec(bb_arr, p):
    """Bogie 수평 투영 팔길이 — 배열 입력"""
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'linear':
        c_h = np.maximum(p['c_b'] * np.abs(np.cos(bb_arr)), 0.01)
        d_h = np.maximum(p['d_b'] * np.abs(np.cos(bb_arr)), 0.01)
    elif mode == 'triangle':
        ang_vert = -np.pi / 2 + bb_arr
        ang_wf = ang_vert + p['beta_b'] / 2
        ang_wm = ang_vert - p['beta_b'] / 2
        c_h = np.maximum(p['L_b1'] * np.abs(np.cos(ang_wf)), 0.01)
        d_h = np.maximum(p['L_b2'] * np.abs(np.cos(ang_wm)), 0.01)
    elif mode == 'frame':
        c_h = np.maximum(np.abs((1 - p['j_b']) * p['T_b'] * np.cos(bb_arr)
                                + p['S_b1'] * np.sin(p['th_b1'] + bb_arr)), 0.01)
        d_h = np.maximum(np.abs(p['j_b'] * p['T_b'] * np.cos(bb_arr)
                                + p['S_b2'] * np.sin(p['th_b2'] - bb_arr)), 0.01)
    else:
        c_h = np.full_like(bb_arr, 0.14)
        d_h = np.full_like(bb_arr, 0.14)
    return np.maximum(c_h, 0.01), np.maximum(d_h, 0.01)


def _rocker_arm_len(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle':
        return p['L_r1'], p['L_r2']
    elif mode == 'frame':
        L1 = np.sqrt(((1 - p['j_r']) * p['T_r'])**2 + p['S_r1']**2)
        L2 = np.sqrt((p['j_r'] * p['T_r'])**2 + p['S_r2']**2)
        return max(L1, 0.05), max(L2, 0.05)
    else:
        return p.get('a_r', 0.22), p.get('b_r', 0.28)


def _bogie_cb0(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle':
        return max(p['L_b1'] * abs(np.sin(p['beta_b'] / 2)), 0.04)
    elif mode == 'frame':
        return max(p['S_b1'] * abs(np.cos(p['th_b1'])), 0.04)
    else:
        return max(p.get('c_b', 0.14), 0.04)


def _bogie_db0(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle':
        return max(p['L_b2'] * abs(np.sin(p['beta_b'] / 2)), 0.04)
    elif mode == 'frame':
        return max(p['S_b2'] * abs(np.cos(p['th_b2'])), 0.04)
    else:
        return max(p.get('d_b', 0.14), 0.04)


def _smooth_filtfilt(arr, win):
    """scipy.signal.filtfilt 기반 제로위상 FIR 필터링.
    CPU 버전 calc_dynamics.py와 동일한 방식: filtfilt(ones(win)/win, 1, arr)
    이전 구현(수동 forward-backward convolution)은 에지 처리가 달라
    가속도 계산에 오차를 유발했음 → scipy 버전으로 교체.
    """
    from scipy.signal import filtfilt
    if len(arr) <= 3 * win:
        return arr.copy()
    kernel = np.ones(win) / win
    return filtfilt(kernel, 1.0, arr)


# ═══════════════════════════════════════
# 메인 함수: 벡터화된 동역학 계산
# ═══════════════════════════════════════

def calc_dynamics_gpu(R, x_arr, x_t, y_t, p):
    """동역학 계산 — for 루프 제거, 벡터 연산 기반

    R, x_arr, x_t, y_t는 numpy 배열을 기대합니다.
    내부에서 jnp 연산이 필요한 부분만 GPU로 오프로드합니다.
    """
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
    R_w = p['R_w']
    mass = p['mass']
    g = p['g']
    ge = gear_ratio * eta_gear

    N = len(x_arr)
    W = mass * g

    # 관성 모멘트 (스칼라, CPU)
    I_wheel = 0.5 * m_wheel * R_w**2
    L_r1_eff, L_r2_eff = _rocker_arm_len(p)
    L_r_tot = max(L_r1_eff + L_r2_eff, 1e-4)
    m_r1 = m_rocker_link * L_r1_eff / L_r_tot
    m_r2 = m_rocker_link * L_r2_eff / L_r_tot
    I_rocker = (1 / 3) * m_r1 * L_r1_eff**2 + (1 / 3) * m_r2 * L_r2_eff**2 + I_rocker_add
    cb0 = _bogie_cb0(p)
    db0 = _bogie_db0(p)
    I_bogie = m_wheel * cb0**2 + m_wheel * db0**2 + (1 / 12) * m_bogie_link * (cb0 + db0)**2 + I_bogie_add
    m_bogie_eff = I_bogie / (cb0**2 + 1e-9)

    # ── CG 가속도 (벡터) ──
    ycg = R.get('ycg', R['y0'] + p.get('h_body', 0.3) * 0.5)
    sm_win = max(5, N // 50)
    ycg_s = _smooth_filtfilt(ycg, sm_win)
    vy_cg = np.gradient(ycg_s, x_arr) * v_robot
    ay_cg = np.clip(_smooth_filtfilt(np.gradient(vy_cg, x_arr) * v_robot, sm_win), -3 * g, 3 * g)

    # ── 링크 각가속도 (벡터) ──
    sm2 = max(5, N // 40)
    ar_s = _smooth_filtfilt(R['ar'], sm2)
    alpha_rocker = np.clip(_smooth_filtfilt(np.gradient(np.gradient(ar_s, x_arr) * v_robot, x_arr) * v_robot, sm2), -50, 50)
    bb_s = _smooth_filtfilt(R['bb'], sm2)
    alpha_bogie = np.clip(_smooth_filtfilt(np.gradient(np.gradient(bb_s, x_arr) * v_robot, x_arr) * v_robot, sm2), -50, 50)

    # ── 경사각 (벡터) ──
    hf_ = np.interp(R['xwf'], x_t, y_t)
    hr_ = np.interp(R['xwr'], x_t, y_t)
    sp = R['xwf'] - R['xwr']
    sp_safe = np.where(np.abs(sp) > 1e-6, sp, 1e-6)
    theta_arr = np.arctan2(hf_ - hr_, sp_safe)

    # ── 에지 감지 (벡터) ──
    dth_dx = np.abs(np.gradient(theta_arr, x_arr))
    edge_mask = dth_dx > np.deg2rad(step_thresh) / 0.30

    # ── 관성력 (벡터) ──
    F_in_y = mass * ay_cg

    # ── 충격력 (벡터) ──
    F_impact = np.zeros(N)
    F_impact2 = np.zeros(N)
    dx_mean = np.mean(np.diff(x_arr)) if N > 1 else 1.0
    k_contact = 1e5

    edge_idx = np.where(edge_mask)[0]
    for i in edge_idx:
        v_app = min(max(abs(vy_cg[i]), 0.01), v_robot)
        dv = (1 + e_restitution) * v_app
        m_eff_c = m_wheel + m_bogie_eff
        t_c = max(np.pi * np.sqrt(m_eff_c / k_contact), 0.002)
        F_imp1 = m_eff_c * dv / (t_c + 1e-9)
        a_bog = F_imp1 * cb0 / (I_bogie + 1e-9)
        t_del = np.sqrt(np.pi / max(a_bog, 0.1))
        i2 = min(i + round(t_del * v_robot / dx_mean), N - 1)
        F_impact[i] = F_imp1
        F_impact2[i2] += F_imp1 * e_restitution

    F_imp_tot = F_impact + F_impact2

    # ── 법선력 분배 (벡터) ──
    a_h, b_h = _rocker_arm_h_vec(R['ar'], p)
    c_h, d_h = _bogie_arm_h_vec(R['bb'], p)

    a_eff = a_h + CG_offset * np.cos(R['ar'])
    b_eff = b_h - CG_offset * np.cos(R['ar'])
    ratio_fm = d_h / np.maximum(c_h, 1e-3)

    b_eff_safe = np.where(np.abs(b_eff) < 1e-3, np.sign(b_eff + 1e-9) * 1e-3, b_eff)
    ratio_rb = a_eff / b_eff_safe

    W_eff_raw = np.minimum(W * np.cos(theta_arr) + F_in_y + F_imp_tot, 5 * W)
    Nb_raw = W_eff_raw / (ratio_rb + 1)
    Nr_raw = Nb_raw * ratio_rb
    Nm_raw = Nb_raw / (1 + ratio_fm)
    Nf_raw = Nm_raw * ratio_fm

    liftoff_r = Nr_raw < 0
    liftoff_f = Nf_raw < 0
    Nr = np.maximum(Nr_raw, 0)
    Nm = np.maximum(Nm_raw, 0)
    Nf = np.maximum(Nf_raw, 0)

    # ── 구동력 (벡터) ──
    F_drv = np.abs(W * np.sin(theta_arr) + 0.02 * W * np.cos(theta_arr)) / 2
    N_tot = Nr + Nm + Nf
    N_tot_safe = np.maximum(N_tot, 1e-6)
    Fdr = F_drv * Nr / N_tot_safe
    Fdm = F_drv * Nm / N_tot_safe
    Fdf = F_drv * Nf / N_tot_safe

    # ── 토크 (벡터) ──
    tau_in_total = I_wheel * np.abs(ay_cg) / R_w
    tau_in_r = tau_in_total * Nr / N_tot_safe
    tau_in_m = tau_in_total * Nm / N_tot_safe
    tau_in_f = tau_in_total * Nf / N_tot_safe

    tau_rk = I_rocker * np.abs(alpha_rocker) / ge
    tau_rk_r = tau_rk * Nr / N_tot_safe
    tau_rk_m = tau_rk * Nm / N_tot_safe
    tau_rk_f = tau_rk * Nf / N_tot_safe

    tau_bg = I_bogie * np.abs(alpha_bogie) / ge
    N_bogie = np.maximum(Nm + Nf, 1e-6)
    tau_bg_m = tau_bg * Nm / N_bogie
    tau_bg_f = tau_bg * Nf / N_bogie

    tau_motor_r = (Fdr * R_w + tau_in_r + tau_rk_r) / ge
    tau_motor_m = (Fdm * R_w + tau_in_m + tau_rk_m + tau_bg_m + F_impact2 * R_w) / ge
    tau_motor_f = (Fdf * R_w + tau_in_f + tau_rk_f + tau_bg_f + F_impact * R_w) / ge

    # ── 마찰/견인 (벡터) ──
    MIN_N = 0.5
    slip_r = np.abs(Fdr) / (mu * np.maximum(Nr, MIN_N))
    slip_m = np.abs(Fdm) / (mu * np.maximum(Nm, MIN_N))
    slip_f = np.abs(Fdf) / (mu * np.maximum(Nf, MIN_N))

    # ── 실패 마스킹 ──
    if 'ok' in R:
        invalid = ~R['ok']
        for arr in [tau_motor_r, tau_motor_m, tau_motor_f, slip_r, slip_m, slip_f]:
            arr[invalid] = np.nan

    # ── 통계 ──
    tau_max_arr = np.maximum(np.maximum(tau_motor_r, tau_motor_m), tau_motor_f)
    stair_zone = np.convolve(edge_mask.astype(float), np.ones(11) / 11, mode='same') > 0
    valid_mask = R['ok'] if 'ok' in R else np.ones(N, dtype=bool)

    valid_stair = stair_zone & valid_mask
    if np.any(valid_stair):
        stair_torque_peak = float(np.nanpercentile(tau_max_arr[valid_stair], 95))
    elif np.any(valid_mask):
        stair_torque_peak = float(np.nanpercentile(tau_max_arr[valid_mask], 95))
    else:
        stair_torque_peak = 100.0

    valid_tau = tau_max_arr[valid_mask]
    stair_torque_max = float(np.nanmax(valid_tau)) if len(valid_tau) > 0 else 100.0

    D = {
        'Nr': Nr, 'Nm': Nm, 'Nf': Nf,
        'Nr_raw': Nr_raw, 'Nf_raw': Nf_raw,
        'liftoff_r': liftoff_r, 'liftoff_f': liftoff_f,
        'Fdr': Fdr, 'Fdm': Fdm, 'Fdf': Fdf,
        'tau_motor_r': tau_motor_r, 'tau_motor_m': tau_motor_m, 'tau_motor_f': tau_motor_f,
        'tau_rocker_inertia': tau_rk, 'tau_bogie_inertia': tau_bg,
        'alpha_rocker': alpha_rocker, 'alpha_bogie': alpha_bogie,
        'tau_max_arr': tau_max_arr, 'edge_mask': edge_mask,
        'theta_local': theta_arr, 'ay_cg': ay_cg,
        'stair_torque_peak': stair_torque_peak, 'stair_torque_max': stair_torque_max,
        'tau_link_max': float(np.max(tau_rk + tau_bg)),
        'tau_link_ratio': float(np.max(tau_rk + tau_bg)) / (stair_torque_max + 1e-9),
        'slip_r': slip_r, 'slip_m': slip_m, 'slip_f': slip_f,
    }
    return D
