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


def motor_tau_max(omega_motor, motor_tau_peak=39.0, omega_no_load_rpm=240.0):
    """모터 토크-속도 곡선 — 1차 선형 근사.

    BL70200 인휠 BLDC (gear 내장 1:5) + 48V 시스템:
      ω_no_load 휠 측 = 240 RPM ≈ 25.1 rad/s (= 2.51 m/s linear).
    τ_available(ω) = τ_peak · (1 − |ω|/ω_no_load), clip to [0.1·τ_peak, τ_peak].

    Args:
        omega_motor: 모터 측 각속도 (rad/s), 스칼라 또는 배열
        motor_tau_peak: 스톨 토크 (Nm)
        omega_no_load_rpm: 무부하 RPM (배터리 전압 × KV)

    Returns:
        τ_max(ω): 동일 shape, 단위 Nm
    """
    omega_no_load = omega_no_load_rpm * 2.0 * np.pi / 60.0
    factor = np.clip(1.0 - np.abs(omega_motor) / omega_no_load, 0.1, 1.0)
    return motor_tau_peak * factor


def trap_velocity_profile(xa, v_max, a_lim):
    """사다리꼴 속도 프로파일 v(x): 0 → v_max(가속) → cruise → v_max → 0 (감속).

    Args:
        xa: 평가 위치 배열 [N]
        v_max: 최대(cruise) 속도 [m/s]
        a_lim: 가속도 한계 [m/s²], 가속/감속 동일

    Returns:
        v_arr: 각 x에서의 종방향 속도 [N]
        a_arr: 각 x에서의 종방향 가속도 [N], 가속+/감속-/cruise=0
    """
    x0 = xa[0]
    x1 = xa[-1]
    L = max(x1 - x0, 1e-6)
    x_acc_dist = v_max**2 / (2.0 * a_lim)  # 가속 구간 길이
    d = xa - x0     # 진입 후 거리
    rem = x1 - xa   # 잔여 거리

    if 2.0 * x_acc_dist >= L:
        # 삼각형 프로파일 (v_max 미도달)
        v_peak = np.sqrt(a_lim * L)
        half = (x0 + x1) / 2.0
        in_acc = xa < half
        v_arr = np.where(in_acc,
                         np.sqrt(2.0 * a_lim * np.maximum(d, 1e-9)),
                         np.sqrt(2.0 * a_lim * np.maximum(rem, 1e-9)))
        v_arr = np.minimum(v_arr, v_peak)
        a_arr = np.where(in_acc, a_lim, -a_lim)
    else:
        x_acc_end = x0 + x_acc_dist
        x_dec_start = x1 - x_acc_dist
        in_acc = xa < x_acc_end
        in_dec = xa > x_dec_start
        v_arr = np.where(in_acc, np.sqrt(2.0 * a_lim * np.maximum(d, 1e-9)),
                np.where(in_dec, np.sqrt(2.0 * a_lim * np.maximum(rem, 1e-9)),
                                 v_max))
        v_arr = np.minimum(v_arr, v_max)
        a_arr = np.where(in_acc, a_lim,
                np.where(in_dec, -a_lim, 0.0))

    v_arr = np.maximum(v_arr, 0.05)  # 시간 미분 발산 방지
    return v_arr.astype(np.float64), a_arr.astype(np.float64)


# ═══════════════════════════════════════
# 메인 함수: 벡터화된 동역학 계산
# ═══════════════════════════════════════

def calc_dynamics_gpu(R, x_arr, x_t, y_t, p):
    """동역학 계산 — for 루프 제거, 벡터 연산 기반

    R, x_arr, x_t, y_t는 numpy 배열을 기대합니다.
    내부에서 jnp 연산이 필요한 부분만 GPU로 오프로드합니다.
    """
    mu = p.get('mu', 0.70)
    gear_ratio = p.get('gear_ratio', 1.0)  # hub motor 내장기어 → 외부 1.0
    eta_gear = p.get('eta_gear', 1.0)
    motor_tau_peak = p.get('motor_tau_peak', 39.0)  # 휠 측 Nm (BL70200)
    m_wheel = p.get('m_wheel', 1.2)
    m_rocker_link = p.get('m_rocker_link', 0.8)
    m_bogie_link = p.get('m_bogie_link', 0.5)
    I_rocker_add = p.get('I_rocker_add', 0)
    I_bogie_add = p.get('I_bogie_add', 0)
    e_restitution = p.get('e_restitution', 0.3)
    v_max_cfg = p.get('v_max', p.get('v_robot', 1.0))
    a_lim_cfg = p.get('a_lim', 1.5)
    step_thresh = p.get('step_thresh', 5.0)
    CG_offset = p.get('CG_offset', 0)
    R_w = p['R_w']
    mass = p['mass']
    g = p['g']
    ge = gear_ratio * eta_gear
    # Phase 3+: 연속 정격 토크 (열적), 모터 전류, 배터리 한계
    motor_tau_cont = p.get('motor_tau_cont', 22.0)   # 휠 측 정격
    Kt_eff = p.get('Kt_eff', motor_tau_cont / 9.0)   # τ/I 비 (BL70200: 22/9 ≈ 2.44 Nm/A)
    battery_max_current = p.get('battery_max_current', 30.0)  # 배터리 연속 한계 (A)
    n_wheel_total = p.get('n_wheel_total', 6)        # 6륜 (좌우 대칭 → 측면 절반 ×2)

    N = len(x_arr)
    W = mass * g
    # ── 측면-절반 모델 일관성 (모델 휠 1개 = 한쪽 면 실제 모터 1개) ──
    # 토크는 모터당 한계(motor_tau_peak)와 비교되므로 각 모델 휠은 한쪽 면의 실모터 1개에 대응.
    # 따라서 한쪽 면이 지지하는 정적 하중은 로봇 전체의 절반(W_side, mass_side)이어야 한다.
    # 구동력/토크/에너지/전류는 종전과 수치 동일하게 유지된다 — F_drv가 (0.5·W) 사용 + 기존 /2 제거로
    # 상쇄되기 때문(아래 참고). 오직 법선력 N만 면-기준(절반)으로 바뀌어,
    # slip = Fd/(μ·N) 가 올바른 면-기준 슬립률로 정정된다(종전엔 N에 전체 W를 써 ~2× 낙관적이었음).
    W_side = 0.5 * W
    mass_side = 0.5 * mass

    # ── 사다리꼴 속도 프로파일 (Phase 1: 가속/감속 포함) ──
    # R['v_arr']가 사전 계산되어 전달되었으면 재사용, 아니면 계산.
    if 'v_arr' in R and 'a_long' in R and len(R['v_arr']) == N:
        v_arr = np.asarray(R['v_arr'], dtype=np.float64)
        a_long = np.asarray(R['a_long'], dtype=np.float64)
    else:
        v_arr, a_long = trap_velocity_profile(x_arr, v_max_cfg, a_lim_cfg)

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

    # ── CG 가속도 (벡터) — v(x) 프로파일로 dy/dt = (dy/dx)·v(x) ──
    ycg = R.get('ycg', R['y0'] + p.get('h_body', 0.3) * 0.5)
    sm_win = max(5, N // 50)
    ycg_s = _smooth_filtfilt(ycg, sm_win)
    vy_cg = np.gradient(ycg_s, x_arr) * v_arr
    ay_cg = np.clip(_smooth_filtfilt(np.gradient(vy_cg, x_arr) * v_arr, sm_win), -3 * g, 3 * g)

    # ── 링크 각가속도 (벡터) ──
    sm2 = max(5, N // 40)
    ar_s = _smooth_filtfilt(R['ar'], sm2)
    alpha_rocker = np.clip(_smooth_filtfilt(np.gradient(np.gradient(ar_s, x_arr) * v_arr, x_arr) * v_arr, sm2), -50, 50)
    bb_s = _smooth_filtfilt(R['bb'], sm2)
    alpha_bogie = np.clip(_smooth_filtfilt(np.gradient(np.gradient(bb_s, x_arr) * v_arr, x_arr) * v_arr, sm2), -50, 50)

    # ── 경사각 (벡터) ──
    hf_ = np.interp(R['xwf'], x_t, y_t)
    hr_ = np.interp(R['xwr'], x_t, y_t)
    sp = R['xwf'] - R['xwr']
    sp_safe = np.where(np.abs(sp) > 1e-6, sp, 1e-6)
    theta_arr = np.arctan2(hf_ - hr_, sp_safe)

    # ── 에지 감지 (벡터) ──
    dth_dx = np.abs(np.gradient(theta_arr, x_arr))
    edge_mask = dth_dx > np.deg2rad(step_thresh) / 0.30

    # ── 관성력 (벡터, 면-기준) ──
    F_in_y = mass_side * ay_cg

    # ── 충격력 (벡터) ──
    F_impact = np.zeros(N)
    F_impact2 = np.zeros(N)
    dx_mean = np.mean(np.diff(x_arr)) if N > 1 else 1.0
    k_contact = 1e5

    edge_idx = np.where(edge_mask)[0]
    for i in edge_idx:
        v_loc = max(v_arr[i], 0.05)
        v_app = min(max(abs(vy_cg[i]), 0.01), v_loc)
        dv = (1 + e_restitution) * v_app
        m_eff_c = m_wheel + m_bogie_eff
        t_c = max(np.pi * np.sqrt(m_eff_c / k_contact), 0.002)
        F_imp1 = m_eff_c * dv / (t_c + 1e-9)
        a_bog = F_imp1 * cb0 / (I_bogie + 1e-9)
        t_del = np.sqrt(np.pi / max(a_bog, 0.1))
        i2 = min(i + round(t_del * v_loc / dx_mean), N - 1)
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

    W_eff_raw = np.minimum(W_side * np.cos(theta_arr) + F_in_y + F_imp_tot, 5 * W_side)
    Nb_raw = W_eff_raw / (ratio_rb + 1)
    Nr_raw = Nb_raw * ratio_rb
    Nm_raw = Nb_raw / (1 + ratio_fm)
    Nf_raw = Nm_raw * ratio_fm

    liftoff_r = Nr_raw < 0
    liftoff_f = Nf_raw < 0
    Nr = np.maximum(Nr_raw, 0)
    Nm = np.maximum(Nm_raw, 0)
    Nf = np.maximum(Nf_raw, 0)

    # ── 구동력 (벡터, 면-기준) — 종방향 관성 포함 ──
    # W_side(=0.5·W)·mass_side 사용 + 기존 /2 제거 → 종전과 수치적으로 동일(한쪽 면의 종방향 부담).
    F_drv = np.abs(W_side * np.sin(theta_arr) + 0.02 * W_side * np.cos(theta_arr) + mass_side * a_long)
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

    # ── 마찰/견인 (벡터) — Phase 2c: 슬립률 = F_demand / (μ·N) ──
    MIN_N = 0.5
    slip_r = np.abs(Fdr) / (mu * np.maximum(Nr, MIN_N))
    slip_m = np.abs(Fdm) / (mu * np.maximum(Nm, MIN_N))
    slip_f = np.abs(Fdf) / (mu * np.maximum(Nf, MIN_N))
    # 포인트별 최대 슬립률 (어떤 휠이든 슬립하면 그 포인트는 미끄러짐)
    slip_max_per_pt = np.maximum(np.maximum(slip_r, slip_m), slip_f)

    # ── 실패 마스킹 ──
    if 'ok' in R:
        invalid = ~R['ok']
        for arr in [tau_motor_r, tau_motor_m, tau_motor_f,
                    slip_r, slip_m, slip_f, slip_max_per_pt]:
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

    # ── 모터 토크-속도 곡선 (Phase 2d) ──
    # 각 포인트의 휠 각속도 → 모터 측 각속도 → τ_max(ω). 토크가 이를 초과하면 포화.
    omega_motor = (v_arr / R_w) * gear_ratio   # 모터 측 각속도 (rad/s)
    omega_no_load_rpm = p.get('omega_no_load_rpm', 3600.0)
    tau_avail_motor = motor_tau_max(omega_motor, motor_tau_peak=motor_tau_peak,
                                    omega_no_load_rpm=omega_no_load_rpm)  # [N]

    # 각 포인트에서 가용 토크 대비 요구 토크 비율 (Phase 1 sat_pct를 속도 인식 형태로 진화).
    tau_demand_max = np.maximum(np.maximum(tau_motor_r, tau_motor_m), tau_motor_f)
    sat_per_pt = tau_demand_max / np.maximum(tau_avail_motor, 0.1)
    if 'ok' in R:
        sat_per_pt[~R['ok']] = np.nan
    sat_valid = sat_per_pt[np.isfinite(sat_per_pt)]
    if len(sat_valid) > 0:
        sat_peak_speed_aware = float(np.max(sat_valid))
        sat_p95_speed_aware = float(np.percentile(sat_valid, 95))
        sat_violation_rate = float(np.mean(sat_valid > 1.0))
    else:
        sat_peak_speed_aware = 0.0
        sat_p95_speed_aware = 0.0
        sat_violation_rate = 0.0

    # ── Phase 3+: 연속 토크 (RMS) / 에너지 / 배터리 전류 ──
    valid_mask_for_thermal = R['ok'] if 'ok' in R else np.ones(N, dtype=bool)

    # 휠별 토크 RMS (열적 연속 한계 비교용). 측면 절반 모델에선 좌/우 대칭 가정.
    tau_per_wheel_arr = np.stack([tau_motor_f, tau_motor_m, tau_motor_r], axis=1)  # [N, 3]
    tau_per_wheel_valid = tau_per_wheel_arr[valid_mask_for_thermal]
    if tau_per_wheel_valid.shape[0] > 0:
        # 휠별 RMS의 최대값 (worst wheel 기준)
        tau_rms_per_wheel = np.sqrt(np.nanmean(tau_per_wheel_valid**2, axis=0))
        tau_rms_worst = float(np.max(tau_rms_per_wheel))
        cont_violation_rate = float(np.mean(
            np.nanmax(np.abs(tau_per_wheel_valid), axis=1) > motor_tau_cont
        ))
    else:
        tau_rms_worst = 0.0
        cont_violation_rate = 0.0

    # 에너지 계산: P = τ · ω (motor side ≡ wheel side since gear=1)
    # 좌/우 대칭 ×2 → 측면 절반 모델의 6륜 시뮬: 각 휠 ×2.
    omega_wheel = v_arr / R_w  # 휠 각속도 (rad/s)
    # 모터 토크 절대값 (각 휠) × ω × 2 (좌우 대칭)
    power_total = (np.abs(tau_motor_f) + np.abs(tau_motor_m) + np.abs(tau_motor_r)) * omega_wheel * 2.0
    power_valid = power_total[valid_mask_for_thermal]

    # 시간 적분: dt(i) = dx(i) / v(i). dx 균일분포 가정 → mean(dx)/v 사용.
    dx_arr = np.diff(x_arr) if len(x_arr) > 1 else np.array([0.01])
    dx_mean_val = float(np.mean(dx_arr))
    dt_per_pt = dx_mean_val / np.maximum(v_arr, 0.05)
    energy_J = float(np.sum(power_total * dt_per_pt))
    energy_Wh = energy_J / 3600.0
    total_time = float(np.sum(dt_per_pt))
    avg_power_W = energy_J / max(total_time, 1e-6)

    # 배터리 전류: 각 시점에 모든 모터의 합 전류
    # I_motor = |τ| / Kt_eff. 측면 절반 ×2 (좌우 대칭).
    current_per_pt = (np.abs(tau_motor_f) + np.abs(tau_motor_m) + np.abs(tau_motor_r)) / Kt_eff * 2.0
    current_per_pt[~valid_mask_for_thermal] = np.nan
    current_valid = current_per_pt[np.isfinite(current_per_pt)]
    if len(current_valid) > 0:
        battery_current_peak = float(np.max(current_valid))
        battery_current_p95 = float(np.percentile(current_valid, 95))
        battery_violation_rate = float(np.mean(current_valid > battery_max_current))
    else:
        battery_current_peak = 0.0
        battery_current_p95 = 0.0
        battery_violation_rate = 0.0

    # ── 슬립 통계 (Phase 2c) ──
    valid_mask_for_slip = R['ok'] if 'ok' in R else np.ones(N, dtype=bool)
    slip_valid = slip_max_per_pt[valid_mask_for_slip] if np.any(valid_mask_for_slip) else slip_max_per_pt
    slip_valid_finite = slip_valid[np.isfinite(slip_valid)]
    if len(slip_valid_finite) > 0:
        slip_peak = float(np.max(slip_valid_finite))
        slip_violation_rate = float(np.mean(slip_valid_finite > 1.0))
        slip_p95 = float(np.percentile(slip_valid_finite, 95))
    else:
        slip_peak = 0.0
        slip_violation_rate = 0.0
        slip_p95 = 0.0

    # ── Phase 3+ (B-1): 시스템 레벨 견인력 한계 ──
    # 휠 개별 슬립과 별도: 전체 견인력이 종방향 요구를 못 따라가는 "stuck" 케이스.
    # F_traction_capacity = μ · ΣN (한쪽 면 합, 이제 면-기준 절반 스케일)
    # F_demand_long = |mass_side·a_long + W_side·sin(θ)|  (가속+중력 종방향 성분, 면-기준)
    # 분자·분모 모두 면-기준(절반)이라 traction_util 비율은 종전과 동일하게 유지된다.
    F_traction_cap = mu * (Nr + Nm + Nf)
    F_demand_long = np.abs(mass_side * a_long + W_side * np.sin(theta_arr))
    traction_util = F_demand_long / np.maximum(F_traction_cap, 1.0)
    if 'ok' in R:
        traction_util[~R['ok']] = np.nan
    traction_util_valid = traction_util[np.isfinite(traction_util)]
    if len(traction_util_valid) > 0:
        traction_util_peak = float(np.max(traction_util_valid))
        system_stuck_rate = float(np.mean(traction_util_valid > 1.0))
    else:
        traction_util_peak = 0.0
        system_stuck_rate = 0.0

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
        'v_arr': v_arr, 'a_long': a_long,
        'stair_torque_peak': stair_torque_peak, 'stair_torque_max': stair_torque_max,
        'tau_link_max': float(np.max(tau_rk + tau_bg)),
        'tau_link_ratio': float(np.max(tau_rk + tau_bg)) / (stair_torque_max + 1e-9),
        'slip_r': slip_r, 'slip_m': slip_m, 'slip_f': slip_f,
        'slip_max_per_pt': slip_max_per_pt,
        'slip_peak': slip_peak,
        'slip_p95': slip_p95,
        'slip_violation_rate': slip_violation_rate,
        # Phase 2d: 속도 인식 모터 포화
        'tau_avail_motor': tau_avail_motor,
        'sat_peak_speed_aware': sat_peak_speed_aware,
        'sat_p95_speed_aware': sat_p95_speed_aware,
        'sat_violation_rate': sat_violation_rate,
        # Phase 3+ A-1: 연속 정격 (열적) 한계
        'tau_rms_worst': tau_rms_worst,
        'cont_violation_rate': cont_violation_rate,
        # Phase 3+ A-2: 에너지
        'energy_J': energy_J,
        'energy_Wh': energy_Wh,
        'avg_power_W': avg_power_W,
        'total_time_s': total_time,
        # Phase 3+ B-1: 시스템 견인력
        'traction_util_peak': traction_util_peak,
        'system_stuck_rate': system_stuck_rate,
        # Phase 3+ B-2: 배터리 전류
        'current_per_pt': current_per_pt,
        'battery_current_peak': battery_current_peak,
        'battery_current_p95': battery_current_p95,
        'battery_violation_rate': battery_violation_rate,
    }
    return D
