"""
calc_stability_jax: 전복 안정성 분석 — 벡터화 GPU 가속 버전

원본의 포인트별 for 루프를 numpy 벡터 연산으로 대체하여
GPU에서 JAX를 통해 wpos 배치 계산 후 numpy로 후처리합니다.
"""
import numpy as np
import jax.numpy as jnp
from .wpos_jax import wpos_batched, pack_params_auto


def calc_stability_gpu(R, x_arr, x_t, y_t_raw, y_t_env, p):
    """전복 안정성 분석 — 벡터화 버전"""
    CG_offset = p.get('CG_offset', 0)
    liftoff_max = p.get('liftoff_max', 0.02)
    phi_r0 = p.get('phi_r0', 0)

    N = len(x_arr)
    W = p['mass'] * p['g']

    x_cg = R.get('xcg', x_arr + CG_offset * np.cos(R['ar']))

    ok = R['ok'] if 'ok' in R else np.ones(N, dtype=bool)

    # ── 법선력 (벡터 연산) ──
    xwf, xwm, xwr = R['xwf'], R['xwm'], R['xwr']
    sp = xwf - xwr
    sp_safe = np.where(np.abs(sp) > 1e-6, sp, 1e-6)

    hf_ = np.interp(xwf, x_t, y_t_env)
    hr_ = np.interp(xwr, x_t, y_t_env)
    theta = np.arctan2(hf_ - hr_, sp_safe)

    # 팔길이 (벡터)
    from .calc_dynamics_jax import _rocker_arm_h_vec, _bogie_arm_h_vec
    a_h, b_h = _rocker_arm_h_vec(R['ar'], p)
    c_h, d_h = _bogie_arm_h_vec(R['bb'], p)

    W_cos = W * np.cos(theta)
    a_eff = a_h + CG_offset * np.cos(R['ar'])
    b_eff = b_h - CG_offset * np.cos(R['ar'])
    ratio_fm = d_h / np.maximum(c_h, 1e-3)
    b_eff_safe = np.where(np.abs(b_eff) < 1e-3, np.sign(b_eff + 1e-9) * 1e-3, b_eff)
    ratio_rb = a_eff / b_eff_safe

    Nb_raw = W_cos / (ratio_rb + 1)
    Nr_raw = Nb_raw * ratio_rb
    Nm_raw = Nb_raw / (1 + ratio_fm)
    Nf_raw = Nm_raw * ratio_fm

    liftoff_r = Nr_raw < 0
    liftoff_f = Nf_raw < 0

    # ── ZMP (벡터) ──
    N_total = Nr_raw + Nm_raw + Nf_raw
    x_zmp = np.where(
        N_total > W * 0.05,
        (Nr_raw * xwr + Nm_raw * xwm + Nf_raw * xwf) / np.maximum(N_total, 1e-9),
        x_cg
    )

    sp_min = np.minimum(xwf, xwr)
    sp_max = np.maximum(xwf, xwr)
    zmp_margin = np.minimum(x_zmp - sp_min, sp_max - x_zmp)
    zmp_ok = (x_zmp >= sp_min) & (x_zmp <= sp_max)

    # ── TOI (벡터) ──
    sp_width = sp_max - sp_min
    narrow = sp_width < 0.05
    TOI_f = np.clip((sp_max - x_cg) / np.maximum(sp_width, 0.01), -2.0, 2.0)
    TOI_r = np.clip((x_cg - sp_min) / np.maximum(sp_width, 0.01), -2.0, 2.0)
    TOI_f = np.where(narrow, 0.5, TOI_f)
    TOI_r = np.where(narrow, 0.5, TOI_r)
    TOI = np.minimum(TOI_f, TOI_r)

    # 무효 포인트 NaN 처리
    invalid = ~ok
    for arr in [x_zmp, zmp_margin, TOI, TOI_f, TOI_r, Nr_raw, Nf_raw]:
        arr[invalid] = np.nan
    zmp_ok[invalid] = False

    # ── 차체 간섭 검사 (GPU 배치 wpos + 완전 벡터화 numpy) ──
    p_arr = pack_params_auto(p)
    X_batch = np.stack([R['y0'], R['ar'], R['bb']], axis=1)
    X_batch_j = jnp.array(X_batch)
    x_arr_j = jnp.array(x_arr)

    positions = np.array(wpos_batched(X_batch_j, x_arr_j, p_arr))
    # positions: [N, 10] = [Wf_x, Wf_y, Wm_x, Wm_y, Wr_x, Wr_y, Pb_x, Pb_y, CG_x, CG_y]

    N_div = 20
    t_div = np.linspace(0, 1, N_div, dtype=np.float32)  # [N_div]

    rocker_mode = p.get('rocker_mode', 'linear').lower()
    bogie_mode = p.get('bogie_mode', 'linear').lower()

    # [N, 2] 배열로 각 포인트 추출
    P0 = np.stack([x_arr, R['y0']], axis=1)   # [N, 2]
    Wf = positions[:, 0:2]                     # [N, 2]
    Wm = positions[:, 2:4]
    Wr = positions[:, 4:6]
    Pb = positions[:, 6:8]

    def seg_min_clearance(A, B):
        """선분 A→B 상 N_div 샘플 포인트의 지형 여유공간 최솟값 [N]
        A, B: [N, 2]  →  반환: [N]
        """
        # pts: [N, N_div, 2]
        pts = A[:, None, :] + t_div[None, :, None] * (B - A)[:, None, :]
        px = pts[:, :, 0].ravel()   # [N*N_div]
        py = pts[:, :, 1].ravel()
        ty = np.interp(px, x_t, y_t_raw)
        return (py - ty).reshape(N, N_div).min(axis=1)  # [N]

    # 로커 선분 목록 생성 (벡터화)
    if rocker_mode == 'frame':
        ar_e = R['ar'] + phi_r0               # [N]
        ur = np.stack([np.cos(ar_e), np.sin(ar_e)], axis=1)  # [N, 2]
        Ptr = P0 - p['j_r'] * p['T_r'] * ur
        Ptf = P0 + (1 - p['j_r']) * p['T_r'] * ur
        rocker_segs = [(Ptr, Ptf), (Ptf, Pb), (Ptr, Wr)]
    else:
        rocker_segs = [(P0, Pb), (P0, Wr)]

    # 보기 선분 목록 생성 (벡터화)
    if bogie_mode == 'frame':
        ubb = np.stack([np.cos(R['bb']), np.sin(R['bb'])], axis=1)  # [N, 2]
        Pbm = Pb - p['j_b'] * p['T_b'] * ubb
        Pbf = Pb + (1 - p['j_b']) * p['T_b'] * ubb
        bogie_segs = [(Pbm, Pbf), (Pbf, Wf), (Pbm, Wm)]
    else:
        bogie_segs = [(Pb, Wf), (Pb, Wm)]

    # 모든 선분의 여유공간 최솟값 (각 선분마다 1회 np.interp 배치 호출)
    clearance = np.full(N, np.inf)
    for A, B in rocker_segs + bogie_segs:
        clearance = np.minimum(clearance, seg_min_clearance(A, B))

    clearance[~ok] = np.nan

    min_clearance = float(np.nanmin(clearance))
    is_collision = min_clearance < 0.01

    # ── 통계 ──
    n_liftoff = int(np.sum(liftoff_r | liftoff_f))
    liftoff_ratio = n_liftoff / N
    n_zmp_out = int(np.sum(~zmp_ok))
    pct_zmpout = n_zmp_out / N

    if np.nanmin(TOI) < 0 or liftoff_ratio > liftoff_max or pct_zmpout > 0.50 or is_collision:
        risk_level = 'danger'
    elif np.nanmin(TOI) < 0.15 or liftoff_ratio > liftoff_max * 0.5 or pct_zmpout > 0.20:
        risk_level = 'warning'
    else:
        risk_level = 'safe'

    S = {
        'x_zmp': x_zmp, 'x_sp_min': sp_min, 'x_sp_max': sp_max,
        'zmp_margin': zmp_margin, 'zmp_ok': zmp_ok,
        'TOI': TOI, 'TOI_front': TOI_f, 'TOI_rear': TOI_r,
        'Nr_raw': Nr_raw, 'Nf_raw': Nf_raw,
        'liftoff_r': liftoff_r, 'liftoff_f': liftoff_f,
        'clearance': clearance, 'min_clearance': min_clearance,
        'is_collision': is_collision,
        'min_TOI': float(np.nanmin(TOI)),
        'min_zmp_margin': float(np.nanmin(zmp_margin)),
        'n_liftoff': n_liftoff, 'liftoff_ratio': liftoff_ratio,
        'n_zmp_out': n_zmp_out, 'risk_level': risk_level,
    }

    badges = {'safe': 'SAFE', 'warning': 'WARNING', 'danger': 'DANGER'}
    col_warn = ' (간섭!)' if is_collision else ''
    print(f'  [안정성] {badges[risk_level]}  TOI_min={S["min_TOI"]:.3f}  '
          f'ZMP이탈={pct_zmpout*100:.1f}%  들림={liftoff_ratio*100:.1f}%  '
          f'여유공간={min_clearance:.3f}m{col_warn}')

    return S
