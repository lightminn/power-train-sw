"""
wpos_jax: 순기구학 — JAX GPU 가속 버전

핵심 변경:
  - numpy → jax.numpy
  - dict 기반 파라미터 → 고정 크기 배열 (JIT 호환)
  - vmap 적용으로 N개 포인트 동시 계산 가능

파라미터 배열(p_arr) 레이아웃 — 총 21개 요소:
  [0]  R_w
  [1]  phi_r0
  [2]  h_body
  [3]  CG_offset
  [4]  h_CG
  [5]  rocker_mode  (0=linear, 1=triangle, 2=frame)
  [6]  bogie_mode   (0=linear, 1=triangle, 2=frame)
  --- rocker ---
  [7]  a_r / L_r1 / T_r
  [8]  b_r / L_r2 / S_r1
  [9]  alpha_r / S_r2
  [10] th_r1
  [11] th_r2
  [12] j_r
  [13] delta_pb
  --- bogie ---
  [14] c_b / L_b1 / T_b
  [15] d_b / L_b2 / S_b1
  [16] beta_b / S_b2
  [17] th_b1
  [18] th_b2
  [19] j_b
  --- bracket ---
  [20] brk_v  (bracket pivot → wheel axle 수직 오프셋, m. 기본 0)
"""
import jax
import jax.numpy as jnp
from functools import partial

# ═══════════════════════════════════════
# 모드별 Rocker 기구학
# ═══════════════════════════════════════

def _rocker_linear(P0, ar_eff, p_arr):
    a_r = p_arr[7]
    b_r = p_arr[8]
    delta_pb = p_arr[13]
    u_r = jnp.array([jnp.cos(ar_eff), jnp.sin(ar_eff)])
    n_r = jnp.array([-jnp.sin(ar_eff), jnp.cos(ar_eff)])
    Pb = P0 + a_r * u_r + delta_pb * n_r
    Wr = P0 - b_r * u_r
    return Pb, Wr


def _rocker_triangle(P0, ar_eff, p_arr):
    L_r1 = p_arr[7]
    L_r2 = p_arr[8]
    alpha_r = p_arr[9]
    ang_pb = ar_eff - alpha_r / 2
    ang_wr = ar_eff - jnp.pi + alpha_r / 2
    Pb = P0 + L_r1 * jnp.array([jnp.cos(ang_pb), jnp.sin(ang_pb)])
    Wr = P0 + L_r2 * jnp.array([jnp.cos(ang_wr), jnp.sin(ang_wr)])
    return Pb, Wr


def _rocker_frame(P0, ar_eff, p_arr):
    T_r = p_arr[7]
    S_r1 = p_arr[8]
    S_r2 = p_arr[9]
    th_r1 = p_arr[10]
    th_r2 = p_arr[11]
    j_r = p_arr[12]
    u_r = jnp.array([jnp.cos(ar_eff), jnp.sin(ar_eff)])
    n_r = jnp.array([-jnp.sin(ar_eff), jnp.cos(ar_eff)])
    Pb = (P0 + (1 - j_r) * T_r * u_r
          + S_r1 * (jnp.sin(th_r1) * u_r - jnp.cos(th_r1) * n_r))
    Wr = (P0 - j_r * T_r * u_r
          + S_r2 * (-jnp.sin(th_r2) * u_r - jnp.cos(th_r2) * n_r))
    return Pb, Wr


# ═══════════════════════════════════════
# 모드별 Bogie 기구학
# ═══════════════════════════════════════

def _bogie_linear(Pb, bb, p_arr):
    c_b = p_arr[14]
    d_b = p_arr[15]
    u = jnp.array([jnp.cos(bb), jnp.sin(bb)])
    Wf = Pb + c_b * u
    Wm = Pb - d_b * u
    return Wf, Wm


def _bogie_triangle(Pb, bb, p_arr):
    L_b1 = p_arr[14]
    L_b2 = p_arr[15]
    beta_b = p_arr[16]
    ang_vert = -jnp.pi / 2 + bb
    ang_wf = ang_vert + beta_b / 2
    ang_wm = ang_vert - beta_b / 2
    Wf = Pb + L_b1 * jnp.array([jnp.cos(ang_wf), jnp.sin(ang_wf)])
    Wm = Pb + L_b2 * jnp.array([jnp.cos(ang_wm), jnp.sin(ang_wm)])
    return Wf, Wm


def _bogie_frame(Pb, bb, p_arr):
    T_b = p_arr[14]
    S_b1 = p_arr[15]
    S_b2 = p_arr[16]
    th_b1 = p_arr[17]
    th_b2 = p_arr[18]
    j_b = p_arr[19]
    u_b = jnp.array([jnp.cos(bb), jnp.sin(bb)])
    n_b = jnp.array([-jnp.sin(bb), jnp.cos(bb)])
    Wf = (Pb + (1 - j_b) * T_b * u_b
          + S_b1 * (jnp.sin(th_b1) * u_b - jnp.cos(th_b1) * n_b))
    Wm = (Pb - j_b * T_b * u_b
          + S_b2 * (-jnp.sin(th_b2) * u_b - jnp.cos(th_b2) * n_b))
    return Wf, Wm


# ═══════════════════════════════════════
# 통합 순기구학 (lax.switch로 JIT 호환 분기)
# ═══════════════════════════════════════

@jax.jit
def wpos_jax(X, xb, p_arr):
    """순기구학 — JAX JIT 호환 (모든 모드 지원)

    Args:
        X: [y0, ar, bb] (3-element)
        xb: Rocker pivot x 좌표 (scalar)
        p_arr: 파라미터 배열 (21-element)

    Returns:
        result: [Wf_x, Wf_y, Wm_x, Wm_y, Wr_x, Wr_y, Pb_x, Pb_y, CG_x, CG_y]
        — Wf/Wm/Wr는 휠 축(axle) 위치이며, brk_v가 0이 아닐 경우
          링크 끝점(브래킷 피벗)에서 y축으로 brk_v만큼 내려간 좌표.
    """
    y0, ar, bb = X[0], X[1], X[2]
    phi_r0 = p_arr[1]
    CG_offset = p_arr[3]
    h_CG = p_arr[4]
    rocker_mode = p_arr[5].astype(jnp.int32)
    bogie_mode = p_arr[6].astype(jnp.int32)
    brk_v = p_arr[20]

    P0 = jnp.array([xb, y0])
    ar_eff = ar + phi_r0

    # Rocker 분기 (lax.switch: JIT에서 if/else 대체)
    Pb, Wr = jax.lax.switch(
        rocker_mode,
        [
            lambda: _rocker_linear(P0, ar_eff, p_arr),
            lambda: _rocker_triangle(P0, ar_eff, p_arr),
            lambda: _rocker_frame(P0, ar_eff, p_arr),
        ]
    )

    # Bogie 분기
    Wf, Wm = jax.lax.switch(
        bogie_mode,
        [
            lambda: _bogie_linear(Pb, bb, p_arr),
            lambda: _bogie_triangle(Pb, bb, p_arr),
            lambda: _bogie_frame(Pb, bb, p_arr),
        ]
    )

    # 브래킷 피벗 → 휠 축 변환: 링크 끝점에서 y축으로 brk_v만큼 하강
    brk_off = jnp.array([0.0, brk_v])
    Wf = Wf - brk_off
    Wm = Wm - brk_off
    Wr = Wr - brk_off

    # CG 위치
    u_h = jnp.array([jnp.cos(ar_eff), jnp.sin(ar_eff)])
    n_h = jnp.array([-jnp.sin(ar_eff), jnp.cos(ar_eff)])
    CG = P0 + CG_offset * u_h + h_CG * n_h

    return jnp.array([Wf[0], Wf[1], Wm[0], Wm[1], Wr[0], Wr[1],
                      Pb[0], Pb[1], CG[0], CG[1]])


# N개 포인트를 GPU에서 동시에 계산
wpos_batched = jax.vmap(wpos_jax, in_axes=(0, 0, None))


# ═══════════════════════════════════════
# 파라미터 변환 헬퍼
# ═══════════════════════════════════════

MODE_MAP = {'linear': 0, 'triangle': 1, 'frame': 2}


def pack_params_frame(p):
    """dict → JAX 배열 (frame-frame 모드용)"""
    return jnp.array([
        p['R_w'],                           # [0]
        p.get('phi_r0', 0.0),               # [1]
        p.get('h_body', 0.3),               # [2]
        p.get('CG_offset', 0.0),            # [3]
        p.get('h_CG', p.get('h_body', 0.3) * 0.5),  # [4]
        MODE_MAP.get(p.get('rocker_mode', 'frame'), 2),  # [5]
        MODE_MAP.get(p.get('bogie_mode', 'frame'), 2),   # [6]
        # rocker
        p.get('T_r', p.get('a_r', 0.22)),   # [7]
        p.get('S_r1', p.get('b_r', 0.28)),  # [8]
        p.get('S_r2', p.get('alpha_r', 0.0)),  # [9]
        p.get('th_r1', 0.0),                # [10]
        p.get('th_r2', 0.0),                # [11]
        p.get('j_r', 0.5),                  # [12]
        p.get('delta_pb', 0.0),             # [13]
        # bogie
        p.get('T_b', p.get('c_b', 0.14)),   # [14]
        p.get('S_b1', p.get('d_b', 0.14)),  # [15]
        p.get('S_b2', p.get('beta_b', 0.0)),  # [16]
        p.get('th_b1', 0.0),                # [17]
        p.get('th_b2', 0.0),                # [18]
        p.get('j_b', 0.5),                  # [19]
        p.get('brk_v', 0.0),                # [20]
    ])


def pack_params_triangle(p):
    """dict → JAX 배열 (triangle-triangle 모드용)"""
    return jnp.array([
        p['R_w'],                           # [0]
        p.get('phi_r0', 0.0),               # [1]
        p.get('h_body', 0.3),               # [2]
        p.get('CG_offset', 0.0),            # [3]
        p.get('h_CG', p.get('h_body', 0.3) * 0.5),  # [4]
        MODE_MAP.get(p.get('rocker_mode', 'triangle'), 1),  # [5]
        MODE_MAP.get(p.get('bogie_mode', 'triangle'), 1),   # [6]
        # rocker
        p.get('L_r1', p.get('a_r', 0.22)),  # [7]
        p.get('L_r2', p.get('b_r', 0.28)),  # [8]
        p.get('alpha_r', 0.0),              # [9]
        0.0,                                # [10] (미사용)
        0.0,                                # [11]
        0.5,                                # [12]
        p.get('delta_pb', 0.0),             # [13]
        # bogie
        p.get('L_b1', p.get('c_b', 0.14)),  # [14]
        p.get('L_b2', p.get('d_b', 0.14)),  # [15]
        p.get('beta_b', 0.0),               # [16]
        0.0,                                # [17]
        0.0,                                # [18]
        0.5,                                # [19]
        p.get('brk_v', 0.0),                # [20]
    ])


def pack_params_auto(p):
    """dict → JAX 배열 (모드 자동 감지)"""
    rm = p.get('rocker_mode', 'linear').lower()
    bm = p.get('bogie_mode', 'linear').lower()

    arr = [
        p['R_w'],                           # [0]
        p.get('phi_r0', 0.0),               # [1]
        p.get('h_body', 0.3),               # [2]
        p.get('CG_offset', 0.0),            # [3]
        p.get('h_CG', p.get('h_body', 0.3) * 0.5),  # [4]
        float(MODE_MAP.get(rm, 0)),         # [5]
        float(MODE_MAP.get(bm, 0)),         # [6]
    ]

    # rocker 파라미터
    if rm == 'linear':
        arr.extend([p.get('a_r', 0.22), p.get('b_r', 0.28), 0.0])
    elif rm == 'triangle':
        arr.extend([p['L_r1'], p['L_r2'], p['alpha_r']])
    elif rm == 'frame':
        arr.extend([p['T_r'], p['S_r1'], p['S_r2']])

    arr.extend([
        p.get('th_r1', 0.0),               # [10]
        p.get('th_r2', 0.0),               # [11]
        p.get('j_r', 0.5),                 # [12]
        p.get('delta_pb', 0.0),            # [13]
    ])

    # bogie 파라미터
    if bm == 'linear':
        arr.extend([p.get('c_b', 0.14), p.get('d_b', 0.14), 0.0])
    elif bm == 'triangle':
        arr.extend([p['L_b1'], p['L_b2'], p['beta_b']])
    elif bm == 'frame':
        arr.extend([p['T_b'], p['S_b1'], p['S_b2']])

    arr.extend([
        p.get('th_b1', 0.0),               # [17]
        p.get('th_b2', 0.0),               # [18]
        p.get('j_b', 0.5),                 # [19]
        p.get('brk_v', 0.0),               # [20]
    ])

    return jnp.array(arr)
