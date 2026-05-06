"""
wpos: 순기구학 — 바퀴 및 피벗 위치 계산 [v2]

입력:
    X   : [y0, ar, bb] (3-element array)
    xb  : Rocker pivot x 좌표 [m]
    p   : 파라미터 dict

출력:
    Wf, Wm, Wr : 바퀴 위치 [2-element array]
    Pb          : Bogie pivot 위치 [2-element array]
    CG          : 차체 무게중심 위치 [2-element array]
"""
import numpy as np


def wpos(X, xb, p):
    y0 = X[0]
    ar = X[1]
    bb = X[2]

    phi_r0 = p.get('phi_r0', 0.0)
    delta_pb = p.get('delta_pb', 0.0)
    rocker_mode = p.get('rocker_mode', 'linear')
    bogie_mode = p.get('bogie_mode', 'linear')
    CG_offset = p.get('CG_offset', 0.0)
    h_CG = p.get('h_CG', p.get('h_body', 0.3) * 0.5)

    P0 = np.array([xb, y0])
    ar_eff = ar + phi_r0

    # ── Rocker 기구학 ──
    mode_r = rocker_mode.lower()
    if mode_r == 'triangle':
        ang_pb = ar_eff - p['alpha_r'] / 2
        ang_wr = ar_eff - np.pi + p['alpha_r'] / 2
        Pb = P0 + p['L_r1'] * np.array([np.cos(ang_pb), np.sin(ang_pb)])
        Wr = P0 + p['L_r2'] * np.array([np.cos(ang_wr), np.sin(ang_wr)])
    elif mode_r == 'frame':
        u_r = np.array([np.cos(ar_eff), np.sin(ar_eff)])
        n_r = np.array([-np.sin(ar_eff), np.cos(ar_eff)])
        Pb = (P0 + (1 - p['j_r']) * p['T_r'] * u_r
              + p['S_r1'] * (np.sin(p['th_r1']) * u_r - np.cos(p['th_r1']) * n_r))
        Wr = (P0 - p['j_r'] * p['T_r'] * u_r
              + p['S_r2'] * (-np.sin(p['th_r2']) * u_r - np.cos(p['th_r2']) * n_r))
    else:  # 'linear'
        u_r = np.array([np.cos(ar_eff), np.sin(ar_eff)])
        n_r = np.array([-np.sin(ar_eff), np.cos(ar_eff)])
        Pb = P0 + p['a_r'] * u_r + delta_pb * n_r
        Wr = P0 - p['b_r'] * u_r

    # ── Bogie 기구학 ──
    mode_b = bogie_mode.lower()
    if mode_b == 'triangle':
        ang_vert = -np.pi / 2 + bb
        ang_wf = ang_vert + p['beta_b'] / 2
        ang_wm = ang_vert - p['beta_b'] / 2
        Wf = Pb + p['L_b1'] * np.array([np.cos(ang_wf), np.sin(ang_wf)])
        Wm = Pb + p['L_b2'] * np.array([np.cos(ang_wm), np.sin(ang_wm)])
    elif mode_b == 'frame':
        u_b = np.array([np.cos(bb), np.sin(bb)])
        n_b = np.array([-np.sin(bb), np.cos(bb)])
        Wf = (Pb + (1 - p['j_b']) * p['T_b'] * u_b
              + p['S_b1'] * (np.sin(p['th_b1']) * u_b - np.cos(p['th_b1']) * n_b))
        Wm = (Pb - p['j_b'] * p['T_b'] * u_b
              + p['S_b2'] * (-np.sin(p['th_b2']) * u_b - np.cos(p['th_b2']) * n_b))
    else:  # 'linear'
        Wf = Pb + p['c_b'] * np.array([np.cos(bb), np.sin(bb)])
        Wm = Pb - p['d_b'] * np.array([np.cos(bb), np.sin(bb)])

    # ── CG 위치 ──
    u_h = np.array([np.cos(ar_eff), np.sin(ar_eff)])
    n_h = np.array([-np.sin(ar_eff), np.cos(ar_eff)])
    CG = P0 + CG_offset * u_h + h_CG * n_h

    return Wf, Wm, Wr, Pb, CG
