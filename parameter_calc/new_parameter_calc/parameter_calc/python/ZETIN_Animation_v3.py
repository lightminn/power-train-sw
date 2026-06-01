"""
ZETIN_Animation_v3.py
최적 Rocker-Bogie (Frame 구조) — 4종 지형 주행 애니메이션

[업데이트 내역]
1. Minkowski 지형 팽창(Envelope)을 적용하여 바퀴가 파묻히는 현상 해결
2. 렌더링 속도 대폭 향상
"""
import os
import sys
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
from matplotlib.animation import FuncAnimation
import pickle

# --- 한글 폰트 및 마이너스 깨짐 방지 세팅 ---
plt.rc('font', family='NanumGothic')
plt.rc('axes', unicode_minus=False)

# 경로 설정
script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from functions.gen_terrain import gen_terrain
from functions.calc_envelope import calc_envelope
from functions.kin_sim import kin_sim
from functions.calc_dynamics import calc_dynamics
from functions.wpos import wpos


# ═══════════════════════════════════════
# 헬퍼 함수
# ═══════════════════════════════════════

def get_cb_fwd_local(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle':
        cb = p['L_b1'] * abs(np.sin(p['beta_b'] / 2))
    elif mode == 'frame':
        cb = p['S_b1'] * abs(np.cos(p['th_b1']))
    else:
        cb = p['c_b']
    return max(cb, p['R_w'])


def get_b_eff_local(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle':
        b = p['L_r2'] * abs(np.cos(p['alpha_r'] / 2))
    elif mode == 'frame':
        b = p['j_r'] * p['T_r'] + p['S_r2'] * abs(np.sin(p['th_r2']))
    else:
        b = p['b_r']
    return max(b, p['R_w'])


def get_a_eff_local(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle':
        a = p['L_r1'] * abs(np.cos(p['alpha_r'] / 2))
    elif mode == 'frame':
        a = (1 - p['j_r']) * p['T_r'] + p['S_r1'] * abs(np.sin(p['th_r1']))
    else:
        a = p['a_r']
    return max(a, p['R_w'])


def draw_wheel(ax, cx, cy, r, col):
    th = np.linspace(0, 2 * np.pi, 32)
    ax.fill(cx + r * np.cos(th), cy + r * np.sin(th), color=col, alpha=0.85)
    for ang in np.arange(0, np.pi, np.pi / 3):
        ax.plot([cx + r * 0.2 * np.cos(ang), cx + r * 0.85 * np.cos(ang)],
                [cy + r * 0.2 * np.sin(ang), cy + r * 0.85 * np.sin(ang)],
                '-', color=[0.55, 0.55, 0.55], linewidth=0.7)
        ax.plot([cx + r * 0.2 * np.cos(ang + np.pi), cx + r * 0.85 * np.cos(ang + np.pi)],
                [cy + r * 0.2 * np.sin(ang + np.pi), cy + r * 0.85 * np.sin(ang + np.pi)],
                '-', color=[0.55, 0.55, 0.55], linewidth=0.7)


def draw_robot_v4(ax, xb, y0, ar, bb, p, col_r, col_b):
    X = np.array([y0, ar, bb])
    Wf, Wm, Wr, Pb, _ = wpos(X, xb, p)
    P0 = np.array([xb, y0])
    phi_r0 = p.get('phi_r0', 0)

    mode_r = p.get('rocker_mode', 'linear').lower()
    if mode_r in ('linear', 'triangle'):
        ax.plot([Wr[0], P0[0], Pb[0]], [Wr[1], P0[1], Pb[1]], '-', color=col_r, linewidth=4.0)
    elif mode_r == 'frame':
        ar_e = ar + phi_r0
        ur = np.array([np.cos(ar_e), np.sin(ar_e)])
        Ptr = P0 - p['j_r'] * p['T_r'] * ur
        Ptf = P0 + (1 - p['j_r']) * p['T_r'] * ur
        ax.plot([Ptr[0], Ptf[0]], [Ptr[1], Ptf[1]], '-', color=col_r, linewidth=4.0)
        ax.plot([Ptf[0], Pb[0]], [Ptf[1], Pb[1]], '-', color=col_r, linewidth=3.0)
        ax.plot([Ptr[0], Wr[0]], [Ptr[1], Wr[1]], '-', color=col_r, linewidth=3.0)

    mode_b = p.get('bogie_mode', 'linear').lower()
    if mode_b in ('linear', 'triangle'):
        ax.plot([Wm[0], Pb[0], Wf[0]], [Wm[1], Pb[1], Wf[1]], '-', color=col_b, linewidth=4.0)
    elif mode_b == 'frame':
        ubb = np.array([np.cos(bb), np.sin(bb)])
        Pbm = Pb - p['j_b'] * p['T_b'] * ubb
        Pbf = Pb + (1 - p['j_b']) * p['T_b'] * ubb
        ax.plot([Pbm[0], Pbf[0]], [Pbm[1], Pbf[1]], '-', color=col_b, linewidth=4.0)
        ax.plot([Pbf[0], Wf[0]], [Pbf[1], Wf[1]], '-', color=col_b, linewidth=3.0)
        ax.plot([Pbm[0], Wm[0]], [Pbm[1], Wm[1]], '-', color=col_b, linewidth=3.0)

    # 차체
    bw = 0.08
    bh = p.get('h_body', 0.3)
    ar_e = ar + phi_r0
    ur = np.array([np.cos(ar_e), np.sin(ar_e)])
    nr = np.array([-np.sin(ar_e), np.cos(ar_e)])
    corners_x = [P0[0] - bw * ur[0], P0[0] + bw * ur[0],
                 P0[0] + bw * ur[0] + bh * nr[0], P0[0] - bw * ur[0] + bh * nr[0]]
    corners_y = [P0[1] - bw * ur[1], P0[1] + bw * ur[1],
                 P0[1] + bw * ur[1] + bh * nr[1], P0[1] - bw * ur[1] + bh * nr[1]]
    ax.fill(corners_x, corners_y, color=[0.4, 0.6, 0.85], alpha=0.55,
            edgecolor=[0.1, 0.3, 0.7], linewidth=2.0)

    ax.plot(P0[0], P0[1], 'o', markersize=8, markerfacecolor=col_r,
            markeredgecolor='w', linewidth=1.5)
    ax.plot(Pb[0], Pb[1], 'o', markersize=6, markerfacecolor=col_b,
            markeredgecolor='w', linewidth=1.5)

    C_WHEEL = [0.18, 0.18, 0.18]
    draw_wheel(ax, Wf[0], Wf[1], p['R_w'], C_WHEEL)
    draw_wheel(ax, Wm[0], Wm[1], p['R_w'], C_WHEEL)
    draw_wheel(ax, Wr[0], Wr[1], p['R_w'], C_WHEEL)


def build_info_str(p):
    s1 = f'[Rocker: {p.get("rocker_mode","linear").upper()}]\n'
    if p.get('rocker_mode', 'linear') == 'frame':
        s1 += f'T_r = {p["T_r"]*1000:.0f} mm\nS_r1 = {p["S_r1"]*1000:.0f} mm\nS_r2 = {p["S_r2"]*1000:.0f} mm\n'
    else:
        s1 += f'L_r1 = {p.get("L_r1",0)*1000:.0f} mm\nL_r2 = {p.get("L_r2",0)*1000:.0f} mm\n'

    s2 = f'\n[Bogie: {p.get("bogie_mode","linear").upper()}]\n'
    if p.get('bogie_mode', 'linear') == 'frame':
        s2 += f'T_b = {p["T_b"]*1000:.0f} mm\nS_b1 = {p["S_b1"]*1000:.0f} mm\nS_b2 = {p["S_b2"]*1000:.0f} mm'
    else:
        s2 += f'L_b1 = {p.get("L_b1",0)*1000:.0f} mm\nL_b2 = {p.get("L_b2",0)*1000:.0f} mm'

    return s1 + s2


# ═══════════════════════════════════════
# 메인 실행
# ═══════════════════════════════════════
def main():
    # 최적 파라미터 로드
    pkl_path = os.path.join(script_dir, 'zetin_optimal_params_v3.pkl')
    if not os.path.exists(pkl_path):
        print(f'zetin_optimal_params_v3.pkl 파일이 없습니다. 최적화를 먼저 실행하세요.')
        print(f'(경로: {pkl_path})')
        return

    with open(pkl_path, 'rb') as f:
        data = pickle.load(f)
    p_opt = data['p_opt']

    print('\n======================================================')
    print(f' 파라미터 로드 완료 (R_w={p_opt["R_w"]*1000:.0f}mm, mass={p_opt["mass"]:.1f}kg)')
    print(f'  Rocker 모드: {p_opt.get("rocker_mode","linear")}')
    print(f'  Bogie 모드 : {p_opt.get("bogie_mode","linear")}')
    print('======================================================')

    # 애니메이션 설정
    SAVE_VIDEO = True
    FPS = 60
    N_FRAMES = 180
    PAUSE_BTWN = 1.0

    terrain_list = ['real_stairs', 'wood_block', 'rough', 'step']
    terrain_names = ['실제 계단 (Rise 80mm x 3단)', '목재 블록 (40~80mm 불규칙)',
                     '불규칙 지형 (사인파 합성)', '단차 150mm']
    terrain_cols = [[0.20, 0.55, 0.35], [0.80, 0.50, 0.15],
                    [0.25, 0.40, 0.80], [0.70, 0.25, 0.20]]

    C_GROUND = [0.78, 0.68, 0.58]
    C_ROCKER = [0.20, 0.38, 0.72]
    C_BOGIE = [0.18, 0.62, 0.38]
    C_NR = [0.10, 0.45, 0.75]
    C_NM = [0.10, 0.60, 0.25]
    C_NF = [0.85, 0.40, 0.10]

    fig = plt.figure('ZETIN 최적 구조 주행 애니메이션', figsize=(14, 8))
    fig.patch.set_facecolor([0.10, 0.10, 0.12])

    for ti, terrain in enumerate(terrain_list):
        t_name = terrain_names[ti]
        t_col = terrain_cols[ti]
        print(f'\n[{ti+1}/4] {terrain} 시뮬레이션 중...', end='')

        # 기구학 시뮬레이션
        x_t, y_t_raw = gen_terrain(terrain, p_opt)
        y_t_env = calc_envelope(x_t, y_t_raw, p_opt['R_w'])

        cb = get_cb_fwd_local(p_opt)
        a_eff = get_a_eff_local(p_opt)
        b_eff = get_b_eff_local(p_opt)

        xs = x_t[0] + b_eff + 0.05
        xe = x_t[-1] - (a_eff + cb) - 0.05
        xa = np.linspace(xs, xe, N_FRAMES)

        R = kin_sim(xa, x_t, y_t_env, p_opt)
        D = calc_dynamics(R, xa, x_t, y_t_env, p_opt)

        print(f' 완료 (기구학 실패율={R["fail_rate"]*100:.1f}%)')

        # 비디오 저장 (matplotlib animation)
        y_min = -0.08
        y_max = np.max(y_t_raw) + p_opt['R_w'] * 4 + p_opt.get('h_body', 0.3) + 0.20

        if SAVE_VIDEO:
            try:
                from matplotlib.animation import FFMpegWriter
                vid_name = os.path.join(script_dir, f'ZETIN_animation_{terrain}.mp4')
                writer = FFMpegWriter(fps=FPS)

                fig_v = plt.figure(figsize=(14, 8))
                fig_v.patch.set_facecolor([0.10, 0.10, 0.12])

                with writer.saving(fig_v, vid_name, dpi=100):
                    for fi in range(N_FRAMES):
                        fig_v.clf()

                        ax1 = fig_v.add_axes([0.03, 0.38, 0.94, 0.57])
                        ax1.set_facecolor([0.08, 0.08, 0.10])

                        x_vis_start = max(x_t[0], xa[fi] - 1.5)
                        x_vis_end = min(x_t[-1], xa[fi] + 2.0)
                        mask_vis = (x_t >= x_vis_start) & (x_t <= x_vis_end)
                        x_v = x_t[mask_vis]
                        y_v = y_t_raw[mask_vis]

                        if len(x_v) > 0:
                            ax1.fill_between(x_v, y_v, y_min, color=C_GROUND, alpha=0.75)
                            ax1.plot(x_v, y_v, '-', color=[0.5, 0.4, 0.3], linewidth=1.5)

                        if fi > 0:
                            trail_len = min(fi, 80)
                            idx_trail = slice(max(0, fi - trail_len), fi)
                            valid = R['ok'][idx_trail]
                            x_trail = xa[idx_trail][valid]
                            y_trail = R['y0'][idx_trail][valid]
                            ax1.plot(x_trail, y_trail, '-', color=t_col + [0.6], linewidth=1.5)

                        if R['ok'][fi]:
                            draw_robot_v4(ax1, xa[fi], R['y0'][fi], R['ar'][fi], R['bb'][fi],
                                          p_opt, C_ROCKER, C_BOGIE)

                        ax1.set_xlim([x_vis_start, x_vis_end])
                        ax1.set_ylim([y_min, y_max])
                        ax1.set_ylabel('Y [m]', fontsize=9, color=[0.8, 0.8, 0.8])

                        prog = fi / (N_FRAMES - 1) * 100
                        ax1.set_title(f'{t_name}   [{ti+1}/4]  진행률: {prog:.0f}%',
                                      fontsize=11, fontweight='bold', color='w')

                        # 하단 패널: 법선력
                        ax2 = fig_v.add_axes([0.03, 0.06, 0.55, 0.27])
                        ax2.set_facecolor([0.08, 0.08, 0.10])

                        if fi > 0:
                            idx_h = slice(0, fi + 1)
                            valid = R['ok'][idx_h]
                            ax2.plot(xa[idx_h][valid], D['Nr'][idx_h][valid] * 1000, '--',
                                     color=C_NR + [0.7], linewidth=1.2)
                            ax2.plot(xa[idx_h][valid], D['Nm'][idx_h][valid] * 1000, '--',
                                     color=C_NM + [0.7], linewidth=1.2)
                            ax2.plot(xa[idx_h][valid], D['Nf'][idx_h][valid] * 1000, '--',
                                     color=C_NF + [0.7], linewidth=1.2)

                        ax2.axhline(p_opt['mass'] * p_opt['g'] / 3 * 1000, linestyle='--',
                                    color=[0.5, 0.5, 0.5], linewidth=1.0)
                        ax2.set_xlim([xs, xe])
                        ax2.set_ylabel('법선력 [mN]', fontsize=9, color=[0.7, 0.7, 0.7])
                        ax2.set_title('각 바퀴의 법선력 분배 곡선', fontsize=10, color=[0.8, 0.8, 0.8])

                        # 하단 패널 2: 실시간 바 그래프
                        ax3 = fig_v.add_axes([0.63, 0.06, 0.34, 0.27])
                        ax3.set_facecolor([0.08, 0.08, 0.10])

                        N_eq = p_opt['mass'] * p_opt['g'] / 3 * 1000
                        if R['ok'][fi]:
                            N_vals = np.array([D['Nr'][fi], D['Nm'][fi], D['Nf'][fi]]) * 1000
                            bar_cols = [C_NR, C_NM, C_NF]
                            for bi2 in range(3):
                                ax3.bar(bi2 + 1, N_vals[bi2], 0.6, color=bar_cols[bi2])

                            if np.mean(N_vals) > 0.1:
                                imbal_cur = (np.max(N_vals) - np.min(N_vals)) / np.mean(N_vals) * 100
                            else:
                                imbal_cur = 0
                            ax3.set_title(f'현재 편중도: {imbal_cur:.1f}%', fontsize=10,
                                          color=[0.8, 0.8, 0.8], fontweight='bold')
                        else:
                            ax3.set_title('기구학 수렴 실패', fontsize=10, color='r', fontweight='bold')

                        ax3.axhline(N_eq, linestyle='--', color=[0.6, 0.6, 0.6], linewidth=1.5)
                        ax3.set_ylim([0, N_eq * 2.5])
                        ax3.set_xticks([1, 2, 3])
                        ax3.set_xticklabels(['Nr(뒤)', 'Nm(중)', 'Nf(앞)'])
                        ax3.set_ylabel('법선력 [mN]', fontsize=9, color=[0.7, 0.7, 0.7])

                        writer.grab_frame()

                plt.close(fig_v)
                print(f'  저장 완료: ZETIN_animation_{terrain}.mp4')
            except Exception as e:
                print(f'  비디오 저장 실패: {e}')
                print('  (ffmpeg가 설치되어 있는지 확인하세요)')

    print('\n=== 모든 애니메이션 완료 ===')
    plt.close('all')


if __name__ == '__main__':
    main()
