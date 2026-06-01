"""
plot_sim: 시뮬레이션 결과 시각화
"""
import numpy as np
import matplotlib.pyplot as plt
from .wpos import wpos


def _draw_robot_snap(ax, xb, R, i, p):
    X = np.array([R['y0'][i], R['ar'][i], R['bb'][i]])
    Wf, Wm, Wr, Pb, _ = wpos(X, xb, p)
    P0 = np.array([xb, R['y0'][i]])

    ax.plot([Wr[0], P0[0], Pb[0]], [Wr[1], P0[1], Pb[1]], 'b-', linewidth=2.0)
    ax.plot([Wm[0], Pb[0], Wf[0]], [Wm[1], Pb[1], Wf[1]], 'g-', linewidth=2.0)

    bw = 0.07
    bh = p.get('h_body', 0.3)
    corners_x = [P0[0] - bw, P0[0] + bw, P0[0] + bw, P0[0] - bw]
    corners_y = [P0[1], P0[1], P0[1] + bh, P0[1] + bh]
    ax.fill(corners_x, corners_y, color=[0.4, 0.6, 0.85], alpha=0.55,
            edgecolor=[0.1, 0.3, 0.7])

    ax.plot(P0[0], P0[1], 'bs', markersize=5, markerfacecolor='b')
    ax.plot(Pb[0], Pb[1], 'gs', markersize=4, markerfacecolor='g')

    th = np.linspace(0, 2 * np.pi, 30)
    for w in [Wf, Wm, Wr]:
        ax.fill(w[0] + p['R_w'] * np.cos(th), w[1] + p['R_w'] * np.sin(th),
                color=[0.18, 0.18, 0.18], alpha=0.72)


def plot_sim(x_arr, x_t, y_t, R, p, label='', col='b'):
    fig, ax = plt.subplots()

    ax.fill_between(x_t, y_t, -0.06, color=[0.78, 0.68, 0.58], alpha=0.5)
    ax.plot(x_t, y_t, 'k-', linewidth=1.2)

    # 스냅샷 로봇 자세
    f_idx = np.round(np.linspace(15, len(x_arr) - 15, 8)).astype(int)
    for fi in f_idx:
        if 0 <= fi < len(x_arr):
            _draw_robot_snap(ax, x_arr[fi], R, fi, p)

    if isinstance(col, str):
        col_arr = col
    else:
        col_arr = col

    h1, = ax.plot(x_arr, R['y0'], '-', color=col_arr, linewidth=2.2, label='차체 높이 (P0)')

    y_wr = np.interp(R['xwr'], x_t, y_t)
    y_wm = np.interp(R['xwm'], x_t, y_t)
    y_wf = np.interp(R['xwf'], x_t, y_t)
    y_avg = (y_wr + y_wm + y_wf) / 3 + p['R_w']
    h_offset = np.nanmean(R['y0'] - y_avg)
    y_ideal = y_avg + h_offset

    col_dark = [c * 0.65 for c in col] if isinstance(col, (list, tuple)) else col
    h2, = ax.plot(x_arr, y_ideal, '--', color=col_dark, linewidth=1.0, label='이상 궤적 (평활화)')

    ax.legend(loc='upper left', fontsize=7)
    ax.grid(True)
    ax.set_xlabel('X [m]', fontsize=8)
    ax.set_ylabel('Y [m]', fontsize=8)
    ax.set_title(label, fontsize=9, fontweight='bold')
    ax.set_xlim([x_t[0], x_t[-1]])
    ax.set_ylim([-0.05, np.max(y_t) + p['R_w'] * 3 + p.get('h_body', 0.3) + 0.1])

    return fig, ax
