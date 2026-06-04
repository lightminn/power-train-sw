"""
plot_geometry.py — v4 최적 설계의 측면뷰 PNG 생성.

pkl을 읽어 본체+로커+보기+휠+브래킷의 측면뷰를 그립니다. HTML viewer의 정적 그림 대용.

용도:
  - 설계 sanity check (눈으로 형상 확인)
  - 리포트/문서 첨부용 PNG
  - 지형 위 배치 (옵션) — 평지에서 정상 자세

사용:
  python plot_geometry.py [--pkl path] [--terrain flat|step|stairs|...] [--out path]
"""
import argparse
import os
import sys
import pickle
import numpy as np
import matplotlib
matplotlib.use('Agg')  # 헤드리스
import matplotlib.pyplot as plt
import matplotlib.patches as mp

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)


# ─── 1. 자세 계산 (평지 정적 자세) ─────────────────────────

def _rocker_kin(p, ar=0.0):
    """Rocker pivot(P0)으로부터 Pb, Wr_axle 위치 (월드 프레임, P0=원점)."""
    rm = p['rocker_mode']
    brk_v = p.get('brk_v', 0.0)
    phi_r0 = p.get('phi_r0', 0.0)
    ar_eff = ar + phi_r0
    u = np.array([np.cos(ar_eff), np.sin(ar_eff)])
    n = np.array([-np.sin(ar_eff), np.cos(ar_eff)])

    if rm == 'triangle':
        ang_pb = ar_eff - p['alpha_r'] / 2.0
        ang_wr = ar_eff - np.pi + p['alpha_r'] / 2.0
        Pb = np.array([p['L_r1'] * np.cos(ang_pb), p['L_r1'] * np.sin(ang_pb)])
        Wr_link = np.array([p['L_r2'] * np.cos(ang_wr), p['L_r2'] * np.sin(ang_wr)])
    elif rm == 'frame':
        Pb = (1 - p['j_r']) * p['T_r'] * u + p['S_r1'] * (np.sin(p['th_r1']) * u - np.cos(p['th_r1']) * n)
        Wr_link = -p['j_r'] * p['T_r'] * u + p['S_r2'] * (-np.sin(p['th_r2']) * u - np.cos(p['th_r2']) * n)
    else:
        raise ValueError(f'mode 미지원: {rm}')
    Wr_axle = Wr_link - np.array([0.0, brk_v])
    return Pb, Wr_link, Wr_axle


def _bogie_kin(p, bb=0.0):
    """Pb 원점에서 Wf, Wm 위치."""
    bm = p['bogie_mode']
    brk_v = p.get('brk_v', 0.0)
    if bm == 'triangle':
        ang_vert = -np.pi / 2.0 + bb
        ang_wf = ang_vert + p['beta_b'] / 2.0
        ang_wm = ang_vert - p['beta_b'] / 2.0
        Wf_link = np.array([p['L_b1'] * np.cos(ang_wf), p['L_b1'] * np.sin(ang_wf)])
        Wm_link = np.array([p['L_b2'] * np.cos(ang_wm), p['L_b2'] * np.sin(ang_wm)])
    elif bm == 'frame':
        u = np.array([np.cos(bb), np.sin(bb)])
        n = np.array([-np.sin(bb), np.cos(bb)])
        Wf_link = (1 - p['j_b']) * p['T_b'] * u + p['S_b1'] * (np.sin(p['th_b1']) * u - np.cos(p['th_b1']) * n)
        Wm_link = -p['j_b'] * p['T_b'] * u + p['S_b2'] * (-np.sin(p['th_b2']) * u - np.cos(p['th_b2']) * n)
    else:
        raise ValueError(f'mode 미지원: {bm}')
    Wf_axle = Wf_link - np.array([0.0, brk_v])
    Wm_axle = Wm_link - np.array([0.0, brk_v])
    return Wf_link, Wm_link, Wf_axle, Wm_axle


def _flat_pose(p):
    """평지 정적 자세: ar/bb를 0으로 두고 P0 높이를 휠 접지에 맞춤.

    Returns: dict with world positions of P0, Pb, Wf, Wm, Wr (axles).
    """
    # ar=bb=0 기준으로 모든 점의 P0-상대 좌표 구함
    Pb_rel, _, Wr_axle_rel = _rocker_kin(p, ar=0.0)
    _, _, Wf_axle_b, Wm_axle_b = _bogie_kin(p, bb=0.0)
    Wf_axle_rel = Pb_rel + Wf_axle_b
    Wm_axle_rel = Pb_rel + Wm_axle_b

    # 세 휠 축 중 가장 낮은 y가 R_w (휠 중심이 지면+R_w)
    R_w = p['R_w']
    min_y = min(Wr_axle_rel[1], Wm_axle_rel[1], Wf_axle_rel[1])
    P0_y = R_w - min_y  # 가장 낮은 휠을 지면+R_w에 맞춤

    P0 = np.array([0.0, P0_y])
    Pb = P0 + Pb_rel
    Wr_axle = P0 + Wr_axle_rel
    Wr_link = Wr_axle + np.array([0.0, p.get('brk_v', 0.0)])  # 링크 끝점 (브래킷 상단)
    Wf_axle = P0 + Wf_axle_rel
    Wm_axle = P0 + Wm_axle_rel
    Wf_link = Wf_axle + np.array([0.0, p.get('brk_v', 0.0)])
    Wm_link = Wm_axle + np.array([0.0, p.get('brk_v', 0.0)])

    return {
        'P0': P0, 'Pb': Pb,
        'Wr_axle': Wr_axle, 'Wr_link': Wr_link,
        'Wf_axle': Wf_axle, 'Wf_link': Wf_link,
        'Wm_axle': Wm_axle, 'Wm_link': Wm_link,
        'rocker_mode': p['rocker_mode'],
        'bogie_mode': p['bogie_mode'],
    }


# ─── 2. 그리기 ──────────────────────────────────────────

def plot_robot(p, ax=None, show_dims=True, alpha=1.0):
    """ax에 측면뷰 로봇 그리기. ax=None이면 새로 생성."""
    if ax is None:
        fig, ax = plt.subplots(figsize=(12, 7))

    pose = _flat_pose(p)
    R_w = p['R_w']
    body_h = p.get('h_body', 0.3)
    body_w = 0.500
    pivot_x = 0.55
    pivot_y = 0.65

    P0 = pose['P0']
    body_left = P0[0] - pivot_x * body_w
    body_right = P0[0] + (1 - pivot_x) * body_w
    body_bottom = P0[1] - pivot_y * body_h
    body_top = P0[1] + (1 - pivot_y) * body_h

    # 지면
    x_range = (P0[0] - 0.7, P0[0] + 0.7)
    ax.axhline(0, color='#888', linewidth=2, alpha=alpha)
    ax.fill_between(x_range, [-0.05, -0.05], [0, 0], color='#bbb', alpha=alpha*0.5)

    # 본체 (직사각형)
    body = mp.Rectangle((body_left, body_bottom), body_w, body_h,
                        linewidth=1.5, edgecolor='#1f77b4', facecolor='#1f77b4',
                        alpha=alpha*0.3, zorder=2)
    ax.add_patch(body)

    # CG 표시
    cg_offset = p.get('CG_offset', 0.0)
    h_CG = p.get('h_CG', body_h * 0.55)
    cg_x = P0[0] + cg_offset
    cg_y = P0[1] + h_CG
    ax.plot(cg_x, cg_y, 'o', color='red', markersize=10, alpha=alpha, zorder=4)
    ax.plot(cg_x, cg_y, 'x', color='white', markersize=6, alpha=alpha, zorder=5)
    ax.annotate('CG', (cg_x, cg_y), textcoords='offset points', xytext=(8, 8),
                fontsize=9, color='red', alpha=alpha)

    # P0 (rocker pivot)
    ax.plot(P0[0], P0[1], 's', color='#1f77b4', markersize=8, alpha=alpha, zorder=5)

    # Rocker (P0 → Pb, P0 → Wr_link)
    Pb = pose['Pb']
    Wr_link = pose['Wr_link']
    ax.plot([P0[0], Pb[0]], [P0[1], Pb[1]], '-', color='#d62728', linewidth=3, alpha=alpha, zorder=3)
    ax.plot([P0[0], Wr_link[0]], [P0[1], Wr_link[1]], '-', color='#d62728', linewidth=3, alpha=alpha, zorder=3)

    # Bogie (Pb → Wf_link, Pb → Wm_link)
    Wf_link = pose['Wf_link']
    Wm_link = pose['Wm_link']
    ax.plot([Pb[0], Wf_link[0]], [Pb[1], Wf_link[1]], '-', color='#2ca02c', linewidth=2.5, alpha=alpha, zorder=3)
    ax.plot([Pb[0], Wm_link[0]], [Pb[1], Wm_link[1]], '-', color='#2ca02c', linewidth=2.5, alpha=alpha, zorder=3)

    # Pb 마커
    ax.plot(Pb[0], Pb[1], 's', color='#d62728', markersize=6, alpha=alpha, zorder=4)

    # 브래킷 (링크 끝점 → 휠 축, 수직선)
    brk_v = p.get('brk_v', 0.0)
    if brk_v > 0.001:
        for link, axle in [(Wr_link, pose['Wr_axle']),
                           (Wf_link, pose['Wf_axle']),
                           (Wm_link, pose['Wm_axle'])]:
            ax.plot([link[0], axle[0]], [link[1], axle[1]],
                    '-', color='#9467bd', linewidth=2, alpha=alpha, zorder=3)

    # 휠 (원)
    for label, axle in [('R', pose['Wr_axle']),
                        ('M', pose['Wm_axle']),
                        ('F', pose['Wf_axle'])]:
        wheel = mp.Circle((axle[0], axle[1]), R_w, linewidth=2,
                          edgecolor='#333', facecolor='#444', alpha=alpha*0.6, zorder=2)
        ax.add_patch(wheel)
        ax.annotate(label, (axle[0], axle[1]), color='white', ha='center', va='center',
                    fontsize=11, fontweight='bold', alpha=alpha, zorder=6)

    # 치수 표시 (옵션)
    if show_dims:
        # 전체 휠베이스
        wheelbase = pose['Wf_axle'][0] - pose['Wr_axle'][0]
        y_dim = -0.18
        ax.annotate('', xy=(pose['Wf_axle'][0], y_dim), xytext=(pose['Wr_axle'][0], y_dim),
                    arrowprops=dict(arrowstyle='<->', color='#666', lw=1))
        ax.text((pose['Wf_axle'][0] + pose['Wr_axle'][0]) / 2, y_dim - 0.04,
                f'wheelbase {wheelbase*1000:.0f}mm', ha='center', fontsize=9, color='#666')

        # 본체 바닥 높이
        ground_clearance = body_bottom
        ax.annotate('', xy=(body_right + 0.06, 0), xytext=(body_right + 0.06, ground_clearance),
                    arrowprops=dict(arrowstyle='<->', color='#666', lw=1))
        ax.text(body_right + 0.10, ground_clearance / 2,
                f'clearance\n{ground_clearance*1000:.0f}mm', fontsize=9, color='#666')

        # 전체 높이
        total_h = body_top
        ax.annotate('', xy=(body_left - 0.06, 0), xytext=(body_left - 0.06, total_h),
                    arrowprops=dict(arrowstyle='<->', color='#666', lw=1))
        ax.text(body_left - 0.15, total_h / 2,
                f'height\n{total_h*1000:.0f}mm', fontsize=9, color='#666', ha='right')

    ax.set_aspect('equal')
    ax.set_xlim(x_range)
    ax.set_ylim(-0.25, body_top + 0.10)
    ax.grid(True, alpha=0.3)
    ax.set_xlabel('x [m]   ← rear        front →')
    ax.set_ylabel('z [m]')

    return ax


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pkl', default=os.path.join(script_dir, 'zetin_optimal_params_v4.pkl'))
    ap.add_argument('--out', default=os.path.join(script_dir, 'design_geometry.png'))
    ap.add_argument('--title', default=None)
    args = ap.parse_args()

    with open(args.pkl, 'rb') as fh:
        result = pickle.load(fh)
    p_opt = result['p_opt']

    fig, ax = plt.subplots(figsize=(12, 7))
    plot_robot(p_opt, ax=ax, show_dims=True)

    title_lines = [
        f'ZETIN 로커-보기 6륜 — 최적 형상 ({result.get("version", "v4")})',
        f'rocker={p_opt["rocker_mode"]}  bogie={p_opt["bogie_mode"]}  '
        f'brk_v={p_opt.get("brk_v",0)*1000:.0f}mm  CG_offset={p_opt.get("CG_offset",0)*1000:.0f}mm',
        f'f_opt={result["f_opt"]:.4f}',
    ]
    ax.set_title('\n'.join(title_lines), fontsize=11)
    fig.tight_layout()
    fig.savefig(args.out, dpi=150, bbox_inches='tight')
    print(f'저장: {args.out}')


if __name__ == '__main__':
    main()
