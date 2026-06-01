"""
plot_diagnostics.py — v4 최적해의 지형별 진단 플롯.

각 지형에서 로봇이 통과하는 동안:
  - 휠별 모터 토크 (한계선 표시)
  - TOI / liftoff
  - 휠별 슬립률 (한계 1.0 표시)
  - 휠별 정상력 N

총 4단 subplot × N개 지형. PNG 저장.

사용:
  python plot_diagnostics.py [--pkl path] [--terrains list] [--outdir dir]
  → outdir/diag_<terrain>.png 생성
"""
import argparse
import os
import sys
import pickle
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

# JAX GPU 설정 (반드시 import 전)
os.environ.setdefault('JAX_PLATFORM_NAME', 'gpu')
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from functions.gen_terrain import gen_terrain
from functions.calc_envelope_jax import calc_envelope_gpu
from functions.newton_solver import kin_sim_gpu
from functions.calc_dynamics_jax import calc_dynamics_gpu
from functions.calc_stability_jax import calc_stability_gpu
from functions.wpos_jax import pack_params_auto

MU_TERRAIN = {
    'flat': 0.70, 'step': 0.65, 'stairs': 0.60, 'real_stairs': 0.60,
    'wood_block': 0.70, 'rough': 0.55, 'curved_ramp': 0.65,
    'incline_15': 0.65, 'incline_30': 0.60,
}
N_PTS = 160


def get_b_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_r2'] * abs(np.cos(p['alpha_r'] / 2)), p['R_w'])
    elif mode == 'frame': return max(p['j_r'] * p['T_r'] + p['S_r2'] * abs(np.sin(p['th_r2'])), p['R_w'])
    return max(p.get('b_r', 0.28), p['R_w'])

def get_a_eff(p):
    mode = p.get('rocker_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_r1'] * abs(np.cos(p['alpha_r'] / 2)), p['R_w'])
    elif mode == 'frame': return max((1 - p['j_r']) * p['T_r'] + p['S_r1'] * abs(np.sin(p['th_r1'])), p['R_w'])
    return max(p.get('a_r', 0.22), p['R_w'])

def get_cb_fwd(p):
    mode = p.get('bogie_mode', 'linear').lower()
    if mode == 'triangle': return max(p['L_b1'] * abs(np.sin(p['beta_b'] / 2)), p['R_w'])
    elif mode == 'frame': return max(p['S_b1'] * abs(np.cos(p['th_b1'])), p['R_w'])
    return max(p.get('c_b', 0.14), p['R_w'])


def evaluate_terrain(p_base, terrain):
    """단일 지형 재평가 → (xa, x_t, y_t_raw, y_t_env, R, D, S)."""
    p = dict(p_base)
    p['mu'] = MU_TERRAIN.get(terrain, 0.65)
    p['liftoff_max'] = 0.02
    p_arr = pack_params_auto(p)

    x_t, y_t_raw = gen_terrain(terrain, p)
    y_t_env = calc_envelope_gpu(x_t, y_t_raw, p['R_w'], patch_width=p.get('patch_width', 0.030))

    b_eff_v = get_b_eff(p); a_eff_v = get_a_eff(p); cb = get_cb_fwd(p)
    xs = x_t[0] + b_eff_v + 0.05
    xe = x_t[-1] - (a_eff_v + cb) - 0.05
    if xs >= xe:
        # 옵티마이저는 이 경우 페널티 반환 — 진단 플롯은 명확히 에러내어 degenerate linspace 방지.
        raise ValueError(
            f'{terrain}: 유효 평가구간 없음 (xs={xs:.3f} ≥ xe={xe:.3f}). '
            '기하가 너무 길거나 지형이 짧음.')
    xa = np.linspace(xs, xe, N_PTS)

    R = kin_sim_gpu(xa, x_t, y_t_env, p_arr, p)
    D = calc_dynamics_gpu(R, xa, x_t, y_t_env, p)
    S = calc_stability_gpu(R, xa, x_t, y_t_raw, y_t_env, p)
    return xa, x_t, y_t_raw, y_t_env, R, D, S


def plot_terrain_diag(p, terrain, out_path):
    """4-panel diagnostic plot for one terrain."""
    xa, x_t, y_t_raw, y_t_env, R, D, S = evaluate_terrain(p, terrain)

    motor_peak = p.get('motor_tau_peak', 39.0)
    motor_cont = p.get('motor_tau_cont', 22.0)

    fig = plt.figure(figsize=(13, 11))
    gs = fig.add_gridspec(5, 1, height_ratios=[1.0, 1.3, 1.0, 1.0, 1.0], hspace=0.35)

    # ── 0. 지형 단면 + 휠 궤적 ──
    ax0 = fig.add_subplot(gs[0])
    ax0.fill_between(x_t, -0.05, y_t_raw, color='#bbb', alpha=0.5, label='terrain')
    ax0.plot(x_t, y_t_env, '--', color='#888', linewidth=0.8, alpha=0.7, label='envelope')
    ax0.plot(R['xwf'], np.interp(R['xwf'], x_t, y_t_env), 'g-', linewidth=1.2, label='Wf path')
    ax0.plot(R['xwm'], np.interp(R['xwm'], x_t, y_t_env), 'b-', linewidth=1.2, label='Wm path')
    ax0.plot(R['xwr'], np.interp(R['xwr'], x_t, y_t_env), 'r-', linewidth=1.2, label='Wr path')
    ax0.set_ylabel('z [m]')
    ax0.set_title(f'Terrain: {terrain}  (μ={MU_TERRAIN.get(terrain, 0.65):.2f})')
    ax0.legend(loc='upper left', fontsize=8)
    ax0.grid(True, alpha=0.3)

    # ── 1. 모터 토크 ──
    ax1 = fig.add_subplot(gs[1])
    ok = R['ok']
    ax1.plot(xa[ok], D['tau_motor_f'][ok], 'g-', linewidth=1.2, label='τ Front')
    ax1.plot(xa[ok], D['tau_motor_m'][ok], 'b-', linewidth=1.2, label='τ Mid')
    ax1.plot(xa[ok], D['tau_motor_r'][ok], 'r-', linewidth=1.2, label='τ Rear')
    # 한계선
    ax1.axhline(motor_peak, color='red', linestyle='--', linewidth=1.5, alpha=0.7,
                label=f'peak {motor_peak:.1f}Nm')
    ax1.axhline(motor_cont, color='orange', linestyle=':', linewidth=1.2, alpha=0.7,
                label=f'cont {motor_cont:.1f}Nm')
    # 속도 인식 가용 토크
    ax1.plot(xa, D['tau_avail_motor'], 'k--', linewidth=0.8, alpha=0.5, label='τ_avail(ω)')
    ax1.set_ylabel('Motor τ [Nm]')
    ax1.legend(loc='upper left', fontsize=7, ncol=2)
    ax1.grid(True, alpha=0.3)
    ymax = max(motor_peak * 1.1, np.nanmax(D['tau_motor_f'][ok]) * 1.15 if np.any(ok) else motor_peak)
    ax1.set_ylim(0, ymax)

    # ── 2. TOI + 들림 ──
    ax2 = fig.add_subplot(gs[2])
    ax2.plot(xa, S['TOI'], 'purple', linewidth=1.2, label='TOI (min margin)')
    ax2.plot(xa, S['TOI_front'], 'g:', linewidth=0.9, alpha=0.7, label='TOI front')
    ax2.plot(xa, S['TOI_rear'], 'r:', linewidth=0.9, alpha=0.7, label='TOI rear')
    ax2.axhline(0, color='red', linestyle='--', linewidth=1.2, alpha=0.5, label='TIP threshold')
    ax2.axhline(0.15, color='orange', linestyle=':', linewidth=1, alpha=0.5, label='warn 0.15')
    ax2.fill_between(xa, -1, 0, color='red', alpha=0.08)
    ax2.set_ylabel('TOI')
    ax2.set_ylim(-1.0, 1.0)
    ax2.legend(loc='upper right', fontsize=7, ncol=3)
    ax2.grid(True, alpha=0.3)

    # ── 3. 슬립률 ──
    ax3 = fig.add_subplot(gs[3])
    ax3.plot(xa, D['slip_f'], 'g-', linewidth=1.0, alpha=0.8, label='slip Front')
    ax3.plot(xa, D['slip_m'], 'b-', linewidth=1.0, alpha=0.8, label='slip Mid')
    ax3.plot(xa, D['slip_r'], 'r-', linewidth=1.0, alpha=0.8, label='slip Rear')
    ax3.axhline(1.0, color='red', linestyle='--', linewidth=1.2, alpha=0.7,
                label='slip = 1.0 (μ limit)')
    ax3.fill_between(xa, 1.0, 5, color='red', alpha=0.08)
    ax3.set_ylabel('Slip ratio')
    ax3.set_ylim(0, max(1.5, np.nanmax(D['slip_max_per_pt']) * 1.1))
    ax3.legend(loc='upper right', fontsize=7, ncol=2)
    ax3.grid(True, alpha=0.3)

    # ── 4. 휠별 normal force ──
    ax4 = fig.add_subplot(gs[4])
    ax4.plot(xa, D['Nf'], 'g-', linewidth=1.0, label='N Front')
    ax4.plot(xa, D['Nm'], 'b-', linewidth=1.0, label='N Mid')
    ax4.plot(xa, D['Nr'], 'r-', linewidth=1.0, label='N Rear')
    ax4.axhline(0, color='black', linewidth=0.5)
    # 들림 표시 (N==0 영역)
    for col, N in [('green', D['Nf']), ('blue', D['Nm']), ('red', D['Nr'])]:
        zero_x = xa[N < 1.0]
        if len(zero_x) > 0:
            for zx in zero_x:
                ax4.axvline(zx, color=col, alpha=0.05, linewidth=1)
    ax4.set_xlabel('x [m] (terrain traverse)')
    ax4.set_ylabel('Normal force [N]')
    ax4.legend(loc='upper right', fontsize=7, ncol=3)
    ax4.grid(True, alpha=0.3)

    # 요약 텍스트 (상단)
    summary = (
        f'τ peak: F={np.nanmax(D["tau_motor_f"]):.1f}  M={np.nanmax(D["tau_motor_m"]):.1f}  '
        f'R={np.nanmax(D["tau_motor_r"]):.1f}  Nm  |  '
        f'sat peak (speed-aware) = {D["sat_peak_speed_aware"]:.2f}  |  '
        f'slip peak = {D["slip_peak"]:.2f}  |  '
        f'TOI min = {S["min_TOI"]:+.3f}  |  '
        f'risk = {S["risk_level"]}'
    )
    fig.suptitle(summary, fontsize=9, y=0.995)

    fig.savefig(out_path, dpi=130, bbox_inches='tight')
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pkl', default=os.path.join(script_dir, 'zetin_optimal_params_v4.pkl'))
    ap.add_argument('--terrains',
                    default='real_stairs,wood_block,rough,step,curved_ramp,incline_15,incline_30',
                    help='쉼표 구분')
    ap.add_argument('--outdir', default=os.path.join(script_dir, 'diagnostics'))
    args = ap.parse_args()

    os.makedirs(args.outdir, exist_ok=True)

    with open(args.pkl, 'rb') as fh:
        result = pickle.load(fh)
    p_opt = result['p_opt']

    print(f'pkl: {args.pkl}')
    print(f'outdir: {args.outdir}')
    print()

    terrains = args.terrains.split(',')
    for t in terrains:
        out = os.path.join(args.outdir, f'diag_{t}.png')
        print(f'▶ {t} → {out}')
        try:
            plot_terrain_diag(p_opt, t, out)
            print(f'  완료')
        except Exception as e:
            print(f'  실패: {e}')


if __name__ == '__main__':
    main()
