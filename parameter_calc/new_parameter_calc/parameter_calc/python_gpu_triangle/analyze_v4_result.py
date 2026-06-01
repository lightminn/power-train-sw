"""
analyze_v4_result.py — v4 최적화 결과 상세 분석 (설계 리뷰용)

기능:
  1. pkl 로드 + 파라미터 요약
  2. 5종 지형 재평가 (kin_sim → dynamics → stability)
  3. 다중 속도 스윕 (0.3 / 0.5 / 0.8 / 1.0 m/s) — 속도별 강건성
  4. 지형별 × 속도별 표 출력: τ_peak, slip 위반율, TOI_min, liftoff
  5. 모터 포화 / 슬립 위반 케이스 자동 강조

사용:
  python analyze_v4_result.py [--pkl path] [--speeds 0.3,0.5,0.8,1.0]
"""
import argparse
import os
import sys
import pickle
import numpy as np

# JAX GPU 설정 (반드시 jax import 전에)
os.environ.setdefault('JAX_PLATFORM_NAME', 'gpu')
os.environ.setdefault('XLA_PYTHON_CLIENT_PREALLOCATE', 'false')

script_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, script_dir)

from functions.gen_terrain import gen_terrain
from functions.calc_envelope_jax import calc_envelope_gpu
from functions.newton_solver import kin_sim_gpu
from functions.calc_dynamics_jax import calc_dynamics_gpu
from functions.calc_stability_jax import calc_stability_gpu
from functions.calc_metrics_jax import calc_metrics_gpu
from functions.wpos_jax import pack_params_auto


# v4_gpu.py의 MU_TERRAIN, TAU_MOTOR_SAT, N_PTS와 동기화 유지.
MU_TERRAIN = {
    'flat': 0.70, 'step': 0.65, 'stairs': 0.60, 'real_stairs': 0.60,
    'wood_block': 0.70, 'rough': 0.55, 'curved_ramp': 0.65,
    'incline_15': 0.65, 'incline_30': 0.60,
}
N_PTS = int(os.environ.get('N_PTS', 100))  # 옵티마이저 기본값(100)과 일치
EDGE_BOOST = 3.0
TAU_MOTOR_PEAK = 39.0  # BL70200 인휠 모터 휠 측 피크 (Nm)


def adaptive_xa(x_t, y_t_env, xs, xe, n_pts=N_PTS, edge_boost=EDGE_BOOST):
    """v4_gpu.py와 동일한 적응적 샘플링 (단차 근처 밀도 ↑)."""
    mask = (x_t >= xs) & (x_t <= xe)
    if np.sum(mask) < 10:
        return np.linspace(xs, xe, n_pts)
    xt_sub = x_t[mask]
    yt_sub = y_t_env[mask]
    grad = np.abs(np.gradient(yt_sub, xt_sub))
    if grad.max() < 1e-3:
        return np.linspace(xs, xe, n_pts)
    grad_norm = grad / (grad.max() + 1e-9)
    density = 1.0 + edge_boost * grad_norm
    cdf = np.cumsum(density)
    cdf = cdf / cdf[-1]
    u = np.linspace(0.0, 1.0, n_pts)
    return np.interp(u, cdf, xt_sub).astype(np.float64)


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


def evaluate_at_speed(p_base, v_max, terrains, verbose=False):
    """단일 속도에서 모든 지형 평가. 결과 dict 반환."""
    p = dict(p_base)
    p['v_max'] = v_max
    p['v_robot'] = v_max
    p['liftoff_max'] = 0.02

    p_arr = pack_params_auto(p)

    rows = []
    for t in terrains:
        try:
            x_t, y_t_raw = gen_terrain(t, p)
            y_t_env = calc_envelope_gpu(x_t, y_t_raw, p['R_w'], patch_width=p.get('patch_width', 0.030))

            b_eff_v = get_b_eff(p); a_eff_v = get_a_eff(p); cb = get_cb_fwd(p)
            xs = x_t[0] + b_eff_v + 0.05
            xe = x_t[-1] - (a_eff_v + cb) - 0.05
            if xs >= xe:
                rows.append({'terrain': t, 'status': 'SKIP (xa empty)'})
                continue
            xa = adaptive_xa(x_t, y_t_env, xs, xe, n_pts=N_PTS)

            R = kin_sim_gpu(xa, x_t, y_t_env, p_arr, p)
            fail_rate = 100.0 * np.sum(~R['ok']) / len(xa)

            p_t = dict(p)
            p_t['mu'] = MU_TERRAIN.get(t, 0.65)

            D = calc_dynamics_gpu(R, xa, x_t, y_t_env, p_t)
            S = calc_stability_gpu(R, xa, x_t, y_t_raw, y_t_env, p_t)

            tau_peak = D['stair_torque_max']
            tau_95 = D['stair_torque_peak']
            sat_pct = 100.0 * tau_peak / TAU_MOTOR_PEAK
            slip_viol = 100.0 * D['slip_violation_rate']
            slip_peak = D['slip_peak']
            toi_min = S['min_TOI']
            liftoff_ratio = 100.0 * S['liftoff_ratio']
            risk = S['risk_level']
            # Phase 3+ 신규 (구 pkl에 없으면 0)
            tau_rms = D.get('tau_rms_worst', 0.0)
            cont_pct = 100.0 * tau_rms / p_base.get('motor_tau_cont', 22.0)
            stuck_rate = 100.0 * D.get('system_stuck_rate', 0.0)
            batt_peak = D.get('battery_current_peak', 0.0)
            batt_pct = 100.0 * batt_peak / p_base.get('battery_max_current', 30.0)
            energy_Wh = D.get('energy_Wh', 0.0)
            avg_power_W = D.get('avg_power_W', 0.0)

            rows.append({
                'terrain': t,
                'fail_rate': fail_rate,
                'tau_peak': tau_peak, 'tau_95': tau_95,
                'sat_pct': sat_pct,
                'slip_viol': slip_viol, 'slip_peak': slip_peak,
                'toi_min': toi_min, 'liftoff': liftoff_ratio,
                'risk': risk,
                'tau_rms': tau_rms, 'cont_pct': cont_pct,
                'stuck_rate': stuck_rate,
                'batt_peak': batt_peak, 'batt_pct': batt_pct,
                'energy_Wh': energy_Wh, 'avg_power_W': avg_power_W,
            })
        except Exception as e:
            rows.append({'terrain': t, 'status': f'ERR: {e}'})
    return rows


def fmt_row(r):
    if 'status' in r:
        return f'  {r["terrain"]:14s}  {r["status"]}'
    flags = []
    if r['sat_pct'] > 100: flags.append('SAT!')
    elif r['sat_pct'] > 80: flags.append('sat')
    if r.get('cont_pct', 0) > 100: flags.append('CONT!')
    elif r.get('cont_pct', 0) > 80: flags.append('cont')
    if r['slip_peak'] > 1.0: flags.append('SLIP!')
    elif r['slip_peak'] > 0.8: flags.append('slip')
    if r.get('stuck_rate', 0) > 5: flags.append('STUCK!')
    if r.get('batt_pct', 0) > 100: flags.append('BATT!')
    elif r.get('batt_pct', 0) > 80: flags.append('batt')
    if r['toi_min'] < 0: flags.append('TIP!')
    elif r['toi_min'] < 0.15: flags.append('tip')
    if r['fail_rate'] > 5: flags.append('FAIL')
    flag_str = ' '.join(flags) if flags else '-'

    return (f'  {r["terrain"]:14s}  '
            f'τ_pk={r["tau_peak"]:4.1f}/RMS={r.get("tau_rms",0):4.1f}({r["sat_pct"]:3.0f}/{r.get("cont_pct",0):3.0f}%)  '
            f'slip={r["slip_peak"]:.2f}({r["slip_viol"]:4.1f}%)stuck={r.get("stuck_rate",0):3.0f}%  '
            f'TOI={r["toi_min"]:+.2f}  '
            f'lift={r["liftoff"]:3.0f}%  '
            f'I={r.get("batt_peak",0):4.1f}A({r.get("batt_pct",0):3.0f}%)  '
            f'E={r.get("energy_Wh",0):.2f}Wh  '
            f'{r["risk"]:6s} {flag_str}')


def _perturb_design(p_base, sigma_rel, rng):
    """설계 파라미터에 상대 가우시안 perturbation (mean=원본, std=원본·sigma_rel)."""
    p = dict(p_base)
    # 길이/길이비 파라미터들
    perturb_keys = []
    rm = p['rocker_mode']
    if rm == 'triangle':
        perturb_keys += ['L_r1', 'L_r2', 'alpha_r']
    else:
        perturb_keys += ['T_r', 'S_r1', 'S_r2', 'th_r1', 'th_r2', 'j_r']
    bm = p['bogie_mode']
    if bm == 'triangle':
        perturb_keys += ['L_b1', 'L_b2', 'beta_b']
    else:
        perturb_keys += ['T_b', 'S_b1', 'S_b2', 'th_b1', 'th_b2', 'j_b']
    perturb_keys += ['brk_v', 'CG_offset', 'h_CG']

    for k in perturb_keys:
        if k in p and isinstance(p[k], (int, float)):
            sigma = abs(p[k]) * sigma_rel + 1e-6
            p[k] = p[k] + rng.normal(0, sigma)
    return p


def _evaluate_summary(rows):
    """rows → (max_sat_pct, max_slip_peak, min_toi, max_fail_rate)."""
    valid = [r for r in rows if 'status' not in r]
    if not valid:
        return None
    return {
        'max_sat_pct': max(r['sat_pct'] for r in valid),
        'max_slip_peak': max(r['slip_peak'] for r in valid),
        'min_toi': min(r['toi_min'] for r in valid),
        'max_fail_rate': max(r['fail_rate'] for r in valid),
        'any_danger': any(r['risk'] == 'danger' for r in valid),
    }


def run_monte_carlo(p_opt, terrains, sigma_rel=0.05, n_samples=20, v_max=0.8, seed=42):
    """Monte Carlo 강건성 평가.

    설계 파라미터에 σ_rel·|p_i| 가우시안 perturbation → 각 perturbed 디자인을
    모든 지형에서 재평가. worst-case 메트릭 분포 출력.

    Returns:
        list of summary dicts, length n_samples.
    """
    rng = np.random.RandomState(seed)
    results = []
    for i in range(n_samples):
        p_pert = _perturb_design(p_opt, sigma_rel, rng)
        rows = evaluate_at_speed(p_pert, v_max, terrains)
        summary = _evaluate_summary(rows)
        if summary is not None:
            results.append(summary)
            # 주: 삼항식 안에서 summary[...]를 먼저 평가하면 None일 때 TypeError → if/else로 분리.
            print(f'  [{i+1}/{n_samples}] sat≤{summary["max_sat_pct"]:5.1f}%  '
                  f'slip≤{summary["max_slip_peak"]:.2f}  TOI≥{summary["min_toi"]:+.2f}  '
                  f'fail≤{summary["max_fail_rate"]:.1f}%  '
                  f'{"DANGER" if summary["any_danger"] else "ok"}')
        else:
            print(f'  [{i+1}/{n_samples}] (failed)')
    return results


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pkl', default=os.path.join(script_dir, 'zetin_optimal_params_v4.pkl'))
    ap.add_argument('--speeds', default='0.3,0.5,0.8,1.0',
                    help='쉼표 구분 속도 리스트 (m/s)')
    ap.add_argument('--terrains',
                    default='flat,real_stairs,wood_block,rough,step,curved_ramp,incline_15,incline_30',
                    help='평가할 지형 목록')
    ap.add_argument('--mc_samples', type=int, default=0,
                    help='Monte Carlo perturbation 샘플 수 (0=비활성)')
    ap.add_argument('--mc_sigma', type=float, default=0.05,
                    help='Monte Carlo 상대 표준편차 (기본 0.05=5%)')
    args = ap.parse_args()

    speeds = [float(s) for s in args.speeds.split(',')]
    terrains = args.terrains.split(',')

    print(f'pkl: {args.pkl}')
    with open(args.pkl, 'rb') as fh:
        result = pickle.load(fh)
    p_opt = result['p_opt']
    f_opt = result['f_opt']

    # ─── 기본 요약 ───
    print('\n' + '═' * 72)
    print(f'  v4 최적화 결과 분석   (f_opt={f_opt:.4f})')
    print('═' * 72)
    print(f'Rocker mode  : {p_opt["rocker_mode"]}')
    if p_opt['rocker_mode'] == 'triangle':
        print(f'  L_r1={p_opt["L_r1"]*1000:.1f}mm  L_r2={p_opt["L_r2"]*1000:.1f}mm  '
              f'α={np.degrees(p_opt["alpha_r"]):.1f}°')
    else:
        print(f'  T_r={p_opt["T_r"]*1000:.1f}mm  S_r1={p_opt["S_r1"]*1000:.1f}mm  '
              f'S_r2={p_opt["S_r2"]*1000:.1f}mm  j_r={p_opt["j_r"]:.2f}')
        print(f'  θ_r1={np.degrees(p_opt["th_r1"]):.1f}°  θ_r2={np.degrees(p_opt["th_r2"]):.1f}°')

    print(f'Bogie mode   : {p_opt["bogie_mode"]}')
    if p_opt['bogie_mode'] == 'triangle':
        Wb = (p_opt['L_b1'] + p_opt['L_b2']) * np.sin(p_opt['beta_b'] / 2)
        print(f'  L_b1={p_opt["L_b1"]*1000:.1f}mm  L_b2={p_opt["L_b2"]*1000:.1f}mm  '
              f'β={np.degrees(p_opt["beta_b"]):.1f}°  W_bot={Wb*1000:.1f}mm')
    else:
        print(f'  T_b={p_opt["T_b"]*1000:.1f}mm  S_b1={p_opt["S_b1"]*1000:.1f}mm  '
              f'S_b2={p_opt["S_b2"]*1000:.1f}mm  j_b={p_opt["j_b"]:.2f}')

    print(f'Bracket      : brk_v={p_opt.get("brk_v", 0)*1000:.1f}mm')
    print(f'CG           : offset={p_opt.get("CG_offset", 0)*1000:.1f}mm 전방, h_CG={p_opt["h_CG"]*1000:.1f}mm')
    print(f'프로파일     : v_max={p_opt.get("v_max", 0.8):.2f}m/s  a_lim={p_opt.get("a_lim", 1.5):.2f}m/s²')
    print(f'모터         : 피크 {TAU_MOTOR_PEAK}Nm × gear {p_opt["gear_ratio"]} × η{p_opt["eta_gear"]:.2f} '
          f'= 휠 {TAU_MOTOR_PEAK*p_opt["gear_ratio"]*p_opt["eta_gear"]:.1f}Nm')

    # ─── 속도별 평가 ───
    print('\n' + '═' * 72)
    print('  속도 스윕 (각 속도 × 5종 지형)')
    print('═' * 72)
    print('  플래그: SAT=토크포화 SLIP=슬립한계초과 TIP=전복위험 FAIL=수렴실패')
    print()

    all_results = {}
    for v in speeds:
        print(f'▶ v_max = {v:.2f} m/s')
        rows = evaluate_at_speed(p_opt, v, terrains)
        for r in rows:
            print(fmt_row(r))
        all_results[v] = rows
        print()

    # ─── 종합 요약: 최악 케이스 ───
    print('═' * 72)
    print('  종합 요약 — 속도×지형 worst case')
    print('═' * 72)
    worst_sat = 0; worst_sat_ctx = ''
    worst_slip = 0; worst_slip_ctx = ''
    worst_toi = 999; worst_toi_ctx = ''
    worst_cont = 0; worst_cont_ctx = ''
    worst_stuck = 0; worst_stuck_ctx = ''
    worst_batt = 0; worst_batt_ctx = ''
    total_energy = 0.0
    for v, rows in all_results.items():
        for r in rows:
            if 'status' in r: continue
            if r['sat_pct'] > worst_sat:
                worst_sat = r['sat_pct']; worst_sat_ctx = f'{r["terrain"]} @ v={v:.2f}'
            if r['slip_peak'] > worst_slip:
                worst_slip = r['slip_peak']; worst_slip_ctx = f'{r["terrain"]} @ v={v:.2f}'
            if r['toi_min'] < worst_toi:
                worst_toi = r['toi_min']; worst_toi_ctx = f'{r["terrain"]} @ v={v:.2f}'
            if r.get('cont_pct', 0) > worst_cont:
                worst_cont = r['cont_pct']; worst_cont_ctx = f'{r["terrain"]} @ v={v:.2f}'
            if r.get('stuck_rate', 0) > worst_stuck:
                worst_stuck = r['stuck_rate']; worst_stuck_ctx = f'{r["terrain"]} @ v={v:.2f}'
            if r.get('batt_pct', 0) > worst_batt:
                worst_batt = r['batt_pct']; worst_batt_ctx = f'{r["terrain"]} @ v={v:.2f}'
            total_energy += r.get('energy_Wh', 0.0)

    print(f'  worst 모터 피크   : {worst_sat:.0f}%  ({worst_sat_ctx})')
    print(f'  worst 모터 연속   : {worst_cont:.0f}%  ({worst_cont_ctx})')
    print(f'  worst 슬립       : {worst_slip:.2f}  ({worst_slip_ctx})')
    print(f'  worst 시스템 견인 부족: {worst_stuck:.0f}%  ({worst_stuck_ctx})')
    print(f'  worst 배터리 전류 : {worst_batt:.0f}%  ({worst_batt_ctx})')
    print(f'  worst TOI        : {worst_toi:+.3f}  ({worst_toi_ctx})')
    print(f'  총 에너지 (모든 시나리오): {total_energy:.2f} Wh')
    print()

    # 설계 검증 신호등
    print('  설계 신호등:')
    print(f'    모터 피크: {"✗ 부적합" if worst_sat > 100 else "△ 마진작음" if worst_sat > 80 else "✓ 충분"}')
    print(f'    모터 연속: {"✗ 열적 위험" if worst_cont > 100 else "△ 한계근접" if worst_cont > 80 else "✓ 충분"}')
    print(f'    접지 마찰: {"✗ 슬립함" if worst_slip > 1.0 else "△ 한계근접" if worst_slip > 0.8 else "✓ 충분"}')
    print(f'    시스템 견인: {"✗ 정체발생" if worst_stuck > 5 else "△ 한계근접" if worst_stuck > 1 else "✓ 충분"}')
    print(f'    배터리   : {"✗ 초과" if worst_batt > 100 else "△ 마진작음" if worst_batt > 80 else "✓ 충분"}')
    print(f'    전복 안정: {"✗ 위험" if worst_toi < 0 else "△ 주의" if worst_toi < 0.15 else "✓ 안전"}')
    print('═' * 72)

    # ─── Monte Carlo 강건성 (옵션) ───
    if args.mc_samples > 0:
        print('\n' + '═' * 72)
        print(f'  Monte Carlo 강건성 — {args.mc_samples}회, σ={args.mc_sigma*100:.0f}%')
        print('═' * 72)
        mc_results = run_monte_carlo(p_opt, terrains, sigma_rel=args.mc_sigma,
                                     n_samples=args.mc_samples, v_max=0.8)
        if mc_results:
            sats = [r['max_sat_pct'] for r in mc_results]
            slips = [r['max_slip_peak'] for r in mc_results]
            tois = [r['min_toi'] for r in mc_results]
            n_danger = sum(1 for r in mc_results if r['any_danger'])
            print(f'\n  통계 (n={len(mc_results)}):')
            print(f'    포화율 worst: 평균 {np.mean(sats):.1f}%  P95 {np.percentile(sats, 95):.1f}%  최대 {max(sats):.1f}%')
            print(f'    슬립 worst  : 평균 {np.mean(slips):.2f}  P95 {np.percentile(slips, 95):.2f}  최대 {max(slips):.2f}')
            print(f'    TOI worst   : 평균 {np.mean(tois):+.3f}  P5 {np.percentile(tois, 5):+.3f}  최악 {min(tois):+.3f}')
            print(f'    DANGER 비율  : {n_danger}/{len(mc_results)}  ({n_danger/len(mc_results)*100:.0f}%)')
            print('═' * 72)
            print('  강건성 판정:')
            danger_rate = n_danger / len(mc_results)
            print(f'    {"✗ 매우 민감 (재설계 필요)" if danger_rate > 0.30 else "△ 일부 민감" if danger_rate > 0.10 else "✓ 강건"}')
            print('═' * 72)


if __name__ == '__main__':
    main()
