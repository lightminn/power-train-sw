"""
cross_validate.py — 옵티마이저(준정적) vs MuJoCo(시간 영역) 교차 검증.

목적: 우리 시뮬레이션 결과를 *얼마나 믿을 수 있는지* 정량화.
       각 지형에서 두 시뮬레이터의 핵심 메트릭을 비교 → fidelity 점수.

비교 메트릭:
  - 피크 모터 토크 (예측 vs 실측)
  - 평균 피치 (안정성 → MuJoCo 피치 진폭이 큰지)
  - 들림 비율 (예측 liftoff_ratio vs MuJoCo 5N 이하 시간)
  - 주행 성공 여부 (MuJoCo가 텀블/저속 정체로 실패한 케이스)

출력:
  cross_validation.md — markdown 표
  cross_validation.csv — raw 데이터

사용:
  python cross_validate.py [--pkl path] [--terrains list]
"""
import argparse
import os
import sys
import pickle
import numpy as np

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

# v4_gpu 헬퍼 재사용
from analyze_v4_result import (
    adaptive_xa, get_b_eff, get_a_eff, get_cb_fwd,
    MU_TERRAIN, TAU_MOTOR_PEAK
)
# N_PTS는 옵티마이저 기본값(100)과 일치시킨다 — analyze 모듈의 값(과거 160)에 의존하면
# 예측 패스가 원 최적화와 다른 샘플수로 돌아 체계적 오프셋이 생긴다.
N_PTS = int(os.environ.get('N_PTS', 100))
from validate_mujoco import build_mjcf, simulate_run


def predict_optimizer(p_base, terrain):
    """옵티마이저(준정적)의 예측 메트릭."""
    p = dict(p_base)
    p['mu'] = MU_TERRAIN.get(terrain, 0.65)
    p['liftoff_max'] = 0.02
    p_arr = pack_params_auto(p)

    x_t, y_t_raw = gen_terrain(terrain, p)
    y_t_env = calc_envelope_gpu(x_t, y_t_raw, p['R_w'], patch_width=p.get('patch_width', 0.030))

    b_eff_v = get_b_eff(p); a_eff_v = get_a_eff(p); cb = get_cb_fwd(p)
    xs = x_t[0] + b_eff_v + 0.05
    xe = x_t[-1] - (a_eff_v + cb) - 0.05
    xa = adaptive_xa(x_t, y_t_env, xs, xe, n_pts=N_PTS)

    R = kin_sim_gpu(xa, x_t, y_t_env, p_arr, p)
    D = calc_dynamics_gpu(R, xa, x_t, y_t_env, p)
    S = calc_stability_gpu(R, xa, x_t, y_t_raw, y_t_env, p)

    return {
        'tau_peak_predicted': float(D['stair_torque_max']),
        'tau_rms_predicted': float(D.get('tau_rms_worst', 0.0)),
        'liftoff_pct_predicted': float(S['liftoff_ratio']) * 100.0,
        'toi_min_predicted': float(S['min_TOI']),
        'risk_predicted': S['risk_level'],
        'fail_rate_predicted': float(np.sum(~R['ok']) / len(xa)) * 100.0,
        'energy_Wh_predicted': float(D.get('energy_Wh', 0.0)),
    }


def measure_mujoco(p_base, terrain, duration=4.0, v_target=0.5):
    """MuJoCo 실측."""
    try:
        xml, hfield = build_mjcf(p_base, terrain=terrain)
        log = simulate_run(xml, hfield_data=hfield, duration=duration,
                           v_target=v_target, verbose=False)

        # settle 이후만
        t = log['t']
        mask = t >= 0.5
        if not np.any(mask):
            return {'status': 'no_settle_data'}

        tau = log['tau'][mask]
        pitch = log['pitch'][mask]
        cN = log['contact_N'][mask]
        x = log['x_chassis']

        # 텀블 감지 (피치 절댓값 > 90°)
        tumbled = bool(np.any(np.abs(np.degrees(pitch)) > 90))

        tau_peak_each = np.max(np.abs(tau), axis=0)
        liftoff_pct = float(np.mean(cN < 5.0, axis=0).max() * 100.0)
        distance = float(x[-1] - x[0])

        return {
            'tau_peak_measured': float(np.max(tau_peak_each)),
            'pitch_amplitude_deg': float(np.degrees(pitch.max() - pitch.min())),
            'liftoff_pct_measured': liftoff_pct,
            'distance_m': distance,
            'tumbled': tumbled,
            'status': 'tumbled' if tumbled else 'completed',
        }
    except Exception as e:
        # 조용한 흡수 금지 — MuJoCo 발산/NaN/모델오류가 표에 'error:'로만 남으면 디버깅 불가.
        import traceback
        traceback.print_exc()
        return {'status': f'error: {e}'}


def cross_validate(p_opt, terrains, duration=4.0, v_target=0.5):
    """모든 지형 교차 검증 → rows."""
    rows = []
    for t in terrains:
        print(f'▶ {t}')
        print('   예측 (옵티마이저)...', end=' ', flush=True)
        pred = predict_optimizer(p_opt, t)
        print(f'τ={pred["tau_peak_predicted"]:.1f}Nm  TOI={pred["toi_min_predicted"]:+.2f}  '
              f'liftoff={pred["liftoff_pct_predicted"]:.1f}%')
        print('   실측 (MuJoCo)...', end=' ', flush=True)
        meas = measure_mujoco(p_opt, t, duration=duration, v_target=v_target)
        if meas.get('status') == 'completed':
            print(f'τ={meas["tau_peak_measured"]:.1f}Nm  pitch±{meas["pitch_amplitude_deg"]/2:.1f}°  '
                  f'liftoff={meas["liftoff_pct_measured"]:.1f}%  주행={meas["distance_m"]*1000:.0f}mm')
        elif meas.get('status') == 'tumbled':
            print(f'❌ 텀블 (피치 > 90°)  τ_max={meas.get("tau_peak_measured", 0):.1f}Nm')
        else:
            print(f'⚠ {meas["status"]}')

        rows.append({'terrain': t, **pred, **meas})
    return rows


def write_report(rows, p_opt, out_dir):
    """markdown + csv 작성."""
    os.makedirs(out_dir, exist_ok=True)
    md_path = os.path.join(out_dir, 'cross_validation.md')
    csv_path = os.path.join(out_dir, 'cross_validation.csv')

    # 합치된 fidelity 점수 계산
    completed = [r for r in rows if r.get('status') == 'completed']
    tumbled_terrains = [r['terrain'] for r in rows if r.get('status') == 'tumbled']
    err_terrains = [r['terrain'] for r in rows if r.get('status', '').startswith('error')]

    if completed:
        tau_errors = [abs(r['tau_peak_predicted'] - r['tau_peak_measured']) /
                      max(r['tau_peak_predicted'], 0.5) for r in completed]
        liftoff_diff = [abs(r['liftoff_pct_predicted'] - r['liftoff_pct_measured'])
                        for r in completed]
        avg_tau_err = np.mean(tau_errors) * 100.0
        avg_liftoff_diff = np.mean(liftoff_diff)
    else:
        avg_tau_err = -1
        avg_liftoff_diff = -1

    # markdown
    lines = [
        '# 옵티마이저 vs MuJoCo 교차 검증 리포트',
        '',
        f'**Design**: rocker={p_opt["rocker_mode"]} bogie={p_opt["bogie_mode"]} '
        f'brk_v={p_opt.get("brk_v",0)*1000:.0f}mm',
        '',
        '## Fidelity 점수',
        '',
    ]
    if completed:
        lines += [
            f'- 완료 지형: {len(completed)}/{len(rows)}',
            f'- 텀블 지형: {len(tumbled_terrains)} ({", ".join(tumbled_terrains) if tumbled_terrains else "-"})',
            f'- 오류 지형: {len(err_terrains)}',
            f'- 평균 토크 오차: **{avg_tau_err:.1f}%**',
            f'- 평균 들림비율 차이: **{avg_liftoff_diff:.1f}%p**',
            '',
        ]
        if avg_tau_err < 30:
            lines += ['**판정**: ✓ 옵티마이저 예측이 MuJoCo와 합리적으로 일치']
        elif avg_tau_err < 60:
            lines += ['**판정**: △ 일부 격차. 디자인 디테일 검증 필요']
        else:
            lines += ['**판정**: ✗ 큰 격차. 준정적 가정이 부족할 가능성']
    else:
        lines += [
            '⚠ 완료된 지형이 없습니다 (모두 텀블/오류).',
            '디자인이 동역학적으로 매우 불안정함을 의미합니다.',
            '',
            f'텀블: {", ".join(tumbled_terrains) if tumbled_terrains else "-"}',
            f'오류: {", ".join(err_terrains) if err_terrains else "-"}',
        ]

    lines += ['', '## 지형별 상세', '',
              '| 지형 | 예측 τpk | 실측 τpk | 차이% | 예측 들림 | 실측 들림 | MuJoCo 피치± | 주행 mm | 상태 |',
              '|------|---------:|---------:|------:|----------:|----------:|-------------:|--------:|------|']
    for r in rows:
        t = r['terrain']
        tau_p = r.get('tau_peak_predicted', 0)
        if r.get('status') == 'completed':
            tau_m = r['tau_peak_measured']
            tau_diff = (tau_m - tau_p) / max(tau_p, 0.5) * 100
            lift_p = r['liftoff_pct_predicted']
            lift_m = r['liftoff_pct_measured']
            pitch = r['pitch_amplitude_deg'] / 2
            dist = r['distance_m'] * 1000
            stat = '✓'
        else:
            tau_m = '-'
            tau_diff = '-'
            lift_p = r.get('liftoff_pct_predicted', 0)
            lift_m = '-'
            pitch = '-'
            dist = '-'
            stat = '❌ ' + r.get('status', '?')[:20]
        lines.append(f'| {t} | {tau_p:.1f} | {tau_m if isinstance(tau_m, str) else f"{tau_m:.1f}"} | '
                     f'{tau_diff if isinstance(tau_diff, str) else f"{tau_diff:+.0f}%"} | '
                     f'{lift_p:.1f}% | {lift_m if isinstance(lift_m, str) else f"{lift_m:.1f}%"} | '
                     f'{pitch if isinstance(pitch, str) else f"{pitch:.1f}°"} | '
                     f'{dist if isinstance(dist, str) else f"{dist:.0f}"} | {stat} |')

    lines += [
        '',
        '## 해석 가이드',
        '',
        '- **τ 차이 < 30%**: 옵티마이저의 토크 예측이 신뢰할 수준. 모터 선정에 사용 가능.',
        '- **들림 차이 < 5%p**: 안정성 예측이 합리적.',
        '- **텀블 발생**: 옵티마이저는 통과했으나 실제 동역학에선 *전복* — 디자인 비교용으로만 사용하고 *반드시 추가 검증 필요*.',
        '- **fidelity 낮음**: 준정적 가정의 한계. 시간 영역 ODE 또는 multibody 시뮬로 최종 검증 필요.',
        '',
    ]

    with open(md_path, 'w') as fh:
        fh.write('\n'.join(lines))

    # csv
    import csv
    if rows:
        with open(csv_path, 'w', newline='') as fh:
            writer = csv.DictWriter(fh, fieldnames=list(rows[0].keys()))
            writer.writeheader()
            for r in rows:
                writer.writerow({k: r.get(k, '') for k in writer.fieldnames})

    return md_path, csv_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pkl', default=os.path.join(script_dir, 'zetin_optimal_params_v4.pkl'))
    ap.add_argument('--terrains',
                    default='flat,step,real_stairs,wood_block,rough,curved_ramp,incline_15',
                    help='쉼표 구분')
    ap.add_argument('--duration', type=float, default=4.0, help='MuJoCo 시뮬 시간')
    ap.add_argument('--v_target', type=float, default=0.5, help='MuJoCo 목표 속도')
    ap.add_argument('--outdir', default=os.path.join(script_dir, 'cross_val_out'))
    args = ap.parse_args()

    with open(args.pkl, 'rb') as fh:
        result = pickle.load(fh)
    p_opt = result['p_opt']

    print(f'pkl: {args.pkl}  f_opt={result["f_opt"]:.4f}')
    print(f'rocker={p_opt["rocker_mode"]}  bogie={p_opt["bogie_mode"]}  '
          f'brk_v={p_opt.get("brk_v",0)*1000:.0f}mm')
    print(f'지형: {args.terrains}')
    print()

    terrains = args.terrains.split(',')
    rows = cross_validate(p_opt, terrains, duration=args.duration, v_target=args.v_target)

    md_path, csv_path = write_report(rows, p_opt, args.outdir)
    print(f'\n✓ 리포트 저장:')
    print(f'  {md_path}')
    print(f'  {csv_path}')


if __name__ == '__main__':
    main()
