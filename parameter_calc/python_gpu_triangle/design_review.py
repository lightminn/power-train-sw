"""
design_review.py — v4 최적해 종합 설계 리뷰 보고서 생성.

호출 도구:
  1. plot_geometry.py    → design_geometry.png
  2. plot_diagnostics.py → diag_<terrain>.png × N
  3. analyze_v4_result.py → 텍스트 표 (stdout 캡처)
  4. (옵션) validate_mujoco.py → flat + step 실측

출력:
  review_YYYYMMDD_HHMM/
    ├── design_geometry.png
    ├── diagnostics/diag_*.png
    ├── analysis_text.txt       # analyze_v4_result.py 출력
    ├── mujoco_flat.txt          # validate_mujoco --terrain flat
    ├── mujoco_step.txt          # validate_mujoco --terrain step
    └── README.md                # 통합 리포트 (모든 결과 링크)

사용:
  python design_review.py [--pkl path] [--mc 30] [--mujoco]
    --mc N: Monte Carlo N회 (기본 0=비활성)
    --mujoco: MuJoCo 검증 포함 (기본 False)
"""
import argparse
import os
import sys
import pickle
import subprocess
import datetime
import numpy as np

script_dir = os.path.dirname(os.path.abspath(__file__))


def run_tool(cmd, capture_path=None, env=None):
    """도구 실행. capture_path가 있으면 stdout 파일로 저장."""
    print(f'  $ {" ".join(cmd[:3])} ...')
    if env is None:
        env = os.environ.copy()
    result = subprocess.run(cmd, capture_output=True, text=True, env=env, cwd=script_dir)
    if capture_path:
        with open(capture_path, 'w') as fh:
            fh.write(result.stdout)
            if result.stderr:
                fh.write('\n--- STDERR ---\n')
                fh.write(result.stderr)
    # 반환코드를 무시하면 하위 도구가 죽어도 "✓ 완료"가 떠 디버깅 불가 → 실패를 표면화·집계.
    if result.returncode != 0:
        tool = os.path.basename(cmd[1]) if len(cmd) > 1 else cmd[0]
        run_tool.failures.append(tool)
        print(f'    ⚠ 비정상 종료 (exit {result.returncode}): {tool}')
        for ln in result.stderr.strip().splitlines()[-3:]:
            print(f'      {ln}')
    return result.returncode, result.stdout, result.stderr


run_tool.failures = []  # 실행 중 실패한 하위 도구 집계


def build_markdown_report(out_dir, p_opt, f_opt, version, run_ts,
                          terrains_diag, run_mujoco, mc_samples):
    """README.md 생성."""
    md_path = os.path.join(out_dir, 'README.md')
    rocker_mode = p_opt['rocker_mode']
    bogie_mode = p_opt['bogie_mode']

    # 형상 표
    geom_rows = []
    if rocker_mode == 'triangle':
        geom_rows += [
            f'| Rocker (triangle) | L_r1 | {p_opt["L_r1"]*1000:.1f} mm |',
            f'| | L_r2 | {p_opt["L_r2"]*1000:.1f} mm |',
            f'| | α_r  | {np.degrees(p_opt["alpha_r"]):.1f}° |',
        ]
    else:
        geom_rows += [
            f'| Rocker (frame) | T_r  | {p_opt["T_r"]*1000:.1f} mm |',
            f'| | S_r1 | {p_opt["S_r1"]*1000:.1f} mm |',
            f'| | S_r2 | {p_opt["S_r2"]*1000:.1f} mm |',
            f'| | θ_r1 | {np.degrees(p_opt["th_r1"]):.1f}° |',
            f'| | θ_r2 | {np.degrees(p_opt["th_r2"]):.1f}° |',
            f'| | j_r  | {p_opt["j_r"]:.2f} |',
        ]
    if bogie_mode == 'triangle':
        Wb = (p_opt['L_b1'] + p_opt['L_b2']) * np.sin(p_opt['beta_b'] / 2)
        geom_rows += [
            f'| Bogie (triangle) | L_b1 | {p_opt["L_b1"]*1000:.1f} mm |',
            f'| | L_b2 | {p_opt["L_b2"]*1000:.1f} mm |',
            f'| | β_b  | {np.degrees(p_opt["beta_b"]):.1f}° |',
            f'| | W_bot | {Wb*1000:.1f} mm |',
        ]
    else:
        geom_rows += [
            f'| Bogie (frame) | T_b | {p_opt["T_b"]*1000:.1f} mm |',
            f'| | S_b1 | {p_opt["S_b1"]*1000:.1f} mm |',
            f'| | S_b2 | {p_opt["S_b2"]*1000:.1f} mm |',
            f'| | θ_b1 | {np.degrees(p_opt["th_b1"]):.1f}° |',
            f'| | θ_b2 | {np.degrees(p_opt["th_b2"]):.1f}° |',
            f'| | j_b  | {p_opt["j_b"]:.2f} |',
        ]
    geom_rows += [
        f'| Bracket | brk_v | {p_opt.get("brk_v", 0)*1000:.1f} mm |',
    ]

    md = f'''# ZETIN 6륜 로커-보기 — 설계 리뷰 ({run_ts})

## 1. 요약

- **버전**: {version}
- **목적함수 값**: {f_opt:.4f}
- **rocker / bogie 모드**: {rocker_mode} / {bogie_mode}

## 2. 시스템 파라미터

| 항목 | 값 |
|------|-----|
| 로봇 질량 | {p_opt["mass"]} kg |
| 휠 반경 | {p_opt["R_w"]*1000:.0f} mm |
| 휠 단위 질량 (모터 포함) | {p_opt["m_wheel"]} kg |
| 모터 정격 토크 | {p_opt.get("motor_tau_cont", 22):.1f} Nm (휠 측) |
| 모터 피크 토크 | {p_opt.get("motor_tau_peak", 39):.1f} Nm (휠 측) |
| 무부하 RPM | {p_opt.get("omega_no_load_rpm", 240):.0f} RPM (휠 측 @ 48V) |
| 설계 속도 (v_max) | {p_opt.get("v_max", 0.8):.2f} m/s |
| 가속 한계 (a_lim) | {p_opt.get("a_lim", 1.5):.2f} m/s² |
| CG_offset | {p_opt.get("CG_offset", 0)*1000:.1f} mm 전방 |
| h_CG | {p_opt.get("h_CG", 0)*1000:.1f} mm |

## 3. 최적 형상

| 부위 | 파라미터 | 값 |
|------|----------|-----|
{chr(10).join(geom_rows)}

![Geometry](design_geometry.png)

## 4. 지형별 진단

다음 7종 지형에서 통과 분석:
'''
    for t in terrains_diag:
        md += f'\n### {t}\n![{t}](diagnostics/diag_{t}.png)\n'

    md += '\n## 5. 다중 속도 스윕 + Monte Carlo\n\n'
    md += '`analysis_text.txt`에 상세 표 — 4가지 속도 (0.3/0.5/0.8/1.0 m/s)에서 각 지형 평가.\n\n'
    if mc_samples > 0:
        md += f'`analysis_text.txt` 하단에 Monte Carlo {mc_samples}회 강건성 평가 (±5% perturbation).\n\n'

    if run_mujoco:
        md += '''## 6. MuJoCo 외부 검증

`mujoco_flat.txt`, `mujoco_step.txt`에 실측 동역학 결과. 최적화 모델의 *준정적 가정 vs 실제 동역학* 격차 확인.

'''

    md += '''## 7. 사용/해석 가이드

- **TOI** (Time Of Impact margin): 0 이상 = 안정, 0 이하 = 전복 위험. 0.15 미만은 주의 영역.
- **slip ratio**: F_demand / (μ·N). 1.0 초과 = 슬립 발생 (실제론 못 굴러감).
- **τ peak vs τ_avail(ω)**: 속도 인식 가용 토크. 100% 초과 = 모터 한계 초과.
- **N (Normal force)**: 휠이 0 N이면 들림. 들림 비율이 LIFTOFF_MAX (2%) 넘으면 페널티.

## 8. 한계

- 시뮬레이션은 *준정적* 가정 (각 x 위치에서 Newton constraint). 시간 영역 적분 아님.
- 좌/우 대칭, 직선 주행 가정. 측면 슬립/yaw 미고려.
- 휠은 점 접촉 (envelope). 변형/접촉 패치 미적용.
- 결과는 *비교용 설계 지표*로만 사용. 실제 제작 전 외부 multibody 검증 권장.
'''

    with open(md_path, 'w') as fh:
        fh.write(md)
    return md_path


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--pkl', default=os.path.join(script_dir, 'zetin_optimal_params_v4.pkl'))
    ap.add_argument('--outdir', default=None,
                    help='출력 디렉토리. 기본 review_YYYYMMDD_HHMM')
    ap.add_argument('--mc', type=int, default=0, help='Monte Carlo 샘플 수 (0=비활성)')
    ap.add_argument('--mujoco', action='store_true', help='MuJoCo 검증 포함')
    ap.add_argument('--terrains',
                    default='real_stairs,wood_block,rough,step,curved_ramp,incline_15,incline_30',
                    help='diagnostic plot 지형')
    args = ap.parse_args()

    with open(args.pkl, 'rb') as fh:
        result = pickle.load(fh)
    p_opt = result['p_opt']
    f_opt = result['f_opt']
    version = result.get('version', 'unknown')

    run_ts = datetime.datetime.now().strftime('%Y%m%d_%H%M')
    out_dir = args.outdir or os.path.join(script_dir, f'review_{run_ts}')
    diag_dir = os.path.join(out_dir, 'diagnostics')
    os.makedirs(diag_dir, exist_ok=True)

    py = sys.executable
    print(f'설계 리뷰 시작 → {out_dir}')
    print(f'pkl: {args.pkl}  f_opt={f_opt:.4f}')
    print()

    # 1. Geometry plot
    print('[1/4] 측면뷰 형상 플롯...')
    geom_out = os.path.join(out_dir, 'design_geometry.png')
    run_tool([py, os.path.join(script_dir, 'plot_geometry.py'),
              '--pkl', args.pkl, '--out', geom_out])

    # 2. Diagnostics per terrain
    terrains_list = args.terrains.split(',')
    print(f'[2/4] 지형별 진단 ({len(terrains_list)}종)...')
    run_tool([py, os.path.join(script_dir, 'plot_diagnostics.py'),
              '--pkl', args.pkl, '--terrains', args.terrains,
              '--outdir', diag_dir])

    # 3. Analysis table (with optional MC)
    print('[3/4] 다중 속도 스윕 + 분석...')
    analysis_out = os.path.join(out_dir, 'analysis_text.txt')
    analyze_cmd = [py, os.path.join(script_dir, 'analyze_v4_result.py'),
                   '--pkl', args.pkl]
    if args.mc > 0:
        analyze_cmd += ['--mc_samples', str(args.mc)]
    run_tool(analyze_cmd, capture_path=analysis_out)

    # 4. MuJoCo (옵션)
    if args.mujoco:
        print('[4/4] MuJoCo 외부 검증 (flat + step)...')
        for t in ['flat', 'step']:
            mj_out = os.path.join(out_dir, f'mujoco_{t}.txt')
            run_tool([py, os.path.join(script_dir, 'validate_mujoco.py'),
                      '--pkl', args.pkl, '--terrain', t, '--duration', '6.0'],
                     capture_path=mj_out)
    else:
        print('[4/4] MuJoCo 검증 생략 (--mujoco 옵션으로 활성)')

    # README.md
    print('\n[리포트] README.md 생성...')
    md_path = build_markdown_report(out_dir, p_opt, f_opt, version, run_ts,
                                    terrains_list, args.mujoco, args.mc)

    if run_tool.failures:
        print(f'\n⚠ 설계 리뷰 완료 — 단, {len(run_tool.failures)}개 도구 실패: '
              f'{", ".join(run_tool.failures)} (로그 확인 필요)')
    else:
        print(f'\n✓ 설계 리뷰 완료')
    print(f'  폴더: {out_dir}')
    print(f'  진입점: {md_path}')


if __name__ == '__main__':
    main()
