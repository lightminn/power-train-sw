# parameter_calc — 형상 파라미터 최적화 (상세 문서)

ZETIN 6륜 로커-보기 로봇의 다리 형상(링크 길이·각도)을 여러 지형에서 시뮬레이션해
모터 토크·안정성·미끄럼·전류·승차감의 가중합 비용을 최소화하는 트랙. MATLAB 원본을
NumPy/SciPy(CPU)와 JAX/CUDA(GPU)로 포팅해 동일 물리식을 공유한다.

## 권위본 (Source of Truth)

- **현재 권위본 = `new_parameter_calc/` 의 v4 면-기준 물리 수정본** (f_opt = 0.2004, HPC A10 GPU 11.7h).
  `new_parameter_calc/parameter_calc/` 안에 matlab/python/python_gpu/python_gpu_triangle 전체가 들어 있고,
  이게 최종 결과·도구(validate_mujoco / cross_validate / analyze / plot)를 포함한 완전본이다.
- 최상위 `python_gpu_triangle/` 는 **초기 v4** (f_opt = 0.2624) — 참조용으로 보존.
- `python_gpu/` = v3 (14차원·4지형), `python/` = CPU 포팅, `matlab/` = 원본 레퍼런스.
- 실행 콘솔 로그·결과 정리: `docs/2026-06-01-param-v4-result-log.md`.
- v4 물리 변경/모델 배경: `new_parameter_calc/docs/specs/2026-05-18-v4-simulation-changes-summary.md`,
  차기 시간영역 모델 설계는 `…/2026-05-18-v5-architecture-design.md`.

> `parameter_calc/` 는 개발 서버 검증본을 그대로 옮긴 것. 코드와 결과물(`*.pkl` / `*.mat` /
> `*.mp4` / `fig*.png`)은 **서버 기준을 신뢰**하고 의도 없이 덮어쓰지 말 것.

## v4 파라미터 공간 (15차원)

면(측면-절반) 기준 물리로, Rocker={triangle, frame} · Bogie={triangle, frame} 모드를 동시 탐색.
7종 지형: 계단(stairs) / 나무 블록(wood) / 거친 노면(rough) / 단차(step) / 곡면 경사(curved_ramp) /
15° 경사 / 30° 경사. (v3 = 14차원·4지형)

## 목적함수 가중치 (v4 권위본 기준)

- 항목 W: tau 0.12 · imbal 0.08 · stab 0.18 · sn 0.06 · fail 0.10 · sat 0.12 · slip 0.12 ·
  cont 0.10 · batt 0.06 · stuck 0.06
- 지형 W: stairs 0.45 · wood 0.12 · rough 0.13 · step 0.10 · curved_ramp 0.10 · incline_15 0.05 · incline_30 0.05
- 스펙: R_w 100 mm, TAU_REF 15 Nm, W_bot 400~700 mm. (질량 가정은 50 kg — 실측 86 kg 재최적화는 미실시)

## 시뮬레이션 파이프라인 (모듈 책임)

| 모듈 (CPU / GPU) | 역할 |
| --- | --- |
| `wpos` / `wpos_jax` | 순기구학: 조인트 각도 → 5점 좌표 (Wf, Wm, Wr, Pb, CG) |
| `ceq` / `ceq_jax` | 휠-지형 접촉 구속 방정식 |
| `kin_sim` / `newton_solver` | 역기구학 (CPU `fsolve` / GPU 배치 Newton + `vmap`) |
| `calc_envelope` / `_jax` | 휠 반경에 대한 지형 Minkowski 합 (상수 의존 → 1회 캐시) |
| `calc_dynamics` / `_jax` | 모터 토크 / 부하 불균형 (면-기준: W_side·mass_side) |
| `calc_stability` / `_jax` | ZMP, Tip-Over Index, 들림 비율, 충돌 |
| `calc_metrics` / `_jax` | 토크 신호 S/N 비 (dB) |
| `gen_terrain` | 지형 프로파일 (v4 7종 / v3 4종, CPU/GPU 공유) |

## 알려진 함정

- **면-기준 물리**: 법선력·수직관성·stuck demand는 측면 절반(0.5W, 0.5mass) 기준. 이전 v4 초기본은
  slip 계산에 전체 W를 써서 2× 낙관이었고 v4 수정본에서 보수적으로 정정 → 이전 pkl 무효, 재실행 필요했음.
- **GPU/CPU 동치 검증**: `test_v4.py`(로컬 v4 함수·15차원·실제 assert 다수)의 핵심 가드 = 평지 `ΣN ≈ 0.5·mass·g`.
- 결과 pkl을 스모크(1-iter) 실행이 덮어쓰지 않도록 주의 (파일명 분리).

## 실행

```bash
# v4 권위본 (GPU, 서버) — new_parameter_calc 안에서
cd parameter_calc/new_parameter_calc/parameter_calc/python_gpu_triangle/
JAX_PLATFORM_NAME=gpu python ZETIN_JointOptSearch_v4_gpu.py

# 결과 시각화 (지형별 주행 mp4)
cd ../python && python ZETIN_Animation_v3.py
```

서버 실행은 SLURM 래퍼(`scripts/run_gpu_triangle.sh`) 사용. 로컬 검증은 x86 dev 컨테이너에서
`jax[cpu]` + scipy 로 가능(테스트·스모크).
