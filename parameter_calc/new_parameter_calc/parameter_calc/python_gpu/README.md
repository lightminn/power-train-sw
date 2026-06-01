# ZETIN Rocker-Bogie 서스펜션 최적 설계 (Python GPU)

## 개요

로커-보기(Rocker-Bogie) 서스펜션 로봇의 **기구학·동역학 시뮬레이션** 및 **14차원 파라미터 최적 탐색** 코드의 **JAX GPU 가속 버전**입니다.

CPU 버전(`python/`)의 주요 병목인 `fsolve` 순차 호출(640회/평가)을 **JAX vmap + 커스텀 Newton 솔버**로 대체하여 GPU에서 160개 포인트를 동시에 풀고, 동역학·안정성 계산도 벡터화하여 **10~50배 속도 향상**을 달성합니다.

### 가속 원리

```
CPU 버전 (python/)                  GPU 버전 (python_gpu/)
─────────────────                   ──────────────────────
fsolve × 640회 순차호출       →     JAX vmap Newton × 160 동시 (GPU)
수치 미분 자코비안            →     jax.jacfwd 자동미분
포인트별 for 루프 동역학      →     numpy 벡터 연산
scipy filtfilt               →     numpy 이동평균 (JIT 호환)
scipy interp1d               →     jnp.interp (GPU 네이티브)
```

| 모듈 | CPU 병목 | GPU 가속 방법 | 예상 속도 향상 |
|------|----------|--------------|---------------|
| `kin_sim` | `fsolve` 640회 순차 | `jax.vmap` + Newton 배치 | **15-40x** |
| `ceq` (자코비안) | 수치 미분 | `jax.jacfwd` 자동미분 | **3-5x** |
| `calc_dynamics` | 포인트별 for 루프 | numpy 벡터 연산 | **5-10x** |
| `calc_envelope` | 윈도우 탐색 루프 | `jax.vmap` 병렬 | **10-20x** |
| `calc_stability` | 법선력/TOI 루프 | 벡터 연산 | **5-10x** |

## 파일 구조

```
python_gpu/
├── ZETIN_JointOptSearch_v3_gpu.py  # GPU 가속 최적화 메인 스크립트
├── requirements_gpu.txt            # Python 패키지 의존성
├── README.md                       # 이 문서
├── README_GPU.md                   # GPU 환경 설정 상세 가이드
├── functions/
│   ├── __init__.py                 # 모듈 내보내기
│   ├── wpos_jax.py                 # 순기구학 (JAX JIT, lax.switch 분기)
│   ├── ceq_jax.py                  # 역기구학 구속 조건 (jnp.interp)
│   ├── newton_solver.py            # 배치 Newton-Raphson 솔버 (핵심 가속 모듈)
│   ├── calc_envelope_jax.py        # Minkowski 지형 팽창 (jax.vmap)
│   ├── calc_dynamics_jax.py        # 동역학 — 벡터화 (numpy)
│   ├── calc_stability_jax.py       # 전복 안정성 — 벡터화 (wpos_batched + numpy)
│   ├── calc_metrics_jax.py         # 주행 품질 지표 (numpy)
│   └── gen_terrain.py              # 지형 생성 (CPU 버전과 동일)
└── zetin_optimal_gpu_v3.pkl        # 최적화 결과 (실행 후 생성)
```

### 핵심 모듈: newton_solver.py

GPU 가속의 핵심입니다. `scipy.optimize.fsolve`를 대체하는 커스텀 솔버로:
- `jax.jacfwd`: 해석적 자코비안 자동 계산 (수치 미분 대비 3-5배 빠름)
- `jax.lax.while_loop`: JIT 호환 반복 루프
- `jax.vmap`: N개 포인트를 GPU에서 동시에 풀이
- `jax.lax.scan`: 실패 포인트에 대한 warm-start 순차 재시도

## 환경 설정

### 지원 GPU

| GPU | VRAM | FP32 성능 | 권장 설정 |
|-----|------|-----------|-----------|
| RTX 3090 | 24 GB | 35.6 TFLOPS | `N_PTS=160, popsize=30` |
| A100 | 40/80 GB | 19.5 TFLOPS | `N_PTS=320, popsize=60` |

N_PTS=160 기준 VRAM 사용량은 1GB 미만입니다.

### 1. CUDA 확인

```bash
nvidia-smi          # CUDA 드라이버 버전 확인
nvcc --version      # CUDA toolkit 버전 확인 (선택)
```

### 2. Python 환경 (conda 권장)

```bash
conda create -n zetin_gpu python=3.11
conda activate zetin_gpu
```

### 3. 패키지 설치

```bash
# JAX GPU (CUDA 12.x)
pip install jax[cuda12]

# 나머지
pip install numpy scipy matplotlib
```

또는:

```bash
pip install -r requirements_gpu.txt
```

### 4. JAX GPU 동작 확인

```python
import jax
print(jax.devices())          # [GpuDevice(id=0, ...)] 이면 성공
print(jax.default_backend())  # 'gpu' 이면 성공
```

### 의존 패키지

| 패키지 | 최소 버전 | 용도 |
|--------|-----------|------|
| jax[cuda12] | >= 0.4.20 | GPU 가속 (JIT, vmap, jacfwd) |
| numpy | >= 1.23 | 수치 연산 |
| scipy | >= 1.9 | differential_evolution 최적화 |
| matplotlib | >= 3.5 | 시각화 |

## 실행 방법

```bash
cd python_gpu/
python ZETIN_JointOptSearch_v3_gpu.py
```

실행 과정:
1. JAX GPU 디바이스 감지 및 확인
2. JIT 워밍업 (~30초, 첫 실행 시 XLA 컴파일)
3. `differential_evolution`으로 14차원 파라미터 탐색 (GPU 가속)
4. 최적 결과를 `zetin_optimal_gpu_v3.pkl`로 저장

### 환경 변수 옵션

```bash
# GPU 강제 사용 (CPU 폴백 방지)
JAX_PLATFORM_NAME=gpu python ZETIN_JointOptSearch_v3_gpu.py

# GPU 메모리 프리얼로케이션 비활성화 (메모리 부족 시)
XLA_PYTHON_CLIENT_PREALLOCATE=false python ZETIN_JointOptSearch_v3_gpu.py

# 두 옵션 동시 사용
JAX_PLATFORM_NAME=gpu XLA_PYTHON_CLIENT_PREALLOCATE=false python ZETIN_JointOptSearch_v3_gpu.py
```

## 커스터마이징

### 최적화 반복 횟수

`ZETIN_JointOptSearch_v3_gpu.py` 내에서 수정:

```python
# [SECTION 7] 최적화 실행
result = differential_evolution(
    objective, bounds,
    maxiter=200,     # ← 최대 세대 수
    popsize=30,      # ← 세대당 개체 수
    tol=1e-4,        # ← 수렴 허용 오차
    seed=2026,       # ← 난수 시드
)
```

| 파라미터 | 기본값 | 설명 | 조절 가이드 |
|----------|--------|------|-------------|
| `maxiter` | 200 | 최대 세대 수 | 50(빠름) ~ 500(정밀) |
| `popsize` | 30 | 세대당 개체 수 (×차원) | 15(빠름) ~ 60(정밀) |
| `tol` | 1e-4 | 수렴 허용 오차 | 1e-3(조기 종료) ~ 1e-6(엄격) |
| `seed` | 2026 | 난수 시드 | 변경 시 다른 탐색 경로 |

### 시뮬레이션 해상도

```python
# [SECTION 2]
N_PTS = 160    # 지형당 평가 포인트 수
```

| N_PTS | 용도 | fsolve 호출/평가 | GPU 메모리 |
|-------|------|-----------------|------------|
| 80 | 빠른 테스트 | ~320회 | < 0.5 GB |
| 160 | 기본 (권장) | ~640회 | < 1 GB |
| 320 | 고정밀 (A100) | ~1280회 | < 2 GB |

GPU에서는 N_PTS를 늘려도 vmap 배치 처리로 인해 실행 시간 증가가 선형보다 적습니다.

### GPU별 권장 설정

#### RTX 3090 (24GB)

```python
N_PTS = 160        # 기본값 유지
popsize = 30       # 기본값 유지
maxiter = 200      # 기본값 유지
```

#### A100 (40/80GB)

```python
N_PTS = 320        # 해상도 2배 (메모리 충분)
popsize = 60       # 개체군 2배 (탐색 다양성 향상)
maxiter = 300      # 세대 수 증가 (더 정밀한 탐색)
```

### 로봇 물리 파라미터

```python
# [SECTION 1]
p0 = {
    'R_w': 0.100,           # 바퀴 반지름 [m]
    'mass': 30,             # 총 중량 [kg]
    'h_body': 0.300,        # 차체 높이 [m]
    'gear_ratio': 5,        # 기어비
    'motor_tau_peak': 4.95, # 피크 토크 [Nm]
    'motor_tau_cont': 2.75, # 연속 토크 [Nm]
    'v_robot': 0.8,         # 계단 진입 속도 [m/s]
    'mu': 0.70,             # 마찰 계수
    ...
}
```

### 목적함수 가중치

```python
# [SECTION 2]
W = {
    'tau':   0.25,   # 모터 토크 최소화
    'imbal': 0.20,   # 하중 편중도 최소화
    'stab':  0.30,   # 전복 안정성 최대화 (가장 높은 비중)
    'sn':    0.15,   # 주행 품질(S/N비) 최대화
    'fail':  0.10,   # 역기구학 실패율 최소화
}

# 지형별 토크 가중치 (합 = 1.0)
W_terrain = {
    'stairs': 0.55,  # 계단 (핵심 — 경진대회 기준)
    'wood':   0.20,  # 목재 블록
    'rough':  0.15,  # 불규칙 노면
    'step':   0.10,  # 단차
}
```

### 강성 제약 임계값

```python
TAU_REF       = 1.85    # 토크 정규화 기준값 [Nm]
WBOT_MIN      = 0.400   # 보기 바퀴 최소 간격 [m]
WBOT_MAX      = 0.700   # 보기 바퀴 최대 간격 [m]
FAIL_MAX      = 0.10    # 역기구학 실패율 허용 상한
LIFTOFF_MAX   = 0.02    # 바퀴 들림 비율 허용 상한
TOI_WARN      = 0.20    # TOI 경고 임계값
P0_HEIGHT_MAX = 0.500   # 피벗 최대 높이 [m]
```

## JIT 컴파일 참고

- 첫 실행 시 JAX가 XLA로 컴파일합니다 (~30초 소요)
- 이후 동일 형태의 호출은 컴파일 캐시를 사용하여 즉시 실행됩니다
- 워밍업은 스크립트 시작 시 자동 수행됩니다
- `N_PTS`를 변경하면 재컴파일이 필요합니다

## 트러블슈팅

### JAX가 CPU로 폴백되는 경우

```bash
# 환경 변수로 강제 GPU 사용
JAX_PLATFORM_NAME=gpu python ZETIN_JointOptSearch_v3_gpu.py
```

스크립트 시작 시 출력되는 `JAX devices`와 `JAX default backend`를 확인하세요. `[CpuDevice]`로 표시되면 CUDA 설치를 점검해야 합니다.

### CUDA 버전 불일치

```bash
# CUDA 11.x 사용 시
pip install jax[cuda11_pip]

# CUDA 12.x 사용 시 (권장)
pip install jax[cuda12]
```

### GPU 메모리 부족 (OOM)

```bash
# 프리얼로케이션 비활성화
XLA_PYTHON_CLIENT_PREALLOCATE=false python ZETIN_JointOptSearch_v3_gpu.py
```

그래도 부족하면 `N_PTS`를 줄이거나 `popsize`를 줄여보세요.

## CPU 버전과의 차이점

| 항목 | CPU 버전 (`python/`) | GPU 버전 (`python_gpu/`) |
|------|---------------------|-------------------------|
| 역기구학 솔버 | `scipy.optimize.fsolve` (순차) | JAX vmap Newton (GPU 배치) |
| 자코비안 | 수치 미분 (fsolve 내장) | `jax.jacfwd` 자동미분 |
| 지형 팽창 | for 루프 | `jax.vmap` 병렬 |
| 동역학 | 포인트별 for 루프 | numpy 벡터 연산 |
| 안정성 | for 루프 + fsolve | `wpos_batched` + 벡터 연산 |
| 필터링 | `scipy.signal.filtfilt` | numpy 이동평균 |
| 파라미터 전달 | Python dict | 20-element JAX array (JIT 호환) |
| 추가 의존성 | 없음 | JAX + CUDA |
