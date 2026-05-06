# ZETIN Rocker-Bogie GPU 가속 버전

## 환경 설정

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

### 4. JAX GPU 동작 확인
```python
import jax
print(jax.devices())      # [GpuDevice(id=0, ...)] 이면 성공
print(jax.default_backend())  # 'gpu' 이면 성공
```

## 실행

```bash
cd python_gpu/

# 최적화 실행 (GPU 가속)
python ZETIN_JointOptSearch_v3_gpu.py
```

## GPU 가속 구조

```
CPU 버전 (python/)              GPU 버전 (python_gpu/)
─────────────────               ──────────────────────
fsolve × 640회/eval     →       JAX vmap Newton × 160 동시 (GPU)
for 루프 동역학          →       numpy 벡터 연산 (CPU, GPU 오프로드 가능)
scipy filtfilt           →       numpy 이동평균 (JIT 호환)
scipy interp1d           →       jnp.interp (GPU 네이티브)
```

### 가속 포인트 상세

| 모듈 | CPU 병목 | GPU 가속 방법 | 예상 속도 향상 |
|------|----------|--------------|---------------|
| `kin_sim` | `fsolve` 640회 순차 | `jax.vmap` + Newton 배치 | **15-40x** |
| `ceq` (자코비안) | 수치 미분 | `jax.jacfwd` 자동미분 | **3-5x** |
| `calc_dynamics` | 포인트별 for 루프 | numpy 벡터 연산 | **5-10x** |
| `calc_envelope` | 윈도우 탐색 루프 | `jax.vmap` 병렬 | **10-20x** |
| `calc_stability` | 법선력/TOI 루프 | 벡터 연산 | **5-10x** |

### JIT 컴파일 참고
- 첫 실행 시 JAX가 XLA로 컴파일 (~30초 소요)
- 이후 동일 형태의 호출은 컴파일 캐시 사용 (즉시 실행)
- 워밍업은 스크립트 시작 시 자동 수행됨

## GPU별 참고사항

### RTX 3090 (24GB VRAM)
- FP32 성능: 35.6 TFLOPS
- 이 코드의 N_PTS=160 기준 VRAM 사용량 < 1GB
- `popsize=30`으로 충분히 빠름

### A100 (40/80GB VRAM)
- FP32 성능: 19.5 TFLOPS, FP64: 9.7 TFLOPS
- 기본적으로 FP32 사용 (JAX default)
- 더 큰 `popsize=60` 또는 `N_PTS=320`도 가능
- 메모리 여유가 크므로 배치 크기 확대 가능:
  ```python
  # ZETIN_JointOptSearch_v3_gpu.py에서:
  N_PTS = 320      # 해상도 2배
  popsize = 60     # population 2배
  ```

## 트러블슈팅

### JAX가 CPU로 폴백되는 경우
```bash
# 환경 변수로 강제 GPU 사용
JAX_PLATFORM_NAME=gpu python ZETIN_JointOptSearch_v3_gpu.py

# GPU 메모리 부족 시 프리얼로케이션 비활성화
XLA_PYTHON_CLIENT_PREALLOCATE=false python ZETIN_JointOptSearch_v3_gpu.py
```

### CUDA 버전 불일치
```bash
# 설치된 CUDA 버전에 맞는 JAX 설치
pip install jax[cuda11_pip]   # CUDA 11.x
pip install jax[cuda12]       # CUDA 12.x
```
