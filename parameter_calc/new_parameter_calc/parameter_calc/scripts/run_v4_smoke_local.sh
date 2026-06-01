#!/bin/bash
# v4 GPU 스모크 테스트 — 로컬 실행용 (RTX 2050 등 소형 GPU 가정)
# 본 실행 전 brk_v + curved_ramp 통합과 5종 지형 평가를 빠르게 검증.
#
# 선행 조건:
#   conda env "zetin_gpu" (jax[cuda12] + scipy + numpy 설치 완료)
#   메모리 부족 시 CPU 모드로 폴백: JAX_PLATFORM_NAME=cpu bash run_v4_smoke_local.sh
#
# 예상 소요: RTX 2050에서 maxiter=200, popsize=15 시 약 15~20분.
#           본 실행(maxiter=2000)은 약 2~3시간.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
V4_DIR="$SCRIPT_DIR/../python_gpu_triangle"
PYTHON_BIN="${PYTHON_BIN:-/home/mebbang/anaconda3/envs/zetin_gpu/bin/python}"

echo "=== v4 스모크 시작: $(date) ==="
nvidia-smi --query-gpu=name,memory.free --format=csv,noheader || echo "GPU 미감지 (CPU 폴백 사용)"

cd "$V4_DIR"
DE_MAXITER="${DE_MAXITER:-200}" DE_POPSIZE="${DE_POPSIZE:-15}" \
JAX_PLATFORM_NAME="${JAX_PLATFORM_NAME:-gpu}" \
XLA_PYTHON_CLIENT_PREALLOCATE=false \
    "$PYTHON_BIN" -u ZETIN_JointOptSearch_v4_gpu.py

echo "=== v4 스모크 종료: $(date) ==="
echo "결과: $V4_DIR/zetin_optimal_params_v4.pkl"
