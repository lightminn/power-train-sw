#!/bin/bash
# v4 (면-기준 물리 수정본) 스모크 — 풀런 전 GPU 파이프라인·정확성 검증.
#   [1] test_v4.py 단위검증 48개 (평지 ΣN≈0.5·mg 무게가드 포함)
#   [2] 옵티마이저 3-iter (캐시·워밍업·DE 배선 확인)
# 경로 정정: 5/20 reorg 후 코드는 parameter_calc/python_gpu_triangle/ 아래.
#SBATCH --job-name=ZETIN_v4_smoke
#SBATCH --partition=gpu2
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=00:30:00
#SBATCH --output=/home1/zetin348/Defence_Robot/logs/smoke_v4_%j.out
#SBATCH --error=/home1/zetin348/Defence_Robot/logs/smoke_v4_%j.err

mkdir -p /home1/zetin348/Defence_Robot/logs
echo "=== 스모크 시작: $(date) ==="
echo "노드: $(hostname)  SLURM_JOB_ID: $SLURM_JOB_ID"
echo "=== GPU 정보 ==="
nvidia-smi
echo "================"

module purge
module load cuda/12.2.1
module load python/3.11.2
source /home1/zetin348/Defence_Robot/python/zetin_env/bin/activate

echo "=== 환경 확인 ==="
echo "Python: $(which python) $(python --version 2>&1)"
python -c "import jax; print('JAX:', jax.__version__); print('Devices:', jax.devices())" 2>&1 | grep -v "plugin configuration"
echo "================"

cd /home1/zetin348/Defence_Robot/parameter_calc/python_gpu_triangle

echo "=== [1/2] 단위 검증 (test_v4.py) ==="
JAX_PLATFORM_NAME=gpu XLA_PYTHON_CLIENT_PREALLOCATE=false python -u test_v4.py
echo "[test_v4 exit=$?]"

echo "=== [2/2] 옵티마이저 스모크 (DE_MAXITER=3, popsize=8, N_PTS=60) ==="
JAX_PLATFORM_NAME=gpu XLA_PYTHON_CLIENT_PREALLOCATE=false \
    DE_MAXITER=3 DE_POPSIZE=8 N_PTS=60 DE_TOL=1e-2 \
    python -u ZETIN_JointOptSearch_v4_gpu.py
echo "[opt smoke exit=$?]"

echo "=== 스모크 종료: $(date) ==="
