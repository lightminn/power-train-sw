#!/bin/bash
# v4 (면-기준 물리 수정본) 풀 최적화 실행.
# 기존 run_gpu_triangle.sh 의 경로(평면 python_gpu_triangle)가 5/20 reorg 후 stale →
# 본 스크립트는 parameter_calc/python_gpu_triangle/ 로 정정.
# 참고: 동일 설정의 과거 실행은 약 18.5시간 소요(maxiter 도달 전 수렴 시 단축).
#SBATCH --job-name=ZETIN_v4_rev
#SBATCH --partition=gpu2
#SBATCH --gres=gpu:1
#SBATCH --cpus-per-task=8
#SBATCH --mem=32G
#SBATCH --time=24:00:00
#SBATCH --output=/home1/zetin348/Defence_Robot/logs/run_v4_rev_%j.out
#SBATCH --error=/home1/zetin348/Defence_Robot/logs/run_v4_rev_%j.err

mkdir -p /home1/zetin348/Defence_Robot/logs
echo "=== 작업 시작: $(date) ==="
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

echo "=== ZETIN v4 (면-기준 물리 수정본) 삼각형+사각형 동시 탐색 ==="
cd /home1/zetin348/Defence_Robot/parameter_calc/python_gpu_triangle
JAX_PLATFORM_NAME=gpu XLA_PYTHON_CLIENT_PREALLOCATE=false \
    python -u ZETIN_JointOptSearch_v4_gpu.py

echo "=== 작업 종료: $(date) ==="
