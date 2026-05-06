#!/bin/bash
#SBATCH --job-name=ZETIN_v4_tri          # 작업 이름
#SBATCH --partition=gpu2                  # GPU 파티션
#SBATCH --gres=gpu:1                      # GPU 1장
#SBATCH --cpus-per-task=8                 # CPU 코어 8개
#SBATCH --mem=32G                         # RAM 32GB
#SBATCH --output=/home1/zetin348/Defence_Robot/logs/run_v4_%j.out
#SBATCH --error=/home1/zetin348/Defence_Robot/logs/run_v4_%j.err

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

echo "=== ZETIN v4 삼각형+사각형 동시 탐색 시작 ==="
cd /home1/zetin348/Defence_Robot/python_gpu_triangle
JAX_PLATFORM_NAME=gpu XLA_PYTHON_CLIENT_PREALLOCATE=false \
    python -u ZETIN_JointOptSearch_v4_gpu.py

echo "=== 작업 종료: $(date) ==="
