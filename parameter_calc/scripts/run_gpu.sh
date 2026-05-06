#!/bin/bash
#SBATCH --job-name=ZETIN_GPU          # 작업 이름
#SBATCH --partition=gpu2            # GPU 파티션
#SBATCH --gres=gpu:1                  # GPU 1장
#SBATCH --cpus-per-task=8             # CPU 코어 4개
#SBATCH --mem=32G                     # RAM 32GB
#SBATCH --output=/home1/zetin348/Defence_Robot/logs/run_%j.out
#SBATCH --error=/home1/zetin348/Defence_Robot/logs/run_%j.err

mkdir -p /home1/zetin348/Defence_Robot/logs

echo "=== 작업 시작: $(date) ==="
echo "노드: $(hostname)  SLURM_JOB_ID: $SLURM_JOB_ID"

# 1. GPU 확인
echo "=== GPU 정보 ==="
nvidia-smi
echo "================"

# 2. 모듈 로드 — CUDA 12.x 필수 (JAX cuda12 플러그인 요구)
#    CUDA 11.2 로드하면 JAX가 GPU를 찾지 못하고 CPU 폴백 발생
module purge
module load cuda/12.2.1
module load python/3.11.2

# 3. 가상환경 활성화
#    주의: source ~/.bashrc는 virtualenv를 무효화하므로 제거
source /home1/zetin348/Defence_Robot/python/zetin_env/bin/activate

# 4. 환경 확인
echo "=== 환경 확인 ==="
echo "Python: $(which python) $(python --version 2>&1)"
python -c "import jax; print('JAX:', jax.__version__); print('Devices:', jax.devices())" 2>&1 | grep -v "plugin configuration"
echo "================"

# 5. 최적화 실행
echo "=== ZETIN GPU 최적화 시작 ==="
cd /home1/zetin348/Defence_Robot/python_gpu
JAX_PLATFORM_NAME=gpu XLA_PYTHON_CLIENT_PREALLOCATE=false \
    python -u ZETIN_JointOptSearch_v3_gpu.py

echo "=== 작업 종료: $(date) ==="
