#!/bin/bash
#SBATCH --nodes=1
#SBATCH --ntasks-per-node=8
#SBATCH --partition=gpu6
#SBATCH --job-name=ZetinOpt
#SBATCH -o SLURM.%N.%j.out
#SBATCH -e SLURM.%N.%j.err

echo "=== Load Python Environment ==="
module load CUDA/11.2.2 
module load python/3.11.2
source zetin_env/bin/activate

echo "=== Run Joint Optimization Search ==="
cd $SLURM_SUBMIT_DIR
python ZETIN_JointOptSearch_v3.py

echo "=== Optimization Done ==="
