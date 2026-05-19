#!/usr/bin/env bash
# yolo_env 콘다 가상환경 설치 스크립트
# 사용법: bash setup_yolo_env.sh
#
# 포함 패키지: Python 3.10, PyTorch(CUDA), ultralytics(YOLOv8),
#              OpenVINO, OpenCV, ODrive, NumPy, Matplotlib 등

set -e

ENV_NAME="yolo_env"
PYTHON_VERSION="3.10"

# ── conda 설치 확인 ────────────────────────────────────────────────────────────
if ! command -v conda &>/dev/null; then
    echo "[오류] conda를 찾을 수 없습니다."
    echo "  Anaconda 또는 Miniconda를 먼저 설치하세요:"
    echo "  https://docs.conda.io/en/latest/miniconda.html"
    exit 1
fi

# conda 초기화 (비대화형 쉘에서도 activate 사용 가능하게)
eval "$(conda shell.bash hook)"

# ── 환경 이미 존재하면 덮어쓸지 확인 ─────────────────────────────────────────
if conda env list | grep -qE "^${ENV_NAME}\s"; then
    echo "[경고] '${ENV_NAME}' 환경이 이미 존재합니다."
    read -rp "  삭제하고 재설치하시겠습니까? (y/N): " CONFIRM
    if [[ "$CONFIRM" =~ ^[Yy]$ ]]; then
        conda env remove -n "$ENV_NAME" -y
    else
        echo "설치를 취소했습니다."
        exit 0
    fi
fi

# ── 환경 생성 ──────────────────────────────────────────────────────────────────
echo ""
echo "[1/3] Python ${PYTHON_VERSION} 환경 생성 중..."
conda create -n "$ENV_NAME" python="$PYTHON_VERSION" -y

# ── pip 패키지 설치 ────────────────────────────────────────────────────────────
echo ""
echo "[2/3] pip 패키지 설치 중 (시간이 걸릴 수 있습니다)..."

conda run -n "$ENV_NAME" pip install --upgrade pip

# PyTorch (CUDA 13.x)
conda run -n "$ENV_NAME" pip install \
    torch==2.11.0 \
    torchvision==0.26.0 \
    triton==3.6.0

# YOLO / 컴퓨터비전
conda run -n "$ENV_NAME" pip install \
    ultralytics==8.4.33 \
    ultralytics-thop==2.0.18 \
    opencv-python==4.13.0.92

# Intel OpenVINO (인텔 CPU 가속)
conda run -n "$ENV_NAME" pip install \
    openvino==2026.0.0 \
    openvino-telemetry==2025.2.0

# ODrive 모터 드라이버
conda run -n "$ENV_NAME" pip install \
    odrive==0.6.10.post0

# 수치연산 / 시각화
conda run -n "$ENV_NAME" pip install \
    numpy==2.2.6 \
    scipy==1.15.3 \
    matplotlib==3.10.8 \
    pillow==12.1.1 \
    sympy==1.14.0

# 데이터처리
conda run -n "$ENV_NAME" pip install \
    polars==1.39.3 \
    networkx==3.4.2

# 유틸리티
conda run -n "$ENV_NAME" pip install \
    ipython==8.39.0 \
    psutil==7.2.2 \
    requests==2.33.1 \
    pyyaml==6.0.3 \
    filelock==3.25.2 \
    fsspec==2026.3.0

# ── 설치 확인 ──────────────────────────────────────────────────────────────────
echo ""
echo "[3/3] 설치 확인..."
conda run -n "$ENV_NAME" python - <<'EOF'
import torch, cv2, odrive
from ultralytics import YOLO
import openvino as ov
print(f"  torch       : {torch.__version__}  (CUDA: {torch.cuda.is_available()})")
print(f"  opencv      : {cv2.__version__}")
print(f"  odrive      : {odrive.__version__}")
print(f"  openvino    : {ov.__version__}")
EOF

echo ""
echo "=========================================="
echo "  '${ENV_NAME}' 환경 설치 완료!"
echo "  활성화: conda activate ${ENV_NAME}"
echo "=========================================="
