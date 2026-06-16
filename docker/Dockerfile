# Power Train SW — 공유 개발용 이미지 (x86_64)
#
# 목표:
#   - 팀원 누구든 동일한 환경에서 motor_control/ 스크립트 실행
#   - 코드는 호스트에 두고 /workspace 에 bind mount (이미지 재빌드 없이 수정/실행)
#   - NVIDIA GPU는 옵션 (compose override 로 활성화)
#   - Jetson Orin Nano 는 별도 Dockerfile.jetson 사용

FROM ubuntu:22.04

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    PIP_NO_CACHE_DIR=1 \
    LANG=C.UTF-8

# 시스템 라이브러리
#   libusb-1.0-0          : ODrive USB 백엔드
#   libsdl2-2.0-0         : pygame 조이스틱(DualSense) 런타임
#   libgl1, libglib2.0-0  : OpenCV 런타임
#   v4l-utils, joystick   : /dev/video*, /dev/input/js* 진단
#   gstreamer1.0-*        : robot_laptop.py JPEG 영상 수신
#   usbutils, udev        : lsusb, hotplug
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3 python3-pip \
        libusb-1.0-0 \
        libsdl2-2.0-0 \
        libgl1 \
        libglib2.0-0 \
        v4l-utils \
        joystick \
        gstreamer1.0-tools \
        gstreamer1.0-plugins-base \
        gstreamer1.0-plugins-good \
        gstreamer1.0-plugins-bad \
        usbutils \
        udev \
        ca-certificates \
    && rm -rf /var/lib/apt/lists/*

# PyTorch — CPU 전용
#   이 이미지는 x86 개발용(코드 작성 · motor_gui 테스트 · OpenVINO/CPU YOLO).
#   실제 YOLO GPU 추론은 Jetson(Dockerfile.jetson, l4t-pytorch)에서만 돈다.
#   ultralytics 가 기본으로 끌어오는 CUDA PyTorch(nvidia/* + triton, ~3.9GB)를 피하려고
#   CPU 전용 torch/torchvision 을 먼저 설치한 뒤 ultralytics 를 올린다 → 이미지 ~4GB 절감.
RUN pip3 install --no-cache-dir \
        --index-url https://download.pytorch.org/whl/cpu \
        torch torchvision

# Python 의존성
#   odrive          : USB ODrive 통신 (motor_control 전 스크립트)
#   pygame          : DualSense (odrive_dualsense_*, robot_*)
#   opencv-python   : cv2 (yolo_*, robot_laptop)
#   ultralytics     : YOLO (위 CPU torch 사용 — x86 은 CPU/OpenVINO, GPU 는 Jetson)
#   openvino        : Intel x86/iGPU 가속 (yolo_openvino_detection)
#   numpy/matplotlib: 일반 도구
#   python-can      : CAN 트랙 (motor_gui CanBackend, steering/ak_control)
#   fastapi/uvicorn : motor_gui 웹 백엔드 (WebSocket 텔레메트리 + REST 제어)
#   httpx, pytest   : motor_gui 테스트 (FastAPI TestClient + 단위/통합)
RUN pip3 install --no-cache-dir \
        odrive \
        pygame \
        opencv-python \
        ultralytics \
        openvino \
        numpy \
        matplotlib \
        python-can \
        fastapi \
        "uvicorn[standard]" \
        httpx \
        pytest

WORKDIR /workspace
CMD ["bash"]
