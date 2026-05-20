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

# Python 의존성
#   odrive          : USB ODrive 통신 (motor_control 전 스크립트)
#   pygame          : DualSense (odrive_dualsense_*, robot_*)
#   opencv-python   : cv2 (yolo_*, robot_laptop)
#   ultralytics     : YOLOv8 (PyTorch 함께 설치 — CPU 기본, NVIDIA 있으면 자동 GPU)
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
