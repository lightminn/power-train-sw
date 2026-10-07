FROM ros:humble-ros-base

# 한글 로케일 설정
RUN apt-get update && apt-get install -y locales \
    && locale-gen ko_KR.UTF-8 \
    && update-locale LANG=ko_KR.UTF-8

ENV LANG=ko_KR.UTF-8

# 필수 패키지 설치
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-turtlesim \
    ros-humble-teleop-twist-keyboard \
    ros-humble-rqt \
    ros-humble-rqt-graph \
    ros-humble-joint-state-publisher-gui \
    ros-humble-dynamixel-sdk \
    ros-humble-dynamixel-workbench \
    python3-serial \
    python3-pip \
    nano \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-plugins-ugly \
    gstreamer1.0-libav \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    && rm -rf /var/lib/apt/lists/*

# Jetson(L4T R36.5, JetPack 6.2, CUDA 12.6 드라이버)에 맞는 PyTorch를 NVIDIA Jetson
# wheel 인덱스에서 먼저 설치한다 — PyPI 기본 torch는 더 최신 CUDA(cu13x)로 빌드돼 있어
# `torch.cuda.is_available()`가 False로 떨어지고 YOLO가 CPU로 폴백된다(2026-07-22
# 실측, 인식/스트리밍 FPS 병목의 근본 원인이었음). --no-deps로 설치하는 이유:
# torch>=2.9는 nvidia-cudss-cu12를 끌어오는데 이게 다시 데이터센터향 범용 aarch64
# cublas(575MB, Tegra iGPU와 호환 안 될 가능성 높음)까지 연쇄 설치함 — torch==2.8.0은
# 이 의존성이 없어 깔끔하다(실측: torch.cuda.is_available()=True, device='Orin').
# ultralytics는 이후에 설치해 이미 만족된 torch/torchvision을 건드리지 않게 한다.
# libopenblas0는 이 wheel의 런타임 의존 라이브러리(apt에 없어 따로 설치 필요).
RUN apt-get update && apt-get install -y libopenblas0 && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-deps torch==2.8.0 torchvision==0.23.0 \
        --index-url https://pypi.jetson-ai-lab.io/jp6/cu126 \
    && pip3 install "numpy<2" ultralytics pyrealsense2 onnx onnxslim \
    && pip3 uninstall -y opencv-python opencv-python-headless

# MoveIt (로봇팔 경로계획) + ros2_control (mock 하드웨어/컨트롤러)
# - ros-humble-moveit: move_group, OMPL, KDL IK, RViz MotionPlanning 플러그인
# - ros-humble-ros2-control: controller_manager, mock_components/GenericSystem
# - ros-humble-ros2-controllers: joint_trajectory_controller, joint_state_broadcaster
RUN apt-get update && apt-get install -y \
    ros-humble-moveit \
    ros-humble-moveit-configs-utils \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    && rm -rf /var/lib/apt/lists/*

# 게임패드 드라이버 (joystick_teleop 의 /joy 소스).
# ros-humble-desktop 에 딸려 들어오지만, 의존성 정책상 명시적으로 설치한다.
RUN apt-get update && apt-get install -y \
    ros-humble-joy \
    && rm -rf /var/lib/apt/lists/*

# ROS 2 환경 자동 소싱
RUN echo "source /opt/ros/humble/setup.bash" >> /root/.bashrc

WORKDIR /root/ros2_ws

CMD ["bash"]
