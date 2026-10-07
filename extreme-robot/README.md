# 2025 극한로봇 경진대회 — 개발 환경

ROS 2 Humble 기반 재난대응 로봇 프로젝트입니다. YOLO 비전으로 목표를 인식하고, Dynamixel 로봇팔과 자율주행 파워트레인이 협동해 임무를 수행합니다.
개발 환경은 Docker로 통일되어 있어, 레포를 clone하고 `docker compose up`만 하면 누구나 동일한 환경에서 작업할 수 있습니다.

## 시스템 한눈에

```
[카메라/LiDAR] ──▶ Jetson (ROS 2 Humble)
                     │  YOLO 인식 · Depth 3D 좌표 · SLAM
                     ▼
              Mission Manager (FSM)
              ├──▶ 로봇팔   : MoveIt 경로계획 → Dynamixel 서보
              └──▶ 파워트레인: 자율주행(Nav2) / CAN 모터
```

> 모든 ROS 2 명령은 **Docker 컨테이너 안**에서 실행합니다. 호스트는 `git`과 `docker compose`에만 씁니다.

---

## 1. 사전 요구사항

| 항목           | 버전       | 확인 명령어              |
| -------------- | ---------- | ------------------------ |
| Ubuntu         | 22.04 이상 | `lsb_release -a`         |
| Docker Engine  | 24.0 이상  | `docker --version`       |
| Docker Compose | v2 이상    | `docker compose version` |
| Git            | 아무 버전  | `git --version`          |

<details>
<summary>Docker 설치 (없을 경우)</summary>

```bash
# Docker 공식 설치 스크립트
curl -fsSL https://get.docker.com | sudo sh
# sudo 없이 docker 사용 (로그아웃 후 재로그인 필요)
sudo usermod -aG docker $USER
```
</details>

---

## 2. 빠른 시작

```bash
# 1) 레포 클론
git clone https://github.com/ksp118/extreme-robot.git
cd extreme-robot

# 2) 이미지 빌드 (첫 빌드는 베이스 이미지 다운로드로 10~20분)
docker compose build

# 3) 컨테이너 시작
xhost +local:docker && docker compose up -d

# 4) 컨테이너 진입 (ROS 2 환경 자동 소싱됨)
docker exec -it ros2_humble bash
```

> **WSL2는 더 이상 지원하지 않습니다.** X11/WSLg가 자주 깨져 유지 비용이 커서 `docker-compose.wsl.yml`을 제거했습니다. Ubuntu 네이티브(또는 Jetson)를 쓰세요.

> **Jetson에서 GPU 가속까지 쓰려면** 아래 §2-1을 보세요. 기본 `docker compose up -d`에는 GPU 설정이 들어있지 않습니다.

컨테이너 안에서 빌드·실행:

```bash
cd /root/ros2_ws
colcon build
source install/setup.bash
```

`./ros2_ws`는 호스트와 컨테이너가 공유합니다. 호스트에서 `ros2_ws/src/`를 수정하면 컨테이너에 즉시 반영됩니다.
빌드 산출물(`build/`, `install/`, `log/`)은 `.gitignore`로 제외되니, 각자 컨테이너에서 `colcon build` 하세요.

---

## 2-1. Jetson GPU 가속 (Jetson 사용자만)

YOLO 추론을 GPU로 돌리려면 `docker-compose.gpu.yml`을 **기본 compose 위에 얹어서** 실행합니다.

```bash
xhost +local:docker
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d
```

`-f`를 두 번 쓰는 이유는, GPU 설정을 기본 `docker-compose.yml`에 직접 넣으면 **GPU가 없는 팀원 환경에서 컨테이너 생성 자체가 실패**하기 때문입니다(`unknown or invalid runtime name: nvidia`). 그래서 GPU 설정만 별도 파일로 분리해 두었습니다.

### 사전 조건

| 항목 | 확인 |
| --- | --- |
| nvidia-container-runtime 등록 | `docker info \| grep -i runtime` 에 `nvidia` 가 보여야 함 |
| NVIDIA 드라이버 | `nvidia-smi` 가 동작해야 함 |
| CUDA 툴킷 | 기본값 `/usr/local/cuda-12.6` |
| cuDNN 라이브러리 | 기본값 `$HOME/.cudnn-libs` (`.so` 파일만 모아둔 디렉터리) |

### 경로가 다를 때

기본 경로는 JetPack 기준입니다. 다르면 환경 변수로 덮어씁니다.

```bash
CUDA_HOME=/usr/local/cuda-12.4 \
CUDNN_LIBS=$HOME/mylibs \
docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d
```

> ⚠️ 없는 경로를 마운트하면 Docker가 **에러 없이 root 소유의 빈 디렉터리를 만듭니다.** 컨테이너는 뜨지만 cuDNN을 못 찾거나 X11 인증이 조용히 깨지므로, 위 경로가 실제로 존재하는지 먼저 확인하세요.

> 참고: PyPI의 `torch`는 범용 aarch64 빌드(CPU 전용)라, Jetson에서 GPU를 쓰려면 Jetson용 wheel로 교체해야 합니다(`pypi.jetson-ai-lab.io/jp6/cu126`). 아직 Dockerfile에 반영되지 않아 컨테이너 재생성 시 수동 재설치가 필요합니다.

---

## 3. 패키지 구성 (`ros2_ws/src/`)

| 패키지 | 역할 |
| ------ | ---- |
| **dynamixel_control** | 핵심 런타임. `yolo_detection`(카메라+YOLO) → `yolo_bridge`(P제어) → `position_node`(XL430 서보 구동) 3노드 파이프라인 |
| **robot_arm_description** | 로봇팔 URDF(**5축** + 랙피니언 그리퍼 + 손목 카메라), `display.launch.py`(RViz 시각화) |
| **robot_arm_moveit_config** | MoveIt 경로계획 설정(SRDF/IK/컨트롤러), `demo.launch.py` |
| **robot_arm_perception** | RealSense + YOLO markerless 인식(`perception_node`), SRT 스트리밍, RViz 캘리브 도구 |
| **robot_arm_msgs** | 파워트레인 팀과 공유하는 커스텀 메시지 5개 |
| **robot_arm_gui** | 브라우저 관제 GUI(읽기 전용). 서보 진단·FSM/계약 상태·YOLO 인식·텔레옵 현황 — §4-4 |
| **pick_test_pkg** | 그리퍼 단독 테스트 노드(`pick_test_node`) |

> 각 패키지·노드의 상세 구조는 [`CLAUDE.md`](CLAUDE.md) 참고.

---

## 4. 실행법

### 4-1. 로봇팔 URDF 시각화

```bash
# 컨테이너 안에서
cd /root/ros2_ws
colcon build --packages-select robot_arm_description
source install/setup.bash
ros2 launch robot_arm_description display.launch.py
```

RViz와 joint_state_publisher_gui 창이 함께 뜹니다. **RViz가 처음 열리면 모델이 안 보이므로** 한 번만 아래 설정을 해주세요:

1. Displays → **Fixed Frame**을 `map` → `base_link`로 변경
2. 좌하단 **Add → RobotModel** 추가
3. RobotModel을 펼쳐 **Description Topic → Durability Policy**를 `Volatile` → `Transient Local`로 변경

설정 후 슬라이더로 각 관절을 움직여볼 수 있습니다.

### 4-2. MoveIt 경로계획 (시뮬레이션)

```bash
# 컨테이너 안에서
cd /root/ros2_ws
colcon build --packages-select robot_arm_description robot_arm_moveit_config
source install/setup.bash
ros2 launch robot_arm_moveit_config demo.launch.py
```

RViz **MotionPlanning** 패널에서 목표 자세를 정하고 **Plan & Execute**하면 경로가 계산·실행됩니다.
현재는 mock(가상) 하드웨어라 **실제 서보는 움직이지 않고** 시뮬상 관절만 동작합니다.

- Planning Group: `arm`(팔, base_link→link_6) / `gripper`(손가락)
- 목표 지정: 말단 마커 드래그 / Joints 탭 슬라이더 / Goal State 드롭다운(`home`, `<random valid>`)
- 마커가 빨간색 = IK 해 없음 또는 충돌 → 도달 가능 범위로 이동

### 4-3. 조이스틱 벤치 텔레옵 (5축)

게임패드로 로봇팔 5축을 직접 조작합니다. **파워트레인 없이 팔만 돌리는 벤치 경로**입니다.

```bash
# 하드웨어 없이 RViz 로 확인
ros2 launch dynamixel_control bench.launch.py rviz:=true

# 실서보까지 (U2D2 + Dynamixel 연결 필요)
ros2 launch dynamixel_control bench.launch.py use_hardware:=true rviz:=true
```

> ⚠️ **이 경로는 파워트레인 계약상 production 금지입니다.** `teleop_core → /dynamixel/goal_position → position_node`는 계약이 금지하는 *"direct dynamixel goal publisher"*입니다. 벤치/개발 전용이며 대회 launch 에 넣지 않습니다.

#### XL430 마스터–슬레이브 TCP 벤치 (1축)

PC의 torque-free XL430을 손으로 움직이면 같은 Wi-Fi의 Jetson에 연결된 XL430이
시작 자세 기준 상대 위치를 30 Hz로 추종하는 단축 HIL 도구입니다. ROS/DDS 계약 및
MoveIt을 우회하므로 **벤치 전용**이며, 기존 `position_node`,
`moveit_dynamixel_bridge`, `arm_fsm`과 동시에 실행하면 안 됩니다.

실측 구성, 모터별 속도·PWM·추정 부하 단위, 안전 제한과 실행 명령은
[`dynamixel_control/MASTER_SLAVE_BENCH.md`](ros2_ws/src/dynamixel_control/MASTER_SLAVE_BENCH.md)를
따릅니다.

#### 키맵 (DualSense)

| 입력 | 동작 |
| --- | --- |
| **L1 (누르고 있기)** | **데드맨 — 누르고 있는 동안만 움직입니다. 떼면 즉시 정지.** |
| 왼쪽 스틱 ↔ | `joint_1` (베이스 회전) |
| 왼쪽 스틱 ↕ | `joint_2` (어깨) |
| 오른쪽 스틱 ↕ | `joint_3` (팔꿈치) |
| 오른쪽 스틱 ↔ | `joint_4` (손목 pitch) |
| L3 / R3 | `joint_5` (손목 roll) − / + |
| R1 | 터보 (속도 배율) |
| △ | home — 전 관절 0 복귀 |
| ○ | stop — 현재 위치 고정 / 비상정지 해제 |
| ✕ | **비상정지** (latched — ○ 로 해제) |
| PS | DRIVE/ARM 전환 (미구현 스텁) |
| L2 / R2 | *[예약] 그리퍼 — 아직 배선하지 않음* |

#### 실물 패드가 오면 — 축 번호부터 확인

PS 패드의 축·버튼 인덱스는 **커널 드라이버(`hid-sony` vs `hid-playstation`)에 따라 다릅니다.** 위 기본값이 안 맞으면 코드가 아니라 **파라미터만** 바꾸면 됩니다.

```bash
ros2 topic echo /joy          # 스틱·버튼을 하나씩 움직이며 인덱스 확인
ros2 run dynamixel_control joystick_teleop --ros-args \
  -p axis_ids:="[0,1,4,3,-1]" -p deadman_button:=4
```

#### 패드 없이 검증하기

가짜 `/joy`를 쏘면 실물 패드 없이 전 경로를 확인할 수 있습니다. 축 순서는 `[LX, LY, L2, RX, RY, R2, dpadX, dpadY]`입니다.

```bash
# joy_node 를 끄고 띄운 뒤
ros2 launch dynamixel_control bench.launch.py joy_node:=false rviz:=true

# 왼쪽 스틱을 오른쪽 끝까지 민 상태 + L1(데드맨) 누름 → joint_1 이 0.6 rad/s 로 회전
ros2 topic pub -r 20 /joy sensor_msgs/msg/Joy \
  '{axes: [1.0,0,1,0,0,1,0,0], buttons: [0,0,0,0,1,0,0,0,0,0,0,0,0]}'
```

L1(`buttons[4]`)을 `0`으로 바꾸면 팔이 즉시 멈춥니다.

### 4-4. 관제 GUI (브라우저, 읽기 전용)

서보 전류·온도, 관절 상태, FSM/파워트레인 계약 상태, YOLO 인식 결과와 영상, 원격조종 현황을
**브라우저 한 페이지**에서 봅니다. 새로 설치할 것은 없습니다(파이썬 표준 라이브러리만 씁니다).

```bash
# 컨테이너 안에서
cd /root/ros2_ws
bash src/robot_arm_gui/scripts/run_monitor.sh

# 하드웨어가 없어도 화면 전체를 확인할 수 있습니다 (가짜 토픽 발행)
bash src/robot_arm_gui/scripts/run_monitor.sh fake:=true

# 포트를 바꾸려면
bash src/robot_arm_gui/scripts/run_monitor.sh port:=8089
```

기본은 `127.0.0.1:8088` 입니다. **원격 PC에서 보려면 SSH 포트포워딩을 권장**합니다:

```bash
# 노트북에서
ssh -L 8088:localhost:8088 <jetson-주소>
# 그 다음 브라우저에서 http://localhost:8088
```

현장 Wi-Fi의 아무 기기(폰 포함)에서 바로 보고 싶으면 `bind:=0.0.0.0` 을 주세요.
⚠️ `network_mode: host` 라 포트가 그대로 열립니다.

| 화면 | 내용 |
| --- | --- |
| 상태 스트립 | `/arm_status` + 신선도(0.5초 넘으면 파워트레인이 차를 세웁니다), 섀시 모드와 **작업 허가/잠금**, 차 주행 가능 여부, 어느 드라이버가 떠 있는지 |
| 서보 | 모터별 위치·목표오차·속도 + **전류/트립 여유**, **급변 트립 여유**, 온도 미터 + 60초 스파크라인 |
| 비전 | 영상(MJPEG) + 검출 목록(클래스·확신도·3D 위치·깊이 유무) + `/pick_target` 강조 |
| 원격조종 | 어느 프론트엔드가 붙었는지, jog 활성, `/joy` 신선도, 데드맨 눌림 |
| 이벤트 | 상태 전이·HW 에러·명령 로그 + **트립 블랙박스** 내려받기 |
| Jetson | CPU·메모리·온도(서멀 스로틀링 확인) |

> **영상 소스 두 가지의 비용이 다릅니다.**
> `raw`(원본)는 인식 노드가 어차피 항상 발행하므로 켜도 그쪽 일이 **늘지 않습니다** — 검출 박스는 이 화면이 대신 그립니다.
> `debug`(마스크·거리 오버레이)는 **구독자가 있는 동안에만** 인식 노드가 그리므로 Jetson 부하가 생깁니다.
> 평소엔 `raw`, 마스크나 거리 표시가 필요할 때만 `debug` 를 쓰세요. 기본은 꺼짐입니다.

> **읽기 전용입니다.** 이 모니터는 어떤 토픽도 발행하지 않습니다 — 화면에서 팔을 움직이거나
> 에러를 해제할 수 없습니다. 관측할 수 없는 항목(E-stop 래치, teleop stop 여부, 입력 전압 등)은
> 화면에 "관측 불가"로 이유와 함께 표시됩니다.

### 4-5. YOLO 카메라-Dynamixel 추적 파이프라인

USB 카메라로 스마트폰을 감지하고 Dynamixel 모터가 카메라를 추적하는 파이프라인입니다.

```
카메라 → yolo_detection_node → /yolo/target_center
                                        ↓
               dynamixel_position_node ← yolo_to_dynamixel_bridge
```

`privileged: true` 설정 덕분에 USB 카메라(`/dev/video*`)와 Dynamixel(`/dev/ttyUSB0`)은 별도 설정 없이 컨테이너에서 바로 접근 가능합니다. 단, **컨테이너 시작 전에 USB 장치를 연결**해두어야 합니다.

#### 사전 확인

```bash
# 호스트 — USB 카메라 연결 확인
ls /dev/video*
# /dev/video0 ... 숫자가 클수록 최근 연결 장치 (보통 video2 또는 video3이 USB 카메라)

# 컨테이너 안 — 사용 가능한 카메라 인덱스 확인
python3 -c "
import cv2
for i in range(4):
    cap = cv2.VideoCapture(i, cv2.CAP_V4L2)
    print(f'video{i}:', cap.isOpened())
    cap.release()
"
# True가 나오는 인덱스 중 가장 큰 번호가 USB 카메라 (보통 2)
```

#### 빌드 및 실행

```bash
# 빌드
cd /root/ros2_ws
colcon build --packages-select dynamixel_control
source install/setup.bash
```

터미널을 3개 열고 (각 터미널에서 `docker exec -it ros2_humble bash` → `source /root/ros2_ws/install/setup.bash`):

```bash
# 터미널 1 — YOLO 감지 노드 (q 누르면 종료, headless면 -p show_window:=false)
ros2 run dynamixel_control yolo_detection --ros-args -p camera_device:=2

# 터미널 2 — YOLO-Dynamixel 브릿지
ros2 run dynamixel_control yolo_bridge

# 터미널 3 — Dynamixel 모터 제어
ros2 run dynamixel_control dynamixel_position
```

감지 결과 확인:

```bash
ros2 topic echo /yolo/target_center
# data: [320, 240]   ← 감지된 객체 중심 [x, y] 픽셀 좌표
```

#### 파라미터 (`yolo_detection_node`)

| 파라미터              | 기본값       | 설명                                        |
| --------------------- | ------------ | ------------------------------------------- |
| `camera_device`       | `0`          | `/dev/videoN`의 N 값 (보통 `2`)             |
| `image_width`         | `640`        | 카메라 캡처 해상도 너비 (px)                |
| `image_height`        | `480`        | 카메라 캡처 해상도 높이 (px)                |
| `model_path`          | `yolov8n.pt` | YOLO 모델 파일 경로                         |
| `target_class`        | `cell phone` | 감지할 COCO 클래스 이름                     |
| `conf_threshold`      | `0.5`        | 감지 신뢰도 임계값 (0.0 ~ 1.0)              |
| `publish_debug_image` | `true`       | 바운딩박스 이미지를 토픽으로 발행할지 여부  |
| `show_window`         | `true`       | 감지 윈도우 표시 여부 (headless 환경엔 false) |

```bash
ros2 run dynamixel_control yolo_detection --ros-args \
  -p camera_device:=2 \
  -p target_class:="cell phone" \
  -p conf_threshold:=0.4 \
  -p show_window:=false
```

#### 발행 토픽

| 토픽                       | 메시지 타입                  | 내용                                  |
| -------------------------- | ---------------------------- | ------------------------------------- |
| `/yolo/target_center`      | `std_msgs/Int32MultiArray`   | 감지된 객체 중심 좌표 `[x, y]` (px)   |
| `/yolo/detection_image`    | `sensor_msgs/Image`          | 바운딩박스가 그려진 디버그 이미지     |
| `/dynamixel/goal_position` | `std_msgs/Int32MultiArray`   | 모터 ID + 목표 위치 `[id, position]`  |

---

## 5. 개발 워크플로우

```bash
# 아침에 시작
git pull
xhost +local:docker && docker compose up -d
docker exec -it ros2_humble bash

# 작업 후 push (호스트에서)
git add . && git commit -m "feat: ..." && git push

# 작업 끝
docker compose down
```

소스코드(`ros2_ws/src/`)만 git으로 관리됩니다. `git pull` 후에는 컨테이너에서 다시 `colcon build` 하세요.

### Dockerfile이 변경된 경우 → 이미지 재빌드 필수

시스템 의존성(apt/pip 패키지)은 **재현성을 위해 Dockerfile에만** 추가합니다. 누군가 Dockerfile을 바꿔 push했다면(예: YOLO·MoveIt 의존성 추가) 반드시 재빌드하세요:

```bash
git pull
docker compose down
docker compose build      # 캐시가 꼬이면 docker compose build --no-cache
docker compose up -d
```

> Jetson에서 GPU를 쓰고 있었다면 마지막 줄을 `docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d` 로 바꿔 실행하세요. 그냥 `up -d` 하면 GPU 설정 없이 재생성됩니다(§2-1).

### 새 ROS 2 패키지 추가

```bash
# 직접 만든 패키지
cd /root/ros2_ws/src
ros2 pkg create --build-type ament_python my_package

# apt/pip 패키지는 Dockerfile에 추가 후 재빌드 (위 참고)
```

---

## 6. 트러블슈팅

**GUI 창이 안 뜸**
```bash
xhost +local:docker
echo $DISPLAY            # 보통 :0 또는 :1
```

**파워트레인과 토픽은 보이는데 데이터가 안 옴** — `ipc: host` 누락입니다.

`ros2 topic list`에는 토픽이 멀쩡히 뜨고 publisher 수도 맞는데 `ros2 topic echo`가 아무것도 출력하지 않는 증상입니다. Fast-DDS는 상대가 같은 호스트면 공유메모리(`/dev/shm`)로 데이터를 보내는데, Docker는 컨테이너마다 별도 `/dev/shm`을 줍니다. discovery는 UDP로 하니 성공하고, 데이터만 조용히 사라집니다.

```bash
docker inspect ros2_humble --format '{{.HostConfig.IpcMode}}'   # host 여야 함
```

**양쪽 컨테이너 모두** `ipc: host`여야 합니다 — 우리 `docker-compose.yml`과 파워트레인의 `docker-compose.jetson.yml`(`powertrain_ros` 서비스) 둘 다입니다.

**ros2 명령이 안 됨** — 소싱이 안 된 경우 수동으로:
```bash
source /opt/ros/humble/setup.bash
source /root/ros2_ws/install/setup.bash
```

**colcon build 에러** — 의존성 누락 가능성:
```bash
cd /root/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build
```

**permission denied (ros2_ws 내 파일)** — 호스트/컨테이너 UID 불일치. 호스트에서:
```bash
sudo chown -R $USER:$USER ros2_ws/
```

---

## 7. 브랜치 전략

`main`은 안정 버전만 유지하고, 기능별로 `feat/*` 브랜치에서 작업 후 PR로 merge합니다.

```bash
git checkout -b feat/vision     # 새 기능 브랜치
git add . && git commit -m "feat: 화재 타겟 인식"
git push -u origin feat/vision  # 이후 GitHub에서 PR
```

---

## 8. URDF 교체 후 테스트 절차

CAD에서 새 URDF·mesh를 가져온 뒤, 커밋 전에 아래 4단계로 검증합니다.

> **현재 구성** — 실물 STL mesh 적용, 활성 관절 3개(`joint_1` ~ `joint_3`), 엔드이펙터 `Link4_1_1`

### 8-1. 빌드

```bash
# 컨테이너 안에서
cd /root/ros2_ws
colcon build --packages-select robot_arm_description robot_arm_moveit_config dynamixel_control
source install/setup.bash
```

빌드 에러가 나면 `rosdep install --from-paths src --ignore-src -r -y` 로 의존성을 먼저 해결하세요.

### 8-2. URDF 문법 검증

```bash
check_urdf install/robot_arm_description/share/robot_arm_description/urdf/robot_arm.urdf
```

```
robot name is: robot_arm
------------- Successfully Parsed XML ---------------
root Link: base_link ...
```

위와 같이 `Successfully Parsed` 가 나오면 통과입니다.

### 8-3. RViz 시각화 (URDF + mesh)

```bash
ros2 launch robot_arm_description display.launch.py
```

RViz가 열리면 한 번만 아래를 설정합니다:

1. **Fixed Frame** → `base_link`
2. **Add → RobotModel** 추가
3. RobotModel의 **Description Topic** 에서 **Durability Policy** → `Transient Local`

**확인 포인트**

- 로봇이 STL 실물 형상(회색)으로 보이는지
- joint_state_publisher_gui 슬라이더로 `joint_1` / `joint_2` / `joint_3` 를 움직이면 해당 관절만 반응하는지
- 빨간 에러 없이 모든 링크가 렌더링되는지

### 8-4. MoveIt mock demo (경로 계획)

```bash
ros2 launch robot_arm_moveit_config demo.launch.py
```

**확인 포인트**

- 터미널 로그에서 `arm_controller`, `joint_state_broadcaster` 가 `active` 상태인지 확인
- MotionPlanning 패널 → Planning Group `arm` → **Goal State: random valid** → **Plan** 클릭 → 궤적 애니메이션이 나오는지
- **Execute** 후 joint_states 토픽에 `joint_1` ~ `joint_3` 만 발행되는지

```bash
# 별도 터미널에서
ros2 topic echo /joint_states
```

### 트러블슈팅

| 증상 | 원인 | 조치 |
|------|------|------|
| 링크가 빨간색으로 표시됨 | mesh STL 파일 경로 불일치 | `ls install/.../meshes/` 로 파일명 확인 |
| IK 해 없음 | `joint_1`이 `continuous` 타입 | URDF에서 `revolute` + `<limit>` 추가 고려 |
| 컨트롤러가 inactive | `ros2_controllers.yaml` 관절명 불일치 | `ros2 control list_controllers` 로 상태 확인 |
| RobotModel이 안 보임 | Durability 설정 누락 | RViz에서 `Transient Local` 로 변경 |
