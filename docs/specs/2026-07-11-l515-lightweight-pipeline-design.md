# L515 경량 ROS2 파이프라인 설계

> 작성: 2026-07-11
> 개정: realsense-ros 경로 폐기, pyrealsense2 직접 노드로 확정
> 범위: 파워트레인 소유 L515의 color/depth/IMU 입력 계층
> 제외: WP6 오도메트리, WP7 레인 추종, PointCloud2, D435i 제어

## 1. 결정과 근거

Jetson USB에는 L515 `00000000F0271544`와 D435IF `250222071245`가 USB3 5 Gbps로 동시에
연결돼 있다. 기존 librealsense 2.55.1은 D435IF만 SDK 장치로 열거했다. 공식 기준상 L515의
마지막 검증 SDK는 2.50.0이고 2.55.1에서는 L515 지원이 제거됐다.

초기 검토한 realsense-ros 4.0.1은 librealsense 2.50.0과 L515를 지원하지만 Humble 출시 전
버전이라 CMake에서 Humble을 명시적으로 거부했다. rosdep는 별도로 최신 librealsense 2.58.2를
설치해 버전 고정도 깨뜨렸다. 따라서 wrapper 패치나 최신 wrapper 혼합 대신 다음으로 확정한다.

- `powertrain_ros`에 librealsense `v2.50.0` Python binding을 소스 빌드한다.
- 자체 `l515_node`가 pyrealsense2를 직접 사용한다.
- ROS 메시지 변환은 작은 `l515_adapter`에 격리한다.
- realsense-ros는 설치하지 않는다.
- 로봇팔 D435i 컨테이너와 `powertrain_jetson`은 변경하지 않는다.

## 2. 버전과 소유권

| 항목 | 고정값 |
|---|---|
| 실행 컨테이너 | `powertrain_ros` |
| ROS | Humble |
| librealsense/pyrealsense2 | `v2.50.0`, RSUSB source build |
| L515 serial | `00000000F0271544` 필수 |
| node | `/l515_camera_node` |
| namespace | `/l515` |
| D435i owner | 로봇팔 `ros2_humble`, pyrealsense2 2.58.2 |

두 컨테이너가 `/dev`를 공유하므로 자동 장치 선택을 금지한다. 파워트레인은 config에서 지정한
L515 serial만 열고, 없으면 실패/재시도한다. 첫 RealSense나 D435i로 대체하지 않는다.
librealsense 2.50.0은 같은 L515 serial을 `f0271544`로 반환하므로 비교할 때만 대소문자와
선행 0을 정규화한다. 일치 장치가 정확히 하나여야 하며 SDK를 열 때는 열거된 원문 serial을 쓴다.

## 3. 구성요소

### 3.1 `l515_adapter.py`

하드웨어와 rclpy Node에 의존하지 않는 변환 함수 모음이다.

- video profile intrinsics → `CameraInfo`
- color ndarray → `Image` (`bgr8`)
- depth ndarray → `Image` (`16UC1`, mm 단위 원본)
- motion vector → `Imu`
- RealSense millisecond timestamp → ROS `Time`

변환 함수는 frame_id와 stamp를 명시적으로 입력받아 단위시험이 가능해야 한다.

### 3.2 `l515_node.py`

다음만 담당한다.

- 지정 serial로 pipeline/config 생성
- color/depth/accel/gyro 활성화
- frame wait와 메시지 발행
- 연결 실패·분리 감지와 제한된 재연결
- 현재 상태와 오류 로그

WP6/WP7 알고리즘, PointCloud, 안전정지 정책을 포함하지 않는다.

### 3.3 launch/config

`l515.yaml`은 serial, color/depth profile, reconnect interval을 소유한다. launch는 config를
노드에 전달할 뿐 카메라 자동탐색이나 다른 기본값을 만들지 않는다.

## 4. 토픽 계약

| 데이터 | 기본값 | 토픽 | frame_id |
|---|---|---|---|
| color | 640×480, 30 Hz, BGR8 | `/l515/color/image_raw` | `l515_color_optical_frame` |
| color info | color와 동일 stamp | `/l515/color/camera_info` | 동일 |
| depth | 640×480, 30 Hz, Z16 | `/l515/depth/image_rect_raw` | `l515_depth_optical_frame` |
| depth info | depth와 동일 stamp | `/l515/depth/camera_info` | 동일 |
| gyro | 장치 지원 기본 고주기 | `/l515/gyro/sample` | `l515_gyro_frame` |
| accel | 장치 지원 기본 주기 | `/l515/accel/sample` | `l515_accel_frame` |

PointCloud2, IR, confidence, depth-color alignment, 합성 IMU, 후처리, rosbag은 생성하지 않는다.

## 5. 메시지와 시간 변환

### 5.1 Image

- color: height/width/step을 ndarray에서 계산하고 `encoding=bgr8`로 발행한다.
- depth: uint16 원본을 복사하고 `encoding=16UC1`, `step=width×2`로 발행한다.
- depth 값은 밀리미터 단위 원본이다. 미터 float image로 바꾸지 않는다.

### 5.2 CameraInfo

RealSense intrinsics의 `fx, fy, ppx, ppy`, distortion model, coefficients를 ROS K/D/P로
변환한다. 해상도와 stamp는 대응 image와 동일해야 한다.

### 5.3 IMU

gyro는 `angular_velocity`, accel은 `linear_acceleration`만 채운다. 제공하지 않는 orientation은
covariance 첫 값을 `-1`로 표시한다. 축 변환/융합은 WP6이 소유하며 이 노드는 센서 원본 좌표를
frame_id로 명시한다.

### 5.4 Timestamp

첫 frame에서 `offset_ns = ros_now_ns - device_timestamp_ms×1_000_000`을 정하고 같은 연결 세션의
모든 stream에 공유한다. 이후 stamp는 `device_timestamp + offset`으로 계산한다. device timestamp가
역행하거나 재연결되면 offset을 다시 초기화한다. color/depth의 동일 frameset frame은 같은 장치
시간축을 유지한다. 수신할 때마다 무조건 `now()`로 덮어쓰지 않는다.

## 6. 실행 모델과 재연결

블로킹 `wait_for_frames`를 ROS timer callback에서 실행하지 않는다. 전용 worker thread가 SDK를
읽어 bounded queue에 최신 메시지만 넣고, rclpy timer가 queue를 비블로킹 drain해 발행한다.
queue는 stream별 최신 1개만 유지해 지연 누적을 막는다.

장치 부재/분리 시:

1. pipeline을 stop하고 상태를 `DISCONNECTED`로 기록한다.
2. 2초 간격으로 지정 serial만 재탐색한다.
3. D435i가 보여도 열지 않는다.
4. 재연결 시 intrinsics와 timestamp offset을 새로 만든다.
5. 기존 stale 메시지를 재발행하지 않는다.

센서 장애는 이 노드가 차체 E-stop으로 직접 변환하지 않는다. WP6/WP7과 command authority가 입력
freshness 정책을 소유하고 US-100은 독립 충돌 안전을 유지한다.

## 7. 컨테이너

`docker/Dockerfile.ros`에 librealsense 2.50.0을 다음 조건으로 빌드한다.

- `FORCE_RSUSB_BACKEND=ON`
- `BUILD_PYTHON_BINDINGS=ON`
- `PYTHON_EXECUTABLE=/usr/bin/python3`
- graphical examples/tests 비활성
- realsense-ros와 binary `ros-humble-librealsense2` 설치 금지

첫 게이트는 ARM64/Python 3.10에서 `import pyrealsense2`, 버전 2.50.0, 지정 L515 열거다.

## 8. 검증

### 자동시험

- Docker version/build flag 계약
- timestamp offset·역행·reset
- color/depth Image 변환
- CameraInfo intrinsics 변환
- gyro/accel Imu 변환과 orientation unknown 표시
- serial 빈값/D435i 값 거부
- launch/config 계약
- queue 최신값 유지와 reconnect 상태 전이
- 기존 powertrain_ros 32개 회귀시험

### Jetson HIL

1. pyrealsense2 2.50.0이 L515 serial과 firmware/profile을 열거한다.
2. L515 단독 color/depth/accel/gyro 60초 측정.
3. color/depth 평균 29 Hz 이상, 완전한 5초 구간 28 Hz 이상.
4. IMU stamp 단조 증가와 실측 주기 기록.
5. PointCloud2 토픽 부재.
6. L515 분리 시 토픽 중단, D435i fallback 없음, 재연결 복구.
7. 로봇팔 D435i 동시 실행 60초에서 양쪽 비간섭과 Jetson CPU/RAM/USB drop 기록.

L515 firmware가 권장 최소 1.5.8.1 미만이면 스트리밍을 중단하고 별도 승인 전 업데이트하지 않는다.

## 9. 완료 기준

- Python 3.10 ARM64 binding build와 L515 열거
- 변환·상태머신 자동시험 통과
- 기존 ROS 회귀시험 통과
- 단독/동시 60초 HIL 통과
- PointCloud2 기본 부재
- 운영 문서·정본·Notion 동기화

완료 후 WP6은 `/wheel_states`와 gyro/accel을 사용하고 WP7은 color/depth를 사용한다.
