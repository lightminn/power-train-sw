# L515 경량 ROS2 파이프라인 설계

> 작성: 2026-07-11  
> 상태: 설계 승인본, 구현 전  
> 범위: 파워트레인 소유 L515의 color/depth/IMU 입력 계층  
> 제외: WP6 오도메트리, WP7 레인 추종, PointCloud2 생성, D435i 제어

## 1. 목적과 확인된 환경

L515는 WP6 오도메트리와 WP7 레인 추종의 공통 입력이다. 기본 출력은 color image,
depth image, accel, gyro이며 PointCloud2는 생성하지 않는다. 센서 처리는 모터 제어와 분리하되
기존 `powertrain_ros` 컨테이너 안에서 실행한다.

2026-07-11 Jetson 실측:

- USB3 5 Gbps에서 L515 `00000000F0271544`와 D435IF `250222071245`가 동시에 열거됐다.
- L515는 커널에서 8개 인터페이스와 752 mA 최대 전력 장치로 정상 인식됐다.
- 기존 `powertrain_jetson`의 librealsense 2.55.1은 D435IF만 SDK 장치로 열거했다.
- librealsense 공식 릴리스 기준 L515 마지막 검증 버전은 2.50.0이고, 2.55.1에서는
  L515/SR300 지원 코드가 제거됐다.
- 로봇팔 `ros2_humble` 컨테이너는 별도 pyrealsense2 2.58.2를 사용한다.

따라서 로봇팔 D435i 환경과 `powertrain_jetson`은 변경하지 않고, `powertrain_ros`에만
L515용 구버전 스택을 고정한다.

## 2. 버전과 소유권

| 항목 | 고정값 |
|---|---|
| 실행 컨테이너 | `powertrain_ros` |
| ROS | Humble |
| librealsense | `v2.50.0` 소스 빌드 |
| Linux backend | `FORCE_RSUSB_BACKEND=ON` |
| realsense-ros | `4.0.1` 소스 빌드 |
| L515 serial | `00000000F0271544` 필수 |
| camera name/namespace | `l515` |
| D435i owner | 로봇팔 `ros2_humble`, 변경 없음 |

realsense-ros 4.0.1은 공식적으로 librealsense 2.50.0과 L515를 지원하지만 Humble 출시 전
버전이다. 첫 구현 게이트는 Humble clean build다. 빌드가 실패하면 임의 패치나 다른 버전으로
조용히 전환하지 않고 설계를 재검토한다.

두 컨테이너가 `/dev`를 공유하므로 카메라 자동 선택을 금지한다. 파워트레인은 반드시 L515
serial을 지정하고, 로봇팔은 기존 D435i 소유권을 유지한다. 파워트레인 노드가 D435i를 열면
실패로 간주한다.

## 3. 컨테이너와 패키지 구조

`docker/Dockerfile.ros`에서 librealsense 2.50.0을 RSUSB backend로 빌드하고 설치한다.
realsense-ros 4.0.1의 다음 패키지를 `ros2/src/`에 vendoring하지 않고 이미지 빌드 단계에서
고정 commit/tag로 소스 빌드한다.

- `realsense2_camera`
- `realsense2_camera_msgs`
- `realsense2_description`

외부 패키지 소스는 이미지 안 `/opt`에 고정하고, 프로젝트 `ros2/` 워크스페이스에는 우리 launch,
설정, 계약시험만 둔다. 이렇게 해야 외부 wrapper 전체가 레포 변경과 colcon 테스트 범위를
오염시키지 않는다.

우리 소유 파일:

- `ros2/src/powertrain_ros/config/l515.yaml`: 시리얼·프로파일·스트림 설정
- `ros2/src/powertrain_ros/launch/l515.launch.py`: 표준 wrapper 기동
- `ros2/src/powertrain_ros/test/test_l515_launch_contract.py`: 정적 launch/config 계약
- `scripts/l515_preflight.sh`: USB serial·중복 소유자·SDK 열거 확인

## 4. 기본 스트림 계약

초기 기준은 안정성과 소비자 호환성을 우선해 다음으로 고정한다.

| 데이터 | 기본 프로파일 | 토픽 |
|---|---|---|
| color | 640×480, 30 Hz | `/l515/color/image_raw` |
| color info | color와 동일 stamp | `/l515/color/camera_info` |
| depth | 640×480, 30 Hz, Z16 | `/l515/depth/image_rect_raw` |
| depth info | depth와 동일 stamp | `/l515/depth/camera_info` |
| gyro | 장치 지원 기본 고주기 | `/l515/gyro/sample` |
| accel | 장치 지원 기본 주기 | `/l515/accel/sample` |

IMU 주기는 librealsense 2.50.0 실기 열거 결과에서 지원 프로파일을 기록하되 wrapper가 제공하는
장치 기본값을 사용한다. 임의 보간으로 가짜 주기를 만들지 않는다. WP6은 accel과 gyro 원본을
받으며, 이 계층에서 `unite_imu_method`로 합성 IMU를 만들지 않는다.

기본 비활성:

- PointCloud2
- infrared/confidence stream
- depth-color alignment
- frame filters와 후처리
- rosbag 기록

소비자가 실제로 요구하기 전에는 활성화하지 않는다. 특히 PointCloud2는 depth image보다 CPU,
메모리, DDS 부하가 크므로 opt-in 계약을 유지한다.

## 5. 프레임과 시간

카메라 내부 optical frame은 wrapper 표준을 유지한다. 로봇 기준 정적 관계는
`base_link → l515_link`이고 REP-103을 따른다. 실제 장착 위치·회전값은 차체 조립 후 측정하므로
이번 구현에서는 임의 숫자를 넣지 않는다. static TF 미설정 상태를 명시적으로 진단하되 이미지와
IMU bring-up을 막지는 않는다.

모든 소비자는 ROS header stamp를 사용한다. 수신 시각으로 덮어쓰지 않는다. 다음을 관찰한다.

- color/depth stamp 단조 증가
- color/depth frame rate
- accel/gyro stamp 단조 증가
- 현재 시각 대비 데이터 age
- 60초 동안 frame drop과 최대 interval

## 6. 기동과 장애 처리

기동 전 preflight가 다음을 검사한다.

1. serial `00000000F0271544`가 USB3로 존재한다.
2. SDK 2.50.0이 해당 serial을 열거한다.
3. 다른 프로세스가 L515를 점유하지 않는다.
4. D435i serial을 선택하지 않는다.

L515 부재, serial 불일치, SDK 열거 실패, 프로파일 불일치 중 하나라도 있으면 노드는 실패 종료한다.
자동으로 첫 번째 RealSense 장치를 선택하거나 D435i로 대체하지 않는다.

런타임 분리·재연결은 wrapper의 장치 복구를 허용하되 소비자가 stale 데이터를 정상으로 오인하지
않도록 토픽 중단으로 드러낸다. 센서 장애가 차체 E-stop을 직접 발생시키는 정책은 이 계층에 넣지
않는다. WP6/WP7과 command authority가 자기 입력 freshness 정책을 소유하며, US-100은 독립
충돌 안전을 계속 담당한다.

## 7. 검증

### 7.1 자동시험

- Dockerfile이 librealsense `v2.50.0`과 realsense-ros `4.0.1`을 고정한다.
- launch가 L515 serial과 namespace를 필수 전달한다.
- color/depth/accel/gyro가 켜지고 PointCloud2·IR·alignment가 꺼져 있다.
- D435i serial 또는 빈 serial 설정을 거부한다.
- 기존 `powertrain_ros` 메시지·WP5 시험이 그대로 통과한다.

### 7.2 Jetson HIL

1. `powertrain_ros` clean image build와 3개 RealSense ROS 패키지 로드.
2. SDK가 L515 한 대를 지정 serial로 열거하고 firmware·지원 프로파일 기록.
3. L515만 기동한 60초 측정.
4. L515와 로봇팔 D435i 동시 기동 60초 측정.
5. color/depth 각각 평균 29 Hz 이상, 완전한 5초 구간 28 Hz 이상.
6. accel/gyro 수신, stamp 단조 증가, 데이터 age 관찰.
7. PointCloud2 토픽이 생성되지 않음.
8. Jetson RAM·CPU·USB drop을 전후 비교해 기록.
9. L515 분리 시 노드/토픽 장애가 명확히 드러나고 D435i로 잘못 전환하지 않음.
10. 재연결 후 지정 L515만 복구.

HIL 중 L515 펌웨어 변경은 하지 않는다. SDK 2.50.0이 요구하는 최소 firmware 1.5.8.1 미만으로
확인될 경우 스트리밍을 중단하고 별도 승인 후 firmware 작업을 계획한다.

## 8. 완료 기준과 후속 작업

다음이 모두 충족되면 L515 경량 파이프라인을 완료한다.

- Humble에서 고정 버전 clean build
- L515 serial 강제 선택
- color/depth/accel/gyro 60초 HIL
- D435i 동시 사용 시 상호 비간섭
- PointCloud2 기본 비활성
- 자원·주기·drop 결과 문서화
- launch/config/운영 문서와 Notion 동기화

완료 후 WP6은 `/wheel_states`와 L515 gyro/accel을 입력으로 오도메트리를 설계한다. WP7은
L515 color/depth image를 사용한다. 이 파이프라인은 두 소비자의 알고리즘을 포함하지 않는다.

## 9. 근거

- librealsense 공식 Release Notes: L515 마지막 검증 SDK 2.50.0, 2.55.1에서 EOL 지원 제거
- realsense-ros 4.0.1 공식 릴리스: librealsense 2.50.0, L515/L535, ROS2 지원
- `docs/plans/2026-07-02-autonomous-driving-kickoff.md`: L515 소유권과 경량 파이프라인 결정
- `docs/reports/2026-07-10-project-and-jetson-state.md`: 센서 분리와 다음 작업 정본
