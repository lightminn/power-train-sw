# 2026 국방로봇경진대회 자율주행 SW 개발계획

> 작성: 2026-07-12  
> 상태: 검토용 새 정본 후보  
> 규정 근거: `docs/국방로봇_규정.pdf` SHA-256
> `2a55aaf26933a59f4b2a4279f0e9534b1d6b812e88ff878cdaf95f091f6cc0d3`  
> 적용 범위: 파워트레인 SW와 로봇팔 SW 간 통신 계약  
> 명시적 제외: 극한로봇대회, CAD, 기구 제작, 전장 설계, 배터리·방수 구조

이 문서는 2026 국방로봇경진대회만을 대상으로 하는 파워트레인 SW 실행계획이다. 기존
`2026-07-02-autonomous-driving-kickoff.md`는 개발 이력으로 보존하고, 본 문서가 승인되면
국방대회 자율주행 우선순위와 완료 기준의 새 정본으로 사용한다.

## 1. 규정에서 직접 도출한 목표

대회는 5개 구간에서 자율주행 1회와 원격주행 1회를 각각 수행한다. 각 구간은 100점이며
자율 70점, 원격 30점으로 총점은 자율 350점과 원격 150점이다. 구간별 제한시간은 20분이고,
트랙 이탈·물자 낙하·선도 로봇 접촉은 감점 대상이다. 규정의 트랙 사진, 곡률, 정지선,
장애물 배열은 예시이며 실제 배치는 달라질 수 있다.

따라서 SW의 최상위 목표는 다음 네 가지다.

1. 미지의 실제 배치에서도 트랙을 이탈하지 않고 저속 완주한다.
2. 의미 객체는 로봇팔 인식 결과를 사용하고, 주행 판단과 정지는 파워트레인이 소유한다.
3. 스모그·가림·험지·센서 단절에서 성능을 낮추더라도 안전하게 계속하거나 정지한다.
4. 자율 실패 시 원격으로 즉시 전환해 30점 경로와 남은 경기시간을 보존한다.

고정 지도, 고정 이동거리, 예시 트랙 좌표에 과적합하지 않는다. Nav2와 완전한 3D SLAM은
현재 점수 획득에 필요한 최소 기능이 아니므로 도입하지 않는다.

## 2. 구간별 SW 득점 목표

| 구간 | 자율 평가 항목 | 직접 담당 SW | 핵심 실패 조건 |
|---|---:|---|---|
| 1. 스모그 정찰 | 스모그 35, 피아식별 20, 완주 15 | terrain path, wheel+IMU, 로봇팔 인식 연동 | RGB/depth 동시 저하, 트랙 이탈 |
| 2. 환경 극복 | 사구·자갈·수중 45, 신호 20, 물자 5 | 험지 속도정책, 신호 이벤트, 팔 핸드셰이크 | 슬립, 빨간불 통과, 물자 작업 중 이동 |
| 3. 장애물 식별 | 비전마커 50, 완주 20 | 인식 이벤트 집계, 정차/서행, 중복 방지 | 5개 중복·누락, 오검출 확정 |
| 4. 겨울 | 빙판 35, 제설 25, 완주 10 | 슬립 감지, 저가속·저감속, 구간 상태머신 | 제자리 헛돌기, 과도한 조향 |
| 5. 정찰 동행 | 간격 유지 35, 재추종 25, 완주 10 | 3D target tracking, 가림 예측, 재획득 | 1.5~2.5 m 이탈, 접촉, 다른 대상 추종 |

배점은 기능 우선순위를 정하는 근거지만 규정에 변경 가능성이 명시돼 있다. 구현은 수치가
바뀌어도 정책 파라미터만 교체할 수 있게 한다.

## 3. 현재 재사용 가능한 기반

- WP1~WP5.1: 단일 `can0` 500 kbps, 10모터 제어, 50 Hz 차체 루프, `/cmd_vel`,
  `/wheel_states`, US-100 안전, latched E-stop과 auto-recovering motion hold.
- ROS2 Humble: `powertrain_ros`와 `robot_arm_msgs`, 로봇팔 그래프와 양방향 DDS 전달.
- L515 Gateway: 단일 SDK 소유, RGB 1280×720 30 fps 목표, raw depth 10 Hz, raw gyro/accel,
  TUI와 SRT의 공동 시작·정지, 재연결과 supervised restart.
- 로봇팔 인식: D435i와 `/detected_objects`, `/arm_status`; 파워트레인은 D435i를 열지 않는다.
- 원격 운용: DualSense 텔레옵, 전용 AP, SRT 영상, 진단 TUI.

차체 조립 뒤 수행할 `base_link→l515_link` 실측, 최종 `stop_mm`, ODrive 13·14 재확인은
SW 구현 완료와 분리된 차량 커미셔닝 게이트로 유지한다.

### 3.1 차체와 트랙의 기하 입력

현재 `/home/light/urdf_2/urdf_2.urdf`를 영점 자세로 계산한 잠정 외형은 폭 약 0.96 m,
길이 약 1.08 m, 높이 약 0.63 m다. 타이어 접지면에서 상부 프레임까지는 약 0.57~0.60 m이고,
좌우 타이어 중심 간격은 약 0.879 m다. 규정의 트랙 폭 0.9144 m와 비교하면 바퀴 중심의
좌우 여유는 각 약 18 mm뿐이다. 이 값은 메시와 영점 자세 기반 추정이며 production 파라미터가
아니다.

차체 조립 후 다음 값을 실측해 terrain footprint 설정으로 고정한다.

- 실제 최대 외폭과 타이어 접촉면의 좌우 끝.
- 평탄면에서 접지한 여섯 바퀴 중심과 유효 wheel footprint.
- L515 렌즈 중심의 `base_link` 위치와 고정 pitch.
- 허용 가능한 타이어 돌출 여부와 최소 경계 여유.

SW는 로봇을 점으로 취급하지 않는다. 추정된 주행 가능 영역을 wheel footprint와 위치 불확실성만큼
안쪽으로 축소한 뒤 경로를 선택한다. 축소 후 영역이 사라지면 임계값을 억지로 줄이지 않고
motion hold한다.

## 4. 목표 아키텍처

```text
L515 RGB/depth ──> terrain_path_estimator ┐
L515 gyro/accel ─> odometry_estimator ───┼─> autonomy_controller ─┐
/wheel_states ───> odometry_estimator ───┘                        │
                                                                  ├─> command_authority ─> /cmd_vel
/detected_objects ─> event_adapter ─> segment_supervisor ─────────┤
/arm_status ───────> arm_handshake ───────────────────────────────┤
teleop command ────────────────────────────────────────────────────┘

/safety_verdict + motor health ─> chassis_node final gate ─> ChassisManager ─> 10 motors
```

실제 로봇과 시뮬레이터는 production 알고리즘을 공유하고 하드웨어 경계만 교체한다.

```text
실차: L515 Gateway + real CAN/ChassisManager ─┐
                                               ├─ 동일 ROS 계약 ─> production autonomy nodes
시뮬: sensor bridge + simulated motor plant ──┘
```

production 코드에는 `if simulator == ...` 분기를 넣지 않는다. simulator 선택은 launch와 bridge
구성으로만 결정한다.

### 4.1 소유권 원칙

- L515 하드웨어 소유자는 Gateway 하나뿐이다. 소비 노드는 SDK를 직접 열지 않는다.
- ROS 입력은 raw depth image와 CameraInfo를 유지한다. terrain 프로세스 내부에서는 필요한 ROI를
  point cloud로 복원하지만 상시 ROS PointCloud2 토픽은 추가하지 않는다.
- `ChassisManager`만 CAN을 소유한다. 자율 노드는 `/wheel_states`만 구독한다.
- 로봇팔 팀이 의미 인식의 단일 소스다. 파워트레인은 신호등·마커·마네킹 모델을 중복 실행하지 않는다.
- `command_authority`만 최종 자율/원격 속도 명령을 `/cmd_vel`에 쓴다.
- `chassis_node`가 US-100, 모터 상태, 명령 freshness를 최종 집행한다.
- 수동 reset이 필요한 상태만 E-stop이다. 자동복구 가능한 인식·경로·명령 상실은 motion hold다.

### 4.2 데이터 freshness

각 입력은 값과 함께 stamp, age, validity를 전달한다. 오래된 마지막 값을 정상값처럼 재사용하지
않는다. 영상·depth·인식·odom 중 하나가 끊겨도 서로의 freshness를 독립적으로 판정한다.

## 5. 공통 기반 작업

### WP6-S. Production-parity 시뮬레이션 기반

시뮬레이터 선택의 최우선 기준은 실제 로봇에 배포할 SW를 수정 없이 실행할 수 있는가다.
MuJoCo를 production SW의 자동·폐루프 검증 authority로 사용하고 Isaac Sim은 고충실도
RGB/depth perception challenge set 생성기로 제한한다. 실물 HIL이 최종 authority다.

#### 공통 시뮬레이터 계약

시뮬레이터 adapter는 실제 하드웨어와 같은 ROS 토픽·frame_id·단위·stamp·freshness를 제공한다.

- 발행: L515 color/depth/CameraInfo/gyro/accel. fast mode의 bridge는 `/wheel_states`도 발행하고,
  vcan full-stack mode에서는 실제 `chassis_node`가 `/wheel_states`를 발행. mock
  `/detected_objects`와 `/arm_status`는 외부 ROS2 Humble mock node가 담당.
- 구독: `/cmd_vel`, `/chassis_mode`, `/arrival_status`.
- 추가 ground truth는 `/sim/*` namespace에만 발행하고 production 노드는 이를 구독하지 않음.
- terrain, odometry, controller, authority, supervisor, tracking 코드는 실차와 동일 package와
  container image를 사용.
- 실제 launch와 simulation launch의 차이는 hardware adapter와 파라미터 파일로 제한.

시뮬레이터 공통 frame은 RGB, depth mm, CameraInfo, gyro, accel, wheel states, ground-truth pose,
ground-truth track edge와 stamp를 포함한다. 같은 procedural scenario seed에서 MuJoCo와 Isaac Sim
adapter가 동일한 기하·센서 계약을 생성해야 한다.

#### MuJoCo fast autonomy mode

MuJoCo가 `/cmd_vel`을 직접 받아 articulation을 움직이고 `/wheel_states`를 생성한다. L515 입력은
MuJoCo bridge가 발행하고 로봇팔 입력은 외부 ROS2 Humble mock node가 발행한다. 이 모드는 CAN을
우회하지만 production autonomy 전체를 그대로 실행하며 수백 개 hidden seed를 빠르게 평가한다.

- procedural elevated track: 3D 중심선, 폭, 높이, 뱅크, 곡률, 마찰, 낙하 경계.
- seed를 dev, regression, hidden evaluation, stress로 분리.
- 평탄·뱅크·뱅크 전환·아래 바닥·센서 dropout을 자동 생성.
- completion, wheel clearance, edge crossing, false hold, recovery, runtime을 기록.

#### MuJoCo vcan full-stack mode

실제 `chassis_node`, `ChassisManager`, AK/ODrive CAN protocol과 50 Hz 루프까지 검증할 때 사용한다.

```text
production chassis_node
    ↕ Linux vcan0
10-motor CAN emulator
    ↕ joint target/feedback
MuJoCo AK steering ×4 + ODrive wheel ×6 plant
```

CAN emulator는 AK 명령/상태, ODrive 명령/heartbeat/encoder, node timeout, fault, 지연과 frame
drop을 재현한다. 이 모드에서 실제 CAN frame codec, motor health, `/wheel_states`, command watchdog,
motion hold와 E-stop 경계를 검증한다. production 모터 코드를 simulator API 호출로 바꾸지 않는다.

#### Isaac Sim perception challenge mode

Isaac Sim은 로봇 제어 authority나 별도 production 구현이 아니다. RTX 워크스테이션에서 RGB/depth,
조명, 재질, 그림자, 반사, 가림막, 선도 로봇, 신호등·마커·마네킹 장면을 다양화해 같은 센서
계약의 fixture를 만든다. terrain estimator와 로봇팔 인식은 Isaac Sim 밖의 production ROS2 Humble
프로세스로 실행한다.

Isaac Sim 내부 Python 3.12에 production Humble Python 3.10 package를 복제하지 않는다. Isaac
bridge는 표준 sensor message만 외부로 보내고, `robot_arm_msgs` 생성과 소비는 외부 production 또는
mock container가 담당한다. Isaac Sim 버전과 USD asset은 팀원이 검증한 조합으로 고정하고 대회 전
API upgrade를 금지한다.

#### 시뮬레이션 완료 기준

- production autonomy source에 simulator 이름 분기 0개.
- 같은 recorded input에서 실차 launch와 simulation launch의 알고리즘 출력 일치.
- MuJoCo fast mode hidden seed에서 낙하 방향 wheel footprint 진입 0회와 fail-open 0회.
- MuJoCo vcan mode에서 실제 50 Hz chassis loop, 10모터 상태, watchdog/fault 전이 검증.
- Isaac Sim fixture가 production ROS topic adapter를 통해 terrain/인식 node에 입력됨.
- 시뮬레이터 통과를 실물 성능으로 과장하지 않고 실제 L515·차체 HIL 차이를 보고서에 기록.

### WP6-A. Wheel+IMU 상태 추정

목적은 절대 위치 탐색이 아니라 수 초 범위의 연속적인 거리·방향·자세 추정이다.

- `/wheel_states`로 병진 속도와 이동거리 계산.
- L515 gyro-z 적분과 정지 중 bias 추정으로 yaw 계산.
- accel 저역통과로 roll/pitch와 기울기 추정.
- 바퀴 yaw는 IMU 이상 시 보조 입력으로만 사용.
- wheel 명령/측정, IMU, 추정 속도의 불일치로 슬립과 stuck 후보 산출.
- `/odom`, `/chassis/tilt`, 진단 상태 발행.
- 고정 거리 `odom_m`을 미션 도착의 단독 조건으로 사용하지 않음.

완료 기준:

- 합성 직진·회전·정지·bias·stamp 역행 단위시험.
- 입력 단절 시 stale 전이와 재연결 초기화 시험.
- 평탄면 5 m 직진 오차 ±5% 이내와 제자리 90° 회전 실측.
- 험지에서는 정확도보다 슬립/stuck 검출률을 별도로 기록.

### WP6-B. Bank-aware terrain path estimator

기존 `lane_follower` 개념을 차선이 없고 지면에서 떠 있으며 뱅크가 적용된 펌프트랙용 terrain
perception으로 바꾼다. L515는 전방 장애물의 유무보다 "앞의 어느 3D 표면에 바퀴를 올릴 수
있는가"를 계산하는 센서로 사용한다.

#### 장착과 관측 범위

- L515 렌즈 중심 약 0.60 m, 고정 하향 pitch 25°를 초기 후보로 사용.
- 최종 장착 전 20°·25°·30°를 비교해 근거리 사각지대, 0.5~4 m track coverage,
  선도 로봇 검출을 함께 평가.
- depth FOV 약 70°×55°, RGB FOV 약 69°×42°를 기준으로 차체·브래킷이 광로와 냉각구를
  가리지 않게 배치.
- 최종 `base_link→l515_link`는 실측 전 임시값으로 production 완료 판정에 사용하지 않음.

#### 내부 3D 처리

상시 PointCloud2 발행은 depth image 대비 메모리 복사와 DDS 직렬화가 크므로 금지한다. 대신
terrain 프로세스가 depth와 intrinsics로 고정 ROI의 XYZ point cloud를 내부 생성한다.

```text
raw depth + CameraInfo
    → 고정 ROI/stride
    → 내부 XYZ point cloud
    → L515 extrinsic + WP6 roll/pitch로 중력 정렬
    → 5 cm 후보 해상도의 2.5D elevation grid
    → 높이·법선·경사·거칠기·관측 신뢰도
    → 뱅크/오르막/낙하 경계/장애물 분류
    → footprint-safe 중심경로
```

- 넓고 연속적인 좌우 경사는 뱅크로 수락하고 장애물로 오인하지 않음.
- 짧은 거리의 큰 음의 높이 변화, 표면 종료 뒤 아래 바닥, 지지 표면과 연결되지 않은 영역은
  낙하 경계로 분류.
- 단일 전역 평면이 아니라 작은 grid의 국소 표면과 연결성을 사용해 뱅크 진입부와 펌프트랙
  굴곡을 보존.
- 좌우 낙하 경계 사이를 wheel footprint와 불확실성만큼 erosion한 뒤 중심선과 heading을 계산.
- 카메라 아래 사각지대는 wheel+IMU odometry로 최근 1~2초의 bounded local grid만 이동·누적해
  보완. 장기 지도, loop closure, 전역 SLAM으로 확장하지 않음.
- RGB는 바닥·경계 후보와 진행 방향을 보조하고 depth와 독립 confidence를 가짐.
- 출력은 path offset, heading error, 좌우 wheel clearance, bank angle, longitudinal slope,
  roughness, confidence, stamp.

#### JAX 계산 backend

production 후보는 JAX GPU, 정확성 기준과 fallback은 NumPy로 둔다. 두 backend는 같은 고정 shape
입력과 같은 결과 계약을 사용한다.

- JAX 대상: depth deprojection, 좌표 변환, mask, elevation scatter, 표면 특징 계산.
- ROI, stride, grid shape를 고정하고 invalid point는 shape 변경 대신 mask로 처리.
- 시작 시 dummy frame으로 JIT warm-up을 끝내고 warm-up 전 자율 arm을 금지.
- 주행 중 새로운 shape나 dtype으로 재컴파일하지 않음.
- `XLA_PYTHON_CLIENT_PREALLOCATE=false`를 기본 후보로 검증해 로봇팔 YOLO와 8 GB RAM을 보호.
- JAX/jaxlib/CUDA 조합은 JetPack R36.5 aarch64 컨테이너에서 qualification을 통과한 정확한 버전으로
  함께 pin하고 `jax.devices()`가 의도한 GPU를 반환하는지 preflight에서 확인.
- backend는 프로세스 시작 때 한 번 선택하고 주행 중 자동 전환하지 않음. JAX 시작 실패 시
  사전 성능승인을 받은 NumPy backend만 허용하며, 둘 다 불가하면 motion hold.

JAX 채택은 kernel 단독 속도가 아니라 전체 시스템 영향으로 결정한다. L515 Gateway, software
x264, 전체 ROS/CAN/SRT, 로봇팔 YOLO를 동시에 실행해 terrain p99 30 ms 이하, depth 10 Hz deadline
준수, RGB SRT receiver 29 fps 이상, 동일 장면 YOLO rate 기준선 대비 지속 저하 5% 이하,
OOM 0을 모두 확인한다. JAX가 이 gate를 통과하지 못하면 NumPy를 production으로 사용한다.

완료 기준:

- NumPy와 JAX가 동일 fixture에서 허용오차 안의 elevation/path 결과를 생성.
- 녹화 depth에서 평탄면, 일정 뱅크, 뱅크 전환, 양쪽 낙하 경계, 트랙 아래 바닥을 구분.
- 직선·곡선·부분 가림·낮은 대비에서 confidence가 기대 방향으로 변화.
- footprint erosion 결과가 없거나 입력이 stale하면 속도 명령을 만들지 않음.
- 20°·25°·30° 장착 비교 HIL로 production pitch와 실제 ROI를 기록.

### WP6-C. Autonomy controller와 command authority

- terrain path offset/heading을 4WS 속도·yaw-rate 명령으로 변환.
- 곡률, 좌우 wheel clearance, bank angle, slope, confidence, tilt, slip에 따라 속도 제한.
- 자율, 원격, mission hold의 우선순위를 하나의 상태머신으로 집행.
- 모드 전환은 먼저 0속도를 확인한 뒤 새 작성자에게 권한을 넘김.
- 자율 프로세스 사망, stale terrain path, stale odom은 자동복구 motion hold.
- US-100 near/no-response와 모터 fault의 latched E-stop 정책은 변경하지 않음.

완료 기준:

- 동시에 둘 이상의 명령 작성자가 `/cmd_vel`을 쓰지 않음.
- 자율↔원격 전환 중 비영점 명령이 이어지지 않음.
- 모든 입력 상실 조합에 대해 hold/E-stop 구분 자동시험.

## 6. 구간별 득점 기능

### WP7. 5구간 선도 로봇 추종

기존 후순위 WP9를 공통 기반 직후로 앞당긴다.

- `/detected_objects`의 선도 로봇 3D 위치로 거리와 좌우 오차 계산.
- 목표 간격 2.0 m, 허용범위 1.5~2.5 m를 설정값으로 소유.
- 검출 중에는 거리 PID와 좌우 P/PD 제어.
- 가림 시작 시 마지막 상대속도와 wheel+IMU로 짧게 예측하되 감속.
- 예측 한계를 넘으면 정지하고 재검출을 기다림.
- 재검출은 위치·크기·class·시간 연속성 gate를 통과해야 같은 대상으로 수락.
- 1.5 m 이하 접근 또는 대상 불확실 시 전진 금지.

완료 기준:

- 가짜 target stream으로 간격·가림·오검출·재획득 단위시험.
- 사람/박스 대역 HIL 뒤 실제 UGV 또는 4족 로봇으로 검증.
- 2.0 m ±0.5 m 유지율, 최소거리, 재획득시간, 접촉 0회를 기록.

### WP8. 이벤트 기반 구간 supervisor

하나의 거대한 코스 시퀀서 대신 5개 독립 구간 profile과 공통 상태머신을 사용한다.

공통 상태는 `READY → DRIVE → EVENT_HOLD → ARM_WORK → RESUME → COMPLETE`로 두고,
각 전이는 인식 이벤트, 팔 상태, 운영자 명령, timeout으로 결정한다. odom 거리는 보조 gate와
진단에만 사용한다.

- 1구간: 스모그 진입/이탈, 로봇팔의 피아식별·LED 완료 결과 2건, 완주. 피아식별과
  LED 구동은 로봇팔 계층이 소유하고 파워트레인은 주행 hold/resume만 담당.
- 2구간: 구호물자 작업 정차, 팔 완료, 빨간불 hold, 초록불 resume, 완주.
- 3구간: 서로 다른 marker 5개 집계, 중복 억제, 실패/성공 기록, 완주.
- 4구간: 빙판/제설 mode, stuck recovery 요청, 완주.
- 5구간: follow mode, 가림, 재획득, 완주.

`MISSION_STOP` 송신 뒤 `ArrivalStatus`를 보내고, 같은 `mission_id`의 `DONE`만 재출발을 허용한다.
timeout은 자동 재출발이 아니라 motion hold와 운영자 통보로 끝낸다.

### WP9. 환경 degradation 정책

- 스모그: RGB/depth 유효도 저하를 측정하고 wheel+IMU 기반 저속 통과 또는 hold.
- 사구·자갈·수중: slip/tilt 기반 속도·가감속 제한, stuck 판정.
- 빙판·WD-40: 조향 변화율과 가속 제한, 헛돌기 시 정지 후 제한된 recovery.
- 센서가 회복되면 즉시 최고속으로 복귀하지 않고 안정 프레임 수를 만족한 뒤 단계 복구.

자동 recovery는 정해진 횟수와 거리·시간 한도를 가진다. 한도를 넘으면 원격 전환 대기 상태로
들어가며 무한 재시도하지 않는다.

## 7. 원격주행 30점 보존

자율 기능이 원격 경로를 약화시키면 안 된다.

- Gateway TUI와 RGB SRT 30 fps 목표를 유지.
- 운영 화면에 영상뿐 아니라 command owner, safety, odom, path confidence, target range,
  segment state를 표시.
- `q`, 클라이언트 단절, SRT receiver 단절이 Gateway와 ROS를 죽이지 않는 기존 계약 유지.
- 자율 실패 시 zero-confirmed handover 후 원격으로 전환.
- 원격 중에도 US-100과 모터 E-stop은 동일하게 적용.
- 네트워크 단절과 조종기 단절은 motion hold로 처리하고 자동 재연결.

## 8. 시험 전략

### 8.1 자동시험

순수 Python 코어를 먼저 만들고 ROS 노드는 변환과 I/O만 담당한다.

- odometry: 단위, bias, integration, stale, reconnect, slip/stuck.
- terrain path: NumPy/JAX 동등성, deprojection, gravity alignment, elevation grid, bank,
  낙하 경계, footprint erosion, confidence, stale.
- tracking: 거리 제어, 가림 예측, 잘못된 대상 거부, 접촉 방지.
- supervisor: 구간별 정상 전이, 중복 이벤트, timeout, mission_id mismatch.
- authority: 동시 작성자 방지, zero-confirmed handover, 프로세스 사망.
- safety: motion hold와 latched E-stop 경계 회귀시험.

### 8.2 기록 재생

실차 튜닝 전에 RGB/depth/IMU/wheel/detected_objects를 동기 기록한다. 같은 기록을 반복 재생해
알고리즘과 파라미터 변경 전후를 비교한다. 성공 장면만 고르지 않고 가림, blur, stale, 오검출,
슬립 사례를 회귀 fixture로 보존한다.

### 8.3 시뮬레이션 검증

1. 분석적 fixture로 JAX/NumPy terrain 수학과 경계조건을 검증한다.
2. MuJoCo perception-in-the-loop에서 정답 pose·track edge와 알고리즘 출력을 비교한다.
3. MuJoCo fast mode로 production autonomy 폐루프와 hidden procedural seed를 반복한다.
4. MuJoCo vcan mode로 production chassis/CAN/safety까지 확장한다.
5. Isaac Sim challenge fixture로 RGB/depth 재질·조명·가림 변화에 대한 perception을 검증한다.
6. 실제 L515 기록을 동일 replay adapter에 넣어 sim-to-real 차이를 noise model과 보고서에 반영한다.

시뮬레이션 성공 기준은 예쁜 단일 데모가 아니라 seed 집합의 정량 결과다. 최소 wheel clearance,
track edge overrun, false hold, fail-open, completion, recovery time, estimator runtime을 seed별로
저장한다. dev/regression seed로 튜닝하고 hidden evaluation seed는 완료 판정 때만 실행한다.

### 8.4 Jetson 통합

- `powertrain_ros`, L515 Gateway, 로봇팔 인식, terrain backend 동시 부하에서 CPU/RSS/GPU
  memory와 각 rate 측정.
- NumPy와 JAX 각각 terrain 평균/p99, depth age, CPU/GPU 전송비용, 추가 JIT compile 횟수 측정.
- RGB 30 fps 전송 목표를 우선 보존하고 depth/terrain path는 주기를 낮출 수 있음.
- 프로세스 강제 종료·카메라 분리·네트워크 단절 뒤 고아 프로세스와 중복 SDK owner 0 확인.
- CAN, L515, D435i 소유권 경계를 침범하지 않는지 확인.

### 8.5 HIL 순서

1. 바퀴 부양 상태에서 command authority와 장애 전이.
2. 평탄 저속에서 odometry와 NumPy terrain 기준 구현.
3. L515 20°·25°·30° 장착 비교와 평탄·뱅크·떠 있는 트랙 낙하 경계 검증.
4. 전체 Jetson 부하에서 JAX/NumPy backend production 선택.
5. 모의 표적 추종과 가림막 재획득.
6. 로봇팔 mock을 이용한 5개 구간 상태머신.
7. 실제 로봇팔과 `MISSION_STOP → ARRIVED_* → DONE → resume` 1사이클.
8. 스모그·모래·자갈·수중·빙판 조건별 degradation 시험.
9. 5개 구간 자율/원격 반복 리허설과 점수표 기록.

## 9. 일정과 개발 순서

### 7월 12~19일: 서류와 설계 고정

- 본 계획 승인과 2026 규정 기준 문서 교정.
- SW 아키텍처, 자체 개발 이력, HIL 증거를 제출자료에 반영.
- WP6-S 공통 simulator contract와 production/hardware 경계 고정.
- WP6-A odometry 설계·순수 코어·합성시험 착수.
- 로봇팔 팀과 2026 구간 event 이름, `mission_id`, `MISSION_STOP`, DONE 계약 확정.

### 7월 20~31일: 공통 자율 기반

- WP6-A wheel+IMU 완료.
- WP6-S MuJoCo procedural elevated track, sensor bridge, fast autonomy mode.
- WP6-B NumPy terrain 기준 구현과 JAX 고정 shape kernel.
- L515 장착각·ROI 비교, 뱅크와 낙하 경계 녹화 재생 baseline.
- WP6-C command authority와 자율↔원격 handover.
- 실내 평탄 저속 통합시험.

### 8월 1~9일: 고배점 기능

- WP7 선도 로봇 추종과 가림 후 재획득.
- WP8 2·3·5구간 event supervisor.
- MuJoCo vcan 10모터 emulator와 full-stack mode.
- Isaac Sim perception challenge fixture adapter.
- 로봇팔 mock 및 실제 그래프 합동 1사이클.

### 8월 10~16일: 환경 대응

- WP8 1·4구간 profile.
- WP9 스모그·험지 degradation 정책.
- 실제 또는 최대한 가까운 대체 환경 HIL.

### 8월 17~23일: 전 구간 통합

- 자율/원격 전환, 재시작, 부분점수 전략 검증.
- Jetson 전체 부하와 장시간 안정성.
- 구간별 파라미터를 코드에서 YAML로 동결.

### 8월 24~30일: 반복 리허설

- 5개 구간 각각 자율 3회 이상, 원격 3회 이상.
- 성공률, 점수 예상, 이탈, hold, E-stop, 복구시간 기록.
- 실패율이 높은 기능은 단순화하고 새 기능 추가를 중단.

### 8월 31일~9월 4일: 출전 동결

- release tag와 Jetson exact-HEAD 고정.
- 컨테이너 image, YAML, 운영 절차, 복구 명령 백업.
- 대회 모드에서는 검증되지 않은 자동 recovery와 디버그 기능 비활성.

### 9월 5~6일: 본선

- 구간별 preflight 후 자율/원격 수행.
- 실패 시 부분점수와 남은 시간을 우선해 원격 전환 또는 다음 시도 결정.

## 10. 완료 정의와 중단 기준

기능 완료는 코드 merge가 아니라 다음 다섯 조건을 모두 만족해야 한다.

1. 순수 코어 자동시험과 기존 회귀시험 통과.
2. 해당 production node를 수정하지 않은 MuJoCo hidden-seed simulation 통과.
3. Jetson exact-HEAD 배포와 전체 프로세스 동시 실행.
4. 해당 구간의 정상·센서 단절·프로세스 사망 HIL.
5. 운영자가 로그 없이도 TUI 상태와 절차로 복구 가능.

다음 조건이면 기능 확장을 중단하고 단순 fallback을 선택한다.

- RGB 30 fps 원격 영상 목표를 지속적으로 훼손함.
- 8 GB Jetson에서 OOM 또는 제어주기 위반을 유발함.
- 실제 환경 성공률이 원격주행보다 낮고 남은 기간에 개선 근거가 없음.
- 안전 상태와 자동복구 상태를 운영자가 구분할 수 없음.
- 로봇팔 또는 L515 하드웨어 소유권을 중복시킴.

## 11. 의도적으로 하지 않는 것

- 극한로봇대회 전용 벽 통로, 극한 미션 YAML, 일정, 문서.
- Nav2, 사전 정밀지도, 전역 경로계획, loop closure를 포함한 완전한 SLAM.
- 상시 ROS PointCloud2 추가와 고주기 RGB-depth alignment. 내부 ROI point cloud와 제한된
  debug 시각화는 허용.
- 파워트레인의 신호등·마커·마네킹 중복 인식 모델.
- odometry 누적거리만으로 미션 도착 확정.
- 센서 단절 시 마지막 명령을 유지하는 fail-open 동작.
- 무제한 자동 재시도와 검증되지 않은 대회 당일 기능 추가.
- simulator 전용 알고리즘 fork, production 코드의 simulator 이름 분기, Isaac Sim 내부 Python으로
  production package 복제.

## 12. 문서 정본 관계

- 대회 요구사항: `docs/국방로봇_규정.pdf`.
- 자율주행 SW 우선순위와 일정: 본 문서.
- L515 production 운영: `AGENTS.md`의 L515 Gateway ownership와
  `docs/reports/2026-07-12-l515-gateway-performance-hil.md`.
- 차체 안전: `docs/specs/2026-07-10-wp5-control-safety-hardening-design.md`와
  `docs/reports/2026-07-10-wp5-control-safety-hil.md`.
- 기존 `docs/plans/2026-07-02-autonomous-driving-kickoff.md`는 2025 규정과 극한로봇 범위를
  포함한 역사 문서이며, 본 문서와 충돌하면 본 문서를 따른다.
