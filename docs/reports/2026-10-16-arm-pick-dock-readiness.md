# 새 로봇팔 픽앤플레이스·고정 도크 준비 상태 검증

2026-10-16, 기준 커밋 `520a3b2459bd682fb6a4035bf555d4f1bb073845`.
카메라 보정 완료를 가정한 소프트웨어 준비 상태 점검이다. 실제 카메라 보정이나 실물
픽앤플레이스 성공을 검증한 기록은 아니다. 제어·안전 정책과 실측 제한값은 변경하지 않았다.

## 판정

- 새 팔 자동 픽앤플레이스: **준비 미완료**. 관절각 기준 불일치와 그리퍼 미션 명령 거절을 재현했다.
- 고정 도크 교체: **설치 ROS mock 검증 통과**, 실제 뒤쪽 도크의 도달성·충돌·결합은 미검증.
- 교체 후 픽 미션 자동 재개: 현재 미연결. TOOL_CHANGE 종료 후 일반 운용 경로로 재기동해야 한다.

## 이번에 다시 실행한 검증

컨테이너 `arm_auto_tool_change_check`, 이미지 `extreme-robot-ros2`, ROS_DOMAIN_ID=77.
하드웨어 접근 없이 mock 브리지와 DDS를 사용했다. `/ws` 소스는 읽기 전용이다.

```bash
source /opt/ros/humble/setup.bash
cd /tmp/build-auto
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
colcon build --base-paths /ws/extreme-robot/ros2_ws/src \
  --packages-select robot_arm_msgs robot_arm_description robot_arm_moveit_config dynamixel_control
source install/setup.bash
export ROS_DOMAIN_ID=77
python3 /ws/extreme-robot/ros2_ws/src/dynamixel_control/scripts/verify_auto_tool_exchange.py
python3 /ws/extreme-robot/ros2_ws/src/dynamixel_control/scripts/verify_auto_tool_change_mock.py
python3 /ws/extreme-robot/ros2_ws/src/dynamixel_control/scripts/verify_tool_change_core.py
```

- 설치 빌드: 4 packages finished, exit 0.
- 관련 단위테스트 6파일: 105 passed, exit 0. 읽기 전용 소스의 pytest 캐시 저장 경고 2건.
  파일: test_tool_change_scope.py, test_dock_poses.py, test_auto_tool_change.py,
  test_toolchanger_lock.py, test_toolchanger_sequence.py, test_toolchanger_bridge.py.
- 설치 ROS 전체 교체 21건: 정상 2건, 거부·실패·단계별 중단 19건 통과, exit 0.
  정상은 이동 7회·confirm 1회. 실패/거부는 confirm 0회 및 후속 이동 차단.
  안착 미달은 이동 2회 후 ABORTED, 잠금 위치 유지, UNLOCK 단계 진입 없음.
- 설치 ROS 교체 입력·teach: 무장/차체 허가 없음 거부, 교체 입력 수락, 일반 미션 입력 차단,
  신선도 만료/비상정지 중단, 토크ON 캡처 거부, 토크OFF 캡처·저장·로드, 손상 파일 거부 통과.
- 설치 코어 + 실제 DDS: 단일 목표 7회, 신선한 피드백 도착 확인, 최종 confirm 1회 통과.
- 이번에는 전체 기능 회귀 스위트를 재실행하지 않았다. 이전 기록의 기존 실패 38건이
  해소됐다고 주장하지 않는다. 실제 새 팔 픽앤플레이스 전체 미션은 실행하지 못했다.

## 재현된 차단 문제

### 1. 모델과 런타임의 관절각 허용 범위 불일치

설치된 URDF를 파싱하고 설치된 joint_limits.clamp 및 validate_docks를 호출했다.
모델의 각도 제한 안에 들어오는 아래 두 샘플이 런타임에서 잘리거나 거절된다.
각도 제한 통과는 충돌 없는 자세나 물리적 도달성을 뜻하지 않는다.

| 관절 | 모델 제한(rad) | 런타임 제한(rad) | 입력 → clamp 결과(rad) |
| --- | --- | --- | --- |
| arm_joint_2 | [-3.14159, 0] | [0, 1.7768] | -0.813795 → 0 |
| arm_joint_3 | [-3.14159, 0] | [-0.0737, 2.8557] | -0.935099 → -0.0737 |

두 샘플을 각각 도크 fixture의 안착 포즈에 넣으면
`dock pose outside joint limit: arm_joint_2` 또는 `arm_joint_3`로 거절된다.
이는 도크 알고리즘 실패가 아니라 새 모델과 기존 하드웨어 기준의 미정합이다.
카메라 보정으로 해결되지 않으며, 새 팔 영점·방향·기어비·실제 제한을 대조해야 한다.
실측 근거 없이 제한을 넓히거나 부호를 일괄 반전하지 않았다.

근거: robot_arm_description/urdf/robot_arm.urdf의 arm_joint_2/3,
dynamixel_control/joint_limits.py의 JOINT_LIMITS.

### 2. 자동 미션의 그리퍼 액션 거절

설치된 `ros2 run dynamixel_control moveit_dynamixel_bridge`를 FULL_ROBOT + mock_mode로
도구별 기동하고 `/control/mode_status`에 FSM을 발행했다. 실제 ActionClient에서
`/gripper_controller/follow_joint_trajectory`로 단일 목표를 전송했다.

- dual_motor_gripper: `accepted=False`.
  로그: `gripper goal rejected: MANUAL ownership or tool backend interlock not ready`.
- spur_1motor_gripper: `accepted=False`.
  로그: `spur gripper action rejected: use the FSM or CalibrationSession ingress`.

소스 대조: arm_fsm_node._send_gripper는 이 액션으로 fire-and-forget 전송한다.
미션 기본 모드는 FSM이고 MANUAL이면 _tick이 미션 실행을 중지한다.
브리지 gripper_goal_callback은 spur 액션을 거절하며 다른 그리퍼도 MANUAL을 요구한다.
따라서 모드만 MANUAL로 바꿔 자동 미션 연결을 해결할 수 없다.

이 진단 프로브의 exit 0은 **기존 차단 문제 재현 성공**을 뜻한다.
픽앤플레이스 준비 완료 판정은 FAIL이다.

## 실기 검증을 진행하지 못한 이유

- 호스트 `/dev/ttyUSB*`, `/dev/ttyACM*` 장치 없음.
- `ssh -o BatchMode=yes -o ConnectTimeout=5 jetson ...`: 호스트 이름 해석 실패.
- 실측 도크 9포즈·이동 시간·허용오차 및 실제 뒤쪽 배치 정보 없음.

Jetson 접속 주소를 사용자에게 요청했다. 기존 계획 §5-5의 사람 입회 아래
무도구 이동 → 더미 도구 → 실제 그리퍼 순서는 유지한다. 실제 결합 센서가 없고
도구 낙하를 감지하지 못한다는 사용자 결정도 변경하지 않는다.

## 원시 증거

호스트 `/tmp/arm-readiness-20261016/`에 build.log, tests.log, exchange.log,
readiness.log, scope.log, core.log와 실제 실행한 readiness_probe.py를 보관했다.
임시 파일이므로 장기 재검증 시 이 보고서의 명령과 소스 근거를 함께 확인한다.
