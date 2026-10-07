# 교체형 end-effector 하드웨어 검증

## 구조

`arm_fsm_node`의 공통 흐름은 `IDLE → PERCEIVE → PLAN → APPROACH →
TOOL_ACTION → RETRACT → LOCK_CHECK → DONE → STOWING → IDLE`이다. 도구별로
FSM 전체를 복제하지 않는다. `TOOL_ACTION`만 다음 sub-FSM으로 dispatch한다.

- `spur_1motor_gripper`: `GRASP → GRASP_CHECK`
- `dual_motor_gripper`: `GRASP → GRASP_CHECK`
- `cleaner`: 기존 `CLEAN_START → CONTACT_CHECK → CLEAN → CLEAN_STOP`

`tool_profiles.py`는 YAML의 엄격한 검증, `tool_manager.py`는 선택 정책을 소유한다.
`ParameterToolIdentityProvider`는 현재 수동 `tool_type` parameter를 제공한다.
나중에는 이 class 대신 lock/tool-ID sensor 기반 `ToolIdentityProvider`를 주입한다.
VLA는 계속 `/vla/command`의 `PICK`, `CLEAN` 같은 상위 명령만 발행하며 raw tick,
ID, velocity topic/action을 알지 못한다.

Bridge는 `/tool/type`과 JSON `/tool/status`를 발행한다. profile 검증, 모든 actuator
ID ping, read-only/mock 상태를 포함한다. FSM은 신선한 status의 `profile_valid`,
`actuators_discovered`, `motion_allowed`가 모두 true가 아니면 arm 접근 동작부터 막는다.
`/tool/emergency_stop` 또는 `/tool/detached`가 true이면 tool torque/velocity를 정지한다.

## 1모터 spur gripper calibration

다른 Dynamixel/ROS bridge를 먼저 종료하고 전원을 즉시 끌 수 있게 준비한다.

```bash
cd /home/asd/extreme-robot/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run dynamixel_control spur_gripper_calibration \
  --actuator-id 5 --read-only
ros2 run dynamixel_control spur_gripper_calibration \
  --actuator-id 5 --armed --profile-velocity 20 --profile-acceleration 5 \
  --samples 20 --output /tmp/spur_1motor_gripper.yaml
```

첫 명령은 torque/write 없이 ID, position, load, hardware error, operating mode만
확인한다. 두 번째는 항상 torque를 끄고 수동 open/close endpoint와 no-load/grasp/
release sample을 받는다. Ctrl-C, 오류, 정상 종료 모두 마지막에 torque-off를 시도한다.
생성 YAML을 검토한 뒤 아래처럼 직접 지정해 검증한다. 측정 전 저장소 기본 profile은
`calibrated: false`라 실제 모션이 불가능하다.

```bash
ros2 launch dynamixel_control interchangeable_tool.launch.py \
  tool_type:=spur_1motor_gripper \
  tool_profile_file:=/tmp/spur_1motor_gripper.yaml
```

## 도구 전환

교체는 팔이 IDLE/STOWED이고 tool actuator가 정지된 상태에서만 한다. 실행 중 선택을
바꾸는 요청은 `ToolManager`가 거부한다. 현재 1차 구현에서는 자동 감지 없이 프로세스를
정상 종료하고 물리 교체 후 같은 launch의 `tool_type`만 바꿔 다시 시작한다.

기존 2모터 profile(ID 3/4 개별 endpoint와 모드 포함):

```bash
ros2 launch dynamixel_control interchangeable_tool.launch.py \
  tool_type:=dual_motor_gripper
```

cleaner 실제 actuator 정보가 모두 있을 때만:

```bash
ros2 launch dynamixel_control interchangeable_tool.launch.py \
  tool_type:=cleaner \
  cleaning_actuator_joint:=cleaning_actuator_joint \
  cleaning_actuator_id:=6 cleaning_direction:=1 cleaning_velocity_raw:=50
```

하나라도 빠지면 cleaner는 fail-closed다. 기존 `cleaning.launch.py`도 동일하게
`tool_type=cleaner`를 전달하도록 유지했다.

## Hardware test

```bash
# 어떤 tool도 움직이지 않는 진단
ros2 launch dynamixel_control interchangeable_tool.launch.py \
  tool_type:=dual_motor_gripper read_only:=true

ros2 topic echo /tool/type
ros2 topic echo /tool/status

# emergency/detach 정지 검증
ros2 topic pub --once /tool/emergency_stop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /tool/detached std_msgs/msg/Bool '{data: true}'
```

실모션 전 `/tool/status`가 정확한 tool type, `profile_valid=true`,
`actuators_discovered=true`, `motion_allowed=true`인지 확인한다. 이후 기존 VLA/FSM
계약대로 `/vla/command`에 PICK 또는 CLEAN을 보낸다.

## Mock test

```bash
ros2 launch dynamixel_control interchangeable_tool.launch.py \
  tool_type:=spur_1motor_gripper mock_mode:=true
ros2 launch dynamixel_control interchangeable_tool.launch.py \
  tool_type:=dual_motor_gripper mock_mode:=true
ros2 launch dynamixel_control interchangeable_tool.launch.py \
  tool_type:=cleaner mock_mode:=true
```

mock에서는 port를 열거나 torque/register write를 하지 않지만 동일한 TOOL_ACTION
dispatch를 탄다. cleaner contact/lock/distance mock도 켜져 있어 기존 sub-FSM을 검증한다.

## 반드시 실제 측정할 값

- actuator ID와 모델/control table
- open→close tick 증감으로 확인한 direction
- open tick, close tick, 충돌 여유를 둔 safe min/max tick
- profile velocity와 profile acceleration
- no-load 정지/이동 load 또는 current 분포(반복 측정)
- 여러 물체의 grasp load/current 분포(반복 측정)
- release/drop 분포와 그 사이 threshold
- 완전 개폐 시간(`action_time`), hardware error와 장시간 유지 온도
- 2모터는 각 ID별 open/close endpoint와 operating mode를 각각 확인

주소 126 값의 물리 의미는 장착 모델 control table에서 반드시 확인한다. 현재 코드는
기존 장비와의 호환을 위해 이를 signed raw `effort`로 전달한다.

## 자동 tool detection 연결 위치

lock/tool-ID sensor driver가 준비되면 `tool_manager.py`의
`ToolIdentityProvider.detected_tool_type()` 구현을 추가한다. FSM/bridge backend나 VLA를
수정하지 않고 provider만 교체하며, sensor 결과가 unknown/stale이거나 현재 FSM state가
IDLE/STOWED가 아니면 선택 변경을 거부한다.
