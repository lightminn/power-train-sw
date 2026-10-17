# 자동 도구 교체 운용 인터페이스

정본 계획: `docs/plans/2026-10-16-arm-auto-tool-change.md`.
실행 결과: `docs/reports/2026-10-16-arm-auto-tool-change.md`.

## 동작

운영자는 교체 세션을 한 번 무장하고 원하는 도구 종류만 보낸다. 팔은 현재 도구의
고정 자리로 가서 내려놓고, 요청 도구의 고정 자리에서 결합한 뒤 시작 자세로 돌아온다.
도구 종류마다 고정 자리 하나를 기억한다. 현재 장착 도구는 기동 시 `tool_type`과
이후 완료된 결합 기록으로 관리하며, 해제 중 실패해 상태가 불명확하면 추측하지 않는다.

이 경로는 `control_scope:=TOOL_CHANGE`로 기동한 동일 브리지 안에서 실행된다.
일반 팔 토픽·액션·수동·진단 입력과 외부 도구 토크ON·개폐는 닫는다. 정상 완료나 중단 뒤에도
이 실행 경로는 무장 해제 상태로 남는다. 일반 픽 미션을 재개하려면 교체 브리지를 종료하고
기존 FULL_ROBOT 실행 경로를 기동한다. 미션 FSM 자체는 변경하지 않았다.

각 이동은 단일 목표 하나다. 발행 전 생성돼 큐에 남은 샘플이나 오래된 stamp는 도착 근거로
쓰지 않는다. `/joint_states`의 새 관절각이 허용오차 안이고 전 관절 속도가 0일 때만 진행한다.
팔 속도는 기존 SyncRead 안의 Present Velocity를 관절 단위 rad/s로 변환한다.
다점 궤적·OMPL·MoveIt pose goal·액션 SUCCESSFUL을 완료 판정에 사용하지 않는다.

## 안전 조건과 한계

- 명시적 ARM, 비상정지 비활성, 유효한 3슬롯 포즈 표, 신선한 MISSION_STOP이 필요하다.
  MISSION_STOP은 기존 arm_fsm과 같은 1초 이내여야 하며 수신 시각과 발행 stamp를 함께 본다.
- 도구 미발견의 팔 궤적 차단은 전역에서 완화했다. 도구 토크ON·개폐 게이트는 유지한다.
- 현재 도구의 고정 자리가 비어 있다고 가정한다. 슬롯 점유 센서나 결합/해제 센서는 없다.
  **덜 안착했거나 도크가 바뀌면 도구가 떨어질 수 있다.** 관절각 미달·이동 시간 초과는 잠금을 막는다.
- 부하 프록시는 사용자 결정대로 자동 교체에서 OFF다. 이 경로는
  `toolchanger_coupling_load_threshold:=0`을 요구한다. 결합 직후 DISABLED 결과,
  새 도구 발견, 토크OFF 재준비 READY를 한 번 확인한다. DISABLED는 실제 결합 센서 확인이 아니다.
- 해제 전 기존 도구 정지·토크OFF, 결합 전 새 도구 발견·토크OFF·정지를 검사한다.
  토크OFF 준비는 잠금 토크ON 허가와 별도의 좁은 게이트다. 기존 토크ON 잠금 게이트는 유지한다.
- 해제한 도구 ID는 SyncRead 목록에서 제거한다. 새 도구 재준비와 팔 피드백은 같은 버스 락으로
  직렬화해, 중간에 바뀐 ID 목록을 SDK가 읽지 않게 한다. 버스 소유자는 브리지 하나다.
- 중단 시 후속 이동을 발행하지 않고 현재 자세를 홀드한다. 팔 토크는 유지하고 도구·잠금
  토크는 끈다. 토크OFF/홀드 확인 실패도 ABORTED다. 자동 재시작·자동 재무장은 없다.
- 잠금의 중간 확인 신호는 밖으로 내보내지 않는다. 새 도구 준비와 복귀까지 모두 통과하고
  종료 처리를 확인한 뒤 `/toolchanger/confirm`에 최종 LOCKED 결과를 한 번만 보낸다.
- 해제 시도가 실패해 장착 상태가 불명확하면 새 교체를 거부한다. 실물을 확인하고 기존 수동
  잠금 경로로 복구한 뒤, 실제 장착 도구와 일치하는 `tool_type`으로 새 런타임을 시작해야 한다.
- **아직 실기 인수 전이다.** 실제 포즈·허용오차·시간을 측정하지 않았다. 테스트 fixture를
  실기 포즈 파일로 복사하면 안 된다. 실기에서는 사람이 팔과 도크를 지켜봐야 한다.

## 준비와 기동

기존 잠금 벤치에서 얻은 검증된 영점 파일이 필요하다. 영점 절차는
`docs/reports/2026-10-15-arm-toolchanger-sequence.md`를 따른다.
실제 버스에 다른 position/teleop/bridge 런타임이 남아 있으면 함께 기동하지 않는다.
`tool_type`은 현재 실제 장착 도구여야 한다. 아래 명령은 **ROS 컨테이너 안**에서 실행한다.

```bash
source /opt/ros/humble/setup.bash
cd <로봇팔_워크스페이스>
colcon build --packages-select robot_arm_msgs robot_arm_description dynamixel_control
source install/setup.bash
ros2 run dynamixel_control moveit_dynamixel_bridge --ros-args \
  -p control_scope:=TOOL_CHANGE \
  -p tool_type:=<현재_장착_도구> \
  -p toolchanger_enabled:=true \
  -p toolchanger_zero_file:=<검증된_잠금_영점_JSON_절대경로> \
  -p toolchanger_dock_file:=<도크_JSON_절대경로> \
  -p toolchanger_coupling_load_threshold:=0
```

기대 결과: 브리지가 TOOL_CHANGE 범위로 기동한다. ARM 전에는 팔 토크를 자동으로 켜지 않는다.
도크 파일이 없거나 손상됐으면 미보정으로 시작하며 자동 교체를 거부한다.

## 도크 가르치기

팔을 받친 상태에서 등록된 축을 토크OFF로 두고, 접근·안착·후퇴 위치를 손으로 맞춘다.
**토크OFF 시 팔이 중력으로 처질 수 있다.** 포즈마다 정지한 뒤 CAPTURE를 보낸다.
잠금 영점을 새로 잡는 명령은 교체 범위에서 받지 않는다.

**ROS 컨테이너 안**, 팔을 받친 뒤:

```bash
ros2 topic pub --once /dynamixel/torque_request std_msgs/msg/Int32MultiArray '{data: [0]}'
ros2 topic echo /toolchanger/dock_status
```

기대 결과: 팔·도구 토크OFF. CAPTURE 결과는 dock_status에서 확인한다.
명령은 `/toolchanger/dock_command`, 타입은 `std_msgs/msg/String`, data 내용은 다음 JSON이다.
숫자 자리는 **실측값**으로 채운다. 아래 꺾쇠 표기는 그대로 실행할 수 있는 숫자가 아니다.

```json
{"command":"CAPTURE","slot_id":"1","pose":"approach","tool_type":"spur_1motor_gripper","tolerance_rad":<실측_허용오차>,"timeout_s":<실측_이동_제한시간>}
```

`pose`는 `approach`/`seat`/`retreat`다. 3개 도구의 고정 자리마다 3포즈씩 저장한다.
슬롯 번호는 저장·정비용이다. 교체할 때는 도구 이름으로 자동 선택한다.
토크ON·이동 중·피드백 실패·리밋 위반이면 파일과 직전 유효 표를 보존하고 캡처를 거부한다.
중간에 종료해도 부분 teach 파일을 LOAD해 이어갈 수 있다.

교체의 공통 실측 설정도 같은 토픽에서 한 번 저장한다:

```json
{"command":"CONFIGURE","exchange":{"feedback_timeout_s":<실측_피드백_유효시간>,"home_timeout_s":<실측_복귀_제한시간>,"home_tolerance_rad":<실측_복귀_허용오차>,"unlock_direction":<1_또는_-1>,"lock_direction":<1_또는_-1>}}
```

`{"command":"LOAD"}`는 파일을 재검증한다. 이름·각도·시간·도구 고정 자리 중복을 검사하고,
필수 9포즈가 있어야 `calibrated: true`다. 공통 설정이 없으면 자동 교체 시작을 거부한다.

## 교체 시작과 중단

실제 파워트레인의 신선한 MISSION_STOP이 계속 들어와야 한다. 벤치에서만 기존
mission_console을 사용할 수 있으며, 가짜 계약 발행자를 production에 띄우면 안 된다.

**ROS 컨테이너 안**, 준비가 끝났을 때:

```bash
ros2 topic pub --once /toolchanger/session_command std_msgs/msg/String '{data: "{\"command\":\"ARM\"}"}'
ros2 topic pub --once /toolchanger/change_tool std_msgs/msg/String '{data: dual_motor_gripper}'
ros2 topic echo /toolchanger/change_status
```

기대 결과: ARMED → 현재 도구 자리 이동·해제 → 요청 도구 자리 이동·결합 → 원 자세 복귀 → DONE.
요청할 수 있는 도구는 `spur_1motor_gripper`/`dual_motor_gripper`/`cleaner`다.
현재 장착 도구와 같은 종류를 요청하면 움직이지 않고 거부한다.

중단 명령:

```bash
ros2 topic pub --once /toolchanger/session_command std_msgs/msg/String '{data: "{\"command\":\"ABORT\"}"}'
```

`change_status`는 state, armed, error, attached_tool, step_index, moves_sent, verified_ready를
제공한다. DONE 뒤 verified_ready는 결합 직후 READY 검사 통과 기록이며, 도구 토크ON 상태를
의미하지 않는다. 종료 뒤 도구는 토크OFF다. 교체 실패 시 원인을 지우거나 자동 재시도하지 않는다.

## x86 mock 재현

**호스트**, 이 저장소 루트에서 실행한다. 장치를 연결하지 않고 도메인 77로 격리한다.

```bash
docker run --rm -v "$PWD":/ws:ro extreme-robot-ros2 bash -lc '
  set -e
  source /opt/ros/humble/setup.bash
  mkdir -p /tmp/auto-tool-test
  cd /tmp/auto-tool-test
  export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
  colcon build --base-paths /ws/extreme-robot/ros2_ws/src \
    --packages-select robot_arm_msgs robot_arm_description robot_arm_moveit_config dynamixel_control
  source install/setup.bash
  export ROS_DOMAIN_ID=77
  python3 /ws/extreme-robot/ros2_ws/src/dynamixel_control/scripts/verify_auto_tool_exchange.py
'
```

기대 결과: 정상 2건과 실패 주입 19건에 PASS가 출력된다. 정상은 7회 이동·confirm 1회,
실패는 ABORTED(무장 없는 요청은 IDLE 거부)·confirm 0회다.
실제 모터 이동, 도크 충돌, 도구 낙하 여부를 이 시험으로 검증한 것은 아니다.
