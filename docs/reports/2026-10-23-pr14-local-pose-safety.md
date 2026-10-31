# PR #14 자세 조작 안전 수정

## 범위

사용자 요청에 따라 자유 지정 해제의 무검사 토크 ON과 예약 자세 이동·볼락 조작 충돌을 수정했다.
자세 이동의 볼락 잠김·준비 상태 게이트는 추가하지 않았다. PR 병합·push·실물 조작은 하지 않았다.

## 동작

- 자유 지정·해제는 writable MANUAL FULL_ROBOT, 비상정지 없음·조그 없음일 때만 허용한다.
  토크 복귀는 현재 위치를 읽은 뒤에도 조건을 재검사해 그 사이 들어온 비상정지를 확인한다.
- 예약·진행 중인 자세 이동이 있으면 볼락 요청만 거부한다. 팔 이동은 계속한다.
- 볼락에 이동 목표가 남아 있는 동안은 새 자세 이동·자유 지정 요청을 거부한다.
  볼락 완료 뒤 다음 요청은 즉시 허용한다. 새 정지 래치·재무장 절차·자동 대기는 없다.
- 요청 검증 실패로 기존 자세 이동을 홀드하지 않는다. 실제 이동 명령 쓰기 실패 때는
  이미 시작한 관절을 홀드하도록 이동 상태를 첫 쓰기 전에 기록한다.
- 기존 관절별 속도, 서보 프로파일과 가속도 설정, 4축 1초 지연, 도착 허용오차 20 tick은 유지한다.
  볼락 정지 검사의 기존 속도·위치 허용 조건도 바꾸지 않았다.

## 검증

새 테스트는 저장소 기존 방식에 맞춰 원본 브리지 메서드를 AST로 추출하고 가짜 레지스터로 실행한다.
실제 callback, 자세 타이머, ToolchangerLock과 ToolchangerSequence를 연결해
자세 이동 → 겹친 볼락 요청 거부 → 자세 완료 → 볼락 이동 → 겹친 자세 요청 거부 →
볼락 완료 → 다음 자세 이동 수락 순서를 확인했다. 정상 이동 중 pose hold 쓰기는 없었다.
이 검증은 ROS/DDS 또는 실물 E2E가 아니다.

추가 회귀 20개 통과. 수정 전 HEAD의 브리지 파일로 같은 테스트를 실행한 음성 대조는
16 failed / 4 passed로 기존 결함을 검출했다. 수정 후 관련 회귀는 다음 명령으로 120 passed였다.

```bash
PYTHONDONTWRITEBYTECODE=1 \
PYTHONPATH=motor_control:ros2/src/powertrain_ros:extreme-robot/ros2_ws/src/dynamixel_control \
python3 -m pytest -q --tb=short -p no:cacheprovider \
  extreme-robot/ros2_ws/src/dynamixel_control/test/test_local_pose_safety.py \
  extreme-robot/ros2_ws/src/dynamixel_control/test/test_local_wrist_jog.py \
  extreme-robot/ros2_ws/src/dynamixel_control/test/test_toolchanger_lock.py \
  extreme-robot/ros2_ws/src/dynamixel_control/test/test_toolchanger_sequence.py \
  extreme-robot/ros2_ws/src/dynamixel_control/test/test_auto_tool_change.py \
  extreme-robot/ros2_ws/src/dynamixel_control/test/test_dock_poses.py \
  extreme-robot/ros2_ws/src/dynamixel_control/test/test_bridge_startup_safety.py
```

## 미검증 범위

현재 호스트에는 ROS·Xvfb 실행 도구가 없으며, 기존 승인된 `arm_local_gui_20261017`
컨테이너도 정지 상태였다. 설치된 ROS 엔트리포인트의 격리 도메인 실행과 실물 부드러움은
아직 재검증하지 않았다. 실행·실기 인수 완료로 해석하지 않는다.
전체 기존 테스트 실패와 #14의 별도 콘솔 스모크 프로브 문제를 해결했다는 주장도 하지 않는다.
