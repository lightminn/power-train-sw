# PR #8 기준 통합 콘솔 실물 연결 준비 상태

대상: https://github.com/lightminn/power-train-sw/pull/8
PR head `c1e915f2e5794a94f3de9b5aa0be7c0b3355ca7c`, merge
`00eaf8b9e262ad305005961fc44a244740ec82b2`. 둘 다 현재 브랜치의 선행 기록이다.
검증 대상 GUI는 현재 작업 중인 통합 GTK 콘솔이며 별도 Qt GUI와 구분한다.

## 판정

기존 듀얼 그리퍼의 MANUAL·토크ON·개폐·정지는 설치 ROS mock 전체 경로에서 통과했다.
그러나 **전체 GUI 실물 연결 준비는 미완료**다. 이번에는 검증·기록만 추가했고 기존 제어
코드·실측값·동료의 미추적 작업을 변경하지 않았다. 실제 팔·조이스틱은 연결돼 있지 않다.

## PR과 실제 런타임 구분

1. 기본 파워트레인 배포는 ros2/src/dynamixel_control을 빌드한다.
   powertrain_ros/launch/arm_tool.launch.py는 END_EFFECTOR_ONLY로 실행하며 팔 관절을
   구동하는 기동 경로가 아니다. PR #8의 dual_relative_targets와 듀얼 FSM 수정은 여기에 있다.
2. 새 잠금·자동 도크 교체 작업은 extreme-robot/ros2_ws/src/dynamixel_control에 있다.
   기존 설치 교체 검증과 잠금 카드 E2E는 이 런타임으로 수행했다.
3. 두 경로에 같은 패키지 이름이 있으므로 실제 source한 install 경로가 실행본을 정한다.
   PR #8의 상대 목표 계산 helper는 extreme-robot 쪽에는 없고, 그쪽 calibration pair jog는
   여전히 두 현재 위치에 같은 tick 증가량을 더한다. 정상 OPEN/CLOSE와는 별도 경로다.
4. PR #8 설명에 대응하는 Xbox X/B/A/Y GUI 코드는 기본 작업 폴더
   /home/jo/ros2_ws/power-train-sw/ros2/src/robot_manual_gui/에서 발견했다.
   git status는 이 디렉터리를 미추적으로 표시하고, PR head의 git ls-tree에는 없다.
   현재 통합 GTK 콘솔에도 해당 조이스틱 이벤트 경로는 없다.
   이것이 PR 당시 실제 실행본과 같은 파일인지까지는 확인하지 않았다.

## 이번 실행 증거

x86 extreme-robot-ros2, ROS_DOMAIN_ID=77. PR #8 런타임은 별도 /tmp/build-pr8에 설치했고,
앞서 설치한 powertrain_ros 브로커·관측 미러와 함께 실행했다.

```bash
source /opt/ros/humble/setup.bash
source /tmp/build-auto/install/setup.bash
export PYTHONPATH=/usr/lib/python3/dist-packages:$PYTHONPATH
mkdir -p /tmp/build-pr8
cd /tmp/build-pr8
colcon build --base-paths /ws/ros2/src/dynamixel_control --packages-select dynamixel_control
source install/setup.bash
python3 -m pytest /ws/ros2/src/dynamixel_control/test/test_dual_gripper_targets.py \
  /ws/ros2/src/dynamixel_control/test/test_dual_motor_gripper_fsm.py -q
```

- 설치 빌드 exit 0, 1 package finished. PR #8 관련 테스트 17 passed.
- 현재 vendored helper와 듀얼 FSM은 PR head와 diff 없음.
- 실제 GTK 버튼 → ConsoleOpsClient(격리 테스트 토큰) → 설치 ops_broker TCP :19002 →
  설치 PR #8 mock bridge → 설치 arm_console_bridge → UDP :15717 → GTK 경로 실행.
- MANUAL 요청 → 도구 토크ON → CLOSE `{3:-2211,4:1017}` → OPEN `{3:-675,4:2559}` →
  도구 STOP 후 ID3/4 실제 mock 토크OFF를 관측했다.
- 명령 발행 ACK가 아니라 수신된 tool FSM과 모터 위치·토크를 완료 근거로 사용했다.

## 확인된 연결 한계

- **좌/우 개별 버튼이 독립 제어가 아니다.** 좌측 닫기/열기를 클릭했는데 ID3/4 모두
  위 목표로 움직였다. ops broker는 params.target을 검사하지만 FSM 발행 시 target을 전달하지 않고
  도구 종류와 OPEN/CLOSE만 보낸다. 실물에서 한쪽만 움직일 것으로 기대하고 사용하면 안 된다.
- **동기 행 조작은 비활성이다.** ArmUiTelemetryBinding은 dual_sync capability를 보고하지 않아
  동기 행 열기/닫기·누름 조그를 사용할 수 없다. 별도 일반 열기/닫기 경로의 정상 동작과 구분한다.
- **팔 관절 제어는 미연결이다.** 현재 binding에 teleop_jog·teleop_pose capability가 없고
  앱의 등록 액션 집합에도 관절 jog/home/pose가 없다. USB 연결만으로 열리지 않는다.
- **PR #8 런타임에는 잠금 카드 입력/상태가 없다.** 해당 bridge에는 toolchanger가 없으므로
  coupling_lock 상태와 capability가 비어 있고 새 카드가 조작 가능해지지 않는다.
  잠금 기능은 extreme-robot 런타임의 별도 검증 결과이지 기본 배포 완료를 뜻하지 않는다.
- 새 팔의 모델/관절각 불일치와 자동 미션 그리퍼 액션 거절은 앞선
  2026-10-16-arm-pick-dock-readiness.md의 별도 차단 항목이다. PR #8로 해결됐다고 볼 수 없다.

## 실물 검증 전 필요한 연결 정리

현재 통합 콘솔에서 사용할 로봇팔 런타임을 하나로 정리하고, GUI의 개별/동기 버튼 의미를
실제 명령과 맞춘 뒤 검증해야 한다. 조이스틱을 사용하려면 로컬 미추적 구현의 보존·정식 반영 및
기존 인증 명령 경로 연결도 필요하다. 실제 모터 영점·방향·한계와 새 팔 도달성은 실측 대상이다.

호스트에 /dev/ttyUSB*, /dev/ttyACM*, /dev/input/js* 장치가 없었다.
기존 jetson SSH 이름도 해석되지 않아 원격 실기·실제 배포본을 점검하지 못했다.
물리 동작 성공이나 바로 꽂아 운용 가능하다는 주장은 하지 않는다.

원시 로그: 컨테이너 /tmp/pr8-runtime-{build,tests}.log,
호스트 /tmp/pr8-console-readiness.log 및 /tmp/pr8-console-{bridge,broker,mirror}.log.
실행 프로브: 호스트 /tmp/verify_pr8_console_path.py. 관련 제어 프로세스는 검증 후 종료했다.
사용자에게 열어둔 통합 콘솔은 별도의 TEST 프리뷰로 유지했다.
