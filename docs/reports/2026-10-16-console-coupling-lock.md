# 통합 콘솔 잠금 카드 구현·검증

PR #6 계열 통합 콘솔의 로봇팔 수동조작 탭에 기존 Qt 잠금 패널을 이식했다.
작업 브랜치는 feat/arm-ball-lock-urdf이며 기존 팔 작업과 같은 체크아웃에서 수정했다.

## 결과

도구 조작 카드 아래에 기존 보라색 스타일의 잠금 카드를 배치했다.
주 버튼은 결합·해제·정지, 준비 영역은 영점·회전 방향·잠금 모터 활성화다.
기존 인증 ops와 관측 UDP를 연결했고 로봇팔 측 게이트는 변경하지 않았다.
팔 이동을 포함한 자동 도크 교체 버튼을 추가한 것은 아니다.

## 실행 검증

- x86 extreme-robot-ros2에서 powertrain_msgs·powertrain_ros 설치 빌드: 2 packages finished.
- ROS 관련 새/기존 테스트 74 passed. 설치 브로커는 좁은 명령 어휘만 기존 토픽에 전달한다.
- 신규 GTK/계약/권한/지연 테스트 22 passed.
- powertrain_observability 66 passed.
- 실제 GTK 버튼 → ConsoleOpsClient(테스트 토큰) → 설치 ops_broker TCP :19001 →
  설치 mock 로봇팔 브리지 → 설치 arm_console_bridge → UDP :15707 → GTK 상태 갱신 통과.
  MANUAL 요청 → 결합 영점 2048 → 모터 활성화 → 역방향 해제 1792/UNLOCKED/CONFIRMED →
  정방향 재결합 2048/LOCKED/CONFIRMED → STOP 토크OFF를 확인했다. 미러 종료 후 상태가 사라지고 이동 버튼 차단·정지 요청 유지도 확인했다.
- 음성 대조: UI binding의 END_EFFECTOR_ONLY 검사를 고의 제거하면 FULL_ROBOT과
  TOOL_CHANGE에서 권한이 생겨 테스트가 실패했다. 복원 후 동일 6개 조합 모두 통과했다.
- 실제 전체 콘솔 runtime_smoke: 5채널 LIVE→STALE, 잠금 카드 LOCKED 표시와 이동 차단,
  callback traceback 없음으로 통과했다. 기존 스모크가 제어권 요청·재탐색까지 이동 버튼으로
  취급하던 관측 항목은 이동 관련 버튼만 확인하도록 구분했다. 제품 게이트를 완화하지 않았다.
- 1600×900 GTK 렌더를 확인하고 화면 첫 부분에서 결합·해제·정지 버튼이 보이도록 배치했다.
- 전체 콘솔: 699 passed / 9 failed / 6 skipped. 별도 원본 HEAD 체크아웃의
  668 passed / 10 failed / 7 skipped와 실패 이름을 대조해 신규 실패 0건을 확인했다.
  기존 스켈레톤 경계·낡은 계약 테스트 9건은 남아 있으며 전체 스위트 green은 아니다.
  원본에서만 실패한 키보드 transaction 테스트는 이번 회차에는 통과했다.

호스트에 Xvfb가 없어 Ubuntu 패키지를 /tmp/console-xvfb 아래에만 풀어 사용했다.
시스템 패키지·저장소 의존성은 변경하지 않았다. 실행 명령:

```bash
export PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros"
PATH="/tmp/console-xvfb/root/usr/bin:$PATH" /usr/bin/python3 -m operator_console.runtime_smoke
/usr/bin/python3 -m pytest operator_console/tests/arm_ui/test_coupling_lock.py -q
```

원시 로그는 /tmp/console-lock-{build,e2e,negative,smoke,final-tests,ros-tests}.log를 참조한다.
빌드·ROS 로그는 arm_auto_tool_change_check 컨테이너 /tmp에 있고 나머지는 호스트 /tmp다.
실제 실행한 E2E 프로브는 호스트 /tmp/verify_console_lock_e2e.py에 보관했다.
렌더는 /tmp/console-coupling-lock-1600.png다.

## 실기 범위

현재 열린 콘솔은 TEST 데이터이며 실제 장치·ops 서버에 연결하지 않았다.
실물 결합·하중·낙하 검증, 뒤쪽 도크 도달성, 자동 도크 교체 UI와 픽 미션 자동 복귀는
이번 GUI 작업에서 완료한 것으로 간주하지 않는다.

## 커밋·푸시 직전 재검증

관련 단위·관측 122 passed, ROS 브로커·계약·미러 74 passed, 전체 콘솔 runtime_smoke 통과.
실제 GTK 클릭의 영점→활성화→해제→재결합→STOP과 미러 단절 검증도 다시 통과했다.
자동 클릭 프로브는 상태 수신뿐 아니라 해당 버튼의 화면 갱신까지 기다린 뒤 클릭한다.
실기 준비 미완료 항목은 PR 본문에 별도 표시하며 Draft를 유지한다.
