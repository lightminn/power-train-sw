# Claude 인계, 2026-10-23

## 먼저 읽을 것

사용자가 토큰 절약을 요청해 현재 상태를 저장했다. 새 조사부터 시작하지 말고 아래 보고서와 실제 diff를 읽고 이어간다.

- 작업 정본: [도구 GUI 통합 결과와 검증](2026-10-23-arm-console-tools-integration.md)
- 안전 수정: [PR14 자세 안전](2026-10-23-pr14-local-pose-safety.md)
- 실행/Jetson 절차: [로컬 팔 콘솔 명세 §2.6~2.7](../specs/2026-10-17-local-arm-console.md)
- 검증 수치와 소스 SHA256: [summary.json](2026-10-23-arm-console-tools-evidence/summary.json)

## 사용자 지시, 반드시 유지

1. Auto Mode로 조사·구현·검증을 자율 진행한다. 반복 확인하지 않는다. 필수 시스템 승인은 우회하지 않는다.
2. PR12와 PR14를 검토했지만 **아직 머지 금지**. 이번 작업도 커밋·push·머지하지 않았다.
3. 안전 #1 자유 지정 해제의 토크 재활성화 검증, #2 예약 자세 이동과 볼락 동시 실행 방지는 구현했다.
4. **안전 #3 저장 자세 이동에 볼락 준비/잠금 게이트를 추가하지 말 것.** 사용자가 명시적으로 보류했다. 실제 동시 동작만 거부하며 기존 움직임을 불필요하게 끊지 않는다.
5. 현재 범위는 GUI 도구 자동 감지, 패드 제어, 개별/동기 제어의 누락 연결 해결. 개발 후 사용자가 Jetson을 연결해 실물 운용 검증할 예정이다.
6. 한국어로 짧게 보고한다. 불필요한 광범위 조사/반복 테스트를 늘리지 않는다.

## 저장소 상태

- 위치 `/home/jo/ros2_ws/power-train-sw`
- 브랜치 `feat/arm-bench-teleop-pose`, 작업 당시 HEAD `f72ef6c`.
- 변경은 **미커밋 로컬 작업**이며 여러 파일에 걸쳐 있다. `git status --short`로 확인한다.
- AGENTS.md와 .claude/CLAUDE.md의 Auto Mode 추가는 이전 사용자 요청에 따른 변경이다. 둘 다 보존한다.
- 미추적 `.claude/worktrees/`, `yolo26n.pt`는 이번 변경물이 아니다. 삭제하거나 일괄 add하지 않는다.
- 활성 브리지 소스는 `extreme-robot/ros2_ws/src/dynamixel_control`이다. 동명 `ros2/src/dynamixel_control`과 혼동하지 않는다.
- Jetson SSH 별칭이 해결되지 않아 현물에 접근하지 않았다. 사용자가 연결하기 전 반복 시도하지 않는다.

## 구현 위치와 핵심 계약

- 새 `tool_manual_control.py`: 도구 타입/세대/타깃/소스/순번/시각 검증, 최대 0.6초 lease, 수신측 만료 홀드. 토크를 유지하며 정지한다.
- `moveit_dynamixel_bridge.py`: manual_command 수신, 50ms watchdog, 실제 위치 홀드, FULL_ROBOT hot swap과 동일 종류 재연결 세대 갱신, 토크 요청 태그, 상태 capabilities. 앞선 자세 안전 수정도 포함한다.
- 단일/듀얼 FSM: 제한된 조그, HOLD, 좌/우 개별 및 동기 제어.
- `ops_broker_core.py`, `ops_broker_node.py`, `ops_contract.py`: 기존 인증 TCP 경유, hello 시각 오프셋으로 기한 변환, ROS 큐까지 남은 기한 유지. 새 요청으로 오래된 조그를 되살리지 않는다. 관측 코어에 GUI 의존성을 넣지 않았다.
- `controller_process.py`, `operation_runtime.py`: 기존 단일 pygame 자식 재사용. 팔 모드에서는 차체 중립, 주행 ARMED 중 팔 전환 거부, 주행 복귀 전 입력 전체 해제.
- 새 `operator_console/arm_pad.py`와 app 연결: 명시적 선택, 포커스/탭/수동 제어권/연결 확인, 교체·재접속 뒤 새 누름 요구. Xbox X/B/A/Y = 열기/닫기/소프트 정지/토크 해제. DualSense □/○/×/△. 알 수 없는 패드는 추측 매핑하지 않는다.
- GUI snapshot/binding/panels: 실제 세대·FSM·동기 상태·capabilities 반영. 고빈도 jog/ACK 로그로 전체 GUI를 다시 그리는 경로 억제.
- 키보드 속도 0~10단 연결, 5단 기존 스텝. 기존 서보 프로파일/가속도 유지.
- `config/local_arm_bench_teleop.yaml`: 자동 감지만 켰다. 기존 영점·자세·볼락 파일을 예제로 덮어쓰지 않는다.

## 검증 결과, 범위를 과장하지 말 것

- 설치 빌드 4개 패키지 성공.
- 실제 GTK → 인증 TCP → 설치 ROS 브리지 → 가상 모터 → 미러 UDP → GUI **36개 통과**. 실제 pygame 자식도 실행했고 장치 입력만 가상 Xbox로 대체했다.
- 관련 설치 ROS/안전/제어/브로커 회귀 **173 passed**.
- `operator_console.runtime_smoke` PASS, 5채널 LIVE/STALE, traceback 없음.
- 전체 콘솔·관측·패드·브로커 회귀 **880 passed / 4 failed / 3 skipped**. 전체 green이 아니다. 실패 ID와 이유는 통합 보고서 및 summary.json에 있다. 3개는 과거 UI 한정 VCS 경계 검사, 1개는 복구 요청까지 모두 비활성이라는 단언이다. 무작정 삭제하지 않는다.
- 세대/기한 가드를 메모리 복사본에서 제거하자 관련 테스트 3개 실패, 정상 소스는 보존했다.
- 마지막 `git diff --check` exit 0. oxlint 설정은 발견되지 않았다.
- 실물 모터·USB·실제 패드·Jetson·무선 지연 검증은 **미실시**.

## 재현 환경과 스크립트

- 빌드: `scripts/build_arm_console_runtime.sh <build-root>`.
- E2E: `scripts/verify_arm_console_tools_mock.py`, 격리 ROS_DOMAIN_ID=77, ROS_LOCALHOST_ONLY=1 필요.
- 임시 `/tmp/arm-qualification`에 설치 overlay와 로그, `/tmp/arm-console-toolchain`에 추출한 Xvfb/xauth/pygame가 있다. 시스템 전역 설치는 하지 않았다.
- ROS 이미지 `extreme-robot-ros2`로 `--network none --init`, 소스 `/ws:ro`, 결과 `/validation`을 사용했다. 실물 장치는 마운트하지 않았다.
- GTK 호스트 라이브러리/GI/폰트/XKB를 readonly 마운트했다. Xvfb는 host xkbcomp+XKB와 `--init`이 필요했다. 같은 환경 문제를 다시 디버깅하지 않는다.
- setuptools 혼용 방지 위해 `/usr/lib/python3/dist-packages`를 pip 패키지보다 먼저 두었다.
- 보존된 증거는 저장소 내 `2026-10-23-arm-console-tools-evidence/`. `/tmp/arm-qualification/full-tests.log`는 전체 원시 출력이다.

## 이어서 할 일

1. 사용자가 Jetson 연결/실물 검증을 요청하면 정본 명세 §2.6~2.7을 따라 먼저 원격 checkout과 미커밋 팀원 파일을 확인한다.
2. 기존 영점·자세·볼락·토큰을 보존한다. 브로커와 새 extreme-robot 브리지를 함께 빌드하고 올바른 overlay를 마지막에 source한다.
3. 실물 정지/기존 소유 프로세스 확인 후 수동 제어권 → 명시적 토크 활성화 → 패드 선택 → 버튼 전체 해제 → 새 누름 순서로 검증한다. 교체 후 자동 토크 ON은 하지 않는다.
4. 실제 교체 인식 시간, 패드 매핑, 좌/우/동기 끝점·방향, 장시간 조그와 단절 홀드, 축별 부드러움, FTDI latency_timer 1ms를 확인한다.

자동 감지는 ID5/3·4/2로 판단하며 장착 도구 모터만 응답하는 배선이 전제다. 0.5초 주기와 동일 관측 2회 확인이므로 순간 인식이나 기계적 결합 증명이 아니다. 자동 도크 실측 teach·결합 인수·자동 픽 미션 재개는 별도 남은 작업이다. 관절 키보드 조그는 기존 GUI/브리지 동일 호스트 계약이며 도구 패드의 다른 시계 처리는 가상 검증까지만 했다.
