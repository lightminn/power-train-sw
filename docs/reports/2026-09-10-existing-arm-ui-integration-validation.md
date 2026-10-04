# 기존 로봇팔 UI 통합 검증 (4단계)

## 확인한 경로

실제 `build_arm_telemetry_payload()` UDP datagram을 `OperatorConsole`의 실제
`LatestArmTelemetryReceiver` 포트로 전송하고, GTK 메인 루프를 실행했다.

| 시나리오 | 결과 |
| --- | --- |
| 단일 ID5 감지 | 기존 선택부에 `단일 그리퍼`, ID5 상태 표시 |
| 1초 초과 stale | 기존 상세 raw 값 제거 및 `정보 없음` 표시 |
| 재수신·단일→듀얼 교체 | `듀얼 그리퍼`, ID3/4만 표시, 이전 ID5 값 미잔존 |
| 잘못된/과대 UDP | 파서가 폐기하고 다음 정상 packet 수신 유지 |
| 기존 런타임 | `runtime_smoke`에서 arm 포함 5개 채널 LIVE→STALE 확인 |

## 실행 결과

- 순수 계약·수신기 테스트: `63 passed, 6 skipped`
- 실제 Xvfb GTK UDP 통합 테스트: `7 passed`
- bridge node 문법 검사 및 `git diff --check`: 통과
- `operator_console.runtime_smoke`: PASS (3단계 실행 결과)

## 실연결 보류

- `docker ps`에는 실행 중인 컨테이너가 없었다. 따라서 실제 ROS publisher,
  QoS, 상태 어휘 및 물리 도구 교체는 검증하지 못했다.
- `operator_console.integrated_runtime_smoke`는 시스템 Python에 pygame이 없어
  `no fresh real controller status`로 실패했다. 이는 팔 UDP/GTK 테스트와
  별개인 게임패드 자식 프로세스 사전조건이다. 관련 코드를 변경하지 않았다.

실기 세션에서는 bridge를 실행한 뒤 `/control/mode_status`, `/fsm/state`,
`/arm_status`, `/joint_states`, `/tool/status`의 publisher/QoS와 UI stale/
재연결을 동일 순서로 확인한다.
