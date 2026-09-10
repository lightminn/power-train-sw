# 기존 로봇팔 상태 UI 연결 (3단계)

## 구현 결과

- 팔 UDP 미러는 읽기 전용으로 `/control/mode_status`, `/fsm/state`,
  `/arm_status`를 구독한다.
- 기존 작업 화면의 `조종 모드`는 control-mode 상태를, 기존 진단의 동작/FSM/
  도구 계약 영역은 각각 control mode, FSM state, arm status를 표시한다.
- `/joint_states`의 `velocity`가 빈 배열인 정상 ROS 메시지도 관절 위치를
  유지한다. `effort`는 변환 근거가 없으므로 `raw`라는 표기만 사용해 기존
  관절 부하와 상세 관절 진단에 표시한다.
- 원천별 수신 시각을 별도로 보존하고 1초가 지나면 `수신 대기`/`정보 없음`으로
  되돌린다. 이 경로는 ROS 명령, 서비스 호출, 모터 접근을 하지 않는다.

## 검증

- 순수 파서·미러·기존 팔 라벨 테스트: `56 passed, 4 skipped`
- GTK 도구 연결 회귀 테스트: `5 passed, 2 skipped`
- 콘솔 GTK 런타임 스모크: PASS (5개 채널에서 LIVE/STALE 전환 확인)
- `git diff --check` 통과

## 남은 실연결 검증

- ROS 실행 환경에서 세 상태 토픽의 실제 타입/QoS와 상태 어휘를 확인한다.
- 실제 `JointState.effort`가 어떤 원시 피드백인지 팔 소유팀과 확인한다.
  현재 UI는 단위 없는 raw 값을 토크/전류로 해석하지 않는다.
- 실물 수신, stale 전환, `/dynamixel/state` 미발행 경로는 4단계 통합 확인에서
  검증한다.
