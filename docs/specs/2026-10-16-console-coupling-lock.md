# 통합 운용 콘솔의 도구 결합 잠금 카드

사용자 요청: PR #6의 통합 운용 콘솔, 로봇팔 수동조작 탭에 별도 Qt GUI의 잠금 패널을
기존 디자인에 맞춰 추가한다. 기존 카드·배지·버튼·반응형 배치를 재사용한다.

## 범위

- 현재 도구 조작 카드 바로 아래에 도구 결합 잠금 카드를 표시한다.
- 결합/분리 위치, 영점, 모터 위치(tick), 진행 상태, 결합 추정, 토크와 오류를 표시한다.
- 도구 결합·해제·정지를 주 버튼으로 둔다. 영점·회전 방향·모터 준비는 펼침 영역에 둔다.
- 팔 이동을 포함한 자동 교체 시작/도크 teach UI는 이번 카드 범위에 포함하지 않는다.
  기존 도구 변경 요청도 기존 `/tool/change` 재탐색 의미를 유지한다.

## 데이터·명령 경계

관측: `/toolchanger/status` → 설치된 arm_console_bridge → 기존 arm UDP의 tool_runtime
내 coupling_lock/coupling_lock_age_s → 검증기 → ArmUiState → GTK 카드.
새 UDP 필드는 선택 사항이며 구버전 송신기에서는 미수신으로 표시한다.

명령: GTK callback → ArmCommandSession → 기존 token-gated ops → 설치된 ops_broker →
기존 `/toolchanger/command`. 카드가 ROS·모터·직접 소켓을 소유하지 않는다.
콘솔 역할만 robot_arm_lock_command / robot_arm_lock_stop을 사용할 수 있다.
허용 명령은 SET_ZERO(LOCKED/UNLOCKED), ENABLE, CHANGE(to 및 ±1 방향), STOP뿐이다.
임의 ID·레지스터·다점 궤적 입력은 없다. 브로커 ACK는 발행 결과이며, 완료는 수신 상태로만 표시한다.

## 허용 조건

명령은 신선한 상태(소스 나이+UDP 수신 나이 ≤1초), 인증 ops 연결, MANUAL 승인,
END_EFFECTOR_ONLY, 쓰기 가능, 비상정지 비활성이 모두 확인돼야 열린다.
잠금 기능이 보고되지 않으면 열지 않는다. 개발자 UI 모드도 이 조건을 건너뛰지 않는다.
영점은 잠금 토크OFF, 활성화는 토크OFF+영점 설정, CHANGE는 토크ON+위치 확정을 요구한다.
정지 요청은 상태·제어권을 잃어도 기존 인증 경로로 요청할 수 있다. 최종 read-only·하드웨어
검사는 기존 로봇팔 브리지가 계속 소유한다.

결합 센서가 없으며 정지 중 빠짐/낙하를 감지하지 못한다는 안내를 항상 표시한다.
기존 자동 교체 계획의 사용자 결정·실측 제한값·인터락은 바꾸지 않는다.
