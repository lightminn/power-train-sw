# 2단계: 기존 도구 UI 상태 연결

## 변경

- 기존 ROS mirror에 `/tool/status` read-only 구독을 추가했다. 명령 발행·서비스 호출·장치 접근은 추가하지 않았다.
- 신규 `powertrain_observability/tool_snapshot.py`가 타입·활성 ID·도구 피드백을 검증/축약한다.
- 기존 UDP v1에 선택 `tool_runtime` 객체를 추가했다. 원천 age를 보내며 4096 byte 한도를 유지한다.
- 신규 `operator_console/tool_binding.py`가 기존 선택기·상태 배지·상세 창을 갱신한다.
  감지 도구를 우선 선택하되 사용자가 다른 후보를 고르면 덮어쓰지 않는다. 선택은 물리 명령을 보내지 않는다.
- 숫자 별칭에 임의 기구 대응을 부여하지 않고 실제 타입을 `단일 그리퍼/듀얼 그리퍼`로 표시한다.
- 모터 감지를 기계 체결로 표현하지 않는다. mock 표시, 미수신, 원천 지연, 분리와 현재 도구 ID 필터를 적용했다.
- 현재 도구 피드백은 상세 창에 표시하고 전체 팔 부하로 합산하지 않는다. 기존 팔 부하 경로를 보존한다.
- 기존 화면 레이아웃, 파워트레인 제어, 영상 송신, ops 명령, Claude 담당 파일은 변경하지 않았다.

## 검증

- 기존 콘솔 전체: 367 passed / 6 skipped (GTK/Xvfb, pygame 없는 환경).
- 신규 도구 경로 + 기존 mirror: 36 passed / 4 skipped. 4개는 rclpy 없는 호스트의 ROS 테스트.
- 신규 테스트에서 실제 GTK 전체 콘솔을 띄워 순수 mirror 인코딩 → UDP → 기존 receiver → 기존 도구 화면/팝업 갱신 및 stale 시 값 제거를 검증했다.
- console.runtime_smoke PASS: 5채널 LIVE→STALE, 31 ticks, traceback 없음.
- integrated_runtime_smoke는 미통과. 초기에는 pygame 부재로 자식 상태가 없었다.
  임시 pygame 환경으로 재실행한 뒤에는 중복 기동 검증(92행)의 오류 문구 조건에서 실패했다.
  중복 프로세스 returncode=2이나 stderr가 비어 있었다. 파워트레인 런처를 수정하지 않고 한계로 남긴다.
- Xvfb와 pygame은 sudo 설치 권한이 없어 apt download + /tmp/arm-ui-xvfb,
  /tmp/arm-ui-pygame 패키지 추출로 검증 환경에만 사용했다. 시스템 설치는 하지 않았다.

실제 ROS/DDS→UDP 실기 검증은 아직 하지 않았다. 현재 로컬에 실행 중인 ROS 컨테이너가 없으며
Jetson 주소 확인도 필요하다. 본 결과는 실제 GTK/UDP 연결과 소스상 ROS 구독 연결을 증명하며
실제 로봇의 연결·체결·모션을 증명하지 않는다.

## 다음 단계

조종 모드와 velocity 생략 관절 피드백은 3단계 대상이다. 배포 시 원래 arm_console_bridge를
교체해야 하며 동일 :5007 송신기를 중복 기동하지 않는다. 도구 변경 명령은 후속 전용 탭에서 연결한다.
