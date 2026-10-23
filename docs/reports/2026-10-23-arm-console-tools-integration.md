# 통합 GUI 도구 자동 인식·패드·개별/동기 조작 연결

## 결과와 범위

자동 도구 인식, 현재 도구에 맞는 패드 조작, 듀얼 개별/동기 조작과 상태 표시를 연결했다.
설치된 ROS 엔트리포인트와 실제 GTK·인증 TCP·UDP를 사용한 가상 운용 36개가 통과했다.
실물·Jetson·무선 지연·실제 패드 GUID는 아직 재검증하지 않았다. PR 병합·push는 하지 않았다.

기존 키보드·저장 자세·자유 지정·볼락 2점 조작을 유지한다. 자세 이동에 볼락 준비 상태
게이트는 추가하지 않았다. 앞선 자유 지정/자세 예약과 볼락 동시 실행 방지 수정도 유지했다.
자동 도크 실측 teach·기계적 결합 인수·교체 후 자동 픽 미션 재개는 이번 수동 운용 연결 범위 밖이다.

## 구현

- 세션 예제의 자동 감지를 켰다. FULL_ROBOT 수동 벤치는 팔 FSM이 없어도 실제 팔 정지와
  예약 이동 없음을 확인하면 도구 전환을 진행한다. 기존 FSM이 작업 중이면 기다린다.
  같은 종류 재연결에도 FSM과 세대를 재구성한다. 전환 실패는 새 도구를 동작 가능 상태로 남기지 않는다.
- 모터 ID5 / ID3·4 / ID2로 단일 그리퍼 / 듀얼 그리퍼 / 클리너를 판별한다.
  장착 도구의 모터만 버스에 응답하는 배선이 전제이며 기계적 결합·슬롯 점유 센서는 없다.
  자동 감지는 0.5초 주기·동일 관측 2회 확인과 전환 상태를 거치므로 즉시 판정은 아니다.
- GUI의 도구 식별·세대·FSM·동기 상태를 실제 수신 상태에서 구성한다. 듀얼 좌/우는 각각
  ID3/4만 지령하고 동기 조작은 검증된 끝점 비율로 이동한다. 중간 정지는 토크를 유지한다.
- 기존 패드 자식 프로세스를 재사용한다. Xbox X/B/A/Y는 열기·닫기·소프트 정지·토크 해제다.
  DualSense는 □/○/×/△, 클리너는 열기/닫기를 좌/우 회전으로 바꾼다.
  GUI에는 명시적 도구 패드 선택과 연결 안내를 추가했다. 알려지지 않은 패드는 임의 매핑하지 않는다.
- 통합 운용에서는 주행을 해제해야 팔 패드를 선택할 수 있다. 팔 선택 중 차체 프레임은 중립이며,
  주행으로 돌아갈 때도 버튼·스틱을 모두 놓아야 한다. 별도의 pygame 모터 제어 경로는 만들지 않았다.
- 도구 교체·포커스 이탈·탭 전환·패드 단절 후에는 이전 누름을 끝내고 버튼 해제 후 새 누름만 받는다.
  도구 조그/정지 액션을 콘솔·서버 양쪽에 등록하고 모든 조작을 기존 인증 ops로 전달한다.
- 조그 기한은 최대 0.6초다. 기존 인증 hello의 클라이언트/서버 단조시각 오프셋으로 기한을
  변환하고, 브로커와 ROS 큐에서도 남은 기한을 줄인다. 이미 만료된 요청을 새 기한으로 되살리지 않는다.
  수신측 watchdog은 GUI의 stop 없이도 홀드한다. 다른 도구 세대나 이전 연결의 지연 명령은 거부한다.
- 제어권 반납 경로와 키보드 0~10단 속도를 연결했다. 5단은 기존 스텝, 서보 프로파일/가속도는 유지한다.
  정상 조그의 제출·ACK·결과 이벤트를 반복 기록해 GUI를 막는 경로도 억제했다.

## 검증 증거

[요약·소스 해시](2026-10-23-arm-console-tools-evidence/summary.json)
· [가상 운용 관측](2026-10-23-arm-console-tools-evidence/operation.json)
· [GUI 화면](2026-10-23-arm-console-tools-evidence/console.png)
· [설치 빌드](2026-10-23-arm-console-tools-evidence/build.log)
· [회귀 출력](2026-10-23-arm-console-tools-evidence/targeted-tests.log)
· [콘솔 스모크](2026-10-23-arm-console-tools-evidence/runtime-smoke.log)

- `scripts/build_arm_console_runtime.sh /validation` 실제 실행: 4개 패키지 설치 빌드 통과.
- `scripts/verify_arm_console_tools_mock.py`: 실제 GUI 버튼 → 인증 TCP → 설치 브리지 →
  가상 모터 → 설치 미러 → UDP → GUI 경로 36개 통과. pygame/자식 프로세스도 실행하되
  장치 입력만 가상 Xbox로 대체했다. 실제 USB·모터·패드를 열지 않았다.
- 좌/우 독립 끝점, 양쪽 동기 끝점, 누름 조그, 해제 토크 유지, 패드 단절·재접속 시
  held 입력 재실행 금지, Y 토크 해제, 단일·클리너 전환, A 정지, 제어권 반납을 확인했다.
- GUI의 stop이나 갱신 없이 단 한 번 보낸 조그도 설치 수신측의 기한 만료로 READY/토크 ON에 복귀했다.
- `operator_console.runtime_smoke`: 5채널 LIVE/STALE, 팔 제어 비활성 검사, traceback 없음으로 PASS.
- 설치 ROS·도구 FSM·감지·정책·자세 안전·브로커 관련 회귀 173 passed.
- 세대/기한 검사를 제거한 메모리 복사본은 관련 회귀 3개가 실패했다. 정상 소스는 변경하지 않았다.
- 전체 콘솔·관측·패드 자식·브로커 코어/계약 회귀는 **880 passed / 4 failed / 3 skipped**다.
  전체 green으로 보고하지 않는다. 실패는 아래 기존 UI 단독 단계의 단언에 남아 있다.

남은 전체 회귀 실패:

1. `test_arm_ops_boundary.py::test_protected_files_are_untouched_since_the_base_commit`
2. `test_arm_ops_boundary.py::test_changes_stay_inside_the_declared_new_paths`
3. `test_arm_ui_boundary.py::test_tab_package_does_not_expand_beyond_the_approved_console_integration`
4. `test_arm_ops_gtk_adapter.py::test_with_the_real_contract_gate_no_press_can_send_anything`

앞의 3개는 과거 UI 단독 변경범위와 현재 통합 브랜치 전체 차이를 비교한다.
마지막은 복구·제어권 요청까지 전부 비활성이어야 한다는 가정이다. 기능을 막거나
검사를 삭제해 green으로 만들지 않았다. 원시 전체 출력은 `/tmp/arm-qualification/full-tests.log`에 있다.

## Jetson 운용 검증

정본 절차: [기능 명세 §2.6~2.7](../specs/2026-10-17-local-arm-console.md).
기존 세션의 영점·자세·볼락 JSON과 토큰을 보존하고 빌드 overlay를 마지막에 source한다.
브로커와 `extreme-robot` 브리지를 한 세트로 배포하며 동명 구 브리지를 섞지 않는다.

수동 제어권 요청 → 도구 토크 활성화 → 도구 패드 선택 → 모든 버튼 해제 → 새 누름 순서다.
교체 후에도 자동으로 토크를 켜지 않는다. 파지 끝점·방향·실제 체결 상태는 현물과 대조한다.
FTDI latency_timer 1ms, 세 종류 실제 교체 인식 시간, 패드별 버튼 번호, 장시간 누름과
통신 단절 정지, 축별 부드러움은 Jetson 실측 항목이다.

팔 관절 키보드 조그는 기존과 같이 GUI와 브리지가 같은 호스트에서 실행되는 계약이다.
도구 패드 경로는 서로 다른 시계를 처리하도록 구현·가상 검증했지만 실제 무선 검증은 별도다.
