# 로봇팔 신규 탭 골격 통합

## Claude 산출물 검토

- 기준 커밋: `241f83a` (`claude/arm-console-ui-skeleton`), 현재 브랜치에는
  `c3147ae`로 cherry-pick 됨.
- 신규 `operator_console/arm_ui/`와 테스트만 추가한 독립 UI 골격이다. ROS,
  소켓, CAN/USB, 기존 GUI 파일을 Claude 변경 자체에서는 수정하지 않았다.
- 실제 Xvfb GTK 환경에서 Claude 전용 테스트 `86 passed`를 재확인했다.

## 통합 범위

- 기존 상단 내비게이션에 `로봇팔 수동조작`, `로봇팔·도구 캘리브레이션` 탭을
  등록했다.
- `arm_ui_binding.py`는 기존 UDP 팔 스냅샷을 새 탭의 읽기 전용 뷰 모델로
  변환한다. 감지 도구, 활성 도구 actuator ID, 관절 각도, FSM/계약 상태만
  전달한다.
- 단일↔듀얼 도구가 바뀔 때 generation을 한 번만 증가시켜 이전 도구의 패널과
  진단값이 남지 않게 한다.
- 인증된 arm ops 명령 계약이 없으므로 `ArmUiCallbacks()`와 capability는 비어
  있다. 따라서 도구 변경, MANUAL 요청, 조그, 그리퍼, 캘리브레이션은 모두
  비활성이며 명령을 전송하지 않는다.

## 검증

- 신규 binding·팔 파서·bridge 순수 테스트: `56 passed, 4 skipped`
- 기존 콘솔 UDP + 신규 탭 GTK 통합 테스트: `7 passed`
- Claude 탭 UI 테스트와 통합 경계 검사: `95 passed`
- 전체 콘솔 GTK runtime smoke: PASS (arm 포함 5채널 LIVE→STALE, 듀얼 도구가
  새 탭에 표시되고 모든 arm 명령 버튼이 비활성인 것까지 확인)
- `git diff --check`: 통과

## 다음 연결 전제

명령을 활성화하려면 먼저 token-gated ops 채널의 arm 명령 계약, MANUAL 승인
확인, 최신값/만료/명시 stop을 팔 소유 경로와 합의해야 한다. UI에 callback만
선연결하거나 `/control/mode_status` 문자열만으로 조작 권한을 추정해서는 안 된다.
