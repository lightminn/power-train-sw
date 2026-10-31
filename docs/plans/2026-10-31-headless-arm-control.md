# GUI 없는 팔 제어 프로그램

기반은 통합 PR #14의 `1402d43`이다. 기존 키보드 텔레옵의 터미널 입력 방식을 참고하되,
새 제어 로직이나 USB 소유자를 만들지 않고 기존 팔 명령/정책/인증/피드백을 재사용한다.

1. GUI 없는 API와 curses 입력기를 구현한다. GTK/ROS/SDK 없는 import를 검증한다.
2. 키 반복과 0.2초 지령 주기를 분리한다. 해제·권한 상실·통신 단절·종료를 검사한다.
3. 설치 ROS + 인증 TCP + UDP + 실제 터미널 PTY 경로를 가상 장치로 실행한다.
4. 기존 GUI 스모크와 팔 회귀를 확인하고 실행 절차·미검증 범위를 기록한다.

구현은 `operator_console/arm_terminal.py`, 계약은
`docs/specs/2026-10-31-headless-arm-control.md`를 따른다.
