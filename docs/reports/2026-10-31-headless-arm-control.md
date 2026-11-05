# GUI 없는 팔 제어 프로그램 검증

## 결과와 구조

`operator_console/arm_terminal.py`에 curses 터미널과 `ArmTerminalClient` Python API를 구현했다.
기반 커밋은 통합 PR #14 `1402d43`이다. 별도 `feat/arm-terminal` 작업 폴더에서 개발했다.
하드웨어/변환/제한 로직을 복제하지 않고 기존 ArmCommandSession, 인증 TCP, UDP 상태와
ROS 팔 브리지를 재사용한다. 클라이언트 import와 `--help`는 GTK/ROS/SDK 없이 실행된다.

기능은 1~5축 조그, 도구 조그/개별·동기 명령, 자세 저장·이동·삭제·home·자유 지정,
볼락 2점·방향 선택·활성화·결합/해제/정지와 슬롯 teach/교체다.
기존 속도/토크/영점/파일 형식은 변경하지 않았다. 생성자에서 토크나 MANUAL을 요청하지 않는다.

GTK 상태 계약을 재사용하도록 `arm_ui`의 위젯 import만 지연시켰다. 기존 공개 API는 유지한다.
터미널 수신기는 UDP 포트를 독점하며 다른 GUI와 같은 포트를 쓰면 시작을 거부한다.
명령 키 반복과 모터 갱신을 분리해 갱신은 0.2초, 입력 해제 판정은 0.15초다.
권한/도구/UDP 또는 인증 연결 상실 시 조그를 끝내고 새 누름을 요구한다.

## 검증

- 새 터미널·기존 상태 projection·UDP 수신 회귀: **60 passed**.
- GTK/ROS/SDK import를 금지한 별도 Python 프로세스의 import와 `--help` 통과.
- 1~5축·속도·제한/트립 전달, 축 전환 시 이전 축 STOP 우선, 0.15초 해제,
  반복 키의 지령 폭증 방지, 권한/인증 연결 상실 후 새 누름, stale 모션 거부/STOP 유지,
  종료 정지 ACK 거부 보고와 포트 충돌 거부를 확인했다.
- 음성 대조: 갱신 제한을 0으로 바꾼 임시 버전은 반복 속도 검사가 실패했다.
  GTK를 eager import하는 임시 버전은 headless import 검사가 실패했다.
  원본을 복원한 뒤 회귀를 재실행했다.
- 기존 팔 UI/세션/패드/binding: **341 passed / 기존 실패 4건**.
  이전 통합 검증과 실패 이름이 같으며 전체 green을 주장하지 않는다.
- 기존 전체 콘솔 runtime_smoke: **PASS**, 비전·지도 포함 7채널 LIVE/STALE, traceback 없음.
- 새 격리 컨테이너에서 설치 ROS 4개 패키지 빌드 통과.
- 설치된 브리지/브로커/미러 + 실제 TCP/UDP + headless API + 실제 curses PTY:
  **17개 경로 검사 통과**. 볼락 활성화·1792 해제·2048 재결합, 재탐색·명시적 도구 활성화,
  좌/우 개별·동기 동작, 도구 조그 해제 후 토크 유지, 입력 중 Ctrl+X 정지,
  SIGINT 종료와 termios 복원, 미러 단절 후 stale 거부/STOP 유지가 포함된다.
- `git diff --check` 통과. oxlint 설정은 발견되지 않았다.

검증 도구: `scripts/verify_arm_terminal_mock.py`.
원시 결과는 `/tmp/arm-terminal-evidence`, 설치 환경은 `arm-terminal-validation` 컨테이너에 있다.
GUI/카메라/CAN/실제 직렬 장치는 이 E2E에 연결하지 않았다.

## 범위와 남은 실기 확인

이번 실행 검증의 실제 모션 대상은 mock 팔 브리지의 볼락/도구다.
새 클라이언트의 1~5축·자세·슬롯은 같은 기존 명령 계약으로 검증했으며,
실제 팔을 연결해 이 터미널로 1~5축/자세/슬롯을 재시험한 것은 아니다.
기존 PR #14의 사용자 확인 실기 성공을 새 클라이언트의 실기 인수로 바꾸어 적지 않는다.
Jetson SSH 별칭은 이름 해석 실패였고 로봇 서비스나 세션 파일을 변경하지 않았다.

서버/역할 토큰/미러는 별도로 필요하다. ROS 없이 USB를 직접 소유하는 프로그램은 아니다.
확정 ACK는 명령 전달 결과이며 물리 완료는 상태로 확인한다. 기존 자세 거부의 상세 사유는
브리지 로그에 남는다. LOCKED/UNLOCKED는 엔코더 위치 판정이며 체결 센서 확인이 아니다.
기존 signed joint state/URDF 보정 문제는 이 변경으로 해결하지 않았다.

실행·키·Python 예제와 정지 의미는
`docs/specs/2026-10-31-headless-arm-control.md`를 따른다.
