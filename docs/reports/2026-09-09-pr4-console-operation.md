# PR #4 GUI 보존과 통합 운용 검증

## 검증 시점과 병합 승인

수정안은 **코드 기준 병합 준비 완료** 판정을 받았다. 최신 main과의 결합, 최종 전체
시험, 실제 기동, 직접 화면 검수와 독립 코드 리뷰를 통과한 뒤 사용자가
2026-09-09 커밋·푸시·PR #4 병합을 승인했다. 최종 병합 커밋과 상태는
[PR #4](https://github.com/lightminn/power-train-sw/pull/4)에서 확인한다.
아래는 승인 요청 전 검증 기록이며, 당시 commit/push/merge 및 Jetson 배포는 실행하지 않았다. 병행 ODrive firmware 디버깅을
보존하기 위해 Jetson SSH·CAN·USB·서비스에는 접근하지 않았다. 초기 로컬 패드 시험의
격리 누락과 수정은 아래에 별도로 기록한다.

검토 기준은 main `b22cd9e1660c4d213647b80104123172b7fd4a2e`,
PR #4 `4ec2c5f87a95d302fe2ce5a2ae71aee2925dee50`이다.
별도 `fix/pr4-console-ready` 작업 트리에서 PR 위에 수정을 준비했다.
최종 확인 중 병행 작업의 로컬 main이 `7c41cd84e4adf87e166c621d782bf4d893d66ff8`로
진행한 것을 확인했다. 승인 요청 당시 원격 main/PR SHA는 위와 같고 OPEN·MERGEABLE·CLEAN이었다.
승인 후 원격 refs/heads/main을 다시 확인했으며 로컬에서 결합 검증한 `7c41cd8`과 같았다.
새 로컬 main의 별도 detached 작업 트리에 PR+수정 전체 patch를 충돌 없이 적용했다.
MKS 변경과 새 운용 문구를 모두 보존하며 이 결합 상태도 별도 검증했다. 최종 인식 데이터 수정 후에도 콘솔 전체·두 기동 검증이 통과했다. 실제 브랜치
병합·rebase·commit은 수행하지 않았다.

## main과 PR 비교 및 수정 방향

| 항목 | main과 PR의 차이/발견 | 보존·수정 방향 |
|---|---|---|
| 화면 구성 | PR은 실시간/시스템 두 탭, PiP, 도구·환경 팝업 중심 | PR 배치를 유지 |
| 복구 조작 | main의 ops 탭이 사라지고 PR에서는 기존 OpsPanel이 화면에 연결되지 않음 | 작은 `복구 · 설정` 버튼으로 동일 인증 패널을 비모달 창에 연결 |
| 종합 준비 상태 | main은 구동 결함을 반영하나 PR 새 종합 표시가 이를 누락 | 공통 구동 판정으로 stale/axis error/조향 fault/CAN·drive unavailable 반영 |
| 환경 예열 | PR 신규 SGP30 예열 플래그를 파싱하지만 정상 측정으로 표시 | eCO₂/TVOC 예열 표시, 정상 집계 제외; CO/LPG 표시는 PR 의도 보존 |
| 인식 데이터 신선도 | PR 새 상태 화면이 일반 1초 기준을 써 기존 상세 화면의 0.25초 기준보다 지연된 데이터를 정상으로 표시 | 상태 전용 0.25초 판정을 공유, overlay 0.50초는 유지 |
| 최근 이벤트 | PR 실시간 목록의 필터와 footer의 원본 목록이 다름 | 해당 화면의 필터 적용 결과와 일치 |
| 통합 화면 | 기존 운용 바와 PR dark 테마 결합 시 글자 대비와 최소 높이 문제 | 해당 영역만 dark 스타일, 작은 화면에서 sidebar 스크롤 |
| 영상 재연결 | 기존 수신 파이프라인을 재시작하면 실제 SRT가 not-linked로 복귀하지 않음 | 별도 실제 영상 회귀로 원인 재현 후 최소 수정 |
| 매일 기동 | 필수 서비스 중 카메라·전원만 on-failure restart 정책 | 통합 Compose에서 다른 필수 서비스와 unless-stopped로 통일 |

환경 수신 포트는 통합 설정의 `environment_telemetry_port`(기본 5008)로
검증한 후 실제 `OperatorConsole` 수신기로 전달한다. 임의의 SSH 기동이나 자동 arm은
추가하지 않는다. 콘솔 내부 수동 운용은 기존 인증 session/ops 게이트를 계속 사용한다.

## 확인한 증거

| 검증 | 실제 결과 | 증명 범위 |
|---|---|---|
| 수정 전 operator_console 기준선 | 326 passed | 기존 host 회귀 |
| 최종 전체 operator_console | **363 passed / 3 skipped** | 최신 main 결합, GTK/Xvfb, 실제 SRT/TCP 3개 회귀; pygame 필요 3개는 conda에서 모두 별도 통과 |
| 최신 main runtime + launcher + smoke helper + 직접 관련 CAN 회귀 | **135 passed** | 로컬 인증·TCP·종료, 병합 Compose 필수 8개 restart 정책, 패드 격리, CAN 조회/피드백/마찰 FF |
| controller_process/remote_operation_client | 38 passed | 입력 프레임·패드 경계·자식 프로세스 종료 |
| runtime_smoke | PASS | 실제 GTK, 31 ticks·5채널 LIVE→STALE, traceback 없음 |
| integrated_runtime_smoke | 최종 SDL 격리 수정 후 PASS | 실제 진입점·자식 상태, 무로봇 대기·중복 차단·SIGINT 자식/lock 정리 |
| missing-pygame 음성 대조 | 예상 exit 1 / FAIL | 잘못된 자식 인터프리터를 PID 존재만으로 통과시키지 않음 |
| 복구 창 음성 대조 | 예상 FAIL | 창 열기를 no-op으로 바꾸면 실제 GTK mapped 단언 실패 |
| 실제 GTK/TCP/SRT 운용 캡처 | PASS | 명시 reset→held start→stop→ESTOP→reset→세션 재연결, arm 누적 1회·IDLE |
| 직접 화면 검수 | 확인 | 실제 영상 ready/armed 1366×768, recovery 560×680, system 1366×768, 환경 정상6/예열2 |

새 테스트는 기존 결함을 먼저 실제 실패로 확인했다. 정상 fixture도 별도로 준비
완료를 표시하는지 확인하여 항상 실패하는 판정으로 바뀌지 않도록 했다.
GTK deprecation 경고와 호스트의 appmenu 모듈 부재 메시지는 기능 실패와 구분한다.

로컬 GTK→인증 session TCP→OpsBrokerCore 경로에서는 명시 초기화 → TELEOP 권한 →
arm → 주행 해제 → ESTOP → 명시 초기화 → 재연결을 실행했고 arm은 한 번만 발생했다.
영상은 실제 videotestsrc/x264/MPEG-TS/SRT 수신이며, 물리 패드와 하드웨어 executor만
fixture다. 재연결 후 새 영상 복귀를 추가 단언하자 아래 결함이 드러났다.

## SRT 재연결 원인 증거

동일한 VideoPanel에서 최초 영상 디코드는 LIVE였으나 `set_endpoint`로 재시작하면
`srtsrc: Internal data stream error / not-linked (-1)`로 반복 재시도했다. 실제 송신기의
`h264parse config-interval=-1`, `mpegtsmux alignment=7`, `srtsink async=false` 옵션을
사용해도 동일하며 송신 프로세스는 계속 실행 중이었다.

시험용으로 tsdemux의 새 pad를 h264parse sink에 다시 연결한 대조에서는 같은 재현이
즉시 LIVE/PLAYING으로 돌아왔다. 이 결과는 pipeline 재시작 후 dynamic pad 연결이
복원되지 않는다는 진단을 지지한다. 제품에는 demux/parser 이름과 지속 `pad-added` 콜백만 추가했다. 새 H264 pad를
미연결 parser sink에 연결하고 기존 파이프라인·GTK sink/widget을 유지한다. 재시작 시
이전 프레임 시각·FPS를 비우는 기존 동작도 유지한다. 콜백에서 GTK를 조작하지 않는다.

최종 회귀는 endpoint 변경, EOS가 일으킨 자동 재시도, 실제 GUI의 인증 세션 재연결
세 경로다. 모두 최초 실제 디코드 LIVE를 확인한 뒤 수정 전 `not-linked`로 실패했고,
수정 후 새로운 프레임·FPS와 LIVE 복귀를 확인했다. GUI 시험은 새 세션이 GTK의 영상
endpoint에 적용된 것을 기다린 후 LIVE를 단언하므로 이전 세션 프레임으로 통과하지 않는다.
`estop_reset → authority_manual → arm → disarm → authority_idle → estop →
estop_reset`을 실제 버튼과 TCP로 실행했고 재연결 후 arm 누적 1회·IDLE를 유지했다.
송신 프로세스 reap과 session/broker TCP listener 종료를 확인한 뒤 성공을 출력한다.
`os._exit`로 실패를 숨기거나 종료 검증을 우회하지 않는다.

## Task 2 추가 검증

실제 송신자의 `chassis_mode/stop_state`는 수동 ESTOP에서 `ESTOP/ESTOP`이며,
`safety_estop_required`는 US-100 판정이어서 동시에 False일 수 있다. 이 입력과
`wheel.mode=FAULT, drive_axis_error=0`에서 종합 준비 표시의 누락을 RED로 확인했다.
공유 표시 helper가 차체 ESTOP와 wheel FAULT를 확인하게 했으며, 수동 ESTOP는 안전
배지·비상정지 값에도 반영한다. 정상 `IDLE/RUN`, `ARMED/RUN` 대조는 정상으로 유지했다.
제어·안전 정책이나 MOTION_HOLD 분류는 바꾸지 않았다.

| Task 2 focused 검증 | 실제 결과 | 범위 |
|---|---|---|
| 실제 GTK/SRT 3개 + interactions/status_view + 기존 TCP 통합 2개 | 60 passed | system Python, Xvfb :153/X11/scale 1 |
| integrated launcher 파일 | 13 passed | conda, 병합 Compose 8개 restart 정책 포함 |
| 스모크 보완 focused | 최종 9 passed | conda, 자식 실상태/신선도·CLI interpreter·EOF 종료·SDL sentinel 2개 |
| 최초 병합 focused 실행 | 72 passed / 1 failed | 기존 installer 테스트에서 system Python의 pygame 부재; conda 재실행으로 해소 |
| diff whitespace | PASS | `git diff --check` |

새 통합 smoke는 `--controller-python`으로 GUI/패드 인터프리터를 분리한다.
단순 PID 존재나 supervisor가 만든 fallback 상태 대신 실제 자식의
`detail=waiting for gamepad`, `age_s<1`을 기다린다. 시험 래퍼는 pygame 열거를 0으로
고정하고 실제 joystick 열기를 금지하며 optional HID 출력 factory도 비활성화한다.
이 옵션은 smoke에만 있고 생산 가상 패드 옵션은 추가하지 않았다. 환경 UDP 포트도
임시 포트로 분리했다. 올바른 인터프리터의 실제 smoke는 PASS했고, pygame이 없는
인터프리터는 실제 자식 상태가 없다는 이유로 FAIL했다. 아래 SDL 격리 보완 후에도
동일 두 검증을 다시 실행해 PASS와 예상 exit 1/FAIL을 각각 확인했다.

시험 격리의 초기 누락: 첫 wrapper focused 1회에서는 pygame 입력 열거만 차단했다.
코드 확인 중 optional DualSense HID 출력 backend가 별도 초기화를 시도할 수 있음을
발견했고 이 호스트 conda에 pydualsense가 있음을 확인했다. 실제 장치 발견·출력 여부는
확인하지 않았으므로 그 1회에 대해 HID 접근이 전혀 없었다고 주장하지 않는다.
이후 래퍼에 출력 factory 차단을 추가하고 focused 7개를 다시 통과했다.
독립 리뷰는 `pygame.init()`과 `pygame.joystick.init()`의 SDL 장치 검색 가능성도
지적했다. 따라서 그 전의 conda wrapper 시험·실제 smoke에 대해서도 SDL 열거가
전혀 없었다고 인증하지 않는다. 실제 장치를 다시 조회하지 않고 시험 초기화 경계와
event 접근을 격리했다. 실제 초기화를 하지 않는 sentinel 회귀 2개가 수정 전
FAIL했고, 수정 후 전체 focused 9개와 실제 통합 smoke가 통과했다. 재리뷰도 승인됐다.

## 최초 준비와 매일 운용의 경계

최초 provisioning, 짝 토큰/robot ID, 검증된 STOP_MM, 이미지/ROS 준비 및
`scripts/robot-start` 한 번은 필요하다. 이후 정상 운용하던 필수 컨테이너가 재부팅 후
복귀하도록 기존 Compose 정책을 정렬한다. 정비자가 명시적으로 정지한 서비스는
자동으로 다시 켜지지 않는다. 정책 의미는
[Docker restart policy](https://docs.docker.com/engine/containers/start-containers-automatically/)를 따른다.

실제 Jetson 부팅/배포, 현장 무선 영상, 실제 패드 조작, 모터 구동·제동과 CAN 장시간
안정성은 이번 로컬 검증으로 인증하지 않는다. 해당 인수는 firmware 디버깅이 끝난 뒤
별도 수행해야 한다. 기존 하드웨어 진단 기록을 이 보고서로 덮어쓰지 않는다.

## 최종 검증과 병합 인계

전체 제품 시험·기동·직접 화면 검수는 위 표와 같다. Task 1 독립 리뷰에서 실제 CAN
`UNHEALTHY` 표시 누락을 수정한 뒤 스펙·품질 승인을 받았다. Task 2 제품 소스는
스펙에 적합하며 추가 제품 결함이 없다는 리뷰를 받았고, 시험용 SDL 초기화 격리도
보완 후 승인을 받았다. 최종 whole-branch 리뷰에서 인식 데이터 신선도 1건을 추가 발견했고, 기존 상세
화면의 0.25초 판정을 공유하도록 수정했다. RED 2 failed/2 passed → GREEN 4 passed,
관련 상태·상호작용 59 passed이며, 독립 재실행 4 passed와 재리뷰를 거쳐 코드 기준
병합 가능 판정을 받았다. 미해결 Critical/Important/Minor 지적은 없다.
최신 main 결합 상태의 마지막 콘솔 전체 363 passed/3 skipped(56.63초), runtime 등
135 passed(29.27초), 두 실제 smoke PASS를 확인했다. 스킵 3개는 pygame이 없는
system Python에 한정되며 conda 실행에 포함돼 모두 통과했다. GitHub의 PR/기준 SHA는
변하지 않았고 OPEN·MERGEABLE·CLEAN이며 원격 CI 검사 목록은 비어 있었다.
원본 main 작업 트리는 clean, SHA는 `7c41cd8`로 유지됐다. 두 준비 작업 트리는
`git diff --check`를 통과했으며 수정·로그·이미지를 별도 작업 트리에 보존했다.

주요 재현 명령(저장소 루트, 2026-09-09 검증 호스트의 GTK/패드 Python 경로):

```bash
export PYTHONPATH="$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros"
env -u WAYLAND_DISPLAY GDK_BACKEND=x11 GDK_SCALE=1 GDK_DPI_SCALE=1 \
  xvfb-run -a /usr/bin/python3 -m pytest operator_console -q
/usr/bin/python3 -m operator_console.runtime_smoke
/usr/bin/python3 -m operator_console.integrated_runtime_smoke \
  --controller-python /home/light/anaconda3/bin/python
```

실제 운용 E2E 테스트의 SRT test pattern은 카메라 실측이 아니며 패드 및 모터 실행은
fixture다. 영상·TCP 리소스 정리까지 검증하고 정상 종료했다. 결과 자료는 작업 트리의
`.superpowers/sdd/2026-09-09-pr4-console-operation/`에 보존한다.
검증 준비 단계에서는 commit/push/merge/deploy 및 Jetson 접근을 수행하지 않았다.
이후 승인 범위는 커밋·푸시·병합이며 Jetson 배포·실물 접근은 포함하지 않는다.
