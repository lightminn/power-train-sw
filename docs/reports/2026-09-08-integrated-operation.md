# 통합 운용 구현·검증 기록 — 2026-09-08

젯슨의 준비된 서비스 기동과 노트북 콘솔·패드·관측 연결을 하나의 운용 경로로 묶었다.
대상은 **CAN 4WS 선택 배포**다. 커밋·푸시, 실차 설치, 모터 조작은 수행하지 않았다.
사용 절차는 [통합 운용 안내](../integrated-operation.md)에 있다.

## 결과

- `scripts/robot_prepare.sh`: 최초 이미지/ROS 빌드, 설정·부팅 준비, 기존 송신자 전환.
- `scripts/robot-start`: 준비된 스택만 기동. 매일 빌드·캘리브레이션·노트북 IP 설정 없음.
- `python -m operator_console`: 등록 로봇 자동 연결, 별도 패드 프로세스 관리,
  현재 연결 상대에 맞춘 UDP 목적지, 세션별 영상 초기화, 시작 조건 표시.
- 운전 시작은 1.5초 명시적 홀드. 필요한 HOLD 해제 → 새 중립 입력 확인 → 수동 권한
  → arm을 순서대로 처리한다. ESTOP 초기화는 별도이며 결과 불명을 성공으로 바꾸지 않는다.
- 입력 채널 종료도 세션을 폐기한다. 새 입력은 신선한 IDLE 차체·안전한 권한·정지 바퀴가
  확인돼야 들어오며, 이전 주행 허가를 상속하지 않는다.

## 실행 근거

다음 수치는 서로 겹치는 스위트이므로 합산한 전체 테스트 수가 아니다.

| 검증 | 실제 결과·범위 |
|---|---|
| 콘솔 전체 | 시스템 Python, X11/Xvfb, **300 passed** |
| 세션·시작·TCP 통합 경로 최종 리뷰 재실행 | **45 passed** — 실제 소켓, 생산 OpsBrokerCore, ROS 실행 경계 fixture |
| 신규 통합 기능 묶음 | **86 passed** — 세션·운전 시작·실제 TCP/UDP/자식·런처 |
| 시작 절차 + 실제 서비스 엔트리포인트 | **22 passed** — 20 상태/명령 테스트 + TCP E2E + `robot_service` 실제 subprocess 기동/종료 |
| 기존 laptop·ROS 순수 회귀 | **176 passed, 4 skipped** — 호스트에서 ROS 환경 필요한 4건 별도 |
| 실제 ROS 컨테이너 | **57 passed** — clear-hold 어댑터, teleop/ops 노드, gateway/authority 회귀 |
| 기존 콘솔 실행 게이트 | **PASS** — 4 UDP 채널 LIVE→STALE, 잘못된 조향 상태 비활성, callback traceback 없음 |
| 새 통합 콘솔 실행 게이트 | **PASS** — 로봇/패드 없는 실제 GTK 대기, 중복 실행 거부, SIGINT 자식/잠금 정리 |
| Compose·셸 | 기본/선택 arm profile 렌더, daily 빌드 없음, `bash -n`, `git diff --check` 통과 |

호스트 명령의 Python은 환경에 맞춰 선택한다. GTK는 `gi`를 제공하는 인터프리터가 필요하다.

```bash
export PYTHONPATH="$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros"
python -m pytest powertrain_runtime/tests \
  operator_console/tests/test_operation_runtime.py \
  motor_control/laptop/tests/test_controller_process.py \
  scripts/tests/test_integrated_launchers.py -q
GDK_BACKEND=x11 xvfb-run -a /usr/bin/python3 -m pytest operator_console/tests -q
/usr/bin/python3 -m operator_console.runtime_smoke
/usr/bin/python3 -m operator_console.integrated_runtime_smoke
```

ROS 검증은 AMD64 `powertrain-sw:ros` 컨테이너에 저장소를 읽기 전용으로 마운트하고,
`/tmp`에서 메시지/패키지를 빌드했다. `--network none`, `ROS_DOMAIN_ID=77`,
`ROS_LOCALHOST_ONLY=1`을 사용했으며 장치·privileged 권한은 주지 않았다.
설치된 `control.launch.py`를 실행해 `127.0.0.1:19000/19001` LISTEN과 wildcard 부재를 확인했다.
종료 후 crash traceback 부재도 확인했다. 이 결과는 Jetson 실기동·CAN 드라이버 검증을 뜻하지 않는다.

## 리뷰에서 고친 실제 결함

- 중복 서버가 bind 실패 전에 기존 목적지 파일을 지움: 실패를 재현하고 포트 선점 뒤에만 기록 정리.
- HOLD의 `input_fresh=False`로 복구 시작 불가: HOLD 해제와 실제 주행 허용의 입력 조건 분리.
- 한쪽만 HOLD일 때 composite 거부: 이미 clear인 ROS 어댑터는 상태를 바꾸지 않는 성공 no-op.
- 빠른 입력 재연결이 이전 주행 허가 유지: 입력 종료 시 전체 임대 폐기, 새 입력 IDLE 확인.
- arm ACK 직후 잠시 오래된 상태를 즉시 거부: 다음 명령 없이 제한 시간 동안 최신 상태 대기.
- raw 입력 healthcheck가 연결/해제 이벤트 유발: 포트를 수동적으로 조회하도록 수정.
- 독립 송신자가 이미지의 카메라 healthcheck 상속: 송신자의 데이터 신선도를 별도로 관측.
- `set -u` 상태에서 ROS setup을 읽으면 선택 환경변수 미정의로 기동 실패:
  실제 ROS 이미지에서 `AMENT_TRACE_SETUP_FILES: unbound variable`을 재현하고,
  준비 스크립트·Compose 모두 setup을 읽은 뒤 `set -u`를 켜도록 수정. 실제 이미지의
  setup 로드 및 설치된 control 런치 기동·종료로 재검증.
- 원시 버튼을 놓지 않았는데 edge 프레임 이후 neutral로 표시: 실제 모든 버튼과 새 샘플 확인.

새 결함 회귀는 수정 전 실패를 실행으로 확인했다. 인증·임대 만료·홀드 취소 보호를
제거한 음성 대조도 실패했다. 공유 소스를 잠시 바꾼 GTK 음성 대조는 원복 후 전체
콘솔/실행 게이트를 재실행했다. 이후 검증은 원복된 소스 기준이다.

## 실차에서 남은 인수

이 세션에서 `jetson-orin.local` 이름 해석이 되지 않아 실차 체크아웃·설치 상태를
새로 확인하지 못했다. 실제 Jetson 이미지 빌드·systemd/Compose 배포, 실제 SRT decode,
PDIST/센서 단일 소유, 패드/햅틱, 모터 정지·ESTOP·전원사이클 캘리브레이션·stop_mm은
실차 인수 항목이다. 자동 캘리브레이션·안전 센서 OFF·자동 ESTOP reset으로 우회하지 않았다.
USB 스키드와 미완성 autonomy/로봇팔 원격 조작은 이 통합 프로필에 추가하지 않았다.

기존 미커밋 작업을 복사한 격리 작업 폴더에서 구현했다. 원래 파일의 기준 내용과
권한을 대조해 충돌이 없음을 확인한 뒤 이번 기능 변경 42개 파일을 원래 체크아웃에
반영했고, 관련 없는 기존 작업 34개 파일은 그대로 보존했다. 두 작업 폴더의 기능
파일이 동일함을 확인했으며 원래 체크아웃에서 통합 GTK 실행 게이트도 다시 통과했다.
기존 작업의 변경 내용을 이 기능의 구현량이나 검증 결과로 포함하지 않는다.
