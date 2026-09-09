# CAN 결함 수정·Jetson 대조·배포 — 2026-09-09

**확인된 애플리케이션 결함 수정과 SW 검증은 완료했다. 실물 CAN 안정성은 미통과다.** 최초에는 10개 모터가 모두 응답했지만 통합 기동 후 ODrive 13·14가 다시 소실됐다. 제어·워치독 정지와 CAN down/up 1회 후에도 최종 인식은 **8/10**이다. 이 현상의 원인을 이번 SW 수정으로 해결했다고 주장하지 않는다.

**원인 복기 추가:** 사용자 확인상 하드웨어 변경이 없었다. 제가 추가한 유휴 조회는 기존 0회/s에서 최초 통합본 600회/s, 보완본 재실행 126회/s로 바뀌었다. 보완본에서도 소실이 재발했다. SW 유발 가능성은 배제되지 않았으며, 프로세스 종료 후 지속된다는 이유로 하드웨어 고장에 귀속하지 않는다. [상세 복기·원인 분리 순서](2026-09-09-can-sw-causality-review.md).

사용자 요청 범위는 이전 [CAN 전수 감사](2026-09-09-can-software-interference-audit.md)의 실제 결함 수정, Jetson 코드 대조, 로컬·GitHub·Jetson 통합 운용 정본 동기화다. 미완성 자율주행, US-100·L515 의도적 분리의 센서 결함 판정, 실물 비영점 주행·제동은 인수에서 제외했다. ODrive NVM 저장·캘리브레이션·펌웨어 플래시는 수행하지 않았다.

## 1. 가장 최근 실물 결과

각 측정은 새 SocketCAN 소켓으로 외부 수신 프레임만 6초간 관찰한다. 측정 스크립트는 CAN 송신 API를 호출하지 않는다. 원시 자료: [수신 전용 측정 스크립트](2026-09-09-can-remediation-evidence/recognize_can.py).

| 조건 | 수신 | 구동 상태 | 링크 상태 |
|---|---|---|---|
| 수정본 배포 전, 제어 프로세스 없음 | AK 1–4, ODrive 11–16 각각 300개 | 6축 IDLE(1), axis_error 0 | 500000, ERROR-ACTIVE, TX/RX 오류 0 |
| 배포·통합 기동 직후 실제 콘솔 관측 | 차체/전원 LIVE, 실제 정지 proof 수신 | wheels_stopped=true, US-100 미응답 ESTOP | 오류 0 |
| 기동 후 재측정 | AK 1–4, ODrive 11/12/15/16 각각 300개; **13/14 0개** | 수신 4축 IDLE, 오류 0; 누락 2축 미확인 | ERROR-ACTIVE, 오류·bus-off·자동 restart 0 |
| session/control/chassis/watchdog 정지 후 | **동일 8/10** | 수신 4축 IDLE, 오류 0 | 오류 0, TX 증가 없음 |
| 공유 maintenance lock으로 수동 down/up 1회 | **동일 8/10** | 수신 4축 IDLE, 오류 0 | reset generation 1, 링크 정상 |
| 이후 writer 정지를 유지한 최종 수신 재측정 | **동일 8/10** | 수신 4축 IDLE, 오류 0 | ERROR-ACTIVE, 오류 0 |

원본 JSON: [배포 전](2026-09-09-can-remediation-evidence/physical-can-before.json), [기동 후](2026-09-09-can-remediation-evidence/physical-can-after.json), [writer 정지 후](2026-09-09-can-remediation-evidence/physical-can-writers-stopped.json), [수동 CAN reset 후](2026-09-09-can-remediation-evidence/physical-can-after-manual-reset.json), [최종 재측정](2026-09-09-can-remediation-evidence/physical-can-final.json). JSON의 `all_motor_errors_zero`는 **수신된 노드에 한한 계산**이다. 누락된 13/14의 오류가 0이라는 뜻이 아니다.

기동 후 수신 중인 4축은 축당 encoder 100개·Iq 25개/6초, 실제 속도 0이었다. 버스 전체 TX는 같은 6초에 768프레임 증가했다. 조회 트래픽만으로 버스 포화나 펌웨어 원인을 확정할 수 없다. 13/14 소실의 정확한 첫 프레임 시각을 연속 기록한 것은 아니므로 기동과 소실의 선후관계를 인과관계로 승격하지 않는다. 사용자 운전 시작·비영점 명령은 실물에 보내지 않았다.

**최종 상태:** motor writer인 `powertrain_session`, `powertrain_control`, `powertrain_chassis`, `powertrain_canwatchdog`는 정지해 두었다. 관측용 Gateway·전원·차체 UDP·observability는 유지한다. 최신 코드는 배포되어 있으며 다음 기동 명령은 `~/power-train-sw-integrated/scripts/robot-start`다. 현재 하드웨어 상태에서 주행 가능 판정을 내리지는 않는다.

ODrive USB 장치는 Jetson USB 목록에 없었다. 따라서 실제 보드 firmware·NVM 통신 설정·CAN 내부 오류를 USB로 읽지 못했다. 13/14 보드를 전원 재인가하기 전에 USB를 연결해 RAM 상태를 확보하는 것이 다음 원인 분리 단계다. 이전 감사 20번의 펌웨어 복구 분기 후보는 여전히 **조건부 후보**이며 임의 패치·플래시 대상이 아니다.

## 2. Jetson 코드 전수 대조

| 체크아웃 | 확인 결과 | 처리 |
|---|---|---|
| `/home/zetin/power-train-sw` | 팀원 `feat/l515-aligned-depth-slam`, HEAD `310163a`, 미커밋·미추적 작업 존재 | 원본·브랜치·설정 보존 |
| `/home/zetin/power-train-sw-integrated` | 기존 운용 배포본은 Git 메타데이터 없는 별도 디렉터리 | 이번 수정의 배포·동기화 정본 |

소스/설정/문서 인벤토리는 팀원 866개, 통합본 853개였다. 로컬 대응 파일은 통합본 **737개 동일·상이 0개**, 팀원 본은 669개 동일·76개 상이였다. 나머지는 설치 산출물이나 Jetson 전용 파일이다. Python AST는 팀원 소스 507개+설치 67개, 통합 소스 535개+설치 67개에서 구문 오류가 없었다. 두 체크아웃 모두 기존 설치된 ROS 모듈 48개는 각자의 소스와 바이트가 같았다. 로컬 최종 소스 인벤토리의 Python 562개도 AST 통과했다. 동일 깊이로 모든 문장을 수동 리뷰했다는 뜻은 아니다.

팀원 본의 추가 `scratchpad_ak_{mit,probe,scan}.py`는 raw CAN으로 명령을 보내며 공통 owner lock을 사용하지 않는다. 특히 probe/scan도 duty/RPM 0을 보내는 **writer**다. `scripts/keyboard_teleop.py`, `scripts/steer_test.py`도 연속 `/cmd_vel` 실험 경로다. 통합 운용본에 복사하거나 실행하지 않았다. 프로젝트 지침에 따라 팀원 미커밋 파일은 수정·삭제하지 않았으며, 이들 보존된 레거시 도구까지 안전한 운용 진입점으로 인증하지 않는다. 프로젝트 lock은 비협조적인 외부 프로그램이나 수동 odrivetool까지 커널 차원에서 금지하지 않는다.

## 3. 감사 항목별 수정

| 이전 번호 | 변경 |
|---|---|
| 01, 03, 04 | `chassis/can_interface.py` 공통 parser: IFF_UP, 정확한 500000, CAN ctrlmode LOOPBACK/LISTEN-ONLY off, BUS-OFF 거부. 정상 UNKNOWN 허용. 셸 준비·watchdog·preflight·두 launcher 연결. |
| 02 | 영구 owner/reset flock 및 reset generation으로 동일 관측의 중복 reset 방지. active owner가 같은 제어 executor에서 ESTOP을 래치한 뒤 복구. 자동 재무장 없음. 미완료 down/up만 해당 세대에서 재시도. |
| 05 | USB 배포의 실제 `powertrain_canwatchdog`와 integrated session 정지 반영. |
| 06 | USB GUI·정본 도구·실물 USB pool도 CAN과 같은 모터 owner lock을 탐색 전에 확보. |
| 07 | 쓰기에 serial·axis·node 필수. sibling node 충돌 방지. 저장 직전과 동일 serial 재연결 후 firmware·양축 node/extended/heartbeat/rate·baud/protocol 대조. 실제 저장은 미실행. |
| 08 | 폐기 X2212/Pi/BL 실험·이름 없는 USB shortcut은 하드웨어 import/연결 전에 하드스톱. 역사 소스 본문 보존. |
| 09 | 유휴 encoder 최대 20 Hz/축, Iq 최대 5 Hz/축. 정지 feedback 유지. GUI legacy RTR 15 Hz×3 지원 조회로 제한. |
| 10 | GUI old-ID stop ACK → close → ID 변경 → connect. 대기 명령에 대상 generation 고정하여 예전 arm/속도/프로파일이 새 대상에 전달되지 않게 함. |
| 11 | 중요 CAN 송신 실패 전파. 6축 공통 150 ms 대기에서 post-request state8/error0 및 fresh encoder/heartbeat를 모두 확인해야 arm 성공. stop 실패가 다른 축 정지를 막지 않음. |
| 12 | AK 송신 실패를 GUI 실패 ACK로 전달, 실패한 active resend/armed 해제. |
| 13, 14 | 실제 수신 timestamp와 encoder/heartbeat 별도 age 사용. out-of-order/nonfinite 거부. encoder 상실 또는 state8 이탈 시 FAULT/ESTOP. |
| 15, 16 | 동일 GUI CAN 소켓은 단일 RX dispatcher. `0x15` sensorless 값을 온도로 해석/조회하던 경로와 화면 신호 제거. |
| 17 | 수동 Twist depth 1. callback 실행 시각 대신 DDS receipt age를 authority/차체에 전달. Humble용 `ReceiptTimeExecutor`가 RMW metadata를 보존. 누락·미래·300 ms 초과는 거부. |
| 18 | 외부 `/wheel_states`로 내부 정지 판정 금지. owner의 실제 6축 backend/상태/피드백 age에서 proof 생산. broker는 그 proof만 소비하고 FAKE·누락·잘못된 age/노드 집합을 거부. |
| 19 | transient accept 오류 재시도, fatal listener와 worker.start 실패 감독·정리. ops listener도 fatal 시 프로세스 재시작 경로 연결. |
| 20 | 펌웨어 후보 유지. 실제 보드 버전·오류 미확인 상태에서 패치하지 않음. |

추가로 prepare의 active owner 거부, legacy/integrated 구성 역전 거부, 최초 runtime-directory 설치 순서, USB IDLE 피드백 누락, USB 느린 순차 읽기 나이 보존을 수정했다. 최신 USB 읽기는 완료 시각으로 오래된 속도를 age 0으로 만들지 않고 **읽기 시작 시각**을 보존한다. USB 6축 RTT의 실물 상한은 별도 검증 대상이다.

독립 검토에서 지적한 네 건(최초 설치 순서, worker.start 실패, GUI 대기 명령의 새 대상 전이, 느린 USB 정지 증거)은 각각 실패 재현 후 수정했다. 단순 의견 수용 대신 생산 경로/원시 로그로 확인했다.

## 4. 실제 실행에서 추가 발견한 환경 문제

- Jetson `rclpy 3.3.21`의 기본 Humble executor는 `take_message()`의 metadata를 버리고 callback에 메시지만 전달했다. 최초 설치 E2E에서 TypeError로 재현했다. 차체 전용 executor가 명시적으로 지정한 motion callback에만 기존 RMW metadata를 보존한다. 나머지 ROS callback 처리는 그대로 위임한다. 실제 DDS backlog 450 ms는 거부되고 새 메시지는 통과했다.
- ROS 이미지에는 `ip`가 없어 `CanBusStatsSampler`의 실제 CAN 상태 진단과 진단 CLI 실행이 실패했다. `docker/Dockerfile.ros`에 iproute2를 추가하고 실제 배포 이미지에도 설치했다. **watchdog reset 자체는 기존처럼 ioctl이며 ip 바이너리를 필요로 하지 않는다.**
- Jetson 외부 DNS가 실패하여 직접 apt 빌드는 실패했다. 노트북에서 Ubuntu 공식 HTTPS 저장소의 arm64 패키지를 받아 index의 SHA256과 대조한 뒤 Jetson에 전달하여 기존 ROS 이미지 위에 설치했다. [패키지 출처·버전·해시](2026-09-09-can-remediation-evidence/ubuntu-can-tools-manifest.json), [성공 빌드 로그](2026-09-09-can-remediation-evidence/jetson-image-can-tools-offline.log).
- Jetson 벽시계는 작업 중 2026-08-18, 로컬 기록일은 2026-09-09였다. 복사된 미래 mtime 때문에 첫 setuptools 증분 빌드가 오래된 파일을 유지했다. 기존 build/install을 보존한 뒤 깨끗한 공간에서 재빌드하고 소스/설치 바이트를 직접 대조했다. 최종 통합 경로에서도 3개 패키지를 재빌드했다. 설치 setup 파일의 존재나 colcon exit 0만 최신성 증거로 삼지 않는다. 서로 다른 기기의 벽시계로 로그의 절대 시각을 정렬하지 않았다.

## 5. 검증 결과

수치는 선택이 겹칠 수 있어 합산하지 않는다. 원시 `.log`의 줄 끝 공백은 보존했고, 코드·문서의 staged diff 공백 검사는 통과했다. 실행 코드와 원시 로그는 [증거 디렉터리](2026-09-09-can-remediation-evidence/)에 있다.

| 검증 | 결과·증거 |
|---|---|
| 호스트 원격 운용 선택 | **1,264 passed**, motor_control/GUI/runtime/scripts/observability/video. [로그](2026-09-09-can-remediation-evidence/host-remote-final.log) |
| 실제 GTK 콘솔 스위트 | Xvfb에서 **300 passed**. [로그](2026-09-09-can-remediation-evidence/console-tests-xvfb-final.log) |
| 콘솔 실행 게이트 | 4채널 LIVE→STALE, 무 traceback PASS; 통합 엔트리포인트·중복 거부·자식 종료 PASS |
| Jetson ROS 원격 선택 46파일+runtime | **621 passed**, 자율·L515 기능 인수 제외. 별도 fresh interpreter 설치본 import **2 passed**. [로그](2026-09-09-can-remediation-evidence/jetson-ros-final.log) |
| 실제 DDS receipt 시험 | **2 passed**: metadata 부재 시 미실행, executor 지연 450 ms 거부·새 메시지 수용 |
| 설치 ROS+vcan77 전체 루프 | PASS: 인증 → 중립 → 1.5초 hold → 실제 driver state8 → 가상 속도 → 노트북 UDP → 입력 단절 → IDLE/0 → 재접속 후 무장 없음. DDS receipt 40샘플 중 invalid 0. [결과](2026-09-09-can-remediation-evidence/loop-result.json) |
| 음성 대조 | FAKE 정지 proof: 입력 채널·held Start 거부. 비중립 입력: held Start 거부. 둘 다 IDLE 유지 |
| GUI 실제 HTTP/WS 서버 | 실제 FastAPI/uvicorn/worker + FakeTransport 11단언 PASS, 정상 종료·무 traceback |
| 실제 배포 콘솔 | 인증, 실제 패드 중립 인식, 차체·전원 LIVE, L515 LIVE 관측. Start 미호출, 정상 종료. US-100 ESTOP 및 차단 표시 유지. [로그](2026-09-09-can-remediation-evidence/production-console.log) |
| 배포 설치본 대조 | ROS 모듈 **50/50 바이트 동일**, 차이 0 |
| `scripts/robot-start` | 8개 컨테이너 기동, healthcheck 있는 5개 모두 healthy, 첫 관측까지 crash/restart 없음. 이후 CAN 재소실 조사로 motor writer 4개 정지 |
| 실제 모터/CAN | 최초 10/10 → 후속 8/10, **안정성 FAIL** |

테스트 환경 문제도 별도로 고쳤다. 실제 패드가 꽂힌 머신에서 “패드 없음” subprocess fixture는 pygame 장치 열거 경계만 고정한다. 전체 GUI 스위트는 실제 Xvfb를 쓴다. ROS 서비스 부재 시험에는 다른 테스트의 DDS discovery 잔류가 섞이지 않도록 고유한 미존재 endpoint를 쓴다. US-100 OFF는 다른 안전 hold를 지우지 않는다는 단언을 강화했다. 셸 CAN 테스트는 옛 sudo 문자열 대신 실제 셸→공통 CLI를 실행하고 외부 명령만 대체한다. skip/xfail로 실패를 숨기지 않았다.

## 6. 배포·동기화와 남은 범위

배포 정본은 로컬/GitHub `main` 및 Jetson `~/power-train-sw-integrated`다. 팀원 `~/power-train-sw`는 독립 상태로 보존한다. 기존 배포 소스/설치 공간은 `~/powertrain-deploy-backups/2026-09-09-can-remediation/`에 보존했고 이전 ROS 이미지는 `powertrain-sw:ros-before-can-20260909`로 남겼다. 실제 토큰·현장 설정·보드 registry는 Git에 넣지 않는다.

초기 소스 비교·검증 후에도 모든 원본 로컬 파일의 해시를 다시 대조해 이 작업 밖의 동시 변경이 없음을 확인했다. 검증한 변경만 파일별로 stage하여 커밋·푸시했다. 수정 코드 `b9292af`는 로컬/GitHub/Jetson 통합본 HEAD가 같고 추적 파일 980개 바이트 차이가 0임을 확인했다. [동기화 증거](2026-09-09-can-remediation-evidence/source-synchronization.json). Jetson 외부 DNS 장애를 피하기 위해, GitHub push를 확인한 같은 `main`의 Git 메타데이터/소스를 SSH로 전달했다. Jetson은 해당 commit부터 시작하는 shallow `main`이며 origin은 GitHub다. 이후 원인 복기 문서·증거 보충도 같은 세 곳에 반영한다.

남은 것은 **13/14 보드 CAN 재소실 원인과 실물 안정성**, 실제 USB/NVM·펌웨어 상태 확인, USB RTT, 실물 주행·제동 인수다. 이는 SW 회귀/가상 CAN PASS로 대체되지 않는다. 현재 센서의 의도적 분리와 보드 무응답을 혼합해 판정하지 않는다.
