# CAN 통신 간섭 가능 경로 전수 조사 — 2026-09-09

> 후속 수정·Jetson 대조 및 재검증: [CAN 수정·배포 기록](2026-09-09-can-remediation.md).
> 아래 내용은 수정 전 감사 시점의 증거와 판단을 보존한 기록이다.

**결론: CAN을 재설정하거나 ODrive에 경쟁 명령을 보내는 경로와, 통신 실패를 정상으로 표시하는 결함을 확인했다. 그러나 최근 node 11–14의 지속적인 무응답 원인은 확정하지 못했다.** 프로세스를 모두 끈 뒤에도 유지되는 현상은 실행 중인 콘솔·ROS·조회 트래픽만으로 설명할 수 없다. 보드의 NVM 설정과 펌웨어 오류 상태는 별도 후보로 남는다.

이번 작업은 사용자 요청에 따른 조사다. 생산 코드·설정, Jetson, 모터 상태는 변경하지 않았다. 추가한 것은 이 보고서와 호스트 전용 재현 자료다. 커밋·푸시는 하지 않았다.

## 1. 범위와 증거 수준

- 대상: 현재 로컬 작업 트리. 기준 HEAD `c3fd078`이며 기존 미커밋 통합 운용 변경을 포함한다. 원격 Jetson의 설치본과 같다고 가정하지 않는다.
- 파일 목록 894개에서 생산 코드·설정 **301개 전체 검색**, Python **259개 AST 파싱**, CAN·ODrive·소유권 관련 **76개 파일과 호출 경로 집중 검토**를 했다. 301개 모두를 동일한 깊이로 행별 검토했다는 뜻은 아니다.
- 기동·Compose·systemd·워치독, 순수 모터 제어, CAN/USB 정비 도구, 모터 GUI, ROS·세션·운용 콘솔 경로를 분담 검토한 뒤 핵심 결론을 실제 코드와 재현으로 대조했다.
- `powertrain_autonomy`는 직접 CAN 접근·명령 발행 연결만 정적으로 확인했다. 자율주행 기능은 사용자 지시대로 인수 테스트에서 제외했다. `powertrain_sim`·`parameter_calc`는 읽기 전용 레거시로 취급했다.
- US-100·L515는 의도적 분리 상태이며 센서 미연결 자체를 CAN 결함으로 판정하지 않았다.
- Jetson은 사용 불가하여 접속·CAN 실측·설치 엔트리포인트 실행을 하지 않았다. 아래 재현은 **가짜 버스·가짜 USB 보드·대체 외부 명령·추출한 순수 메서드**를 이용한다. 실제 CAN 소켓, USB, 모터, Docker 서비스는 시작하지 않는다.
- [파일 인벤토리](2026-09-09-can-software-audit/inventory.json)와 [소스 해시](2026-09-09-can-software-audit/source-sha256.json)를 첨부한다. 인벤토리 수치는 보고서 생성 전 조사 시점의 스냅샷이다.

| 영역 | 검색한 생산 코드·설정 파일 |
|---|---:|
| motor_control | 99 |
| ros2 | 59 |
| scripts | 40 |
| l515_dashboard | 21 |
| operator_console | 20 |
| motor_gui | 18 |
| powertrain_autonomy | 14 |
| powertrain_observability | 9 |
| powertrain_runtime / docker | 각 8 |
| remote_video / tools | 3 / 2 |

## 2. 이번 무응답 현상과 대조

이전 대화의 마지막 측정에서는 AK 1–4와 ODrive 15/16은 계속 수신됐고, 11–14는 수신되지 않았다. Jetson만 재부팅한 뒤와 CAN down/up 20회 및 추가 1회에서도 같은 노드 구성이 관찰됐다. 당시 Jetson CAN 상태는 ERROR-ACTIVE, loopback off, 오류 카운터 0으로 보고됐다. **이는 이전 측정 기록이며 이번 조사에서 재측정한 값이 아니다.** 당시 `/tmp` 원시 로그는 현재 세션에서 다시 조회할 수 없다.

따라서 다음을 구분해야 한다.

| 설명 대상 | 소프트웨어 조사 결과 |
|---|---|
| 운용 중 잠깐 끊김·갑작스러운 정지·재연결 실패 | 중복 리셋, USB 경쟁 제어, GUI 재연결, 세션 리스너 종료가 후보다. 실행 조건 확인이 필요하다. |
| 응답 일부 누락·가짜 정상 표시 | 같은 GUI 소켓의 수신 경쟁, 송신 예외 은폐, 오래된 패킷의 나이 초기화를 재현했다. |
| 모든 제어 프로세스 종료 후에도 특정 보드의 양축이 지속적으로 무응답 | 위 프로세스 내부 결함만으로는 설명되지 않는다. 보드 NVM 변경 또는 보드 펌웨어의 복구 누락은 조건부 후보이며 하드웨어 원인도 배제되지 않는다. |
| Jetson의 tx/rx 오류 카운터가 0 | 모든 ODrive가 정상이라는 증거가 아니다. 정상인 다른 노드가 버스에 남아 있는 상황과 양립한다. |

## 3. 직접 간섭하거나 보드 설정을 바꾸는 경로

우선순위 P1은 먼저 막아야 할 운용 결함, P2는 조건부 간섭·진단 개선 항목을 뜻한다. 실제 사고 발생이나 이번 원인 확정을 뜻하지 않는다. “신규”는 이번 통합 운용 작업 트리에서 추가된 경로, “기존”은 HEAD에도 있던 동작이다.

| 번호 | 우선 / 구분 | 발견 내용·발생 조건 | 근거 |
|---|---|---|---|
| 01 | P1 / 신규·재현 | `robot-start`가 CAN 컨트롤러의 LOOPBACK·LISTEN-ONLY를 통과시킨다. 실제 모드는 두 번째 `can <...>` 줄에 있는데 첫 줄 인터페이스 플래그만 본다. 가짜 `ip` 출력으로 **두 모드 모두 PASS 및 Compose 기동 명령**을 재현했다. BUS-OFF는 정상적으로 거부했다. | `scripts/robot-start:86–101`; `startup_repro.log` |
| 02 | P1 / 기존·재현 | Python 워치독 인스턴스 사이에 리셋 직렬화가 없다. 같은 정체 관측을 두 인스턴스에 주입하면 **리셋 2회**가 발생한다. 레거시 teleop은 CAN 소유권 락을 얻기 전에 워치독을 시작하므로, 소유권 획득 실패가 워치독의 선행 동작까지 막지는 않는다. | `motor_control/corner_module/can_watchdog.py:110–169`; `motor_control/chassis/teleop_server.py:387–390,416–417`; `core_repro.log` |
| 03 | P1 / 기존·재현 | `jetson_gui_up.sh`는 문자열 `state UP`만 준비 완료로 인정한다. 정상적인 `<UP,...> state UNKNOWN`을 넣으면 `can_setup.sh` 재실행을 요청한다. 기존 차체 소유자 확인보다 먼저 도달하며, 권한이 있으면 운용 중 CAN down/up을 유발할 수 있다. | `scripts/jetson_gui_up.sh:112–115,211–221`; `scripts/can_setup.sh`; `startup_repro.log` |
| 04 | P2 / 기존·재현 | 셸 워치독은 리셋 뒤 출력에 `loopback off`가 있어야 한다고 검사한다. iproute2는 꺼진 모드를 보통 생략하므로 **리셋 명령이 성공해도 검사에서 종료 코드 1로 죽는다.** 현재 Compose의 Python 워치독과는 다른 수동 실행 경로다. | `scripts/can_watchdog.sh:30–45`; `startup_repro.log` |
| 05 | P2 / 기존·정적 | USB 스키드 배포의 충돌 컨테이너 목록이 실제 이름 `powertrain_canwatchdog` 대신 `canwatchdog`를 사용한다. 이 배포 경로에서는 기존 워치독을 놓칠 수 있다. 최근 수동으로 정확한 컨테이너 이름을 지정해 모두 정지한 작업과는 구분한다. | `scripts/deploy_usb_skid_to_jetson.sh:124,130–134`; `docker/docker-compose.jetson.yml:49–60` |
| 06 | P1 / 기존·정적 | USB GUI는 CAN 소유권 락을 거치지 않고 `find_any()`로 첫 보드에 연결한다. 워커 시작 시 disarm도 실행한다. CAN 차체가 같은 보드를 운용 중이어도 별도 USB 프로세스가 상태·모드·목표를 덮어쓸 수 있다. 일반 USB 실험 도구에도 같은 우회가 있다. | `motor_gui/backend/transport/usb_odrive.py:42–49`; `motor_gui/backend/worker.py:86–94`; `motor_control/safety_us100/teleop_odrive_only.py:111`; `motor_control/pi/` |
| 07 | P1 / 기존·재현 | `bl70200_setup.py --apply`는 serial 선택이 필수가 아니고 기본 axis1/node11을 NVM에 저장한다. 가짜 13/14 보드에 기본 호출을 실행해 **13/11 저장**을 확인했다. 실제로 잘못 실행했다면 node 변경·중복이 전원 재인가 후에도 남을 수 있다. 저장 뒤 재탐색도 serial 미지정이면 다른 보드를 잡을 수 있다. | `motor_control/drive/bl70200/bl70200_setup.py:68–74,101–117,190–228`; `direct_tools_repro.log` |
| 08 | P1 / 기존·정적·수동 한정 | 폐기 표시만 있는 X2212 도구는 여전히 첫 USB 보드 설정 삭제·node1 지정·250 kbps 설정 시도·NVM 저장/재부팅을 실행할 수 있다. Pi 위치/속도 서버도 시작 시 보드 재부팅·재캘리를 수행하고 조건에 따라 반복한다. 자동 통합 기동 경로에서는 호출을 찾지 못했다. | `motor_control/drive/x2212_test/odrive_can_setup.py:13–43,99–100`; `init_odrive.py:45–58`; `motor_control/pi/pi_server_position.py:126–145,196–205,228–240`; `pi_server_velocity.py:133–153,209–219,247–258` |
| 09 | P2 / 신규 연결 포함·재현 | 신규 IDLE/FAULT 피드백 폴링은 6축 × 2 RTR × 50 tick = **600 RTR**를 만든다. 기본 50 Hz에서는 초당 600요청이며, 정상 정지 중에도 발생한다. GUI의 레거시 통합 CAN backend는 100회 sample에 300 RTR를 추가한다. 버스의 실제 총 부하·응답 지연 측정 없이 포화라고 단정할 수는 없다. | `motor_control/corner_module/corner_module.py:127–141`; `drive_odrive_can.py:249–254`; `motor_gui/backend/transport/can_bus.py:119–122`; `core_repro.log`, `direct_tools_repro.log` |
| 10 | P1 / 기존·재현 | 모터 GUI의 CAN ID 전환은 **새 ID 설정 후 close/disarm** 순서다. 11→13 전환에서 IDLE은 13에 두 번 가고 11에는 가지 않는다. 기존 모터 정지를 놓치면서 새 대상에 명령을 보낼 수 있다. 이것은 GUI 대상 변경이며 보드의 CAN node NVM을 바꾸는 동작은 아니다. | `motor_gui/backend/worker.py:360–377`; `motor_gui/backend/transport/odrive_can_device.py:146–147,320–324`; `direct_tools_repro.log` |

01의 실제 출력 형식은 [iproute2 v5.15의 CAN 모드 출력 구현](https://github.com/iproute2/iproute2/blob/v5.15.0/ip/iplink_can.c#L85-L105)과 대조했다. 첫 줄의 IFF_LOOPBACK과 CAN 컨트롤러 LOOPBACK은 같은 필드가 아니다. 과거 실제 외부 프레임 수신과 명시적인 loopback off 관측까지 이 결함으로 뒤집지는 않는다.

07은 **설정 변경 경로의 결함을 재현한 것**이다. 그 명령이 이번 Jetson에서 실행됐다는 증거는 없다. 기본 호출 한 번의 13/14→13/11 변경만으로 11–14 네 축의 동시 무응답을 모두 설명할 수도 없다. `--read` 출력은 펌웨어·전압·모터 설정은 보여주지만 serial, 양축 CAN node, baud, protocol, extended ID를 모두 대조하지 않는다(`bl70200_setup.py:36–53`). 기존 읽기 출력만으로 NVM 통신 설정을 정상 인증하면 안 된다.

08의 X2212 baud 설정은 예외를 잡고 계속하므로 “항상 250 kbps로 바뀐다”고 확정하지 않는다. Pi 두 서버의 설정 변경은 RAM 경로이며 그 자체를 NVM 저장으로 분류하지 않는다. 반면 `drive/bl70200/archive/`의 격리된 두 스크립트는 ODrive import 전에 하드스톱하므로, 그대로인 파일은 정상 실행으로 보드를 변경하지 않는다.

09는 정지 상태에서 실제 속도를 갱신하기 위해 넣은 변경이다. 제거하면 정지 판정에 오래된 속도가 남는 문제가 돌아온다. 공통 폴링 예산·주기 분산·RX 우선 처리로 다뤄야 한다. ARMED에서는 기존에도 각 축의 속도 명령 및 encoder/Iq 조회가 있었다. 신규 유휴 폴링은 RTR 조회이며 arm·속도 변경·재부팅·NVM 저장 명령은 아니다.

추가 기동 조합도 확인했다. `robot_prepare.sh:231`이 비활성 preflight 유닛을 시작하면 기존 운용 프로세스 정지 확인 없이 `can_setup.sh`에 도달할 수 있다. 이미 active인 `RemainAfterExit` 유닛을 단순 start하는 경우에는 다시 실행되지 않는다. 통합 Compose 실행 후 레거시 `jetson_gui_up.sh:360–370`을 실행하면 base Compose만으로 control/chassis를 재생성하면서 기존 session의 9000/9001과 충돌할 수 있다. 이는 구성 역전·재시작/접속 장애 후보이며 자동으로 CAN 소유자 두 개가 생긴다는 뜻은 아니다.

## 4. 통신 실패를 가리거나 수신을 잃는 결함

| 번호 | 우선 / 구분 | 확인 내용 | 근거 |
|---|---|---|---|
| 11 | P1 / 기존·재현 | 구동 `_send()`가 모든 `can.CanError`를 삼킨다. 6축 arm 송신 24개를 모두 실패시켜도 `arm()=True`, 차체 `ARMED`가 된다. **실제 축 상태가 IDLE(1)인 하트비트와 encoder 응답만 계속 주면 5 tick 뒤에도 차체는 ARMED**다. 송신 성공 및 모터의 실제 state8을 확인한 성공 상태가 아니다. | `motor_control/corner_module/drive_odrive_can.py:120–127,202–212`; `corner_module.py:164–173`; `motor_control/chassis/chassis_manager.py:492–504`; `core_repro.log` |
| 12 | P1 / 기존·재현 | AK `safe_send()`의 실패 반환을 GUI가 버려서, 버스 송신 예외에도 `ok=true, detail=sent`를 반환한다. 화면에서 명령 실패를 알 수 없다. | `motor_control/steering/ak_control.py:92–103`; `motor_gui/backend/transport/ak_device.py:97–100,191,215`; `direct_tools_repro.log` |
| 13 | P1 / 기존·재현 | ODrive RX 처리가 패킷의 수신 timestamp 대신 처리 시각을 기록한다. **10초 전 encoder 패킷을 넣어도 age=0, encoder_stale=false**가 된다. 일시적인 executor 정체 뒤 큐를 비우는 동안 오래된 상태를 현재 상태로 오인할 수 있다. | `motor_control/corner_module/drive_odrive_can.py:151–174`; `core_repro.log` |
| 14 | P2 / 기존 정책·신규 진단과 불일치·재현 | 구동 tick은 일반 RX stale와 axis_error만 검사한다. 하트비트가 살아 있고 encoder가 한 번도 오지 않아도 코너는 ARMED를 유지한다. 신규 `encoder_stale`는 WheelStates의 stale에 반영됐지만 구동 tick의 직접 게이트에는 없다. 피드백 상실 정책을 명확히 맞춰야 한다. | `motor_control/corner_module/corner_module.py:164–173`; `motor_control/chassis/chassis_manager.py:903–906`; `core_repro.log` |
| 15 | P1 / 기존·재현 | 레거시 GUI 통합 CAN backend에서 ODrive 수신 루프와 AK.poll이 **같은 소켓 큐**를 번갈아 소비하면서 상대 프로토콜 프레임을 버린다. AK 12.3°·ODrive 위치 7.5 프레임이 각각 사라져 화면은 0을 유지하는 것을 재현했다. 다른 프로세스의 별도 SocketCAN 소켓에서 프레임을 빼앗는 현상은 아니다. | `motor_gui/backend/transport/can_bus.py:119–145`; `motor_control/steering/ak_control.py:152–169`; `direct_tools_repro.log` |
| 16 | P2 / 기존·정적 및 디코더 재현 | GUI가 CANSimple `0x15`를 온도로 요청·해석한다. fw-v0.5.6에서는 sensorless estimates 명령이다. 위치 123.5 응답을 FET 온도 123.5로 표시하는 것을 재현했다. 불필요한 조회와 잘못된 진단이며, 해당 ID가 재부팅 명령인 것은 아니다. | `motor_gui/backend/transport/can_bus.py:31,121,162–164`; `odrive_can_device.py:24,182,203–205`; `direct_tools_repro.log` |

11에는 작동하는 방어도 있다. **피드백까지 전부 없으면 다음 tick에서 ESTOP**으로 바뀌는 것을 함께 재현했다. 따라서 “송신이 실패해도 모든 상황에서 무기한 ARMED”라고 일반화하지 않는다. 문제는 하트비트 수신을 송신 도달/폐루프 진입 증거로 대신 쓰는 경로다. 최근 축당 100 ms 대기를 제거한 변경은 arm 지연을 줄였지만 이 기존 성공 판정 문제를 해결하지는 않았다.

16의 명령 의미는 [ODrive fw-v0.5.6 CANSimple enum](https://github.com/odriverobotics/ODrive/blob/a308314ed2ca613164b81e7bbdfacc53cd1859ff/Firmware/communication/can/can_simple.hpp)과 [실제 응답 구현](https://github.com/odriverobotics/ODrive/blob/a308314ed2ca613164b81e7bbdfacc53cd1859ff/Firmware/communication/can/can_simple.cpp)으로 확인했다. 실제 보드가 다른 펌웨어면 그 버전의 계약으로 다시 대조해야 한다.

`preflight_hil.py:61`의 CAN 검사에도 기존 거짓 정상 판정이 있다. bitrate=250000 또는 CAN `<LOOPBACK>` 출력을 넣어도 OK다. 이 함수는 값과 주의 문구를 출력하지만 실제 일치 여부를 강제하지 않는다. 정상 `state UNKNOWN`은 반대로 DOWN으로 오인한다. 새 `robot-start`가 이 함수를 호출하는 것은 아니다.

## 5. CAN 장애처럼 보일 수 있는 상위 계층 경로

| 번호 | 우선 / 구분 | 확인 내용과 제한 | 근거 |
|---|---|---|---|
| 17 | P2 / 기존·정적 및 메서드 재현 | 단일 ROS executor 안의 동기 arm/disarm은 입력 처리를 지연시킬 수 있다. Twist는 source timestamp 없이 전달 시점으로 freshness를 시작하므로 지연된 명령이 새 명령처럼 취급된다. 큐 깊이 10도 남아 있다. 실제 ROS 큐 지연은 호스트에서 재현하지 않았고 타임스탬프 부여 메서드만 확인했다. | `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py:542–552,1415–1420,1686–1693,2158–2178,2370`; `ros_session_repro.log` |
| 18 | P1 / 기존·조건부 재현 | 같은 DDS domain에 FAKE/중복 wheel publisher가 있으면, 발행자 소유권 확인 없이 6개 정상·정지 값을 받아 `wheels_stopped=true`로 갱신할 수 있다. FAKE snapshot을 주입해 수용을 재현했다. **실제 Jetson domain에 그런 publisher가 있었다는 증거는 없다.** raw CAN 하트비트 소실의 설명도 아니다. | `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py:391–395,711–714,1471–1498`; `ops_broker_node.py:265–278`; `ros_session_repro.log` |
| 19 | P1 / 신규·재현 | 세션 서버 `_accept()`는 timeout 외의 모든 OSError에서 루프를 영구 종료한다. 일시적인 ECONNABORTED 한 번을 넣었을 때 accept 1회 뒤 반환한다. 프로세스가 살아 있어도 재연결을 받지 못할 수 있다. CAN 설정이나 모터 펌웨어를 바꾸지는 않는다. | `powertrain_runtime/session.py:136–143`; `ros_session_repro.log` |

운용 콘솔·인증 세션·영상 송수신은 직접 CAN 모터 소유자가 아니다. 연결 해제나 lease 만료가 게이트된 정지 요청을 만들 수는 있으므로 “운용에 전혀 영향 없다”는 표현도 맞지 않는다. 다만 프로세스 종료 후 특정 ODrive 보드의 하트비트가 사라지는 원인과는 별개다.

설치본 드리프트도 조건부로 남는다. Compose에서 설치 setup 파일의 존재만 확인하는 것은 최신 소스 재빌드 증거가 아니다. 지난 배포에서 colcon 재빌드한 기록과 별개로, 이번에는 Jetson 설치본을 조회하지 못했으므로 현재 소스와 동일하다고 인증하지 않는다.

## 6. 보드 펌웨어의 조건부 후보

**20 — fw-v0.5.6 CAN 서버의 오류 복구 분기 누락.** 공식 태그를 commit `a308314ed2ca613164b81e7bbdfacc53cd1859ff`로 고정해 검토했다. 서버 루프는 HAL 오류가 NONE일 때만 통신을 처리하고, 정확히 TIMEOUT일 때만 복구를 시도한다. 다른 nonzero 오류에는 RX/주기 서비스·자체 복구 분기가 없고, 송신 함수도 오류가 남으면 실패한다. 해당 오류가 실제로 고정되면 외부 재초기화 전까지 보드 CAN 통신이 멎을 수 있다. 두 축이 한 보드 CAN 서버를 공유하므로 양축 무응답 형태와 양립한다. [ODrive CAN 서버·송신·오류 콜백](https://github.com/odriverobotics/ODrive/blob/a308314ed2ca613164b81e7bbdfacc53cd1859ff/Firmware/communication/can/odrive_can.cpp#L45-L74)

이 후보에는 강한 제한이 있다.

- v3 보드 설정은 **AutoBusOff=ENABLE, AutoRetransmission=ENABLE**이다. “자동 bus-off 복구가 꺼져 있다”는 진단은 틀리다. [공식 v3 CAN 초기화](https://github.com/odriverobotics/ODrive/blob/a308314ed2ca613164b81e7bbdfacc53cd1859ff/Firmware/Board/v3/Src/can.c#L59-L73)
- 활성화된 인터럽트와 HAL 구현을 대조하면 일반 ACK 오류·bus-off가 곧바로 위 소프트웨어 오류 고정의 원인이라고 단정할 수 없다. TX 완료 오류 등 실제 도달 조건과 HAL 상태를 확인해야 한다. 자동 재전송이 켜져 있으므로 단순 arbitration loss가 항상 실패 완료가 된다는 가정도 성립하지 않는다. TIMEOUT 복구 역시 HAL 상태에 따라 실패할 여지가 있지만 실물 상태를 읽지 않았다. [해당 버전 STM32 HAL CAN 구현](https://github.com/odriverobotics/ODrive/blob/a308314ed2ca613164b81e7bbdfacc53cd1859ff/Firmware/ThirdParty/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_can.c)
- 실제 ODrive의 펌웨어 버전을 이번에 확인하지 못했다. 저장소에는 fw0.5.1 표기와 fw-v0.5.6 표기가 공존하며 Python 라이브러리 버전만으로 실물 펌웨어를 정할 수 없다.
- Jetson만 reboot/down/up하면 ODrive RAM은 초기화되지 않는다. 반대로 **ODrive 전원까지 실제로 재인가한 직후부터** 같은 현상이면, 오류의 재발 조건 또는 NVM/물리 원인을 추가로 설명해야 한다. 이 펌웨어 코드만으로 최근 고장을 확정하지 않는다.

## 7. 배제하거나 범위를 좁힌 가설

- 정상 구성의 AK extended ID와 ODrive standard node11–16은 별도 형식이다. 숫자가 작다는 이유만으로 직접 ID 충돌이라고 할 수 없다. 실제 NVM node 중복 여부는 별도다.
- 일반 CAN 제어 진입점은 `RealCanSession` 및 공유 `/run/powertrain` 락을 사용한다. “모든 프로세스가 무조건 동시에 CAN을 쓴다”는 결론은 틀리다. 위 USB 경로·워치독·설정 리셋이 보호 범위의 빈틈이다.
- 독립 SocketCAN 소켓들은 매칭 프레임의 사본을 받는다. `candump`나 다른 수신 프로세스가 있다는 이유만으로 차체 소켓에서 패킷을 훔쳐 간다고 할 수 없다. 15는 **동일 소켓 객체** 내부 경쟁이다. [Linux SocketCAN 수신 모델](https://docs.kernel.org/networking/can.html)
- fw-v0.5.6의 encoder/Iq 요청 처리에서는 RTR가 허용된다. `DLC=0` RTR 자체를 통신 버그로 판정하지 않았다. 응답 전송 mailbox 부족 시 실패·미재시도는 일부 조회 누락 후보이며 보드 전체 무응답과는 다르다. [CANSimple 요청 처리](https://github.com/odriverobotics/ODrive/blob/a308314ed2ca613164b81e7bbdfacc53cd1859ff/Firmware/communication/can/can_simple.cpp)
- 호스트 python-can 4.6.1의 SocketcanBus.send에서 timeout=None은 0으로 처리된다. “timeout 생략이 곧 무한 송신 블로킹”이라는 가설은 이 구현에 맞지 않는다. Jetson 패키지 버전은 미확인이다.
- 통합 기본 기동에서 자동 NVM 저장·설정 삭제·풀캘리 경로는 발견하지 못했다. 별도로 실행 가능한 수동 도구의 위험과 구분한다.
- 새 폴링, GUI 큐 손실, 콘솔의 잘못된 표시만으로 **모든 관련 프로세스 정지 후 raw CAN에서 계속 빠지는 11–14**를 설명했다고 결론내리지 않는다.

## 8. 검증 결과와 재현 방법

| 검증 | 결과 | 증거 범위 |
|---|---|---|
| 생산 Python AST | 259개, 구문 오류 0 | 문법 검사만 |
| corner_module + chassis manager/runtime lock 선택 테스트 | **214 passed** | 실제 원본 출력 `core-tests.log` |
| motor_gui + BL70200 선택 테스트 | **165 passed** | 분담 검토자의 실행 결과; 별도 원시 로그는 첨부하지 않음 |
| session/console runtime 및 ROS 순수 계약 선택 테스트 | **111 passed** | 명령·실행 결과 기록 `ros-session-tests.txt`; 원시 출력 그대로가 아닌 실행 기록임을 명시 |
| 4개 호스트 재현 스크립트 | 모두 exit 0 | 결함을 드러내는 관측/단언 완료. 시스템 정상 판정이 아님 |
| `test_clear_hold_adapters.py` | 수집 불가: rclpy 없음 | 위 111 통과에 포함하지 않음 |
| Jetson 설치 노드·실제 CAN·주행 E2E | 미실행 | 사용 불가. 복구/주행 통과 선언 안 함 |

재현 자료는 [2026-09-09-can-software-audit/](2026-09-09-can-software-audit/)에 있다. `core_repro.py`, `direct_tools_repro.py`는 실제 생산 클래스에 가짜 bus/board를 주입한다. `startup_repro.py`는 `ip`·`sudo`·`docker` 등을 임시 가짜 명령으로 바꾸고 셸 경로를 실행한다. 일부 함수는 현재 소스의 고정 행 범위를 추출하므로 소스가 바뀌면 추출 범위도 검토해야 한다. `ros_session_repro.py`는 rclpy 없이 AST에서 메서드 본문만 추출한다.

저장소 루트에서 python-can·pytest 등 기존 호스트 테스트 의존성이 있는 인터프리터로 실행한다.

```bash
export PYTHONDONTWRITEBYTECODE=1
export PYTHONPATH=.:motor_control:ros2/src/powertrain_ros
python docs/reports/2026-09-09-can-software-audit/core_repro.py
python docs/reports/2026-09-09-can-software-audit/startup_repro.py
python docs/reports/2026-09-09-can-software-audit/direct_tools_repro.py
python docs/reports/2026-09-09-can-software-audit/ros_session_repro.py
```

결함 재현의 기대 출력은 동봉된 `*_repro.log`다. 예를 들어 `old_frame_marked_fresh`와 `failed_tx_with_fresh_idle_feedback`가 나타나는 것은 **결함 재현 성공**이다. core의 `status stale → estop` stderr는 음성 대조에서 의도적으로 발생한다. 생산 코드 수정 후에는 일부 재현 단언이 깨지는 것이 정상이며, 수정된 계약에 맞는 회귀 테스트를 따로 작성해야 한다.

## 9. 수정 우선순위와 다음 실기 확인

1. **성공 판정부터 바로잡기:** 중요 제어 송신 실패를 기록·전파하고, arm 성공을 각 축의 최신 state8 및 오류0 확인과 묶는다. 정지/복구에서 필요한 피드백 freshness도 수신 시각과 명령 전후를 구분한다. ROS callback을 오래 막지 않는 비동기 확인 방식이 필요하다.
2. **CAN 준비·리셋 경로 통합:** 올바른 ctrlmode/bitrate parser 하나로 기동과 preflight를 맞춘다. 워치독 reset은 단일 소유·직렬화하고, 정상 UNKNOWN 때문에 운용 중 링크를 재설정하지 않게 한다. 잘못된 컨테이너 이름과 셸 워치독 종료 결함도 함께 정리한다.
3. **USB와 NVM의 보드 소유권 보강:** CAN/USB를 보드 단위로 조율하고, 쓰기에는 serial·대상 축·node를 명시한다. 저장 전후 serial 및 양축 통신 설정을 대조한다. 폐기 수동 도구는 정본 운용에서 실행될 수 없게 분리한다.
4. **GUI/상위 계층 수정:** ID 변경 전에 기존 대상을 정지하고, 한 소켓은 단일 RX dispatcher로 처리한다. 잘못된 0x15 온도 해석, AK 거짓 성공 ACK, 세션의 transient accept 오류를 수정한다. DDS 실제 발행자와 지연 명령 정책을 명확히 한다.
5. **통신 부하를 계측 가능한 형태로 만들기:** 유휴/주행/GUI별 송신량·실패 수·응답 age를 관측하고 RTR 폴링 예산을 정한다. 지금 프레임 수만으로 baud 변경이나 기존 정지 피드백 제거를 결정하지 않는다.

Jetson이 돌아오면 우선 **세 보드의 serial·실제 firmware·양축 node/extended ID/heartbeat 주기·board baud/protocol을 읽기 전용으로 보존**해야 한다. 이후 리셋 주체/시각, 송신 예외, raw CAN 수신을 같은 시간축에 기록해 단일 소유자 조건에서 비교한다. NVM 저장이나 보드 전원 재인가에 앞서 현재 값을 남겨야 원인 증거가 사라지지 않는다. 펌웨어 오류 후보는 실제 보드 버전과 내부 오류 상태를 확보해야 다음 단계로 올릴 수 있다.

### 원인 해석 보충 — 사용자 요청에 따른 실행 복기

프로세스 종료 후 무응답이 유지된다는 사실은 SW 유발 가능성을 배제하지 않는다. 신규 유휴 조회가 보드의 지속 오류 상태를 유발했을 가능성은 미확정 후보로 유지한다. 같은 정지 조건에서 기존 Git 0회/s, 최초 통합본 600회/s, 보완본 126회/s의 조회 패턴을 재현했다. [실행 순서·원인 복기](2026-09-09-can-sw-causality-review.md).
