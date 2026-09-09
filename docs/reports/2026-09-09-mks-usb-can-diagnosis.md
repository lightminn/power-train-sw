# MKS ODrive USB 진단·CAN부 복구 — 2026-09-09

**node ID는 정상이며, 보드 CAN부만 재초기화해 13·14를 복구했다. 최종 CAN 인식은 10/10이다.**
새 조회 트래픽이 MKS 펌웨어의 송신 경합을 유발했을 가능성이 남는다. 원시 로그의 Jetson 벽시계는 2026-08-18로 어긋나 있어 같은 기기 안의 순서·경과 시간을 기준으로 비교했다. 제조사 소스에서
해당 결함 경로를 고장 주입으로 재현했지만, 실물의 정확한 내부 오류값은 읽지 못했다.
영구 수정·장시간 안정성·주행 인수 완료를 뜻하지 않는다.

사용자가 보드를 **ODrive-MKS**라고 확인했다. 공식 ODrive 제품이나 0.5.6 펌웨어로
간주하지 않는다. [이전 SW 유발 가능성 복기](2026-09-09-can-sw-causality-review.md)의
USB 미연결·8/10 상태 뒤에 수행한 후속 진단이다.

## 1. USB에서 직접 읽은 값

| 항목 | 실측 |
|---|---|
| USB serial | `336A33523235`, registry상 13/14 보드 |
| 제품 식별 | USB는 `ODrive 3.6 CDC Interface`, 사용자 확인은 MKS |
| 펌웨어 / 보드 보고값 | **0.5.1, unreleased=1** / 3.6, variant 56 |
| axis0 / axis1 node | **13 / 14** |
| CAN | **500000 bps, protocol=0(Simple), 표준 ID, heartbeat 20 ms** |
| 전압 | 약 **46.87 V** |
| 양축 상태 | IDLE(1), requested_state 0, input_vel·실측 속도 0 |
| 오류 | 양축 axis·motor·encoder·controller·sensorless 오류 모두 0 |
| 캘리·시작 설정 | 양축 motor calibrated·encoder ready 및 pre_calibrated true, 자동 폐루프/캘리 시작 false |
| 보드가 노출한 can.error | 0. **STM32 HAL 오류값이 아니다.** |
| 양축 보드 watchdog | **enable_watchdog=false, timeout=0** |

이는 실행 중인 속성의 읽기 결과다. NVM 이미지 직접 비교나 전원 재인가 후 영속성 시험은
수행하지 않았다. 다른 두 보드의 firmware를 USB로 읽은 것도 아니다.
[원본 USB 스냅샷](2026-09-09-mks-usb-can-evidence/usb-board-initial.json).

USB 케이블 연결 후 및 USB 속성 읽기 후에도 수신 전용 6초 측정은 각각 8/10이었다.
USB 연결/읽기 자체가 13·14를 복구한 것은 아니다. 제어·session·watchdog는 계속 정지했다.

## 2. 보드 CAN부만 재초기화한 실제 결과

원본 상태 보존 후 동일 serial을 다시 확인하고 공유 motor owner lock 아래에서 USB API의
`board.can.set_baud_rate(500000)`를 **한 번** 호출했다. 제조사 코드에서 이 함수는 같은
baud를 설정하고 CAN Stop→Init→Start를 수행한다. 모터 상태 명령, NVM 저장, 보드 reboot,
캘리 또는 firmware flash는 호출하지 않았다. Jetson의 can0 down/up도 하지 않았다.

| 구간 | CAN 수신 |
|---|---|
| 호출 전 3초 | 기존 8개 정상, 13·14 각각 0 |
| 호출 후 약 8초 | **13=403, 14=402 heartbeat**, 다른 8개도 정상 |
| 후속 조회 시험 종료 후 최종 6초 | **AK1–4·ODrive11–16 각각 300프레임, 10/10** |

복구 호출 전후 serial·node·baud·protocol·heartbeat 설정은 동일했다. 이 복구 구간에서
Jetson CAN TX 증가량은 **0**이고 양축은 IDLE·오류 0을 유지했다. 따라서 node ID를
고쳐서 살아난 것이 아니라 **보드 CAN부의 재초기화로 통신이 돌아왔다**는 것이 실측 결론이다.

`system_stats.uptime`은 이전 두 USB 진단에서 동일한 4,090,445 ms였지만 CAN부 복구 후
6,901,806 ms로 진행했다. MKS 소스에서 이 값은 `vApplicationIdleHook()`가 갱신한다.
따라서 정체된 값은 보드의 실제 경과 시간을 항상 반영하지 않는다. CAN 오류 분기의
busy loop가 idle hook을 굶겼을 가능성과 양립한다. 이 값만으로 CPU 부하나 HAL 오류
종류를 확정하지 않는다. 복구 후 수신 시각과 uptime 차이로 통계 정체 시각을 역산하면
약 **06:58:22 UTC**다. 수정본 기동 06:56:57과 첫 소실 관측 06:59:04 사이에 들어간다.
이는 Jetson 벽시계가 그 사이 변하지 않았다는 가정의 근사치이며 실제 첫 소실 프레임 시각은 아니다.

[복구 스냅샷](2026-09-09-mks-usb-can-evidence/usb-can-peripheral-reset.json),
[최종 10/10](2026-09-09-mks-usb-can-evidence/can-usb-final.json),
[연속 프레임](2026-09-09-mks-usb-can-evidence/usb-can-peripheral-frames.jsonl.gz).

## 3. MKS 제조사 소스와 송신 경합 반례

검토 기준은 [Makerbase의 V3.6 배포 패키지](https://github.com/makerbase-motor/MKS-ODrive/tree/e15782976ae93d42b1f0648ceec96503141a343b/Firmware/ODrive_V3.6)의
`ODrive-fw-v0.5.1.zip`이다. ZIP SHA256은
`1c9ff347f997bbbf8cb693cdb44d93fe6f6ef992aefde60b6a65b0e1fa25a777`이다.
CAN 서버·CANSimple·HAL·axis·API 정의 등 핵심 9개 파일은 공식 fw-v0.5.1 commit
`7831d795235e5ef8535e4b46621a0721b458ec8f`와 바이트 단위로 일치했다.
**실물 unreleased 바이너리와 해당 ZIP의 동일성까지 확인한 것은 아니다.**

송신 경합 경로는 다음과 같다.

1. 각 axis thread는 `Axis::do_updates()`에서 heartbeat를 보낸다(`axis.cpp:208`).
   CAN thread도 encoder/Iq 요청을 처리하면서 응답을 보낸다.
2. 모두 같은 `ODriveCAN::write()`를 호출한다. 이 함수에는 여러 thread를 직렬화하는
   보호가 없으며, 빈 TX mailbox 확인과 `HAL_CAN_AddTxMessage()` 호출이 분리되어 있다
   (`interface_can.cpp:81–98`).
3. 조회 응답이 빈 슬롯 하나를 확인한 직후 heartbeat가 선점하여 그 슬롯을 소비하면,
   원래 응답의 HAL 호출은 슬롯 부족으로 `HAL_CAN_ERROR_PARAM`을 설정한다
   (`stm32f4xx_hal_can.c:943–949`). 하드웨어 CAN 오류 프레임이 없어도 가능한 API 오류다.
4. 이후 `write()`는 HAL 오류가 남아 있는 동안 모든 송신을 거부한다. CAN server도
   NONE/TIMEOUT 외 오류에 서비스·복구·대기 분기가 없다(`interface_can.cpp:23–45`).
5. USB `can.error`는 별도 `error_`이며, 이 버전 API의 오류 플래그는 중복 CAN ID다
   (`odrive-interface.yaml:200–205`). HAL PARAM과 자동 연결되지 않는다. 따라서
   **USB can.error 0·axis 오류 0인데 양축 CAN만 무응답**인 상태가 가능하다.

제조사 원문 `write()`와 `HAL_CAN_AddTxMessage()`를 그대로 추출·컴파일하고, 빈 슬롯
확인 직후 heartbeat 선점을 넣었다. 정상 송신 성공 → HAL PARAM `0x00200000` → USB
can.error 0 유지 → 빈 슬롯이 다시 생겨도 후속 송신 실패를 재현했다.
이는 **제조사 소스에 대한 결정적 반례**다. STM32 실기 실행이나 당시 HAL값 측정은 아니다.
[재현 스크립트](2026-09-09-mks-usb-can-evidence/reproduce_mks_tx_race.py),
[결과](2026-09-09-mks-usb-can-evidence/mks-tx-race.log),
[제조사 소스 출처·해시](2026-09-09-mks-usb-can-evidence/mks-source-manifest.json).

## 4. 복구된 13·14에만 한 제한 조회 시험

다른 보드에는 조회를 보내지 않았다. CAN raw TX/RX를 연속 기록하고 13·14 heartbeat가
400 ms 넘게 사라지면 중단하도록 했다. 모터 arm·속도·IDLE 등 상태 변경 지령은 없다.

| 구간 | 시간 | 결과 |
|---|---:|---|
| 수신만 | 6초 | 6축 heartbeat 모두 수신 |
| encoder RTR 단독, 목표 20 Hz/축 | 15초 | 지속 소실 없음 |
| Iq RTR 단독, 목표 5 Hz/축 | 15초 | 지속 소실 없음 |
| 현재 `DriveOdriveCan.poll_feedback()`, 50 Hz 호출 | 30초 | 지속 소실 없음 |
| 조회 중단 후 수신만 | 6초 | 6축 heartbeat 모두 수신 |

raw 프레임 39,832개 중 Jetson 송신은 1,914개이며, 전부 13/14의 `0x09`/`0x14`
RTR였다. 관측 오류 프레임은 0개다. **완전 무손실은 아니다:** 13은 encoder 764/Iq 193개
모두 응답했지만 14는 같은 요청 수에 761/191개로 총 5개가 덜 수신됐다. 14의 heartbeat
최대 간격은 조회 중 약 40 ms, 수신만 할 때 약 20 ms였다. 이 짧은 누락이 TX mailbox
경합 때문인지까지 직접 측정하지는 못했다.

따라서 조회 프로토콜은 실제로 동작하며 **이번 제한 조건의 지속 소실은 미재현**이다.
전체 6축 동시 조회, 실제 기동의 다른 지령, 장시간 운용과는 부하 조건이 다르다.
그 결과로 SW 유발 가설을 배제하거나 안정성을 인증하지 않는다.
[시험 결과](2026-09-09-mks-usb-can-evidence/usb-can-query-result.json),
[raw 분석](2026-09-09-mks-usb-can-evidence/raw-capture-analysis.json),
[연속 프레임](2026-09-09-mks-usb-can-evidence/usb-can-query-frames.jsonl.gz).

## 5. 현재 판단과 다음 수정 경계

- **확인:** 현재 node ID·CAN 설정이 맞고 MCU/USB/axis가 응답한다. 보드 CAN부만
  재초기화해 통신이 회복됐으며 최종 10/10·오류 0이다.
- **유력 후보:** 새 유휴 조회가 MKS의 공유 송신 경합을 유발하여 HAL 오류가 고정되는 경로.
  소스 반례 및 CAN부 복구·idle 통계 정체와 양립한다. 실물 HAL값은 USB API에 없어
  직접 읽지 못했고, 해당 바이너리와 소스의 동일성도 미확인이다.
- **미완료:** 지속 소실의 통제된 실물 재현, 설치된 firmware의 정확한 식별, 영구 수정 및
  장시간·주행 인수. source-level 영구 수정은 모든 송신 주체의 직렬화와 HAL 오류 복구를
  함께 다뤄야 한다. GPIO/부트/캘리 호환성을 확인하지 않고 다른 제품 firmware를 올리지 않는다.

또한 USB 실측상 두 축의 `enable_watchdog=false`, `watchdog_timeout=0`이었다.
통신이 멎었을 때 보드 자체의 watchdog timeout 정지를 기대할 수 없는 설정이다.
CAN에서 정지 지령을 보내는 것만으로 통신이 멎은 축의 제동까지 보장하지 못하므로
실주행 인수 전에 이 보호도 별도 검증해야 한다. 이번 진단에서 이 설정은 바꾸지 않았다.

현재 session/control/chassis/watchdog는 계속 정지 상태다. 통신이 돌아온 것을 자동 운전
재개 조건으로 쓰지 않았다. 이번에 생산 제어 코드나 보드 firmware/NVM은 변경하지 않았다.
