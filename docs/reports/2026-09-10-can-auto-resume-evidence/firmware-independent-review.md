# CAN 자동 재개 patch2 독립 펌웨어 리뷰 — 2026-09-10

**판정: 검토한 실제 patch2 소스에서 배포를 막을 결함을 발견하지 못했다.** 아래 결과는 펌웨어 소스와 결정적 HAL/RTOS 경계 시험에 한정한다. 호스트 자동 재개 구현, Cortex-M4 빌드 산출물, 실제 IRQ 타이밍·CAN 전송·모터·NVM 보존은 이 리뷰의 검증 범위가 아니다.

## 대상과 변경 범위

- 승인 계약: `/home/light/ZETIN/robotics/power-train-sw/docs/specs/2026-09-10-can-auto-resume.md`.
- 기준: `/tmp/mks-can-auto-resume-20260910/patch1-source/Firmware/`.
- 검토본: `/tmp/mks-can-auto-resume-20260910/patch2-source/Firmware/`.
- 아래 `Firmware/` 경로는 모두 위 검토본을 가리킨다.
- 실제 차이는 `MotorControl/axis.hpp`, `communication/can_simple.cpp`, `communication/interface_can.cpp`, `communication/interface_can.hpp`, `odrive-interface.yaml` 5개 파일이다.
- 검토본 SHA-256과 동일성 검사는 `firmware-independent-source-fingerprints.json`에 기록했다. 리뷰 중 구현 파일을 수정하지 않았다.

## 확인한 계약과 근거

| 항목 | 판정과 구체적 근거 |
|---|---|
| Heartbeat 표준 접두부 | `Firmware/communication/can_simple.cpp:455`에서 critical section을 시작해 bytes0–3에 axis error, byte4에 axis state를 복사한다. 메시지 길이는8, node/command ID 구성은 기존과 같다. |
| patch2 식별·상태 | 같은 파일 `:464`에서 byte5=`0xA2`, bytes6의 bit0/1/2는 latch/in-progress/eligible, byte7은 generation이다. snapshot을 끝낸 뒤에만 `odCAN->write()`를 호출한다. `Firmware/communication/interface_can.hpp:37`의 patch identity도2다. |
| 최초 자격 생성 | `Firmware/MotorControl/axis.hpp:119`의 `!can_recovery_latched_` 분기 안에서만 결정한다. `:124`–`:128`은 CLOSED_LOOP, requested_state=UNDEFINED, axis/motor/encoder/controller/sensorless error 모두0을 요구한다. IDLE와 pending IDLE에서는 새 자격이 생기지 않는다. |
| HAL과 수동 재초기화 구별 | `Firmware/communication/interface_can.cpp:35`–`:45`가 queued manual origin을 읽고 지우는 동작과 두 축의 latch/disarm을 하나의 바깥 critical section 안에서 수행한다. 동시 수동 요청은 해당 section 전에 반영되거나 section 뒤에서 자격을 즉시 취소한다. |
| 반복 HAL 복구 | 이미 latch가 있으면 `axis.hpp:119` 조건을 건너뛰어 기존 허가/거부 판정을 보존한다. `:130`의 uint8 generation은 시도마다 증가하며256회에서0으로 되돌아가는 것은 명시된 동작이다. 복구 중 manual/clear/E-stop이 취소한 false를 재시도가 true로 만들지 않는다. |
| 수동 CAN 요청 취소 | `interface_can.cpp:162`–`:167` 및 `:171`–`:175`는 pending request와 manual flag 설정 및 두 축 eligible=false를 critical section 안에서 처리한다. 유효하지 않은 baud는 `:158`–`:159`에서 아무 요청도 enqueue하지 않는다. |
| 명시 CAN E-stop 취소 | `can_simple.cpp:189`–`:194`는 eligible=false와 ESTOP error 설정을 critical section으로 묶는다. 정상 수신 프레임의 길이/RTR 검증은 `:63`–`:67`에 있다. |
| clear_errors 취소 | `axis.hpp:143`–`:145`가 자격을 먼저 취소한다. 진행 중·아직 비IDLE·queued arm인 경우에는 `:149`–`:152`에서 반환해 latch/error를 유지한다. 완료된 IDLE에서만 `:154`–`:165`가 입력을 다시0/현재위치로 만들고 latch/error를 지운다. |
| 재부팅 기본값 | `axis.hpp:110`–`:113`은 RAM 필드를 false/false/false/0으로 초기화한다. readonly Fibre 노출(`odrive-interface.yaml:235`–`:239`)은 NVM 필드 추가가 아니다. |
| 즉시 정지·입력 삭제 | `axis.hpp:131`–`:138`은 latch/progress/error 설정, velocity/torque=0, position=현재위치, requested IDLE, PWM disarm을 수행한다. `low_level.cpp:113`–`:119`의 실제 정지 함수는 motor armed state를 DISARMED로 바꾸고 TIM MOE를 끈다. |
| latch 우회 방지 | `can_simple.cpp:71`–`:80`에서 latched drive setpoint를 차단하고 `:90`–`:97`은 IDLE 외 요청·anticogging을 차단한다. `low_level.cpp:100`–`:105`는 latch가 있을 때 PWM arm을 거부한다. 직접 USB axis.error 쓰기도 `axis.hpp:170`–`:173`의 check에서 latch를 우회하지 못한다. |
| 복구 완료·실패 경계 | `interface_can.cpp:48`–`:57`은 실제 재초기화 성공이며 추가 요청도 없을 때만 I/O 및 in-progress를 풀고, 성공·실패 모두10ms yield한다. 실패/새 요청에서는 I/O gate가 닫힌 채 유지된다. PWM 재무장이나 cached setpoint 복원이 이 경로에 없다. |
| TX/RX·HAL 경계 | 기존 `interface_can.cpp:104`–`:118`, `:130`–`:154` gate를 유지한다. HAL Stop/Init은 provenance/disarm critical section 밖(`:48`, `:178`–`:225`)에서 실행된다. `MotorControl/low_level.h:62`–`:69`는 이전 PRIMASK를 복원하므로 바깥/안쪽 critical section이 중첩되어도 중간에 IRQ를 열지 않는다. |
| NVM 배치 | `axis.hpp:28`–`:64`, `interface_can.hpp:24`–`:27`의 두 Config_t 선언은 patch1과 문자 그대로 같다. `MotorControl/main.cpp`와 `MotorControl/nvm_config.hpp` 전체 파일도 byte-identical이다. 저장 타입/순서는 `main.cpp:29`–`:41`, 실제 저장 인자는 `:43`–`:56`이며 추가 RAM 필드는 포함되지 않는다. 이는 소스 배치 동일성 증거이며 실물 NVM readback을 대체하지 않는다. |
| Watchdog 유지 | `MotorControl/axis.cpp` 전체가 patch1과 byte-identical이다. `can_simple.cpp:181`의 accepted control만 watchdog feed하는 경로도 변경되지 않았다. telemetry/RTR/invalid frame이 watchdog을 연장하지 않는 기존 시험이 계속 통과했다. |

## 직접 실행한 검증

저장소 루트에서 다음 명령을 실행했다.

```bash
/home/light/anaconda3/bin/python firmware/mks_odrive/tests/run_source_tests.py \
  /tmp/mks-can-auto-resume-20260910/patch2-source
```

- **51 passed, 0 failed.** 최초 전달 당시50개에서 `resume_manual_at_dispatch`가 추가된 현재 하니스로 실행했다.
- 로그: `/tmp/mks-can-auto-resume-20260910/firmware-independent-source-tests.log`.
- 기존26개 TX 경합·HAL 실패·재초기화 배타·zero/latch·명령 검증·watchdog 시험과, 새25개 heartbeat/provenance/반복 복구/취소/오류/재부팅/generation 시험을 포함한다.

같은 하니스에 patch1을 입력한 음성 대조도 직접 실행했다.

```bash
/home/light/anaconda3/bin/python firmware/mks_odrive/tests/run_source_tests.py \
  /tmp/mks-can-auto-resume-20260910/patch1-source
```

- **26 passed, 25 failed, exit1.** 새25개는 patch2 marker/identity 부재로 실패했고 기존26개는 통과했다.
- 로그: `/tmp/mks-can-auto-resume-20260910/firmware-independent-patch1-negative.log`.

추가로 두 Config_t의 중괄호 전체 선언 및 `main.cpp`, `nvm_config.hpp`, `axis.cpp`, `low_level.cpp` 파일을 직접 byte 비교해 patch1 동일성을 확인했다.

## 증거 해석의 제한

1. 하니스는 실제 CAN/Axis/PWM 함수 본문을 추출·컴파일하지만 Axis 전체 RTOS 실행이나 STM32 전기적 출력을 재현하지 않는다. 정지 시점과 인터럽트 지연의 물리 증거로 승격하지 않는다.
2. `tests/run_source_tests.py:44`–`:48`의 `observe_tx_attempt()`는 TX 경계 **시도**를 관측한다. I/O gate가 닫힌 복구 중 heartbeat encoder flags 시험의 성공은 그 패킷이 실제 CAN으로 전송됐다는 뜻이 아니다. 실제 전송은 별도 vcan/보드 readback으로 확인해야 한다.
3. heartbeat generation은 uint8이며 수동 요청 취소 자체의 nonce가 아니다. 펌웨어는 flags로 취소를 명시한다. 호스트가 generation 단독으로 복귀를 결정해서는 안 된다는 경계는 승인 스펙과 동일하다.
4. 호스트 구현·바이너리 provenance·실제 flash/NVM·주행·ROS 인수는 검토 제외다. 이 보고서의 'blocker 없음'은 검토한 소스 범위에 한정한다.

**최종 발견사항: 소스 계약 위반으로 확인된 blocker 없음. 수정 제안 없이 검증 범위와 한계를 인계한다.**
