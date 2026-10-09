# patch3 guarded RESUME 독립 소스 리뷰 — 2026-09-10

**판정: 검토한 최종 patch3 펌웨어 소스에서 추가 blocker를 발견하지 못했다.** 실제 patch2와 대조하고 최신86개 source scenario를 직접 실행했다. 이 판정은 호스트/STM32 실행 시점/물리 CAN·모터·flash 인수와 구분한다. 생산 코드 수정, Jetson/하드웨어 접근, 커밋/푸시는 하지 않았다.

- 대상: `/tmp/mks-can-auto-resume-20260910/patch3-source/Firmware`.
- 비교본: `/tmp/mks-can-auto-resume-20260910/patch2-source/Firmware`.
- 전체 Firmware 파일 비교 결과 실제 차이는 `communication/can_simple.cpp`, `communication/interface_can.hpp` 두 파일뿐이다. 추가/삭제 파일은 없다.
- 최종 계약: `docs/specs/2026-09-10-can-auto-resume.md`의 opcode7/DLC8/`<IBBBB>(8,0xA3,generation,0,0)` guarded RESUME.

## 안전 분기와 근거

아래 파일:행은 위 patch3 Firmware 기준이다.

| 확인 항목 | 실제 근거와 판정 |
|---|---|
| 프레임 길이/RTR 선검사 | `communication/can_simple.cpp:46-67`에서 state 명령은 non-RTR+DLC4/8, manual clear는 non-RTR+DLC0/8만 허용한다. 바이트 읽기 전에 검증한다. |
| guard 형식 및 일반 ARM fallback 차단 | 같은 파일 `:90-99`에서 비영 suffix는 state8/A3/reserved0인 guard만 인정한다. 잘못된 marker, reserved 또는 state는 즉시 반환한다. latch 상태에서 ordinary state8은 차단한다. |
| device 내부 atomic compare | 같은 파일 `:261-274`에서 critical section에 들어간 뒤 odCAN 존재, pending reinit 없음, 재초기화 중 아님, HAL error0, eligible+latch, progress=false, generation 일치, current IDLE, requested UNDEFINED/IDLE, exact0x4000, motor/encoder/controller/sensorless 오류0을 검사한다. |
| clear/zero/request8 단일 커밋 | 같은 파일 `:275-280`에서 `clear_errors()`와 requested state8 쓰기가 동일한 바깥 critical section 안이다. `MotorControl/axis.hpp:142-166`의 clear는 eligible를 취소하고 입력 velocity/torque0 및 input_pos=current position으로 바꾼다. guard의 선조건은 clear의 early-return 조건을 모두 배제한다. |
| nested critical 유지 | `MotorControl/low_level.h:62-69`가 기존 PRIMASK를 복원한다. 바깥 mask가1인 상태의 중첩 clear는 IRQ를 먼저 열지 않는다. 실제 함수 호출을 추출한 `guard_resume_atomic`은 IRQ가 처음 열리는 시점에 clear+zero+requested8이 전부 커밋되었음을 단언한다. |
| 거부 불변성 | `can_simple.cpp:279-283`: guard 조건이 거부되어도 critical section 종료 뒤 바로 return한다. 일반 requested-state 대입은 실행하지 않는다. 오류·하위 오류·입력·requested state·latch/eligible/generation 보존을 scenario가 검사한다. |
| 외부 manual clear 경합 종결 | guard 전에 external `clear_errors()`가 실행되면 eligible/latch가 false라 guarded RESUME는 거부된다. clear와 별도의 ordinary ARM을 실행하는 펌웨어 경로가 없다. critical 내부에는 thread/IRQ clear가 끼어들 수 없다. guard 뒤 새 fault는 다시 지우지 않는다(`guard_resume_external_clear`, `guard_resume_replay_after_fault`, `guard_resume_late_fault`). 호스트도 guard 뒤 일반ARM을 보내지 않는지는 별도 host review 대상이다. |
| cmd0x18 범위 | `can_simple.cpp:473-479`: 검증된 DLC0/8에서 모든 전달 바이트가0이어야 manual clear를 호출한다. 과거 비영 CAS payload 및 임의 비영 clear는 거부된다. 정상 manual zero8/empty clear는 유지된다. |
| heartbeat 호환 | `can_simple.cpp:482-506`: 기존 error32/state8 접두부 유지, byte5=A3, byte6 latch/progress/eligible, byte7 generation. snapshot 전체를 critical로 묶고 write는 종료 뒤 수행한다. `interface_can.hpp:37`의 patch identity는3이다. |
| 반복 HAL 및 manual revoke 보존 | `communication/interface_can.cpp`와 `MotorControl/axis.hpp`가 patch2와 byte-identical이다. pending manual origin/양축 latch의 원자 캡처(`interface_can.cpp:35-45`), bounded HAL reset 뒤 재요청 재검사(`:48-57`), manual 요청 즉시 양축 revoke(`:162-175`)와 기존 zero/PWM disarm 경계가 그대로다. guard는 그 pending/HAL 조건을 재확인한다. |
| 직접 PWM 재무장 없음 | guard는 requested state만 설정한다. 실제 arm은 기존 Axis state machine 소유(`axis.cpp:479-506`, `:565-577`). source scenario는 callback에서 PWM이 계속 disarmed임을 검사한다. |
| watchdog 의미 유지 | `MotorControl/axis.cpp` 전체가 byte-identical이다. guard는 accepted motion-input 명령이 아니므로 CAN handler `:186`의 watchdog_feed를 호출하지 않는다. 기존 state machine의 CLOSED_LOOP 진입 feed(`axis.cpp:570`)는 유지된다. |

## NVM/소스 동일성

`Axis::Config_t`와 `ODriveCAN::Config_t` 선언은 patch2와 문자 그대로 같고, `MotorControl/main.cpp`, `nvm_config.hpp`, `axis.hpp`, `axis.cpp`, `low_level.cpp`, `low_level.h`, `communication/interface_can.cpp`, `odrive-interface.yaml` 전체가 byte-identical이다. 비교 결과와 SHA-256은 `patch3-independent-fingerprints.json`에 저장했다. NVM 저장 타입/필드/순서 변경이 없는 소스 증거이며 실물 NVM readback을 대신하지 않는다.

## 직접 실행한 시험

병행 추가 테스트의 스냅샷을 고정하기 위해 repository의 runner/hal_boundary/scenarios 세 파일을 `patch3-independent-harness/`에 복사하고 SHA-256을 기록했다. 함수 추출 대상은 실제 patch3/patch2 소스다. 시험 중 production 파일은 수정하지 않았다.

```bash
/home/light/anaconda3/bin/python /tmp/mks-can-auto-resume-20260910/patch3-independent-harness/run_source_tests.py \
  /tmp/mks-can-auto-resume-20260910/patch3-source
/home/light/anaconda3/bin/python /tmp/mks-can-auto-resume-20260910/patch3-independent-harness/run_source_tests.py \
  /tmp/mks-can-auto-resume-20260910/patch2-source
```

- **patch3:86 passed,0 failed, exit0.** 기존26+metadata25+guard/manual boundary35개. 최종 추가된 eligible 상태 reserved-byte 거부도 포함한다.
- **patch2 음성 대조:49 passed,37 failed, exit1.**25개는 A3 marker/identity 부재,12개는 guard 원자성/거부/수동clear 경계 부재를 검출했다. 기존 latch만으로 거부되는 guard 음성 scenario 일부는 patch2에서도 통과하므로 모든 신설 시험이 patch3 고유 검출기인 것은 아니다.
- 로그: `patch3-independent-source-tests.log`, `patch3-independent-patch2-negative.log`.

HAL/RTOS 경계 하니스는 실제 함수 본문을 g++로 실행하되 IRQ/스케줄러/전기적 동작을 결정적 더블로 대체한다. 실제 Cortex-M4 빌드, 임계구역 실행 시간, 보드 flash/NVM 보존, ROS 설치 E2E, 실물 구동은 root의 별도 증거로 판정해야 한다. TX attempt 관측은 물리 버스 전송 증거가 아니다.
