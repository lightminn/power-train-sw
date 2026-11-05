# MKS CAN 신뢰성 patch4 설계 — 정보성 오류 비정지·CAN 제어 수신 만료

2026-10-15 사용자 지시("설계 진행해, codex 수렴검토 포함")로 작성한다. 운전 재개를 위한
콘솔 시동 재실행은 허용 범위이며, 보드 전원 재인가가 필요한 경로는 만들지 않는다(사용자 확인).
patch3([설계](2026-09-10-can-auto-resume.md), [보고](../reports/2026-09-10-can-auto-resume.md))의
후속이며 범위는 원격주행 CAN이다. 자율주행·US-100·L515는 제외한다.

## 1. 배경 — 2026-10-15 patch3 검토에서 확인한 결함

| # | 결함 | 근거(패치 적용 소스) | 판정 |
|---|---|---|---|
| R1 | LEC(stuff/form 등)·RX FIFO overrun 한 번이 재초기화와 양축 ESTOP 래치를 일으킨다. 필터가 모든 ID를 받아 다른 9개 모터 프레임이 깊이 3 FIFO를 채우고, 복구 성공 후 10 ms 대기 중 4프레임이 오면 다시 overrun이 난다. | `interface_can.cpp:50,57,214-219,262`, HAL `stm32f4xx_hal_can.c:1435,1577-1611` | P1 |
| R2 | uint8 복구 세대가 한 바퀴 돌면 과거 guarded RESUME이 현재 래치를 해제한다. | `axis.hpp:119,130`, `can_simple.cpp:265` | P2 |
| R3 | 재초기화 요청이 서버에 처리되기 전 USB `clear_errors()`가 래치를 해제하고 PWM arm 게이트를 통과한다. | `axis.hpp:142-149`, `interface_can.cpp:162`, `low_level.cpp:102` | P2 |

운용 기록의 HAL 이력 `0x218` = RX_FOV0(0x200)|FOR(0x10)|STF(0x8)이 R1 경로와 일치한다.

**원래 lock의 범위 정정.** 벤더 0.5.1은 RX pending 알림만 켜고 `HAL_CAN_ErrorCallback()`에서
`HAL_CAN_ResetError()`를 불렀다(원본 `interface_can.cpp:28,79,213`). 원래 영구 무응답은 TX
경합으로 생긴 **동기 API 오류(PARAM)** 를 벤더 `write()`·서버가 `ErrorCode` 검사로 고착시킨
것이다([9/9 진단 §3](../reports/2026-09-09-mks-usb-can-diagnosis.md)). `HAL_CAN_AddTxMessage()`
자체는 `ErrorCode`를 검사하지 않는다(`stm32f4xx_hal_can.c:872` 이하). LEC·overrun을 정지
사유로 만든 것은 patch1이 오류 알림을 전부 켜고 치명 처리한 결과다.

## 2. 계약

patch 식별자는 `can.reliability_patch == 4`. **CAN 와이어 포맷(heartbeat byte5–7 `0xA3`/flags/
generation, cmd `0x07` `<IBBBB>` guarded RESUME, cmd `0x18` clear)과 Config_t/NVM 레이아웃은
변경하지 않는다.** 호스트 코드 변경은 없다.

### D0. CAN 제어 수신 만료 (신규, 안전)

보드 watchdog은 NVM상 비활성(`enable_watchdog=false`)이고, 호스트는 watchdog feed를 쓰지 않는다
(USB 스키드 `DriveOdriveUsbAxis` 포함). NVM watchdog을 켜면 USB 운전 경로가 끊기므로, CAN
제어 세션에만 적용하는 펌웨어 만료를 둔다.

- 축 RAM 필드: `can_owner_`, `can_feed_valid_`, `can_last_feed_tick_`, `can_entry_recorded_`,
  `can_entry_tick_`. 모든 갱신은 `cpu_enter_critical()` 안에서 한다.
- **세션 시작:** IDLE(1) 이외의 CAN `0x07` 상태 요청 수락(state8, guarded RESUME 커밋, state2
  STARTUP_SEQUENCE, 캘리브레이션 등) 시, `requested_state_` 쓰기와 같은 critical section에서
  `can_owner_`가 false면 true로 바꾸고 `feed_valid=false`, `entry_recorded=false`로 둔다. 이미
  true면 바꾸지 않는다(멱등, 타이머 보존). CAN IDLE 요청은 소유권을 바꾸지 않는다.
- **유예 기준점:** 상태머신 CLOSED_LOOP case에서 `run_closed_loop_control_loop()` 호출 직전
  (기존 `watchdog_feed()` 옆, `axis.cpp:570`) `owner && !entry_recorded`이면 진입 시각을 기록한다.
  재진입은 기록을 갱신하지 않는다.
- **피드:** CLOSED_LOOP 중 유효 CAN motion setpoint(patch3 watchdog feed와 같은 술어 —
  Set_Input_Pos 8 B, Set_Input_Vel 8 B, Set_Input_Torque 4/8 B, RTR·짧은 DLC·NaN/Inf 제외)를
  수락하면 `owner=true`, `feed_valid=true`, `last_feed=now`. owner가 false→true면 진입 기록도
  함께 세운다. → USB로 arm한 축도 CAN motion을 받는 순간 만료 대상이 된다.
- **세션 종료:** 유일한 경로는 상태머신이 IDLE case를 실행할 때 시작부에서
  `requested_state_ == UNDEFINED`이면 owner·feed를 해제하는 것이다. IDLE을 거치지 않는
  state8 재진입(`axis.cpp:502-506` task chain 덮어쓰기, CAN 또는 USB `requested_state=8`)은 소유권과
  타이머를 바꾸지 않는다. CLOSED_LOOP 사전검사 실패·함수 내부 조기 반환은 IDLE을 거쳐 해제된다.
- **만료:** `run_control_loop`의 `watchdog_check()`와 같은 지점에서 `enable_watchdog`과
  무관하게, `current_state_ == CLOSED_LOOP && owner &&
  (feed_valid ? now−last_feed > T_cmd : entry_recorded && now−entry_tick > T_grace)`이면
  `error_ |= ERROR_WATCHDOG_TIMER_EXPIRED`로 false를 반환해 기존 루프 break 경로로 disarm한다.
  자동 재개 자격은 없다. 호스트는 기존대로 비관련 오류로 처리하며, 재개는 콘솔 시동 재실행이다.
- `T_cmd = 300 ms`, `T_grace = 500 ms`(벤치 후보값). CLOSED_LOOP 외 상태(캘리브레이션 등)는
  검사하지 않는다. query·heartbeat·clear·state 명령과 다른 노드의 ACK는 피드가 아니다.
- 시계: `osKernelSysTick()`(FreeRTOS 1 kHz, uint32 tick, `cmsis_os.c:227-234`,
  `FreeRTOSConfig.h:101,106`). 차분은 unsigned로 계산해 49.7일 wrap을 가로질러도 맞게 한다.
  `>` 비교라 tick 차분 301/501부터 만료하며, 실제 차단에는 tick 양자화·축 루프(8 kHz)·PWM 차단
  지연이 더해진다. 300/500 ms는 하드 상한이 아니다. tick 자체가 멎는 고장은 보호 범위 밖이다.
- **기준 시각은 보드가 setpoint를 마지막으로 수락한 시각이다.** 호스트 정지 시각이 아니다. 호스트
  SocketCAN 송신 큐(`txqueuelen 1000`, `can_interface.py:74`)에 쌓인 과거 비영속도 프레임이 늦게
  도착하면 그만큼 만료가 늦어진다. 호스트 큐 체류시간은 별도 실측 항목이다(5절).
- 미지원 혼용: CAN으로 arm한 축에 USB setpoint만 쓰면 정지한다(안전 방향). CAN motion을 한 번이라도
  받은 세션은 마지막 수락 후 300 ms, 받지 않은 세션은 CLOSED_LOOP 진입 후 500 ms.
  sensorless 제어는 이 시스템에서 쓰지 않으며 대상이 아니다.
- **부팅 자동 시작은 D0 대상이 아니다.** 부팅 시 상태머신이 스스로 STARTUP_SEQUENCE를
  요청하므로(`axis.hpp:282`, `axis.cpp:481-494`) CAN 요청 없이 폐루프에 들어가면 owner=false다.
  따라서 모든 `startup_*`=false는 운용 계약이 아니라 **D0 안전의 필수 전제**이며, 5절 NVM 인수
  조건으로 플래시 전·부팅 후에 확인한다.
- 벤더 동작 유지: CLOSED_LOOP 진입은 `input_vel_`을 지우지 않는다. 호스트 arm 경로는 zero 입력을
  먼저 보내며(`drive_odrive_can.py:355`), D0이 남은 입력의 지속 시간을 상한한다.

### D1. 오류 분류

- **정보성** — 계수하고 `ErrorCode`에서 해당 비트만 제거한다. 재초기화·래치는 없다:
  LEC 계열(STF/FOR/ACK/BR/BD/CRC), EWG, EPV, BOF 발생, RX_FOV0/FOV1.
  - ISR에서는 `HAL_CAN_ErrorCallback()` 안에서 처리한다. 스레드 경로의 읽기→계수→비트 제거는
    한 PRIMASK 구간에서 한다. `HAL_CAN_ResetError()`로 전체를 지우지 않는다.
  - 정보성 mask = 위 비트의 OR = `0x000007FF`(`stm32f4xx_hal_can.h:251-273`). API 계열 비트
    (PARAM/NOT_INITIALIZED/TIMEOUT/NOT_READY/NOT_STARTED 등)가 섞여 있으면 정보성 비트만 지우고
    나머지는 고착 처리한다. 예: `0x00200218` → 정보성 `0x218` 계수, PARAM `0x00200000`은 고착.
  - IRQ 폭주 억제는 patch3 방식(콜백에서 비-RX 알림 비활성화, 서버 루프 재활성화)을 유지한다.
- **고착** — 재초기화와 기존 래치 경로: (a) HAL API 실패 또는 API 계열 `ErrorCode`,
  (b) 재초기화 중이 아닌데 handle State ≠ LISTENING, (c) USB 정비 요청.
- BOFF 지속 감지와 TX 무진행 감지는 두지 않는다. 보드가 호스트를 못 듣는 경우의 정지는 D0이
  보장한다. TX만 막힌 보드는 호스트의 정지 명령을 여전히 받는다. 지속 BOFF는 ABOM이 담당하며,
  물리 결함으로 통신이 돌아오지 않으면 정비 재초기화가 필요하다(통신 복구 보장 범위 밖).
- ISR은 재초기화를 요청하지 않는다(D5의 문맥 제약).
- 새 Fibre RAM 계수: `lec_error_count`, `rx_overrun_count`, `bus_off_count`,
  `error_passive_count`, `error_warning_count`, `control_deadline_count`.

### D2. 수신 필터 — 자기 node 두 개만

- 32-bit IDMASK, FIFO0. bank0 = axis0, bank1 = axis1(CAN1, `SlaveStartFilterBank=14`와 비충돌).

  | 형식 | ID word | Mask word | node 범위 |
  |---|---|---|---|
  | STD | `node << 26` | `0xFC000004` | 0…0x3F |
  | EXT | `(node << 8) \| 4` | `0xFFFFFF04` | 0…0xFFFFFF |

  RTR(bit1)는 don't-care(조회에 필요). IDE(bit2)는 비교해 STD/EXT를 구분한다. AK45-36은 EXT ID를
  쓰므로(`ak_control.py:99-103`) STD 필터에 들어오지 않는다. 범위 밖 node는 무매칭 필터. `0x3F`는
  이 벤더 라우터에서 broadcast가 아니다.
- **송신 측 범위 검사(신규).** 벤더 송신은 범위를 검사하지 않고(`can_simple.cpp:252-254`
  node 변경, `:482-486` ID 생성), HAL은 STD ID를 21비트 이동해 32비트 TIR에 쓰며 범위 검사는
  비활성 `assert_param`뿐이다(`stm32f4xx_hal_can.c:904-908`, `stm32f4xx_hal_conf.h:172`). 그래서
  STD node 77의 heartbeat `0x9A1`이 `0x34200000`으로 잘려 node13 heartbeat(`0x1A1`)와 같은 프레임이
  된다. ID 생성 전에 uint32 이동이 먼저 wrap하므로(예: node `0x0800000D` → `uint32(node<<5)|1` =
  `0x1A1`), 생성된 ID만 검사하면 통과한다. 따라서 두 겹으로 막는다.
  1. **ID 생성 전 node 검사:** `can_simple.cpp`의 모든 `txmsg.id = axis->config_.can_node_id <<
     NUM_CMD_ID_BITS` 생성 지점(heartbeat 포함)을 단일 helper로 모은다. helper는 원래 node가
     STD ≤ `0x3F`, EXT ≤ `0xFFFFFF`인지 먼저 검사하고, 범위 밖이면 프레임을 만들지 않고
     `tx_drop_count`로 센다. node를 CAN setter가 아닌 USB로 직접 바꿔도 송신 시점에 검사되므로
     setter 검사에 의존하지 않는다.
  2. **송신 직전 ID 검사:** `ODriveCAN::write()`에서 STD ID > `0x7FF`, EXT ID > `0x1FFFFFFF`는 HAL에
     넘기지 않고 `tx_drop_count`로 센다.
- node/extended 설정 변경은 서버 루프가 캐시와 비교해 재적용한다. 재적용 중 FINIT 구간의
  수신 손실은 수용한다(손실된 제어는 D0이 정지로 귀결). 운전 중 node 변경은 지원하지 않는다.
  같은 node 중복 설정은 벤더 라우터 동작을 유지한다.

### D3. 복구 성공 후 대기 제거

성공하면 대기 없이 일반 루프(`osSemaphoreWait(sem_can, 10)` 후 FIFO 처리)로 돌아간다. 실패하면
`osDelay(10)` 후 재시도한다.

### D4. 복구 세대

- patch3대로 복구 **시도마다** 세대를 올린다(호스트가 세대 변화로 200 ms 안정 판정을 다시 시작,
  `can_recovery.py:85`).
- **축별** 래치 에피소드(래치 설정 ~ 래치 해제) 안의 복구 시도 수를 포화 카운터로 센다. 17번째
  시도부터 그 축의 자동 재개 자격을 에피소드가 끝날 때까지 복구하지 않는다(재초기화 재시도 자체는
  계속되며 세대도 계속 증가·wrap한다). 따라서 **자격이 살아 있는 동안** 같은 에피소드 안에서 세대가
  한 바퀴 돌 수 없다. 카운터와 취소 상태는 축에 귀속하며 다른 축의 clear로 초기화되지 않는다.
- 에피소드 간 재사용(ABA)은 **운용 전제**로 막는다. 여기서 에피소드는 호스트의 5 s 복구
  에피소드가 아니라 **보드 축의 래치 에피소드**다(호스트 에피소드 하나에 세대가 다른 RESUME이
  여러 번 나갈 수 있다, `chassis_manager.py:1105-1109`). 보드 래치 에피소드는 guarded RESUME 커밋
  또는 수동 clear로 끝나고, 호스트는 한 래치 에피소드에 축별 RESUME을 1회 보낸다. 과거 RESUME이
  같은 ID(`node<<5|0x07`)의 이후 프레임들이 전달되는 동안 남아 있으려면 같은 ID 프레임 순서 역전이
  필요하다. 이 전제는 장치가 검사할 수 없으며, **Jetson socket→qdisc→드라이버→컨트롤러 전체 경로의
  같은 ID 순서 보존(mailbox 포화·재전송·bus-off 복구 포함) 실측을 자동 재개 배포의 인수 게이트**로
  둔다. 미확인이면 patch4 펌웨어는 배포하되 호스트 자동 재개 의존 운용은 보류한다.

### D5. 재초기화 요청과 래치

- 단일 helper `request_reinit(manual)`: 한 critical section에서 `reinit_requested_=true`, 각 축
  `can_recovery_in_progress_=true`, manual이면 자격 취소와 `episode_revoked_=true`. 그 뒤
  `sem_can`을 release한다. **PWM disarm·`requested_state_` 변경은 하지 않는다.**
- 래치(`latch_can_fault`)는 patch3과 같이 서버 스레드(osPriorityNormal)에서만 건다. 축 스레드는
  더 높은 우선순위로 `wait_for_current_meas()`에서만 양보하므로, 상태 검사와 `do_checks()`
  사이에 래치가 끼어들어 `ERROR_MOTOR_DISARMED`(0x20)가 붙는 경합이 없다(`axis.hpp:198`,
  `axis.cpp:172`). 자격은 에피소드 첫 래치에서 `기존 건강 CLOSED_LOOP 조건 && !episode_revoked_`로
  정한다.
- `clear_errors()`: 자격 취소와 `episode_revoked_=true`를 먼저 하고,
  `in_progress || (latched && (state≠IDLE || requested∉{UNDEFINED, IDLE}))`이면 거절한다.
  latch 전 pending clear도 거절되고 서버가 자격을 되살리지 못한다.
- `safety_critical_arm_motor_pwm()` 조건에 `!can_recovery_in_progress_`를 추가한다.
- `episode_revoked_`는 래치 정상 해제(수동 clear 성공, guarded RESUME 커밋)와 MCU 재시작 때 해제한다.
- 요청→래치 지연은 서버 응답시간이다(실측 대상). 이 구간의 CAN 소유 축은 D0 상한을 추가로
  가진다. USB 전용 축은 D0 대상이 아니다.

## 3. 수치 근거

| 값 | 근거 |
|---|---|
| bxCAN RX FIFO 깊이 3. `ReceiveFifoLocked=DISABLE`이라 overrun 때 마지막 저장 메시지가 새 메시지로 덮인다 | ST RM0090 §32.7.3, `Board/v3/Src/can.c:75` |
| CAN 500 kbps = 42 MHz ÷ (prescaler 4 × 21 TQ). 소스 기본값은 250K이므로 NVM baud가 500K여야 한다 | `interface_can.cpp:209`, `interface_can.hpp:25` |
| 축 제어 루프 8 kHz(125 µs) | `Board/v3/Inc/main.h:69-75,171-172` |
| RTOS tick 1 kHz, uint32 → wrap 약 49.7일 | `cmsis_os.c:227-234`, `FreeRTOSConfig.h:101,106` |
| 축 스레드 High/High+1 > CAN 서버 Normal | `board_config_v3.h:81,125`, `interface_can.cpp:87` |
| 500 kbps 8 B 표준 프레임 약 0.22–0.27 ms | 108 bit + 최대 stuff 24 bit + IFS, 2 µs/bit |
| ABOM 복구 최소 1408 bit = 2.816 ms | 11 recessive bit × 128회 |
| 6축 arm·확인 예산 150 ms < T_grace 500 ms | `chassis_manager.py:625` |
| 호스트 제어 tick 20 ms(50 Hz) ≪ T_cmd 300 ms | 무장 중 매 tick `SET_INPUT_VEL` (`corner_module.py:249`, `drive_odrive_can.py:395-400`) |
| 패드 송신 30 Hz(≈33 ms) | `remote_operation_client.py:32,511` |
| 호스트 수동입력 TTL 300 ms < T_grace 500 ms | `chassis_manager.py:1005`. 자동 재무장 후 WAIT_INPUT에서 새 입력이 없으면 호스트가 먼저 취소 |
| 호스트 피드백 freshness 200 ms, 안정 200 ms, 에피소드 5 s | `can_recovery.py:49-52,100`, `chassis_manager.py:1085` |
| 세대 시도 상한 K=16 < 256 | D4. 16 자체의 운용 적합성은 도출값이 아닌 후보값 |

## 4. 수용한 한계

1. D0 만료(호스트 루프 정지 300 ms 초과 등)는 콘솔 시동 재실행으로 재개한다. 보드 전원 재인가는
   필요 없다.
2. 보드 래치 없이 200 ms를 넘는 통신 공백은 기존 호스트의 보수 동작대로 자동 재개되지 않는다
   (stale 축은 자기 eligible 복구 출처가 필요, `can_recovery.py:70,87`).
3. CAN arm + USB setpoint 혼용은 미지원이며 만료로 정지한다.
4. 300/500 ms와 K=16은 후보값이며 벤치에서 확인한다.
5. 이 패치는 초기 HAL 오류의 물리 원인을 밝히지 않는다. 오류를 정지로 바꾸지 않을 뿐이다.
6. D0 만료는 루프 종료와 PWM 차단(상 floating, `low_level.cpp:108-118`)이다. 지상 속도 0이나
   제동거리를 보장하지 않는다.
7. D0 보호는 patch4가 적용된 보드에만 있다. node11/12가 패치되기 전에는 로버 전체의 수신 만료
   정지를 주장하지 않는다.
9. **CAN으로 arm하고 setpoint를 300 ms보다 길게 끊는 기존 도구는 patch4 보드에서 만료된다**
   (`ERROR_WATCHDOG_TIMER_EXPIRED`, 0x800). 2026-10-15 확인: motor_gui `odrive_can` 트랙은
   사용자 조작 때만 setpoint를 보내고(`motor_gui/backend/transport/odrive_can_device.py`),
   `motor_control/drive/bl70200/can_drive_test.py`는 arm 뒤 한 번 보내고 2 s씩 대기한다.
   이 오류를 `enable_watchdog`이나 하드웨어 문제로 오진하지 않는다. 두 도구의 주기 재송신은
   후속 작업이며, CAN 캘리브레이션(`can_calibrate_all.py`, CLOSED_LOOP 미진입)은 영향이 없다.
   운용 경로(ChassisManager)는 무장 중 50 Hz로 setpoint를 보낸다.
8. 필터는 소프트웨어 수신 부하만 줄인다. 버스 점유·물리 오류는 남으며, 그로 인한 자기 제어 손실은
   D0 정지, 피드백 200 ms 초과 공백은 기존 호스트 정지로 이어진다.

## 5. 시험·인수

### 소스 하네스 (RED → GREEN)

- 실제 `HAL_CAN_IRQHandler`를 ESR/RF0R/MSR/IER/TSR 레지스터 모형과 함께 컴파일하고, 가상 시각과
  FIFO 깊이 3·도착 시간열을 모형화한다(overrun은 마지막 저장 메시지 덮어쓰기로 모형화). Axis
  상태머신 dispatcher와 CLOSED_LOOP 함수, PWM arm 함수도 추출해 결합한다. STM32 실행 검증과
  구분한다.
- patch3에서 실패해야 하는 시험: LEC 1회·overrun·EWG/EPV/BOF 사건 → 래치 없음; 다른 node 프레임
  필터 거부; 복구 성공 직후 4프레임 → 재복구 없음; latch 전 pending clear 거부·자격 취소 유지;
  pending 중 USB arm 거부; CAN 소유 축 setpoint 단절 → 300 ms 만료(`enable_watchdog=false`);
  query·heartbeat·ACK만 있음 → 만료; CAN·USB state8 재요청 → 마지막 motion 후 300 ms 만료;
  `startup_closed_loop_control=true` + CAN state2 → 진입 후 500 ms 만료; 캘리 포함 startup → 진입
  시각 기준; 에피소드 내 17번째 시도 → 자격 취소, 256·512번째 시도(세대 wrap)에도 자격 false 유지,
  다른 축 clear가 이 축 카운터를 초기화하지 않음; 범위 밖 node의 heartbeat·응답 TX가 HAL에 도달하지
  않음(STD: `0x40`·77·`0x0800000D` 거부, EXT: `0x0800000D` 거부·`0x40`·77 허용, 32개 cmd 전부);
  tick wrap을 가로지르는 만료 판정; 정보성|PARAM 혼합 `0x00200218` → 정보성 계수 + PARAM 고착.
- 영향 없음 확인: USB 전용 운전, CAN 캘리 전용 요청, CLOSED_LOOP 사전검사 실패 후 USB 운전, IDLE
  진입과 동시 도착한 CAN state8의 소유권 유지, CAN IDLE 요청.
- 회귀: API 오류|LEC 혼합 → 고착; ISR이 재초기화를 요청하지 않음; 기존 86개 중 patch 식별자·
  세대 기대값은 새 계약으로 명시 갱신한다.
- 음성 대조: 각 신규 계약을 하나씩 깨뜨린 변이가 지정 시험에서 실패한다.

### 빌드

`build_firmware.py` 고정 이미지로 Cortex-M4 빌드, Config_t/NVM 직렬화 불변 확인.

### 실물 (별도 사용자 승인 후, 13/14 보드 단일 → 15/16 확대)

0. **NVM 인수 조건(플래시 전과 부팅 후, USB로 3장 모두 읽기)** — 하나라도 어긋나면 해당 보드
   배포 중단: 모든 `startup_*`=false(D0 필수 전제), CAN baud 500K, protocol Simple, 각 축 node가
   형식별 범위 안, **세 보드 6축의 node 전역 중복 없음**(보드 내부 검사 `can_simple.cpp:29-40`만으로는
   보드 간 중복을 못 잡는다), heartbeat 주기, `enable_watchdog` 값 기록.
1. 기존 guarded DFU 절차: 정확한 serial, 1 MiB 2회 백업 대조, application 영역만 기록, NVM·설정·
   캘리 읽기 대조.
2. 바퀴를 든 벤치에서 10모터 실트래픽 10분: `recovery_count` 증가 0·래치 0, 정보성 계수 기록.
3. Jetson 가지만 분리하고 다른 노드 ACK 유지, 0속도 CLOSED_LOOP → D0 만료 정지와 시간 측정.
4. 보드 단독 고립, 수동 재초기화 → 래치 → guarded RESUME 경로.
5. USB 스키드 경로 영향 없음.
6. D4 인수 게이트: Jetson CAN 송신 경로의 같은 ID 순서 보존 실측.
7. 호스트 송신 큐 체류시간: 정상·버스 오류 부하에서 큐 깊이와 setpoint 지연을 측정해 D0 기준
   시각(보드 수락)과 호스트 정지 시각의 차이를 기록한다.
8. 6축 arm에서 첫 축 arm부터 첫 setpoint까지의 실제 시간(150 ms 예산은 하드 상한이 아님,
   `chassis_manager.py:625-639,664-675`)이 T_grace보다 충분히 작은지 측정한다.
9. 비영속도 실주행·지상 제동은 별도 인수로 남긴다. node11/12 적용은 이 인수 뒤 별건이다.

## 6. 설계 검토 기록

- **Codex 적대적 검토(같은 스레드, 2026-10-15) — 5회 반복 후 수렴.**
  1. 초안: 원래 lock 범위 정정, D0 부재(P1), TX 완료 표본 오판(P2), 에피소드 간 세대 재사용(P2),
     latch 전 pending clear(P2), 래치 없는 재연결(P2) → 전부 반영.
  2. 2차: USB arm 축의 CAN motion으로 D0 우회(P1), 세대 고정이 호스트 재검증 무력화(P2),
     IRQ 즉시 disarm이 `MOTOR_DISARMED` 추가(P2) → 소유권 재정의, 시도별 세대 복귀, 서버 전용 래치.
  3. 3차: state8 재진입으로 만료 연장(P1), 사전검사 실패 후 pending 잔존(P2) → 세션 경계를 IDLE
     실행으로 재정의.
  4. 4차: CAN state2 startup 경로의 owner 누락(P1) → 비-IDLE CAN 상태 요청 전체를 세션 시작으로,
     유예는 첫 CLOSED_LOOP 진입 기준.
  5. 5차: "새 P1/P2 없음 — 수렴". D4 순서 보존 실측 게이트 유지 조건부.
  - patch3 검토의 R2·R3 심각도는 Codex가 P1로 냈다가 근거 확인 후 P2로 정정했다.
- **원자료 재계산(Claude):** 필터 word/mask를 bxCAN 레지스터 배치(STID 31:21, EXID 31:3, IDE bit2,
  RTR bit1)에서 직접 계산해 표와 일치 확인. ABOM 시간, 프레임 시간, 3절 호스트 수치를 각 파일에서
  다시 읽어 확인. 패치 소스 시험 86/0, 원본 4/82를 재현(고정 ZIP SHA256 일치).
- **독립 검토(새 Codex 스레드, 6절을 뺀 사본만 제공).** 조건부 보류 판정. 지적과 처리:
  - P2 범위 밖 node의 송신 별칭(STD node 77 heartbeat가 node13과 같은 프레임) → D2에 송신 측 범위
    검사 추가. 재검토에서 ID 생성 단계 uint32 wrap(node `0x0800000D`) 반례(P2) → ID 생성 전 node 검사
    helper를 추가해 두 겹으로 막음.
  - 3회째 재검토: "새 P1/P2 없음 — 수렴"(운용 전제·인수 게이트 유지 조건). STD/EXT 경계 기대값을 구분하라는
    지적을 시험 항목에 반영.
  - P3 K=16 설명이 세대 wrap과 자격 재사용을 혼동 → D4를 축별 포화 카운터·자격 비복구로 재서술.
  - P3 혼용 정지 시간 300/500 구분 → D0 정정.
  - 미기재 전제: 부팅 자동 시작이 D0 밖(→ `startup_*`=false를 D0 필수 전제·NVM 인수 조건으로),
    node 전역 중복·범위·baud 500K(→ 5절 NVM 인수 0번), tick 단위·wrap(→ D0 시계 명시), 큐 지연으로
    기준 시각이 보드 수락임(→ D0 명시·실물 7번), 150 ms 예산이 하드 상한 아님(→ 실물 8번),
    보드/호스트 에피소드 구분(→ D4 정정), FIFO 덮어쓰기 동작(→ 하네스 모형), D0 만료 ≠ 물리 정지와
    node11/12 미보호(→ 4절 6·7).
  - 수치 재계산: 필터 word/mask, 프레임 시간, ABOM, 정보성 mask `0x7FF`, 500 kbps 분주를 독립적으로
    맞다고 확인.
