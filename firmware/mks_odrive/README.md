# MKS ODrive v3.6 CAN reliability patch 4

고정한 Makerbase **0.5.1 소스**에 적용하는 재현 가능한 패치다. 실물에서 읽은
`0.5.1 unreleased` 바이너리와 제조사 ZIP의 동일성은 **입증되지 않았다**.
아래 호스트 시험은 실제 소스를 실행하지만 STM32 실행·IRQ 지연·모터 제동 검증은 아니다.
patch4의 설계와 검토 기록은 [patch4 설계](../../docs/specs/2026-10-15-mks-can-patch4.md),
구현 순서와 검증 로그는 [patch4 계획](../../docs/plans/2026-10-15-mks-can-patch4.md)을 따른다.

## 소스 준비와 실행 시험

필요 도구: Python 3, GNU patch, C++17 g++. 소스 준비는 네트워크를 사용하지 않는다.
`--dest`는 존재하지 않는 경로여야 하며 기존 작업을 덮어쓰지 않는다.

```bash
python firmware/mks_odrive/prepare_source.py \
  --zip /path/to/mks-v36-fw051.zip --dest /tmp/mks-original --unpatched
python firmware/mks_odrive/tests/run_source_tests.py /tmp/mks-original
# 기대: 8 passed, 126 failed (수정 계약에 대한 원본의 RED)

python firmware/mks_odrive/prepare_source.py \
  --zip /path/to/mks-v36-fw051.zip --dest /tmp/mks-patched
python firmware/mks_odrive/tests/run_source_tests.py /tmp/mks-patched
# 기대: 134 passed, 0 failed
python firmware/mks_odrive/tests/test_prepare.py --zip /path/to/mks-v36-fw051.zip
# 기대: 준비 도구 3개 통과 + 새 추출/패치 소스에서 134개 통과
python firmware/mks_odrive/tests/test_regen_patch.py   # MKS_VENDOR_ZIP 필요
# 기대: 2개 통과 (regen_patch.py가 작업 트리를 정확히 재현)
```

patch4의 계약은 patch3 소스(음성 대조)에서 **88 passed, 46 failed**를 확인했다. 계약마다
구현 전 RED를 보고 구현했으며, 다음 변이가 지정 시험에서 실패한다: 정보성 mask에서 RX_FOV 제외,
복구 성공 뒤 대기 복원, 반복 복구 백오프 제거, `take_informational_errors`의 critical 제거,
이동 전 node 검사 제거, STD 필터의 IDE 비교 제거, 래치 자격에서 `episode_revoked` 무시,
시도 상한 255, `run_control_loop`의 `watchdog_ok` 무시, 만료 비교 `>=`, 세션 시작 비멱등,
IDLE 진입이 대기 요청 무시. patch3의 provenance·guarded RESUME 계약도 함께 유지한다.
준비 시험의 전체 재생은 `--zip` 또는 `MKS_VENDOR_ZIP`이 필요하며, 미지정 시 해당 시험만
명시적으로 skip한다. 패치를 고칠 때는 벤더 소스를 직접 수정한 작업 트리에서
`regen_patch.py --original <원본> --work <작업 트리>`로 `reliability.patch`를 다시 만든다.

원본 ZIP SHA256:
`1c9ff347f997bbbf8cb693cdb44d93fe6f6ef992aefde60b6a65b0e1fa25a777`.
[제조사 고정 출처](https://github.com/makerbase-motor/MKS-ODrive/tree/e15782976ae93d42b1f0648ceec96503141a343b/Firmware/ODrive_V3.6).
`mks-source-manifest.json`은 ZIP·패치 해시와 원본/패치 파일별 해시를 남긴다.
벤더 소스 전체와 기존 바이너리는 저장소에 복제하지 않는다.

시험은 `interface_can.cpp`, `can_simple.cpp`의 전체 구현 및 실제 헤더 선언,
원문 `HAL_CAN_AddTxMessage`와 `HAL_CAN_IRQHandler`, 실제 CMSIS/HAL CAN 상수,
Axis의 watchdog/clear/latch/error-check·CAN 제어 세션 함수, 실제 `run_state_machine_loop`·
`run_control_loop`·`run_closed_loop_control_loop`·`run_idle_loop`, 실제 PWM arm/disarm 함수를
컴파일한다. 경계 모형은 다음과 같다: bxCAN 레지스터(IER/MSR/ESR/RF0R/TSR, 쓰기-1-클리어),
깊이 3 FIFO와 overrun 시 마지막 메시지 덮어쓰기, CAN1 소유 bank·mask/list 모드 필터, NVIC
차단 중 CAN IRQ 미실행(MspInit 경로에서만 재활성화), 이진 세마포어, 축 루프 대기마다 1 ms
진행하는 가상 tick. 캘리브레이션·엔코더·모터 갱신은 결정적 stub이며, `do_checks`/`do_updates`는
실제처럼 `check_for_errors()`로 루프를 끝내고 heartbeat를 보낸다. 마지막 슬롯 확인 직후의
선점과 ErrorCode 읽기 직후의 IRQ는 PRIMASK 해제까지 지연된다. 실제 watchdog 감소·만료
코드는 원문이다. heartbeat 데이터는 실제 송신 함수가 만든 패킷을 `write()` 진입 경계에서
관측한다.

## 빌드

`build_firmware.py`는 고정 ZIP을 새 임시 소스에 준비하고 패치를 적용한 뒤
**MKS ODrive v3.6-56V** 설정(`USB=native`, `UART=ascii`, `DEBUG=false`)으로 빌드한다.
출력 경로는 명시해야 하며 이미 존재하면 내용을 보존한 채 거부한다. 기본 이미지는 실제
성공한 Docker image ID `sha256:977d830e78e15bf17d84d16f17b1c7eb5ad4a6eb4a79cc561cd5221a88e13258`이다.
컨테이너 네트워크는 빌드 중 차단된다.

```bash
python firmware/mks_odrive/build_firmware.py \
  --zip /path/to/mks-v36-fw051.zip \
  --output /tmp/mks-v36-56v-build
```

출력은 `ODriveFirmware.elf`, `ODriveFirmware.bin`, 원본 준비 매니페스트,
`mks-build-manifest.json`, `build.log`이다. 매니페스트와 로그에는 ZIP·패치 해시,
실제 Docker image ID, 패키지·컴파일러 버전, 실행 명령, ELF section 크기와 산출물
크기·SHA256이 남는다. 생성된 tup 스크립트는 `bash -e`로 실행하므로 실패 뒤 오래된
출력을 성공으로 취급하지 않는다. **이전 patch1의 비교 기준**은 ELF 1,102,364 bytes
(`fd75b613…22d2`), BIN 247,880 bytes (`75678094…c710`),
text/data/bss/dec = 246244/1580/136064/383888이다.
patch4는 코드가 달라 이 값과 일치하지 않는다(2026-10-15 patch4 빌드: ELF 1,103,132 bytes, BIN 251,868 bytes). 빌드 매니페스트는 현재 패치 ID와
`reference_reliability_patch=1`을 구분하며, 현재 산출물의 크기·SHA256을 별도로 기록한다.

이미지가 없는 머신은 다음 Dockerfile로 대체 이미지를 만들고 `--image`로 지정할 수 있다.
Ubuntu 22.04 base digest와 직접 빌드 의존성 버전은 `Dockerfile.build`와
`build-packages.lock`에 고정돼 있다.

```bash
docker build -f firmware/mks_odrive/Dockerfile.build \
  -t powertrain-mks-fw:local firmware/mks_odrive
python firmware/mks_odrive/build_firmware.py \
  --image powertrain-mks-fw:local \
  --zip /path/to/mks-v36-fw051.zip \
  --output /tmp/mks-v36-56v-build
```

패키지 저장소 상태와 transitive dependency 때문에 Dockerfile 재빌드의 image ID나 산출물
바이트 동일성은 별도 입증 대상이다. 스크립트는 실제 비교 결과를 기록하며 환경 간 동일성을
가정하지 않는다. 패치의 version generator는 **0.5.1-dev /
fw_version_unreleased=1**을 생성하고 식별자는 `can.reliability_patch == 4`이다.
이 경로는 ZIP·벤더 소스·바이너리를 저장소에 복제하지 않으며 flash 동작을 전혀 수행하지 않는다.

## 런타임 계약

patch4가 더한 계약(설계 D0~D5)을 먼저 적고, patch3에서 이어지는 계약을 그 뒤에 둔다.

- **D1 오류 분류.** HAL `ErrorCode`의 정보성 비트 `0x000007FF`(EWG·EPV·BOF 사건·LEC
  STF/FOR/ACK/BR/BD/CRC·RX_FOV0/1)는 계수(`lec_error_count`·`rx_overrun_count`·`bus_off_count`·
  `error_passive_count`·`error_warning_count`)한 뒤 그 비트만 지우며, 재초기화·래치·정지를 하지
  않는다. 읽기→계수→지우기는 한 PRIMASK 구간에서 하고, IRQ 문맥(`HAL_CAN_ErrorCallback`)은
  재초기화를 요청하지 않는다. API 계열 비트(PARAM 등)가 남아 있거나 재초기화 중이 아닌데
  handle State가 LISTENING이 아니면 재초기화한다. 정보성 비트만으로는 송·수신을 막지 않는다.
- **D2 수신 필터와 송신 ID.** CAN1 bank0/bank1은 각 축 node만 받는다(STD id `node<<26`·mask
  `0xFC000004`, EXT id `(node<<8)|4`·mask `0xFFFFFF04`, RTR don't-care, FIFO0,
  `SlaveStartFilterBank=14`). 범위 밖 node는 bank를 비활성화한다. node 설정이 바뀌면 서버
  루프가 필터를 다시 쓴다(실패 시 재초기화). 송신은 ID를 만들기 전에 node 범위(STD ≤ `0x3F`,
  EXT ≤ `0xFFFFFF`)를 검사하고, `write()`도 STD ≤ `0x7FF`·EXT ≤ `0x1FFFFFFF`를 확인한다.
  어긋나면 HAL에 넘기지 않고 `tx_drop_count`로 센다.
- **D3 복구 뒤 대기.** 복구에 성공하면 대기 없이 RX 서비스로 돌아간다. 마지막으로 성공한
  복구로부터 10 ms 안에 다시 복구해야 하면 그 시도 전에 `osDelay(10)`으로 반드시 차단한다.
  실패한 시도도 10 ms 양보한다.
- **D4 시도 상한.** 축별 래치 에피소드(래치 설정~해제)의 복구 시도를 포화 카운터로 세고,
  17번째 시도부터 그 에피소드의 자동 재개 자격을 복구하지 않는다(세대는 계속 증가).
  에피소드 사이의 세대 재사용은 호스트 송신 경로의 같은 ID 순서 보존을 운용 전제로 막으며,
  그 실측은 자동 재개 의존 운용의 인수 게이트다.
- **D5 요청 표시.** 모든 재초기화 요청은 즉시 양축 `can_recovery_in_progress`를 세운다(수동이면
  자격 취소와 에피소드 취소 표시도). `clear_errors()`는 in-progress이거나 래치가 IDLE 전이면
  거절하되 자격은 먼저 취소하며, PWM arm도 in-progress 동안 거절한다. 래치(PWM disarm·입력 0·
  IDLE 요청)는 CAN 서버 스레드에서만 건다.
- **D0 CAN 제어 수신 만료.** CAN으로 시작한 제어 세션의 CLOSED_LOOP에서 유효 CAN motion
  setpoint를 마지막으로 수락한 뒤 300 ms(첫 setpoint 전에는 첫 CLOSED_LOOP 진입 뒤 500 ms)를
  넘기면 `ERROR_WATCHDOG_TIMER_EXPIRED`로 루프를 끝내고 실제 IDLE 경로로 PWM을 끈다
  (`enable_watchdog`과 무관, `control_deadline_count` 증가, 자동 재개 자격 없음). 세션은
  IDLE 이외의 CAN 상태 요청(guarded RESUME 포함)이나 CLOSED_LOOP 중 CAN motion 수락으로
  시작하고, 상태머신이 실제 IDLE을 요청 없이 실행할 때만 끝난다. state8 재진입은 타이머를
  갱신하지 않는다. USB로만 운전하는 축은 대상이 아니다. 기준 시각은 보드의 마지막 수락
  시각이며 호스트 송신 큐 지연만큼 늦어질 수 있다. 부팅 자동 시작은 CAN 세션이 아니므로
  **모든 `startup_*`=false가 D0의 필수 전제**다. 만료하면 입력(vel/torque 0, pos=현재 위치)도
  지워 이후 재무장이 이전 지령을 재생하지 않는다. CAN으로 arm하고 setpoint를 300 ms보다 길게
  끊는 기존 도구(motor_gui `odrive_can` 트랙, `motor_control/drive/bl70200/can_drive_test.py`)는
  patch4 보드에서 0x800으로 멈춘다 — 오진하지 말 것. 운용 경로(ChassisManager)는 50 Hz로 보낸다.

- `write()`는 짧은 PRIMASK 구간에서 TX 허용 여부·빈 슬롯·HAL 등록을 함께 처리한다.
  세 슬롯이 모두 차면 `0`을 반환하고 drop을 센다. 재시도·blocking mutex는 없다.
  HAL 실패도 `0`으로 반환하고 실제 error와 실패 횟수를 기록한다.
- CAN 서버가 모든 런타임 HAL 재초기화를 소유한다. USB `can.set_baud_rate(baud)`와
  `reinit_can()`은 **비동기 요청**이다. 유효 baud의 연속 요청은 마지막 값으로 합친다.
  같은 baud 요청도 양축 정지 래치를 거는 정비 동작이다. 호출 반환은 완료 ACK가 아니다.
  `reinit_requested == false`, `reinitializing == false`, 적용된 `config.baud_rate`,
  `recovery_count` 증가와 `hal_error == 0`을 시간 제한을 두고 확인한다.
- 재초기화 요청부터 TX/RX 진입을 막고, 서버가 양축 ESTOP 래치·입력 vel/torque 0·
  현재 encoder 위치·IDLE 요청·PWM disarm을 먼저 적용한다. pending TX abort와 CAN1
  RCC force/release reset이 과거 TX/RX를 제거한다. CAN IRQ 네 개만 disable한 뒤
  Stop 결과·HAL 오류를 보존하고 RCC reset → pending IRQ clear → HAL Init/filter/start를
  수행한다. 벤더 DeInit은 clock을 끈 뒤 MCR_RESET를 쓰므로 이 경로를 사용하지 않는다. `rx_discard_count`는 리셋 직전 FIFO
  관측 수이며 리셋 중 도착한 프레임의 정확한 총 폐기량을 뜻하지 않는다.
- HAL 재초기화와 RTOS 대기는 **IRQ를 마스킹하지 않는다**. 벤더 HAL Init/Stop/Start의
  각 poll은 `CAN_TIMEOUT_VALUE=10 ms` 제한이 있다. 양보 규칙은 위 D3를 따르며,
  실패하면 다음 유한 시도까지 정지 래치와 I/O 차단을 유지한다. HAL/RTOS tick 자체가
  멎는 고장은 이 소스 시험의 보장 범위가 아니다. PRIMASK 실행시간은 MCU에서 측정해야 한다.
- 재초기화는 CLOSED_LOOP를 요청하거나 ESTOP를 자동 해제하지 않는다. Axis가 실제
  IDLE이고 CAN 재초기화가 끝나야 명시적 `axis.clear_errors()`가 래치를 해제한다.
  clear는 입력을 다시 0/현재 위치로 지운다. 수동 재시작은 **IDLE 관측 → 명시 clear → 새
  0속도/0토크 → 명시 arm** 순서를 따른다. routine telemetry와 자동 PWM rearm,
  USB의 `axis.error=0`만으로는 래치를 우회하지 못한다.
- 자동 HAL 복구가 최초 래치 직전 **오류 없는 CLOSED_LOOP**를 중단한 축에만 자동
  재개 자격을 부여한다. Axis·motor·encoder·controller·sensorless 오류와 대기 중인
  상태 요청이 없어야 한다. 이미 IDLE이거나 IDLE 요청이 대기 중이면 자격이 없다.
  재시도는 기존 결정을 유지한다. 같은 baud를 포함한 유효 수동 재초기화 요청은
  enqueue 시 양축 자격을 즉시 취소하며 동시 HAL 오류보다 우선한다. CAN E-stop,
  `clear_errors()`(아직 IDLE이 아니라 거절된 clear 포함), MCU 재시작도 자격을 취소한다.
  이 자격만으로 펌웨어가 arm하거나 과거 입력을 재생하지 않는다.
  호스트의 전체 정지·새 피드백·guarded RESUME·새 수동 입력 계약은
  [자동 재개 사양](../../docs/specs/2026-09-10-can-auto-resume.md)을 따른다.
- 자동 재개는 **cmd `0x07`, DLC8, `<IBBBB` = `(8, 0xA3, expected_generation, 0, 0)`**
  하나로 요청한다. 같은 critical section에서 eligible+latch, !in-progress, IDLE,
  pending 상태 UNDEFINED/IDLE, 정확한 axis error `0x4000`, 일치하는 세대, 모든 하위
  motor/encoder/controller/sensorless error 0, CAN pending/reinitializing 없음과 현재
  HAL error 0을 검사한다. 통과하면 clear→입력 0/현재 위치→requested state8을 원자적으로
  커밋한다. 실제 PWM arm은 기존 Axis 상태머신이 담당한다. 거절은 상태·오류·입력을
  바꾸지 않고 watchdog도 갱신하지 않는다. IRQ 복원 뒤 새 오류는 기존 health check가
  보존한다. 호스트는 이 명령 뒤 별도 generic clear/일반 arm을 보내면 안 된다.
- cmd `0x18`은 수동 zero8/기존 DLC0 clear만 허용하며, 그 외 payload는 거부한다.
  분리된 guarded clear 명령은 없다. 일반 state 요청은 DLC4 또는 zero-padded DLC8을
  유지하되, 비zero suffix는 정확한 guarded RESUME만 허용한다. 외부 manual clear가
  먼저 실행되면 자격과 래치가 취소되므로 늦은 guarded RESUME는 arm을 요청하지 않는다.
  영향받지 않은 error0 peer의 자동 재무장에서는 호스트가 clear 자체를 생략해야 한다.
- **수동 경로의 새 zero 프레임 순서는 호스트 계약**이다. 펌웨어가 clear 이후 별도 neutral ACK를
  강제하는 것은 아니다. clear 자체가 입력을 0/현재 위치로 지우며, 이후 명시 arm을
  허용한다. 적용 대상은 **모든 `startup_*` 설정이 false인 보드**로 제한하고 flash 전과
  부팅 후 대조한다. 래치는 RAM 상태이므로 전원·USB reboot·CAN Reset ODrive 명령으로
  MCU가 재시작하면 사라진다. 재부팅을 가로질러 래치를 보존한다고 주장하지 않는다.
  startup 자동진입이 켜진 보드는 이 운용 계약의 인수 대상이 아니며, D0 만료도 적용되지 않는다.

## Watchdog와 호스트 입력

CAN watchdog 갱신은 유효한 Set_Input_Pos(8 bytes), Set_Input_Vel(8 bytes),
Set_Input_Torque(4 또는 8 bytes) 처리 이후에만 일어난다. RTR, 짧은 DLC, NaN/Inf,
알 수 없는 명령, heartbeat, telemetry query, clear/state 명령은 갱신하지 않는다.
두 float가 있는 입력은 둘 다 검증한 뒤 일괄 적용한다. 복구 래치 동안에는 motion
setpoint를 적용하거나 feed하지 않는다. 상태/설정/clear 명령도 RTR와 길이를 검증한다.

`IDLE`에서는 새 watchdog timeout을 만들지 않으며 기존 만료 오류는 남긴다.
활성 CLOSED_LOOP를 포함한 **IDLE 이외 상태**에서는 기존 watchdog 감소/만료 동작을
유지한다. 폐루프 진입 시 벤더의 기존 `watchdog_feed()`가 초기 유예를 제공한다.
캘리/센서리스/호밍에서도 watchdog을 enable하면 운용자가 적절한 갱신 경로를 마련해야 한다.
USB Fibre에서 input 값을 쓰는 것만으로는 watchdog이 갱신되지 않는다. USB 운용은
별도의 `axis.watchdog_feed()` API를 검증된 주기로 호출해야 한다. ASCII control은 벤더의
기존 feed 동작을 유지하므로 별도 경로 시험이 필요하다.

`enable_watchdog`, `watchdog_timeout`, startup·캘리 설정의 NVM 기본값과 Config_t 레이아웃은
변경하지 않는다. **300 ms는 벤치 후보값이며 인증된 운용값이 아니다.** 값 설정·NVM 저장은
별도 승인된 커미셔닝에서만 한다. query 트래픽만으로 활성 제어 watchdog이 유지되지
않는 것과 실제 출력 차단·제동·재기동 동작을 각기 검증해야 한다.

## 읽기 전용 Fibre 진단

| 속성 | 의미 |
|---|---|
| `can.reliability_patch` | 이 패치의 정수 식별자 `4` |
| `can.hal_error` | 현재 STM32 HAL ErrorCode 값 |
| `can.last_hal_error` | 가장 최근에 기록한 0이 아닌 HAL 오류 snapshot |
| `can.hal_error_history` | 부팅 이후 기록한 HAL 오류의 OR |
| `can.tx_drop_count` | mailbox 부족/재초기화 배제/잘못된 내부 DLC/범위 밖 node·ID의 송신 drop |
| `can.tx_error_count` | HAL 오류로 거절하거나 AddTxMessage가 실패한 송신 |
| `can.recovery_count` | USB 정비 요청을 포함한 재초기화 시도 수 |
| `can.recovery_failure_count` | HAL 재초기화가 실패한 시도 수 |
| `can.rx_discard_count` | 리셋 직전 관측한 오래된 FIFO 프레임 수 |
| `can.reinit_requested`, `can.reinitializing` | 대기/진행 중 상태 |
| `can.lec_error_count` | 정보성 LEC(stuff/form/ACK/bit/CRC) 사건 수 |
| `can.rx_overrun_count` | 정보성 RX FIFO overrun 사건 수 |
| `can.bus_off_count`, `can.error_passive_count`, `can.error_warning_count` | 정보성 bus-off·error passive·error warning 사건 수 |
| `axisN.can_recovery_latched`, `axisN.can_recovery_in_progress` | 운전 금지 래치/복구 진행 |
| `axisN.can_recovery_auto_resume_eligible` | 자동 HAL 복구가 중단한 건강한 폐루프 축의 재개 자격 |
| `axisN.can_recovery_generation` | 각 복구 시도마다 증가하는 uint8 세대, 255→0 wrap |
| `axisN.control_deadline_count` | D0 CAN 제어 수신 만료 횟수 |

heartbeat는 error의 little-endian 4바이트와 **byte4 axis state**를 그대로 유지한다.
byte5는 `0xA3`, byte6의 bit0/1/2는 각각 latch/in-progress/eligible이며 나머지 비트는 0이다.
byte7은 복구 세대다. error/state/flags/generation을 같은 짧은 critical section에서
snapshot하고 IRQ를 복원한 뒤 TX gate로 보낸다. 정상 부팅은 flags=0/generation=0이다.
clear는 자격·허용된 래치를 지우지만 세대는 유지하며, MCU 재시작 때 세대도 0이 된다.
마커 없는 기존 펌웨어나 guarded RESUME가 없는 patch2/A2의 `0x4000`으로 자동 재개를 허용하면 안 된다.

counter는 RAM의 uint32이며 재부팅 때 초기화되고 wrap할 수 있다. `can.error`는 기존
중복 node-ID flag이므로 HAL 오류와 구분한다. 보드 backup·복구 경로와 실제 firmware
식별을 확보한 뒤 단일 보드 정지/복구/통신 단절 시험을 한다. 이 패치의 호스트 시험
통과만으로 flash 호환성·전기 안전·지상 제동 완료를 선언하지 않는다.
