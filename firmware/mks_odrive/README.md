# MKS ODrive v3.6 CAN reliability patch 1

고정한 Makerbase **0.5.1 소스**에 적용하는 재현 가능한 패치다. 실물에서 읽은
`0.5.1 unreleased` 바이너리와 제조사 ZIP의 동일성은 **입증되지 않았다**.
아래 호스트 시험은 실제 소스를 실행하지만 STM32 실행·IRQ 지연·모터 제동 검증은 아니다.

## 소스 준비와 실행 시험

필요 도구: Python 3, GNU patch, C++17 g++. 소스 준비는 네트워크를 사용하지 않는다.
`--dest`는 존재하지 않는 경로여야 하며 기존 작업을 덮어쓰지 않는다.

```bash
python firmware/mks_odrive/prepare_source.py \
  --zip /path/to/mks-v36-fw051.zip --dest /tmp/mks-original --unpatched
python firmware/mks_odrive/tests/run_source_tests.py /tmp/mks-original
# 기대: 4 passed, 22 failed (수정 계약에 대한 원본의 RED)

python firmware/mks_odrive/prepare_source.py \
  --zip /path/to/mks-v36-fw051.zip --dest /tmp/mks-patched
python firmware/mks_odrive/tests/run_source_tests.py /tmp/mks-patched
# 기대: 26 passed, 0 failed
python firmware/mks_odrive/tests/test_prepare.py --zip /path/to/mks-v36-fw051.zip
# 기대: 준비 도구 3개 통과 + 새 추출/패치 소스에서 26개 통과
```

원본 ZIP SHA256:
`1c9ff347f997bbbf8cb693cdb44d93fe6f6ef992aefde60b6a65b0e1fa25a777`.
[제조사 고정 출처](https://github.com/makerbase-motor/MKS-ODrive/tree/e15782976ae93d42b1f0648ceec96503141a343b/Firmware/ODrive_V3.6).
`mks-source-manifest.json`은 ZIP·패치 해시와 원본/패치 파일별 해시를 남긴다.
벤더 소스 전체와 기존 바이너리는 저장소에 복제하지 않는다.

시험은 `interface_can.cpp`, `can_simple.cpp`의 전체 구현 및 실제 헤더 선언,
원문 `HAL_CAN_AddTxMessage`, Axis의 watchdog/clear/latch/error-check 함수,
실제 PWM arm/disarm 함수를 컴파일한다. RTOS 스케줄링·레지스터·FIFO·모터 주변장치가
경계 stub이다. 마지막 슬롯 확인 직후의 선점은 PRIMASK 해제까지 지연되며, HAL 상태
고장은 IRQ 마스킹과 무관하게 주입한다. `get_watchdog_reset()`은 설정 계산 경계이며
호출 횟수와 결정적인 작은 tick 수를 제공한다. 실제 watchdog 감소·만료 코드는 원문이다.

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
출력을 성공으로 취급하지 않는다. 고정 입력과 기본 이미지의 확인값은 ELF 1,102,364 bytes
(`fd75b613…22d2`), BIN 247,880 bytes (`75678094…c710`),
text/data/bss/dec = 246244/1580/136064/383888이다.

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
fw_version_unreleased=1**을 생성하고 식별자는 `can.reliability_patch == 1`이다.
이 경로는 ZIP·벤더 소스·바이너리를 저장소에 복제하지 않으며 flash 동작을 전혀 수행하지 않는다.

## 런타임 계약

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
  각 poll은 `CAN_TIMEOUT_VALUE=10 ms` 제한이 있다. 실패와 성공 복구 모두 10 ms 양보하며,
  실패하면 다음 유한 시도까지 정지 래치와 I/O 차단을 유지한다. HAL/RTOS tick 자체가
  멎는 고장은 이 소스 시험의 보장 범위가 아니다. PRIMASK 실행시간은 MCU에서 측정해야 한다.
- 재초기화는 CLOSED_LOOP를 요청하거나 ESTOP를 자동 해제하지 않는다. Axis가 실제
  IDLE이고 CAN 재초기화가 끝나야 명시적 `axis.clear_errors()`가 래치를 해제한다.
  clear는 입력을 다시 0/현재 위치로 지운다. 호스트는 **IDLE 관측 → 명시 clear → 새
  0속도/0토크 → 명시 arm** 순서를 따라야 한다. routine telemetry와 자동 PWM rearm,
  USB의 `axis.error=0`만으로는 래치를 우회하지 못한다.
- **새 zero 프레임의 순서는 호스트 계약**이다. 펌웨어가 clear 이후 별도 neutral ACK를
  강제하는 것은 아니다. clear 자체가 입력을 0/현재 위치로 지우며, 이후 명시 arm을
  허용한다. 적용 대상은 **모든 `startup_*` 설정이 false인 보드**로 제한하고 flash 전과
  부팅 후 대조한다. 래치는 RAM 상태이므로 전원·USB reboot·CAN Reset ODrive 명령으로
  MCU가 재시작하면 사라진다. 재부팅을 가로질러 래치를 보존한다고 주장하지 않는다.
  startup 자동진입이 켜진 보드는 이 운용 계약의 인수 대상이 아니다.

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
| `can.reliability_patch` | 이 패치의 정수 식별자 `1` |
| `can.hal_error` | 현재 STM32 HAL ErrorCode 값 |
| `can.last_hal_error` | 가장 최근에 기록한 0이 아닌 HAL 오류 snapshot |
| `can.hal_error_history` | 부팅 이후 기록한 HAL 오류의 OR |
| `can.tx_drop_count` | mailbox 부족/재초기화 배제/잘못된 내부 DLC의 송신 drop |
| `can.tx_error_count` | HAL 오류로 거절하거나 AddTxMessage가 실패한 송신 |
| `can.recovery_count` | USB 정비 요청을 포함한 재초기화 시도 수 |
| `can.recovery_failure_count` | HAL 재초기화가 실패한 시도 수 |
| `can.rx_discard_count` | 리셋 직전 관측한 오래된 FIFO 프레임 수 |
| `can.reinit_requested`, `can.reinitializing` | 대기/진행 중 상태 |
| `axisN.can_recovery_latched`, `axisN.can_recovery_in_progress` | 운전 금지 래치/복구 진행 |

counter는 RAM의 uint32이며 재부팅 때 초기화되고 wrap할 수 있다. `can.error`는 기존
중복 node-ID flag이므로 HAL 오류와 구분한다. 보드 backup·복구 경로와 실제 firmware
식별을 확보한 뒤 단일 보드 정지/복구/통신 단절 시험을 한다. 이 패치의 호스트 시험
통과만으로 flash 호환성·전기 안전·지상 제동 완료를 선언하지 않는다.
