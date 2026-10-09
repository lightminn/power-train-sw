# CAN 자동 재개 firmware patch2 구현 결과 — 2026-09-10

**펌웨어 구현·실제 소스 회귀·고정 Cortex-M4 빌드 완료.** 최종 후보는
`/tmp/mks-can-auto-resume-20260910/build/ODriveFirmware.bin`이다. 이 작업자는
호스트 제어 코드, Jetson, USB/CAN 실물, NVM, git commit/push를 조작하지 않았다.
소스와 빌드의 완료는 호스트 자동 재개나 실제 모터 동작의 인수를 뜻하지 않는다.

## 최종 산출물

| 항목 | 값 |
|---|---|
| 펌웨어 | 0.5.1-dev, unreleased=1, `can.reliability_patch=2` |
| 보드/프로토콜 | MKS v3.6-56V, USB native, UART ascii, DEBUG=false |
| BIN 크기 | 248612 bytes |
| BIN SHA256 | `0b42bfdbaa0a377df5b0270a8e79b014d551aac649cbd5b323b35033343a68fc` |
| ELF 크기 | 1102364 bytes |
| ELF SHA256 | `88c534981fbe3ca50362b86cf78b53ac4baaec46ed64c666a05c46e6e90c9d62` |
| ELF text/data/bss | 246972 / 1580 / 136064 |
| 최종 reliability.patch SHA256 | `1683da2489246cc0f5ea3aa1952890b2abc32ff9413ceb39f9e8f319ca46afbc` |
| build manifest SHA256 | `c1b0622f0853f5907aa998a49d4cb881a95727c60c49a1c73867e9d4603a40cd` |
| source manifest SHA256 | `17f12de62792a0f8d88347050be202d2159cef8cee740682f66fba63b4d327a7` |
| vendor ZIP SHA256 | `1c9ff347f997bbbf8cb693cdb44d93fe6f6ef992aefde60b6a65b0e1fa25a777` |
| 빌드 image ID | `sha256:977d830e78e15bf17d84d16f17b1c7eb5ad4a6eb4a79cc561cd5221a88e13258` |

`build/mks-build-manifest.json`, `build/mks-source-manifest.json`, `build/build.log`에
전체 환경·명령·파일 해시가 남는다. 최종 source manifest의 **388개 파일 해시가
검토한 `patch2-source`와 모두 일치**한다. 최종 후보의 DFU application vector/크기
검사도 통과했다. 빌드 로그에서 compiler warning/error는 없고 patch2 표기를 확인했다.

첫 빌드 `build-initial/`도 보존했다. 이후 version generator의 안내 문구만 patch1→2로
고쳤으며 최종 빌드를 `build/`에 새로 수행했다. 두 빌드의 BIN/ELF 바이트는 동일했다.
다만 두 입력 패치의 안내 문구가 달랐으므로 이를 동일 입력으로 독립 빌드한 시험이라고
표현하지 않는다. Dockerfile 재구축·환경 간 바이트 동일성도 별도 미확인이다.

## 변경 범위

저장소 변경은 `firmware/mks_odrive/` 아래의 다음 파일로 한정했다.

- `reliability.patch`: 기존 고정 vendor ZIP에서 patch2를 만드는 통합 패치.
- `prepare_source.py`: 준비 매니페스트의 현재 patch ID를2로 기록.
- `build_firmware.py`: 현재 source patch ID를 매니페스트에 기록하고 기존 비교값을
  명시적으로 historical patch1 기준으로 표시. 기존 이미지/패키지 고정·출력 덮어쓰기
  거부·`bash -e` 빌드 경로는 유지.
- `tests/run_source_tests.py`, `tests/hal_boundary.hpp`, `tests/scenarios.cpp`:
  실제 Axis RAM 초기값/메서드와 CAN 구현을 사용하는 heartbeat/provenance 시험 추가.
- `tests/test_prepare.py`: 새 추출/패치 결과의 patch2 계약 확인.
- `README.md`: wire/Fibre 계약, 취소·세대 의미, 시험 수와 빌드 비교 기준 갱신.

patch1 대비 실제 vendor 코드 변경은 `Firmware/MotorControl/axis.hpp`,
`Firmware/communication/interface_can.cpp`, `interface_can.hpp`, `can_simple.cpp`,
`Firmware/odrive-interface.yaml` 및 version generator의 안내 문구다.
기존 patch1의 나머지 변경은 그대로 포함한다. 전체 vendor 소스는 저장소에 추가하지 않았다.

## 복구 출처 계약

Heartbeat bytes0–3은 기존 little-endian axis error, byte4는 기존 axis state다.
byte5=`0xA2`, byte6의 bit0=latch/bit1=in-progress/bit2=eligible, 나머지 비트는0이며
byte7은 uint8 generation이다. 이 정보를 같은 짧은 critical section에서 snapshot하고
IRQ를 복원한 다음 기존 TX gate로 보낸다. 기존 firmware의 `0x4000`만으로 자격을
추정할 수 없다.

최초 자동 HAL 복구 latch 직전에 다음 조건을 모두 만족한 축에만 자격을 부여한다.

- current_state=CLOSED_LOOP_CONTROL, requested_state=UNDEFINED.
- axis, motor, encoder, controller, sensorless error 모두0.
- 기존 CAN recovery latch가 없는 최초 판정.

이미 래치된 축의 반복 시도는 기존 자격 결정을 보존한다. IDLE·pending IDLE·기존 오류로
최초 거절된 축을 재시도가 승격하지 않는다. 정상 폐루프에서 자동 HAL 복구가 시작되어도
입력은0/현재위치로 지워지고 양축 PWM disarm과 IDLE 요청이 우선한다. 펌웨어는
자율 arm이나 cached setpoint 재생을 추가하지 않았다.

유효한 수동 `set_baud_rate()`/`reinit_can()`은 enqueue 때 양축 자격을 즉시 취소하고
manual origin을 sticky pending flag로 기록한다. 서버의 origin 판정과 양축 latch를 같은
바깥 critical section으로 묶어, 동시 수동 요청은 판정 전에 반영되거나 판정 뒤에 자격을
취소한다. 자동 HAL 오류 기록은 pending manual origin을 덮지 않는다. 해당 critical에는
기존 latch/PWM register 정지와 유한한 RAM 작업만 있고 HAL Stop/Init·RTOS 대기는 없다.

명시 CAN E-stop은 해당 축 자격을 취소한다. `clear_errors()`는 아직 복구 중이라 clear를
거절하는 경우에도 자격을 취소한다. 허용된 clear는 기존처럼 IDLE/복구완료 확인 뒤에만
stop latch를 푼다. 직접 `latch_can_fault()` 호출의 기본값도 noneligible다.
유효하지 않은 baud는 재초기화 요청 자체를 만들지 않는다.

generation은 각 실제 latch/복구 시도마다 증가하고255→0으로 wrap한다. clear는 세대를
유지하며 MCU 재시작의 RAM 기본값은 latch=false/progress=false/eligible=false/generation=0이다.
Fibre에는 `axisN.can_recovery_auto_resume_eligible`와
`axisN.can_recovery_generation`을 읽기 전용으로 추가했다.

## 실제 실행한 시험

저장소 `/home/light/ZETIN/robotics/power-train-sw`에서 실행했다.

```bash
export TMPDIR=/tmp/mks-can-auto-resume-20260910
/home/light/anaconda3/bin/python firmware/mks_odrive/tests/run_source_tests.py \
  /tmp/mks-can-auto-resume-20260910/patch1-source
# 26 passed, 25 failed; exit1 — patch1 음성 대조

/home/light/anaconda3/bin/python firmware/mks_odrive/tests/run_source_tests.py \
  /tmp/mks-can-auto-resume-20260910/patch2-source
# 51 passed, 0 failed

MKS_VENDOR_ZIP=/tmp/can-usb-diagnosis-20260909/mks-v36-fw051.zip \
  /home/light/anaconda3/bin/python -m pytest firmware/mks_odrive/tests -q
# 6 passed — 새 ZIP 추출/패치의 내부 실제 소스51건 포함

/home/light/anaconda3/bin/python firmware/mks_odrive/build_firmware.py \
  --zip /tmp/can-usb-diagnosis-20260909/mks-v36-fw051.zip \
  --output /tmp/mks-can-auto-resume-20260910/build
# Full Cortex-M4 build PASS; 출력 경로가 이미 있으면 보존하고 거부
```

기존26건을 유지한 채 신규25건을 추가했다. 최초 RED는 신규24건 상태에서
26pass/24fail이었고, dispatch 직후 수동 요청 경계를 추가한 최종 하니스에서는
patch1이26pass/25fail이다. 무패치 vendor도 같은 최종 하니스로4pass/47fail을 확인했다.

추가로 실제 소스의 다음 결함을 각각 임시 복사본에 주입해 음성 대조했다.

| 주입 결함 | 실제 실패한 핵심 시나리오 |
|---|---|
| 수동 origin을 무시하고 항상 automatic 처리 | `resume_manual` |
| 재시도 때 이미 내린 자격 판정을 다시 계산 | `resume_retries` |
| CAN E-stop의 자격 취소 제거 | `resume_estop_after` |
| clear_errors의 자격 취소 제거 | `resume_clear` |
| provenance 판정과 latch 사이에 IRQ를 먼저 복원 | `resume_manual_at_dispatch` |

각 변이는 실제 소스 하니스에서 exit1과 해당 실패를 확인했다. 변이는 최종 패치에 없다.
로그는 이 디렉터리의 `mutation-*.log`, `patch1-red.log`, `patch2-green.log`,
`vendor-red.log`, `preparation-green.log`에 있다. `git diff --check -- firmware/mks_odrive`
도 통과했다.

## 독립 검토와 NVM

Root가 별도로 배정한 독립 리뷰어의 `firmware-independent-review.md`를 확인했다.
리뷰어도 최종51건 PASS, patch1 음성25건 FAIL을 직접 실행했고 차단 결함을 찾지 못했다.
리뷰 뒤 C++ 구현 변경은 없으며 version generator의 출력 문구만 patch2로 맞췄다.

준비 시험은 실제 vendor ZIP과 Axis/CAN 두 Config_t의 전체 선언을 비교한다.
독립 리뷰는 patch1 대비 두 Config_t와 main.cpp의 저장 형식, nvm_config.hpp,
axis.cpp, low_level.cpp의 byte 동일성도 확인했다. 새 상태는 Config_t 밖의 RAM 필드다.
watchdog 활성 상태/IDLE 예외, feed 허용 프레임, 기존 PWM stop/rearm gate는 유지한다.
실제 보드의 NVM·307개 설정·캘리 보존은 root의 별도 flash/readback 게이트다.

## 남은 검증 범위

- 실제 HAL 오류·인터럽트 시간·전기적 PWM 차단은 이 호스트 소스 시험으로 인증하지 않는다.
- TX 경계의 heartbeat 관측은 패킷 **생성/송신 시도** 증거다. 복구 중에는 기존 gate가
  실제 HAL 전송을 막으므로 wire에서 flags7을 반드시 수신할 수 있다는 뜻이 아니다.
- 호스트는 eligibility/generation만으로 재개할 수 없다. 승인 사양의 여섯 축 정지·신선도·
  200ms 안정성·안전 조건·clear/zero/arm·재무장 후 새 수동 입력 계약이 따로 필요하다.
- MCU reset으로 래치를 유지하거나 자동 재개하는 계약은 없다. startup 설정/NVM은 바꾸지 않았다.
- 실제 flash·원본 rollback·보드별 patch2 확인·ROS/vcan·실물 zero-command 커미셔닝은 root 담당이다.
- 기존 설치 vendor 바이너리와 고정 ZIP의 동일성은 여전히 NOT PROVEN이다.

**작업 범위 내 미해결 blocker 없음. 최종 펌웨어 후보/소스는 동결해 root에 전달했다.**
