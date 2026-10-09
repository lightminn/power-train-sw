# MKS CAN reliability patch3 — guarded RESUME

2026-09-10. 승인 범위의 펌웨어 구현·소스 시험·독립 리뷰·ARM 후보 빌드를 완료했다. 후보를 동결했고 root에 SHA-256을 전달했다. 이 작업자는 Jetson·하드웨어·flash·host 코드·NVM 설정·공유 문서·git commit/push를 조작하지 않았다.

## 최종 후보

| 항목 | 결과 |
|---|---|
| BIN | `build-patch3/ODriveFirmware.bin`, 248988 bytes |
| BIN SHA-256 | `022ac8f148501c084748336f7a65126ab2f1d96c381a4c95bc2079deaa497b86` |
| ELF | `build-patch3/ODriveFirmware.elf`, 1102688 bytes |
| ELF SHA-256 | `a709f42074d2afaafb8982e08dd234804b4deacfe1f588ce1359382b721126fe` |
| text / data / bss | 247352 / 1580 / 136064 bytes |
| `reliability.patch` SHA-256 | `9b0e7b271d10b824e7edd70ae8b540f61e9b018a73f592928eb020de6d52a2ca` |
| Build manifest SHA-256 | `5d0530017f384d1924762a3231c56748c141555c6fd8cb6c567f815e17093126` |
| Source manifest SHA-256 | `3b27bb658d5edd968022de59a4f19f8de0a365e19a263cfdeae9ec1ba2562d6e` |
| 식별 | firmware 0.5.1-dev, unreleased=1, Fibre `can.reliability_patch=3`, heartbeat marker `0xA3` |

이 보고서의 상대 경로는 `/tmp/mks-can-auto-resume-20260910/` 기준이다. 고정 ZIP은 `/tmp/can-usb-diagnosis-20260909/mks-v36-fw051.zip`, SHA-256 `1c9ff347f997bbbf8cb693cdb44d93fe6f6ef992aefde60b6a65b0e1fa25a777`이다. 실제 Docker image ID는 `sha256:977d830e78e15bf17d84d16f17b1c7eb5ad4a6eb4a79cc561cd5221a88e13258`이며 gcc-arm-none-eabi 10.3.1 / tup 0.7.8 / MKS v3.6-56V / native USB / ASCII UART / release optimization 설정을 사용했다.

`patch3-build-identity-check.json`은 검토 소스 388개와 빌드 준비 manifest의 파일별 해시가 모두 일치함을 기록한다. patch·source manifest·BIN·ELF를 다시 해시했고, version generator의 patch3 출력도 build.log에서 확인했다. 독립 리뷰 fingerprint의 두 변경 Firmware 파일 및 하니스 해시도 같은 최종 파일을 가리킨다. 서로 다른 빌드 환경의 바이너리 동일성이나 원래 설치된 제조사 바이너리와 ZIP의 동일성은 입증하지 않았다.

기존 `build/`, `build-initial/`, `patch2-source/`와 patch2 증거는 덮어쓰지 않았다. 기존 patch2 BIN SHA `0b42bfdbaa0a377df5b0270a8e79b014d551aac649cbd5b323b35033343a68fc`도 마지막에 재확인했다.

## 결함과 최종 프로토콜

host가 최신 상태를 확인한 뒤 generic clear를 보내기 전에 새 motor fault 또는 explicit E-stop이 생기면 generic clear가 이를 지울 수 있었다. 최초 guarded clear 설계도 외부 manual clear가 그 사이 실행되면 guarded clear가 거절된 후 이어지는 ordinary arm이 성공하는 경합을 남겼다. root 및 host 구현자와 계약을 변경해 자동 경로를 하나의 guarded RESUME로 묶었다.

- opcode `0x07`, non-RTR, DLC8, little-endian `<IBBBB>(8, 0xA3, expected_generation, 0, 0)`만 자동 요청이다. bytes는 `08 00 00 00 A3 GG 00 00`이다.
- 같은 critical section에서 CAN 객체 존재, pending/reinitializing 없음, 현재 HAL error0, eligible+latch, progress=false, generation 일치, current IDLE, requested UNDEFINED/IDLE, axis error가 정확히 `0x4000`, motor/encoder/controller/sensorless error 모두0을 검사한다.
- 통과하면 기존 `clear_errors()`로 오류·자격·래치를 해제하고 입력 velocity/torque0·현재 position을 적용한 뒤, IRQ를 열기 전에 requested state8을 쓴다. 실제 PWM arm은 기존 Axis 상태머신이 수행한다.
- 거부하면 오류·하위 오류·자격·래치·입력·requested state를 바꾸지 않는다. ordinary arm으로 fallback하지 않는다. 호스트도 이 축에 별도 generic clear/ordinary arm을 이어 보내면 안 된다.
- 일반 state 명령은 기존 DLC4 또는 zero-padded DLC8을 유지한다. 비zero suffix는 위 정확한 guarded RESUME만 허용한다. 잘못된 magic/state/reserved/DLC/RTR은 거부한다.
- opcode `0x18`은 explicit manual zero8 또는 legacy DLC0 clear만 유지한다. 비zero payload와 초기 제안의 A3+generation clear도 거부한다. 분리된 automatic clear API는 없다.
- 영향받지 않은 error0 peer의 자동 재무장에서는 호스트가 clear 자체를 생략한다. 이 host 계약 구현·실행 검증은 별도 담당자의 범위다.

heartbeat bytes0–3 axis error와 byte4 state는 그대로다. byte5=A3, byte6 bit0 latch / bit1 progress / bit2 eligible, byte7 uint8 generation이며 generation은 clear 후 유지되고 MCU reset 때0이다. patch2/A2는 guarded RESUME가 없으므로 자동 경로 대상으로 인정하면 안 된다.

## 변경 범위와 보존 경계

patch2 대비 준비된 vendor 소스의 차이는 정확히 세 파일이다.

1. `Firmware/communication/can_simple.cpp`: guarded RESUME, manual clear payload 제한, A3 heartbeat.
2. `Firmware/communication/interface_can.hpp`: read-only patch ID2→3.
3. `tools/odrive/mks_reliability_version.py`: patch3 표시. 생성되는 0.5.1-dev/unreleased version 값은 동일하다.

`Axis::Config_t`·`ODriveCAN::Config_t` 선언은 동일하며 `axis.hpp`, `axis.cpp`, `low_level.cpp`, `low_level.h`, `main.cpp`, `nvm_config.hpp`, `interface_can.cpp`, Fibre YAML도 patch2와 byte-identical이다. RAM/NVM 설정·watchdog·HAL 오류 정책·복구 backoff·PWM disarm은 바꾸지 않았다. 기존 초기 source replay 시험은 두 Config_t를 고정 제조사 ZIP과도 비교한다.

guarded RESUME callback은 watchdog을 feed하지 않는다. 기존 CLOSED_LOOP 진입 feed와 활성 제어 timeout, IDLE exemption/latched expiry 보존은 유지한다. 복구 자체가 자율적으로 arm하거나 과거 setpoint를 재생하지 않는다. 동작 개시는 호스트가 보낸 새 명령의 결과다.

## 실행 결과

저장소 루트에서 conda Python과 `PYTHONDONTWRITEBYTECODE=1`, `TMPDIR=/tmp/mks-can-auto-resume-20260910`을 사용했다.

| 실행 | 결과 | 로그 |
|---|---|---|
| 구현 전 guarded clear 계약 대 patch2 | 31 PASS / 44 FAIL | `patch3-red-against-patch2.log` |
| 구현 전 atomic RESUME 계약 대 patch2 | 44 PASS / 37 FAIL | `patch3-resume-red-against-patch2.log` |
| 최종 하니스 대 patch2 | 49 PASS / 37 FAIL | `patch3-final-red.log` |
| 최종 하니스 대 patch3 | **86 PASS / 0 FAIL** | `patch3-final-green.log` |
| 동일 최종 하니스 대 원본 vendor | 4 PASS / 82 FAIL | `patch3-vendor-red.log` |
| firmware preparation/build Python tests, 실제 ZIP 포함 | **6 PASS** | `patch3-preparation-green.log` |
| 안전 조건 제거 변이 | **9개 모두 지정 반례 FAIL** | `patch3-mutations.json`, `patch3-mutation-*.log` |
| 독립 리뷰 고정 하니스 대 patch3 / patch2 | 86 PASS / 0 FAIL, 49 PASS / 37 FAIL | `patch3-independent-source-tests.log`, `patch3-independent-patch2-negative.log` |
| ARM 전체 빌드 | PASS | `build-patch3/build.log` |
| `git diff --check -- firmware/mks_odrive` | PASS | 실행 출력 없음 |

최종 하니스는 기존26개 + provenance25개 + guard/manual boundary35개다. patch2 음성 대조37개 실패 중25개는 새 A3/version 부재,12개는 수락·거부·원자성·수동 clear 경계 결함이다. 기존 latch만으로 이미 거부되는 음성 시나리오도 있으므로 신규 시험 전부가 patch3 고유 검출기라고 주장하지 않는다. 초기 구현은 heartbeat에 A2가 남아57 PASS/24 FAIL이었고 이를 수정했다(`patch3-initial-marker-mismatch.log` 보존).

대표 명령:

```bash
/home/light/anaconda3/bin/python firmware/mks_odrive/tests/run_source_tests.py \
  /tmp/mks-can-auto-resume-20260910/patch3-source
MKS_VENDOR_ZIP=/tmp/can-usb-diagnosis-20260909/mks-v36-fw051.zip \
  /home/light/anaconda3/bin/python -m pytest firmware/mks_odrive/tests -q -p no:cacheprovider
/home/light/anaconda3/bin/python firmware/mks_odrive/build_firmware.py \
  --zip /tmp/can-usb-diagnosis-20260909/mks-v36-fw051.zip \
  --output /tmp/mks-can-auto-resume-20260910/build-patch3
```

마지막 build 명령은 완료 기록이다. 같은 출력 경로를 재실행하면 기존 후보 보존을 위해 거부한다. 변이 생성/실행 기록은 `validate_patch3.py`에 있고 scratch 변이는 실제 배포 소스나 patch 파일로 사용하지 않는다.

9개 변이는 exact axis error, eligibility, generation, pending CAN, HAL error, motor error 조건 제거와 clear/request 사이 critical 분할, reserved-byte 검증 제거, manual clear payload 제한 제거다. 실제 source scenario가 각각 기대한 오류로 실패했다. 외부 manual clear 후 늦은 RESUME, explicit E-stop, 새 세대·HAL 오류, 하위 오류, ISR 복원 뒤 새 fault, 잘못된 payload, 정상 수동 state4/state8/clear0/clear8을 직접 실행했다.

독립 리뷰는 `patch3-independent-review.md`에 있다. 검토자는 실제 두 버전 및 NVM/IRQ source 근거와 고정 하니스를 다시 확인했고 추가 source blocker를 발견하지 못했다. 해당 의견과 로컬 테스트/파일 해시를 대조했으며 불일치는 없다.

## 남은 검증과 별도 관측

소스 하니스는 실제 CAN/Axis/PWM 함수 본문을 실행하지만 HAL/RTOS/레지스터/스케줄링 경계가 결정적 더블이다. PRIMASK 실측 시간·실제 MCU 실행·CAN 전송·모터 제동·flash readback/NVM 보존·설치 ROS E2E를 대신하지 않는다. TX 경계 관측도 송신 시도와 물리 송신을 구분한다. 하드웨어 배포와 정지 검증은 root가 이 동결 SHA로 수행한다.

root가 전달한 **15/16 patch1 USB 관측**은 별도 미확정 진단으로 남긴다: current HAL error0, last HAL error512(`0x200`, RXFIFO0_OVERRUN), history536, recovery_count3464, failure_count0, tx_drop2632, 양축 IDLE/error0/latchfalse. 13/14의 전달된 before count는10이었다. 관측 기간·트래픽 조건·원인을 이 작업자가 직접 검증하지 않았으며 숫자 차이만으로 오류율이나 HALL/부하/하드웨어 원인을 단정하지 않는다. root는15/16 reboot 후 정상이라고 보고했다. 불필요 RX 프레임 또는 잦은 reset이 안정성에 영향을 주는지는 별도 진단 항목이다. patch3는 guarded RESUME 경합만 수정하며 HAL 오류를 무시하거나 필터·복구 정책을 넓게 바꾸지 않았다.
