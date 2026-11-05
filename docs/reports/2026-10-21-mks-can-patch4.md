# MKS CAN patch4 실물 적용 — node13/14 보드

2026-10-21 Jetson에서 **serial `336A33523235`, node13/14, MKS v3.6-56V** 보드를
CAN patch3에서 **patch4**로 갱신했다. 계획 `docs/plans/2026-10-15-mks-can-patch4.md`
Task 10 Step 1에 해당한다. 설계는 `docs/specs/2026-10-15-mks-can-patch4.md`, 빌드 재현은
`firmware/mks_odrive/README.md`를 따른다.

이번 적용은 **USB 인수까지**다. CAN 버스는 끊긴 상태였고(Jetson 쪽 CAN 트랜시버만 연결),
실트래픽·D0 만료·송신 순서 시험(Step 2–7)은 하지 않았다. node15/16(`337733643235`)은
patch3, node11/12는 미패치 그대로다.

## 적용 이미지와 원본

| 항목 | 값 |
|---|---|
| 적용 BIN | 251,868 bytes, SHA256 `15462f7f64745b03d1347fccda8091cd34b795056fefee64f87e794dc8a4b3fc` (PR #17 병합본 빌드, x86·Jetson aarch64 소스 시험 134/0) |
| 원본 flash 1 MiB | SHA256 `f1e985fc3ae3c43f3deb372301e71e074a6362d9eb33c56407b95195613f864e`, ROM DFU에서 두 번 읽어 일치 |
| 원본 application 768 KiB | SHA256 `3560ae0984b7bac51a1c03077b78c4ee0cf9bf578411a63c116892e767f49a90` (patch3) |
| 원본 NVM 256 KiB | SHA256 `94f30a820aa27c3f00b838bc9649fc5599ea3ef4ac7e3792979b9a2f1223edb0` — 2026-09-10 patch3 적용 기록과 같다 |
| 설정 기준 | 적용 전 RAM 설정 307개(`336A33523235-patch3-running-configuration.json`) |

## 적용 전 상태

양축 IDLE, 입력 0, 속도 0, motor·encoder·controller 오류 0, calibrated/ready,
`pre_calibrated` 유지, 모든 `startup_*` false였다. 버스가 끊겨 있어 patch3는 CAN 오류마다
복구를 되풀이하고 있었다(`recovery_count` 205,704, 백업 뒤 재부팅하고 다시 34,252 — 뒤 값은 작업 중 읽기 전용 조회로 본 값이며 원로그는 첨부하지 않았다, 양축
`axis.error = 0x4000` 복구 래치, 한때 `0x2` 저전압 동반). 이 상태는 patch3의 알려진 결함
그대로이며, 적용 전 점검은 `0x4000`·`0x4002`만 허용하고 나머지 안전 조건은 바꾸지 않았다.
배터리 전압은 약 42 V(UV 트립 40 V)였다(같은 조회, 원로그 미첨부).

## 절차

`powertrain_chassis`를 정지하고 공유 `motor_session` 잠금 안에서 일회성 컨테이너
(`powertrain-sw:jetson`, privileged, **host network**, `/dev/bus/usb`·`/run/powertrain` 공유)로
작업했다. 실행 스크립트는 2026-09-10 `flash_board.py`·`backup_current.py`를 고친
`flash_patch4.py`이며, 단계마다 새 프로세스로 실행했다.

| 단계 | 결과 |
|---|---|
| backup (network none) | 보드는 ROM DFU에 들어갔지만 컨테이너가 새 장치를 못 봄. 쓰기 없음 |
| backup-from-dfu (host network, 새 프로세스) | 1 MiB 두 번 읽기 일치·저장, 앱 복귀. 같은 프로세스의 재탐색이 멈춰 300 s 시간 초과 — 새 프로세스로 재조회하니 11.5 s에 정상(시간 초과·재조회는 작업 중 관찰, 원로그 미첨부) |
| program | 적용 전 점검 통과(설정 차이 0) → ROM DFU 진입. 같은 프로세스가 새 DFU 장치를 못 봐 종료. 쓰기 없음 |
| program-from-dfu (새 프로세스) | 전체 flash = 백업 확인 → application sector 0–9만 지우고 기록 → application 768 KiB·NVM 256 KiB 전체 읽기 대조 통과 → 앱 점프 |
| verify (새 프로세스) | 아래 인수 항목 전부 통과 |
| `powertrain_chassis` 재시작 | Up (healthy), RestartCount 0 (`docker ps`·`docker inspect` 관찰, 원로그 미첨부) |

DFU 진입은 backup과 program 두 번이고, backup-from-dfu·program-from-dfu는 이미 DFU인 장치에 붙었다. 쓰기는 program-from-dfu 한 번뿐이다. `tools/mks_dfu.py`의
`replace_application`은 현재 flash 전체가 백업과 같아야 지우기 시작하고, 실패하면 원본
application을 다시 쓴다.

## 인수 결과 (USB, 버스 단선)

| 항목 | 결과 |
|---|---|
| 펌웨어 | 0.5.1 unreleased(-dev), `can.reliability_patch = 4` |
| 축 상태 | 양축 IDLE, `axis.error = 0`, motor·encoder·controller 오류 0, calibrated/ready·pre_calibrated 유지 |
| 복구 상태 | 래치·진행·자동 재개 자격 모두 false, `reinitializing`·`reinit_requested` false |
| 설정 | 적용 전 307개와 차이 0 |
| 3초 뒤 | `recovery_count` 0 → 0, `recovery_failure_count` 0, `hal_error` 0, `control_deadline_count` 0/0 |
| 정보성 계수 (3초) | `lec_error_count` 388,775 → 432,779(약 14.7k/s), error passive·warning 비슷한 증가, `tx_drop_count` 2,685 → 2,987, `bus_off_count` 0 |

버스가 끊겨 ACK가 없으니 송신마다 LEC가 나는 것이 정상이다. patch4는 이를 계수만 하고
정지·재초기화하지 않는다(설계 D1). patch3는 같은 조건에서 복구를 되풀이하며 양축에 정지
래치를 걸었다. 버스가 연결된 상태에서 이 계수가 멈추는지는 아직 재지 않았다.

## 함정 — Jetson DFU는 단계마다 새 프로세스

- 같은 프로세스에서 fibre(앱)로 `enter_dfu_mode()`를 부른 뒤 pyusb가 새로 열거된
  ROM DFU 장치를 찾지 못했다(두 번 재현, host network에서도 같음).
- 같은 프로세스에서 ROM DFU를 쓴 뒤 fibre `find_any`가 앱을 다시 찾지 못하고 멈췄다.
- 새 프로세스는 두 경우 모두 바로 찾았다. 그래서 `program`(점검·DFU 진입) →
  `program-from-dfu`(기록·점프) → `verify`(인수) → 필요 시 `rollback`으로 나눴다.
- network none 컨테이너에서도 새 DFU 장치를 못 봤다. host network의 새 프로세스에서는 성공했지만, 프로세스도 새로 띄웠으므로 network none 실패가 네트워크 모드 때문인지는 가르지 못했다. 다음에도 host network로 띄운다.
- 상주 `powertrain_canwatchdog` 컨테이너는 나중에 꽂은 USB 장치를 못 보므로 쓰지 않았다.

rollback은 양축 IDLE·정지일 때만 동작한다. verify나 rollback이 거부·실패하면
`powertrain_chassis`를 다시 켜지 않고, 현장에서 안전 정지한 뒤 물리 DFU 스위치로 원본을
복원하기로 했다(이번에는 쓰지 않았다).

## 검토

`flash_patch4.py`는 단계를 바꿀 때마다 Codex 읽기 전용 검토를 같은 스레드에서 받았다.
P2 지적(마지막은 rollback이 거부될 때의 절차 누락)을 반영해 "새 P1/P2 없음"으로
수렴한 뒤 실행했다. `program-from-dfu`도 Codex가 모의 시험 8건(잘못된 기록·flash 불일치
거부, 쓰기 실패 시 원본 복원)으로 확인했다. 검토 원문은 작업 원장에 있고 이 증거 폴더에는 넣지 않았다.
적용 BIN의 소스 시험 134/0은 계획의 구현 검증 로그를 따른다.

## 남은 일

- Step 1 중 **부팅 후 Step 0 재점검**은 하지 않았다. 이번 Step 1은 플래시와 이 보드 단독 USB 대조(설정 307개·캘리·patch 식별)까지다.
- Step 0 NVM 인수: 세 보드를 함께 USB로 연결해야 한다(이번에는 이 보드만 연결).
- Step 2–7: CAN 버스 연결 뒤 실트래픽 10분, Jetson 가지 분리 D0 만료, 수동 재초기화,
  USB 스키드 무영향, 송신 순서 게이트, arm→첫 setpoint 시간.
- Step 8: node15/16 보드 적용. node11/12는 firmware 확인부터.
- patch4 보드에서 motor_gui `odrive_can` 트랙과 `can_drive_test.py`는 setpoint가 300 ms 넘게
  끊기면 D0로 만료된다(설계 §4-9).

## 증거

[증거 폴더](2026-10-21-mks-can-patch4-evidence/)에 압축 원본 flash, 적용 전 설정, 단계별 결과,
실행 스크립트, 적용 BIN과 빌드 기록을 두었다. `SHA256SUMS`로 대조한다.
