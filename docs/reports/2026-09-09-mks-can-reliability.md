# MKS CAN 신뢰성 수정 및 단일 보드 검증

## 확인된 결과

2026-09-09, 호스트의 6축 조회를 분산하고 MKS CAN 송신·복구·watchdog 결함을 수정했다.
**13·14번 보드 한 장**에 수정 펌웨어를 적용했으며, 원본 설정 307개와 캘리브레이션 상태가
유지됐다. 지상 부하 상태의 **0속도** 시험에서 조회만으로 watchdog이 연장되지 않고,
CAN 재초기화 뒤 IDLE 정지가 유지됨을 확인했다. 최종 드라이버로 6축을 600초 조회하는 동안
10개 모터 상태가 연속 수신됐고 encoder stale은0이었다. **지상 제한 위치 시험은 목표 미달로
FAIL**이며, 6축 전체 펌웨어 적용과 실주행 인수는 완료하지 않았다.

대상: serial `336A33523235`, MKS v3.6-56V, 원본 firmware 0.5.1 unreleased,
CAN node13/14·500 kbps. 다른 두 보드는 당시 USB에 연결되지 않아 수정하지 않았다.
사용자가 허용한 물리 범위는 모든 바퀴가 땅에 닿은 부하 상태에서 **모터축 1회전 이내**다.
US-100/L515 의도적 분리와 미완성 자율주행은 이번 원격주행 판정에서 제외한다.

Jetson 시계는 최초2026-08-18, 전원 재인가 뒤1970-01-01로 어긋나 있었다. 아래 ms 값은 동일 Jetson의 monotonic 시각 차이며,
로그의 UTC 문자열을 실제 작업 날짜로 해석하지 않는다.

## 바뀐 동작

- `DriveOdriveCan`은 50 Hz 속도 송신과 조회를 분리한다. 공유 `CanFeedbackScheduler`가
  논리 주기 0→1→2를 진행해 주기당 encoder 최대2개, Iq 최대1개로 제한한다.
  nominal encoder 간격60ms, Iq1.2s이며 기존200ms freshness는 유지한다.
  arm 확인 중에도 세 주기를 진행하고 같은 주기 추가 호출은 중복 조회하지 않는다.
- 최초 절대시간 슬롯 구현은 15/15/30ms 지터에서 node13/16을 영구적으로 건너뛰었다.
  독립 리뷰의 반례를 실패 시험으로 고정하고 공유 논리 주기와 standalone overdue deadline으로 수정했다.
- MKS `write()`의 mailbox 확인과 HAL 등록을 짧은 critical section으로 묶었다.
  슬롯 부족은 bounded drop이며 HAL 실패는 기록하고 CAN 서버에서 복구한다.
  재초기화 중 TX/RX를 막고 양축 PWM 차단·IDLE·오류 래치를 먼저 적용한다.
- CAN1 전용 IRQ disable→RCC reset→pending IRQ clear→HAL Init/filter/start로 과거 TX/RX를
  제거한다. 모터 IRQ를 막은 채 HAL을 기다리지 않으며 실패·성공 복구 모두 RTOS에 양보한다.
- CAN watchdog은 유효한 위치·속도·토크 setpoint에만 갱신된다. RTR·조회·잘못된 길이·
  NaN/Inf는 갱신하지 않는다. IDLE의 새 timeout은 면제하고 이미 발생한 timeout은 유지한다.

fresh zero 순서는 **호스트 운용 계약**이다. 펌웨어 clear는 IDLE에서만 래치를 해제하며
입력을0/현재위치로 지운다. clear 뒤 별도 neutral ACK를 강제하는 것은 아니다.
래치는 RAM 상태이므로 MCU reset을 넘지 않는다. 적용 대상의 **모든 startup_* = false**를
flash 전과 이후 설정 대조로 확인한다. 이 전제를 바꾸면 현재 인수 범위를 벗어난다.

## 빌드·원본 보존

| 항목 | SHA256 / 결과 |
|---|---|
| 고정 제조사 ZIP | `1c9ff347f997bbbf8cb693cdb44d93fe6f6ef992aefde60b6a65b0e1fa25a777` |
| 패치 | `6ac98d05506b3124034d08cc9c0995dcb04a3cc99ee43a99e531e7eecfc638bf` |
| 적용 BIN | `7567809465b9646d39b1f9595f8a8e037ed758344f90269ba49038f8c6bfc710` |
| 실물 원본 flash1MiB | `794ed7b812282cffae5297adb683a29dff5fc4f9f5af3207ea6c5c18404ab9c7` |
| 원본 NVM256KiB | `94f30a820aa27c3f00b838bc9649fc5599ea3ef4ac7e3792979b9a2f1223edb0` |
| 전체 ARM 빌드 | text246244 / data1580 / bss136064 bytes |
| 부팅 후 식별 | 0.5.1-dev, `can.reliability_patch=1` |

실물 원본 flash는 DFU로 **두 번 읽어 바이트 일치**를 확인했다. 제조사 제공 바이너리는
실물 원본과 다르므로 복원 이미지로 대체하지 않았다. 쓰기 직전에 현재 전체 flash와 백업을
다시 비교한 뒤 application768KiB만 갱신하고 application 전체와 NVM 전체를 읽어 검증했다.
설정 저장·캘리브레이션은 수행하지 않았다. `tools/mks_dfu.py`는 자동 복원 및 실패 경계를
시험했지만, 전원/USB가 끊겨 ROM에 접근할 수 없는 상태의 자동 복원까지 보장하지 않는다.

## 실행 결과

| 검증 | 결과 | 증거 경계 |
|---|---|---|
| motor_control+firmware/tools 회귀 | **1005 passed** | 하드웨어 접근 없는 전체 해당 스위트 |
| 그중 corner_module/chassis 회귀 | **682 passed** | fake bus/clock 및 정책 경계 |
| 실제 vendor 소스 회귀 | 원본4pass22fail → **수정26pass** | HAL/RTOS/주변장치는 stub |
| 오프라인 소스 준비 | **3 passed** | 고정 ZIP·patch 재적용·Config_t 동일성 |
| guarded DFU | **32 passed** | 실제 Python transport+독립 flash/ROM 모델 |
| 전체 Cortex-M4 빌드 | **PASS** | 새 추출/패치 소스, GNU Arm10.3 |
| 설치된 ROS+vcan77 전체 루프 | **PASS** | 시작·가상 구동 피드백·입력 단절·재접속 후 비재무장 |
| 첫 조회 분산안+기존 firmware600s | **10/10 연속 수신**, stale0, bus error0 | 최종 논리 주기안/수정 firmware 조합의 장시간 증거는 아님 |
| 최종 드라이버+patch1 보드, 6축 조회600s | **PASS** | 10/10 연속수신, stale0, host CAN 오류/드롭0; 무구동 |
| 실물13/14 flash·부팅 | **PASS** | patch1,307설정동일,calibrated/ready,IDLE,error0 |
| IDLE에서 watchdog 후보300ms | **PASS** | 새 timeout 없음 |
| zero 제어 중단+계속된 조회 | **PASS** | 13/14 약302.5ms 뒤 IDLE, error0x800 |
| zero 제어 중 동일baud CAN reset | **PASS** | IDLE,error0x4000,latchtrue,자동재진입없음 |
| 명시 clear만 수행 | **PASS** | IDLE 유지 |
| clear→새zero→명시arm | **PASS** | 두 축 state8/error0 |
| zero 시험 종료 | **PASS** | 엔코더 최대변위0, IDLE/error0,307설정복원 |
| 전원 재인가1회 | **설정·캘리 유지** | Stuff error 복구 후 정지 래치 관측, 안정 확인 뒤 명시 clear |
| 지상 ±0.25 motor rev 목표 | **FAIL** | 4초 내 목표 미달, 한 회전 제한 내 정지 |
| 최종 USB+6초 passive CAN 확인 | **PASS** | patch1,307설정동일,6축IDLE/error0,10모터수신 |

첫600s 조회 시험의 최대 heartbeat 간격은28.78ms 미만이었다. 양축별 encoder 약10,000회,
Iq 약500회이며 다른 모터 상태도 함께 관측했다. 소스 시험과 실물 정상 경로가 통과했다고
해서 과거 장애의 실물 HAL ErrorCode를 소급해서 확정하지 않는다.

300ms watchdog은 이번 시험의 **RAM 후보 설정**이다. 시험 종료 시 원래 disabled/timeout0로
복원했다. 전원 재인가 후 자동으로 켜진다고 가정하면 안 된다. 기존 USB 운용·캘리 경로의
명시 feed 및 나머지 보드 검증 전까지 전 보드 NVM에 일괄 저장하지 않는다.

## 전원 재인가 후 관측

사용자 전원 재인가 후에도 patch1·307설정·두 축 calibrated/ready·pre_calibrated가 유지됐다.
실제 HAL 진단은 **Stuff error0x08**, recovery_count2, recovery_failure0, tx_error0이었다.
현재 HAL 오류는0이고 두 축은 입력0·IDLE·ESTOP latch0x4000 상태였다. 즉 캘리 소실이 아니라
CAN 복구 후의 정지 래치였다. 제조사 HAL 헤더에서0x08을 Stuff error로 대조했다.
이 관측만으로 오류가 생긴 전기적 원인이나 이전 장애의 HAL 값을 소급 확정하지 않는다.

## 지상 부하 제한 위치 시험

두 축에 절대 위치 기준13번+0.25회전,14번−0.25회전을 명령했다. 전류한계9A와 게인은
유지하고 RAM의 position/trap 모드·속도1rev/s·overspeed1.2·watchdog300ms를 적용했다.
목표4초 제한, 독립 송신기8초 제한, 피드백60ms 제한, 최대변위0.40/누적0.45회전 제한을
두었으며 stop 이후에는 watchdog을 연장하는 setpoint를 보내지 않았다.

**목표 도달은 FAIL**이다. CAN 프레임 송수신과 폐루프 진입은 수행됐지만4초 안에
두 축이 목표에 도달하지 못했다. 최대 엔코더 변위13번0.06733/14번0.10217회전,
누적13번0.19823/14번0.17246회전, 최대속도0.38625/0.46875motor rev/s였다.
두 축 모두 IDLE/error0으로 정지했고307설정이 복원됐다. 허용1회전 안에서 끝났으나,
이 저속·지상 부하 시험을 정상 실주행 성공으로 취급하지 않는다. 목표 미달 원인은
현재 자료로 확정하지 않았고 전류·게인을 임의로 올려 재시험하지 않았다. 원시4,193프레임 대조에서도
각축 현재위치7회→고정목표198회, 명령최대간격21.25ms, 목표·단위·순서가 일치했다.
두 축은4초간state8/error0을 유지하고 위치응답은각260건이었다. 실제 Iq와 내부position
setpoint를 시험 중 기록하지 않아 전류·HALL·부하 원인을 확정할 근거는 부족하다.
소스 planner의약0.71초 수렴은 내부 목표 궤적이며 실물 부하 모터 도달시간이 아니다.

## 최종 CAN 조회와 남은 인수

0속도 시험 뒤 사용자가 전원을 껐다 켰다. 제한 위치 시험 전송/실행은 SSH의
`No route to host`로 실패했고, 로봇 Wi-Fi가 사라진 동안 노트북이 학교망으로 자동 전환된 것을
확인해 저장된 로봇망으로 복귀했다. 이후10모터 연속수신·6초 복구횟수 불변을 확인하고
IDLE에서 명시적으로 정지 래치만 해제했다. 그 뒤 위 제한 위치 시험을 수행했다.
최종 드라이버(`d8220e52d39f18f173b3be5510c4d98aa305de3efab64eb2bac3bcf544d64a23`)로
6축 encoder/Iq를600.000초 조회했다. 실제 `DriveOdriveCan`과 공유 scheduler 경로를 사용했고
송신63,000건은 encoder RTR 각10,000회·Iq RTR 각500회뿐이다. 구동 지령은 없었다.
10개 모터 heartbeat/status는 각각30,001~30,002건이며 최대 간격26.589ms, stale0,
6축IDLE/error0, 젯슨 can0 RX/TX 오류·드롭0이었다.

최종 USB 대조에서도 patch1·307설정·캘리가 유지됐다. HAL 현재오류0, recovery_count2와
failure0은 전원 재인가 뒤 값에서 늘지 않았다. 단 **펌웨어 tx_drop_count는2→4**로 증가했다.
이 누계는 명시 clear·제한 이동·최종 조회를 포함하는 구간이고, 조회 직전 USB 누계가 없으므로
그 증가를600초 조회에만 귀속할 수 없다. 젯슨 can0의 드롭0과 다른 계층의 카운터다.
최종6초 수동 관측에서도10모터가 응답하고 복구 횟수는 불변이었다.

나머지 두 보드 적용, 전 보드 watchdog 영속 설정, 실주행·육안 이동·제동거리 인수는 남아 있다.
목표 미달의 원인은 확정하지 않았다. 이번 작업에서 추가 이동이나 전류·게인 증가는 하지 않았다.

## 반영과 검토

코드 변경은 `2bf5a28`(`fix(can): MKS 오류 복구와 분산 조회를 보강한다`)에 묶었다.
로컬 main·GitHub main·Jetson canonical `~/power-train-sw-integrated`에 같은 코드 커밋을
반영했다. 이 결과 문서와 원시 증거는 후속 문서 커밋으로 함께 동기화한다.
팀원 Jetson `~/power-train-sw`는 별도 체크아웃이며 변경을 보존한다.
실제 제어·차체·CAN watchdog 송신 프로세스는 정지 상태로 남겼다.

호스트·펌웨어·DFU·실기 시험 경계를 독립 리뷰했다. 절대시간 슬롯 starvation과 DFU 전송,
시험 종료 경계의 지적은 수정하고 반례를 재실행했다. fresh-zero/MCU reset 보장 범위에
관한 차이는 위 운용 계약과 RAM 래치 한계로 명시했으며, 최종 코드에 남은 차단급 지적은 없다.
지상 목표 미달은 이 코드 리뷰 통과와 별개로 미해결 결과다.

증거: [`2026-09-09-mks-can-reliability-evidence/`](2026-09-09-mks-can-reliability-evidence/).
flash/zero 결과는 SSH stdout과 장치의 전체 JSON·raw CAN을 함께 회수해 보존했다. 재현 방법은 [`firmware/mks_odrive/README.md`](../../firmware/mks_odrive/README.md).
이전 진단은 [`MKS USB/CAN 기록`](2026-09-09-mks-usb-can-diagnosis.md)이다.
