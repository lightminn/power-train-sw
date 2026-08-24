# 파워트레인 SW 트러블슈팅 역사 (2026-05 ~ 2026-07)

> 이 문서는 **파워트레인 SW 담당이 실제로 겪고 해결한 결함·함정의 통합 기록**이다.
> 출처 = 레포 `docs/`(specs·plans·reports), 프로젝트 자동메모리, 팀 Notion 💻 Software 페이지,
> git 커밋 이력. 각 항목은 **증상 → 원인 → 해결 → 교훈** 형식이며, 날짜·커밋·수치는
> 원 기록의 실측값을 그대로 옮겼다.
>
> 성격상 이 문서는 "무엇을 만들었나"가 아니라 **"무엇에 당했고 어떻게 빠져나왔나"**의 기록이다.
> 진행 상태·설계 정본은 `docs/reports/2026-07-16-project-state-and-handoff.md`를 볼 것.

---

## 목차

- [0. 한눈에 보기 — 타임라인](#0-한눈에-보기--타임라인)
- [1. CAN 버스 · 통신 계층](#1-can-버스--통신-계층)
- [2. 구동 모터 — ODrive + BL70200](#2-구동-모터--odrive--bl70200)
- [3. 조향 — AK45-36 / AK40-10](#3-조향--ak45-36--ak40-10)
- [4. Jetson 운영 환경 · 컨테이너 · 배포](#4-jetson-운영-환경--컨테이너--배포)
- [5. 비전 · 스트리밍 (D435i / L515)](#5-비전--스트리밍-d435i--l515)
- [6. ROS2 제어 · 안전 계층](#6-ros2-제어--안전-계층)
- [7. 운용 콘솔 GUI · ops 채널](#7-운용-콘솔-gui--ops-채널)
- [8. 시뮬레이션 (MuJoCo → Isaac Sim)](#8-시뮬레이션-mujoco--isaac-sim)
- [9. 자율주행 추정기 · 컨트롤러 (실코스)](#9-자율주행-추정기--컨트롤러-실코스)
- [10. 파라미터 최적화 (parameter_calc)](#10-파라미터-최적화-parameter_calc)
- [11. 오진·철회 기록 (틀렸던 진단들)](#11-오진철회-기록-틀렸던-진단들)
- [12. 반복 패턴 — 이 프로젝트가 배운 것](#12-반복-패턴--이-프로젝트가-배운-것)
- [13. 미해결·감시 항목](#13-미해결감시-항목)

---

## 0. 한눈에 보기 — 타임라인

| 시기 | 대표 사건 | 층위 |
|---|---|---|
| 2026-05-20~25 | motor_gui 게인 스윕 함정, anticogging brick, 1M 종단저항 | 구동/GUI |
| 2026-05-25 | 코너 모듈 HIL — enum TypeError, arm 직후 stale 오판, DualSense 매핑 오류 | 제어 |
| 2026-06-10~14 | 스트리밍 랙 규명(WiFi 절전 · rs.align 108 ms · NVENC 부재) | 비전 |
| 2026-06-20 | **can0 loopback sticky** — 진단 통째 오염 | CAN |
| 2026-06-23 | **1 Mbps 드롭 24~52% → 500 kbps + 50 Hz 확정** | CAN |
| 2026-06-24 | ADM3053 isoPower 전원 부족으로 버스 무음 | CAN |
| 2026-06-24~25 | 듀얼축 브링업 — calib_scan_omega, **shadow_count 폭주**, ILLEGAL_HALL_STATE | 구동 |
| 2026-06-25 | **motor_gui가 BL70200 NVM 오염** (X2212 기본값) | 구동/GUI |
| 2026-07-05 | **좀비 teleop이 모터 테스트 오염** (반나절 소모) · 10모터 4WS HIL 버그 2건 | 운영/제어 |
| 2026-07-06~07 | **모터 PWM 노이즈 → 젯슨 CAN TX 오염** (실험 16종) + **mttcan TX 웻지** | CAN |
| 2026-07-12 | L515 Gateway 성능 결함 6종 | 비전 |
| 2026-07-16 | **첫 FULL HIL** — 블로킹 서비스發 거짓 estop 래치 · US-100 UART 결합 | 안전 |
| 2026-07-18 | **콘솔 3연속 기동 실패** → 「완료 선언 규칙」 신설 · 텔레메트리 5,586회 크래시루프 | 콘솔 |
| 2026-07-18 | **구동 기어비 1:5 사양 정정** (전 체인 5배 오차) | 구동 |
| 2026-07-19 | 적대 리뷰 CRITICAL 10건 (depth scale 4배 · 레거시 NVM 오염 · AK brake 폭주) | 전역 |
| 2026-07-19 | 콘솔 GUI 전 유즈케이스 E2E 결함 20건 | 콘솔 |
| 2026-07-20 | 벤치 구동 — 전원 3 A 한계 UV 트립 · teleop↔chassis 토픽 불일치 | 구동/제어 |
| 2026-07-21 | **6 m 벽 = MuJoCo `mj_multiRay` 결함** (추정기 무죄) | 시뮬 |
| 2026-07-23 | 원격주행 결함 11건 일괄 수정 | 제어 |
| 2026-07-28~31 | 실코스 자율주행 — **m5 스폰 12 cm 편심**, 게이트 자체 무효 규명, 단일시드 통계 오류 | 자율 |

---

## 1. CAN 버스 · 통신 계층

가장 오래, 가장 깊게 헤맨 영역. **물리층·드라이버·프로토콜 3층이 각각 다른 방식으로 배신했다.**

### 1.1 can0 loopback 모드가 sticky — 진단 전체를 오염 (2026-06-20)

- **증상**: AK45-36 HIL 중 `cansend` 는 성공(TX 카운터 증가)하는데 모터가 무반응.
  `candump` 에는 우리 echo 만 보이고 모터 status(50~100 Hz)는 안 들림.
- **함정**: `ip link set can0 up type can ... loopback on` 을 한 번 켜면
  이후 `down; up type can bitrate ...` 로 재기동해도 **loopback 플래그가 남는다**
  (SocketCAN 컨트롤 모드는 sticky — 명시적 `loopback off` 필요). 리부팅하면 클린.
- **왜 위험한가**: 자기 자신에게 self-ACK 하므로 **모든 bitrate(1M/500k/250k/125k)에서
  "ACK 성공"** 이 뜬다. 실제 버스는 완전 무음인데 링크가 살아있는 것처럼 보여
  "AK 가 MIT 모드인가?" 같은 **완전히 틀린 방향**으로 샜다.
- **구분법**: `ip -d link show can0` 에 `<LOOPBACK>` 표기 / 진짜 죽은 버스 = TX 0 +
  bus-off·error-warn 상승 / loopback = TX 완료 + passive candump 무음 + 어느 baud 든 ACK.
- **교훈**: **모든 baud 에서 ACK 가 성공하면 100% loopback 을 의심**할 것.
  진단 시작 전 항상 `loopback off` 를 명시하거나 리부팅으로 클린 스타트.

### 1.2 1 Mbps 드롭율 24~52% → 500 kbps + 50 Hz 확정 (2026-06-23)

- **증상**: AK 2모터(ID1/2) 동시 10 rpm 2초 × 10회 반복에서 명령 드롭율이
  **시도마다 5.9 → 24 → 52%** 로 들쭉날쭉. bus-off 15~24회, run 후반 TX wedge.
- **진단 과정**:
  - 드롭율이 시도마다 변동 = **간헐 접촉(커넥터)** 의 신호. 트위스트로 못 고침.
  - 단독 측정 시 ID1·ID2 둘 다 7~9% 드롭 → 개별 모터가 아니라 **공통 경로**
    (Jetson J17↔트랜시버 배선·종단)가 의심 1순위.
  - 로봇이 커서 **케이블을 못 줄인다** → 길이 대신 비트레이트로 마진 확보.
- **결과 (실측 A/B)**:

  | 설정 | 드롭율 | bus-off | 에러프레임 |
  |---|---|---|---|
  | 1 Mbps + 100 Hz 피드백 | **24~52%** | 15~24 | 다수 |
  | **500 kbps + 50 Hz 피드백** | **0.00%** (2100/2100) | 0 | 0 |

- **부수 함정**: `restart-ms 0`(기본)은 bus-off 시 latch 되어 복구 안 됨 →
  **`restart-ms 100`** 필수. 종단 60 Ω, 버스 양 끝 2곳만(중간 모터 종단 OFF).
- **측정 아티팩트**: 모터 주기피드백이 흐르면 `set_origin`+poll 위치읽기가 stale 프레임에
  오염돼 Δ가 가짜로 들쭉여 보인다. **신뢰할 지표 = 드롭율(tx_packets delta)·bus-off·에러프레임.**
- **대역폭 여유**: 500k 에서 10모터(명령+피드백 50 Hz) ≈ 130 kbit/s = **~26% 부하**.
  100 Hz 가 필요해지면 ~52% 로 빡빡해져 조향/구동 버스 2개 분리가 다음 카드.

### 1.3 ADM3053 절연 트랜시버 — isoPower 전원 부족으로 버스 완전 무음 (2026-06-24)

- **증상**: 신형 CAN Isolator Click(ADM3053BRWZ)으로 교체한 뒤 버스가 **완전 무음**.
  ODrive heartbeat·AK status 둘 다 0, 우리가 TX 하면 무ACK → bus-off.
- **원인 2개**:
  1. **isoPower(절연측 버스전원 VISO)는 VCC=5V 로 생성**하는데,
     **Jetson 40핀 5V 는 isoPower 부하(~100 mA + DC-DC inrush)에 딸려 2.5 V 로 주저앉는다**
     (무부하 5 V, 물리면 분압). → **외부 5 V 공급 필수** + 외부 5V GND 를 로직 GND 와 공통화.
  2. **CTX/CRX 스왑** — 실제로 한 번 바뀌어 있어 양방향 무음.
- **진단 키**: `VISO_OUT`(ADM3053 pin12) 측정. ~5 V 면 전원부 정상, 0 V 면 5V/GND 문제.
  ⚠️ **절연 보드라 +5V 를 GND_ISO(버스측 GND)로 재면 절연막 때문에 떠서 가짜 ~2.5 V** 가 찍힌다.
  반드시 로직 GND(3.3V 잴 때 그 GND) 기준으로 측정.

### 1.4 ★ 모터 PWM 노이즈 → 젯슨 CAN TX 오염 (2026-07-06~07)

이 프로젝트 최대의 진단 사건. 누적 bus-off **1460회+** 를 만든 2층 구조 문제였다.
전말: `docs/specs/2026-07-07-can-pwm-noise-tx-wedge.md`.

- **증상**: 무선 텔레옵 주행 중 **"잘 되다가 전조 없이 완전 먹통"**.
  서버·클라·CAN 노드가 전부 "정상"으로 보이고, 전원을 껐다 켜면 됐다가 좀 쓰면 재발.
  재현 조건이 안 잡힘.
- **판별 실험 16종** (raw CAN 소켓 + `CAN_RAW_ERR_FILTER`, `berr-reporting on`):

  | # | 실험 | 결과 | 판정 |
  |---|---|---|---|
  | 1 | 젯슨 침묵, 노드 트래픽만 | 2881프레임 중 에러 1 | 배선·종단·타 노드 무죄 |
  | 2 | 젯슨 TX 시 에러 분류 | 11~75%, **BIT1 97%** | 젯슨 TX 만 오염 |
  | 3 | sample-point 0.87→0.875 | 무변화 | mttcan 비트타이밍 무죄 |
  | 5 | **모터 CLOSED_LOOP vs IDLE** | **28~75% vs 0.0%** (4582프레임 에러 0) | ★ PWM 노이즈 확정 |
  | 6 | GND 공통화 | 28.8→19.6% | 저주파만 개선, HF 잔존 |
  | 13 | **정지 폐루프 vs 회전** | **27.9% ↔ 1.6%** (가역) | ★ 정지 폐루프가 지배 변수 |
  | 14 | 속도 스윕 0→4 rev/s | 5.1→0.2(@0.6)→0.0% **매끈한 감소** | 전류클램프 기각, 변조지수 확정 |
  | 15 | 로터 전기각 12스텝 스캔 | 3.9~10.5% 각도 의존 | "세션 복불복"의 정체 |
  | 16 | **절연 트랜시버 교체 A/B** | 27.9%→**0.0%**, 폭격 74.6%→**0.00%** | ★★ 종결 |

- **근본 원인 (메커니즘)**:
  - **(a) 왜 정지가 최악** — 센터정렬 SVM 에서 vel=0 이면 3상 듀티가 전부 ~50% 로 같아져
    스위칭 에지가 동시 발화 → 공통모드 전압 풀스윙 한 방. 회전하면 역기전력만큼 듀티가
    갈라져 에지가 분산된다.
  - **(b) 왜 세션마다 복불복** — 정지 시 잔여 전압벡터 방향(로터 전기각)이 상 정렬을 결정.
  - **(c) 왜 젯슨 송신만** — **그라운드 도메인 비대칭**. ODrive 트랜시버들은 PWM 과 함께
    출렁이는 모터전원 GND 위에 있어 서로 무결하지만, 젯슨(비절연)은 조용한 도메인이라
    젯슨이 버스를 구동하면 ODrive 수신기들이 CM 요동으로 recessive 를 오독 → 에러플래그 →
    젯슨 BIT1. (RX 는 고임피던스 차동 감지라 무결 — TX 만 깨지는 비대칭의 설명.)
- **왜 "잘 되다가"였나**: 에러율 27% 는 TEC 폭주 문턱(+8/−1 수지 0 ≈ 11%) 훨씬 위.
  **armed 정차 대기 = 폭풍 / 실주행(≥0.6 rev/s) = 청정** → "주행 잘 되다 세워두면 죽는"
  실사용 패턴과 정확히 정합.
- **해결**: **절연형 CAN 트랜시버 교체**로 결합 경로 제거.
  정차 대기 폭풍·경사 정차 estop 리스크 소멸. 우회책(정차 CAN 침묵·코스트·3선화·초크)은
  전부 불필요해져 기록으로만 보존.

### 1.5 ★ mttcan TX 웻지 — "아예 안 됨"의 정체 (2026-07-07)

- **증상**: 노이즈로 bus-off 가 반복 누적된 뒤, **`berr 0`·ERROR-ACTIVE 로 멀쩡해 보이는데
  모든 send 가 ENOBUFS**. qdisc 백로그 139p 고착. 전원사이클·프로그램 재시작 무관.
- **결정타**: 사용자 관찰 — "에러는 일정한데 증상이 변한다".
  → 매번 같이 돌리던 `can_setup`(down/up)이 **진짜 복구 요인**이었음을 역추적.
- **재현**: 45 s TX 폭격(bus-off +122) → 프로브 0/30 영구 사망 → `down/up` 만으로 30/30 부활.
- **해결 — 워치독 3형태** (정본 = ① 상주 서비스):
  1. compose `canwatchdog` 서비스(`docker-compose.jetson.yml`, `restart: unless-stopped`)
  2. 텔레옵 진입점 인프로세스 내장(`corner_module/can_watchdog.py`)
  3. 호스트판 `scripts/can_watchdog.sh`(비상용)
  - 감지 = 1 s 프로브(미사용 노드 21 RTR) 실패 **+ `tx_packets` 정지** 2연속.
    일시 폭주는 tx_packets 가 계속 증가해 구분 → **오탐 0**(폭격 3라운드 bus-off +664 검증).
  - 복구 = 순수 ioctl(SIOCSIFFLAGS) down/up + txqueuelen 1000, 총 ~2 s.
    `ip` 바이너리 없는 컨테이너에서 동작, 기존 소켓 유지(ifindex 불변).
- **잔존 운용**: 노이즈가 절연으로 해결된 뒤에도 **워치독은 보험으로 상주 유지**
  (비용 0, 트랜시버 전원 문제 등 회귀 대비).
- ⚠️ `pkill -f can_watchdog` 을 ssh 원격 복합명령 안에서 쓰면 **자기매치로 셸 자살** →
  pidfile(`/tmp/can_watchdog.pid`) 사용.

### 1.6 기타 CAN 함정

| 함정 | 증상 | 해결 |
|---|---|---|
| 공장출고 ODrive = 250 k / node 0 | 500 k 버스에 꽂으면 프레임이 깨져 들어와 `ERROR-PASSIVE`(berr rx 127) + 가짜 "node 0 / cmd4 / garbage 120 Hz" | USB 로 `bl70200_setup.py --node N --apply`(baud 까지 교정) |
| 1 Mbps 종단저항 미흡 | tx-error 만 누적(rx=0) → ERROR-PASSIVE → bus-off, 텔레메트리 프리즈·ENOBUFS 잼 | 120 Ω ×2 필수. 정상화 후 tx-error **0** |
| 호스트 TX 과다 | 100 Hz × 4 RTR(=400/s)이 1 M 마진버스에서 bus-off 유발 | `request()` 를 15 Hz throttle, CanError 흡수, txqueuelen 1000 |
| AK 소켓 무필터 | 다중모터 버스에서 AK status 굶음 → 코너 stale | `SteerAk40` 에 STATUS_1 필터 추가 (`4e5cf1c`) |
| 빈/죽은 CAN 버스 | ACK 없어 TX 큐 참 → ENOBUFS(CanOperationError)로 서버 크래시 | `DriveOdriveCan._send` 가 CanError 흡수 + 제어루프 try/except (`dedb2fc`) |
| bus-off 누적 카운터 | `restarts 129 / error-pass 191 / bus-off 130` 같은 절대값 | 과거 1 M 디버깅 누적치 — **절대값 말고 전후 델타만** 볼 것 |

---

## 2. 구동 모터 — ODrive + BL70200

### 2.1 문서와 실물의 사양 불일치 — pp=5/cpr=30 vs pp=10/cpr=60

- **발견 (2026-06-23)**: 레포 `odrive_calibration.py` 와 CLAUDE.md 는 pp=5/cpr=30 이라
  적혀 있었으나 **실제 보드 NVM = pp=10 / cpr=60**(커뮤테이션 깨끗·정확히 1바퀴 도달로 검증).
- **위험**: 레거시 스크립트는 pp=5 를 **강제**하므로 그대로 돌리면 보드 설정을 파괴.
  게다가 `odrive_calibration.py`·`odrive_diff_drive_test.py` 는 **pp=5/cpr=30/UV=8V 를 NVM 에 기록**했다.
- **처리 (2026-07-19, 적대 리뷰 CRITICAL)**: 두 스크립트를 `drive/bl70200/archive/` 로 이동하고
  **import 시 하드스톱**(`94475ef`, 누락분 `2196a72`). 정본 = `bl70200_setup.py` / `can_calibrate_all.py`.

### 2.2 fw 0.5.1 · odrive 라이브러리 함정 모음

| 함정 | 증상 | 해결 |
|---|---|---|
| enum 이 비-IntEnum | `ax.requested_state = AxisState.X` → `TypeError: int() argument ... not 'AxisState'` | `.value` 붙이거나 flat 상수(`AXIS_STATE_*`) |
| device-level clear_errors 없음 | `odrv.clear_errors()` AttributeError | axis-level `ax.clear_errors()` |
| CAN baud 속성쓰기 거부 | `can.config.baud_rate=` → "cannot be written to" | `drv.can.set_baud_rate(500000)` 메서드 |
| node_id 경로 차 | `axis1.config.can.node_id` AttributeError | `axis1.config.can_node_id = 11` |
| `encoder_rate_ms` 부재 | pos/vel 주기 방송 설정이 아예 없음 | 자동 방송은 heartbeat 뿐 — pos/vel/Iq 는 **RTR 폴링 전용** |
| `odrv.config.gpioN_mode` 부재 | 레포 diff_drive 의 gpio9 출력 코드가 에러 | 이 보드엔 GPIO 모드 설정 불가/불필요 |
| **라이브러리 ≠ 보드** | 라이브러리(fw-v0.5.6 소스)에 HALL polarity 캘리 enum 이 있어도 **보드는 0.5.1 이라 미지원**(0.5.2+ 도입) | "라이브러리에 있으니 보드도 된다"고 착각하지 말 것 |

### 2.3 캘리브레이션 지옥 (2026-06-23~25)

- **오프셋 캘리 ~55 s** (`calib_scan_distance=150`) — wait 타임아웃 45 s 면 스캔 중간에 잘려
  "타임아웃"처럼 보이지만, 연결이 끊기면 펌웨어가 알아서 끝내 eready=True 가 된다(**가짜 실패**).
  → 타임아웃 ≥ 90 s.
- **`calib_scan_omega` 기본 12.566 이면 OFFSET 캘리 실패** — M1 은 6.0 이었는데 새로 단 M0 만
  기본값이라 `CPR_POLEPAIRS_MISMATCH(0x2)` 로 15~30 s 에 조기중단.
  HALL polarity 단계는 통과하고 offset 단계만 실패 → calib_range 를 늘려도 무효, **omega 가 진짜 레버**.
- **★ shadow_count 폭주 — 최대 함정**:
  캘리를 반복 실패하면 ODrive 가 **progressively 나빠지는 latched 상태**에 빠진다.
  `shadow_count` 변동폭이 실 HALL 전이수의 49→58→231→452배로 **매 실행 단조 증가**
  (실전이 ~145 인데 shadow 7천~6.5만). 무전원 IDLE 에선 변동 0(=노이즈는 전류 흐를 때만).
  **전류↑↓·스캔·omega·range·공장초기화·동일설정 전부 무효.**
  → **해결 = ODrive 물리 전원 사이클 1회**(reboot 아님). 이후 즉시 깨끗한 캘리(55 s, err 0x0).
  **교훈: 파라미터 무관하게 지속되면 파라미터를 만지지 말고 전원부터 내릴 것.**
- **`save_configuration()` 직후 `0x8`(CURRENT_MEASUREMENT_TIMEOUT) latch** —
  clear_errors 로 안 빠지고 모터캘리(state 4)도 0x8 로 실패. **해결 = ODrive 리부팅**
  (단 pre_calibrated=False 라 캘리 소실 → 재캘리 필요).
- **CAN 풀캘리 `state 6` 요청 시 `INVALID_STATE(0x1)`** — state 6 = ENCODER_INDEX_SEARCH 로
  HALL 엔 index 단계가 없다. **FULL_CAL = state 3** 을 요청하면 펌웨어가 내부적으로
  MOTOR_CAL(4)→OFFSET_CAL(7)로 분해 실행(heartbeat 4→7→1), 6축 6/6 성공.
- **캘리는 RAM-only** — 전원 사이클마다 재캘리. 이 때문에 `can_calibrate_all.py` 가 6축을
  순차 일괄 처리(한 축씩 = 전류 스파이크 방지).

### 2.4 폐루프 중 간헐 트립 — ILLEGAL_HALL_STATE

- **증상**: 폐루프 회전 중 `axis 0x100(ENCODER_FAILED)` = `encoder.error 0x10(ILLEGAL_HALL_STATE)`.
  HALL 이 회전+전류 스트레스에서 순간 불법상태(000/111)를 읽는다. 어느 축이 트립할지는 고정 아님.
- **완화**: `encoder.config.ignore_illegal_hall_state = True`(직전 유효상태 유지) →
  적용 후 위치제어 3종 전부 err 0x0.
- ⚠️ **밴드에이드임을 명시**: 플래그는 **트립만** 막고 node12·16 의 **역방향 속도피드백 불안정은
  그대로**(제자리선회에서 −1.0 미추종, 정지 시 유령 −0.7). 근본 = HALL 접지/필터캡(라인→GND 22~47 nF) HW.

### 2.5 저속 코깅존 — "텔레메트리는 그럴듯한데 바퀴가 안 돈다" (2026-07-05)

- **사건**: WP3 10모터 4WS HIL 의 첫 "성공" run(v=0.15→0.24 rev/s)이 **실제로는 바퀴가 안 돌았다.**
  텔레메트리(순간 RTR 샘플)만 그럴듯했고, **사용자 육안 지적으로 발각**.
- **원인**: HALL(cpr 60) 저속 코깅존 — 바퀴 지령 <0.3 rev/s 면 실물이 정지한 채 있을 수 있다.
- **속도 양끝 제약 (2026-07-05 실측)**:
  - 하단 <0.3 rev/s = 저속 코깅
  - 상단 >~10~12 rev/s = HALL 엣지 과속 → 미스카운트 → 동기상실
    (15 지령 시 실제 3 으로 붕괴 + Iq 6.8 A. 6모터 동시 12 rev/s 에선 marginal 보드가
    `0x2 CPR_POLEPAIRS_MISMATCH` 트립)
  - **깨끗한 대역 ≈ 0.5~10 rev/s** (실주행 0.8 m/s ≈ 1.3 rev/s 라 여유 충분)
- **HIL 통과조건 개정**: 테스트는 **v ≥ 0.4 m/s(바퀴 ≥ 0.6 rev/s) + 실물 육안 확인**을 포함할 것.
- **대책 변천**: `min_drive_turns_per_s` 최저속도 플로어(기본 1.0) 도입(`2291774`) →
  2026-07-17 D3 결정으로 **전면 폐지(기본 0)**, `friction_ff`/`v_knee` torque_ff 피드포워드로 대체.

### 2.6 안티코깅 — 두 번의 배제

1. **fw 0.5.1 anticogging 이 폐루프를 brick** (X2212 트랙, 2026-05):
   `start_anticogging_calibration` 이 불완전 종료되며 `anticogging_enabled=True` + 무효 맵으로
   남아 **명령을 줘도 모터가 안 움직인다(에러도 안 뜸)**. GUI 에서 기능 제거.
   **복구법**: `anticogging_enabled=False`, `pre_calibrated=False` → `save_configuration()` →
   `reboot()` → IDLE 대기 후 폐루프 재진입. (`docs/motor-gui-tuning-guide.md` §6)
2. **원리적 배제 (2026-07-06 조사)**: ODrive cogging_map 은 **absolute/index 엔코더에서만** 저장·로드된다.
   HALL(60 포인트/rev, RAM-only 재캘리)로는 코깅맵을 못 그린다. fw 0.6.x 신형도 해상도 제약 불변.
   → 저속 대책은 bandwidth 30 + 속도 플로어가 사실상 최선. 진짜 해법은 고해상도 절대엔코더(AS5047 등, HW).
3. **`enable_phase_interpolation=False` 도 무효** (2026-07-06, 4라운드 A/B):
   Iq_std 차이 +2%~−8% 로 전부 반복편차 안 = 무의미.
   ⚠️ **`vel_std` 를 비교에 쓰면 안 된다** — phase_interp 가 `vel_estimate` readout 자체를
   smoothing 하므로 True 가 구조적으로 낮게 나온다(실제 매끈함과 무관). **계측기가 결과를 만든 사례.**

### 2.7 게인 튜닝 — 측정 조건이 최적값을 바꾼다

- **1차 (2026-06-24)**: 핵심 레버는 게인이 아니라 **`encoder.config.bandwidth` 100→30**
  (단독 −38%). 저속 회전 진동 vel_std 0.338→0.193(−43%).
  낮춘 bandwidth 덕에 vel_gain 을 0.06 까지 올려 댐핑 확보.
- **2차 재튜닝 (2026-07-04)**: 1차의 vel_gain 0.06 은 **포지션 ±1바퀴 제약 안에서만 잰 값**이었다.
  무부하 자유회전으로 다중 시나리오(정속·가감속 ramp·방향전환)를 재스윕하니
  **0.12 까지 상향 가능**하고 전 구간 개선(정속 리플 ~2×↓, 오버슈트 ~4×↓, 방향전환 리플 ~6×↓).
  0.14 는 다시 악화 → **0.12 가 천장**.
- **교훈**: 최적값은 **측정 시나리오의 함수**다. 제약된 조건에서 뽑은 최적값을 일반화하지 말 것.
- **측정법**: HALL 리플은 코깅위상·방향 의존이라 단발측정 편차가 2배 → **양방향 등속 풀링 + 반복평균**.

### 2.8 motor_gui 가 BL70200 NVM 을 오염 (2026-06-25)

- **증상 (헷갈림 주의)**: 캘리 정상(is_calibrated·enc_ready True), CLOSED_LOOP state=8, err 0x0 인데
  **속도 명령에 모터가 안 돈다.**
- **원인**: `motor_gui --track usb` 의 `DEFAULT_TUNABLES` 는 **X2212-13 스윕 최적값**인데,
  이걸 BL70200 에 적용하고 `save_configuration()` 까지 하면 NVM 이 오염된다.

  | 항목 | 오염값 | 정상값(BL70200) |
  |---|---|---|
  | `motor.config.current_lim` | **100 A** ⚠️ | 9 A |
  | `controller.config.pos_gain` | 8.0 | 2.0 |
  | `controller.config.input_filter_bandwidth` | 50.0 | 2.0 |
  | `controller.config.vel_gain` | 0.015 | 0.06 (→0.12) |
  | `controller.config.vel_integrator_gain` | **0.0** | 0.2 |

- **왜 안 도나**: `vel_int=0` + `vel_gain=0.015` 라 P항만 있고 적분기가 안 감겨
  **정지마찰/코깅을 못 이긴다**(vel_setpoint 은 정상인데 Iq 가 ~0.1 A 로 안 치솟음).
  무부하 자유축이면 안 들키고, 부하가 걸리면 드러난다.
- **해결**: NVM 전수 대조 후 복구. 2026-07-19 에는 **motor_gui 가 게인을 자동으로 쓰지 않도록** 수정(`8a413ad`).
- **교훈**: CLAUDE.md **"Never mix tracks on the same ODrive"** 의 실사례. `current_lim 100 A` 는
  모터·보드 손상 가능성까지 있는 실질 위험이었다.

### 2.9 전원 용량 부족 → 동시 arm UV 트립 (2026-07-20 벤치)

- **증상**: `current_lim` 기본 9 A 로 **6축 동시 arm 하면 전압 붕괴 → 40 V UV 트립(`axis_error=2`)** 재현.
- **원인**: 벤치 전원 최대 3 A (48 V ≈ 144 W).
- **우회**: CAN `Set_Limits`(0x0F)로 **2 A** 하향 후 순차 arm → 6축 전부 통과(전압강하 0.12 V).
- ⚠️ 이 2 A 는 **RAM-only** — ODrive 전원 사이클하면 9 A 로 복귀. 영속화 정책 미결.

### 2.10 구동 기어비 1:5 사양 정정 (2026-07-18, 사용자 발견)

- **사건**: BL70200 은 인휠 직결이 아니라 **감속 1:5**(모터 5회전 = 바퀴 1회전, HALL 은 모터축).
  기존 **전 체인이 직결을 가정**하고 있었다 → 명령 실효 1/5, 피드백·오도메트리 5배 과대.
- **수정**: `DriveOdriveCan` 경계에서 단일 변환(명령 ×ratio, 피드백 ÷ratio, `gear_ratio` 기본 5.0)
  + chassis_node 파라미터 + motor_gui ODrive 트랙 동일 적용(`ad93513`, `e117de9`).
- **파급**: **과거 HIL 수치(2.40 rev/s 등)는 전부 모터축 관측이었음**을 소급 정정.
- **교훈**: 기계 사양은 SW 가 추정하면 안 된다 — 한 상수가 전 체인을 5배 틀리게 만든다.

### 2.11 우측 구동축 미러 장착 (2026-07-28, `848a3f3`)

- 각 보드 M1 축(node 12/14/16) = 로봇 오른쪽 바퀴이며 좌측과 **반대로 돌아야 정방향**.
- `DriveOdriveCan(invert=True)` 가 **CAN 프레임 경계에서만** 명령·엔코더 부호를 뒤집고,
  드라이버 바깥(chassis·odometry·텔레메트리)은 전부 바퀴 프레임 "+=전진" 유지.
- ⚠️ 반면 **raw 스크립트·motor_gui 는 모터 프레임** — `can_drive_test.py` 전진은
  좌(11/13/15)+ / 우(12/14/16)−, 제자리선회는 6축 전부 +.

### 2.12 속도모드 런어웨이

- ODrive 속도모드에서 IDLE 로 안 내리고 스크립트가 죽으면 **마지막 `input_vel` 로 무한 회전**한다
  (ODrive 가 마지막 지령을 유지). → `try/finally` 로 `Set_Axis_State=IDLE(1)` 필수.

---

## 3. 조향 — AK45-36 / AK40-10

| 사건 | 증상 | 원인 | 해결 |
|---|---|---|---|
| **spd/acc ×10 해석** (05-21) | position 속도제한을 무시하고 무부하 최대까지 감 | `send_pos_out` 의 spd/acc 필드를 AK 가 ×10 으로 해석 | ERPM 을 **÷10 해서 전송** (`ak_control.py`) |
| **brake/current 모드 폭주** (05-21) | 2 A 명령 → speed ~−380 | 이 AK 펌웨어가 `SET_BRAKE(2)`/`SET_CURRENT(1)` 를 기대대로 처리 안 함 | 두 모드 **삭제**. 2026-07-19 에 `send_brake` 를 AK45-36 에서 **금지**(`7fb0eb2`) |
| **STATUS 위치 int16 한계** | 3200° 에서 표시가 막힘 | STATUS 위치가 int16(×10도) → ±3276.7° | 명령은 int32 라 멀티턴 OK, 조향엔 충분 → 그대로 둠 |
| **"CAN 죽음" 진단** | ERROR-PASSIVE | berr **tx 만 오르고 rx=0** = AK 가 ACK 안 함 | AK 전원/케이블 문제(SW 아님) → 전원 확인 → `can_setup.sh` |
| **arm 직후 false stale** (05-25) | 첫 tick 에 estop | `steer_ak40.arm()` 이 `_last_rx_ms` 미기록 | arm 시 타임스탬프 기록 (`8e10341`) |
| **6코너 순차 arm 지연** (07-05) | 첫 tick 무조건 false-estop | `CornerModule.tick` 이 `steer.tick` 전에 `state()` 로 stale 판정하는데 6코너 arm 에 ~1.2 s 소요 | `state()` 가 stale 판정 전 `poll(0)` 로 커널버퍼 드레인(자가회복, `91c71e8`) |
| **조향 응답성** (07-05) | 원격조종 시 조향이 느림 | `DEFAULT_SPD_ERPM=1500` | 4500 으로 상향(출력축 ~47°/s, 45° 0.85 s, 정격 5180 이내) (`922fa6b`) |

### 3.1 DualSense 축/버튼 매핑이 환경마다 다르다 (2026-07-06)

- **증상**: 원격조종 축이 이상하게 동작.
- **원인**: 5월 corner_module HIL 때 "검증됨"이라 기록한
  `RT=axis4 / LT=axis3 / □=btn0 / ○=btn2` 가 **현재 노트북 DualSense(SDL 2.28)와 불일치**.
  매핑은 **컨트롤러·연결방식(USB↔BT)·SDL 버전마다 다르다.**
- **해결**: 가이드형 finder 도구 신설 — `laptop/dualsense_axis_finder.py`
  (시키는 대로 조작하면 축/버튼을 자동 판별하고 붙여넣기 블록 출력).
  실측 현재값 = `LX=axis0 · RT=axis5 · LT=axis2 · □=btn3 · ○=btn1`.
  액티브 텔레옵 5개를 전부 이 값으로 통일(`9b2a346`).
- **교훈**: 외부 입력장치 매핑은 **"한 번 검증했다"가 유효기간을 갖지 않는다.**
  컨트롤러/연결이 바뀌면 finder 재실행이 절차.

---

## 4. Jetson 운영 환경 · 컨테이너 · 배포

### 4.1 ★ 좀비 프로세스 — 두 번의 반나절

**(a) 좀비 teleop 이 모터 테스트를 오염 (2026-07-05, 반나절 소모)**

- **증상**: 모터가 갑자기 거칠게 진동 / 지령의 77% 로 undershoot / 간헐 멈춤.
  하드웨어·캘리·게인·HALL·전원을 다 확인했는데 **전부 정상**이라 원인을 못 찾음.
- **원인**: `chassis.teleop_dualsense --no-us100` 을 켜놓고 안 꺼서 **58분째 백그라운드 실행 중**.
  스틱 입력이 0 이라 매 tick `ChassisManager.set(0,0)` 을 6모터에 계속 명령 →
  새 테스트 스크립트의 지령과 **한 버스에서 싸움**.
  → ①0↔지령 surging(거친 진동) ②0 으로 잘려 평균 undershoot ③재수없으면 그놈이 이겨 정지.
- **결정타 진단**: USB 로 `ax.controller.input_vel = 2.0` 을 쓰고 0.3 s 뒤 읽으면
  **0.00 으로 리셋**되어 있음(setpoint=0, Iq≈0). 워치독이 off 인데 리셋 = **다른 writer 존재 확정**.
- **절차화**: 모터 실기 테스트 전 루틴으로
  `docker exec powertrain_jetson ps -eo pid,etime,args | grep -iE 'teleop|chassis|corner|motor_gui'`
  후 `pkill -f teleop`.

**(b) 좀비 ROS 노드가 도메인을 가로챔 (2026-07-18 §9-5 · 2026-07-28 approach HIL, 반나절)**

- **증상**: 격리 스모크가 반복 실패. 코드는 정상인데 "간헐 실패"로 위장.
- **원인**: `timeout N ssh …` 가 발화하면 **원격 subprocess(chassis/approach 노드)가
  컨테이너에 orphan 으로 잔존**(killpg 미실행). 원격 프로브의 `proc.terminate()` 도
  `ros2 run` 래퍼만 죽이고 노드 자식은 좀비로 남는다.
  → 좀비가 도메인 77 에서 `mission_arrive` 서비스를 EVENT_HOLD 로 가로채거나,
  구버전 코드로 FAILED_HOLD 를 발행해 **신규 회귀로 오진**.
- **처방**: 프로브는 **process-group kill**. 스모크/E2E 는 `timeout` 래핑 없이 자연완주시키거나
  매번 `docker exec <c> pkill -9 -f "lib/powertrain_ros/(approach|chassis)"`.
  ⚠️ 같은 `bash -lc` 안에서 pkill 하면 패턴이 자기 셸까지 죽여 무출력 → **direct exec 로 분리**.
- **절차화**: **스위트 실패 진단 1순위 = 좀비 ps 확인.**

### 4.2 컨테이너 · 배포 함정

| 함정 | 내용 |
|---|---|
| **작업 분담** | can0 링크 셋업은 **HOST**(sudo, `can_setup.sh`), python(ak_control/motor_gui)은 **컨테이너**. `python-can` 은 컨테이너에만 있고 host python3 엔 없다. 컨테이너엔 `ip`/`busybox`/`sudo` 가 없다 |
| **compose 코드 반영** | 코드는 컨테이너 **시작 시** colcon 빌드로만 반영 — 설정 불변이면 `up -d` 는 **no-op**. `--force-recreate` 필요 |
| **dustynv pip 죽은 미러** | `dustynv/l4t-pytorch` 베이스의 pip 인덱스가 죽은 미러(jetson.webredirect.org)로 고정 → 파생 이미지에서 `pip install` 시 **`--index-url https://pypi.org/simple` 명시 필수**. PyYAML·pytest 도 베이스에 없음 |
| **`:9000` 테스트-라이브 충돌** | 로봇 위 스위트는 상주 `powertrain_control` 이 :9000 을 점유 → **테스트가 라이브 서버에 붙는다**(DDS 누수의 TCP 판). → autouse 에페메랄 포트 격리(`253d70c`) |
| **install_space 테스트 오탐** | cwd 가 src 패키지 안이면 소스를 import 해 오탐 → `/workspace/ros2` 에서 `pytest src/powertrain_ros/test` |
| **launch 반쪽 생존** | 노드 하나가 죽어도 스택이 반쯤 살아남음 → launch `on_exit=Shutdown`(노드 사망 = 전체 재시작) (`eaa9bc3`) |
| **고아 정리** | 래퍼 패턴이 아니라 **노드 바이너리 패턴**으로 kill 해야 함 |
| **mDNS** | ① `jetson-orin.local` 이 IPv6 link-local(fe80::)로 먼저 풀려 gst SRT URI 가 무한 접속실패 → **IPv4 강제**. ② **컨테이너는 nss-mdns 가 없어 `.local` 불가** — 호스트에서 resolve 후 IP 주입 |
| **Codex 샌드박스** | `.git` read-only → 커밋 불가. 운용 = "구현·테스트=Codex(no-git), diff 검증·커밋=리뷰어". 호스트 conda 에 python-can/serial 없음 |
| **파이프가 에러를 삼킴** | ① 장시간 런 판정에 `\| head` 를 쓰면 파이프가 닫혀 에러 메시지를 못 본다 → **파일로 받고 grep**. ② `pipefail` 미적용 시 파이프가 pytest exit code 를 삼켜 **fail-open** (1회 실제 발생 → 상시화) |
| **파이썬 블록버퍼링** | stdout 리다이렉트 시 `flush=True` 없으면 장기실행 로그가 유령 "무응답"으로 보임 |
| **텔레메트리 크래시루프** (07-18) | chassis sender 가 03:03 부터 **5,586회 크래시루프** — frozen(`MappingProxyType`) gateway payload 를 JSON 에 직삽입. 조용히 돌고 있었음. → 인코더 deep-thaw(`9cb8f3e`) + **배포 체크에 유닛 active·ExecStart 확인 포함** |
| **유닛 ExecStop 패턴 불일치** | restart 마다 sender 가 누수 → 좀비 축적(젯슨 재시작 2회로 실증, `8079952`) |

### 4.3 로봇 전용 AP (GL-SFT1200)

- ⚠️ 라우터 전원을 젯슨 USB 로 따면 전류부족으로 **WiFi 미부팅·리부팅 반복**(SSID 안 뜸)
  → 자체 5V2A 어댑터 필수.
- 라우터 dropbear 가 ssh-rsa 호스트키만 제시 → ssh 에 `-o HostKeyAlgorithms=+ssh-rsa` 필요.
- 검증: NITEZ 경유 4.9 ms vs 전용 5G 링크 **2.19 ms**(loss 0%).

---

## 5. 비전 · 스트리밍 (D435i / L515)

### 5.1 스트리밍 랙 규명 (2026-06-10)

- **1순위 용의자 = 수신 노트북 WiFi 절전모드.** AP 가 패킷을 버퍼링했다 몰아줘 초 단위 랙.
  **저 fps 스트림일수록 악화**(프레임 간 공백에 라디오 doze 발동) —
  "30 fps raw 는 멀쩡한데 YOLO 15 fps 만 랙 걸리는" 미스터리의 정체였다.
  → NM 프로파일 `802-11-wireless.powersave 2` 영구 반영.
- `avdec_h264` 기본 멀티스레드는 프레임 **개수** 단위 고정지연(코어수만큼) → 저 fps 에서 초 단위 증폭.
  → `max-threads=1`.
- 인코더 파이프 write 가 인코딩 끝까지 블록 → 검출 루프와 직렬화되어 fps 절반 이하.
  → **AsyncWriter 스레드(최신 1장만, drop)** 로 5.5 → 21 fps.
- TRT FP16 단독으론 효과 미미 — **병목은 대개 인코딩/네트워크/수신**이지 추론이 아니다.

### 5.2 `rs.align` 이 루프의 80% (2026-06-14, `18ec0e5`)

- **증상**: 사용자 보고 "화면 작고 느림".
- **진단**: 단계별 `time.time()` 누적 프로파일러 → `rs.align(depth→color)` 전체 프레임 정렬이
  **Orin Nano CPU 에서 ~108 ms/프레임 = 루프의 80%** (YOLO TRT 는 24 ms 뿐).
- **수정**: 박스 중심 몇 점의 depth 만 필요한데 전체 정렬은 과함 →
  검출별 `rs2_project_color_pixel_to_depth_pixel` + `rs2_transform_point_to_point` 로 대체(2.9 ms).
- **결과**: **7 → 30 fps**(카메라 상한), frame_age 165 → 60 ms.
  좌표 일치도 평균 11 mm / 95%ile 25 mm(센서 노이즈 이내).
- **교훈**: **Jetson 에서 sparse depth 조회에 full-frame align 을 쓰지 말 것.**

### 5.3 하드웨어·SDK 함정

| 함정 | 내용 |
|---|---|
| **Orin Nano 에 NVENC 가 없다** | Orin NX/AGX 만 있음. 과거 "ABI 불일치" 주석은 **오진**이었다. → SW 인코딩 전제(x264 zerolatency 기본, openh264 폴백). 실측 x264 17.4 fps vs openh264 8.8 fps |
| **librealsense `make install` 버그** | 메인 `.so` 와 `__init__.py` 를 site-packages 에 안 넣고 `pyrsutils` 만 넣음 → `import pyrealsense2` 가 **빈 네임스페이스 패키지**(`rs.context` 없음). → build/Release 에서 직접 복사 + `__init__.py` 생성 |
| **`IMPORT_DEPTH_CAM_FW=OFF` 필수** | 안 끄면 cmake configure 가 펌웨어를 다운로드하다 **도커 빌드 네트워크에서 SSL connect error** (실행 컨테이너는 host net 이라 되지만 buildkit 빌드 net 은 다름) |
| **`FORCE_RSUSB_BACKEND=ON`** | 컨테이너라 커널 패치 불가 → libusb 유저스페이스 백엔드 |
| **srtsrc caller 무한대기** | 접속실패/링크사망 시 **EOF 없이 무한대기** → 수신측 select 기반 stall 워치독 필수 |
| **SIGTERM gst 고아** | 죽인 수신기의 gst 고아가 다음 listener 를 오염 → signal 핸들러로 자식 정리 |

### 5.4 L515 Gateway 성능 결함 6종 (2026-07-12)

| 문제 | 실측 원인 | 수정 |
|---|---|---|
| active stream 이 1~2초 뒤 끊김 | RSUSB 사용 중 새 context 로 `query_devices()` 반복 시 일시적으로 none 반환 | 시작 전 exact-serial 열거만 유지, active 는 video callback freshness 로 판정 |
| 첫 alignment 에서 Gateway FAULT | SDK callback 객체는 base `frame`, `rs.align.process` 는 `composite_frame` 요구 | `as_frameset()` 변환 |
| raw Depth 약 6.4 Hz | 55 ms 작업 뒤 다시 100 ms 를 기다리는 work+period cadence | deadline 기반 10 Hz, overrun catch-up burst 금지 |
| RGB SRT 23~24 Hz | **all-zero 벤치마크가 실제 영상 x264 비용을 과소평가** | real-frame sweep 으로 ultrafast/threads=3 선택 |
| RGB writer 와 alignment 경쟁 | RGB 모드에도 불필요한 alignment 수행 | RGB 는 alignment 0회 |
| in-process restart exit 139 | librealsense 2.50 RSUSB pipeline 을 같은 프로세스에서 재사용 | 정리 후 exit 1, Compose `on-failure:5` 로 supervised restart |

- 최종 인수: RGB 60 s 에 ROS color 30.0~30.2 Hz, SRT drop **0**, gap **0**, 수신 29.91 fps.
- **교훈**: **합성 데이터 벤치마크는 인코더 비용을 과소평가한다** — 실제 프레임으로 스윕할 것.

### 5.5 depth scale 4배 오차 (2026-07-19, `690266b`)

- 코드가 L515 depth scale 을 **0.001 로 하드코딩**하고 있었으나 실제 스케일은 그 1/4 →
  **모든 terrain 거리가 4배로 계산**됐다. 적대 리뷰 CRITICAL 로 발견(`690266b`).
  테스트도 L515 scale 에서 합성 depth raw 단위를 유도하도록 수정(`aed8ea4`).

### 5.6 거리 정본 = SDK depth (2026-07-29)

- 콘솔 PR#3 이 거리 표시를 Z → 3D magnitude 로 바꿨던 것을 **되돌림**.
  실측 대조 0.2883 vs 0.3199(+3.16 cm, θ≈25.7°) — magnitude = depth/cos θ 라
  화면 중앙에서 벗어날수록 벌어진다. **다시 뒤집지 말 것.**

---

## 6. ROS2 제어 · 안전 계층

### 6.1 ★ 블로킹 서비스發 거짓 `safety_topic_stale` 래치 (2026-07-16, 첫 FULL HIL)

- **증상**: arm 직후 또는 유휴 중 간헐적으로 **latched ESTOP**. US-100 거리 정상(2399 mm),
  클럭 점프 0, `/wheel_states` 49.7 Hz, 외부 verdict 5 Hz 무결점 — **수 시간 유령**.
- **근본 원인**: `~/arm`(~0.8 s: 코너 6개 폐루프 진입+피드백 대기)·`~/disarm`(~1.3 s) 같은
  블로킹 Trigger 서비스가 **단일 스레드 executor 를 점유** → 그동안 `/safety_verdict` 콜백이
  큐잉 → 서비스 종료 후 첫 50 Hz tick 이 `age > 750 ms`(실측 **783 ms / 1264 ms**)로 오판.
- **확정 방법**: 젯슨 워크트리 디버그 계측으로 **arm 직후 수신 갭 800 ms 를 직접 관측**.
- **수정**: `_refresh_safety_baseline()` 헬퍼 — 블로킹 서비스 4종 완료 직후 `_last_safety_ms` 재기준화
  (단 최초 verdict 미수신 `None` 이면 그대로 = startup 게이트 유지).
  ⚠️ arm 만 고친 `a191116` 이후 **15분 소크 종료 시점 disarm 에서 같은 클래스가 재발**(age 1264 ms)
  → `149302e` 에서 4종 전부로 일반화.
- **진단 공백도 함께 해소**: stale 래치 전이에 ERROR 로그, verdict 수신 갭 >500 ms 에 WARN 신설.
  **이 유령을 잡기 어려웠던 이유가 "래치가 무로그"였기 때문**이라 재발 방지를 같이 넣었다.

### 6.2 US-100 발행이 블로킹 UART 에 결합 (2026-07-16, `09cb606`)

- **증상**: 유휴 12분 중 2회, `/safety_verdict` 발행이 최대 **1.17 s 정지**(5 Hz 기대).
  **센서 딸꾹질 1회가 곧바로 안전 하트비트 정지로 전파**되는 구조.
- **수정**: 노드 소유 리더 스레드 신설 — 스레드가 블로킹 `monitor.tick()` 을 돌고
  ROS 타이머는 록 보호된 최신 스냅샷만 발행. 리더 스레드 사망 시 fail-safe verdict.
  `us100.py` 에 `write_timeout=0.1` 추가.
- **교훈**: **안전 하트비트를 블로킹 I/O 와 같은 스레드에 두지 말 것.**

### 6.3 teleop ↔ chassis 토픽 불일치 (2026-07-20 발견 → 2026-07-23 근본수정)

- **증상**: 배포 기본값 조합에서 **명령이 구조적으로 전달 불가** →
  `/cmd_vel` 부재 → 0.5 s 커맨드 워치독 → `MOTION_HOLD` 지속.
- **원인**: `teleop_command_node` 는 `/teleop/cmd_vel` 로만 발행하는데,
  `chassis_node` 는 `authority_enabled=False` 면 레거시 `/cmd_vel` 을 구독
  (`/teleop/cmd_vel` 구독은 `True` 분기에만 존재).
- **당시 대응**: 런타임 리맵(`-r /teleop/cmd_vel:=/cmd_vel`)으로 우회 후 **원복** → 재현 예약됨.
- **근본수정 (07-23, `2474797`)**: compose `wp5_control` 에 `authority_enabled:=true`
  (launch 기본값 false 는 벤치용 유지). **검증: chassis 가 `/teleop/cmd_vel` 구독,
  레거시 `/cmd_vel` orphan 소멸.**

### 6.4 pre-hold 명령 재생 (2026-07-15)

- 팀원 PR#1(MOTION_HOLD 중 명령 폐기) 통합 중 발견:
  `chassis_node` 의 mission hold 가 **interlock 을 직접 호출해 래퍼를 우회** →
  hold 해제 시 **pre-hold 명령이 재생**(실측 0.8 → 1.27 rev/s).
- 수정 = 래퍼 경유 + **아키텍처 계약 테스트**(`cm._interlock.set_motion_hold` 직접 호출 금지)로 고정.
- 이후 CMASK(07-18)에서 `command_recovery` 자동해제를 제거해 이 보호를 보존.

### 6.5 원격주행 결함 11건 일괄 (2026-07-23, `2474797`/`49b5a20`/`867d8f5`)

전수 분석(정독 + 서브에이전트 4 + Codex 적대검증) → 구현 → 검증(**음성대조 42건 HEAD-FAIL 실증**).

- **teleop_command**: accept 루프 사망(→0.2 s 백오프), 커넥션 셋업 보호,
  **E-stop 프레임병합 누락 → sticky latch**(motion 슬롯을 덮어써도 유지).
- **authority**: IDLE/clear_hold 진입 시 1회 `ok=True` 제로(0.5 s 코스팅 제거).
- **ops**: 비상 arm 게이트에 stale + estop_latched 추가, 클라이언트 PENDING-pop →
  **status_query 4상태 복구**, 직접응답 캐싱, **인플라이트 캐시 고정**(evict 된 estop 중복실행 봉쇄).
- **레거시 teleop**: disconnect 시 disarm, arm 중립게이트(급발진 방지),
  ○estop 중립게이트 관통, `OverflowError`, 수신버퍼 4 KiB 캡 3곳,
  ops 토큰 없으면 햅틱 미기동(가짜 LINK_LOSS 진동 제거).
- ⚠️ `queried_request_id` 가 additive 필드라 **노트북·젯슨 동시 배포 필요**.

### 6.6 적대 리뷰 CRITICAL (2026-07-19)

| 커밋 | 결함 |
|---|---|
| `7fb0eb2` | **AK45-36 `send_brake` 가 runaway 유발** → 금지 |
| `94475ef`/`2196a72` | 레거시 pp=5/cpr=30 스크립트가 ODrive NVM 오염 → archive + 하드스톱 |
| `8a413ad` | motor_gui 가 X2212 게인을 자동 기록 → 중단. E-stop latch, AK motion 게이트 |
| `690266b` | L515 depth scale 0.001 → 모든 terrain 거리 4배 |
| `1706725` | **비유한(non-finite) 명령이 full output 으로 saturate** → 거부하도록 수정 |
| `b62db5f` | ops_broker 의 E-stop 이 거부되거나 already-succeeded 로 위장될 수 있었음 |
| `3995a94` | BL70200 CAN heartbeat 50 Hz(20 ms) 방송 추가 |

### 6.7 approach 컨트롤러 조향 부호 반전 (2026-07-28, `5a7f924`)

- **CRITICAL**: `omega = -k_yaw * y` 였다(정본 `follow.py` 는 `+k_yaw * lat`).
  → 좌우 대상에서 **멀어져 절대 정렬 불가**.
- ⚠️ **테스트가 틀린 부호를 그대로 박아넣어 못 걸렀다.**
  코드리뷰(max) 라운드에서 15 findings 중 F1 으로 발견.
- **교훈**: 테스트는 구현을 고정할 뿐 **정확성을 보증하지 않는다** — 부호·방향은
  독립 정본(다른 모듈의 동일 법칙)과 대조할 것.

### 6.8 기타

- **rclpy 로거는 %-포맷 포지셔널 인자를 못 받는다** → TypeError (팀원 pdist80b 코드 정합화 중 발견).
- **rclpy `Node._clients` 내부속성 섀도잉** → `create_client` 파괴 (A2a 리뷰 발견).
- **ros2 CLI 데몬이 컨테이너 재기동 직후 불안정**(`rcl node's context is invalid`)
  → 서비스 호출은 rclpy 직접 클라이언트 스크립트로.
- **MISSION_STOP·arrival 은 동일 tick 발행이라 크로스토픽 순서 비보장**(24 ms skew)
  → 동시활성 0.5 s 로 판정.

---

## 7. 운용 콘솔 GUI · ops 채널

### 7.1 ★ 콘솔 3연속 기동 실패 → 「완료 선언 규칙」 신설 (2026-07-18)

- **사건**: 스위트가 전부 green 인 상태에서 **사용자가 직접 기동하다 3연속 실패**를 맞았다.
  1. `gi`(PyGObject) 부재 — conda base 로 실행해서 (→ 시스템 파이썬 고정)
  2. `gtksink` 부재 — Arch 에서 gst-plugins-good 에서 분리됨 (→ `gst-plugin-gtk` 설치)
  3. `_rss` **AttributeError** — `ChassisTelemetryPanel._refresh` 가 다른 클래스의
     staticmethod 를 호출. **LIVE 데이터에서만 도는 경로**인데다
     **PyGObject 가 콜백 예외를 삼켜서** 조용히 죽었다.
- **사용자 질책 원문**: *"제발 코드 짰으면 실행좀 해보고 버그좀 잡아라"*,
  *"뭐 만들었으면 실제 실행 및 실전과 같은 경로로 테스트까지 한번 쫙 돌리도록"*.
- **제도화**:
  - `operator_console/runtime_smoke.py` **실행 게이트 신설** —
    Xvfb 실기동 + 4채널 LIVE 주입 + STALE/sparse 페이로드 + traceback 검출, 호스트 스위트에 편입.
  - 프로젝트 `CLAUDE.md` 「완료 선언 규칙」 + `AGENTS.md` 「Definition of done」에 쌍으로 반영.
  - **게이트를 새로 만들면 음성 대조**(알려진 버그 재주입 → FAIL 확인)로 게이트 자체를 증명.
  - `c18424e` — runtime_smoke 가 "안 죽었다"가 아니라 **LIVE 를 단언**하도록 강화.
- **같은 날 동반 사고**: 텔레메트리 sender 가 **5,586회 크래시루프**를 남몰래 돌고 있었다(§4.2).

### 7.2 ops broker 영구 웨지 (2026-07-18, `5085e9f`)

- **증상**: CMASK 토글 실사용 중 명령 채널이 영구 정지.
- **원인**: **부재 서버에 대한 `call_async` 가 변이 슬롯을 영구 점유.**
- **수정**: `SERVICE_ORDER_ABANDON_S` 10 s 2차 데드라인
  (never-ready → REJECTED unavailable / ready-무응답 → OUTCOME_UNKNOWN, 슬롯 해제).
  07-19 에 never-ready 는 **3.0 s** 로 더 단축(`F13`).

### 7.3 콘솔 GUI 전 유즈케이스 E2E 결함 20건 (2026-07-19, `65d4ef6`)

사용자 지시로 콘솔과 젯슨측 상대를 전수 리뷰. 대표 결함:

| # | 심각도 | 결함 |
|---|---|---|
| F1 | critical | **비상정지 즉시발동 회귀** — 승인 스펙(KGUI D2 = 무확인 즉시)을 구현했는데, **스펙을 모르는 적대 리뷰(R06)가 "결함"으로 오판해 2단 확인으로 되돌리고 테스트로 고정**했다 |
| F3 | critical | **팔 텔레메트리 수신 스레드 독살** — `OverflowError`/`RecursionError` 가 except 밖이라 **독성 1패킷에 스레드 영구 사망**(실재현) |
| F11 | critical | `mission_clear_grip_lost` 서비스 타입 불일치(계약 Trigger vs chassis SetBool) → 버튼이 영원히 실패 + 채널 10초 웻지 |
| F12 | high | **유휴 콘솔이 10초마다 강제 절단**(브로커 `CLIENT_IDLE_TIMEOUT_S`, 실측 t=10.0 s) → ~10초마다 ~1초씩 비상 경로 사각 |
| F2 | high | ops 링크 사망 미감지 — 하부 클라이언트가 소켓 사망을 삼켜 상태 냉동·명령 무통보 증발 |
| A#2 | high | 합법 JointState(velocity 생략·NaN) 하나가 :5007 전체를 침묵시켜 **모터 온도까지 소실** |
| F4 | high | CAN 미수신이 배너에 LIVE 로 표시(송신 `"UNAVAILABLE · …"` vs 수신 정확일치 `"unavailable"`) |
| F9 | medium | srtsrc auto-reconnect 가 리스너 사망을 ERROR 없이 삼켜 **웻지 스트림 영구 STALE** |
| F16 | 게이트 | 위 타입 불일치 재발 방지 — 계약↔chassis_node **소스스캔 정합 테스트** 신설(음성 대조 RED 확인) |

- **교훈 2건 (문서에 명기)**:
  1. **스펙을 모르는 적대 리뷰는 승인 결정을 회귀시킨다.**
     안전 시맨틱 변경은 반드시 설계문서와 대조할 것.
  2. **계약-노드 타입 정합은 소스스캔 게이트로 고정**한다.

---

## 8. 시뮬레이션 (MuJoCo → Isaac Sim)

### 8.1 ★ 6 m 벽 = MuJoCo `mj_multiRay` 결함 (2026-07-21)

- **증상**: 전 가족(flat/bank/clothoid/friction/smog)이 **~6 m 에서 영구 정지**.
  해상도·서스펜션·바퀴반경·오도메트리·카메라 장착과 전부 무관.
- **선행 진단 2건이 실측으로 반증됨**:

  | 기존 주장 | 실측 반증 |
  |---|---|
  | far 타일이 품질 게이트로 배제 | 프레임 0 hard-reject 타일 **0개**, 유효 셀 3.93 m 까지 존재 |
  | 확인에 support ~4 m 필요 → 단일프레임 불가 | 프레임 0 이 support 2.32 m 로 **확인 성공** |

- **근본 원인**: **MuJoCo 3.10.0 `mj_multiRay` 는 cutoff 를 "geom 앵커점까지의 거리"로 프루닝한다.**
  무한 plane(앵커=원점)은 카메라가 원점에서 cutoff(6 m) 밖으로 나가는 순간 **통째로 사라진다**
  — 수직 1 m 아래 바닥도 NO_HIT. `mj_ray` 는 정상.
  최소 재현: plane 위 (x,0,1)에서 수직 아래 레이, cutoff 6 → x=5.90 은 1.00 보고, **x=5.99 부터 NO_HIT**.
- **수정**: 시뮬 센서 **한 곳**만 — 레이캐스트 컷오프를 센서 사거리와 분리
  (`RAY_PRUNING_CUTOFF_M = 1e6`), 사거리는 hit 마스크가 정의. **추정기는 한 줄도 안 고쳤다.**
- **결과**: flat 0.396 → **0.9418**, 전 가족 fail_open 0 · edge_overrun 0.
- **핵심 교훈**: **추정기의 fail-closed 정지는 올바른 동작이었다.**
  낙하 증거가 물리적으로 소실된 것을 "추정기 결함"으로 오래 오해했다.
  **실차 L515 엔 이 메커니즘이 없다** — 실기 결함이 아니었다.
- **회귀 핀**: `test_depth_ray_cutoff.py::test_mujoco_multiray_anchor_pruning_quirk_is_pinned`
  (업스트림 변경 카나리로 quirk 자체를 고정).

### 8.2 P0 "조임목 미정지" — 2회 재규명 (2026-07-21 → 07-22 → 07-27)

- **1차 진단(07-21)**: 일관성 필터가 좁은 에지를 "데이터 결손"으로 강등 → corridor 상속 → 통과.
- **2차 재조사(07-22)로 대체**: depth back-project 결과
  **0.5 m 노치는 어떤 접근거리에서도 낙하가 관측되지 않는다**.
  좁아졌다 다시 넓어지는 구조라 전방 하향 광선이 노치를 건너뛰어 뒤쪽 데크에 꽂힌다.
  **이는 first-hit 기하이므로 렌더러 종류와 무관** — Isaac RTX 로 바꿔도 동일.
- **심각도 재평가 (길이 스윕)**:

  | 조임목 길이 | 완주율 | min_clr | edge_overrun | 판정 |
  |---|---|---|---|---|
  | 0.5 m | 0.946 | −0.026 | 1 | 통과(P0) |
  | 1.0 / 2.0 / 3.0 m | 0.369 / 0.276 / 0.244 | +0.34 | 0 | **정지 = 안전** |

- **처리 (추정기 무변경)**: 테스트를 셋으로 분리 —
  현실적 좁아짐 정지(GREEN) / 안전 불변식 회귀(GREEN) /
  **`test_short_occludable_notch_is_a_known_perception_limitation`(strict xfail)**.
  **게이트를 느슨하게 풀어 숨기지 않고**, 고칠 수 없는 perception 한계만 xfail 로 추적.
- **최종 (07-27 사용자 확인)**: 대회 지형이 고정이라 로봇폭보다 좁고 0.5 m 미만인 짧은 구멍은
  물리적으로 안 나옴 → **대회 완주 비블로킹**. xfail 은 정직한 기록으로 유지.

### 8.3 MuJoCo 헤드리스 렌더 — GL 백엔드 3종 함정 (2026-07-22)

1. `MUJOCO_GL=egl`(surfaceless)는 mujoco egl 헬퍼가 `EGL_EXT_platform_device` 를 못 잡아 실패
   (eglinfo 엔 확장이 있는데도).
2. `MUJOCO_GL=glfw` 는 (a) 시스템 `libglfw.so` 부재 (b) GLFW 가 Wayland 로 붙으려다
   conda xkb 데이터 부재로 init 실패 → **X11 강제** (c) **conda 의 libstdc++(GLIBCXX 3.4.29)가
   Mesa 26.1(3.4.35 요구)보다 낮아** "No GLXFBConfigs" → 시스템 libstdc++ preload.
3. 모델 오프스크린 프레임버퍼 기본 640 → 렌더 전에 `model.vis.global_.offwidth/offheight` 상향.
- ⚠️ 추가로: **fast 모델은 물리용 primitive 프록시**라 렌더하면 "마인크래프트 카트"로 보인다.
  실 CAD 를 보려면 URDF 스키닝이 필요.

### 8.4 Isaac Sim 마이그레이션 함정 (2026-07-23 ~ 07-28)

| 영역 | 함정 |
|---|---|
| **RTX 카메라** | ① focalLength 를 **최소 0.5 로 클램프** — 1/10 단위 변환값 0.18 이 플로어 아래라 광학이 무시됨 ② `verticalAperture` 는 `hAp×H/W` 로 강제(정사각픽셀) ③ fx 는 수평지면에서 관측 불가(퇴화) → `camera_params` 로 검증 |
| **estimator 소비** | 무한 평면에선 estimator 가 **설계상 fail-close**(고가 데크 세계관) → 프로덕션 데크 재현 필수 |
| **자기가림 마스크** | median 만 쓰면 경계 플리커 1 px 가 그리드 시간융합에 유입돼 **영구 obstacle_blocks_path**. → union + dilation 2px. 주행 중엔 정적 마스크로 부족(로커보기 관절 운동) → `instance_id_segmentation` 동적 필터 |
| **부팅** | 존재하지 않는 zero-delay kit 파일을 발명해 부팅 실패 / annotator 워밍업 없으면 `get_depth` 가 replicator 내부 None TypeError |
| **진동 5겹** | 조향 슬루·관절 armature(0 이면 60 Hz 접촉 붕괴)·타이어 해석 원통 콜라이더·고무 컴플라이언트 접촉·체이스캠 스무딩 → z진동 0.732 → 0.118 mm |
| **USD** | ⓐ CollisionAPI 는 Mesh 자식이 아니라 `collisions/.../Xform` 에 붙음 ⓑ 중첩 instanceable 은 근접조상 1회 해제로 안 풀림 → **고정점 반복 루프** ⓒ `UsdGeom.Cylinder` 기본축 = Z → 휠 조인트 축 정렬 + `\|dot\|>0.99` 페일패스트(안 하면 로봇이 0.002 m/s 로 부동) |
| **M4 결함 6종** | argparse `--record` 가 `--record-res` 와 모호(→`allow_abbrev=False`) / 비항등 자세에서 `GfQuatf/d` 타입불일치 크래시 / 실패 시 exit 0 / **레이거리 6 m 계약 누락**(어댑터가 optical-Z 만 체크) / settle 속도0 홀드 경사 크리프 / CONTROLLED_HOLD 크리프 |
| **크리프의 정체** | 실기 ODrive 는 `vel_int 0.2` 적분기가 정지위치를 유지하지만 **Isaac PhysX velocity 조인트엔 적분기가 없다** → 시뮬 충실도 갭. 구동 position hold 핸드브레이크로 대응 |
| **녹화 아티팩트** | 사용자 "bank 안 움직임" 리포트 → 계측 결과 **물리는 정상**(x 0→14 m, 바퀴 3.74 rad/s). 체이스캠이 로봇 중앙고정 + 무특징 직선데크라 전진이 안 보였을 뿐 → 데크 줄무늬 추가(색상 only, 완주율 비트 동일로 무영향 확증) |
| **clothoid 한계진동** | 곡선에서 `steer_target ±66°` 진동, `steer_actual` 이 슬루 47°/s 로 못 따라가 역위상 → 폐루프 한계진동. **MuJoCo 는 조향 position 서보에 rate-limit 이 없어 이 결함을 가리고 있었다.** → 선회의도 게이트 yaw-rate 감쇠(`5f458cd`)로 clothoid 0.814→0.935 |
| **운영** | Codex 프롬프트의 백틱을 zsh 가 **명령치환**해 스펙 훼손 → 단일따옴표 heredoc / ssh 런 명령 끝의 트레일링 `&` 는 parent bash 종료 시 SIGHUP 으로 orphan → 금지 / `M4-SUM` 은 **시작 배너**이지 종료 요약이 아님 |

---

## 9. 자율주행 추정기 · 컨트롤러 (실코스)

가장 최근이자 **아직 완주 미달성**인 영역. 진단이 여러 번 뒤집혔고, 그 과정 자체가 기록 가치가 있다.

### 9.1 ★ 진짜 근본원인 = m5 스폰 12.05 cm 편심 (2026-07-30)

- **배경**: v2 URDF 에서 앞·중·뒤 바퀴쌍 중점이 전부 정확히 `+0.1200` =
  **`base_link` 가 로버 중심선에서 12 cm 벗어남**(CAD 원점 선택).
- **함정**: 시뮬은 이를 `CERTIFIED_ROVER_CENTRE_LOCAL_X_M = 0.1205` 로 보정하게 되어 있고
  m3a 헬퍼와 m4 캠페인은 적용하는데, **이를 오버라이드하는 m5 실코스 하네스만 빠뜨렸다.**
  → 모든 실코스 런이 로버를 능선 중심에서 12 cm 치우쳐 스폰. **편측 여유 6 cm 코스에서 치명적.**
- **검증**: 로버 중심 기준 실제 능선 `[−0.330, +0.571]` vs estimator 보고 corridor `[−0.350, +0.500]`
  → **양쪽 다 보수적, 편향 없음** = estimator 무죄.
- **효과**: 중심정렬만으로 `erosion_empty` 999 → 14/1000.
- **파급**: 지난 세션이 4라운드를 쏟은 **"비대칭 우측 에지 검출 버그"는 존재하지 않았다.**
  로버가 실제로 치우쳐 있었을 뿐. 8패밀리(m4)는 보정을 적용하므로 무관·회귀 없음.

### 9.2 ★ 게이트가 실코스 영역을 아예 안 지나고 있었다 (2026-07-30, `89dfc64`/`382d5a1`)

컨트롤러 변경 2종을 "8패밀리 게이트 불합격"을 근거로 revert 했는데, **게이트 자체가 무효였다.**

1. **`ROBOT_FOOTPRINT_WIDTH_M = 0.949` 가 pre-v2 stale.**
   주석이 스스로 유도식(widest |y| 0.4395 + 0.035)을 적어놨는데,
   `3df0114` 가 0.3595 로 고친 뒤 따라오지 않음 → **0.789 가 정답**, 0.160 m 과대.
   훈련 7종은 1.6 m 트랙이라 편측 여유 0.4055 m → **clearance ramp 가 항상 포화 1.0**
   → 실코스 0.90~0.95 m 대역 **커버 0**.
2. **pinch 가 자기 좁힘에 도달한 적이 없다.**
   좁힘은 x=6.6~7.0 m 인데 `duration_s=12.0` 이라 Isaac 로버는 4.07 m 에서 종료.
   12 s 는 더 빠른 MuJoCo 기준이었다. → `TRAINING_DURATION_S`(40 s)로.
   ⇒ pinch completion **0.2713 → 0.7647**.
- **재판정 결과**: 되돌린 두 컨트롤러 변경 모두 **무죄(중립)**.
  clothoid `edge_overrun=1` 은 **무수정 기준선에도 있는 pre-existing** 결함이었다.
- **교훈**: **"기준선을 실제로 측정하지 않고 비교한 것"이 오류의 근원.**
  메모리에 적어둔 "8패밀리 수치 전부 무효" 경고를 스스로 적용하지 않았다.

### 9.3 블로커가 층층이 드러난 순서

각 수정이 다음 층을 드러내는 전형적 양파 구조였다.

```
① 얼음(drop_boundaries_unobserved) → 아레나 바닥 모델링 갭
② erosion_empty                    → geometry stale + 마진
③ 크롤(clearance_slow)             → 속도 프로파일(clearance_full 0.30 이 코리도 1.389 m 요구)
④ pitch_limit                      → 램프 grade 16.8° > max_slope 15°
⑤ obstacle_blocks_path             → 코너 선회항법 부재
⑥ 횡오차 관측 불가                  → 에지 provenance(비국소 인증)
```

- **①의 정체**: `course.stl` 이 도로 옆을 **void 로 남겨** 낙하 증거가 0 → 게이트 A 전면 거부.
  ⚠️ 첫 필러를 **도로 높이(coplanar)로 깔았더니 오히려 낙하증거를 제거해 역효과**.
  도로 아래(z=0.05, >0.18 m 낮게) 깔자 통과. **모델링 갭이 블로커였고, 현실을 반영하니 풀렸다.**
- **⑥의 정체 (`138292a`)**: 행의 support 경계를 "실제 트랙 에지"로 인증하던 두 신호
  (`floor_columns` 열 투영 + `_fov_limits` 평면 가정)가 **둘 다 비국소**였다.
  → **관측이 끝난 지점을 트랙 에지로 인증** → corridor 가 트랙이 아니라 **카메라(=로버)를 따라간다**
  → 횡오차가 원리적으로 관측 불가. 실측: 계획창 34행 중 **행-국소 인접성 0/34**.

### 9.4 카메라 자기가림이 지배 변수 (2026-07-29)

- 정지 pose 에서 **실측 주행폭 0.950 m 인데 perception 은 0.550 m** — 부족분 0.41 m 가 전부 한쪽.
  그 구간은 `valid_mask` 자체가 없다(= FOV 가 아니라 **관측 부재**).
- 원인 = **자기가림 마스크**. `mount=B(0.48,−0.12,0.24,25°)` 는 17.8% 마스킹이고
  하단부에 폭 ~200 px(640 중) 창만 남긴다 ≈ 1.5 m 에서 ±0.31 m.
  **0.889 m 를 인증할 수 없는 물리적 시야.** 마스크를 0 으로 지워도 나아지지 않음(차체가 가짜 지형이 됨).
- **마운트 스윕**: 카메라 **높이가 지배 변수**. z 0.24 → 수용률 4~5%, z 0.55 → 86%, z 0.60/30° → 98~99.7%.
  ⚠️ 단 **마스크 커버리지가 수용률을 예측하지 못한다**(x60/z40 은 자기가림 0% 인데 수용률 4.8%) —
  높이·pitch 가 바꾸는 것은 가림뿐 아니라 **보이는 지면의 근/원거리 밴드**다.
- ⚠️ **실물 마운트는 여전히 미실측**(v2 CAD 에 카메라 링크 없음) — 상향 권고의 근거일 뿐 확정 사양 아님.

### 9.5 실물 L515 정합 — 시뮬이 실물을 잘못 반영하고 있었다 (2026-07-29)

- 젯슨 실물 `fx 464.52 / fy 464.25 / cx 351.89 / cy 245.49` vs
  시뮬 `fx=fy 457.01, cx=320 cy=240`(완전 대칭 가정).
  **주점이 31.89 px 편심**인데 시뮬은 이를 한 번도 겪지 않았다.
- `_fov_limits` 가 `(0−cx)/fx`·`(W−1−cx)/fx` 라 실물 관측한계는 비대칭
  (−0.7575..+0.6181) vs 시뮬 대칭(−0.7002..+0.6980) = **1.5 m 전방에서 한쪽 21 cm 차이**.
  ⇒ **시뮬은 한쪽 corridor 에지 문제에 낙관적이었다.**
- production 노드는 정상(실제 cx 를 전파). 시뮬 RTX optics 재튜닝(power-train-sim `49f00b1`).
- ⚠️ **기존 8패밀리 수치 전부 무효**(옛 대칭 카메라 산출) — 이 경고가 §9.2 오류로 이어졌다.

### 9.6 ★★ 단일 시드·단일 스폰은 통계적으로 무의미했다 (2026-07-31)

세션 마지막에 파라미터 스윕 하네스(power-train-sim `a91adbb`)를 만들자마자 **자신의 방법론이 반박됐다.**

- **문제**: 캠페인은 family 마다 dev 시드 고정, 실코스는 `SPAWN_X/Y` 상수 하나
  ⇒ **설정당 궤적이 정확히 하나**. 파라미터 효과와 "그 한 번이 어디로 갈렸는가"가 분리 안 됨.
- **스모크 결과**:
  - 실코스 동일 설정, 스폰 −0.03 vs +0.03 → **0.2893 m vs 0.8551 m (3배)**
  - clothoid 동일 설정, 시드 0/1/2 → completion spread 0.018 인데 **`min_clear` spread 0.2705**
- ⇒ **설정 판정 근거로 쓰던 차이(~0.1)가 같은 설정의 시드 산포보다 작았다.**
  세션 중의 설정 간 마진 비교 상당수가 통계적으로 무의미.
- **정직한 기준선 (9 family × 3 시드 = 27런)**:
  안전지표는 **27/27 전부 `edge_overrun 0`, `fail_open 0`**(단일시드보다 훨씬 강한 진술).
- **실코스 재측정**:

  | 마운트 | 스폰별 거리 | 중앙값 | 범위 |
  |---|---|---|---|
  | z0.55 | −0.03→0.324, −0.015→0.902, **0.0→2.355**, +0.015→0.822, +0.03→0.868 | **0.868** | 0.324~2.355 |
  | 운영(0.24) | −0.03→0.320, **0.0→1.093**, +0.03→0.340 | **0.340** | 0.320~1.093 |

  ⇒ 세션 내내 돌린 단일 설정이 곧 스폰 오프셋 0 이고, **그게 양쪽 모두 분포의 최대값**이었다.
  **"실코스 2.355 m 세션 최고" 같은 보고는 정직한 대표값이 아니라 최상 표본**이었다.
  정직한 값은 **z0.55 0.87 m / 운영 0.34 m**.
- **물리적 함의**: **초기 횡배치에 극도로 민감**(편측 여유 8 cm 에서 ±3 cm 는 예산의 1/3).
  실대회에서 mm 단위 배치는 불가능하므로 **이 민감도 자체가 설계 결함**이다.

### 9.7 계측기가 만든 가짜 결론들

- **`curvature_slow` 는 범인이 아니라 계측기 결함이었다** (`b663e83`):
  코드가 `if omega_p:` 라 **조향이 0 이 아니기만 하면** 사유를 붙였다(다른 `*_slow` 는
  soft 임계 아래에서 스케일이 정확히 1.0 이라 조용한데 곡률만 데드존 없음).
  실측 스케일은 `0.055, 1.000, 1.000, 1.000` — 곡률 기여는 2% 뿐이고 묶는 건 clearance.
  수정 후 카운트 2415 → 395(**84%가 잡음**), 실코스 거리는 비트 동일.
- **오프셋 추정 자체가 잡음이었다** (`0237a3c`):
  ground truth 대조 결과 실제 횡오차는 −0.036~+0.165 로 작고 매끄러운데
  **추정치는 ±0.275 로 튀고 따라가지 않는다**(0.9 m 구간에서 추정 0.14 이동 vs 실제 0.02
  = **잡음이 신호의 7배**). 전 표본이 `state=observed` = 폴백이 아니라 인증 경로.
  → 이것이 **적분기 시도가 실패한 이유**(잡음을 적분해 클램프 포화, −0.15 rad/s 상수 바이어스로 작동
  → 오버스티어 → revert).
- **거리 15배 차이 나는 두 런의 `edge_overrun`/`min_clear` 직접 비교는 무효** —
  **안 움직이는 로버는 여유를 자동으로 유지한다.** 같은 함정이 creep 기각(0.0468 vs 0.3888)에도 있었다.

### 9.8 설치/복원 규율이 깨진 사고 (power-train-sim `56ee0e8`)

- 스윕이 박스 production 체크아웃에 설치본 5개를 남겼다.
  **복원 블록이 두 번 실행됐고, 두 번째가 이미 설치본이 든 백업을 되돌려 원본 대신 변형본을 복원**했다
  (`restored md5` 카운트가 4가 아니라 8이었던 게 단서).
- **처방**: `install_one` 은 백업이 이미 있으면 **거부** / `restore_one` 은 **멱등** /
  복원을 `trap ... EXIT` 로 묶기 / 사후 확인은 `git status` 와 **원본 md5 대조를 둘 다**.
- **교훈**: **복원 로그가 찍혔다고 원본이라는 보장이 없다.**

---

## 10. 파라미터 최적화 (parameter_calc)

> ⛔ **2026-07-19 트랙 종료** — 최종 CAD 확정으로 더 이상 손대지 않는다.
> 버그를 발견해도 보고만 하고 `f_opt 0.2004` 는 재계산하지 않는다.

- **면-기준(측면-절반) 물리 불일치 (2026-06-04)**: 법선력 계산에 전체 W 를 써서
  **slip 이 2× 낙관**이던 버그. `W_side=0.5W`, `mass_side=0.5mass` 도입으로 정정
  (F_drv·토크·에너지·전류·stuck 은 수치 불변, **slip 만 보수적으로 교정**).
- **`test_v4.py` 가 무효였다**: v3 함수 import + 14차원 + 30 kg 상수 + **assert 0개**.
  → 로컬 v4 함수·15차원·실제 assert 48개로 전면 재작성.
  핵심 가드 = 평지 `ΣN ≈ 0.5·mass·g`(=245.2 N, 면-기준 검증 + 무게회귀 검출).
- **스모크가 결과 pkl 을 덮어썼던 문제** — 2026-05-25 HPC 풀런(A10 GPU, 11.7 h, 430,200 평가)으로 해소.
- **envelope/지형 캐시**: 지형·envelope 가 상수에만 의존하는데 objective 매 호출 재계산 → 1회 캐시(결과 동일).
- **2026-07-18 적대 리뷰 물리 결함 7건 HIGH — 의도적 미수정**:
  로커 하중분배 부호 반전, 30 mm 스무딩이 충돌 envelope 을 낮춰 바퀴가 계단 모서리 3.66 mm 관통,
  프레임-보기 폭 제약의 sin/cos 혼동, `a_long` 이 stability 로 전달 안 됨, MuJoCo 측면모델 32.5 kg 등.
  → 트랙 종료 결정에 따라 기록만 남김.

---

## 11. 오진·철회 기록 (틀렸던 진단들)

이 프로젝트는 **틀린 진단을 지우지 않고 기록**하는 규율을 유지했다. 같은 함정에 다시 빠지지 않기 위해서다.

| 시기 | 틀린 진단 | 실제 |
|---|---|---|
| 2026-06-20 | "AK 가 MIT 모드인가?" | can0 loopback sticky 로 버스가 무음이었을 뿐 |
| 2026-06-13 | 스트리밍 NVENC "ABI 불일치" | **Orin Nano 에 NVENC 하드웨어가 아예 없다** |
| 2026-07-07 | CAN 에러율의 **per-node 채널 가설**(node16 11% / node12 0.1%) | 세션 간 붕괴(대조군 node12 가 0.1%→31%). 진짜 변수는 **모터 상태(정지 폐루프)** |
| 2026-07-07 | "전원 껐다 켜면 된다" | 전원이 아니라 **모터 IDLE 화 + 같이 실행한 can_setup 의 down/up** |
| 2026-07-21 | 6 m 벽 = 추정기 품질 게이트 / 단일프레임 사거리 | **MuJoCo `mj_multiRay` 앵커거리 프루닝**(시뮬 렌더러) |
| 2026-07-21 | P0 조임목 = 일관성 필터 강등 | **first-hit 기하로 낙하가 원리적으로 관측 불가**(렌더러 무관) |
| 2026-07-26 | 실코스 스케일업(1.2×/2×)으로 폭 가설 검증 | **스케일업은 무효 레버** — 2× 는 코스를 L515 사거리 밖으로 밀어 depth 소멸(`no_valid_depth`) |
| 2026-07-26 | m5 pitch −16° = "스폰 방향 버그" | `forward_local=(0,1,0)` 이라 identity 가 이미 정확. 실제는 바퀴가 바닥 gap 위 |
| 2026-07-28 | v2 로버 "0.79 < 0.83 이니 통과 가능" | 물리적으로는 맞지만 **여유 4 cm 로는 자율주행 불가**(마진 0 에서도) — 과대해석 정정 |
| 2026-07-29 | "비대칭 우측 에지 검출 버그" (4라운드 소모) | **로버가 실제로 12 cm 치우쳐 스폰**되고 있었을 뿐 |
| 2026-07-29 | "혼합셀 roughness 계단이 valley 누출 원인" | 누출 셀 roughness 전부 0.00 — 실제는 valley 두께 8.4 cm < `max_step` 0.12 |
| 2026-07-29 | "현위치 clearance 홀드는 이중계산" | **실제 근접 안전판이었다** — 두 커밋 모두 revert |
| 2026-07-30 | "정본 마스크가 중심 카메라로 구워졌다" | md5 는 다르지만 **배열은 실질 동일**(차이 4 px / 열 시프트 0 px) |
| 2026-07-30 | "`m4_campaign.py` 는 마스크를 안 쓴다" | 쓴다. 앞선 판단은 **`\| head` 가 파이프를 닫아 에러를 못 본 잘린 출력** 탓 |
| 2026-07-30 | "단일 m5 런은 지표로 못 쓴다"(재현성 없음) | 동일 스크립트·설정 반복은 **비트 동일**. 갈린 건 서로 다른 스크립트였음 |
| 2026-07-30 | `curvature_slow` 2415틱 = 지배적 병목 | **계측기 결함**(데드존 없음). 84%가 잡음 |
| 2026-07-31 | "실코스 2.355 m 세션 최고" | 분포의 **최대값**. 정직한 중앙값은 0.868 m |

---

## 12. 반복 패턴 — 이 프로젝트가 배운 것

### 12.1 계측기를 먼저 의심하라

같은 실패 양식이 최소 6번 반복됐다.

- can0 **loopback** 이 self-ACK 로 모든 baud 를 "성공"으로 보이게 함
- `vel_std` 가 `phase_interpolation` 에 의해 구조적으로 낮게 나옴
- MuJoCo `mj_multiRay` 가 바닥을 삭제해 추정기를 범인으로 만듦
- **all-zero 벤치마크**가 실제 x264 비용을 과소평가
- `curvature_slow` 사유가 데드존 없이 항상 붙음
- **차폭 상수 stale + duration 부족**으로 게이트가 대상 영역을 안 지남

> **원칙: "측정값이 이상하다"보다 "측정 방법이 이상하다"를 먼저 검증한다.**

### 12.2 기준선 없는 A/B 는 결론이 아니다

- 컨트롤러 변경 2종을 잘못된 기준선으로 revert 했다가 **둘 다 무죄**로 판명.
- "8패밀리 수치 전부 무효" 경고를 **메모리에 적어두고도 스스로 적용하지 않았다.**
- 단일 시드·단일 스폰 비교는 **같은 설정의 산포보다 작은 차이**를 근거로 삼고 있었다.

> **원칙: 변경을 판정하기 전에 무수정 기준선을 같은 하네스로 실측한다.
> 산포를 모르면 차이를 해석하지 않는다.**

### 12.3 실행하지 않은 코드는 완료가 아니다

- 콘솔 3연속 기동 실패 — **전부 스위트 green 상태에서** 사용자가 직접 맞았다.
- 텔레메트리 sender 5,586회 크래시루프가 **조용히** 돌고 있었다.
- approach 조향 부호 반전을 **테스트가 틀린 부호로 고정**해 못 걸렀다.

> **원칙 (CLAUDE.md 「완료 선언 규칙」)**: 만들었으면 ①실제로 실행하고
> ②실전과 같은 경로로 E2E 를 한 번 돌린다. 게이트를 새로 만들면 **음성 대조로 게이트를 증명**한다.

### 12.4 텔레메트리만 믿지 말 것

- 바퀴 지령 <0.3 rev/s(HALL 코깅존)에서 **실물은 정지한 채 텔레메트리만 그럴듯**했다.
- Isaac 에서 "bank 가 안 움직인다"는 리포트는 **녹화 아티팩트**였다(물리는 정상).

> **원칙: HIL 통과조건에 실물 육안 확인을 포함한다. 반대로 "안 보인다"도 계측으로 반증한다.**

### 12.5 상태를 오염시키는 것이 있는지 먼저 본다

- 좀비 teleop(v=0 계속 명령) — 반나절
- 좀비 ROS 노드(도메인 서비스 가로챔) — 반나절
- 좀비 gst / 좀비 sender / 좀비 클라이언트

> **원칙: 실기 테스트 실패 진단 1순위 = `ps` 로 좀비 확인.**

### 12.6 파라미터를 만지기 전에 전원부터

- `shadow_count` 폭주는 **전류·스캔·omega·range·공장초기화·동일설정 전부 무효**였고
  **물리 전원 사이클 1회**가 답이었다.

### 12.7 밴드에이드와 근본 원인을 구분해서 기록한다

- `ignore_illegal_hall_state=True` 는 **트립만** 막고 역방향 피드백 품질은 그대로 —
  근본은 HALL 접지/필터캡 HW.
- 캘리 RAM-only, `current_lim 2A` RAM-only 같은 **임시 상태를 명시**했다.

### 12.8 스펙을 모르는 리뷰는 승인 결정을 회귀시킨다

- KGUI D2(비상정지 무확인 즉시)를 구현했는데 적대 리뷰가 "결함"으로 오판해
  2단 확인으로 되돌리고 **테스트로 고정**까지 했다.

> **원칙: 안전 시맨틱 변경은 반드시 설계문서와 대조. 계약 정합은 소스스캔 게이트로 봉인.**

### 12.9 안전 게이트를 완화해서 문제를 숨기지 않는다

- 0.5 m 조임목은 고칠 수 없는 perception 한계로 **strict xfail** 로 남기고,
  안전 불변식(fail_open 0 · edge_overrun ≤1)은 GREEN 으로 규정했다.
- undulating fail-close 는 "안전한 수정 불가"로 정직하게 문서화했다.

---

## 13. 미해결·감시 항목

### 13.1 하드웨어 소관

- **node 12·16 HALL 역방향 피드백 불안정** — 근본은 HALL 접지/필터캡(라인→GND 22~47 nF) HW 보강.
- **보드2(336A…) axis1 `PHASE_RESISTANCE_OUT_OF_RANGE`** — M1 상선 커넥터 의심, 배선 점검 중.
- **`current_lim` 영속화 정책 미결** (벤치 전원 3 A 제약 하 RAM-only 2 A 운용 중).
- **L515 실물 마운트 미실측** — v2 CAD 에 카메라 링크가 없다.
  시뮬은 `mount=B` 가정값이며 런치파일도 "실측 전 플레이스홀더". 시뮬 스윕은 **z 0.55 부근 상향을 권고**하지만 확정 사양 아님.
- **실코스 편측 여유 6~8 cm** — 초기 횡배치 ±3 cm 가 예산의 1/3 을 먹는 민감도는 **설계 결함**.
  코스 확폭 또는 차폭 축소는 기구/코스 소관.

### 13.2 SW 감시 항목

- **~530 ms ARMED 유휴 executor 스톨** (2026-07-16 3회 관측, 임계 미만).
  격리 소크(FAKE, 30분 90k tick)에서 갭 0 → **chassis_node 결백**. 라이브 스택 재현이 남음.
- **health matrix 유휴 시 거짓 stale 플래핑** (`3c1e098` 로 근본수정했으나 감시).
- **`test_slow_terrain_update_does_not_starve_command_timer`** — 라이브 스택 부하 시 ~반반 실패,
  단독 통과. 부하 민감 항목.
- **runtime_smoke** 는 무거운 스위트와 동시 실행 시 40 s 타임아웃 플레이크 → **단독 실행이 정본**.
- **`autonomy_controller` 스위트 사전 red** — `ec3e484` 가 terrain grid 를 160×120 으로 바꿨는데
  테스트 하네스 기본이 80×60 그대로. 랩탑에 rclpy 가 없어 젯슨 미실행인 채 push 됐다.
- **clothoid `edge_overrun`** — `138292a` 로 해소됐으나 `narrow_curve` 로 이동(③ 잔여).
- **τ(`path_estimate_tau_s`) 미확정** — 단조롭지 않아 단일 런으로 못 고른다. 다중 시드 배치 필요.
- **pinch 마진 4.4 mm** (이탈 임계 0.035 근접) — 주시 대상.

### 13.3 방법론 부채

- 실코스·캠페인 **다중 시드 재판정** — 세션의 설정 간 마진 비교 상당수를 다시 돌려야 한다.
- **시뮬 odometry parity** — carry 오차 성장이 ground-truth 자세라 검증 안 됨(`odometry-parity-deferred`).
- `lateral_reference_carry_drift` 0.03 은 **가정치이고 이 차량 실측 아님. 시뮬로 튜닝 금지.**
- hidden/stress 시드의 **미보정 debt**(dev 1시드 실측 상한을 적용하지 않음).

---

## 부록. 출처

| 종류 | 위치 |
|---|---|
| 심층 규명 문서 | `docs/specs/2026-07-07-can-pwm-noise-tx-wedge.md`(CAN 16종 실험)<br>`docs/reports/2026-07-21-6m-wall-rootcause-and-fix.md`(6 m 벽 + P0 후속)<br>`docs/reports/2026-07-16-full-hil-safety-fixes.md`(첫 FULL HIL 안전 결함 2건)<br>`docs/reports/2026-07-19-console-gui-e2e-review-fixes.md`(콘솔 20건)<br>`docs/reports/2026-07-12-l515-gateway-performance-hil.md`(L515 성능 6종)<br>`docs/reports/2026-07-22-undulating-sensing-limitation.md` |
| 운영 가이드 | `docs/motor-gui-tuning-guide.md` §5 결과해석 / §6 anticogging 복구 / §9 핵심 함정 |
| 정본·핸드오프 | `docs/reports/2026-07-16-project-state-and-handoff.md`<br>`docs/plans/2026-07-28-wp6-real-course-autonomy-development-plan.md` |
| 프로젝트 자동메모리 | `~/.claude/projects/-home-light-ZETIN-robotics-power-train-sw/memory/`<br>(`can-bus-multidevice-topology` · `bl70200-odrive-jetson-bringup` ·<br>`jetson-docker-motor-control` · `jetson-can-loopback-footgun` ·<br>`motor-gui-bl70200-config-contamination` · `real-course-autonomy-blockers` ·<br>`isaac-fork-workspace` · `runtime-surfaces-need-execution` 등) |
| Notion (팀 공용) | 「CAN 자동복구 워치독 — PWM 노이즈 TX 웻지 해결」(3952d27b…)<br>「ODrive(BL70200) 셋업」§7 트러블슈팅 표(3882d27b…)<br>「단일 CAN 버스 10모터 독립제어」·「motor_gui」·「무선 원격주행」 각 §6 트러블슈팅 표 |
| 규율 문서 | 프로젝트 `.claude/CLAUDE.md` 「완료 선언 규칙 — 실행·실전 경로 검증 필수」 / `AGENTS.md` |

---

*작성 2026-07-31. 이 문서는 회고 기록이며 설계 정본이 아니다 — 현재 상태·권위값은 핸드오프 보고서를 따를 것.*
