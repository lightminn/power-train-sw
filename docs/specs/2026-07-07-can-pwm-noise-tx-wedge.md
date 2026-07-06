# CAN "잘 되다가 아예 안 됨" 규명 — 모터 PWM 노이즈 + mttcan TX 웻지 (2026-07-07)

무선 텔레옵 실주행 중 반복된 **"조종 잘 되다가 전조 없이 완전 먹통"** 증상의 근본원인
규명 기록과 해결책(자동복구 워치독). 누적 bus-off **1460회**를 만든 2층 구조의 문제였다.

## 1. 증상

- DualSense 무선 텔레옵 주행 중 갑자기 모터 무반응 (서버·클라·CAN 노드 전부 "정상"으로 보임)
- 전원 껐다 켜면 됐다가, 좀 쓰면 다시 먹통 — 재현 조건이 잡히지 않음
- `ip -s link show can0`: bus-off 누적 수백 회, ERROR-PASSIVE 빈발

## 2. 진단 — 판별 실험 체인

에러프레임 직접 캡처(raw CAN 소켓 + `CAN_RAW_ERR_FILTER`, `berr-reporting on`)로 소거:

| # | 실험 | 결과 | 기각/확정 |
|---|---|---|---|
| 1 | 젯슨 침묵, 노드 트래픽만 관찰 | 2881프레임 에러 **1** | 배선·종단·타 노드 무죄 |
| 2 | 젯슨 TX 시 에러 분류 | 11~75%, **BIT1 97%** (수신측 에러플래그) | 젯슨 TX 만 오염 |
| 3 | sample-point 0.87→0.875 변경 | 무변화 | mttcan 비트타이밍 무죄 |
| 4 | RTR vs 데이터프레임, 응답 유무 | 무관 (프레임 길수록 에러↑ = 비트단위 오염) | RTR/펌웨어 버그 무죄 |
| 5 | **모터 CLOSED_LOOP vs IDLE** | **28~75% vs 0.0%** (4582프레임 에러 0) | ★ PWM 노이즈 확정 |
| 6 | GND 공통화 (젯슨↔서플라이) | 28.8→19.6% (미미) | 저주파만 개선, HF 잔존 |
| 7 | 노드별 격리 (1축씩, 2라운드 재현) | **node16≈11% · 11≈7% · 15≈1.6% · 12≈0.1%**, 전축 20.1%(≈합) | 채널별 커플링 품질 |
| 8 | TX 폭격 후 깨끗한 버스에서 프로브 | **0/30 영구 사망**, berr 0인데 qdisc 백로그 139p 고착 | ★ mttcan 웻지 확정 |
| 9 | `ip link set can0 down/up` 만 실행 | **30/30 즉시 부활** | 복구 방법 확정 |

## 3. 근본 원인 — 2층 구조

```
[1층] 구동모터(BL70200) PWM 스위칭 노이즈 (채널별: node16·11 이 주범)
      → 젯슨 CAN 송신 프레임만 비트 오염 (비절연 트랜시버, TX 리턴전류 경로 취약)
      → TX 에러율이 bus-off 폭주 문턱(TEC +8/-1, 에러율 11% ≈ 수지 0)의 칼끝
      → 부하↑(주행) = bus-off 폭풍 / 부하↓(정지) = 회복    ← 간헐성의 정체
[2층] bus-off 반복 후 mttcan 드라이버가 TX 큐를 영구 정지 (웻지)
      → berr 0·ERROR-ACTIVE 로 "멀쩡해 보이는데" 모든 send 가 ENOBUFS
      → down/up 없이는 영원히 죽음                        ← "아예 안 됨"의 정체
```

- 젯슨 **수신(RX)은 무결** — 다른 노드들 프레임은 같은 버스에서 에러 0. 오염은 젯슨
  **송신에만** 나타남 (수신은 고임피던스 차동 감지라 노이즈 마진이 큼).
- "전원 껐다 켜면 됐다" = 전원이 아니라 **모터 IDLE화**(1층) + **같이 실행한
  can_setup 의 down/up**(2층)이 각각 약이었던 것.

## 4. 해결 — 자동복구 워치독 (2층 대응, SW)

**정본 = 컨테이너 상주 서비스** (`docker-compose.jetson.yml` 의 `canwatchdog`):
컨테이너 스택이 올라오면 항상 같이 돌고(`restart: unless-stopped`) 재부팅에도 살아난다.

```bash
# 상태 확인 / 로그
docker ps --filter name=canwatchdog
docker logs powertrain_canwatchdog --tail 5
# (수동 기동이 필요하면) 레포에서:
docker compose -f docker/docker-compose.jetson.yml up -d canwatchdog
```

구현 = `motor_control/corner_module/can_watchdog.py` (stdlib 전용, python-can 무의존):

- **감지**: 1초 주기 프로브 프레임(미사용 노드 21 RTR — 전 노드 ACK만) 송신 실패
  **+ `/sys/class/net/can0/statistics/tx_packets` 정지**가 2연속 → 웻지 판정.
  일시 폭주는 tx_packets 가 계속 증가해 구분 → **오탐 0** (폭격 3라운드 실측).
- **복구**: 순수 ioctl(SIOCSIFFLAGS)로 down→up + txqueuelen 1000 복원, 총 ~2s.
  `ip` 바이너리 없는 privileged 컨테이너에서 동작. 기존 SocketCAN 소켓은 리셋 후
  그대로 유지(ifindex 불변)라 제어 프로그램은 프레임 몇 개 유실 후 재개.
- **보조 2형태**: ① 텔레옵 진입점(chassis.teleop_server·teleop_dualsense·corner
  teleop)에 인프로세스 내장(`CanWatchdog(channel).start()`) — 상주 서비스와 중복
  가동해도 무해. ② 호스트판 `scripts/can_watchdog.sh`(qdisc 백로그 기반) — 컨테이너
  밖 비상용.

리셋 순간 조향 status 공백으로 코너 stale→**FAULT** 가 뜰 수 있다 → 텔레옵에서 **□
재무장**하면 계속. (무선 클라 상태줄에 `서버[FAULT]` 로 표시됨.)

## 5. 검증 (2026-07-07 실기)

| 항목 | 결과 |
|---|---|
| 웻지 재현 | 45s TX 폭격 → bus-off +122 → TX 0/30 영구 사망 (백로그 139p 고착) |
| down/up 복구 | 다른 조작 없이 30/30 즉시 부활 |
| 호스트판 자가복구 | 폭격 중 웻지 3회 발생 → 3회 전부 ~2s 자동복구, 최종 30/30 |
| 인프로세스판 | can0 강제 down → 서버 내장 워치독 ~2s 감지·부활, 서버 무중단 |
| 상주 서비스판 | can0 강제 down → `powertrain_canwatchdog` ~2s 감지·부활, TX 10/10 |
| 오탐 | 폭격 3라운드(bus-off +664) 동안 불필요 리셋 **0회** |

## 6. 남은 일 (1층 = 노이즈 자체 저감, HW)

워치독은 2층(웻지)을 무력화하지만 1층(노이즈→bus-off 폭풍→순간 FAULT)은 남는다.
폭풍 자체를 없애려면:

- **node16(rear_right)·node11(front_left) 상선 커플링을 node12(0.1%) 수준으로** —
  상선-CAN 하네스 이격/트위스트/경로 (둘 다 고치면 전축 ~2-3% = TEC 안정권)
- 젯슨 CAN 스텁에 공통모드 초크 / 절연 트랜시버(전원 여유 확보 후) 검토
- node12·16 HALL 접지/필터캡(라인→GND 22~47nF) 과제와 같은 노이즈 계열

## 7. 코드 · 참고

- `motor_control/corner_module/can_watchdog.py` — CanWatchdog (상주/인프로세스 겸용)
- `docker/docker-compose.jetson.yml` — `canwatchdog` 서비스
- `scripts/can_watchdog.sh` — 호스트판 (비상용)
- 커밋: `e729f92`(호스트판) → `f9c87f2`(인프로세스) → 상주 서비스화
- 관련: `scripts/can_setup.sh`(txqueuelen 1000 포함), Notion 「단일 CAN 버스 다중모터 독립제어」
