# USB 스키드 주행 브링업 — 최소 절차

- 작성일: 2026-08-05
- 대상: AK 조향 없이 ODrive USB 6축만으로 스키드(차동) 주행
- 설계: `docs/superpowers/specs/2026-08-04-usb-skid-steer-design.md`
- 코드: `b56245d` ~ `94f6158`

## 0. 이 문서의 범위

**노트북 DualSense → 젯슨 → 6축 구동**까지의 최소 경로다. ROS·콘솔 없이
`teleop_server` 하나로 돈다. 콘솔에서 모드를 고르는 경로는 §5 에 따로 적었다.

⚠️ **아직 아무도 실물로 돌려보지 않았다.** 호스트 테스트만 통과한 상태이고,
특히 **USB 왕복 지연이 50 Hz 를 지키는지는 미검증**이다(§4 P0).

## 1. 사전 확인 (한 번만)

### 1-1. 좀비 프로세스

```bash
docker exec powertrain_jetson sh -c "ps aux | grep -E 'teleop|chassis' | grep -v grep"
```

✅ 기대: 출력 없음. 뭔가 돌고 있으면 죽인다.

> ⚠️ 안 끈 좀비가 v=0 을 계속 명령해 새 테스트와 싸우면서 반나절을 날린 전례가
> 있다(2026-07-05).

### 1-2. 보드 시리얼 뽑기

```bash
docker exec -it powertrain_jetson sh -c \
  "cd /workspace/motor_control && python3 -c \"
from corner_module.drive_odrive_usb_axis import UsbBoardPool
print(UsbBoardPool().discover_serials())\""
```

✅ 기대: 시리얼 3개짜리 리스트. 빈 리스트면 pyusb 미설치 또는 USB 미연결이다.

### 1-3. 레지스트리 작성

`<repo>/config/bl70200_boards.json` 에 저장한다. `[axis0_node, axis1_node]`
순서이며 **axis1 = 로봇 우측**이므로 짝수 node 가 뒤에 온다.

```json
{
  "<시리얼-1>": [11, 12],
  "<시리얼-2>": [13, 14],
  "<시리얼-3>": [15, 16]
}
```

어느 보드가 어느 node 쌍인지는 CAN 셋업 때 정한 값이다(`bl70200_setup.py --node N`).

검증:

```bash
docker exec -it powertrain_jetson sh -c \
  "cd /workspace/motor_control && python3 -c \"
from chassis.chassis_manager import build_usb_skid_corners
print(sorted(build_usb_skid_corners('/workspace/config/bl70200_boards.json')))\""
```

✅ 기대: 바퀴 6개 이름.
❌ `ValueError` 가 나면 메시지가 어느 node 가 빠졌는지 / 우측 바퀴가 axis1 이
아닌지 알려준다. **추측으로 넘기지 말 것** — 바퀴를 잘못 배정하면 조용히 반대로
주행한다.

> 이 파일은 기기마다 다르므로 `.gitignore` 에 있다.

### 1-4. 캘리브레이션

**불필요하다.** NVM 에 영속화돼 있다. `arm()` 이 미캘리 축을 거부하므로, 만약
거부당하면 그때만 `bl70200_setup.py --persist-calibration` 으로 다시 넣는다.

## 2. 주행 (매번)

### 2-1. 젯슨 — 서버

```bash
docker exec -it powertrain_jetson sh -lc \
  "cd /workspace/motor_control && python3 -m chassis.teleop_server \
     --skid-usb --no-us100 --diagnostic-direct-can --confirm-arm-stowed"
```

- `--skid-usb` 가 **can0 을 아예 열지 않는다** — CanWatchdog·RealCanSession 미기동.
- 레지스트리 경로는 기본값이 레포 루트로 풀리므로 지정할 필요 없다.
- `--no-us100` — **컨테이너에 pyserial 이 없어 US-100 을 켜면 기동이 죽는다**
  (`ModuleNotFoundError: No module named 'serial'`, 2026-08-05 젯슨 실측).
  import 가 `if use_us100:` 블록 안에 있어 이 플래그면 아예 안 탄다.
- `--confirm-arm-stowed` — 로봇팔 미사용이라 확인 프롬프트를 건너뛴다.

> ⚠️ **US-100 을 끄면 접근 시 자동 정지가 없다.** 차체 워치독·estop 전파·링크
> 끊김 보호는 그대로지만 장애물 감지는 사라진다. 바퀴 들고 하는 벤치 전용이며
> 지상 주행 전에 다시 판단할 것.
>
> 되살리려면 컨테이너에 pyserial 이 있어야 한다 — 인터넷 되는 곳에서 이미지
> 재빌드.

✅ 기대 출력:

```
🛠️ USB 스키드 모드 — AK 조향 미사용, can0 미개방 (track_gain 1.00)
⚠️ 조향축이 무통전이다. 스키드 중 각이 밀리는지 육안 확인할 것.
=== 차체 4WS 무선 텔레옵 서버 — 포트 9000 대기 (US-100 OFF) ===
```

### 2-2. 노트북 — 클라이언트

```bash
python3 motor_control/laptop/laptop_client_chassis.py --host <젯슨IP> --port 9000
```

프로토콜·매핑 무변경. **지금까지 버려지던 좌스틱 X 가 드디어 ω 로 쓰인다.**

### 2-3. 조작

| 입력 | 동작 |
|---|---|
| RT / LT | 전진 / 후진 |
| 좌스틱 X | 제자리 선회 (좌우 차동) |
| □ | arm / disarm |
| ○ | 정지 후 종료 |

## 3. 예상 동작 (모터 프레임 부호)

호스트에서 실측 검증한 값이다 — 실물이 이와 다르면 배선이나 레지스트리가 틀렸다.

| 명령 | 모터 프레임 |
|---|---|
| 전진 | 좌(11/13/15) `+` · 우(12/14/16) `−` |
| 우선회 | 6축 전부 `+` |
| 좌선회 | 6축 전부 `−` |

바퀴 속도 크기는 윤거 비율을 따른다 — 중간(719 mm) > 앞(545) > 뒤(425).
제자리 선회 ω=1.0 일 때 중간 0.553 / 앞 0.419 / 뒤 0.327 rev/s.

## 4. 반드시 확인할 것

### P0 — 지연 (설계 최대 미지수)

USB 는 속성 하나가 왕복 1회다. 50 Hz = 20 ms 안에 쓰기 6축 + 읽기 4종이 들어가야
한다. 일요일 스크립트는 **쓰기만** 돌려본 것이라 읽기를 얹은 상태는 미검증이다.

주행 중 콘솔에 나오는 tick 주기를 보거나, 텔레메트리의 `last_rx_age_ms` 가
`stale_ms`(기본 500) 를 넘지 않는지 본다. 넘으면 `poll_period_ticks` 를 늘리거나
`loop_hz` 를 낮춰야 한다.

### P1 — 무통전 조향축 (바퀴 들고 먼저)

**AK 조향축이 통전되지 않은 상태로 스키드를 돌린다.** CAD 상 킹핀 스크럽 반경이
0 이라 측면력이 킹핀 모멘트를 거의 안 만들지만, 타이어 셀프얼라이닝 토크 대
백드라이브 0.8 Nm 의 대소는 미지다.

**바퀴를 띄운 상태에서 조향각이 밀리는지 육안으로 먼저 본다.** 밀리면 기계적
고정(핀·스토퍼)이 필요하고 그건 SW 범위 밖이다.

### P2 — 안전

- ○ estop → 6축 0
- 노트북 클라이언트 강제 종료 → 6축 0
- (US-100 은 이번 구성에서 미사용 — pyserial 미설치. 장애물 자동정지 없음)

⚠️ 바퀴 지령 0.3 rev/s 미만은 HALL 코깅존이라 텔레메트리만 그럴듯하고 실물이
안 돈다. **v ≥ 0.4 m/s 로 시험하고 육안 확인 필수.**

## 5. ROS·콘솔 경로 (선택)

`teleop_server` 대신 ROS 스택으로 띄우면 콘솔 배지·조향모드 토글이 붙는다.

```bash
echo usb | sudo tee /etc/powertrain/drive_transport
ros2 launch powertrain_ros autonomy.launch.py chassis:=true
```

- 모드 파일이 없거나 값이 이상하면 `can` 으로 뜬다.
- `drive_transport:=usb` 를 인자로 직접 줘도 된다.
- 콘솔에 `구동 USB` / `조향 스키드` 배지가 뜨고, 조향모드 토글은 회색이다
  (USB 스택에는 조향 액추에이터가 없어 애커만으로 못 돌아간다).

⚠️ 이 경로는 **ROS 환경에서 한 번도 안 돌려봤다.** 서비스 핸들러 테스트 4개와
`colcon test` 전체가 미실행이다.

## 6. 미검증 목록

| # | 항목 |
|---|---|
| V1 | P0 USB 지연 실측 |
| V2 | P1 스키드 E2E + 무통전 조향축 밀림 |
| V3 | P2 안전 경로 |
| V4 | `colcon test --packages-select powertrain_ros` |
| V5 | 서비스 핸들러 테스트 4개 (`test_chassis_node_steering_mode.py`) |
| V6 | 노드 구독 배선 실동작 (`/chassis/safety_state` → 기하 스왑) |
| V7 | 조향모드 전환 E2E (45° 꺾인 상태) |
| V8 | `/odom` 요레이트 ≠ 0 (결함 B 실증) |
| V9 | 콘솔 USB 스택 배포 후 journal 무크래시 |
| V10 | `track_gain` 지상 실측 (명령 ω 대비 실측 ω) |
