# 로봇팔 벤치 운용 가이드 — 시연장 조작자 + AI 에이전트용 (2026-11-05)

시연장에서 로봇팔과 통합 운용 콘솔(GUI)을 다루는 팀원, 그리고 그 팀원을 돕는 Claude·Codex가
**이 문서 하나만 보고** 팔을 켜고, 조작하고, 막혔을 때 복구할 수 있도록 정리했다.
기능 명세·파라미터의 정본은 [`docs/specs/2026-10-17-local-arm-console.md`](specs/2026-10-17-local-arm-console.md)다
(§번호는 그 문서를 가리킨다). 이 문서와 정본이 다르면 정본을 따르고 이 문서를 고친다.

- 1장 = 사람이 따라 하는 순서. 2장 = GUI 화면별 사용법. 3장 = 막혔을 때.
- 4장 = **AI 에이전트가 먼저 읽을 맥락**(구성·경로·절대 하지 말 것).

---

## 1. 켜기 (시연 당일 순서)

### 1.1 전원·네트워크

1. 로봇 메인 전원을 켠다. 젯슨·팔 서보·바퀴 모터가 같은 전원 라인이다.
2. 노트북 Wi-Fi를 **로봇 AP `ZETIN-ROBOT`(2.4 GHz)** 에 붙인다(5 GHz는 지연이 커서 주행이 끊긴 기록이 있다).
   - 젯슨 주소: 로봇 AP = `jetson-orin.local` / `192.168.8.106`, Wi-Fi `ZETIN_5G` = `192.168.50.98`, USB-C = `192.168.55.1`.
3. 연결 확인 — **어디서: 노트북**
   ```bash
   ssh -o ConnectTimeout=5 zetin@jetson-orin.local 'hostname; uptime'
   ```
   ✅ 기대 출력: `jetson-orin`과 가동 시간. 실패하면 3장 "젯슨에 접속이 안 된다".

### 1.2 팔 스택 + GUI 한 번에 띄우기

**어디서: 노트북 터미널** (이 창은 GUI가 떠 있는 동안 계속 열어 둔다)
```bash
ssh -Y -C -o ServerAliveInterval=5 zetin@jetson-orin.local \
    '~/power-train-sw-pr12/scripts/start_jetson_arm_bench.sh'
```
런처(`scripts/start_jetson_arm_bench.sh`)가 알아서 하는 일:
팔 컨테이너 `arm_pr12_jetson` 시작 → 브리지 기동(서보를 못 찾으면 3초 간격 최대 5회 재시도) →
ops 브로커 → 콘솔 미러 → GUI 실행.

✅ 기대 출력:
```
[arm-bench] 브리지 기동 (1/5)          (또는 "브리지 정상 실행 중")
[arm-bench] 브리지 정상: 팔 5축 토크 확인
[arm-bench] ops 브로커 기동
[arm-bench] 콘솔 미러 기동
[arm-bench] 콘솔 실행
```
그리고 노트북 화면에 GUI 창이 뜬다(화면은 젯슨에서 그려 X11로 넘어온다 — 노트북보다 젯슨 모니터가 빠르다).

⚠️ `브리지가 서보를 찾지 못했다`가 나오면 서보 전원과 U2D2 배선을 확인하고
GUI의 **`브릿지 재시작`** 버튼을 누르거나 런처를 다시 실행한다(3장).

### 1.3 조작 시작 전 체크

1. 상단 탭에서 **`로봇팔 수동조작`** 을 연다.
2. 링크 배지가 **`정상수신(LIVE)`**, 3D 카드가 **`자세 수신`** 인지 본다.
3. **`수동 제어 요청`** → 승인된 모드가 `MANUAL`인지 본다(벤치 브리지는 MANUAL로 시작하지만, GUI를 다시 켜면 다시 요청해야 한다).
4. 키보드 조작은 **GUI 창에 포커스가 있고 이 탭이 보일 때만** 먹는다(입력칸에 커서가 있으면 무시).

---

## 2. GUI 사용법 (`로봇팔 수동조작` 탭)

전체화면이면 위쪽이 2×2 작업 영역이고, 나머지는 스크롤하면 아래에 있다(폭 900 px 미만은 1열).

| 위치 | 카드 | 용도 |
|---|---|---|
| 좌상 | 로봇팔 자세(3D) | 실제 관절각을 따라 움직이는 3D. 도구가 버스에서 감지될 때만 그리퍼가 보인다 |
| 좌하 | 키보드 텔레옵 | 축별 조그, 관절 제한·전류 트립 토글, 자세 저장/이동/자유 지정 |
| 우상 | 도구 결합 잠금 | 볼락(Ball-Lock) 결합·해제·정지, 잠김/열림 위치 기록 |
| 우하 | 도구 교체(슬롯) | 슬롯 티칭과 자동 교체 |
| 아래 | 제어권·도구·상태·진단 | 수동 제어 요청/반납, 도구 토크, 도구 재탐색, **브릿지 재시작** |

### 2.1 키보드 텔레옵 (§2.2)

| 키 | 동작 |
|---|---|
| `1`~`5` / 숫자패드 | 조그할 축 선택 (6 = 그리퍼, 그리퍼 도구일 때만) |
| `↑` / `W` | 선택 축 + 방향, **누르는 동안만** 이동 |
| `↓` / `S` | 선택 축 − 방향, 누르는 동안만 |
| `[` / `]` | 조그 속도 단계 −/+ (0~10, 기본 5) |
| `Space` | **정지** — 5축·잠금 모터 토크 OFF (팔 1~4축은 현재 자세 유지) |
| `T` | **복귀** — 정지/E-stop 래치를 풀고 현재 자세로 토크 복귀 |
| `H` | 저장 자세 `home`으로 이동 |

- 키를 놓거나 갱신이 1초 끊기면 그 자리에서 멈춘다(힘을 빼지 않는다).
- **관절 제한** 토글(기본 OFF, 2026-11-06): ⚠️ OFF면 소프트 범위 보호 없이 하드스톱까지 움직일 수 있으니 눈으로 보며 조작한다. ON이면 1~4축은 관절 제한 범위, 5축은 영점 ±10°이고 범위 밖으로 **더 나가는 방향만** 막는다.
- **전류 트립** 토글(기본 OFF): 켜면 전류가 문턱을 넘을 때 멈춘다(토크는 유지).
- 패드(Xbox/DualSense)도 같다: LB/RB 축 선택, 십자키 ↑↓ 조그, ←→ 속도. 도구는 X/B 열기·닫기, A 소프트 정지, Y 토크 해제(§2.7).

### 2.2 자세 (§2.3)

- `현재 자세 저장` → 이름 입력(1~4축만 저장, 5축 제외). 이름 `home`은 `H` 키와 교체 끝 위치로 쓰인다.
- `선택 자세로 이동`: 1축이 가장 빠르고(≈37°/s) 4축은 1초 늦게 출발한다. 관절 제한 ON에서 범위 밖 자세는 **이동 전체를 거부**한다.
- `자세 자유 지정`: 1~4축 토크 OFF — **반드시 팔을 손으로 받친 뒤 누른다.** `자유 지정 해제`로 현재 자세에서 토크 복귀.
- 거부 이유는 GUI가 아니라 브리지 로그(`local pose rejected/held`)에 남는다.

### 2.3 도구 결합 잠금 (볼락, §2.5)

- `도구 결합` / `도구 해제` / `잠금 정지`. 기록된 잠김/열림 위치로 움직이고, 기록이 없으면 현재 위치에서 22.5° 돈다.
- **자동 결합**(`tool_auto_lock_on_attach: true`): 빠져 있던 도구의 모터 ID가 다시 읽히면 약 1초 안에 볼락이 스스로 잠긴다. 도구 프로필 검사에 실패해도 잠근다. 도구를 끼운 채 `도구 해제`를 누르거나 도구가 끼워진 채 브릿지를 재시작해도 잠기지 않고, 수동 모드·E-stop 해제·팔 정지·ID1 응답 상태가 될 때까지 기다린다. **ID가 읽혔다는 건 전기 연결뿐이다.** 끼울 때 손을 볼락에서 떼고, 잠긴 뒤 도구가 실제로 물렸는지 눈으로 확인한다.
- **도구는 굴삭기(ID5)와 랙앤피니언 그리퍼(ID3/4) 2종이다.** 새 XL430을 끼웠는데 ID가 안 읽히면 공장값(ID1, 57600 bps)일 수 있다. 브릿지를 멈추고 57600에서 스캔해 ID를 바꾼 뒤 1 Mbps로 맞춘다. ID1은 볼락이므로 그대로 쓰면 안 된다. 굴삭기는 끝점 캘리브레이션 전까지 구동이 거부된다.
- **키보드 텔레옵 6축은 달린 도구의 모터를 움직인다.** 굴삭기는 ID5, 랙앤피니언 그리퍼는 ID3이고 그리퍼일 때 ID4 토크는 꺼진다. 첫 조그 때 토크가 켜진다. 끝점을 쓰지 않아 굴삭기는 한 바퀴(0~4095)와 Goal PWM 400, 그리퍼는 Goal PWM 800으로만 제한하므로(2026-11-06 뻑뻑해 사용자 지시로 상향, 파지 동작은 280 유지), 끝에 닿으면 바로 손을 뗀다. `관절 제한` 토글은 6축에 적용되지 않는다.
- **교체가 끝나면 도구가 스스로 한 번 움직인다.** home 복귀 뒤 랙앤피니언 그리퍼는 ID3 +600틱(벌림) 뒤 원위치, 굴삭기는 +300틱 뒤 원위치한다. 끝점을 보지 않으므로 끝에 막혀 6초 안에 못 가면 교체가 `중단`으로 끝나고 토크가 꺼진다. 도구 주변을 비운다.
- **랙앤피니언 그리퍼는 ID3만 구동한다.** ID3 단독 조작(`좌`) 때 ID4 토크를 끈다. 그리퍼를 슬롯에 반납할 때는 5 cm 위에서 ID3를 기억한 Tick(`local_tool_change_gripper_tick`, 한 바퀴 안의 값 0~4095)으로 맞춘 뒤 팔이 내려간다. 재장착하면 회전수가 사라지므로 가장 가까운 같은 Tick으로 간다. ID3는 Operating Mode 4여야 하며, 시작 전에 모드를 확인한다.
- 팔 위치와 무관하게 명령을 받는다(유지보수용). 단, 팔 조그·자세 이동이 진행 중이면 거부한다.
- 상태: 기록 위치 ±20 tick 안 = 잠김(LOCKED)/열림(UNLOCKED), 그 밖 = BETWEEN.
- ⚠️ **결합 센서가 없다.** LOCKED는 "로터가 잠금 위치에 있다"는 뜻이지 볼이 걸렸다는 증명이 아니다.
- 잠김/열림 위치 다시 기록: 카드의 `영점 · 회전 방향 · 잠금 모터 준비` 펼침 → `현재 위치를 결합/분리 영점으로`.

- `축별 속도 적용`(자세 관리): 1~5축 속도(1~60°/s)를 정한다. 자세 이동과 도구 교체의 슬롯·home 이동에 바로 쓰이고 `/session/pose_speeds.json`에 저장돼 재시작 뒤에도 유지된다. 슬롯 5 cm 수직 이동은 `local_tool_change_speed_mps`를 따른다.

### 2.4 도구 교체 (슬롯, §2.8)

1. 슬롯마다 **① 체결 자세**(머리가 바닥을 향함)와 **② 그 5 cm 위 자세**를 저장한다. 도구마다 슬롯 1개.
1-1. 슬롯은 도구마다 하나다. 굴삭기는 슬롯 1, 랙앤피니언 그리퍼는 슬롯 2로 고정이라 도구만 고르면 된다.
1-2. `도구 해제 (슬롯에 반납)`: 달린 도구를 자기 슬롯에 놓고 분리를 확인한 뒤 home으로 돌아간다. 도구 결합 잠금의 `도구 해제`와 달리 팔이 슬롯까지 움직인다.
2. `이 도구로 교체 시작`: 지금 도구를 자기 슬롯에 반납 → 분리 확인 → 새 도구 슬롯에서 체결 → 장착 확인 → `home`.
2-1. 랙앤피니언 그리퍼를 가져올 때는 체결 자세로 내려간 뒤 1축을 +5° 돌려 도구 ID가 읽히게 하고, 볼락을 잠근 다음 그 각도를 유지한 채 올라온다. 굴삭기는 돌리지 않는다.
2-2. 슬롯에서 올라올 때는 저장한 5 cm 위 자세에서 5 cm 더 올라온다(`local_tool_change_extra_rise_m`).
2-3. 반납은 그 높이에서 수직으로 내려와 체결 자세로 간다. 저장한 두 자세의 1축이 다르면 그 구간에서 함께 돈다.
2-5. 5 cm 위 지점은 저장한 자세 그대로 지난다.
2-4. 축이 목표 앞에서 1초 멈추면 그 축 목표를 128틱 더 밀어 한 번 더 시도하고, 그래도 안 되면 오차를 안고 다음 단계로 간다.
3. `도구 없이 동작만 시험`을 켜면 잠금·확인 없이 이동만 한다.
3-1. 이동 단계는 전 축이 ±12틱 안에 들어오거나, 모든 축이 1초 멈추면 넘어간다. 뻑뻑한 축이 모자라게 멈춰도 오차를 안은 채 계속하므로, `bridge-rt.log`의 `continuing short of target` 값과 실제 정렬을 눈으로 확인한다.
4. `교체 정지`는 항상 눌린다. 정지·시간 초과·장착 확인 실패는 모두 **팔 위치 유지**로 끝나고, 움직이던 볼락만 멈춘다. 멈춰 있던 볼락은 토크를 유지해 도구를 계속 잡는다.
- 교체 뒤 새 도구에 힘을 주려면 `도구 토크 활성화`를 직접 누른다(자동으로 켜지지 않는다).
- 도구 인식은 버스 응답 ID로 한다: 단일 그리퍼 5, 듀얼 3/4, 클리너 2. 장착된 도구 모터만 응답하는 배선이 전제다.

### 2.5 가동범위 캘리브레이션 (§2.9) — `로봇팔·도구 캘리브레이션` 탭 → `로봇팔` → `3. 가동범위`

1~4축 관절 제한을 다시 잴 때만 쓴다. MANUAL 제어권 필요. 기어비·영점은 GUI에서 지원하지 않는다.
1. `측정` → 각 축을 양쪽 하드스톱까지 **천천히** 움직인다(`자세 자유 지정`으로 손으로, 또는 관절 제한 OFF 조그). 세게 부딪히지 않는다.
2. `측정`을 한 번 더 → 기록 종료. 5° 미만으로 움직인 축은 기존 범위 유지.
3. `검증` → 하드스톱에서 2° 여유를 뺀다. 저장 자세가 새 범위 밖이면 **실패**(끝까지 안 민 것) → 다시 측정.
4. `임시 적용`(지금 실행 중인 브리지에만) → 확인 후 `저장`(`/session/arm_joint_limits.json`, 다음 기동부터).

### 2.6 GUI 없이 터미널·Python으로 조작 (`operator_console.arm_terminal`)

GUI 대신 터미널 화면(curses)이나 파이썬 코드로 같은 기능을 쓴다. 정본: [`docs/specs/2026-10-31-headless-arm-control.md`](specs/2026-10-31-headless-arm-control.md).
GTK·ROS·Dynamixel SDK가 필요 없고, **브리지·브로커·미러는 떠 있어야 한다**(같은 인증 ops 경로를 쓴다).
⚠️ 같은 UDP 수신 포트(15707)를 쓰므로 **GUI를 먼저 끈다**(4.3의 "GUI만 끄기", 브리지는 유지). 둘을 동시에 띄우면 터미널이 시작을 거부한다.

**어디서: 젯슨 호스트** (`ssh zetin@jetson-orin.local` 후)
```bash
cd ~/power-train-sw-pr12
python3 -m operator_console.arm_terminal --host 127.0.0.1 --port 29001 \
    --token-file ~/pr12-arm-session/ops_console.token --telemetry-port 15707 --bench --ignore-lock
```
✅ 상태·각도·볼락·자세 목록과 단축키가 보이는 터미널 화면. 연결만으로는 아무것도 보내지 않는다 → `a`로 MANUAL 요청.
- 키: `1`~`5` 축(6 도구), `↑/W`·`↓/S` 조그, `[`/`]` 속도, `H` home, `L` 관절 제한, `Y` 전류 트립, `Space`(명령 입력 중엔 `Ctrl+X`) 정지, `q` 종료(정지 후 종료).
- `:` 뒤 명령 + Enter: `save/move/delete 이름`, `freedrive`/`release`, `lock`/`unlock`/`lock-stop`, `capture 1 single seat`, `exchange single [dry-run]`, `tool open|close|stop`, `limits on|off`, `trip on|off` 등(전체 목록은 정본 명세).
- 터미널은 키 떼기를 알 수 없어 마지막 입력 0.15초 뒤 조그를 놓은 것으로 본다.
- 파이썬: `from operator_console.arm_terminal import ArmTerminalClient` → `with ArmTerminalClient("127.0.0.1", 29001, 토큰경로) as arm:` 안에서 `arm.command("manual")`, `arm.wait_until(조건)`, 누르는 동안 `arm.jog(축, ±1)` 반복(예제는 정본 명세 "Python API").
- 끝나면 GUI를 다시 쓰려면 1.2 명령을 다시 실행한다(브리지가 정상이면 그대로 두고 GUI만 띄운다).

### 2.7 제어권 카드의 복구 버튼

- **`도구 재탐색`**: 같은 도구를 다시 찾고 도구 FSM을 초기화한다(도구를 다시 꽂은 뒤).
- **`브릿지 재시작`**(§2.10): 브리지 프로세스를 강제로 끄고 다시 띄운다. **서보 토크·자세는 유지**되고,
  진행 중인 조그·자세 이동·교체는 멈춘다. 팔 상태가 끊긴 상태에서도 눌린다. 누른 뒤 10~20초 기다린다.

---

## 3. 막혔을 때

| 증상 | 원인 | 조치 |
|---|---|---|
| 브리지 로그 `startup torque read failed`, 3D가 `정보 대기`, 팔 무반응 | 기동 시 서보를 하나도 못 읽음. 서보 전원·배선 문제이거나, 2026-11-06 이전 코드라면 재부팅 뒤 첫 기동의 FTDI 지연 쓰기가 원인(수정됨) | 서보 전원 확인 → `브릿지 재시작` 버튼 또는 런처 재실행 |
| 볼락 상태 `FAULT`, 위치 미수신, 결합 버튼 무반응 | ID1 서보 설정이 위치 모드가 아니다(`ID1 requires healthy XL430 mode 3…`). 2026-11-06 실측에서 속도 모드(1)였고 누가 바꿨는지는 미확인 | Operating Mode(주소 11)를 3으로 되돌린다(토크 OFF에서 EEPROM 쓰기). 브리지를 끄고 Dynamixel Wizard 등으로 한다 |
| `H`·자세 이동·교체가 반응 없음, 로그 `outside joint limits` | ⚠️ 관절 제한을 ON으로 켰다. 2026-11-06 기준 저장 자세 5개 모두 1축 등이 제한(1축 ±14.35°) 밖이라 이동 전체가 거부된다 | 키보드 텔레옵의 `관절 제한`을 OFF로 되돌리거나, 가동범위를 다시 측정한다(2.5) |
| 버튼이 전부 회색, 툴팁 `수동 제어권 미승인` | MANUAL 아님(GUI 재시작 시 초기화) | `수동 제어 요청` |
| 키를 눌러도 안 움직임 | 창 포커스 없음 / 다른 탭 / 입력칸에 커서 | GUI 창 클릭, `로봇팔 수동조작` 탭, 입력칸 밖 클릭 |
| Space 이후 안 움직임, E-stop 표시 | 정지/E-stop 래치 | `T`(복귀) |
| 자세 이동이 안 됨 | 관절 제한 ON에서 저장 자세가 범위 밖 / 볼락이 움직이는 중 | 브리지 로그 확인, 제한 OFF 또는 자세 재저장 |
| 3D가 `자세 수신` ↔ `정보 대기` 반복 | 빈 `/joint_states`(관절 이름 없음)가 섞여 나옴 | 4.4의 확인 명령. 빈 메시지면 브리지 재시작 |
| GUI가 갑자기 꺼짐 | 노트북 Wi-Fi가 다른 AP로 넘어가 SSH가 끊김 | 노트북을 로봇 AP에 다시 붙이고 1.2 다시 실행(브리지는 계속 돌고 있다) |
| 젯슨에 접속이 안 된다 | 노트북이 다른 망(예: 학교 Wi-Fi)에 붙음 / 젯슨 꺼짐 | `ip -4 addr` 로 노트북 주소 확인(로봇 AP면 192.168.8.x). 세 주소 모두 ping |
| 젯슨이 혼자 재부팅됨(로그에 정상 종료 없음) | ⚠️ 원인 미확인 — 팔 토크 인가 시 전원 강하 의심 | 전원·배터리 확인 후 1.2부터. 재발 시 기록해 둔다 |

---

## 값 튜닝 이어가기 (에이전트·운용자 공통)

현장에서 "1축 -2도", "볼락 PWM 낮춰" 같은 말로 값을 맞출 때의 기준이다. 실제로 쓰이는 값은 젯슨 `~/pr12-arm-session`(컨테이너 `/session`)에 있고, `config/arm_bench_tuned/`가 커밋된 사본이다.

**반드시 지킬 순서**

1. `/tool/status`의 `local_tool_change.state`를 읽는다. `IDLE`·`DONE`·`ABORTED`가 아니면 **기다린다.** 교체 중 브릿지를 재시작하면 시퀀스가 끊긴다(2026-11-06 실제로 끊었다).
2. 바꿀 파일을 `/session/backup-20261106/`에 복사한다.
3. 값을 바꾸고 `docker exec arm_pr12_jetson bash /ws/scripts/restart_arm_bridge.sh`로 재시작한다. 이 명령과 같은 셸 문자열에 브릿지 프로세스 이름을 넣지 않는다.
4. 재시작 뒤 도구 인식, 볼락 상태, Traceback 유무를 확인한다.
5. 코드를 바꿨다면 노트북에서 고쳐 `scp`로 `~/power-train-sw-pr12`에 넣는다. 새 모듈 파일을 추가했을 때만 런타임 빌드가 필요하다. 상태 표시를 바꿨다면 `arm_console_bridge`와 `ops_broker`, GUI도 재시작한다(런처는 이미 뜬 프로세스를 다시 띄우지 않는다).

| 바꾸려는 것 | 위치 | 현재 값 |
|---|---|---|
| 슬롯 체결·5 cm 위 자세 | `/session/tool_slots.json` (rad) | `config/arm_bench_tuned/tool_slots.json` |
| 자세 이동·슬롯 이동 축 속도 | GUI `축별 속도 적용` 또는 `/session/pose_speeds.json` | 37, 16, 15, 20, 27 °/s |
| 슬롯 수직 이동 속도 | `session.yaml` `local_tool_change_speed_mps` | 0.01 m/s |
| 슬롯 추가 상승량 | `session.yaml` `local_tool_change_extra_rise_m` | 0.05 m |
| 볼락 PWM 상한 | `session.yaml` `toolchanger_goal_pwm` | 600 |
| 볼락 잠김 인정 범위 | `session.yaml` `toolchanger_locked_tolerance_ticks` | 100 (2964 이상) |
| 1축 Goal PWM | `session.yaml` `arm_goal_pwm` | 800 |
| 그리퍼 반납 ID3 Tick | `session.yaml` `local_tool_change_gripper_tick` | 126 |
| 굴삭기 반납 Tick | 미설정, 코드에 단계 없음 | 다음 결합 때 정함 |
| 체결 자세 1축 회전 | `bench_tool_change.py` `SEAT_NUDGE_RAD`·`SEAT_NUDGE_TOOLS` | 랙앤피니언만 +5° |
| 6축 PWM | `tool_profiles.yaml` `axis_goal_pwm`·`goal_pwm` | 그리퍼 800, 굴삭기 400 |
| 교체 후 시험 동작 폭 | `local_tool_change_bridge.py` `TOOL_EXERCISE_TICKS` | 그리퍼 600, 굴삭기 300 |
| 3D 모델 영점 | `operator_console/rover_3d.py` `BRIDGE_TO_CAD` | 5축 -0.18715 |

**이미 푼 판정.** 사용자 지시로 팔이 목표에 못 닿아도 진행한다. 슬롯 저장·교체 시작의 정지 판정, 슬롯 머리 방향 판정, 볼락·도구 인식의 목표 거리·속도 판정을 뺐다. 이동 단계는 전 축이 1초 멈추면 넘어간다. **되살리지 않는다.**

**남긴 판정.** E-stop, 수동 모드, 5축 연결, 교체 시작 시 토크 꺼진 축, 축 하드웨어 오류, `교체 정지`.

## 4. AI 에이전트(Claude·Codex)용 맥락

> 팀원의 에이전트는 작업 전에 이 장과 정본 명세 §2.6~2.10을 읽는다. 저장소 규칙은 `AGENTS.md`/`.claude/CLAUDE.md`.

### 4.1 구성

```
노트북 ──ssh -Y──▶ 젯슨 호스트 (zetin@jetson-orin.local)
                    ├─ GUI: /usr/bin/python3 -m operator_console.app (호스트에서 실행, 화면만 X11)
                    └─ 컨테이너 arm_pr12_jetson (host network, privileged, ROS_DOMAIN_ID=78)
                         ├─ moveit_dynamixel_bridge  — /dev/ttyUSB0(U2D2, 1 Mbps) 단일 소유자
                         ├─ ops_broker  127.0.0.1:29001 (토큰 인증, 콘솔 명령 게이트)
                         └─ arm_console_bridge — UDP 15707(텔레메트리)/15703 → 127.0.0.1의 GUI
GUI → TCP 29001(ops 액션) → 브로커 → ROS 토픽 → 브리지 → 서보
브리지 → /joint_states, /tool/status → 미러 → UDP → GUI
```

| 항목 | 값 |
|---|---|
| 젯슨 레포 체크아웃 | `~/power-train-sw-pr12` (컨테이너에 `/ws`로 **읽기 전용** 마운트) |
| 세션 디렉터리 | `~/pr12-arm-session` (컨테이너 `/session`): `session.yaml`, `arm_poses.json`, `lock_points.json`, `tool_slots.json`, `arm_joint_limits.json`, `ops_console.token`, 로그 `bridge-rt.log`·`ops-rt.log`·`mirror-rt.log` |
| 빌드 산출물 | `/session/runtime` (`bash scripts/build_arm_console_runtime.sh /session/runtime`, symlink install — 파이썬 소스 수정은 재빌드 없이 반영) |
| 브리지 파라미터 정본 | `config/local_arm_bench_teleop.yaml` → 세션 `session.yaml`로 복사해 쓴다 |
| 서보 | 1축 XM540 ID11, 2축 XM540 ID14(9.034:1), 3축 XM540 ID13(4.040:1), 4축 XM540 ID12, 5축 XL430 ID16, 볼락 XL430 ID1 |

### 4.2 절대 하지 말 것 (실제 사고·실측에서 나온 규칙)

- ⚠️ **브리지를 정상 종료(SIGTERM/Ctrl-C/`docker stop`)하지 않는다.** FULL_ROBOT에서 정상 종료는 팔 토크를 끈다 → 팔이 떨어진다.
  재시작은 항상 `scripts/restart_arm_bridge.sh`(SIGKILL)로 한다. 새 브리지가 `Adopted holding`으로 자세를 이어받는다.
  컨테이너를 내려야 하면 먼저 `docker exec arm_pr12_jetson pkill -9 -f '[m]oveit_dynamixel_bridge'`.
- ⚠️ **파워트레인 컨테이너(`powertrain_*`)는 건드리지 않는다.** 부팅 때 자동으로 뜨고 젯슨 CPU를 많이 쓰지만 주행 스택이다.
  can0·chassis 문제는 팔 작업 범위 밖이다(사용자 확인 없이 재시작 금지).
- ⚠️ Dynamixel SDK 내부 `LATENCY_TIMER`를 줄이지 않는다. 5축 SyncRead가 시간 초과돼 **빈 `/joint_states`** 가 나가고
  3D·조그가 끊긴다(2026-11-04 실측, 되돌림). FTDI 지연은 브리지가 기동 시 `ftdi_latency_timer: 1`로 맞춘다. ⚠️ 장치가 열린 상태에서 지연을 쓰면 버스가 약 0.25초 끊기므로 반드시 잠금·포트 열기 **전에** 쓴다(2026-11-06 실측).
- `/dev/ttyUSB0`은 브리지 하나만 연다. 다른 서보 스크립트(`measure_*.py` 등)를 쓸 때는 브리지를 먼저 SIGKILL하거나 `read_only:=true`로 띄운다.
- 영점(`centers`)·기어비를 바꾸면 저장 자세·슬롯·3D 매핑(`operator_console/rover_3d.py` `BRIDGE_TO_CAD`)이 전부 틀어진다. 바꾸기 전에 사용자에게 묻는다.
- 젯슨에 외부 바이너리(py-spy 등)를 내려받아 실행하지 않는다. 젯슨은 인터넷이 없다.
- 팀원의 미커밋·미추적 파일(예: 젯슨 `~/power-train-sw`)은 보존한다. 팔 벤치 체크아웃은 `~/power-train-sw-pr12`다.

### 4.3 자주 쓰는 명령 (어디서: 노트북)

```bash
J=zetin@jetson-orin.local; C=arm_pr12_jetson
# 프로세스 상태
ssh $J "docker exec $C pgrep -fa '[m]oveit_dynamixel|[o]ps_broker|[a]rm_console_bridge' | cut -c1-120"
# 브리지 기동 결과 (✅ 5축 'Torque enabled safely' 또는 'Adopted holding', ❌ 'startup torque read failed')
ssh $J "docker exec $C grep -E 'Torque|Adopt|started|failed' /session/bridge-rt.log | cut -c1-140"
# 브리지만 재시작 (SIGKILL — 토크 유지)
ssh $J "docker exec $C bash /ws/scripts/restart_arm_bridge.sh && echo OK"
# GUI만 끄기 (브리지는 유지)
ssh $J 'pkill -f "[o]perator_console.app --host 127.0.0.1 --ops-host 127.0.0.1 --ops-port 29001"'
# 노트북 수정본을 젯슨 체크아웃에 반영 (파이썬은 재빌드 불필요, 브리지·브로커는 재시작 필요)
rsync -a --relative <변경 파일들> $J:power-train-sw-pr12/
```

### 4.4 정상 판정 기준

- `/joint_states`에 **관절 이름이 채워진 메시지만** 약 19 Hz(빈 메시지 0). 확인(어디서: 노트북):
  ```bash
  ssh zetin@jetson-orin.local "docker exec arm_pr12_jetson bash -c 'source /opt/ros/humble/setup.bash; \
    source /session/runtime/install/setup.bash; export ROS_DOMAIN_ID=78; \
    timeout 4 ros2 topic echo --once /joint_states | head -12'"
  ```
  ✅ `name:`에 `arm_joint_1`~`arm_joint_5`. ❌ `name: []` 이면 서보 미응답 → 브리지 재시작.
- 서보 직접 확인이 필요하면 브리지를 SIGKILL한 뒤 컨테이너에서 dynamixel_sdk로 1 Mbps ping(ID 11·12·13·14·16, 볼락 1). 끝나면 런처나 재시작 스크립트로 브리지를 다시 띄운다.

### 4.5 코드 위치 (수정할 때)

| 기능 | 파일 |
|---|---|
| 팔 탭 화면 | `operator_console/arm_ui/manual_tab.py`, `keyboard_panel.py`, `coupling_lock.py`, `calibration_tab.py` |
| 텔레메트리 → 화면 상태·버튼 허용 | `operator_console/arm_ui_binding.py`, `arm_ui/contracts.py`(`gate`) |
| 명령 생성·안전 정책 | `operator_console/arm_ops/{contract,policy,session,adapter}.py` |
| 3D | `operator_console/rover_3d.py`(관절 매핑), `assets/rover3d/viewer.js` |
| 브로커(명령 검증·전달) | `ros2/src/powertrain_ros/powertrain_ros/ops_broker_node.py`, `ops_contract.py` |
| 브리지(서보 소유) | `extreme-robot/ros2_ws/src/dynamixel_control/dynamixel_control/moveit_dynamixel_bridge.py` 외 `local_wrist_jog.py`·`toolchanger_lock.py`·`arm_range_calibration.py`·`joint_limits.py` |
| 상태 투영(필드 화이트리스트) | `powertrain_observability/tool_snapshot.py` — 새 상태 필드는 여기 통과시켜야 GUI에 도착한다 |
| GUI 없는 터미널·Python 클라이언트 | `operator_console/arm_terminal.py` (가상 검증: `scripts/verify_arm_terminal_mock.py`) |
| 기동 | `scripts/start_jetson_arm_bench.sh`, `scripts/restart_arm_bridge.sh` |

새 ops 액션은 세 곳을 함께 고친다: `arm_ops/contract.py`, `ops_contract.ACTIONS`, `app.py`의 `registered_actions`.

### 4.6 테스트 (어디서: 노트북 레포 루트)

```bash
export PYTHONPATH="$PWD:$PWD/motor_control:$PWD/ros2/src/powertrain_ros"
/usr/bin/python3 -m pytest operator_console/tests/arm_ops operator_console/tests/arm_ui \
    operator_console/tests/test_arm_*.py -q          # ✅ 기존 실패 4건(경계 계약 3·gtk_adapter 1)만
python3 -m pytest powertrain_observability -q
# 브리지·브로커(rclpy·robot_arm_msgs 필요) — 로컬 이미지 extreme-robot-ros2:latest 또는 젯슨 컨테이너에서
```
`operator_console/tests/test_integrated_app.py` 등 GTK 4파일은 노트북 실디스플레이에서 main에서도 segfault한다(판정 제외).
콘솔을 바꾸면 정본 명세 §3 회귀 점검표를 실물로 확인한다.
