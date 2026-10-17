# PR #12 통합 콘솔 로봇팔 조작 기능 명세

사용자가 GUI라고 지칭하는 화면은 `operator_console`의 PR #12 통합 운용 콘솔이다.
Qt `robot_manual_gui`는 이번 조작 화면으로 사용하지 않는다.

> **개발 원칙.** 이 문서의 기능은 콘솔 위에 이미 쌓아 둔 것이다. 파워트레인 GUI가 갱신되면
> 새로 만들지 않고 **이 기능 위에서** 이어 개발하며, 아래 §3 회귀 점검표를 매번 통과시킨다.

## 1. 기본 동작 (PR #12, 벤치 확장 OFF)

- USB 1 Mbps, XL430 ID16 = 5축, ID1 = 볼락. 1~4축 기존 캘리브레이션은 변경하지 않는다.
- 2026-10-17 노트북 시험 기준: 5축 0° = 2525 tick, 볼락 LOCKED = 3051 tick.
- 명시적 `manual_coupling_lock=true` + FULL_ROBOT + MANUAL에서만 로컬 시험을 제공한다.
  기본값 false와 기존 END_EFFECTOR_ONLY 게이트는 유지한다.
- 통합 콘솔 → 인증 ops → ROS 브리지 → 단일 USB 소유자 → 실제 피드백으로 연결한다.
- 로컬 조그는 5축만 허용한다. 갱신당 최대12 tick(1.0546875°), 영점±114 tick
  (±10.01953125°)로 제한한다. 명령의 단조시각 만료는 1초 이내여야 하며 콘솔 TTL은0.6초다.
  수신 중단·MANUAL 상실·잠금 준비 상실 시 현재 위치를 홀드한다.
- 단조시각 계약은 **같은 호스트와 Docker 컨테이너 사이 전용**이다. 노트북 GUI→Jetson
  원격 조그는 인수 범위가 아니다(GUI 프로세스는 Jetson에서 돌고 화면만 X11로 넘긴다).
- 볼락 목표는256 tick(22.5°)이며 위치 허용오차는 기본4 tick(0.3515625°)이다.
  활성화는 현재 위치를 먼저 동기화하며, 그 인덱스 안의4 tick 이하 오차만 정착시킨다.
  다른 인덱스로 자동 이동하지 않는다. 팔 조그 해제·정지 확인 전 잠금 전환은 거부한다.
- STOP은5축과 잠금 토크OFF, 조그 버튼을 놓으면5축 현재 위치 홀드다.
  재활성화는 잠금 준비 후 현재 위치를 동기화한다.
- 결합 센서는 없다. LOCKED/UNLOCKED는 엔코더 판정이며 실제 결합 확인이 아니다.

## 2. Jetson 벤치 확장 (2026-10-17, 기본 OFF)

모든 확장은 브리지 파라미터 또는 환경변수로만 켜진다. 끄면 §1 동작 그대로다.
세션 설정 정본은 `config/local_arm_bench_teleop.yaml`, 저장 자세는
`config/local_arm_poses.json`, 볼락 기록 위치는 `config/local_arm_lock_points.json`이다.

### 2.1 스위치

| 위치 | 이름 | 값 | 의미 |
|---|---|---|---|
| 브리지 | `local_jog_all_axes` | true | 조그 대상 1~5축, 이미 토크 ON인 팔 서보를 재시작 시 현재 위치로 인수 |
| 브리지 | `local_wrist_lock_gate` | false | 조그·자세가 볼락 준비 상태를 요구하지 않음 |
| 브리지 | `local_jog_step_rad` | 0.14 | 갱신(0.2초)당 관절 이동량 ≈ 0.7 rad/s (원 텔레옵 `jog_velocity_rad_s`) |
| 브리지 | `arm_profile_velocity` / `profile_acceleration` | 50 / 25 | 원 텔레옵 `position_node` 기본값 |
| 브리지 | `local_jog_trip_threshold` | 500 | 전류 트립 문턱(Present Current/Load raw) |
| 브리지 | `local_poses_file` | `/session/arm_poses.json` | 자세 저장 파일(teleop_core 와 같은 `{이름: {관절: rad}}`) |
| 브리지 | `local_pose_profile_velocity` | 20 | 축별 속도 미지정 시 자세 이동 프로파일 |
| 브리지 | `local_pose_joint_speeds_deg_s` | [10, 30, 15, 15] | 자세 이동 관절 속도(1~4축). 서보 Velocity Limit 으로 상한 |
| 브리지 | `local_pose_joint_delays_s` | [0, 0, 0, 1.0] | 자세 이동 시작 후 축별 출발 지연(4축 1초) |
| 브리지 | `toolchanger_two_point` | true | 볼락 2점 방식(아래 2.5) |
| 브리지 | `toolchanger_points_file` | `/session/lock_points.json` | 볼락 기록 위치 |
| 브리지 | `toolchanger_state_tolerance_ticks` | 20 | 볼락 상태 판정·도달 허용오차 |
| 브로커 env | `ARM_LOCAL_ALL_AXES=1` | | 1~5축 조그·자세·자유지정 액션 허용 |
| 콘솔 env | `ARM_LOCAL_ALL_AXES=1` | | 축 선택, 제한/트립 토글 표시 |
| 콘솔 env | `ARM_LOCAL_WRIST_IGNORE_LOCK=1` | | 볼락 미준비여도 조그 버튼 활성 |

### 2.2 키보드 텔레옵

- 축 선택: 숫자 1~5 / 숫자패드 1~5(NumLock 무관) / 축 버튼. 6번은 그리퍼 도구가 있을 때만.
- ↑ / W = + 방향, ↓ / S = − 방향, **누르는 동안만** 이동한다.
  콘솔이 0.2초마다 갱신하고 브리지는 매 갱신 `현재 위치 + 스텝`만 목표로 쓰므로 목표가
  한 스텝 이상 앞서지 않는다. 놓거나 0.6초 갱신이 끊기면 현재 위치 홀드.
- X11 자동반복의 떼기→누르기 쌍은 120ms 유예로 하나의 누름으로 합친다.
- 키는 `로봇팔 수동조작` 탭이 보이고 창에 포커스가 있을 때만 받는다(입력칸 포커스 제외).
  `app.py`가 `ArmManualTab.attach_keys(window)`를 호출해야 한다.
- **관절 제한 토글**: ON이면 1~4축은 `joint_limits.py`, 5축은 영점±114 tick. 범위 밖으로
  **더 나가는 방향만** 막고 범위 밖에서 안쪽으로는 움직인다(경계로 끌려오지 않음).
- **전류 트립 토글**: ON이면 갱신마다 전류/부하를 읽어 문턱 초과 시 홀드(토크는 유지).
- 정지(Space)는 기존 의미(5축·잠금 토크OFF)다. 복귀(T)로 되살린다.

### 2.3 자세 관리

- 저장 대상은 **1~4축**이다. 5축·그리퍼는 원 텔레옵과 같이 제외한다.
- `현재 자세 저장`: 이름 입력 후 현재 각도를 저장한다(이름은 소문자로 정규화).
- `선택 자세로 이동` / `home`(H): 저장 자세로 이동한다. `home`은 이름이 `home`인 자세.
  - 관절 제한 ON이고 저장 자세가 범위 밖이면 **잘라서 가지 않고 이동 전체를 거부**한다.
  - 축별 속도·지연은 2.1 표를 따른다(2축 최우선 ≈19.5°/s, 4축 1초 뒤 ≈15°/s).
  - 정지·새 조그·MANUAL 상실·E-stop·전류 트립(축 출발 0.5초 뒤부터) 시 전 축 홀드.
  - 도착(±20 tick) 또는 홀드 시 조그 프로파일로 되돌린다.
- `자세 자유 지정`: 1~4축 토크 OFF(손으로 자세 잡기). **팔을 받친 뒤 누른다.**
- `자유 지정 해제`: 현재 위치를 목표로 맞춘 뒤 1~4축 토크 ON.
- 거부 사유는 GUI가 아니라 브리지 로그(`local pose rejected/held`)에 남는다.

### 2.4 ops 액션 (브로커 `ops_contract.ACTIONS`, 콘솔 `registered_actions`)

| 액션 | 브리지 토픽·명령 |
|---|---|
| `robot_arm_jog` / `robot_arm_jog_stop` | `/arm/local_wrist_jog` JOG/STOP (+`limits`,`trip`) |
| `robot_arm_stop` / `robot_arm_resume_hold` | `/arm/local_wrist_jog` DISABLE/ENABLE |
| `robot_arm_pose_save` / `_move` / `_delete` / `robot_arm_home` | `/arm/local_pose` SAVE/MOVE/DELETE/HOME |
| `robot_arm_freedrive` / `robot_arm_freedrive_release` | `/arm/local_pose` FREEDRIVE/FREEDRIVE_RELEASE |
| `robot_arm_lock_command` / `robot_arm_lock_stop` | `/toolchanger/command` |

콘솔의 액션은 세 곳이 모두 맞아야 나간다: `arm_ops/contract.py`(파라미터 명세·설명),
`ops_contract.ACTIONS`(브로커), `app.py`의 `registered_actions`(콘솔 게이트).

### 2.5 볼락 2점 방식

- `현재 위치를 결합 영점으로` = 잠김 위치 기록, `현재 위치를 분리 영점으로` = 열림 위치 기록.
  토크 ON/OFF 무관. 두 기록이 ±40 tick 안이면 새 기록만 남긴다.
- 상태: 잠김 ±20 = LOCKED, 열림 ±20 = UNLOCKED, 그 밖 = BETWEEN(UNKNOWN 아님).
- `도구 결합`/`도구 해제`는 항상 활성이다. 기록이 있으면 그 절대 위치로, 없으면 현재 위치에서
  22.5°(256 tick) 회전한다(0~4095를 넘으면 반대 방향). 토크 OFF면 자동 활성화.
- `잠금 모터 활성화` / `잠금 모터 비활성화`(토크 OFF) / `잠금 정지`를 카드에 노출한다.
- 2026-10-17 기록값: 잠김 3064, 열림 2837.

### 2.6 Jetson 기동 절차 (레포 = 이 PR 체크아웃, 세션 디렉터리 = `~/pr12-arm-session`)

1. 세션에 `config/local_arm_bench_teleop.yaml`을 `session.yaml`로, 자세·볼락 파일을
   `arm_poses.json`·`lock_points.json`으로 복사하고 `ops_console.token`(0600)을 만든다.
2. `extreme-robot-ros2` 컨테이너를 `--network host --privileged -e ROS_DOMAIN_ID=78`,
   `-v 세션:/session -v /dev:/dev -v 레포:/ws:ro`로 띄우고, `extreme-robot/ros2_ws/src`의
   `robot_arm_msgs dynamixel_control robot_manual_gui`, 이어서 `ros2/src`의
   `powertrain_msgs powertrain_ros`를 `/session`에 `--symlink-install`로 빌드한다.
3. **`/sys/bus/usb-serial/devices/ttyUSB0/latency_timer`를 1로** 쓴다(재연결·재부팅 시 16으로 복귀).
4. 브리지(`--params-file /session/session.yaml`, `PYTHONUNBUFFERED=1`) → ops 브로커
   (`port:=29001`, `ARM_LOCAL_ALL_AXES=1`; 19001은 운영 스택이 사용) → `arm_console_bridge`.
5. 콘솔은 Jetson 호스트에서 `ssh -Y -C -o ServerAliveInterval=5` 로 띄운다.
   `ARM_LOCAL_ALL_AXES=1 ARM_LOCAL_WRIST_IGNORE_LOCK=1`, `--ops-port 29001`.
   Jetson 호스트에 gst-libav가 없으면 이미지의 `libgstlibav.so`와 누락 라이브러리를
   세션 하위 폴더에 두고 `GST_PLUGIN_PATH`/`LD_LIBRARY_PATH`로만 적용한다.
6. GUI에서 `수동 제어 요청` → 조작. 브리지를 재시작하면 모드는 FSM으로 돌아간다.

## 3. 회귀 점검표 (GUI 갱신 때마다)

| # | 점검 | 근거 |
|---|---|---|
| 1 | 숫자패드 선택 → ↑ 누름 동안 `/arm/local_wrist_jog` JOG가 0.2초 간격으로 이어진다 | 콘솔 키 연결·갱신 |
| 2 | 브리지 로그에 `expired or foreign-clock` 거부가 늘지 않는다 | USB latency·버스 부하 |
| 3 | 키를 오래 눌러도 "응답 없음"이 뜨지 않는다 | 성공 조그 이벤트 기록 억제 |
| 4 | 관절 제한 ON에서 범위 밖 자세 이동이 거부되고 일부 축만 움직이지 않는다 | 2.3 |
| 5 | 자세 이동 시 2축이 가장 빠르고 4축이 1초 뒤 출발한다 | 2.1 |
| 6 | 자유 지정 → 저장 → 해제 후 자세가 튀지 않는다 | 2.3 |
| 7 | 볼락 결합/해제가 기록 위치 ±20 안에 멈추고 상태가 LOCKED/UNLOCKED로 바뀐다 | 2.5 |
| 8 | 확장 스위치를 모두 끄면 §1 기본 동작과 기존 테스트가 그대로다 | 기본 OFF |

## 4. 알려진 제약

- 노트북으로 X11 포워딩한 화면은 무겁다. Jetson 모니터에서 직접 띄우는 것이 가장 빠르다.
- 이벤트 기록(`EventLog._render`)은 항목마다 최근 100줄을 다시 만든다. 고빈도 이벤트를
  기록에 넣으면 GUI가 멈춘다 — 성공한 조그·조그 정지와 그 ACK는 기록하지 않는다.
- 브리지는 레지스터를 개별로 읽는다. FTDI latency 16ms에서는 버스가 포화돼 조그가 만료된다.
- 결합 센서·슬롯 점유 센서가 없다. LOCKED/UNLOCKED는 위치 판정이다.
