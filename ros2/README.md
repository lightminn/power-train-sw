# ros2/ — 파워트레인 ROS2 워크스페이스

로봇팔 팀(`ksp118/extreme-robot`)과 **분리 개발**하는 우리 ROS2 층이다. 각 팀은 자기
노드·컨테이너를 소유하고, `robot_arm_msgs` 계약만 공유하며 DDS(host network, domain 0)로
통신한다.

> **WP5.1 상태 (2026-07-11): HIL 완료.** 기존 `/cmd_vel → 10모터` 실증과 새
> `/safety_verdict`·`/wheel_states`·latched E-stop·실제 50 Hz 결과를 합쳐 완료 판정했다.
> 실행 HEAD `ec452f6474b6fc57437d576298f2bc954649be42`에서 `motor_control` 198,
> `motor_gui` 91, Jetson `powertrain_ros` 32/32가 통과했다. 지상 제동과 최종 `stop_mm`은
> 차체 조립 후 실차 커미셔닝으로 분리한다. 상세는
> [`WP5.1 HIL 보고서`](../docs/reports/2026-07-10-wp5-control-safety-hil.md)를 따른다.

## 구조

```text
ros2/
├── src/
│   ├── robot_arm_msgs/      벤더링 사본(정본=ksp118). VENDORED.md 참조
│   ├── powertrain_msgs/     SafetyVerdict·WheelState·WheelStates
│   └── powertrain_ros/      얇은 내부 ROS 어댑터
│       ├── bringup_node     WP4 공유 메시지 왕복 진단
│       ├── us100_safety     블로킹 UART 측정, 5~10 Hz 판정 발행
│       ├── chassis          50 Hz 최종 안전 집행·10모터·wheel state
│       ├── message_adapter  순수 Python 상태↔ROS 메시지 변환
│       ├── contract.py      로봇팔 공유 문자열의 단일 출처
│       └── launch/
│           └── wp5_control.launch.py  stop_mm 필수인 HIL/생산 결합 기동
└── scripts/
    └── sync_check_msgs.sh   벤더 msg와 로봇팔 정본 드리프트 검사
```

**ROS2는 껍데기다.** 제어·안전 정책과 `can0`·10모터의 단일 소유권은
`../motor_control/`의 순수 Python `SafetyInterlock`·`ChassisManager`에 둔다. 블로킹 가능한
US-100 UART는 별도 프로세스에서만 실행해 50 Hz 차체 tick을 지연시키지 않는다. 이 구조는
ROS 없는 텔레옵에도 같은 latch/reset/arm 의미를 재사용하면서, 최종 E-stop 권한을
`ChassisManager` 한 곳에 유지한다.

```text
US-100 UART
  └─ us100_safety_node (5~10 Hz)
       └─ /safety_verdict (RELIABLE, depth 1)
            ↓ latest cache
/cmd_vel → chassis_node (50 Hz)
            ├─ SafetyInterlock → ChassisManager
            ├─ can0 500 kbps → AK45-36 ×4 + ODrive/BL70200 ×6
            └─ /wheel_states (50 Hz)
```

## 환경

- 실행 위치: Jetson Orin Nano의 `powertrain_ros` 컨테이너
- ROS: Humble, host network, 기본 `ROS_DOMAIN_ID=0`
- 모터 버스: 단일 `can0`, 500 kbps, AK id 1~4 + ODrive node 11~16
- US-100: `/dev/ttyTHS1`, 9600 baud, 5~10 Hz
- 실기 전제: 시나리오 1~8은 바퀴 6개 부양, 시나리오 9는 별도 승인한 50 kg 지상주행;
  두 단계 모두 48V 물리 E-stop 접근과 경쟁 chassis/teleop 프로세스 없음

## 빌드와 실행

Jetson에 SSH 접속한 직후 홈 `~`에서 시작한다. 생산 Gateway가 실행 중인 컨테이너에서
`colcon build/test`를 수행하면 live install을 바꾸므로 금지한다. 빌드·시험이 필요하면 먼저
Gateway를 정상 정지하고, entrypoint를 덮어쓴 one-off 컨테이너에서 수행한다.

```bash
set -eu
cd ~/power-train-sw
docker compose -f docker/docker-compose.jetson.yml stop powertrain_ros
docker compose -f docker/docker-compose.jetson.yml build powertrain_ros
docker compose -f docker/docker-compose.jetson.yml run --rm --no-deps \
  --entrypoint bash powertrain_ros -lc '
    source /opt/ros/humble/setup.bash
    cd /workspace/ros2
    colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros
    source install/setup.bash
    colcon test --packages-select robot_arm_msgs powertrain_msgs powertrain_ros
    colcon test-result --verbose
  '
docker compose -f docker/docker-compose.jetson.yml up -d canwatchdog powertrain_ros
```

이 one-off 절차도 bind-mounted `ros2/build`, `install`, `log`를 갱신하므로 반드시 생산
`powertrain_ros`가 정지한 상태에서만 사용한다. live Gateway의 install을 병행 수정하지 않는다.

### L515 전용 빌드·preflight·launch

생산 컨테이너는 `docker/powertrain_ros_entrypoint.sh`로 headless Gateway를 직접 관리한다.
Dashboard는 `docker exec -it powertrain_ros python3 -m l515_dashboard`로 접속하며, `q`는
클라이언트만 종료한다. 상세 운용 계약은 [`l515_dashboard/README.md`](../l515_dashboard/README.md)다.

L515는 파워트레인 소유 serial `00000000F0271544`만 사용한다. D435i serial
`250222071245`는 로봇팔 전용이며 이 파이프라인에서 열지 않는다. `powertrain_ros` 이미지의
librealsense/pyrealsense2 pin은 **v2.50.0**이고 RSUSB backend를 쓴다. SDK가 L515를
`f0271544`로 표기할 수 있어 비교 시 대소문자와 선행 0만 정규화하고 정확히 하나의 동일
serial 장치가 아니면 실패한다. Jetson 호스트에서
아래 순서 그대로 이미지 빌드와 USB3/SDK fail-closed preflight를 수행한다.

Orin Nano에는 NVENC가 없으므로 SRT는 의도적으로 `videoconvert → x264enc` SW 경로를 쓴다.
Gateway status의 native callback Hz, ROS 토픽별 Hz, SRT submit/sent/drop Hz,
aligned-depth age, CPU/RSS를 60초 성능 인수의 정본 계측으로 기록한다.

```bash
set -eu
cd ~/power-train-sw
docker compose -f docker/docker-compose.jetson.yml build powertrain_ros
docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros
bash scripts/l515_preflight.sh
```

컨테이너 entrypoint는 fresh checkout에서 아래 3패키지를 자동 build하고 setup을 source한 뒤
`python3 -m l515_dashboard.gateway_main`을 exec한다. 설치 후 source가 더 새로우면 자동 재빌드한다.
따라서 Gateway와 동시에 레거시 `l515.launch.py`를 별도로 실행하지 않는다.

```bash
docker logs powertrain_ros
docker exec -it powertrain_ros python3 -m l515_dashboard
```

`restart: on-failure:5`는 연속된 짧은 startup 실패를 최대 5회로 제한한다. Docker의 재시작
카운터는 컨테이너가 약 10초 이상 정상 실행되면 초기화되므로, 장시간 정상 실행 뒤 새 crash에는
새 retry budget이 적용된다. 확인된 `Shift+Q`는 정상 종료 0이어서 재시작하지 않고 정지 상태를
유지한다. 재기동은 `docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros`로
명시적으로 수행한다.

발행 토픽은 `/l515/color/image_raw`, `/l515/color/camera_info`,
`/l515/depth/image_rect_raw`, `/l515/depth/camera_info`, `/l515/gyro/sample`,
`/l515/accel/sample`이다. color는 1280×720@30, raw depth는 640×480@30이며 IMU 두 토픽은
장치 원본 stream이다. IR, confidence, alignment, 합성 IMU와 **PointCloud2 토픽은 없다.**

2026-07-11 D435i perception 동시 60초 HIL에서 color/depth 29.750/29.450 Hz,
accel/gyro 30.166 Hz, 최소 5초 창 28.8 Hz, 비증가 stamp 0, USB error delta 0을 확인했다.
SDK frameset이 같은 video sample을 다시 줄 수 있으므로 source는 스트림별 동일 device
timestamp를 폐기한다. 역행 timestamp는 mapper reset을 위해 보존한다.

### 실기 launch 게이트

`wp5_control.launch.py`는 `stop_mm`을 생략할 수 없는 결합 실기 진입점이다. 현재 벤치/HIL
값은 200 mm이고 생산 기본값은 없다. 차체 조립 후 50 kg 실차 지상 커미셔닝에서 제동거리를
측정해 최종 운용값을 튜닝한다. 이는 완료된 WP5.1 HIL과 별도다.

먼저 사용자가 물리 준비를 명시적으로 확인해야 한다.

- 시나리오 1~8: 바퀴 6개 완전 부양, 48V 물리 E-stop 접근, 회전체 배제구역·감시자 확보
- 시나리오 9: 별도 통제 주행로, 단계적 저속, spotter, 배제구역, 물리 E-stop 확보 후
  **바퀴를 내리기 직전 별도 사용자 확인**. 앞 단계의 부양 확인을 승계하지 않는다.

확인 뒤 Jetson 호스트에서 다음 host-wide preflight를 순서대로 실행한다. 컨테이너 안의
`ps` 하나를 호스트 전체 검사로 해석하지 않는다. 아래 검사는 Jetson 호스트 프로세스와 실행
중인 **모든** 컨테이너의 프로세스를 각각 확인한다.

```bash
set -eu
cd ~/power-train-sw
git rev-parse HEAD
test "$(docker inspect -f '{{.State.Running}}' powertrain_canwatchdog)" = "true"
test "$(docker inspect -f '{{.State.Running}}' powertrain_ros)" = "true"

CONTROL_RE='[r]os2 .*powertrain_ros|[c]hassis([_. /]|$)|[t]eleop|[m]otor_gui|[c]an_drive|[c]alibrat(e|ion|_all)|[o]drive|[a]k_control|[a]k.*(can|motor|drive)'

PS_SNAPSHOT=$(ps -eo pid=,user=,args=)
HOST_CONTROL=$(printf '%s\n' "$PS_SNAPSHOT" | grep -Ei "$CONTROL_RE" || true)
CONTAINER_CONTROL=$(
  for container in $(docker ps --format '{{.Names}}'); do
    top_output=$(docker top "$container" -eo pid,user,args) || exit 1
    rows=$(printf '%s\n' "$top_output" | grep -Ei "$CONTROL_RE" || true)
    if [ -n "$rows" ]; then
      printf '%s\n' "$rows" | sed "s/^/$container: /"
    fi
  done
  true
)

if [ -n "$HOST_CONTROL" ] || [ -n "$CONTAINER_CONTROL" ]; then
  echo "ABORT: unexpected motor-control process before launch" >&2
  printf '%s\n%s\n' "$HOST_CONTROL" "$CONTAINER_CONTROL" >&2
  exit 1
fi
```

`powertrain_canwatchdog`는 반드시 running이어야 한다. 워치독은 TX wedge probe용
AF_CAN raw socket을 열고 `can0`에 bind하지만 빈 RX filter를 설정한 **TX-only** 소켓이다.
따라서 아래 receiver 목록에는 나타나지 않으며, 임의의 다른 TX-only 소켓도 이 목록으로는
찾을 수 없다. 프로세스/container allowlist는 이 사각지대의 운영상 완화책일 뿐 완전한 증명이
아니다. 이름을 바꾼 TX-only 소유자가 남을 잔여 위험이 있으므로, 프로세스가 하나라도 검출되면
launch하지 말고 소유자·작업 목적을 확인한다. 알 수 없는 팀원 프로세스를 자동 `kill`하지 않는다.

프로세스 감사 뒤 launch 전 CAN receiver가 0개인지 확인한다.

```bash
test -d /proc/net/can
CAN_FILES=$(find /proc/net/can -maxdepth 1 -type f -name 'rcvlist_*' -print)
test -n "$CAN_FILES"
CAN_RECEIVERS=$(awk '
  ($1 == "can0" || $1 == "any") && $2 ~ /^[[:xdigit:]]+$/ && (length($2) == 3 || length($2) == 8) {
    print FILENAME ":" $0
  }
' $CAN_FILES)
if [ -n "$CAN_RECEIVERS" ]; then
  echo "ABORT: unexpected CAN receiver before chassis launch" >&2
  printf '%s\n' "$CAN_RECEIVERS" >&2
  exit 1
fi
```

그 다음 sticky loopback을 명시적으로 끄고 저장소 스크립트로 CAN을 올린다.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can loopback off
./scripts/can_setup.sh
ip -details -statistics link show can0
```

출력에서 `bitrate 500000`, `restart-ms 100`, loopback 비활성, 초기 tx/rx error·bus-off·restart
counter를 확인·기록한 뒤에만 컨테이너에서 launch한다. `<provisional-mm>`은 실제 통제 HIL
임시값으로 바꿔야 하며 그대로 복사하면 안 된다.

```bash
docker exec -it powertrain_ros bash
cd /workspace/ros2
source install/setup.bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=<provisional-mm>
```

아직 arm하거나 `/cmd_vel`을 보내지 않는다. 별도 SSH 터미널의 Jetson 호스트에서 launch 뒤
단일 소유권을 다시 확인한다. 허용되는 제어 프로세스는 `powertrain_ros` 컨테이너의 launch
supervisor 1개와 chassis 실행기 1개뿐이다. `us100_safety`는 CAN을 열지 않는다.

```bash
set -eu
cd ~/power-train-sw
CONTROL_RE='[r]os2 .*powertrain_ros|[c]hassis([_. /]|$)|[t]eleop|[m]otor_gui|[c]an_drive|[c]alibrat(e|ion|_all)|[o]drive|[a]k_control|[a]k.*(can|motor|drive)'

PS_SNAPSHOT=$(ps -eo pid=,user=,args=)
ROS_TOP=$(docker top powertrain_ros -eo pid,user,args)
LAUNCH_COUNT=$(printf '%s\n' "$ROS_TOP" | grep -Ec \
  '[r]os2 .*powertrain_ros wp5_control\.launch\.py' || true)
CHASSIS_COUNT=$(printf '%s\n' "$ROS_TOP" | grep -Ec \
  '/powertrain_ros/chassis([[:space:]]|$)' || true)
test "$LAUNCH_COUNT" -eq 1
test "$CHASSIS_COUNT" -eq 1

ROS_CHASSIS_PIDS=$(printf '%s\n' "$ROS_TOP" | awk \
  '/\/powertrain_ros\/chassis([[:space:]]|$)/ {print $1}' | sort -n -u)
HOST_CHASSIS_PIDS=$(printf '%s\n' "$PS_SNAPSHOT" | awk \
  '/\/powertrain_ros\/chassis([[:space:]]|$)/ {print $1}' | sort -n -u)
test "$(printf '%s\n' "$ROS_CHASSIS_PIDS" | sed '/^$/d' | wc -l)" -eq 1
test "$HOST_CHASSIS_PIDS" = "$ROS_CHASSIS_PIDS"

HOST_UNEXPECTED=$(printf '%s\n' "$PS_SNAPSHOT" | grep -Ei "$CONTROL_RE" | grep -Ev \
  '[r]os2 .*powertrain_ros wp5_control\.launch\.py|/powertrain_ros/chassis([[:space:]]|$)' || true)
CONTAINER_UNEXPECTED=$(
  for container in $(docker ps --format '{{.Names}}'); do
    top_output=$(docker top "$container" -eo pid,user,args) || exit 1
    rows=$(printf '%s\n' "$top_output" | grep -Ei "$CONTROL_RE" || true)
    if [ "$container" = "powertrain_ros" ]; then
      rows=$(printf '%s\n' "$rows" | grep -Ev \
        '[r]os2 .*powertrain_ros wp5_control\.launch\.py|/powertrain_ros/chassis([[:space:]]|$)' || true)
    fi
    if [ -n "$rows" ]; then
      printf '%s\n' "$rows" | sed "s/^/$container: /"
    fi
  done
  true
)
if [ -n "$HOST_UNEXPECTED" ] || [ -n "$CONTAINER_UNEXPECTED" ]; then
  echo "ABORT: unexpected second motor-control owner after launch" >&2
  printf '%s\n%s\n' "$HOST_UNEXPECTED" "$CONTAINER_UNEXPECTED" >&2
  exit 1
fi

CAN_FILES=$(find /proc/net/can -maxdepth 1 -type f -name 'rcvlist_*' -print)
test -n "$CAN_FILES"
POST_CAN_RECEIVERS=$(awk '
  ($1 == "can0" || $1 == "any") && $2 ~ /^[[:xdigit:]]+$/ && (length($2) == 3 || length($2) == 8) {
    print FILENAME ":" $0
  }
' $CAN_FILES)
test -n "$POST_CAN_RECEIVERS"
printf '%s\n' "$POST_CAN_RECEIVERS"
```

chassis 한 프로세스가 10모터용 SocketCAN socket을 여러 개 열므로 receiver 행 수를 소유자
수로 해석하지 않는다. host `ps` snapshot의 chassis PID 집합이 `docker top powertrain_ros`의
단 하나인 chassis PID와 정확히 같아야 하므로 native·다른 container 중복도 실패한다. receiver
목록은 launch 전 예기치 않은 RX socket 부재와 launch 후 의도한 RX socket 출현을 보여줄 뿐,
TX-only socket 소유권을 증명하지 않는다. 위 post-launch 게이트가 실패하면 arm하지 말고 launch 터미널에서
`Ctrl-C`로 의도한 launch를 종료한다. 필요하면 물리 E-stop을 누른 뒤 원인을 조사하며, 알 수
없는 프로세스를 자동 종료하지 않는다.

시나리오 9에서 산정·승인·재검증한 뒤 생산 명령은 다음 형식만 허용한다.

```bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=<HIL-approved-mm>
```

노드를 직접 분리 실행하는 경로는 진단 전용이며 생산 운용에 사용하지 않는다.

```bash
ros2 run powertrain_ros us100_safety --ros-args -p stop_mm:=<diagnostic-mm>
ros2 run powertrain_ros chassis
ros2 run powertrain_ros bringup
```

FAKE 차체는 무하드웨어 배선 검증 전용이다. 기본 `safety_required=true`와 별도 테스트
publisher로 안전 경로까지 검증한다. 안전 publisher 없이 차체만 의도적으로 시험하는 경우에만
BENCH/FAKE에서 `safety_required=false`를 명시할 수 있다.

```bash
ros2 run powertrain_ros chassis --ros-args -p fake:=true
ros2 run powertrain_ros chassis --ros-args \
  -p fake:=true -p safety_required:=false
```

> ⚠️ `safety_required=false`는 BENCH/FAKE 전용 우회다. 실기 launch나 자율주행에서
> 사용하지 않는다.

## 핵심 안전 파라미터

| 노드 | 파라미터 | 생산 기본 | 계약 |
|---|---|---:|---|
| `chassis_node` | `cmd_timeout` | 0.5 s | 만료 시 자동복구 `MOTION_HOLD` |
| `chassis_node` | `safety_required` | `true` | `false`는 BENCH/FAKE 전용 |
| `chassis_node` | `safety_topic_timeout` | 0.75 s | 최솟값도 0.75 s; `age > threshold` 다음 50 Hz tick에 latched `ESTOP` |
| `chassis_node` | `safety_startup_timeout` | 1.0 s | 첫 판정 미수신 시 latched `ESTOP` |
| `us100_safety_node` | `sample_hz` | 5.0 Hz | 허용 범위 5~10 Hz |
| `us100_safety_node` | `fail_stop_count` | 3 | 거리와 0x50 생존 확인 모두 연속 실패한 횟수 |
| `us100_safety_node` | `stop_mm` | 생산 결합 launch 기본 없음 | 결합 launch에 필수; 차체 조립 후 지상 커미셔닝의 제동 실측으로 최종 운용값 튜닝 |

0.75초 freshness 계약은 `age > 0.75 s` 조건이 참이 된 뒤 다음 20 ms tick에 집행되므로
명목 E-stop 시점은 마지막 판정 후 0.75~0.77초다. 이는 0.5초 `/cmd_vel` watchdog과 서로
다른 고장 경로다. 200 mm는 현재 벤치/HIL 값이며 최종 운용값은 차체 조립 후 커미셔닝한다.

## 토픽과 서비스 계약

| 방향 | 이름 | 타입 | 주기/의미 |
|---|---|---|---|
| 구독 | `/cmd_vel` | `geometry_msgs/Twist` | 최종 주행 명령; 0.5초 timeout은 `MOTION_HOLD` |
| 구독 | `/safety_verdict` | `powertrain_msgs/SafetyVerdict` | RELIABLE depth 1, US-100 최신 판정 |
| 구독 | `/arm_status` | `robot_arm_msgs/ArmStatus` | WP8가 사용할 로봇팔 완료 이벤트 |
| 발행 | `/wheel_states` | `powertrain_msgs/WheelStates` | 명목 50 Hz, 6바퀴 실측 상태·tick 시간·overrun |
| 발행 | `/chassis_mode` | `robot_arm_msgs/ChassisMode` | 로봇팔 자세 의도 |
| 발행 | `/chassis_state` | `robot_arm_msgs/ChassisMode` | 현재 차체 진단 문자열 |
| 발행 | `/arrival_status` | `robot_arm_msgs/ArrivalStatus` | WP8 도착 이벤트 훅 |
| 서비스 | `/chassis_node/arm` | `std_srvs/Trigger` | `IDLE`에서 별도 arm; latch 중 거부 |
| 서비스 | `/chassis_node/disarm` | `std_srvs/Trigger` | 모터를 `IDLE`로 내림 |
| 서비스 | `/chassis_node/estop` | `std_srvs/Trigger` | 수동 latched `ESTOP` |
| 서비스 | `/chassis_node/reset_estop` | `std_srvs/Trigger` | 활성 위험 해소 뒤 `IDLE`; 자동 arm 안 함 |

`SafetyVerdict.status`는 다음 네 상태만 사용한다.

| 상태 | 의미 | 차체 정책 |
|---|---|---|
| `CHECKING` | 기동 또는 거리·생존 응답 1~2회 누락 | 자동복구 `MOTION_HOLD` |
| `VALID` | 20~4000 mm 유효 거리 | `< stop_mm`이면 latched `ESTOP`, 아니면 `RUN` |
| `INVALID_READING` | 거리는 무효지만 0x50 응답 있음 | `RUN` |
| `NO_RESPONSE` | 거리와 0x50 응답이 연속 3회 모두 없음 | latched `ESTOP` |

0x50 응답은 US-100의 MCU/UART 생존만 증명한다. 초음파 송신기·수신기 고장을 배제하지
못하므로 `INVALID_READING` 정상 통과는 의도적으로 수용한 잔여 위험이다.

정지 상태의 의미는 다음과 같다.

- `RUN`: 주행 허용.
- `MOTION_HOLD`: `CHECKING`, `/cmd_vel` timeout, 연결 단절처럼 원인 해소 후 자동복구 가능한
  구동 억제. `/cmd_vel` timeout은 새 명령이 와야 해제된다.
- `ESTOP`: 유효 근거리, 확인된 `NO_RESPONSE`, safety topic startup/stale, 모터 fault/stale,
  수동 정지처럼 reset 전까지 유지되는 정지. reset과 arm은 반드시 별도 단계다.

## 검증 상태

- 배포 HEAD `c3610c136357a8c881263926ec18bcd7e3432a5d`에서 root가 직접 관찰한 로컬 결과:
  `motor_control` **189 passed** (`.superpowers/sdd/final-motor-control-c3610c1.xml`),
  `motor_gui` **91 passed** (`.superpowers/sdd/final-motor-gui-c3610c1.xml`).
- 같은 HEAD의 격리 read-only ROS 워크스페이스에서 `robot_arm_msgs`·`powertrain_msgs`·
  `powertrain_ros` 3패키지 clean build와 `powertrain_ros` **31/31 passed**
  (`.superpowers/sdd/final-ros-c3610c1.xml`).
- Jetson도 정확히 같은 HEAD에서 3패키지 build와 `powertrain_ros` **31/31 passed**. raw XML은
  `/home/zetin/power-train-sw/ros2/build/powertrain_ros/pytest.xml`.
- Jetson software-only FAKE(commit `49831bb42058a177ed9c41d72d0273f4f0a8f535`): **PASS**.
  startup `ESTOP`; far `ARMED/RUN`; 60초 count 3000, mean/minimum 5 s window 50.000 Hz,
  tick p99 0.280 ms, overrun 0, max interval 21.453 ms; near `ESTOP`; far 뒤 latch;
  reset→`IDLE`이며 implicit arm 없음; separate arm; publisher-death `ESTOP` delay 0.753 s.
  이 FAKE tool capture는 파일로 보존되지 않아 최종 재실행 raw log가 대기 중이다.
- WP5.1 Jetson/10모터/US-100 HIL: **NOT RUN**.

환경 확인, 아홉 시나리오, 시간·CAN counter, `stop_mm` 산정과 최종 go/no-go는
[`2026-07-10-wp5-control-safety-hil.md`](../docs/reports/2026-07-10-wp5-control-safety-hil.md)에만 기록한다.

## 로봇팔 공유 계약

`src/powertrain_ros/powertrain_ros/contract.py`가 문자열 어휘의 단일 출처다. 남은 통합
항목은 `MISSION_STOP`, 락 해제 순서, 그리고
`ARRIVED_* → 팔 작업 → DONE → 재출발` 합동 1사이클이다.
