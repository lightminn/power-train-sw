# WP5.1 차체 제어·안전 HIL 검증 보고서

> **상태: PARTIAL HIL / NO-GO — ODrive node 13·14 부재**
>
> 작성 기준일: 2026-07-11 KST
> 범위: 파워트레인 SW의 50 Hz 차체 제어, US-100 안전 경로, 10모터 단일 CAN, ROS 계약
> 소프트웨어 상태: **Tasks 1~8 완료**
> 실행 HEAD `ec452f6474b6fc57437d576298f2bc954649be42`. 실제 Jetson·CAN·US-100 HIL을
> 수행했으나 ODrive node 13·14가 물리적으로 부재해 10모터 완전체와 Phase B 지상 제동은
> 실행하지 않았다. present ODrive 11·12·15·16은 CAN 캘리브레이션 4/4 성공했고 AK 1~4와
> 함께 수동 heartbeat가 확인됐다. 최종 판정은 **NO-GO**이며 13·14 설치 뒤 재검증해야 한다.

### 2026-07-11 실행 요약

- CAN: 500 kbps, AK 1~4 약 50 Hz, ODrive heartbeat 11·12·15·16 확인, 13·14 미검출.
- ODrive: present 4축 캘리브레이션 4/4 성공; 건강판정은 present 4축 정상, absent 2축 stale.
- fail-safe: absent 13·14로 전체 섀시가 latched `ESTOP`; CAN error/passive/bus-off delta 0.
- 실제 US-100: 원거리 2433~2469 mm `VALID`; 근거리 75~96 mm `too_close`→`ESTOP`;
  전원 분리 `NO_RESPONSE/liveness_timeout`→`ESTOP`; 재연결 뒤 센서 정상이어도 latch 유지.
- reset은 `IDLE`까지만 복구하고 별도 arm이 필요함을 fake chassis와 실제 센서 결합으로 확인.
- `INVALID_READING`은 실내에서 무반사 조건을 만들 수 없어 미검증.
- 보정된 실제 chassis 60초 측정: 3000 samples, 평균 50.000 Hz, 완전한 5초 구간
  49.8~50.2 Hz, overrun 0, CAN error/passive/bus-off 0.
- HIL 중 발견·수정: DOWN CAN watchdog reset 폭주, launch 정수→double 형변환,
  AK E-stop 2초 블로킹, ODrive buffered heartbeat false-stale. 수정 후 fault tick 최대 0.854 ms.
- 안전 종료: 제어 프로세스 전부 종료, watchdog 정지, `can0` DOWN/STOPPED.

## 목차

1. 환경
2. 핵심 개념·파라미터
3. 배선·물리 안전
4. 설치·사전 준비
5. 실행 시나리오
6. 트러블슈팅
7. 검증 결과·go/no-go
8. 코드·참고

## 1. 환경

### 1.1 검증 실행 식별자

| 필드 | 기록 |
|---|---|
| 실행 상태 | `NOT RUN` |
| 사용자 준비 확인 시각 | `— (NOT RUN)` |
| HIL 실행 시작/종료 KST | `— (NOT RUN)` / `— (NOT RUN)` |
| 실행자 / 관찰자 | `— (NOT RUN)` / `— (NOT RUN)` |
| 로컬 브랜치 | `— (NOT RUN)` |
| 로컬 검증 commit(40자) | `— (NOT RUN)` |
| origin 동일 commit 확인 | `— (NOT RUN)` |
| Jetson checkout / commit | `— (NOT RUN)` / `— (NOT RUN)` |
| Jetson 이미지 / 컨테이너 | `— (NOT RUN)` / `— (NOT RUN)` |
| ROS distro / domain | `— (NOT RUN)` / `— (NOT RUN)` |
| 로그·rosbag·사진 디렉터리 | `— (NOT RUN; 생성 후 경로 기록)` |

### 1.2 Phase A 사용자 물리 준비 — 시나리오 1~8, 바퀴 부양

- [ ] 바퀴 6개가 지면에서 완전히 부양됐다.
- [ ] 48 V 물리 E-stop에 즉시 손이 닿고 차단 동작을 확인했다.
- [ ] AK id 1~4와 ODrive node 11~16의 전원·배선이 완료됐다.
- [ ] US-100이 `/dev/ttyTHS1`에 연결됐다.
- [ ] 테스트 중 주변 인원에게 모터 구동을 고지하고 회전체 접근을 통제했다.

| 확인 필드 | 기록 |
|---|---|
| 사용자 명시 확인 원문/요약 | `— (NOT RUN)` |
| 확인 시각 KST | `— (NOT RUN)` |
| 48 V 차단 확인 방법 | `— (NOT RUN)` |
| 부양 상태 사진/증거 | `— (NOT RUN)` |

### 1.3 Phase B 별도 사용자 허가 — 시나리오 9, 50 kg 지상주행

시나리오 9는 실제 50 kg 로봇의 제동거리 측정이므로 Phase A의 바퀴 부양 확인을 승계할 수
없다. 하나의 최종 HIL batch에서 1~9를 연속 실행할 수는 있지만, 바퀴를 내리기 직전에
아래 조건을 새로 확인하고 사용자의 명시적 지상주행 허가를 받아야 한다.

- [ ] 직선 통제 주행로와 충분한 정지 여유를 확보했다.
- [ ] 시험 속도를 최저 단계부터 올리는 staged low-speed 계획을 승인했다.
- [ ] 전담 spotter가 물리 E-stop을 잡고 있다.
- [ ] 주행로와 예상 제동거리 전체를 exclusion zone으로 통제했다.
- [ ] 로봇 총질량 50 kg 구성과 적재 상태를 기록했다.
- [ ] 바퀴를 지면에 내리기 직전 사용자가 별도 실행 허가를 명시했다.

| 지상주행 허가 필드 | 기록 |
|---|---|
| 사용자 명시 확인 원문/요약 | `— (NOT RUN)` |
| 바퀴를 내린 시각 KST | `— (NOT RUN)` |
| 통제 주행로 길이·폭 | `— m / — m (NOT RUN)` |
| spotter / 물리 E-stop 확인 | `— / — (NOT RUN)` |
| exclusion zone 확인 | `— (NOT RUN)` |
| 50 kg 구성 증거 | `— (NOT RUN)` |

### 1.4 목표 구성

- Jetson Orin Nano, ROS2 Humble, host network
- 단일 `can0`, 500 kbps, 50 Hz
- AK45-36 ×4(id 1~4) + ODrive/BL70200 ×6(node 11~16)
- US-100 `/dev/ttyTHS1`, 9600 baud, 5~10 Hz
- `us100_safety_node` → `/safety_verdict` → `chassis_node` → 10모터
- `chassis_node` → `/wheel_states`, 명목 50 Hz

## 2. 핵심 개념·파라미터

### 2.1 권한과 프로세스 경계

제어·안전 정책은 ROS 없는 순수 Python `SafetyInterlock`과 `ChassisManager`에 둔다.
`ChassisManager`가 CAN과 10모터, 최종 E-stop의 유일한 소유자다. ROS2는 내부 전송층이다.
블로킹 가능한 US-100 UART는 5~10 Hz 별도 프로세스에서만 읽고, 50 Hz `chassis_node`는
최신 `/safety_verdict`의 상태와 freshness를 매 tick 확인한다. 이 분리는 UART 지연과 무관한
결정적 차체 tick, 단일 최종 E-stop 권한, ROS 없는 텔레옵과 같은 안전 의미 재사용을 위한 것이다.

### 2.2 정지·복구 계약

| 상태 | 원인 예 | 복구 |
|---|---|---|
| `RUN` | 안전 조건 정상 | 해당 없음 |
| `MOTION_HOLD` | `CHECKING`, `/cmd_vel` 0.5초 timeout, 연결 단절 | 원인 해소 후 자동; cmd timeout은 새 명령 필요 |
| `ESTOP` | 유효 근거리, `NO_RESPONSE`, safety topic startup/stale, 모터 fault/stale, 수동 정지 | 위험 해소 → reset → `IDLE` → 별도 arm |

reset과 arm은 한 동작이 아니다. reset 성공만으로 모터가 회전하면 실패다.

### 2.3 US-100 상태 계약

| 상태 | 판정 | 정책 |
|---|---|---|
| `CHECKING` | 기동 또는 거리·생존 응답 1~2회 누락 | `MOTION_HOLD` |
| `VALID` | 20~4000 mm 유효 거리 | `< stop_mm`이면 latched `ESTOP`, 그 외 `RUN` |
| `INVALID_READING` | 거리 무효, 0x50 응답 있음 | 정상 `RUN` |
| `NO_RESPONSE` | 거리와 0x50 응답 모두 연속 3회 없음 | latched `ESTOP` |

0x50은 MCU/UART 생존만 증명한다. 초음파 송신기·수신기 정상은 증명하지 않는다.
`INVALID_READING`을 정상 통과시키는 정책은 명시적 잔여 위험이다.

### 2.4 생산 기본과 HIL 확정 대상

| 항목 | 현재 계약 | HIL 기록 |
|---|---:|---|
| 차체 loop | 50 Hz | `— (NOT RUN)` |
| `/cmd_vel` watchdog | 0.5 s, `MOTION_HOLD`; safety-topic freshness와 별개 | `— (NOT RUN)` |
| US-100 sample | 5~10 Hz | `— (NOT RUN)` |
| `safety_topic_timeout` | 0.75 s 최솟값·기본값 | `— (NOT RUN)` |
| freshness 집행 | `age > 0.75 s` 뒤 다음 50 Hz tick, 명목 0.75~0.77 s | `— (NOT RUN)` |
| startup timeout | 1.0 s | `— (NOT RUN)` |
| `fail_stop_count` | 3 | `— (NOT RUN)` |
| `safety_required` | 생산 `true`; `false`는 BENCH/FAKE만 | `— (NOT RUN)` |
| `stop_mm` | 기본값 없음; 결합 launch에 항상 명시 | `— (Phase A 임시값 / Phase B 승인값 모두 기록)` |

생산 승인 전 결합 launch는 Phase A HIL 후보이며 명시적 임시 `stop_mm`과 통제 저속만
허용한다. Phase B 제동 실측으로 승인·재검증한 값만 생산 launch에 사용할 수 있다.

## 3. 배선·물리 안전

### 3.1 사전 체크리스트

- [ ] 전원 OFF에서 CAN 양 끝 2종단의 합성 저항을 측정하고 기록했다.
- [ ] ADM3053 외부 5 V, TX/RX, 절연측 배선을 확인했다.
- [ ] CANH/CANL 트위스트·락킹/크림프 상태를 육안 확인했다.
- [ ] AK id 1~4와 ODrive node 11~16 중복이 없다.
- [ ] 48 V·브레이크 저항·물리 E-stop 경로를 확인했다.
- [ ] US-100 UART TX/RX/GND와 `/dev/ttyTHS1` 소유자를 확인했다.
- [ ] 바퀴·조향 링크의 회전 반경에 사람·공구·케이블이 없다.

| 측정/확인 | 기록 | 증거 |
|---|---|---|
| CAN 종단 저항 | `— Ω (NOT RUN)` | `—` |
| ADM3053 절연측 공급 | `— V (NOT RUN)` | `—` |
| 48 V bus | `— V (NOT RUN)` | `—` |
| US-100 device 권한 | `— (NOT RUN)` | `—` |
| 모터 id/node 목록 | `— (NOT RUN)` | `—` |

## 4. 설치·사전 준비

### 4.1 로컬 자동시험 — 관찰 결과

이 표의 PASS는 pre-HIL 작업에서 관찰된 로컬 결과다. 실기 HIL 또는 ROS 런타임 결과로
확대 해석하지 않는다.

| suite | 결과 | 하드웨어 | 원본 로그/commit |
|---|---|---|---|
| 지원 `motor_control` suite | **189 passed** | 없음 | commit `c3610c136357a8c881263926ec18bcd7e3432a5d`; `.superpowers/sdd/final-motor-control-c3610c1.xml` |
| `motor_gui/tests` | **91 passed** | 없음 | commit `c3610c136357a8c881263926ec18bcd7e3432a5d`; `.superpowers/sdd/final-motor-gui-c3610c1.xml` |

### 4.2 로컬 임시 read-only ROS build/test — 관찰 결과

소스 worktree에 ROS build 산출물을 쓰지 않는 임시 복사본에서 다음 결과를 관찰했다. 이는
메시지·패키지 build/test 증거일 뿐 Jetson 배포, FAKE acceptance, DDS 실기, UART 또는 CAN
증거가 아니다.

| 필드 | 기록 |
|---|---|
| build 범위 | `robot_arm_msgs`, `powertrain_msgs`, `powertrain_ros` — **3 packages clean build passed** |
| test 범위 | `powertrain_ros` — **31/31 passed** |
| source/worktree 변경 | 없음; 임시 read-only 검증 |
| 하드웨어·Jetson·FAKE | 사용 안 함 / `NOT RUN` |
| provenance | commit `c3610c136357a8c881263926ec18bcd7e3432a5d`; `.superpowers/sdd/final-ros-c3610c1.xml` |

### 4.3 Jetson software-only FAKE acceptance — PASS, HIL 아님

- [x] Jetson 배포 checkout이 `49831bb42058a177ed9c41d72d0273f4f0a8f535`임을 확인했다.
- [x] 후속 배포 HEAD `c3610c136357a8c881263926ec18bcd7e3432a5d`에서 Jetson 3패키지 build와
  `powertrain_ros` 31/31이 통과했다. 이는 아래 49831bb FAKE 동작 측정과 구분한다.
- [x] 경쟁 chassis/teleop 없이 software-only FAKE chassis를 기동했다.
- [x] startup 무판정 `ESTOP`, far `ARMED/RUN`, near `ESTOP`을 확인했다.
- [x] far 복귀만으로 latch가 풀리지 않음을 확인했다.
- [x] reset은 `IDLE`이며 implicit arm이 없고, 별도 arm만 재가동함을 확인했다.
- [x] `/wheel_states` 60초 3000 samples와 tick/overrun을 측정했다.
- [x] publisher 종료 후 strict freshness 경로가 0.753초에 `ESTOP`함을 확인했다.

| 필드 | 기록 |
|---|---|
| FAKE 실행 commit | `49831bb42058a177ed9c41d72d0273f4f0a8f535` |
| 60초 `/wheel_states` | count 3000; mean 50.000 Hz; minimum 5 s window 50.000 Hz |
| tick p99 / overrun / max interval | 0.280 ms / 0 / 21.453 ms |
| 상태 전이 | startup `ESTOP`; far `ARMED/RUN`; near `ESTOP`; far latch; reset `IDLE`; no implicit arm; separate arm |
| safety publisher kill→ESTOP | 0.753 s |
| 최신 Jetson ROS raw XML (c3610c1 자동검증) | `/home/zetin/power-train-sw/ros2/build/powertrain_ros/pytest.xml` |
| FAKE raw log | 미보존; 위 값은 tool capture summary이며 최종 재실행 raw log `PENDING` |

이 PASS는 software-only FAKE acceptance다. CAN, UART, US-100, 모터, 바퀴, 지상주행 또는
제동 성능을 입증하지 않으며 최종 HIL 판정에 대체 사용할 수 없다.

### 4.4 Jetson pre-HIL 상태 — PENDING

다음 명령은 해당 Phase의 사용자 물리 준비 확인 뒤 Jetson 호스트에서 순서대로 실행한다.
이 문서 갱신 중에는 실행하지 않았다.

```bash
set -eu
cd ~/power-train-sw
git status --short
git rev-parse HEAD
docker compose -f docker/docker-compose.jetson.yml up -d canwatchdog powertrain_ros
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

이 검사는 host `ps`와 실행 중인 모든 컨테이너의 `docker top`을 각각 검사한다. 컨테이너
내부 `ps`만으로 host-wide 검사를 주장하지 않는다. `powertrain_canwatchdog`는 반드시
running이어야 한다. 워치독은 TX wedge probe용 AF_CAN raw socket을 열고 `can0`에 bind하되
빈 RX filter를 설정한 TX-only 소켓이므로 receiver 목록에는 나타나지 않는다. 임의의 다른
TX-only socket도 receiver 목록으로 찾을 수 없다. 프로세스/container allowlist는 운영상
완화책일 뿐 완전한 증명이 아니며, 이름을 바꾼 TX-only 소유자는 잔여 위험이다. 출력이 하나라도
있으면 launch하지 말고 소유자와 작업 목적을 확인한다. 알 수 없는 팀원 프로세스를 자동 종료하지 않는다.

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

그 다음 sticky loopback을 명시적으로 끄고 저장소의 권위 스크립트를 실행한다.

```bash
sudo ip link set can0 down
sudo ip link set can0 type can loopback off
./scripts/can_setup.sh
ip -details -statistics link show can0
```

마지막 출력에서 `bitrate 500000`, `restart-ms 100`, loopback 비활성, tx/rx packet/error,
error-passive, bus-off, restart 기준값을 기록한다. 하나라도 확인하지 못하면 launch하지 않는다.

- [ ] Jetson 추적 파일에 겹치는 로컬 변경이 없다.
- [ ] 기존 미추적 `motor_control/vision/tests/`를 보존했다.
- [ ] 검증 branch와 40자 commit이 로컬·origin·Jetson에서 일치한다.
- [ ] host `ps`와 모든 running container `docker top`에 경쟁 teleop/chassis/motor 제어가 없다.
- [ ] `powertrain_canwatchdog`가 running이다.
- [ ] launch 전 `/proc/net/can/rcvlist_*`에 CAN receiver가 없다.
- [ ] can0 loopback이 명시적으로 OFF다.
- [ ] `scripts/can_setup.sh`로 can0 500 kbps를 올렸다.
- [ ] node 1~4·11~16 heartbeat/상태 존재를 기록했다.
- [ ] `/wheel_states` baseline을 기록했다.

| pre-HIL 필드 | 값 |
|---|---|
| can0 state/bitrate/restart-ms/loopback | `— (NOT RUN)` |
| tx/rx packets·errors | `— / — / — (NOT RUN)` |
| error-passive/bus-off/restarts | `— / — / — (NOT RUN)` |
| host/container 경쟁 프로세스 감사 | `— (NOT RUN)` |
| launch 전 CAN receiver 목록 | `— (NOT RUN; 기대=없음)` |
| AK id 1~4 상태 | `— (NOT RUN)` |
| ODrive node 11~16 상태 | `— (NOT RUN)` |
| `/wheel_states` baseline | `— (NOT RUN)` |
| 증거 경로 | `— (NOT RUN)` |

### 4.5 결합 launch 명령 게이트 — PENDING

결합 launch에는 기본 `stop_mm`이 없다. Phase A는 생산 승인이 아니라 HIL 후보이므로 실제
통제 임시값을 명시한다. `<provisional-mm>` 문자열 자체를 실행하면 안 된다.

```bash
docker exec -it powertrain_ros bash
cd /workspace/ros2
source install/setup.bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=<provisional-mm>
```

arm이나 `/cmd_vel` 전에 별도 SSH 터미널에서 다음 post-launch gate를 실행한다. 허용되는 제어
프로세스는 `powertrain_ros`의 launch supervisor 1개와 chassis 실행기 1개뿐이다.

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

chassis 한 프로세스가 여러 SocketCAN socket을 열므로 receiver 행 수를 소유자 수로 해석하지
않는다. host `ps` snapshot의 chassis PID 집합이 `docker top powertrain_ros`에서 얻은 단 하나의
chassis PID와 정확히 같아야 하므로 native·다른 container 중복도 실패한다. receiver 목록은
launch 전 예기치 않은 RX socket 부재와 launch 후 의도한 RX socket 출현을 보여줄 뿐 TX-only
socket 소유권을 증명하지 않는다. 실패하면 arm하지 않고 launch 터미널에서 `Ctrl-C`로
의도한 launch를 종료한다. 필요하면 물리 E-stop을 누르고 조사하며, 알 수 없는 프로세스를
자동 종료하지 않는다.

Phase B에서 산정·승인·재검증한 뒤 생산 실행은 다음 형식만 허용한다.

```bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=<HIL-approved-mm>
```

`ros2 run powertrain_ros us100_safety`와 `ros2 run powertrain_ros chassis` 직접 실행은 분리
진단 전용이다. 결합 안전 경로를 대신하는 생산 명령으로 사용하지 않는다.

## 5. 실행 시나리오

하나의 최종 HIL batch에 두 Phase를 포함할 수 있지만 승인 경계는 분리한다. 시나리오 1~8은
Phase A 부양 HIL이고, 시나리오 9는 별도 사용자 확인 뒤 바퀴를 내리는 Phase B 지상주행
HIL이다. Phase B는 Phase A의 부양 확인을 승계하지 않는다. 공통 CAN 필드는 시나리오 전후
`state`, tx/rx packet/error, error-passive, bus-off, restart의 절대값과 delta를 모두 기록한다.
공통 안전 필드는 US-100 status, chassis stop state/mode, latch, active/first E-stop source를
기록한다. 동영상만으로 수치를 추정하지 않는다.

### Phase A — 바퀴 6개 부양, 시나리오 1~8

1.2의 사용자 확인이 유효한 동안만 실행한다. 모터를 회전시키더라도 로봇 본체는 지상에서
이동하지 않아야 한다.

### 5.1 시나리오 1 — CAN 60초 50 Hz 기준선

**기대:** 10모터 CAN 연결 상태에서 `/wheel_states` 평균 49~51 Hz, 지속 48 Hz 미만 없음,
tick p99 <20 ms, CAN bus-off·error-passive 증가 0.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 시각 KST | [ ] `NOT RUN` / `—` |
| 관찰 결과 | `— (NOT RUN)` |
| `/wheel_states` 평균·최저·지속 <48 Hz | `— Hz` / `— Hz` / `— s` |
| tick p99·max·overrun delta | `— ms` / `— ms` / `—` |
| CAN pre→post / delta | `— (NOT RUN)` |
| safety status / chassis mode | `—` / `—` |
| E-stop source / latch | `—` / `—` |
| 모터 육안 관찰 | `— (NOT RUN)` |
| 로그·rosbag·영상 증거 | `— (NOT RUN)` |

### 5.2 시나리오 2 — 빈 공간·무반사 `INVALID_READING`

**기대:** 거리값은 무효지만 0x50 응답이 있으면 `INVALID_READING`, `estop_required=false`,
`RUN`. 0x50은 MCU/UART 생존만 입증한다.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 시각 KST | [ ] `NOT RUN` / `—` |
| 관찰 결과 / raw bytes | `—` / `—` |
| sensor interval / processing delay | `— ms` / `— ms` |
| wheel rate / tick p99 | `— Hz` / `— ms` |
| CAN pre→post / delta | `— (NOT RUN)` |
| US-100 status / chassis stop state | `—` / `—` |
| E-stop source / latch | `—` / `—` |
| 모터 육안 관찰 | `— (NOT RUN)` |
| 로그·rosbag·영상 증거 | `— (NOT RUN)` |

### 5.3 시나리오 3 — 먼 표적 `VALID`

**기대:** 거리 `≥ stop_mm`인 유효 표적에서 `VALID`, `estop_required=false`, `RUN`.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 시각 KST | [ ] `NOT RUN` / `—` |
| 표적 거리 / sensor raw | `— mm` / `—` |
| 관찰 결과 / processing delay | `—` / `— ms` |
| wheel rate / tick p99 | `— Hz` / `— ms` |
| CAN pre→post / delta | `— (NOT RUN)` |
| US-100 status / chassis stop state | `—` / `—` |
| E-stop source / latch | `—` / `—` |
| 모터 육안 관찰 | `— (NOT RUN)` |
| 로그·rosbag·영상 증거 | `— (NOT RUN)` |

### 5.4 시나리오 4 — 가까운 유효 표적

**기대:** 유효 거리 `< stop_mm`에서 즉시 latched `ESTOP`, 전체 10모터 정지.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 시각 KST | [ ] `NOT RUN` / `—` |
| 표적 거리 / 명령 속도 | `— mm` / `— m/s` |
| 감지→ESTOP / 감지→육안 정지 | `— ms` / `— ms` |
| wheel rate / tick p99 | `— Hz` / `— ms` |
| CAN pre→post / delta | `— (NOT RUN)` |
| US-100 status / chassis stop state | `—` / `—` |
| E-stop source / latch | `—` / `—` |
| 10모터 육안 관찰 | `— (NOT RUN)` |
| 로그·rosbag·영상 증거 | `— (NOT RUN)` |

### 5.5 시나리오 5 — 표적 제거 뒤 latch 유지

**기대:** 가까운 표적을 제거해 먼 `VALID`로 돌아와도 reset 전 `ESTOP` 유지, 모터 정지.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 시각 KST | [ ] `NOT RUN` / `—` |
| 관찰 결과 / 유지 시간 | `—` / `— s` |
| wheel rate / tick p99 | `— Hz` / `— ms` |
| CAN pre→post / delta | `— (NOT RUN)` |
| US-100 status / chassis stop state | `—` / `—` |
| first/active E-stop source / latch | `—` / `—` |
| 모터 육안 관찰 | `— (NOT RUN)` |
| 로그·rosbag·영상 증거 | `— (NOT RUN)` |

### 5.6 시나리오 6 — reset과 arm 분리

**기대:** 위험 해소 뒤 reset은 `IDLE`·바퀴 정지. reset만으로 회전하지 않고, 별도 arm 뒤에만
`ARMED` 및 명령 회전.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 시각 KST | [ ] `NOT RUN` / `—` |
| reset 응답 / reset 뒤 mode | `—` / `—` |
| reset→arm 사이 관찰 시간 / 속도 | `— s` / `— turns/s` |
| arm 응답 / arm 뒤 mode | `—` / `—` |
| wheel rate / tick p99 | `— Hz` / `— ms` |
| CAN pre→post / delta | `— (NOT RUN)` |
| E-stop source / latch | `—` / `—` |
| 모터 육안 관찰 | `— (NOT RUN)` |
| 로그·rosbag·영상 증거 | `— (NOT RUN)` |

### 5.7 시나리오 7 — US-100 분리·연속 3회 실패

**기대:** 1~2회 실패는 `CHECKING`·`MOTION_HOLD`; 거리와 0x50이 모두 3회 연속 실패하면
`NO_RESPONSE`·latched `ESTOP`, 전체 10모터 정지.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 분리 시각 KST | [ ] `NOT RUN` / `—` |
| 실패 1/2/3 상태 | `—` / `—` / `—` |
| sample intervals / 3회 도달시간 | `— ms` / `— ms` |
| 3회 도달→ESTOP | `— ms` |
| wheel rate / tick p99 | `— Hz` / `— ms` |
| CAN pre→post / delta | `— (NOT RUN)` |
| chassis stop state / E-stop source / latch | `—` / `—` / `—` |
| 10모터 육안 관찰 | `— (NOT RUN)` |
| 로그·rosbag·영상 증거 | `— (NOT RUN)` |

### 5.8 시나리오 8 — 단일 모터 fault/stale

**기대:** 한 축 fault 또는 stale 주입 시 latched `ESTOP`이 전체 10모터로 전파된다. 한
코너 정지 예외가 있어도 나머지 정지 시도를 계속한다.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 주입 시각 KST | [ ] `NOT RUN` / `—` |
| 주입 node·축 / 방법 | `—` / `—` |
| 주입→ESTOP / 전체 정지 | `— ms` / `— ms` |
| wheel rate / tick p99 | `— Hz` / `— ms` |
| CAN pre→post / delta | `— (NOT RUN)` |
| chassis stop state / E-stop source / latch | `—` / `—` / `—` |
| 10모터 육안 관찰 | `— (NOT RUN)` |
| 복구·reset·arm 관찰 | `— (NOT RUN)` |
| 로그·rosbag·영상 증거 | `— (NOT RUN)` |

### Phase B — 별도 승인 50 kg 지상주행

1.3의 통제 주행로·단계적 저속·spotter·exclusion zone·물리 E-stop·별도 사용자 확인을 모두
새로 기록한 뒤에만 바퀴를 내린다. 이 확인 없이는 시나리오 9를 시작하지 않는다.

### 5.9 시나리오 9 — 50 kg 실차 속도별 감지·제동과 `stop_mm`

**기대:** 통제 주행로에서 최저 속도부터 단계적으로 각 속도의 최악 센서주기, 처리지연,
실제 50 kg 로봇 제동거리를 측정하고 안전여유를 더해 생산 `stop_mm`을 결정한다. 측정 전
임의 값을 승인하지 않는다.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 시각 KST | [ ] `NOT RUN` / `—` |
| Phase B 사용자 허가 / 바퀴 내림 시각 | `— (NOT RUN)` / `— (NOT RUN)` |
| 통제 주행로 / spotter / exclusion zone | `—` / `—` / `—` |
| 물리 E-stop 확인 / 로봇 질량 | `—` / `— kg` |
| 시험 속도 목록 | `— m/s (NOT RUN)` |
| 최악 sensor interval / processing delay | `— s` / `— s` |
| 실측 최대 제동거리 / 안전여유 | `— mm` / `— mm` |
| wheel rate / tick p99 | `— Hz` / `— ms` |
| CAN pre→post / delta | `— (NOT RUN)` |
| safety status / E-stop source / latch | `—` / `—` / `—` |
| 모터·차체 육안 관찰 | `— (NOT RUN)` |
| 로그·영상·거리 측정 증거 | `— (NOT RUN)` |

생산 임계 계산식은 다음과 같다.

```text
stop_mm ≥ 최고속도_mm_s × (최악_센서주기_s + 최악_처리지연_s)
          + 실측_제동거리_mm + 안전여유_mm
```

| speed_m_s | sensor interval worst_ms | processing worst_ms | brake distance_mm | margin_mm | required stop_mm | 반복/증거 |
|---:|---:|---:|---:|---:|---:|---|
| `—` | `—` | `—` | `—` | `—` | `—` | `NOT RUN` |
| `—` | `—` | `—` | `—` | `—` | `—` | `NOT RUN` |
| `—` | `—` | `—` | `—` | `—` | `—` | `NOT RUN` |

| 최종 산정 필드 | 기록 |
|---|---|
| 시험 최고속도 | `— m/s (NOT RUN)` |
| 식으로 계산한 최솟값 | `— mm (NOT RUN)` |
| 채택 생산 `stop_mm` | `— mm (PENDING)` |
| 선택 근거·반올림·여유 | `— (PENDING)` |
| 재검증 결과 | `— (PENDING)` |

## 6. 트러블슈팅

| 증상 | 확인 | 조치·기록 원칙 |
|---|---|---|
| 모든 bitrate에서 송신 ACK처럼 보이나 버스가 조용함 | can0 loopback sticky 여부 | loopback을 명시적으로 OFF하고 재확인. self-ACK를 HIL 증거로 쓰지 않음 |
| 새 시험과 속도 0 명령이 충돌 | zombie teleop/chassis 프로세스 | 프로세스 소유자 확인 뒤 하나의 제어자만 남김 |
| can0 ERROR-PASSIVE/bus-off 증가 | 종단·공통배선·node baud·전원 | 즉시 정지, 전후 counter 보존. 원인 해소 전 다음 시나리오 금지 |
| US-100 `INVALID_READING` 지속 | 0x55 거리와 0x50 생존 raw 분리 | 0x50을 초음파 정상 증거로 해석하지 않음 |
| safety publisher 종료 뒤 즉시 E-stop이 아님 | `age >0.75 s`와 50 Hz tick 위상 | 마지막 stamp와 E-stop tick을 함께 기록; 명목 0.75~0.77초 |
| reset 뒤 arm이 안 됨 | active E-stop source·모터 fault | active source를 제거하고 reset 재시도. reset과 arm을 한 호출로 합치지 않음 |
| `/wheel_states` rate 저하 | tick p99·overrun·DDS·CAN backlog | 수치와 로그 보존, 48 Hz 미만 지속 시 HIL 실패 |
| HALL 저속에서 바퀴가 안 돎 | 실제 육안과 turns/s 비교 | 텔레메트리만으로 통과시키지 않음; 기존 코깅 제약을 기록 |

## 7. 검증 결과·go/no-go

### 7.1 결과 요약

| 검증 게이트 | 상태 | 증거 |
|---|---|---|
| 지원 로컬 `motor_control` suite | `PASS (198 passed)` | commit ec452f6; 2026-07-11 재실행 |
| `motor_gui/tests` | `PASS (91 passed)` | commit c3610c1; `.superpowers/sdd/final-motor-gui-c3610c1.xml` |
| 격리 read-only ROS 3패키지 clean build | `PASS` | commit `c3610c136357a8c881263926ec18bcd7e3432a5d`; `robot_arm_msgs`, `powertrain_msgs`, `powertrain_ros` |
| 로컬 `powertrain_ros` tests | `PASS (32/32)` | typed launch parameter fix 포함 |
| Jetson 동일 HEAD 3패키지 build·`powertrain_ros` tests | `PASS (32/32)` | commit ec452f6 배포·build |
| Jetson software-only FAKE 60초·freshness | `PASS (HIL 아님)` | commit `49831bb42058a177ed9c41d72d0273f4f0a8f535`; raw log 미보존 |
| Phase A 사용자 부양 확인 | `PASS WITH CONSTRAINT` | 바퀴 부양·물리 E-stop 준비; ODrive 13·14 부재 사전 고지 |
| 실제 CAN·present motor health | `PARTIAL PASS` | AK 1~4, ODrive 11·12·15·16; present 캘리 4/4, errors 0 |
| US-100 near/no-response/latch/reset | `PASS` | 75~96 mm near; power-loss NO_RESPONSE; reconnect latch; reset→IDLE |
| US-100 `INVALID_READING` | `NOT RUN` | 실내에서 신뢰 가능한 무반사 조건 확보 불가 |
| 실제 60초 50 Hz | `PASS` | 3000/60 s; 49.8~50.2 Hz/5 s; overrun 0; CAN errors 0 |
| 완전한 10모터 Phase A | `FAIL / BLOCKED` | ODrive node 13·14 물리 부재 |
| Phase B 지상주행·생산 `stop_mm` | `NOT RUN` | 완전체 Phase A 실패로 안전상 금지; 생산값 미확정 |

최신 자동검증은 실행 HEAD `ec452f6474b6fc57437d576298f2bc954649be42`에서 관찰했다.
Jetson software-only FAKE summary는 별도 49831bb 관찰이며 raw log가 없으므로 현재 HEAD의
실기 HIL 증거로 확대하지 않는다.

### 7.2 잔여 위험

| 위험 | 현재 통제 | HIL 뒤 판단 |
|---|---|---|
| 0x50은 MCU/UART만 확인해 초음파 송수신부 고장을 놓칠 수 있음 | `INVALID_READING`을 진단에 노출, 저속·운영 감시 | 실내 조건으로 실측 미검증; 잔여 위험 유지 |
| 단일 can0 공통고장과 물리층 간헐 접촉 | ADM3053·워치독·counter delta·물리 E-stop | `— (PENDING)` |
| safety topic stale까지 명목 0.75~0.77초 | 생산 최솟값 고정, `stop_mm` 식에 지연 반영 | `— (PENDING)` |
| node 12/16 HALL 품질과 저속 코깅 | fault/stale 전체 E-stop, 육안 확인 | `— (PENDING)` |
| `/wheel_states` 발행 실패가 제어와 분리됨 | 로그·rate·overrun 관찰 | WP6 입력 안전정책에서 결정 |
| 복수 `/cmd_vel` 작성자 충돌 | 현재 미해결을 명시 | HIL 뒤 command-authority spec |
| L515 PointCloud2 기본 사용 시 불필요한 부하 | color/depth image+IMU 기본, PointCloud2 opt-in | 별도 L515 경량 spec |
| reset 후 무심코 재가동 | reset→`IDLE`, 별도 arm | 실제 US-100+fake chassis 결합에서 확인 |
| ODrive 13·14 부재 | 전체 motor stale을 latched E-stop으로 처리 | 설치·설정·캘리 후 10모터 재검증 필수 |

### 7.3 최종 판정

| 항목 | 기록 |
|---|---|
| 최종 go/no-go | **NO-GO — PARTIAL HIL** |
| 판정자 / 시각 | Codex+사용자 / 2026-07-11 KST |
| 미통과 항목 | ODrive 13·14, 완전한 10모터 Phase A, `INVALID_READING`, Phase B, 생산 `stop_mm` |
| 조건부 제한 | 자율 실차·지상주행 금지; 센서/소프트웨어 벤치와 command-authority 개발만 가능 |
| 실행 commit / 설정 | `ec452f6474b6fc57437d576298f2bc954649be42`; can0 500 kbps; HIL 후보 stop_mm 200 mm |

ODrive 13·14 설치 뒤 10모터 HIL과 Phase B를 재실행한다. 하드웨어 대기 중에는
command-authority spec을 병행할 수 있으며, 이후 L515 경량 color/depth image+IMU pipeline
(PointCloud2 optional) → WP6 순서로 진행한다. WP8, `MISSION_STOP`, unlock ordering,
`ARRIVED_* → 팔 작업 → DONE → 재출발` 합동 1사이클은 별도 미결이다.

## 8. 코드·참고

### 8.1 원시 증거 보존 계획

현재 원시 결과는 로컬 189/91 JUnit
`.superpowers/sdd/final-motor-control-c3610c1.xml`·
`.superpowers/sdd/final-motor-gui-c3610c1.xml`, 격리 ROS 31/31 JUnit
`.superpowers/sdd/final-ros-c3610c1.xml`, 그리고 Jetson pytest XML
`/home/zetin/power-train-sw/ros2/build/powertrain_ros/pytest.xml`이다. FAKE summary raw log는
보존되지 않았다. 최종 재실행 때 아래 경로를 생성해 commit·명령·stdout, JUnit/XML, CAN
counter, rosbag과 시나리오 기록을 영속 보존한다. 아래는 **계획 경로**이며 아직 존재한다고
주장하지 않는다.

- `/home/zetin/power-train-sw/artifacts/wp5-control-safety/<HIL-commit>/local-motor-control.log`
- `/home/zetin/power-train-sw/artifacts/wp5-control-safety/<HIL-commit>/local-motor-gui.log`
- `/home/zetin/power-train-sw/artifacts/wp5-control-safety/<HIL-commit>/powertrain_ros-pytest.xml`
- `/home/zetin/power-train-sw/artifacts/wp5-control-safety/<HIL-commit>/colcon-test-result.txt`
- `/home/zetin/power-train-sw/artifacts/wp5-control-safety/<HIL-commit>/fake-acceptance.log`
- `/home/zetin/power-train-sw/artifacts/wp5-control-safety/<HIL-commit>/hil-scenarios-1-8.log`
- `/home/zetin/power-train-sw/artifacts/wp5-control-safety/<HIL-commit>/hil-scenario-9-ground.log`

### 8.2 권위 코드·문서

- 설계: `docs/specs/2026-07-10-wp5-control-safety-hardening-design.md`
- 구현 계획: `docs/plans/2026-07-10-wp5-control-safety-hardening-plan.md`
- 상위 계획: `docs/plans/2026-07-02-autonomous-driving-kickoff.md`
- 현재 상태: `docs/reports/2026-07-10-project-and-jetson-state.md`
- ROS 실행·계약: `ros2/README.md`
- 순수 안전 코어: `motor_control/chassis/safety_interlock.py`
- 차체 단일 권한: `motor_control/chassis/chassis_manager.py`
- US-100 판정: `motor_control/safety_us100/`
- ROS 노드: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py`,
  `ros2/src/powertrain_ros/powertrain_ros/us100_safety_node.py`
- ROS 메시지: `ros2/src/powertrain_msgs/msg/`
