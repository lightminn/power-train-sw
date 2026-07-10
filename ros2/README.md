# ros2/ — 파워트레인 ROS2 워크스페이스

로봇팔 팀(`ksp118/extreme-robot`)과 **분리 개발**하는 우리 ROS2 층이다. 각 팀은 자기
노드·컨테이너를 소유하고, `robot_arm_msgs` 계약만 공유하며 DDS(host network, domain 0)로
통신한다.

> **WP5.1 상태 (2026-07-10): Tasks 1~8 소프트웨어 완료, 최종 실기 HIL 미실행.** 2026-07-07의
> WP4 양방향 DDS와 기존 WP5 `/cmd_vel → 10모터` HIL 이력은 보존한다. 아래 새
> `/safety_verdict`·`/wheel_states`·latched E-stop 경로는
> [`WP5.1 HIL 보고서`](../docs/reports/2026-07-10-wp5-control-safety-hil.md)의
> `NOT RUN` 항목을 통과하기 전까지 실기 완료로 주장하지 않는다. 로컬 관찰 증거는
> `motor_control` 189 passed, `motor_gui` 91 passed, 임시 read-only ROS 워크스페이스의
> 3패키지 build와 `powertrain_ros` 23 tests passed까지다. FAKE·Jetson·실기 HIL은 대기 중이다.

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
│           └── wp5_control.launch.py  생산용 두 노드 동시 기동
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
- 실기 전제: 바퀴 6개 부양, 48V 물리 E-stop 접근, 경쟁 chassis/teleop 프로세스 없음

## 빌드와 실행

Jetson에 SSH 접속한 직후 홈 `~`에서 시작한다.

```bash
cd ~/power-train-sw
docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros
docker exec -it powertrain_ros bash
```

컨테이너 안에서 빌드하고 환경을 source한다.

```bash
cd /workspace/ros2
colcon build
source install/setup.bash
colcon test
colcon test-result --verbose
```

생산 실행은 US-100와 차체 노드를 함께 기동한다. `safety_required=true`가 기본이다.

```bash
ros2 launch powertrain_ros wp5_control.launch.py
```

노드를 분리 진단할 때만 다음 진입점을 사용한다.

```bash
ros2 run powertrain_ros us100_safety
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
| `us100_safety_node` | `stop_mm` | 생산값 미확정 | 저속 HIL의 감지·제동 실측으로 결정 |

0.75초 freshness 계약은 `age > 0.75 s` 조건이 참이 된 뒤 다음 20 ms tick에 집행되므로
명목 E-stop 시점은 마지막 판정 후 0.75~0.77초다. 이는 0.5초 `/cmd_vel` watchdog과 서로
다른 고장 경로다. 생산 `stop_mm`은 HIL 측정 전까지 승인값이 없다.

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

- 관찰된 로컬 결과: `motor_control` 지원 suite 189 passed, `motor_gui` 91 passed.
- 임시 read-only ROS 워크스페이스: `robot_arm_msgs`, `powertrain_msgs`, `powertrain_ros`
  3패키지 build 완료, `powertrain_ros` 23 tests passed.
- FAKE 50 Hz acceptance와 Jetson ROS build/deploy: **PENDING / NOT RUN**.
- WP5.1 Jetson/10모터/US-100 HIL: **NOT RUN**.

환경 확인, 아홉 시나리오, 시간·CAN counter, `stop_mm` 산정과 최종 go/no-go는
[`2026-07-10-wp5-control-safety-hil.md`](../docs/reports/2026-07-10-wp5-control-safety-hil.md)에만 기록한다.

## 로봇팔 공유 계약

`src/powertrain_ros/powertrain_ros/contract.py`가 문자열 어휘의 단일 출처다. 남은 통합
항목은 `MISSION_STOP`, 락 해제 순서, 그리고
`ARRIVED_* → 팔 작업 → DONE → 재출발` 합동 1사이클이다.
