# WP5.1 차체 제어·안전 경로 보강 설계

> **2026-07-11 완료 override:** 기존 10모터 실증과 이번 US-100·fail-safe·실제 50 Hz
> 결과를 합쳐 WP5.1 HIL 완료로 판정한다. 아래 Phase B 표현은 당시 검증 설계 이력이며,
> 지상 제동과 최종 `stop_mm` 선정은 차체 조립 후 실차 커미셔닝으로 재분류됐다.

> 작성: 2026-07-10
> 상태: Tasks 1~8 소프트웨어 완료, Jetson software-only FAKE PASS, Task 9 실기 HIL 대기
> 상위 계획: `docs/plans/2026-07-02-autonomous-driving-kickoff.md` WP5 확장·WP6 선행
> 구현 원칙: 제어·안전 정책은 ROS 없는 순수 Python, ROS2는 노드 사이 데이터 전달을 맡는 얇은 껍데기

> **운영 권한 오버라이드 (2026-07-11):** 결합 실기 launch는 commit `b715ba7`부터
> 기본값 없는 필수 `stop_mm` 인자를 요구하고, commit `60a813f`의 workspace-independent
> 계약시험이 이 게이트를 검증한다. `us100_safety_node`의 독립 실행 기본값 `200.0`은 진단과
> 통제된 저속 HIL용 임시 후보일 뿐 생산 승인값이 아니다. 실기 HIL은 Phase A 시나리오
> 1~8을 바퀴 부양 상태에서 먼저 수행하고, 별도 사용자 승인 뒤 Phase B 시나리오 9만
> 50 kg 차체 지상주행으로 수행한다. 아래의 과거 코드·절차 예시와 충돌하면 이 오버라이드와
> §10.3을 따른다. commit `49831bb`의 Jetson FAKE PASS는 실기 HIL 증거가 아니며 Phase A와
> Phase B는 모두 대기 중이다.

## 1. 목적

WP5는 `/cmd_vel → ChassisManager → 10모터` 실기 HIL을 통과했지만, 현재 구현을 그대로
WP6 오도메트리의 50 Hz 데이터 원천으로 쓰기에는 다음 문제가 있다.

1. ODrive 6축이 각각 4 ms, AK 4축이 각각 최대 5 ms를 순차 대기해 20 ms 제어 주기를
   넘길 수 있다.
2. US-100 `read()`는 최소 0.1초 블로킹하므로 차체 tick 안에서 호출할 수 없다.
3. 기존 US-100의 `None`은 거리 측정불가와 센서 무응답을 구분하지 못한다.
4. US-100 정지는 자동으로 풀리는 게이트이며, 사용자가 결정한 수동 해제형 E-stop 정책과
   다르다.
5. WP6이 CAN을 다시 폴링하지 않도록 6바퀴 실측 상태를 50 Hz로 전달하는 계약이 없다.

본 작업은 위 다섯 문제를 해결해 WP6이 신뢰할 수 있는 50 Hz 차체 상태와 일관된 안전
경로를 제공한다.

## 2. 범위

### 2.1 포함

- ODrive·AK의 주기 tick을 비블로킹 CAN I/O로 변경
- 순수 Python `SafetyInterlock` 추가
- 자동복구 정지 `MOTION_HOLD`와 수동해제 정지 `ESTOP` 분리
- US-100 거리·생존 상태 구분과 별도 ROS 노드
- `chassis_node` 최종 E-stop 집행과 안전 토픽 stale 감시
- `/wheel_states` 50 Hz 발행
- E-stop reset 후 별도 arm을 요구하는 2단계 복구
- 순수 단위시험, ROS FAKE 통합시험, Jetson HIL 기준

### 2.2 제외

- WP6 오도메트리 계산 자체
- 레인·추종 주행 명령 사이의 단일 `/cmd_vel` 권한 조정기
- L515 color/depth/IMU 기동과 PointCloud 선택 정책
- 로봇팔 `MISSION_STOP`·락 해제 순서 계약

제외 항목은 독립적인 실패 원인과 HIL 기준을 가지므로 WP5.1 완료 후 별도 spec으로
진행한다. 순서는 `WP5.1 → command authority → L515 경량 파이프라인 → WP6`이다.

## 3. 변경하지 않는 전제

- 단일 `can0`, 500 kbps, AK45-36 ×4와 ODrive/BL70200 ×6을 유지한다.
- `ChassisManager`가 CAN과 10모터의 유일한 소유자다.
- 명목 제어·건강 판정 주기는 50 Hz, 한 tick 예산은 20 ms다.
- US-100 물리 측정 주기는 50 Hz로 억지로 올리지 않는다. 센서는 별도 노드에서 약
  5~10 Hz로 읽고 차체가 최신 판정을 매 50 Hz마다 확인한다.
- ODrive 정본 설정은 pp=10, cpr=60, bandwidth=30, vel_gain=0.12,
  vel_integrator_gain=0.2, node 11~16이다.
- L515는 파워트레인 RGB/depth/IMU, D435i는 로봇팔 전용이다.
- 로봇팔과 공유하는 `robot_arm_msgs` 5종은 변경하지 않는다.

## 4. 전체 구조

```text
US-100 UART
  └─ us100_safety_node (5~10 Hz, 블로킹은 이 프로세스 안에서만)
       └─ /safety_verdict (powertrain_msgs/SafetyVerdict)
            ↓ latest cache
/cmd_vel → chassis_node (50 Hz)
            ├─ SafetyInterlock (순수 Python)
            ├─ ChassisManager (순수 Python)
            │    └─ CornerModule ×6 → can0 → 10모터
            └─ /wheel_states (powertrain_msgs/WheelStates, 50 Hz)
```

ROS 노드는 센서·제어 프로세스 사이 전달과 관찰만 담당한다. E-stop 분류, latch, reset
조건, CAN stale 판정, 바퀴 상태 추출은 `motor_control/`의 순수 Python 코드이며 ROS 없이
pytest할 수 있어야 한다.

기존 `ChassisManager.monitor.tick()` 경로는 제거한다. `chassis_node`는 ROS 메시지를 원시
Python 값으로 변환해 `ChassisManager.update_external_safety(status, estop_required,
detail)`에 전달한다. `ChassisManager`와 `SafetyInterlock`은 ROS 메시지 타입을 import하지
않는다.

## 5. 50 Hz CAN I/O

### 5.1 현재 문제

`ChassisManager.tick()`은 6개 코너를 순차 호출한다. 현재 `DriveOdriveCan.tick()`은 매번
`_poll(0.004)`, `SteerAk40.tick()`은 `poll(0.005)`를 호출한다. ODrive만 최저 약 24 ms를
소비하므로 50 Hz를 보장할 수 없다.

### 5.2 새 규칙

각 액추에이터 tick은 다음 두 단계만 수행하며 기다리지 않는다.

1. 커널 SocketCAN 버퍼에 이미 도착한 프레임을 `recv(timeout=0.0)`으로 모두 drain한다.
2. 이번 tick의 명령과 다음 상태용 RTR을 전송하고 즉시 반환한다.

응답은 다음 tick 시작 때 회수하므로 텔레메트리는 최대 한 tick(약 20 ms) 늦을 수 있다.
제어 주기의 결정성을 얻는 대신 허용하는 의도적 지연이다.

`_poll(0.0)`처럼 기존 함수 인자만 바꾸지 않는다. 현재 `_poll()`은 deadline이 0이면
`recv()`를 한 번도 호출하지 않기 때문이다. 별도 `_drain_available(max_frames=16)`을 두어
버퍼 고갈까지 논블로킹으로 읽는다. CAN 필터가 노드별 프레임만 통과시키므로 16프레임
상한은 정상 트래픽을 충분히 수용하면서 비정상 backlog가 한 tick을 독점하지 못하게 한다.

### 5.3 arm 예외

`arm()`의 초기 상태 시드용 짧은 대기는 50 Hz 루프 진입 전 실행되므로 이번 비블로킹
요구에서 제외한다. 주기 호출되는 `tick()`과 `state()`만 블로킹 0이어야 한다.

### 5.4 시간 계측

`chassis_node`는 각 tick의 실행시간과 누적 overrun 수를 기록한다. 정상 기준은 다음과 같다.

- 목표 주기: 20 ms
- 60초 HIL에서 `/wheel_states` 평균 발행률: 49~51 Hz
- 지속적으로 48 Hz 미만인 구간 없음
- tick 실행시간 99백분위가 20 ms 미만
- CAN bus-off·error-passive 증가 0

## 6. 정지 용어와 상태

### 6.1 세 상태

| 상태 | 의미 | 해제 |
|---|---|---|
| `RUN` | 주행 허용 | 해당 없음 |
| `MOTION_HOLD` | 자동복구 가능한 주행 억제 | 원인이 사라지면 자동 |
| `ESTOP` | 위험 또는 고장으로 전체 모터 정지 후 latch | 수동 reset 후 별도 arm |

개별 모터·코너의 `FAULT`는 장치 고장 진단 명칭으로 유지한다. 장치 `FAULT`가 차체까지
전파되면 차체 안전 상태는 `ESTOP`이다.

### 6.2 `MOTION_HOLD` 원인

- `/cmd_vel` 타임아웃
- 텔레옵 연결 단절로 입력 갱신 중단
- 미래 WP8의 미션 정차
- US-100가 기동 후 첫 상태를 확정하기 전인 `US100_CHECKING`

hold 중에는 구동 목표를 0으로 보내고 조향은 현재 목표를 유지한다. 새 `/cmd_vel`이
계속 들어와도 실제 구동은 0이며 hold 중 수신한 명령은 저장하지 않는다. 원인이 사라진
뒤 새 명령을 한 번 받아야 내부 `command_recovery` hold가 풀린다. 따라서 재무장 없이
자동 복구하되, hold 직전 또는 hold 중의 비영점 명령을 자동 재생하지 않는다.

### 6.3 `ESTOP` 원인

- 수동 E-stop 버튼 또는 `~/estop` 서비스
- US-100 유효 거리 `< stop_mm`
- US-100 거리 요청과 생존 확인이 연속 `fail_stop_count`회 모두 무응답
- `/safety_verdict` 최초 수신 실패 또는 수신 후 stale
- AK fault·stale·과전류
- ODrive axis error·stale
- 제어 tick에서 밖으로 전파된 복구 불가능한 하드웨어 예외

### 6.4 latch와 reset

`SafetyInterlock`은 다음 순수 Python API를 제공한다.

```python
set_motion_hold(source: str, active: bool, detail: str = "") -> None
set_estop_condition(source: str, active: bool, detail: str = "") -> None
trip_estop(source: str, detail: str = "") -> None
reset_estop() -> bool
snapshot() -> SafetySnapshot
```

- `trip_estop()`과 활성 E-stop 조건은 즉시 latch한다.
- 최초 원인, 상세, 발생시각과 현재 활성 원인 집합을 보존한다.
- 같은 원인의 반복 호출은 상태를 중복 생성하지 않는다.
- US-100 근거리·무응답·안전 토픽 stale처럼 정지 중에도 관찰 가능한 위험은
  `set_estop_condition()`으로 유지하며, 활성 상태에서는 `reset_estop()`이 `False`다.
- 수동 버튼·장치 fault·제어 예외는 `trip_estop()` 이벤트로 latch한다. 운영자가 장치를
  점검한 뒤 reset할 수 있고, fault가 실제로 남아 있으면 arm 또는 다음 tick에서 즉시
  다시 E-stop한다.
- interlock reset 성공 후 `ChassisManager.reset_estop()`이 코너를 안전한 `IDLE`로만
  옮긴다. 모터를 움직이려면 이후 `arm()`이 필요하다.
- `arm()`은 latch 상태에서 거부한다.
- 원인이 재발하거나 장치 fault가 남아 있으면 다음 tick에서 다시 E-stop한다.

### 6.5 모터 정지의 견고성

`ChassisManager.estop()`은 멱등이어야 한다. 한 코너의 `estop()`이 예외를 내도 나머지
코너를 계속 정지시키고, 전체 시도 후 최초 예외를 진단에 남긴다. E-stop 처리 도중의
추가 예외가 latch를 풀거나 `RUN`으로 되돌려서는 안 된다.

## 7. US-100 상태 모델

### 7.1 상태

| 상태 | 판정 | 차체 동작 |
|---|---|---|
| `CHECKING` | 기동 후 생존 미확정 또는 거리·생존 응답 1~2회 누락 | `MOTION_HOLD` |
| `VALID` | 20~4000 mm 유효 거리 수신 | `< stop_mm`이면 `ESTOP`, 나머지 `RUN` |
| `INVALID_READING` | 거리값은 유효하지 않지만 0x50 생존 응답 수신 | `RUN` |
| `NO_RESPONSE` | 거리와 0x50 생존 응답이 연속 3회 없음 | `ESTOP` |

사용자 결정에 따라 `warn_mm`은 주행 억제에 쓰지 않는다. 진단용으로 남길 수 있지만
`stop_mm` 미만과 `NO_RESPONSE`만 E-stop한다.

### 7.2 생존 확인

거리 명령 `0x55`에서 유효 거리를 얻지 못하면 온도 명령 `0x50`을 보내 센서 UART
제어부의 생존을 확인한다.

- 온도 응답 있음: 빈 공간·무반사·범위 밖으로 보고 `INVALID_READING`
- 온도 응답도 없음: 실패 횟수 증가, 1~2회는 `CHECKING`
- 다음 정상 거리 또는 생존 응답: 실패 횟수 0으로 초기화
- 연속 실패가 `fail_stop_count=3`에 도달: `NO_RESPONSE`

거리 응답과 온도 응답이 섞이지 않도록 요청 전 입력 버퍼를 비우고, 요청별 기대 바이트
수와 deadline을 분리한다. 구체 deadline은 기존 0.1초를 초기값으로 사용하며, 실제 US-100
HIL에서 빈 공간·가까운 표적·센서 분리 세 조건의 바이트와 지연을 기록해 조정한다.

### 7.3 알려진 한계

`0x50`은 UART 제어부 생존만 확인한다. 초음파 송신기나 수신기만 고장 난 경우에도 온도
응답이 오면 `INVALID_READING`으로 분류되어 정상 통과할 수 있다. 이는 “뻥 뚫린 구간의
측정불가를 정지시키지 않는다”는 사용자 정책의 잔여 위험이며 문서·HIL 결과에 명시한다.

### 7.4 거리 기준

`us100_safety_node`를 독립 실행할 때의 `stop_mm=200.0` 기본값은 진단과 통제된 저속
HIL을 위한 임시 후보다. 결합 실기 launch에는 기본값이 없으며, `200.0`을 생산값으로
승인하지 않는다. 1.5 m/s에서 센서 0.1초 지연만으로 약 150 mm를 이동하므로 실제 제동
여유가 부족할 수 있다. 최종 기준은 다음 조건으로 Phase B HIL에서 정한다.

```text
stop_mm ≥ 최고속도 × (최악 센서주기 + 처리지연) + 실측 제동거리 + 안전여유
```

기준을 실측하기 전 실기 시험은 통제된 저속으로 제한한다. Phase A에서 200 mm 후보를
사용할 때도 `stop_mm:=200`을 명시해야 한다. 생산 실행은 Phase B에서 실측·승인·재검증한
값을 `stop_mm:=<HIL-approved-mm>`으로 명시한 경우만 허용한다.

## 8. ROS 계약

파워트레인 내부 계약은 로봇팔 팀의 `robot_arm_msgs`와 분리된 로컬
`powertrain_msgs` 패키지에 둔다.

### 8.1 `SafetyVerdict.msg`

```text
uint8 CHECKING=0
uint8 VALID=1
uint8 INVALID_READING=2
uint8 NO_RESPONSE=3

std_msgs/Header header
uint8 status
float32 distance_mm
bool estop_required
uint32 consecutive_failures
string detail
```

- 유효하지 않은 `distance_mm`는 IEEE NaN이다.
- `estop_required`는 센서 정책의 결과이며, `chassis_node`는 별도로 토픽 freshness를
  검사해 최종 E-stop을 결정한다.
- QoS는 reliable, depth 1이다.

### 8.2 `WheelState.msg`

```text
string name
string corner_mode
float32 drive_turns_per_s
float32 steer_deg
float32 drive_current_a
float32 steer_current_a
bool drive_stale
bool steer_stale
uint32 drive_axis_error
uint8 steer_fault
```

### 8.3 `WheelStates.msg`

```text
std_msgs/Header header
string chassis_mode
string stop_state
bool healthy
float32 tick_duration_ms
uint32 overrun_count
powertrain_msgs/WheelState[] wheels
```

- 순서는 `front_left`, `front_right`, `mid_left`, `mid_right`, `rear_left`,
  `rear_right`로 고정한다.
- `drive_turns_per_s`와 `steer_deg`는 명령값이 아니라 실제 피드백이다.
- `healthy`는 모든 바퀴가 non-stale이고 fault·axis_error가 0일 때만 참이다.
- `chassis_node`는 `ChassisManager.tick()` 직후 같은 50 Hz 타이머에서 발행한다.
- WP6은 이 토픽만 구독하고 CAN을 직접 열지 않는다.

### 8.4 차체 노드 파라미터와 서비스

추가 파라미터:

```text
safety_required=true
safety_topic_timeout=0.75
safety_startup_timeout=1.0
```

- 실기 기본은 `safety_required=true`다.
- `false`는 FAKE·벤치 시험에서만 명시적으로 사용하며 경고 로그를 남긴다.
- `safety_topic_timeout`의 운영 기본값이자 허용 최솟값은 0.75초다. US-100의 거리 요청과
  생존 확인 한 sample이 최악 0.4초 걸릴 수 있고, 타이머 스케줄링·DDS 전달 지연에
  0.35초 여유를 둔 값이다. 0.75초 미만 또는 유한하지 않은 값은 시작 단계에서 거부한다.
- 기존 `~/estop`, `~/arm`, `~/disarm`을 유지하고 `~/reset_estop` Trigger 서비스를
  추가한다.
- `~/arm`은 latch 상태에서 실패와 원인을 반환한다.

## 9. 오류 처리

- 안전 토픽이 아직 없으면 시작 후 1초까지 `MOTION_HOLD`, 이후 `ESTOP`이다.
- 마지막 안전 토픽의 age가 0.75초를 초과한 뒤 다음 50 Hz tick에서
  `SAFETY_TOPIC_STALE` E-stop이다(명목 0.75–0.77초).
- 유효한 안전 토픽을 다시 받아도 E-stop은 자동 해제하지 않는다.
- CAN 송신 예외를 드라이버가 흡수한 경우 stale 상태가 E-stop을 일으킨다.
- `ChassisManager.tick()` 밖으로 나온 예외는 `CONTROL_EXCEPTION` E-stop을 일으키고
  50 Hz 타이머는 살아 있어야 한다.
- 잘못된 wheel 이름·누락된 6바퀴 매핑은 시작 단계에서 실패한다.
- 메시지 발행 실패가 모터 정지 동작을 막아서는 안 된다. 순서는 항상
  `안전 평가 → 모터 tick/정지 → snapshot → ROS 발행`이다.

## 10. 시험 전략

### 10.1 순수 Python TDD

`SafetyInterlock`:

- hold 원인 활성·해제에 따른 `RUN ↔ MOTION_HOLD`
- E-stop 최초 원인 latch와 반복 호출 멱등성
- 활성 위험 중 reset 거부
- 위험 해소 후 reset은 `IDLE`이며 자동 arm하지 않음
- hold가 풀려도 E-stop은 풀리지 않음

US-100:

- 먼 유효거리 → `VALID`, E-stop 없음
- `stop_mm` 미만 → E-stop
- 범위 밖 거리 + 온도 응답 → `INVALID_READING`, E-stop 없음
- 거리·온도 무응답 1~2회 → `CHECKING`, 자동복구형 `MOTION_HOLD`
- 3회 연속 무응답 → `NO_RESPONSE`, E-stop
- 정상 응답 복구 후 failure count 초기화

CAN:

- tick이 `recv(timeout=0.0)`만 사용
- 이전 tick에 큐에 든 encoder/Iq/heartbeat를 다음 tick에서 반영
- 비정상 backlog가 `max_frames`를 넘겨 한 tick을 독점하지 않음
- stale·axis error·AK fault가 기존 계약대로 유지

ChassisManager:

- `MOTION_HOLD`에서 6구동 목표 0, 차체는 arm 상태 유지
- E-stop에서 6코너 모두 `FAULT`/정지
- 한 코너 estop 예외에도 나머지 코너 정지 시도
- reset 전 arm 거부, reset 후 별도 arm 성공
- 6바퀴 snapshot 필드와 고정 순서

구동 건강 판정은 `CornerModule`이 `drive.state()`의 `stale`과 `axis_error`를 검사해
장치 `FAULT`로 올리고, `ChassisManager`가 이를 차체 `ESTOP`으로 전파한다. ODrive
과전류는 보드의 설정된 current limit과 axis error를 정본으로 사용한다. 실측 근거 없는
별도 Python 전류 임계값은 이번 작업에서 추가하지 않는다.

### 10.2 ROS FAKE 통합시험

- `powertrain_msgs`와 `powertrain_ros` colcon build
- fake chassis + fake safety publisher로 50 Hz `/wheel_states`
- 가까운 거리 verdict → `ESTOP`
- 이후 먼 거리 verdict → latch 유지
- reset → `IDLE`, arm 전 구동 0
- safety publisher 중단 → age >0.75초 후 다음 50 Hz tick에서 `ESTOP`
  (명목 0.75–0.77초)
- `safety_required=false`에서 경고 후 FAKE 구동 가능

### 10.3 Jetson HIL

실기 검증은 하나의 최종 HIL batch로 이어서 수행할 수 있지만, 물리 승인 경계는 두
Phase로 분리한다. Phase A의 바퀴 부양 승인은 Phase B 지상주행 승인으로 승계되지 않는다.

#### Phase A — 시나리오 1~8, 바퀴 6개 부양

사용자에게 바퀴 6개 완전 부양, 접근 가능한 48V 물리 E-stop, AK ×4·ODrive ×6 전원과
단일 `can0` 500 kbps, US-100 `/dev/ttyTHS1`, 주변 인원 구동 고지를 확인받는다. 좀비
teleop/chassis 프로세스와 CAN loopback이 없음을 확인한 뒤에만 실행한다. 생산
`stop_mm`은 아직 없으므로 통제된 저속에서 임시 200 mm 후보를 쓸 경우에도 다음처럼
명시한다.

```bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=200
```

인자를 생략한 결합 launch는 실패해야 한다. `200`은 Phase A 진입을 위한 임시값이며
생산 승인으로 해석하지 않는다.

1. CAN만으로 60초 50 Hz tick·wheel state rate·bus error 측정
2. 빈 공간 또는 무반사에서 `INVALID_READING`이고 주행 허용 확인
3. 먼 표적에서 `VALID`·주행 허용 확인
4. 가까운 표적에서 즉시 E-stop 확인
5. 표적 제거 후 latch 유지 확인
6. reset 후 `IDLE`, 별도 arm 후에만 회전 확인
7. US-100 분리 후 3회 실패로 E-stop 확인
8. 모터 한 축 fault/stale 주입 후 전체 10모터 정지 확인

#### Phase B — 시나리오 9, 별도 승인 50 kg 지상주행

시나리오 1~8을 마친 뒤 정지·disarm 상태를 확인한다. 바퀴를 내리기 직전에 통제된
주행로, 최저 속도부터 올리는 단계적 저속 계획, spotter, exclusion zone, 즉시 접근 가능한
물리 E-stop을 새로 확인하고 사용자의 명시적 승인을 다시 받아야 한다. 이 확인이 하나라도
없으면 바퀴를 내리거나 시나리오 9를 시작하지 않는다.

9. 실제 50 kg 차체의 속도별 감지지연·제동거리를 측정하고 안전여유를 더해
   `stop_mm`을 결정한 뒤 같은 조건에서 재검증한다.

승인 뒤 생산 결합 실행은 다음 형식만 허용한다.

```bash
ros2 launch powertrain_ros wp5_control.launch.py stop_mm:=<HIL-approved-mm>
```

## 11. 완료 기준

- 모든 신규 순수 Python 시험 통과
- 전체 기존 motor_control 회귀시험 통과
- ROS2 colcon build와 FAKE 통합시험 통과
- 60초 HIL에서 `/wheel_states` 평균 49~51 Hz, 지속 48 Hz 미만 없음
- HIL 전후 CAN bus-off·error-passive 증가 0
- US-100 네 상태와 사용자 결정 E-stop 정책 실기 확인
- 모든 E-stop 원인이 수동 reset 전까지 latch
- reset과 arm이 분리되어 reset만으로 모터가 움직이지 않음
- 권위 계획서·ROS README·AI 에이전트 지침이 구현과 일치

## 12. 후속 작업

1. 단일 `/cmd_vel` 작성자: lane/follow 명령을 mission sequencer 또는 command arbiter가
   선택하고 최종 `/cmd_vel`은 한 노드만 발행한다.
2. L515 경량 파이프라인: color+depth image+IMU를 기본으로 하고 PointCloud2는 필요할
   때만 활성화한다.
3. WP6: `/wheel_states`와 `/l515/imu`를 이용한 순수 `OdometryEstimator` + 얇은 ROS 노드.
