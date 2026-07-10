# WP5.1 차체 제어·안전 HIL 검증 보고서

> **상태: NOT RUN — 최종 실기 HIL 미실행**
>
> 작성 기준일: 2026-07-10 KST
> 범위: 파워트레인 SW의 50 Hz 차체 제어, US-100 안전 경로, 10모터 단일 CAN, ROS 계약
> 소프트웨어 상태: **Tasks 1~8 완료**
> 현재 증거: 로컬 `motor_control` 189 passed, `motor_gui` 91 passed, 임시 read-only ROS
> 워크스페이스 3패키지 build와 `powertrain_ros` 23 tests passed. FAKE·Jetson·실물 측정값은 없음.
> 판정: **PENDING**. 아래 체크와 측정란을 실제 관찰로 채우기 전 HIL 통과를 주장하지 않는다.

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

### 1.2 사용자 물리 준비 확인 — 응답 전 하드웨어 접근 금지

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

### 1.3 목표 구성

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
| `stop_mm` | 생산값 미확정 | `— (HIL 산정 전 승인값 없음)` |

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
| 지원 `motor_control` suite | **189 passed** | 없음 | 로컬 관찰 |
| `motor_gui/tests` | **91 passed** | 없음 | 로컬 관찰 |

### 4.2 로컬 임시 read-only ROS build/test — 관찰 결과

소스 worktree에 ROS build 산출물을 쓰지 않는 임시 복사본에서 다음 결과를 관찰했다. 이는
메시지·패키지 build/test 증거일 뿐 Jetson 배포, FAKE acceptance, DDS 실기, UART 또는 CAN
증거가 아니다.

| 필드 | 기록 |
|---|---|
| build 범위 | `robot_arm_msgs`, `powertrain_msgs`, `powertrain_ros` — **3 packages passed** |
| test 범위 | `powertrain_ros` — **23 tests passed** |
| source/worktree 변경 | 없음; 임시 read-only 검증 |
| 하드웨어·Jetson·FAKE | 사용 안 함 / `NOT RUN` |

### 4.3 FAKE acceptance — PENDING

- [ ] Jetson에서 `robot_arm_msgs`, `powertrain_msgs`, `powertrain_ros`를 build했다.
- [ ] `colcon test-result --verbose`가 failure·error 0임을 기록했다.
- [ ] 경쟁 chassis/teleop 프로세스가 없는 새 FAKE chassis를 기동했다.
- [ ] 먼 판정을 10 Hz로 발행하고 arm·`/cmd_vel`을 수행했다.
- [ ] `/wheel_states` 60초 평균 49~51 Hz, 지속 48 Hz 미만 없음, tick p99 <20 ms를 기록했다.
- [ ] 근거리 → `ESTOP`, 먼 판정 후 latch 유지, reset → `IDLE`, 별도 arm을 확인했다.
- [ ] publisher 종료 후 `age >0.75 s` 다음 tick, 명목 0.75~0.77초에 `ESTOP`을 확인했다.

| 필드 | 기록 |
|---|---|
| FAKE용 ROS build/deploy 결과 | `— (PENDING; NOT RUN)` |
| FAKE 실행 commit | `— (PENDING; NOT RUN)` |
| 60초 `/wheel_states` 평균/최저 | `— Hz` / `— Hz` |
| tick p99 / overrun 증가 | `— ms` / `—` |
| close/far/reset/arm 관찰 | `— (PENDING; NOT RUN)` |
| safety publisher kill→ESTOP | `— s (PENDING; NOT RUN)` |
| 로그 경로 | `— (PENDING; NOT RUN)` |

### 4.4 Jetson pre-HIL 상태 — PENDING

다음 명령은 사용자 준비 확인 뒤 Jetson에서 실행한다. 이 문서 작성 중 실행하지 않았다.

```bash
git status --short
git rev-parse HEAD
ip -details -statistics link show can0
docker exec powertrain_ros ps -ef
```

- [ ] Jetson 추적 파일에 겹치는 로컬 변경이 없다.
- [ ] 기존 미추적 `motor_control/vision/tests/`를 보존했다.
- [ ] 검증 branch와 40자 commit이 로컬·origin·Jetson에서 일치한다.
- [ ] 좀비 teleop/chassis/모터 제어 프로세스가 없다.
- [ ] can0 loopback이 명시적으로 OFF다.
- [ ] `scripts/can_setup.sh`로 can0 500 kbps를 올렸다.
- [ ] node 1~4·11~16 heartbeat/상태 존재를 기록했다.
- [ ] `/wheel_states` baseline을 기록했다.

| pre-HIL 필드 | 값 |
|---|---|
| can0 state/bitrate/restart-ms/loopback | `— (NOT RUN)` |
| tx/rx packets·errors | `— / — / — (NOT RUN)` |
| error-passive/bus-off/restarts | `— / — / — (NOT RUN)` |
| 경쟁 프로세스 목록 | `— (NOT RUN)` |
| AK id 1~4 상태 | `— (NOT RUN)` |
| ODrive node 11~16 상태 | `— (NOT RUN)` |
| `/wheel_states` baseline | `— (NOT RUN)` |
| 증거 경로 | `— (NOT RUN)` |

## 5. 실행 시나리오

모든 시나리오는 사용자 물리 준비 확인 뒤 순서대로 실행한다. 공통 CAN 필드는 시나리오
전후 `state`, tx/rx packet/error, error-passive, bus-off, restart의 절대값과 delta를 모두
기록한다. 공통 안전 필드는 US-100 status, chassis stop state/mode, latch, active/first
E-stop source를 기록한다. 동영상만으로 수치를 추정하지 않는다.

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

### 5.9 시나리오 9 — 속도별 감지·제동과 `stop_mm`

**기대:** 통제된 저속부터 각 속도의 최악 센서주기, 처리지연, 실제 제동거리를 측정하고
안전여유를 더해 생산 `stop_mm`을 결정한다. 측정 전 임의 값을 승인하지 않는다.

| 필드 | 기록 |
|---|---|
| 실행 체크 / 시각 KST | [ ] `NOT RUN` / `—` |
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
| 지원 로컬 `motor_control` suite | `PASS (189 passed)` | 로컬 관찰 |
| `motor_gui/tests` | `PASS (91 passed)` | 로컬 관찰 |
| 임시 read-only ROS 3패키지 build | `PASS` | `robot_arm_msgs`, `powertrain_msgs`, `powertrain_ros` |
| `powertrain_ros` tests | `PASS (23 tests passed)` | 임시 read-only ROS 워크스페이스 |
| Jetson ROS build/test | `PENDING / NOT RUN` | `—` |
| FAKE ROS 60초·freshness | `PENDING / NOT RUN` | `—` |
| 사용자 물리 준비 확인 | `PENDING / NOT RUN` | `—` |
| HIL 시나리오 1~9 | `PENDING / NOT RUN` | `—` |
| 생산 `stop_mm` | `PENDING / 미확정` | `—` |

### 7.2 잔여 위험

| 위험 | 현재 통제 | HIL 뒤 판단 |
|---|---|---|
| 0x50은 MCU/UART만 확인해 초음파 송수신부 고장을 놓칠 수 있음 | `INVALID_READING`을 진단에 노출, 저속·운영 감시 | `— (PENDING)` |
| 단일 can0 공통고장과 물리층 간헐 접촉 | ADM3053·워치독·counter delta·물리 E-stop | `— (PENDING)` |
| safety topic stale까지 명목 0.75~0.77초 | 생산 최솟값 고정, `stop_mm` 식에 지연 반영 | `— (PENDING)` |
| node 12/16 HALL 품질과 저속 코깅 | fault/stale 전체 E-stop, 육안 확인 | `— (PENDING)` |
| `/wheel_states` 발행 실패가 제어와 분리됨 | 로그·rate·overrun 관찰 | WP6 입력 안전정책에서 결정 |
| 복수 `/cmd_vel` 작성자 충돌 | 현재 미해결을 명시 | HIL 뒤 command-authority spec |
| L515 PointCloud2 기본 사용 시 불필요한 부하 | color/depth image+IMU 기본, PointCloud2 opt-in | 별도 L515 경량 spec |
| reset 후 무심코 재가동 | reset→`IDLE`, 별도 arm | 시나리오 6 실기 확인 대기 |

### 7.3 최종 판정

| 항목 | 기록 |
|---|---|
| 최종 go/no-go | **PENDING — HIL NOT RUN** |
| 판정자 / 시각 | `— (PENDING)` |
| 미통과 항목 | FAKE, Jetson ROS, 사용자 확인, HIL 1~9, 생산 `stop_mm` |
| 조건부 제한 | WP5.1 HIL 승인 전 자율 실차 운용 승인 없음 |
| 승인 commit / 설정 | `— (PENDING)` |

HIL 승인 뒤에만 command-authority spec → L515 경량 color/depth image+IMU pipeline
(PointCloud2 optional) → WP6 순서로 진행한다. WP8, `MISSION_STOP`, unlock ordering,
`ARRIVED_* → 팔 작업 → DONE → 재출발` 합동 1사이클은 별도 미결이다.

## 8. 코드·참고

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
