# 코너 모듈 컨트롤러 (Corner Module Controller) — 설계 문서

**작성일:** 2026-05-25
**트랙:** motor_control
**상태:** 설계 합의 완료 → 구현 계획 작성 대기

## 1. 개요 (Overview)

ZETIN 방어로봇(6륜 로커보기)의 **단일 코너 모듈** — 조향 모터 1개 + 구동 모터 1개 — 을
협조 제어하는 재사용 가능한 라이브러리와 DualSense 텔레옵 데모.

현재 보유 하드웨어로 실제 구동·검증 가능한 "6륜 시스템의 1/6 빌딩블록"이며, 나중에
모터가 늘면 이 모듈을 ×4(조향)/×6(구동)으로 인스턴스화하고 그 위에 애커만 키네마틱스
레이어만 올리는 구조를 목표로 한다.

### 목표 (Goals)
- 코너당 `(조향각°, 구동속도 turns/s)` 명령을 받는 깔끔한 제어 API.
- 그 위에 올라가는 DualSense 텔레옵 데모 앱.
- 트랜스포트(USB/CAN) 무관한 액추에이터 추상화 → 구동 USB→CAN 전환이 구현 교체만으로 가능.
- 하드웨어 없이 동작 검증 가능한 Fake 액추에이터 + 단위 테스트.

### 비목표 (Non-Goals, YAGNI)
- 6륜/4WS 애커만 키네마틱스 (미래 별도 레이어 — 본 모듈의 소비자).
- m/s 등 차체 단위 변환 (액추에이터 네이티브 단위만; 나중에 바퀴반경 0.1m 상수로 역산).
- motor_gui 웹 통합 (인터페이스를 GUI 비의존으로 두어 나중에 얇은 어댑터만 추가 — 접근 C).
- 협조 로직 기본 활성화 (hook만 마련, 기본 OFF).

## 2. 하드웨어 구성 (Hardware Context)

| 역할 | 장치 | 인터페이스 (현재) | 인터페이스 (추후) | 단위 |
|------|------|------------------|------------------|------|
| 조향 | CubeMars AK40-10 | CAN (socketcan can0) | CAN (동일) | 출력축 도(°), ERPM, A |
| 구동 | SunnySky X2212 + ODrive 3.6 | **USB** (`odrive.find_any`) | **CAN** (CANSimple) | turn, turn/s |

**전환 제약:** Jetson–AK–ODrive 전부 CAN으로 묶을 케이블이 아직 없어 현재는 구동 USB +
조향 CAN 혼합. 추후 CAN-only로 전환 예정. → 컨트롤러는 트랜스포트에 무관하게 설계한다.

**미래 핵심 제약:** 실전에서는 **4개 조향 모터를 애커만 조향으로 동시 제어**해야 한다.
미래 키네마틱스 레이어가 `(차체속도, 조향반경/yaw)` → 각 코너 `(steer_deg, drive_vel)`로
변환해 여러 `CornerModule.set()`을 호출하는 소비자가 된다. 코너의 기하 위치·애커만 계산은
그 레이어 몫이므로, 현재 `CornerModule` 인터페이스(코너당 steer°+drive turns/s)는
변경 없이 그 확장을 수용한다.

## 3. 아키텍처 (Architecture)

접근 A — 독립 경량 라이브러리. `motor_control/corner_module/` 신규 폴더.

```
corner_module/
├── __init__.py
├── actuator.py          # Actuator ABC + SteerActuator/DriveActuator 인터페이스
├── steer_ak40.py        # AK40 백엔드 SteerActuator (ak_control.AK40 래핑)
├── drive_odrive_usb.py  # ODrive USB DriveActuator (현재)
├── drive_odrive_can.py  # ODrive CAN DriveActuator (추후 — 인터페이스만 예약)
├── corner_module.py     # CornerModule 협조 제어기 (핵심)
├── fake.py              # 무하드웨어 테스트용 Fake 액추에이터
├── config.py            # 한계·상수 (조향각 min/max, 구동 vel limit, 워치독 등)
├── teleop_dualsense.py  # 데모 앱
└── tests/test_corner_module.py
```

**의존 방향:** `corner_module` → `motor_control/steering/ak_control.py` (기존 자산 재사용).
`motor_control`은 `motor_gui`를 **import 하지 않는다** (역의존 방지; motor_gui가
motor_control을 import하는 기존 방향 유지).

## 4. 컴포넌트 설계 (Component Design)

### 4.1 액추에이터 추상화 (`actuator.py`)

```python
class Actuator(ABC):
    def connect(self): ...      # 버스/USB 연결
    def arm(self): ...          # 폐루프 진입 — 점프방지(현재상태로 타깃 동기)
    def disarm(self): ...
    def tick(self): ...         # 매 루프 통신 서비스(명령 재전송·상태 폴링)
    def state(self) -> dict: ...# 정규화 텔레메트리
    def estop(self): ...        # 즉시 정지
    def close(self): ...

class SteerActuator(Actuator):
    def set_angle(self, deg: float): ...        # 출력축 목표각(°), config 한계로 clamp

class DriveActuator(Actuator):
    def set_velocity(self, turns_per_s: float): ...  # ODrive 네이티브 단위
```

- `tick()` 분리로 단일 제어 루프(50Hz)가 두 액추에이터를 블로킹 없이 서비스.
- AK는 매 tick 위치 재전송 + 상태 폴링; ODrive USB는 vel 쓰기 + 상태 읽기.

### 4.2 구체 드라이버

- **`steer_ak40.SteerAk40`**: `ak_control.AK40`를 래핑. `set_angle`→`send_pos_out`,
  `tick`→`poll`+위치 재전송, `state`→`pos_out_deg/spd_erpm/cur_a/fault`. AK 내장
  전류/fault 트립 활용. CAN 버스/모터 lifecycle은 `CANSession` 또는 직접 Bus 보유.
- **`drive_odrive_usb.DriveOdriveUsb`**: `odrive.find_any` → `axis1`. VELOCITY_CONTROL +
  PASSTHROUGH. `arm`→ 인코더 읽어 `input_vel=0`으로 폐루프 진입(점프방지), `set_velocity`→
  `input_vel`, `state`→`vel_estimate/Iq_measured`. `vel_limit`/`current_lim`은 NVM 설정값 사용.
- **`drive_odrive_can.DriveOdriveCan`** (추후): 동일 인터페이스. CANSimple
  `(NODE_ID<<5)|cmd`, fw-v0.5.6, 현재위치 동기 점프방지(기존 `odrive_can_drive.py` 패턴).
  본 작업에서는 인터페이스 자리만 예약(NotImplementedError 또는 미생성), 케이블 확보 후 구현.

### 4.3 코너 제어기 (`corner_module.py`)

상태머신:
```
DISCONNECTED → connect() → IDLE → arm() → ARMED ⇄ (set/tick 루프) → disarm() → IDLE → close()
                                            └── estop()/트립 → FAULT → (disarm으로 복귀)
```

공개 API:
```python
class CornerModule:
    def __init__(self, steer: SteerActuator, drive: DriveActuator, cfg: CornerConfig): ...
    def connect(self): ...
    def arm(self):     # 점프방지: steer 목표=현재각, drive 목표=0 으로 폐루프 진입
    def set(self, steer_deg: float, drive_vel: float):  # 목표 저장 + clamp + 워치독 갱신
    def tick(self):    # 안전검사 → 목표 push → 양쪽 tick() → 상태수집
    def state(self) -> dict: ...
    def disarm(self): ...
    def estop(self):   # 양쪽 즉시 정지, FAULT 진입
    def close(self): ...
    def run(self, hz=50):  # 편의 루프 (외부 루프가 tick() 직접 호출도 가능)
```

`state()` 반환 스키마 (motor_gui 텔레메트리 호환):
```python
{'mode': 'ARMED',
 'steer': {'target_deg': 12.0, 'actual_deg': 11.3, 'cur_a': 0.8, 'fault': 0, 'stale': False},
 'drive': {'target_vel': 2.0, 'actual_vel': 1.97, 'cur_a': 3.1},
 'faults': []}
```

### 4.4 안전 로직 (tick마다 검사, 우선순위 순)

1. **AK 전류/fault 트립** — `ak_control` 제공(`cur_a` 한계, `fault!=0`). 트립 시 `estop()`.
2. **텔레옵 워치독** — `set()`이 `cfg.watchdog_ms`(기본 300ms) 내 재호출 안 되면 구동을
   0으로 램프다운. 조향은 마지막 목표 유지.
3. **조향각 한계** — `set()`에서 `[cfg.steer_min_deg, cfg.steer_max_deg]`로 clamp.
4. **구동 속도 한계** — `[-cfg.drive_vel_limit, +cfg.drive_vel_limit]`로 clamp.
5. **CAN stale** — AK status 연속 미수신 임계 초과 시 `state().steer.stale=True` → estop.

### 4.5 협조 로직 (옵션, 기본 OFF)

`cfg.steer_gate`(기본 False): 켜면 `조향오차 = |목표각−실제각| > cfg.gate_deg`인 동안
구동속도를 비례 감속(또는 0). 바퀴가 크게 도는 중 스크럽 부하/구동 폭주 방지.
현재는 바퀴가 땅에 안 닿고 1개만 굴리므로 OFF, 로직 자리(hook)만 마련.

### 4.6 설정 (`config.py` — `CornerConfig` 데이터클래스)

| 필드 | 기본값 | 설명 |
|------|--------|------|
| `steer_min_deg` / `steer_max_deg` | −45 / +45 | 조향각 기구 한계 |
| `drive_vel_limit` | 5.0 | 최대 구동속도 (turns/s) |
| `watchdog_ms` | 300 | 텔레옵 입력 타임아웃 |
| `loop_hz` | 50 | 제어 루프 주기 |
| `steer_gate` | False | 협조 로직 on/off |
| `gate_deg` | 10.0 | 협조 감속 시작 조향오차 |
| `stale_ms` | 200 | AK status 미수신 stale 임계 |

## 5. 텔레옵 데모 (`teleop_dualsense.py`)

기존 `odrive_dualsense_vel_test.py` 패턴 계승.
```
DualSense 입력 → 매핑 → CornerModule.set(steer_deg, drive_vel) → 50Hz 루프 tick()
  • 좌스틱 X축    → 조향각   (−45°~+45°, deadzone 0.05)
  • RT/LT 트리거  → 구동속도 ((rt−lt) × drive_vel_limit, deadzone 0.05)
  • □(Square)     → arm/disarm 토글
  • ○(Circle)     → estop
```
- 매 사이클 `set()` 호출로 워치독 자동 갱신. 입력 없으면 스틱 중립=0이라 안전.
- 종료 시 finally 블록에서 `disarm()` → `close()`.
- 콘솔에 1Hz로 `state()` 요약 출력.

## 6. 테스트 전략 (Testing)

`fake.py`의 `FakeSteer`/`FakeDrive`: 명령을 받아 1차 지연으로 actual이 target에 수렴하는
간단 모델 (motor_gui FakeTransport 발상). 하드웨어 없이 CI 가능.

`tests/test_corner_module.py` 검증 항목:
1. 조향각 clamp (범위 밖 입력 → 한계로 제한)
2. 구동속도 clamp
3. 워치독 — `set()` 멈추면 N tick 후 drive→0
4. arm 점프방지 — arm 직후 drive 목표=0, steer 목표=현재각
5. estop → 양쪽 0 + FAULT 모드
6. AK fault 주입 시 자동 estop
7. `state()` dict 스키마 일치

## 7. 에러 처리 (Error Handling)

- **연결 실패**: `connect()`에서 USB `find_any` 타임아웃 / can0 미설정 → 명확한 예외 +
  "bash scripts/can_setup.sh 먼저" 안내 (ak_control 관례 계승).
- **런타임 CAN 끊김**: AK status 미수신 시 `state().steer.stale`, 임계 초과 → estop.
- **부분 실패**: 한쪽 액추에이터만 트립해도 `estop()`은 양쪽 모두 정지 (코너 단위 안전).
- **arm 안 된 상태 set()**: 무시 + 경고 로그 (제어 루프 보호).
- **로깅**: 표준 `logging` 모듈 (motor_gui worker 관례).

## 8. 미래 확장 (Future Work, 본 스펙 범위 밖)

- `drive_odrive_can.py` 구현 (CAN-only 전환, 케이블 확보 후).
- 애커만 4WS 키네마틱스 레이어 — 여러 `CornerModule`의 소비자.
- motor_gui 어댑터 (접근 C) — `state()` dict를 그대로 텔레메트리로 노출.
