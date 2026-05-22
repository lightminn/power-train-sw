# AK-CAN GUI (컴포저블 CAN 디바이스 구조) 설계

**작성일**: 2026-05-21
**상태**: 승인됨 (구현 대기)

## 1. 개요 / 목적

motor_gui 의 ODrive-USB 트랙(`UsbOdriveBackend`)에서 검증된 웹 진단/제어 GUI 기능을
**AK40 조향 모터(CAN)** 에도 제공한다. 더 나아가, 단일 CAN 버스에 ODrive 와 AK 를
함께 얹어 동시 제어하는 미래 단계까지 **재작성 없이 확장**되도록 디바이스 컴포지션
구조를 도입한다.

## 2. 단계별 로드맵 (전체 그림)

| 단계 | 구현 | 본 spec 범위 |
|---|---|---|
| 1. ODrive-USB | `UsbOdriveBackend` (모놀리식 Transport) | 완료 (변경 없음) |
| **2. AK-CAN** | `CanTransport([AkDevice])` `--track ak` | **본 spec** |
| 3. ODrive-CAN | `CanTransport([OdriveCanDevice])` | 다음 spec (ODrive 를 CAN 으로 이전 후) |
| 4. ODrive+AK 동시 | `CanTransport([OdriveCanDevice, AkDevice])` `--track can` | 다음 spec (2·3 디바이스 조합) |

본 spec 은 **2단계(AkDevice 구현 + 검증)** 와 **3·4단계를 가능케 하는 컴포저블
아키텍처 확정**을 다룬다. `OdriveCanDevice` 는 인터페이스에 맞춰 설계만 확정하고
구현은 다음 spec 으로 분리한다 (ODrive 가 현재 USB 에 있어 ODrive-CAN HIL 검증 불가).

## 3. 아키텍처

### 3.1 디바이스 인터페이스 `CanDevice` (ABC)

한 CAN 버스에 얹히는 제어 유닛의 계약. AK 와 ODrive-CAN 양쪽에 들어맞도록 설계한다.

```python
class CanDevice(ABC):
    name: str                          # "ak" / "odrive"
    def capabilities_fragment(self) -> dict:
        """이 디바이스의 signals/commands/control_modes/inputs/tunables/limits/signal_meta 조각."""
    def request(self, bus) -> None:
        """폴링형 디바이스가 RTR 송신. ODrive=Get_Iq/Temp/Bus_VI, AK=no-op."""
    def on_rx(self, msg) -> None:
        """내 프레임이면 캐시 상태 갱신. AK=STATUS_1, ODrive=heartbeat/enc/iq/temp/vbus."""
    def tick(self, bus) -> None:
        """워치독 재전송. AK=마지막 활성 명령 재송신(스로틀), ODrive=no-op(래치)."""
    def sample(self) -> dict:
        """캐시 상태 → 텔레메트리 조각 {'<dev>.<sig>': value}."""
    def apply(self, bus, op: str, args: dict) -> dict:
        """이 디바이스 대상 명령 처리. ack dict 반환."""
    def close(self, bus) -> None:
        """안전 정지."""
```

### 3.2 `CanTransport` (Transport ABC 구현)

버스 1개 + 디바이스 리스트를 묶는 얇은 집계자.

- `__init__(devices, channel="can0")`: 디바이스 리스트 보관.
- `connect()`: `can.interface.Bus(socketcan)` 1개 open (실패 시 `TransportError`).
- `sample()`:
  1. 각 디바이스 `request(bus)` (RTR 송신)
  2. **버스 recv 단일 드레인 루프** (deadline ~8ms): 매 프레임을 모든 디바이스
     `on_rx(msg)` 에 분배 (각자 자기 ID 만 파싱)
  3. 각 디바이스 `tick(bus)` (AK 워치독 재전송)
  4. 각 디바이스 `sample()` 병합 + `{"t_mono": ...}`
- `apply(cmd)`: `cmd["target"]` → 해당 디바이스 `apply()` 라우팅. 미지원 target 은 ack 거부.
- `capabilities()`: 디바이스 조각 병합 — devices 리스트, signals 합집합, commands/control_modes/
  inputs/tunables/limits 는 디바이스별 맵, signal_meta 합집합, notes.
- `close()`: 전 디바이스 `close()` + `bus.shutdown()`.

### 3.3 버스 공존 / recv 소유권

- AK = 확장 ID(29bit), ODrive CANSimple = 표준 ID(11bit) → 한 버스 공존, ID 로 필터.
- **recv 는 `CanTransport` 단독 소유**. 기존 `AK40.poll()` 의 자체 recv 루프는 멀티
  디바이스 환경에서 프레임을 훔치므로 사용하지 않는다. `AkDevice` 는 공유 루프가
  넘겨주는 프레임을 `AK40._parse_status(data)` 에 먹인다 (AK40 의 send/parse 메서드는 재사용).

## 4. `AkDevice` 상세

### 4.1 제어 모드 / 명령

| 모드 | AK 패킷 | 입력 키 | 단위 | 메서드 |
|---|---|---|---|---|
| position | PKT_SET_POS_SPD(6) `>ihh` (deg×1e4, spd, acc) | `pos_deg` | ° | `send_pos_out` (기존) |
| velocity | PKT_SET_RPM(3) `>i` (erpm) | `rpm` | RPM(출력축) | `send_rpm_out` (기존) |
| brake | PKT_SET_BRAKE(2) `>i` (mA) | `brake_cur` | A | `send_brake` **신규** |
| duty | PKT_SET_DUTY(0) `>i` (duty×1e5) | `duty` | -1~1 | `send_duty` **신규** |

`send_brake`/`send_duty` 는 `ak_control.AK40` 에 추가한다 (현재 미구현 — 데모의
`send_brake` 호출은 깨진 상태이므로 동시에 수정됨).

### 4.1.1 capabilities 조각 (구체)

```
commands["ak"]      = ["set_mode", "set_input", "set_param", "set_origin", "estop"]
control_modes["ak"] = ["position", "velocity", "brake", "duty"]
inputs["ak"]        = {position:{key:"pos_deg",unit:"°"}, velocity:{key:"rpm",unit:"RPM"},
                       brake:{key:"brake_cur",unit:"A"}, duty:{key:"duty",unit:""}}
tunables["ak"]      = [{op:"set_param",key:"spd_erpm"}, {op:"set_param",key:"acc_erpm_s2"},
                       {op:"set_param",key:"max_cur_a"}]
limits["ak"]        = {pos_deg:..., rpm:..., brake_cur:..., duty:1.0}
```

`set_param` 은 `commands["ak"]` 에 포함되어야 `commands.py:normalize` 검증을 통과한다.
`set_input` 의 타깃값은 `limits["ak"]` 로 클램프된다 (기존 normalize 로직 재사용).

### 4.2 명령 동작

- `set_mode(m)`: 활성 모드 저장 + 안전 중립 송신 (position→현재위치 hold, velocity→rpm0,
  brake/duty→0). 모드 전환 시 점프 방지.
- `set_input(target)`: 활성 명령 `(send_fn, args)` 갱신.
- `set_param`: `spd_erpm`/`acc_erpm_s2`/`max_cur_a` 저장 (튜닝 패널용).
- `set_origin`: `set_origin_here()`.
- `estop`: `stop()` (rpm0 반복).

### 4.3 워치독

`tick()` 이 활성 명령을 **~20Hz 로 스로틀**해 재전송 (100Hz 그대로면 버스 폭주).
idle/estop 후엔 rpm0 keepalive 로 텔레메트리 유지.

### 4.4 신호 (텔레메트리)

| 키 | 의미 | 단위 |
|---|---|---|
| `ak.pos_deg` | 출력축 위치 | ° |
| `ak.speed` | 출력축 속도 = `spd_erpm / (POLE_PAIRS×GEAR_RATIO)` | RPM |
| `ak.current` | 전류 | A |
| `ak.temp` | 온도 | ℃ |
| `ak.fault` | fault 코드 | (enum) |

`ak.speed` 는 명령 단위(출력축 RPM)와 맞추기 위해 erpm 에서 변환한다.

### 4.5 안전: 과전류 자동정지

`sample()`/`tick()` 에서 `|cur_a| > max_cur_a` 이면 rpm0 송신 + 활성명령 해제 +
fault 로깅. `move_rel_out` 의 abort 로직 계승.

## 5. 데이터 흐름

```
Worker(100Hz) → CanTransport.sample() [request→recv드레인→on_rx분배→tick→sample병합] → WS → 프론트
프론트 → POST /api/command → worker.submit → CanTransport.apply → target 라우팅 → device.apply → 프레임 → ack
```

프론트는 capabilities 기반 자동 렌더 (모드선택·타깃입력·튜닝패널 그대로 동작).

## 6. 에러 처리

- can0 open 실패 → `TransportError("... bash scripts/can_setup.sh 먼저")`.
- AK 무응답 → 마지막 캐시값 유지. 송신 실패는 `AK40._safe_send` 가 흡수.
- 과전류 → 자동정지 (4.5).
- estop → 전 디바이스 stop.
- **AK fault 디코드 (프론트)**: VESC `mc_fault_code` **enum 값 매핑** (비트필드 아님):
  0 NONE, 1 OVER_VOLTAGE, 2 UNDER_VOLTAGE, 3 DRV, 4 ABS_OVER_CURRENT,
  5 OVER_TEMP_FET, 6 OVER_TEMP_MOTOR, 7 GATE_DRIVER_OVER_VOLTAGE,
  8 GATE_DRIVER_UNDER_VOLTAGE, 9 MCU_UNDER_VOLTAGE, 10 BOOTING_FROM_WATCHDOG_RESET,
  11 ENCODER_SPI, 12/13 ENCODER_SINCOS, 14 FLASH_CORRUPTION, 18 UNBALANCED_CURRENTS.
  `app.js` 에 `AK_FAULT_CODES` 표 + `monitorSample` 의 ak.fault 디코드 추가.

## 7. 프론트엔드 영향

데이터-드리븐이라 4모드/타깃입력/튜닝패널/플롯은 capabilities 만으로 자동 렌더된다.
**유일한 추가 = AK fault 코드 디코드** (6절). 기존 ODrive 그래프 오버레이/모터정보
패널 등 ODrive 전용 로직은 AK 트랙에서 자연히 비활성(신호 부재 → `has()` 필터).

## 8. 파일 구조

```
motor_gui/backend/transport/
  can_device.py     신규 — CanDevice ABC + CanTransport
  ak_device.py      신규 — AkDevice (AK40 래핑, 워치독, capabilities 조각)
  can_bus.py        보존 (기존 CanBackend — 3·4단계에서 OdriveCanDevice 추출 원본)
motor_control/steering/
  ak_control.py     수정 — send_brake / send_duty 추가
motor_gui/backend/
  server.py         수정 — --track ak → CanTransport([AkDevice()])
motor_gui/frontend/
  app.js            수정 — AK_FAULT_CODES + ak.fault 디코드
```

## 9. 테스트

- **단위(무 HW)**:
  - `AkDevice` 프레임 인코딩: pos/rpm/brake/duty 의 arbitration_id + data 바이트가
    VESC 스펙과 일치하는지 (스텁 bus 로 송신 캡처).
  - `CanTransport`: target 라우팅, sample 병합, capabilities 병합 (스텁 디바이스).
- **HIL (실 AK, can0 연결됨)**:
  - `--track ak` 서버 → 브라우저: 4모드(위치 deg 이동 / 속도 rpm / 브레이크 / duty),
    영점, 튜닝(spd/accel/maxcur), 과전류 자동정지, fault 디코드, CSV 로깅.
  - 실행 환경: Jetson `powertrain_jetson` 컨테이너 (network_mode host → can0 공유),
    사전 `bash scripts/can_setup.sh` (1Mbps).

## 10. 범위 밖 (Out of scope)

- `OdriveCanDevice` 구현 (3단계, 다음 spec) — 본 spec 은 인터페이스 적합성만 확정.
- ODrive-USB 트랙 변경 (1단계 완료, 무관).
- 기존 `CanBackend` 삭제 — 3·4단계 추출 전까지 보존.
