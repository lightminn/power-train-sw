# ODrive-CAN GUI 설계 (motor_gui 3단계)

**작성일**: 2026-05-23
**상태**: 구현·HIL 완료 (main 병합)
**관련**: `docs/specs/2026-05-21-ak-can-gui-design.md` (컴포저블 CAN 구조의 원형)

## HIL 검증 결과 (2026-05-23, 실 ODrive·1Mbps)

실하드웨어 검증에서 설계 대비 다음을 발견·수정했다 (구현에 반영됨):

1. **1Mbps 는 CAN 종단저항(120Ω ×2) 필수.** 종단 미흡 시 비트에러로 tx-error 가
   누적→ERROR-PASSIVE→bus-off, 텔레메트리 프리즈/명령 유실. 종단 정상화 후 tx-error 0
   안정. (250kbps 는 비트타임 4× 여유라 마진 배선에서도 동작 — 검증용 폴백.)
2. **호스트 TX 최소화 필수.** 100Hz×4 RTR(=400/s) 폴링이 1M 마진버스에서 bus-off 유발 →
   `request()` 를 `_POLL_HZ`(15Hz)로 throttle(=54/s), `_request()` 가 CanError/ENOBUFS 흡수,
   `can_setup.sh` 에 restart-ms 100 + txqueuelen 1000 추가. 다중 모터(10개) 1M 대비.
3. **CAN `Set_Linear_Count`(0x019)는 절대엔코더(TLE5012B) zero 불가** (raw 로도 확인).
   네이티브 영점 폐기 → 소프트 오프셋(`_pos_offset`, 모놀리식 can_bus.py 방식) 회귀.
   set_origin 시 현재 raw 를 offset 으로 잡아 sample 의 pos 에서 차감, set_input pos 엔 가산.
4. **ODrive 캘리/baud/node 설정은 USB 필요** (CANSimple 로 config-write 불가). 구동은 CAN-only.
   기본 baud 250000 → 1M 은 USB `odrv0.can.set_baud_rate(1000000)`+`save_configuration()`.
5. **알려진 한계:** position_traj + 극저 vel_limit(예 1.0) + 높은 pos_gain(8.0) 조합에서
   위치보정 항이 좁은 하드캡(vl×1.3)을 넘겨 overspeed(axis_err 0x200) 트립. **TRAP 에선
   vel_limit ≥ 3 권장** (그 이상은 캡 여유 충분, 정상). 정밀 저속은 position(POS_FILTER) 모드 사용.

검증 통과: 폐루프 진입·position·position_traj 사다리꼴·velocity·소프트 영점·setpoint
오버레이·Kt 편집·하드웨어 재연결·estop, 모두 1M tx-error 0 에서 정상.

## 목표

motor_gui 에 ODrive 를 **CAN(socketcan can0)** 으로 제어하는 트랙을 추가한다.
USB 트랙(`UsbOdriveBackend`)과 동일한 진단 GUI 경험을 CAN 위에서 제공하되,
CANSimple 프로토콜 한계 안에서 가능한 것만 노출한다. AK-CAN 에서 확립한
컴포저블 디바이스 구조(`CanDevice`/`CanTransport`)를 그대로 재사용한다.

이 작업은 4단계 로드맵의 3단계다:
1. ODrive-USB ✅ (`--track usb`)
2. AK-CAN ✅ (`--track ak`, `CanTransport([AkDevice()])`)
3. **ODrive-CAN** ← 이 문서 (`--track odrive_can`, `CanTransport([OdriveCanDevice()])`)
4. ODrive-CAN + AK-CAN 동시 (`CanTransport([OdriveCanDevice(), AkDevice()])`)

## 배경: 연결 검증 (2026-05-23 HIL)

ODrive 가 can0(1Mbps)에 정상 연결됨을 확인:
- can0 UP, ERROR-ACTIVE, berr tx/rx 0 (정상)
- node 0(axis0) + node 1(axis1) 하트비트 ~10Hz, 둘 다 state=1(IDLE), axis_error=0
- **RTR 요청 양방향 통신 검증**: node1 에 `Get_Encoder_Estimates(0x09)` RTR →
  즉시 응답(pos≈0, vel=0). 주기 텔레메트리(엔코더/Iq/온도)는 **꺼져 있어**
  RTR 폴링으로 읽어야 함.

## 아키텍처

### 신규: `OdriveCanDevice(CanDevice)`
보존된 모놀리식 `transport/can_bus.py` 의 ODrive CANSimple 로직(cmd id, decode,
apply)을 컴포저블 `CanDevice` 로 추출한다. `name="odrive"`, `node_id=1`(axis1 컨벤션).

`CanDevice` 계약 구현:
- **attach(bus)**: 버스 참조 저장, 캐시 초기화. 연결 시 기본 게인/한계(`DEFAULT_TUNABLES`)를
  ODrive RAM 에 1회 push → UI prefill 값과 실제 장치 상태 일치.
- **request(bus)**: 매 샘플마다 RTR 폴링 송신 — `Get_Encoder_Estimates(0x09)`,
  `Get_Iq(0x14)`, `Get_Temp(0x15)`, `Get_Bus_VI(0x17)`. (Heartbeat 0x01 은 수동 수신)
- **on_rx(msg)**: 11-bit 표준 ID, `node == node_id` 인 프레임만 디코드 → `_state` 캐시 갱신.
- **tick(bus)**: no-op. ODrive 는 setpoint 를 자체 유지하므로 AK 같은 워치독 재전송 불필요
  (기본 watchdog_timeout=0=비활성 전제).
- **sample()**: `_state` 스냅샷 + `torque_est = iq_meas × _torque_const`(편집 가능한 Kt)
  + 로컬 추적 setpoint(`pos_setpoint`/`vel_setpoint`).
- **apply(bus, op, args)**: 아래 명령 처리.
- **close(bus)**: `Set_Axis_State(IDLE)` 안전 정지.

### 수정: `server.py`
`_make_transport` 에 분기 추가, argparse choices 에 `odrive_can` 추가:
```python
if track == "odrive_can":
    from .transport.can_device import CanTransport
    from .transport.odrive_can_device import OdriveCanDevice
    return CanTransport([OdriveCanDevice()], track="can")
```

### 프론트엔드: 변경 없음
프론트(`app.js`/`plots.js`)는 capabilities 의 `signals`/`control_modes`/`inputs`/
`tunables`/`signal_meta` 로 데이터-드리븐 렌더링한다. ODrive 신호 패널(위치/속도
오버레이, Iq 전류, 추정토크, 온도/버스)은 이미 `has(key)` 게이트로 구현돼 있어,
`OdriveCanDevice` 가 내보내는 신호에 맞춰 자동 적응한다. (Id 신호를 안 내보내면
전류 패널은 Iq 만 그린다.)

## USB 대비 차이 (CANSimple 한계)

| 항목 | CAN 지원 | 비고 |
|------|----------|------|
| pos / vel | ✅ | `Get_Encoder_Estimates(0x09)` RTR |
| Iq 측정/목표 | ✅ | `Get_Iq(0x14)` RTR (iq_set + iq_meas) |
| **Id (자속축)** | ❌ | `Get_Iq` 에 Id 없음 → 신호 미발행, 전류 패널은 Iq 만 |
| FET 온도 | ✅ | `Get_Temp(0x15)` RTR |
| Vbus / Ibus | ✅ | `Get_Bus_VI(0x17)` RTR |
| state / axis_err | ✅ | Heartbeat(0x01) 수동 수신 |
| motor_err/enc_err/ctrl_err | ❌ | 0.5.x heartbeat 는 axis_err+state 만 (full 코드 별도 Get 필요, YAGNI) |
| setpoint 오버레이 | ✅ | CAN 엔 setpoint readback 없음 → **명령값 로컬 추적**(AkDevice 방식) |
| **추정 토크** | ✅ | `iq_meas × Kt`. **Kt 는 편집 가능 prefill 튜너블**(아래 참조) |
| 모터정보 패널(R/L/pp) | ❌ | CAN 으로 motor.config 읽기 불가 → 패널 생략(가짜값 표시 안 함) |
| 영점 | ✅ | 네이티브 `Set_Linear_Count(0x019)` IDLE-바운스 (USB 와 동일) |
| NVM 저장 | ❌ | CANSimple 에 save 명령 없음 |

## 토크 상수 Kt — 편집 가능 prefill 튜너블

CAN 으로는 `motor.config.torque_constant` 를 읽을 수 없다. 하드코딩 대신
**사용자가 변경 가능한 텍스트박스 하나**로 노출한다:
- 튜너블 항목: `{"op": "set_param", "key": "torque_constant",
  "label": "토크 상수 Kt [Nm/A]", "value": 0.0084, "help": "Iq→토크 환산용.
  기본값은 X2212-13 추정치(8.27/KV). USB 트랙 모터정보 readout 값으로 교체 가능."}`
- 기본 prefill 값: `0.0084` (X2212-13, 8.27/980KV 추정). 정확값은 USB readout 으로 확인.
- 신규 op **`set_param`** (AkDevice 의 set_param 과 동형): `torque_constant` 인자로
  `_torque_const` 갱신 → 다음 `sample()` 부터 `torque_est` 즉시 반영.
- 프론트는 `t.value` 로 prefill (기존 AK 와 동일 경로, 프론트 수정 불필요).

## 제어 모드 (3모드 — torque 제외)

| 모드 | ControlMode | InputMode(기본) | 입력 |
|------|-------------|-----------------|------|
| position | POSITION(3) | POS_FILTER(3) | pos [turn] |
| position_traj | POSITION(3) | TRAP_TRAJ(5) | pos [turn] |
| velocity | VELOCITY(2) | VEL_RAMP(2) | vel [rev/s] |

**torque 모드 제외 사유**: 무부하 runaway 를 막는 `enable_current_mode_vel_limit`
은 config 항목이라 CANSimple 로 설정 불가(USB 는 connect 시 설정). CAN 에서 토크
모드는 안전장치 없이 동작 → AK 토크/브레이크 폭주와 동일한 위험. 사용자가 USB 로
`enable_current_mode_vel_limit=True` 를 NVM 저장해둔 경우에 한해 추후 활성화 검토.

## 이식하는 HIL 픽스 (USB 트랙에서 검증됨)

1. **폐루프 진입 점프 방지**: `Set_Axis_State(CLOSED_LOOP)` 직전
   `Set_Input_Pos(현재 pos)` 발행. set_mode 가 position 계열로 바뀔 때도 현재 위치 hold.
2. **TRAP 속도캡 헤드룸**: position_traj 에서 순항속도(`Set_Traj_Vel_Limit`)는
   사용자 vel_limit, 컨트롤러 하드캡(`Set_Limits` 의 vel_limit)은 `max(vl×1.3, |현재속도|×1.3)`.
   헤드룸이 없으면 피드포워드가 캡을 다 먹어 위치오차 windup → 캡 해제 시 스파이크.
   현재 속도 아래로 안 내려 overspeed 트립도 방지. (USB `_sync_vel_limit` 와 동일 로직)
3. **trap_vel_limit UI 제거**: vel_limit 에 결합(위 헤드룸 규칙). 별도 노출 시 windup
   유발 → CAN 튜너블에서 `trap_vel_limit` 필터링(`ODRIVE_TUNABLES_CAN` 에서 추가 제거).
4. **set_limit 페어 프레임 병합**: CAN `Set_Limits`/`Set_Vel_Gains`/`Set_Traj_Accel_Limits`
   는 두 값을 한 프레임에 실음 → 부분 업데이트 시 마지막 값 캐시 병합(모놀리식 로직 유지).
5. **네이티브 영점**: `Set_Axis_State(IDLE)` → 0.2s → `Set_Linear_Count(0)` →
   `Set_Input_Pos(0)` → (원래 폐루프였으면) `Set_Axis_State(CLOSED_LOOP)`.
   `_pos_offset` 소프트 오프셋 폐기(모놀리식 대비 개선).

## 명령 (apply)

`set_mode` / `set_input` / `set_gain` / `set_limit` / `set_state` / `calibrate` /
`clear_errors` / `set_param`(Kt) / `set_origin` / `estop`. (USB 대비 `save_nvm` 없음)

## 신호 목록

```
odrive.pos, odrive.pos_setpoint, odrive.vel, odrive.vel_setpoint,
odrive.iq_meas, odrive.iq_set, odrive.torque_est,
odrive.temp_fet, odrive.vbus, odrive.ibus, odrive.state, odrive.axis_err
```
(USB 대비 제외: id_meas, id_set, motor_err, enc_err, ctrl_err, vel_integrator)

## CAN 프레임 참조 (CANSimple fw-v0.5.6, arb = (node_id<<5)|cmd)

| cmd | 이름 | 방향 | 페이로드 |
|-----|------|------|----------|
| 0x001 | Heartbeat | RX(수동) | `<I` axis_err + `B` state |
| 0x002 | Estop | TX | (없음) |
| 0x007 | Set_Axis_State | TX | `<I` state |
| 0x009 | Get_Encoder_Estimates | RTR→RX | `<ff` pos, vel |
| 0x00B | Set_Controller_Mode | TX | `<ii` control_mode, input_mode |
| 0x00C | Set_Input_Pos | TX | `<fhh` pos, vel_ff, torque_ff |
| 0x00D | Set_Input_Vel | TX | `<ff` vel, torque_ff |
| 0x00F | Set_Limits | TX | `<ff` vel_limit, current_lim |
| 0x011 | Set_Traj_Vel_Limit | TX | `<f` vel_limit |
| 0x012 | Set_Traj_Accel_Limits | TX | `<ff` accel, decel |
| 0x014 | Get_Iq | RTR→RX | `<ff` iq_set, iq_meas |
| 0x015 | Get_Temp | RTR→RX | `<ff` fet, motor |
| 0x017 | Get_Bus_VI | RTR→RX | `<ff` vbus, ibus |
| 0x018 | Clear_Errors | TX | (없음) |
| 0x019 | Set_Linear_Count | TX | `<i` count |
| 0x01A | Set_Pos_Gain | TX | `<f` pos_gain |
| 0x01B | Set_Vel_Gains | TX | `<ff` vel_gain, vel_integrator_gain |

(AXIS_IDLE=1, CLOSED_LOOP=8, FULL_CALIB=3)

## 에러 처리

- **연결 실패**: `CanTransport.connect()` 가 socketcan open 실패 시 `TransportError`
  ("can_setup.sh 먼저"). 기존 로직 그대로.
- **apply 예외**: `{"ok": False, ...}` ack (디바이스 내부 try/except).
- **미지원 op/mode**: `{"ok": False, "detail": "..."}` 거부 (모터 안 건드림).
- **CAN 버스 죽음 진단**: `ip -details link show can0` ERROR-PASSIVE + berr tx만 상승
  = ODrive ACK 안 함 = 전원/케이블 (소프트 아님). 운영 메모 참조.

## 테스트

- **신규** `tests/test_odrive_can_device.py`: 가짜 버스 주입(`FakeBus`, 송신 메시지 캡처).
  - `request()` 가 올바른 RTR arb id 4개 송신
  - `on_rx()` 가 heartbeat/enc/iq/temp/bus 프레임을 정확히 디코드(다른 node 무시)
  - `sample()` 가 `torque_est = iq×Kt` + 추적 setpoint 반환
  - `set_param(torque_constant)` 가 Kt 갱신 → torque_est 변화
  - `apply` set_mode/set_input/set_origin/set_limit 가 올바른 프레임 송신
  - 미지원 op / torque 모드 거부
- **확장** `tests/test_server.py`: `--track odrive_can` 가 `CanTransport([OdriveCanDevice])` 구성
- 기존 56 테스트 전부 유지(green)

## HIL 검증 (Jetson)

`--track odrive_can` 기동 후 실 ODrive(node1)로:
1. 텔레메트리: pos/vel/iq/온도/Vbus 가 RTR 폴링으로 갱신되는지
2. 폐루프 진입(점프 없는지) → position / position_traj / velocity 각 모드
3. 네이티브 영점(pos=0 복귀, fling 없는지)
4. TRAP 속도캡 변경 중 스파이크 없는지
5. Kt 텍스트박스 변경 → 추정토크 그래프 스케일 반영
6. 하드웨어 재연결(`/api/reconnect`)
7. estop / clear_errors

배포 워크플로: `~/orin_mount` 마운트 cp → `powertrain_jetson` 컨테이너 exec →
host-network curl. (운영 메모 jetson-deploy-can 참조)
