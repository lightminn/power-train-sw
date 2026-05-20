# 모터 통합 관제 GUI (벤치 진단 도구) — 설계

날짜: 2026-05-20
저장소: https://github.com/lightminn/power-train-sw (branch `main`)
관련: `2026-05-20-motor-control-reorg-design.md`, [HANDOFF.md]

---

## 배경 / 목적

ODrive·AK 모터를 **CLI 로 굴리는 것 자체는 이미 충분**하다. 이 도구를 만드는 진짜
이유는 **실시간 모터 반응(위치·속도·전류·온도)과 파라미터 변화를 plot 으로 한눈에
보면서** 캘리·게인 튜닝·수동 지령을 수행하는 **벤치 브링업·진단 환경**이 필요해서다.

- **역할**: 벤치 브링업·진단 도구 (개발 중 엔지니어 사용). 실차 운영 대시보드 아님.
- **모니터링이 1차 가치**, 제어는 2차. 정밀 실시간 텔레메트리 + plot 중심.

## 범위 (트랙 완전 분리)

USB 트랙과 CAN 트랙은 **완전히 분리** 운영. 런처에서 한 트랙만 기동.

| 트랙 | 전송 | 장치 | NVM 저장 |
| --- | --- | --- | --- |
| **USB** | `odrive` lib | ODrive 구동 모터 1대 (`axis1`) | 가능 (`save_configuration`) |
| **CAN** | `python-can` socketcan `can0` | ODrive(node_id=1) + AK 조향(motor_id=10) 동시 | **불가** (CANSimple 에 명령 없음 → USB 트랙에서만) |

- 실행 위치: **Jetson Orin Nano** (헤드리스). 노트북 브라우저로 원격 접속.
- 범위 밖: US100 등 센서, 실차 다중 노드 운영, 인증/멀티유저 권한.

## 제어 액션 (USB·CAN 공통, capabilities 로 트랙별 노출 조정)

- 제어 모드: **위치 / 속도 / 토크**
- 입력 모드: PASSTHROUGH / POS_FILTER / VEL_RAMP / TRAP_TRAJ (ODrive)
- 수동 지령: 위치 / 속도 / 토크 (슬라이더·수치)
- 게인·한계 라이브 튜닝: `pos_gain`, `vel_gain`, `vel_integrator_gain`, `current_lim`, `vel_limit`, `input_filter_bandwidth`
- 캘리브레이션: FULL_CALIBRATION_SEQUENCE 트리거, (USB) `pre_calibrated` + `save_configuration` + reboot
- 폐루프 진입/IDLE, 에러 클리어·덤프, **E-stop**
- AK (CAN): 위치(SET_POS_SPD)·속도(SET_RPM)·토크(SET_CURRENT)·브레이크·원점(SET_ORIGIN)

## 텔레메트리

- **목표 100 Hz** 실시간 plot. CAN 은 ODrive cyclic 브로드캐스트(`encoder_rate_ms=10` 등)로
  폴링 없이 스트림; USB 는 폴링 최대치.
- 신호 (네임스페이스 `<device>.<signal>`, 항상 `t_mono` 포함):
  - ODrive: `pos`, `vel`, `iq_meas`, `iq_set`, `temp_fet`, `vbus`, `ibus`, `state`,
    `axis_err`, `motor_err`, `enc_err`, `ctrl_err`, `vel_integrator`
  - AK: `pos_deg`, `speed`, `current`, `temp`, `fault`
- **파일 로깅: 선택(토글, 기본 off)**. CSV 또는 parquet. 활성 트랙 신호명이 헤더.

## 하드웨어 기능 조사 결과 (fw-v0.5.6 / CubeMars AK servo)

> 라이브 property 트리 열거는 하드웨어 power-on 시 검증 예정. 아래는 프로토콜 스펙 +
> 기존 코드(`odrive_can_drive.py`, `ak_control.py`) 기반.

### ODrive CANSimple cmd (CAN ID = `(node_id<<5)|cmd`)
- 텔레메트리: Heartbeat `0x001`(상태+에러), Get_Encoder_Estimates `0x009`(pos,vel),
  Get_Iq `0x014`, Get_Temperature `0x015`, Get_Bus_Voltage_Current `0x017`,
  Get_*_Error `0x003~5`, Get_Controller_Error `0x01D`
- 제어: Set_Axis_State `0x007`, Set_Controller_Mode `0x00B`, Set_Input_Pos `0x00C`,
  Set_Input_Vel `0x00D`, **Set_Input_Torque `0x00E`**, Set_Limits `0x00F`,
  Set_Pos_Gain `0x01A`, Set_Vel_Gains `0x01B`, Clear_Errors `0x018`, **Estop `0x002`**, Reboot `0x016`
- **NVM 저장 명령 없음** → USB 전용. 게인 cyclic 주기: `heartbeat_rate_ms`, `encoder_rate_ms`.

### CubeMars AK servo (ext ID = `(packet<<8)|motor_id`)
- 제어: SET_DUTY `0`, SET_CURRENT `1`(=토크), SET_CURRENT_BRAKE `2`, SET_RPM `3`,
  SET_POS `4`, SET_ORIGIN `5`, SET_POS_SPD `6`
- 상태(STATUS `0x29`): pos(0.1°), speed(10 ERPM), current(0.01 A≈토크), temp(°C), fault
- MIT 모드(임피던스)도 존재하나 servo 모드 유지 (기존 코드 호환).

---

## 아키텍처

### 3계층 + 단일 직렬화 경계 (C-승격 대비)

```
[브라우저: 바닐라 JS + uPlot]
   │  WS /ws/telemetry (push)   │  POST /api/command · GET /api/capabilities
[웹 레이어: server.py (FastAPI + uvicorn)]
   │  worker.submit(cmd:dict) / worker.subscribe() → sample(dict)   ← 유일한 인터페이스
[HardwareWorker]   # A: 같은 프로세스 스레드 / C(향후): 원격 프로세스 프록시
   │  transport.sample()/apply()  (plain JSON-able dict)            ← seam
[Transport: usb_odrive | can_bus | fake]
   │
[하드웨어: ODrive USB / can0]
```

**동시성 (접근법 A)**: `HardwareWorker` 가 Transport 를 단독 소유. 백그라운드 스레드 1개가
100 Hz 로 `sample()` + command 큐 drain → `apply()`. 샘플과 명령이 **같은 스레드**라
Transport 동시접근 0. 웹 레이어(asyncio)는 큐 2개(`sample_bus`, `command_in`)로만 통신.

**C-승격 seam**: `sample()`/`apply()`/`submit()` 가 **JSON-직렬화 가능한 dict 만** 교환.
향후 `HardwareWorker` 를 "하드웨어 프로세스에 unix 소켓으로 붙는 프록시"로 교체하면
`server.py` 무수정 (HardwareWorker 만 import). Transport 는 하드웨어 프로세스로 이사.
변경이 HardwareWorker 내부 + 런처에 국한.

### 모듈 구조

```
motor_control/gui/
├── backend/
│   ├── transport/
│   │   ├── base.py          # Transport ABC + 신호 네임스페이스 규약
│   │   ├── usb_odrive.py     # UsbOdriveBackend (odrive lib, axis1, .value enum)
│   │   ├── can_bus.py        # CanBackend (ODrive node1 + AK id10, ak_control.AK 재사용)
│   │   └── fake.py           # FakeTransport (시뮬 모터 — 하드웨어 없이 개발/테스트/데모)
│   ├── worker.py             # HardwareWorker (Transport 소유 + 100Hz 루프 + 큐)
│   ├── commands.py           # envelope 검증·클램프·디스패치 (순수함수 위주)
│   ├── recorder.py           # 선택적 CSV/parquet 로깅 (sample_bus tap)
│   └── server.py             # FastAPI: WS telemetry + REST command/capabilities/record
├── frontend/
│   ├── index.html
│   ├── app.js                # capabilities→UI 렌더, WS→링버퍼, 컨트롤→command
│   └── plots.js              # uPlot 패널 (pos/vel/iq/temp/vbus ...)
└── README.md                 # 실행법
```

### 인터페이스 계약

```python
class Transport(ABC):
    name: str                          # "usb" | "can" | "fake"
    def connect(self) -> None: ...
    def sample(self) -> dict: ...      # 1프레임, flat JSON-able, t_mono 포함
    def apply(self, cmd: dict) -> dict # 명령 적용 + ack(JSON-able)
    def capabilities(self) -> dict: ...# {devices, signals, commands, notes}
    def close(self) -> None: ...
```

**명령 envelope**:
```json
{"target":"odrive|ak", "op":"set_mode|set_input|set_gain|set_limit|calibrate|clear_errors|estop|save_nvm|set_state", "args":{...}}
```

**capabilities 예**:
- USB: `{devices:[odrive], commands:[..., "save_nvm"], signals:[odrive.*]}`
- CAN: `{devices:[odrive, ak], commands:[... no save_nvm, "estop"], signals:[odrive.*, ak.*]}`

프론트는 capabilities 로 패널·버튼 동적 렌더 → "NVM 저장 USB 만", "AK 패널 CAN 만" 자동 처리.

### 서버 엔드포인트
- `GET /` 정적 프론트
- `GET /api/capabilities`
- `WS /ws/telemetry` — sample_bus fan-out (~100 Hz, 브라우저가 못 따라가면 coalesce)
- `POST /api/command` — envelope submit → ack
- `POST /api/record/start` (path, fmt) / `POST /api/record/stop`
- 런처: `python -m motor_control.gui.backend.server --track usb|can|fake [--port 8000]`

---

## 데이터 흐름

- **텔레메트리(단방향 push)**: 하드웨어 → `sample()`[100Hz worker] → 링버퍼+sample_bus →
  WS fan-out → 브라우저 링버퍼(rolling 10~30 s) → uPlot. 백프레셔 시 server coalesce.
- **명령(요청-응답)**: 위젯 → envelope → `/api/command` → `worker.submit` → command 큐 →
  worker 스레드 `apply()` → ack. 적용 직후 다음 sample 에 변화 잡혀 plot 으로 즉시 확인.
- **로깅**: recorder 가 sample_bus tap → 별도 스레드 버퍼링 flush (디스플레이 경로 독립).

---

## 에러 처리 & 안전

- `connect()` 실패 / `can0` down → 명확한 에러 (+ `scripts/can_setup.sh` 안내), worker 미기동.
- 런타임 트립(axis.error≠0 / heartbeat err) → 에러 필드 그대로 스트림, UI 강조,
  **자동복구 안 함** (엔지니어 판단). 트립 순간 plot 관찰 가능.
- **E-stop**: 항상 가능한 최우선 경로 — command 큐를 건너뛰는 전용 플래그(매 루프 체크).
  CAN=Estop `0x02`(래치, clear 로 해제), USB=`requested_state=IDLE`, AK=`rpm 0`+brake.
- 명령 안전: 적용 전 pos/vel/torque 한계 클램프, capabilities 외 op 거부,
  **폐루프 진입 시 `input=현재값`** 으로 점프 방지.
- WS 끊김 → 브라우저 자동 재연결, worker 계속. worker 스레드 크래시 → catch+로그+UI 통지
  (A 의 한계, C 승격이 격리). 프로세스 종료 → 모터 IDLE/stop + 버스 close.

---

## 테스트

- **`FakeTransport`** (핵심): 명령에 반응하는 시뮬 모터. 같은 ABC → 하드웨어 없이 백엔드+프론트
  전체 개발·테스트. `--track fake` 로 데모/오프라인 모드 겸용.
- `commands.py`: 순수 단위테스트 (검증·클램프·미지원 op 거부).
- `worker.py`: FakeTransport 로 샘플 루프 주기·명령 적용·큐·**estop 우선순위** 검증.
- `server.py`: FastAPI TestClient + WS 테스트 — capabilities 스키마, 텔레메트리 프레임,
  명령 ack, record start/stop.
- HIL(수동): Jetson 실하드웨어 스모크 체크리스트 (connect→calibrate→closed-loop→jog→
  게인변경 plot 반영→estop). 문서화.
- 프론트: 최소 — 수동 + canned WS 스트림 uPlot 렌더 스모크.

---

## 의존성 / 환경

- 백엔드: `fastapi`, `uvicorn`, `websockets` (또는 fastapi 내장), 기존 `odrive`·`python-can`.
  Dockerfile.jetson 에 추가 (`pip install fastapi uvicorn`).
- 프론트: 빌드스텝 없음. `uplot` (단일 .js/.css, 정적 동봉).
- 실행: Jetson Docker 컨테이너 안 `uvicorn`, `network_mode: host` 라 노트북에서
  `http://jetson-orin.local:8000` 접속. CAN 트랙 전 `bash scripts/can_setup.sh`.

---

## Open items (이번 scope 밖 / 후속)

- **C 승격(프로세스 격리)**: seam 만 준비, 실제 IPC 구현은 별도 plan (하드웨어 행 빈발 시).
- 라이브 ODrive property 트리 열거 — 하드웨어 power-on 후 신호 목록 최종 확정.
- AK MIT 모드(임피던스 제어) 지원 — servo 모드로 충분하면 보류.
- 게인 프리셋 저장/불러오기(JSON) — 1차 범위 후 추가 검토.
- 인증/멀티유저 — 벤치 단일 사용자 전제, 도입 안 함.
- BL70200 트랙(HALL) 전용 캘리 파라미터 GUI 노출 — USB 트랙 capabilities 에서 모터별 분기.
