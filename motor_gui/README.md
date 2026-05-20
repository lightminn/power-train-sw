# motor_gui — 모터 통합 관제 GUI (벤치 진단 도구)

Jetson 에서 실행, 노트북 브라우저로 접속하는 웹 기반 모터 진단·튜닝 도구.
ODrive(USB/CAN)·AK(CAN) 의 100 Hz 텔레메트리를 uPlot 으로 실시간 plot,
위치/속도/토크 제어·라이브 게인 튜닝·캘리·E-stop 수행.

설계: `docs/specs/2026-05-20-motor-gui-design.md`
계획: `docs/plans/2026-05-20-motor-gui-plan.md`

## 실행

Jetson 컨테이너 안에서 (`docker compose -f docker/docker-compose.jetson.yml exec powertrain bash`):

```bash
cd /workspace

# fake (하드웨어 없이 — 개발/데모)
python3 -m motor_gui.backend.server --track fake

# USB 트랙 (ODrive USB 연결)
python3 -m motor_gui.backend.server --track usb

# CAN 트랙 (ODrive+AK, can0) — 먼저 bash scripts/can_setup.sh
python3 -m motor_gui.backend.server --track can
```

노트북 브라우저에서 `http://jetson-orin.local:8000` 접속.
(컨테이너가 `network_mode: host` 라 포트 매핑 불필요.)

## 트랙

| 트랙 | 전송 | 장치 | NVM 저장 |
| --- | --- | --- | --- |
| `usb` | odrive lib | ODrive 1대 (axis1) | O |
| `can` | python-can can0 | ODrive(node1) + AK(id10) 동시 | X (USB 전용) |
| `fake` | 시뮬 | odrive+ak 슈퍼셋 | (noop) |

## 테스트

dev 컨테이너(x86) 안에서:
```bash
docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/ -q"
```

## 구조

```
motor_gui/
├── backend/
│   ├── transport/{base,fake,usb_odrive,can_bus}.py   # 장치 I/O (공통 Transport ABC)
│   ├── worker.py        # 100 Hz 스레드, Transport 단독 소유, 큐 + estop
│   ├── commands.py      # envelope 검증·클램프
│   ├── recorder.py      # 선택적 CSV/parquet 로깅
│   └── server.py        # FastAPI: WS 텔레메트리 + REST 제어
└── frontend/            # 바닐라 JS + uPlot (capabilities 기반 동적 UI)
```

웹↔하드웨어는 JSON dict seam 으로 분리 (worker.submit/subscribe). 향후 하드웨어
프로세스 격리(접근법 C) 시 server 무수정. `can_bus.py` 는 `motor_control/steering/
ak_control.py` 의 AK 클래스를 재사용 (hw 로직 단일 소스).
