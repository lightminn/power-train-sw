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

# USB 트랙: 읽기 정본으로 확인한 실제 serial/axis/node를 명시한다.
python3 -m motor_gui.backend.server --track usb --usb-serial <SERIAL> --usb-axis 1 --usb-node 12

# CAN 트랙 (ODrive+AK, can0) — 먼저 bash scripts/can_setup.sh
python3 -m motor_gui.backend.server --track can
```

노트북 브라우저에서 `http://jetson-orin.local:8000` 접속.
(컨테이너가 `network_mode: host` 라 포트 매핑 불필요.)

## 트랙

| 트랙 | 전송 | 장치 | NVM 저장 |
| --- | --- | --- | --- |
| `usb` | odrive lib | 명시한 serial의 axis0 또는 axis1 | 양축 IDLE에서만 |
| `can` | python-can can0 | ODrive(node11) + AK(id1) 동시 | X (USB 전용) |
| `fake` | 시뮬 | odrive+ak 슈퍼셋 | (noop) |

## CAN·USB 소유권과 정비

CAN과 USB는 같은 모터를 제어하므로 `/run/powertrain/can0.lock`을 공유한다.
차체가 운용 중이면 USB GUI·정비 도구도 연결 전에 거부된다. USB만 사용할 때도
호스트에서 `sudo bash scripts/install_powertrain_runtime_dir.sh`로 런타임 디렉터리를
먼저 준비한다. CAN 인터페이스는 USB 때문에 생성하거나 재설정하지 않는다.

GUI의 CAN ID 변경은 기존 대상 정지 명령의 송신 ACK가 성공한 뒤 재연결하며,
실패하면 기존 ID를 유지한다. 이 ACK 자체는 실물 정지 확인을 대신하지 않는다.
CAN 조회는 15 Hz로 제한하고 AK·ODrive 수신은 한 곳에서 분배한다.
fw-v0.5.6 CAN은 FET 온도를 지원하지 않아 온도 그래프를 만들지 않고 미지원 안내를 표시한다.

NVM 저장은 GUI에서 모든 장치를 disarm한 뒤, 실제 보드 양축 IDLE일 때만 허용한다.
저장 전후 동일 serial·양축 CAN 통신 설정을 비교하며, 불일치는 성공으로 표시하지 않는다.
USB 정본 CLI 예시(컨테이너 `/workspace`, `<SERIAL>`은 실제 확인값으로 대체):

```bash
python3 motor_control/drive/bl70200/bl70200_setup.py --read --serial <SERIAL>
python3 motor_control/drive/bl70200/bl70200_setup.py --apply --serial <SERIAL> --axis both --node 11
python3 motor_control/drive/bl70200/bl70200_setup.py --persist-calibration --serial <SERIAL> --axis both --node 11
```

`--node`는 첫 선택축의 node이며 `--axis both`는 다음 축에 `node+1`을 사용한다.
쓰기 대상 세 값이 빠지면 USB 탐색 전에 거부한다. 저장은 보드를 재부팅하므로
실물 모터 정지·벤치 준비를 확인한 정비 시에만 실행한다.

## 명령 timeout 계약

REST 명령과 프로파일 적용은 worker ACK를 최대 2초 기다린다. timeout 시점까지
worker가 요청을 시작하지 않았으면 요청을 원자적으로 취소하고
`FINAL_REJECTED`를 반환한다. 이 요청은 나중에도 하드웨어에 적용되지 않는다.
이미 worker가 실행을 시작했다면 중단 가능 여부를 추측하지 않고
`OUTCOME_UNKNOWN`을 반환한다. 이 경우 텔레메트리로 실제 상태를 확인하기 전에는
같은 명령을 자동 재시도하지 않는다. E-stop REST 경로는 이 큐를 거치지 않고 즉시
worker의 안전 래치를 세운다.

## 테스트

dev 컨테이너(x86) 안에서:
```bash
docker compose -f docker/docker-compose.yml exec -T powertrain bash -lc "cd /workspace && python3 -m pytest motor_gui/tests/ -q"
```

브라우저 없이 실제 `app.js`의 timeout ACK 표시를 Node VM에서 검사하려면:

```bash
node motor_gui/tests/frontend_timeout_ack_test.mjs
```

같은 검사는 전체 pytest에도 연결돼 있다. 모의 HTTP 응답으로 실제 `app.js`의 명령·프로파일 처리 함수를 실행하며,
Node.js가 없는 환경에서는 명시적인 사유와 함께 해당 1건만 skip한다.

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
