# motor_control 재구성 + Jetson 독자 작업분 통합

날짜: 2026-05-20
저장소: https://github.com/lightminn/power-train-sw (branch `main`)
관련: [HANDOFF.md], `2026-05-10-vision-motor-integration-design.md`

---

## 배경 / 문제

5/10 ~ 5/17 사이 사용자가 Jetson Orin Nano 의 로컬 작업트리에서 다음 작업을
독자 진행 (노트북 repo 에는 미반영):

- ODrive 통신을 USB ↔ CAN bus 양쪽 지원 (`odrive_can_setup.py`, `odrive_can_drive.py`)
- 조향용 서보 추가: CubeMars AK40-10 (테스트) / AK45 (실전) — CAN, 동일 API
  (`ak40_control.py` 메인 + `calibrate_ak40.py` · `status_ak40.py` · `run_ak40.py`)
- 거리 센서 US100 (UART /dev/ttyTHS1) 검증 (`uart_test.py`, `us100_test.py`)
- can0 1 Mbps 셋업 스크립트 (`~/can_setup.sh`)
- `docker/Dockerfile.jetson` + `docker/docker-compose.jetson.yml` 수정
  (`python-can` pip, `/dev/ttyTHS1` device 패스)

동시에 기존 README/CLAUDE.md/HANDOFF 에 박혀있던 모터 인벤토리가 사실과 다름:

| 항목 | 기존 (잘못) | 실제 |
| --- | --- | --- |
| 구동 모터 (실전) | D6374 | **BL70200** + 내장 HALL ×3 (pp=5, cpr=30) |
| 구동 모터 (테스트) | D6374 (HALL 트랙) | **SunnySky X2212-13** + TLE5012B 16384 CPR |
| 조향 모터 | 없음 | **AK40-10 / AK45** (CAN, 동일 API) |

기존 HALL 트랙 코드의 캘리값 (pp=5, cpr=30) 은 우연히 BL70200 과 일치 — 파일명 정정만
하면 그대로 운영 가능.

`motor_control/` 루트가 평면적이라 새 hw 라인 (조향, 센서, CAN 통신) 추가 시
어느 모터 / 통신 / 노드 가정인지 파일명만으로 판단 불가. 트랙 정리 필요.

---

## 결정 사항

### 분류 기준: 모터 hw 1차, 보조 카테고리 (vision/sensors/network) 2차

| 폴더 | 가정 hw | 통신 | 노드 |
| --- | --- | --- | --- |
| `drive/x2212_test/` | SunnySky X2212-13 + TLE5012B | ODrive USB · CAN 양쪽 | Jetson 또는 x86 |
| `drive/bl70200/` | BL70200 + 내장 HALL ×3 | ODrive USB (현재) | Jetson 또는 x86 |
| `steering/` | AK40-10 / AK45 | CAN (socketcan can0) | Jetson |
| `vision/` | 모터 명령 X | — | Jetson 또는 x86 |
| `sensors/` | US100 (UART) | UART `/dev/ttyTHS1` | Jetson |
| `laptop/` (기존) | — | TCP `:9000` | 노트북 |
| `pi/` (기존) | — | TCP `:9000`, GStreamer `:5000` | Pi |

### 파일 매핑

```
[이동 — git mv]
motor_control/init_odrive.py                      → drive/x2212_test/init_odrive.py
motor_control/odrive_dualsense_test.py            → drive/x2212_test/odrive_dualsense_test.py
motor_control/odrive_dualsense_vel_test.py        → drive/x2212_test/odrive_dualsense_vel_test.py
motor_control/yolo_odrive_jetson.py               → drive/x2212_test/yolo_odrive_jetson.py
motor_control/yolo_odrive_motor_test.py           → drive/x2212_test/yolo_odrive_motor_test.py
motor_control/odrive_yolo_object_tracking.py      → drive/x2212_test/odrive_yolo_object_tracking.py
motor_control/odrive_calibration.py               → drive/bl70200/odrive_calibration.py
motor_control/odrive_basic_test.py                → drive/bl70200/odrive_basic_test.py
motor_control/odrive_closed_loop_test.py          → drive/bl70200/odrive_closed_loop_test.py
motor_control/odrive_diff_drive_test.py           → drive/bl70200/odrive_diff_drive_test.py
motor_control/odrive_position_hold_test.py        → drive/bl70200/odrive_position_hold_test.py
motor_control/odrive_velocity_hold_test.py        → drive/bl70200/odrive_velocity_hold_test.py
motor_control/yolo_openvino_detection.py          → vision/yolo_openvino_detection.py
motor_control/yolo_cuda_stream.py                 → vision/yolo_cuda_stream.py
motor_control/setup_yolo_env.sh                   → vision/setup_yolo_env.sh

[새 파일 — Jetson 에서 회수 + 신규 위치 배치]
(Jetson) motor_control/odrive_can_setup.py        → drive/x2212_test/odrive_can_setup.py
(Jetson) motor_control/odrive_can_drive.py        → drive/x2212_test/odrive_can_drive.py
(Jetson) motor_control/ak40_control.py            → steering/ak_control.py        (rename)
(Jetson) motor_control/calibrate_ak40.py          → steering/calibrate_ak.py      (rename)
(Jetson) motor_control/status_ak40.py             → steering/status_ak.py         (rename)
(Jetson) motor_control/run_ak40.py                → steering/run_ak.py            (rename)
(Jetson) us100_test.py                            → sensors/us100_basic.py        (rename)
(Jetson) uart_test.py                             → sensors/us100_robust.py       (rename)
(Jetson) ~/can_setup.sh                           → scripts/can_setup.sh

[Docker 수정분 — 회수]
(Jetson) docker/Dockerfile.jetson                 (python-can pip 추가, 들여쓰기)
(Jetson) docker/docker-compose.jetson.yml         (/dev/ttyTHS1 device 마운트)
```

### gitignore 추가

```
yolov8n.pt
yolov8n*.onnx
yolov8n*_*.engine
.*.kate-swp
```

### 문서 갱신

- `README.md`: 트랙 분류표 재작성 (drive/x2212_test, drive/bl70200, steering, vision, sensors,
  laptop, pi). D6374 → BL70200 교정. SunnySky X2212-13 / AK40-10 + AK45 인벤토리 명시.
  실행 절차 (캘리 → 단축 검증 → 통합) 새 경로로.
- `.claude/CLAUDE.md`: Directory Layout 갱신, Working in motor_control 인벤토리 새 경로,
  Robot Specification 의 D6374 항목 제거.
- `HANDOFF.md`: 새 구조 반영 + 모터 인벤토리 정정 (있는 그대로 후속 세션이 받게).

---

## 실행 절차

5 단계로 처리 — 각 단계는 다음 단계 시작 전에 검증.

1. **회수**: 노트북에서 `rsync` 로 Jetson 의 새 파일 8개 + 수정 Docker 파일 2개를 가져옴.
   기존 위치 (motor_control/ 루트, 레포 루트) 에 untracked / modified 로 적재.
2. **이동·rename**: 노트북에서 신규 폴더 `drive/{x2212_test,bl70200}`, `steering/`,
   `vision/`, `sensors/` 생성. `git mv` 로 기존 추적 파일 이동, 새 파일은 `git add` + rename 후 위치.
3. **문서 갱신**: README, CLAUDE.md, HANDOFF, .gitignore 동기화. D6374 표현 grep 으로 전수 교정.
4. **commit + push**: 노트북에서 단일 commit 으로. 메시지에 hw 인벤토리 정정 명시.
5. **Jetson 동기화**: Jetson 측 `git stash -u` (untracked + modified 한꺼번에 보관) → `git pull origin main`
   → stash 폐기 (`git stash drop`) → 새 경로에서 import dry-run 검증.

각 단계 끝에 `git status` 검증. 5단계의 실행 검증은 모터·CAN 통전 없는 dry-import 만.

---

## 검증 기준

- `git ls-files motor_control/ steering/ vision/ sensors/ scripts/` 가 디자인의 매핑과 일치.
- `git grep -ni "D6374"` 결과가 historical document (HANDOFF 의 과거 commit 메시지 등) 외에 없음.
- Jetson 에서 `git pull` 후 working tree clean.
- 노트북 `cd vision && python -c "import yolo_cuda_stream"` 등 import dry-run 통과 (Jetson 에서 동일).

---

## Open items (이번 작업 scope 밖)

- BL70200 실차 부착 시 vbus 24 V 환경에서 `odrive_calibration.py` 의 `current_lim` /
  `vel_limit` 등 NVM 게인 재튜닝 필요할 수 있음.
- AK45 도착 시 hw 동작 확인 — `steering/ak_control.py` 의 `GEAR_RATIO`/`POLE_PAIRS` 값 재검증.
- CAN bus 충돌 검토 — 현재 ODrive node_id=1, AK40 motor_id=10. 향후 다축 ODrive 추가 시 ID 맵 정의.
- `pi/` 트랙이 실제 hw 라인으로 살아있는지 / Jetson 통합 후 deprecate 인지 미정 — 이번 작업에서는 유지.
- 5/10 HANDOFF 의 "마일스톤 5건" (실차 부착, DualSense+비전 토글, 다축, NVENC, INT8) 은 별도 spec 로.
