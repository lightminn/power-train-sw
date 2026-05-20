# HANDOFF — Defence_Robot Vision + Motor 통합 세션

날짜: 2026-05-10
저장소: https://github.com/lightminn/power-train-sw  (branch `main`)
마지막 commit: `a39f225` (push 완료, working tree clean)

이 파일은 다른 폴더 / 다른 Claude 세션이 위 저장소의 작업을 콜드 스타트로 이어받기 위한
브리핑. 기존 conversation 의 모든 결정 사항·발견·실행 명령을 self-contained 로 정리.

---

## TL;DR

**중간 목표**: 비전 (YOLOv8 CUDA/TRT) + 모터 제어 (ODrive axis1 추종) 를 Jetson Orin Nano
단일 노드에서 통합. **현재 상태: 무부하 검증 완료** (2026-05-10).

- Phase A (환경): ODrive Python lib import + USB find_any + TRT 동시 OK
- Phase B/C (추종 only): 30 fps, infer 9.8 ms, MAX_TURNS=2.0 안전 한계 / Ctrl-C IDLE 정상
- Phase D (추종 + 스트리밍): avg_fps 28.3, infer 9.8 ms, 노트북 영상 + 검출 박스 + 추종 동시 정상

**남은 큰 마일스톤** (우선순위 순):
1. **실차 차체 부착 + 노면 운영** — 토크/전류 한계 재튜닝
2. **DualSense 텔레옵 + 비전 토글** (스틱 우선, 객체 lock 시 자동 추종 스위치)
3. **다축 (axis0 + axis1 차동 구동)** — 6륜 rocker-bogie 매핑
4. **NVENC 활성화** — openh264 소프트웨어 인코딩이 30fps cap 의 병목
5. **INT8 양자화** — TRT export `int8=True` + 캘리브레이션, 추론 추가 1.5–2× 가속

---

## 하드웨어 실측 구성

| 항목 | 값 |
|---|---|
| 호스트 노트북 | x86_64 Arch Linux (개발 + 영상 수신) |
| 엣지 보드 | Jetson Orin Nano 8GB Developer Kit, Super 모드 (25W), JetPack 6.2.2 / L4T R36.5.0 |
| Jetson 호스트명 | `jetson-orin` (mDNS, ssh `zetin@jetson-orin.local`) |
| 컨테이너 베이스 | `dustynv/l4t-pytorch:r36.4.0` (Docker Hub) |
| ODrive | v3.6, fw `fw-v0.5.6`, vbus 11.96 V (※ 무부하 검증 한정) |
| 모터 / 엔코더 | **실전 구동: BL70200 + 내장 HALL ×3 (pp=5, cpr=30, HALL 모드)** / **테스트 구동: SunnySky X2212-13 + TLE5012B 16384 CPR (pp=7, 외장 엔코더)** / **조향: AK40-10 (테스트) → AK45 (실전), CAN 동일 API**. 5/10 무부하 검증은 SunnySky 트랙에서 수행 |
| 카메라 | USB UVC Microsoft LifeCam Studio (`/dev/video0`, MJPG 640×480@30, 720p max) |

**중요**: spec 작성 시 모터 인벤토리 오기 존재 — 실제 hw 는 위 표대로 세 가지. 5/20
재구성 commit (`docs/specs/2026-05-20-motor-control-reorg-design.md`) 에서 폴더
구조 + 문서 전반 교정 완료.

---

## 결정적 발견 (Hard-won) — 다음 세션도 그대로 유효

### 1. ODrive 저장소가 git LFS 사용 (libfibre prebuilt .so)
`pip install git+https://github.com/odriverobotics/ODrive@fw-v0.5.6#subdirectory=tools` 는
LFS 객체 fetch 안 함 → `.so` 가 pointer 텍스트만 받아져서 `is too small` 에러로 import 실패.
→ Dockerfile.jetson 에서 `git-lfs` apt 설치 + 직접 `git clone` + `git lfs pull` 후
로컬 경로 pip install 로 우회. (`21f82f9`)

### 2. fw-v0.5.6 odrive.enums 는 plain Enum (IntEnum 아님)
fibre serializer 가 wire I/O 시 `int(value)` 호출하는데 plain Enum 은 coerce 실패 →
`TypeError: int() argument ... not 'ControlMode'`. → 모듈 상단에서 `.value` 로 int 추출해
`AXIS_IDLE` / `AXIS_CLOSED_LOOP` / `CTRL_POSITION` 상수로 두고 wire I/O 에 사용. (`68389ee`)

기존 `motor_control/drive/x2212_test/odrive_yolo_object_tracking.py` 가 같은 패턴 (`ControlMode.POSITION_CONTROL`)
으로 안 깨졌던 건 거기 동작 환경의 ODrive lib 가 newer (0.6.x, IntEnum) 라서.

### 3. 폐루프 진입 직전 `input_pos = origin` 박는 게 안전상 결정적
`requested_state = CLOSED_LOOP_CONTROL` 직후 보드 내부의 `input_pos` 가 이전 명령 값 그대로 —
origin (현재 위치) 과 다르면 모터가 즉시 그쪽으로 가속 → runaway. 통합 스크립트 첫 시도에서
이 fix 없었으면 위험했음. (`f3454fd`)

```python
# 폐루프 진입 *전* 에 박아야 함
origin = ax.encoder.pos_estimate
ax.controller.input_pos = origin   # ← 이 줄
ax.requested_state = AXIS_CLOSED_LOOP
```

### 4. dustynv 컨테이너 GStreamer (1.20+) ↔ L4T NVENC plugin (1.14 ABI) 불일치
`nvv4l2h264enc` (NVENC) 인식 X. 5/8 plan 에서 소프트웨어 `openh264enc` 으로 우회 — 30 fps cap
의 주된 병목. 720p/30fps 까진 ARM A78AE 6코어로 충분. NVENC 활성화는 향후 작업.

### 5. dustynv 컨테이너의 opencv-python (pip) 은 GStreamer 백엔드 미포함
`cv2.VideoWriter` 의 GStreamer 백엔드가 NO. → `subprocess.Popen(gst-launch-1.0, ...)` 로
raw BGR `frame.tobytes()` 를 stdin 으로 흘려보내는 방식. fdsrc 출력은 byte stream 이라
`rawvideoparse format=bgr width=W height=H framerate=F/1` 추가 필요.

### 6. TRT engine 사이즈별 캐시
ultralytics `model.export()` 는 자체 캐싱 안 함 — 입력 사이즈를 파일명 `yolov8n_HxW_fp16.engine`
에 인코딩, 존재 시 reuse. (5/8 작업분)

---

## 저장소 구조 (working tree)

```
Defence_Robot/
├── README.md                       # 한국어, 상세 — 트랙별 워크플로우
├── HANDOFF.md                      # 본 문서
├── .claude/
│   └── CLAUDE.md                   # 영어, 요약형 — Claude Code 가이드
├── docker/
│   ├── Dockerfile.jetson           # dustynv/l4t-pytorch + ultralytics + odrive(LFS)
│   ├── docker-compose.jetson.yml   # privileged, /dev mount, NVENC plugin mount
│   └── (Dockerfile, docker-compose.yml, docker-compose.gpu.yml: x86 개발용)
├── motor_control/
│   ├── drive/                      # 구동 모터
│   │   ├── x2212_test/             # SunnySky X2212-13 + TLE5012B (테스트)
│   │   │   ├── init_odrive.py            # USB NVM 셋업 (pp=7, cpr=16384)
│   │   │   ├── odrive_can_setup.py       # CAN NVM 셋업 (IDLE 안전 부팅)
│   │   │   ├── odrive_can_drive.py       # CAN 폐루프 데모
│   │   │   ├── yolo_odrive_jetson.py     # **vision + 모터 통합 (5/10)**
│   │   │   ├── odrive_dualsense_test.py
│   │   │   ├── odrive_dualsense_vel_test.py
│   │   │   ├── yolo_odrive_motor_test.py
│   │   │   └── odrive_yolo_object_tracking.py
│   │   └── bl70200/                # BL70200 + 내장 HALL ×3 (실전)
│   │       └── odrive_*.py (6개)         # HALL 모드 (pp=5, cpr=30)
│   ├── steering/                   # AK40/AK45 조향 (CAN, 동일 API)
│   │   ├── ak_control.py                 # 메인 — AK 클래스 + CANSession
│   │   └── calibrate_ak.py / status_ak.py
│   ├── vision/                     # 비전 단독 (모터 명령 없음)
│   │   ├── yolo_cuda_stream.py           # **5/8 결과물** — Jetson CUDA/TRT + UDP H.264
│   │   ├── yolo_openvino_detection.py    # x86 OpenVINO 참조
│   │   └── setup_yolo_env.sh
│   ├── sensors/                    # US100 거리 (UART /dev/ttyTHS1)
│   │   ├── us100_basic.py                # 0x55 기본
│   │   └── us100_robust.py               # Jetson UART TX 떨림 버그 우회
│   ├── laptop/laptop_client_{basic,velocity,video}.py    # 노트북 측 텔레옵
│   └── pi/pi_server_{basic,velocity,position,video}.py   # Pi 측 서버 (별 hw 라인)
├── parameter_calc/                 # 14차원 형상 파라미터 최적화 (별 트랙, 손 안 댐)
├── scripts/
│   ├── recv_stream.sh              # 노트북 측 GStreamer UDP H.264 수신 헬퍼
│   └── can_setup.sh                # Jetson can0 1 Mbps 셋업 (mttcan + devmem)
└── docs/
    ├── specs/
    │   ├── 2026-05-08-jetson-yolo-stream-design.md     # 비전 단독 [완료]
    │   ├── 2026-05-10-vision-motor-integration-design.md  # 통합 [완료]
    │   └── 2026-05-20-motor-control-reorg-design.md    # hw 폴더 재구성 [완료]
    └── plans/
        ├── 2026-05-08-jetson-yolo-stream-plan.md          # [완료, 측정 결과 + 이슈 표]
        ├── 2026-05-10-vision-motor-integration-plan.md    # [완료, 측정 결과 + 이슈 표]
        └── 2026-05-20-motor-control-reorg-plan.md         # [완료]
```

---

## 빠른 실행 (다음 세션 즉시 재개)

### Jetson 접속 + 컨테이너 진입

```bash
ssh zetin@jetson-orin.local
cd ~/Defence_Robot && git pull origin main
sudo docker compose -f docker/docker-compose.jetson.yml up -d
sudo docker compose -f docker/docker-compose.jetson.yml exec powertrain bash
```

### 비전 단독 (영상만 노트북에 송신)

```bash
# Jetson 컨테이너 안
python3 motor_control/vision/yolo_cuda_stream.py \
    --backend trt --host <노트북IP> \
    --width 640 --height 480 --fps 30
```

```bash
# 노트북 호스트
~/Defence_Robot/scripts/recv_stream.sh 5000
```

### 비전 + 모터 통합 (axis1 추종, 무부하)

ODrive 가 처음이면 한 번:
```bash
python3 motor_control/drive/x2212_test/init_odrive.py    # 1회만. NVM 저장 + reboot
```

이후 매번:
```bash
python3 motor_control/drive/x2212_test/yolo_odrive_jetson.py \
    --backend trt --width 640 --height 480 --fps 30 \
    --target bottle \
    --scale 5.0 --max-turns 2.0 --deadzone 0.05 --input-filter 2.0 \
    [--stream --host <노트북IP> --port 5000]    # 옵션: 영상 송신 동시
```

`--no-motor` 추가하면 ODrive 명령 안 보내고 검출 로직만 검증 (dry-run).
Ctrl-C 시 원점 복귀 + IDLE 자동 처리.

---

## 호스트 (Arch) 측 사전 준비 — 이미 적용됨

이전 세션에서 설치/구성 끝난 것들. 다른 폴더 Claude 가 같은 노트북에서 작업한다면 그대로 사용 가능:

- `/usr/local/bin/service` — Debian-style `service` → `systemctl` shim (NVIDIA SDK Manager 호환용)
- `/usr/local/bin/ping6` — `ping -6` shim (OpenSSH 10.x DSA 제거 우회 시 필요했던 헬퍼)
- `/etc/nsswitch.conf` 의 hosts 라인에 `mdns_minimal [NOTFOUND=return]` 추가 — `jetson-orin.local`
- `nss-mdns` + `avahi-daemon` active

GStreamer 수신 패키지: `sudo pacman -S --needed gstreamer gst-plugins-{base,good,bad,ugly} gst-libav`

---

## 5/10 통합 plan 검증 결과 (요약)

| Phase | 모드 | avg_fps | infer (ms) | 비고 |
|---|---|---|---|---|
| B/C | 추종 only | ≈ 30 | ≈ 9.8 | 페트병 좌우 이동에 axis1 추종 정상, MAX_TURNS=2.0 saturation, Ctrl-C IDLE OK |
| D | 추종 + 스트리밍 | **28.3** | **9.8** | 300 frame 평균. `tgt` ±0.97↔±0.16 변동, 모터 추종 + 노트북 영상 동시 정상 |

상세 측정 환경 + 발견 이슈 5건 영구 fix 표는
[`docs/plans/2026-05-10-vision-motor-integration-plan.md`](docs/plans/2026-05-10-vision-motor-integration-plan.md) 끝부분.

---

## 최근 commit 흐름 (5/8 → 5/10)

```
a39f225 docs: vision+모터 통합 검증 결과 + 트랙 정정 + init_odrive.py 반영
f3454fd fix(motor_control): init_odrive.py 분리 + yolo_odrive_jetson.py 동작 수정
68389ee fix(motor_control): odrive fw-v0.5.6 plain-Enum 호환 (.value)
21f82f9 fix(docker): odrive 설치에 git-lfs 추가 (libfibre LFS)
d88b8d0 feat(motor_control): yolo_odrive_jetson.py — vision + 모터제어 통합
91d2645 docs: vision+모터 통합 spec/plan 신규 작성
4a7e99a docs: Jetson 스트리밍 작업 결과를 README + .claude/CLAUDE.md 에 반영
─── 이전 5/8 plan (비전 단독) ───
ad92d39 docs(plan): Jetson YOLO 스트리밍 측정 결과 + 이슈/fix 정리
05bc99a fix(motor_control): TensorRT engine 사이즈별 캐시
945f400 fix(motor_control): fdsrc → rawvideoparse 로 video frame 화
034b60c fix(motor_control): cv2.VideoWriter → subprocess + gst-launch
e43196a fix(motor_control): NVENC 미동작 → openh264enc 우회
9219cee fix(docker): numpy<2 + privileged 모드 (V4L2 cgroup 우회)
1fb4c78 fix(docker): pip 인덱스 PyPI 표준 강제
905f619 fix(docker): odrive PyPI 패키지 일단 제외 (이번엔 vision 중심)
cf3a11c fix(docker): dustynv/l4t-pytorch:r36.4.0 베이스로 정정
ffb6bea docs: Jetson YOLO 스트리밍 spec + plan
c7e2561 feat(scripts): recv_stream.sh
cd9eec7 feat(motor_control): yolo_cuda_stream.py
d447b55 feat(docker): Jetson 이미지 베이스 변경
```

---

## 미해결 / 다음 세션이 결정할 것

1. **실차 차체 부착 시 vbus / 전류 한계 재검토**: 현 검증은 11.96 V (배터리 또는 벤치 전원).
   실차 운영 시 정격 24 V 가정 — `init_odrive.py` 의 `current_lim` 등 NVM 값 재튜닝 필요할 수
   있음. 5/10 spec 의 Open Question 5번에 적힌 NVM 캘리 유효성 매번 재확인.
2. **카메라 마운트 좌우 반전 여부**: 검증 시 페트병 좌→우 시 모터 어느 방향? — 답은 stdout
   메모 안 했음. 실차 부착 후 결정. `--scale` 부호 또는 `cv2.flip(frame, 1)` 로 보정.
3. **다음 마일스톤 우선순위**: 위 "남은 큰 마일스톤" 5개 중 사용자 결정.
4. **BL70200 실차 부착 후 캘리 검증**: HALL 트랙 코드 (`drive/bl70200/`) 의 캘리값
   pp=5/cpr=30 은 BL70200 과 일치하나, vbus 24 V 환경에서 `current_lim` /
   `vel_limit` 재튜닝 필요할 가능성. NVM 캘리 유효성 매부착 시 재확인.

---

## 외부 참조

- ODrive fw-v0.5.6 docs: https://docs.odriverobotics.com/v/0.5.6/
- dustynv jetson-containers: https://github.com/dusty-nv/jetson-containers
- ultralytics: https://github.com/ultralytics/ultralytics
- README 사사 문구 (UBAI 슈퍼컴 — 별 트랙): `parameter_calc/` 작업물에만 적용

---

## Claude 행동 가이드

- `parameter_calc/` 는 별 트랙. 손대기 전 `parameter_calc/CLAUDE.md` 의 GPU 버그 히스토리 필독.
- HALL / 엔코더 트랙을 한 ODrive 에서 번갈아 쓰지 말 것 (NVM 캘리 설정 차이로 폭주 위험).
- 모든 테스트 axis = `axis1`. axis0 금지.
- 결과 파일 (`*.pkl`, `*.mat`, `*.mp4`, `fig*.png`) 은 서버 검증본 — 의도 없이 덮어쓰지 말 것.
- 비전 + 모터 통합 스크립트는 **반드시 무부하 (차체 미부착)** 에서 시작. 폐루프 진입 시 모터 점프
  방지를 위해 `input_pos = origin` 을 진입 전에 박을 것.
- ODrive USB 분리 = 즉시 토크 해제 (비상정지 백업).
