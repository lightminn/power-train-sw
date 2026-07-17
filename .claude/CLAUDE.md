# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working in this repository.

## Project Overview

ZETIN Defense Robot — a 6-wheeled rocker-bogie suspension robot. Work in this repository is split into two independent tracks:

1. **Parameter optimization** (`parameter_calc/`) — multi-criteria optimization that selects the optimal rocker-bogie geometry across terrain types. Current authoritative track is **v4** (`python_gpu_triangle/`): 15-dimensional, 7 terrains (stairs, wood, rough, step, curved ramp, 15°/30° incline), 면-기준 물리 수정본. v3 (`python_gpu/`): 14-dim, 4 terrains.
2. **Motor control** (`motor_control/`) — runtime control software for the physical robot: ODrive driver scripts, DualSense gamepad teleop, YOLO-based object tracking, and laptop↔robot networking.

## WP5.1 Completion Override (2026-07-11)

- WP5.1 HIL is complete using the prior 10-motor integration evidence plus the 2026-07-11
  US-100, fail-safe, and real 50 Hz evidence. ODrive 13/14 were temporarily absent only.
- Ground braking and final `stop_mm` selection are vehicle commissioning after chassis assembly,
  not an open WP5.1 gate. Keep 200 mm as the bench/HIL value until commissioning.
- Next software task: single `/cmd_vel` command-authority specification.

## WP5.1 Authority Override (2026-07-10)

This block supersedes conflicting status and safety semantics below while preserving them as
historical context.

- Existing WP5 `/cmd_vel → ChassisManager → 10 motors` HIL is historical baseline evidence.
  **WP5.1 Tasks 1–8 software are complete; final Jetson/10-motor/US-100 HIL is NOT RUN.**
  At deployed HEAD `c3610c136357a8c881263926ec18bcd7e3432a5d`, root-observed evidence is
  motor_control 189 passed, motor_gui 91 passed, an isolated read-only ROS 3-package clean build
  with powertrain_ros 31/31, and the Jetson exact-HEAD 3-package build with powertrain_ros 31/31.
  Jetson software-only FAKE acceptance was observed separately at commit
  `49831bb42058a177ed9c41d72d0273f4f0a8f535`: 3000/60 s, mean and minimum 5 s window
  50.000 Hz, tick p99 0.280 ms, overrun 0, maximum interval 21.453 ms, and publisher-death
  E-stop at 0.753 s. Startup `ESTOP`, far `ARMED/RUN`, near `ESTOP`, far-return latch,
  reset→`IDLE` with no implicit arm, and separate arm were confirmed. FAKE is not HIL;
  final hardware HIL remains pending.
- Policy and motor ownership live in the pure-Python `SafetyInterlock` and `ChassisManager`.
  Thin internal ROS nodes isolate blocking US-100 UART at 5–10 Hz, transport `/safety_verdict`,
  run the 50 Hz chassis loop, and publish `/wheel_states`. The hardware topology remains one
  `can0` at 500 kbps, AK45-36 ×4 plus ODrive/BL70200 ×6.
- US-100 uses `VALID`, `INVALID_READING`, `CHECKING`, and `NO_RESPONSE`. `INVALID_READING` is
  normal operation: 0x50 confirms MCU/UART liveness only. `CHECKING`, the 0.5 s `/cmd_vel`
  watchdog, and disconnect are auto-recovering `MOTION_HOLD`; the command watchdog is distinct
  from safety-topic freshness. Valid near distance or three consecutive distance-and-liveness
  misses are latched `ESTOP`.
- Reset returns only to `IDLE`; arm is a separate action. Production safety-topic timeout is
  0.75 s (`age > threshold`, next 50 Hz tick nominally 0.75–0.77 s), startup timeout is 1.0 s,
  and `safety_required=false` is BENCH/FAKE only.
- Combined hardware launch always requires an explicit `stop_mm`; there is no production default.
  Before approval, only a controlled low-speed HIL candidate may use an explicit provisional value.
  Scenarios 1–8 require all wheels lifted. Scenario 9 is a separately authorized ground-motion
  phase for the real 50 kg robot and cannot inherit the wheels-up confirmation; only its measured,
  approved `stop_mm` may be used for production.
- Any migrated `safe/warn/stop`, `Verdict.level`, startup/`None`→`stop`, or auto-clearing gate text
  is **superseded historical memory**, not current authority. Use the 2026-07-10 WP5.1 design,
  implementation plan, and `docs/reports/2026-07-10-wp5-control-safety-hil.md`.
- After final HIL: command-authority spec → L515 lightweight color-image + depth-image + IMU
  pipeline (PointCloud2 optional) → WP6. WP8 plus `MISSION_STOP`, unlock ordering, and the full
  `ARRIVED_* → arm work → DONE → resume` handshake remain open.
- Provenance at `c3610c136357a8c881263926ec18bcd7e3432a5d`: local JUnit files are
  `.superpowers/sdd/final-motor-control-c3610c1.xml`,
  `.superpowers/sdd/final-motor-gui-c3610c1.xml`, and `.superpowers/sdd/final-ros-c3610c1.xml`;
  Jetson ROS raw XML is `/home/zetin/power-train-sw/ros2/build/powertrain_ros/pytest.xml`.
  The FAKE summary remains the distinct 49831bb observation without a preserved raw log.

## Current Source of Truth (2026-07-16)

- Read `docs/reports/2026-07-16-project-state-and-handoff.md` first — it is the handoff
  authority (bootstrap reading order, WP status table, verification recipes, deploy steps,
  backlog). Then the master plan `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`.
  It supersedes the 2026-07-10 project-state report and older status banners on conflict.
- **Complete (vehicle-free software, as of 2026-07-17): WP1–WP5.3 (Tasks 1–7 SW),
  WP6-A core, WP6-S P0+P1 (`e22e364`, closed loop `d30ace1` + hidden_eval CLI),
  WP6-B NumPy terrain core (`eba8b74`) + JAX kernels/equivalence (`5a415e9`),
  WP6-C terrain autonomy controller (`c744936`), remote assist + protocol v2
  (`f41730f`), laptop dual-video viewer (`0198830`), console CAN unification
  (`f9d01df`), idle-drain flapping fix (`3c1e098`), WP7 follow controller
  (`158b863`), WP8 section-supervisor skeleton (`72ec7e4`).** Wheel-stop thresholds
  are HIL-qualified. Remaining work is hardware/bench gates (mount-angle HIL,
  profile-preset HIL, JAX Jetson full-load qualification, D-runbook remote E2E
  smoke, live-stack ~530 ms stall observation) and cross-team contracts (gateway
  wiring after the teammate's l515 WIP lands, WP8 real perception-event topics,
  `MISSION_STOP` unlock ordering, one full handshake). See the handoff report §6.
- **FULL HIL mode**: the user performs physical actions only (power, lifts, visual checks);
  all commands run via agent SSH. Motor motion needs prior physical confirmation. The chassis
  is NOT assembled (bench motors, arm absent) — ground measurements (odometry 5 m/90°,
  `stop_mm` commissioning) wait for vehicle assembly.
- Sensors: **L515=powertrain RGB/depth/IMU, D435i=robot-arm exclusive, US-100=independent safety**.
- ODrive authority: pp10/cpr60/bw30/vg0.12/vi0.2, node11–16. Use `bl70200_setup.py` and
  `can_calibrate_all.py`; never use legacy single-axis pp5 `odrive_calibration.py` on real hardware.
- v4 calculation mass is final at **50 kg**; do not schedule an 86 kg rerun.
- Before work, inspect GitHub and both Jetson checkouts for dirty/ahead/behind state. Preserve all
  teammate untracked and uncommitted files.

## Directory Layout

```
Defence_Robot/
├── .claude/              Claude Code settings + this file
├── parameter_calc/       Geometry optimization (trusted server build)
│   ├── CLAUDE.md         → detailed docs for this track
│   ├── matlab/           Original MATLAB reference (functions/, *.m, *.mat)
│   ├── python/           CPU port (NumPy/SciPy) + final v4 result pkl (f_opt 0.2004)
│   ├── python_gpu/       GPU port (JAX/CUDA 12.x) — v3
│   ├── python_gpu_triangle/  v4 authoritative (15-dim/7-terrain + validate/cross_validate/analyze/plot tools)
│   ├── archive/          initial v4 result (f_opt 0.2624) kept for history
│   └── scripts/          run_gpu.sh, run_gpu_triangle.sh, run_v4_*.sh
├── motor_control/        ODrive · AK 조향 · YOLO · US100 센서 · 텔레옵
│   ├── drive/            구동 모터
│   │   ├── x2212_test/   SunnySky X2212-13 + TLE5012B (ODrive USB · CAN)
│   │   └── bl70200/      BL70200 + 내장 HALL ×3 (실전, ODrive USB·CAN)
│   ├── steering/         AK40/AK45 조향 (CAN, 동일 API)
│   ├── vision/           검출·스트리밍 (기존 D435i 자산 + L515 자율주행 입력)
│   ├── sensors/          US100 거리 (UART /dev/ttyTHS1)
│   ├── safety_us100/     US-100 충돌방지 안전 모듈 (거리→safe/warn/stop, publish-only)
│   ├── corner_module/    코너 모듈 패키지 (조향+구동 협조 제어 + US-100 게이팅 텔레옵)
│   ├── chassis/          4WS 차체 통합 (애커만 kinematics + ChassisManager, 실기 10모터 HIL 완료)
│   ├── laptop/           Laptop-side TCP teleop clients (DualSense → robot)
│   └── pi/               Raspberry-Pi-side servers (paired 1:1 with laptop/)
├── motor_gui/            웹 진단 GUI (FastAPI + 트랜스포트 추상화, AK/ODrive CAN·USB)
├── operator_console/     운용 PC read-only GTK 콘솔 (SRT 2화면 + UDP 텔레메트리, 명령 표면 없음)
├── docker/               Container definitions (x86 dev + Jetson Orin Nano deploy)
├── scripts/              Host-side helpers (recv_stream.sh · recv_yolo3d.py · can_setup.sh ·
│                         install_*.sh + systemd/ 유닛·udev·tmpfiles 아티팩트)
└── docs/
    ├── specs/            Per-task design docs (requirements, interfaces)
    ├── plans/            Per-task implementation plans + verification logs
    └── reports/          진행 보고·결과 로그 (Notion 백업, 파라미터 결과 등)
```

Detailed simulation pipeline, parameter space (v4 15-dim / v3 14-dim), objective weights, GPU acceleration strategy, and known GPU bug history live in `parameter_calc/CLAUDE.md`. Read that file before touching anything in `parameter_calc/`. Per-task background for Jetson / streaming work lives under `docs/specs/` and `docs/plans/` — read those before editing the matching scripts.

## Source-of-Truth Note

`parameter_calc/` was downloaded from the development server and is the authoritative implementation — its code and any persisted results (`*.pkl`, `*.mat`) should be trusted over historical local copies. Earlier local-only versions were removed during the directory cleanup.

## SW Notion 문서 표준

기능별 사용법은 팀 Notion 허브 `극한로봇 파워트레인 → 💻 Software` 에 정리한다 (repo `README.md`
의 매핑표가 레포 영역 ↔ Notion 페이지 인덱스 = source-of-truth). 새 SW 페이지는 **`📄 SW 문서
표준 템플릿`** 페이지를 복제해 작성한다. 표준 구조: 개요 콜아웃 → 목차 → ①환경 → ②핵심개념/
파라미터 → ③배선(HW 시) → ④설치·사전준비 → ⑤실행 → ⑥트러블슈팅 표 → ⑦검증결과 표 → ⑧코드·참고.
컨벤션:

- **④설치·⑤실행은 "Jetson 에 SSH 접속 직후(홈 `~`)" 기준**으로, 순서대로 복붙만 하면 목표
  달성하도록 쓴다 (레포 이동 → 호스트 준비 → 컨테이너 진입까지 포함; 컨테이너 떠 있음·CAN 올라옴
  같은 중간 상태 가정 금지).
- 콜아웃 색: 파랑=개요, 빨강=안전/위험, 회색=팁/함정. **함정(footgun)은 ⚠️ 명시.** 수치·모델은
  현재값(AK45-36 ×4 = CAN id 1~4 / ODrive 듀얼축 3보드 = node 11/12·13/14·15/16 / CAN 500 kbps /
  구동 게인 bw30·vel_gain 0.12·vel_int 0.2).
- 구버전은 삭제 대신 상단 ⛔ DEPRECATED 콜아웃 + 정본 링크 후 Archive 로 이동.
- **초보자 복붙 기준 + 소스코드 분리** (2026-06-25 추가): 문서는 **초보자가 복붙만 따라
  해도 바로 실행**되도록 자세히 쓴다 (SW 문외한 기준 — **접속(ssh)→호스트(can_setup)→
  컨테이너 진입(docker exec)→실행**을 빠짐없이, odrivetool 쓰면 **켜는 법(`odrivetool` 실행)도
  명시**. 각 명령 블록에 **어디서**(노트북/호스트/컨테이너/odrivetool) 치는지 + **✅ 기대 출력**을
  적고, 쓸데없는 부가설명은 빼서 명령·체크에 집중). 단 **노션엔 풀 파이썬 소스코드를 넣지 않는다** —
  풀 스크립트(.py; python-can·멀티함수·루프 등)는 **레포에 올리고 파일 경로·이름만** 노션에
  적어 필요한 사람이 찾아보게 한다(예: `motor_control/can_ak_odrive_demo.py` + 실행 한 줄).
  노션에 직접 적는 코드는 **odrivetool 인터랙티브에서 한 줄씩 바로 칠 수 있는 형태**
  (`odrv0.axis1.controller.input_vel = 1.0` 식)만 둔다. bash 준비명령(can_setup 등)·
  프로토콜 표·수치·실행 명령은 노션에 둬도 됨.

## Working in `motor_control/`

Mostly self-contained scripts; two shared packages (`corner_module/`, `chassis/`). Three motor
hardware lines, isolated by subfolder. **Never mix tracks on the same ODrive** (calibration
/ gain / current limits diverge).

- **drive/bl70200/** (BL70200 + 내장 HALL ×3, 실전 구동 — **실측 `pp=10, cpr=60`**, HIGH_CURRENT):
  정본 셋업 = `bl70200_setup.py` (`--read`/`--apply`/`--calibrate`/`--node N`, 최적 NVM CFG 한곳 —
  bw30·vel_gain 0.12·vel_int 0.2·ignore_illegal_hall_state=True·48V UV40), `bl70200_dual_axis.py`
  (듀얼축 M0+M1 캘리·데모), **CAN 다축 도구**: `can_calibrate_all.py`(node 11~16 일괄 풀캘리 —
  캘리 RAM-only 라 전원 켤 때마다 필요), `can_drive_test.py`(6축 동시 주행 브링업). 레거시 단축
  테스트: `odrive_calibration.py`, `odrive_*_test.py`(구스크립트 일부 pp=5 하드코딩 — 그대로 쓰지 말 것).
- **drive/x2212_test/** (SunnySky X2212-13 + TLE5012B, **레거시·deprecated** — BL70200 도착 전
  임시 엔코더 테스트모터; 엔코더 기반 X2212 제어는 폐기(실전 BL70200=HALL), ODrive CAN 일반
  실험데이터는 유효 → 「AK + ODrive 동시 CAN」 정본으로 이관):
  `init_odrive.py` (USB 1회 NVM 셋업, pp=7 cpr=16384), `odrive_can_setup.py` /
  `odrive_can_drive.py` (CAN), `odrive_dualsense_*.py` (텔레옵), `yolo_odrive_jetson.py`
  (Jetson 비전 추종, USB), `yolo_odrive_motor_test.py` · `odrive_yolo_object_tracking.py`
  (x86 OpenVINO 추종, 참조).
- **steering/** (실전 AK45-36 = `ACTIVE_MOTOR` 기본값, 레거시 테스트 AK40-10, CAN socketcan can0): `ak_control.py` (메인 라이브러리 —
  python-can socketcan 직접 제어; **위치제어 슬루 `DEFAULT_SPD_ERPM=4500`**(2026-07 원격조종
  응답성 상향, 1500→4500 ≈ 출력축 47°/s·45° 0.85s, 정격 5180 이내; 무부하 실측 오버슈트 0,
  실링키지 부하 시 재확인)·`DEFAULT_ACC_ERPM_S2=20000`), `calibrate_ak.py` (기어비 1회성),
  `status_ak.py` (CAN RX 디버깅). 사전 준비: `bash scripts/can_setup.sh`. ⚠️ can0 가 LOOPBACK
  으로 sticky하게 걸리면(down/up 무효) `ip link set can0 type can loopback off` 명시 필요(버스 무음).
  ⚠️ **모터 PWM 노이즈 → 젯슨 CAN TX 오염 + mttcan 웻지** (2026-07-07 규명 → **절연
  트랜시버 교체로 종결**): 비절연 트랜시버 시절 젯슨 송신만 에러(정지 폐루프 유지 ≈27% ≫
  회전 ≈2%; 원인 = SVM 에지 정렬 + 그라운드 도메인 비대칭) → bus-off 폭풍 → **mttcan
  드라이버 TX 큐 영구 정지**(berr 0 인데 모든 send ENOBUFS, down/up 만이 복구). **절연형
  트랜시버 장착 후 동일 A/B 에서 노이즈 완전 소멸**(최악 정지 27.9%→0.0%, 폭격
  74.6%→0.00%/13,205프레임). **웻지 워치독은 보험으로 상주 유지** — compose `canwatchdog`
  서비스(컨테이너 스택과 자동 기동, 구현 `corner_module/can_watchdog.py`, 감지 ~2s ioctl
  down/up·오탐 0) + 텔레옵 인프로세스 내장 + 호스트판 `scripts/can_watchdog.sh`(비상용).
  전말·실험 16종: `docs/specs/2026-07-07-can-pwm-noise-tx-wedge.md`.
- **vision/** (모터 명령 없음): `gst_stream.py` (공용 송신 파이프라인 — H.264
  SW 인코딩(x264/openh264, **Orin Nano 는 NVENC 없음**) + SRT listener,
  `--srt-latency` 기본 60ms), `yolo_depth_3d.py` (YOLO+depth 3D 좌표 — color
  영상 SRT + 좌표 UDP JSON 분리 송신, 영상을 YOLO 추론 전에 먼저 송신해 영상
  지연 최소화, 기본 848x480/YOLO26n/x264), `yolo_cuda_stream.py` (Jetson
  CUDA/TRT USB 카메라 송신), `realsense_test.py` (RealSense D435i depth+color
  점검), `realsense_stream.py` (color+depth 진단 송신 — sidebyside/overlay,
  원격주행용 아님), `yolo_openvino_detection.py` (x86 OpenVINO),
  `setup_yolo_env.sh` (x86 conda, Docker 권장).
  - **노트북 수신 2경로**(둘 다 SRT caller, `--srt-latency`/`latency` 송수신
    같이 맞춤): ① `scripts/recv_stream.sh [PORT] [HOST] [LATENCY]` — 저지연
    네이티브 gst 뷰어(오버레이 없음), **원격주행용**. ② `scripts/recv_yolo3d.py`
    — 좌표 박스 오버레이 cv2 뷰어(`--scale`/`--clock`), 표시 지연 더 큼,
    정밀 접근·좌표 점검용.
- **sensors/** (UART `/dev/ttyTHS1`): `us100_basic.py` (US100 0x55 기본),
  `us100_robust.py` (Jetson UART TX 떨림 버그 우회 — 0xFF prefix).
- **safety_us100/** (US-100 충돌방지, publish-only): 거리→`safe`/`warn`/`stop` 판정만
  내보냄(모터 직접 제어 X). `evaluator`/`safety_monitor`/`verdict`/`config`(stop 200/warn 400/
  hyst 30mm), `us100.py`(실센서), `fake_sensor`+`tests`, `demo.py`,
  `teleop_odrive_only.py`(US-100 게이팅 ODrive 단독 텔레옵 — 조향 없는 구동만).
  못 읽으면 fail-safe `stop`. 코너 모듈 텔레옵이 물려 `stop` 시 구동 0.
- **corner_module/** (조향+구동 협조 제어 패키지, 코너 1개 = 로커보기 1/6): `corner_module.py`
  (`CornerModule` — 상태머신·워치독·estop·과전류 트립·폐루프 점프방지), `actuator.py`
  (트랜스포트 무관 `Actuator`/`SteerActuator`/`DriveActuator` ABC), `steer_ak40.py`(AK CAN —
  자기 STATUS_1 만 받는 CAN 필터 + state() stale 자가회복)·`null_steer.py`(고정 바퀴 no-op)·
  `drive_odrive_usb.py`(USB)·`drive_odrive_can.py`(**CAN 정본, WP1 완료** — CANSimple, 노드별
  소켓+필터, bus 주입으로 무하드웨어 테스트) 드라이버, `fake.py`(테스트 더블),
  `teleop_dualsense.py` (`python3 -m corner_module.teleop_dualsense`; US-100 충돌방지 연동 —
  `stop` 판정 시 구동 0). 단위테스트 34 + HIL(조향·구동·통합·텔레옵) 검증. 4WS 의 빌딩블록.
- **chassis/** (4WS 차체 통합 패키지, WP2+WP3): `kinematics.py`(차체 (v,ω)→바퀴별 조향각·속도,
  애커만+조향/속도 자동 클램프), `chassis_manager.py`(`ChassisManager` — 코너 6개 통합,
  `DEFAULT_WHEEL_MAP` = AK id 1~4 조향 + ODrive node 11~16 구동, estop 전파·US-100 게이팅·차체
  워치독, **`min_drive_turns_per_s` 최저 구동속도 플로어**(0=off; 0<|명령|<이 값이면 부호 유지
  상향 → 저속 HALL 코깅존 회피. **2026-07-17 D3/D4: 기본값 전면 0 = 폐지** — 저속 코깅
  대응은 `DriveOdriveCan`의 `friction_ff`/`v_knee`(torque_ff 피드포워드, 기본 off, 값 튜닝은
  벤치)로 대체, 스펙 docs/superpowers/specs/2026-07-17-abc-program-design.md §2.2/§2.2b);
  `build_real_corners("can0", friction_ff=, v_knee_turns_s=)` 로 실기 코너 생성),
  **`teleop_dualsense.py`**(`python3 -m chassis.teleop_dualsense [--no-us100]` — DualSense →
  (v,ω) → 10모터 4WS 수동주행; RT/LT=전후진, 좌스틱X=회전, 트리거0+스틱=피벗; 기본 min-rev 0·
  v-max 1.5). **무선판**(DualSense→노트북→젯슨→모터): `teleop_server.py`(젯슨,
  `python3 -m chassis.teleop_server --no-us100`) ↔ `laptop/laptop_client_chassis.py`(노트북 —
  DualSense **raw 입력**만 TCP:9000 송신, 매핑·속도한계·min_drive·피벗은 전부 서버쪽; □arm/○estop,
  끊기면 구동0). 프로토콜 `"left_x rt lt sq ci\n"`. **ops 채널 :9001**(A2a, 2026-07-18):
  복구·운용 명령 단일 게이트 `ops_broker` 노드 — 역할 토큰(`/etc/powertrain/ops_*.token`,
  서버가 토큰→역할 매핑), 4상태 ACK(PENDING/FINAL_*/OUTCOME_UNKNOWN)·멱등 재전송·비상
  2단계 서버 검증(reset 5s/arm 3s), 복구 chord = `recovery-v1-initial-candidate`(⚠️ 전부
  임시: □+CREATE 2s=hold해제, D-pad↓/↑=권한, L1+R1+□/△=비상 reset/arm). 배포 =
  `control.launch.py`(teleop+broker)를 powertrain_control 서비스가 실행. ⚠️ 빈/죽은 CAN 버스여도 서버 안 죽게 강건화
  (`DriveOdriveCan._send` CanError 흡수 + 제어루프 try/except). 단위테스트 34. **실기 10모터 협조
  4WS HIL 통과(2026-07-05, 실물 확인) + 무선 엔드투엔드 검증(2026-07-06: 유선판과 동일 코드경로 —
  전진 4축 2.40~2.42 rev/s 균일, 좌회전 애커만 차동 좌1.46<우2.01 실측, arm/estop/끊김 동작)**.
  ⚠️ HIL 교훈: 바퀴 지령 <0.3 rev/s(HALL 코깅존)면 실물이 정지한 채 텔레메트리만 그럴듯함 —
  테스트는 v≥0.4 m/s + 실물 육안 확인 필수. ⚠️ 모터 실기 테스트 전 좀비 teleop/제어루프(`docker exec ps|grep
  teleop`) 죽일 것(v=0 계속 명령해 새 테스트와 싸움).
- **Networked teleop** (1:1 pairs): `laptop/laptop_client_*.py` ↔ `pi/pi_server_*.py`
  (TCP `:9000`, newline-delimited `%.4f\n` velocity); `laptop_client_video.py` adds
  GStreamer JPEG video at `:5000`.

ODrive 펌웨어 v0.5.x (CAN 트랙 fw-v0.5.6 검증). 구동은 **듀얼축 보드 3장**(M0=`axis0`+M1=`axis1`
양축, CAN node 11/12·13/14·15/16) — 단축 레거시 스크립트만 `axis1` 기준. 폐루프 진입 전
`input_pos = 현재위치`(위치모드) 또는 `input_vel = 0`(속도모드) 설정으로 모터 점프 방지.
캘리는 RAM-only — 전원 사이클마다 `bl70200/can_calibrate_all.py` 로 재캘리.

## Working in `motor_gui/`

브라우저 기반 모터 진단·튜닝 GUI. `python3 -m motor_gui.backend.server --track {fake|usb|ak|odrive_can|can}`
(FastAPI, 브라우저 `http://<host>:8000`, network_mode host → 포트매핑 불필요). 핵심은
`backend/transport/` 의 `Transport`/`CanDevice` ABC — AK·ODrive 를 컴포저블 디바이스로 묶어
한 can0 버스에 다중 디바이스 운용 가능. 신규 디바이스는 ABC 구현 후 `worker.py`(100 Hz 샘플)에
드롭인. CSV/Parquet 레코더(`recorder.py`), 텔레메트리 WebSocket 제공. `motor_control/` 을
import(예: `ak_control`)하지만 **`motor_control` 이 `motor_gui` 를 import 하면 안 됨**(역의존 금지).
테스트 `motor_gui/tests/` (dev 컨테이너 pytest). 코너 모듈 HIL 때 `--track usb`/`--track ak` 로 실하드웨어 검증함.

## 테스트·실행 환경 (Jetson 우선)

실제 실행·검증은 **Jetson Orin Nano 에서 직접 돌려보는 것을 우선**한다 (런타임 타깃이
Jetson). x86 노트북의 dev 컨테이너(`powertrain-sw:dev`, `docker/docker-compose.yml`)는
**Jetson 을 쓸 수 없을 때의 차선 환경**이다 — "무하드웨어 전용"이 아니라, 무하드웨어
`pytest`(`motor_gui/tests/`·`corner_module`·`safety_us100` + fake 드라이버)·코드 작성에
더해 **ODrive 를 노트북에 USB 직결해 실제 모터를 굴리는 작업까지 포함**한다(compose 가
`/dev` 마운트 + `SYS_RAWIO` 제공; 단 ODrive USB reset ioctl 때문에 `docker run --privileged`
로 띄워야 연결됨 — `cap_add` 만으론 I/O 에러). odrive 파이썬 라이브러리는 Jetson 과 동일한
**git `fw-v0.5.6`(=0.5.6) 소스**로 맞춰 fw 0.5.x 보드·검증 스크립트(axis-level
`odrv.axis1.clear_errors()` 등)와 호환된다(PyPI 는 0.5.6 미배포). 이 x86 이미지는 **CPU 전용**
— YOLO GPU 추론은 Jetson `Dockerfile.jetson` 에서만 한다.

## Jetson Orin Nano deployment

`docker/Dockerfile.jetson` + `docker/docker-compose.jetson.yml` build on top of
`dustynv/l4t-pytorch:r36.4.0` (CUDA + cuDNN + TensorRT + ARM PyTorch). Compose
file mounts `/dev`, NVENC GStreamer plugin, and runs `privileged: true` so V4L2
cameras + USB devices are accessible from the container. The image also source-builds
the **Intel RealSense SDK** (librealsense + pyrealsense2, RSUSB backend) for RealSense
RGB-D cameras. Vision/streaming entry points are `motor_control/vision/yolo_cuda_stream.py`
and `realsense_stream.py` (color+depth); the laptop runs
`scripts/recv_stream.sh <port>` to display the decoded stream. Background and
verification log: `docs/specs/2026-05-08-jetson-yolo-stream-design.md`,
`docs/plans/2026-05-08-jetson-yolo-stream-plan.md`. Hardware reinventory + folder
reorg (5/20): `docs/specs/2026-05-20-motor-control-reorg-design.md`,
`docs/plans/2026-05-20-motor-control-reorg-plan.md`.

## Robot Specification (shared across both tracks)

- 6 wheels, rocker-bogie suspension
- Wheel radius: 100 mm
- Parameter-optimization reference mass: **50 kg** (v4 확정값; 86 kg 재최적화 안 함). 실차 질량·최대 허용 질량은 별도 기계 사양으로 혼동하지 않는다.
- Drive motor (test): SunnySky X2212-13 + TLE5012B 16384 CPR encoder
- Drive motor (real): BL70200 + internal HALL ×3 (**pp=10, cpr=60** — 2026-06 실측; 구문서의 pp=5/cpr=30 은 오기) ×6, ODrive v3.6 듀얼축 보드 3장(CAN node 11~16, 500 kbps)
- Sensors (2026-07-07 확정, 2026-07-10 Jetson USB 실재 확인): **L515=파워트레인 RGB/depth/IMU**, **D435i=로봇팔 인식 전용**, **US-100=독립 충돌 안전**. 파워트레인은 D435i 원본을 직접 점유하지 않고 `/detected_objects`를 구독한다.
- Steering: CubeMars **AK45-36** (real/active, 36:1; peak 24 Nm, rated 8 Nm, KV80, peak current 65 A, backlash 12 arcmin, back-drive 0.8 Nm) / AK40-10 (legacy test, 10:1), CAN bus, identical API. 기본 `ACTIVE_MOTOR="AK45-36"` (`ak_control.py`의 `MOTOR_PROFILES`로 전환)

## 2026 국방로봇 자율주행 전체 계획 Notion 정본

- 팀 공용 Notion 설명본은 **「2026 국방로봇 자율주행 SW 전체 개발계획」**
  (page ID `39c2d27b-08d3-8172-8c1a-de21cc72216b`,
  https://app.notion.com/p/39c2d27b08d381728c1ade21cc72216b)이다.
- 이 페이지는 로봇팔 통합만 다루는 문서가 아니다. 대회 목표·전체 아키텍처·WP5.2 안전 기반·
  WP5.3 관측성·시뮬레이션·WP6-A/B/C·WP7~9·원격운용·시험/HIL·일정·완료/중단 기준을
  모두 포함해야 한다. 로봇팔 협업과 원격 ARM은 전체 계획의 한 섹션으로 유지한다.
- 세부 기술 정본은 `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`,
  `docs/plans/2026-07-13-wp5.2-arm-collaboration-safety-plan.md`,
  `docs/plans/2026-07-13-observability-data-quality-remote-assist-plan.md`다. Notion을 수정할 때
  세 문서의 범위·의존 순서·acceptance를 함께 동기화하고 쓰기 뒤 반드시 재조회한다.
- 2026-07-13 전체 페이지 개편 기준 commit은
  `36902efc9104938559d195853bd8e1ddc5c0f036`이다. DualSense 물리 키매핑은 확정값이 아니라
  HIL·운전자 피드백 뒤 변경 가능한 versioned 초기 후보로 표기한다.
