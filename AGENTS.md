# AGENTS.md

> **이 파일은 `.claude/CLAUDE.md` 의 미러다.** Codex·범용 에이전트가 읽는다.
> 규칙을 바꿀 땐 **두 파일 모두** 동일하게 고친다 — 한쪽만 고치면 다른 에이전트가
> stale한 규칙으로 돈다. 아래 본문은 `.claude/CLAUDE.md` 와 같은 내용이다.
>
> 이전 판(1,084행)에 인라인돼 있던 **Claude 자동메모리 덤프(~850행)와 날짜별
> "CURRENT STATE OVERRIDE" 5겹은 2026-08-24 에 제거**했다. 그 내용은 이미 stale했고,
> 살아 있는 사실은 아래 본문(§2 현재 상태, §6 하드웨어 정본, §8 함정)에 흡수했다.

---


Claude Code(claude.ai/code)가 이 저장소에서 일할 때 따르는 지침이다.

> ⚠️ **짝 파일이 있다.** 같은 내용을 Codex·범용 에이전트용 `AGENTS.md`가 미러한다.
> **규칙을 바꾸면 두 파일 모두 고친다** — 한쪽만 고치면 다른 에이전트가 stale한 규칙으로 돈다.

---

## 1. 프로젝트 개요

ZETIN 6륜 로커-보기(rocker-bogie) 국방/극한 로봇의 **파워트레인 SW**. 사용자 역할은
파워트레인 SW 담당이며, CAD·전장(파워)·로봇팔 본체는 소관이 아니다(인계 요구사항으로만 다룬다).

두 갈래로 나뉜다.

| 갈래 | 위치 | 상태 |
|---|---|---|
| **런타임 제어 SW** | `motor_control/` · `ros2/` · `powertrain_autonomy/` 외 | **활성** — 이 저장소 작업의 거의 전부 |
| 기하 파라미터 최적화 | `parameter_calc/` | ⛔ **트랙 종료(2026-07-19)** — 아래 §2 참조 |

---

## 2. 현재 상태 (2026-09-09 Jetson 검증 반영)

이 섹션이 **유일한 현재 상태 선언**이다. 예전 문서·보고서에 남아 있는 날짜별
"CURRENT STATE OVERRIDE" 문구는 전부 **역사적 기록**이지 현재 권위가 아니다.

### 구현·검증 기록 (실기 재검증일과 구분)

- **WP1–WP5.3** — 코너 모듈 → 4WS 차체 → DDS → chassis_node → 안전 인터록 → 관측성.
  실기 10모터 협조 4WS HIL 통과, 무선 엔드투엔드 검증 완료.
- **WP6-A/B/C** — 지형 추정 코어와 자율주행 컨트롤러. 7/19 Jetson 백엔드 선택은
  **NumPy**로 종결됐다. JAX는 x86 동등성 검증용 실험 커널이다. 선택 근거는 라이브 부하 중
  커널 3케이스 × 100샘플이며, 30분 전체 파이프라인 검증을 뜻하지 않는다.
  근거: `docs/reports/2026-07-19-terrain-backend-jetson-qualification.md`.
- **WP6-S** — 시뮬레이터 중립 시나리오 계약·리플레이.
- **WP7** — 추종(follow) 컨트롤러. **WP8** — 구간 감독자 골격.
- **원격 운용** — ops 채널(:9001) 게이트, 운용 콘솔, L515 Gateway, SRT 2화면.
- **통합 운용 경로(2026-09-08)** — 최초 준비 후 `scripts/robot-start` +
  `python -m operator_console`, 인증 세션·패드 자식 프로세스·동적 UDP 목적지·명시적
  길게 누르기 운전 시작을 추가했다. 9/9 Jetson 별도 배포·ROS 재빌드·실제 콘솔 수신과
  가상 모터 ROS 전체 루프를 검증했다. 현재 인수 범위는 **원격주행**이며 미완성 자율주행은
  테스트 판정 범위에서 제외한다(9/9 사용자 지시). 선 재연결 후 CAN 구동 6축 조회와
  조향 4축 상태 수신은 통과했고 버스/모터 오류는 0이다. 실물 구동·제동은 별도 인수다.
  US-100·L515는 사용자 의도적 분리로 이번 CAN 시험의 센서 결함 판정에서 제외한다.
  `docs/integrated-operation.md`와
  `docs/reports/2026-09-09-integrated-jetson-validation.md`를 따른다.
- **CAN 간섭 결함 수정(2026-09-09)** — CAN/USB 단일 소유권, 소유자 정지 래치 후
  직렬 리셋, 6축 실제 state8 확인, 수신 시각 기반 피드백, GUI 대상 전환·실패 ACK,
  USB/NVM 명시 대상과 저장 전후 대조를 적용했다. Humble에서 DDS 수신 시각을 보존하는
  차체 전용 executor와 CAN 상태 진단용 ROS 이미지 iproute2 의존성도 포함한다.
  설치 ROS+가상 CAN 전체 루프는 통과했다. 실제 can0는 최초 10모터 수신·오류 0이었으나
  배포 후 **13·14가 재소실**, 제어/워치독 정지 후에도 **8/10**이다. 실물 CAN 안정성
  인수는 미통과이며 과거 간헐 무응답의 단일 원인·실물 주행·NVM도 미인증이다.
  신규 유휴 조회가 보드 통신의 지속 오류를 유발했을 가능성도 미배제다. 프로세스 종료 후
  지속된다는 사실만으로 하드웨어 고장에 귀속하지 않는다.
  `docs/reports/2026-09-09-can-sw-causality-review.md`에 세 버전 송신 패턴 복기를 남겼다.
  근거: `docs/reports/2026-09-09-can-remediation.md`.
- **USB 스키드 조향 레이어**(2026-08-05 작성, **실기 주행 확인 후 2026-08-24 main 병합**) —
  `DriveOdriveUsbAxis` USB 다보드 구동 → `skid_geometry()` → 애커만↔스키드 런타임 전환
  → ops 액션 `steer_mode_skid` → 콘솔 배지 → 젯슨 원클릭 배포 스크립트.
  can0가 죽었거나 조향이 없어도 구동 6축만으로 굴릴 수 있는 경로다.
- **PDIST80B 플래그 정본**(2026-08-24) — 매뉴얼 V1.7 p.16 PID 238 기준
  `battery & 0xFC` / `protection & 0x0F`만 fault. 관측되던 `0x02`(자동 충전기 결합)와
  `0x20`(외부 제어)은 **정상 상태 비트**라 충전 중 상시 오경보가 났었다.
  근거 PDF가 `docs/PDIST_사용자매뉴얼_V1.7.pdf`로 저장소에 동봉돼 있다.

### 종료·폐기된 트랙 (건드리지 말 것)

| 트랙 | 결정 | 의미 |
|---|---|---|
| `parameter_calc/` | 2026-07-19 **종료** | 최종 CAD 확정. 버그를 찾아도 **보고만** 하고 f_opt 0.2004 재계산은 하지 않는다. 계산 질량 **50 kg 확정**(86 kg 재최적화 없음). |
| `powertrain_sim/` (MuJoCo) | 2026-07-24 **폐기** | 읽기전용 레거시. 시뮬은 별도 레포 **Isaac(`power-train-sim`)만** 쓴다. 여기 수치는 역사적 앵커로만. 8/24 기준선의 17건 실패는 이 폐기 트랙 안이다. |
| `motor_control/drive/x2212_test/` | deprecated | BL70200 도착 전 임시 테스트 모터. ODrive CAN 일반 실험 데이터만 유효. |
| `drive/bl70200/archive/` | 2026-07-19 **import 하드스톱** | `odrive_calibration.py`·`odrive_diff_drive_test.py`가 pp=5/cpr=30/UV 8V를 NVM에 써서 격리했다. |

### 남은 일

**런타임 연결 작업과 하드웨어·팀 계약 게이트가 함께 남아 있다.**

- SW 연결: autonomy 전용 Compose는 의도적으로 idle이며 배포 발행 경로가 아니다.
  receiver feedback은 노트북 송신·정책 코어까지 있고 Jetson 수신·프로파일 적용은 미연결이다.
  부팅 자격 게이트는 기본 OFF이며 실드라이버의 자격 증거 연결·검증이 필요하다.
- 벤치/HIL: 마운트 각도, 프로파일 프리셋, NumPy를 포함한 전체 파이프라인 장시간 부하,
  원격 영상 E2E, 지상 제동·최종 `stop_mm`, USB 스키드의 지연·안전·오도메트리 커미셔닝.
- 캘리 NVM: 운용 정본에는 node 11/12의 전원사이클 3회 직진입과 나머지 4축의 전원
  재인가 후 재캘리 없는 운용이 기록돼 있다. **13~16의 동일한 3회 반복시험·USB 플래그
  직접 대조는 별도 미완료**다. 이번 문서 대조로 하드웨어 상태를 새로 인증하지 않는다.
- 팀 계약: 로봇팔 실 인식 이벤트 토픽, `MISSION_STOP` 잠금 해제 순서,
  `ARRIVED_* → arm work → DONE → resume` 전체 핸드셰이크 1회.
- 지상 측정(오도메트리 5 m/90°, `stop_mm`)은 조립·운용 조건 확인 후 수행한다.

### 브랜치·동기화

- `main`의 동기 여부는 작업마다 확인한다. 특정 날짜의 동기 상태를 상시 사실로 쓰지 않는다.
- `feat/l515-aligned-depth-slam`은 팀원 작업이므로 손대지 않는다.
- `agent/operator-console-live-telemetry`의 **PR #4는 9/8 확인 시 OPEN/DRAFT**다.
  환경 센싱 SSH JSONL → UDP :5008 → 콘솔 탭은 이 PR의 기능이며 현재 main에 없다.

⚠️ **작업 착수 전 GitHub와 젯슨 로컬 체크아웃을 둘 다 확인한다.** 팀원이 젯슨에서 직접
작업하고 안 올렸을 수 있다. "머지했다"는 구두 정보는 실제로 검증한다.
`git fetch --all` / `ssh jetson 'git status; git log @{u}..'` / `gh pr list`.
**팀원의 미추적·미커밋 파일은 절대 보존한다.**

---

## 3. 저장소 구조

```text
power-train-sw/
├── .claude/CLAUDE.md   이 파일 (Claude Code 용)
├── AGENTS.md           이 파일의 미러 (Codex·범용 에이전트 용) — 함께 갱신할 것
├── README.md           레포 소개 + 레포영역 ↔ Notion 페이지 매핑표
│
├── motor_control/      ★ 하드웨어 소유권 + 순수 파이썬 제어·안전 정책
│   ├── drive/bl70200/    BL70200 + 내장 HALL ×3 (실전 구동). 정본 = bl70200_setup.py
│   │   ├── archive/      ⛔ import 하드스톱 (NVM 오염 스크립트 격리)
│   │   └── x2212_test/   ⛔ deprecated (구 테스트 모터)  ※ drive/ 하위
│   ├── steering/         AK45-36 조향 (CAN). 메인 = ak_control.py
│   ├── corner_module/    코너 1개(조향+구동) 협조 제어 + 트랜스포트 무관 Actuator ABC
│   ├── chassis/          4WS 차체 통합 — kinematics · ChassisManager · teleop_server
│   ├── safety_us100/     US-100 충돌방지 판정 (publish-only)
│   ├── sensors/          US100 UART 드라이버
│   ├── vision/           검출·스트리밍 (gst_stream · yolo_depth_3d · realsense_*)
│   └── laptop/ pi/       1:1 짝 TCP 텔레옵 클라이언트/서버
│
├── ros2/               ★ ROS2 워크스페이스 — **얇은 어댑터 층**
│   └── src/
│       ├── powertrain_ros/    노드 22종 (chassis · us100_safety · odometry · imu_tilt ·
│       │                      autonomy_controller · lane_follower · lead_follower ·
│       │                      section_supervisor · ops_broker · arm_console_bridge ·
│       │                      pdist80b_monitor · l515_cloud · mission · … )
│       ├── powertrain_msgs/   SafetyVerdict · WheelState · WheelStates
│       └── robot_arm_msgs/    벤더링 사본 (정본 = 로봇팔팀 ksp118/extreme-robot)
│
├── powertrain_autonomy/       WP6 자율주행 **순수 코어** — 지형 추정·컨트롤러.
│                              ROS·rclpy·하드웨어·시뮬 분기 없음. powertrain_ros를 import 안 함
├── powertrain_observability/  진단 이벤트·헬스 순수 코어
├── remote_video/              원격 영상 **수신측** 계약
│
├── operator_console/   운용 PC GTK 콘솔 — SRT 2화면 + UDP 텔레메트리.
│                       헌장: **관측 수신 전용**, 조작은 게이트된 ops 채널(:9001) 경유만
├── l515_dashboard/     L515 Gateway(카메라 단일 소유) + 소켓 전용 Textual 대시보드
├── motor_gui/          웹 모터 진단·튜닝 GUI (FastAPI + 트랜스포트 추상화)
│
├── powertrain_sim/     ⛔ MuJoCo 시뮬 — 폐기·읽기전용 (§2 참조)
├── parameter_calc/     ⛔ 기하 최적화 — 트랙 종료 (§2 참조). 상세는 parameter_calc/CLAUDE.md
│
├── docker/             x86 dev + Jetson Orin Nano 배포 컨테이너 정의
├── scripts/            호스트 헬퍼 — recv_stream.sh · can_setup.sh · deploy_*.sh ·
│                       pdist80b_view.py · systemd 유닛·udev 아티팩트
├── tools/  config/     보조 도구 / 보드 레지스트리
├── tests/              최상위 통합·계약 테스트 (패키지 경계를 넘는 것만)
└── docs/
    ├── specs/          기능별 설계 문서       ├── plans/    구현 계획·검증 로그
    ├── reports/        진행 보고·결과 로그    ├── superpowers/  스킬 워크플로 산출물
    ├── patent/         특허 백서             └── hil_data/ · ui_screenshots/ 등
```

⚠️ 위 구조에서 **`motor_control/`이 하드웨어와 정책을 소유하고, `ros2/`는 껍데기**라는
방향성이 핵심이다. 제어·안전 정책과 can0·10모터 단일 소유권은 순수 파이썬
`SafetyInterlock`·`ChassisManager`에 있다. 블로킹 가능한 US-100 UART는 별도 프로세스에서만
돌려 50 Hz 차체 tick을 늦추지 않는다. **`powertrain_autonomy`가 `powertrain_ros`를
import하면 안 된다**(의존 방향은 ROS 어댑터 → 순수 코어). 마찬가지로
**`motor_control`이 `motor_gui`를 import하면 안 된다.**

---

## 4. 테스트·검증

### 실행 환경 우선순위

실제 실행·검증은 **Jetson Orin Nano에서 직접** 돌리는 것을 우선한다(런타임 타깃이 젯슨).
x86 노트북 dev 컨테이너(`powertrain-sw:dev`, `docker/docker-compose.yml`)는 차선이며,
"무하드웨어 전용"이 아니라 **ODrive를 노트북에 USB 직결해 실제 모터를 굴리는 작업까지 포함**한다.

⚠️ ODrive USB reset ioctl 때문에 `docker run --privileged`로 띄워야 연결된다 —
`cap_add`만으론 I/O 에러. odrive 파이썬 라이브러리는 젯슨과 동일한 git `fw-v0.5.6` 소스로
맞춘다(PyPI엔 0.5.6 미배포). x86 이미지는 **CPU 전용** — YOLO GPU 추론은 젯슨에서만.

### ⚠️ 호스트에서 pytest 돌리는 법 — PYTHONPATH가 필수다

여러 패키지가 `chassis`·`powertrain_ros`를 import한다. **PYTHONPATH 없이 돌리면 수집
에러가 나고, 그걸 "테스트 실패"로 오진하기 쉽다.**

```bash
cd <repo>
export PYTHONPATH="$PWD/motor_control:$PWD/ros2/src/powertrain_ros"

# 대부분: python-can·pyserial 이 설치된 인터프리터
python -m pytest motor_control -q

# operator_console 만: PyGObject(`gi`) 가 있는 인터프리터 — conda 환경엔 보통 없어 시스템 python 을 쓴다
/usr/bin/python3 -m pytest operator_console -q
```

### 기준선 (2026-08-24 실측, 호스트)

| 스위트 | 인터프리터 | 결과 |
|---|---|---|
| `motor_control` | conda | **824 passed** |
| `l515_dashboard` | conda | **311 passed** |
| `operator_console` | 시스템 (`gi`) | **274 passed** |
| `powertrain_autonomy` | conda | **199 passed** |
| `motor_gui` | conda | **135 passed** |
| `scripts/tests` | conda | **106 passed** (`test_recv_yolo3d.py`는 `cv2` 필요 → 컨테이너) |
| `powertrain_observability` | conda | **66 passed** |
| `remote_video` | conda | **30 passed** |
| `tests/` | conda | 45 passed / **2 failed** — 매니페스트 체크섬 드리프트(기존 결함) |
| `powertrain_sim` | conda | 130 passed / **17 failed** — ⛔ 폐기 MuJoCo 트랙 |
| `ros2/powertrain_ros` | `rclpy` 필요 | 호스트 불가 → **젯슨/컨테이너에서만** |

호스트 합계 **1,945 passed**. 아래 2건은 알려진 기존 실패이며 회귀가 아니다:
`tests/test_environment_manifest.py`·`test_fixture_class_contracts.py`의
`procedural-dev-0/analytic` 체크섬 불일치.

---

## 5. 완료 선언 규칙 — 실행·실전 경로 검증 필수 (2026-07-18 사용자 지시)

**뭔가 만들었으면(코드·노드·GUI·스크립트·유닛) done 선언 전에 ① 실제로 실행하고
② 실전과 같은 경로로 E2E를 한 번 쫙 돌린다.** 단위테스트 green + diff 리뷰만으로 완료
선언 금지 — 실행 시에만 드러나는 결함(콜백 예외 삼킴, 속성 오타, 환경 의존)이 반드시
있다고 가정한다.

- **콘솔**: `/usr/bin/python3 -m operator_console.runtime_smoke`
  (Xvfb 실기동 + 4채널 LIVE 주입 + traceback 검출) 필수.
- **ROS 노드/서비스**: 젯슨에서 **설치된 엔트리포인트**(`ros2 run …`)를 도메인 77 격리로
  실기동하는 스모크(fixture 발행 → 실응답 단언 → killpg).
- **배포 유닛/컨테이너**: 배포 후 `systemctl is-active` + journal 무크래시 확인
  (크래시루프는 조용히 돈다 — 07-18 sender 5,586회 사고).
- **검증 게이트를 새로 만들면 음성 대조**(알려진 버그 재주입 → FAIL 확인)로 게이트 자체를 증명한다.

근거: 07-18 콘솔 3연속 기동 실패(gi 부재 → gtksink 부재 → `_rss` AttributeError —
전부 스위트 green 상태에서 사용자가 직접 맞음).

---

## 6. 하드웨어 정본

- 6륜 로커-보기. **v4 최적화 기준 반지름 100 mm·질량 50 kg**과 제작 v2 기하를 구분한다.
  런타임 `default_geometry()`의 v2 반지름은 **103.56 mm**, 앞/중/뒤 윤거는
  **545/719/425 mm**다. 축간거리는 CAD 기준 **875.5 mm**, 반올림한 런타임 좌표
  `x = ±0.4377 m` 기준 **875.4 mm**다. 정본은 `chassis/kinematics.py`다.
  설계 기본 속도 0.80 m/s와 원격 수동운용 설정 1.5 m/s·1.2 rad/s도 별도 값이다.
- **구동**: BL70200 + 내장 HALL ×3 — **pp=10, cpr=60**(2026-06 실측. 구문서의 pp=5/cpr=30은
  오기) ×6. ODrive v3.6 **듀얼축 보드 3장** = CAN node 11/12 · 13/14 · 15/16. 감속 1:5
  (모터 5회전 = 바퀴 1회전) — `DriveOdriveCan`이 바퀴 단위로 변환.
- **조향**: CubeMars **AK45-36** ×4 = CAN id 1~4 (36:1, peak 24 Nm, 정격 8 Nm, KV80,
  백래시 12 arcmin). 레거시 테스트용 AK40-10과 API 동일(`ak_control.py`의 `MOTOR_PROFILES`).
- **버스**: 단일 `can0` @ **500 kbps**, AK ×4 + ODrive ×6 = **10모터**.
- **ODrive 게인 정본**: pp10 / cpr60 / **bw30 · vel_gain 0.12 · vel_int 0.2** /
  `ignore_illegal_hall_state=True` / 48V UV40. 셋업은 `bl70200_setup.py`,
  일괄 캘리는 `can_calibrate_all.py`.
- **센서 분리(2026-07-07 확정)**: **L515 = 파워트레인 RGB/depth/IMU**,
  **D435i = 로봇팔 전용**, **US-100 = 독립 충돌 안전**. 파워트레인은 D435i 원본을 직접
  점유하지 않고 `/detected_objects`를 구독한다.
- 펌웨어 ODrive v0.5.x(CAN 트랙 fw-v0.5.6 검증). 폐루프 진입 전
  `input_pos = 현재위치`(위치모드) 또는 `input_vel = 0`(속도모드)로 점프 방지.

---

## 7. 트랙별 작업 안내

### `motor_control/`

**같은 ODrive에 트랙을 절대 섞지 말 것**(캘리·게인·전류한계가 갈린다).

- **drive/bl70200/** — 정본 셋업 `bl70200_setup.py`
  (`--read --serial <SERIAL>`, 쓰기는 `--apply`/`--calibrate`와
  `--serial <SERIAL> --axis both --node 11` 필수, 보드별 좌측 node는 11/13/15).
  CAN 다축: `can_calibrate_all.py`(node 11~16 일괄 풀캘리 — **이 명령은 RAM만 갱신하며
  NVM 저장은 하지 않는다**), `can_drive_test.py`(6축 동시 주행). 벤치 전용 USB 직결 텔레옵:
  `dualsense_usb_teleop.py`(⚠️ 안전 게이팅 전무 — 바퀴 들고 쓸 것).
  - ⚠️ **우측 구동축 미러 장착(2026-07-28 실물 확인)**: 각 보드 M1축 = node 12/14/16 =
    로봇 오른쪽 바퀴이며 좌측과 반대로 돌아야 정방향. `DriveOdriveCan(invert=True)`가
    **CAN 프레임 경계에서만** 부호를 뒤집고, 드라이버 바깥(chassis·odometry·텔레메트리)은
    전부 바퀴 프레임 "+=전진"이다. **raw 스크립트와 `motor_gui`의 회전 방향 부호는 모터 기준**이다.
    GUI의 ODrive 속도는 backend에서 감속비를 반영한 휠 rev/s이며, 우측 장착 방향 자동 반전과는
    별개다. `can_drive_test.py` 전진 = 좌(11/13/15)+ / 우(12/14/16)−, 제자리선회 = 6축 전부 +.
  - **NVM 영속화**: `bl70200_setup.py --persist-calibration --serial <SERIAL> --axis both --node 11`로
    보드별 양축 캘리 상태 확인 후 저장한다(node는 해당 보드의 11/13/15). 저장 여부를 가정하지 말고 시작 전 오류·준비
    상태를 확인하며, 미준비 축은 바퀴를 든 벤치에서 재캘리한다. 축별 검증 범위는 §2와
    [운용 정본](https://www.notion.so/3b02d27b08d381d99641e3565fe40ca2)을 따른다.
- **steering/** — `ak_control.py`(python-can socketcan 직접 제어. 위치제어 슬루
  `DEFAULT_SPD_ERPM=4500` ≈ 출력축 47°/s·45° 0.85 s, `DEFAULT_ACC_ERPM_S2=20000`),
  `calibrate_ak.py`(기어비 1회성), `status_ak.py`(CAN RX 디버깅).
  사전 준비 `bash scripts/can_setup.sh`.
- **corner_module/** — `CornerModule`(상태머신·워치독·estop·과전류 트립·폐루프 점프방지),
  `Actuator`/`SteerActuator`/`DriveActuator` ABC, 드라이버
  `steer_ak40`·`null_steer`·`drive_odrive_usb`·`drive_odrive_can`(CAN 정본)·
  `DriveOdriveUsbAxis`(USB 다보드), `fake.py` 테스트 더블.
- **chassis/** — `kinematics.py`(차체 (v,ω) → 바퀴별 조향각·속도, 애커만 + 자동 클램프 +
  `skid_geometry()`), `chassis_manager.py`(코너 6개 통합, estop 전파·US-100 게이팅·워치독,
  `build_real_corners()` / `build_usb_skid_corners()`),
  `teleop_dualsense.py`(유선), `teleop_server.py`(무선. `--skid-usb`로 USB 스키드).
  - **min_drive_turns_per_s는 2026-07-17 D3/D4로 기본값 전면 0 = 폐지.** 저속 코깅 대응은
    `DriveOdriveCan`의 `friction_ff`/`v_knee`(torque_ff 피드포워드, 기본 off)로 대체.
  - **ops 채널 :9001** — 복구·운용 명령 단일 게이트 `ops_broker`. 역할 토큰
    (`/etc/powertrain/ops_*.token`), 4상태 ACK, 멱등 재전송, 비상 2단계 서버 검증
    (reset 5 s / arm 3 s). 복구 chord는 `recovery-v1-initial-candidate`로 **전부 임시**다.

### `ros2/`

로봇팔 팀(`ksp118/extreme-robot`)과 **분리 개발**한다. 각 팀이 자기 노드·컨테이너를
소유하고 `robot_arm_msgs` 계약만 공유하며 DDS(host network)로 통신한다.
`ros2/scripts/sync_check_msgs.sh`로 벤더 msg 드리프트를 검사한다.
결합 기동 런치는 항상 **명시적 `stop_mm`을 요구**한다(생산 기본값 없음).

### `motor_gui/`

`python3 -m motor_gui.backend.server --track {fake|usb|ak|odrive_can|can}`
(FastAPI, `http://<host>:8000`, network_mode host). 핵심은 `backend/transport/`의
`Transport`/`CanDevice` ABC — AK·ODrive를 컴포저블 디바이스로 묶어 한 can0에서 다중
디바이스 운용. 신규 디바이스는 ABC 구현 후 `worker.py`(100 Hz 샘플)에 드롭인.

### `l515_dashboard/` — Gateway 단일 소유권

- 생산 L515 접근은 **오직** `python3 -m l515_dashboard.gateway_main`(powertrain_ros 안)만 갖는다.
- 대시보드(`python3 -m l515_dashboard`)는 same-UID 보호되는 리눅스 **추상 소켓**
  `@powertrain-l515-gateway`로 붙는다. `q`·SIGHUP·클라이언트 죽음은 Gateway·ROS·SRT를
  살려둔다 — 확인된 `Shift+Q`만 `stop_gateway`를 보낸다.
- 싱글턴 소유권은 영속 `/run/powertrain/l515-gateway.lock` + `flock`이다.
  ⚠️ **stale해 보여도 그 파일을 지우지 말 것.** `powertrain_ros`가 호스트
  `/run/powertrain`을 같은 경로로 bind-mount하고 host networking을 쓰므로, 중복 컨테이너도
  두 소유권 네임스페이스를 공유한다. 기동 순서는 guard → 추상 서버 → SDK/ROS/SRT이고,
  중복 bind는 카메라를 건드리면 안 된다.
- 호스트에서 `powertrain_ros`를 처음 compose 배포하기 전에
  `sudo bash scripts/install_powertrain_runtime_dir.sh`를 돌린다(재부팅 후
  root:root 0750 `/run/powertrain`을 되살리는 systemd-tmpfiles 규칙). compose는
  `bind.create_host_path: false`다 — ⚠️ 없는/불안전한 런타임 디렉토리를 수동 0755 생성으로
  우회하지 말 것.
- 승인된 RealSense 직접 정비 전에는 Gateway를 명시적으로 정지시킨다.
- ⚠️ **Orin Nano SRT는 의도적으로 소프트웨어 `videoconvert → x264enc`를 쓴다**
  (NVENC 없음). `nvv4l2h264enc`를 가용한 것처럼 노출·문서화하지 말 것.
- 성능 정본: `docs/reports/2026-07-12-l515-gateway-performance-hil.md` — RGB는 x264
  ultrafast/3 threads에 alignment 억제, raw Depth는 deadline 기반 10 Hz, Depth/오버레이
  SRT는 best effort. 감독 재시작은 Compose로 프로세스를 교체하며, RSUSB 파이프라인을
  in-process로 재사용하지 않는다.

### `operator_console/`

**관측 수신 전용**이 헌장이다. 조작은 게이트된 ops 채널(:9001 역할 토큰) 경유만 —
ConfirmFlow 2단 확인 패널 + 송신표면 계약 테스트로 봉인돼 있다.
컴포넌트 4종(drive/steer/us100/robot_arm) 토글은 무영속(재시작 = 전부 ON)이고,
OFF = 미장착 모드(estop/hold 비체결, 모터는 IDLE 한정), us100 OFF 시 SAFETY DISABLED 배너.

---

## 8. 함정 (footgun) 모음

- ⚠️ **모터 테스트 전 좀비 프로세스를 죽인다.** 안 끈 teleop/제어루프가 v=0을 계속 명령해
  새 테스트와 싸우고, surging·undershoot·정지로 오진단된다(2026-07-05 반나절 소모).
  `docker exec … ps | grep -E 'teleop|chassis'` 확인 후 pkill.
- ⚠️ **바퀴 지령 <0.3 rev/s는 HALL 코깅존** — 실물이 멈춰 있는데 텔레메트리만 그럴듯하다.
  테스트는 v ≥ 0.4 m/s + **실물 육안 확인** 필수.
- ⚠️ **can0 LOOPBACK이 sticky**하게 걸리면 down/up으로 안 풀린다 →
  `ip link set can0 type can loopback off` 명시 필요(안 그러면 버스 무음인데 가짜 self-ACK로
  모든 baud에서 ACK 성공 → 진단 통째로 오염).
- ⚠️ **CAN PWM 노이즈 TX 웻지**는 절연 트랜시버(ADM3053) 교체로 **종결**됐다
  (최악 정지 27.9% → 0.0%). 워치독은 보험으로 상주 유지 —
  compose `canwatchdog` 서비스 + 텔레옵 인프로세스 내장 + `scripts/can_watchdog.sh`.
  전말: `docs/specs/2026-07-07-can-pwm-noise-tx-wedge.md`.
- ⚠️ **`motor_gui` usb 트랙 기본값이 BL70200 NVM을 오염시킨 사례**가 있다(X2212 게인이
  들어가 current_lim 100A·vel_int 0 → 캘리 OK·state 8인데 부하에서 안 돎).
  BL70200 셋업 §2 전수 대조로 복구한다.
- ⚠️ **젯슨 모터/CAN 파이썬은 호스트가 아니라 컨테이너 안에서** 돈다(python-can이 컨테이너에만
  있다). host network + privileged로 can0 공유, repo = `/workspace`.
- ⚠️ **zsh에서 `ls`는 `eza` alias**다 — `ls -v` 같은 GNU 플래그가 다르게 동작한다.
  스크립트에선 `find`나 `command ls`를 쓴다.

---

## 9. 문서·노션 규약

### 저장소 문서

- 문서는 **파워트레인 SW 담당 관점**으로 쓴다. CAD·전장은 인계 요구사항으로만 다루고,
  팀 전체 발표 전략·일정으로 범위를 넓히지 않는다.
- 새 기능은 `docs/specs/`(설계) → `docs/plans/`(구현 계획·검증 로그) →
  `docs/reports/`(결과) 순으로 남긴다.

### 노션

기능별 사용법은 팀 Notion 허브 `극한로봇 파워트레인 → 💻 Software`에 정리한다
(레포 `README.md`의 매핑표가 레포 영역 ↔ Notion 페이지 인덱스 정본).
⚠️ **기존 페이지 무단 수정 금지** — 신규 SW 문서 생성은 OK.

새 SW 페이지는 **`📄 SW 문서 표준 템플릿`**을 복제해 작성한다. 표준 구조:
개요 콜아웃 → 목차 → ①환경 → ②핵심개념/파라미터 → ③배선 → ④설치·사전준비 → ⑤실행
→ ⑥트러블슈팅 표 → ⑦검증결과 표 → ⑧코드·참고.

- **④설치·⑤실행은 "젯슨에 SSH 접속 직후(홈 `~`)" 기준**으로, 순서대로 복붙만 하면
  목표를 달성하도록 쓴다(레포 이동 → 호스트 준비 → 컨테이너 진입까지 포함.
  컨테이너가 떠 있음·CAN이 올라옴 같은 중간 상태를 가정하지 않는다).
- **초보자 복붙 기준**: 각 명령 블록에 **어디서**(노트북/호스트/컨테이너/odrivetool) 치는지와
  **✅ 기대 출력**을 적는다. odrivetool을 쓰면 켜는 법도 명시.
- **노션엔 풀 파이썬 소스코드를 넣지 않는다** — 풀 스크립트는 레포에 올리고 **파일 경로·이름만**
  적는다. 노션에 직접 쓰는 코드는 odrivetool 인터랙티브 한 줄짜리
  (`odrv0.axis1.controller.input_vel = 1.0` 식)만. bash 준비명령·프로토콜 표·수치·실행
  명령은 노션에 둬도 된다.
- 콜아웃 색: 파랑 = 개요, 빨강 = 안전/위험, 회색 = 팁/함정. **함정은 ⚠️ 명시.**
- 구버전은 삭제 대신 상단 ⛔ DEPRECATED 콜아웃 + 정본 링크 후 Archive로 이동.

### 전체 계획 정본

팀 공용 설명본은 Notion **「2026 국방로봇 자율주행 SW 전체 개발계획」**
(`39c2d27b-08d3-8172-8c1a-de21cc72216b`)이다. 세부 기술 정본은
`docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`,
`docs/plans/2026-07-13-wp5.2-arm-collaboration-safety-plan.md`,
`docs/plans/2026-07-13-observability-data-quality-remote-assist-plan.md`.
노션을 수정할 땐 세 문서의 범위·의존 순서·acceptance를 함께 동기화하고 **쓰기 뒤 반드시
재조회**한다. DualSense 물리 키매핑은 확정값이 아니라 HIL·운전자 피드백 뒤 변경 가능한
versioned 초기 후보로 표기한다.
