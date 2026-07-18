# Power Train SW

ZETIN 6륜 로커-보기(rocker-bogie) 방위 로봇의 파워트레인 SW 저장소. 두 트랙은 서로 독립적으로 동작한다.

| 트랙 | 폴더 | 역할 |
| --- | --- | --- |
| 파라미터 최적화 | `parameter_calc/` | 형상 파라미터 최적화 (v4: 15차원·7지형, MATLAB / NumPy / JAX-CUDA) |
| 모터 제어 | `motor_control/` | ODrive 구동 · AK 조향 · 코너 모듈 · YOLO/RealSense 비전 · US-100 · 노트북-Pi 텔레옵 |

> `parameter_calc/`는 개발 서버 검증본을 그대로 옮긴 것 — 결과물(`*.pkl`)을 신뢰할 수 있는 기준 코드.

이 README는 **저장소 소개와 개발 환경 셋업**만 다룬다. 각 기능을 실제로 써보는
방법은 팀 Notion 문서에 정리돼 있다 (아래 [기능별 문서](#기능별-문서--notion)).
스크립트 단위의 개발자용 상세는 [`.claude/CLAUDE.md`](.claude/CLAUDE.md) 참고.
2026-07-10 기준 완료 상태·Jetson 실측·다음 작업은
[`docs/reports/2026-07-10-project-and-jetson-state.md`](docs/reports/2026-07-10-project-and-jetson-state.md)를 정본으로 본다.

> **WP5.1 상태 (2026-07-11): HIL 완료.** 기존 `/cmd_vel → 10모터` 실증과 새 US-100·
> fail-safe·실제 50 Hz 결과를 합쳐 완료 판정했다. 실행 HEAD `ec452f6474b6fc57437d576298f2bc954649be42`에서
> `motor_control` 198, `motor_gui` 91, Jetson `powertrain_ros` 32/32가 통과했다.
> ODrive 13·14는 이번 실행에만 일시 부재했고 기존 정상 이력이 있다. 지상 제동과 최종
> `stop_mm` 선정은 차체 조립 후 실차 커미셔닝이다. 상세는
> [`WP5.1 HIL 보고서`](docs/reports/2026-07-10-wp5-control-safety-hil.md)를 따른다.

> **WP5.2 (2026-07-14): Task 1~6 + 감사 갭 4건 완료** — 계약 v2·ArmInterlock·RealCanSession·
> CommandAuthority·원격 gateway·MissionSupervisor·DetectionAdapter.
> **WP5.3 (2026-07-16): Task 1~5 완료·젯슨 배포** — mission journal·observability 데몬·CAN
> health matrix·depth/TF qualification·팔 결과 adapter + read-only operator console(PR #2).
> 상세: [`WP5.3 구현 보고`](docs/reports/2026-07-16-wp53-observability-implementation.md).

---

## 저장소 구조

```
.
├── parameter_calc/      형상 파라미터 최적화 (v4 권위본 python_gpu_triangle/, CLAUDE.md 필독)
├── motor_control/       실차 런타임 제어
│   ├── drive/           구동 모터 (bl70200 실전 · x2212_test 테스트, ODrive USB/CAN)
│   ├── steering/        조향 (AK45-36 실전, CAN socketcan can0)
│   ├── vision/          검출·스트리밍 (기존 D435i 실험 코드 + L515 자율주행 입력, 모터 명령 없음)
│   ├── sensors/         US-100 초음파 거리 (UART /dev/ttyTHS1)
│   ├── safety_us100/    US-100 거리·UART 생존 판정 (CHECKING/VALID/INVALID_READING/NO_RESPONSE)
│   ├── corner_module/   코너 1개(조향+구동) 협조 제어 + DualSense 텔레옵
│   ├── chassis/         순수 Python 4WS 제어·SafetyInterlock·10모터 단일 권한
│   ├── laptop/          노트북 측 텔레옵 클라이언트 (velocity·video·chassis 무선)
│   └── pi/              라즈베리파이 측 서버 (laptop/ 과 1:1 짝)
├── motor_gui/           웹 진단·튜닝 GUI (FastAPI + 트랜스포트 추상화, AK/ODrive CAN·USB)
├── l515_dashboard/      L515 Gateway·TUI (단일 SDK 소유 — SRT 송신 + ROS 발행 + Textual 대시보드)
├── operator_console/    운용 PC용 read-only GTK 콘솔 (L515·D435i SRT 뷰 + UDP 텔레메트리 수신)
├── ros2/                얇은 ROS2 층 (US-100 별도 노드, chassis 노드, powertrain_msgs)
├── docker/              컨테이너 정의 (x86 dev + Jetson Orin Nano 배포)
├── scripts/             호스트 헬퍼 (recv_* · can_setup.sh · install_*.sh, systemd/ = 유닛·udev·tmpfiles)
└── docs/                설계(specs) · 계획(plans) · 보고(reports) + 대회 규정 / FSM
```

> 구동 ODrive 는 **듀얼축 보드 3장**(M0=`axis0`+M1=`axis1` 양축 사용) — CAN node 11/12·13/14·15/16.
> 단일 축만 쓰는 레거시/단축 스크립트는 `axis1` 기준.

---

## 개발 환경

코드는 호스트에 두고 컨테이너 `/workspace` 로 bind mount — 이미지를 다시 빌드하지 않고
수정/실행한다.

> **실행·검증은 Jetson 에서 직접 하는 것을 우선**한다(런타임 타깃이 Jetson). x86 노트북
> 컨테이너 테스트(무하드웨어 `pytest` + fake 드라이버)는 **Jetson 에 접근 불가할 때의 차선책**.

### x86 (개발·테스트)

```bash
xhost +local:docker                                         # cv2 창을 호스트로 띄우려면
docker compose -f docker/docker-compose.yml up -d --build
docker compose -f docker/docker-compose.yml exec powertrain bash
```

CPU 전용 이미지(~3.3GB, Ubuntu 22.04 + ODrive · pygame · OpenCV · ultralytics · OpenVINO).
코드 작성 · 단위테스트(`pytest`) · OpenVINO/CPU YOLO 용. YOLO GPU 추론은 Jetson 에서만 한다.

### Jetson Orin Nano (배포)

```bash
git clone https://github.com/lightminn/power-train-sw.git && cd power-train-sw
sudo docker compose -f docker/docker-compose.jetson.yml up -d --build
sudo docker compose -f docker/docker-compose.jetson.yml exec powertrain bash
```

베이스 `dustynv/l4t-pytorch:r36.4.0` (CUDA + cuDNN + TensorRT + ARM PyTorch) + RealSense SDK
(librealsense / pyrealsense2) 소스 빌드 포함. JetPack 의 `nvidia-container-runtime` 으로 추가
설정 없이 동작. **Orin Nano 는 NVENC 하드웨어 인코더가 없어**(Orin NX/AGX 만 탑재) 영상은
SW 인코딩(`x264enc`) + SRT(ARQ 손실복구) 로 보낸다.
Gateway 상태에는 SDK native callback Hz, ROS 6토픽별 Hz, SRT submit/sent/drop Hz,
aligned-depth age, 프로세스 CPU/RSS가 포함된다.
2026-07-12 real-image HIL에서 RGB receiver 29.91 fps, ROS color 30.0~30.2 Hz,
raw Depth 10.0 Hz, SRT drop 0, SDK frame gap 0을 확인했다. 상세는
[`docs/reports/2026-07-12-l515-gateway-performance-hil.md`](docs/reports/2026-07-12-l515-gateway-performance-hil.md)다.

실차 센서 소유권은 **L515=파워트레인 RGB/depth/IMU**, **D435i=로봇팔 인식 전용**,
**US-100=독립 충돌 안전**으로 분리한다. 2026-07-10 Jetson USB에서 L515와 D435i
동시 연결을 확인했다. `motor_control/vision/`의 D435i 코드는 기존 실험·스트리밍 자산이며,
자율주행 신규 ROS 입력은 `powertrain_ros`의 경량 L515 노드가 제공한다.

### L515 경량 ROS 파이프라인

운용 시 L515는 장기 실행 Gateway 하나가 점유하며 ROS 6개 토픽과 SRT를 함께 제공한다.
별도 Textual Dashboard는 same-UID `SO_PEERCRED`로 보호된 abstract Unix socket
`@powertrain-l515-gateway`만 사용하므로 `q`/SSH 종료가 Gateway를 멈추지 않는다.
Jetson `powertrain_ros`는 host `/run/powertrain`을 같은 경로에 bind-mount하고 host network를
사용하므로 중복 container도 동일한 flock inode와 abstract endpoint에서 충돌 후 카메라 접근 전에 종료된다.
최초 1회 `sudo bash scripts/install_powertrain_runtime_dir.sh`를 실행해야 하며, 설치된
systemd-tmpfiles 규칙이 매 부팅마다 root:root 0750 runtime directory를 재생성한다.
실행, 수신기 명령, 키와 singleton 장애 대응은 [`l515_dashboard/README.md`](l515_dashboard/README.md)를 따른다.

파워트레인 L515는 serial `00000000F0271544`만 열며, `powertrain_ros` 이미지에
librealsense/pyrealsense2 **정확히 v2.50.0**을 RSUSB backend로 빌드한다. D435i
`250222071245`는 로봇팔 전용이므로 이 노드가 열지 않는다. 실행 전 Jetson 호스트에서
SDK가 반환하는 `f0271544`는 대소문자·선행 0 정규화 후 같은 장치로 엄격 매칭한다.
`bash scripts/l515_preflight.sh`를 실행한다. 이 게이트는 USB PID `8086:0b64`, 5000 Mbps
이상 링크, 컨테이너 SDK의 지정 serial 선택이 모두 확인되지 않으면 실패한다. 정확한 이미지
빌드·source·launch 명령과 토픽은 [`ros2/README.md`](ros2/README.md)에 있다.

이 경량 파이프라인은 color/depth `Image`·`CameraInfo`와 gyro/accel `Imu`만 발행한다.
**PointCloud2는 설치·생성·발행하지 않는다.**

2026-07-11 HIL에서 분리·자동 재연결과 로봇팔 D435i 동시 실행을 통과했다. 동시 60초
실측은 color 29.750 Hz, depth 29.450 Hz, accel/gyro 30.166 Hz, 모든 5초 창 ≥28.8 Hz,
stamp 비증가 0, USB error delta 0이었다. 다음 소비 계층은 WP6 오도메트리이며 최종 장착 후
`base_link→l515_link` static TF를 실측한다.

## WP5.1 제어·안전 계약

제어·안전 정책과 10모터 소유권은 ROS 없는 순수 Python `ChassisManager`와
`SafetyInterlock`에 둔다. ROS2는 블로킹 UART를 격리한 `us100_safety_node`, 최신 판정
캐시와 50 Hz 제어를 담당하는 `chassis_node`로 구성된 얇은 내부 전송층이다. 실차 버스는
단일 `can0` 500 kbps에서 AK45-36 ×4와 ODrive/BL70200 ×6을 50 Hz로 운용한다.

- US-100 상태는 `CHECKING`, `VALID`, `INVALID_READING`, `NO_RESPONSE`다.
  `INVALID_READING`은 0x50 응답으로 MCU/UART 생존만 확인된 정상 통과 상태이며,
  초음파 송수신부 정상까지 증명하지 않는다.
- `CHECKING`, 0.5초 `/cmd_vel` watchdog, 텔레옵 연결 단절은 원인이 해소되면 자동복구하는
  `MOTION_HOLD`다. 이 명령 watchdog은 아래 0.75초 safety-topic freshness와 별개다. 유효
  근거리와 거리·0x50 생존 확인이 모두 연속 3회 실패한
  `NO_RESPONSE`는 latched `ESTOP`이다.
- `ESTOP`은 원인을 제거한 뒤 reset해야 하며, reset은 `IDLE`까지만 복구한다. 모터 구동에는
  별도 arm이 필요하다.
- `us100_safety_node`는 5~10 Hz로 `/safety_verdict`를 발행하고, `chassis_node`는
  `/wheel_states`를 50 Hz로 발행한다. 생산 기본 안전 토픽 timeout은 0.75초이며
  `age > 0.75 s`가 된 다음 50 Hz tick, 명목상 0.75~0.77초에 E-stop한다. 최초 수신 대기는
  1.0초다.
- `safety_required=false`는 BENCH/FAKE 전용이다. 실기는 항상 기본값 `true`를 사용한다.
- 결합 실기 launch는 `stop_mm` 인자를 생략할 수 없다. 현재 벤치/HIL 값은 200 mm다.
  차체 조립 후 50 kg 실차 지상 커미셔닝에서 제동거리를 측정해 최종 운용값을 튜닝하며,
  이 커미셔닝은 완료된 WP5.1 HIL과 별도다.

Jetson FAKE 관찰값은 60초 3000 samples, mean/min-5s 50.000 Hz, tick p99 0.280 ms,
overrun 0, max interval 21.453 ms, publisher-death E-stop 0.753 s다. start-up E-stop,
far `ARMED/RUN`, near E-stop, far 복귀 뒤 latch, reset→`IDLE` 무암시 arm, 별도 arm도 확인했다.
FAKE 원시 로그는 보존되지 않아 최종 HIL 전 재실행 로그가 필요하다.

ROS 실행·토픽·서비스 표는 [`ros2/README.md`](ros2/README.md), HIL 전제와 기록란은
[`docs/reports/2026-07-10-wp5-control-safety-hil.md`](docs/reports/2026-07-10-wp5-control-safety-hil.md)를 따른다.

### 호스트 사전 준비

| 항목 | 내용 |
| --- | --- |
| CAN | CAN 트랙·조향 사용 전 `bash scripts/can_setup.sh` (can0 500 kbps, mttcan + devmem) |
| CAN 워치독 | **컨테이너 스택에 상주** (`canwatchdog` 서비스, 자동 기동·재부팅 생존) — PWM 노이즈로 bus-off 반복 후 mttcan TX 웻지(전송 영구정지) 감지·복구 (~2s). 텔레옵 진입점에도 내장. 상세: `docs/specs/2026-07-07-can-pwm-noise-tx-wedge.md` |
| ODrive udev | `/etc/udev/rules.d/91-odrive.rules` 있어야 일반 사용자 권한으로 USB 인식 |
| Wayland | XWayland 가 떠 있어야 cv2 창 표시 (`echo $XDG_SESSION_TYPE` 확인) |
| USB 디바이스 | ODrive · DualSense · 카메라는 `/dev` 마운트로 컨테이너에 자동 노출 |

---

## 기능별 문서 → Notion

각 기능을 실제로 써보는 방법(셋업·실행·검증)은 팀 Notion 허브
[극한로봇 파워트레인](https://app.notion.com/p/31d2d27b08d38030832ac73b42ce0c03) 의 💻 Software
섹션에 정리돼 있다.

레포 영역과 Notion 문서를 같은 구조로 맞춰 둔다.

| 레포 영역 | Notion 문서 |
| --- | --- |
| `parameter_calc/` 파라미터 최적화 (v4) | [로커보기 파라미터 최적화 v4 — 결과·주행 애니메이션](https://app.notion.com/p/36b2d27b08d3819b9303d1f8554b0425) |
| `motor_control/drive/bl70200/` ODrive 구동 셋업 | [ODrive(BL70200) 셋업 — 공장초기화→구동](https://app.notion.com/p/3882d27b08d381fcbe3cd0c829687c3a) |
| `motor_control/drive`+`steering/` 단일 CAN 버스 10모터 (AK45-36 ×4 + ODrive ×6) | [단일 CAN 버스 다중모터 독립제어 — AK45-36 조향 ×4 + ODrive 구동 ×6](https://app.notion.com/p/3882d27b08d381efa56bd5fe310e3198) |
| `corner_module/can_watchdog.py`+`docker/` CAN 자동복구 워치독 (PWM 노이즈→TX 웻지) | [CAN 자동복구 워치독 — PWM 노이즈 TX 웻지 해결](https://app.notion.com/p/3952d27b08d381308d0eeafa8242e509) |
| `motor_control/corner_module/` 코너 모듈 (조향+구동 통합) | [코너 모듈 컨트롤러 — 조향+구동 통합 제어 API](https://app.notion.com/p/36b2d27b08d381818b04c1d194bcade1) |
| `motor_control/chassis/kinematics.py` 4WS 애커만 키네마틱스 (WP2) | [4WS 애커만 키네마틱스 — 차체 명령(v, ω) → 바퀴 조향·속도](https://app.notion.com/p/3912d27b08d381a0a452fa4afdc61c45) |
| `motor_control/chassis/` 4WS 차체 통합 제어 (ChassisManager, WP3 — 실기 HIL 완료) | [차체 통합 제어 ChassisManager — 코너 6개를 하나의 4WS 차체로](https://app.notion.com/p/3912d27b08d381e79716e04398e34bd2) |
| `chassis/teleop_server.py`+`laptop/laptop_client_chassis.py` 무선 원격주행 (DualSense 텔레옵) | [무선 원격주행 — DualSense→노트북→젯슨→10모터 4WS](https://app.notion.com/p/39b2d27b08d38140bf8df53fe7661c6c) |
| `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md` 자율주행 전체 계획 (**정본**) | [2026 국방로봇 자율주행 SW 전체 개발계획](https://app.notion.com/p/39c2d27b08d381728c1ade21cc72216b) — 이력: [착수 계획(~07-11)](https://app.notion.com/p/3912d27b08d381af9e8ed16fb08b0840) |
| `l515_dashboard/` L515 Gateway·TUI | [L515 Gateway·TUI — 카메라 단일 소유·SRT 원격주행](https://app.notion.com/p/39a2d27b08d381eb8307fa7d136ad374) |
| `ros2/` RViz 시각화 (벤치 자산) | [RViz 로봇 시각화 — 오도메트리·IMU·장애물 감지](https://app.notion.com/p/39b2d27b08d3815da7c6f46e173d7a8a) |
| `scripts/recv_*` + 운용 콘솔 | [통신 GUI·스트리밍·전원 텔레메트리 — 통합 현황](https://app.notion.com/p/39d2d27b08d3815c907ae8aa338c5fa8) |
| `motor_control/vision/` 기존 D435i 실험·로봇팔 인식 참고 | [RGB-D 카메라 D435i 켜는 법 — 로봇팔 인식용 참고](https://app.notion.com/p/3752d27b08d381619d73d6bc19fc02d2) |
| `motor_control/vision/` YOLO + Depth 3D 좌표 | [YOLO+Depth 융합 — 검출 물체 3D 좌표 추출](https://app.notion.com/p/37b2d27b08d38147b9aceb16268615a8) |
| `motor_control/sensors`+`safety_us100/` US-100 거리 + 충돌방지 | [US-100 초음파 센서 — UART 거리 측정](https://app.notion.com/p/35d2d27b08d380f591b9d6553c6a320d) |
| `motor_gui/` 웹 진단·튜닝 GUI | [motor_gui — 웹 모터 진단·튜닝 GUI](https://app.notion.com/p/3892d27b08d3811eb174e787808db3c2) |
| (Firmware) ODrive 펌웨어 플래시 | [ODrive 펌웨어 플래시 — 보드당 1회](https://app.notion.com/p/33a2d27b08d38002b0f7d21fda39e8d2) |
| (네트워크) GL-SFT1200 전용 AP — 노트북↔젯슨 링크 | [무선 라우터 GL-SFT1200 — 노트북↔젯슨 전용망 셋업](https://app.notion.com/p/38e2d27b08d38122942bff3f12534e58) |

> **구버전(Archive)** — 아래는 위 정본으로 대체됨: [Jetson CAN 모터 제어](https://app.notion.com/p/35d2d27b08d38062bf19f53e5f1c78cf)(AK40-10/SN65HVD230) · [YOLO 실습](https://app.notion.com/p/33a2d27b08d380dfb71bd86f0e3e7aeb)(YOLOv8/x86 PoC) · [모터 원격제어·영상 스트리밍](https://app.notion.com/p/34f2d27b08d380a89272cc20dfcd0f04)(Pi/gstreamer) · [ODrive CAN 제어](https://app.notion.com/p/3622d27b08d38054a4cafb7d9ca78b02)(X2212 엔코더 테스트모터 — BL70200 도입으로 폐기; ODrive CAN 일반은 「AK + ODrive 동시 CAN」 정본).

코드 단위 상세는 레포 in-repo README 도 참고 — `safety_us100/README.md`(충돌방지 모듈 코드).

---

## 기여 가이드

- `parameter_calc/` 수정 전 [`parameter_calc/CLAUDE.md`](parameter_calc/CLAUDE.md) 의 GPU 버그 히스토리 섹션 필독.
- **BL70200 트랙(HALL 모드) / X2212 트랙(엔코더 모드) 을 한 ODrive 에서 번갈아 쓰지 말 것** — NVM 에 남은 캘리 설정이 의도치 않게 적용된다(폭주/과전류). BL70200 복구·대조·적용은 최신 정본 `drive/bl70200/bl70200_setup.py --read/--apply/--calibrate`를 사용한다. 구형 `odrive_calibration.py`·`odrive_diff_drive_test.py`는 pp=5/cpr=30(캘리 스크립트는 UV 도 8V)을 NVM 에 써서 보드를 손상시키므로 `drive/bl70200/archive/` 로 이동하고 import 시 하드스톱으로 막았다(2026-07-19).
- 구동 ODrive 는 듀얼축(M0=`axis0`+M1=`axis1`) 3보드 — 축·node 매핑은 `chassis/chassis_manager.py` 의 `DEFAULT_WHEEL_MAP` 이 기준. 단축 레거시 스크립트는 `axis1`.
- 결과 파일(`*.pkl`, `*.mat`, `*.mp4`, `fig*.png`)은 서버 검증본 — 의도 없이 덮어쓰지 말 것.
- `motor_control/` 스크립트는 독립 실행형 원칙 유지 — 공용 모듈 분리는 사전 합의. `motor_gui` 는 `motor_control` 을 import 하되 역의존 금지.

## 라이선스 / 연락

내부 프로젝트. 외부 공개·재배포 전 ZETIN 측 확인.
