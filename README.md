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

---

## 저장소 구조

```
.
├── parameter_calc/      형상 파라미터 최적화 (v4 권위본 python_gpu_triangle/, CLAUDE.md 필독)
├── motor_control/       실차 런타임 제어
│   ├── drive/           구동 모터 (bl70200 실전 · x2212_test 테스트, ODrive USB/CAN)
│   ├── steering/        조향 (AK45-36 실전, CAN socketcan can0)
│   ├── vision/          검출·스트리밍 (YOLO26 + RealSense D435i, 모터 명령 없음)
│   ├── sensors/         US-100 초음파 거리 (UART /dev/ttyTHS1)
│   ├── safety_us100/    US-100 충돌방지 판정 (publish-only safe/warn/stop)
│   ├── corner_module/   코너 1개(조향+구동) 협조 제어 + DualSense 텔레옵
│   ├── laptop/          노트북 측 텔레옵 클라이언트
│   └── pi/              라즈베리파이 측 서버 (laptop/ 과 1:1 짝)
├── motor_gui/           웹 진단·튜닝 GUI (FastAPI + 트랜스포트 추상화, AK/ODrive CAN·USB)
├── docker/              컨테이너 정의 (x86 dev + Jetson Orin Nano 배포)
├── scripts/             호스트 헬퍼 (recv_stream.sh · recv_yolo3d.py · can_setup.sh)
└── docs/                설계(specs) · 계획(plans) · 보고(reports) + 대회 규정 / FSM
```

> 모든 모터 제어 스크립트는 `axis1` 사용으로 통일.

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

### 호스트 사전 준비

| 항목 | 내용 |
| --- | --- |
| CAN | CAN 트랙·조향 사용 전 `bash scripts/can_setup.sh` (can0 500 kbps, mttcan + devmem) |
| ODrive udev | `/etc/udev/rules.d/91-odrive.rules` 있어야 일반 사용자 권한으로 USB 인식 |
| Wayland | XWayland 가 떠 있어야 cv2 창 표시 (`echo $XDG_SESSION_TYPE` 확인) |
| USB 디바이스 | ODrive · DualSense · 카메라는 `/dev` 마운트로 컨테이너에 자동 노출 |

---

## 기능별 문서 → Notion

각 기능을 실제로 써보는 방법(셋업·실행·검증)은 팀 Notion 허브
[극한로봇 파워트레인](https://app.notion.com/p/31d2d27b08d38030832ac73b42ce0c03) 의 💻 Software
섹션에 정리돼 있다.

| 레포 기능 | Notion 문서 |
| --- | --- |
| 파라미터 최적화 (v4) | [로커보기 파라미터 최적화 결과 (v4) + 주행 애니메이션](https://app.notion.com/p/36b2d27b08d3819b9303d1f8554b0425) |
| 코너 모듈 (조향+구동 통합) | [코너 모듈 컨트롤러 — 조향+구동 통합 제어 API (HIL 검증)](https://app.notion.com/p/36b2d27b08d381818b04c1d194bcade1) |
| ODrive CAN 제어 | [Odrive CAN 제어](https://app.notion.com/p/3622d27b08d38054a4cafb7d9ca78b02) |
| Jetson CAN 모터 제어 | [CAN 모터 제어 on Jetson](https://app.notion.com/p/35d2d27b08d38062bf19f53e5f1c78cf) |
| 원격제어 + 영상 스트리밍 | [모터 원격제어 및 영상 스트리밍](https://app.notion.com/p/34f2d27b08d380a89272cc20dfcd0f04) |
| 비전 — YOLO | [YOLO 실습](https://app.notion.com/p/33a2d27b08d380dfb71bd86f0e3e7aeb) |
| 비전 — RealSense RGB-D | [RGB-D 카메라(RealSense D435i) 켜는 법](https://app.notion.com/p/3752d27b08d381619d73d6bc19fc02d2) |
| 비전 — YOLO + Depth 3D 좌표 | [YOLO + Depth 융합 — 검출 물체 3D 좌표 추출](https://app.notion.com/p/37b2d27b08d38147b9aceb16268615a8) |
| US-100 거리 + 충돌방지 안전 | [US100 초음파 센서 UART 거리 측정](https://app.notion.com/p/35d2d27b08d380f591b9d6553c6a320d) |
| (Firmware) ODrive 세팅 · 통신 | [ODrive 세팅](https://app.notion.com/p/33a2d27b08d38002b0f7d21fda39e8d2) · [통신 방식](https://app.notion.com/p/32c2d27b08d380bab5c6ef6da5d0ae91) |

Notion 문서가 아직 없는 기능은 in-repo README 를 참고한다 — `motor_gui/README.md`(웹
진단 GUI), `safety_us100/README.md`(충돌방지 모듈 코드).

---

## 기여 가이드

- `parameter_calc/` 수정 전 [`parameter_calc/CLAUDE.md`](parameter_calc/CLAUDE.md) 의 GPU 버그 히스토리 섹션 필독.
- **BL70200 트랙(HALL 모드) / X2212 트랙(엔코더 모드) 을 한 ODrive 에서 번갈아 쓰지 말 것** — NVM 에 남은 캘리 설정이 의도치 않게 적용된다(폭주/과전류). 교차 사용 시 `drive/bl70200/odrive_calibration.py` 로 모드 강제 재설정 후 시작.
- 모든 모터 테스트는 `axis1`. axis0 사용 금지.
- 결과 파일(`*.pkl`, `*.mat`, `*.mp4`, `fig*.png`)은 서버 검증본 — 의도 없이 덮어쓰지 말 것.
- `motor_control/` 스크립트는 독립 실행형 원칙 유지 — 공용 모듈 분리는 사전 합의. `motor_gui` 는 `motor_control` 을 import 하되 역의존 금지.

## 라이선스 / 연락

내부 프로젝트. 외부 공개·재배포 전 ZETIN 측 확인.
