# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Language

**Always respond to the user in Korean (한국어).** Documentation, commit messages, and PR descriptions are written in Korean too.

## Requirements (항상 참고)

**Before any design or implementation decision, consult `docs/requirements/요구사항.md`.** It is the consolidated digest of the competition rules, the team's Pipeline architecture doc, and meeting notes (raw sources in `docs/requirements/raw/`).

⚠️ **`docs/`는 `.gitignore`로 통째로 제외돼 있어 clone 직후엔 존재하지 않는다.** 먼저 `ls docs/requirements/`로 확인하고, 없으면 추측으로 대체하지 말고 사용자에게 요청할 것. 레포에 실제로 들어있는 문서는 `project_docs/`다 — `CLAUDE_Plan.md`(통합 개발 계획, §1 메시지 인터페이스·§6 인식 알고리즘), `WORK_STATUS.md`(세션 인수인계 로그), `PHASE3_FSM_설계.md`, `파워트레인_계약_충돌점검.md`, `하드웨어_없이_할일.md`.

Treat it as a **draft, not a fixed spec** — the team is still converging and the docs contradict each other and the current code. Its §8 lists the live mismatches (DOF 5 vs 6, motor model XL430 vs XM540/XC430, direct-servo vs MoveIt2 path, CAN vs UDP/TCP, etc.) — check there before trusting any single number. Priority is a working implementation over faithfully following the docs.

## What this is

ROS 2 Humble workspace for the 2025 극한로봇 (Extreme Robot) competition: a robot arm that visually tracks a target with YOLO and drives Dynamixel servos to follow it. The dev environment is fully containerized so the team shares one identical setup. Documentation and commit messages are in Korean.

## Environment model (important)

**All ROS 2 commands run *inside* the Docker container, not on the host.** The host is only used for `git` and `docker compose`.

- `./ros2_ws` is bind-mounted to `/root/ros2_ws` in the container, so host edits to `ros2_ws/src/` appear instantly inside.
- **Only `ros2_ws/src/` is version-controlled.** Build outputs (`build/`, `install/`, `log/`) are gitignored — each developer runs `colcon build` in their own container.
- Compose files: `docker-compose.yml` (기본) + `docker-compose.gpu.yml` (Jetson GPU override — `-f`로 얹어 쓴다). WSL2 지원은 제거됨. 컨테이너는 `ros2_humble`, `privileged: true` + `network_mode: host` + **`ipc: host`**.
- **`ipc: host`는 파워트레인 연동에 필수다.** 파워트레인은 같은 Jetson의 **별도 컨테이너**에서 ROS 2 노드를 돌리고 DDS로만 통신한다. Fast-DDS는 같은 호스트면 공유메모리(`/dev/shm`)로 데이터를 보내는데 Docker는 컨테이너마다 별도 `/dev/shm`을 준다 → `ipc: host`가 없으면 **discovery는 되는데 데이터가 한 건도 안 오는 조용한 실패**가 난다. 양쪽 컨테이너 모두 필요하다.
- The container runs `privileged`, so host devices (e.g. the Dynamixel USB serial adapter at `/dev/ttyUSB0`, the camera) are reachable without explicit `devices:` mappings — but the hardware must actually be plugged into the host.

## Common commands

Start container + enter (host):
```bash
xhost +local:docker && docker compose up -d
# Jetson GPU 가속까지 쓸 때만
xhost +local:docker && docker compose -f docker-compose.yml -f docker-compose.gpu.yml up -d

docker exec -it ros2_humble bash   # ROS already sourced via .bashrc
```

Build & run (inside container):
```bash
cd /root/ros2_ws
colcon build
source install/setup.bash
```

- Build one package: `colcon build --packages-select <pkg>`
- Resolve missing deps before reporting a build failure: `rosdep install --from-paths src --ignore-src -r -y`
- Rebuild image after a `Dockerfile` change: `docker compose build` then `up -d`.

## Dependency policy

System dependencies go in the **`Dockerfile`**, not ad-hoc `apt install`, so the team's environment stays reproducible. Already installed there: `dynamixel-sdk`, `dynamixel-workbench` (apt), `joint-state-publisher-gui`, **`moveit`, `ros2-control`, `ros2-controllers`** (apt, for MoveIt + mock hardware/controllers), a full `gstreamer1.0` plugin set (`base`/`good`/`bad`/`ugly`/`libav` + dev headers, for `stream_node`'s SRT streaming), and `ultralytics` + `numpy<2` + `pyrealsense2` + `onnx`/`onnxslim`(TensorRT export path, below) via pip (with `opencv-python` uninstalled so it doesn't clash with ROS's `cv_bridge` OpenCV).

**GPU 가속(CUDA/cuDNN/TensorRT)은 pip/apt로 컨테이너 안에 다시 설치하지 않고, 호스트 JetPack에 이미 있는 걸 `docker-compose.gpu.yml`로 마운트한다** (이미지 용량·빌드 시간 절감 + 호스트 드라이버와의 버전 불일치 회피):
- `torch`/`torchvision`만 예외 — Dockerfile에서 `pypi.jetson-ai-lab.io/jp6/cu126` 인덱스로 JetPack 6.2(CUDA 12.6) 호환 빌드를 `--no-deps`로 설치(PyPI 기본 torch는 더 최신 CUDA로 빌드돼 있어 `torch.cuda.is_available()`가 `False`로 떨어지고 YOLO가 CPU 폴백된다 — 2026-07-22 실측·수정, `torch.cuda.is_available()=True`/`device='Orin'` 확인됨).
- `CUDA_HOME`(기본 `/usr/local/cuda-12.6`) + `CUDNN_LIBS`(기본 `~/.cudnn-libs`, cuDNN 9.3 `.so` curated 디렉터리) + `TENSORRT_LIBS`/`TENSORRT_PYTHON`(기본 `~/.tensorrt-libs`/`~/.tensorrt-python`, TensorRT 10.3 `.so`+파이썬 바인딩 curated 디렉터리)를 read-only 마운트, `LD_LIBRARY_PATH`/`PYTHONPATH`에 연결(`docker-compose.gpu.yml` 주석에 curated 디렉터리 준비 명령 있음). Tegra 드라이버 레벨 라이브러리(`libcuda`/`libnvrm_*`/`libnvdla_*`/`libcudla`)는 `runtime: nvidia`가 자동으로 넣어주므로 별도 마운트 불필요.
- **TensorRT 백엔드(`perception_node`의 `backend:=trt` 파라미터) 실측(2026-07-22, box seg 모델 848×480)**: CUDA(pt) 17.1Hz → TensorRT FP16 25.1~25.5Hz(`/detected_objects` 실측). 첫 실행 시 `.pt→.onnx→.engine` 변환에 ~8분 소요, 이후는 캐시된 `.engine` 재사용(같은 입력 크기 한정).
  ⚠️ **함정**: `.engine` 파일은 task 메타데이터를 보존하지 않아 ultralytics가 자동 추정 시 seg 모델도 `task='detect'`로 오판하고 `r0.masks`가 **에러 없이 조용히 `None`**이 돼 markerless pose가 깨진다 — `model_presets.py`의 `task` 필드를 `YOLO(path, task=...)`에 명시적으로 넘겨서 해결(perception_node.py 참고). 새 preset 추가 시 `task` 필드 채우는 것 잊지 말 것.

> Base image is `ros:humble-ros-base` + `ros-humble-desktop` layered on top (split from the former `osrf/ros:humble-desktop-full` monolith), built for `linux/arm64` (`docker-compose.yml`) — this targets the Jetson deployment, not just dev laptops.

> Note: the Dynamixel libraries are pulled via apt (`ros-humble-dynamixel-sdk`, `ros-humble-dynamixel-workbench`) — **not** git submodules. An earlier broken submodule/gitlink for these was removed.

## 벤치 텔레옵 (파워트레인 없이 팔만 구동)

`bench.launch.py` — `joy_node` → `joystick_teleop` → `teleop_core` → `position_node`. 키보드 프론트엔드(`keyboard_teleop`, 2026-08-06부터 **curses TUI 콘솔**)도 같은 토픽을 쓰므로 대체 가능하다 — 단 stdin 포커스가 필요해 launch에 못 넣고 별도 터미널에서 `ros2 run dynamixel_control keyboard_teleop`.

> 별개의 벤치 경로로 **마스터-슬레이브 HIL 프로브**(`master_slave_master`/`master_slave_slave`)도 있다 — 토크 푼 XL430 하나를 읽어 TCP로 흘리는 1축짜리 의도적 최소 구성이고, 레지스터를 쓰지 않는다. 이것도 production 금지(프로덕션은 MoveIt Servo 경로).

- **격리는 launch 파일이 전부다.** `arm_fsm`을 안 띄우므로 계약 토픽(`/arm_status`·`/chassis_mode`·`/arrival_status`)이 **아예 생기지 않는다.** 노드 코드에 "테스트 모드면 건너뛰기" 분기는 **넣지 않았고, 넣지 말 것** — 안전 게이트에 스킵 분기가 있으면 실기에서 켜진 채 도는 사고가 난다(파워트레인도 같은 원칙: *"production source에 simulator 이름 분기를 넣지 않는다"*).
- ⚠️ **이 경로는 계약상 production 금지다.** `/dynamixel/goal_position` 직접 발행은 계약이 금지하는 *"direct dynamixel goal publisher"*이고, `home`(전 관절 0)은 금지된 *"all-zero home"*이다. **파워트레인과 "팔 단독 벤치 profile 허용"을 합의해야 한다**(그쪽은 자기 쪽에 `arm_gate_mode=arm_absent_field`라는 대칭 장치를 이미 뒀다).
- `JointJog.velocities`는 **rad/s**다(표준). `displacements`는 `jog_step_rad` 배수(키보드용). 예전엔 velocity가 "초당 jog_step 개수"로 해석돼 풀스틱이 0.05 rad/s밖에 안 나왔다 — 고쳤다.
- **velocity 프론트엔드는 매 발행마다 전 관절을 실어야 한다.** `teleop_core.on_jog`는 메시지에 없는 관절의 velocity를 0으로 만들지 않아, 움직이는 관절만 골라 보내면 **놓은 관절이 마지막 속도로 계속 돈다.**

### 그리퍼는 이제 이 경로에도 배선돼 있다 (2026-08-02 이후)

`position_node`의 `motor_ids` 기본값 `[11, 14, 13, 12, 16, 3]` 마지막 `3`이 그리퍼다(`joint_names` 마지막 = `gripper_left_pinion_joint`). 표현도 `gripper_presets.py` 한 곳으로 통일됐다 — 옛 세 갈래(`gripper_a_joint5~23` / `left_finger_joint` prismatic 미터 / HW-8 degree)는 전부 사라졌다.

- ⚠️ **소유권 중복은 여전히 유효한 금기다.** `moveit_dynamixel_bridge`도 같은 ID 3을 구동하므로 **두 런타임을 같은 버스에 동시에 띄우지 말 것**(버스 전체에 대해 이미 적용되는 원칙과 같다).
- ⚠️ **2026-08-02 실기로 2모터(ID 3,4) 동시구동 → 단일모터(ID 3) 구동으로 바꿨다.** 두 독립 위치제어 루프가 같은 강체 레일을 밀며 서로 힘을 겨뤄 전류가 계속 오르다 트립하고, 트립 후엔 goal을 갱신해도 두 모터 다 velocity/current=0으로 응답을 멈췄다(**하드웨어 에러 플래그는 안 뜬다** — 이걸 모르면 원인을 못 찾는다). ID 4는 토크도 안 걸어 자유회전으로 둔다. 2모터로 되돌리려면 서로 겨루지 않는 마스터-팔로워(한쪽은 위치, 다른 쪽은 전류/토크) 구조가 필요하다.

> ⚠️ **배선할 때 반드시**: Dynamixel **Profile Acceleration(주소 108) / Velocity(112)를 25/80으로 설정**할 것. 기본값 `0`(=최고속 즉시 이동)이면 그리퍼가 움직일 때마다 순간 과전류로 **토크가 풀린다**(HW-8 실기 검증, 재현율 100%, 명령 후 0.3초 내 트립). Hardware Error Status는 트립 해소 후 0으로 복귀해 관찰 시점엔 안 보인다 — 이걸 모르면 원인을 못 찾는다.

## 파워트레인 계약 (중요)

파워트레인 팀([power-train-sw](https://github.com/lightminn/power-train-sw))과 **같은 Jetson의 별도 컨테이너**에서 각자 ROS 2 노드를 돌리고 **DDS로만** 통신한다. 워크스페이스를 서로 오버레이하지 않으며, 공유하는 것은 메시지 계약뿐이다. 파워트레인은 `robot_arm_msgs`의 `.msg`만 벤더링해 자기들이 직접 빌드한다(ROS 2는 wire에서 **패키지명 + 구조 해시**로 매칭하므로 동일한 `.msg`로 각자 빌드하면 붙는다).

- **값 어휘의 단일 출처는 `dynamixel_control/contract.py`** — 파워트레인의 `powertrain_ros/contract.py`와 짝이다. **여기 없는 status 문자열을 새로 만들지 말 것.** 파워트레인은 `contract.ARM_STATUSES` 밖의 값을 받으면 즉시 `CONTRACT_VIOLATION` + motion hold를 건다. 어휘 변경은 **양 팀 합의 사항**이다.
- QoS는 `dynamixel_control/qos_profiles.py`. heartbeat 계열은 **KeepLast 1**이다 — depth를 키우면 낡은 샘플이 큐에 쌓여 파워트레인의 신선도 판정이 어긋난다.
- **`/arm_status`는 `arm_fsm`의 heartbeat 타이머 한 곳에서만 발행한다.** 상태 핸들러는 `_set_status()`로 값만 바꾼다. 발행 경로가 둘이 되면 `header.stamp` 순서가 뒤집힐 수 있는데, 파워트레인은 stamp가 0.5초 이상 역행하면 **영구 latch**(프로세스 재시작 전까지 해제 불가)를 건다.
- **`MISSION_STOP`만이 팔 작업 허가다.** `DRIVING`을 포함한 나머지 mode는 전부 잠금(default-deny).
  - **`arm_fsm_node.py`는 2026-07-15부터 이를 준수한다** — `LOCK_MODES`를 `contract.py`(파워트레인과 짝인 단일 출처) 걸로 통일해 `DRIVING` 수신 시 `_on_chassis_mode()`가 `_enter_locked()`를 건다. 언락은 `_try_advance()`의 `MISSION_STOP` + 같은 mission_id `ArrivalStatus` conjunction 경로 하나뿐(자동 언락 분기 없음).
- **차가 움직이려면 팔이 `STOWED_LOCKED` 또는 `CARRYING_LOCKED`를 신선하게 발행해야 한다**(`contract.DRIVE_READY_STATUSES`). 그 외 status는 전부 주행 불가다.
  - **이제 둘 다 발행한다**(`_is_settled()` 게이트 통과 후 `_set_status()`) — 접힘 모션(`_begin_stow_move`)도 구현됨.
  - **`stow_joint_positions` 기본값은 2026-07-29 팀 결정으로 all-zero로 확정됐다**(PR #31, 랙피니언 URDF 기준 재결정 — 주행 안정성). 코드도 `[0.0] * len(ARM_JOINT_NAMES)`라 축 수가 바뀌어도 따라간다(리터럴로 박지 말 것). SRDF에도 `stow` named state가 all-zero로 들어갔다. ⚠️ 예전 문서의 "all-zero는 계약상 금지" 및 placeholder 기본값 `[0.0, -0.6, 1.2]` 서술은 **이 결정으로 폐기됨** — 벤치 텔레옵의 `home`(전 관절 0) 금지와 혼동하지 말 것(그건 여전히 금지).

### robot_arm_msgs (ament_cmake) — 공통 메시지 패키지
양팀(로봇팔·파워트레인)이 공유하는 커스텀 메시지 5개: `DetectedObject`(class_id/name/confidence/`geometry_msgs/Pose`/bbox), `DetectedObjectArray`(header + objects[]), `ArrivalStatus`, `ChassisMode`, `ArmStatus`. 인터페이스 상세는 `project_docs/CLAUDE_Plan.md` §1 참고.

### robot_arm_perception (ament_python) — markerless 인식 노드
**entry point 7개** (2026-08-07 기준): `perception_node`·`stream_node`(아래) + `metadata_sender_node` + `vision_test`(V4L2 USB 캠 + YOLO seg 확인용 뷰어, PR #41) + `detection_markers`/`ground_truth_markers`(RViz 마커) + `camera_tf_tuner`(RViz에서 `base_link→camera_link`를 슬라이더로 맞추는 캘리브 GUI, PR #42 — `launch/camera_calib.launch.py`·`camera_calib_solver.py`·`scripts/calibrate_camera_pose.py`와 한 세트) + `calib_status_view`(아래). 품질 지표는 `perception_quality.py`(유일하게 실기 없이 도는 유닛테스트 `test/test_perception_quality.py` 보유).

`perception_node`: RealSense D435i color+depth → YOLO **segmentation** 추론 → `/detected_objects`(`DetectedObjectArray`) 30Hz publish. **markerless pose**(대회 규정상 타겟 마커 부착 금지): translation은 마스크 centroid의 depth median deproject(`yolo_depth_3d.py` 로직 포팅, align 생략), orientation은 마스크 (u,v) 픽셀 **2D PCA** 주축각 → optical Z yaw quaternion. 카메라 intrinsics는 RealSense 스트림에서 직접 취득(calibration yaml 불필요). 또한 `/pick_target`(`DetectedObject`, transient_local latched)을 publish: `pick_classes` 화이트리스트 ∩ `pick_min_conf` 이상 ∩ depth 조건(`require_depth`) 만족 객체 중 confidence 최고 하나(신호등/정지선 등 관찰 전용은 화이트리스트로 자동 제외). **모델 선택은 `model_name` preset 하나로 한다**(PR #28, `model_presets.py`의 `MODEL_PRESETS`, 기본 `box`) — preset이 `model_path`/`task`/`classes`/`pick_classes`를 한 벌로 채우고, 개별 파라미터로 덮어쓸 수도 있다. **새 모델을 추가할 땐 노드가 아니라 이 dict에 항목만 추가**하고, `task`(`segment`|`detect`) 필드를 반드시 채울 것(위 TensorRT 함정 참고). 나머지 파라미터: `backend`(`pt`|`trt`), `camera_mode`(`realsense`|`test`), `conf_threshold`(0.55), `width`/`height`/`fps`(848×480×30), `pick_min_conf`, `require_depth`, `frame_id`, `publish_cloud`/`cloud_rate_hz`/`cloud_decimation`(포인트클라우드, 추론 대역폭을 뺏지 않게 기본 off/저속). **seg 모델이면 markerless pose 전체 활성, detection 전용이어도 bbox 중심 depth로 translation은 폴백**한다. ArUco 경로는 제거됨. 진행 상황은 `project_docs/CLAUDE_Plan.md`·`project_docs/WORK_STATUS.md`.

**손목 카메라(`wrist_camera` entry point, 2026-08-13 신설)는 전방 캠과 완전히 분리된 별도 노드다.** USB 캠(그리퍼 고정)으로 **근접도와 파지 상태**를 관측한다. `/wrist/raw_image`·`/wrist/debug_image`(구독자 게이트)·`/wrist/detected_objects`·`/wrist/metrics`(JSON)를 발행하고, **`/pick_target` publisher 를 아예 만들지 않는다** — `perception_node` 를 두 번째 인스턴스로 띄우는 대신 별도 노드로 만든 이유가 이것이다(remap 하나만 빠뜨리면 손목 캠이 본 박스가 latched 로 박혀 팔이 엉뚱한 좌표로 가고, 노드를 내려도 값이 남는다). 지표 계산은 ROS 비의존 `wrist_metrics.py`/`wrist_color_mask.py` 로 분리돼 하드웨어 없이 pytest 로 검증된다(31케이스). 표본 수집만 ROS 에 의존하며 `wrist_sampling.py` 에 따로 있다(실측 스크립트 2종이 공유 — 각자 구현하면 필터가 갈라져 두 실측이 서로 다른 표본을 쓰게 된다).
- ⚠️ **파지 거리에서는 YOLO 가 대상 상자를 못 본다**(2026-08-13 실측: `conf 0.20` 까지 낮춰도 0건, 같은 프레임의 배경 택배 상자는 0.97). 전방 D435i 는 같은 상자를 0.9대로 잡으므로 클래스 문제가 아니라 **근접·잘림이 학습 분포 밖**인 것이다. 그래서 `mask_source` 기본값이 `color`(HSV 빨강∪파랑 마스크)이고, **GPU 를 전혀 쓰지 않아** 전방 캠 추론 대역을 뺏지 않는다. `yolo` 경로도 남겨뒀다.
- ⚠️ **ROI 게이트가 필수다.** 배경 선반의 택배 상자도 파랑/분홍이라 confidence·면적 최고로 고르면 **배경이 이긴다**(실제로 그리퍼에 물린 상자 대신 배경이 지표로 나갔다). 그리퍼는 화면에서 늘 같은 자리(하단 중앙)이므로 위치가 더 강한 근거다 — `roi` 파라미터(기본 `0.10,0.90,0.45,1.00`). **카메라를 재장착하면 다시 잡을 것.**
- 실측 HSV(2026-08-13, 실내 형광등): 파랑 H 104~117 / 빨강 H 171~178. 대상은 **95mm 큐브(454~754g)**.
- ⚠️ **케이블은 색이 아니라 두께로 걸러진다.** 그리퍼 옆 빨간 케이블이 상자 마스크에 간헐적으로 달라붙어 가로 픽셀 sd 가 31px(13%)까지 벌어졌었다 — 상자와 **같은 빨강**이라 HSV 로는 못 가른다. `wrist_color_mask` 가 두 단계로 턴다: (1) `thin_reject_px`(기본 15) 커널 열림으로 가는 구조물 제거, (2) 남은 덩어리 가장자리에서 두께가 90백분위수의 `trim_frac`(0.25)배 미만인 열·행 잘라내기. **2026-08-13 실기 20프레임 재측정: sd 31px → 1.5px(0.7%).**
  - ⚠️ **잘라낸 양(`metrics.trimmed_px`)이 0 이 아닌 게 정상이다** — 케이블이 없어도 상자 가장자리에서 늘 덩어리의 2~3%(139~514px)가 잘린다. `trimmed_px > 0` 을 "케이블 붙음"으로 읽으면 **쓸 만한 표본이 0개**가 된다(실제로 그렇게 짰다가 실기에서 걸렸다). 판정은 비율로 하고, 그 기준은 `wrist_metrics.DEFAULT_MAX_TRIM_RATIO`(0.10) 한 곳에 있다.
  - `_core_span` 의 기준은 **중앙값이 아니라 90백분위수**다 — 잔가지가 상자보다 길면(열 개수가 많으면) 중앙값 자체가 잔가지 두께로 끌려가 문턱이 무너진다.
- ⚠️ **거리는 bbox 가로 폭으로만 낸다.** 같은 프레임의 같은 상자로 뽑은 `f_px` 가 **가로 412 / 세로 150** 으로 2.7배 어긋난다(상자를 비스듬히 내려다봐 세로만 단축). 세로를 쓰면 거리가 2.7배 가깝게 나와 하강을 일찍 멈춘다. 노드 파라미터 이름이 `box_size_m`(높이가 아니라 **가로로 보이는 변**)인 이유다.
- **실측 도구 2종**(`scripts/`, `dynamixel_control` 의 `measure_*.py` 와 같은 계열): `measure_wrist_proximity.py`(거리 곡선 → `f_px`), `measure_wrist_grasp_band.py`(정상 파지 vs 빈/어긋난 파지의 `fill`·`u`·`v` 밴드 + **분리 여부**). 계산부는 전부 `wrist_metrics` 의 순수 함수(`robust_stats`/`suggest_band`/`fit_distance_curve`/`usable`)라 pytest 로 고정돼 있다.
  - ⚠️ **거리 점을 여러 개 재는 건 선택이 아니다.** `f_px = w*d/S` 는 `d` 가 렌즈 주점 기준일 때만 맞는데 자로 재는 기준점은 늘 몇 cm 어긋나 있고, 한 점 계산에서는 그 오프셋이 통째로 `f_px` 오차가 된다. `d = a/w + b` 로 맞춰 `a`→`f_px`, `b`→기준점 오프셋을 분리한다.
  - ⚠️ **겉보기 크기 차이가 20% 미만이면 스크립트가 값을 내지 않는다**(`fit['plausible']`). 상자를 안 옮기고 두 거리를 입력해 봤더니 `f_px=47198`·오프셋 `-20.9m` 라는 **형태만 멀쩡한 쓰레기**가 나왔고 그대로 '적용하세요'로 출력됐다 — 그 분기를 막아 뒀다.
  - 파지 밴드는 실패 조건(`e`/`s`) 표본이 없으면 `separated=False` 로 보고하고 임계값을 확정하지 않는다. **FSM 2·3단계의 전제가 이 분리 확인이다.**
- `wrist_camera_link → wrist_camera_optical_frame` static TF 는 `wrist_camera.launch.py` 가 같이 띄운다(REP-103 회전, `optical_tf:=false` 로 끔). ⚠️ **장착 오차를 이 TF 로 보정하지 말 것** — 카메라가 어느 방향을 보는가는 URDF `fixed_joint_035` 한 곳이 정한다(현재 `rpy="0 0 0"`, CAD 자동생성분이라 **실물 검증 아직 안 됨**). 여기서 각도를 만지면 진실이 두 곳으로 갈려 다음 CAD 재export 때 조용히 어긋난다.
- **거리 캘리브 확정(2026-08-14 실기): `f_px = 360.7`(가로 기준), `box_size_m = 0.095`** — launch 기본값에 들어가 있어 그냥 띄우면 `metrics.distance_m` 이 나온다. 160/200/260mm 3점 맞춤, 잔차 최대 1.4mm, 기준점 오프셋 −0.2mm. 같은 프레임의 **독립적인 세 특징**으로 교차검증됨(큐브 실루엣 95mm +0.2% / 큐브 면의 빨간 원판 50mm −0.5% / 그 원판의 구멍 25mm −4.0%). 실기 검증: 자로 잰 260mm ↔ 발행값 259.6mm.
  - ⚠️ 옛 값 412 는 **폐기**. 케이블이 붙어 부푼 마스크로 잰 단일점이라 14% 컸다.
  - ⚠️ **겉보기 폭은 상자의 방위각에도 좌우된다** — 큐브를 비스듬히 보면 두 면이 겹쳐 폭이 최대 √2배(대각선 134mm) 부푼다. 실측에서 상자를 화면 왼쪽으로 옮긴 점 하나가 예측 대비 +31% 로 튀어 곡선을 깼다. 하강은 수직(`approach_height` 위에서 곧장)이라 방위각이 안 변하므로 운용에는 문제없지만, **캘리브할 때 상자를 옆으로 옮기면 안 된다.** 반대로 고도각(위에서 내려다보는 각)은 세로만 누르고 가로는 안 건드린다 — 거리 신호로 가로만 쓰는 이유가 이것이다.
- **파지 검증 확정(2026-08-14 실기, `wrist_metrics.GRASP_*`): 판정은 `fill` 하나로 하고, 파지 자세의 손목 롤은 `arm_joint_5 = 0` 으로 고정한다.**
  - 실측: 정상 파지 0.0483~0.0511(40프레임, 검출률 100%) / 실패 2종(모서리만 물림·빠져나옴) 0.0714·0.0724 / 빈 그리퍼 **검출 0%**. 임계 `GRASP_FILL_MAX = 0.06`.
  - **실패 쪽이 오히려 크게 보인다** — 제대로 물리면 큐브가 조 안쪽에 들어가 그리퍼 몸체에 가려지고, 실패하면 밖으로 나와 온전히 보이기 때문이다. 우연이 아니라 기구적 필연이라 두 실패 모드가 같은 값을 냈다.
  - ⚠️ **롤 고정이 전제다.** 롤 4개를 재보니 `fill` 이 0.0203/0.0268/0.0345/0.0495 로 2.5배 흔들리고, 어떤 롤에서는 큐브가 조 안쪽 깊이 들어가 **검출률이 53%** 로 떨어졌다(멀쩡히 물었는데 절반이 미검출). 롤을 안 고정하면 정상 범위가 실패값까지 넓어져 판정이 성립하지 않는다. 거리도 같이 망가진다 — 가려진 롤에서는 실제 160mm 를 220mm 로 보고했다.
  - ⚠️ **`u`/`v`/`bbox_w` 를 게이트로 쓰지 말 것.** 실패 표본 60/60 이 정상 밴드 안에 들어와 전혀 안 갈렸다. 진단용으로만 `GRASP_REFERENCE` 에 남겨 뒀다.
  - ⚠️ **미검출은 실패가 아니라 '판정 불가'** 다(`classify_grasp` 가 `'unknown'` 을 돌려준다). 미검출을 실패로 읽으면 성공한 파지를 놓고 재시도를 반복한다.
- ⚠️ **게이트 3종이 전부 "파지 시점에는 정상인 상황"에서 오발동했다** — 잘림(`border_contacts>=2`), 잔가지(`trimmed_px`), 가림(`occluded`). 특히 `occluded` 는 그리퍼 몸체가 큐브를 가려 mask/bbox 가 0.26(게이트 0.55)이라 **전 프레임이 버려졌다.** 파지 밴드를 잴 때는 셋 다 꺼야 한다(`measure_wrist_grasp_band.py` 가 그렇게 한다). 원거리 캘리브에서는 반대로 켜야 한다 — 같은 게이트가 두 구간에서 정반대 의미를 갖는다.
- 아직 안 된 것: FSM 결합(2·3단계), URDF 손목 카메라 방향 실물 검증, `arm_joint_5=0` 자세를 브릿지로 재확인(각도는 사용자 확인, `/joint_states` 실측은 미실시).

**캘리브 안내는 두 갈래로 나간다** (2026-08-13). `camera_tf_tuner`가 `/perception/calib_status`(RViz 3D 텍스트 마커, **영어** — Ogre 기본 폰트에 한글 글리프가 없다)와 `/perception/calib_guide`(`std_msgs/String`, **한국어**)를 같은 내용으로 발행하고, 두 언어를 `_guide_lines(ko=)` **한 함수**에서 만든다(따로 두면 한쪽만 고쳐져 어긋난다). 후자를 `calib_status_view`가 별도 Tk 창에 크게 띄운다 — 3D 텍스트는 월드 스케일이라 줌아웃하면 단어가 화면 사방으로 흩어져 사실상 못 읽는다(실측 확인). 이 창은 현재 카메라 자세(TF에서 직접 읽음)와 `/detected_objects` 목록도 같이 보여주고, **발행 토픽이 0개인 읽기 전용**이다(관제 GUI와 같은 사상). `camera_calib.launch.py`가 기본으로 띄우며 `status_view:=false`로 끈다.
- ⚠️ **컨테이너 이미지엔 한글 폰트가 하나도 없다**(`fc-list :lang=ko` = 0건). `docker-compose.yml`이 호스트 `/usr/share/fonts`를 `/usr/share/fonts/host`로 read-only 마운트해 해결한다(이미지에 넣으면 팀 전원 arm64 재빌드). 폰트를 못 찾으면 이 창은 영어 마커 토픽으로 자동 폴백하고 그 사실을 하단에 띄운다.
- ⚠️ **Tk 창을 띄우는 노드는 `rclpy.ok()`를 주기 콜백에서 직접 확인해야 한다.** rclpy의 시그널 핸들러는 컨텍스트만 내리고 프로세스를 끝내지 않는데, Tk `mainloop()`가 메인 스레드를 잡고 있으면 **`pkill`/Ctrl-C에도 창이 살아남아** 옛 값을 계속 띄우는 유령이 된다(2026-08-13 실측 — `kill -9`로만 죽었다).

영상 토픽은 **둘이고 게이트가 다르다**. `/perception/raw_image`(원본)는 캡처 스레드가 프레임을 받는 즉시 **무조건** publish한다 — 추론 스레드와 분리돼 있어 YOLO 가 느려져도 raw 주기가 끌려가지 않는다(2026-07-15 분리). 반면 `/perception/debug_image`(bbox·마스크·거리 오버레이, pick 타겟=초록/나머지=파란색)는 **구독자가 있을 때만** 그린다. `stream_node`는 둘 중 하나(`image_topic` 파라미터, 기본 `/perception/raw_image`)를 구독해 `gst-launch-1.0` 서브프로세스(x264enc→SRT)로 원격 PC에 H.264/SRT 스트리밍(`recv_stream.sh`로 수신) — 하드웨어 테스트 중 원격 모니터링용, `/pick_target` 등 제어 경로와 무관.

### dynamixel_control (ament_python) — the core runtime
네 갈래 런타임이 한 패키지에 있다(entry points in `setup.py`) — **셋 다 `/dev/ttyUSB0` 버스를 잡으므로 동시에 띄우지 말 것**:
1. **legacy YOLO→servo P-control 파이프라인**(3 노드, 아래)
2. **Phase 3 MoveIt/FSM 파이프라인**(`moveit_dynamixel_bridge` + `arm_fsm`) — 실제 구간2 pick 경로
3. **벤치 텔레옵**(`teleop_core` + `keyboard_teleop`/`joystick_teleop`, `master_slave_*`) — 위 "벤치 텔레옵" 절
4. **실기 캘리브레이션 도구**(2026-08-07 신설, PR #40): `gripper_calibration`/`gripper_load_calibration` 노드 + `scripts/` 5종(`measure_zero_offset`·`measure_gear_ratio`·`measure_joint_limits`·`measure_gripper_endpoints`·`calibrate_camera_pose`) — 손으로 관절을 돌려 영점·기어비·가동범위·그리퍼 끝단·카메라 TF를 실측한다. **`JOINT_CONFIG`/`gripper_presets`/`joint_limits`의 숫자는 전부 이 도구들의 산출물이므로, 값이 의심스러우면 다시 재는 게 정답이다.**

- **`joint_limits.py` (2026-08-07 신설) — 관절 안전 가동범위의 단일 출처.** `rad_to_tick`이 그동안 **서보 tick 범위로만** clamp해서("서보가 표현 가능한 값" ≠ "구조물에 안 부딪는 범위") IK가 엉뚱한 각도를 내면 그대로 나가 구조물을 때렸다. URDF를 쓸 수 없는 이유도 명시돼 있다 — `arm_joint_2/3`의 URDF 리밋은 `0~π` 자동생성 placeholder(CAD 미반영), `arm_joint_1/5`는 `continuous`라 리밋 자체가 없고, 믿을 수 있는 건 `arm_joint_4`뿐이다. 여기 값은 전부 **관절각(rad) 도메인**(= MoveIt/`arm_fsm`과 같은 도메인, `JOINT_CONFIG`의 `center`/`gear_ratio`로 환산된 쪽) — tick 도메인과 헷갈리지 말 것.

```
yolo_detection ──/yolo/target_center──▶ yolo_bridge ──/dynamixel/goal_position──▶ position_node ──▶ physical XL430 servos
   (camera+YOLO)     [cx, cy]            (P-control)        [id, goal_pos]                          + /joint_states, /dynamixel/state
```

- `yolo_detection` (`yolo_detection_node.py`): opens the camera with `cv2.VideoCapture`, runs `ultralytics` YOLO, publishes the best target's pixel center to `/yolo/target_center`. **Does not use `rclpy.spin`** — it runs its own blocking `while rclpy.ok()` loop in `run()`; an OpenCV preview window (`show_window` param) needs X/GUI forwarding. Tunable params: `model_path`, `target_class`, `conf_threshold`, `camera_device`, etc.
- `yolo_bridge` (`yolo_to_dynamixel_bridge.py`): converts pixel error `cx - 320` into a goal position via simple proportional gain, publishes `[id=1, goal]` to `/dynamixel/goal_position`. Currently hardcoded to motor ID 1.
- `position_node` (`dynamixel_position_node.py`): legacy 파이프라인 + 벤치 텔레옵이 공유하는 하드웨어 노드. `/dev/ttyUSB0` 1 Mbps, protocol 2.0. **더 이상 하드코딩이 아니라 전부 파라미터다** — `motor_ids` 기본 `[11, 14, 13, 12, 16, 3]` / `joint_names` 기본 `arm_joint_1..5` + `gripper_left_pinion_joint`(**`joint_1..5` 옛 이름·`DXL_IDS=[0..4]` 서술은 폐기됨**), `port`/`baudrate`, `profile_acceleration`/`profile_velocity`, `extended_position_ids`(기본 `[14, 13, 3]` — 다회전축), `force_position_mode`, `read_rate_hz`(30)/`write_rate_hz`(100)/`temperature_poll_hz`(2), `ftdi_latency_timer`(1), 그리고 과전류 보호(`current_trip_threshold`/`current_trip_enabled`, `current_spike_delta_threshold`/`current_spike_enabled`). `/dynamixel/goal_position` 구독 → `/dynamixel/state` + `/joint_states` publish.

**MoveIt/FSM pipeline (Phase 3 — the real pick path; both nodes touch `/dev/ttyUSB0`/MoveIt, don't run alongside `position_node` on the same bus):**
- `moveit_dynamixel_bridge` (`moveit_dynamixel_bridge.py`): hardware node for the MoveIt path. Implements `/arm_controller/follow_joint_trajectory` + `/gripper_controller/follow_joint_trajectory` action servers, so MoveIt/`arm_fsm` execute on real servos (a lighter substitute for a full `ros2_control` HW interface). Reads `HARDWARE_ERROR_STATUS`(70,1)~`PRESENT_POSITION`(132,4) in one 66-byte SyncRead (X-series control table is contiguous there — Hardware Error Status/Present Load/**Present Velocity**/Present Position all in one bus transaction) → publishes `/joint_states` with **position + velocity + effort (raw signed load)**. **`PRESENT_VELOCITY`(128,4) parsing added 2026-07-31** — the bytes were already inside the SyncRead range (no new bus transaction) but sat unparsed; `VELOCITY_LSB_TO_RAD_S`(arm, divided by `direction`) / `gripper_velocity_to_rad_s()`(gripper, via calibrated tick↔rad span) convert the datasheet's `0.229 rev/min` per-LSB unit. ⚠️ Sign/scale not yet real-hardware-verified (Notion "그리퍼 tick/wrist_to_gripper/PRESENT_VELOCITY 실측·검증 절차" §2-3) — `arm_fsm_node.py`'s `_is_settled()` still uses its own position finite-difference, unchanged, pending that verification. Gripper is module-dependent (`gripper_a` = XL430 2-motor rack-pinion, ids from `gripper_ids`) reported as one logical joint (position/velocity from the first-responding id, effort = max abs load across ids — conservative grasp/drop detection); `gripper_ids`/`gripper_open_tick`/`gripper_close_tick` are params, defaulted per-module from `gripper_presets.GRIPPER_PRESETS` via a `gripper_type` param (default `gripper_a`, matching `robot_arm_description`'s `xacro:arg gripper`) — still individually overridable by CLI/launch, empty `gripper_ids` disables the gripper → mock-friendly. **Arm `JOINT_CONFIG`는 2026-08-19 재조립 후 다시 전면 실측됐다 — 이제 `arm_joint_1`~`arm_joint_5` 5축, ids `11/14/13/12/16`**(옛 `arm_joint_1..3`/ids 0,1,2 서술도, "4축" 서술도 폐기). 관절마다 `center`(영점 tick, **2026-08-19 실측 2081/506/1855/1184/675**) · `direction` · `gear_ratio`(**1.0 / 9.034 / 4.040 / 1.0 / 1.0 — joint_2와 joint_3은 서로 다른 감속기다, 오타 아님. joint_1은 감속기 없는 직결**) · `extended`(Extended Position Mode 여부)를 갖는다. 영점은 URDF home 자세에서 잰 값이고, 그 전까지 쓰던 "전 축 2048" 가정은 축마다 최대 1100 tick(≈97°) 어긋나 있었다 — **기어비든 영점이든 틀리면 IK 결과가 통째로 그만큼 어긋난다. 팔을 분해·재조립하거나 서보를 뿔에서 뺐다 끼우면 재측정 필수**(`scripts/measure_zero_offset.py`·`measure_gear_ratio.py`).
  - 🔁 **`arm_joint_1`(베이스 요축, ID 11)에는 2026-08-19 기준 XM540-W270 서보가 실장돼 있다** — 재조립 후 버스 스캔에서 정상 응답했고 기구 결합도 확인됨(사용자). 옛 "모터가 물리적으로 없다(2026-08-07)" 서술은 **폐기**이며, `JOINT_CONFIG`에 정식 등록됐고 `STATIC_JOINTS`는 **빈 dict**가 됐다. 아래는 모터가 없던 시절의 배경으로만 읽을 것 — URDF에서는 `link_002→link_004`를 잇는 관절이라 `/joint_states`에 값이 없으면 `robot_state_publisher`가 이 관절을 못 넘어가 **TF 트리가 두 조각으로 갈리고**(`Tf has two or more unconnected trees`) `base_link→link_043`(tip) 변환이 아예 안 만들어져 arm_fsm의 IK·carry pose가 **전부 실패**한다. 그래서 그때는 브릿지가 **고정값(`STATIC_JOINTS`)으로 발행**했다(지금은 실측값이 나간다). MoveIt도 5축 전체 관절값을 기대하므로 같은 이유로 필요했다. **2026-08-12 그 고정값을 1.405 → 0.0으로 정정했었다** — "팔이 정면(+x)을 향한다"는 옛 전제가 틀렸고 실제로는 오른쪽으로 틀어져 있다(카메라 캘리브 후 박스를 그리퍼 정면에 놓고 확인: joint_1=0 모델의 tip 방위각 −80.7°, 박스 −90.7°로 일치). 1.405를 쓰면 브릿지가 떠 있을 때만 TF가 80° 돌아가 **RViz만 보면 멀쩡한데 arm_fsm의 목표만 틀어지는** 조용한 오차가 된다. **Only IDs whose torque-enable actually succeeds get registered in the SyncRead group** — a missing/unpowered servo no longer breaks readback for the rest of the bus.
- `arm_fsm` (`arm_fsm_node.py`): the 구간2 pick FSM (**17 states** `IDLE`~`LOCKED`, MoveIt 단일 경로 '가'). ⚠️ **State(17종)는 어느 토픽으로도 나가지 않는다** — `/arm_status` 로 나가는 status 는 10종뿐이고 `APPROACH`/`DESCEND`/`GRASP`/`LIFT` 는 전부 `EXECUTING` 하나로 뭉개진다. 세부 상태는 이 노드의 로그에서만 볼 수 있다. Subscribes `/pick_target`(latched)·`/arrival_status`·`/chassis_mode`·`/joint_states`, publishes `/arm_status`. Sends pose goals to MoveIt `move_action`; grasp/DROP decided from `/joint_states.effort` (raw-current thresholds). Gripper params (`gripper_joints`/`gripper_open`/`gripper_close`/`grasp_effort_thresh`/`drop_effort_thresh`/`gripper_action_time`) default from the same `gripper_presets.py`/`gripper_type` mechanism as the bridge (kept in sync intentionally — a mismatch here previously left both nodes defaulting to stale `left_finger_joint`/`right_finger_joint` names that didn't match the `gripper_a.xacro`/SRDF joints `gripper_a_joint5`/`gripper_a_joint6`). `_carry_pose()` looks up TF (`base_frame`←`tip_link`) for a base_link +Z lift (`lift_height`) → needs `tf2_ros` (in `package.xml`). Status string enums (`ARRIVED_PICKUP`/`DONE`/…) are **provisional, pending powertrain-team agreement**. Hardware-free smoke test: launch + mock-pub `/pick_target`(transient_local) + `/arrival_status` → expect `IDLE→PERCEIVE→PLAN→DESCEND` then a `move_action 미준비` warning (no move_group).
  - **`gripper_presets.py` (신규):** shared preset dict (`GRIPPER_PRESETS`, keyed by gripper name) consumed by both nodes above — adding a new gripper module (e.g. `gripper_b`) means adding one preset entry here, not editing either node's code. Currently only `gripper_a` is defined — **tick 값은 `gripper_presets.py` 가 단일 출처다 — 2026-08-19 확인 기준 `gripper_open_tick=-805` / `gripper_close_tick=-1865`, `gripper_ids=[3]`, `extended=True`** (⚠️ 이 문서에 오래 적혀 있던 `1083`/`-401` 은 코드와 안 맞는 낡은 값이다 — 숫자는 항상 preset 파일에서 확인할 것). 그 밖에 `extended=True`, 각도 도메인은 `gripper_open_rad=1.9444`(URDF 상한, 완전 열림)/`gripper_close_rad=0.0`(완전 닫힘).** 옛 값(open=2446/close=3186, HW-8 단일서보 ID 5 유산)은 현재 조립과 안 맞아 tick 974에서 5.81 rad(URDF 상한의 3배)로 보고되는 증상을 냈다.
    - ⚠️ **개폐 방향 부호가 뒤집혔다** — 옛날엔 open<close("열기=tick 감소"), 지금은 open>close("열기=tick 증가"). 옛 값으로 구동했다면 여닫이가 **반대로** 갔다.
    - ⚠️ **`close_tick`이 음수다** → 다회전(Extended Position) 영역이라 `extended: True`가 필수다. 단일회전으로 clamp하면 완전 닫힘이 tick 0에서 잘려 **401 tick(≈35°) 덜 닫힌다**(`teleop_core`의 `EXTENDED_POSITION_NAMES`에 그리퍼가 들어있는 것도 같은 이유). **Closing-ratio checkpoint (2026-07-28, silicone test object, `gripper_load_calibration.py`'s own ID3/ID4 ratio scale — a separate domain from the `gripper_open_tick`/`gripper_close_tick` above):** ratio 1.05 gave the most stable grip (ID3 load=-266, ID4 load=-175, hwerr=0x00), now recorded as `RECOMMENDED_GRASP_RATIO` in that script. `grasp_effort_thresh`/`drop_effort_thresh` themselves are still **not** finalized — the empty/grasp/drop 5-trial-each measurement (`thresholds` command) is still pending.
  - **IK note (HW-7 2026-07-05, 갱신 2026-08-07):** 기본 `ik_mode='analytic'`은 MoveGroup을 우회한다 — FK 서비스(`/compute_fk`, **별도 헬퍼 노드** `arm_fsm_fk_client`로 호출; `_tick` 타이머 콜백 안에서 `self`로 부르면 reentrant spin으로 데드락) + 유한차분 야코비안으로 **위치만** 맞추고(방향은 버림) `/arm_controller/joint_trajectory`에 직접 publish한다. MoveGroup 경로(`ik_mode='moveit'`)는 지우지 않고 남겨뒀다. 실기 end-to-end 검증됨: bottle detection → analytic IK → descend → gripper close → effort 기반 grasp 확인.
    - ⚠️ **"URDF가 3축뿐이라서"가 아니다.** URDF/SRDF는 5축이고, analytic을 쓰는 건 **solver를 아직 5DOF로 확장하지 않아서**다(코드 주석이 명시). 예전 문서의 "URDF가 3축만 모델링" 서술은 2026-07-15 URDF 교체로 폐기됐다.
    - ⚠️ **관절 수를 상수로 박지 말 것** — `f336d93`에서 3DOF 하드코딩(`range(3)`)을 제거하고 `ARM_JOINT_NAMES`(=`JOINT_CONFIG`, 현재 4개)에서 받아오게 고쳤다. 축이 4개가 되면서 `q + dq`가 shape `(4,)+(3,)`로 깨졌던 실제 버그다.

### robot_arm_description (ament_cmake)
Compiles nothing — `CMakeLists.txt` only installs `urdf/`, `launch/`, `rviz/`, `config/` to `share/`. Adding a resource dir requires adding it to the `install(DIRECTORY ...)` block.
- **`urdf/robot_arm.urdf` — 2026-07-31 CAD zip 전체(`2026_07_29_랙피니언_URDF_description.zip`) 재생성 (현재 소스, 5-DOF+그리퍼+손목카메라 통합).** 2026-07-15 최초 Isaac Sim 재export 이후 그리퍼 랙피니언 2모터 스왑(2026-07-16)·손목카메라 CAD 실측(2026-07-29)을 반영하려고 전체를 다시 자동 생성 스크립트로 재구성함 — **링크 번호가 또 바뀌었다**(`link_039`가 결번되고 `link_051`이 새 번호로 재등장 등, 이전 재export와 번호 대응 없음). 편집 전 항상 실제 URDF에서 번호를 재확인할 것. 결과: **52 link(`base_link` + `link_001`..`051`, `link_039`만 결번) / 51 joint**. 관절 이름은 `arm_joint_1..5` + 그리퍼 4개 — 구동은 `gripper_left_pinion_joint`(revolute) 하나뿐이고 나머지(`gripper_right_pinion_joint`/`gripper_left_rack_joint`/`gripper_right_rack_joint`)는 URDF `<mimic>`으로 종속(2026-07-16 랙피니언 스왑 이후 확정된 이름 — 그 이전 `gripper_a_joint5/6`나 첫 재export 때의 `gripper_drive_joint`는 더 이상 존재하지 않음, `gripper_a_*` 링크 접두어도 이번 재생성으로 사라짐, 순수 `link_0XX` 이름뿐). 그리퍼가 물리적으로 붙는 링크(=tip_link)는 `link_043`(아래 SRDF 참고) — URDF 레벨 그리퍼 교체(`urdf/grippers/*.xacro` 스왑)는 없고 그리퍼 모듈화는 **로직 레벨**(`dynamixel_control`의 `gripper_type`/`gripper_presets.py`)에서만 유지됨. `urdf/robot_arm.urdf.xacro`는 이 `robot_arm.urdf`를 그대로 `xacro:include`하는 얇은 wrapper(변경 없음).
- **손목 카메라는 2026-07-31부터 이 트리에 통합됨** — `wrist_camera_link`가 `robot_arm.urdf` 안에 `link_035` 기준 fixed joint(`fixed_joint_035`, rpy 포함 CAD zip 자동 재생성분, xyz는 2026-07-29 실측)로 직접 포함됐다. `camera_tf.launch.py`의 구 static TF는 제거됨(아래 참고) — `robot_state_publisher`가 팔 자세를 따라 자동 갱신. 디스플레이 없는 환경이라 이 세션에선 URDF 파싱(`check_urdf`)만 확인했고, RViz/실기로 카메라 방향 육안 검증은 아직 안 됨.
- `launch/display.launch.py`: `robot_arm.urdf.xacro`를 `xacro.process_file()`로 처리해 로드(예전엔 raw `.urdf` 직접 로드였으나 이미 xacro 경유로 전환됨) + robot_state_publisher + joint_state_publisher_gui + rviz2. RViz launches with no saved config, so the model is invisible until you set Fixed Frame to `base_link`, add a RobotModel display, and set its Description Topic durability to `Transient Local` (see README).
- `launch/camera_tf.launch.py`: **전방 RGB-D(차체 고정)만** static TF 발행 — `base_link→camera_link`(장착 오프셋 launch arg `cam_x/y/z`·`cam_roll/pitch/yaw`, 기본값은 **2026-08-07 `camera_tf_tuner`로 RViz에서 맞춘 벤치 실측** `x=-0.5526, y=-0.4469, z=0.1654, roll=0.0267, pitch=0.0210, yaw=0.4718` — ⚠️ **카메라를 책상에 올려둔 배치**라 차체 정식 장착 후 반드시 재측정할 것. 이전 CAD 추정값 `x=0.123, z=0.082, pitch=-0.26`은 launch 파일 주석에 보존돼 있음) + `camera_link→camera_color_optical_frame`(REP-103 optical 회전 `-π/2,0,-π/2` 고정). 손목 RGB는 2026-07-31부터 이 launch가 다루지 않음(위 `wrist_camera_link` 항목 참고 — URDF 통합으로 대체, 동시 발행 시 TF 충돌이라 제거됨). `perception_node`가 TF를 발행하지 않으므로, MoveIt이 `/pick_target`(camera frame) 목표를 `base_link`로 변환하려면 이 launch가 떠 있어야 함.
- `config/controllers.yaml`: `arm_controller`(`joint_trajectory_controller`)가 **`arm_joint_1`..`arm_joint_3`만**(update_rate 100) — URDF는 5축이지만 이 파일은 아직 실서보 배선이 끝난 3축만 등록(§ Watch out for 참고). 그리퍼 조인트 없음. `robot_arm_moveit_config/config/ros2_controllers.yaml`(아래)이 5축+그리퍼 전체를 다루는 더 최신/완전한 버전이니 실제로는 그쪽을 참고할 것.

### robot_arm_moveit_config (ament_cmake) — MoveIt 경로 계산용
Generated by MoveIt Setup Assistant; structure is complete and ready for motion planning. Use this package for path/trajectory planning.
- **Planning groups (`config/robot_arm.srdf`, 2026-07-31 URDF 재생성에 맞춰 재생성됨):** `arm` is the kinematic chain `base_link` → `link_043`(tip_link, `arm_joint_1..5`); `gripper` group = `gripper_left_pinion_joint`, end effector parented to `link_043`. **`tip_link`은 재export마다 계속 바뀌어 온 값**이다(`link_051`→`link_039`→현재 `link_043`, SRDF 주석에 이력 남아있음) — `arm_fsm_node.py`의 `tip_link` 파라미터 기본값과 반드시 동기화 유지할 것(2026-07-31 기준 둘 다 `link_043`으로 일치 확인). Named state `home` = all arm joints at 0. `gripper_open`/`gripper_closed` named states are **0/1.9444444444444444 placeholder**(캘리브 전, URDF 관절 리밋 그대로). Virtual joint `world` → `base_link` (fixed).
- **Collision matrix (`disable_collisions`)**: 두 종류로 구성 — "Adjacent"(부모-자식 조인트에서 기계적으로 100% 도출, 값 조작 없음)와 "Default"(**2026-07-31 재산출: `/check_state_validity`로 관절 리밋 안쪽 무작위 200샘플**(`arm_joint_1~5` + `gripper_left_pinion_joint`, seed=42) — 정식 MoveIt Setup Assistant GUI "Regenerate Default Collision Matrix"의 대체, 디스플레이 없는 환경이라 GUI 재실행 불가했음. 옛 40샘플·중간 3000샘플 결과와 같은 방법론이고, 3000샘플 원본은 `project_docs/collision_matrix_3000samples_2026-07-24.txt`에 남아있다 — 거기 "애매하게 충돌"로 분류된 65쌍은 **진짜 self-collision 위험이라 disable 금지**다). 그리퍼 완전 닫힘 자세는 200샘플이 못 훑어서 팔 무작위 30샘플로 별도 재검증(30/30 항상 충돌)했다. **⚠️ 200샘플도 Setup Assistant 기본값(수천)보다 적어 근거가 여전히 약함** — 디스플레이 있는 환경에서 기회 되면 GUI로 재생성해 이 블록을 정식 결과로 덮어쓸 것(SRDF 파일 안에 이 위험도 주석으로 남아있음).
- **IK solver (`config/kinematics.yaml`):** KDL (`kdl_kinematics_plugin/KDLKinematicsPlugin`) for the `arm` group.
- **Controllers:** MoveIt sends `FollowJointTrajectory` to `arm_controller` and `gripper_controller` (`config/moveit_controllers.yaml`); the matching `ros2_control` controllers are in `config/ros2_controllers.yaml` — `arm_controller`는 **`arm_joint_1..5`(5축 전체)**, `gripper_controller`는 `gripper_left_pinion_joint` (update_rate 100 Hz, position command interface). 이 파일이 `robot_arm_description/config/controllers.yaml`(3축만)보다 최신/완전하다.
- **`demo.launch.py` is mock-only, not real hardware.** `config/robot_arm.ros2_control.xacro` loads the `mock_components/GenericSystem` plugin (the SetupAssistant `FakeSystem`), so `demo.launch.py` plans against fake joints — it does **not** drive the physical Dynamixels, and you must **not** run it alongside the bridge (its mock `ros2_control_node` competes for `/joint_states` and `/arm_controller`). To execute MoveIt plans on **real servos**, run `move_group.launch.py` + `rsp.launch.py` and let `dynamixel_control`'s `moveit_dynamixel_bridge` act as the controller (it implements the `/arm_controller`+`/gripper_controller` action servers MoveIt drives — a lighter alternative to a full `ros2_control` HW interface). **2026-08-19부터 실서보가 5축 전부(`arm_joint_1`~`arm_joint_5`, ids 11/14/13/12/16) 배선돼 있다** — 베이스 요축 서보 장착이라는 선행 과제는 **해소됐고**, 이 파일의 5축 planning 결과와 브릿지의 축 구성이 이제 일치한다.
- **MoveIt mock demo works (재검증 2026-07-22).** `ros-humble-moveit` + `ros-humble-ros2-control` + `ros-humble-ros2-controllers` are in the Dockerfile. Run with:
  ```bash
  cd /root/ros2_ws && colcon build --packages-select robot_arm_description robot_arm_moveit_config
  source install/setup.bash && ros2 launch robot_arm_moveit_config demo.launch.py
  ```
  This brings up `move_group` + mock `ros2_control` + RViz MotionPlanning; all 3 controllers (`arm_controller`/`gripper_controller`/`joint_state_broadcaster`) go `active`, logs "You can start planning now!". Plan & Execute in RViz drives the *mock* joints only (not real servos).
- **Fixes baked in (don't regress):** (1) `urdf/robot_arm.urdf`의 Gazebo `ign_ros2_control/IgnitionSystem` 블록이 MoveIt mock `FakeSystem`과 충돌해 `ros2_control_node`를 크래시시켰던 문제 — 주석 처리됨(Gazebo Ignition 쓸 때만 재활성화). (2) `config/moveit_controllers.yaml`에 `action_ns: follow_joint_trajectory`가 빠져 MoveIt이 컨트롤러 0개로 봤던 문제 — 추가됨.
- The MoveIt SRDF uses link names `base_link`+`link_001`..`link_051`(`link_039`만 결번, 2026-07-31 재생성 기준 — `link_043`이 tip, SRDF `<chain tip_link="link_043"/>`/`<end_effector parent_link="link_043"/>` 확인됨); confirm these match `robot_arm_description/urdf/robot_arm.urdf` when editing the URDF, or planning/collision checks break. **이 번호 체계는 CAD zip을 재export할 때마다 통째로 바뀌어 왔다**(2026-07-15/07-16/07-31 세 번 모두 다른 번호) — 특정 `link_0XX`를 코드에 하드코딩하기 전에 항상 최신 URDF에서 재확인할 것.

### pick_test_pkg (ament_python)
Standalone gripper test: `pick_test_node` listens on `/fake_object_position` (`Point`) and sends a `FollowJointTrajectory` action to `/gripper_controller/follow_joint_trajectory` for `left_finger_joint`/`right_finger_joint`. ⚠️ **이 두 조인트 이름은 현재 URDF/SRDF에 존재하지 않는다**(랙피니언 교체 후 구동 조인트는 `gripper_left_pinion_joint` 하나) — 이 패키지만 옛 이름에 머물러 있어 지금 돌리면 컨트롤러가 거부한다. 쓰려면 먼저 조인트명을 맞출 것.

### robot_arm_gui (ament_python) — 브라우저 관제 GUI, **읽기 전용**
서보 진단(전류·온도·트립 여유)·관절·FSM/계약 상태·YOLO 인식·텔레옵 현황을 브라우저 한 페이지에서 본다. 지금까지 이 정보들이 서로 다른 터미널 로그에 흩어져 있어서 "왜 안 움직이지?"를 찾는 데 여러 창을 뒤져야 했다.

**entry point 2개**: `monitor`(= `telemetry_node:main`, 노드명 `/robot_arm_monitor`) + `fake_publisher`(하드웨어 없이 전 토픽을 실제 규약대로 발행하는 **벤치 검증 전용** — 실기와 같이 띄우면 가짜 `/dynamixel/state`가 진짜와 섞인다).

```bash
bash src/robot_arm_gui/scripts/run_monitor.sh              # 실기 옆에서
bash src/robot_arm_gui/scripts/run_monitor.sh fake:=true   # 하드웨어 없이 화면만 검증
bash src/robot_arm_gui/scripts/run_monitor.sh port:=8089   # 포트 변경도 launch 인자다
# 원격 PC:  ssh -L 8088:localhost:8088 <jetson>  후 http://localhost:8088
```

- **파일 구성** — `telemetry_node.py`(ROS 구독 전담·2Hz 조정 타이머) / `state_store.py`(락 보호 최신값 + 링버퍼) / `http_server.py`(SSE·MJPEG·정적 서빙) / `video_hub.py`(refcount 기반 동적 구독 + 단일 JPEG 인코더) / `hw_error_parse.py`·`topic_health.py`·`system_stats.py` / `fake_publisher.py` / `web/{index.html,app.js,style.css}` / `test/` 3개. **`state_store`·`hw_error_parse`·`topic_health`·`system_stats` 는 ROS 비의존**이라 하드웨어·ROS 없이 pytest 로 검증된다(`contract.py` 를 ROS 비의존으로 유지하는 것과 같은 사상).
- **`dynamixel_control` 을 직접 import 한다** (`package.xml` 의 `<exec_depend>`). `telemetry_node.py`·`fake_publisher.py` 가 `dynamixel_control.contract`(LOCK_MODES·DRIVE_READY_STATUSES·HEARTBEAT_TIMEOUT_S)와 `qos_profiles`(HEARTBEAT_QOS·ARRIVAL_QOS)를 그대로 쓴다 — **계약 어휘를 프론트엔드에 복사해 두면 언젠가 실제 게이트와 어긋나기 때문**이다. 그래서 GUI 만 빌드해도 `dynamixel_control` 이 install 돼 있어야 한다.
- **노드 파라미터는 10개인데 launch 로 넘어가는 건 6개뿐이다** — `bind`/`port`/`video_fps`/`video_quality`/`video_default_source`/`web_root` 는 launch 인자로 있고(그 외 `fake` 는 가짜 발행자 토글이라 노드 파라미터가 아니다), `warn_temp_c`(60)·`warn_current_ratio`(0.7)·`driver_node`·`joystick_node` 는 **`--ros-args -p` 로만** 바꾼다. `web_root` 를 소스 트리로 가리키면 HTML 한 줄 고칠 때마다 `colcon build` 를 안 돌려도 된다.
- ⚠️ **`setup.py` 의 `data_files` 는 `glob('web/*')` 라 하위 디렉터리를 못 담는다** — `web/` 은 평면으로 유지할 것.

- **툴킷은 파이썬 stdlib 뿐이다** — `http.server` + SSE(텔레메트리) + MJPEG(영상), JPEG 인코딩만 기존 `cv2`. 컨테이너에 flask/rosbridge/foxglove/web_video_server가 하나도 없고 Dockerfile 을 바꾸면 팀 전원이 arm64 이미지를 재빌드해야 하므로, **새 의존성 0개**로 맞췄다. headless Jetson 에서 X11 없이 쓰는 게 목적이라 RViz/PyQt 는 후보에서 뺐다.
- **퍼블리셔를 하나도 만들지 않는다.** `ros2 node info /robot_arm_monitor` 의 Publishers 가 `/rosout`·`/parameter_events` 뿐인 것으로 검증한다. `/dynamixel/goal_position` 은 **구독**한다 — 계약이 금지하는 건 발행이지 구독이 아니고, 목표 대비 오차를 볼 유일한 경로다. 파라미터도 읽기만 한다(특히 `teleop_core.publish_rate_hz` 는 set 하면 조그 속도가 틀어지므로 노출 금지).
- **기본 바인딩은 `127.0.0.1`**, 포트 **8088**(5000/5002/5003/5004 SRT·메타데이터와 26650~26700 DDS 대역을 피한 값). `network_mode: host` 라 `bind:=0.0.0.0` 은 곧바로 현장 네트워크 노출이다.
- **영상 소스 둘의 비용이 다르다 — 이걸 모르면 raw 도 부하가 있는 줄 안다.**
  - `raw`(`/perception/raw_image`)는 `perception_node` 캡처 스레드가 **게이트 없이 무조건** 발행한다(추론과 분리돼 있고, 모듈 주석이 "raw sender 는 YOLO/debug 를 기다리지 않는다"고 못 박았다) → 구독해도 인식 노드 일이 **늘지 않는다.** 검출 박스는 GUI 가 브라우저 canvas 에서 직접 그리므로(Jetson 비용 0) **평소엔 raw 로 충분하다.**
  - `debug`(`/perception/debug_image`)는 `get_subscription_count() > 0` 게이트 안에서 `color_img.copy()` + `_draw_debug` + imgmsg 변환을 **추론 주기마다** 한다 → 마스크·거리 표시가 필요할 때만 켠다.
  - 그래서 **구독 자체를 동적으로 만들고 파괴한다**(refcount 0 → `destroy_subscription`). 인수 시험: `ros2 topic info /perception/debug_image -v` 의 subscription count 가 브라우저 탭에 따라 `0 ↔ 1`. UI 기본값은 꺼짐이고, 인코딩은 클라이언트 수와 무관하게 **프레임당 1회**다.
- **`/dynamixel/hardware_error` 파싱에 쉼표 함정이 있다.** 항목 구분자는 `,` 인데 `HW_ERROR_BITS[7] = "전류급변(SW,비상정지)"` 라벨 **안에 쉼표가 있어** 단순 `split(",")` 이면 모터가 하나 더 생긴다. `hw_error_parse.py` 가 lookahead 로 자르고 pytest 로 고정했다. (기존 `keyboard_teleop` 은 원문을 출력만 해서 이 함정에 안 걸렸다 — GUI 가 처음 밟았다.)
- **트립 블랙박스**: hw 에러 상승 엣지에서 직전 3초(90샘플) 전 모터 `/dynamixel/state` 링버퍼를 동결해 `/api/trace/<id>.jsonl` 로 내려받는다. 트립 당시 수치는 노드에서 로그로만 남고 사라지는데, 이 데이터가 `DEFAULT_CURRENT_SPIKE_DELTA=350` 같은 미확정 임계값을 실측으로 좁힐 근거가 된다. GUI 는 노드의 급변 판정(`baseline = min(window)` **를 append 전에** 뜬다)을 그대로 재현해 "트립까지 남은 여유"를 실시간으로 보여준다.
- **화면이 "관측 불가"를 명시한다.** E-stop 래치·teleop stop 여부·입력 전압·트립 수치 상세·브릿지 경로의 관절별 HW 에러·FSM State 17종은 지금 구조상 알 수 없다. 운영자가 GUI 를 완전하다고 오인하지 않게 이유와 함께 띄운다. (**데드맨만은 관측 가능**하다 — `joystick_teleop` 의 `deadman_button` 파라미터를 읽기 전용으로 조회해 `/joy`.buttons 의 해당 인덱스를 보면 된다. 노드가 안 떠 있으면 실측 확정값 9 로 폴백하고 화면에 '기본값 가정'을 붙인다.)
- ⚠️ **`video_default_source` 에 `'off'` 를 쓰지 말 것** — launch 가 파라미터를 YAML 로 넘기는데 YAML 1.1 이 `off/on/yes/no` 를 불리언으로 강제 변환해 `InvalidParameterTypeException` 으로 노드가 죽는다. 값 어휘를 `none|debug|raw` 로 둔 이유다.
- `ament_pep257` 은 일부러 안 건다(D213 이 이 저장소의 docstring 관례와 정면 충돌 — 기존 패키지도 97건/32건 위반 중). flake8 만 건다.

## Watch out for

- **Joint-count mismatches across files are the #1 source of bugs here (재확인 2026-08-08).** URDF/SRDF는 5축(`arm_joint_1..5`)이다. 이름은 2026-08-07 기준 전부 `arm_joint_*`로 통일됐다(옛 `joint_1..5` 무접두 서술 폐기). 남은 실제 불일치는 **파일마다 등록 축이 다르다**는 것:

| 파일 | 등록 축 |
| --- | --- |
| `urdf/robot_arm.urdf` · `robot_arm.srdf` · `moveit_config/config/ros2_controllers.yaml` | `arm_joint_1`~`5` (전체) + 그리퍼 |
| `moveit_dynamixel_bridge.py`의 `JOINT_CONFIG`(→ `arm_fsm`의 `ARM_JOINT_NAMES`) | **`arm_joint_1`~`5` 5축** (ids 11/14/13/12/16) — 2026-08-19 ID 11 실장으로 4축에서 5축이 됨 |
| `robot_arm_description/config/controllers.yaml` | `arm_joint_1`~`3` 3축 (가장 낡음 — `moveit_config` 쪽이 최신) |
| `position_node`의 `motor_ids`/`joint_names` 기본값 | `arm_joint_1`~`5` + 그리퍼 = ids `[11,14,13,12,16,3]` (2026-08-19 ID 11 실장으로 **이제 실물과 일치**) |

  어느 하나를 고치면 나머지를 함께 확인할 것. 특히 **축 수를 코드에 상수로 박지 말 것** — `ARM_JOINT_NAMES`에서 받아오는 게 규약이고, 어긴 탓에 실제로 IK가 깨진 적이 있다(`f336d93`).

- Hardware nodes fail without the real devices: `position_node` / `moveit_dynamixel_bridge` need the servo bus on `/dev/ttyUSB0` (and must not share the bus — pick one runtime); `yolo_detection` / `perception_node` need a camera (RealSense for `perception_node`). All rely on `privileged` for device access.
- `wrist_camera_link` is now a URDF fixed joint (`robot_arm.urdf`, 2026-07-31) that `robot_state_publisher` updates as the arm moves — the old home-pose-only static TF placeholder (`camera_tf.launch.py`) is gone. Not yet verified on real hardware/RViz (no display in this dev session) — confirm camera orientation matches physical mounting before trusting pick geometry from it.
- **`ros2 run`/`ros2 launch` leak child nodes:** `kill <PID>`/`Ctrl-C` often kills only the wrapper, leaving the python node or `static_transform_publisher` running (→ CPU spin, `/arm_status` noise, stale TF). Clean up with `pkill -f <node>` and verify via `ps aux | grep ros2`.
  - **관제 GUI 는 증상이 다르게 나타난다** — 흘린 프로세스가 8088 을 잡고 있으면 브라우저는 **옛 프로세스에 계속 붙어 "값이 왜 안 바뀌지"** 로 보이고, 다시 띄우면 `OSError: [Errno 98] Address already in use` 로 죽는다(`allow_reuse_address` 는 TIME_WAIT 만 해소한다). 정리: `pkill -f "monitor[.]launch[.]py"; pkill -f "robot_arm_gui/[mf]"`.
  - ⚠️ **`pkill -f` 패턴이 자기 자신을 매칭하지 않게 할 것.** `pkill -f robot_arm_gui` 는 그 명령을 담은 셸까지 죽여서, 뒤에 이어 붙인 `colcon build` 가 **조용히 실행되지 않는다**(이번 작업에서 두 번 걸렸고, 빌드가 안 된 줄 모르고 옛 바이너리를 디버깅했다). 대괄호(`[m]onitor`)로 자기 매칭을 피한다 — `run_keyboard_bench.sh:74` 가 같은 이유로 쓰는 방식이다.
- Branch strategy: `main` stays stable; feature work on `feat/*` branches.
</content>
