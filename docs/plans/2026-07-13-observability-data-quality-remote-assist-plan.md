# Observability, Data Quality, and Remote Assist Implementation Plan

> **For Codex:** 이 계획은 테스트 우선으로 한 Task씩 구현하고, 각 Task의 검증 명령과 실제 출력이
> 확인된 뒤 다음 Task로 이동한다. 실물 HIL은 소프트웨어·simulation fault injection이 모두 끝난
> 마지막 단계에서 한 번에 수행한다.

**Goal:** 2025 출품작들에서 확인한 좋은 SW 관행을 현재 powertrain 아키텍처에 맞게 도입해,
실패 원인이 관측 가능하고 depth·시간·TF 품질이 수치화되며 원격주행과 원격 팔 조종이 네트워크
열화에도 L515·D435i 동시 30 fps와 공통 안전 경로를 보존하게 한다. DualSense 하나로 DRIVE와
ARM을 상호배타 전환하고 ARM에서는 5개 관절과 그리퍼를 개별 조작한다.

**Architecture:** 새 ROS wire schema를 만들지 않는다. 제어와 안전은
`/autonomy/cmd_vel|/teleop/cmd_vel → chassis_node 내부 CommandAuthority → ChassisManager` 경로를
유지하고 외부 final `/cmd_vel` 경계를 만들지 않는다. 관측성은 same-UID 보호 Linux
abstract socket을 통해 비차단 이벤트를 단일 journal daemon에 모으고, TUI는 L515 Gateway와
observability daemon을 독립적으로 조회한다. D435i raw 영상과 YOLO metadata는 분리 전송하고
노트북 receiver가 최신 결과만 합성한다. terrain·wheel consistency·remote assist는 ROS 없는
순수 Python 코어를 먼저 구현하고 얇은 ROS adapter만 추가한다. 원격 팔 input은 WP5.2의 단일
gateway와 로봇팔 `ArmCommandAuthority`를 재사용하며 관측 프로세스가 command owner가 되지 않는다.

**Tech Stack:** Python 3.10, pytest, rclpy/ROS2 Humble adapter, Textual, NumPy 기준 backend,
JAX qualification backend, Linux abstract Unix socket, `flock`, JSONL, existing x264/SRT pipeline.

---

## Global constraints

- **선행조건:** `2026-07-13-wp5.2-arm-collaboration-safety-plan.md`의 Task 1·3·4·5를 순서대로
  완료한 뒤 Task 1~3·5를 시작한다. Task 4는 WP6-B 앞, Task 6은 WP6-B/C 뒤, Task 7~8은
  P0/P1 simulation과 replay 뒤에 실행한다. 두 계획이 같은 파일을 동시에 수정하지 않는다.
- 팔 안전 어휘·`ArmInterlock`·mission FSM·command authority·CAN lock의 단일 소유자는 WP5.2다.
  이 계획은 해당 산출물을 수정하거나 재정의하지 않고 관측 adapter와 후속 기능만 추가한다.
  단 하나의 명시적 예외: Task 6은 `command_authority.py`에 remote-assist profile 연결(보정 합성
  hook)을 추가하는 범위에 한해 수정할 수 있으며, 상태 집합·전환 규칙·안전 gate 의미는 변경하지
  않는다.
- Verify 공통 규칙: 셸 체인은 `set -e`로 실패를 전파하고(pytest가 마지막 명령이 아닌 체인에서
  `;`가 실패를 은폐하지 않게), generated msg를 import하는 node 통합 시험은 WP5.2 계획의
  colcon `/tmp` 빌드 규칙(`--build-base /tmp/build --install-base /tmp/install` 후 source)을 따른다.
- L515 SDK owner와 SRT owner는 계속 `python3 -m l515_dashboard.gateway_main` 하나다.
- D435i SDK owner는 로봇팔 camera-owner process 하나다. 파워트레인은 D435i를 열거나 fallback하지
  않으며 로봇팔의 현행 `/perception/debug_image → SRT` 시험 경로를 production 입력으로 채택하지 않는다.
- observability 장애·디스크 지연·TUI 종료는 chassis 50 Hz, safety, 두 camera owner와 두 SRT
  sender를 중단시키지 않는다.
- mission journal과 health snapshot은 진단이지 command authority가 아니다.
- 새 PointCloud2 토픽, 중복 camera owner와 중복 encoder를 만들지 않는다. D435i의 승인된 arm-side
  raw sender 하나만 추가하며 powertrain process가 이를 복제하지 않는다.
- L515 RGB 1280×720×30과 D435i RGB 848×480×30을 동시에 우선 보존한다. L515 depth/overlay와
  진단 주기는 양보할 수 있지만 두 raw RGB stream의 해상도·FPS는 몰래 낮추지 않는다.
- CAN health는 실제 `flock` owner가 측정한 값만 권위 있게 표시한다.
- production threshold와 장착·TF 값은 qualification 뒤 YAML로 동결한다. run 중 자동 튜닝하지 않는다.
- 네트워크 profile, NumPy/JAX backend와 FP precision은 arm 전에 선택한다. 운용 중 backend 자동
  전환은 금지한다.
- 반자동 원격 보조는 운영자의 속도 의도와 공통 chassis safety gate를 우회하지 않는다.
- DualSense DRIVE/ARM gateway, remote-input schema와 arm command authority는 WP5.2 Task 4·7이
  소유한다. 이 계획은 해당 제어 계약을 바꾸지 않고 dual-video operator console, channel feedback와
  관측 이벤트만 연결한다.
- production Compose service는 `powertrain_observability`라는 별도 supervised process다.
  `network_mode: host`, 명시적 command/entrypoint, `PYTHONPATH=/workspace`,
  `/run/powertrain`·`/var/lib/powertrain/runs` bind, `restart: unless-stopped`와 healthcheck를 둔다.

## Recommendation traceability

| 2025 사례에서 취한 장점 | 현재 계획에 맞춘 적용 | 구현 Task |
|---|---|---|
| iRASC 상태전이 기록 | append-only mission journal | 1~2 |
| Dolbat 구동계 상태 가시화 | CAN 10-node health matrix | 3 |
| KUDOS·Angchicken 3D 품질 처리 | robust ROI depth와 표면 일관성 | 4 |
| KUDOS·RO:BIT 좌표계 검증 | sensor time/TF/known-target qualification | 4 |
| Dolbat 통신 열화 대응 | L515·D435i 동시 30 fps 정적 video profile | 6 |
| Zenith 운전자 보조 | 공통 authority 안의 semi-auto remote assist | 6 |
| Dolbat 구동 동기 진단 | wheel mismatch/virtual-shaft monitor | 3 |
| Dolbat·RO:BIT manipulator safety | WP5.2 팔 실패 결과의 journal adapter | 5 |
| RO:BIT 센서 설정 검증 | L515 commissioning과 YAML 동결 | 4, 8 |
| KUDOS 독립 통신 진단 | 채널별 health와 kill/restart matrix | 2, 7 |
| KUDOS·Dolbat 환경 강건성 | simulator/real replay 공통 regression set | 7 |

## Task 1: append-only mission journal과 health snapshot 코어

**Files:**

- Create: `powertrain_observability/__init__.py`
- Create: `powertrain_observability/events.py`
- Create: `powertrain_observability/journal.py`
- Create: `powertrain_observability/health.py`
- Create: `powertrain_observability/tests/test_events.py`
- Create: `powertrain_observability/tests/test_journal.py`
- Create: `powertrain_observability/tests/test_health.py`
- Modify: `docker/Dockerfile.ros`
- Modify: `docker/docker-compose.jetson.yml`

**Event contract:**

- 필수 필드: `schema_version`, `run_id`, `sequence`, `wall_time_ns`, `monotonic_ns`, `source`,
  `event_type`, `severity`, `payload`.
- `event_type`: `FSM_TRANSITION`, `COMMAND_OWNER`, `MOTION_HOLD`, `ESTOP`, `MISSION`, `ARM_RESULT`,
  `GRIP_LOST`, `CONTRACT_VIOLATION`, `OPERATOR_ACTION`, `TERRAIN_REJECT`, `CHANNEL_HEALTH`, `CAN_HEALTH`.
- unknown event와 추가 payload key는 보존하되 필수 필드 누락·NaN·무한대·oversize record는 거부한다.
- JSONL은 run마다 새 파일을 사용하고 sequence는 daemon이 단독 부여한다.

**Steps:**

1. event validation, deterministic JSON encoding, immutable health snapshot의 실패 테스트를 작성한다.
2. journal open/append/flush/rotate와 마지막 partial line 무시 복구 테스트를 작성한다.
3. bounded in-memory queue가 가득 차면 제어 producer를 block하지 않고 진단 drop counter만 올리는
   테스트를 작성한다.
4. 순수 코어를 최소 구현한다. 파일 flush 오류는 daemon health를 `DEGRADED`로 만들되 producer를
   멈추지 않는다.
5. `powertrain_ros` 서비스의 `PYTHONPATH`에 `/workspace`를 명시해 console-script로 실행되는
   `chassis_node`도 top-level `powertrain_observability`를 import하게 한다. `Dockerfile.ros`와
   Compose 양쪽에서 import smoke test를 고정한다.

**Verify:**

```bash
docker build -f docker/Dockerfile.ros -t powertrain-sw:ros docker
docker run --rm --entrypoint bash -v "$PWD:/workspace:ro" -w /workspace \
  -e PYTHONPATH=/workspace:/workspace/ros2/src/powertrain_ros:/workspace/motor_control \
  powertrain-sw:ros -lc 'source /opt/ros/humble/setup.bash; \
  python3 -c "import powertrain_observability"; \
  pytest -q powertrain_observability/tests'
```

## Task 2: singleton observability daemon과 기존 TUI의 독립 조회

**Files:**

- Create: `powertrain_observability/protocol.py`
- Create: `powertrain_observability/server.py`
- Create: `powertrain_observability/client.py`
- Create: `powertrain_observability/main.py`
- Create: `powertrain_observability/tests/test_protocol.py`
- Create: `powertrain_observability/tests/test_server.py`
- Create: `powertrain_observability/tests/test_process_integration.py`
- Modify: `l515_dashboard/app.py`
- Modify: `l515_dashboard/__main__.py`
- Modify: `l515_dashboard/tests/test_app.py`
- Modify: `docker/docker-compose.jetson.yml`
- Modify: `scripts/install_powertrain_runtime_dir.sh`

**Runtime contract:**

- event ingress: same-UID protected abstract datagram socket `@powertrain-observability-events`.
- status query: same-UID protected abstract stream socket `@powertrain-observability-status`.
- stream peer는 accept 직후 `SO_PEERCRED`로 UID를 확인한다. datagram은 socket의 `SO_PASSCRED`를 켜고
  매 packet의 `SCM_CREDENTIALS`를 검증한다. payload 안의 self-reported UID는 신뢰하지 않는다.
- singleton lock: `/run/powertrain/observability.lock`; stale lock file를 삭제하지 않고 `flock`만 사용.
- run logs: root-owned runtime installation으로 준비한 `/var/lib/powertrain/runs` 아래에 기록.
- TUI는 Gateway client와 observability client를 따로 poll한다. 어느 한쪽이 끊겨도 다른 client와
  프로세스를 stop/restart하지 않는다.

**Steps:**

1. 잘못된 UID, oversize datagram, malformed JSON, duplicate daemon, status client disconnect의 실패
   테스트를 작성한다.
2. server가 bounded queue와 journal core를 소유하고 최근 event·channel health·drop counter만 status
   snapshot으로 제공하게 구현한다.
3. TUI에 command owner, hold/E-stop source, segment/FSM, mission/arm result, 채널 health를 추가한다.
4. `q`, SIGHUP, status client 실패가 daemon·Gateway·ROS·SRT를 죽이지 않고 confirmed `Shift+Q`도
   L515 Gateway만 정지하는 기존 의미를 유지한다.
5. Compose가 `powertrain_observability` service를 `python3 -m powertrain_observability.main`으로
   실행한다. host network, `PYTHONPATH=/workspace`, runtime/data bind, healthcheck와 restart policy를
   명시하고 `/run/powertrain`과 `/var/lib/powertrain/runs` ownership을 검증한다.

**Verify:**

```bash
docker run --rm --entrypoint bash -v "$PWD:/workspace:ro" -w /workspace \
  -e PYTHONPATH=/workspace:/workspace/ros2/src/powertrain_ros:/workspace/motor_control \
  powertrain-sw:ros -lc 'source /opt/ros/humble/setup.bash; pytest -q \
  powertrain_observability/tests l515_dashboard/tests/test_app.py \
  l515_dashboard/tests/test_process_integration.py'
```

## Task 3: CAN 10-node health matrix와 wheel consistency monitor

**Files:**

- Modify: `motor_control/chassis/telemetry.py`
- Modify: `motor_control/chassis/chassis_manager.py`
- Modify: `motor_control/corner_module/steer_ak40.py`
- Modify: `motor_control/corner_module/drive_odrive_can.py`
- Create: `motor_control/corner_module/tests/test_driver_health_fields.py`
- Create: `motor_control/chassis/wheel_consistency.py`
- Create: `motor_control/chassis/tests/test_wheel_consistency.py`
- Modify: `motor_control/chassis/tests/test_chassis_manager.py`
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py`
- Modify: `l515_dashboard/app.py`
- Modify: `l515_dashboard/tests/test_app.py`
- Create: `ros2/src/powertrain_ros/test/test_observability_event_integration.py`
- Create: `l515_dashboard/tests/test_observability_matrix_integration.py`

**Health matrix fields:**

- AK 1~4: physical wheel, last feedback age, feedback rate, steer fault, stale, recovery count.
- ODrive 11~16: physical wheel, last heartbeat/encoder age, axis state/error, stale, recovery count.
- bus: rx/tx packet delta, error-warning, error-passive, bus-off delta, restart count.
- owner: PID, process name, lock path and acquisition time.
- interlock: current motion-hold sources, latched E-stop sources, reset-required 여부.

**Steps:**

1. snapshot schema와 10개 node의 정상/stale/fault/recovery 표시 테스트를 먼저 작성한다.
2. same-side wheel command/measurement delta, left-right wheel yaw와 IMU yaw mismatch, single-wheel
   spin/stop, command/encoder response ratio의 순수 monitor 테스트를 작성한다.
3. `ChassisManager.snapshot()`이 한 tick에서 immutable node health와 wheel consistency 결과를 만들게
   구현한다. 진단 계산은 모터 command 경로에서 blocking I/O를 추가하지 않는다. 요구 필드
   (feedback age/rate, recovery count, ODrive axis state)는 현행 드라이버 `state()`가 노출하지
   않으므로 `steer_ak40`/`drive_odrive_can`에 수동 집계 필드 추가를 포함한다(기존 수신 경로에서
   카운터만 갱신, 추가 I/O 없음).
4. 초기 production action은 `WARN`과 terrain-profile speed cap만 허용한다. 자동 wheel별 torque
   redistribution API는 만들지 않는다.
5. chassis node가 nonblocking observability event를 보내고 TUI가 10-node matrix를 표시하게 한다.
6. 실제 daemon socket fixture를 사이에 두고 chassis event가 TUI row까지 도달하는 통합시험과 daemon
   단절 시 control callback이 block되지 않는 시험을 작성한다. 순수 monitor 단위시험만으로 완료하지 않는다.

**Verify:**

```bash
docker run --rm -v "$PWD:/workspace" -w /workspace/motor_control powertrain-sw:dev \
  python3 -m pytest chassis/tests -q
docker run --rm --entrypoint bash -v "$PWD:/workspace:ro" -w /workspace \
  -e PYTHONPATH=/workspace:/workspace/ros2/src/powertrain_ros:/workspace/motor_control \
  powertrain-sw:ros -lc 'source /opt/ros/humble/setup.bash; pytest -q \
  ros2/src/powertrain_ros/test/test_observability_event_integration.py \
  l515_dashboard/tests/test_observability_matrix_integration.py'
```

## Task 4: robust depth 품질과 sensor time/TF qualification

**Files:**

- Create: `powertrain_autonomy/__init__.py`
- Create: `powertrain_autonomy/terrain/__init__.py`
- Create: `powertrain_autonomy/terrain/depth_quality.py`
- Create: `powertrain_autonomy/sensor_qualification.py`
- Create: `powertrain_autonomy/tests/test_depth_quality.py`
- Create: `powertrain_autonomy/tests/test_sensor_qualification.py`
- Create: `scripts/l515_commissioning.py`
- Create: `ros2/src/powertrain_ros/config/l515_terrain.yaml`
- Create: `docker/Dockerfile.autonomy`
- Modify: `docker/docker-compose.jetson.yml`

**Depth quality contract:**

- 입력: depth ROI, scale/intrinsics, frame stamp, 이전 품질 snapshot.
- 출력: robust depth, valid ratio, median, MAD, lower/upper percentile, temporal delta, connected ratio,
  normal consistency, confidence, reject reasons.
- center pixel 하나만으로 거리·높이 결정을 만들지 않는다.
- invalid·0·범위 밖 depth, isolated spike, depth hole, abrupt temporal jump와 disconnected lower floor는
  명시적 reject reason을 가진다.

**Qualification contract:**

- header stamp와 local receive clock 차이, equal/regressing stamp count, RGB/depth/IMU/wheel skew.
- `base_link→l515_link`, D435i optical axis·부호, known-target base-frame XYZ error.
- 팔 접힘·작업·운반·비정상 정지 자세별 L515 ROI occlusion과 extrinsic 반복성.
- pitch 20°/25°/30°별 near blind spot, 0.5~4 m coverage, footprint clearance, below-floor separation.

**Steps:**

1. 합성 ROI의 hole/spike/outlier/반사·비반사 대용/temporal jump 테스트를 먼저 작성한다.
2. robust median, MAD/percentile reject, connectivity와 normal consistency를 NumPy로 구현한다.
3. stamp skew, axis sign, TF stale와 known-target XYZ error의 qualification 테스트를 작성한다.
4. commissioning CLI가 각 후보 설정의 raw metrics와 pass/fail을 JSONL/CSV로 저장하게 한다.
5. 승인된 pitch·ROI·threshold·TF를 `l515_terrain.yaml`에 쓰고 SHA-256을 출력한다. CLI의 production
   mode는 측정만 하며 운용 중 YAML을 수정하지 않는다.
6. 20°/25°/30°를 반복 재현하는 브래킷·기준면 fixture는 기구팀 인계 입력으로 명시한다. SW는 각도별
   raw metric을 비교할 뿐 임시 고정 상태를 production angle로 추정하지 않는다.
7. L4T R36.5 aarch64에 맞춘 `powertrain_autonomy` image/service를 추가한다. terrain와 controller는
   같은 process에서 immutable dataclass를 주고받고 외부에는 `/autonomy/cmd_vel`만 발행한다. NumPy가
   첫 production backend이며 JAX/CUDA는 별도 전체부하 qualification 뒤에만 같은 image의 pin된
   profile로 활성화한다.

**Verify:**

```bash
docker build -f docker/Dockerfile.autonomy -t powertrain-sw:autonomy .
docker run --rm --entrypoint bash -v "$PWD:/workspace:ro" -w /workspace \
  -e PYTHONPATH=/workspace:/workspace/motor_control \
  powertrain-sw:autonomy -lc 'pytest -q \
  /workspace/powertrain_autonomy/tests/test_depth_quality.py \
  /workspace/powertrain_autonomy/tests/test_sensor_qualification.py'
```

## Task 5: WP5.2 arm 결과의 mission journal adapter

**Files:**

- Create: `powertrain_observability/arm_adapter.py`
- Create: `powertrain_observability/tests/test_arm_adapter.py`
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py`
- Create: `ros2/src/powertrain_ros/test/test_arm_observability_integration.py`
- Modify: `l515_dashboard/tests/test_app.py`

**Steps:**

1. WP5.2 필수 `FAILED`, `GRIP_LOST`, posture heartbeat와 선택 진단 8종을 각각 `ARM_RESULT` event로
   변환하는 테스트를 작성한다. adapter는 안전 결정을 만들지 않는다.
2. 실패가 current `mission_id`, arm posture, source detail과 함께 journal event로 보존되는 테스트를
   작성한다. 선택 진단 미지원은 `FAILED` 하나로 정상 기록한다.
3. WP5.2가 거부한 unknown status를 `CONTRACT_VIOLATION`으로 기록하고 TUI에 raw status·stamp를
   표시한다. adapter failure는 WP5.2의 hold 판정을 바꾸지 않는다.
4. `chassis_node`가 이미 산출한 immutable interlock result만 nonblocking event producer에 넘기고
   `contract.py`, `arm_interlock.py`, `mission_supervisor.py`를 이 Task에서 수정하지 않는다.
5. unknown/FAILED/GRIP_LOST fixture가 실제 daemon socket을 거쳐 TUI의 raw status·mission ID·hold
   reason까지 표시되는 통합시험을 둔다. adapter 단위시험만으로 완료하지 않는다.

**Verify:**

```bash
docker run --rm --entrypoint bash -v "$PWD:/workspace:ro" -w /workspace \
  -e PYTHONPATH=/workspace:/workspace/ros2/src/powertrain_ros:/workspace/motor_control \
  powertrain-sw:ros -lc 'source /opt/ros/humble/setup.bash; \
  pytest -q powertrain_observability/tests/test_arm_adapter.py \
  ros2/src/powertrain_ros/test/test_arm_observability_integration.py \
  l515_dashboard/tests/test_app.py'
```

## Task 6: dual-camera network profile과 semi-auto remote assist

**Files:**

- Create: `l515_dashboard/network_profiles.py`
- Modify: `l515_dashboard/config.py`
- Modify: `l515_dashboard/streamer.py`
- Modify: `l515_dashboard/gateway.py`
- Modify: `l515_dashboard/protocol.py`
- Create: `l515_dashboard/receiver_feedback.py`
- Create: `l515_dashboard/tests/test_receiver_feedback.py`
- Create: `l515_dashboard/tests/test_network_profiles.py`
- Modify: `l515_dashboard/tests/test_streamer.py`
- Modify: `l515_dashboard/tests/test_gateway.py`
- Create: `remote_video/__init__.py`
- Create: `remote_video/contract.py`
- Create: `remote_video/metadata.py`
- Create: `remote_video/tests/test_contract.py`
- Create: `remote_video/tests/test_metadata.py`
- Create: `scripts/recv_remote_operation.py`
- Create: `tests/test_recv_remote_operation.py`
- Modify after WP5.2 Task 4: `motor_control/laptop/remote_operation_client.py`
- Modify after WP5.2 Task 4: `motor_control/laptop/tests/test_remote_operation_client.py`
- Create: `ros2/src/powertrain_ros/powertrain_ros/remote_assist.py`
- Create: `ros2/src/powertrain_ros/test/test_remote_assist.py`
- Modify: `ros2/src/powertrain_ros/powertrain_ros/command_authority.py`
- Modify: `ros2/src/powertrain_ros/test/test_command_authority.py`

**Channel contract:**

- L515 driving RGB: H.264/SRT listener `:5000`, 1280×720×30.
- D435i arm RGB: H.264/SRT listener `:5002`, 848×480×30. 로봇팔 camera owner가 한 번 capture한
  raw RGB에서 직접 fan-out하며 YOLO 완료와 `/perception/debug_image`를 기다리지 않는다.
- D435i detection metadata: best-effort UDP JSON `:5003`. schema version, sender session ID,
  source frame sequence/capture stamp, bbox, class와 confidence를 포함한다. 16 KiB 초과, 미인식 version,
  같은 session의 역행 sequence와 비정상 bbox는 폐기한다. source stamp는 correlation·로그용이고
  Jetson과 노트북 clock의 직접 비교에는 쓰지 않는다. stale은 노트북 local receive monotonic TTL로 판정한다.
- YOLO worker 입력은 bounded latest-only slot이다. sender보다 느리면 backlog를 처리하지 않고 최신
  frame으로 건너뛴다. metadata rate는 inference rate 그대로이며 raw video 30 fps 인수조건이 아니다.
- 노트북 `recv_remote_operation.py`는 두 SRT를 동시에 decode/display하고 D435i metadata만 client-side
  합성한다. metadata age가 qualification threshold를 넘으면 bbox를 숨기고 `OVERLAY_STALE`을 표시한다.
  stale overlay나 packet loss가 raw frame 표시를 막지 않는다.
- `recv_remote_operation.py`는 Task 4의 `remote_operation_client`를 import해 DualSense를 한 번만
  연다. 별도 controller reader를 띄우지 않으며 DRIVE/ARM requested mode, Jetson ACK mode, selected
  joint, deadman, 각 gate freshness와 hold reason을 두 영상 옆에 표시한다. client 요청을 ACK처럼
  표시하지 않는다.
- 로봇팔 저장소의 D435i owner/sender는 cross-team deliverable이다. powertrain 쪽은 protocol fixture와
  receiver를 소유하고, 실제 SDK·sender 코드를 중복 구현하지 않는다.

**Static video profiles:**

- `NORMAL`: L515 1280×720×30과 D435i 848×480×30 raw RGB, approved bitrate, D435i metadata.
  L515 depth/overlay는 별도 동시 채널이 아니다 — 현행 L515 Gateway는 단일 GStreamer
  process/mode를 소유하므로 depth/overlay는 같은 `:5000` 스트림의 operator-selected **mode
  전환**으로만 제공한다. 전환 중 L515 RGB feedback은 stale이 되어 원격주행이 hold되며(의도된
  동작), mode 선택은 운영자 명시 행위로 journal에 기록한다. 원격주행 가용성 판정은 RGB mode
  복귀 후의 receiver feedback만 사용한다.
- `CONGESTED`: 두 raw stream의 resolution/FPS 유지, 각 bitrate를 qualification된 한 단계로 낮춤.
- `EMERGENCY_REMOTE`: 두 raw RGB를 유지하고 L515 depth/overlay SRT submit을 중단(L515 mode를
  RGB로 고정). D435i metadata는
  best effort로 계속 보내되 raw video나 command path를 block하지 않음.
- receiver는 채널별 decode/display fps, frame age, sequence gap, RTT/loss heartbeat를 역방향 control
  channel로 보낸다. 이 feedback가 원격 가용성의 authority이고 sender submit/sent/drop은 downgrade
  힌트다. L515 feedback stale은 `REMOTE_DRIVE_VIDEO_UNAVAILABLE`, D435i raw feedback stale은
  `REMOTE_ARM_VIDEO_UNAVAILABLE`이다. 진입과 복귀 threshold를 다르게 하고 최소 dwell time을 둔다.
- profile 전환은 기존 x264 process를 supervised replacement하는 경계에서만 수행한다. 운용 중
  새 encoder 종류나 임의 pipeline string은 허용하지 않는다.
- 최저 qualified bitrate의 `EMERGENCY_REMOTE`에서도 어느 raw receiver든 29 fps를 유지하지 못하면
  해상도·fps를 몰래 낮추지 않는다. L515 stale은 원격주행, D435i stale은 원격 팔 명령을 motion
  hold하고 reconnect를 계속 시도한다. companion stream 장애를 무관한 subsystem의 latched E-stop으로
  승격하지 않지만 dual-stream readiness acceptance는 실패시킨다.
- 전환 acceptance는 채널별 orphan/encoder overlap 0, 최대 blackout, 첫 IDR 수신시간과 복구 뒤
  동일한 첫 완전한 5초 window에서 두 receiver 모두 ≥29 fps로 판정한다. blackout 동안 마지막
  frame만 보고 명령을 계속하지 않는다.

**Remote arm video safety:**

- 원격 팔 명령은 fresh D435i raw receiver feedback, operator deadman, fresh remote input과
  `MISSION_STOP`/wheel-stop 계약을 모두 요구한다. 하나라도 stale이면 팔 명령을 hold하고 차체는
  `MISSION_STOP`을 유지한다. 이 경로는 chassis latched E-stop이 아니다.
- overlay metadata는 보조 정보이며 팔 명령 authority가 아니다. metadata 유실·stale은 overlay만
  숨기고 raw video와 deadman이 fresh하면 수동 팔 조종을 허용한다.
- L515와 D435i는 동시에 표시하지만 safety predicate는 operation별이다. L515 raw freshness는
  remote drive, D435i raw freshness는 remote arm을 gate한다. 비권위 companion 장애를 다른 동작의
  안전근거로 오판하지 않는다.
- 초기 후보 mapping은 `CREATE+OPTIONS` 1초 hold로 DRIVE/ARM 전환 요청, D-pad 좌/우로
  `joint_1`~`joint_5` 선택, 우스틱 Y로 selected joint signed jog, R2/L2로 gripper open/close,
  L1 hold-to-run이다. 구체적인 버튼·축은 HIL과 운전자 피드백 뒤 versioned config로 변경할 수 있다.
  mapping과 무관하게 같은 frame의 drive/arm 동시 출력 금지, trigger conflict hold와 전역 수동
  latched E-stop 의미는 유지한다.
- 원격 팔 control은 기존 `robot_arm_msgs` 5종을 변경하지 않는다. Task 4 gateway가 표준
  `control_msgs/msg/JointJog`를 만들고 로봇팔 `ArmCommandAuthority`가 FSM과 상호배타로 선택한다.
  console은 direct Dynamixel command와 all-zero `home`을 제공하지 않는다.
- ARM→DRIVE 요청 뒤 console은 `STOW_REQUEST/STOWING`을 표시하고 fresh `STOWED_LOCKED` ACK 전까지
  현재 ARM mapping의 입력을 차체 의미로 바꾸지 않고 출력 0으로 보낸다. client-side mode 선반영으로
  갑자기 차체가 움직이는 경로를 금지한다.

**Remote assist contract:**

- operator가 signed speed intent를 계속 제공한다.
- assist는 기본 OFF인 opt-in 기능이다. 조종기 전용 `ASSIST_BYPASS` 버튼 하나를 누르거나 유지하면
  다음 authority tick에서 correction을 0으로 만들고 raw teleop intent로 즉시 복귀한다. raw teleop도
  동일 `chassis_node` 내부 `CommandAuthority → final gate`를 통과한다.
- assist는 terrain center/heading correction, bank/clearance speed cap, lead-distance cap, 작업점
  alignment와 zero-confirmed stop만 제공한다.
- terrain/target confidence가 낮거나 stale이면 해당 correction을 제거하고 속도 상한을 낮춘다.
- operator deadman·remote freshness 상실은 motion hold다. assist 출력은 `/teleop/cmd_vel` source를
  `chassis_node` 내부 `CommandAuthority`가 선택한 뒤 합성하며 외부 final `/cmd_vel` 경계를 만들지 않는다.

**Steps:**

1. 두 receiver feedback freshness, profile hysteresis, dwell, 동시 RGB 30 fps invariant와 L515
   overlay/depth disable의 순수 테스트를 작성한다.
2. D435i metadata schema, sequence/stamp, TTL, packet loss·reorder·oversize와 stale overlay hide를
   시험하고 YOLO가 지연돼도 raw receiver cadence가 독립임을 fake sender로 검증한다.
3. operator speed ownership, correction clamp, confidence degradation, stale, deadman,
   `ASSIST_BYPASS` 1-tick 해제와 zero-confirmed handover 테스트를 작성한다.
4. DRIVE/ARM request와 ACK 불일치, joint selection, trigger conflict, deadman release, D435i stale,
   reconnect session 교체와 stow-before-drive를 console fixture로 시험한다.
5. static profile state machine, dual receiver와 supervised x264 replacement를 구현한다.
6. remote assist 순수 함수를 구현하고 command authority의 remote profile에만 연결한다.
7. 채널별 profile·receiver health·overlay age, DRIVE/ARM authority, selected joint, hold reason과 assist
   개입량·해제 원인을 observability event로 기록한다.

**Verify:**

```bash
docker run --rm --entrypoint bash -v "$PWD:/workspace:ro" -w /workspace \
  -e PYTHONPATH=/workspace:/workspace/ros2/src/powertrain_ros:/workspace/motor_control \
  powertrain-sw:ros -lc 'source /opt/ros/humble/setup.bash; pytest -q \
  l515_dashboard/tests/test_network_profiles.py \
  l515_dashboard/tests/test_receiver_feedback.py \
  l515_dashboard/tests/test_streamer.py l515_dashboard/tests/test_gateway.py \
  remote_video/tests/test_contract.py remote_video/tests/test_metadata.py \
  tests/test_recv_remote_operation.py \
  motor_control/laptop/tests/test_remote_operation_client.py \
  ros2/src/powertrain_ros/test/test_remote_assist.py \
  ros2/src/powertrain_ros/test/test_command_authority.py'
```

## Task 7: 환경 regression set과 독립 channel fault injection

**Files:**

- Create: `tests/fixtures/environment/manifest.yaml`
- Create: `tests/fixtures/environment/scenario.schema.yaml`
- Create: `tests/fixtures/environment/README.md`
- Create: `scripts/run_autonomy_regression.py`
- Create: `scripts/fault_injection/run_channel_matrix.sh`
- Create: `tests/test_environment_manifest.py`
- Create: `tests/test_channel_fault_matrix.py`
- Modify: `docs/plans/2026-07-12-defense-robot-autonomy-software-plan.md`

**Fixture classes:**

- fog/smoke, shadow/backlight, reflective/nonreflective surface.
- arm/payload occlusion, depth hole/jump, below-floor mixture.
- partial lead-robot occlusion, marker duplicate/false positive.
- bank entry/exit transition, wheel slip/stuck and one-wheel mismatch.

각 fixture는 stable ID, source(`analytic`, `mujoco`, `isaac`, `real-replay`), expected result,
allowed tolerance, sensor contract version과 checksum을 가진다. 실제 대회 지도나 정확한 트랙 사전
지식을 요구하지 않는다.

simulator-neutral `scenario.yaml`은 SI 단위, frame, PRNG algorithm/seed, track·sensor·fault parameter와
expected metric을 가진다. P0는 analytic/replay/MuJoCo fast, P1은 hidden-seed closed loop,
P2/stretch는 vcan 10모터와 Isaac adapter다. 일정상 P2를 P0/P1보다 먼저 요구하지 않는다.

**Steps:**

1. manifest schema와 checksum drift 실패 테스트를 작성한다.
2. regression runner가 같은 fixture ID의 backend별 결과를 비교하고 fail-open, false hold, minimum
   clearance, runtime, reject reason을 기록하게 한다.
3. channel matrix가 ROS/DDS, L515/D435i SRT receiver, D435i metadata, remote input, arm heartbeat,
   CAN telemetry와 두 camera owner를 하나씩 kill/restart하고 operation별 기대 hold·TUI 표시·journal
   event·고아 process 수를 검사하게 한다.
4. production process kill은 Compose supervised replacement를 사용하고 Gateway RSUSB pipeline을
   in-process 재사용하지 않는다.

**Verify:**

```bash
docker run --rm --entrypoint bash -v "$PWD:/workspace:ro" -w /workspace \
  -e PYTHONPATH=/workspace:/workspace/ros2/src/powertrain_ros:/workspace/motor_control \
  powertrain-sw:ros -lc 'set -e; source /opt/ros/humble/setup.bash; \
  pytest -q tests/test_environment_manifest.py tests/test_channel_fault_matrix.py; \
  python3 scripts/run_autonomy_regression.py \
  --manifest tests/fixtures/environment/manifest.yaml --dry-run'
```

## Task 8: 최종 Jetson/HIL acceptance와 production 동결

**Files:**

- Create: `docs/reports/2026-08-23-observability-data-quality-remote-assist-hil.md`
- Modify: `ros2/src/powertrain_ros/config/l515_terrain.yaml`
- Modify: `docker/docker-compose.jetson.yml`
- Modify: `AGENTS.md`
- Modify: `.claude/CLAUDE.md`

**Software acceptance before HIL:**

1. 전체 관련 pytest와 기존 motor/L515/ROS 회귀시험 통과.
2. MuJoCo hidden seed와 Isaac/real replay 환경 manifest 통과.
3. observability daemon kill·디스크 오류가 chassis 50 Hz와 Gateway RGB에 영향을 주지 않음.
4. network profile 전환에서 두 sender 각각 orphan/overlap 0, blackout·첫 IDR 한계 충족, 복구 뒤
   동일한 첫 완전한 5초 window에서 L515·D435i receiver 각각 29 fps 이상.
5. remote assist stale·사망·disable에서 마지막 correction 유지 0.
6. WP5.2의 cross-container CAN lock과 ODrive 11~16 wheel-stop YAML이 qualified 상태임.

**One-shot HIL sequence:**

1. 10개 CAN node matrix 정상/전원단절/stale/fault/복구 식별.
2. L515 pitch 20°/25°/30°, known-target XYZ, 팔/물자 가림, depth hole·아래 바닥·뱅크 기록.
3. ROS, L515/D435i SRT, D435i metadata, remote, arm, CAN과 두 camera owner를 하나씩
   분리·재연결해 독립 channel 진단과 recovery 확인.
4. 제한된 네트워크에서 `NORMAL → CONGESTED → EMERGENCY_REMOTE → NORMAL` hysteresis와 두 raw
   RGB 30 fps를 확인한다. L515 feedback stale은 원격주행, D435i raw feedback stale은 원격 팔
   조종을 hold하고 metadata stale은 overlay만 숨기는지 검증한다.
5. 바퀴 부양 상태에서 반자동 원격 centering·speed cap·`ASSIST_BYPASS` 즉시 해제,
   zero-confirmed stop과 모든 chassis gate.
6. 필수 `FAILED`와 지원되는 선택 팔 진단을 포함한 mission journal 상관관계와 locked-posture resume gate.
7. 30분 전체부하에서 chassis 50 Hz, 같은 5초 window의 L515·D435i receiver 각각 29 fps 이상,
   YOLO backlog 0, OOM 0, journal drop 0 또는 승인된 상한 이하, 고아 process 0.

**Freeze:**

- 승인된 YAML, container image digest, release commit, environment manifest checksum, TF와 video profile
  값을 HIL 보고서에 기록한다.
- 대회 시작 후 자동 parameter tuning, backend switch, 새 encoder, 실험 torque redistribution을
  비활성 상태로 고정한다.
- `git diff --check`, 전체 test 명령, Jetson exact-HEAD와 Compose process 상태를 보고서 마지막에
  원문으로 보존한다.
- HIL 결과가 운용 규칙을 바꾸면 `AGENTS.md`와 `.claude/CLAUDE.md`를 같은 커밋에서 동일하게
  갱신한다. 한쪽만 수정한 상태로 freeze하지 않는다.
