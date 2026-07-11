# L515 Direct pyrealsense2 Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** `powertrain_ros`에서 pyrealsense2 2.50.0으로 지정 L515의 color/depth/accel/gyro를 경량 ROS2 토픽으로 발행한다.

**Architecture:** realsense-ros를 사용하지 않는다. 하드웨어 독립 변환은 `l515_adapter.py`, SDK worker/reconnect는 `l515_source.py`, rclpy 발행은 `l515_node.py`로 분리한다. stream별 최신값 1개만 유지하고 지정 serial 외 장치를 절대 열지 않는다.

**Tech Stack:** ROS2 Humble, Python 3.10, pyrealsense2/librealsense 2.50.0 RSUSB, NumPy, sensor_msgs, pytest, Docker, Jetson Orin Nano

## Global Constraints

- 변경 SDK는 `powertrain_ros`뿐이다. `powertrain_jetson`과 로봇팔 컨테이너는 변경하지 않는다.
- librealsense/pyrealsense2는 정확히 `v2.50.0`이다. realsense-ros와 `ros-humble-librealsense2`를 설치하지 않는다.
- L515 serial `00000000F0271544` 필수. 빈값과 D435i `250222071245`는 거부한다.
- color/depth는 640×480×30, accel/gyro는 장치 기본 지원 주기다.
- PointCloud2·IR·confidence·alignment·합성 IMU·후처리는 없다.
- worker는 블로킹 SDK 호출을 맡고 ROS callback은 블로킹하지 않는다.
- stream별 bounded latest-value queue 크기는 1이다.
- firmware는 변경하지 않는다.
- Jetson checkout과 미추적 `motor_control/vision/tests/`를 보존한다.

---

### Task 1: Build pyrealsense2 2.50.0 in powertrain_ros

**Files:**
- Modify: `docker/Dockerfile.ros`
- Create: `ros2/src/powertrain_ros/test/test_l515_image_contract.py`

**Interfaces:**
- Produces: `/usr/local/lib/python3.10/dist-packages/pyrealsense2`, SDK tools, no wrapper packages.

- [ ] Write RED tests asserting `ARG LIBREALSENSE_TAG=v2.50.0`, `FORCE_RSUSB_BACKEND=ON`, `BUILD_PYTHON_BINDINGS=ON`, `PYTHON_EXECUTABLE=/usr/bin/python3`, and absence of `realsense-ros`, `ros-humble-librealsense2`, `pip3 install pyrealsense2`.
- [ ] Run `/home/light/anaconda3/bin/pytest -q ros2/src/powertrain_ros/test/test_l515_image_contract.py`; expect failures.
- [ ] Replace the abandoned wrapper block with a librealsense-only source build:

```dockerfile
ARG LIBREALSENSE_TAG=v2.50.0
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git pkg-config python3-dev \
        libusb-1.0-0-dev libssl-dev libudev-dev \
    && git clone --depth 1 --branch ${LIBREALSENSE_TAG} \
        https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_PYTHON_BINDINGS=ON \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_UNIT_TESTS=OFF -DBUILD_TOOLS=ON \
    && cmake --build /tmp/librealsense/build -j2 \
    && cmake --install /tmp/librealsense/build \
    && PY_SO=$(find /tmp/librealsense/build -name 'pyrealsense2*.so' -print -quit) \
    && test -n "$PY_SO" \
    && cp "$PY_SO" /usr/local/lib/python3.10/dist-packages/ \
    && ldconfig \
    && rm -rf /tmp/librealsense /var/lib/apt/lists/*
```

- [ ] Run static tests and `git diff --check`; expect PASS.
- [ ] Build `powertrain-sw:ros-l515-plan`, then run:

```bash
docker run --rm powertrain-sw:ros-l515-plan python3 -c '
import pyrealsense2 as rs
print(rs.__file__)
print(rs.context())
'
```

Expected: import succeeds under `/usr/bin/python3`; image contains no `realsense2_camera` package.
- [ ] Commit: `build(ros): pin L515 pyrealsense2 2.50.0`.

---

### Task 2: Implement Hardware-Independent ROS Message Adapters

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/l515_adapter.py`
- Create: `ros2/src/powertrain_ros/test/test_l515_adapter.py`

**Interfaces:**
- Produces:
  - `TimestampMapper.map_ms(device_ms: float, ros_now_ns: int) -> int`
  - `image_from_array(array, encoding, frame_id, stamp) -> sensor_msgs.msg.Image`
  - `camera_info_from_intrinsics(intrinsics, frame_id, stamp) -> CameraInfo`
  - `imu_from_vector(vector, kind, frame_id, stamp) -> Imu`

- [ ] Write RED tests with fake intrinsics/vector objects for image dimensions/step/bytes, K/D/P, gyro/accel fields, orientation covariance `-1`, timestamp shared offset, backward timestamp reset.
- [ ] Run adapter tests; expect import failure.
- [ ] Implement `TimestampMapper` with one shared offset and reset on decreasing device time.
- [ ] Implement Image conversion without cv_bridge; accept contiguous NumPy arrays and copy `tobytes()`.
- [ ] Implement CameraInfo using `fx/fy/ppx/ppy`, mapped distortion model, coefficients, and matching stamp/frame.
- [ ] Implement raw Imu conversion; reject kind other than `gyro`/`accel` with `ValueError`.
- [ ] Run adapter tests in ROS test image; expect all PASS.
- [ ] Commit: `feat(l515): add ROS message adapters`.

---

### Task 3: Implement Serial-Locked SDK Source and Reconnect State

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/l515_source.py`
- Create: `ros2/src/powertrain_ros/test/test_l515_source.py`

**Interfaces:**
- Produces `L515Config`, `L515State`, `LatestFrames`, `L515Source.start()`, `stop()`, `poll_latest()`.
- `poll_latest()` is nonblocking and returns/clears the latest stream payloads.

- [ ] Write RED tests using a fake `rs` module: expected serial selection, empty/D435 serial rejection, exact 640×480×30 requests, accel/gyro enabled, queue overwrite, disconnect→2-second reconnect, no stale replay, timestamp mapper reset.
- [ ] Implement immutable `L515Config` with expected serial default and validation.
- [ ] Implement `LatestFrames` with a lock and one slot per stream.
- [ ] Implement worker thread: query expected serial, configure four streams, wait for frames, store latest, stop on exception, retry only expected serial after interval.
- [ ] Ensure `stop()` joins the worker with bounded timeout and pipeline stop is best-effort.
- [ ] Run source tests; expect PASS and no sleeping real time (inject clock/wait function).
- [ ] Commit: `feat(l515): add serial-locked SDK source`.

---

### Task 4: Add rclpy Node, Config, and Launch

**Files:**
- Create: `ros2/src/powertrain_ros/powertrain_ros/l515_node.py`
- Create: `ros2/src/powertrain_ros/config/l515.yaml`
- Create: `ros2/src/powertrain_ros/launch/l515.launch.py`
- Create: `ros2/src/powertrain_ros/test/test_l515_node.py`
- Create: `ros2/src/powertrain_ros/test/test_l515_launch_contract.py`
- Modify: `ros2/src/powertrain_ros/setup.py`
- Modify: `ros2/src/powertrain_ros/package.xml`

**Interfaces:**
- Publishes `/l515/color/image_raw`, `/l515/color/camera_info`, `/l515/depth/image_rect_raw`, `/l515/depth/camera_info`, `/l515/gyro/sample`, `/l515/accel/sample`.

- [ ] Write RED node tests with fake source verifying nonblocking timer drain, matching image/CameraInfo stamps, correct frame IDs, and shutdown stop.
- [ ] Write RED launch/config tests for exact serial, 640×480×30, reconnect 2.0, installed config/launch, console entry point.
- [ ] Implement node publishers with sensor-data QoS and a 200 Hz nonblocking drain timer.
- [ ] Convert SDK payloads through Task 2 adapters only; do not import NumPy conversion logic into node.
- [ ] Add YAML parameters and a launch containing one `powertrain_ros` executable.
- [ ] Register `l515_camera = powertrain_ros.l515_node:main`; install config and launch; add `sensor_msgs` dependency.
- [ ] Run node/launch tests and clean three-package ROS build/test; expect existing 32 plus new tests PASS.
- [ ] Commit: `feat(ros2): publish lightweight L515 streams`.

---

### Task 5: Add USB3/SDK Preflight and Operations Documentation

**Files:**
- Create: `scripts/l515_preflight.sh`
- Create: `ros2/src/powertrain_ros/test/test_l515_preflight.py`
- Modify: `README.md`, `ros2/README.md`, `AGENTS.md`

- [ ] Write RED subprocess tests with fake `lsusb`, sysfs speed, and container Python output for: success, missing USB, 480 Mbps rejection, SDK missing serial, wrong serial.
- [ ] Implement fail-closed preflight: PID `8086:0b64`, sysfs speed ≥5000, `powertrain_ros` pyrealsense context contains only the requested selection.
- [ ] Add exact build/source/launch commands and state PointCloud2 absence, D435i ownership, SDK pin.
- [ ] Run all new pytest files, clean ROS build/test, `bash -n`, and `git diff --check`.
- [ ] Commit: `docs(l515): add preflight and operations`.

---

### Task 6: Jetson L515-Only HIL

**Files:**
- Create: `docs/reports/2026-07-11-l515-lightweight-pipeline-hil.md`

- [ ] Audit Jetson checkout/processes; preserve its 41 local commits and untracked vision tests.
- [ ] Transfer exact branch commit with `git archive` to `/tmp/powertrain-l515-$COMMIT`; build a uniquely tagged image without retagging production.
- [ ] Run pyrealsense2 enumeration. Require serial `00000000F0271544`; record firmware/profiles. Stop if firmware <1.5.8.1.
- [ ] Launch only L515 node and measure exactly 60 seconds after discovery/warm-up: counts, mean Hz, complete 5-second minima, max interval, stamp monotonicity, max age, CPU/RAM, USB errors.
- [ ] Require color/depth mean ≥29 Hz and every complete 5-second window ≥28 Hz; accel/gyro present; PointCloud2 absent.
- [ ] Ask user before unplug; verify topic stop, no D435 fallback, and reconnect recovery.
- [ ] Record evidence and commit: `docs(l515): record single-camera HIL`.

---

### Task 7: D435i Concurrent HIL and Closure

**Files:**
- Modify: L515 HIL report, project state, autonomous kickoff, README/ROS README/AGENTS.md
- Update: relevant Notion pages after fetch, then re-fetch.

- [ ] Re-audit owners; start robot-arm `ros2_humble` and `/root/ros2_ws/run_perception.sh` without editing its checkout.
- [ ] Confirm D435IF serial `250222071245` and `/detected_objects` continuity.
- [ ] Repeat identical L515 60-second metrics under concurrent load; require both cameras remain available and no mutual reset.
- [ ] Run all automatic L515 tests plus full clean ROS regression.
- [ ] Update authorities/Notion with measured topics, IMU rates, resource cost, PointCloud2 disabled, and remaining static-TF commissioning.
- [ ] Dispatch final whole-branch review, fix Important/Critical findings, verify again.
- [ ] Commit/push feature and finish branch integration only after review.
