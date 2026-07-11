# L515 Lightweight ROS2 Pipeline Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Jetson의 `powertrain_ros` 컨테이너에서 지정 L515의 color/depth/accel/gyro만 안정적으로 발행하는 경량 ROS2 파이프라인을 만든다.

**Architecture:** `powertrain_ros` 이미지에 librealsense 2.50.0과 realsense-ros 4.0.1을 고정된 외부 underlay로 빌드한다. 프로젝트는 L515 serial을 강제하는 launch/config와 preflight만 소유하며, D435i 로봇팔 컨테이너는 변경하지 않는다. PointCloud2·IR·alignment는 기본 비활성이고 실제 L515 단독 및 D435i 동시 HIL로 주기·자원·비간섭을 검증한다.

**Tech Stack:** Docker, ROS2 Humble, librealsense 2.50.0 RSUSB, realsense-ros 4.0.1, launch_ros, pytest, Bash, Jetson Orin Nano

## Global Constraints

- librealsense는 정확히 `v2.50.0`, realsense-ros는 정확히 `4.0.1`을 사용한다.
- 변경 대상 SDK는 `powertrain_ros` 이미지뿐이다. `powertrain_jetson`과 로봇팔 `ros2_humble` 이미지는 변경하지 않는다.
- L515 serial `00000000F0271544`를 항상 명시한다. 빈 serial이나 D435i serial `250222071245`를 허용하지 않는다.
- namespace와 camera name은 `l515`다.
- color/depth는 640×480, 30 Hz를 요청한다.
- accel/gyro 원본 토픽을 사용하고 wrapper의 합성 IMU를 만들지 않는다.
- PointCloud2, infrared, confidence, depth alignment, 후처리는 기본 비활성이다.
- `base_link → l515_link` 수치는 차체 조립 전 임의로 만들지 않는다.
- L515 firmware는 이 계획에서 변경하지 않는다.
- Jetson `~/power-train-sw`의 기존 커밋과 미추적 `motor_control/vision/tests/`를 보존한다. 배포는 덮어쓰기나 reset 없이 별도 검증 checkout/복사본으로 수행한다.

## File Structure

| 파일 | 책임 |
|---|---|
| `docker/Dockerfile.ros` | L515용 librealsense/realsense-ros 고정 underlay 빌드 |
| `ros2/src/powertrain_ros/config/l515.yaml` | serial과 경량 스트림 설정의 단일 정본 |
| `ros2/src/powertrain_ros/launch/l515.launch.py` | 표준 wrapper launch 포함 및 설정 전달 |
| `ros2/src/powertrain_ros/setup.py` | config/launch 설치 |
| `ros2/src/powertrain_ros/test/test_l515_image_contract.py` | Docker 버전·격리 계약 |
| `ros2/src/powertrain_ros/test/test_l515_launch_contract.py` | launch/config 안전 계약 |
| `scripts/l515_preflight.sh` | USB3·serial·SDK·중복 점유 사전검사 |
| `ros2/src/powertrain_ros/test/test_l515_preflight.py` | fake command 환경의 preflight 회귀시험 |
| `docs/reports/2026-07-11-l515-lightweight-pipeline-hil.md` | 단독/동시 HIL과 자원 측정 결과 |
| `README.md`, `ros2/README.md`, `AGENTS.md` | 운영법·현재 상태·에이전트 정본 |

---

### Task 1: Pin the L515 SDK Underlay in the ROS Image

**Files:**
- Modify: `docker/Dockerfile.ros`
- Create: `ros2/src/powertrain_ros/test/test_l515_image_contract.py`

**Interfaces:**
- Consumes: Docker build context `docker/`, ROS Humble base image.
- Produces: `/opt/realsense_ros_ws/install/setup.bash`, `rs-enumerate-devices` 2.50.0, ROS packages `realsense2_camera`, `realsense2_camera_msgs`, `realsense2_description`.

- [ ] **Step 1: Write failing static image-contract tests**

```python
from pathlib import Path


ROOT = Path(__file__).resolve().parents[4]
DOCKERFILE = ROOT / "docker" / "Dockerfile.ros"


def dockerfile_text() -> str:
    return DOCKERFILE.read_text(encoding="utf-8")


def test_ros_image_pins_l515_supported_versions():
    text = dockerfile_text()
    assert "ARG LIBREALSENSE_TAG=v2.50.0" in text
    assert "ARG REALSENSE_ROS_TAG=4.0.1" in text
    assert "FORCE_RSUSB_BACKEND=ON" in text


def test_ros_image_builds_realsense_as_separate_underlay():
    text = dockerfile_text()
    assert "/opt/realsense_ros_ws/src/realsense-ros" in text
    assert "/opt/realsense_ros_ws/install/setup.bash" in text


def test_ros_image_does_not_install_latest_pyrealsense_wheel():
    text = dockerfile_text()
    assert "pip3 install pyrealsense2" not in text
```

- [ ] **Step 2: Run the tests and confirm RED**

Run:

```bash
pytest -q ros2/src/powertrain_ros/test/test_l515_image_contract.py
```

Expected: at least the version-pin assertions fail because `Dockerfile.ros` currently installs no RealSense SDK.

- [ ] **Step 3: Add the pinned SDK and wrapper build**

Append before `WORKDIR` in `docker/Dockerfile.ros`:

```dockerfile
ARG LIBREALSENSE_TAG=v2.50.0
ARG REALSENSE_ROS_TAG=4.0.1

RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git pkg-config \
        libusb-1.0-0-dev libssl-dev libudev-dev \
        python3-rosdep \
    && rm -rf /var/lib/apt/lists/* \
    && git clone --depth 1 --branch ${LIBREALSENSE_TAG} \
        https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
    && cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_TOOLS=ON \
    && cmake --build /tmp/librealsense/build -j2 \
    && cmake --install /tmp/librealsense/build \
    && ldconfig \
    && rm -rf /tmp/librealsense \
    && mkdir -p /opt/realsense_ros_ws/src \
    && git clone --depth 1 --branch ${REALSENSE_ROS_TAG} \
        https://github.com/IntelRealSense/realsense-ros.git \
        /opt/realsense_ros_ws/src/realsense-ros \
    && (rosdep init 2>/dev/null || true) \
    && rosdep update \
    && . /opt/ros/humble/setup.sh \
    && rosdep install --from-paths /opt/realsense_ros_ws/src \
        --ignore-src --rosdistro humble -y \
    && . /opt/ros/humble/setup.sh \
    && cd /opt/realsense_ros_ws \
    && colcon build --merge-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && test -f /opt/realsense_ros_ws/install/setup.bash
```

Do not alter `docker/Dockerfile.jetson`.

- [ ] **Step 4: Run the static tests and confirm GREEN**

Run:

```bash
pytest -q ros2/src/powertrain_ros/test/test_l515_image_contract.py
git diff --check
```

Expected: all image-contract tests pass and diff check is clean.

- [ ] **Step 5: Build the image as the compatibility gate**

Run from repository root:

```bash
docker build -f docker/Dockerfile.ros -t powertrain-sw:ros-l515-plan docker
```

Then:

```bash
docker run --rm powertrain-sw:ros-l515-plan bash -lc '
  source /opt/ros/humble/setup.bash
  source /opt/realsense_ros_ws/install/setup.bash
  rs-enumerate-devices --version
  ros2 pkg prefix realsense2_camera
  ros2 pkg prefix realsense2_camera_msgs
  ros2 pkg prefix realsense2_description
'
```

Expected: SDK reports 2.50.0 and all three ROS package prefixes resolve. If realsense-ros 4.0.1 does not clean-build on Humble, stop this plan and revise the approved design; do not patch forward or change tags silently.

- [ ] **Step 6: Commit Task 1**

```bash
git add docker/Dockerfile.ros ros2/src/powertrain_ros/test/test_l515_image_contract.py
git commit -m "build(ros): pin L515-compatible RealSense underlay"
```

---

### Task 2: Add the Serial-Locked Lightweight Launch Contract

**Files:**
- Create: `ros2/src/powertrain_ros/config/l515.yaml`
- Create: `ros2/src/powertrain_ros/launch/l515.launch.py`
- Create: `ros2/src/powertrain_ros/test/test_l515_launch_contract.py`
- Modify: `ros2/src/powertrain_ros/setup.py`

**Interfaces:**
- Consumes: `realsense2_camera` underlay package from Task 1.
- Produces: `ros2 launch powertrain_ros l515.launch.py`, topics under `/l515/*`.

- [ ] **Step 1: Write failing launch/config contract tests**

```python
from pathlib import Path
import yaml


ROOT = Path(__file__).resolve().parents[4]
PKG = ROOT / "ros2" / "src" / "powertrain_ros"


def load_config():
    return yaml.safe_load((PKG / "config" / "l515.yaml").read_text())


def test_l515_config_locks_device_and_lightweight_streams():
    params = load_config()["/**"]["ros__parameters"]
    assert params["serial_no"] == "00000000F0271544"
    assert params["camera_name"] == "l515"
    assert params["enable_color"] is True
    assert params["enable_depth"] is True
    assert params["enable_accel"] is True
    assert params["enable_gyro"] is True
    assert params["pointcloud.enable"] is False
    assert params["align_depth.enable"] is False
    assert params["enable_infra1"] is False
    assert params["enable_infra2"] is False
    assert params["unite_imu_method"] == 0


def test_l515_profiles_are_640x480_at_30hz():
    params = load_config()["/**"]["ros__parameters"]
    assert params["depth_module.profile"] == "640x480x30"
    assert params["rgb_camera.profile"] == "640x480x30"


def test_setup_installs_l515_config_and_launch():
    setup = (PKG / "setup.py").read_text()
    assert '"launch/l515.launch.py"' in setup
    assert '"config/l515.yaml"' in setup
```

- [ ] **Step 2: Run tests and confirm RED**

```bash
pytest -q ros2/src/powertrain_ros/test/test_l515_launch_contract.py
```

Expected: file-not-found failures for config and launch.

- [ ] **Step 3: Create the exact config**

Create `ros2/src/powertrain_ros/config/l515.yaml`:

```yaml
/**:
  ros__parameters:
    serial_no: "00000000F0271544"
    camera_name: "l515"
    enable_color: true
    enable_depth: true
    enable_accel: true
    enable_gyro: true
    enable_infra1: false
    enable_infra2: false
    pointcloud.enable: false
    align_depth.enable: false
    unite_imu_method: 0
    depth_module.profile: "640x480x30"
    rgb_camera.profile: "640x480x30"
    initial_reset: false
```

- [ ] **Step 4: Create the wrapper launch**

Create `ros2/src/powertrain_ros/launch/l515.launch.py`:

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from pathlib import Path


def generate_launch_description():
    wrapper_share = FindPackageShare("realsense2_camera")
    config_path = Path(__file__).resolve().parents[1] / "config" / "l515.yaml"
    wrapper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [wrapper_share, "/launch/rs_launch.py"]
        ),
        launch_arguments={
            "config_file": str(config_path),
            "camera_name": "l515",
            "serial_no": "00000000F0271544",
        }.items(),
    )
    return LaunchDescription([wrapper])
```

Wrapper 4.0.1의 `rs_launch.py`가 선언한 정확한 인자 `config_file`, `camera_name`, `serial_no`만 사용한다.

- [ ] **Step 5: Install launch/config via setup.py**

Extend `data_files` in `ros2/src/powertrain_ros/setup.py`:

```python
("share/" + package_name + "/launch", [
    "launch/wp5_control.launch.py",
    "launch/l515.launch.py",
]),
("share/" + package_name + "/config", ["config/l515.yaml"]),
```

- [ ] **Step 6: Run tests and a clean three-package build**

```bash
pytest -q ros2/src/powertrain_ros/test/test_l515_launch_contract.py
docker run --rm \
  -v "$PWD:/src/repo:ro" \
  powertrain-sw:ros-l515-plan bash -lc '
    source /opt/ros/humble/setup.bash
    source /opt/realsense_ros_ws/install/setup.bash
    cp -a /src/repo/ros2 /tmp/ws
    cd /tmp/ws
    colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros
    source install/setup.bash
    ros2 launch powertrain_ros l515.launch.py --show-args
  '
```

Expected: tests and build pass; launch arguments resolve without opening hardware.

- [ ] **Step 7: Commit Task 2**

```bash
git add ros2/src/powertrain_ros/config/l515.yaml \
  ros2/src/powertrain_ros/launch/l515.launch.py \
  ros2/src/powertrain_ros/test/test_l515_launch_contract.py \
  ros2/src/powertrain_ros/setup.py
git commit -m "feat(ros2): add serial-locked L515 launch"
```

---

### Task 3: Add a Fail-Closed L515 Preflight

**Files:**
- Create: `scripts/l515_preflight.sh`
- Create: `ros2/src/powertrain_ros/test/test_l515_preflight.py`

**Interfaces:**
- Consumes: host `lsusb`, container `rs-enumerate-devices`, expected serial.
- Produces: exit 0 only when the expected L515 is USB3 and SDK-visible; diagnostic stderr otherwise.

- [ ] **Step 1: Write subprocess tests with fake tools**

```python
from pathlib import Path
import os
import subprocess


ROOT = Path(__file__).resolve().parents[4]
SCRIPT = ROOT / "scripts" / "l515_preflight.sh"


def run_preflight(tmp_path, usb, sdk, speed="5000"):
    for name, body in {
        "lsusb": f"#!/bin/sh\nprintf '%s\\n' '{usb}'\n",
        "docker": f"#!/bin/sh\nprintf '%s\\n' '{sdk}'\n",
    }.items():
        path = tmp_path / name
        path.write_text(body)
        path.chmod(0o755)
    speed_file = tmp_path / "speed"
    speed_file.write_text(speed)
    env = {
        **os.environ,
        "PATH": f"{tmp_path}:{os.environ['PATH']}",
        "L515_USB_SPEED_FILE": str(speed_file),
    }
    return subprocess.run([SCRIPT], env=env, text=True, capture_output=True)


def test_preflight_accepts_expected_l515(tmp_path):
    result = run_preflight(
        tmp_path,
        "Bus 002 Device 005: ID 8086:0b64 Intel RealSense 515",
        "Intel RealSense L515 00000000F0271544 USB3.2",
    )
    assert result.returncode == 0


def test_preflight_rejects_usb_only_device(tmp_path):
    result = run_preflight(
        tmp_path,
        "Bus 002 Device 005: ID 8086:0b64 Intel RealSense 515",
        "Intel RealSense D435IF 250222071245",
    )
    assert result.returncode != 0
    assert "SDK" in result.stderr


def test_preflight_rejects_wrong_serial(tmp_path):
    result = run_preflight(
        tmp_path,
        "Bus 002 Device 005: ID 8086:0b64 Intel RealSense 515",
        "Intel RealSense L515 WRONG USB3.2",
    )
    assert result.returncode != 0


def test_preflight_rejects_non_usb3_link(tmp_path):
    result = run_preflight(
        tmp_path,
        "Bus 001 Device 005: ID 8086:0b64 Intel RealSense 515",
        "Intel RealSense L515 00000000F0271544 USB2.1",
        speed="480",
    )
    assert result.returncode != 0
    assert "USB3" in result.stderr
```

- [ ] **Step 2: Run tests and confirm RED**

```bash
pytest -q ros2/src/powertrain_ros/test/test_l515_preflight.py
```

Expected: file-not-found failure for `scripts/l515_preflight.sh`.

- [ ] **Step 3: Implement the shell preflight**

Create executable `scripts/l515_preflight.sh`:

```bash
#!/usr/bin/env bash
set -euo pipefail

SERIAL="00000000F0271544"
USB_LINE=$(lsusb -d 8086:0b64 || true)
if [[ -z "$USB_LINE" ]]; then
  echo "L515 USB device 8086:0b64 not found" >&2
  exit 1
fi

SPEED_FILE=${L515_USB_SPEED_FILE:-$(
  grep -l '^0b64$' /sys/bus/usb/devices/*/idProduct 2>/dev/null \
    | head -n1 | sed 's|idProduct$|speed|'
)}
if [[ -z "$SPEED_FILE" || ! -r "$SPEED_FILE" || "$(cat "$SPEED_FILE")" -lt 5000 ]]; then
  echo "L515 is not connected at USB3 5 Gbps" >&2
  exit 1
fi

SDK_OUTPUT=$(docker exec powertrain_ros bash -lc \
  'source /opt/realsense_ros_ws/install/setup.bash; rs-enumerate-devices -s' 2>&1 || true)
if ! grep -q "$SERIAL" <<<"$SDK_OUTPUT"; then
  echo "L515 exists on USB but SDK cannot enumerate serial $SERIAL" >&2
  exit 1
fi
if grep -q "250222071245" <<<"$SDK_OUTPUT" && ! grep -q "$SERIAL" <<<"$SDK_OUTPUT"; then
  echo "SDK selected D435i instead of L515" >&2
  exit 1
fi

echo "L515 preflight PASS: serial=$SERIAL"
```

- [ ] **Step 4: Run tests and shell validation**

```bash
chmod +x scripts/l515_preflight.sh
pytest -q ros2/src/powertrain_ros/test/test_l515_preflight.py
bash -n scripts/l515_preflight.sh
```

Expected: all tests pass and shell syntax is valid.

- [ ] **Step 5: Commit Task 3**

```bash
git add scripts/l515_preflight.sh ros2/src/powertrain_ros/test/test_l515_preflight.py
git commit -m "feat(l515): add fail-closed camera preflight"
```

---

### Task 4: Integrate Automatic Regression and Operator Documentation

**Files:**
- Modify: `README.md`
- Modify: `ros2/README.md`
- Modify: `AGENTS.md`
- Test: all `powertrain_ros` tests

**Interfaces:**
- Consumes: Tasks 1–3 commands and paths.
- Produces: one operator workflow and durable agent constraints.

- [ ] **Step 1: Run the full pre-documentation regression**

```bash
pytest -q ros2/src/powertrain_ros/test/test_l515_image_contract.py \
  ros2/src/powertrain_ros/test/test_l515_launch_contract.py \
  ros2/src/powertrain_ros/test/test_l515_preflight.py
```

Then in the ROS image:

```bash
docker run --rm -v "$PWD:/src/repo:ro" powertrain-sw:ros-l515-plan bash -lc '
  source /opt/ros/humble/setup.bash
  source /opt/realsense_ros_ws/install/setup.bash
  cp -a /src/repo/ros2 /tmp/ws
  cd /tmp/ws
  colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros
  source install/setup.bash
  colcon test --packages-select powertrain_ros --event-handlers console_direct+
  colcon test-result --verbose
'
```

Expected: zero failures, including all existing WP5 contracts.

- [ ] **Step 2: Document exact build and run commands**

Add to `ros2/README.md`:

```markdown
### L515 경량 파이프라인

L515 serial은 `00000000F0271544`로 고정한다. D435i 자동 선택은 금지한다.

```bash
docker compose -f docker/docker-compose.jetson.yml build --no-cache powertrain_ros
docker compose -f docker/docker-compose.jetson.yml up -d --force-recreate powertrain_ros
./scripts/l515_preflight.sh
docker exec -it powertrain_ros bash
source /opt/ros/humble/setup.bash
source /opt/realsense_ros_ws/install/setup.bash
cd /workspace/ros2
source install/setup.bash
ros2 launch powertrain_ros l515.launch.py
```

기본 출력은 color/depth/accel/gyro다. PointCloud2·IR·alignment는 비활성이다.
```

Update `README.md` and the newest `AGENTS.md` override with the same version pins, serial, and D435i ownership in concise form.

- [ ] **Step 3: Check documentation consistency**

```bash
rg -n "2\.50\.0|4\.0\.1|00000000F0271544|PointCloud2|D435i" \
  README.md ros2/README.md AGENTS.md \
  docs/specs/2026-07-11-l515-lightweight-pipeline-design.md
git diff --check
```

Expected: all authority files state the same pins and ownership; no whitespace errors.

- [ ] **Step 4: Commit Task 4**

```bash
git add README.md ros2/README.md AGENTS.md
git commit -m "docs(l515): add pinned pipeline operations"
```

---

### Task 5: Deploy Safely and Run L515-Only HIL

**Files:**
- Create: `docs/reports/2026-07-11-l515-lightweight-pipeline-hil.md`
- Do not modify Jetson teammate files.

**Interfaces:**
- Consumes: built `powertrain_ros` image, L515 serial, launch from Tasks 1–4.
- Produces: measured profiles, firmware, 60-second rates, resource deltas, reconnect behavior.

- [ ] **Step 1: Audit Jetson before deployment**

```bash
ssh jetson '
  git -C ~/power-train-sw status --short --branch
  git -C ~/power-train-sw log -5 --oneline --decorate
  docker ps --format "{{.Names}} {{.Status}}"
  ps -eo pid,args | grep -Ei "[r]ealsense|[p]yrealsense|[p]erception_node" || true
'
```

Expected: record the current non-origin Jetson commit chain and preserve untracked vision tests. Do not `git pull`, reset, or overwrite this checkout.

- [ ] **Step 2: Build from an isolated source snapshot**

From the local repository, transfer a clean archive of the exact commit without touching the Jetson checkout:

```bash
COMMIT=$(git rev-parse --short=12 HEAD)
git archive --format=tar HEAD | ssh jetson \
  "SNAPSHOT=/tmp/powertrain-l515-${COMMIT}; mkdir -p \"\$SNAPSHOT\"; tar -xf - -C \"\$SNAPSHOT\""
ssh jetson \
  "cd /tmp/powertrain-l515-${COMMIT} && docker build -f docker/Dockerfile.ros \
   -t powertrain-sw:ros-l515-${COMMIT} docker"
```

Expected: Task 1 smoke commands pass on ARM64. Do not retag `powertrain-sw:ros` until this passes.

- [ ] **Step 3: Enumerate L515 and record firmware/profiles**

```bash
COMMIT=$(git rev-parse --short=12 HEAD)
IMAGE="powertrain-sw:ros-l515-${COMMIT}"
docker run --rm --privileged --network host -v /dev:/dev \
  "$IMAGE" bash -lc '
    source /opt/realsense_ros_ws/install/setup.bash
    rs-enumerate-devices -s
  '
```

Expected: L515 serial `00000000F0271544` appears. Record firmware and accel/gyro profiles. If firmware is below 1.5.8.1, stop without updating it.

- [ ] **Step 4: Start only the L515 pipeline**

```bash
COMMIT=$(git rev-parse --short=12 HEAD)
ssh jetson "
  docker rm -f powertrain_l515_hil >/dev/null 2>&1 || true
  docker run -d --name powertrain_l515_hil \
    --privileged --network host \
    -v /dev:/dev \
    -v /tmp/powertrain-l515-${COMMIT}:/workspace:ro \
    powertrain-sw:ros-l515-${COMMIT} bash -lc '
      source /opt/ros/humble/setup.bash
      source /opt/realsense_ros_ws/install/setup.bash
      cp -a /workspace/ros2 /tmp/ws
      cd /tmp/ws
      colcon build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros
      source install/setup.bash
      exec ros2 launch powertrain_ros l515.launch.py
    '
  sleep 8
  docker logs powertrain_l515_hil
  docker top powertrain_l515_hil -eo pid,args
"
```

Expected: one `realsense2_camera_node` with namespace/name `l515/l515`; logs identify serial
`00000000F0271544`. No D435i-owning process is started by this task.

- [ ] **Step 5: Measure 60 seconds**

Use a temporary rclpy probe subscribing to:

- `/l515/color/image_raw`
- `/l515/depth/image_rect_raw`
- `/l515/gyro/sample`
- `/l515/accel/sample`

Discard discovery and one second of warm-up, then measure exactly 60 seconds. Record count, mean Hz, complete 5-second window minimum, maximum interval, non-monotonic stamps, and maximum data age for every topic. Record `docker stats --no-stream`, host free memory, and USB errors before/after.

Expected: color/depth mean ≥29 Hz and every complete 5-second window ≥28 Hz; accel and gyro both receive monotonic data; no PointCloud2 topic exists.

- [ ] **Step 6: Test disconnect and reconnect**

With the subscriber running, ask the user before physical unplug. Verify topic cessation, no fallback to D435i, then reconnect and verify only serial `00000000F0271544` resumes. Do not change firmware.

- [ ] **Step 7: Write and commit the L515-only HIL evidence**

Record exact commit, image ID, commands, firmware, profiles, rates, resource values, and disconnect/reconnect observations in `docs/reports/2026-07-11-l515-lightweight-pipeline-hil.md`.

```bash
git add docs/reports/2026-07-11-l515-lightweight-pipeline-hil.md
git commit -m "docs(l515): record single-camera HIL"
```

---

### Task 6: Run D435i Concurrent HIL and Close the Track

**Files:**
- Modify: `docs/reports/2026-07-11-l515-lightweight-pipeline-hil.md`
- Modify: `docs/reports/2026-07-10-project-and-jetson-state.md`
- Modify: `docs/plans/2026-07-02-autonomous-driving-kickoff.md`
- Modify: Notion pages for autonomous plan and L515/RealSense operations.

**Interfaces:**
- Consumes: passing L515-only HIL and robot-arm D435i container.
- Produces: proven two-camera coexistence and the WP6-ready input contract.

- [ ] **Step 1: Re-audit camera owners**

Ensure L515 is owned only by the powertrain L515 launch and D435i only by the robot-arm perception process. Record PIDs, containers, and serials. Do not kill unknown teammate processes.

- [ ] **Step 2: Start the robot-arm D435i path with its repository script**

Do not edit the robot-arm checkout. Run its existing script:

```bash
docker start ros2_humble
docker exec -d ros2_humble bash -lc '
  source /opt/ros/humble/setup.bash
  source /root/ros2_ws/install/setup.bash
  exec /root/ros2_ws/run_perception.sh
'
```

Confirm the process opens D435IF serial `250222071245` and publishes `/detected_objects`. The
robot-arm image uses pyrealsense2 2.58.2, which does not enumerate L515, so it cannot steal the
L515 selected by the powertrain wrapper.

- [ ] **Step 3: Repeat the 60-second L515 measurement under concurrent load**

Use the same warm-up, duration, metrics, and thresholds as Task 5. Additionally record D435i topic continuity, Jetson CPU/RAM, and USB topology. Expected: L515 color/depth thresholds still pass, IMU remains monotonic, D435i remains available, and neither process resets the other camera.

- [ ] **Step 4: Run final automatic regression**

```bash
pytest -q ros2/src/powertrain_ros/test/test_l515_image_contract.py \
  ros2/src/powertrain_ros/test/test_l515_launch_contract.py \
  ros2/src/powertrain_ros/test/test_l515_preflight.py
```

Run the full clean ROS build/test command from Task 4. Expected: zero failures.

- [ ] **Step 5: Update authorities and Notion**

Mark the L515 lightweight pipeline complete only if single-camera and concurrent-camera HIL both pass. Record exact default topics, measured IMU rates, resource cost, PointCloud2 disabled state, and remaining static-TF commissioning. Fetch each Notion page before update and re-fetch after update.

- [ ] **Step 6: Commit and push**

```bash
git add docs/reports/2026-07-11-l515-lightweight-pipeline-hil.md \
  docs/reports/2026-07-10-project-and-jetson-state.md \
  docs/plans/2026-07-02-autonomous-driving-kickoff.md \
  README.md ros2/README.md AGENTS.md
git commit -m "docs(l515): complete lightweight pipeline HIL"
git push origin main
```

Expected: local main and origin/main point to the same commit; user-owned untracked files remain untouched.
