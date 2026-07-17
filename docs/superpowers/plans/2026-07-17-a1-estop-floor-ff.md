# A1 배치 구현 계획 — ○ E-stop 전역 latch + min_rev 폐지 + friction_ff

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.
> **이 레포의 실행 관례:** 본 계획은 Codex 위임 파이프라인의 지시서로도 쓴다 — 태스크 단위로 위임하고, 리뷰어(Claude)가 태스크마다 검증 게이트를 잡는다.

**Goal:** 스펙 r6(§2.1·§2.2·§2.2b)의 A1 배치 — ○ 버튼을 전역 latched E-stop으로 정합하고, min_rev 속도 플로어를 폐지(기본 0)하며, 저속 마찰/코깅 보상 torque_ff 노브를 신설한다.

**Architecture:** teleop_command_node가 `estop_edge`를 TRANSIENT_LOCAL latched 토픽 `/teleop/estop`으로 1초 재발행 → chassis_node가 event_id 멱등 dedup 후 `cm.estop("remote_operator", ...)`(코너 6개 물리 정지 포함) 호출. 플로어는 메커니즘 보존·기본값만 0. friction_ff는 `DriveOdriveCan.tick()`의 기존 `Set_Input_Vel <ff (vel, tq_ff)` 2번째 필드로 구현하고 `build_real_corners`→CLI/파라미터로 배선한다.

**Tech Stack:** Python 3.10, rclpy(Humble), python-can(socketcan), pytest.

**Spec:** `docs/superpowers/specs/2026-07-17-abc-program-design.md` (r6) — §2.1, §2.2, §2.2b, §7.

## Global Constraints

- 3환경 회귀 기준선: 호스트 240 / dev 컨테이너 979 passed+2 skipped / ros 컨테이너 410 (신규 테스트만큼 증가 허용, 실패 0).
- 호스트 pytest 프리픽스: `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest`.
- dev 컨테이너 전체 회귀: `docker run --rm -v "$PWD:/workspace" -w /workspace powertrain-sw:dev python3 -m pytest motor_control motor_gui powertrain_observability powertrain_autonomy powertrain_sim remote_video tests operator_console -q`.
- ros 컨테이너 회귀: 핸드오프 보고서 `docs/reports/2026-07-16-project-state-and-handoff.md` §4의 "isolated read-only colcon /tmp build + src/powertrain_ros/test" 레시피 그대로 (테스트는 conftest가 DDS domain 77로 격리).
- 테스트와 커밋은 반드시 `&&`로 체인 (테스트 실패 시 커밋 금지).
- `docs/defence_docs/`, `docs/creativeEngineering/` 접근 금지. `.claude/settings*.json`, `.codex/` 커밋 금지.
- 커밋 메시지 말미: `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.
- 모터 실기 실행 없음 — A1은 전부 소프트웨어. friction_ff **값 튜닝**은 A배치 벤치 스모크(별도 세션) 몫이며 이 계획은 기본 0(off)으로만 출하.
- `Set_Input_Vel` tq_ff 필드 단위(A vs N·m)는 fw 0.5.1 벤치에서 확정 예정 — 코드·도움말에는 "raw torque_ff 단위(벤치 확정 전)"로 표기.

---

### Task 1: DriveOdriveCan에 friction_ff/v_knee 추가

**Files:**
- Modify: `motor_control/corner_module/drive_odrive_can.py:65-82` (생성자), `:193-198` (tick)
- Test: `motor_control/corner_module/tests/test_drive_friction_ff.py` (신규)

**Interfaces:**
- Consumes: 기존 `DriveOdriveCan(node_id, channel, stale_ms, bus, clock)` 계약.
- Produces: `DriveOdriveCan(..., friction_ff: float = 0.0, v_knee: float = 0.5)` — Task 2의 `build_real_corners`가 이 두 kwargs를 사용한다. `tick()`이 `0 < |target_vel| < v_knee`일 때 `sign(target_vel)*friction_ff`를 Set_Input_Vel의 tq_ff로 전송.

- [x] **Step 1: 실패하는 테스트 작성**

`motor_control/corner_module/tests/test_drive_friction_ff.py` 신규 (FakeCanBus 패턴은 `test_driver_health_fields.py`와 동일):

```python
"""§2.2b 저속 마찰/코깅 보상 torque_ff — Set_Input_Vel 2번째 필드 검증."""
import struct

import pytest

from corner_module.drive_odrive_can import DriveOdriveCan

_SET_INPUT_VEL = 0x0D


class FakeCanBus:
    def __init__(self):
        self.sent = []

    def recv(self, timeout=0.0):
        return None

    def send(self, message):
        self.sent.append(message)


def _input_vel_frames(bus, node_id):
    out = []
    for m in bus.sent:
        if m.arbitration_id == ((node_id << 5) | _SET_INPUT_VEL) and not m.is_remote_frame:
            out.append(struct.unpack("<ff", bytes(m.data)[0:8]))
    return out


def _drive(**kwargs):
    bus = FakeCanBus()
    drive = DriveOdriveCan(node_id=11, bus=bus, **kwargs)
    return drive, bus


@pytest.mark.parametrize(
    ("target", "expected_ff"),
    [
        (0.3, 0.25),     # knee 아래 전진 → +ff
        (-0.3, -0.25),   # knee 아래 후진 → -ff (부호 추종)
        (0.5, 0.0),      # knee 경계(포함 안 함) → 0
        (0.8, 0.0),      # knee 위 → 0
        (0.0, 0.0),      # 정지 지령 → 정확히 0 (크리프 방지)
    ],
)
def test_tick_applies_friction_ff_only_inside_knee(target, expected_ff):
    drive, bus = _drive(friction_ff=0.25, v_knee=0.5)
    drive.set_velocity(target)
    drive.tick()
    frames = _input_vel_frames(bus, 11)
    assert len(frames) == 1
    vel, ff = frames[0]
    assert vel == pytest.approx(target)
    assert ff == pytest.approx(expected_ff)


def test_default_is_off_and_sends_zero_ff():
    drive, bus = _drive()
    drive.set_velocity(0.3)
    drive.tick()
    assert _input_vel_frames(bus, 11)[0][1] == pytest.approx(0.0)


def test_arm_disarm_estop_always_send_zero_ff():
    drive, bus = _drive(friction_ff=0.25, v_knee=0.5)
    drive.set_velocity(0.3)
    drive.arm()
    drive.disarm()
    drive.estop()
    for _vel, ff in _input_vel_frames(bus, 11):
        assert ff == pytest.approx(0.0)
```

- [x] **Step 2: 실패 확인**

Run: `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest motor_control/corner_module/tests/test_drive_friction_ff.py -v`
Expected: FAIL — `TypeError: __init__() got an unexpected keyword argument 'friction_ff'`

- [x] **Step 3: 구현**

`drive_odrive_can.py` 생성자 시그니처·본문 (기존 `:65-82` 교체):

```python
    def __init__(self, node_id: int = 11, channel: str = "can0",
                 stale_ms: float = 200.0, bus=None, clock=None,
                 friction_ff: float = 0.0, v_knee: float = 0.5):
        self._node_id = node_id
        self._channel = channel
        self._stale_ms = stale_ms
        self._bus = bus
        self._owns_bus = bus is None
        self._friction_ff = max(0.0, float(friction_ff))
        self._v_knee = max(0.0, float(v_knee))
        self._target_vel = 0.0
```

(이하 기존 필드 초기화 그대로 유지.)

docstring Parameters에 추가:

```python
    friction_ff:
        저속 마찰/코깅 보상 피드포워드(raw torque_ff 단위 — fw 0.5.1 벤치 확정 전).
        0 < |target| < v_knee 일 때만 부호 추종으로 Set_Input_Vel 2번째 필드에 실린다.
        0.0(기본) = off. 스펙 r6 §2.2b(D4).
    v_knee:
        friction_ff 적용 상한(turns/s, 기본 0.5). 경계값은 포함하지 않는다.
```

`tick()` 교체 (기존 `:193-198`):

```python
    def _friction_torque_ff(self) -> float:
        t = self._target_vel
        if self._friction_ff > 0.0 and 0.0 < abs(t) < self._v_knee:
            return self._friction_ff if t > 0.0 else -self._friction_ff
        return 0.0

    def tick(self) -> None:
        """제어 루프마다: 목표 속도(+저속 마찰 보상 ff) 전송 + RTR 폴링."""
        self._drain_available()
        self._send(_SET_INPUT_VEL,
                   struct.pack("<ff", self._target_vel, self._friction_torque_ff()))
        self._send(_GET_ENCODER_ESTIMATES, rtr=True)
        self._send(_GET_IQ, rtr=True)
```

`arm()`/`disarm()`/`estop()`의 명시적 `struct.pack("<ff", 0.0, 0.0)`은 그대로 둔다(정지 지령 ff=0 계약).

- [x] **Step 4: 통과 확인**

Run: `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest motor_control/corner_module/tests/ -v`
Expected: 신규 7개 PASS + 기존 corner_module 테스트 전부 PASS

- [x] **Step 5: 커밋**

```bash
git add motor_control/corner_module/drive_odrive_can.py motor_control/corner_module/tests/test_drive_friction_ff.py
git commit -m "feat: low-speed friction-compensation torque_ff knob in DriveOdriveCan

Spec r6 §2.2b (D4): sign-following feedforward on the existing CAN
Set_Input_Vel tq_ff field, applied only for 0<|target|<v_knee; default
0=off. Unit (A vs N*m on fw 0.5.1) pending bench confirmation.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: friction 노브 배선 — build_real_corners → CLI/파라미터

**Files:**
- Modify: `motor_control/chassis/chassis_manager.py:115-130` (build_real_corners)
- Modify: `motor_control/chassis/teleop_server.py:279-281` 부근 (인자 추가), `:322` (build_real_corners 호출)
- Modify: `motor_control/chassis/teleop_dualsense.py:146-148` 부근 (인자 추가), `:192` (호출)
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py:134` 부근 (파라미터), `:240` (호출)
- Test: `motor_control/chassis/tests/test_friction_ff_plumbing.py` (신규)

**Interfaces:**
- Consumes: Task 1의 `DriveOdriveCan(..., friction_ff, v_knee)`.
- Produces: `build_real_corners(channel, cfg=None, wheel_map=None, friction_ff: float = 0.0, v_knee_turns_s: float = 0.5)`. CLI `--friction-ff`/`--v-knee`(teleop 2종), ROS 파라미터 `friction_ff`/`friction_v_knee`(chassis_node).

- [x] **Step 1: 실패하는 테스트 작성**

`motor_control/chassis/tests/test_friction_ff_plumbing.py` 신규:

```python
"""friction_ff 노브가 build_real_corners→DriveOdriveCan까지 배선되는지 검증."""
import re
from pathlib import Path

import chassis.chassis_manager as chassis_manager

CHASSIS_DIR = Path(chassis_manager.__file__).resolve().parent
TELEOP_SERVER = (CHASSIS_DIR / "teleop_server.py").read_text(encoding="utf-8")
TELEOP_DUALSENSE = (CHASSIS_DIR / "teleop_dualsense.py").read_text(encoding="utf-8")
CHASSIS_NODE = (
    CHASSIS_DIR.parents[1]
    / "ros2/src/powertrain_ros/powertrain_ros/chassis_node.py"
).read_text(encoding="utf-8")


class _RecordingDrive:
    instances = []

    def __init__(self, node_id, channel="can0", **kwargs):
        self.node_id = node_id
        self.kwargs = kwargs
        _RecordingDrive.instances.append(self)


class _RecordingSteer:
    def __init__(self, motor_id, channel="can0"):
        self.motor_id = motor_id


def test_build_real_corners_passes_friction_kwargs(monkeypatch):
    import corner_module.drive_odrive_can as drive_mod
    import corner_module.steer_ak40 as steer_mod

    _RecordingDrive.instances = []
    monkeypatch.setattr(drive_mod, "DriveOdriveCan", _RecordingDrive)
    monkeypatch.setattr(steer_mod, "SteerAk40", _RecordingSteer)

    chassis_manager.build_real_corners(
        "can0", friction_ff=0.25, v_knee_turns_s=0.4
    )

    assert len(_RecordingDrive.instances) == 6
    for drive in _RecordingDrive.instances:
        assert drive.kwargs["friction_ff"] == 0.25
        assert drive.kwargs["v_knee"] == 0.4


def test_build_real_corners_defaults_are_off():
    import inspect

    signature = inspect.signature(chassis_manager.build_real_corners)
    assert signature.parameters["friction_ff"].default == 0.0
    assert signature.parameters["v_knee_turns_s"].default == 0.5


def test_teleop_clis_expose_friction_knobs_with_off_defaults():
    for source in (TELEOP_SERVER, TELEOP_DUALSENSE):
        assert re.search(
            r'"--friction-ff",\s*type=float,\s*default=0\.0', source
        ), "teleop CLI must expose --friction-ff default 0.0"
        assert re.search(
            r'"--v-knee",\s*type=float,\s*default=0\.5', source
        ), "teleop CLI must expose --v-knee default 0.5"
        assert "friction_ff=args.friction_ff" in source
        assert "v_knee_turns_s=args.v_knee" in source


def test_chassis_node_declares_friction_parameters():
    assert '"friction_ff", 0.0' in CHASSIS_NODE
    assert '"friction_v_knee", 0.5' in CHASSIS_NODE
    assert "friction_ff=friction_ff" in CHASSIS_NODE
    assert "v_knee_turns_s=friction_v_knee" in CHASSIS_NODE
```

- [x] **Step 2: 실패 확인**

Run: `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest motor_control/chassis/tests/test_friction_ff_plumbing.py -v`
Expected: FAIL — `TypeError: build_real_corners() got an unexpected keyword argument 'friction_ff'`

- [x] **Step 3: 구현**

`chassis_manager.py` — `build_real_corners` 교체:

```python
def build_real_corners(channel: str = "can0", cfg: CornerConfig = None,
                       wheel_map=None, friction_ff: float = 0.0,
                       v_knee_turns_s: float = 0.5) -> dict:
    """실기용 — AK 조향(CAN) + ODrive 구동(CAN) 코너 6개. 하드웨어 라이브러리는
    지연 import(무하드웨어 pytest 가 python-can/odrive 없이 이 모듈을 쓰게).

    friction_ff/v_knee_turns_s 는 저속 마찰/코깅 보상 노브(스펙 r6 §2.2b, 기본 off)
    — DriveOdriveCan 으로 그대로 전달된다.

    CAN 단독 소유권은 라이브러리 함수가 아니라 실물 실행 진입점의
    ``chassis.runtime_lock.RealCanSession``이 이 함수 호출 전에 획득한다. Fake/MuJoCo
    빌더가 이 함수와 lock에 결합되지 않도록 여기서는 버스 객체만 구성한다.
    """
    import corner_module.steer_ak40 as steer_mod
    import corner_module.drive_odrive_can as drive_mod   # WP1 완료 필요
    return build_corners(
        steer_factory=lambda cid: steer_mod.SteerAk40(motor_id=cid, channel=channel),
        drive_factory=lambda nid: drive_mod.DriveOdriveCan(
            node_id=nid, channel=channel,
            friction_ff=friction_ff, v_knee=v_knee_turns_s,
        ),
        cfg=cfg, wheel_map=wheel_map,
    )
```

(⚠️ import 형태를 `from X import Y`에서 `import X as mod` + 속성 접근으로 바꿔야 monkeypatch가 동작한다 — 테스트 계약.)

`teleop_server.py` `_parse_args`의 `--min-rev` 인자 바로 아래에 추가:

```python
    p.add_argument("--friction-ff", type=float, default=0.0,
                   help="저속 마찰/코깅 보상 torque_ff (raw 단위, 0=off — 스펙 r6 §2.2b)")
    p.add_argument("--v-knee", type=float, default=0.5,
                   help="friction-ff 적용 상한 turns/s (기본 0.5)")
```

`teleop_server.py` `main()`의 build_real_corners 호출 교체:

```python
        corners = build_real_corners(
            args.channel, wheel_map=wheel_map,
            friction_ff=args.friction_ff, v_knee_turns_s=args.v_knee,
        )
```

`teleop_dualsense.py` — `--min-rev` 아래 동일 인자 2개 추가(도움말 동일), `main()` 호출 교체:

```python
        corners = build_real_corners(
            args.channel,
            friction_ff=args.friction_ff, v_knee_turns_s=args.v_knee,
        )
```

`chassis_node.py` — `:134` `min_rev` 선언 아래에:

```python
        self.declare_parameter("friction_ff", 0.0)
        self.declare_parameter("friction_v_knee", 0.5)
```

`:154` 파라미터 로드부에:

```python
        friction_ff = float(self.get_parameter("friction_ff").value)
        friction_v_knee = float(self.get_parameter("friction_v_knee").value)
```

`:240` 실기 코너 생성 교체:

```python
            corners = build_real_corners(
                channel, wheel_map=wheel_map,
                friction_ff=friction_ff, v_knee_turns_s=friction_v_knee,
            )
```

- [x] **Step 4: 통과 확인**

Run: `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest motor_control/chassis/tests/test_friction_ff_plumbing.py motor_control/chassis/tests/ -q`
Expected: 신규 4개 포함 전부 PASS

- [x] **Step 5: 커밋**

```bash
git add motor_control/chassis/chassis_manager.py motor_control/chassis/teleop_server.py motor_control/chassis/teleop_dualsense.py ros2/src/powertrain_ros/powertrain_ros/chassis_node.py motor_control/chassis/tests/test_friction_ff_plumbing.py
git commit -m "feat: plumb friction_ff/v_knee through build_real_corners, teleop CLIs, chassis_node

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 3: min_rev 플로어 폐지 — 기본값 전면 0.0 (D3)

**Files:**
- Modify: `motor_control/chassis/teleop_server.py:21`(docstring), `:279-280`(default)
- Modify: `motor_control/chassis/teleop_dualsense.py:15`, `:22`(docstring), `:146-147`(default)
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py:134`
- Modify: `ros2/src/powertrain_ros/launch/autonomy.launch.py:74-75`
- Test: `motor_control/chassis/tests/test_friction_ff_plumbing.py` (Task 2 파일에 추가)

**Interfaces:**
- Consumes: 없음 (기본값 변경만 — `ChassisConfig.min_drive_turns_per_s` dataclass 기본은 이미 0.0).
- Produces: 모든 진입점의 min_rev 기본 = 0.0. 기존 `test_chassis_manager.py`의 명시적 `min_drive_turns_per_s=1.0` 테스트들은 메커니즘 검증이므로 그대로 둔다.

- [x] **Step 1: 실패하는 테스트 추가**

`test_friction_ff_plumbing.py` 말미에 추가:

```python
def test_min_rev_floor_is_abolished_to_zero_defaults():
    """D3(스펙 r6 §2.2): 플로어 기본값 전면 0 — 메커니즘은 opt-in 노브로만 보존."""
    for source, label in (
        (TELEOP_SERVER, "teleop_server"),
        (TELEOP_DUALSENSE, "teleop_dualsense"),
    ):
        assert re.search(
            r'"--min-rev",\s*type=float,\s*default=0\.0', source
        ), f"{label}: --min-rev default must be 0.0 (floor abolished)"
    assert '"min_rev", 0.0' in CHASSIS_NODE

    launch = (
        CHASSIS_DIR.parents[1]
        / "ros2/src/powertrain_ros/launch/autonomy.launch.py"
    ).read_text(encoding="utf-8")
    assert re.search(
        r'"min_rev",\s*default_value="0\.0"', launch
    ), "autonomy.launch: min_rev default must be 0.0"
```

- [x] **Step 2: 실패 확인**

Run: `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest motor_control/chassis/tests/test_friction_ff_plumbing.py::test_min_rev_floor_is_abolished_to_zero_defaults -v`
Expected: FAIL — `default=1.0` 매칭 실패

- [x] **Step 3: 구현 (기본값·도움말·독스트링 4개 사이트)**

`teleop_server.py:279-280`:

```python
    p.add_argument("--min-rev", type=float, default=0.0,
                   help="최저 구동속도 turns/s (0=off 기본 — 플로어 폐지 D3; "
                        "저속 보상은 --friction-ff 사용)")
```

`teleop_server.py:21` 옵션 요약 줄의 `--min-rev 1.0` → `--min-rev 0`.

`teleop_dualsense.py:146-147`:

```python
    parser.add_argument("--min-rev", type=float, default=0.0,
                        help="최저 구동속도 turns/s (0=off 기본 — 플로어 폐지 D3; "
                             "저속 보상은 --friction-ff 사용)")
```

`teleop_dualsense.py:15` 줄을 다음으로 교체:

```python
▸ 저속 HALL 코깅존 대응: 플로어(--min-rev)는 폐지(기본 0, 스펙 r6 §2.2 D3) —
  저속 보상은 `--friction-ff`(torque feedforward, §2.2b) 를 쓴다.
```

`teleop_dualsense.py:22` 옵션 요약의 `--min-rev 1.0` → `--min-rev 0`.

`chassis_node.py:134`:

```python
        self.declare_parameter("min_rev", 0.0)
```

`autonomy.launch.py:74-75`:

```python
        DeclareLaunchArgument("min_rev", default_value="0.0",
                              description="코깅존 플로어 — 폐지(기본 0, 스펙 r6 §2.2 D3). "
                                          "재도입은 커미셔닝 재량. 이력: docs/specs/2026-07-13-min-rev-speed-range.md"),
```

- [x] **Step 4: 통과 확인 (플로어 메커니즘 회귀 포함)**

Run: `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest motor_control/chassis/tests/ motor_control/corner_module/tests/ -q`
Expected: 전부 PASS (기존 `test_chassis_manager.py`의 명시적 1.0 테스트 포함 — 기본값 변경은 이를 건드리지 않음)

- [x] **Step 5: 커밋**

```bash
git add motor_control/chassis/teleop_server.py motor_control/chassis/teleop_dualsense.py ros2/src/powertrain_ros/powertrain_ros/chassis_node.py ros2/src/powertrain_ros/launch/autonomy.launch.py motor_control/chassis/tests/test_friction_ff_plumbing.py
git commit -m "feat: abolish min_rev speed floor - all defaults to 0 (spec r6 D3)

Mechanism retained as opt-in commissioning knob; low-speed cogging is
now addressed by friction_ff (D4) instead of speed flooring.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: `/teleop/estop` latched 발행 (teleop_command_node)

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/teleop_command_node.py:8-33`(import·상수), `:96-110`(publisher), `:236-268`(_drain_events), `:301-318`(_tick)
- Test: `ros2/src/powertrain_ros/test/test_teleop_estop_topic.py` (신규)

**Interfaces:**
- Consumes: `RemoteInputFrame.estop_edge`(기존 디코더 산출).
- Produces: 토픽 `/teleop/estop` — `std_msgs/String`, JSON `{"event_id": "<hex>", "stamp_s": <float>}`, QoS = RELIABLE + **TRANSIENT_LOCAL** + depth 1, edge 시점부터 **1.0초간 매 틱 동일 event_id 재발행**. Task 5의 chassis_node가 구독.

- [x] **Step 1: 실패하는 테스트 작성**

`ros2/src/powertrain_ros/test/test_teleop_estop_topic.py` 신규:

```python
"""○ E-stop 전역 latch 정합(스펙 r6 §2.1) — /teleop/estop latched 발행."""
import json
import time
import uuid

import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String

from powertrain_ros.remote_input import DPad, NormalizedAxes, RemoteInputFrame
from powertrain_ros.teleop_command_node import TeleopCommandNode


@pytest.fixture(scope="module", autouse=True)
def ros():
    rclpy.init()
    yield
    rclpy.shutdown()


def _estop_frame(sequence=0):
    return RemoteInputFrame(
        schema_version=2,
        session_id=str(uuid.uuid4()),
        sequence=sequence,
        client_monotonic_ns=0,
        mode="DRIVE",
        deadman=False,
        axes=NormalizedAxes(
            left_x=0.0, right_y=0.0, left_trigger=0.0, right_trigger=0.0
        ),
        dpad=DPad(x=0, y=0),
        mode_chord=False,
        estop_edge=True,
        assist_bypass=False,
        received_s=time.monotonic(),
    )


def _latched_listener(received):
    listener = Node("estop_probe_%s" % uuid.uuid4().hex[:8])
    qos = QoSProfile(
        depth=1,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
        history=HistoryPolicy.KEEP_LAST,
    )
    listener.create_subscription(
        String, "/teleop/estop", lambda m: received.append(m.data), qos
    )
    return listener


def test_estop_edge_publishes_latched_event_visible_to_late_joiner():
    node = TeleopCommandNode()
    try:
        node._events.put(("frame", _estop_frame()))
        node._tick()

        # 발행 **이후** 구독을 만들어도(late join) TRANSIENT_LOCAL 로 수신돼야 한다
        # — 구독자(chassis_node) 재시작 창에서도 latch 유실 없음(스펙 §2.1).
        received = []
        listener = _latched_listener(received)
        try:
            deadline = time.monotonic() + 3.0
            while not received and time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=0.05)
                rclpy.spin_once(node, timeout_sec=0.0)
            assert received, "late-joining subscriber did not get latched estop"
            payload = json.loads(received[0])
            assert set(payload) == {"event_id", "stamp_s"}
            assert isinstance(payload["event_id"], str) and payload["event_id"]
            assert isinstance(payload["stamp_s"], float)
        finally:
            listener.destroy_node()
    finally:
        node.close()
        node.destroy_node()


def test_rebroadcast_reuses_same_event_id_within_window():
    node = TeleopCommandNode()
    try:
        received = []
        listener = _latched_listener(received)
        try:
            node._events.put(("frame", _estop_frame()))
            node._tick()
            node._tick()
            deadline = time.monotonic() + 3.0
            while len(received) < 2 and time.monotonic() < deadline:
                rclpy.spin_once(listener, timeout_sec=0.05)
                rclpy.spin_once(node, timeout_sec=0.0)
            assert len(received) >= 2
            ids = {json.loads(item)["event_id"] for item in received}
            assert len(ids) == 1, "rebroadcast must reuse the same event_id"
        finally:
            listener.destroy_node()
    finally:
        node.close()
        node.destroy_node()


def test_rebroadcast_stops_after_window(monkeypatch):
    from powertrain_ros import teleop_command_node as module

    monkeypatch.setattr(module, "ESTOP_REBROADCAST_S", 0.0)
    node = TeleopCommandNode()
    try:
        node._events.put(("frame", _estop_frame()))
        node._tick()          # edge 시점 1회 발행 후, 창(0초) 만료로 즉시 정리
        assert node._estop_event is None
    finally:
        node.close()
        node.destroy_node()
```

- [x] **Step 2: 실패 확인**

ros 컨테이너 레시피(Global Constraints)로:
Run: `python3 -m pytest src/powertrain_ros/test/test_teleop_estop_topic.py -v`
Expected: FAIL — `AttributeError: ... no attribute 'pub_estop'` (혹은 토픽 미수신)

- [x] **Step 3: 구현**

`teleop_command_node.py` import에 추가:

```python
import json
import uuid

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
```

기존 `from std_msgs.msg import Bool` 줄은 `from std_msgs.msg import Bool, String` 으로 교체.

상수(`MAX_VIOLATION_EVENTS_PER_S` 아래):

```python
# ○ E-stop 전역 latch(스펙 r6 §2.1): edge 시점부터 이 시간 동안 매 틱 재발행.
# 발행 자체는 TRANSIENT_LOCAL latched 라 재발행은 구독자 프로세스 재시작
# '창'에 대한 보험이다.
ESTOP_REBROADCAST_S = 1.0
```

`__init__`의 publisher 블록(`pub_assist_bypass` 아래)에:

```python
        estop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.pub_estop = self.create_publisher(
            String,
            "/teleop/estop",
            estop_qos,
        )
        self._estop_event = None
```

`_drain_events`의 `elif event == "frame":` 분기 교체:

```python
            elif event == "frame":
                if payload.estop_edge:
                    self._begin_estop_event(drain_now_s)
                self._gateway.submit(payload)
```

새 메서드 2개(`_publish_drive` 위):

```python
    def _begin_estop_event(self, now_s):
        self._estop_event = {
            "event_id": uuid.uuid4().hex,
            "stamp_s": float(now_s),
            "until_s": float(now_s) + ESTOP_REBROADCAST_S,
        }
        self._publish_estop_event()

    def _publish_estop_event(self):
        if self._estop_event is None:
            return
        message = String()
        message.data = json.dumps(
            {
                "event_id": self._estop_event["event_id"],
                "stamp_s": self._estop_event["stamp_s"],
            },
            separators=(",", ":"),
        )
        self.pub_estop.publish(message)
```

`_tick()` 서두(`self._drain_events()` 다음 줄)에:

```python
        if self._estop_event is not None:
            if time.monotonic() < self._estop_event["until_s"]:
                self._publish_estop_event()
            else:
                self._estop_event = None
```

- [x] **Step 4: 통과 확인**

Run: `python3 -m pytest src/powertrain_ros/test/test_teleop_estop_topic.py src/powertrain_ros/test/test_teleop_command_node.py -v` (ros 컨테이너)
Expected: 신규 3개 + 기존 teleop 테스트 전부 PASS

- [x] **Step 5: 커밋**

```bash
git add ros2/src/powertrain_ros/powertrain_ros/teleop_command_node.py ros2/src/powertrain_ros/test/test_teleop_estop_topic.py
git commit -m "feat: publish latched /teleop/estop event on circle-button edge

Spec r6 §2.1: TRANSIENT_LOCAL depth-1 + 1s per-tick rebroadcast with a
stable event_id, so a restarting subscriber cannot lose the global
E-stop latch. Gateway-local hold entry is unchanged (defense in depth).

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 5: chassis_node 구독 — event_id 멱등 dedup + `cm.estop("remote_operator")`

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py` (구독 생성은 `_initialize`의 `self.cm` 생성 이후 아무 무조건 구역, 콜백은 `_srv_estop` 인근)
- Test: `ros2/src/powertrain_ros/test/test_teleop_estop_latch.py` (신규)

**Interfaces:**
- Consumes: Task 4의 `/teleop/estop` JSON 계약·QoS.
- Produces: `ChassisNode._on_teleop_estop(msg)` — 유효 payload면 최초 1회 `self.cm.estop("remote_operator", "teleop circle edge event_id=<id>")`. `self._teleop_estop_seen`: `collections.OrderedDict` (최대 32 event_id 보존).

- [x] **Step 1: 실패하는 테스트 작성**

`ros2/src/powertrain_ros/test/test_teleop_estop_latch.py` 신규 (AST 추출 패턴 — `test_chassis_arm_gate.py`와 동일 스타일, ROS 노드 생성 없음):

```python
"""○ E-stop 전역 latch(스펙 r6 §2.1) — chassis_node 구독 dedup·진입점 검증."""
import ast
import collections
import json
import re
from pathlib import Path
from types import SimpleNamespace

PACKAGE = Path(__file__).resolve().parents[1]
CHASSIS_NODE = PACKAGE / "powertrain_ros/chassis_node.py"
SOURCE = CHASSIS_NODE.read_text(encoding="utf-8")


def _extract_method(name):
    tree = ast.parse(SOURCE)
    cls = next(
        item
        for item in tree.body
        if isinstance(item, ast.ClassDef) and item.name == "ChassisNode"
    )
    method = next(
        item
        for item in cls.body
        if isinstance(item, ast.FunctionDef) and item.name == name
    )
    module = ast.Module(body=[method], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {"json": json, "collections": collections}
    exec(compile(module, str(CHASSIS_NODE), "exec"), namespace)
    return namespace[name]


class _RecordingCm:
    def __init__(self):
        self.calls = []

    def estop(self, source, detail=""):
        self.calls.append((source, detail))


class _Logger:
    def error(self, *_args):
        pass


def _node(cm):
    return SimpleNamespace(
        cm=cm,
        _teleop_estop_seen=collections.OrderedDict(),
        get_logger=lambda: _Logger(),
    )


def _msg(event_id="abc123", stamp_s=1.5):
    return SimpleNamespace(
        data=json.dumps({"event_id": event_id, "stamp_s": stamp_s})
    )


def test_first_event_trips_cm_estop_with_remote_operator_source():
    on_estop = _extract_method("_on_teleop_estop")
    cm = _RecordingCm()
    node = _node(cm)

    on_estop(node, _msg())

    assert len(cm.calls) == 1
    source, detail = cm.calls[0]
    assert source == "remote_operator"
    assert "abc123" in detail


def test_duplicate_event_id_is_idempotent():
    on_estop = _extract_method("_on_teleop_estop")
    cm = _RecordingCm()
    node = _node(cm)

    on_estop(node, _msg())
    on_estop(node, _msg())          # 재발행(같은 event_id) — 1회만 trip

    assert len(cm.calls) == 1


def test_new_event_id_trips_again_and_ledger_is_bounded():
    on_estop = _extract_method("_on_teleop_estop")
    cm = _RecordingCm()
    node = _node(cm)

    for index in range(40):
        on_estop(node, _msg(event_id="event-%d" % index))

    assert len(cm.calls) == 40
    assert len(node._teleop_estop_seen) <= 32


def test_invalid_payload_is_rejected_without_trip():
    on_estop = _extract_method("_on_teleop_estop")
    cm = _RecordingCm()
    node = _node(cm)

    on_estop(node, SimpleNamespace(data="not json"))
    on_estop(node, SimpleNamespace(data=json.dumps({"event_id": "x"})))

    assert cm.calls == []


def test_subscription_is_unconditional_and_transient_local():
    """구독은 authority_enabled 와 무관하게 항상 생성 + latched QoS 여야 한다."""
    assert re.search(
        r'create_subscription\(\s*String,\s*"/teleop/estop",\s*'
        r"self\._on_teleop_estop,",
        SOURCE,
    ), "chassis_node must subscribe /teleop/estop"
    assert "DurabilityPolicy.TRANSIENT_LOCAL" in SOURCE
```

- [x] **Step 2: 실패 확인**

Run: `python3 -m pytest src/powertrain_ros/test/test_teleop_estop_latch.py -v` (ros 컨테이너; 이 테스트는 rclpy 초기화가 없어 호스트에서도 동작: `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest ros2/src/powertrain_ros/test/test_teleop_estop_latch.py -v`)
Expected: FAIL — `StopIteration` (`_on_teleop_estop` 메서드 없음)

- [x] **Step 3: 구현**

`chassis_node.py` — 파일 상단 import에 `collections`가 없으면 추가하고, rclpy QoS import 추가:

```python
import collections

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
```

`_initialize`에서 `self.cm.connect()` 직후(무조건 구역)에:

```python
        # ○ E-stop 전역 latch(스펙 r6 §2.1): authority 와 무관한 고정 안전 계약.
        # TRANSIENT_LOCAL — 이 노드가 발행 후 재시작해도 latch 이벤트를 놓치지 않는다.
        self._teleop_estop_seen = collections.OrderedDict()
        teleop_estop_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            String,
            "/teleop/estop",
            self._on_teleop_estop,
            teleop_estop_qos,
        )
```

콜백(`_srv_estop` 바로 위에):

```python
    def _on_teleop_estop(self, msg):
        try:
            payload = json.loads(msg.data)
            event_id = str(payload["event_id"])
            stamp_s = float(payload["stamp_s"])
        except (KeyError, TypeError, ValueError):
            self.get_logger().error("invalid /teleop/estop payload; ignored")
            return
        if event_id in self._teleop_estop_seen:
            return
        self._teleop_estop_seen[event_id] = stamp_s
        while len(self._teleop_estop_seen) > 32:
            self._teleop_estop_seen.popitem(last=False)
        # 물리 정지까지 포함한 정본 진입점(cm.estop) — raw interlock trip 금지
        # (다음 50 Hz 틱까지 모터가 돈다). 스펙 r6 §2.1.
        self.cm.estop(
            "remote_operator",
            "teleop circle edge event_id=%s" % event_id,
        )
        self.get_logger().error(
            "REMOTE E-STOP latched (event_id=%s)" % event_id
        )
```

- [x] **Step 4: 통과 확인**

Run: `python3 -m pytest src/powertrain_ros/test/test_teleop_estop_latch.py src/powertrain_ros/test/test_chassis_arm_gate.py -v` (ros 컨테이너)
Expected: 신규 6개 + 기존 arm-gate 테스트 전부 PASS

- [x] **Step 5: 커밋**

```bash
git add ros2/src/powertrain_ros/powertrain_ros/chassis_node.py ros2/src/powertrain_ros/test/test_teleop_estop_latch.py
git commit -m "feat: chassis_node latches remote-operator E-stop from /teleop/estop

Idempotent event_id dedup (bounded 32-entry ledger) into cm.estop
('remote_operator'), which stops all six corners immediately. Reset
stays ~/reset_estop -> ~/arm (no implicit arm). Spec r6 §2.1.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 6: 문서 동기 + 3환경 전체 회귀

**Files:**
- Modify: `.claude/CLAUDE.md` (chassis 절의 min-rev 서술)
- Modify: `docs/specs/2026-07-13-min-rev-speed-range.md` (상단 결정 배너)
- Modify: `docs/reports/2026-07-16-project-state-and-handoff.md` (§2 커밋 체인에 A1 항목 추가)

**Interfaces:**
- Consumes: Task 1~5의 커밋 해시.
- Produces: 문서 정합. 코드 변경 없음.

- [x] **Step 1: `.claude/CLAUDE.md` 갱신**

chassis 절의 `**`min_drive_turns_per_s` 최저 구동속도 플로어**(0=off; 0<|명령|<이 값이면 부호 유지 상향 → 저속 HALL 코깅존 회피)` 서술 뒤에 추가하고, `기본 min-rev 1.0` 문구를 `기본 min-rev 0(플로어 폐지)` 로 교체:

```
(2026-07-17 D3/D4: 플로어 기본값 전면 0 = 폐지. 저속 코깅 대응은
DriveOdriveCan friction_ff/v_knee(torque_ff 피드포워드, 기본 off) —
스펙 docs/superpowers/specs/2026-07-17-abc-program-design.md §2.2/§2.2b.)
```

- [x] **Step 2: `docs/specs/2026-07-13-min-rev-speed-range.md` 상단에 결정 배너 추가**

```markdown
> **⛔ 2026-07-17 결정(D3/D4)**: min_rev 플로어는 **폐지**(기본값 전면 0) —
> 1.0은 임의 벤치값이었고 실차 저속 특성은 조립 후 재결정. 메커니즘은
> 커미셔닝 opt-in 노브로만 보존. 저속 코깅 대응은 friction_ff(torque
> feedforward)로 대체. 정본: `docs/superpowers/specs/2026-07-17-abc-program-design.md` §2.2·§2.2b.
> 아래 본문은 플로어 도입 당시의 이력 문서다.
```

- [x] **Step 3: 핸드오프 보고서 §2 커밋 체인에 A1 줄 추가** (커밋 해시는 Task 1~5 실제 해시)

```markdown
- A1 (○ E-stop 전역 latch + 플로어 폐지 D3 + friction_ff D4): <task1>..<task5>
  — /teleop/estop TRANSIENT_LOCAL latch, cm.estop("remote_operator"),
  min_rev 기본 0, DriveOdriveCan friction_ff/v_knee(기본 off).
```

- [x] **Step 4: 3환경 전체 회귀**

Run (호스트): `PYTHONPATH=ros2/src/powertrain_ros:motor_control:. /home/light/anaconda3/bin/python -m pytest motor_control -q`
Expected: 기존 240 + 신규(Task 1: 7, Task 2/3: 5) 전부 PASS

Run (dev 컨테이너): Global Constraints의 dev 전체 회귀 명령
Expected: 979+신규 passed, 2 skipped, 실패 0

Run (ros 컨테이너): 핸드오프 §4 레시피
Expected: 410+신규(Task 4: 3, Task 5: 6) passed, 실패 0

- [x] **Step 5: 커밋**

```bash
git add .claude/CLAUDE.md docs/specs/2026-07-13-min-rev-speed-range.md docs/reports/2026-07-16-project-state-and-handoff.md
git commit -m "docs: sync min_rev abolition (D3) + friction_ff (D4) + A1 commit chain

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## A1 완료 기준 (스펙 §7 대조)

- ○ edge → `/teleop/estop` latched 발행(재발행 창 1 s, 동일 event_id) → chassis 멱등 latch → `~/reset_estop`(원인 잔존 시 거부)→`~/arm` 분리 해제 흐름 회귀 테스트 통과.
- E-stop 구독자 재시작(late join) 시나리오 = TRANSIENT_LOCAL 테스트로 커버.
- min_rev 기본 0 — 4개 진입점 + 문서 동기, 플로어 메커니즘 테스트는 명시 config로 존치.
- friction_ff 기본 off·knee 경계·부호 추종·정지 0 계약 테스트 통과. 값 튜닝은 A배치 벤치 스모크로 이월(계획 밖).
- 3환경 기준선 회귀 실패 0.
