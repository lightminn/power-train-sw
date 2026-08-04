# USB 스키드 조향 레이어 + 콘솔 모드 선택 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** AK 조향 없이 USB ODrive 6축만으로 스키드 조향하는 계층을 만들고, 운용 콘솔에서 CAN/USB · 애커만/스키드를 선택할 수 있게 한다.

**Architecture:** 새 키네마틱스 수식을 쓰지 않는다 — `kinematics.solve()` 의 고정 바퀴 분기가 이미 스키드 식(`vx = v − ω·y`)이므로, 전 바퀴를 `steerable=False` 로 둔 기하 하나로 기존 인터록·워치독·estop 전파·텔레메트리를 그대로 재사용한다. 실작업은 ① USB 다보드 `DriveActuator` 구현, ② 스키드에서 특이행렬이 되는 오도메트리 수정, ③ `ChassisManager` 런타임 조향모드 전환, ④ ops 채널·콘솔 노출이다.

**Tech Stack:** Python 3 (stdlib + `odrive` fw-v0.5.6 · `pyusb`), pytest, ROS 2 (`rclpy`, `std_srvs`), GTK3 (`operator_console`).

**설계 정본:** `docs/superpowers/specs/2026-08-04-usb-skid-steer-design.md`

## Global Constraints

- **좌표계는 REP-103** — x=앞, y=왼쪽, ω>0=좌회전(CCW). 단위 m·rad·s.
- **드라이버 바깥은 전부 바퀴 프레임.** 감속비(모터 5회전 = 바퀴 1회전)와 우측 미러 부호 반전은 **드라이버 경계 안에서만** 적용한다.
- **감속비 `gear_ratio=5.0`**, 바퀴 반경 `0.10356 m`, 구동 상한 `0.80 m/s`, 조향 한계 `±45°`.
- **ODrive 캘리브레이션은 RAM-only** — 전원 사이클마다 필요(축당 ~55 s). 코드가 이를 가정해서는 안 된다.
- **하드웨어 라이브러리는 지연 import.** `odrive`·`usb.core`·`can` 은 함수 안에서 import 한다. 모듈 최상단 import 는 무하드웨어 pytest 를 깨뜨린다.
- **제어 루프는 죽지 않는다.** 모든 하드웨어 I/O 예외는 흡수하고 `stale`/`error_count` 로 드러낸다 (`drive_odrive_can.py:123-127` 과 동형).
- **`state()` 는 캐시만 반환한다.** `CornerModule.tick()` 이 매 tick 호출하므로 여기서 I/O 를 하면 예산이 터진다.
- **기존 4WS 경로는 바이트 단위로 불변**이어야 한다. 회귀 테스트가 이를 지킨다.
- **테스트 실행 위치**
  - `motor_control/` 계열: `cd motor_control && python -m pytest <경로> -q`
  - ROS: `cd ros2 && colcon test --packages-select powertrain_ros` (또는 ROS 환경에서 `python -m pytest src/powertrain_ros/test/<파일> -q`)
  - 콘솔: 레포 루트에서 `python3 -m pytest operator_console/tests -q`
- **커밋은 태스크마다 1개.** 태스크 안에서 테스트가 green 이 된 직후에 커밋한다.

## 스펙 수정 1건 (Task 0 에서 반영)

스펙 §7.1 은 조향 0° 수렴 타임아웃 시 "전환을 거부하고 **hold 를 유지**한다"로 적혀 있다. 그런데 조사 결과 `SafetyInterlock` 의 motion hold 를 밖에서 지울 수 있는 ops 경로가 없다 — `authority_clear_hold`(`chassis_node.py:570`)는 `self._authority.clear_hold()` 를 부를 뿐(`chassis_node.py:1454`) 인터록 hold 와 무관하다. hold 를 유지하면 **운전자가 풀 방법이 없는 상태로 차체가 갇힌다.**

→ **타임아웃 시 전환을 취소하고 hold 를 해제하며, 직전 모드를 그대로 유지한다.** 직전 모드는 방금까지 정상 동작하던 구성이므로 안전하고 복구 가능하다. 조향이 정말로 고장이면 `CornerModule.tick()` 의 fault/stale/과전류 검사가 별도로 estop 을 건다.

## File Structure

| 파일 | 책임 | 태스크 |
|---|---|---|
| `motor_control/chassis/kinematics.py` (수정) | `skid_geometry()` 추가. `solve()` 는 불변 | 1 |
| `motor_control/chassis/odometry.py` (수정) | 측면식 0개일 때 vy 사전분포 1행 | 2 |
| `motor_control/corner_module/drive_odrive_usb_axis.py` (신규) | `UsbBoardPool` + `DriveOdriveUsbAxis` + `calibrate_axis()` | 3 |
| `motor_control/chassis/chassis_manager.py` (수정) | `build_usb_skid_corners()` (T4), 조향모드 전환 상태머신 (T7) | 4, 7 |
| `motor_control/chassis/teleop_server.py` (수정) | `--skid-usb` 계열 플래그 | 5 |
| `config/bl70200_boards.json` (신규) | 보드 시리얼 ↔ CAN node 쌍 레지스트리 | 4 |
| `ros2/.../chassis_node.py` (수정) | `drive_transport`/`steering_mode` 파라미터, `~/steer_mode_skid` 서비스, safety_state 3필드 | 8 |
| `ros2/.../state_estimation.py` (수정) | `StateEstimator.set_geometry()` | 9 |
| `ros2/.../odometry_node.py`, `imu_tilt_node.py` (수정) | `/chassis/safety_state` 구독 → 기하 동기 스왑 | 9 |
| `ros2/.../ops_contract.py` (수정) | `steer_mode_skid` 액션 | 10 |
| `operator_console/ops_panel.py` (수정) | 토글 행 · 배지 · 회색 처리 판정 | 11 |
| `operator_console/status_view.py` (수정) | 배지 렌더 | 11 |
| `ros2/.../launch/control.launch.py` (수정) | 트랜스포트 launch 인자 + 모드 파일 | 12 |

---

# Phase A — 스키드 주행 계층

**Phase A 끝나면 무선 텔레옵으로 USB 스키드 주행이 된다.** 여기서 벤치 검증(P0~P2)을 통과한 뒤 Phase B 로 넘어간다.

---

### Task 0: 스펙 정정 반영

**Files:**
- Modify: `docs/superpowers/specs/2026-08-04-usb-skid-steer-design.md` §7.1

- [ ] **Step 1: 스펙 §7.1 마지막 문단 교체**

기존:

```markdown
**3번이 핵심이다.** 45° 로 꺾인 상태에서 곧바로 차동 구동을 걸면 격렬한 스크럽이
난다. 타임아웃으로 수렴에 실패하면 전환을 거부하고 hold 를 유지한다.
```

교체:

```markdown
**3번이 핵심이다.** 45° 로 꺾인 상태에서 곧바로 차동 구동을 걸면 격렬한 스크럽이
난다.

타임아웃으로 수렴에 실패하면 **전환을 취소하고 hold 를 해제하며 직전 모드를
유지한다.** `SafetyInterlock` 의 motion hold 를 밖에서 지우는 ops 경로가 없기
때문이다 — `authority_clear_hold`(`chassis_node.py:570`)는
`self._authority.clear_hold()` 만 부르며(`chassis_node.py:1454`) 인터록 hold 와
무관하다. hold 를 유지하면 운전자가 풀 방법이 없는 상태로 차체가 갇힌다. 직전
모드는 방금까지 정상 동작하던 구성이므로 안전하고 복구 가능하며, 조향이 실제로
고장이면 `CornerModule.tick()` 의 fault/stale/과전류 검사가 별도로 estop 을 건다.
```

- [ ] **Step 2: 커밋**

```bash
git add docs/superpowers/specs/2026-08-04-usb-skid-steer-design.md
git commit -m "docs(chassis): 조향모드 전환 타임아웃은 hold 유지가 아니라 취소·해제

인터록 motion hold 를 밖에서 지우는 ops 경로가 없어(authority_clear_hold 는
authority 전용) hold 를 유지하면 운전자가 풀 수 없는 상태로 갇힌다."
```

---

### Task 1: `skid_geometry()`

**Files:**
- Modify: `motor_control/chassis/kinematics.py` (파일 끝, `four_wheel_geometry()` 다음)
- Test: `motor_control/chassis/tests/test_kinematics.py` (파일 끝에 추가)

**Interfaces:**
- Consumes: 기존 `Wheel`, `ChassisGeometry`, `default_geometry`, `solve`
- Produces: `skid_geometry(track_gain: float = 1.0, base: ChassisGeometry = None) -> ChassisGeometry` — 모든 바퀴가 `steerable=False` 이고 `y` 가 `track_gain` 배인 기하. `wheel_radius_m`·`steer_limit_deg`·`drive_limit_mps` 는 `base` 에서 승계.

- [ ] **Step 1: 실패하는 테스트 작성**

`motor_control/chassis/tests/test_kinematics.py` 끝에 추가. import 줄도 함께 고친다:

```python
from chassis.kinematics import (
    Wheel, ChassisGeometry, WheelCommand, SolveResult, solve, default_geometry,
    four_wheel_geometry, skid_geometry,
)
```

(기존 import 문에 `four_wheel_geometry, skid_geometry` 를 더한다. 이미 있으면 중복하지 않는다.)

```python
# ── 스키드 조향 기하 ──────────────────────────────────────────────────────


def sg(track_gain=1.0):
    return skid_geometry(track_gain)


def test_skid_geometry_has_no_steerable_wheel():
    """스키드는 조향륜이 0개다 — 이게 solve() 를 스키드 식으로 바꾸는 유일한 스위치."""
    assert [w.name for w in sg().wheels if w.steerable] == []
    assert len(sg().wheels) == 6


def test_skid_geometry_keeps_base_scalars():
    base = default_geometry()
    skid = sg()

    assert skid.wheel_radius_m == base.wheel_radius_m
    assert skid.drive_limit_mps == base.drive_limit_mps
    assert skid.steer_limit_deg == base.steer_limit_deg


def test_skid_straight_drives_all_wheels_equally():
    result = solve(sg(), v_mps=0.4, omega_rad_s=0.0)

    speeds = [wc.drive_mps for wc in result.wheels.values()]
    assert speeds == pytest.approx([0.4] * 6)
    assert all(wc.steer_deg == 0.0 for wc in result.wheels.values())


def test_skid_pivot_counter_rotates_left_against_right():
    """v=0, ω>0(좌회전) → 왼쪽은 뒤로, 오른쪽은 앞으로."""
    result = solve(sg(), v_mps=0.0, omega_rad_s=1.0)

    for name, wc in result.wheels.items():
        if name.endswith("_left"):
            assert wc.drive_mps < 0.0
        else:
            assert wc.drive_mps > 0.0


def test_skid_pivot_speed_follows_each_wheel_lateral_offset():
    """as-built 윤거가 축마다 달라 같은 편이라도 속도가 다르다 — 그게 강체 정답이다."""
    result = solve(sg(), v_mps=0.0, omega_rad_s=1.0)

    assert result.wheels["mid_right"].drive_mps == pytest.approx(0.3595)
    assert result.wheels["front_right"].drive_mps == pytest.approx(0.2725)
    assert result.wheels["rear_right"].drive_mps == pytest.approx(0.2125)


def test_skid_never_reports_steer_clamping():
    """조향륜이 없으므로 _peak_steer 가 항상 0 → ω 를 깎지 않는다."""
    result = solve(sg(), v_mps=0.0, omega_rad_s=3.0)

    assert result.steer_clamped is False
    assert result.omega_applied == pytest.approx(3.0)


def test_skid_track_gain_scales_the_left_right_difference():
    """track_gain 은 유효 윤거 — 좌우 속도차가 그 배수로 커진다."""
    base = solve(sg(1.0), v_mps=0.0, omega_rad_s=1.0)
    wide = solve(sg(2.0), v_mps=0.0, omega_rad_s=1.0)

    for name in base.wheels:
        assert wide.wheels[name].drive_mps == pytest.approx(
            2.0 * base.wheels[name].drive_mps)


def test_skid_track_gain_does_not_touch_straight_line_speed():
    result = solve(sg(1.5), v_mps=0.4, omega_rad_s=0.0)

    assert [wc.drive_mps for wc in result.wheels.values()] == pytest.approx([0.4] * 6)


@pytest.mark.parametrize("bad", [0.0, -1.0, float("nan"), float("inf")])
def test_skid_geometry_rejects_invalid_track_gain(bad):
    with pytest.raises(ValueError):
        skid_geometry(bad)


def test_skid_geometry_accepts_a_base_geometry():
    skid = skid_geometry(1.0, base=four_wheel_geometry())

    assert sorted(w.name for w in skid.wheels) == [
        "front_left", "front_right", "rear_left", "rear_right"]
    assert all(not w.steerable for w in skid.wheels)


def test_default_geometry_is_untouched_by_skid_geometry():
    """스키드 기하를 만들어도 원본이 오염되면 안 된다 (같은 Wheel 객체 공유 금지)."""
    skid_geometry(2.0)

    assert [w.steerable for w in default_geometry().wheels] == [
        True, True, False, False, True, True]
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd motor_control && python -m pytest chassis/tests/test_kinematics.py -q
```

Expected: `ImportError: cannot import name 'skid_geometry'` — 파일 전체 수집 실패.

- [ ] **Step 3: `skid_geometry()` 구현**

`motor_control/chassis/kinematics.py` 파일 끝에 추가:

```python
def skid_geometry(track_gain: float = 1.0,
                  base: ChassisGeometry = None) -> ChassisGeometry:
    """🛠️ **스키드(차동) 조향 기하** — 모든 바퀴를 고정륜으로 둔다.

    새 수식이 필요 없다. `solve()` 의 고정 바퀴 분기가 이미 `vx = v − ω·yᵢ` 로
    스키드 식이며(측면 성분 `vy = ω·xᵢ` 는 스크럽으로 버린다), 조향륜이 0개면
    `_peak_steer()` 가 항상 0 을 반환해 ω 클램프도 자동으로 비활성화된다.
    속도 상한 스케일링과 비유한 검사는 그대로 살아 있다.

    바퀴별 `y` 를 그대로 쓰므로 **강체 기준 종방향 슬립이 0** 이다 — 각 바퀴는
    자기 위치가 요구하는 전진속도만 받는다. 탱크식(편측 동일 속도)을 쓰지 않는
    이유가 이것이다: as-built 윤거가 앞 545 / 중간 719 / 뒤 425 mm 로 모두 달라
    같은 속도를 주면 같은 편 바퀴끼리 종방향으로 싸운다.

    Parameters
    ----------
    track_gain:
        **유효 윤거 배수.** 실차 스키드는 타이어 측면 미끄럼 저항 때문에 명령보다
        덜 돈다. 같은 ω 를 실제로 내려면 좌우 속도차를 더 벌려야 하므로
        **1.0 보다 큰 값이 보정 방향**이다. 기본 1.0(무보정)이며 실측 전까지
        그대로 둔다 — 지상 커미셔닝에서 명령 ω 대비 실측 ω 비로 확정한다.

        기하에 곱하는 이유는 명령과 추정이 한 모델을 쓰게 하기 위해서다.
        오도메트리는 바퀴속도에서 ω 를 역산하므로(ω ≈ Δv / 2y_eff), 유효 윤거가
        크면 ω 추정도 같이 작아져 "실제로 덜 도는" 물리와 방향이 일치한다.
    base:
        바탕 기하(기본 `default_geometry()`). `skid_geometry(base=
        four_wheel_geometry())` 로 4륜 스키드도 만들 수 있다. 원본은 수정하지
        않고 새 `Wheel` 을 만들어 복사한다.
    """
    if not math.isfinite(track_gain) or track_gain <= 0.0:
        raise ValueError("track_gain must be finite and positive")
    src = base if base is not None else default_geometry()
    return ChassisGeometry(
        wheels=[Wheel(w.name, w.x, w.y * track_gain, False) for w in src.wheels],
        wheel_radius_m=src.wheel_radius_m,
        steer_limit_deg=src.steer_limit_deg,
        drive_limit_mps=src.drive_limit_mps,
    )
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
cd motor_control && python -m pytest chassis/tests/test_kinematics.py -q
```

Expected: 전부 PASS. 기존 4WS 테스트도 함께 green 이어야 한다.

- [ ] **Step 5: 커밋**

```bash
git add motor_control/chassis/kinematics.py motor_control/chassis/tests/test_kinematics.py
git commit -m "feat(chassis): 스키드 조향 기하 skid_geometry()

solve() 의 고정 바퀴 분기가 이미 vx = v - w*y 라, 전 바퀴를 steerable=False 로
둔 기하만으로 스키드가 된다. track_gain 은 유효 윤거 배수이며 실차가 명령보다
덜 도는 것을 보정한다(기본 1.0 = 무보정, 지상 커미셔닝에서 확정)."
```

---

### Task 2: 오도메트리 vy 사전분포 (결함 A)

**Files:**
- Modify: `motor_control/chassis/odometry.py:76-84` (`OdometryConfig`), `:120-144` (`_rows`), `:199-222` (`solve_twist`)
- Test: `motor_control/chassis/tests/test_odometry.py` (파일 끝에 추가)

**Interfaces:**
- Consumes: Task 1 의 `skid_geometry`
- Produces: `OdometryConfig.vy_prior_weight: float = 1e-3`. `_rows()` 의 행 이름이 `str | None` 이 된다 (`None` = 바퀴가 아닌 사전분포 행).

**배경:** 조향륜이 0개면 모든 행이 `[1, 0, −y]` 뿐이라 정규방정식의 vy 열·행이 0 이 되고, `_solve3()` 이 피벗 `< 1e-12` 로 `None` 을 반환해(`odometry.py:106-107`) `solve_twist()` 가 항상 `(0, 0, 0)` fail-safe 를 낸다(`odometry.py:224`).

- [ ] **Step 1: 실패하는 테스트 작성 (음성 대조 포함)**

`motor_control/chassis/tests/test_odometry.py` 끝에 추가:

```python
from chassis.kinematics import skid_geometry, solve


def _observations_from_command(geom, v_mps, omega_rad_s):
    """명령을 그대로 실측이라고 가정한 관측 — 슬립 0 인 이상적 케이스."""
    result = solve(geom, v_mps, omega_rad_s)
    return [
        WheelObservation(name=name, drive_mps=wc.drive_mps, steer_deg=wc.steer_deg)
        for name, wc in result.wheels.items()
    ]


def test_skid_twist_is_recovered_from_wheel_speeds():
    """수정 전에는 정규방정식이 특이해 (0,0,0) fail-safe 가 나왔다 — 음성 대조."""
    geom = skid_geometry()
    observations = _observations_from_command(geom, 0.3, 0.5)

    twist = solve_twist(geom, observations)

    assert twist.vx == pytest.approx(0.3, abs=1e-6)
    assert twist.omega == pytest.approx(0.5, abs=1e-6)
    assert twist.vy == pytest.approx(0.0, abs=1e-6)
    assert twist.used == 6


def test_skid_pivot_twist_is_recovered():
    geom = skid_geometry()
    observations = _observations_from_command(geom, 0.0, -0.8)

    twist = solve_twist(geom, observations)

    assert twist.vx == pytest.approx(0.0, abs=1e-6)
    assert twist.omega == pytest.approx(-0.8, abs=1e-6)


def test_vy_prior_is_not_counted_as_a_wheel():
    """사전분포 행이 used·rejected·잔차 집계에 새면 신뢰도 지표가 오염된다."""
    geom = skid_geometry()
    observations = _observations_from_command(geom, 0.3, 0.5)

    twist = solve_twist(geom, observations)

    assert twist.used == 6
    assert twist.rejected == ()
    assert twist.residual_mps == pytest.approx(0.0, abs=1e-6)


def test_ackermann_rows_are_unchanged_by_the_prior():
    """조향륜이 하나라도 있으면 사전분포 행을 넣지 않는다 (4WS 회귀 0)."""
    from chassis.odometry import OdometryConfig, _rows

    geom = default_geometry()
    observations = _observations_from_command(geom, 0.3, 0.3)
    obs_map = {o.name: o for o in observations}

    rows = _rows(geom, obs_map, OdometryConfig())

    assert all(name is not None for name, *_ in rows)
    # 조향 4륜 × 2행 + 고정 2륜 × 1행
    assert len(rows) == 10


def test_no_observations_produces_no_lone_prior_row():
    from chassis.odometry import OdometryConfig, _rows

    geom = skid_geometry()

    assert _rows(geom, {}, OdometryConfig()) == []
```

파일 상단 import 에 `default_geometry` 와 `solve_twist`, `WheelObservation` 이 이미 있는지 확인하고, 없으면 더한다.

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd motor_control && python -m pytest chassis/tests/test_odometry.py -q
```

Expected: `test_skid_twist_is_recovered_from_wheel_speeds` 가 `assert 0.0 == approx(0.3)` 로 FAIL. **이것이 결함 A 의 음성 대조다 — 반드시 이 실패를 눈으로 확인하고 넘어간다.**

- [ ] **Step 3: `OdometryConfig` 에 가중치 추가**

`motor_control/chassis/odometry.py:84` `max_reject` 다음 줄에 추가:

```python
    vy_prior_weight: float = 1e-3    # 스키드(조향륜 0개) 전용 비홀로노믹 사전분포.
                                     # 측면식이 하나도 없으면 정규방정식이 특이해져
                                     # solve_twist 가 항상 (0,0,0) 을 낸다. "측면 속도는
                                     # 명령하지 않는다"를 약하게 한 줄 넣어 vy 를 고정.
```

- [ ] **Step 4: `_rows()` 에 조건부 사전분포 행 추가**

`motor_control/chassis/odometry.py:120-144` 를 통째로 교체:

```python
def _rows(geom, obs_map, cfg):
    """각 바퀴의 관측을 선형식 행(row)들로 전개.

    반환: list[(wheel_name, [a0,a1,a2], b, weight)] — a·(vx,vy,ω) = b
          wheel_name 이 None 인 행은 바퀴가 아니라 사전분포다(아래 참조).
    """
    circ = 2.0 * math.pi * geom.wheel_radius_m
    rows = []
    lateral_rows = 0
    for w in geom.wheels:
        o = obs_map.get(w.name)
        if o is None or not o.valid:
            continue

        weight = 1.0 if w.steerable else cfg.mid_weight          # 스크럽 보정
        if abs(o.drive_mps) / circ < cfg.hall_trust_rev_s:        # HALL 코깅존
            weight *= cfg.low_speed_weight

        if w.steerable:
            d = math.radians(o.steer_deg)
            # 접지점 속도벡터 = s·(cos δ, sin δ)
            rows.append((w.name, [1.0, 0.0, -w.y], o.drive_mps * math.cos(d), weight))
            rows.append((w.name, [0.0, 1.0,  w.x], o.drive_mps * math.sin(d), weight))
            lateral_rows += 1
        else:
            # 고정륜은 전진성분만 관측 — 측면성분은 스크럽이라 방정식에 넣지 않는다
            rows.append((w.name, [1.0, 0.0, -w.y], o.drive_mps, weight))

    if rows and lateral_rows == 0:
        # 스키드 구성(조향륜 0개): 측면 성분을 관측하는 식이 하나도 없어 정규방정식의
        # vy 열·행이 전부 0 → _solve3 이 특이로 판정 → solve_twist 가 항상 (0,0,0).
        # "스키드는 측면 속도를 명령하지 않는다"는 비홀로노믹 사전분포를 약하게 한 줄
        # 넣어 vy 를 고정한다. 스크럽으로 실제 측면 이동이 생겨도 바퀴로는 관측할 수
        # 없으므로 0 이 최선의 사전분포다. 이름 None = 바퀴가 아님(잔차·배제에서 제외).
        rows.append((None, [0.0, 1.0, 0.0], 0.0, cfg.vy_prior_weight))
    return rows
```

- [ ] **Step 5: `solve_twist()` 에서 사전분포 행 제외**

`motor_control/chassis/odometry.py:199-222` 의 루프에서 두 곳을 고친다.

`names` 계산 (기존 `names = {n for n, *_ in rows}`):

```python
        names = {n for n, *_ in rows if n is not None}
```

잔차 루프 (기존 `for n, a, b, _w in rows:` 블록):

```python
        res = {}
        for n, a, b, _w in rows:
            if n is None:
                continue                              # 사전분포는 바퀴가 아니다
            e = sum(a[i] * x[i] for i in range(3)) - b
            res[n] = math.hypot(res.get(n, 0.0), e)
```

- [ ] **Step 6: 테스트 통과 확인**

```bash
cd motor_control && python -m pytest chassis/tests/test_odometry.py -q
cd motor_control && python -m pytest chassis/tests/ corner_module/tests/ -q
```

Expected: 전부 PASS. 특히 기존 4WS 오도메트리 테스트가 하나도 깨지지 않아야 한다.

- [ ] **Step 7: 커밋**

```bash
git add motor_control/chassis/odometry.py motor_control/chassis/tests/test_odometry.py
git commit -m "fix(odometry): 조향륜 0개일 때 정규방정식 특이 → vy 사전분포 한 줄

_rows() 가 측면식을 조향륜에만 넣어, 스키드 구성에서는 모든 행이 [1,0,-y] 뿐이라
vy 열이 0 이 되고 _solve3 이 특이 판정 → solve_twist 가 항상 (0,0,0) fail-safe 를
냈다. 측면식이 하나도 없을 때만 비홀로노믹 사전분포를 넣어 4WS 경로는 불변으로 둔다."
```

---

### Task 3: USB 다보드 구동 드라이버

**Files:**
- Create: `motor_control/corner_module/drive_odrive_usb_axis.py`
- Test: `motor_control/corner_module/tests/test_drive_odrive_usb_axis.py`

**Interfaces:**
- Consumes: `corner_module.actuator.DriveActuator`
- Produces:
  - `UsbBoardPool(find_timeout=20.0, finder=None, enumerator=None)` — `.discover_serials() -> list[str]`, `.axis(serial, axis_index) -> object`, `.close()`
  - `DriveOdriveUsbAxis(pool, serial, axis_index, *, node_id=None, gear_ratio=5.0, invert=False, current_lim_a=9.0, stale_ms=500.0, poll_slot=0, poll_period_ticks=6, clock=None)` — `DriveActuator` 계약 + `state()` 가 `DriveOdriveCan.health_state()` 와 같은 키를 낸다
  - `calibrate_axis(axis, label, current_lim_a=9.0, timeout_s=120.0, clock=None, sleep=None) -> bool`

**배경:** 시퀀스는 `motor_control/drive/bl70200/dualsense_usb_teleop.py` 에서 이식한다 — 벤치에서 3보드 6축 실회전이 확인된 코드다. `odrive.enums` 대신 평면 int 상수를 쓰는 이유는 `drive_odrive_usb.py:6-13` 에 기록돼 있고(클래스 enum 대입 시 `TypeError`), 무하드웨어 pytest 가 `odrive` 없이 이 모듈을 import 할 수 있게 하는 효과도 있다.

- [ ] **Step 1: 실패하는 테스트 작성**

`motor_control/corner_module/tests/test_drive_odrive_usb_axis.py` 신규:

```python
"""USB 다보드 구동 드라이버 — odrive 없이 fake 핸들 트리로 검증한다.

fake 는 우리 ABC 가 아니라 **odrive 라이브러리의 객체 트리**를 흉내낸다
(axis.controller.input_vel, axis.motor.current_control.Iq_measured 등).
그래서 corner_module/fake.py 가 아니라 이 테스트 파일에 둔다.
"""
import pytest

from corner_module.drive_odrive_usb_axis import (
    DriveOdriveUsbAxis, UsbBoardPool, calibrate_axis,
)

_AXIS_IDLE = 1
_AXIS_CLOSED_LOOP = 8


class _Ns:
    def __init__(self, **kw):
        self.__dict__.update(kw)


class FakeAxis:
    """odrive axis 객체 트리 흉내. fail=True 면 모든 속성 접근이 터진다."""

    def __init__(self, calibrated=True, fail=False):
        self.fail = fail
        self.error = 0
        self.requested_state = _AXIS_IDLE
        self.current_state = _AXIS_IDLE
        self.motor = _Ns(
            error=0,
            is_calibrated=calibrated,
            config=_Ns(current_lim=0.0, calibration_current=0.0),
            current_control=_Ns(Iq_measured=1.25),
        )
        self.encoder = _Ns(
            error=0,
            is_ready=calibrated,
            vel_estimate=0.0,
            config=_Ns(ignore_illegal_hall_state=False, calib_scan_omega=0.0,
                       calib_scan_distance=0, calib_range=0.0),
        )
        self.controller = _Ns(
            error=0,
            input_vel=0.0,
            config=_Ns(control_mode=0, input_mode=0),
        )

    def __getattribute__(self, name):
        if name != "fail" and object.__getattribute__(self, "fail"):
            raise OSError("USB gone")
        return object.__getattribute__(self, name)


class FakeBoard:
    def __init__(self, **axes):
        for name, axis in axes.items():
            setattr(self, name, axis)


def make_driver(axis=None, **kw):
    axis = axis or FakeAxis()
    pool = UsbBoardPool(finder=lambda serial: FakeBoard(axis0=axis, axis1=axis))
    kw.setdefault("clock", lambda: make_driver.now)
    driver = DriveOdriveUsbAxis(pool, "SN1", 0, **kw)
    driver.connect()
    return driver, axis


make_driver.now = 0.0


# ── 프레임 변환 ──────────────────────────────────────────────────────────


def test_wheel_command_is_scaled_by_the_gear_ratio():
    driver, axis = make_driver(gear_ratio=5.0)
    driver.arm()

    driver.set_velocity(1.0)          # 바퀴 1 rev/s
    driver.tick()

    assert axis.controller.input_vel == pytest.approx(5.0)   # 모터 5 turns/s


def test_invert_flips_only_at_the_driver_boundary():
    driver, axis = make_driver(gear_ratio=5.0, invert=True)
    driver.arm()

    driver.set_velocity(1.0)
    driver.tick()

    assert axis.controller.input_vel == pytest.approx(-5.0)


def test_measured_velocity_is_reported_in_the_wheel_frame():
    driver, axis = make_driver(gear_ratio=5.0, invert=True, poll_period_ticks=1)
    driver.arm()
    axis.encoder.vel_estimate = -5.0        # 모터 프레임
    driver.tick()

    assert driver.state()["actual_vel"] == pytest.approx(1.0)   # 바퀴 프레임


# ── 폴링 스케줄 ──────────────────────────────────────────────────────────


def test_telemetry_is_polled_only_on_the_assigned_slot():
    driver, axis = make_driver(poll_slot=2, poll_period_ticks=3)
    driver.arm()
    before = driver.state()["rx_polls"]

    for _ in range(3):
        driver.tick()

    assert driver.state()["rx_polls"] == before + 1


def test_command_is_written_every_tick_regardless_of_slot():
    driver, axis = make_driver(poll_slot=0, poll_period_ticks=6)
    driver.arm()

    for expected in (0.2, 0.4, 0.6):
        driver.set_velocity(expected)
        driver.tick()
        assert axis.controller.input_vel == pytest.approx(expected * 5.0)


def test_state_never_touches_usb():
    """CornerModule.tick() 이 매 tick 부르므로 여기서 I/O 하면 예산이 터진다."""
    driver, axis = make_driver(poll_period_ticks=1)
    driver.arm()
    driver.tick()
    axis.fail = True                       # 이제 어떤 속성 접근도 터진다

    snapshot = driver.state()              # 예외가 나면 안 된다

    assert snapshot["target_vel"] == 0.0


# ── 건강 · 예외 흡수 ─────────────────────────────────────────────────────


def test_arm_seeds_the_receive_timestamp():
    """arm 직후 stale 오판이 나면 첫 tick 에 estop 이 걸린다 (steer_ak40 실사고)."""
    make_driver.now = 100.0
    driver, _axis = make_driver(stale_ms=500.0)
    driver.arm()

    assert driver.state()["stale"] is False


def test_stale_turns_true_when_polls_stop_landing():
    make_driver.now = 100.0
    driver, axis = make_driver(stale_ms=500.0, poll_period_ticks=1)
    driver.arm()
    driver.tick()
    assert driver.state()["stale"] is False

    axis.fail = True
    make_driver.now = 100.9                # 900 ms 경과
    driver.tick()

    assert driver.state()["stale"] is True


def test_usb_failure_is_absorbed_and_counted():
    driver, axis = make_driver(poll_period_ticks=1)
    driver.arm()
    axis.fail = True

    driver.set_velocity(0.5)
    driver.tick()                          # 예외가 새어나오면 제어 루프가 죽는다

    assert driver.state()["error_count"] >= 1


def test_axis_error_is_surfaced_for_the_corner_fault_check():
    driver, axis = make_driver(poll_period_ticks=1)
    driver.arm()
    axis.error = 0x40
    driver.tick()

    assert driver.state()["axis_error"] == 0x40


# ── 상태 전이 ────────────────────────────────────────────────────────────


def test_arm_refuses_an_uncalibrated_axis():
    axis = FakeAxis(calibrated=False)
    driver, _ = make_driver(axis=axis)

    with pytest.raises(RuntimeError, match="캘리"):
        driver.arm()


def test_arm_zeroes_the_command_before_closing_the_loop():
    driver, axis = make_driver()
    driver.set_velocity(9.0)

    driver.arm()

    assert axis.controller.input_vel == 0.0
    assert axis.requested_state == _AXIS_CLOSED_LOOP


def test_estop_zeroes_and_idles():
    driver, axis = make_driver()
    driver.arm()
    driver.set_velocity(1.0)

    driver.estop()

    assert axis.controller.input_vel == 0.0
    assert axis.requested_state == _AXIS_IDLE
    assert driver.state()["target_vel"] == 0.0


def test_estop_survives_a_dead_link():
    driver, axis = make_driver()
    driver.arm()
    axis.fail = True

    driver.estop()                         # 예외가 새면 estop 전파가 끊긴다

    assert driver.state()["target_vel"] == 0.0


# ── 보드 풀 ──────────────────────────────────────────────────────────────


def test_board_pool_finds_each_board_once():
    calls = []

    def finder(serial):
        calls.append(serial)
        return FakeBoard(axis0=FakeAxis(), axis1=FakeAxis())

    pool = UsbBoardPool(finder=finder)
    pool.axis("SN1", 0)
    pool.axis("SN1", 1)
    pool.axis("SN2", 0)

    assert calls == ["SN1", "SN2"]


def test_board_pool_enumerator_is_sorted_and_deduped():
    pool = UsbBoardPool(enumerator=lambda: ["B", "A", "B"])

    assert pool.discover_serials() == ["A", "B"]


# ── 캘리브레이션 ─────────────────────────────────────────────────────────


def test_calibrate_sets_the_verified_scan_omega():
    """0.5 세션 실측: 기본 12.566 이면 offset 캘리가 깨진다."""
    axis = FakeAxis(calibrated=False)
    ticks = iter([0.0, 1.0, 2.0])

    def clock():
        return next(ticks, 3.0)

    def sleep(_seconds):
        axis.current_state = _AXIS_IDLE

    calibrate_axis(axis, "test", current_lim_a=9.0, clock=clock, sleep=sleep)

    assert axis.encoder.config.calib_scan_omega == pytest.approx(6.0)
    assert axis.motor.config.current_lim == pytest.approx(9.0)
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd motor_control && python -m pytest corner_module/tests/test_drive_odrive_usb_axis.py -q
```

Expected: `ModuleNotFoundError: No module named 'corner_module.drive_odrive_usb_axis'`

- [ ] **Step 3: 드라이버 구현**

`motor_control/corner_module/drive_odrive_usb_axis.py` 신규:

```python
"""ODrive 3.6(USB) **다보드** 구동 드라이버 — `DriveOdriveCan` 과 동일 계약.

`drive_odrive_usb.py` 의 `DriveOdriveUsb` 는 `find_any()` 로 보드 1장의 axis1
하나만 잡는 레거시 단축 경로다(아직 `corner_module/teleop_dualsense.py` 가
쓰므로 건드리지 않는다). 이 모듈은 **3보드 6축**을 시리얼+축 인덱스로 주소지정
하고, 감속비·미러 반전·건강 키를 붙여 `ChassisManager` 가 CAN 과 동일하게 쓸 수
있게 한다.

연결·캘리·arm 시퀀스는 `motor_control/drive/bl70200/dualsense_usb_teleop.py`
에서 이식했다 — 벤치에서 3보드 6축 실회전이 확인된 코드다.

**평면 int 상수를 쓰는 이유** (`drive_odrive_usb.py:6-13`): 이 펌웨어의 odrive
라이브러리는 클래스 enum 객체를 config 에 대입하면 int 변환이 안 돼
``TypeError`` 가 난다. 부수 효과로 이 모듈은 `odrive` 없이 import 되므로
무하드웨어 pytest 가 그대로 돈다.

**지연 예산.** USB 는 속성 하나가 왕복 1회다. 50 Hz = 20 ms 안에 6축 쓰기 +
텔레메트리 읽기가 들어가야 하므로, **쓰기는 매 tick, 읽기는 축별 라운드로빈**
으로 나눈다. `state()` 는 캐시만 반환하며 USB 를 절대 건드리지 않는다 —
`CornerModule.tick()` 이 매 tick 호출하기 때문이다.
"""
import math
import time

from corner_module.actuator import DriveActuator

ODRIVE_VID = 0x1209
ODRIVE_PID = 0x0D32

# odrive.enums 의 평면 상수와 같은 값 (위 docstring 참조)
_AXIS_IDLE = 1                      # AXIS_STATE_IDLE
_AXIS_FULL_CALIBRATION = 3          # AXIS_STATE_FULL_CALIBRATION_SEQUENCE
_AXIS_CLOSED_LOOP = 8               # AXIS_STATE_CLOSED_LOOP_CONTROL
_CTRL_VELOCITY = 2                  # CONTROL_MODE_VELOCITY_CONTROL
_INPUT_PASSTHROUGH = 1              # INPUT_MODE_PASSTHROUGH

#: 캘리 스캔 각속도. ⚠️ 기본값 12.566 은 이 모터에서 offset 캘리가 깨진다
#: (2026-07-04 벤치 실측). 바꾸지 말 것.
CALIB_SCAN_OMEGA = 6.0
CALIB_SCAN_DISTANCE = 150
CALIB_RANGE = 0.05
CALIB_CURRENT_A = 8.0
CALIB_HEADROOM_CURRENT_A = 20.0


class UsbBoardPool:
    """USB ODrive 핸들 풀 — 보드 1장당 `find_any()` 를 1회만 돈다.

    같은 보드의 두 축이 하나의 핸들을 공유하므로, 6축 = 3회 탐색이다.

    Parameters
    ----------
    finder:
        테스트 주입용 ``serial -> board_handle``. 없으면 `odrive.find_any` 사용.
    enumerator:
        테스트 주입용 ``() -> [serial]``. 없으면 pyusb 로 VID/PID 열거.
    """

    def __init__(self, find_timeout: float = 20.0, finder=None, enumerator=None):
        self._find_timeout = find_timeout
        self._finder = finder
        self._enumerator = enumerator
        self._boards = {}

    def discover_serials(self) -> list:
        """USB 에 붙은 ODrive 시리얼 목록(정렬·중복제거). 실패하면 빈 리스트."""
        if self._enumerator is not None:
            return sorted({str(s) for s in self._enumerator()})
        try:
            import usb.core
            import usb.util
        except Exception:
            return []
        serials = []
        try:
            found = usb.core.find(find_all=True, idVendor=ODRIVE_VID,
                                  idProduct=ODRIVE_PID)
            for dev in found:
                try:
                    serial = usb.util.get_string(dev, dev.iSerialNumber)
                except Exception:
                    serial = None
                if serial:
                    serials.append(serial.strip())
        except Exception:
            return []
        return sorted(set(serials))

    def board(self, serial: str):
        if serial not in self._boards:
            self._boards[serial] = self._find(serial)
        return self._boards[serial]

    def _find(self, serial: str):
        if self._finder is not None:
            return self._finder(serial)
        import odrive
        handle = odrive.find_any(serial_number=serial, timeout=self._find_timeout)
        if handle is None:
            raise RuntimeError(
                "ODrive %s USB 미발견 — 케이블·전원·권한(udev) 확인." % serial)
        return handle

    def axis(self, serial: str, axis_index: int):
        if axis_index not in (0, 1):
            raise ValueError("axis_index must be 0 or 1, got %r" % (axis_index,))
        return getattr(self.board(serial), "axis%d" % axis_index)

    def close(self) -> None:
        self._boards.clear()


def calibrate_axis(axis, label: str, current_lim_a: float = 9.0,
                   timeout_s: float = 120.0, clock=None, sleep=None) -> bool:
    """축 1개 풀캘리 (~55 s 회전, 출력축이 자유로워야 한다).

    ⚠️ 캘리 결과는 **RAM-only** 라 전원 사이클마다 다시 해야 한다.
    """
    clock = clock or time.monotonic
    sleep = sleep or time.sleep
    axis.error = 0
    axis.motor.error = 0
    axis.encoder.error = 0
    axis.controller.error = 0
    axis.motor.config.calibration_current = CALIB_CURRENT_A
    axis.motor.config.current_lim = CALIB_HEADROOM_CURRENT_A
    axis.encoder.config.calib_scan_omega = CALIB_SCAN_OMEGA
    axis.encoder.config.calib_scan_distance = CALIB_SCAN_DISTANCE
    axis.encoder.config.calib_range = CALIB_RANGE
    axis.requested_state = _AXIS_FULL_CALIBRATION
    started = clock()
    while axis.current_state != _AXIS_IDLE:
        if clock() - started > timeout_s:
            break
        sleep(0.5)
    axis.motor.config.current_lim = current_lim_a
    return bool(axis.motor.is_calibrated and axis.encoder.is_ready)


class DriveOdriveUsbAxis(DriveActuator):
    """ODrive 3.6 USB 축 1개 — velocity control, 바퀴 프레임 입출력.

    Parameters
    ----------
    pool, serial, axis_index:
        어느 보드의 어느 축인지. `axis_index` 1 = 각 보드의 M1 = 로봇 우측.
    node_id:
        같은 축의 CAN node 번호. 텔레메트리 라벨 통일용이며 통신에는 안 쓴다.
    gear_ratio:
        모터 회전수 / 바퀴 회전수 (기본 5.0). 양수·유한이어야 한다.
    invert:
        우측 바퀴의 물리적 미러 장착. 반전은 **USB 경계에서만** 하고 드라이버
        바깥은 전부 바퀴 프레임이다.
    stale_ms:
        마지막 성공 폴링 후 이 시간을 넘으면 ``state()["stale"]=True``.
        라운드로빈 주기(`poll_period_ticks / loop_hz`)의 3~4배로 둔다.
    poll_slot, poll_period_ticks:
        ``tick_index % poll_period_ticks == poll_slot`` 인 tick 에만 읽는다.
        6축이면 슬롯 0~5 · 주기 6 → 축당 갱신 주기 = 6 / loop_hz.
    """

    def __init__(self, pool, serial, axis_index, *, node_id=None,
                 gear_ratio: float = 5.0, invert: bool = False,
                 current_lim_a: float = 9.0, stale_ms: float = 500.0,
                 poll_slot: int = 0, poll_period_ticks: int = 6, clock=None):
        gear_ratio = float(gear_ratio)
        if not math.isfinite(gear_ratio) or gear_ratio <= 0.0:
            raise ValueError("gear_ratio must be finite and positive")
        if int(poll_period_ticks) < 1:
            raise ValueError("poll_period_ticks must be >= 1")
        self._pool = pool
        self._serial = str(serial)
        self._axis_index = int(axis_index)
        self._node_id = node_id
        self._gear_ratio = gear_ratio
        self._invert = bool(invert)
        self._sign = -1.0 if self._invert else 1.0
        self._current_lim_a = float(current_lim_a)
        self._stale_ms = float(stale_ms)
        self._poll_period_ticks = int(poll_period_ticks)
        self._poll_slot = int(poll_slot) % self._poll_period_ticks
        self._now = time.monotonic if clock is None else clock
        self._axis = None
        self._target_vel = 0.0
        self._actual_vel = 0.0
        self._cur_a = 0.0
        self._axis_error = 0
        self._axis_state = 0
        self._last_rx_ms = None
        self._tick_index = -1
        self._rx_polls = 0
        self._error_count = 0

    @property
    def invert(self) -> bool:
        return self._invert

    @property
    def label(self) -> str:
        return "%s/ax%d" % (self._serial[-6:], self._axis_index)

    def _now_ms(self) -> float:
        return self._now() * 1000.0

    # ------------------------------------------------------------------
    # Actuator 인터페이스
    # ------------------------------------------------------------------

    def connect(self) -> None:
        self._axis = self._pool.axis(self._serial, self._axis_index)

    def arm(self) -> None:
        """velocity-control + passthrough 로 폐루프 진입 (input_vel=0 점프 방지)."""
        axis = self._axis
        if not axis.motor.is_calibrated or not axis.encoder.is_ready:
            raise RuntimeError(
                "%s 미캘리 — 캘리는 RAM-only 라 전원 사이클마다 다시 해야 한다."
                % self.label)
        axis.error = 0
        axis.motor.error = 0
        axis.encoder.error = 0
        axis.controller.error = 0
        axis.motor.config.current_lim = self._current_lim_a
        axis.encoder.config.ignore_illegal_hall_state = True
        axis.controller.config.control_mode = _CTRL_VELOCITY
        axis.controller.config.input_mode = _INPUT_PASSTHROUGH
        axis.controller.input_vel = 0.0
        self._target_vel = 0.0
        axis.requested_state = _AXIS_CLOSED_LOOP
        self._poll_now()      # arm 직후 stale 오판 방지 — last_rx 시드
                              # (steer_ak40 이 이걸 빼서 첫 tick estop 이 났었다)

    def disarm(self) -> None:
        self._target_vel = 0.0
        self._safe(lambda axis: setattr(axis.controller, "input_vel", 0.0))
        self._safe(lambda axis: setattr(axis, "requested_state", _AXIS_IDLE))

    def set_velocity(self, turns_per_s: float) -> None:
        """다음 tick() 에 전송할 **바퀴** 목표 속도(turns/s)."""
        self._target_vel = turns_per_s

    def tick(self) -> None:
        """쓰기는 매 tick, 읽기는 자기 슬롯 차례에만."""
        self._tick_index += 1
        motor_tps = self._target_vel * self._gear_ratio * self._sign
        self._safe(lambda axis: setattr(axis.controller, "input_vel", motor_tps))
        if self._tick_index % self._poll_period_ticks == self._poll_slot:
            self._poll_now()

    def state(self) -> dict:
        """**캐시만** 반환한다 — USB 를 건드리지 않는다."""
        age_ms = (
            None if self._last_rx_ms is None
            else max(0.0, self._now_ms() - self._last_rx_ms)
        )
        return {
            "node_id": self._node_id,
            "serial": self._serial,
            "axis": self._axis_index,
            "target_vel": self._target_vel,
            "actual_vel": self._actual_vel * self._sign / self._gear_ratio,
            "cur_a": self._cur_a,
            "axis_error": self._axis_error,
            "axis_state": self._axis_state,
            "stale": age_ms is None or age_ms > self._stale_ms,
            "last_rx_age_ms": age_ms,
            "rx_polls": self._rx_polls,
            "error_count": self._error_count,
        }

    def estop(self) -> None:
        self._target_vel = 0.0
        self._safe(lambda axis: setattr(axis.controller, "input_vel", 0.0))
        self._safe(lambda axis: setattr(axis, "requested_state", _AXIS_IDLE))

    def close(self) -> None:
        self._safe(lambda axis: setattr(axis, "requested_state", _AXIS_IDLE))

    # ------------------------------------------------------------------
    # 내부 — 모든 USB 접근은 여기를 지난다
    # ------------------------------------------------------------------

    def _safe(self, action) -> bool:
        """USB 접근 1회. 예외는 흡수하고 error_count 만 올린다(제어 루프 보호)."""
        if self._axis is None:
            return False
        try:
            action(self._axis)
        except Exception:
            self._error_count += 1
            return False
        return True

    def _poll_now(self) -> None:
        """텔레메트리 4종을 한 번에 읽는다. 하나라도 실패하면 캐시를 갱신하지 않아
        `stale` 로 드러난다."""
        if self._axis is None:
            return
        try:
            axis = self._axis
            actual_vel = axis.encoder.vel_estimate
            cur_a = axis.motor.current_control.Iq_measured
            axis_error = axis.error
            axis_state = axis.current_state
        except Exception:
            self._error_count += 1
            return
        self._actual_vel = actual_vel
        self._cur_a = cur_a
        self._axis_error = axis_error
        self._axis_state = axis_state
        self._last_rx_ms = self._now_ms()
        self._rx_polls += 1
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
cd motor_control && python -m pytest corner_module/tests/test_drive_odrive_usb_axis.py -q
```

Expected: 전부 PASS.

- [ ] **Step 5: 커밋**

```bash
git add motor_control/corner_module/drive_odrive_usb_axis.py \
        motor_control/corner_module/tests/test_drive_odrive_usb_axis.py
git commit -m "feat(corner_module): USB 다보드 구동 드라이버 DriveOdriveUsbAxis

3보드 6축을 시리얼+축으로 주소지정하고 감속비·미러반전·건강키를 붙여
DriveOdriveCan 과 같은 계약을 만든다. 연결·캘리·arm 시퀀스는 벤치에서 6축
실회전이 확인된 dualsense_usb_teleop.py 에서 이식했다.

USB 는 속성 하나가 왕복 1회라 쓰기는 매 tick, 읽기는 축별 라운드로빈으로
나눈다. state() 는 캐시만 반환한다 — CornerModule.tick() 이 매 tick 부른다."
```

---

### Task 4: `build_usb_skid_corners()` + 보드 레지스트리

**Files:**
- Modify: `motor_control/chassis/chassis_manager.py` (`build_real_corners` 다음, `:156` 부근)
- Create: `config/bl70200_boards.json`
- Test: `motor_control/chassis/tests/test_chassis_manager.py` (파일 끝에 추가)

**Interfaces:**
- Consumes: Task 3 의 `UsbBoardPool`·`DriveOdriveUsbAxis`, 기존 `DEFAULT_WHEEL_MAP`·`RIGHT_WHEELS`·`NullSteer`·`CornerModule`, `drive.bl70200.board_registry.load`
- Produces: `build_usb_skid_corners(registry_path, cfg=None, wheel_map=None, gear_ratio=5.0, current_lim_a=9.0, stale_ms=500.0, pool=None) -> dict[str, CornerModule]`

- [ ] **Step 1: 실패하는 테스트 작성**

`motor_control/chassis/tests/test_chassis_manager.py` 끝에 추가:

```python
import json

from chassis.chassis_manager import build_usb_skid_corners
from corner_module.null_steer import NullSteer


class _FakePool:
    """UsbBoardPool 스텁 — 어떤 (serial, axis) 든 고유 객체를 돌려준다."""

    def __init__(self):
        self.requested = []

    def axis(self, serial, axis_index):
        self.requested.append((serial, axis_index))
        return object()

    def close(self):
        pass


def _write_registry(tmp_path, mapping):
    path = tmp_path / "boards.json"
    path.write_text(json.dumps(mapping), encoding="utf-8")
    return path


def _good_registry(tmp_path):
    # axis0 = 좌(홀수 node), axis1 = 우(짝수 node)
    return _write_registry(tmp_path, {
        "SN-A": [11, 12], "SN-B": [13, 14], "SN-C": [15, 16],
    })


def test_usb_skid_corners_cover_every_default_wheel(tmp_path):
    corners = build_usb_skid_corners(_good_registry(tmp_path), pool=_FakePool())

    assert sorted(corners) == [
        "front_left", "front_right", "mid_left", "mid_right",
        "rear_left", "rear_right"]


def test_usb_skid_corners_use_null_steer_everywhere(tmp_path):
    """AK 를 전혀 쓰지 않는 구성이므로 조향 액추에이터가 하나도 없어야 한다."""
    corners = build_usb_skid_corners(_good_registry(tmp_path), pool=_FakePool())

    assert all(isinstance(c.steer, NullSteer) for c in corners.values())


def test_usb_skid_inverts_exactly_the_right_wheels(tmp_path):
    corners = build_usb_skid_corners(_good_registry(tmp_path), pool=_FakePool())

    inverted = {name for name, c in corners.items() if c.drive.invert}
    assert inverted == {"front_right", "mid_right", "rear_right"}


def test_usb_skid_assigns_a_distinct_poll_slot_per_wheel(tmp_path):
    """전 축이 같은 tick 에 폴링하면 라운드로빈이 무의미해진다."""
    corners = build_usb_skid_corners(_good_registry(tmp_path), pool=_FakePool())

    slots = sorted(c.drive._poll_slot for c in corners.values())
    assert slots == [0, 1, 2, 3, 4, 5]
    assert all(c.drive._poll_period_ticks == 6 for c in corners.values())


def test_usb_skid_rejects_a_registry_missing_a_drive_node(tmp_path):
    path = _write_registry(tmp_path, {"SN-A": [11, 12], "SN-B": [13, 14]})

    with pytest.raises(ValueError, match="node 15"):
        build_usb_skid_corners(path, pool=_FakePool())


def test_usb_skid_rejects_a_registry_that_breaks_the_mirror_convention(tmp_path):
    """우측 바퀴(node 12/14/16)는 반드시 각 보드의 axis1 이어야 한다 — 실물 미러 장착."""
    path = _write_registry(tmp_path, {
        "SN-A": [12, 11], "SN-B": [13, 14], "SN-C": [15, 16],
    })

    with pytest.raises(ValueError, match="axis1"):
        build_usb_skid_corners(path, pool=_FakePool())


def test_usb_skid_never_opens_can(tmp_path, monkeypatch):
    """can0 을 아예 건드리지 않는 것이 이 구성의 존재 이유다."""
    import corner_module.drive_odrive_can as drive_can

    def explode(*_args, **_kwargs):
        raise AssertionError("CAN driver must not be constructed")

    monkeypatch.setattr(drive_can, "DriveOdriveCan", explode)

    build_usb_skid_corners(_good_registry(tmp_path), pool=_FakePool())


def test_usb_skid_pairs_with_a_four_wheel_map(tmp_path):
    from chassis.chassis_manager import FOUR_WHEEL_MAP

    corners = build_usb_skid_corners(
        _good_registry(tmp_path), wheel_map=FOUR_WHEEL_MAP, pool=_FakePool())

    assert sorted(corners) == [
        "front_left", "front_right", "rear_left", "rear_right"]
    assert all(c.drive._poll_period_ticks == 4 for c in corners.values())
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd motor_control && python -m pytest chassis/tests/test_chassis_manager.py -q
```

Expected: `ImportError: cannot import name 'build_usb_skid_corners'`

- [ ] **Step 3: `drive.bl70200.board_registry` import 경로 확인**

```bash
cd motor_control && python -c "import importlib; print(importlib.import_module('drive.bl70200.board_registry').load)"
```

Expected: `<function load at 0x...>` — **2026-08-04 확인 완료**. `motor_control/drive/`
와 `drive/bl70200/` 에는 `__init__.py` 가 없지만 PEP 420 암묵적 네임스페이스
패키지로 import 된다(`drive/bl70200/tests/test_board_registry.py:8` 이 이미 같은
방식을 쓴다). 그래서 빌더도 `importlib.import_module` 을 쓴다 — 정적 `from
drive.bl70200 import board_registry` 는 린터가 못 찾을 수 있다.

- [ ] **Step 4: 빌더 구현**

`motor_control/chassis/chassis_manager.py` 의 `build_real_corners()` 정의 바로 뒤(`:156` 다음)에 추가:

```python
def build_usb_skid_corners(registry_path, cfg: CornerConfig = None,
                           wheel_map=None, gear_ratio: float = 5.0,
                           current_lim_a: float = 9.0,
                           stale_ms: float = 500.0, pool=None) -> dict:
    """🛠️ **USB 스키드 구성** — 조향 없이 ODrive USB 구동만으로 코너를 만든다.

    AK 조향을 전혀 쓰지 않으므로 조향은 전부 `NullSteer` 이고, **can0 을 열지
    않는다**(`RealCanSession`·`CanWatchdog` 도 필요 없다). 반드시
    `kinematics.skid_geometry()` 와 **짝으로** 쓴다 — 애커만 기하와 섞으면
    조향 명령이 갈 곳이 없다.

    바퀴↔노드 권위는 `DEFAULT_WHEEL_MAP` 을 그대로 쓰고, 노드↔(시리얼, 축) 만
    보드 레지스트리에서 해석한다. 표를 새로 만들지 않아 CAN 경로와 권위가
    하나로 유지된다.

    ⚠️ 레지스트리에 없는 노드는 **거부**한다. 바퀴를 잘못 배정하면 조용히 틀린
    방향으로 주행한다.

    Parameters
    ----------
    registry_path:
        `drive.bl70200.board_registry` JSON — ``{serial: [axis0_node, axis1_node]}``.
    pool:
        테스트 주입용 `UsbBoardPool` 대체품.
    """
    import importlib

    board_registry = importlib.import_module("drive.bl70200.board_registry")
    usb_mod = importlib.import_module("corner_module.drive_odrive_usb_axis")

    source_map = tuple(wheel_map or DEFAULT_WHEEL_MAP)
    registry = board_registry.load(registry_path)

    node_to_axis = {}
    for serial, (node_axis0, node_axis1) in registry.items():
        node_to_axis[node_axis0] = (serial, 0)
        node_to_axis[node_axis1] = (serial, 1)

    resolved = []
    for wm in source_map:
        if wm.drive_node_id not in node_to_axis:
            raise ValueError(
                "보드 레지스트리에 구동 node %d(%s)가 없다: %s"
                % (wm.drive_node_id, wm.wheel, registry_path))
        serial, axis_index = node_to_axis[wm.drive_node_id]
        inverted = wm.wheel in RIGHT_WHEELS
        if inverted != (axis_index == 1):
            # 각 보드의 M1(axis1)이 로봇 우측이며 좌측과 미러로 장착돼 있다
            # (2026-07-28 실물 확인). 레지스트리가 이를 어기면 부호가 뒤집힌다.
            raise ValueError(
                "레지스트리가 미러 장착 규약을 어긴다: %s(node %d) → %s/axis%d. "
                "우측 바퀴(%s)는 반드시 axis1 이어야 한다."
                % (wm.wheel, wm.drive_node_id, serial, axis_index,
                   ", ".join(RIGHT_WHEELS)))
        resolved.append((wm, serial, axis_index, inverted))

    pool = pool if pool is not None else usb_mod.UsbBoardPool()
    period = len(resolved)
    cfg = cfg or CornerConfig()
    corners = {}
    for slot, (wm, serial, axis_index, inverted) in enumerate(resolved):
        drive = usb_mod.DriveOdriveUsbAxis(
            pool, serial, axis_index, node_id=wm.drive_node_id,
            gear_ratio=gear_ratio, invert=inverted,
            current_lim_a=current_lim_a, stale_ms=stale_ms,
            poll_slot=slot, poll_period_ticks=period,
        )
        corners[wm.wheel] = CornerModule(NullSteer(), drive, cfg)
    return corners
```

- [ ] **Step 5: 보드 레지스트리 파일 생성**

`config/bl70200_boards.json` 신규. **시리얼은 벤치에서 채운다** — 지금은 자리표시가 아니라 *실행 가능한 스텁*으로, 값이 비어 있으면 로더가 거부하도록 명시적으로 잘못된 상태를 두지 않고 파일 자체를 만들지 않는 편이 낫다. 따라서 이 스텝은 **파일 생성 대신 벤치 절차를 문서화**한다.

`config/README-bl70200-boards.md` 신규:

```markdown
# BL70200 보드 레지스트리 (`config/bl70200_boards.json`)

USB 스키드 구성이 "어느 보드의 어느 축이 어느 바퀴인가"를 푸는 유일한 근거다.
CAN node 번호를 매개로 하므로 CAN 경로와 권위가 하나로 유지된다.

## 만드는 법 (젯슨, 컨테이너 안)

1. USB 에 붙은 보드 시리얼을 뽑는다.

   ```bash
   python3 -c "
   from corner_module.drive_odrive_usb_axis import UsbBoardPool
   print(UsbBoardPool().discover_serials())"
   ```

   ✅ 기대 출력: 시리얼 3개짜리 리스트.

2. 각 보드가 어느 node 쌍인지는 **CAN 셋업 때 정한 값**이다
   (`bl70200_setup.py --node N` 로 써 넣은 값). 보드 1장 = 연속한 두 node.

3. 아래 형식으로 저장한다. `[axis0_node, axis1_node]` 순서이며
   **axis1 = 로봇 우측**이므로 짝수 node 가 뒤에 온다.

   ```json
   {
     "<시리얼-1>": [11, 12],
     "<시리얼-2>": [13, 14],
     "<시리얼-3>": [15, 16]
   }
   ```

4. 검증한다.

   ```bash
   cd motor_control && python3 -c "
   from chassis.chassis_manager import build_usb_skid_corners
   c = build_usb_skid_corners('../config/bl70200_boards.json')
   print(sorted(c))"
   ```

   ✅ 기대 출력: 바퀴 6개 이름. ❌ `ValueError` 가 나면 메시지가 어느 node 가
   빠졌는지 / 미러 규약을 어겼는지 알려준다.

⚠️ 이 파일은 **기기마다 다르다**(시리얼이 보드 고유값). 레포에 커밋하지 않는다.
```

`.gitignore` 에 추가:

```
config/bl70200_boards.json
```

- [ ] **Step 6: 테스트 통과 확인**

```bash
cd motor_control && python -m pytest chassis/tests/test_chassis_manager.py -q
```

Expected: 전부 PASS.

- [ ] **Step 7: 커밋**

```bash
git add motor_control/chassis/chassis_manager.py \
        motor_control/chassis/tests/test_chassis_manager.py \
        config/README-bl70200-boards.md .gitignore
git commit -m "feat(chassis): build_usb_skid_corners() — USB 스키드 코너 빌더

바퀴↔노드 권위는 DEFAULT_WHEEL_MAP 을 그대로 쓰고 노드↔(시리얼,축)만 보드
레지스트리에서 해석한다. 조향은 전부 NullSteer 이고 can0 을 열지 않는다.

레지스트리에 없는 노드와 미러 장착 규약 위반(우측 바퀴가 axis1 이 아님)은
시작을 거부한다 — 바퀴 오배정은 조용히 틀린 방향으로 주행한다.
축별 폴링 슬롯을 0..N-1 로 배분해 라운드로빈이 실제로 분산되게 한다."
```

---

### Task 5: `teleop_server --skid-usb`

**Files:**
- Modify: `motor_control/chassis/teleop_server.py:320-346` (`_parse_args`), `:349-401` (`main` 초기화 블록)
- Test: `motor_control/chassis/tests/test_teleop.py` (파일 끝에 추가)

**Interfaces:**
- Consumes: Task 1 `skid_geometry`, Task 4 `build_usb_skid_corners`
- Produces: CLI 플래그 `--skid-usb`, `--board-registry`, `--track-gain`, `--usb-current-lim`

- [ ] **Step 1: 실패하는 테스트 작성**

`motor_control/chassis/tests/test_teleop.py` 끝에 추가:

```python
from chassis import teleop_server


def test_skid_usb_and_four_wheel_are_mutually_exclusive():
    with pytest.raises(SystemExit):
        teleop_server._parse_args([
            "--diagnostic-direct-can", "--confirm-arm-stowed",
            "--skid-usb", "--four-wheel",
        ])


def test_skid_usb_defaults_are_declared():
    args = teleop_server._parse_args([
        "--diagnostic-direct-can", "--confirm-arm-stowed", "--skid-usb",
    ])

    assert args.skid_usb is True
    assert args.track_gain == pytest.approx(1.0)
    assert args.board_registry.endswith("bl70200_boards.json")


def test_skid_usb_builds_usb_corners_and_never_touches_can(monkeypatch):
    """can0 을 안 여는 것이 이 모드의 존재 이유다 — 워치독도 lock 도 없어야 한다."""
    calls = {"watchdog": 0, "can_session": 0, "usb_corners": 0}

    class _Watchdog:
        def __init__(self, *_a, **_k):
            calls["watchdog"] += 1

        def start(self):
            pass

    class _Session:
        def __init__(self, *_a, **_k):
            calls["can_session"] += 1

    import corner_module.can_watchdog as watchdog_mod
    import chassis.runtime_lock as lock_mod
    import chassis.chassis_manager as manager_mod

    monkeypatch.setattr(watchdog_mod, "CanWatchdog", _Watchdog)
    monkeypatch.setattr(lock_mod, "RealCanSession", _Session)

    def _fake_usb_corners(*_a, **_k):
        calls["usb_corners"] += 1
        raise RuntimeError("stop here — 초기화 경로만 검증한다")

    monkeypatch.setattr(manager_mod, "build_usb_skid_corners", _fake_usb_corners)

    with pytest.raises(RuntimeError, match="stop here"):
        teleop_server.main([
            "--diagnostic-direct-can", "--confirm-arm-stowed",
            "--skid-usb", "--no-us100",
        ])

    assert calls["usb_corners"] == 1
    assert calls["watchdog"] == 0
    assert calls["can_session"] == 0
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd motor_control && python -m pytest chassis/tests/test_teleop.py -q
```

Expected: `AttributeError: 'Namespace' object has no attribute 'skid_usb'`

- [ ] **Step 3: 플래그 추가**

`motor_control/chassis/teleop_server.py` 의 `_parse_args` 안, `--v-knee` 인자 다음(`:342-343` 뒤)에 추가:

```python
    p.add_argument("--skid-usb", action="store_true",
                   help="🛠️ AK 조향을 전혀 쓰지 않고 ODrive USB 6축만으로 "
                        "스키드(차동) 조향한다. can0 을 열지 않는다. "
                        "⚠️ 조향축이 무통전이라 스키드 중 밀릴 수 있다 — "
                        "바퀴 띄운 벤치에서 먼저 확인할 것")
    p.add_argument("--board-registry", default="config/bl70200_boards.json",
                   help="USB 보드 시리얼↔CAN node 레지스트리 JSON "
                        "(--skid-usb 전용, 만드는 법은 config/README-bl70200-boards.md)")
    p.add_argument("--track-gain", type=float, default=1.0,
                   help="스키드 유효 윤거 배수 (>1.0 이 보정 방향, 기본 1.0=무보정)")
    p.add_argument("--usb-current-lim", type=float, default=9.0,
                   help="USB 축당 전류 제한 A (기본 9.0, 전원 약하면 2.0)")
```

`_parse_args` 의 `args = p.parse_args(argv)` 다음, `require_diagnostic_direct_can` 호출 **앞**에 추가:

```python
    if args.skid_usb and args.four_wheel:
        p.error("--skid-usb 와 --four-wheel 은 같이 쓸 수 없다 (v1 미지원)")
```

- [ ] **Step 4: `main()` 초기화 분기**

`motor_control/chassis/teleop_server.py:353-357` 을 교체한다. 기존:

```python
    from chassis.chassis_manager import ChassisManager, ChassisConfig, build_real_corners
    from chassis.runtime_lock import RealCanSession
    from corner_module.can_watchdog import CanWatchdog

    CanWatchdog(args.channel).start()    # mttcan TX 웻지 자가복구 (데몬 스레드)
```

교체 후:

```python
    import chassis.chassis_manager as manager_mod
    from chassis.chassis_manager import ChassisManager, ChassisConfig

    if not args.skid_usb:
        # USB 스키드는 can0 을 아예 열지 않으므로 워치독도 lock 도 필요 없다.
        from corner_module.can_watchdog import CanWatchdog
        CanWatchdog(args.channel).start()   # mttcan TX 웻지 자가복구 (데몬 스레드)
```

`can_session` 생성부(`:362-365`)를 교체. 기존:

```python
    can_session = RealCanSession(
        channel=args.channel,
        owner="teleop_server",
    )
```

교체 후:

```python
    can_session = None
    if not args.skid_usb:
        from chassis.runtime_lock import RealCanSession
        can_session = RealCanSession(
            channel=args.channel,
            owner="teleop_server",
        )
```

코너 생성부(`:380-399`)를 교체. 기존:

```python
        can_session.__enter__()
        wheel_map = None
        if args.four_wheel:
            from chassis.chassis_manager import FOUR_WHEEL_MAP
            wheel_map = FOUR_WHEEL_MAP
            print("🛠️ 4륜 모드 — 중륜(node 13/14) 없이 앞뒤 4륜만 구동한다 (임시 구성)")
        corners = build_real_corners(
            args.channel, wheel_map=wheel_map,
            friction_ff=args.friction_ff, v_knee_turns_s=args.v_knee,
        )

        cfg = ChassisConfig(min_drive_turns_per_s=args.min_rev)
        if args.four_wheel:
            # ★ 기하와 매핑은 **반드시 짝**이어야 한다 (이름이 어긋나면 KeyError)
            from chassis.kinematics import four_wheel_geometry
            cfg.geometry = four_wheel_geometry()
```

교체 후:

```python
        if can_session is not None:
            can_session.__enter__()
        wheel_map = None
        cfg = ChassisConfig(min_drive_turns_per_s=args.min_rev)
        if args.four_wheel:
            from chassis.chassis_manager import FOUR_WHEEL_MAP
            wheel_map = FOUR_WHEEL_MAP
            print("🛠️ 4륜 모드 — 중륜(node 13/14) 없이 앞뒤 4륜만 구동한다 (임시 구성)")
        if args.skid_usb:
            # ★ 기하와 코너는 **반드시 짝** — 조향이 NullSteer 뿐이므로 애커만
            #   기하를 물리면 조향 명령이 갈 곳이 없다.
            from chassis.kinematics import skid_geometry
            corners = manager_mod.build_usb_skid_corners(
                args.board_registry, wheel_map=wheel_map,
                current_lim_a=args.usb_current_lim,
            )
            cfg.geometry = skid_geometry(args.track_gain)
            print("🛠️ USB 스키드 모드 — AK 조향 미사용, can0 미개방 "
                  "(track_gain %.2f)" % args.track_gain)
            print("⚠️ 조향축이 무통전이다. 스키드 중 각이 밀리는지 육안 확인할 것.")
        else:
            corners = manager_mod.build_real_corners(
                args.channel, wheel_map=wheel_map,
                friction_ff=args.friction_ff, v_knee_turns_s=args.v_knee,
            )
            if args.four_wheel:
                # ★ 기하와 매핑은 **반드시 짝**이어야 한다 (이름이 어긋나면 KeyError)
                from chassis.kinematics import four_wheel_geometry
                cfg.geometry = four_wheel_geometry()
```

정리부에서 `can_session.close()` 를 호출하는 두 곳(`:410`, 그리고 함수 끝의 정리 블록)을 `None` 안전하게 고친다:

```python
        if can_session is not None:
            can_session.close()
```

`grep -n "can_session" motor_control/chassis/teleop_server.py` 로 **모든** 사용처를 찾아 빠짐없이 고친다.

- [ ] **Step 5: 테스트 통과 확인**

```bash
cd motor_control && python -m pytest chassis/tests/ corner_module/tests/ -q
```

Expected: 전부 PASS. 기존 CAN 경로 텔레옵 테스트가 하나도 깨지지 않아야 한다.

- [ ] **Step 6: 커밋**

```bash
git add motor_control/chassis/teleop_server.py motor_control/chassis/tests/test_teleop.py
git commit -m "feat(chassis): teleop_server --skid-usb — USB 스키드 무선 주행

can0 을 열지 않으므로 CanWatchdog 도 RealCanSession 도 잡지 않는다. US-100 은
UART 라 그대로 살아 있고 인터록·워치독·estop 전파도 전부 유효하다.
지금까지 버려지던 클라이언트의 left_x 가 드디어 omega 로 쓰인다."
```

---

### Task 6: Phase A 벤치 검증 (P0~P2)

**Files:** 없음 (실행 게이트). 결과는 `docs/reports/2026-08-04-usb-skid-bench.md` 에 기록한다.

**⚠️ 이 태스크는 사람이 물리 확인을 해야 한다. 코드만으로 완료 선언하지 않는다.**

- [ ] **Step 1: 좀비 프로세스 정리**

```bash
docker exec powertrain_jetson sh -c "ps aux | grep -E 'teleop|chassis' | grep -v grep"
```

무언가 돌고 있으면 죽인다. **v=0 을 계속 명령하는 좀비가 새 테스트와 싸워 반나절을 날린 전례가 있다.**

- [ ] **Step 2: 보드 레지스트리 작성**

`config/README-bl70200-boards.md` 절차대로 `config/bl70200_boards.json` 을 만들고 검증 명령까지 통과시킨다.

- [ ] **Step 3: 캘리 (전원 사이클 후 필수)**

```bash
docker exec -it powertrain_jetson python3 \
  /workspace/motor_control/drive/bl70200/dualsense_usb_teleop.py --calibrate --no-auto-arm
```

✅ 기대: 6축 각각 `motor=True encoder=True err=0x0`.

- [ ] **Step 4: P0 — 지연 실측 (바퀴 들린 상태)**

`scripts/usb_skid_latency_probe.py` 를 만들어 실행한다:

```python
#!/usr/bin/env python3
"""P0 — USB 스키드 50 Hz 지연 예산 실측. 바퀴를 띄우고 돌린다.

쓰기 6축 + 라운드로빈 읽기 1축이 20 ms 안에 들어가는지 본다.
"""
import argparse
import statistics
import time

from chassis.chassis_manager import build_usb_skid_corners


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--registry", default="config/bl70200_boards.json")
    parser.add_argument("--seconds", type=float, default=100.0)
    parser.add_argument("--hz", type=float, default=50.0)
    args = parser.parse_args()

    corners = build_usb_skid_corners(args.registry)
    for corner in corners.values():
        corner.connect()
        corner.arm()

    period = 1.0 / args.hz
    durations = []
    overruns = 0
    deadline = time.monotonic() + args.seconds
    while time.monotonic() < deadline:
        started = time.monotonic()
        for corner in corners.values():
            corner.set(0.0, 0.0)
            corner.tick()
        elapsed = time.monotonic() - started
        durations.append(elapsed * 1000.0)
        if elapsed > period:
            overruns += 1
        time.sleep(max(0.0, period - elapsed))

    durations.sort()
    p99 = durations[int(len(durations) * 0.99)]
    print("tick ms  p50 %.3f  p99 %.3f  max %.3f  overrun %d / %d"
          % (statistics.median(durations), p99, durations[-1],
             overruns, len(durations)))
    for name, corner in sorted(corners.items()):
        state = corner.drive.state()
        print("  %-12s age %6.1f ms  polls %5d  errors %d  stale %s"
              % (name, state["last_rx_age_ms"] or -1.0, state["rx_polls"],
                 state["error_count"], state["stale"]))
    for corner in corners.values():
        corner.disarm()
        corner.close()


if __name__ == "__main__":
    main()
```

```bash
docker exec -it powertrain_jetson sh -c \
  "cd /workspace/motor_control && python3 ../scripts/usb_skid_latency_probe.py"
```

✅ **통과 기준: p99 < 20 ms, overrun 0, 축별 `age` < 500 ms, `stale False`.**

❌ 넘으면 `poll_period_ticks` 를 늘리거나(`build_usb_skid_corners` 호출부에서
`stale_ms` 도 같이 늘린다) `ChassisConfig.loop_hz` 를 낮춘다. **조정한 값을
`chassis_manager.build_usb_skid_corners` 기본값에 반영하고 그 근거를 주석에 남긴다.**

- [ ] **Step 5: P1 — 스키드 E2E (바퀴 들린 상태)**

사용자에게 **바퀴가 전부 들려 있는지 물리 확인**을 요청한 뒤:

```bash
# 젯슨
docker exec -it powertrain_jetson sh -c \
  "cd /workspace/motor_control && python3 -m chassis.teleop_server \
     --skid-usb --no-us100 --diagnostic-direct-can --board-registry ../config/bl70200_boards.json"
# 노트북
python3 motor_control/laptop/laptop_client_chassis.py --host <젯슨IP> --port 9000
```

✅ 확인 항목:
- RT 만: 6축 전부 같은 방향 회전
- 좌스틱 좌로 끝까지 (v=0): 좌 3축 역방향 · 우 3축 정방향
- 전진 + 좌회전: 우측이 더 빠름
- **조향축(AK)이 무통전인데 각이 밀리는지 육안 확인** — 밀리면 즉시 중단하고
  기계적 고정이 필요하다고 보고한다 (SW 범위 밖)

⚠️ 바퀴 지령 0.3 rev/s 미만은 HALL 코깅존이라 텔레메트리만 그럴듯하고 실물이
안 돈다. **v ≥ 0.4 m/s 로 시험하고 반드시 육안으로 확인한다.**

- [ ] **Step 6: P2 — 안전 경로**

- ○ 버튼 estop → 6축 0
- 노트북 클라이언트 강제 종료(링크 끊김) → 6축 0
- `--no-us100` 을 빼고 재실행 후 센서 앞에 손 → 구동 0

- [ ] **Step 7: 결과 기록 + 커밋**

`docs/reports/2026-08-04-usb-skid-bench.md` 에 P0 수치(p50/p99/overrun/축별 age),
P1 육안 결과, P2 결과, 조향축 밀림 여부를 적는다.

```bash
git add scripts/usb_skid_latency_probe.py docs/reports/2026-08-04-usb-skid-bench.md
git commit -m "test(chassis): USB 스키드 벤치 검증 P0~P2 + 지연 실측 프로브"
```

---

# Phase B — 콘솔 모드 선택

**Phase A 의 P0~P2 가 통과한 뒤에 시작한다.** USB 스키드가 실제로 도는 것을 확인하지 못한 채 GUI 를 얹으면 어느 층이 문제인지 못 가른다.

---

### Task 7: `ChassisManager` 조향모드 런타임 전환

**Files:**
- Modify: `motor_control/chassis/chassis_manager.py` — `ChassisConfig`(`:50-67`), `ChassisManager.__init__`(`:162-217`), `tick()`(`:570-625`)
- Test: `motor_control/chassis/tests/test_chassis_manager.py`

**Interfaces:**
- Consumes: Task 1 `skid_geometry`
- Produces:
  - 모듈 상수 `STEERING_ACKERMANN = "ackermann"`, `STEERING_SKID = "skid"`
  - `ChassisConfig.steering_mode: str`, `.skid_track_gain: float`, `.steer_settle_deg: float`, `.steer_settle_timeout_s: float`
  - `ChassisManager.steering_mode -> str` (property), `.steering_available -> bool` (property), `.request_steering_mode(mode) -> tuple[bool, str]`

- [ ] **Step 1: 실패하는 테스트 작성**

`motor_control/chassis/tests/test_chassis_manager.py` 끝에 추가한다.

⚠️ **이 파일에는 이미 `_fake_corners(cfg=None)`(`:37`)와 `_armed_manager(cfg=None, clock=None)`(`:47`), `FakeClock`(`:54`)이 있다.** 같은 이름을 새로 정의하면 기존 테스트 100여 개가 그 정의를 쓰게 되어 조용히 깨진다. **재정의하지 말고 그대로 쓴다.**

- `_fake_corners()` = 조향 4륜 `FakeSteer` + 중륜 2개 `NullSteer` + 구동 전부 `FakeDrive`
- `_armed_manager(cfg)` 의 `cfg` 는 `ChassisConfig` 이며, 내부에서 `_fake_corners(cfg.corner)` 를 만들고 `connect()` + `arm()` 까지 한다

```python
from chassis.chassis_manager import STEERING_ACKERMANN, STEERING_SKID


def _skid_cfg(**kw):
    """조향모드 관련 인자를 얹은 ChassisConfig — 기존 _armed_manager 에 넘긴다."""
    return ChassisConfig(**kw)


def _steer_hard_left(manager, ticks=6):
    """조향을 한계 근처까지 꺾어 놓는다 (전환 게이트를 실제로 시험하려면 필요)."""
    manager.set(0.3, 0.6)
    for _ in range(ticks):
        manager.tick()
    return abs(manager.corners["front_left"].steer.state()["actual_deg"])


def test_manager_starts_in_ackermann_by_default():
    manager = _armed_manager()

    assert manager.steering_mode == STEERING_ACKERMANN
    assert [w.steerable for w in manager.cfg.geometry.wheels] == [
        True, True, False, False, True, True]


def test_manager_can_start_in_skid():
    manager = _armed_manager(_skid_cfg(steering_mode=STEERING_SKID))

    assert manager.steering_mode == STEERING_SKID
    assert all(not w.steerable for w in manager.cfg.geometry.wheels)


def test_switch_to_skid_holds_until_steering_settles():
    manager = _armed_manager()
    assert _steer_hard_left(manager) > 3.0

    ok, _reason = manager.request_steering_mode(STEERING_SKID)

    assert ok is True
    assert manager.steering_mode == STEERING_ACKERMANN      # 아직 안 바뀜
    assert "steer_mode_change" in manager.safety_snapshot().hold_sources


def test_switch_to_skid_completes_once_steering_reaches_zero():
    manager = _armed_manager()
    _steer_hard_left(manager)
    manager.request_steering_mode(STEERING_SKID)

    for _ in range(40):
        manager.tick()

    assert manager.steering_mode == STEERING_SKID
    assert all(not w.steerable for w in manager.cfg.geometry.wheels)
    assert "steer_mode_change" not in manager.safety_snapshot().hold_sources


def test_drive_is_gated_to_zero_while_the_mode_change_is_pending():
    """45° 꺾인 채 차동이 걸리면 격렬한 스크럽이 난다."""
    manager = _armed_manager()
    _steer_hard_left(manager)
    manager.request_steering_mode(STEERING_SKID)
    manager.tick()

    for corner in manager.corners.values():
        assert corner.drive.state()["target_vel"] == pytest.approx(0.0)


def test_mode_change_timeout_cancels_and_releases_the_hold():
    """인터록 hold 를 밖에서 지울 ops 경로가 없어, 유지하면 차체가 갇힌다."""
    clock = FakeClock()
    manager = _armed_manager(_skid_cfg(steer_settle_timeout_s=1.0), clock=clock)
    for corner in manager.corners.values():
        if isinstance(corner.steer, FakeSteer):
            corner.steer._actual = 30.0    # 수렴하지 않는 조향

    manager.request_steering_mode(STEERING_SKID)
    for corner in manager.corners.values():
        if isinstance(corner.steer, FakeSteer):
            corner.steer._actual = 30.0    # tick 이 수렴시키기 전에 다시 벌려 둔다
    clock.t = 5.0
    manager.tick()

    assert manager.steering_mode == STEERING_ACKERMANN
    assert "steer_mode_change" not in manager.safety_snapshot().hold_sources


def test_skid_rebuilds_the_wheel_consistency_monitor():
    """모니터가 생성자에서 한 번만 만들어지므로 기하를 바꾸면 같이 갈아야 한다."""
    manager = _armed_manager()
    before = manager._wheel_consistency
    manager.request_steering_mode(STEERING_SKID)
    for _ in range(40):
        manager.tick()

    assert manager._wheel_consistency is not before
    assert manager._wheel_consistency.geometry is manager.cfg.geometry


def test_ackermann_is_refused_without_steering_hardware():
    corners = {
        name: CornerModule(NullSteer(), FakeDrive(), CornerConfig())
        for name in _fake_corners()
    }
    manager = ChassisManager(corners, _skid_cfg(steering_mode=STEERING_SKID))
    manager.connect()

    ok, reason = manager.request_steering_mode(STEERING_ACKERMANN)

    assert ok is False
    assert reason == "steering_unavailable"
    assert manager.steering_available is False


def test_unknown_steering_mode_is_refused():
    manager = _armed_manager()

    ok, reason = manager.request_steering_mode("crab")

    assert ok is False
    assert reason == "unknown_steering_mode"


def test_skid_track_gain_reaches_the_active_geometry():
    manager = _armed_manager(
        _skid_cfg(steering_mode=STEERING_SKID, skid_track_gain=1.4))
    wheels = {w.name: w for w in manager.cfg.geometry.wheels}

    assert wheels["front_left"].y == pytest.approx(0.2725 * 1.4)


def test_drive_limit_survives_a_mode_switch():
    """chassis_node 가 생성 전에 올려놓은 상한이 전환에서 날아가면 안 된다."""
    cfg = ChassisConfig()
    cfg.geometry.drive_limit_mps = 1.5
    manager = _armed_manager(cfg)
    manager.request_steering_mode(STEERING_SKID)
    for _ in range(40):
        manager.tick()

    assert manager.cfg.geometry.drive_limit_mps == pytest.approx(1.5)
```

⚠️ `test_ackermann_is_refused_without_steering_hardware` 는 6바퀴 전부 `NullSteer` 인 코너를 만든다 — `_fake_corners()` 는 조향 4륜에 `FakeSteer` 를 넣으므로 그대로 쓸 수 없고 이름만 빌려 온다.

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd motor_control && python -m pytest chassis/tests/test_chassis_manager.py -q
```

Expected: `ImportError: cannot import name 'STEERING_ACKERMANN'`

- [ ] **Step 3: 상수와 설정 추가**

`motor_control/chassis/chassis_manager.py` 상단, `COMPONENTS` 정의(`:44`) 다음:

```python
STEERING_ACKERMANN = "ackermann"
STEERING_SKID = "skid"
STEERING_MODES = (STEERING_ACKERMANN, STEERING_SKID)
_STEER_MODE_HOLD = "steer_mode_change"
```

import 에 `skid_geometry` 를 더한다 (`:22`):

```python
from chassis.kinematics import (
    ChassisGeometry, default_geometry, skid_geometry, solve,
)
```

`ChassisConfig`(`:50-67`) 끝에 추가:

```python
    steering_mode: str = STEERING_ACKERMANN   # 기동 시 조향모드
    skid_track_gain: float = 1.0              # 스키드 유효 윤거 배수(>1 이 보정 방향)
    steer_settle_deg: float = 3.0             # 전환 전 조향 0° 수렴 허용 오차
    steer_settle_timeout_s: float = 3.0       # 수렴 대기 상한 — 넘으면 전환 취소
```

- [ ] **Step 4: `__init__` 에 기하 파생 로직 추가**

`ChassisManager.__init__` 에서 `self.cfg = cfg or ChassisConfig()` 다음 줄에 삽입:

```python
        # 애커만 기하를 원본으로 보관한다. 스키드 기하는 steerable 정보를 잃어
        # 되돌릴 수 없으므로, 활성 기하는 항상 여기서 파생한다.
        # (drive_limit_mps 등 호출자가 생성 전에 올려놓은 값도 함께 승계된다.)
        self._base_geometry = self.cfg.geometry
        if self.cfg.steering_mode not in STEERING_MODES:
            raise ValueError("unknown steering_mode: %r" % (self.cfg.steering_mode,))
        self._steering_mode = self.cfg.steering_mode
        self._pending_steering_mode = None
        self._steering_change_started_s = None
        self.cfg.geometry = self._geometry_for(self._steering_mode)
```

⚠️ 이 블록은 `self._wheel_consistency` 생성(`:188-191`)과 매핑 검증(`:208-217`)
**앞**에 와야 한다. 둘 다 `self.cfg.geometry` 를 읽기 때문이다.

`_now` 초기화가 이 블록보다 뒤에 있으므로, `self._now` 를 쓰는 부분은 없다는 것을 확인한다(위 블록은 시계를 쓰지 않는다).

- [ ] **Step 5: 전환 API 구현**

`ChassisManager` 에 메서드를 추가한다 (`component_mask` property 근처, `:222` 부근):

```python
    def _geometry_for(self, mode: str) -> ChassisGeometry:
        if mode == STEERING_SKID:
            return skid_geometry(self.cfg.skid_track_gain, base=self._base_geometry)
        return self._base_geometry

    @property
    def steering_mode(self) -> str:
        return self._steering_mode

    @property
    def pending_steering_mode(self):
        return self._pending_steering_mode

    @property
    def steering_available(self) -> bool:
        """실제 조향 액추에이터가 하나라도 있는가 (없으면 애커만 불가)."""
        return any(
            not isinstance(corner.steer, NullSteer)
            for corner in self.corners.values()
        )

    def request_steering_mode(self, mode: str) -> tuple:
        """조향모드 전환을 요청한다. 즉시 바뀌지 않고 tick() 이 수렴 후 적용한다.

        45° 로 꺾인 상태에서 곧바로 차동 구동을 걸면 격렬한 스크럽이 나므로,
        전환 대기 중에는 MOTION_HOLD 로 구동을 0 으로 묶고 조향 0° 수렴을
        기다린다.

        Returns
        -------
        (accepted, reason)
        """
        if mode not in STEERING_MODES:
            return False, "unknown_steering_mode"
        if mode == STEERING_ACKERMANN and not self.steering_available:
            return False, "steering_unavailable"
        if mode == self._steering_mode and self._pending_steering_mode is None:
            return True, "already_%s" % mode
        self._v = 0.0
        self._omega = 0.0
        self._pending_steering_mode = mode
        self._steering_change_started_s = self._now()
        self._interlock.set_motion_hold(
            _STEER_MODE_HOLD, True, "steering mode -> %s" % mode)
        logger.warning("조향모드 전환 대기: %s → %s", self._steering_mode, mode)
        return True, "pending_%s" % mode

    def _tick_steering_mode(self) -> None:
        """전환 대기 중이면 조향 0° 수렴을 확인하고 기하를 스왑한다."""
        mode = self._pending_steering_mode
        if mode is None:
            return
        settled = True
        for wheel in self._base_geometry.wheels:
            if not wheel.steerable:
                continue
            corner = self.corners.get(wheel.name)
            if corner is None:
                continue
            try:
                actual_deg = corner.steer.state().get("actual_deg", 0.0)
            except Exception:
                actual_deg = 0.0            # 읽을 수 없으면 코너 자체 검사가 잡는다
            if abs(actual_deg) > self.cfg.steer_settle_deg:
                settled = False
        if settled:
            self._apply_steering_mode(mode)
            return
        elapsed = self._now() - self._steering_change_started_s
        if elapsed <= self.cfg.steer_settle_timeout_s:
            return                          # 계속 대기 (hold 유지)
        # 타임아웃: 전환을 취소하고 hold 를 해제해 직전 모드로 남는다.
        # 인터록 motion hold 를 밖에서 지울 ops 경로가 없어서(authority_clear_hold
        # 는 authority 전용) hold 를 유지하면 운전자가 풀 방법 없이 갇힌다.
        # 직전 모드는 방금까지 정상 동작하던 구성이므로 안전하다. 조향이 진짜
        # 고장이면 CornerModule.tick() 의 fault/stale/과전류 검사가 estop 을 건다.
        self._pending_steering_mode = None
        self._steering_change_started_s = None
        self._interlock.set_motion_hold(_STEER_MODE_HOLD, False)
        logger.error(
            "조향 0° 수렴 실패(%.1fs) → 조향모드 전환 취소, %s 유지",
            elapsed, self._steering_mode)

    def _apply_steering_mode(self, mode: str) -> None:
        self._steering_mode = mode
        self.cfg.geometry = self._geometry_for(mode)
        self._wheel_consistency = WheelConsistencyMonitor(
            self.cfg.geometry,
            self.cfg.wheel_consistency,
        )
        self._pending_steering_mode = None
        self._steering_change_started_s = None
        self._interlock.set_motion_hold(_STEER_MODE_HOLD, False)
        logger.warning("조향모드 전환 완료: %s", mode)
```

- [ ] **Step 6: `tick()` 에 배선**

`ChassisManager.tick()`(`:570`) 에서 `safety = self._interlock.snapshot()` **앞**에 한 줄을 넣는다:

```python
        self._tick_steering_mode()
        safety = self._interlock.snapshot()
```

전환 대기 중에는 `_STEER_MODE_HOLD` 때문에 `safety.state != RUN` 이므로 기존
`drive_enabled = safety.state == RUN`(`:600`) 이 구동을 0 으로 게이팅하고,
조향은 계속 명령되어 0° 로 수렴한다. **추가 게이팅 코드가 필요 없다.**

- [ ] **Step 7: 테스트 통과 확인**

```bash
cd motor_control && python -m pytest chassis/tests/ corner_module/tests/ -q
```

Expected: 전부 PASS.

- [ ] **Step 8: 커밋**

```bash
git add motor_control/chassis/chassis_manager.py motor_control/chassis/tests/test_chassis_manager.py
git commit -m "feat(chassis): 조향모드 런타임 전환 (애커만 <-> 스키드)

45도 꺾인 채 차동이 걸리면 격렬한 스크럽이 나므로, 전환은 MOTION_HOLD 로 구동을
묶고 조향 0도 수렴을 기다린 뒤에만 기하를 스왑한다. 활성 기하는 항상 애커만
원본에서 파생한다 — 스키드 기하는 steerable 정보를 잃어 되돌릴 수 없다.

타임아웃 시에는 전환을 취소하고 hold 를 해제한다. 인터록 motion hold 를 밖에서
지울 ops 경로가 없어 유지하면 차체가 갇힌다."
```

---

### Task 8: `chassis_node` — 트랜스포트 파라미터 · 조향모드 서비스 · 상태 방송

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/chassis_node.py` — 파라미터 선언부, 코너 생성부(`:325-400`), 서비스 등록부(`:690-702` 부근), `_publish_safety_state`(`:1663-1693`)
- Test: `ros2/src/powertrain_ros/test/test_chassis_node_steering_mode.py` (신규)

**Interfaces:**
- Consumes: Task 4 `build_usb_skid_corners`, Task 7 `request_steering_mode`/`steering_mode`/`steering_available`
- Produces:
  - 파라미터 `drive_transport` (`"can"`|`"usb"`, 기본 `"can"`), `steering_mode` (`"ackermann"`|`"skid"`), `board_registry`, `skid_track_gain`, `usb_current_lim`
  - 서비스 `~/steer_mode_skid` (`SetBool`, true=스키드)
  - `/chassis/safety_state` JSON 에 `steering_mode`, `steering_available`, `drive_transport`

- [ ] **Step 1: 실패하는 테스트 작성**

`ros2/src/powertrain_ros/test/test_chassis_node_steering_mode.py` 신규. 기존 chassis_node 테스트가 노드를 어떻게 띄우는지 `ls ros2/src/powertrain_ros/test/ | grep chassis` 로 확인하고 그 픽스처 방식을 따른다. 노드 전체를 띄우기 무거우면 핸들러만 직접 검증한다:

```python
"""chassis_node 의 조향모드 서비스와 상태 방송 — 핸들러 단위 검증.

노드 전체 기동은 P5 스모크가 담당한다. 여기서는 서비스 핸들러가
ChassisManager 계약을 올바로 옮기는지만 본다.
"""
import json

import pytest

from powertrain_ros import chassis_node as node_mod


class _FakeManager:
    def __init__(self, available=True, accept=True, reason="pending_skid"):
        self.steering_mode = "ackermann"
        self.steering_available = available
        self.requested = []
        self._accept = accept
        self._reason = reason

    def request_steering_mode(self, mode):
        self.requested.append(mode)
        return self._accept, self._reason


class _Response:
    success = None
    message = None


class _Request:
    def __init__(self, data):
        self.data = data


def _node_with(manager):
    node = object.__new__(node_mod.ChassisNode)
    node.cm = manager
    return node


def test_steer_mode_service_maps_true_to_skid():
    manager = _FakeManager()
    node = _node_with(manager)

    response = node._srv_steer_mode_skid(_Request(True), _Response())

    assert manager.requested == ["skid"]
    assert response.success is True


def test_steer_mode_service_maps_false_to_ackermann():
    manager = _FakeManager(reason="pending_ackermann")
    node = _node_with(manager)

    node._srv_steer_mode_skid(_Request(False), _Response())

    assert manager.requested == ["ackermann"]


def test_steer_mode_service_reports_refusal():
    manager = _FakeManager(available=False, accept=False,
                           reason="steering_unavailable")
    node = _node_with(manager)

    response = node._srv_steer_mode_skid(_Request(False), _Response())

    assert response.success is False
    assert "steering_unavailable" in response.message


def test_steer_mode_service_survives_a_missing_manager():
    node = object.__new__(node_mod.ChassisNode)
    node.cm = None

    response = node._srv_steer_mode_skid(_Request(True), _Response())

    assert response.success is False


def test_usb_transport_rejects_ackermann_at_startup():
    with pytest.raises(ValueError, match="ackermann"):
        node_mod.validate_transport_mode("usb", "ackermann")


def test_usb_transport_accepts_skid():
    node_mod.validate_transport_mode("usb", "skid")     # 예외가 나면 실패


def test_can_transport_accepts_both():
    node_mod.validate_transport_mode("can", "ackermann")
    node_mod.validate_transport_mode("can", "skid")


def test_unknown_transport_is_refused():
    with pytest.raises(ValueError, match="drive_transport"):
        node_mod.validate_transport_mode("spi", "skid")


def test_safety_state_payload_carries_the_mode_fields():
    payload = node_mod.steering_state_fields(
        _FakeManager(), drive_transport="usb")

    assert payload == {
        "steering_mode": "ackermann",
        "steering_available": True,
        "drive_transport": "usb",
    }
    json.dumps(payload)          # 직렬화 가능해야 한다
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd ros2 && python -m pytest src/powertrain_ros/test/test_chassis_node_steering_mode.py -q
```

Expected: `AttributeError: module 'powertrain_ros.chassis_node' has no attribute 'validate_transport_mode'`

- [ ] **Step 3: 순수 헬퍼 2개 추가**

`ros2/src/powertrain_ros/powertrain_ros/chassis_node.py` 의 클래스 정의 **앞**(모듈 최상단 상수 근처)에 추가:

```python
DRIVE_TRANSPORTS = ("can", "usb")


def validate_transport_mode(drive_transport, steering_mode):
    """기동 파라미터 조합을 검증한다. 잘못되면 ``ValueError``.

    USB 스택에는 조향 액추에이터가 없다(조향 AK45-36 은 CAN 전용). 따라서
    USB × 애커만은 존재할 수 없는 조합이다.
    """
    if drive_transport not in DRIVE_TRANSPORTS:
        raise ValueError(
            "unknown drive_transport %r (expected one of %s)"
            % (drive_transport, ", ".join(DRIVE_TRANSPORTS)))
    if steering_mode not in ("ackermann", "skid"):
        raise ValueError("unknown steering_mode %r" % (steering_mode,))
    if drive_transport == "usb" and steering_mode == "ackermann":
        raise ValueError(
            "drive_transport=usb 는 ackermann 을 지원하지 않는다 — "
            "USB 스택에는 조향 액추에이터가 없다(AK 는 CAN 전용). "
            "steering_mode=skid 로 기동하라.")


def steering_state_fields(manager, drive_transport):
    """`/chassis/safety_state` 에 실을 조향/트랜스포트 필드."""
    return {
        "steering_mode": str(getattr(manager, "steering_mode", "ackermann")),
        "steering_available": bool(getattr(manager, "steering_available", False)),
        "drive_transport": str(drive_transport),
    }
```

- [ ] **Step 4: 서비스 핸들러 추가**

`_srv_component_enable`(`:1288`) 바로 앞에 추가:

```python
    def _srv_steer_mode_skid(self, request, response):
        """SetBool: true=스키드, false=애커만. 즉시 바뀌지 않고 수렴 후 적용된다."""
        mode = "skid" if bool(request.data) else "ackermann"
        manager = getattr(self, "cm", None)
        if manager is None:
            response.success = False
            response.message = "chassis manager unavailable"
            return response
        accepted, reason = manager.request_steering_mode(mode)
        response.success = bool(accepted)
        response.message = reason or mode
        return response
```

- [ ] **Step 5: 파라미터 선언 · 서비스 등록 · 코너 생성 분기**

파라미터 선언부(다른 `declare_parameter` 들이 모인 곳)에 추가:

```python
        self.declare_parameter("drive_transport", "can")
        self.declare_parameter("steering_mode", "ackermann")
        self.declare_parameter("board_registry", "config/bl70200_boards.json")
        self.declare_parameter("skid_track_gain", 1.0)
        self.declare_parameter("usb_current_lim", 9.0)
```

코너 생성부(`:332` `four_wheel = bool(...)` 다음)에 추가:

```python
        drive_transport = str(self.get_parameter("drive_transport").value)
        steering_mode = str(self.get_parameter("steering_mode").value)
        validate_transport_mode(drive_transport, steering_mode)
        self._drive_transport = drive_transport
```

`cfg = ChassisConfig(...)` 호출에 인자를 더한다:

```python
        cfg = ChassisConfig(
            watchdog_ms=self._cmd_timeout * 1000.0,
            min_drive_turns_per_s=min_rev,
            extraction_enabled=extraction_enabled,
            steering_mode=steering_mode,
            skid_track_gain=float(self.get_parameter("skid_track_gain").value),
        )
```

⚠️ `ChassisConfig.geometry` 는 **애커만 원본**이어야 한다 — `ChassisManager` 가
`steering_mode` 를 보고 파생한다. `four_wheel` 분기는 그대로 `four_wheel_geometry()`
를 넣으면 되고, 스키드 파생은 매니저가 알아서 한다.

`if fake:` / `else:` 의 실물 분기(`:361-380`)를 3갈래로 바꾼다:

```python
        if fake:
            corners = self._build_fake_corners(cfg)
            self.get_logger().warning(
                "FAKE mode: no real motors are controlled"
            )
        elif drive_transport == "usb":
            # USB 스키드 — can0 을 아예 열지 않으므로 RealCanSession 을 잡지 않는다.
            corners = manager_mod.build_usb_skid_corners(
                str(self.get_parameter("board_registry").value),
                wheel_map=wheel_map,
                gear_ratio=gear_ratio,
                current_lim_a=float(self.get_parameter("usb_current_lim").value),
            )
            self.get_logger().warning(
                "🛠️ USB 스키드 — AK 조향 미사용, can0 미개방. "
                "조향축이 무통전이니 각이 밀리는지 확인할 것.")
        else:
            from chassis.runtime_lock import RealCanSession
            ...   # 기존 코드 그대로
```

`build_usb_skid_corners` 를 부르려면 상단 import 블록(`:325-330`)에
`import chassis.chassis_manager as manager_mod` 를 더한다.

서비스 등록부(컴포넌트 서비스들 근처, `:690-702`)에 추가:

```python
        self.create_service(
            SetBool,
            "~/steer_mode_skid",
            self._srv_steer_mode_skid,
        )
```

- [ ] **Step 6: `_publish_safety_state` 에 필드 추가**

`chassis_node.py:1685-1687` 의 `"component_mask": ...` 다음에 추가:

```python
                **steering_state_fields(
                    self.cm, getattr(self, "_drive_transport", "can")),
```

- [ ] **Step 7: 테스트 통과 확인**

```bash
cd ros2 && python -m pytest src/powertrain_ros/test/test_chassis_node_steering_mode.py -q
cd ros2 && colcon test --packages-select powertrain_ros && colcon test-result --verbose
```

Expected: 신규 테스트 PASS + 기존 powertrain_ros 스위트 회귀 없음.

- [ ] **Step 8: 커밋**

```bash
git add ros2/src/powertrain_ros/powertrain_ros/chassis_node.py \
        ros2/src/powertrain_ros/test/test_chassis_node_steering_mode.py
git commit -m "feat(ros): chassis_node 트랜스포트 파라미터 + 조향모드 서비스

drive_transport=usb 는 build_usb_skid_corners 로 코너를 만들고 can0 을 열지
않는다. USB x 애커만은 조향 액추에이터가 없어 기동 시 거부한다.
~/steer_mode_skid (SetBool) 로 런타임 전환하고, steering_mode /
steering_available / drive_transport 를 /chassis/safety_state 로 방송한다."
```

---

### Task 9: 오도메트리 노드 기하 동기화 (결함 B)

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/state_estimation.py:141-146` (`StateEstimator`)
- Modify: `ros2/src/powertrain_ros/powertrain_ros/odometry_node.py:46-50`
- Modify: `ros2/src/powertrain_ros/powertrain_ros/imu_tilt_node.py:65,127`
- Test: `ros2/src/powertrain_ros/test/test_state_estimation_geometry_sync.py` (신규)

**Interfaces:**
- Consumes: Task 1 `skid_geometry`, Task 8 의 `/chassis/safety_state` 필드
- Produces: `StateEstimator.set_geometry(geometry) -> None`, 모듈 함수 `powertrain_ros.state_estimation.geometry_for_steering_mode(mode, track_gain=1.0)`

**배경:** `odometry_node.py:46` 과 `imu_tilt_node.py:65` 가 `default_geometry()` 를 고정으로 들고 있다. 스키드 주행 중 이쪽이 애커만 기하로 남으면, 조향륜마다 측면식 `[0, 1, xᵢ]·(vx,vy,ω) = drive·sin(0°) = 0` 이 들어가고 `x = +0.4377` 과 `x = −0.4377` 이 동시에 걸려 **vy 뿐 아니라 ω 까지 0 으로 강제**된다.

- [ ] **Step 1: 실패하는 테스트 작성**

`ros2/src/powertrain_ros/test/test_state_estimation_geometry_sync.py` 신규:

```python
"""스키드 주행 중 추정 노드가 애커만 기하로 남으면 요레이트가 0 으로 눌린다."""
import pytest

from chassis.kinematics import default_geometry, skid_geometry, solve
from chassis.odometry import WheelObservation, solve_twist
from powertrain_ros.state_estimation import (
    StateEstimator, geometry_for_steering_mode,
)


def _skid_observations(v_mps, omega_rad_s):
    geom = skid_geometry()
    result = solve(geom, v_mps, omega_rad_s)
    return [
        WheelObservation(name=name, drive_mps=wc.drive_mps, steer_deg=0.0)
        for name, wc in result.wheels.items()
    ]


def test_ackermann_geometry_crushes_yaw_for_skid_observations():
    """이것이 결함 B 다 — 기하가 어긋나면 요레이트가 0 으로 눌린다."""
    twist = solve_twist(default_geometry(), _skid_observations(0.0, 0.8))

    assert twist.omega == pytest.approx(0.0, abs=1e-3)


def test_skid_geometry_recovers_yaw_for_the_same_observations():
    twist = solve_twist(skid_geometry(), _skid_observations(0.0, 0.8))

    assert twist.omega == pytest.approx(0.8, abs=1e-6)


def test_geometry_for_steering_mode_maps_both_modes():
    assert any(w.steerable for w in geometry_for_steering_mode("ackermann").wheels)
    assert not any(w.steerable for w in geometry_for_steering_mode("skid").wheels)


def test_geometry_for_steering_mode_applies_the_track_gain():
    wheels = {w.name: w for w in geometry_for_steering_mode("skid", 1.4).wheels}

    assert wheels["front_left"].y == pytest.approx(0.2725 * 1.4)


def test_geometry_for_steering_mode_falls_back_to_ackermann():
    """알 수 없는 값이 오면 조용히 스키드로 바꾸지 않는다."""
    assert any(w.steerable for w in geometry_for_steering_mode("crab").wheels)


def test_estimator_geometry_can_be_swapped():
    estimator = StateEstimator(default_geometry())

    estimator.set_geometry(skid_geometry())

    assert not any(w.steerable for w in estimator.geometry.wheels)
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd ros2 && python -m pytest src/powertrain_ros/test/test_state_estimation_geometry_sync.py -q
```

Expected: `ImportError: cannot import name 'geometry_for_steering_mode'`

- [ ] **Step 3: `state_estimation.py` 에 헬퍼와 스왑 API 추가**

import 에 `skid_geometry` 를 더하고(`from chassis.kinematics import ... skid_geometry`), 모듈 함수를 추가:

```python
def geometry_for_steering_mode(steering_mode, track_gain=1.0):
    """조향모드 문자열 → 추정용 기하.

    스키드에서 애커만 기하를 쓰면 조향륜마다 측면식이 들어가고, 앞뒤 x 부호가
    반대라 vy 뿐 아니라 **요레이트까지 0 으로 강제**된다. 알 수 없는 값이면
    조용히 스키드로 바꾸지 않고 애커만으로 남는다(보수적 기본값).
    """
    from chassis.kinematics import default_geometry, skid_geometry

    if str(steering_mode) == "skid":
        return skid_geometry(float(track_gain))
    return default_geometry()
```

`StateEstimator` 에 메서드 추가 (`__init__` 다음):

```python
    def set_geometry(self, geometry) -> None:
        """조향모드 전환에 맞춰 추정 기하를 갈아끼운다."""
        self.geometry = geometry
```

- [ ] **Step 4: `odometry_node` 가 `/chassis/safety_state` 를 구독**

`odometry_node.py` 의 `__init__` 에서 `self.geom = default_geometry()` 다음에 추가:

```python
        self._steering_mode = "ackermann"
        self._skid_track_gain = 1.0
        self.create_subscription(
            String, "/chassis/safety_state", self._on_safety_state, 10)
```

`String` import 가 없으면 `from std_msgs.msg import String` 을 더한다.

핸들러를 추가:

```python
    def _on_safety_state(self, message):
        """조향모드가 바뀌면 추정 기하를 함께 갈아끼운다.

        chassis_node 와 기하가 어긋나면 스키드 주행 중 요레이트 추정이 0 으로
        눌린다(설계 문서 §6.2). 전환은 MOTION_HOLD 하에서만 일어나 차체가 정지
        상태이므로, 두 노드의 기하가 잠시 어긋나도 잘못된 트위스트가 적분되지
        않는다.
        """
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            return
        mode = str(payload.get("steering_mode", self._steering_mode))
        if mode == self._steering_mode:
            return
        self._steering_mode = mode
        self.geom = geometry_for_steering_mode(mode, self._skid_track_gain)
        self.estimator.set_geometry(self.geom)
        self.get_logger().warning("추정 기하 전환: %s" % mode)
```

`json` 과 `geometry_for_steering_mode` import 를 더한다.

- [ ] **Step 5: `imu_tilt_node` 에 같은 처리 적용**

`imu_tilt_node.py:65` 도 `default_geometry()` 를 고정으로 들고 `StateEstimator`
(`:127`)에 넘긴다. Step 4 와 **동일한** 구독·핸들러를 추가한다. 코드는 같으므로
그대로 복사하되, `self.geom` / `self.estimator` 속성 이름이 실제 파일과 맞는지
확인하고 조정한다.

- [ ] **Step 6: 테스트 통과 확인**

```bash
cd ros2 && python -m pytest src/powertrain_ros/test/test_state_estimation_geometry_sync.py -q
cd ros2 && colcon test --packages-select powertrain_ros && colcon test-result --verbose
```

- [ ] **Step 7: 커밋**

```bash
git add ros2/src/powertrain_ros/powertrain_ros/state_estimation.py \
        ros2/src/powertrain_ros/powertrain_ros/odometry_node.py \
        ros2/src/powertrain_ros/powertrain_ros/imu_tilt_node.py \
        ros2/src/powertrain_ros/test/test_state_estimation_geometry_sync.py
git commit -m "fix(ros): 조향모드 전환 시 추정 노드 기하 동기화

odometry_node 와 imu_tilt_node 가 default_geometry() 를 고정으로 들고 있어,
스키드 주행 중 조향륜 측면식이 들어가고 앞뒤 x 부호가 반대라 vy 뿐 아니라
요레이트까지 0 으로 눌렸다. /chassis/safety_state 의 steering_mode 를 구독해
기하를 함께 갈아끼운다."
```

---

### Task 10: ops 계약에 조향모드 액션 추가

**Files:**
- Modify: `ros2/src/powertrain_ros/powertrain_ros/ops_contract.py:102-125`
- Test: `ros2/src/powertrain_ros/test/test_ops_contract.py`

**Interfaces:**
- Consumes: Task 8 의 `/chassis_node/steer_mode_skid`
- Produces: `ACTIONS["steer_mode_skid"]` — `ActionSpec(_CONSOLE, "service_setbool", ("/chassis_node/steer_mode_skid",))`

- [ ] **Step 1: 실패하는 테스트 작성**

`ros2/src/powertrain_ros/test/test_ops_contract.py` 끝에 추가:

```python
def test_steer_mode_action_is_console_only_setbool():
    spec = ops_contract.ACTIONS["steer_mode_skid"]

    assert spec.kind == "service_setbool"
    assert spec.target == ("/chassis_node/steer_mode_skid",)
    assert spec.roles == frozenset({ops_contract.ROLE_CONSOLE})


def test_steer_mode_action_is_not_an_emergency_action():
    """비상 2단계 검증 대상이 아니다 — 전환 자체가 정지를 동반한다."""
    assert ops_contract.ACTIONS["steer_mode_skid"].emergency_roles == frozenset()


def test_steer_mode_request_decodes():
    line = json.dumps({
        "schema_version": ops_contract.SCHEMA_VERSION,
        "token": "t", "request_id": "r1", "sequence": 0,
        "action": "steer_mode_skid", "params": {"value": True},
        "stamp_s": 1.0,
    })

    assert ops_contract.decode_request(line)["action"] == "steer_mode_skid"
```

`json` / `ops_contract` import 가 파일에 이미 있는지 확인하고 없으면 더한다.

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd ros2 && python -m pytest src/powertrain_ros/test/test_ops_contract.py -q
```

Expected: `KeyError: 'steer_mode_skid'`

- [ ] **Step 3: 액션 추가**

`ops_contract.py` 의 `"robot_arm_enable"` 항목(`:114-117`) 다음에 추가:

```python
    "steer_mode_skid": ActionSpec(
        _CONSOLE, "service_setbool",
        ("/chassis_node/steer_mode_skid",),
    ),
```

- [ ] **Step 4: 테스트 통과 확인**

```bash
cd ros2 && python -m pytest src/powertrain_ros/test/ -q
```

Expected: 전부 PASS — 특히 `test_ops_contract_chassis_alignment.py`(계약↔노드 정합 검사)가 green 이어야 한다. 실패하면 Task 8 의 서비스 이름이 계약과 다른 것이다.

- [ ] **Step 5: 커밋**

```bash
git add ros2/src/powertrain_ros/powertrain_ros/ops_contract.py \
        ros2/src/powertrain_ros/test/test_ops_contract.py
git commit -m "feat(ops): 조향모드 전환 액션 steer_mode_skid

콘솔 전용 service_setbool — true=스키드, false=애커만. component_enable_* 과
같은 모양이다."
```

---

### Task 11: 콘솔 — 토글 행 · 배지 · 회색 처리

**Files:**
- Modify: `operator_console/ops_panel.py` (`PANEL_ACTIONS` + 헬퍼)
- Modify: `operator_console/status_view.py` (배지 렌더)
- Test: `operator_console/tests/test_ops_panel.py`

**Interfaces:**
- Consumes: Task 8 의 ops state 3필드
- Produces:
  - `steering_mode_from_state(state) -> str | None`
  - `steering_available_from_state(state) -> bool`
  - `drive_transport_from_state(state) -> str | None`
  - `action_is_available(action, state) -> tuple[bool, str]` — 회색 처리 판정과 사유
  - `PANEL_ACTIONS` 에 `steer_mode_skid` 행
  - `status_view.drive_mode_badges(state) -> tuple[str, str]`

- [ ] **Step 1: 실패하는 테스트 작성**

`operator_console/tests/test_ops_panel.py` 끝에 추가:

```python
from operator_console.ops_panel import (
    PANEL_ACTIONS, action_is_available, drive_transport_from_state,
    steering_available_from_state, steering_mode_from_state,
)


def _state(**kw):
    base = {
        "steering_mode": "ackermann",
        "steering_available": True,
        "drive_transport": "can",
        "component_mask": {"drive": True, "steer": True,
                           "us100": True, "robot_arm": True},
    }
    base.update(kw)
    return base


def test_steering_fields_are_read_from_state():
    state = _state(steering_mode="skid", drive_transport="usb",
                   steering_available=False)

    assert steering_mode_from_state(state) == "skid"
    assert steering_available_from_state(state) is False
    assert drive_transport_from_state(state) == "usb"


def test_steering_fields_tolerate_a_missing_state():
    assert steering_mode_from_state(None) is None
    assert steering_available_from_state(None) is False
    assert drive_transport_from_state({}) is None


def test_steer_mode_row_exists_and_is_a_bool_toggle():
    row = next(a for a in PANEL_ACTIONS if a.action == "steer_mode_skid")

    assert row.needs_bool is True
    assert row.bool_value_from_state is not None
    assert row.confirm_text


def test_steer_mode_toggle_requests_the_opposite_mode():
    row = next(a for a in PANEL_ACTIONS if a.action == "steer_mode_skid")

    assert row.bool_value_from_state(_state(steering_mode="ackermann")) is True
    assert row.bool_value_from_state(_state(steering_mode="skid")) is False


def test_steer_mode_is_greyed_out_without_steering_hardware():
    """USB 스택에는 조향 액추에이터가 없어 애커만으로 되돌릴 수 없다."""
    state = _state(steering_mode="skid", steering_available=False,
                   drive_transport="usb")

    available, reason = action_is_available("steer_mode_skid", state)

    assert available is False
    assert "조향" in reason


def test_steer_mode_is_available_on_a_can_stack():
    available, reason = action_is_available("steer_mode_skid", _state())

    assert available is True
    assert reason == ""


def test_other_actions_are_unaffected_by_the_availability_gate():
    for action in ("estop", "arm", "drive_enable"):
        assert action_is_available(action, _state()) == (True, "")


def test_availability_gate_is_conservative_without_state():
    available, _reason = action_is_available("steer_mode_skid", None)

    assert available is False
```

`operator_console/tests/test_status_view.py` 끝에 추가:

```python
from operator_console.status_view import drive_mode_badges


def test_drive_mode_badges_render_both_axes():
    badges = drive_mode_badges({"drive_transport": "usb", "steering_mode": "skid"})

    assert badges == ("구동 USB", "조향 스키드")


def test_drive_mode_badges_render_can_ackermann():
    badges = drive_mode_badges({"drive_transport": "can",
                                "steering_mode": "ackermann"})

    assert badges == ("구동 CAN", "조향 애커만")


def test_drive_mode_badges_mark_unknown_state():
    assert drive_mode_badges(None) == ("구동 —", "조향 —")
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
python3 -m pytest operator_console/tests/test_ops_panel.py operator_console/tests/test_status_view.py -q
```

Expected: `ImportError: cannot import name 'action_is_available'`

- [ ] **Step 3: `ops_panel.py` 헬퍼 추가**

`_component_toggle_value`(`:158-166`) 다음에 추가:

```python
_STEERING_KOREAN = {"ackermann": "애커만", "skid": "스키드"}
_TRANSPORT_KOREAN = {"can": "CAN", "usb": "USB"}


def steering_mode_from_state(state: Mapping[str, Any] | None) -> str | None:
    if state is None:
        return None
    mode = state.get("steering_mode")
    return mode if isinstance(mode, str) and mode else None


def steering_available_from_state(state: Mapping[str, Any] | None) -> bool:
    if state is None:
        return False
    return state.get("steering_available") is True


def drive_transport_from_state(state: Mapping[str, Any] | None) -> str | None:
    if state is None:
        return None
    transport = state.get("drive_transport")
    return transport if isinstance(transport, str) and transport else None


def _steer_mode_toggle_value(state: dict[str, Any]) -> bool:
    """현재가 애커만이면 True(스키드로), 스키드면 False(애커만으로)."""
    mode = steering_mode_from_state(state)
    if mode is None:
        raise RuntimeError("steering mode unavailable")
    return mode != "skid"


def action_is_available(action: str, state: Mapping[str, Any] | None) -> tuple:
    """행을 누를 수 있는지와 회색 사유. 상태를 모르면 보수적으로 막는다.

    USB 스택에는 조향 액추에이터가 아예 없어(AK 는 CAN 전용) 애커만으로 되돌릴
    수 없다. 그 칸은 눌러도 서버가 거부하므로 콘솔에서 먼저 막는다.
    """
    if str(action) != "steer_mode_skid":
        return True, ""
    if state is None or steering_mode_from_state(state) is None:
        return False, "차대 상태 수신 전"
    if steering_mode_from_state(state) == "skid" and \
            not steering_available_from_state(state):
        return False, "조향 모터가 없는 구성입니다 (USB 스택)"
    return True, ""
```

`PANEL_ACTIONS` 의 `robot_arm_enable` 행 다음에 추가:

```python
    PanelAction(
        "steer_mode_skid",
        "조향: 스키드",
        GESTURE_STRIP,
        needs_bool=True,
        confirm_text="조향 방식을 전환합니까? 차대가 멈추고 조향이 0° 로 돌아온 "
                     "뒤에 적용됩니다.",
        bool_value_from_state=_steer_mode_toggle_value,
    ),
```

- [ ] **Step 4: `status_view.py` 배지 추가**

`operator_console/status_view.py` 에 순수 함수를 추가한다 (렌더 함수가 아니라
문자열 생성 — 기존 파일의 순수/렌더 분리 규약을 따른다):

```python
def drive_mode_badges(state):
    """(구동 트랜스포트, 조향 방식) 배지 문자열 2개.

    운전자가 지금 어떤 구성으로 달리고 있는지는 한눈에 보여야 한다 —
    트랜스포트는 재기동해야만 바뀌므로 화면이 유일한 근거다.
    """
    from operator_console.ops_panel import (
        _STEERING_KOREAN, _TRANSPORT_KOREAN,
        drive_transport_from_state, steering_mode_from_state,
    )

    transport = drive_transport_from_state(state)
    steering = steering_mode_from_state(state)
    return (
        "구동 %s" % _TRANSPORT_KOREAN.get(transport, "—"),
        "조향 %s" % _STEERING_KOREAN.get(steering, "—"),
    )
```

배지를 실제로 그리는 곳은 기존 상태 화면 렌더 함수다. `grep -n "def render\|Label(" operator_console/status_view.py` 로 라벨을 붙이는 지점을 찾아 두 문자열을 추가한다. **회색 처리**는 `app.py` 가 `action_is_available()` 을 호출해 버튼 감도를 정하게 배선한다 — `grep -n "PANEL_ACTIONS" operator_console/app.py` 로 행을 만드는 루프를 찾는다.

- [ ] **Step 5: 테스트 통과 확인**

```bash
python3 -m pytest operator_console/tests -q
```

Expected: 전부 PASS (기준선 215 + 신규).

- [ ] **Step 6: 콘솔 실기동 스모크**

```bash
/usr/bin/python3 -m operator_console.runtime_smoke
```

Expected: PASS, traceback 0. **이 스텝을 건너뛰지 않는다** — 스위트 green 상태에서 기동이 3연속 실패한 전례가 있다.

- [ ] **Step 7: 커밋**

```bash
git add operator_console/ops_panel.py operator_console/status_view.py \
        operator_console/app.py operator_console/tests/
git commit -m "feat(console): 조향모드 토글 + 구동/조향 배지 + 회색 처리

component_enable_* 과 같은 모양의 SetBool 토글 행 1개와, 지금 어떤 구성으로
달리는지 보여주는 배지 2개. USB 스택은 조향 액추에이터가 없어 애커만으로
되돌릴 수 없으므로 콘솔에서 먼저 막는다."
```

---

### Task 12: launch 인자 + 트랜스포트 모드 파일

**Files:**
- Modify: `ros2/src/powertrain_ros/launch/control.launch.py`
- Create: `ros2/src/powertrain_ros/powertrain_ros/transport_mode.py`
- Test: `ros2/src/powertrain_ros/test/test_transport_mode.py` (신규)

**Interfaces:**
- Produces: `powertrain_ros.transport_mode.read(path="/etc/powertrain/drive_transport") -> str` — 파일 내용이 `can`/`usb` 가 아니거나 읽을 수 없으면 `"can"`

- [ ] **Step 1: 실패하는 테스트 작성**

`ros2/src/powertrain_ros/test/test_transport_mode.py` 신규:

```python
"""트랜스포트 모드 파일 — 런타임 전환이 불가능해 기동 시 읽는 단일 근거."""
from powertrain_ros import transport_mode


def test_reads_usb(tmp_path):
    path = tmp_path / "drive_transport"
    path.write_text("usb\n", encoding="utf-8")

    assert transport_mode.read(path) == "usb"


def test_reads_can(tmp_path):
    path = tmp_path / "drive_transport"
    path.write_text("can", encoding="utf-8")

    assert transport_mode.read(path) == "can"


def test_ignores_surrounding_whitespace_and_case(tmp_path):
    path = tmp_path / "drive_transport"
    path.write_text("  USB  \n", encoding="utf-8")

    assert transport_mode.read(path) == "usb"


def test_missing_file_falls_back_to_can(tmp_path):
    assert transport_mode.read(tmp_path / "absent") == "can"


def test_unrecognised_value_falls_back_to_can(tmp_path):
    """오타 하나로 조향 없는 스택이 뜨면 안 된다 — 보수적 기본값."""
    path = tmp_path / "drive_transport"
    path.write_text("usbb", encoding="utf-8")

    assert transport_mode.read(path) == "can"


def test_empty_file_falls_back_to_can(tmp_path):
    path = tmp_path / "drive_transport"
    path.write_text("", encoding="utf-8")

    assert transport_mode.read(path) == "can"
```

- [ ] **Step 2: 테스트 실패 확인**

```bash
cd ros2 && python -m pytest src/powertrain_ros/test/test_transport_mode.py -q
```

Expected: `ImportError: cannot import name 'transport_mode'`

- [ ] **Step 3: 모드 파일 리더 구현**

`ros2/src/powertrain_ros/powertrain_ros/transport_mode.py` 신규:

```python
"""구동 트랜스포트 모드 파일.

트랜스포트(CAN↔USB)는 런타임 전환이 불가능하다 — 드라이버 재생성과 전원
사이클마다의 풀캘리(축당 ~55 s)가 필요하고, corners 는 `ChassisManager`
생성자에서 고정된다. 그래서 콘솔은 "다음 기동에 무엇을 쓸지"만 이 파일에 남기고,
`control.launch.py` 가 기동 시 읽는다.

위치는 `/etc/powertrain/ops_*.token` 과 같은 규약이다.

⚠️ 인식할 수 없는 값은 **can 으로 떨어진다.** 오타 하나로 조향 없는 스택이
뜨는 것이 더 위험하기 때문이다.
"""
from pathlib import Path

DEFAULT_PATH = "/etc/powertrain/drive_transport"
VALID = ("can", "usb")
FALLBACK = "can"


def read(path=DEFAULT_PATH) -> str:
    """모드 파일을 읽어 ``"can"`` 또는 ``"usb"`` 를 반환한다."""
    try:
        raw = Path(path).read_text(encoding="utf-8")
    except (OSError, UnicodeError):
        return FALLBACK
    value = raw.strip().lower()
    return value if value in VALID else FALLBACK
```

- [ ] **Step 4: `chassis_node` 를 띄우는 launch 파일에 인자 배선**

⚠️ **`control.launch.py` 는 `teleop_command` 와 `ops_broker` 만 띄운다.**
`chassis_node` 를 띄우는 것은 **`robot_viz.launch.py` 와 `autonomy.launch.py`**
두 개다(2026-08-04 확인). **두 파일 모두** 고쳐야 한다 — 한쪽만 고치면 그 경로로
기동했을 때 조용히 CAN 으로 뜬다.

각 파일의 `DeclareLaunchArgument` 목록에 추가:

```python
        DeclareLaunchArgument(
            "drive_transport",
            default_value=transport_mode.read(),
            description="구동 트랜스포트 can|usb. 기본값은 "
                        "/etc/powertrain/drive_transport 에서 읽는다.",
        ),
        DeclareLaunchArgument(
            "steering_mode",
            default_value="skid" if transport_mode.read() == "usb" else "ackermann",
            description="조향 방식 ackermann|skid. usb 는 skid 만 가능하다.",
        ),
```

파일 상단에 `from powertrain_ros import transport_mode` 를 더한다.

`chassis_node` 를 띄우는 `Node(...)` 의 `parameters=[{...}]` 에 추가:

```python
                "drive_transport": LaunchConfiguration("drive_transport"),
                "steering_mode": LaunchConfiguration("steering_mode"),
```

두 파일에 같은 내용을 넣는다. 확인:

```bash
grep -rn "drive_transport" ros2/src/powertrain_ros/launch/
```

✅ 기대: `robot_viz.launch.py` 와 `autonomy.launch.py` 양쪽에서 각각 2줄 이상.

- [ ] **Step 5: 테스트 통과 확인**

```bash
cd ros2 && python -m pytest src/powertrain_ros/test/test_transport_mode.py -q
cd ros2 && colcon test --packages-select powertrain_ros && colcon test-result --verbose
```

- [ ] **Step 6: 커밋**

```bash
git add ros2/src/powertrain_ros/powertrain_ros/transport_mode.py \
        ros2/src/powertrain_ros/test/test_transport_mode.py \
        ros2/src/powertrain_ros/launch/robot_viz.launch.py \
        ros2/src/powertrain_ros/launch/autonomy.launch.py
git commit -m "feat(ros): 트랜스포트 모드 파일 + launch 인자

트랜스포트는 런타임 전환이 불가능하므로 기동 시 /etc/powertrain/drive_transport
를 읽는다. 인식할 수 없는 값은 can 으로 떨어진다 — 오타 하나로 조향 없는 스택이
뜨는 것이 더 위험하다."
```

---

### Task 13: Phase B 통합 검증 (P4 · P5)

**Files:** 없음 (실행 게이트). 결과는 `docs/reports/2026-08-04-usb-skid-bench.md` 에 이어 쓴다.

- [ ] **Step 1: P5 — 콘솔 실기동**

```bash
/usr/bin/python3 -m operator_console.runtime_smoke
```

✅ traceback 0. 이어서 실제 콘솔을 젯슨에 붙여 띄우고:
- 배지 2개가 현재 구성을 맞게 보이는가 (`구동 CAN` / `조향 애커만`)
- 조향모드 토글 행이 보이고 확인 흐름이 도는가
- USB 스택으로 기동했을 때 그 행이 회색이고 사유가 뜨는가

- [ ] **Step 2: 게이트 자체를 음성 대조로 증명**

`chassis_manager._tick_steering_mode` 의 수렴 대기를 **일부러 제거**한
브랜치를 만들어 P4 가 FAIL 하는지 확인한다. FAIL 하지 않으면 게이트가 아무것도
검증하지 않는 것이다. 확인 후 되돌린다.

- [ ] **Step 3: P4 — 조향모드 전환 E2E (바퀴 들린 상태, CAN 스택)**

사용자에게 바퀴가 들려 있는지 물리 확인을 요청한 뒤:

1. CAN 스택으로 기동해 애커만으로 arm
2. 좌회전을 걸어 조향을 45° 로 꺾는다
3. 콘솔에서 조향모드 토글 → **스키드**
4. 확인 항목:
   - 구동이 즉시 0 이 되는가
   - 조향이 0° 로 돌아오는가
   - **조향이 0° 에 닿기 전에는 차동이 걸리지 않는가** (텔레메트리에서 좌우
     구동 지령이 0 인지 확인)
   - 0° 도달 후 배지가 `조향 스키드` 로 바뀌는가
   - 이제 좌스틱으로 좌우 차동이 걸리는가
5. 되돌리기: 토글 → **애커만**. 배지·동작이 되돌아오는가

- [ ] **Step 4: P4b — 요레이트 추정 확인 (결함 B 회귀)**

스키드로 제자리 선회를 시키고 `/odom` 의 `twist.twist.angular.z` 가 0 이
아닌지 확인한다.

```bash
ros2 topic echo /odom --field twist.twist.angular.z
```

✅ 명령 ω 와 같은 부호로 0 이 아닌 값이 나와야 한다. 0 이면 Task 9 가 실제로는
안 붙은 것이다.

- [ ] **Step 5: USB 스택으로 전환 검증**

```bash
echo usb | sudo tee /etc/powertrain/drive_transport
sudo systemctl restart powertrain_control
systemctl is-active powertrain_control
journalctl -u powertrain_control -n 50 --no-pager
```

✅ `active`, 크래시루프 없음. 콘솔 배지가 `구동 USB` / `조향 스키드`, 조향모드
행이 회색.

⚠️ 배포 후 journal 확인을 건너뛰지 않는다 — 크래시루프는 조용히 돈다(07-18
sender 5,586회 사고).

- [ ] **Step 6: 전체 스위트 최종 확인**

```bash
cd motor_control && python -m pytest chassis/tests/ corner_module/tests/ -q
python3 -m pytest operator_console/tests -q
cd ros2 && colcon test --packages-select powertrain_ros && colcon test-result --verbose
/usr/bin/python3 -m operator_console.runtime_smoke
```

- [ ] **Step 7: 결과 기록 + 커밋**

```bash
git add docs/reports/2026-08-04-usb-skid-bench.md
git commit -m "test: 조향모드 전환 E2E(P4) + 콘솔 실기동(P5) 검증 기록"
```

---

## Self-Review 결과

**스펙 커버리지**

| 스펙 절 | 태스크 |
|---|---|
| §3 `skid_geometry()` · `track_gain` | 1 |
| §4 USB 다보드 드라이버 · 지연 예산 | 3, 6(P0) |
| §5 빌더 · 보드 레지스트리 | 4 |
| §6.1 결함 A (vy prior) | 2 |
| §6.2 결함 B (노드 간 기하) | 9, 13(P4b) |
| §7 조향모드 런타임 전환 | 7, 8, 10 |
| §8 트랜스포트 기동 시 선택 | 12, 13(Step 5) |
| §9 GUI | 11 |
| §10 진입점 | 5, 8 |
| §11 테스트 | 각 태스크 Step 1 |
| §12 검증 게이트 P0~P5 | 6, 13 |

**미커버 항목 (의도된 범위 밖, 스펙 §13 과 일치)**: `track_gain` 실측값 확정(P3 — 차체 조립 후), 콘솔발 스택 재기동, 구동 USB + 조향 CAN 혼합.

**타입 일관성 확인**
- `skid_geometry(track_gain, base)` — Task 1 정의, Task 5·7·9 에서 동일 시그니처로 호출
- `build_usb_skid_corners(registry_path, cfg, wheel_map, gear_ratio, current_lim_a, stale_ms, pool)` — Task 4 정의, Task 5·8 에서 키워드로 호출
- `DriveOdriveUsbAxis(pool, serial, axis_index, *, node_id, gear_ratio, invert, current_lim_a, stale_ms, poll_slot, poll_period_ticks, clock)` — Task 3 정의, Task 4 에서 전부 키워드로 전달
- `request_steering_mode(mode) -> (bool, str)` — Task 7 정의, Task 8 이 그대로 사용
- `steering_mode` / `steering_available` / `drive_transport` — Task 8 이 발행, Task 11 이 소비

**계획 작성 중 실물 확인한 것 (2026-08-04)**

| 확인 | 결과 |
|---|---|
| `drive.bl70200.board_registry` import | PEP 420 네임스페이스 패키지로 **동작함** (`__init__.py` 불필요) |
| `test_chassis_manager.py` 기존 헬퍼 | `_fake_corners(cfg)`·`_armed_manager(cfg, clock)`·`FakeClock` 존재 → **재정의 금지**, Task 7 테스트가 이를 재사용하도록 작성됨 |
| `chassis_node` 를 띄우는 launch | `control.launch.py` 가 아니라 **`robot_viz.launch.py` + `autonomy.launch.py`** → Task 12 가 두 파일 모두 수정 |
| `/chassis/safety_state` 토픽 | `chassis_node.py:666-670`, `component_mask` 가 이미 같은 방식으로 실려 나감 |
| `authority_clear_hold` | `self._authority.clear_hold()` 만 호출 — 인터록 motion hold 와 무관 → 스펙 §7.1 정정 근거 (Task 0) |

**남은 확인 항목 (해당 태스크 Step 1 에서 grep 으로 맞출 것)**

- Task 11: `operator_console/app.py` 의 `PANEL_ACTIONS` 렌더 루프 위치와 버튼 감도 설정 지점, `status_view.py` 의 라벨 추가 지점
- Task 8: `chassis_node` 의 `declare_parameter` 가 모여 있는 정확한 블록, `gear_ratio` 지역변수 이름
- Task 9: `imu_tilt_node` 의 `self.geom` / `self.estimator` 실제 속성 이름
