# 시뮬 depth 레이캐스트 컷오프 수정 (6 m 벽 해소) Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** powertrain_sim depth 렌더러의 `mj_multiRay` 앵커-거리 프루닝 결함(6 m 벽)을 컷오프 비프루닝화로 고치고, 음성 대조·전 가족 캠페인으로 안전 불변식을 증명한 뒤 영향 앵커를 정직 재기준선한다.

**Architecture:** production 추정기(`powertrain_autonomy`)는 **한 줄도 바꾸지 않는다**. 수정은 `powertrain_sim/mujoco_fast/sensors.py` 의 mj_multiRay 컷오프 한 곳. 검증은 (1) mj_ray ground-truth 등가성 테스트(결함 재현 = RED→GREEN), (2) MuJoCo 프루닝 quirk 핀 테스트, (3) 전 가족 dev 캠페인 fail_open 0 · edge_overrun 0, (4) false_hold 과도 에피소드 특성화 후 유계 확인 시에만 expected_metrics 재핀.

**Tech Stack:** Python(conda base `/home/light/anaconda3/bin/python`), MuJoCo 3.10.0, pytest. 모든 명령 앞에 `PYTHONPATH=motor_control:ros2/src/powertrain_ros:.`, 실행 위치 = 레포 루트.

**Spec:** `docs/superpowers/specs/2026-07-21-sim-depth-floor-pruning-6m-wall-design.md`

## Global Constraints

- **절대 게이트: 전 가족 캠페인 `fail_open_count == 0` AND `edge_overrun_count == 0`.** 하나라도 깨지면 즉시 되돌리고 중단 보고. 예외 없음 (스펙 §4 V4).
- `powertrain_autonomy/` 아래 파일은 수정 금지 (스펙 §3 기각 대안 D).
- 센서 사거리 의미 불변: depth 프레임에 6.0 m 초과 유효값 금지 (스펙 §4 V2).
- 재기준선은 실측값 + 날짜 주석으로만 (기존 앵커 주석 패턴 유지). 특성화 결정 규칙(Task 3)을 통과하지 못한 재핀 금지.
- 커밋 메시지 끝: `Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>`.

## 사전 실측 (참고 — Task 2 에서 재측정해 그 값을 쓴다)

모끼패치 프로브(수정과 동일 코드 경로), 정본 dev 문서:

| family | completion | fail_open | edge_overrun | false_hold |
|---|---|---|---|---|
| flat | 0.9418 | 0 | 0 | 13 |
| bank | 0.9429 | 0 | 0 | 15 |
| pinch | 0.5075 | 0 | 0 | 0 |
| clothoid | 0.9527 | 0 | 0 | 13 |
| undulating | 0.1528 | 0 | 0 | 0 |
| friction | 0.9609 | 0 | 0 | 10 |
| smog | 0.9384 | 0 | 0 | 22 |

수정 전 flat = 0.396 (5.9 m 벽). undulating 0.15 는 벽과 무관한 별건(계획 A Task 5 재료)이므로 이 계획에서 고치려 하지 말 것.

---

### Task 1: 결함 재현·등가성 테스트 + 센서 수정

**Files:**
- Create: `powertrain_sim/tests/test_depth_ray_cutoff.py`
- Modify: `powertrain_sim/mujoco_fast/sensors.py` (`MAX_VALID_DEPTH_M` 정의부 ~line 18, `_ray_depth_m` 의 `mj_multiRay` 호출 ~line 155–168)

**Interfaces:**
- Produces: `powertrain_sim.mujoco_fast.sensors.RAY_PRUNING_CUTOFF_M: float = 1.0e6` (모듈 상수, `__all__` 에 추가하지 않음), 기존 `MAX_VALID_DEPTH_M = 6.0` 의미 불변.

- [ ] **Step 1: 실패하는 테스트 작성**

`powertrain_sim/tests/test_depth_ray_cutoff.py` 전체 내용:

```python
"""Depth raycast cutoff regression: mj_multiRay must not prune the floor plane.

6 m 벽 근본 원인 회귀 핀 — 스펙 docs/superpowers/specs/
2026-07-21-sim-depth-floor-pruning-6m-wall-design.md §2/§4.
"""
from __future__ import annotations

import mujoco
import numpy as np
import pytest

from powertrain_sim.campaign import build_family_document
from powertrain_sim.mujoco_fast.plant import MujocoFastPlant
from powertrain_sim.mujoco_fast.sensors import FastSensorSuite, MAX_VALID_DEPTH_M
from powertrain_sim.scenario import parse_scenario


GEOM_GROUP = np.array((1, 0, 0, 0, 0, 0), dtype=np.uint8)


def _flat_suite_at(x_m: float) -> tuple[MujocoFastPlant, FastSensorSuite]:
    """Flat dev scenario, depth noise off, base teleported to x_m (no stepping)."""
    document = build_family_document("flat", seed=0, seed_class="dev")
    document["sensors"]["depth"]["noise_std_m"] = 0.0
    scenario = parse_scenario(document)
    plant = MujocoFastPlant(scenario)
    suite = FastSensorSuite(scenario, plant)
    model, data = plant.model, plant.data
    qpos_address = model.jnt_qposadr[model.body_jntadr[plant.base_body_id]]
    data.qpos[qpos_address] = x_m
    mujoco.mj_forward(model, data)
    return plant, suite


def _ground_truth_axial(
    plant: MujocoFastPlant, suite: FastSensorSuite, flat_index: int
) -> float:
    """Single-ray mj_ray ground truth with the sensor's own range mask."""
    body_matrix = np.asarray(
        plant.data.xmat[plant.base_body_id], dtype=float
    ).reshape(3, 3)
    origin = np.asarray(plant.data.site_xpos[plant.depth_site_id], dtype=float)
    direction = suite._depth_directions_body[flat_index] @ body_matrix.T
    geom_id = np.zeros(1, dtype=np.int32)
    distance = mujoco.mj_ray(
        plant.model,
        plant.data,
        origin,
        np.ascontiguousarray(direction),
        GEOM_GROUP,
        True,
        plant.base_body_id,
        geom_id,
    )
    if distance < 0.0 or distance > MAX_VALID_DEPTH_M:
        return 0.0
    return float(distance * suite._depth_axial_cos[flat_index])


def _frame_vs_ground_truth(x_m: float) -> tuple[np.ndarray, np.ndarray]:
    plant, suite = _flat_suite_at(x_m)
    frame = suite.sample_depth(0)
    assert frame is not None
    height, width = frame.depth_roi.shape
    sensor = frame.depth_roi.astype(float) * frame.depth_scale_m
    sampled_sensor = []
    sampled_truth = []
    for row in range(0, height, 8):
        for col in range(0, width, 8):
            sampled_sensor.append(sensor[row, col])
            sampled_truth.append(
                _ground_truth_axial(plant, suite, row * width + col)
            )
    return np.asarray(sampled_sensor), np.asarray(sampled_truth)


def test_depth_matches_single_ray_ground_truth_far_from_origin():
    """카메라가 월드 원점에서 6 m 를 넘어도 depth 는 mj_ray 와 등가여야 한다.

    수정 전에는 mj_multiRay 앵커-거리 프루닝이 lower_floor plane 을 통째로
    제외해 측면 바닥 픽셀이 전부 0 이 된다 (6 m 벽의 근본 원인)."""
    sensor, truth = _frame_vs_ground_truth(x_m=8.0)
    floor_visible = truth > 0.0
    assert np.count_nonzero(floor_visible) > 0
    np.testing.assert_allclose(sensor, truth, atol=2.0e-3)


def test_depth_matches_single_ray_ground_truth_at_spawn():
    """벽 이전 포즈(스폰)에서의 등가성 — 수정이 기존 유효 depth 를 바꾸지 않음."""
    sensor, truth = _frame_vs_ground_truth(x_m=0.0)
    np.testing.assert_allclose(sensor, truth, atol=2.0e-3)


@pytest.mark.parametrize("x_m", (0.0, 8.0))
def test_no_depth_beyond_max_valid_range(x_m):
    """수정이 센서 사거리를 확장하면 안 된다 (스펙 §4 V2)."""
    _, suite = _flat_suite_at(x_m)
    frame = suite.sample_depth(0)
    assert frame is not None
    depth_m = frame.depth_roi.astype(float) * frame.depth_scale_m
    assert float(depth_m.max()) <= MAX_VALID_DEPTH_M + 2.0e-3


def test_mujoco_multiray_anchor_pruning_quirk_is_pinned():
    """MuJoCo 3.10.0 quirk 핀: cutoff 프루닝은 geom 앵커점 거리 기준이라,
    원점-앵커 plane 은 원점에서 cutoff 밖 origin 의 컷오프-이내 히트를 잃는다.

    이 테스트가 실패하면 업스트림 의미론이 바뀐 것 — RAY_PRUNING_CUTOFF_M
    상수의 필요성을 재검토하고 스펙 §2 를 갱신할 것."""
    xml = """
    <mujoco><worldbody>
      <geom name="floor" type="plane" pos="0 0 0" size="50 50 0.05" group="0"/>
      <body name="base" pos="0 0 10"><freejoint/>
        <geom type="sphere" size="0.05" group="1" mass="1"/></body>
    </worldbody></mujoco>"""
    model = mujoco.MjModel.from_xml_string(xml)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    origin = np.array((8.0, 0.0, 1.0))
    down = np.array((0.0, 0.0, -1.0))
    geom_id = np.full(1, -1, dtype=np.int32)
    distance = np.full(1, -1.0)
    mujoco.mj_multiRay(
        model, data, origin, np.ascontiguousarray(down).ravel(), GEOM_GROUP,
        True, model.body("base").id, geom_id, distance, None, 1, 6.0,
    )
    single_geom = np.zeros(1, dtype=np.int32)
    single_distance = mujoco.mj_ray(
        model, data, origin, down, GEOM_GROUP, True,
        model.body("base").id, single_geom,
    )
    assert single_distance == pytest.approx(1.0, abs=1e-9)
    assert geom_id[0] == -1  # the quirk: in-cutoff hit dropped by multiRay
```

- [ ] **Step 2: 테스트가 의도대로 실패하는지 확인 (RED)**

Run: `PYTHONPATH=motor_control:ros2/src/powertrain_ros:. /home/light/anaconda3/bin/python -m pytest powertrain_sim/tests/test_depth_ray_cutoff.py -v`

Expected: `test_depth_matches_single_ray_ground_truth_far_from_origin` **FAIL** (바닥 픽셀 sensor=0 vs truth>0 불일치 다수). 나머지 3개 PASS. far 테스트가 PASS 로 나오면 결함 재현 실패 — 진행 중단, 원인 조사.

- [ ] **Step 3: 센서 수정 (GREEN 최소 구현)**

`powertrain_sim/mujoco_fast/sensors.py` 의 `MAX_VALID_DEPTH_M = 6.0` 바로 아래에 추가:

```python
# MuJoCo mj_multiRay 는 cutoff 로 geom 을 "앵커점(geom pos)까지의 거리" 기준
# 프루닝한다(3.10.0 실측). 무한 plane 인 lower_floor 는 앵커가 월드 원점이라,
# 카메라가 원점에서 cutoff 이상 멀어지는 순간 컷오프-이내 바닥 히트까지 통째로
# 사라진다 — 훈련 트랙 ~6 m 지점 전 가족 영구 정지(6 m 벽)의 근본 원인.
# 컷오프는 프루닝이 일어나지 않는 값으로 두고, 센서 사거리는 _ray_depth_m 의
# hit 마스크(MAX_VALID_DEPTH_M)만이 정의한다. 스펙:
# docs/superpowers/specs/2026-07-21-sim-depth-floor-pruning-6m-wall-design.md
RAY_PRUNING_CUTOFF_M = 1.0e6
```

`_ray_depth_m` 의 `mj_multiRay(...)` 마지막 인자 `MAX_VALID_DEPTH_M` → `RAY_PRUNING_CUTOFF_M` 로 교체. `hit = (distances >= 0.0) & (distances <= MAX_VALID_DEPTH_M)` 및 이하 로직은 그대로.

- [ ] **Step 4: 테스트 통과 확인 (GREEN)**

Run: `PYTHONPATH=motor_control:ros2/src/powertrain_ros:. /home/light/anaconda3/bin/python -m pytest powertrain_sim/tests/test_depth_ray_cutoff.py -v`

Expected: 5개 전부 PASS (parametrize 포함).

- [ ] **Step 5: 커밋**

```bash
git add powertrain_sim/mujoco_fast/sensors.py powertrain_sim/tests/test_depth_ray_cutoff.py
git commit -m "fix(sim): decouple depth ray cutoff from sensor range — 6 m wall root fix

mj_multiRay prunes geoms by anchor-point distance, dropping the infinite
lower_floor plane entirely once the camera passes 6 m Euclidean from the
world origin; the side-floor band vanished from depth and the production
estimator correctly fail-closed forever. Cast with a non-pruning cutoff
and keep MAX_VALID_DEPTH_M as the only range authority. Pins the MuJoCo
quirk and mj_ray ground-truth equivalence at near and far poses.

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 2: false_hold 과도 에피소드 특성화 (재기준선 결정 자료)

**Files:**
- Create: 스크래치 전용 `/tmp/claude-1000/-home-light-ZETIN-robotics-power-train-sw/4f54ae08-d399-4211-a582-605c43194618/scratchpad/hold_char.py` (레포에 커밋하지 않음)
- 산출: 가족별 표(아래 Step 3 형식)를 Task 3·최종 보고에 전달

**Interfaces:**
- Consumes: Task 1 커밋 후의 `run_closed_loop` (수정된 센서 경로)
- Produces: 가족별 `{completion, fail_open, edge_overrun, false_hold, max_recovery_s, 에피소드 duration 목록}` 표 + 결정 규칙 판정

- [ ] **Step 1: 특성화 프로브 작성**

```python
"""Post-fix hold-episode characterization for honest re-baselining."""
from __future__ import annotations

import json
import sys
from pathlib import Path

from powertrain_sim.campaign import FAMILY_DEV_SEEDS, build_family_document
from powertrain_sim.closed_loop import TerrainAutonomyDriver
from powertrain_sim.mujoco_fast.runner import run_scenario
from powertrain_sim.scenario import parse_scenario

OUT = Path(sys.argv[1])
OUT.mkdir(parents=True, exist_ok=True)
FAMILIES = ("flat", "bank", "pinch", "clothoid", "undulating", "friction", "smog")

for family in FAMILIES:
    seed = FAMILY_DEV_SEEDS.get(family, 0)
    document = build_family_document(family, seed=seed, seed_class="dev")
    scenario = parse_scenario(document)
    driver = TerrainAutonomyDriver(scenario)
    timeline = []

    def observe(elapsed_s, snapshot):
        actual, should = driver.hold_state(elapsed_s, snapshot)
        x = snapshot.pose.x_m if snapshot is not None else None
        timeline.append((elapsed_s, x, actual, should))
        return actual, should

    run_dir = OUT / family
    run_dir.mkdir(exist_ok=True)
    report = run_scenario(
        scenario, run_dir,
        command_source=driver.command,
        hold_state_source=observe,
        depth_tap=driver.on_depth,
        geometry=driver.geometry,
    )
    episodes = []
    start = None
    for elapsed_s, x, actual, should in timeline:
        false_hold = actual and not should
        if false_hold and start is None:
            start = (elapsed_s, x)
        elif not false_hold and start is not None:
            episodes.append(
                {"start_s": start[0], "x_m": start[1],
                 "duration_s": round(elapsed_s - start[0], 3)}
            )
            start = None
    if start is not None:
        episodes.append({"start_s": start[0], "x_m": start[1],
                         "duration_s": "OPEN-ENDED"})
    row = {
        "family": family,
        "completion": report.completion_ratio,
        "fail_open": report.fail_open_count,
        "edge_overrun": report.edge_overrun_count,
        "false_hold": report.false_hold_count,
        "max_recovery_s": report.max_recovery_time_s,
        "episodes": episodes,
    }
    print(json.dumps(row))
    (OUT / f"{family}.json").write_text(json.dumps(row, indent=1))
```

- [ ] **Step 2: 실행**

Run: `PYTHONPATH=motor_control:ros2/src/powertrain_ros:. /home/light/anaconda3/bin/python <scratchpad>/hold_char.py <scratchpad>/holdchar`

Expected: 가족별 JSON 라인. fail_open·edge_overrun 전부 0 (아니면 **즉시 중단·되돌림·보고**).

- [ ] **Step 3: 결정 규칙 적용**

- **모든** false-hold 에피소드 `duration_s <= 1.0` **AND** `"OPEN-ENDED"` 없음 **AND** 가족별 `max_recovery_s <= 0.5` → **유계 과도 판정**: Task 3 재핀 진행. 측정 표를 보고에 기록.
- 규칙 위반(장기/개방 에피소드) 발견 → **재핀 금지**, Task 3 건너뛰고 발견 내용을 최종 보고에 기술(수정 자체는 유지 — 절대 게이트는 이미 통과).

---

### Task 3: 정직 재기준선 (조건부 — Task 2 유계 판정 시에만)

**Files:**
- Modify: `powertrain_sim/family_scenarios.py` (`_terrain_document` ~line 63 및 clothoid·smog 빌더의 동일 지점 — `document["clock"]["duration_s"]` 를 만지는 자리)
- Modify: `powertrain_sim/tests/test_closed_loop.py:109-114` (flat dev 앵커 주석+값)

**Interfaces:**
- Consumes: Task 2 실측 표 (아래 값은 사전 실측 — Task 2 값으로 대체)
- Produces: dev 훈련 가족 문서의 `expected_metrics` 재핀 (`false_hold_count`, `max_recovery_time_s`)

- [ ] **Step 1: 가족 문서 expected_metrics 재핀**

`_terrain_document` 의 `document["clock"]["duration_s"] = TRAINING_DURATION_S` 아래에 추가 (clothoid·smog 빌더도 각자의 duration 설정 지점에 동일 패턴):

```python
    # 6 m 벽(시뮬 depth 렌더러 mj_multiRay plane 프루닝) 수정 후 정직 재기준선
    # (2026-07-21). 15 m 기복 트랙에서 크레스트 통과마다 짧은 fail-closed 과도
    # hold(전부 <=1.0 s, 회복 <=0.5 s 실측)가 생긴다 — Task 2 특성화 표:
    # flat 13 / bank 15 / clothoid 13 / friction 10 / smog 22 (pinch·undulating 0).
    # 상한은 실측 x1.5 여유. fail_open/edge_overrun 0 은 절대 불변.
    document["expected_metrics"]["false_hold_count"] = FAMILY_FALSE_HOLD_BOUND[family]
    document["expected_metrics"]["max_recovery_time_s"] = 0.5
```

파일 상단 상수(값은 Task 2 실측 ×1.5 올림으로 교체):

```python
# 가족별 false-hold 상한 = Task 2 실측 x1.5 올림 (2026-07-21).
FAMILY_FALSE_HOLD_BOUND = {
    "flat": 20, "bank": 23, "clothoid": 20, "friction": 15, "smog": 33,
    "pinch": 0, "undulating": 0,
}
```

clothoid/smog 빌더가 `_terrain_document` 를 쓰지 않으면 각 빌더에서 같은 두 줄을 문서에 직접 설정한다(family 키는 해당 이름). `pinch_document` 는 실측 0 이므로 손대지 않는다.

- [ ] **Step 2: flat dev 앵커 재핀**

`powertrain_sim/tests/test_closed_loop.py` 의 `assert first.completion_ratio > 0.70` 을:

```python
    # 6 m 벽(시뮬 depth 렌더러 결함) 수정 후 재기준선 (2026-07-21): 15 m 기복
    # 트랙 dev seed 실측 0.9418. 이전 0.709 앵커는 2.5 m 트랙 시절 값.
    # Change only after reviewing a dev seed run.
    assert first.completion_ratio > 0.90
```

(기존 109–113 주석 블록은 위 내용으로 대체.)

- [ ] **Step 3: sim 스위트 실행·기준선 대조**

Run: `PYTHONPATH=motor_control:ros2/src/powertrain_ros:. /home/light/anaconda3/bin/python -m pytest powertrain_sim/tests -q`

Expected: 벽 증상이던 `test_too_narrow_pinch_stops_before_the_drop_boundary`, `test_clothoid_closed_loop_stays_bounded_without_fail_open` **green 전환**. Task 1 신규 5개 green. 사전 문서화된 무관 실패(follow 간격 0.9 mm, three_percent, canonical_json_hash, l515_wide_fov — `docs/reports/2026-07-21-sim-fidelity-completion-progress.md` §4)만 남을 것. **새 red 가 나타나면 원인 규명 전 커밋 금지.** (undulating 테스트의 baseline 상태는 Task 착수 시점의 기준선 기록과 대조 — 완주 0.15 는 벽과 무관한 기지 이슈.)

- [ ] **Step 4: 커밋**

```bash
git add powertrain_sim/family_scenarios.py powertrain_sim/tests/test_closed_loop.py
git commit -m "test(sim): honest re-baseline after 6 m wall fix — transient hold bounds and flat anchor

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

### Task 4: 전 가족 dev 캠페인 절대 게이트 검증 (정본 CLI)

**Files:** 산출물만 (`/tmp` 캠페인 디렉토리, 커밋 없음)

- [ ] **Step 1: 정본 캠페인 실행**

Run: `PYTHONPATH=motor_control:ros2/src/powertrain_ros:. /home/light/anaconda3/bin/python -m powertrain_sim.campaign <scratchpad>/campaign_postfix`

Expected: 표 출력. exit code 는 passed 여부 — Task 3 재핀이 반영됐으면 0 기대.

- [ ] **Step 2: 절대 게이트 기계 검증 (fail_open + edge_overrun)**

```bash
/home/light/anaconda3/bin/python - <<'EOF'
import json, sys
from pathlib import Path
base = Path("<scratchpad>/campaign_postfix")
report = json.loads((base / "campaign.json").read_text())
bad = [r for r in report["results"] if r["fail_open"] != 0]
overruns = []
for metrics_path in base.glob("runs/*/seed-*/metrics.json"):
    metrics = json.loads(metrics_path.read_text())
    if metrics["edge_overrun_count"] != 0:
        overruns.append(str(metrics_path))
print("fail_open violations:", bad)
print("edge_overrun violations:", overruns)
sys.exit(1 if (bad or overruns) else 0)
EOF
```

Expected: 두 목록 모두 비어 있고 exit 0. **위반 시 즉시 전체 되돌림(Task 1·3 커밋 revert)·중단 보고.**

- [ ] **Step 3: 결과 수치 기록**

campaign.json 의 family/completion/fail_open/recovery 표를 최종 보고서(Task 5)에 옮긴다.

---

### Task 5: 자율 코어 회귀 + 문서·보고

**Files:**
- Modify: `docs/reports/2026-07-21-sim-fidelity-completion-progress.md` (§3-B 상단에 supersession 콜아웃 1개)
- Create: `docs/reports/2026-07-21-6m-wall-rootcause-and-fix.md`

- [ ] **Step 1: powertrain_autonomy 회귀 (무변경 확인)**

Run: `PYTHONPATH=motor_control:ros2/src/powertrain_ros:. /home/light/anaconda3/bin/python -m pytest powertrain_autonomy -q`

Expected: `134 passed`. (이 계획은 해당 트리를 건드리지 않으므로 실패 시 환경 문제 — diff 확인.)

- [ ] **Step 2: 기존 보고서에 supersession 표기**

`docs/reports/2026-07-21-sim-fidelity-completion-progress.md` §3-B 제목 바로 아래 추가:

```markdown
> ⛔ **2026-07-21 후속 규명으로 대체됨**: 여기의 "품질 타일 게이팅 / 단일프레임
> 사거리" 진단은 반증됐다. 6 m 벽의 근본 원인은 시뮬 depth 렌더러의 MuJoCo
> `mj_multiRay` plane 앵커-거리 프루닝이다(실차 무관, production 추정기 무결).
> 정본: `docs/superpowers/specs/2026-07-21-sim-depth-floor-pruning-6m-wall-design.md`,
> 결과: `docs/reports/2026-07-21-6m-wall-rootcause-and-fix.md`.
```

- [ ] **Step 3: 결과 보고서 작성**

`docs/reports/2026-07-21-6m-wall-rootcause-and-fix.md`: 스펙 §1–§2 요약(반증 실측 3개 + 최소 재현), 수정 내용 1문단, Task 2 특성화 표, Task 4 캠페인 표(수정 전 0.396 대비), 안전 게이트 증빙(V1–V5 각 어디서 증명됐는지), 남은 것(undulating 0.15 별건, Task 5 완주율 기준선 재개, 무관 실패 4건 재핀 대기, MuJoCo 업스트림 리포트 선택).

- [ ] **Step 4: 커밋**

```bash
git add docs/reports/2026-07-21-sim-fidelity-completion-progress.md docs/reports/2026-07-21-6m-wall-rootcause-and-fix.md
git commit -m "docs(reports): 6 m wall root cause resolution — sim renderer fix verified across families

Co-Authored-By: Claude Fable 5 <noreply@anthropic.com>"
```

---

## Self-Review 결과

- 스펙 §3(수정) → Task 1, §4 V1·V2·V3 → Task 1 테스트 4종, V4 → Task 4, V5 → Task 3 Step 3 + Task 5 Step 1. §5 비목표(추정기 무변경) → Global Constraints. 누락 없음.
- 앵커 수치는 전부 실측 기반이며 Task 2 재측정으로 대체하도록 명시.
- `RAY_PRUNING_CUTOFF_M` 이름은 Task 1 Step 3 과 테스트 주석에서 일관.
