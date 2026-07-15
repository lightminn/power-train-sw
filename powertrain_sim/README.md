# WP6-S simulator-neutral fixtures and replay

`powertrain_sim` implements the simulator-independent part of WP6-S P0. It has
no ROS or simulator runtime dependency. MuJoCo and ROS adapters must consume
these value contracts; production packages must not gain simulator-name
branches.

## Scenario contract

One `scenario.yaml` is the sole owner of every input that may change a run:

- `schema_version`: currently exactly `1`.
- `units`: exact SI spellings for distance, angle, time, velocity,
  acceleration, curvature and friction. L515-compatible depth samples use
  millimetres and declare their metre scale separately.
- `frames`: world, body, depth optical and IMU frame names.
- `clock`: positive `start_s`, `dt_s` and `duration_s`. Duration must be an
  integer multiple of `dt_s`; generated stamps are derived only from this
  clock.
- `prng`: exactly `PCG64`, an integer seed and one of `dev`, `regression`,
  `hidden_evaluation` or `stress`.
- `track`: aligned station arrays for the 3D centreline, width, elevated
  surface height, bank, curvature, friction and left/right drop boundaries.
  `height_m[i]` must equal `centerline_m[i][2]`, preventing two competing
  elevation definitions.
- `motion`, `sensors` and `faults`: analytic motion profile, wheel/IMU/depth
  sampling and noise, ordered wheel identifiers, and
  slip/dropout/hole/spike injection.
- `expected_metrics`: completion, minimum clearance, edge overrun count,
  false-hold and fail-open counts, maximum recovery time and estimator runtime.

Unknown fields/PRNG algorithms, unclear unit spellings, missing required keys,
non-finite numbers, unknown fault wheels and out-of-ROI depth faults fail
validation. Pivot target, rate sign and available duration are checked
together. `hidden_evaluation` seeds are completion
evidence only: do not inspect them or tune algorithms/parameters against them.
Tune with `dev` and `regression`; use `stress` for declared degradation cases.

Representative contracts live in `scenarios/`:

- `flat_straight_5m.yaml`: constant-speed distance regression.
- `pivot_90deg.yaml`: counter-clockwise yaw regression.
- `bank_transition.yaml`: acceleration/deceleration, bank, slip, IMU dropout,
  depth hole and reflection-spike injection.

## Analytic fixtures

```python
from powertrain_sim.fixtures import generate_fixture
from powertrain_sim.scenario import load_scenario

scenario = load_scenario("powertrain_sim/scenarios/flat_straight_5m.yaml")
fixture = generate_fixture(scenario)
```

The generator instantiates `numpy.random.Generator(PCG64(seed))` directly.
For the same scenario and dependency versions it produces byte-identical
wheel, IMU and depth arrays. It returns production-core `WheelSample` and
`ImuSample` values, Task 4-compatible `DepthFrame` values, and a separate
ground-truth sequence. Depth frames provide the exact arguments required by
`analyze_depth_quality`: `depth_roi`, `depth_scale_m`, `intrinsics` and
`stamp_s`.

Motion advances an arc-length station along the declared 3D centreline;
ground-truth position, height, bank and heading come from that spatial station,
not elapsed-time percentage. Pivot motion clamps at `target_yaw_rad` and emits
zero yaw rate for any declared time remaining after the target.

No wall clock is read. Every timestamp is `clock.start_s + index * clock.dt_s`.

## Recorded run format and replay

```text
run-directory/
  wheel.jsonl
  imu.jsonl
  depth.jsonl
  detections.jsonl
  ground_truth.jsonl       # isolated /sim/* equivalent
  depth/
    000000000123.npz       # one lossless ROI array per depth record
```

`RunWriter` accepts only value objects and primitive mappings, never ROS
messages. JSONL uses `powertrain_observability.events.encode_event`; reading
uses the journal's `recover_records`, so an incomplete final line is ignored
with the same recovery rule as the mission journal. A depth NPZ is flushed and
atomically renamed before its complete JSONL metadata record is appended. An
interruption can therefore leave only an unreferenced NPZ, never a complete
record pointing at a partial file.

The event envelope's `monotonic_ns` is derived from the sample stamp and
`wall_time_ns` is `0`; replay does not synthesize a host clock. A global writer
sequence preserves arrival order when multiple streams have the same stamp.
`RecordedRun.iter_records()` sorts by stamp then sequence and yields only
wheel, IMU, depth and detection values. `iter_ground_truth()` is deliberately
separate, and `Replayer.replay()` exposes no ground-truth callback. JSONL
metadata is merged first, but each potentially large NPZ is loaded only when
its depth record is yielded.

```python
from powertrain_sim.recording import RecordedRun, Replayer

run = RecordedRun("run-directory")
Replayer(run).replay(
    wheel=state_estimator_wheel_callback,
    imu=state_estimator_imu_callback,
    depth=terrain_depth_callback,
    detections=detection_callback,
)
```

## Host verification

```bash
PYTHONPATH=ros2/src/powertrain_ros:motor_control \
PYTHONPYCACHEPREFIX=/tmp/w6spc \
/home/light/anaconda3/bin/python -m pytest powertrain_sim/tests -q

/home/light/anaconda3/bin/python -m pytest \
  powertrain_observability/tests powertrain_autonomy/tests -q
```
