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

## Procedural elevated tracks

`procedural.py` creates a complete scenario document from an explicit frozen
`GenerationParameters`, an integer seed and a seed class. It instantiates only
`numpy.random.Generator(PCG64(seed))`; no host clock, global NumPy RNG or dict
iteration order controls an output. Length, width, elevation, bank magnitude,
curvature, station spacing, friction, motion profile, speed and acceleration
ranges are all parameters. Every generated track has aligned station arrays,
`height_m[i] == centerline_m[i][2]`, and left/right drop boundaries. The
terrain family is flat, constant bank or bank transition. Dropout, hole and
spike schedules are always declared, while `stress` adds all-stream dropout
and wheel measurement slip with longer or stronger intervals.

```python
from powertrain_sim.procedural import (
    GenerationParameters,
    dump_scenario_yaml,
    generate_scenario,
)

document = generate_scenario(
    GenerationParameters(),
    seed=20260716,
    seed_class="dev",
)
dump_scenario_yaml(document, "generated.yaml")
```

The YAML helper runs the part-one `parse_scenario` validator before writing.
Canonical JSON SHA-256 for regression seed drift is available through
`canonical_json_sha256`. Tune only with `dev` and `regression`; a
`hidden_evaluation` seed is generated and validated for completion evidence
only. Do not inspect its generated content or tune any algorithm or parameter
against it.

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

## MuJoCo fast autonomy bridge

`powertrain_sim.mujoco_fast` is the optional, headless P0 physics bridge. The
third-party `mujoco` package is imported only inside that subpackage, so
`scenario`, `fixtures` and `recording` continue to import on a ROS container
without MuJoCo. The bridge never imports `rclpy` and never creates a GL, EGL or
OSMesa renderer.

The MJCF builder uses one thin, slightly overlapping box per centreline
segment. This choice preserves each segment's centreline, elevation, width,
bank and friction while avoiding a mesh-wide single friction coefficient. A
large static floor at z=0 lies below the elevated boxes, so leaving a declared
left/right edge produces a physical fall and depth rays can see the lower
floor. Short start/finish aprons support the rover footprint while completion
is still measured only against the declared centreline stations.

The rover is a free rigid chassis with four nested steering/drive wheel bodies
and two fixed-steer drive wheel bodies. Wheel x/y positions, 0.10 m radius,
steering limit and drive limit come from production
`chassis.kinematics.default_geometry()`. `apply_command(v, omega)` calls the
production `solve()` exactly once and only converts its steering degrees to
radians and drive turns/s to joint rad/s. Fast mode intentionally omits
rocker-bogie articulation: its purpose is rapid autonomy and estimator
closed-loop regression; suspension fidelity belongs to later vcan/full-stack
simulation and physical HIL.

The IMU site is rigidly attached to the scenario's named IMU frame at the fast
model mount (x=0.12 m, z=0.18 m from the simulated base), and the depth site is
at x=0.30 m, z=0.18 m with a fixed 25 degree downward optical axis. These
mounts are fast-model inputs, not a claim about the final unmeasured L515
extrinsic. Gyro, accelerometer, frame position and frame quaternion are native
MuJoCo sensors. Depth uses `mj_multiRay` over the declared ROI and pinhole
intrinsics. A miss or a hit beyond 6.0 m is raw depth zero; valid ray ranges
are converted to optical-axis Z (the real L515 depth-image convention, so
pinhole reconstruction with the declared intrinsics is geometrically exact)
and rounded with the scenario `depth_scale_m`. Declared dropout creates an actual
stream gap, and hole/spike faults are applied to the ray result with the same
half-open interval and ROI rules as analytic fixtures.

Wheel slip uses measurement-side injection. That preserves the part-one
`measurement_scale` contract exactly and deterministically while station
friction still controls baseline physical contact. A later physical-slip
scenario needs a distinct schema field rather than silently changing this
fault's meaning.

```bash
PYTHONPATH=ros2/src/powertrain_ros:motor_control \
python -m powertrain_sim.mujoco_fast \
  powertrain_sim/scenarios/flat_straight_5m.yaml /tmp/w6s_run_demo
```

The default command source reuses the analytic fixture `_motion` profile. A
caller may inject `command_source(t_s, latest_estimate) -> (v, omega)` for a
future WP6-B controller. `hold_state_source(t_s, latest_estimate) ->
(actual_hold, should_hold)` is the independent policy-report hook used to
score false-hold, fail-open and release-to-recovery time without changing the
command callback contract.

Every run is written by `RunWriter`; ground truth remains in its isolated
stream and is never passed through `Replayer`. `metrics.json` and the printed
`MetricsReport` define:

- completion: maximum chassis centreline station divided by track length;
- wheel clearance: minimum wheel contact-point distance to the active drop
  boundary;
- edge overrun: count of outside-boundary footprint entry episodes, not the
  number of samples spent outside;
- false hold / fail-open: policy mismatch episode counts from the hold hook;
- recovery: longest declared-hold release to actual-hold release interval;
- runtime: total wall-clock run time plus maximum production estimator update
  time (the only wall-clock measurements in simulation logic).

The report compares completion, minimum clearance, maximum episode counts,
maximum recovery and maximum estimator time with `scenario.expected_metrics`
and returns an explicit pass flag and reason list. The CLI exit code is 0 only
when the report passes, 1 otherwise. Distance and yaw estimator error ratios
are diagnostic fields used by the flat and pivot regressions; a relative yaw
error against a near-zero true yaw (flat, bank) is not meaningful. The shipped
scenario `expected_metrics` were calibrated against the 2026-07-16 MuJoCo fast
physical evaluation with production geometry (see comments in each YAML).

## P1 hidden-seed closed loop

`closed_loop.TerrainAutonomyDriver` wires the fast plant to the production
WP6-B `TerrainEstimator` and WP6-C `AutonomyController` through the
`run_scenario` `command_source`/`hold_state_source`/`depth_tap` callbacks.
Everything runs in scenario time (no wall clock), tilt and SE(2) odometry
deltas come from the production `StateSnapshot` (never ground truth), the
depth-pose reference advances only after a successful terrain update, and the
arm gate is fabricated as a fresh `STOWED_LOCKED` heartbeat — a documented
simulation assumption. The camera extrinsic is derived from the fast-model
mount constants plus the spawn height, not the provisional production 0.60 m.

`python -m powertrain_sim.hidden_eval <seed> <run_dir>` generates one
procedural scenario (canonical sha256 recorded into `scenario.yaml`), runs the
closed loop, and exits 0/1 from `MetricsReport.passed`. Closed-loop documents
are generated with `expected_completion=False`: a fail-closed terrain
controller must stop about 0.55 m (the front-corner radius) before the
terminal drop of an elevated track, so the 95 % completion boolean is
physically unreachable and the honest acceptance is no fail-open, expected
clearance, and hold behaviour. `hold_state` compares the controller state
against the terrain reference the decision actually consumed, so hold metrics
carry no one-tick instrumentation phase artifact and recovery after an input
dropout is measured as same-tick (0.0 s).

## Host verification

```bash
PYTHONPATH=ros2/src/powertrain_ros:motor_control \
PYTHONPYCACHEPREFIX=/tmp/w6spc \
/home/light/anaconda3/bin/python -m pytest powertrain_sim/tests -q

/home/light/anaconda3/bin/python -m pytest \
  powertrain_observability/tests powertrain_autonomy/tests -q
```
