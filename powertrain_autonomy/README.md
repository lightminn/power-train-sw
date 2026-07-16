# Powertrain autonomy pure cores

`powertrain_autonomy` contains the value-based NumPy production authority and
an optional fixed-shape JAX terrain kernel. It has no ROS, `rclpy`, hardware,
or simulator branch. In particular, powertrain_autonomy does not import powertrain_ros.
The WP6-C adapter translates WP6-A `TiltSnapshot`
and `PoseSnapshot` values into the local `BodyTilt` and `OdometryDelta`
dataclasses. This keeps the dependency direction from ROS adapters toward pure
cores and avoids a package cycle. The terrain core reads production wheel
centre coordinates from `chassis.kinematics.default_geometry()`; it does not
modify `motor_control`.

## WP6-B NumPy terrain pipeline

The implemented pipeline follows the WP6-B authority:

```text
raw depth + CameraInfo
    → fixed ROI/stride
    → ROI valid ratio + robust median/MAD/percentile rejection
    → internal XYZ point cloud
    → L515 extrinsic + WP6 roll/pitch gravity alignment
    → 5 cm candidate-resolution 2.5D elevation grid
    → height, normal/slope, roughness, observation confidence
    → bank/uphill/drop-edge/obstacle classification
    → footprint-safe centre path
```

`TerrainFrame` uses the L515 optical-Z contract: a two-dimensional depth ROI,
`uint16` millimetres in normal operation, a positive `depth_scale_m`, pinhole
intrinsics, and a monotonic seconds stamp. ROI bounds and stride are fixed in
`TerrainEstimatorConfig`. The sampled shape and the elevation grid are fixed shape
for the lifetime of an estimator; invalid cells remain a mask and never
shrink an array. The optional JAX kernel retains that NumPy contract unchanged;
the shared NumPy/JAX kernel boundary enforces the same array contract before
the branch-heavy terrain classifications.

The estimator calls `terrain.depth_quality` for frame and fixed sub-ROI valid
ratio, median/MAD/percentile, pixel connectivity, normal consistency,
spike/hole, disconnected-lower-floor, and temporal decisions. It does not
duplicate those checks. Their reasons are inherited into the estimate. A
rejected tile cannot seed support for a hole or temporal jump; individual
out-of-range samples remain masked. Elevation cells separately use their
required robust height median, point count, residual MAD roughness, local
finite-difference slopes, and local height-consistent support connectivity. No
single global terrain plane is fitted.

Positive x is forward, positive y is left, and positive z is up. Optical x is
right, optical y is down, and optical z is forward. Camera points are converted
with full rotation matrices for the optical frame, configured mount rotation,
and injected body roll/pitch; there is no small-angle approximation. Wide,
continuous lateral slope is reported as bank rather than an obstacle. A local
high protrusion is excluded from support as an obstacle candidate. Surface
termination plus disconnected lower-floor evidence defines the two drop
boundaries used by erosion.

## Drop-boundary corridor semantics

Occlusion geometry displaces lower-floor observations systematically forward
of the edge row that causes them, and on a banked or offset track the floor can
land outside the fixed grid's lateral range entirely. Side drop evidence is
therefore collected twice: from in-grid lower-floor cells, and pre-grid from
every in-range raw depth point that falls at least `drop_height_m` below the
interpolated per-row support reference, outside that row's support edges, with
a minimum point count so an isolated spike cannot fabricate evidence.

Whether one row's support edge is a real track edge is decided against the
analytic per-row field-of-view limit (the camera frustum intersected with that
row's support height, using the full extrinsic and injected tilt). An edge
sitting at the FOV limit is observation truncation, not a boundary; an edge
strictly inside the FOV with side evidence is a real-edge candidate, and it
must also stay within two cells of the per-side corridor median (track
boundaries are spatially continuous; isolated deviations are data gaps).
FOV-clipped and data-gap rows inherit the corridor and only contribute
support-coverage checks, so truncation can only shrink the usable path —
fail-closed. A lower-floor detection strictly inside the corridor (a local
choke narrower than the eroded footprint) rejects the whole frame instead of
routing around it with wider rows. Row centres for offset/heading come from
real edges where available, reconstructing with the corridor-width prior when
only one side is real, so the 5 cm grid quantization does not dominate the
heading fit.

Runtime on the x86 dev host is ~31 ms mean per 60x80 frame (74x60 grid). The
fixed-shape JAX kernels and NumPy/JAX grid equivalence are implemented for
depth deprojection, gravity-aligned coordinate transformation, masked
elevation scatter, per-cell median/MAD, and slope. NumPy remains the
correctness reference for this candidate backend.

## Grid history and footprint contract

Only the latest 1.5 seconds (configurable up to the intended 1–2 second local
window) are retained. The injected previous-to-current `(dx, dy, dyaw)` moves
old cell centres through exact planar SE(2), then re-bins them to integer cells.
A fractional residual is deliberately not interpolated: carried confidence is
reduced, and the largest residual is added to footprint erosion uncertainty.
Current observations replace carried observations; history fills only current
blind or invalid cells. There is no long-term map, loop closure, or SLAM.

The chassis half-footprint is derived as the maximum absolute production wheel
y coordinate (currently 0.4395 m), plus configured wheel half-width, configured
uncertainty, and any odometry re-bin residual. The support interval between the
left and right drop edges is eroded by that amount. The estimator reports path
offset and heading only when enough contiguous fixed-grid rows remain.
Left/right wheel clearance excludes the extra uncertainty term so it remains a
measurable physical clearance from the current outer wheel edge to each drop
boundary.

`TerrainEstimate` is immutable. A stale/future/regressing frame, a temporal
jump, missing connected support, unobserved drop evidence, or empty erosion
returns `path_available=False` with explicit reject reasons and zero confidence.
The estimator does not reuse a prior path as a motion basis.

## WP6-C controller core

`controller.AutonomyController` consumes an immutable WP6-B `TerrainEstimate`,
a WP6-A-derived `MotionState`, a required-arm-status `ProfileGate`, and optional
`DriveDiagnostics`. Each injected `now_s` call returns a frozen
`ControllerDecision` with finite, nonnegative forward speed, bounded yaw rate,
state, and explicit hold or slowdown reasons. The core reads no wall clock and
does not subscribe or publish.

`BLOCKED` is reserved for the arm collaboration gate: a missing, stale, future,
or profile-mismatched arm status forces immediate zero and resets the slew
origin. `CONTROLLED_HOLD` covers missing or stale terrain/motion, unavailable or
low-confidence path, clearance/tilt limits, and a fresh stuck diagnostic. It
ramps the last output to zero at the active profile's deceleration and yaw-slew
limits, then automatically resumes from zero when inputs recover. Stale or
missing diagnostics only remove their optional slip and speed-cap constraints.
Recovery from `CONTROLLED_HOLD` requires three consecutive fresh controller ticks;
the entry thresholds and immediate `BLOCKED` semantics are unchanged.

The `EMPTY_STOWED` preset is provisionally limited to 0.8 m/s, 0.5 m/s²
acceleration, 0.8 m/s² deceleration, 0.8 rad/s yaw rate, 15° bank, and 15°
slope. `CARRYING_LOCKED` is provisionally more conservative at 0.5 m/s,
0.3 m/s² acceleration, 0.6 m/s² deceleration, 0.5 rad/s yaw rate, 10° bank,
and 12° slope. These values are HIL candidates, not production-qualified limits.
The controller core lives here; ROS subscriptions and `/autonomy/cmd_vel` publication live in `powertrain_ros` `autonomy_controller_node`.
Final command selection remains the existing `chassis_node` CommandAuthority.

## Provisional extrinsic and deferred work

The default camera height 0.60 m and downward pitch 25 degrees are provisional,
unmeasured configuration candidates. They must not be used as evidence for
production completion. Production requires the planned physical 20/25/30
degree comparison and a measured `base_link→l515_link` transform.

RGB auxiliary confidence is explicitly deferred.
NumPy is the only production authority for terrain estimation. The optional
JAX module provides `warmup(config)`,
rejects shape/dtype drift before JIT dispatch, and validates device results at
the CPU boundary, but it is not selected by the estimator or any launch
profile. Jetson qualification and backend selection remain deferred, including
the full-load latency/resource gate, accelerator version pinning, and launch
memory policy such as `XLA_PYTHON_CLIENT_PREALLOCATE=false`. The WP6-C
controller is backend-neutral and consumes the immutable estimator result.
