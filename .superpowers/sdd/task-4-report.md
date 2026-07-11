# Revised Task 4 report — singleton guard and Gateway SDK source

## Scope completed

- Added `ResourceGuard.acquire()/release()` with atomic `O_EXCL` lock creation, PID plus `/proc/<pid>/stat` start-time identity, inode-safe stale reclamation, stale socket cleanup, and no process signalling.
- Added `GatewayFrames` latest-one-slot handoff and `L515GatewaySource` with exact-serial selection, 1280×720 color, 640×480 raw depth, accel/gyro, `rs.align(color)`, separate raw/aligned depth, per-session dedup, reconnect clearing, and bounded stop.
- Changed dashboard video contract to fixed 1280×720 for RGB, aligned Depth, and alpha overlay. Removed variable-width side-by-side GStreamer startup behavior; all modes now retain one fixed child argv.

## TDD evidence

Initial focused RED command:

`/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_resource_guard.py l515_dashboard/tests/test_gateway_source.py l515_dashboard/tests/test_config.py l515_dashboard/tests/test_frame_modes.py`

Observed collection failures for missing `l515_dashboard.resource_guard`, missing `l515_dashboard.gateway_source`, and missing `FrameMode.OVERLAY`. After the first implementation, the same suite exposed disconnect-clearing and validation-order issues; those were corrected before the green run.

## Fresh verification

- Focused revised Task 4: 65 passed in 0.26 s.
- Tasks 1–4 dashboard regression: `80 passed in 0.44 s`.
- Existing L515 source regression: `19 passed in 0.12 s` with `PYTHONPATH=ros2/src/powertrain_ros`.
- `git diff --check`: clean.
- `/home/light/anaconda3/bin/python -m compileall -q l515_dashboard`: exit 0.

## Self-review notes

- Lock reclamation compares the observed inode before unlinking, preventing a stale-reader from deleting a concurrently replaced lock.
- Release removes paths only while still owning the recorded lock inode; it cannot remove a successor's socket after lock replacement.
- The guard never calls `kill`; malformed or unverifiable ownership is treated as stale pathname state only.
- SDK session-local mapper and timestamp history are recreated after reconnect, and stale slots are cleared before the new session streams.
- Stop invalidates the generation before touching the SDK pipeline. If `pipeline.start()` completes after cancellation, the worker observes cancellation and stops that pipeline instead of publishing frames.

## Concern / follow-up boundary

The existing `fc87492` streamer remains provisional architecture. Task 4 changes only its fixed-size/mode assumptions required by the revised contract; Gateway orchestration and revised Task 5 behavior are intentionally not implemented here.
