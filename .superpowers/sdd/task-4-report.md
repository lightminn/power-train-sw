# Revised Task 4 review-fix report

## Review findings resolved

1. Lock publication now writes and fsyncs a private temporary inode before atomically linking it into the public lock path. A per-resource `flock` serializes publication and stale reclamation. Malformed ownership is fail-closed, not classified stale.
2. `L515GatewaySource` now carries the proven generation/public-lock/lifecycle-lock design, final worker cleanup, pre-start validation, late-frame commit guard, and bounded worker cleanup. `stop()` never calls `pipeline.stop()` while native start remains in progress; the worker performs exactly one cleanup.
3. Frame rendering requires exact contiguous 1280×720 BGR8 color and exact contiguous 1280×720 Z16 aligned depth. It never resizes. RGB returns the original array and overlay uses configured alpha.
4. `DashboardConfig` now owns socket/lock paths, color/depth profiles, overlay alpha, reconnect interval, and maximum message size with strict validation. Gateway source directly consumes this one typed config.
5. Production mapper default lazily constructs `powertrain_ros.l515_adapter.TimestampMapper`.
6. Socket removal requires a recorded `(device, inode)` from `claim_socket()`. Unknown and replaced socket paths survive release/reclamation.
7. Trailing whitespace/new-EOF findings were removed; `git diff --check` is clean.

## TDD RED evidence

Command:

`PYTHONPATH=ros2/src/powertrain_ros /home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_resource_guard.py l515_dashboard/tests/test_gateway_source.py l515_dashboard/tests/test_config.py l515_dashboard/tests/test_frame_modes.py`

Before fixes: 42 failed, 63 passed in 2.10 s. Failures covered partial lock publication, unknown/replaced socket removal, missing config contract, non-strict frame boundaries, missing real mapper, late worker state regression, and pre-start cleanup races.

After fixes and expanded exact race tests: 109 passed in 0.80 s.

## Fresh final verification

Commands to be run immediately before commit:

`PYTHONPATH=ros2/src/powertrain_ros /home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests`

`PYTHONPATH=ros2/src/powertrain_ros /home/light/anaconda3/bin/python -m pytest -q ros2/src/powertrain_ros/test/test_l515_source.py`

`git diff --check`

`/home/light/anaconda3/bin/python -m compileall -q l515_dashboard`

Exact fresh outputs:

- Dashboard Tasks 1–4: `124 passed in 0.88s`.
- Existing L515 source lifecycle regression: `19 passed in 0.14s`.
- `git diff --check`: exit 0, no output.
- `compileall`: exit 0, no output.

## Scope boundary

Only fixed-size and overlay-alpha plumbing needed by revised Task 4 touched the provisional streamer. Revised Task 5 Gateway orchestration remains out of scope.

## Final re-review fixes

- Runtime defaults now use the exact socket `/run/powertrain/l515-gateway.sock` and consistent lock `/run/powertrain/l515-gateway.lock`.
- Public stop and worker-finally share a pipeline-specific cleanup record. The first path atomically claims cleanup under the lifecycle lock; all later paths skip it. Normal streaming therefore calls `pipeline.stop()` exactly once, including when the bounded stop thread remains blocked. Native-start cancellation still leaves cleanup to the worker and calls stop once after start completes.

TDD RED command:

`PYTHONPATH=ros2/src/powertrain_ros /home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests/test_config.py l515_dashboard/tests/test_gateway_source.py`

RED result: `3 failed, 84 passed in 0.49s`; failures were the exact socket default, ordinary double-stop, and overlapping blocking double-stop.

Fresh verification after fixes:

- Focused config/Gateway source: `87 passed in 0.29s`.
- Dashboard Tasks 1–4: `126 passed in 0.75s`.
- Existing L515 lifecycle regression: `19 passed in 0.12s`.
- `git diff --check`: exit 0, no output.
- `compileall`: exit 0, no output.
