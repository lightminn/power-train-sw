# Task 6 report: Unix control service and Gateway lifecycle

## Result

- Added protocol v1 newline-delimited JSON envelopes with strict fields, command payload validation, and configured byte bounds.
- Added an independent-thread Unix socket server. Partial reads are framed correctly; slow or disconnected clients do not stop the server or Gateway.
- Added the headless Gateway lifecycle and `python3 -m l515_dashboard.gateway_main` entrypoint.
- State-changing commands are serialized by the Gateway lifecycle lock.
- L515 disconnect/reconnect maps to `DEGRADED`/`RUNNING`; stale frames are not replayed because each loop drains one latest slot.
- GStreamer failure disables streaming and degrades the service while SDK/ROS/socket remain alive. ROS publication failure transitions to `FAULT` and runs full cleanup.
- Dashboard disconnect is intentionally a no-op.
- One idempotent cleanup path blocks frame intake, then stops SRT, SDK source, ROS, socket, and guard.

## TDD evidence

- Initial focused run: collection failed with `ModuleNotFoundError` for the three intentionally absent Task 6 modules.
- Optional SRT start test then failed because the initial lifecycle treated it as fatal; implementation was narrowed so only SRT is optional and degraded.
- Final dashboard suite: `151 passed in 0.95s`.
- `python -m compileall -q l515_dashboard`: pass.
- `git diff --check`: pass.

## ROS regression limitation

Host conda collection cannot run the ROS suite because the host environment lacks ROS 2 modules (`launch`, `rclpy`, `builtin_interfaces`, and `ament_index_python`). The configured `powertrain-sw:ros` image is not present locally, so no fresh ROS-container result is claimed. Task 6 does not modify ROS package sources or message contracts; existing pure-Python `GatewayRosPublisher` tests are included in the 151 passing dashboard tests.

## Remaining HIL

Jetson validation is still required for real signal/container stop, L515 unplug/replug, GStreamer process crash/restart, and zero OS-owned resource counts. These are hardware/runtime acceptance items, not claimed by the software suite.

## Deep-review remediation

Commit follow-up resolves all nine review findings:

- Every SRT generation is retained until stopped; crashed/stopped workers are reaped before replacement and cleanup visits every still-owned generation.
- One reentrant lifecycle lock covers poll, ROS publish, SRT submit, component restart, and shutdown.
- The server never blindly removes a pre-existing path. Production binding requires an acquired `ResourceGuard`, runtime directories are mode 0750 or stricter, socket mode is 0660, and inode-checked cleanup preserves replacements.
- Components are registered as owned before `start()`, so an exception after partial initialization invokes rollback stop.
- Stop/restart commands return an accepted response before a post-send action. Restart is an internal SDK/ROS/SRT restart which keeps the guard and control socket; stop cleanly exits the main loop with status 0, while fatal errors return 1.
- Client count, message size, and read-idle time are bounded; all tracked client sockets and threads are closed/joined during server stop.
- Status includes exact SDK serial/profile/source state, injected rolling diagnostics, ROS publication counts, full SRT counters/error, injected CPU/RAM data, and the last Gateway error.
- CONNECTING remains STARTING, only a streaming source can become RUNNING, and SRT failure text propagates to Gateway status.
- Once JSON parsing recovers a request ID, validation errors retain it in the error response.

Fresh verification after remediation:

- `/home/light/anaconda3/bin/python -m pytest -q l515_dashboard/tests`: `161 passed in 1.49s`.
- `/home/light/anaconda3/bin/python -m compileall -q l515_dashboard`: pass.
- `git diff --check`: pass.
- No HIL was run, as requested for this review follow-up.
