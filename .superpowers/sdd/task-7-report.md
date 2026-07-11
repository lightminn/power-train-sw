# Task 7 report: socket Dashboard and process independence

## Result

- Added a socket-only `GatewayClient` with bounded newline JSON requests, protocol/request-id
  validation, immutable snapshots, disconnect state, polling reconnect, and command acknowledgements.
- Added a Textual Dashboard for full Gateway/SDK/ROS/SRT/resource/error status. Keys `1/2/3`, `s`,
  `r`, and client-only `q` are direct; `Shift+Q` requires `y` confirmation before `stop_gateway`.
- Added the `python3 -m l515_dashboard` entrypoint and made `powertrain_ros` explicitly supervise
  the headless Gateway through `docker/powertrain_ros_entrypoint.sh`.
- Added operator documentation for remote driving, the GStreamer receiver, singleton diagnosis,
  Dashboard detachment, and the direct-maintenance exclusion.

## TDD and process evidence

- Initial client/UI collection failed because `client.py` and `app.py` did not exist.
- Focused client/UI GREEN: 6 passed; later disconnect coverage brings the focused behavior count to 7.
- The process integration test initially failed because its real fake-Gateway helper did not exist.
  The completed test starts an actual Gateway process and an actual SRT child, kills independent
  Dashboard stand-ins with SIGHUP and SIGKILL, proves both owned processes remain alive, then sends
  `stop_gateway` through the real Unix client and proves the Gateway, child, and socket are reaped.

## Verification

- Full `l515_dashboard/tests`: 184 passed (fresh final run recorded before commit).
- Docker Compose rendering: `docker compose -f docker/docker-compose.jetson.yml config` passed.
- Forbidden client imports: no `pyrealsense2`, ROS message, or Image import in client/UI entrypoints.
- `git diff --check` passed.
- No local `powertrain-sw:ros` image was available, so a fresh clean ROS container test could not be
  run on this host. Task 7 does not change ROS packages or message contracts.

## Review notes

- Gateway lifecycle remains independent of every client connection; the client opens one bounded
  request socket at a time and owns no long-lived child or hardware handle.
- Destructive shutdown remains server-acknowledged before cleanup, matching Task 6 deferred actions.
