# Integrated robot operation

User approved on 2026-09-08: Jetson starts through one script; laptop opens one
console; existing video, telemetry, gamepad and gated operations connect
automatically. One explicit held Start enables manual driving. ESTOP reset stays
separate. Autonomy and arm teleoperation are not activated by this change.

## Runtime boundaries

The new opt-in deployment uses `powertrain_runtime` (standard-library Python).
The existing ROS control processes bind loopback ports 19000/19001. A robot
session service exposes authenticated proxies on 9000/9001 and a session control
socket on 9002. Raw-input schema and existing ops token roles remain unchanged
inside the proxies. Only one console lease may own these channels. A robot ID
and mutual HMAC challenge using the configured console token bind discovery to
the paired robot. Nonces and per-session tickets are random, never logged.

The console resolves configured IPv4 host candidates, authenticates the robot,
and registers its socket peer address as the telemetry destination. A bounded
lease expires on heartbeat loss; channels close and disarm is requested. The
existing 0.2 s input watchdog remains the primary input-loss path. Reconnect
creates a new lease and never restores driving intent. Explicit release and
process shutdown request stop before closing; failure is reported, not success.
The end of an authenticated input channel also revokes its whole lease, even
when the control socket is still alive. A new input channel is admitted only
after fresh IDLE chassis, IDLE/MOTION_HOLD authority and stopped-wheel evidence;
otherwise safe stop is requested while the ops diagnostic channel stays usable.

Telemetry owners read `/run/powertrain/operator-session.json` at send time in
integrated mode. The record has host, lease_id and expires_at (monotonic seconds).
Absent/expired records suppress sending; legacy mode retains configured targets.
No service restart or persistent privileged IP rewrite is needed on reconnect.

## Laptop

`operator_console.operation_runtime.OperationRuntime` manages SessionClient and a
separate gamepad process, feeds immutable status snapshots to GTK and exposes
nonblocking start/stop controls. The child owns pygame only and communicates by
bounded JSONL stdin/stdout. It uses the existing DualSense mapping and input
encoder; no input-policy duplication. Missing/disconnected pads stay waiting.
Parent stdin EOF or parent heartbeat expiry ends input. Console singleton lock
prevents duplicate children. GTK and gamepad Python interpreters are configured
once; no machine-specific interpreter path is committed.

Operator config `~/.config/powertrain/operator.json` contains robot_id, hosts,
session_port (9002), token_file (console token), controller_python and profile.
`python -m operator_console` launches the integrated console; the existing
`operator_console.app --host` remains an explicit legacy entrypoint.

## Drive start and readiness

Session RPC `start_begin` records intent; `start` requires a 1.5 s hold. A single
asynchronous ManualDriveService sequence checks fresh ops state, ESTOP absence,
released neutral input and stopped wheels; clears transient HOLD only if present;
requests MANUAL; rechecks state; requests arm; verifies final MANUAL/ARMED state.
Every stage waits for terminal success. Rejected/unknown results or freshness timeout abort;
no automatic ESTOP reset, arm retry or old motion replay. Stop requests disarm
and IDLE. Hardware policy remains ChassisManager/SafetyInterlock-owned.
HOLD acknowledgement may precede gateway input_fresh: the existing gateway
intentionally marks gated HOLD input not fresh. After clearing, a new fresh
neutral DRIVE frame is required before manual/arm. Already-clear ROS targets
acknowledge a no-op. Stale post-ACK snapshots wait within the deadline without
sending another mutation; fresh unsafe evidence aborts immediately.

Console readiness displays connection, pad, video, chassis/safety independently.
Remote-drive Start additionally requires a decoded live front frame and fresh
gamepad-neutral state. Optional camera/arm absence does not fabricate readiness
or stop the whole UI. Disconnected/failed capability displays its actual reason.

## Deployment

`scripts/robot_prepare.sh` performs explicit install/build/provisioning checks;
`scripts/robot-start` launches an already-prepared integrated Compose profile
without building, calibrating, killing unrelated processes, or weakening safety.
It is idempotent and works with no laptop present. Only the explicitly supported
CAN 4WS profile is enabled initially; USB skid stays its existing separate bench
deployment until its ROS/safety commissioning is complete. Legacy Compose remains
available. A desktop launcher and one-time laptop setup generate real paths.

## Verification

Tests must run RED before implementation and GREEN afterward. Include real TCP
mutual-auth/lease/proxy tests, real UDP destination changes, pad hotplug/EOF,
held-start cancellation/unknown response, duplicate console, both boot orders,
and actual GTK/Xvfb integrated smoke. Shell tests run real entrypoints with fake
external commands. Negative controls remove auth/lease/readiness protections and
must fail. Jetson installed ROS, decoded real SRT, motor stop and physical ESTOP
remain separate real-hardware acceptance; never infer these from host tests.
