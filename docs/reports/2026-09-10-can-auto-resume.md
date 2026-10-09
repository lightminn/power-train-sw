# CAN-only automatic remote-driving resume — 2026-09-10

Status: implementation, related regression tests, installed ROS/TCP/vcan acceptance and deployment completed. Nodes13–16 run final firmware patch3. Production is IDLE with zero wheel commands and the console/pad connected. Nonzero physical driving and real HAL-fault motion resumption have not been performed in this task.

## Request and observed fault

The user requested motion resumption after CAN communication recovery and authorized testing/deployment while not driving. Autonomy is outside acceptance. Existing unrelated work and the team's separate Jetson checkout are preserved.

USB on serial `336A33523235` identified nodes13/14, MKS v3.6-56V, firmware0.5.1-dev with reliability patch1. CAN had recovered (`hal_error=0`, recovery_count10, failures0), while both axes remained IDLE with `0x4000` and `can_recovery_latched=true`. HAL history was `0x218` (stuff/form/RX FIFO0-overrun), last error `0x10` (form). The deliberate firmware stop latch explains why recovered communication alone did not restore driving. The physical or timing origin of the initial HAL error remains unproven.

A later pre-backup read found node14 IDLE with axis_error`0x40` and motor_error`0x1000` (`CURRENT_LIMIT_VIOLATION`), while node13 was error0. This distinct fault is excluded from automatic resume. No current limit, gain or calibration was changed to suppress it. Before the final update both axes were IDLE/error0; the original observation is preserved in the historical board evidence.

Node15/16's pre-update patch1 snapshot recorded recovery_count3464, failures0, HAL0, history`0x218` and tx_drop_count2632. These cumulative counters do not establish a continuously active fault or its cause. The final observation below is bounded to the measured intervals.

## Implemented behavior

- Firmware patch3 retains immediate PWM stop and zeroed inputs. Heartbeat bytes5–7 add marker`0xA3`, latch/progress/eligibility flags and a recovery generation. Only automatic HAL recovery interrupting healthy CLOSED_LOOP can grant eligibility. E-stop, manual CAN reinit, explicit clear and reboot revoke it.
- ChassisManager coordinates all six axes: stop, retain the interrupted driving intent, require200ms stable stopped/fresh feedback and healthy steering, request guarded resume, verify actual error0/state8, then wait for a new real manual input. The episode expires after5s. Disarm, E-stop, input/authority loss and unrelated faults cancel it.
- Affected axes receive one atomic guarded RESUME: CAN opcode`0x07`, DLC8, `struct.pack('<IBBBB', 8, 0xA3, generation, 0, 0)`. Firmware validates the cause, generation, state and all errors inside one critical section before clear/zero/request8. The host sends neither unconditional clear nor ordinary ARM afterward. Continuously fresh error0 peers use ordinary zero/arm without clear.
- Actual TCP receipt time and the server-generated connection UUID travel with the manual command through ROS. Cached50Hz republication cannot count as new input. Rapid reconnect cancels the episode even if the disconnect edge disappears between publications.
- Every axis observed missing/stale must supply its own eligible origin. An eligible peer cannot excuse a rebooted/error0 axis. WAIT_INPUT validates every drive and steering sample before any nonzero dispatch.
- Normal healthy/qualified-stop gates remain strict. The earlier CAN-only IDLE`0x4000` hardware-stop proof correction is preserved; stopped evidence is not authorization to clear or arm. Deadman release remains a fresh zero command; legacy Twist still supports manual operation without automatic-resume provenance.

Independent review found and closed stale-steering partial dispatch, missing/rebooted-peer exemption, rapid-reconnect cancellation and separate clear/arm race defects. Final firmware and host reviews found no remaining blocker within their stated source/test scope. See [specification](../specs/2026-09-10-can-auto-resume.md), [plan](../plans/2026-09-10-can-auto-resume.md), and [evidence index](2026-09-10-can-auto-resume-evidence/README.md).

## Firmware identity and preservation

Final application: **248988bytes**, SHA-256 **`022ac8f148501c084748336f7a65126ab2f1d96c381a4c95bc2079deaa497b86`**. Final patch SHA-256: `9b0e7b271d10b824e7edd70ae8b540f61e9b018a73f592928eb020de6d52a2ca`. All388 prepared source hashes match the reviewed source; the pinned Cortex-M4 build passed. Config_t/NVM serialization and existing Axis/PWM/watchdog paths are unchanged from the reviewed preceding patch.

| Nodes / serial | Final application | Full1MiB backup immediately before final update | Preserved NVM SHA-256 |
|---|---|---|---|
|13/14 / `336A33523235`|patch3; full application readback passed|Current intermediate patch2: `830646082d14cd1c77b6961b2346b8a0798c36e1955f10c14674723a8d9b7372`|`94f30a820aa27c3f00b838bc9649fc5599ea3ef4ac7e3792979b9a2f1223edb0`|
|15/16 / `337733643235`|patch3; full application readback passed|Current patch1: `9a3805a324de22e383adc16dd997f2710519f125c876c7b10157ff8e7c9a23b9`|`ab01ebabde5eb5e0a6de7e610816ff6fea7c65247eec763036466e095741688d`|
|11/12|Unpatched; USB firmware version not verified|No update performed|No NVM operation performed|

Each backup was read twice and compared byte-for-byte. Guarded DFU pins the exact serial, current image and candidate, writes application sectors0–9 only, verifies the full application and last256KiB NVM, and retains application rollback. Saved configuration and calibration readiness were preserved on both boards. There was no configuration save, recalibration, gain/current-limit or watchdog setting change. Known earlier RAM input_mode1→saved2 differences were recorded before firmware update; final13/14's patch2 backup had no configuration difference.

The earlier13/14 patch1 backup (`0f4d5ecd5b1a77298d59c779c8d9ecef3fdeb596ed533e988c70cd158fe9db94`) and intermediate patch2 evidence remain historical. Patch2 was superseded by patch3's atomic guard and is not automatically resumable by the final host.

## Acceptance results

| Layer | Actual result | Boundary |
|---|---|---|
|Actual firmware-source scenarios|86passed; patch2 negative control49passed/37failed;9 mutation controls detected|HAL/IRQ/RTOS are deterministic doubles; real IRQ timing is not certified|
|Preparation/build checks|6passed; Cortex-M4 application built;388 source hashes matched|Build/source identity, not physical motion|
|Host regression|1151passed: motor_control, powertrain_runtime/tests, tools/tests/test_mks_dfu.py|Includes32 unchanged DFU transport tests; isolated task runtime avoids the live console's controller lock|
|Jetson ROS regression|231passed across22 selected remote-operation ROS test files|Actual Humble/rclpy; excludes unfinished autonomy|
|Installed ROS/TCP/vcan|7cases passed: resume, cached_input, disarm, disconnect, timeout, estop, reconnect|Installed entrypoints, typed DDS and real CAN drivers against virtual6ODrive+4AK; domain77/network-none/vcan77 with can0 absent|
|Independent host recheck|45targeted tests and4separate probes passed|Overlaps other counts; do not sum as unique tests|
|Application/NVM/config readback|Both13/14 and15/16 passed final patch3|Exact physical boards above|
|Physical IDLE negative checks|Both boards rejected guarded resume after manual CAN reinit and again in healthy IDLE; input/velocity0|No successful arm or nonzero motion command; positive automatic motion remains untested physically|
|Production deployment|25 source/test hashes matched;3 ROS packages rebuilt; installed Python and ManualDriveCommand fields verified|Canonical Jetson checkout, not the team's checkout|
|Production runtime|Daily scripts/robot-start passed;8 services running, restart counts0, configured healthchecks healthy; new motor-service logs have no traceback/process death|Startup warning still identifies stop_mm as BENCH provenance|
|Production CAN|30.005s; all10motors1500–1501frames; maximum gap25.714ms; all6drive heartbeats error0/state1|Read-only monitor transmitted0frames; host CAN errors/drops/bus-off/restarts0 before and after|
|Production wheel/console path|6fresh wheel records, drive/steering faults0, commands/velocities0; console and pad connected, ready=true, chassisIDLE|US-100 and robot_arm disabled per the existing setup; no drive start requested|

Physical manual restart on13/14 produced `0040000001a30101`: exact0x4000, IDLE, A3, latch1/eligible0, generation1. Recovery count stayed1→1 during the final5s passive check; HAL history0. Node15/16 produced generation2 after manual restart plus a recorded RXFIFO0-overrun recovery; final recovery count stayed2→2, HAL0. Both final5s checks saw all10motors and six IDLE/error0 drive axes.

At production startup, default US-100 ON briefly caused its expected liveness stop. Through the gated ops channel, the absent-component mask was restored and the now-inactive stop explicitly reset. The console's reconnect briefly reported an unknown stop outcome/rate limit; its final live state confirms no start blocker, fresh neutral pad input and ready=true. The old outcome remains in the probe as historical UI detail. The existing console process was retained and no arm action was issued.

The first host run had one controller-lock collision with the running console; rerunning the full suite with a separate XDG runtime passed1151. The first staged ROS run omitted two unchanged documentation/compose fixtures; copying those exact files and rerunning all22files passed231. These environmental retries did not weaken assertions.

## Final locations and remaining boundaries

- Local checkout and canonical Jetson `/home/zetin/power-train-sw-integrated` contain matching task changes. Production pre-update files are backed up under `/home/zetin/powertrain-can-evidence-20260910/runtime-production-before`.
- Local main, fetched origin/main and canonical Jetson HEAD are `7d7f00130a62dcffae71c50924f92a14a5bfeba7`. This task remains **uncommitted/unpushed**; it does not claim GitHub contains the new changes. The separate team checkout `/home/zetin/power-train-sw` is preserved.
- The isolated test container is stopped. Production remains IDLE; the console is connected. No autonomy, optimization or legacy simulation work was performed.
- Node11/12 firmware requires a separate USB-identified update before those axes can supply automatic-resume provenance. A loss on those axes is intentionally not auto-resumed.
- Nonzero driving, real HAL-fault positive resumption, repeated power-cycle/ground acceptance and permanent CAN stability remain unverified. Actual HAL error cause is not proven by this recovery change.
- Jetson wall clock reports1970; measured intervals use monotonic time. Build clock-skew warnings were recorded, and fresh installed artifact identity was checked separately. The report date comes from the development host.
