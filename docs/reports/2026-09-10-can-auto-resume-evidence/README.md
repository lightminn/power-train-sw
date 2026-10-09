# CAN automatic resume evidence — 2026-09-10

The [report](../2026-09-10-can-auto-resume.md) states final acceptance and physical limits. Final firmware is **patch3/A3**, application SHA-256 `022ac8f148501c084748336f7a65126ab2f1d96c381a4c95bc2079deaa497b86`. `SHA256SUMS` covers this directory except itself.

## Final evidence

| Path | Meaning |
|---|---|
|`build-patch3/`|Final compressed application, pinned build/source manifests and build log|
|`patch3-independent-review.md`, `patch3-report.md`|Final source review and implementation ledger|
|`patch3-independent-source-tests.log`|86passed against actual extracted firmware source|
|`patch3-independent-patch2-negative.log`|49passed/37failed against patch2; expected negative control|
|`patch3-mutations.json`, `patch3-mutation-*.log`|Nine deliberately broken variants rejected|
|`patch3-preparation-green.log`, `patch3-build-identity-check.json`|6preparation/build checks and388source hashes|
|`board-336A33523235-patch3/`|Final13/14 update from a fresh two-read patch2 backup; application/NVM/config verification and physical IDLE negative checks|
|`board-337733643235/`|Final15/16 update from a two-read patch1 backup; application/NVM/config verification and physical IDLE negative checks|
|`host-full-regression-isolated.log`|1151host tests passed; isolated task XDG runtime to avoid the live controller lock|
|`host-independent-final-review.md`, `host-independent-final-tests.log`|Final scoped review,45targeted tests; related independent probes also retained|
|`ros-regression-final.log`, `ros-regression-files.json`|231Jetson Humble tests across22selected files|
|`installed-vcan/{resume,cached_input,disarm,disconnect,timeout,estop,reconnect}/`|Actual installed ROS/TCP/vcan7case results and three node logs each; these are virtual motors, not physical motion|
|`production-delta.json`, `runtime-production-apply.json`|25source hashes, exact canonical checkout and pre-update backup location|
|`production-colcon-build.log`, `production-install-verified.json`|Three-package production build and actual installed Python/new message fields|
|`production-service-health.json`, `powertrain_*-current-start.log`|Eight running services, configured healthchecks, no new motor-service crash/traceback|
|`production-idle-restored.json`|Gated absent-component mask restore and inactive stop reset; no arm|
|`production-passive-can.json`, `production-can-before.json`, `production-can-after.json`|30sread-only physical CAN monitor, all10motors, zero errors/drops|
|`production-live-wheels.json`, `production-console-probe.json`|Production ROS wheel feedback and actual connected console/pad in IDLE, zero commands|

## Historical intermediate evidence

`board-336A33523235/` contains the earlier patch1 backup and **intermediate patch2** application/manual-origin results. Top-level `mks-build-manifest.json`, `mks-source-manifest.json`, `build.log` and `firmware-*` also describe the intermediate firmware. They are retained to explain the tested progression and are not final patch3 identities. `host-independent-review.md` records issues later closed in the final review. RED/mutation logs intentionally fail.

Compressed full flash backups decompress to exactly1MiB. Board helper scripts are immutable task evidence with pinned serials and old-image hashes, not reusable deployment commands. Current-board matching and the guarded DFU checks remain mandatory; do not rerun a helper assuming the saved old image is still installed. These files contain no configuration-save or calibration operation.

Jetson timestamps are1970because its wall clock is unset; all test intervals use monotonic time. Physical evidence covers firmware/readback, IDLE negative cases and passive reception. Nonzero real driving, positive physical HAL-fault resumption, node11/12 firmware update and repeated power-cycle acceptance remain unperformed.
