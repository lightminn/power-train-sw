# MKS CAN single-board evidence — 2026-09-09

Result and acceptance boundaries: [report](../2026-09-09-mks-can-reliability.md).
Only serial `336A33523235` / CAN nodes13 and14 was flashed. The other four drive axes
retained their existing firmware. Autonomy and intentionally absent US-100/L515 were excluded.

| Evidence | Meaning |
|---|---|
| `336A33523235-*`, `backup-*.log` | Original 1MiB flash, two-read comparison, NVM and307-field configuration |
| `build-*.json`, `mks-source-manifest.json`, `build-*.log` | Pinned source/patch and two clean whole-firmware builds |
| `combined-regression.log`, `source-final-green.log`, `host-regression-final.log` | Actual host/source test output |
| `loop-client-final.log`, `loop-wheel-observation.json` | Installed ROS/vcan77 end-to-end loop, no physical driving |
| `flash-update-result.json`, `flash-update.log` | Application/NVM readback and device identity/configuration after flash |
| `zero-watchdog-recovery-*` | Zero-setpoint watchdog and same-baud CAN recovery trials |
| `post-reboot-*` | One user power cycle, Stuff error recovery latch, stable observation and explicit clear |
| `bounded-position-*` | Ground-loaded ±0.25 motor-turn target trial: FAILED target attainment; stopped and restored |
| `six-axis-queries-v1-*` | Earlier scheduling candidate with original board firmware; historical comparison |
| `final-six-axis-queries-*` | Final shared scheduler/driver, 600-second feedback-only real CAN run |
| `final-hardware-result.json`, `final-hardware-readback.log` | Final USB configuration/diagnostics plus six-second passive all-motor capture |
| `*.py` | Exact task-specific acquisition/flash/trial helpers; hardware actions require the documented setup and scope |
| `SHA256SUMS` | SHA256 for every file in this evidence directory except this manifest itself |

Full result JSON and compressed raw CAN are retained. Early `*-stdout-summary.json` files
were captured before the full artifacts were retrieved; their references to pending collection
are historical. Flash readback and build manifests are stronger evidence than console summaries.

The Jetson clock initially showed2026-08-18 and reset to1970-01-01 after the power cycle.
Use monotonic differences within a boot for durations; do not interpret those wall timestamps
as the date of this work. JSONL `t` is monotonic seconds; `timestamp` is SocketCAN wall time.
Firmware cumulative `tx_drop_count` and host SocketCAN dropped counters are distinct.

The watchdog300ms candidate was tested in RAM and restored to disabled/timeout0.
The bounded movement trial consumed cumulative encoder travel about0.19823 and0.17246
motor turns on nodes13/14. It does not establish real driving, visual motion, or braking acceptance.
