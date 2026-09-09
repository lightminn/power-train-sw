# Operator Console

Integrated entrypoint: `python -m operator_console` uses the prepared paired-robot
config, reconnects automatically, supervises a separate gamepad process and offers
a held manual-start action. See [the integrated operation runbook](../docs/integrated-operation.md)
for setup, legacy-service migration and the pending real-hardware acceptance.
`python -m operator_console.app --host ...` remains the explicit legacy entrypoint.

Native GTK/GStreamer operator console. Observation is RX-only; operator
actions travel only through the token-gated ops channel. It embeds L515 driving
RGB SRT (`:5000`) and D435i raw RGB SRT (`:5002`) with per-channel receiver health.
It also listens for D435i YOLO metadata JSON on UDP `:5003`; stale metadata is
hidden rather than delaying raw video. Robot health snapshots use UDP `:5004`.
The console shows `UNAVAILABLE` until an approved data owner
(ChassisManager/odometry/power monitor) sends a real snapshot; it never probes
ODrive over USB or CAN to fill this panel.

Telemetry v1 is one bounded JSON UDP datagram. A future sender may omit an
unknown physical measurement as `null`, but must not invent it:

```json
{"schema_version":1,"sequence":42,"odometry_source":"wheel+imu","x_m":1.2,"y_m":-0.4,"yaw_rad":0.3,"voltage_v":null,"current_a":null,"power_w":null,"drive_state":"IDLE","can_state":"OK"}
```

`x_m`, `y_m`, and `yaw_rad` are the robot `odom` pose (wheel states fused with
L515 gyro/accel in WP6), not a camera-pixel coordinate. D435 YOLO object
coordinates stay associated with each camera bounding box on UDP `:5003`.

The chassis sender on UDP `:5005` also mirrors the Gateway's scalar L515
observability contract: native callback rates, all six ROS topic rates,
SRT submit/sent/drop rates, aligned-Depth age, and Gateway process CPU/RSS.
Missing Gateway data remains `null`/`UNAVAILABLE`; the console never opens a
camera to fill it.

Robot-arm telemetry arrives on UDP `:5007` (override with
`--arm-telemetry-port`) from the `arm_console_bridge` ROS node, which mirrors
the arm team's `/dynamixel/state`, `/joint_states`, `/detected_objects`, and
latched `/pick_target` without touching the arm repo. The panel lists each
Dynamixel as `ID · angle° · current(raw) · temperature[state]` — thresholds
WARN 55 ℃ / CRIT 65 ℃ are provisional until the arm team confirms per-model
limits — plus joint angles. Detection overlays on the D435i panel add
`yaw ±deg°` and highlight the latched pick target in orange. To start the
bridge on the robot (arm stack running, single :5003 sender rule — keep the
arm `metadata_sender_node` down while the bridge runs):

```bash
ros2 run powertrain_ros arm_console_bridge --ros-args -p console_host:=<laptop-ip>
```

```bash
/usr/bin/python3 -m operator_console.app --host 192.168.8.106
```

The judge-facing interface uses **실시간 화면** (mission/video) and **시스템 상태**
(integrated power, communication, and safety summaries with collapsed raw
diagnostics). Open **복구 · 설정** beside the two-tab navigation to reach the
existing token-gated controls for E-STOP reset, component settings, steering
mode, and advanced recovery actions. This non-modal window hides on close and
cancels any unfinished confirmation; reopening it keeps the same authenticated
ops session. Press `F11` for the competition display. Camera panels can be
clicked to exchange the large and preview views; technical stream errors
remain in the collapsed event log and robot diagnostics.

`/usr/bin/python3 -m operator_console.runtime_smoke` launches the real GTK
application under Xvfb, injects maximal synthetic datagrams into isolated UDP
ports, and verifies LIVE→STALE, environment rendering, sparse-payload,
clean-shutdown, traceback, role-layout, and no-automatic-camera-swap paths. A
loopback ops TCP fixture also sends an unknown steering mode through the real
client and checks that the GTK control stays unavailable. It does not inject or
observe a decoded real SRT frame, prove GStreamer frame rendering, or exercise
a real network or hardware sender. Those need separate runtime or E2E evidence.

The lower-right mission rail is the robot-arm/tool hub. It exposes only the
four confirmed equipment categories (그리퍼 1, 그리퍼 2, 청소 모듈, 환경 센서
모듈), plus attachment state, joint-load summary, and the explicit fact that
arm control-mode telemetry is not yet wired. Selecting a tool or pressing its
detail button opens a common non-modal detail window. The environment window
groups climate, air quality, and hazard readings. CO/LPG use plain numeric
values, flame detection is shown as O/X, and SGP30 eCO₂/TVOC values display
`예열 중` until warmup ends. Lost sensor power or stale input clears the previous
readings and displays 미연결. Source/sequence details stay collapsed.

The **시스템 상태** tab is a single RX-only live view for PDIST80B power,
communications, and safety. Drive speed and AI summaries are on the mission
page; drive pose/mode and standalone arm diagnostics are absent from the
operator surface. PDIST80B shows only fields already carried by telemetry:
voltage, charge/discharge current, power, SOC, operating/protection flags,
RS485 state, and freshness. Missing protocol fields remain `정보 없음` and are
never read directly by the GUI or synthesized; see `docs/ui_data_gap.md`.

The **시연 화면 → 화면 표시** checkboxes independently control YOLO
box/class/confidence and target-distance text. They do not stop metadata
reception or alter target selection, and stale metadata still hides every
overlay after the measured-inference-aware 500 ms deadline. A valid target is
held for at most 350 ms across a brief empty detection frame to prevent visual
flicker; the value is never extended past the stale deadline. The work-camera preview keeps
the native 848:480 ratio and becomes the main view only after its explicit
`클릭하여 크게 보기` action. Display quality gates omit detections clipped on
three or more frame borders; the immutable raw metadata snapshot remains
available to diagnostics.

### Isolated telemetry replay

Before hardware integration, captured JSONL packets can be sent through the
same UDP decoders on ports isolated from live operation. Each line uses
`{"t":0.0,"channel":"power","payload":{...}}`; the payload is transmitted
unchanged.

```bash
/usr/bin/python3 -m operator_console.app \
  --host 127.0.0.1 --metadata-port 15003 --telemetry-port 15004 \
  --chassis-telemetry-port 15005 --arm-telemetry-port 15007 \
  --input-source REPLAY

/usr/bin/python3 tools/operator_console_replay.py capture.jsonl
```

The replay tool defaults to 15003/15004/15005/15007 and rejects the live UDP
ports. Recorded video or a GStreamer test sender must still use the existing
SRT receiver contract; no DEMO control is added to the operator UI.

Use the system interpreter explicitly: a conda-base `python3` has no GTK
bindings (`gi`) and dies with ModuleNotFoundError. The systemd unit already
pins `/usr/bin/python3`.

If startup fails with `no element "gtksink"`, install `gst-plugin-gtk` —
Arch split the GTK3 sink out of gst-plugins-good (bit us 2026-07-18).

The ops broker defaults to the same host on TCP `:9001`, with its token read
from `~/.config/powertrain/ops_console.token`. Override these independently
with `--ops-host`, `--ops-port`, and `--ops-token-file`. If the token file is
absent or empty, the command panel is disabled while every observation channel
continues normally.

For the integrated `python -m operator_console` entrypoint, the validated
`~/.config/powertrain/operator.json` keys include
`environment_telemetry_port` (default `5008`). Overrides must be integer ports
from 1 through 65535 and must match the robot-side sender configuration.

Observation remains caller/receiver-only: it never opens D435i/L515 or writes
CAN. Commands are possible only through the authenticated ops client and every
panel action passes state-revision revalidation plus its confirmation gesture.
