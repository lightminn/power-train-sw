# Operator Console

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

```bash
python3 -m operator_console.app --host 192.168.8.106
```

The ops broker defaults to the same host on TCP `:9001`, with its token read
from `~/.config/powertrain/ops_console.token`. Override these independently
with `--ops-host`, `--ops-port`, and `--ops-token-file`. If the token file is
absent or empty, the command panel is disabled while every observation channel
continues normally.

Observation remains caller/receiver-only: it never opens D435i/L515 or writes
CAN. Commands are possible only through the authenticated ops client and every
panel action passes state-revision revalidation plus its confirmation gesture.
