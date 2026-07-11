# L515 Gateway Dashboard

The Gateway is the only process allowed to open the powertrain L515. The Dashboard is a
socket-only Textual client: closing SSH, pressing `q`, receiving SIGHUP, or crashing the client
does not stop camera capture, ROS publication, or SRT.

The local control endpoint is the Linux abstract Unix socket `@powertrain-l515-gateway`.
It creates no socket pathname, and the Gateway accepts only same-UID peers using `SO_PEERCRED`.

Run the managed Gateway in `powertrain_ros`, then attach any number of dashboards. On a fresh
checkout the container entrypoint builds the three ROS packages before starting the Gateway; it
also rebuilds when a source file is newer than the installed workspace:

```bash
sudo bash scripts/install_powertrain_runtime_dir.sh  # one-time host install
docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros
docker exec -it powertrain_ros python3 -m l515_dashboard
```

The installer writes `/etc/tmpfiles.d/powertrain-gateway.conf`, creates and verifies root-owned
mode 0750 `/run/powertrain`, and is idempotent. The installed tmpfiles rule recreates this volatile
runtime directory after every reboot. Compose deliberately sets `create_host_path: false`; if the
one-time install was skipped or provisioning is wrong, startup fails instead of creating mode 0755.

Keys: `1` RGB, `2` aligned depth, `3` overlay, `s` streaming on/off, `r` restart Gateway
components, and `q` detach the Dashboard. `Shift+Q` displays a confirmation; `y` sends the
destructive `stop_gateway` command and the managed Gateway reaps its SRT child.

The container uses `restart: on-failure:5` to bound consecutive short startup failures to five
retries. Docker resets that retry counter after roughly 10 seconds of healthy runtime, so a later
crash gets a new retry budget. An acknowledged `Shift+Q` produces exit status 0 and stays stopped.
Start it again explicitly with `docker compose ... up -d powertrain_ros`.

Receive the fixed 1280×720×30 stream on the driving laptop:

```bash
gst-launch-1.0 srtsrc uri="srt://JETSON_IP:5000?mode=caller&latency=60" ! \
  tsdemux ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

Orin Nano has no NVENC encoder. The deployed path intentionally uses software
`videoconvert → x264enc` (`ultrafast`, `zerolatency`, three threads); the image includes
GStreamer base/good/bad/ugly plugins. Status reports five-second native SDK callback rates,
ROS rates for all six topics, SRT submit/sent/drop rates, aligned-Depth age, and Gateway CPU/RSS.

If startup reports a singleton/lock failure, do not delete the persistent
`/run/powertrain/l515-gateway.lock` file or kill an unknown process. A stale file is normal;
only the held `flock` denotes ownership. Check the existing `l515_gateway` owner and container state first. The Gateway excludes direct
RealSense viewers and maintenance scripts while it owns the camera; stop it explicitly before
approved SDK maintenance. It never opens the robot-arm D435i.

The Jetson compose service bind-mounts host `/run/powertrain` at the same container path and uses
host networking. Both the persistent flock inode and abstract endpoint are therefore shared across
replacement containers. Gateway acquires/binds both before opening the L515 SDK.
