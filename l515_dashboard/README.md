# L515 Gateway Dashboard

The Gateway is the only process allowed to open the powertrain L515. The Dashboard is a
socket-only Textual client: closing SSH, pressing `q`, receiving SIGHUP, or crashing the client
does not stop camera capture, ROS publication, or SRT.

Run the managed Gateway in `powertrain_ros`, then attach any number of dashboards. On a fresh
checkout the container entrypoint builds the three ROS packages before starting the Gateway; it
also rebuilds when a source file is newer than the installed workspace:

```bash
docker compose -f docker/docker-compose.jetson.yml up -d powertrain_ros
docker exec -it powertrain_ros python3 -m l515_dashboard
```

Keys: `1` RGB, `2` aligned depth, `3` overlay, `s` streaming on/off, `r` restart Gateway
components, and `q` detach the Dashboard. `Shift+Q` displays a confirmation; `y` sends the
destructive `stop_gateway` command and the managed Gateway reaps its SRT child.

The container uses `restart: on-failure:5`: a crash gets at most five automatic recovery attempts,
while an acknowledged `Shift+Q` produces exit status 0 and stays stopped. Start it again explicitly
with `docker compose ... up -d powertrain_ros`.

Receive the fixed 1280×720×30 stream on the driving laptop:

```bash
gst-launch-1.0 srtsrc uri="srt://JETSON_IP:5000?mode=caller&latency=60" ! \
  tsdemux ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false
```

If startup reports a singleton/lock failure, do not delete the lock or kill an unknown process.
Check the existing `l515_gateway` owner and container state first. The Gateway excludes direct
RealSense viewers and maintenance scripts while it owns the camera; stop it explicitly before
approved SDK maintenance. It never opens the robot-arm D435i.
