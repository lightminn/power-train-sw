"""Read-only CAD viewer fed exclusively by the console's received snapshots."""
from __future__ import annotations

from dataclasses import asdict
from functools import partial
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
import json
import math
import os
from pathlib import Path
import threading
import time
from urllib.parse import urlsplit

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gtk  # noqa: E402

from .telemetry import TelemetrySnapshot
from .arm_telemetry import ArmTelemetrySnapshot


ASSET_ROOT = Path(__file__).parent / "assets" / "rover3d"
WHEELS = frozenset({
    "front_left", "front_right", "mid_left", "mid_right", "rear_left", "rear_right",
})
SUSPENSION_JOINTS = frozenset({
    "ml_rocker_joint", "mr_rocker_joint", "bl_bogie_joint", "br_bogie_joint",
})
ARM_JOINT_LIMITS = {
    "arm_J1_yaw": (-math.tau, math.tau),
    "arm_J2_shoulder": (-math.pi, 0.0),
    "arm_J3_elbow": (-math.pi, 0.0),
    "arm4_rotation": (-math.radians(100), math.radians(100)),
    "axis5_rotation": (-math.pi, math.pi),
}

# Bench bridge joints -> CAD joints: (CAD name, sign, bridge angle at CAD zero).
# The arm URDF shares the CAD joint origins and axes, but the bridge calibration
# has its own zero; joints 2/3 count upward as positive (CAD: -pi..0) and
# joint 4 turns opposite to the CAD axis (operator-checked 2026-11-04).
# Zeros are the 2026-11-04 bench pose the operator matched to the CAD zero pose.
BRIDGE_TO_CAD = {
    "arm_joint_1": ("arm_J1_yaw", 1.0, -0.48627190975974827),
    "arm_joint_2": ("arm_J2_shoulder", -1.0, -0.4705181274331538),
    "arm_joint_3": ("arm_J3_elbow", -1.0, -0.21225130208615678),
    "arm_joint_4": ("arm4_rotation", -1.0, 0.0015339807878856412),
    "arm_joint_5": ("axis5_rotation", 1.0, -2.146039122252012),
}


def _finite(value):
    if value is None or isinstance(value, bool):
        return None
    value = float(value)
    return value if math.isfinite(value) else None


def rover_state(snapshot: TelemetrySnapshot | None, *, now_s=None) -> dict:
    """Keep unavailable, stale and measured zero distinct at the browser seam."""
    now_s = time.monotonic() if now_s is None else now_s
    age = None if snapshot is None else now_s - snapshot.received_monotonic_s
    fresh = age is not None and 0 <= age <= 1.0
    result = dict(live=fresh, roll_rad=None, pitch_rad=None, wheels={}, suspension={})
    if not fresh:
        return result
    result.update(roll_rad=_finite(snapshot.roll_rad), pitch_rad=_finite(snapshot.pitch_rad))
    for wheel in snapshot.wheel_statuses:
        if wheel.name not in WHEELS:
            continue
        row = asdict(wheel)
        for key in ("drive_turns_per_s", "command_turns_per_s", "steer_deg",
                    "drive_current_a", "steer_current_a"):
            row[key] = None if wheel.stale else _finite(row[key])
        result["wheels"][wheel.name] = row
    sensor_age = _finite(snapshot.suspension_age_ms)
    if sensor_age is not None and 0 <= sensor_age + age * 1000 <= 1000:
        result["suspension"] = {
            name: value for name, angle in snapshot.suspension_joints_rad
            if name in SUSPENSION_JOINTS and (value := _finite(angle)) is not None
        }
    return result


def arm_state(snapshot: ArmTelemetrySnapshot | None, *, now_s=None):
    """Accept CAD angles (or mapped bridge angles), with source age independent of chassis."""
    result = dict(arm_live=False, arm={}, tool_attached=False)
    if snapshot is None:
        return result
    now = time.monotonic() if now_s is None else now_s
    elapsed = now - snapshot.received_monotonic_s
    # Draw the gripper only while the bridge sees its actuators on the bus.
    runtime = snapshot.tool_runtime or {}
    tool = runtime.get("tool") or {}
    tool_age = runtime.get("source_age_s")
    result["tool_attached"] = bool(
        _finite(tool_age) is not None and math.isfinite(elapsed)
        and 0 <= tool_age + elapsed <= 3
        and tool.get("actuators_discovered") is True
        and not tool.get("tool_detached") and not tool.get("physical_tool_detached"))
    age = snapshot.joints_age_s
    if age is None or not math.isfinite(age) or not math.isfinite(elapsed) or age < 0 or elapsed < 0 or age + elapsed > 1:
        return result
    for name, angle in zip(snapshot.joint_names, snapshot.joint_position_rad):
        bounds = None
        if name in BRIDGE_TO_CAD:
            name, sign, zero = BRIDGE_TO_CAD[name]
            angle = sign * (angle - zero)
            bounds = ARM_JOINT_LIMITS[name]
            # The matched zero sits on the CAD limit; a real joint just past it
            # is drawn at the limit instead of dropping out of the view.
            if math.isfinite(angle):
                angle = min(max(angle, bounds[0]), bounds[1])
        bounds = bounds or ARM_JOINT_LIMITS.get(name)
        if bounds and math.isfinite(angle) and bounds[0] <= angle <= bounds[1]:
            result["arm"][name] = angle
    result["arm_live"] = bool(result["arm"])
    return result


class _AssetHandler(SimpleHTTPRequestHandler):
    """Serve only bundled viewer files on a process-local loopback port."""

    def do_GET(self):
        relative = urlsplit(self.path).path.lstrip("/")
        path = (ASSET_ROOT / relative).resolve()
        if not path.is_relative_to(ASSET_ROOT.resolve()) or not path.is_file():
            self.send_error(404)
            return
        super().do_GET()

    def log_message(self, *_args):
        pass


class Rover3DView(Gtk.Box):
    """Lazily start the CAD view inside the mobility motor card."""

    def __init__(self, *, input_source="LIVE", view_mode="rover"):
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        self.set_border_width(0)
        self.set_hexpand(True)
        self.set_vexpand(True)
        self.set_size_request(260, 140)
        self._input_source = input_source
        self._view_mode = "arm" if view_mode == "arm" else "rover"
        self._authority = "UNAVAILABLE"
        self._snapshot = None
        self._view = None
        self._server = None
        self._server_thread = None
        self._ready = False
        self._busy = False
        self._closed = False
        self._latest = rover_state(None)
        self._latest_arm = arm_state(None)
        self._last_sent = None
        self._last_sent_at = 0.0
        self._selected_wheel = None
        self._render_info = {}
        self._status = Gtk.Label(label="3D 모델 준비 중", xalign=0)
        self.pack_start(self._status, False, False, 0)
        self.connect("map", self._on_map)
        self.connect("unmap", self._on_unmap)
        self.connect("destroy", self._close)

    def _on_map(self, *_args):
        if self._view is None and not self._closed:
            self._start()
        if self._ready:
            self._last_sent = None
            self._send()

    def _start(self):
        try:
            try:
                gi.require_version("WebKit2", "4.1")
            except ValueError:
                # The Jetson host ships only the libsoup2 build (4.0); its API
                # is the same for this view.
                gi.require_version("WebKit2", "4.0")
            from gi.repository import WebKit2
            if not (ASSET_ROOT / "model.json").is_file():
                raise RuntimeError("3D 모델 파일이 없습니다")
            self._server = ThreadingHTTPServer(
                ("127.0.0.1", 0), partial(_AssetHandler, directory=str(ASSET_ROOT)),
            )
            self._server_thread = threading.Thread(
                target=self._server.serve_forever, name="rover-3d-assets", daemon=True,
            )
            self._server_thread.start()
            self._origin = f"http://127.0.0.1:{self._server.server_port}"
            # WebKitGTK 2.54's FTL JIT intermittently turns the viewer's
            # normal smoothing into NaN, rendering whole CAD parts black.
            # The web process inherits this environment when it is spawned.
            os.environ.setdefault("JSC_useFTLJIT", "false")
            context = WebKit2.WebContext.new_ephemeral()
            self._view = WebKit2.WebView.new_with_context(context)
            self._view.set_zoom_level(1.0)
            self._view.connect("notify::zoom-level", self._reset_page_zoom)
            self._view.connect("event", lambda _view, event:
                               event.type == Gdk.EventType.TOUCHPAD_PINCH)
            manager = self._view.get_user_content_manager()
            manager.register_script_message_handler("rover")
            manager.connect("script-message-received::rover", self._message)
            self._view.connect("decide-policy", self._navigation)
            self._view.connect("web-process-terminated", self._terminated)
            self.pack_start(self._view, True, True, 0)
            self._view.show()
            self._view.load_uri(self._origin + "/index.html?view=" + self._view_mode)
        except (ImportError, ValueError, RuntimeError, OSError) as exc:
            self._status.set_text(f"3D 표시를 시작할 수 없습니다: {exc}")

    @staticmethod
    def _reset_page_zoom(view, _property):
        # OrbitControls locks camera distance; WebKit page zoom is independent.
        if view.get_zoom_level() != 1.0:
            view.set_zoom_level(1.0)

    def _navigation(self, _view, decision, decision_type):
        from gi.repository import WebKit2
        if decision_type == WebKit2.PolicyDecisionType.NAVIGATION_ACTION:
            uri = decision.get_navigation_action().get_request().get_uri()
            if not uri.startswith(self._origin + "/"):
                decision.ignore()
                return True
        return False

    def _message(self, _manager, message):
        try:
            payload = json.loads(message.get_js_value().to_string())
        except (ValueError, TypeError):
            return
        if not isinstance(payload, dict):
            return
        if payload.get("type") == "ready":
            self._ready = True
            self._render_info = payload
            self._status.set_text(
                "더미 엔코더 시연 · 실제 로봇 미연결" if self._input_source == "TEST"
                else "드래그로 360° 회전 · 바퀴를 눌러 모터 상태 확인")
            self.set_tooltip_text(self._status.get_text())
            self._status.set_no_show_all(True)
            self._status.hide()
            if self.get_mapped():
                self._send()
            else:
                self._on_unmap()
        elif payload.get("type") == "wheel" and payload.get("wheel") in WHEELS:
            self._selected_wheel = payload["wheel"]
        elif payload.get("type") == "error":
            self._status.set_text("3D 모델 표시 오류 · " + str(payload.get("message", ""))[:160])
            self._status.show()

    def _terminated(self, *_args):
        self._ready = False
        self._status.set_text("3D 표시가 종료됐습니다. 콘솔을 다시 열어주세요")
        self._status.show()

    def set_state(self, snapshot, _ops_state=None, *, now_s=None):
        self._snapshot = snapshot
        self._authority = "UNAVAILABLE" if _ops_state is None else str(_ops_state.get("authority_mode", "UNAVAILABLE"))
        self._latest = rover_state(snapshot, now_s=now_s)
        if self._ready and self.get_mapped():
            self._send()

    def _send(self):
        if self._busy or self._closed or not self._ready:
            return
        arm = dict(self._latest_arm)
        if self._input_source == "TEST":
            arm["tool_attached"] = True  # the dummy demo has no tool feed
        state = json.dumps(dict(self._latest, **arm, input_source=self._input_source), allow_nan=False, separators=(",", ":"))
        now = time.monotonic()
        if state == self._last_sent and now-self._last_sent_at < 0.5:
            return
        self._last_sent = state
        self._last_sent_at = now
        self._busy = True
        script = f"window.rover3d.receive({state}); window.rover3d.setActive(true);"

        def complete(view, result, _data):
            self._busy = False
            if self._closed:
                return
            try:
                view.evaluate_javascript_finish(result)
            except Exception as exc:
                self._status.set_text("3D 데이터 표시 오류 · " + str(exc)[:120])

        self._view.evaluate_javascript(script, -1, None, None, None, complete, None)

    def set_arm_state(self, snapshot, *, now_s=None):
        self._latest_arm = arm_state(snapshot, now_s=now_s)
        if self._ready and self.get_mapped():
            self._send()

    def _on_unmap(self, *_args):
        if self._ready and not self._closed:
            self._view.evaluate_javascript("window.rover3d.setActive(false)", -1, None, None, None, None, None)

    def _close(self, *_args):
        self._closed = True
        if self._server is not None:
            self._server.shutdown()
            self._server.server_close()
            self._server = None
        if self._server_thread is not None:
            self._server_thread.join(timeout=1)
