"""Subprocess scenarios for the actual GTK/TCP/SRT regression gate.

Only FakePadBoundary and the broker's hardware executor are fixtures. Production
SessionServer, ManualDriveService, OpsBrokerCore and both clients use loopback TCP.
No physical inputs, camera SDKs or robot services are opened.
"""
from contextlib import contextmanager
from dataclasses import replace
import json
import os
from pathlib import Path
import socket
import socketserver
import subprocess
import sys
import tempfile
import threading
import time

from operator_console.app import OperatorConsole, VideoPanel, Gtk, Gst, GLib, Gdk
from operator_console.operation_runtime import OperationRuntime
from powertrain_runtime.tests.test_integrated_operation import BrokerHandler, FakePadBoundary
from powertrain_runtime.drive_service import ManualDriveService
from powertrain_runtime.session import SessionServer
from powertrain_ros.ops_broker_core import OpsBrokerCore, OpsState
from motor_control.laptop.ops_channel_client import OpsChannelClient


def pump_until(predicate, label, timeout=10):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        while GLib.MainContext.default().pending():
            GLib.MainContext.default().iteration(False)
        if predicate():
            return
        time.sleep(.01)
    raise AssertionError(label)


def pump(seconds=.2):
    deadline = time.monotonic() + seconds
    pump_until(lambda: time.monotonic() >= deadline, "GTK settling", seconds + 1)


def udp_port():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.bind(("127.0.0.1", 0))
        return sock.getsockname()[1]


def capture(window, name):
    directory = os.environ.get("PR4_SCREENSHOT_DIR")
    if not directory:
        return
    path = Path(directory)
    path.mkdir(parents=True, exist_ok=True)
    window.present()
    pump(.3)
    surface = window.get_window()
    _, x, y = surface.get_origin()
    pixels = Gdk.pixbuf_get_from_window(Gdk.get_default_root_window(), x, y,
                                       surface.get_width(), surface.get_height())
    assert pixels is not None and any(pixels.get_pixels())
    pixels.savev(str(path / name), "png", [], [])


@contextmanager
def srt_sender():
    """Actual test video with production parser/mux/SRT sender options."""
    port = udp_port()
    sender = subprocess.Popen([
        "gst-launch-1.0", "-q", "videotestsrc", "is-live=true", "!",
        "video/x-raw,width=320,height=240,framerate=15/1", "!", "x264enc",
        "tune=zerolatency", "speed-preset=ultrafast", "key-int-max=15", "!",
        "h264parse", "config-interval=-1", "!", "mpegtsmux", "alignment=7", "!",
        "srtsink", f"uri=srt://127.0.0.1:{port}?mode=listener",
        "wait-for-connection=false", "sync=false", "async=false",
    ], stdout=subprocess.DEVNULL, stderr=subprocess.PIPE)
    try:
        yield port, sender
    finally:
        sender.terminate()
        try:
            _, err = sender.communicate(timeout=4)
        except subprocess.TimeoutExpired:
            sender.kill()
            _, err = sender.communicate(timeout=3)
        assert sender.poll() is not None, "SRT sender was not reaped"
        if err:
            print("sender stderr: " + err.decode(errors="replace"), file=sys.stderr)


def decoded(panel, sender, label):
    def fresh():
        assert sender.poll() is None, "SRT sender exited unexpectedly"
        return panel.health_state() == "LIVE" and bool(panel.last_fps)
    try:
        pump_until(fresh, label, 15)
    except AssertionError as error:
        raise AssertionError(f"{label}: {panel.health_state()}, "
                             f"{panel._detail.get_text()}") from error
    print(f"{label}: LIVE, fps={panel.last_fps}, age={panel.last_frame_age_s}", flush=True)


def video_restart(scenario):
    with srt_sender() as (port, sender):
        window = Gtk.Window()
        events = []
        panel = VideoPanel("전방 카메라", "127.0.0.1", port, 60,
                           event_sink=lambda *event: events.append(event))
        window.add(panel)
        window.show_all()
        try:
            decoded(panel, sender, "initial decoded video")
            original_pipeline, original_widget = panel._pipeline, panel._video_widget
            previous_frame = panel._last_frame_monotonic
            if scenario == "endpoint":
                panel.set_endpoint("127.0.0.1")
                assert panel.last_frame_age_s is None, "old session frame was retained"
            else:
                # A real EOS bus message exercises the existing automatic retry path.
                panel._pipeline.send_event(Gst.Event.new_eos())
                pump_until(lambda: panel._reconnects > 0, "EOS schedules automatic retry")
                pump_until(lambda: panel._retry_source_id is None, "retry timer executes")
            decoded(panel, sender, f"{scenario} restarted decoded video")
            assert panel._last_frame_monotonic > previous_frame
            assert panel._pipeline is original_pipeline
            assert panel._video_widget is original_widget
            assert not any("not-linked" in message for _, message in events), events
            return dict(scenario=scenario, video="LIVE", fps=panel.last_fps)
        finally:
            panel.stop()
            window.destroy()


@contextmanager
def robot_fixture(directory):
    token = directory / "console.token"
    token.write_text("local-e2e-fixture-token")
    broker = socketserver.ThreadingTCPServer(("127.0.0.1", 0), BrokerHandler)
    broker.daemon_threads = False
    broker.lock, broker.done, broker.actions = threading.Lock(), threading.Event(), []
    broker.state = OpsState(
        revision=1, authority_mode="ESTOP", chassis_mode="ESTOP", gateway_state="ESTOP",
        gateway_input_fresh=True, gateway_neutral=True, estop_latched=True,
        active_estop_sources=(), wheels_stopped=True,
        field_age_s=dict(authority=0., gateway=0., safety=0., wheels=0.))
    def execute(action):
        changes = {
            "estop": dict(estop_latched=True, chassis_mode="ESTOP", authority_mode="ESTOP", gateway_state="ESTOP"),
            "estop_reset": dict(estop_latched=False, chassis_mode="IDLE", authority_mode="IDLE", gateway_state="DRIVE", gateway_input_fresh=True),
            "authority_manual": dict(authority_mode="TELEOP"),
            "arm": dict(chassis_mode="ARMED"),
            "disarm": dict(chassis_mode="IDLE"),
            "authority_idle": dict(authority_mode="IDLE"),
        }
        broker.state = replace(broker.state, revision=broker.state.revision + 1, **changes[action])
    broker.execute = execute
    broker.core = OpsBrokerCore({token.read_text(): "console"}, clock=time.monotonic,
                                state_provider=lambda: broker.state)
    thread = threading.Thread(target=broker.serve_forever)
    thread.start()
    drive = ManualDriveService(lambda: OpsChannelClient("127.0.0.1", broker.server_address[1], token.read_text()), timeout_s=1)
    server = SessionServer(dict(
        robot_id="fixture", token_file=str(token), host="127.0.0.1", session_port=0,
        input_port=0, ops_port=0, ops_target_port=broker.server_address[1],
        input_target_port=broker.server_address[1], destination_file=str(directory / "session.json"),
        lease_timeout_s=1.), drive.start, drive.stop, drive.snapshot).start()
    runtime = OperationRuntime(dict(robot_id="fixture", token_file=str(token), hosts=["127.0.0.1"],
                                    session_port=server.session_port), supervisor=FakePadBoundary())
    ports = [broker.server_address[1]] + [sock.getsockname()[1] for sock in server._listeners]
    try:
        yield token, broker, drive, server, runtime
    finally:
        runtime.close()
        server.close()
        drive.close()
        broker.done.set()
        broker.shutdown()
        broker.server_close()
        thread.join(timeout=3)
        assert not thread.is_alive(), "broker thread survived cleanup"
        assert not any(thread.is_alive() for thread in server._threads), "session thread survived cleanup"
        for port in ports:
            with socket.socket() as sock:
                sock.settimeout(.2)
                assert sock.connect_ex(("127.0.0.1", port)) != 0, f"TCP listener {port} survived cleanup"


def console_operation():
    with tempfile.TemporaryDirectory(prefix="console-operation-e2e-") as directory:
        with robot_fixture(Path(directory)) as (token, broker, drive, server, runtime):
            with srt_sender() as (video_port, sender):
                window = OperatorConsole(
                    "127.0.0.1", udp_port(), video_port, udp_port(), 60, udp_port(), udp_port(),
                    arm_telemetry_port=udp_port(), environment_telemetry_port=udp_port(),
                    ops_token_file=str(token), operation_runtime=runtime)
                window.set_decorated(False)
                window.resize(1366, 768)
                window.move(0, 0)
                window.show_all()
                try:
                    pump_until(lambda: runtime.snapshot().get("connected") and window._global_estop.get_sensitive(), "ops authenticated")
                    assert broker.actions == []
                    assert not window._operation_panel.start_button.get_sensitive()
                    window._ops_settings_button.clicked()
                    pump()
                    reset = window._ops_panel._action_buttons["estop_reset"]
                    assert reset.get_mapped(), "visible recovery action is unreachable"
                    capture(window._ops_settings_window, "recovery-real-session.png")
                    assert broker.actions == [], "opening recovery sent a command"
                    reset.clicked()
                    pump(.05)
                    assert window._ops_panel._confirm_button.get_mapped()
                    window._ops_panel._confirm_button.clicked()
                    pump_until(lambda: broker.actions == ["estop_reset"] and broker.state.chassis_mode == "IDLE", "explicit confirmed reset")
                    window._ops_settings_window.hide()
                    decoded(window._l515, sender, "ready decoded video")
                    pump_until(lambda: window._operation_panel.start_button.get_sensitive(), "video and pad ready")
                    assert broker.actions == ["estop_reset"], "recovery automatically armed"
                    capture(window, "mission-real-srt-ready.png")
                    button = window._operation_panel.start_button
                    button.emit("pressed")
                    pump(.3)
                    assert broker.actions == ["estop_reset"], "short hold armed the chassis"
                    pump_until(lambda: drive.snapshot()["status"] == "SUCCEEDED", "held start succeeded")
                    button.emit("released")
                    assert broker.actions == ["estop_reset", "authority_manual", "arm"], broker.actions
                    assert broker.state.chassis_mode == "ARMED"
                    capture(window, "mission-real-srt-armed.png")
                    window._operation_panel.stop_button.clicked()
                    pump_until(lambda: drive.snapshot()["action"] == "stop" and drive.snapshot()["status"] == "SUCCEEDED", "stop succeeded")
                    assert broker.actions[-2:] == ["disarm", "authority_idle"]
                    assert broker.state.chassis_mode == "IDLE" and broker.state.authority_mode == "IDLE"
                    window._global_estop.clicked()
                    pump_until(lambda: broker.state.estop_latched, "global ESTOP")
                    window._ops_settings_button.clicked()
                    pump(.15)
                    reset.clicked()
                    pump(.05)
                    window._ops_panel._confirm_button.clicked()
                    pump_until(lambda: broker.actions.count("estop_reset") == 2 and not broker.state.estop_latched, "second explicit reset")
                    window._ops_settings_window.hide()
                    generation = runtime.snapshot()["session_generation"]
                    previous_frame = window._l515._last_frame_monotonic
                    with server._lock:
                        server._revoke()
                    pump_until(lambda: runtime.snapshot()["session_generation"] > generation, "session reconnect")
                    pump_until(
                        lambda: window._operation_session[0]
                        and window._operation_session[2] > generation,
                        "GTK applied the new authenticated video endpoint")
                    decoded(window._l515, sender, "decoded video after session reconnect")
                    assert window._l515._last_frame_monotonic > previous_frame
                    pump(.3)
                    assert broker.actions.count("arm") == 1, broker.actions
                    assert broker.state.chassis_mode == "IDLE"
                    return dict(scenario="operation", actions=list(broker.actions), video="LIVE",
                                session_generation=runtime.snapshot()["session_generation"])
                finally:
                    window.destroy()


def main():
    Gst.init(None)
    scenario = sys.argv[1]
    result = console_operation() if scenario == "operation" else video_restart(scenario)
    # Reached only after every assertion and resource context's cleanup succeeded.
    result.update(result="PASS", cleanup="sender reaped; TCP listeners closed")
    print(json.dumps(result), flush=True)


if __name__ == "__main__":
    main()
