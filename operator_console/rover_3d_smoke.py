"""Exercise the actual console UDP receiver, CAD view and motor popup in isolation.

Run: xvfb-run -a /usr/bin/python3 -m operator_console.rover_3d_smoke
All receiver ports are ephemeral. No device or ops command channel is opened.
"""
import json
import math
import os
from pathlib import Path
import socket
import sys
import tempfile
import time

from .app import OperatorConsole, Gtk, Gst, GLib
from .rover_3d_demo import dummy_payload, dummy_arm_payload


def main():
    Gst.init(None)
    ports = []
    for _ in range(8):
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as channel:
            channel.bind(("127.0.0.1", 0))
            ports.append(channel.getsockname()[1])
    with tempfile.TemporaryDirectory(prefix="rover3d-smoke-") as tmp:
        console = OperatorConsole(
            "127.0.0.1", latency_ms=60, l515_port=ports[0], d435_port=ports[1],
            metadata_port=ports[2], telemetry_port=ports[3],
            chassis_telemetry_port=ports[4], arm_telemetry_port=ports[5],
            environment_telemetry_port=ports[6], terrain_port=ports[7], terrain_map_port=0,
            ops_token_file=str(Path(tmp) / "missing-token"), input_source="TEST",
        )
        console.show_all()
        console.resize(1100, 680)
        console._stack.set_visible_child_name("mobility")
        widget = console._mobility._rover_graphic
        assert console._stack.get_child_by_name("rover3d") is None
        assert widget.is_ancestor(console._mobility._motor_card)
        sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        phase = "loading"
        started = time.monotonic()
        baseline = None
        first_pose = None
        sent_at = None
        sequence = 0
        busy = False
        passed = False
        report = []
        arm_first = None
        arm_second = None

        def save_frame(filename, continuation, view_widget=None):
            from gi.repository import WebKit2
            output = Path(__file__).resolve().parents[1] / 'docs/ui_screenshots' / filename
            def complete(view, result, _data):
                try:
                    view.get_snapshot_finish(result).write_to_png(str(output))
                    continuation()
                except Exception as exc:
                    finish(exc)
            (view_widget or widget)._view.get_snapshot(WebKit2.SnapshotRegion.VISIBLE,
                WebKit2.SnapshotOptions.NONE, None, complete, None)

        def finish(error=None):
            nonlocal passed, phase
            passed = error is None
            if error:
                report.append(dict(error=str(error)))
            phase = "done"
            console.destroy()

        def evaluate(script, callback, view_widget=None):
            nonlocal busy
            busy = True

            def complete(view, result, _data):
                nonlocal busy
                busy = False
                try:
                    value = json.loads(view.evaluate_javascript_finish(result).to_string())
                    callback(value)
                except Exception as exc:
                    finish(exc)

            (view_widget or widget)._view.evaluate_javascript(script, -1, None, None, None, complete, None)

        def have_baseline(value):
            nonlocal baseline, phase
            adjustment = console._mobility_scroll.get_vadjustment()
            assert adjustment.get_upper() <= adjustment.get_page_size() + 1, dict(content=adjustment.get_upper(), page=adjustment.get_page_size())
            assert console.get_size().width <= 1100 and console.get_size().height <= 680, tuple(console.get_size())
            report.append(dict(layout_1100x680=dict(page=adjustment.get_page_size(), content=adjustment.get_upper())))
            baseline = value
            assert len(value["wheels"]) == 6
            assert value["modelKind"] == "cad-lod" and value["displayMeshes"] >= 15
            assert value["triangles"] < 300000
            assert value["zoomEnabled"] is False and value["panEnabled"] is False
            assert not value["live"]
            check_page_zoom(widget)
            phase = "inject"

        def check_page_zoom(view_widget):
            from gi.repository import Gdk
            view = view_widget._view
            for zoom in (.5, 2.0):
                view.set_zoom_level(zoom)
                assert view.get_zoom_level() == 1.0, ("WebKit zoom changed", zoom, view.get_zoom_level())
            assert view.emit("event", Gdk.Event.new(Gdk.EventType.TOUCHPAD_PINCH))
            report.append(dict(page_zoom_locked=view_widget._view_mode))

        def have_live(value):
            nonlocal phase, sent_at, first_pose
            assert value["live"] and value["selected"] == "front_left", value
            assert "3.50 A" in value["popup"] and "12 rpm" in value["popup"], value
            q = value["joints"]["ml_rocker_joint"]
            assert abs(q[1] - math.sin(0.2 / 2)) < 0.00001, q
            assert math.dist(baseline["frontLeft"], value["frontLeft"]) > 0.01
            report.append(dict(live=value))
            first_pose = value
            phase = "capture_first"
            def proceed():
                nonlocal phase, sent_at
                phase = "inject_second"
                sent_at = time.monotonic()
            save_frame('2026-10-15-mobility-encoder-a.png', proceed)

        def have_second(value):
            nonlocal phase, sent_at
            assert value["live"], value
            q = value["joints"]["ml_rocker_joint"]
            assert abs(q[1] - math.sin(-0.2 / 2)) < 0.00001, q
            distance = math.dist(first_pose["frontLeft"], value["frontLeft"])
            assert distance > 0.03, distance
            report.append(dict(second_pose=value, wheel_displacement_m=distance))
            phase = "capture_second"
            def proceed():
                nonlocal phase, sent_at
                phase = "stale"
                sent_at = time.monotonic()
            save_frame('2026-10-15-mobility-encoder-b.png', proceed)

        def have_stale(value):
            nonlocal phase, sent_at
            assert not value["live"], value
            assert "3.50 A" not in value["popup"] and "미연결" in value["popup"], value
            report.append(dict(stale=value))
            phase = "arm_a"
            sent_at = time.monotonic()

        def have_arm_first(value):
            nonlocal arm_first, phase, sent_at
            assert value["armLive"] and value["live"], value
            assert len(value["armJoints"]) == 5
            assert value["armJoints"]["arm_J2_shoulder"][1] > .1
            arm_first = value
            report.append(dict(arm_first=value))
            phase = "arm_b"
            sent_at = time.monotonic()
            save_frame('2026-10-16-arm-pose-a.png', lambda: None)

        def have_arm_second(value):
            nonlocal arm_second, phase, sent_at
            assert value["armLive"], value
            displacement = math.dist(arm_first["armTip"], value["armTip"])
            assert displacement > .02, displacement
            assert math.dist(arm_first["frontLeft"], value["frontLeft"]) < .0001
            arm_second = value
            report.append(dict(arm_second=value, arm_tip_displacement_m=displacement))
            def checks_complete(checks):
                interaction = checks["interaction"]
                assert interaction["fixedDistance"] and interaction["twoFingerDisabled"]
                assert interaction["pageWheelBlocked"] and interaction["pageKeyBlocked"] and interaction["pinchBlocked"]
                assert all(abs(interaction[k] - interaction["zoomBefore"]) < 1e-9 for k in ("zoomAfter", "zoomOutAfter", "ctrlWheelAfter"))
                assert checks["fontLoaded"] and "NoonnuBasicGothic" in checks["fontFamily"]
                report.append(dict(view_checks=checks))
            evaluate("JSON.stringify({performance:window.rover3d.performanceCheck(),interaction:window.rover3d.interactionCheck(),fontLoaded:document.fonts.check('15px NoonnuBasicGothic'),fontFamily:getComputedStyle(document.body).fontFamily})", checks_complete)
            phase = "arm_stale"
            sent_at = time.monotonic()
            save_frame('2026-10-16-arm-pose-b.png', lambda: None)

        def have_arm_stale(value):
            nonlocal phase, sent_at
            assert value["live"] and not value["armLive"], value
            assert math.dist(arm_second["armTip"], value["armTip"]) < .0001
            report.append(dict(arm_stale=value))
            console._stack.set_visible_child_name("arm_manual")
            phase = "arm_preview"
            sent_at = time.monotonic()

        def have_arm_preview(value):
            assert value["viewMode"] == "arm" and value["armLive"] and value["wheels"] == [], value
            assert value["zoomEnabled"] is False and value["panEnabled"] is False
            check_page_zoom(console._arm_pose)
            report.append(dict(arm_preview=value))
            save_frame('2026-10-16-arm-only.png', finish, console._arm_pose)

        def progress():
            nonlocal sequence, sent_at, phase
            now = time.monotonic()
            if phase == "done":
                return False
            if now - started > 45:
                finish("CAD/UDP/popup smoke timed out: " + phase)
                return False
            if busy:
                return True
            if phase == "loading" and widget._ready:
                evaluate("JSON.stringify(window.rover3d.inspect())", have_baseline)
            elif phase in ("inject", "inject_second"):
                sequence += 1
                payload = dict(
                    schema_version=1, sequence=sequence, roll_rad=0.08, pitch_rad=0.03,
                    wheel_statuses=[dict(name=name, mode="ARMED", drive_turns_per_s=0.2,
                        command_turns_per_s=0.2, steer_deg=12, drive_current_a=3.5,
                        steer_current_a=0.4, stale=False, drive_axis_error=0, steer_fault=0)
                        for name in ("front_left", "front_right", "mid_left", "mid_right", "rear_left", "rear_right")],
                    suspension_joints_rad=dict(ml_rocker_joint=0.2, mr_rocker_joint=-0.1,
                        bl_bogie_joint=-0.15, br_bogie_joint=0.1), suspension_age_ms=10,
                )
                if phase == "inject_second":
                    payload['suspension_joints_rad'] = dict(ml_rocker_joint=-0.2,
                        mr_rocker_joint=0.15, bl_bogie_joint=0.2, br_bogie_joint=-0.15)
                sender.sendto(json.dumps(payload).encode(), ("127.0.0.1", ports[4]))
                if sent_at is None:
                    sent_at = now
                if now - sent_at > 0.8:
                    if phase == "inject":
                        evaluate("window.rover3d.clickWheel('front_left'); JSON.stringify(window.rover3d.inspect())", have_live)
                    else:
                        evaluate("JSON.stringify(window.rover3d.inspect())", have_second)
            elif phase == "stale" and now - sent_at > 1.8:
                evaluate("JSON.stringify(window.rover3d.inspect())", have_stale)
            elif phase.startswith("arm_"):
                sequence += 1
                sender.sendto(json.dumps(dummy_payload(sequence, 0)).encode(), ("127.0.0.1", ports[4]))
                if phase != "arm_stale":
                    sender.sendto(json.dumps(dummy_arm_payload(sequence, 0 if phase == "arm_a" else 2)).encode(), ("127.0.0.1", ports[5]))
                if now - sent_at > (1.8 if phase == "arm_stale" else 1.2):
                    if phase == "arm_a":
                        evaluate("JSON.stringify(window.rover3d.inspect())", have_arm_first)
                    elif phase == "arm_b":
                        evaluate("JSON.stringify(window.rover3d.inspect())", have_arm_second)
                    elif phase == "arm_stale":
                        evaluate("JSON.stringify(window.rover3d.inspect())", have_arm_stale)
                    elif console._arm_pose._ready:
                        evaluate("JSON.stringify(window.rover3d.inspect())", have_arm_preview, console._arm_pose)
            return True

        GLib.timeout_add(100, progress)
        Gtk.main()
        sender.close()
        print(json.dumps(dict(passed=passed, checks=report), ensure_ascii=False, indent=2))
        return 0 if passed else 1


if __name__ == "__main__":
    result = main()
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(result)
