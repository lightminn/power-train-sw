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
            environment_telemetry_port=ports[6],
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

        def save_frame(filename, continuation):
            from gi.repository import WebKit2
            output = Path(__file__).resolve().parents[1] / 'docs/ui_screenshots' / filename
            def complete(view, result, _data):
                try:
                    view.get_snapshot_finish(result).write_to_png(str(output))
                    continuation()
                except Exception as exc:
                    finish(exc)
            widget._view.get_snapshot(WebKit2.SnapshotRegion.VISIBLE,
                WebKit2.SnapshotOptions.NONE, None, complete, None)

        def finish(error=None):
            nonlocal passed, phase
            passed = error is None
            if error:
                report.append(dict(error=str(error)))
            phase = "done"
            console.destroy()

        def evaluate(script, callback):
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

            widget._view.evaluate_javascript(script, -1, None, None, None, complete, None)

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
            phase = "inject"

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
            assert not value["live"], value
            assert "3.50 A" not in value["popup"] and "미연결" in value["popup"], value
            report.append(dict(stale=value))
            finish()

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
