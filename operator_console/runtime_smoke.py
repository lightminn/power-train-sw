"""Launch the real console under Xvfb and exercise every LIVE panel path.

Pure-unit tests cannot catch Gtk-callback crashes (PyGObject swallows the
exception after printing a traceback), so panel refresh code that only runs
on LIVE data can rot silently — the 2026-07-18 `_rss` AttributeError shipped
exactly that way. This harness is the execution gate: it starts the installed
console with ephemeral receive ports, feeds maximal synthetic datagrams on
every UDP channel for a few seconds, then fails on any stderr traceback or
early exit. Run it (or the pytest wrapper) after every operator_console
change:

    xvfb-run 없이 직접:  /usr/bin/python3 -m operator_console.runtime_smoke
    (하니스가 스스로 xvfb-run -a 로 감싼다 — 사용자 화면에 창을 띄우지 않음)
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import shutil
import socket
import subprocess
import sys
import tempfile
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
SYSTEM_PYTHON = "/usr/bin/python3"
RUN_S = 6.0


def _free_udp_port() -> int:
    probe = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    probe.bind(("127.0.0.1", 0))
    port = probe.getsockname()[1]
    probe.close()
    return port


def _telemetry_payload(sequence: int) -> dict:
    return {
        "schema_version": 1, "sequence": sequence,
        "odometry_source": "wheel+imu", "x_m": 1.2, "y_m": -0.4,
        "yaw_rad": 0.3, "voltage_v": 47.6, "current_a": -0.3, "power_w": 14.0,
        "drive_state": "IDLE/OK", "can_state": "ERROR-ACTIVE",
        "pdist_soc_percent": 80, "pdist_battery_flags": 0,
        "pdist_protection_flags": 2, "pdist_charge_current_a": -0.1,
        "rs485_state": "OK", "rs485_consecutive_failures": 0,
        "rs485_detail": "pid 238",
        "unit_status": {"preflight": "SUCCESS"},
        "compose_status": {"powertrain_ros": "healthy"},
        "journal_tail": ["COMPONENT_MASK drive off"],
        "safety_status": "VALID", "safety_distance_mm": 812.0,
        "safety_estop_required": False, "safety_consecutive_failures": 0,
        "safety_detail": "",
    }


def _chassis_payload(sequence: int) -> dict:
    wheels = [
        {
            "name": name, "mode": "ARMED", "drive_turns_per_s": 0.5,
            "steer_deg": -3.2, "stale": name == "mid_left",
            "drive_axis_error": 16 if name == "rear_right" else 0,
            "steer_fault": 0,
        }
        for name in ("front_left", "front_right", "mid_left",
                     "mid_right", "rear_left", "rear_right")
    ]
    return {
        "schema_version": 1, "sequence": sequence,
        "odometry_source": "wheel+imu", "x_m": 0.8, "y_m": 0.1,
        "yaw_rad": -0.05, "voltage_v": None, "current_a": None,
        "power_w": None, "drive_state": "DRIVING/RUN",
        "can_state": "OK · error-active",
        "l515_state": "RUNNING", "l515_detail": "", "l515_mode": "srt",
        "l515_color_hz": 30.0, "l515_depth_hz": 30.0,
        "l515_submitted_hz": 29.8, "l515_sent_hz": 29.8, "l515_drop_hz": 0.0,
        "l515_ros_topic_rates_hz": {"/l515/depth": 29.9, "/l515/color": 30.1},
        "l515_aligned_depth_age_ms": 34.0,
        "l515_process_cpu_percent": 41.0,
        "l515_process_rss_bytes": 512 * 1024 * 1024,
        "safety_status": "VALID", "safety_distance_mm": 1450.0,
        "safety_estop_required": False, "safety_consecutive_failures": 0,
        "safety_detail": "", "wheel_count": 6, "wheel_fault_count": 0,
        "wheel_stale_count": 1, "wheel_axis_error_count": 1,
        "wheel_steer_fault_count": 0, "wheel_statuses": wheels,
        # drive OFF: MASK 배너·DISABLED 경로까지 실행시킨다.
        "component_mask": {"drive": False, "steer": True,
                           "us100": True, "robot_arm": True},
    }


def _metadata_payload(sequence: int) -> dict:
    theta = 0.6
    return {
        "schema_version": 1, "capture_sequence": sequence,
        "capture_stamp_ns": sequence * 10**9, "frame_width": 848,
        "frame_height": 480, "frame_id": "d435_color",
        "detections": [
            {"class_id": 3, "class_name": "relief_box", "confidence": 0.91,
             "bbox_xywh": [100, 120, 60, 40], "position_m": [0.1, -0.2, 0.8],
             "yaw_rad": theta, "is_pick_target": True},
            {"class_id": 1, "class_name": "door", "confidence": 0.44,
             "bbox_xywh": [300, 40, 120, 200], "position_m": None,
             "yaw_rad": -math.pi / 2, "is_pick_target": False},
        ],
    }


def _arm_payload(sequence: int) -> dict:
    return {
        "schema_version": 1, "sequence": sequence, "stamp_s": time.time(),
        "dynamixel": [
            {"id": 11, "position_raw": 3072, "position_deg": 90.0,
             "velocity": 5, "current": -12, "temperature_c": 34},
            # CRIT 온도로 경고 이벤트 경로까지 태운다.
            {"id": 12, "position_raw": 2048, "position_deg": 0.0,
             "velocity": 0, "current": 20, "temperature_c": 66},
        ],
        "joints": {"names": ["arm_joint_1", "arm_joint_2"],
                   "position_rad": [0.25, -0.5], "velocity": [0.0, 0.1]},
        "source_age_s": {"dynamixel": 0.1, "joints": 0.1, "detections": 0.2},
        "truncated": False,
    }


def run_smoke(run_s: float = RUN_S) -> tuple[bool, str]:
    """Return (passed, report). Never raises for a product failure."""
    if not Path(SYSTEM_PYTHON).exists():
        return False, f"system python missing: {SYSTEM_PYTHON}"
    if shutil.which("xvfb-run") is None:
        return False, "xvfb-run missing (pacman -S xorg-server-xvfb)"
    probe = subprocess.run(
        [SYSTEM_PYTHON, "-c", "import gi"], capture_output=True,
    )
    if probe.returncode != 0:
        return False, "system python has no gi (pacman -S python-gobject)"

    ports = {
        "metadata": _free_udp_port(), "telemetry": _free_udp_port(),
        "chassis": _free_udp_port(), "arm": _free_udp_port(),
    }
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="operator-console-smoke-token-",
        delete=False,
    ) as token_handle:
        token_handle.write("runtime-smoke-token")
        token_file = token_handle.name
    try:
        console = subprocess.Popen(
            [
                "xvfb-run", "-a", SYSTEM_PYTHON, "-m", "operator_console.app",
                "--host", "127.0.0.1",
                "--metadata-port", str(ports["metadata"]),
                "--telemetry-port", str(ports["telemetry"]),
                "--chassis-telemetry-port", str(ports["chassis"]),
                "--arm-telemetry-port", str(ports["arm"]),
                "--ops-token-file", token_file,
            ],
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
        )
    except BaseException:
        Path(token_file).unlink(missing_ok=True)
        raise
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    builders = {
        "telemetry": _telemetry_payload, "chassis": _chassis_payload,
        "metadata": _metadata_payload, "arm": _arm_payload,
    }
    try:
        deadline = time.monotonic() + run_s
        sequence = 0
        while time.monotonic() < deadline:
            if console.poll() is not None:
                break
            sequence += 1
            for channel, build in builders.items():
                sender.sendto(
                    json.dumps(build(sequence)).encode("utf-8"),
                    ("127.0.0.1", ports[channel]),
                )
            time.sleep(0.2)
        # phase 2 — 주입 중단: 전 패널 LIVE→STALE 전이 + 오버레이 숨김 경로.
        stale_deadline = time.monotonic() + 2.5
        while time.monotonic() < stale_deadline and console.poll() is None:
            time.sleep(0.2)
        # phase 3 — sparse/truncated 최소 페이로드: optional 필드 부재 분기.
        sequence += 1
        sparse = {
            "telemetry": {"schema_version": 1, "sequence": sequence},
            "chassis": {"schema_version": 1, "sequence": sequence,
                        "truncated": True, "wheel_count": 6},
            "metadata": {"schema_version": 1, "capture_sequence": sequence,
                         "frame_width": 848, "frame_height": 480,
                         "detections": []},
            "arm": {"schema_version": 1, "sequence": sequence,
                    "dynamixel": None, "joints": None},
        }
        for channel, payload in sparse.items():
            sender.sendto(
                json.dumps(payload).encode("utf-8"),
                ("127.0.0.1", ports[channel]),
            )
        sparse_deadline = time.monotonic() + 1.0
        while time.monotonic() < sparse_deadline and console.poll() is None:
            time.sleep(0.2)
        early_exit = console.poll() is not None
        if not early_exit:
            console.terminate()
        try:
            _, stderr = console.communicate(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(console.pid, 9)
            _, stderr = console.communicate()
    finally:
        sender.close()
        Path(token_file).unlink(missing_ok=True)
        if console.poll() is None:
            os.killpg(console.pid, 9)

    text = stderr.decode("utf-8", "replace")
    if early_exit:
        return False, f"console exited early (rc={console.returncode})\n{text}"
    if "Traceback" in text:
        return False, f"callback traceback detected:\n{text}"
    return True, f"PASS · {sequence} ticks on 4 channels, no tracebacks"


def main() -> int:
    passed, report = run_smoke()
    print(f"CONSOLE-RUNTIME-SMOKE: {'PASS' if passed else 'FAIL'}")
    print(report)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
