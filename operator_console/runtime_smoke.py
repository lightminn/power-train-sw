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
    (하니스가 WAYLAND_DISPLAY 를 제거하고 GDK_BACKEND=x11 로 고정한 뒤
    xvfb-run -a 로 감싸므로 사용자 화면에 창을 띄우지 않는다.)
"""
from __future__ import annotations

import json
import math
import os
from pathlib import Path
import shutil
import signal
import socket
import subprocess
import sys
import tempfile
import threading
import time


REPO_ROOT = Path(__file__).resolve().parents[1]
SYSTEM_PYTHON = "/usr/bin/python3"
RUN_S = 6.0
# xvfb + Gtk + GStreamer 기동. 부하가 높으면 느려지므로 넉넉히 준다.
STARTUP_TIMEOUT_S = 40.0

# 이 패널들은 주입 중 LIVE 에 도달하고, 주입을 끊으면 STALE 로 전이해야 한다.
# 하나라도 못 하면 그 채널은 실제로는 죽어 있는 것이다.
REQUIRED_PANELS = frozenset({
    "telemetry", "chassis", "metadata", "arm", "environment",
})


def smoke_child_env(
    base: dict[str, str] | None = None,
) -> dict[str, str]:
    """Force the console child onto X11 so Xvfb actually isolates it."""
    env = dict(os.environ if base is None else base)
    env.pop("WAYLAND_DISPLAY", None)
    env["GDK_BACKEND"] = "x11"
    return env


def _console_child_pid(group_pid: int) -> int | None:
    """xvfb-run 래퍼가 감싼 실제 콘솔 프로세스의 pid.

    래퍼에 신호를 보내면 콘솔의 종료 코드가 가려져 종료 경로 결함을 못 본다.
    """
    result = subprocess.run(
        ["pgrep", "-g", str(group_pid), "-f", "operator_console.app"],
        capture_output=True, text=True,
    )
    pids = [int(line) for line in result.stdout.split() if line.isdigit()]
    return max(pids) if pids else None


def _probe_states(probe_file: Path, wanted: str) -> set[str]:
    """프로브 파일에서 지금 `wanted` 상태인 패널 이름을 읽는다.

    콘솔이 아직 안 썼거나 쓰는 중이면 조용히 빈 집합 — 폴링이라 다음 턴에 다시 본다.
    """
    try:
        states = json.loads(probe_file.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return set()
    if not isinstance(states, dict):
        return set()
    return {name for name, state in states.items() if state == wanted}


def _probe_main_video(probe_file: Path) -> str | None:
    try:
        states = json.loads(probe_file.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    if not isinstance(states, dict):
        return None
    value = states.get("main_video")
    return str(value) if value is not None else None


def _probe_rover_widths(probe_file: Path) -> tuple[int, int] | None:
    try:
        states = json.loads(probe_file.read_text(encoding="utf-8"))
        return int(states["rover_l515_width"]), int(states["rover_d435_width"])
    except (OSError, ValueError, KeyError, TypeError):
        return None


def _probe_environment_values(probe_file: Path) -> tuple[str, str, str] | None:
    try:
        states = json.loads(probe_file.read_text(encoding="utf-8"))
        return tuple(str(states[key]) for key in (
            "environment_climate", "environment_air", "environment_hazard",
        ))
    except (OSError, ValueError, KeyError, TypeError):
        return None


def _probe_ops_steering(
    probe_file: Path,
) -> tuple[str, bool, int, str] | None:
    try:
        states = json.loads(probe_file.read_text(encoding="utf-8"))
        label = states["ops_steering_label"]
        sensitive = states["ops_steering_sensitive"]
        revision = states["ops_state_revision"]
        mode = states["ops_state_steering_mode"]
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if (
        not isinstance(label, str)
        or not isinstance(sensitive, bool)
        or not isinstance(revision, int)
        or isinstance(revision, bool)
        or not isinstance(mode, str)
    ):
        return None
    return label, sensitive, revision, mode


def _probe_arm_tabs(probe_file: Path) -> tuple[str, bool] | None:
    """Read the new tabs' observed tool and their no-command safety gate."""
    try:
        states = json.loads(probe_file.read_text(encoding="utf-8"))
        tool = states["arm_ui_detected_tool"]
        actions_disabled = states["arm_ui_actions_disabled"]
    except (OSError, ValueError, KeyError, TypeError):
        return None
    if not isinstance(tool, str) or not isinstance(actions_disabled, bool):
        return None
    return tool, actions_disabled


class _OpsStateFixture:
    """Serve one malformed-but-well-framed ops state to the real Gtk client."""

    def __init__(self) -> None:
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen(1)
        self._listener.settimeout(0.2)
        self.port = self._listener.getsockname()[1]
        self._stop = threading.Event()
        self._thread = threading.Thread(
            target=self._run,
            name="runtime-smoke-ops-fixture",
            daemon=True,
        )

    def start(self) -> None:
        self._thread.start()

    def _run(self) -> None:
        connection = None
        try:
            while not self._stop.is_set() and connection is None:
                try:
                    connection, _address = self._listener.accept()
                except TimeoutError:
                    continue
                except OSError:
                    return
            if connection is None:
                return
            connection.settimeout(0.2)
            hello = bytearray()
            while not self._stop.is_set() and b"\n" not in hello:
                try:
                    chunk = connection.recv(4096)
                except TimeoutError:
                    continue
                if not chunk:
                    return
                hello.extend(chunk)
            payload = {
                "schema_version": 1,
                "push": "ops_state",
                "revision": 1,
                "authority_mode": "IDLE",
                "chassis_mode": "IDLE",
                "estop_latched": False,
                "estop_source": "",
                "estop_detail": "",
                "active_estop_sources": [],
                "component_mask": {
                    "drive": True,
                    "steer": True,
                    "us100": True,
                    "robot_arm": True,
                },
                "wheels_stopped": True,
                "steering_mode": "invalid-smoke-mode",
                "steering_available": True,
            }
            record = (json.dumps(payload) + "\n").encode("utf-8")
            while not self._stop.is_set():
                try:
                    connection.sendall(record)
                except (OSError, TimeoutError):
                    return
                self._stop.wait(0.1)
        finally:
            if connection is not None:
                try:
                    connection.close()
                except OSError:
                    pass

    def close(self) -> None:
        self._stop.set()
        try:
            self._listener.close()
        except OSError:
            pass
        self._thread.join(timeout=1.0)


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
        "tool_runtime": {
            "tool": {
                "tool_type": "dual_motor_gripper", "actuator_ids": [3, 4],
                "actuators_discovered": True,
                "actuators": [{"id": 3, "online": True}, {"id": 4, "online": True}],
            },
            "source_age_s": 0.1,
        },
        "arm_runtime": {
            "control_mode": "MANUAL", "fsm_state": "IDLE", "arm_status": "READY",
            "source_age_s": {"control_mode": 0.1, "fsm_state": 0.1, "arm_status": 0.1},
        },
        "truncated": False,
    }


def _environment_payload(sequence: int) -> dict:
    return {
        "schema_version": 1,
        "sequence": sequence,
        "source": "runtime-smoke-rpi",
        "sensor_ok": True,
        "errors": [],
        "temperature_c": 29.08,
        "humidity_pct": 56.87,
        "pressure_hpa": 1003.14,
        "eco2_ppm": 401.0,
        "tvoc_ppb": 1.0,
        "sgp30_warming_up": False,
        "co_estimated_ppm": 1.0,
        "co_rs_ro": 1.019,
        "co_range": "below_detection_range",
        "co_quality": "uncalibrated_estimate",
        "lpg_estimated_ppm": 27.9,
        "lpg_rs_ro": 1.012,
        "lpg_range": "below_detection_range",
        "lpg_quality": "uncalibrated_estimate",
        "flame_detected": False,
        "flame_voltage_v": 2.754,
        "flame_threshold_v": 1.5,
    }


def run_smoke(
    run_s: float = RUN_S,
    *,
    omit_channel_for_test: str | None = None,
) -> tuple[bool, str]:
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
        "environment": _free_udp_port(),
    }
    ops_fixture = _OpsStateFixture()
    ops_fixture.start()
    with tempfile.NamedTemporaryFile(
        mode="w",
        encoding="utf-8",
        prefix="operator-console-smoke-token-",
        delete=False,
    ) as token_handle:
        token_handle.write("runtime-smoke-token")
        token_file = token_handle.name
    probe_file = Path(
        tempfile.mkdtemp(prefix="operator-console-smoke-probe-")
    ) / "panels.json"
    try:
        console = subprocess.Popen(
            [
                "xvfb-run", "-a", SYSTEM_PYTHON, "-m", "operator_console.app",
                "--host", "127.0.0.1",
                "--metadata-port", str(ports["metadata"]),
                "--telemetry-port", str(ports["telemetry"]),
                "--chassis-telemetry-port", str(ports["chassis"]),
                "--arm-telemetry-port", str(ports["arm"]),
                "--environment-telemetry-port", str(ports["environment"]),
                "--ops-token-file", token_file,
                "--ops-port", str(ops_fixture.port),
                "--smoke-probe-file", str(probe_file),
            ],
            cwd=REPO_ROOT,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            start_new_session=True,
            env=smoke_child_env(),
        )
    except BaseException:
        Path(token_file).unlink(missing_ok=True)
        ops_fixture.close()
        raise
    sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    builders = {
        "telemetry": _telemetry_payload, "chassis": _chassis_payload,
        "metadata": _metadata_payload, "arm": _arm_payload,
        "environment": _environment_payload,
    }
    live_seen: set[str] = set()
    stale_seen: set[str] = set()
    unexpected_auto_swap_seen = False
    role_sized_rovers_seen = False
    environment_values_seen = False
    invalid_steering_held_seen = False
    arm_tabs_seen = False
    try:
        # 콘솔이 Gtk 루프에 진입하기 전에 주입 창을 소진하면 LIVE 를 한 번도
        # 못 보고 거짓 FAIL 이 난다(부하가 높으면 xvfb 기동이 수 초 걸린다).
        # 거짓 FAIL 은 게이트를 무시당하게 만드니, 첫 _refresh_health 가 프로브를
        # 쓸 때까지 기다린 뒤에 시간을 재기 시작한다.
        ready_deadline = time.monotonic() + STARTUP_TIMEOUT_S
        while time.monotonic() < ready_deadline:
            if console.poll() is not None or probe_file.exists():
                break
            time.sleep(0.2)
        if console.poll() is None and not probe_file.exists():
            os.killpg(console.pid, 9)
            _, stderr = console.communicate()
            return False, (
                f"console never reached its first refresh within "
                f"{STARTUP_TIMEOUT_S:.0f}s\n"
                f"{stderr.decode('utf-8', 'replace')}"
            )
        deadline = time.monotonic() + run_s
        sequence = 0
        while time.monotonic() < deadline:
            if console.poll() is not None:
                break
            sequence += 1
            for channel, build in builders.items():
                if channel == omit_channel_for_test:
                    continue
                sender.sendto(
                    json.dumps(build(sequence)).encode("utf-8"),
                    ("127.0.0.1", ports[channel]),
                )
            time.sleep(0.2)
            live_seen |= _probe_states(probe_file, "LIVE")
            unexpected_auto_swap_seen |= (
                _probe_main_video(probe_file) == "작업 카메라"
            )
            role_sized_rovers_seen |= _probe_rover_widths(probe_file) == (290, 90)
            environment_values = _probe_environment_values(probe_file)
            if environment_values is not None:
                climate, air, hazard = environment_values
                environment_values_seen |= all((
                    "29.08" in climate, "56.87" in climate,
                    "1003.14" in climate, "401" in air, "1" in air,
                    "1.0" in hazard, "27.9" in hazard,
                    "불꽃 X" in hazard,
                ))
            invalid_steering_held_seen |= _probe_ops_steering(probe_file) == (
                "조향 방식 [상태 미확인]",
                False,
                1,
                "invalid-smoke-mode",
            )
            arm_tabs_seen |= _probe_arm_tabs(probe_file) == (
                "dual_gripper", True,
            )
        # phase 2 — 주입 중단: 전 패널 LIVE→STALE 전이 + 오버레이 숨김 경로.
        stale_deadline = time.monotonic() + 3.5
        while time.monotonic() < stale_deadline and console.poll() is None:
            time.sleep(0.2)
            stale_seen |= _probe_states(probe_file, "STALE")
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
            "environment": {
                "schema_version": 1, "sequence": sequence,
                "sensor_ok": False, "errors": ["fixture sparse packet"],
            },
        }
        for channel, payload in sparse.items():
            if channel == omit_channel_for_test:
                continue
            sender.sendto(
                json.dumps(payload).encode("utf-8"),
                ("127.0.0.1", ports[channel]),
            )
        sparse_deadline = time.monotonic() + 1.0
        while time.monotonic() < sparse_deadline and console.poll() is None:
            time.sleep(0.2)
        early_exit = console.poll() is not None
        shutdown_timed_out = False
        if not early_exit:
            # 사용자가 창을 닫거나 Ctrl+C 를 누르는 것과 같은 경로로 내린다.
            # SIGKILL 로 내리면 종료 경로의 결함이 영원히 안 보인다 —
            # 2026-07-29 실사고: 창을 닫으면 libsrt 전역 소멸자가 자기 워커
            # 스레드가 살아있는 채로 큐를 파괴해 메인 스레드가
            # pthread_cond_destroy 에서 멈추고(터미널 안 돌아옴) 수신 워커가
            # SIGSEGV 로 죽었다.
            # Popen 의 pid 는 xvfb-run 래퍼다.  래퍼째로 신호를 맞으면 콘솔의
            # 종료 코드가 래퍼 것에 가려지므로, 그룹 안에서 콘솔 자식만 찾아
            # 보낸다(xvfb-run 은 자식의 종료 코드를 그대로 돌려준다).
            console_pid = _console_child_pid(console.pid)
            os.kill(console_pid or console.pid, signal.SIGINT)
        try:
            _, stderr = console.communicate(timeout=15)
        except subprocess.TimeoutExpired:
            shutdown_timed_out = True
            os.killpg(console.pid, 9)
            _, stderr = console.communicate()
    finally:
        sender.close()
        ops_fixture.close()
        Path(token_file).unlink(missing_ok=True)
        shutil.rmtree(probe_file.parent, ignore_errors=True)
        if console.poll() is None:
            os.killpg(console.pid, 9)

    text = stderr.decode("utf-8", "replace")
    if early_exit:
        return False, f"console exited early (rc={console.returncode})\n{text}"
    if shutdown_timed_out:
        return False, (
            "console did not exit within 15s of SIGINT — 종료 경로가 막혔다\n"
            f"{text}"
        )
    if console.returncode != 0:
        signal_name = (
            signal.Signals(-console.returncode).name
            if console.returncode < 0 else str(console.returncode)
        )
        return False, (
            f"console shutdown was not clean (rc={console.returncode}"
            f" · {signal_name})\n{text}"
        )
    if "Traceback" in text:
        return False, f"callback traceback detected:\n{text}"
    # 기동·무traceback 만으로는 아무것도 보장하지 못한다. 수신 스레드를 통째로
    # 죽여도 그 두 조건은 통과했다(2026-07-18 R06 #10). 패널이 실제로 데이터를
    # 받아 LIVE 가 됐고, 주입을 끊었을 때 STALE 로 전이했는지까지 단언한다.
    missing_live = sorted(REQUIRED_PANELS - live_seen)
    if missing_live:
        return False, (
            f"panels never reached LIVE: {', '.join(missing_live)}\n"
            f"(saw LIVE on: {', '.join(sorted(live_seen)) or 'nothing'})\n{text}"
        )
    missing_stale = sorted(REQUIRED_PANELS - stale_seen)
    if missing_stale:
        return False, (
            f"panels never went STALE after injection stopped: "
            f"{', '.join(missing_stale)}\n{text}"
        )
    if unexpected_auto_swap_seen:
        return False, "D435i became MAIN without an operator click\n" + text
    if not role_sized_rovers_seen:
        return False, (
            "default front-MAIN/work-PiP placeholder sizes changed\n" + text
        )
    if not environment_values_seen:
        return False, "environment values never rendered in the GUI\n" + text
    if not invalid_steering_held_seen:
        return False, "invalid steering ops state was not held disabled\n" + text
    if not arm_tabs_seen:
        return False, "arm tabs did not render live tool in safe disabled state\n" + text
    return True, (
        f"PASS · {sequence} ticks on 5 channels · "
        f"LIVE+STALE observed on {', '.join(sorted(REQUIRED_PANELS))} · "
        "invalid steering mode held disabled · "
        "arm tabs observed safely disabled · "
        "no automatic camera swap observed · no tracebacks"
    )


def main() -> int:
    passed, report = run_smoke()
    print(f"CONSOLE-RUNTIME-SMOKE: {'PASS' if passed else 'FAIL'}")
    print(report)
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
