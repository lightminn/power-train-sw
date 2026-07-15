import os
from pathlib import Path
import signal
import socket
import subprocess
import sys
import time
import uuid

import pytest


ROOT = Path(__file__).parents[2]
COMPOSE = ROOT / "docker" / "docker-compose.jetson.yml"
INSTALLER = ROOT / "scripts" / "install_powertrain_runtime_dir.sh"


def modules():
    from powertrain_observability.client import EventClient, ObservabilityClient

    return EventClient, ObservabilityClient


def wait_for(predicate, timeout=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        value = predicate()
        if value:
            return value
        time.sleep(0.02)
    return predicate()


def require_abstract_socket_runtime():
    probe = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)
    try:
        probe.setsockopt(socket.SOL_SOCKET, socket.SO_PASSCRED, 1)
        probe.bind("\0test-observability-probe-" + uuid.uuid4().hex)
    except PermissionError as exc:
        pytest.skip(f"sandbox blocks AF_UNIX abstract sockets/SO_PASSCRED: {exc}")
    finally:
        probe.close()


def test_real_daemon_survives_status_disconnect_and_sighup(tmp_path):
    require_abstract_socket_runtime()
    EventClient, ObservabilityClient = modules()
    suffix = f"{os.getpid()}-{uuid.uuid4().hex}"
    event_socket = f"@test-process-events-{suffix}"
    status_socket = f"@test-process-status-{suffix}"
    lock_path = tmp_path / "observability.lock"
    run_directory = tmp_path / "runs"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "powertrain_observability.main",
            "--event-socket",
            event_socket,
            "--status-socket",
            status_socket,
            "--lock-path",
            str(lock_path),
            "--run-directory",
            str(run_directory),
        ]
    )
    client = ObservabilityClient(status_socket, request_timeout_s=0.2)
    try:
        first = wait_for(client.poll)
        assert first is not None
        os.kill(process.pid, signal.SIGHUP)
        assert wait_for(client.poll) is not None

        EventClient(event_socket).emit(
            {
                "schema_version": 1,
                "wall_time_ns": time.time_ns(),
                "monotonic_ns": time.monotonic_ns(),
                "source": "process_test",
                "event_type": "MISSION",
                "severity": "INFO",
                "payload": {"result": "PASS"},
            }
        )
        observed = wait_for(
            lambda: (
                current
                if (current := client.poll()) and current.payload["recent_event"]
                else None
            )
        )
        assert observed.payload["recent_event"]["payload"] == {"result": "PASS"}
        assert process.poll() is None
    finally:
        if process.poll() is None:
            process.terminate()
        process.wait(timeout=3)

    assert lock_path.is_file()
    assert list(run_directory.glob("*.jsonl"))


def test_compose_observability_service_contract_and_socket_healthcheck():
    text = COMPOSE.read_text()
    service = text.split("  powertrain_observability:", 1)[1]

    assert "image: powertrain-sw:ros" in service
    assert "network_mode: host" in service
    assert "PYTHONPATH=/workspace" in service
    assert "python3" in service and "powertrain_observability.main" in service
    assert "source: /run/powertrain" in service
    assert "source: /var/lib/powertrain" in service
    assert service.count("create_host_path: false") >= 2
    assert "restart: unless-stopped" in service
    assert "ObservabilityClient" in service
    assert "pgrep" not in service


def test_runtime_installer_provisions_root_owned_runs_directory():
    text = INSTALLER.read_text()

    assert 'RUNS_DIR="/var/lib/powertrain/runs"' in text
    assert 'install -d -o root -g root -m 0750 "$RUNS_DIR"' in text
    assert 'stat -c \'%U:%G:%a:%F\' "$RUNS_DIR"' in text
