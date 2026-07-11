import os
from pathlib import Path
import subprocess

import pytest


SCRIPT=Path(__file__).parents[2]/"docker"/"powertrain_ros_entrypoint.sh"
COMPOSE=Path(__file__).parents[2]/"docker"/"docker-compose.jetson.yml"
TMPFILES=Path(__file__).parents[2]/"docker"/"powertrain-gateway-tmpfiles.conf"
INSTALLER=Path(__file__).parents[2]/"scripts"/"install_powertrain_runtime_dir.sh"


def environment(tmp_path, *, colcon_exit=0):
    root=tmp_path/"workspace"; (root/"ros2/src/powertrain_ros").mkdir(parents=True)
    (root/"ros2/src/powertrain_ros/node.py").write_text("source")
    bindir=tmp_path/"bin"; bindir.mkdir(); log=tmp_path/"log"
    (bindir/"colcon").write_text(f"#!/bin/sh\necho colcon:$@ >> '{log}'\nmkdir -p install\nprintf 'export ROS_SETUP_SOURCED=yes\\n' > install/setup.bash\nexit {colcon_exit}\n")
    (bindir/"python3").write_text(f"#!/bin/sh\necho python:$@:$ROS_SETUP_SOURCED >> '{log}'\n")
    for item in bindir.iterdir(): item.chmod(0o755)
    env={**os.environ,"WORKSPACE_ROOT":str(root),"ROS_DISTRO_SETUP":str(tmp_path/"ros.bash"),"PATH":f"{bindir}:{os.environ['PATH']}"}
    (tmp_path/"ros.bash").write_text("export ROS_BASE_SOURCED=yes\n")
    return root,log,env


def test_fresh_checkout_builds_sources_then_sources_and_execs(tmp_path):
    root,log,env=environment(tmp_path)
    result=subprocess.run([SCRIPT],env=env,text=True,capture_output=True)
    assert result.returncode==0
    lines=log.read_text().splitlines()
    assert lines[0].startswith("colcon:build --packages-select robot_arm_msgs powertrain_msgs powertrain_ros")
    assert lines[1]=="python:-m l515_dashboard.gateway_main:yes"


def test_build_failure_exits_nonzero_without_starting_gateway(tmp_path):
    _,log,env=environment(tmp_path,colcon_exit=7)
    result=subprocess.run([SCRIPT],env=env,text=True,capture_output=True)
    assert result.returncode==7
    assert "python:" not in log.read_text()


def test_newer_source_rebuilds_existing_install(tmp_path):
    root,log,env=environment(tmp_path)
    install=root/"ros2/install/setup.bash"; install.parent.mkdir(parents=True)
    install.write_text("export ROS_SETUP_SOURCED=yes\n")
    os.utime(install,(1,1))
    result=subprocess.run([SCRIPT],env=env,text=True,capture_output=True)
    assert result.returncode==0
    assert log.read_text().splitlines()[0].startswith("colcon:build")


def test_compose_bounds_crash_restarts_but_keeps_clean_stop_stopped():
    text=COMPOSE.read_text()
    service=text.split("  powertrain_ros:",1)[1]
    assert 'restart: "on-failure:5"' in service


def test_compose_shares_persistent_gateway_flock_with_host():
    text=COMPOSE.read_text()
    service=text.split("  powertrain_ros:",1)[1]
    assert "source: /run/powertrain" in service
    assert "target: /run/powertrain" in service
    assert "create_host_path: false" in service

    rendered=subprocess.run(
        ["docker","compose","-f",str(COMPOSE),"config"],
        cwd=COMPOSE.parents[1],text=True,capture_output=True,check=True,
    ).stdout
    ros_service=rendered.split("  powertrain_ros:",1)[1]
    assert "type: bind" in ros_service
    assert "source: /run/powertrain" in ros_service
    assert "target: /run/powertrain" in ros_service


def test_tmpfiles_contract_and_root_only_installer():
    assert TMPFILES.read_text() == "d /run/powertrain 0750 root root -\n"
    text=INSTALLER.read_text()
    assert 'EUID' in text and 'must run as root' in text
    assert '/etc/tmpfiles.d/powertrain-gateway.conf' in text
    assert 'systemd-tmpfiles --create' in text
    assert 'install -D -o root -g root -m 0644' in text
    assert "stat -c '%U:%G:%a:%F'" in text


def test_runtime_installer_fails_closed_for_non_root(tmp_path):
    if os.geteuid() == 0:
        pytest.skip("non-root execution is covered by the static EUID contract")
    result=subprocess.run([INSTALLER],text=True,capture_output=True)
    assert result.returncode != 0
    assert "must run as root" in result.stderr
