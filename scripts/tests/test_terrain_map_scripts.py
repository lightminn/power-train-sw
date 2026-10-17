"""Guards of the terrain map bag scripts (spec §6): replay domain, rate, name, disk space."""
import os
from pathlib import Path
import subprocess

ROOT = Path(__file__).resolve().parents[2]
RECORD = ROOT / "scripts/record_terrain_map_bag.sh"
REPLAY = ROOT / "scripts/replay_terrain_map.sh"


def stub_bin(tmp_path):
    """ros2 / python3 stubs that log their argv and the ROS domain instead of running."""
    bin_dir = tmp_path / "bin"
    bin_dir.mkdir(exist_ok=True)
    log = tmp_path / "calls.log"
    log.unlink(missing_ok=True)                      # each run reports only its own calls
    for name in ("ros2", "python3"):
        path = bin_dir / name
        path.write_text(f'#!/bin/bash\necho "{name} domain=$ROS_DOMAIN_ID $*" >> "{log}"\n')
        path.chmod(0o755)
    return bin_dir, log


def run(script, *args, env=None, tmp_path):
    bin_dir, log = stub_bin(tmp_path)
    full = {**os.environ, "PATH": f"{bin_dir}:{os.environ['PATH']}", **(env or {})}
    result = subprocess.run(["bash", str(script), *args], capture_output=True, text=True, env=full, timeout=20)
    return result, (log.read_text() if log.exists() else "")


def test_replay_forces_domain_78_and_sends_to_the_console(tmp_path):
    result, calls = run(REPLAY, "bag_dir", "--rate", "2.0", env={"ROS_DOMAIN_ID": "0"}, tmp_path=tmp_path)
    assert result.returncode == 0, result.stderr
    assert calls.strip() == ("python3 domain=78 -m powertrain_ros.terrain_map_replay bag_dir "
                             "--send 127.0.0.1:5010 --rate 2.0")


def test_replay_refuses_the_production_domain(tmp_path):
    result, calls = run(REPLAY, "bag_dir", env={"TERRAIN_REPLAY_DOMAIN": "0"}, tmp_path=tmp_path)
    assert result.returncode == 2 and "production" in result.stderr and calls == ""
    result, calls = run(REPLAY, "bag_dir", env={"TERRAIN_REPLAY_DOMAIN": "78", "PRODUCTION_ROS_DOMAIN_ID": "78"},
                        tmp_path=tmp_path)
    assert result.returncode == 2 and calls == ""


def test_ros_replay_is_rate_one_only(tmp_path):
    result, calls = run(REPLAY, "--ros", "bag_dir", "--rate", "0.5", tmp_path=tmp_path)
    assert result.returncode == 2 and "rate 1.0 only" in result.stderr and calls == ""
    result, calls = run(REPLAY, "--ros", "bag_dir", tmp_path=tmp_path)
    assert result.returncode == 0, result.stderr
    assert "ros2 domain=78 bag play --rate 1.0 --delay 2.0 bag_dir" in calls
    assert "ros2 domain=78 run powertrain_ros terrain_map --ros-args -p operator_host:=127.0.0.1 -p operator_port:=5010" in calls


def test_record_checks_name_existing_dir_and_free_space(tmp_path):
    bags = tmp_path / "bags"
    result, _ = run(RECORD, "bad name", str(bags), tmp_path=tmp_path)
    assert result.returncode == 2
    result, calls = run(RECORD, "run1", str(bags), env={"TERRAIN_BAG_MIN_FREE_MB": "999999999"}, tmp_path=tmp_path)
    assert result.returncode == 3 and "need" in result.stderr and calls == ""
    result, calls = run(RECORD, "run1", str(bags), env={"TERRAIN_BAG_MIN_FREE_MB": "1"}, tmp_path=tmp_path)
    assert result.returncode == 0, result.stderr
    assert calls.strip().endswith(f"bag record -o {bags}/run1 /l515/depth/image_rect_raw /l515/depth/camera_info "
                                  "/l515/color/camera_info /l515/gyro/sample /wheel_states")
    (bags / "run1").mkdir()
    result, _ = run(RECORD, "run1", str(bags), env={"TERRAIN_BAG_MIN_FREE_MB": "1"}, tmp_path=tmp_path)
    assert result.returncode == 2 and "exists" in result.stderr
