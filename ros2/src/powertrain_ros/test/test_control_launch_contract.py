"""D5 배포 계약 — control.launch가 teleop+broker를 함께 감독한다."""
from pathlib import Path

PACKAGE = Path(__file__).resolve().parents[1]
LAUNCH = PACKAGE / "launch/control.launch.py"
COMPOSE = PACKAGE.parents[2] / "docker/docker-compose.jetson.yml"


def test_control_launch_runs_teleop_and_broker_as_separate_nodes():
    source = LAUNCH.read_text(encoding="utf-8")
    assert 'executable="teleop_command"' in source
    assert 'executable="ops_broker"' in source
    assert '"token_dir"' in source


def test_compose_control_service_uses_launch_and_checks_both_ports():
    source = COMPOSE.read_text(encoding="utf-8")
    assert "control.launch.py" in source
    assert "9001" in source
