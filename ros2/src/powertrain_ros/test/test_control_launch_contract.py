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


def test_compose_chassis_service_is_persistent_and_explicit_about_stop_mm():
    # 2026-07-18 사용자 결정: 벤치 상시 chassis 스택. stop_mm 은 env 명시
    # 전달만(부재 시 기동 거부), healthcheck 는 래퍼가 아닌 실제 노드
    # 프로세스를 본다(cmdline grep 함정 회피).
    source = COMPOSE.read_text(encoding="utf-8")
    assert "powertrain_chassis:" in source
    assert "wp5_control.launch.py stop_mm:=$$STOP_MM" in source
    assert "STOP_MM missing" in source
    assert "us100_safety" in source


def test_compose_chassis_enables_command_authority():
    source = COMPOSE.read_text(encoding="utf-8")
    assert (
        "wp5_control.launch.py stop_mm:=$$STOP_MM "
        "authority_enabled:=true"
    ) in source


def test_wp5_launch_shuts_down_whole_stack_when_any_node_dies():
    # 반쪽 생존 금지(07-18 실측: chassis만 죽고 us100은 살아 unhealthy 방치)
    # — 노드 사망 = launch 종료 = compose restart 전체 복구.
    source = (PACKAGE / "launch/wp5_control.launch.py").read_text(
        encoding="utf-8"
    )
    assert source.count("on_exit=Shutdown()") == 2
