import importlib
from pathlib import Path
from types import SimpleNamespace


def _setup_module():
    return importlib.import_module("drive.bl70200.bl70200_setup")


class FallbackAxisConfig:
    def __init__(self) -> None:
        self.can = SimpleNamespace(node_id=12, heartbeat_rate_ms=100, is_extended=False)

    def __setattr__(self, name, value) -> None:
        if name in ("can_heartbeat_rate_ms", "can_node_id"):
            raise AttributeError(name)
        object.__setattr__(self, name, value)


def _axis(*, fallback=False):
    config = (
        FallbackAxisConfig()
        if fallback
        else SimpleNamespace(can_node_id=12, can_heartbeat_rate_ms=100, can_extended_id=False)
    )
    return SimpleNamespace(
        config=config,
        motor=SimpleNamespace(
            config=SimpleNamespace(
                motor_type=0,
                pole_pairs=10,
                current_lim=9.0,
                torque_constant=0.353,
            ),
            is_calibrated=True,
        ),
        encoder=SimpleNamespace(
            config=SimpleNamespace(
                mode=1,
                cpr=60,
                bandwidth=30.0,
                calib_scan_omega=6.0,
                ignore_illegal_hall_state=True,
            ),
            is_ready=True,
        ),
        controller=SimpleNamespace(
            config=SimpleNamespace(
                pos_gain=2.0,
                vel_gain=0.12,
                vel_integrator_gain=0.2,
                input_filter_bandwidth=2.0,
                vel_limit=50.0,
            )
        ),
    )


def _board(*, fallback_axis1=False):
    board = SimpleNamespace(
        serial_number=0x3352,
        fw_version_major=0,
        fw_version_minor=5,
        fw_version_revision=1,
        vbus_voltage=48.0,
        config=SimpleNamespace(
            dc_bus_undervoltage_trip_level=40.0,
            dc_bus_overvoltage_trip_level=56.0,
            brake_resistance=2.0,
        ),
        can=SimpleNamespace(set_baud_rate=lambda baud: None,
                            config=SimpleNamespace(baud_rate=500000, protocol=0)),
        axis0=_axis(),
        axis1=_axis(fallback=fallback_axis1),
        save_configuration=lambda: None,
    )
    board.axis0.config.can_node_id = 11
    return board


class FakeOdrive:
    def __init__(self, board) -> None:
        self.board = board

    def find_any(self, **kwargs):
        return self.board


def _patch_apply_runtime(monkeypatch, module) -> None:
    enums = SimpleNamespace(
        MOTOR_TYPE_HIGH_CURRENT=0,
        ENCODER_MODE_HALL=1,
        CONTROL_MODE_VELOCITY_CONTROL=2,
        INPUT_MODE_VEL_RAMP=3,
    )
    monkeypatch.setattr(module, "_load_enums", lambda: enums)
    monkeypatch.setitem(module.run.__kwdefaults__, "sleep_fn", lambda seconds: None)


def test_apply_sets_50hz_heartbeat_on_both_axes(monkeypatch, tmp_path) -> None:
    module = _setup_module()
    board = _board()
    _patch_apply_runtime(monkeypatch, module)
    from chassis.runtime_lock import RealCanSession
    monkeypatch.setattr(module, "motor_session", lambda owner: RealCanSession(owner=owner, path=str(tmp_path / "can0.lock")))

    assert module.main(
        ["--apply", "--axis", "both", "--serial", "3352", "--node", "11"], odrive_module=FakeOdrive(board)
    ) == 0

    assert board.axis0.config.can_heartbeat_rate_ms == 20
    assert board.axis1.config.can_heartbeat_rate_ms == 20


def test_apply_falls_back_to_nested_heartbeat_rate(monkeypatch, tmp_path) -> None:
    module = _setup_module()
    board = _board(fallback_axis1=True)
    _patch_apply_runtime(monkeypatch, module)
    from chassis.runtime_lock import RealCanSession
    monkeypatch.setattr(module, "motor_session", lambda owner: RealCanSession(owner=owner, path=str(tmp_path / "can0.lock")))

    assert module.main(["--apply", "--axis", "1", "--serial", "3352", "--node", "12"], odrive_module=FakeOdrive(board)) == 0

    assert board.axis1.config.can.heartbeat_rate_ms == 20


def test_read_shows_heartbeat_rate(capsys) -> None:
    module = _setup_module()

    assert module.main(["--read"], odrive_module=FakeOdrive(_board())) == 0

    assert "board   UV=40 OV=56 brake=2.0 hb=100ms | cal:" in capsys.readouterr().out
