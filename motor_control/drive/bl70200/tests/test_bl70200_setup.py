import importlib
from types import SimpleNamespace


def _setup_module():
    return importlib.import_module("drive.bl70200.bl70200_setup")


class FallbackAxisConfig:
    def __init__(self) -> None:
        self.can_node_id = 0
        self.can = SimpleNamespace(node_id=0, heartbeat_rate_ms=100)

    def __setattr__(self, name, value) -> None:
        if name == "can_heartbeat_rate_ms":
            raise AttributeError(name)
        object.__setattr__(self, name, value)


def _axis(*, fallback=False):
    config = (
        FallbackAxisConfig()
        if fallback
        else SimpleNamespace(can_node_id=0, can_heartbeat_rate_ms=100)
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
    return SimpleNamespace(
        fw_version_major=0,
        fw_version_minor=5,
        fw_version_revision=1,
        vbus_voltage=48.0,
        config=SimpleNamespace(
            dc_bus_undervoltage_trip_level=40.0,
            dc_bus_overvoltage_trip_level=56.0,
            brake_resistance=2.0,
        ),
        can=SimpleNamespace(set_baud_rate=lambda baud: None),
        axis0=_axis(),
        axis1=_axis(fallback=fallback_axis1),
        save_configuration=lambda: None,
    )


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


def test_apply_sets_50hz_heartbeat_on_both_axes(monkeypatch) -> None:
    module = _setup_module()
    board = _board()
    _patch_apply_runtime(monkeypatch, module)

    assert module.main(
        ["--apply", "--axis", "both"], odrive_module=FakeOdrive(board)
    ) == 0

    assert board.axis0.config.can_heartbeat_rate_ms == 20
    assert board.axis1.config.can_heartbeat_rate_ms == 20


def test_apply_falls_back_to_nested_heartbeat_rate(monkeypatch) -> None:
    module = _setup_module()
    board = _board(fallback_axis1=True)
    _patch_apply_runtime(monkeypatch, module)

    assert module.main(["--apply"], odrive_module=FakeOdrive(board)) == 0

    assert board.axis1.config.can.heartbeat_rate_ms == 20


def test_read_shows_heartbeat_rate(capsys) -> None:
    module = _setup_module()

    assert module.main(["--read"], odrive_module=FakeOdrive(_board())) == 0

    assert "board   UV=40 OV=56 brake=2.0 hb=100ms | cal:" in capsys.readouterr().out
