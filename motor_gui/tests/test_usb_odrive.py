from types import SimpleNamespace

import pytest

from motor_gui.backend.transport.usb_odrive import UsbOdriveBackend


def _backend(gear_ratio=5.0):
    current = SimpleNamespace(
        Iq_measured=2.0,
        Iq_setpoint=1.5,
        Id_measured=0.1,
        Id_setpoint=0.0,
    )
    controller = SimpleNamespace(
        pos_setpoint=4.0,
        vel_setpoint=10.0,
        vel_integrator_torque=0.25,
        input_vel=None,
        error=0,
        config=SimpleNamespace(
            input_mode=2,
            vel_limit=50.0,
            pos_gain=8.0,
            vel_gain=0.015,
            vel_integrator_gain=0.0,
            input_filter_bandwidth=50.0,
        ),
    )
    motor = SimpleNamespace(
        current_control=current,
        error=0,
        config=SimpleNamespace(current_lim=10.0),
    )
    axis = SimpleNamespace(
        encoder=SimpleNamespace(pos_estimate=3.0, vel_estimate=10.0, error=0),
        controller=controller,
        motor=motor,
        trap_traj=SimpleNamespace(config=SimpleNamespace(
            vel_limit=20.0,
            accel_limit=15.0,
            decel_limit=20.0,
        )),
        current_state=8,
        error=0,
    )
    backend = UsbOdriveBackend(gear_ratio=gear_ratio)
    backend._ax = axis
    backend._drv = SimpleNamespace(vbus_voltage=24.0, ibus=0.5)
    backend._enums = {"TRAP_TRAJ": 5, "CLOSED_LOOP": 8}
    return backend, axis


def test_velocity_command_and_feedback_cross_wheel_motor_boundary():
    backend, axis = _backend(gear_ratio=5.0)

    ack = backend.apply({"target": "odrive", "op": "set_input", "args": {"vel": 1.0}})
    sample = backend.sample()

    assert ack["ok"] is True
    assert axis.controller.input_vel == 5.0
    assert sample["odrive.vel"] == 2.0
    assert sample["odrive.vel_setpoint"] == 2.0
    assert sample["odrive.pos"] == 3.0
    assert sample["odrive.iq_meas"] == 2.0
    assert backend.capabilities()["drive_gear_ratio"] == 5.0


def test_gear_ratio_one_preserves_usb_velocity_units():
    backend, axis = _backend(gear_ratio=1.0)
    backend.apply({"target": "odrive", "op": "set_input", "args": {"vel": 1.0}})
    assert axis.controller.input_vel == 1.0
    assert backend.sample()["odrive.vel"] == 10.0


def test_usb_velocity_tunables_cross_wheel_motor_boundary():
    backend, axis = _backend(gear_ratio=5.0)

    backend.apply({"target": "odrive", "op": "set_limit",
                   "args": {"vel_limit": 4.0}})
    backend.apply({"target": "odrive", "op": "set_gain",
                   "args": {"trap_accel_limit": 2.0, "trap_decel_limit": 3.0}})

    assert axis.controller.config.vel_limit == 20.0
    assert axis.trap_traj.config.vel_limit == 20.0
    assert axis.trap_traj.config.accel_limit == 10.0
    assert axis.trap_traj.config.decel_limit == 15.0
    tunables = backend.read_tunables()
    assert tunables["vel_limit"] == 4.0
    assert tunables["trap_vel_limit"] == 4.0
    assert tunables["trap_accel_limit"] == 2.0
    assert tunables["trap_decel_limit"] == 3.0
    assert backend.capabilities()["limits"]["odrive"]["vel"] == 40.0


@pytest.mark.parametrize("ratio", [0.0, -1.0])
def test_usb_gear_ratio_rejects_nonpositive_values(ratio):
    with pytest.raises(ValueError, match="gear_ratio must be finite and positive"):
        UsbOdriveBackend(gear_ratio=ratio)
