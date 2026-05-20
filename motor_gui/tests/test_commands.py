import pytest

from motor_gui.backend.commands import normalize, CommandError

CAPS = {
    "track": "fake",
    "devices": ["odrive", "ak"],
    "commands": {
        "odrive": ["set_mode", "set_input", "set_limit", "estop"],
        "ak": ["set_input", "estop"],
    },
    "limits": {"odrive": {"vel": 20.0, "torque": 10.0}, "ak": {"pos_deg": 360.0}},
}


def test_rejects_unknown_target():
    with pytest.raises(CommandError):
        normalize({"target": "ghost", "op": "estop", "args": {}}, CAPS)


def test_rejects_unsupported_op():
    with pytest.raises(CommandError):
        normalize({"target": "odrive", "op": "save_nvm", "args": {}}, CAPS)


def test_clamps_velocity_to_limit():
    out = normalize({"target": "odrive", "op": "set_input",
                     "args": {"vel": 999.0}}, CAPS)
    assert out["args"]["vel"] == 20.0
    out2 = normalize({"target": "odrive", "op": "set_input",
                      "args": {"vel": -999.0}}, CAPS)
    assert out2["args"]["vel"] == -20.0


def test_set_limit_floored_at_zero():
    out = normalize({"target": "odrive", "op": "set_limit",
                     "args": {"vel_limit": -3.0}}, CAPS)
    assert out["args"]["vel_limit"] == 0.0


def test_passes_valid_command_through():
    out = normalize({"target": "ak", "op": "estop", "args": {}}, CAPS)
    assert out == {"target": "ak", "op": "estop", "args": {}}
