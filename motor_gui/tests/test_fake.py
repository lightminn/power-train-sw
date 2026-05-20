import math

from motor_gui.backend.transport.fake import FakeTransport


def test_capabilities_lists_both_devices_and_save_nvm():
    t = FakeTransport()
    caps = t.capabilities()
    assert caps["track"] == "fake"
    assert set(caps["devices"]) == {"odrive", "ak"}
    assert "save_nvm" in caps["commands"]["odrive"]
    assert "odrive.pos" in caps["signals"]
    assert "ak.pos_deg" in caps["signals"]


def test_sample_has_t_mono_and_known_keys():
    t = FakeTransport()
    t.connect()
    s = t.sample()
    assert "t_mono" in s
    for key in caps_signal_keys():
        assert key in s


def caps_signal_keys():
    return FakeTransport().capabilities()["signals"]


def test_velocity_command_drives_velocity_up():
    t = FakeTransport()
    t.connect()
    t.apply({"target": "odrive", "op": "set_mode",
             "args": {"control_mode": "velocity"}})
    t.apply({"target": "odrive", "op": "set_input", "args": {"vel": 5.0}})
    last_vel = 0.0
    for _ in range(200):              # 200 틱 적분
        s = t.sample()
        last_vel = s["odrive.vel"]
    assert last_vel > 1.0             # 0 → 목표(5)로 상승
    assert abs(s["odrive.iq_meas"]) >= 0.0


def test_estop_zeros_commands():
    t = FakeTransport()
    t.connect()
    t.apply({"target": "odrive", "op": "set_input", "args": {"vel": 5.0}})
    ack = t.apply({"target": "odrive", "op": "estop", "args": {}})
    assert ack["ok"] is True
    for _ in range(300):
        s = t.sample()
    assert abs(s["odrive.vel"]) < 0.5
