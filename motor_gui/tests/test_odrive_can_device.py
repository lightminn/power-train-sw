import struct
import can

from motor_gui.backend.transport.odrive_can_device import (
    OdriveCanDevice, NODE_ID, C_HEARTBEAT, C_SET_POS_GAIN, C_SET_VEL_GAINS,
    C_GET_ENC_EST, C_GET_IQ, C_GET_TEMP, C_GET_BUS_VI,
    C_SET_CTRL_MODE, C_SET_INPUT_POS, C_SET_INPUT_VEL, C_SET_LIMITS,
    C_SET_STATE, C_SET_LINEAR_COUNT, C_CLEAR_ERR, C_ESTOP,
    AXIS_IDLE, AXIS_CLOSED_LOOP, AXIS_FULL_CALIB,
)


class StubBus:
    def __init__(self):
        self.sent = []
    def send(self, msg, timeout=None):
        self.sent.append(msg)


def _arb(cmd, node=NODE_ID):
    return (node << 5) | cmd


def _sent_cmds(bus):
    return [m.arbitration_id & 0x1F for m in bus.sent]


def _mk():
    d = OdriveCanDevice()
    bus = StubBus()
    d.attach(bus)
    return d, bus


def test_capabilities_three_modes_no_torque():
    f = OdriveCanDevice().capabilities_fragment()
    assert f["devices"] == ["odrive"]
    assert f["control_modes"]["odrive"] == ["position", "position_traj", "velocity"]
    assert "torque" not in f["control_modes"]["odrive"]
    assert set(f["inputs"]["odrive"]) == {"position", "position_traj", "velocity"}


def test_capabilities_commands_include_set_param_not_save_nvm():
    f = OdriveCanDevice().capabilities_fragment()
    cmds = f["commands"]["odrive"]
    assert "set_param" in cmds
    assert "set_origin" in cmds
    assert "save_nvm" not in cmds


def test_capabilities_tunables_prefill_values():
    f = OdriveCanDevice().capabilities_fragment()
    tk = {t["key"]: t for t in f["tunables"]["odrive"]}
    assert tk["pos_gain"]["value"] == 8.0
    assert tk["vel_limit"]["value"] == 5.0
    assert tk["current_lim"]["value"] == 10.0
    assert "trap_vel_limit" not in tk
    assert "input_filter_bandwidth" not in tk
    assert tk["torque_constant"]["op"] == "set_param"
    assert abs(tk["torque_constant"]["value"] - 0.0084) < 1e-9


def test_signals_exclude_id_and_suberrors():
    f = OdriveCanDevice().capabilities_fragment()
    sig = f["signals"]
    assert "odrive.pos" in sig and "odrive.torque_est" in sig
    assert "odrive.id_meas" not in sig
    assert "odrive.motor_err" not in sig


def test_attach_pushes_default_gains():
    d, bus = _mk()
    cmds = _sent_cmds(bus)
    assert C_SET_POS_GAIN in cmds
    assert C_SET_VEL_GAINS in cmds
    assert C_SET_LIMITS in cmds


def _enc_msg(pos, vel, node=NODE_ID):
    return can.Message(arbitration_id=_arb(C_GET_ENC_EST, node),
                       data=struct.pack("<ff", pos, vel), is_extended_id=False)


def _iq_msg(iq_set, iq_meas):
    return can.Message(arbitration_id=_arb(C_GET_IQ),
                       data=struct.pack("<ff", iq_set, iq_meas), is_extended_id=False)


def _heartbeat_msg(axis_err=0, state=AXIS_IDLE):
    return can.Message(arbitration_id=_arb(C_HEARTBEAT),
                       data=struct.pack("<IB3x", axis_err, state), is_extended_id=False)


def test_request_sends_four_rtr_polls():
    d, bus = _mk()
    bus.sent.clear()
    d.request(bus)
    rtr = [m.arbitration_id & 0x1F for m in bus.sent if m.is_remote_frame]
    assert set(rtr) == {C_GET_ENC_EST, C_GET_IQ, C_GET_TEMP, C_GET_BUS_VI}


def test_on_rx_decodes_encoder_and_heartbeat():
    d, bus = _mk()
    d.on_rx(_enc_msg(2.5, -1.25))
    d.on_rx(_heartbeat_msg(axis_err=0x20, state=AXIS_CLOSED_LOOP))
    s = d.sample()
    assert abs(s["odrive.pos"] - 2.5) < 1e-6
    assert abs(s["odrive.vel"] + 1.25) < 1e-6
    assert s["odrive.state"] == AXIS_CLOSED_LOOP
    assert s["odrive.axis_err"] == 0x20


def test_on_rx_ignores_other_node_and_extended():
    d, bus = _mk()
    d.on_rx(_enc_msg(9.9, 9.9, node=NODE_ID + 1))      # 다른 node
    ext = can.Message(arbitration_id=0x2901, data=struct.pack("<ff", 5.0, 5.0),
                      is_extended_id=True)
    d.on_rx(ext)                                        # 확장 ID(AK)
    s = d.sample()
    assert s["odrive.pos"] == 0.0 and s["odrive.vel"] == 0.0


def test_sample_torque_est_is_iq_times_kt():
    d, bus = _mk()
    d.on_rx(_iq_msg(1.0, 2.0))                          # iq_meas=2.0
    s = d.sample()
    assert abs(s["odrive.iq_meas"] - 2.0) < 1e-6
    assert abs(s["odrive.torque_est"] - 2.0 * 0.0084) < 1e-9


def test_sample_includes_tracked_setpoints():
    d, bus = _mk()
    d._pos_setpoint = 3.0
    d._vel_setpoint = 4.0
    s = d.sample()
    assert s["odrive.pos_setpoint"] == 3.0
    assert s["odrive.vel_setpoint"] == 4.0
