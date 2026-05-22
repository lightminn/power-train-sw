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


def _last(bus, cmd):
    """해당 cmd 의 마지막 송신 메시지(없으면 None)."""
    hits = [m for m in bus.sent if (m.arbitration_id & 0x1F) == cmd and not m.is_remote_frame]
    return hits[-1] if hits else None


def test_set_mode_position_sets_ctrl_and_holds_pos():
    d, bus = _mk()
    d.on_rx(_enc_msg(1.5, 0.0))
    bus.sent.clear()
    ack = d.apply(bus, "set_mode", {"control_mode": "position"})
    assert ack["ok"] is True
    cm = _last(bus, C_SET_CTRL_MODE)
    assert struct.unpack("<ii", cm.data) == (3, 3)        # POSITION / POS_FILTER
    ip = _last(bus, C_SET_INPUT_POS)
    pos, _vff, _tff = struct.unpack("<fhh", ip.data)
    assert abs(pos - 1.5) < 1e-6                          # 현재 위치 hold(점프 방지)
    assert abs(d._pos_setpoint - 1.5) < 1e-6


def test_set_mode_torque_rejected():
    d, bus = _mk()
    ack = d.apply(bus, "set_mode", {"control_mode": "torque"})
    assert ack["ok"] is False


def test_set_input_pos_sends_frame_and_tracks_setpoint():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "position"})
    bus.sent.clear()
    d.apply(bus, "set_input", {"pos": 4.0})
    ip = _last(bus, C_SET_INPUT_POS)
    pos, _v, _t = struct.unpack("<fhh", ip.data)
    assert abs(pos - 4.0) < 1e-6
    assert abs(d._pos_setpoint - 4.0) < 1e-6


def test_set_input_vel_tracks_vel_setpoint():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "velocity"})
    d.apply(bus, "set_input", {"vel": 2.5})
    iv = _last(bus, C_SET_INPUT_VEL)
    vel, _tff = struct.unpack("<ff", iv.data)
    assert abs(vel - 2.5) < 1e-6
    assert abs(d._vel_setpoint - 2.5) < 1e-6


def test_set_input_no_known_key_rejected():
    d, bus = _mk()
    ack = d.apply(bus, "set_input", {"bogus": 1.0})
    assert ack["ok"] is False


def test_set_gain_partial_vel_merges_cached_pair():
    d, bus = _mk()
    bus.sent.clear()
    d.apply(bus, "set_gain", {"vel_gain": 0.05})
    vg = _last(bus, C_SET_VEL_GAINS)
    g, ig = struct.unpack("<ff", vg.data)
    assert abs(g - 0.05) < 1e-6
    assert abs(ig - 0.0) < 1e-6          # DEFAULT_TUNABLES vel_integrator_gain


def test_set_limit_velocity_mode_no_headroom():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "velocity"})
    bus.sent.clear()
    d.apply(bus, "set_limit", {"vel_limit": 10.0})
    lim = _last(bus, C_SET_LIMITS)
    cap, cur_lim = struct.unpack("<ff", lim.data)
    assert abs(cap - 10.0) < 1e-6        # velocity 모드 = 정확한 캡(헤드룸 없음)


def test_set_limit_traj_mode_has_headroom():
    d, bus = _mk()
    d.apply(bus, "set_mode", {"control_mode": "position_traj"})
    bus.sent.clear()
    d.apply(bus, "set_limit", {"vel_limit": 10.0})
    lim = _last(bus, C_SET_LIMITS)
    cap, _cur = struct.unpack("<ff", lim.data)
    assert abs(cap - 13.0) < 1e-6        # max(10*1.3, 0) = 13 (헤드룸)


def test_set_param_torque_constant_updates_torque_est():
    d, bus = _mk()
    d.on_rx(_iq_msg(0.0, 3.0))
    d.apply(bus, "set_param", {"torque_constant": 0.02})
    s = d.sample()
    assert abs(s["odrive.torque_est"] - 3.0 * 0.02) < 1e-9


def test_set_origin_native_zero_sequence():
    d, bus = _mk()
    d.on_rx(_heartbeat_msg(state=AXIS_CLOSED_LOOP))
    bus.sent.clear()
    d.apply(bus, "set_origin", {})
    cmds = [m.arbitration_id & 0x1F for m in bus.sent if not m.is_remote_frame]
    assert C_SET_LINEAR_COUNT in cmds
    lc = _last(bus, C_SET_LINEAR_COUNT)
    assert struct.unpack("<i", lc.data)[0] == 0
    states = [struct.unpack("<I", m.data)[0] for m in bus.sent
              if (m.arbitration_id & 0x1F) == C_SET_STATE]
    assert states[0] == AXIS_IDLE and states[-1] == AXIS_CLOSED_LOOP
    assert d._pos_setpoint == 0.0


def test_set_state_closed_loop_holds_pos():
    d, bus = _mk()
    d.on_rx(_enc_msg(2.0, 0.0))
    bus.sent.clear()
    d.apply(bus, "set_state", {"state": "closed_loop"})
    ip = _last(bus, C_SET_INPUT_POS)
    pos, _v, _t = struct.unpack("<fhh", ip.data)
    assert abs(pos - 2.0) < 1e-6         # 폐루프 진입 전 현재 위치 hold
    st = _last(bus, C_SET_STATE)
    assert struct.unpack("<I", st.data)[0] == AXIS_CLOSED_LOOP


def test_calibrate_and_clear_and_estop_frames():
    d, bus = _mk()
    bus.sent.clear()
    d.apply(bus, "calibrate", {})
    assert struct.unpack("<I", _last(bus, C_SET_STATE).data)[0] == AXIS_FULL_CALIB
    d.apply(bus, "clear_errors", {})
    assert _last(bus, C_CLEAR_ERR) is not None
    d.apply(bus, "estop", {})
    assert _last(bus, C_ESTOP) is not None


def test_unknown_op_rejected():
    d, bus = _mk()
    ack = d.apply(bus, "frobnicate", {})
    assert ack["ok"] is False


def test_set_param_unknown_key_rejected():
    d, bus = _mk()
    ack = d.apply(bus, "set_param", {"bogus": 1.0})
    assert ack["ok"] is False


def test_request_throttled_to_poll_rate():
    d, bus = _mk()
    bus.sent.clear()
    d.request(bus)                       # 첫 호출: 폴링 발생 (4 RTR)
    n1 = len([m for m in bus.sent if m.is_remote_frame])
    d.request(bus)                       # 즉시 재호출: throttle 로 추가 폴링 없음
    n2 = len([m for m in bus.sent if m.is_remote_frame])
    assert n1 == 4
    assert n2 == 4


def test_request_swallows_send_error():
    class RaisingBus:
        def send(self, msg, timeout=None):
            raise can.CanError("ENOBUFS")
    d = OdriveCanDevice()
    d._bus = RaisingBus()
    d._last_poll = 0.0
    d.request(d._bus)                    # CanError 를 삼키고 예외 전파 안 해야 함 (텔레메트리 보호)
