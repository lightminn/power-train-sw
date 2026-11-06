"""Keyboard axis 6 drives the fitted tool's motor: excavator ID5, rack gripper ID3."""

import threading
from types import SimpleNamespace

import pytest

from dynamixel_control.moveit_dynamixel_bridge import MoveItDynamixelBridge as Bridge


class Bus:
    def __init__(self, position=2608, torque=0, mode=3, hw=0):
        self.reg = {11: mode, 64: torque, 70: hw, 116: position, 132: position}
        self.id4_torque = 1
        self.writes = []

    def read(self, dxl_id, addr, _length, _what, signed=False):
        value = self.id4_torque if dxl_id == 4 else self.reg[addr]
        return value - (1 << 32) if signed and value >= 1 << 31 else value

    def write(self, dxl_id, addr, _length, value, _what):
        self.writes.append((dxl_id, addr, value))
        if dxl_id == 4:
            self.id4_torque = value
        else:
            self.reg[addr] = value


def node(bus, **overrides):
    fields = dict(
        _read_register=bus.read, _write_register=bus.write, control_mode='MANUAL',
        emergency_stop_active=False, _wrist_lock_ok=lambda: True,
        local_jog_step_rad=0.05, tool_type='spur_1motor_gripper', tool_discovered=True,
        tool_detached=False, _physical_tool_detached=False, _tool_samples={},
        tool_profile=dict(goal_pwm=400, profile_acceleration=5, profile_velocity=20))
    fields.update(overrides)
    target = SimpleNamespace(**fields)
    target._enable_tool_axis = lambda *args: Bridge._enable_tool_axis(target, *args)
    return target


def jog(target, direction=1, level=5, motor=(5, 3)):
    Bridge._jog_tool_axis(target, *motor, {'speed_level': level}, direction)


def test_the_first_jog_applies_limits_before_torque_and_syncs_the_goal():
    bus = Bus()
    jog(node(bus))
    assert bus.writes[:5] == [(5, 100, 400), (5, 108, 5), (5, 112, 20),
                              (5, 116, 2608), (5, 64, 1)]
    assert bus.writes[-1] == (5, 116, 2608 + 33)


def test_later_jogs_leave_torque_and_limits_alone():
    bus = Bus(torque=1)
    jog(node(bus), direction=-1)
    assert bus.writes == [(5, 116, 2608 - 33)]


@pytest.mark.parametrize('position,direction,goal', [(4090, 1, 4095), (10, -1, 0)])
def test_the_goal_stays_inside_one_turn(position, direction, goal):
    bus = Bus(position=position, torque=1)
    jog(node(bus), direction=direction)
    assert bus.writes == [(5, 116, goal)]


@pytest.mark.parametrize('overrides,bus_args', [
    ({'emergency_stop_active': True}, {}),
    ({'control_mode': 'FSM'}, {}),
    ({'_wrist_lock_ok': lambda: False}, {}),
    ({}, {'hw': 32}),
])
def test_a_closed_gate_writes_nothing(overrides, bus_args):
    bus = Bus(**bus_args)
    with pytest.raises(RuntimeError):
        jog(node(bus, **overrides))
    assert bus.writes == []


def test_the_goal_builds_up_while_the_motor_lags_and_stops_at_the_lead():
    bus = Bus(position=-3900, torque=1, mode=4)
    bus.reg[116] = -3800
    jog(node(bus), motor=(3, 4))
    assert bus.writes == [(3, 116, (-3800 + 33) & 0xffffffff)]
    bus.reg[116] = -3650
    jog(node(bus), motor=(3, 4))
    assert bus.writes[-1] == (3, 116, (-3900 + 256) & 0xffffffff)


def test_the_jog_refreshes_the_displayed_position():
    bus = Bus(torque=1)
    target = node(bus, _tool_samples={5: {'position': None}})
    jog(target)
    assert target._tool_samples[5]['position'] == 2608


def test_a_wrong_mode_never_gets_torque():
    bus = Bus(mode=1)
    with pytest.raises(RuntimeError, match='operating mode 3'):
        jog(node(bus))
    assert (5, 64, 1) not in bus.writes


@pytest.mark.parametrize('overrides,motor', [
    ({}, (5, 3)), ({'tool_type': 'dual_motor_gripper'}, (3, 4)),
    ({'tool_type': 'cleaner'}, None), ({'tool_discovered': False}, None),
    ({'_physical_tool_detached': True}, None), ({'tool_detached': True}, None)])
def test_axis_six_needs_a_fitted_tool(overrides, motor):
    assert Bridge._tool_axis_motor(node(Bus(), **overrides)) == motor


def test_the_rack_gripper_drives_id3_with_id4_limp():
    bus = Bus(position=-3969, mode=4)
    jog(node(bus, tool_type='dual_motor_gripper', tool_profile=dict(
        goal_pwm=280, axis_goal_pwm=800, profile_acceleration=25, profile_velocity=80)),
        motor=(3, 4))
    assert bus.writes[:2] == [(4, 64, 0), (3, 100, 800)]
    assert bus.writes.index((4, 64, 0)) < bus.writes.index((3, 64, 1))
    assert (3, 116, (-3969) & 0xffffffff) in bus.writes
    assert bus.writes[-1] == (3, 116, (-3969 + 33) & 0xffffffff)


def test_a_failed_hold_cannot_kill_the_bridge():
    def fail(*_args, **_kwargs):
        raise RuntimeError('read failed')

    logged = []
    target = SimpleNamespace(
        _hold_local_wrist_position=fail, _wrist_jog=SimpleNamespace(deadline=1.0),
        get_logger=lambda: SimpleNamespace(error=logged.append))
    Bridge._hold_local_wrist(target)
    assert target._wrist_jog.deadline is None and logged


def test_goal_pwm_is_a_two_byte_register_write():
    written = []

    def write(_port, dxl_id, addr, value):
        written.append((dxl_id, addr, value))
        return 0, 0

    target = SimpleNamespace(
        port_handler=None,
        packet_handler=SimpleNamespace(
            write1ByteTxRx=None, write4ByteTxRx=None, write2ByteTxRx=write))
    Bridge._write_register(target, 5, 100, 2, 400, 'excavator PWM')
    assert written == [(5, 100, 400)]


def test_a_joint_resting_short_of_its_goal_counts_as_stopped():
    regs = {70: 0, 128: 0, 132: 2625, 116: 2700}
    target = SimpleNamespace(
        _require_no_manual_arm_command=lambda: None, mock_mode=False, active_ids={11},
        _bus_lock=threading.RLock(),
        _read_register=lambda _i, addr, *_a, **_k: regs[addr])
    Bridge._require_manual_lock_arm_stopped(target)


def test_pose_speeds_need_five_values_inside_the_range():
    from dynamixel_control.moveit_dynamixel_bridge import _valid_pose_speeds
    assert _valid_pose_speeds([37, 16, 15, 20, 27]) == [37.0, 16.0, 15.0, 20.0, 27.0]
    for bad in ([1, 2, 3, 4], [1, 2, 3, 4, 61], [1, 2, 3, 4, 0], 'fast'):
        with pytest.raises(ValueError):
            _valid_pose_speeds(bad)


def test_set_speeds_reach_the_next_pose_and_slot_move(tmp_path):
    saved = tmp_path / 'speeds.json'
    target = SimpleNamespace(
        _tc_running=lambda: False, _bus_lock=threading.RLock(),
        local_pose_joint_speeds=[10.0] * 5, local_pose_speeds_file=str(saved),
        get_logger=lambda: SimpleNamespace(info=lambda _m: None),
        _joint_gear_ratio=lambda _j: 1.0,
        _read_register=lambda *_a: 1000)
    message = SimpleNamespace(data='{"command": "SPEEDS", "speeds": [30, 10, 10, 10, 10]}')
    Bridge._local_pose_command(target, message)
    assert saved.read_text() == '[30.0, 10.0, 10.0, 10.0, 10.0]'
    assert Bridge._pose_profile(target, 'arm_joint_1', 11) == round(30 / 1.374)


def test_a_pose_move_that_stops_short_ends_after_settling(monkeypatch):
    now = [100.0]
    monkeypatch.setattr('dynamixel_control.moveit_dynamixel_bridge.time.monotonic',
                        lambda: now[0])
    regs = {132: 2625, 116: 2700}
    logged, restored = [], []
    target = SimpleNamespace(
        _pose_move={'ids': [11], 'trip': False, 'starts': {11: 99.0}, 'pending': []},
        control_mode='MANUAL', emergency_stop_active=False, _bus_lock=threading.RLock(),
        _read_register=lambda _i, addr, *_a, **_k: regs[addr],
        _restore_jog_profile=restored.append,
        get_logger=lambda: SimpleNamespace(warn=logged.append))
    Bridge._local_pose_watch(target)
    assert target._pose_move is not None
    now[0] += 0.5
    Bridge._local_pose_watch(target)
    assert target._pose_move is not None
    now[0] += 0.6
    Bridge._local_pose_watch(target)
    assert target._pose_move is None and restored and logged
