"""Execute real bridge methods against fake registers, without ROS or motors."""

import ast
import json
import math
from pathlib import Path
import threading
from types import SimpleNamespace

import pytest

from dynamixel_control import calib_math, joint_limits
from dynamixel_control.local_wrist_jog import WristJogLease
from dynamixel_control.toolchanger_lock import LockError
from dynamixel_control.toolchanger_lock import ToolchangerLock
from dynamixel_control.toolchanger_sequence import ToolchangerSequence
from test_toolchanger_lock import IO


SOURCE = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'


def bridge():
    tree = ast.parse(SOURCE.read_text())
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef)
               and n.name == 'MoveItDynamixelBridge')
    names = {'_local_pose_command', '_local_freedrive', '_local_pose_watch',
             '_hold_local_pose', '_restore_jog_profile', '_pose_profile',
             '_require_manual_lock_arm_stopped', '_toolchanger_command',
             '_toolchanger_command_locked', '_toolchanger_snapshot', '_local_wrist_command',
             '_hold_local_wrist', '_wrist_lock_ok', '_local_wrist_deadman'}
    methods = [n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name in names]
    config = next(n for n in tree.body if isinstance(n, ast.Assign)
                  and any(isinstance(t, ast.Name) and t.id == 'JOINT_CONFIG'
                          for t in n.targets))
    joints = ast.literal_eval(config.value)
    clock = [100.0]
    namespace = dict(json=json, math=math, time=SimpleNamespace(monotonic=lambda: clock[0]),
                     JOINT_CONFIG=joints, ARM_IDS={c['id'] for c in joints.values()},
                     calib_math=calib_math, joint_limits=joint_limits, LockError=LockError,
                     String=SimpleNamespace,
                     DXL_EXTENDED_MIN_TICK=-1048575, DXL_EXTENDED_MAX_TICK=1048575,
                     DXL_MINIMUM_POSITION_VALUE=0, DXL_MAXIMUM_POSITION_VALUE=4095)
    exec(compile(ast.Module(body=methods, type_ignores=[]), str(SOURCE), 'exec'), namespace)
    b = SimpleNamespace(
        control_mode='MANUAL', control_scope='FULL_ROBOT', read_only=False,
        emergency_stop_active=False, mock_mode=False, _bus_lock=threading.RLock(),
        _wrist_jog=SimpleNamespace(deadline=None), _pose_move=None,
        toolchanger=SimpleNamespace(target=None, state='UNLOCKED'),
        LOCAL_POSE_EXCLUDED=('arm_joint_5',), active_ids={11, 14, 13, 12},
        torque_enabled_ids=set(), arm_profile_velocity=50,
        local_pose_joint_speeds=[10.0, 30.0, 15.0, 15.0],
        local_pose_joint_delays=[0.0, 0.0, 0.0, 1.0], local_jog_trip_threshold=500,
        local_wrist_lock_gate=True, _lock_ready=lambda: False,
        _local_poses={'home': {j: 0.1 for j in joints if j != 'arm_joint_5'}},
        writes=[], warnings=[], clock=clock, registers={},
        _tc_running=lambda: False,
    )
    for j, c in joints.items():
        b.registers[c['id']] = {44: 100, 70: 0, 126: 0, 128: 0,
                                132: c['center'], 116: c['center'], 64: 1}
    b._joint_center = lambda j: joints[j]['center']
    b._joint_gear_ratio = lambda j: joints[j]['gear_ratio']
    b._read_register = lambda i, address, *a, **kw: b.registers[i][address]

    def write(i, address, size, value, label):
        b.writes.append((i, address, value, label))
        b.registers[i][address] = value

    b._write_register = write
    b.get_logger = lambda: SimpleNamespace(info=lambda _: None, warn=b.warnings.append)
    for name in names:
        setattr(b, name, namespace[name].__get__(b))
    return b


def send(b, command, **params):
    b._local_pose_command(SimpleNamespace(data=json.dumps(dict(command=command, **params))))


@pytest.mark.parametrize('command', ['FREEDRIVE', 'FREEDRIVE_RELEASE'])
@pytest.mark.parametrize('field,value', [
    ('emergency_stop_active', True), ('read_only', True),
    ('control_mode', 'FSM'), ('control_scope', 'TOOL_CHANGE'),
    ('control_scope', 'END_EFFECTOR_ONLY'),
])
def test_freedrive_rejects_unsafe_control_without_writes(command, field, value):
    b = bridge()
    setattr(b, field, value)
    send(b, command)
    assert b.warnings
    assert not b.writes


def test_freedrive_release_rechecks_estop_after_position_read():
    b = bridge()
    read = b._read_register

    def read_then_estop(*args, **kwargs):
        value = read(*args, **kwargs)
        b.emergency_stop_active = True
        return value

    b._read_register = read_then_estop
    send(b, 'FREEDRIVE_RELEASE')
    assert not any(address == 64 and value == 1 for _, address, value, _ in b.writes)


def test_normal_freedrive_release_sets_present_goal_before_torque_on():
    b = bridge()
    send(b, 'FREEDRIVE')
    assert b.torque_enabled_ids == set()
    b.writes.clear()
    send(b, 'FREEDRIVE_RELEASE')
    assert b.torque_enabled_ids == b.active_ids
    for i in b.active_ids:
        writes = [(a, v) for motor, a, v, _ in b.writes if motor == i]
        assert writes == [(116, b.registers[i][132]), (64, 1)]
    assert not b.warnings


@pytest.mark.parametrize('level', [0, 1, 5, 10])
def test_keyboard_speed_scales_only_next_bounded_jog_and_keeps_deadman(level):
    b = bridge()
    b.local_jog_all_axes = True
    b.manual_coupling_lock = True
    b.local_jog_step_rad = .14
    b.local_wrist_lock_gate = False
    b._wrist_jog = WristJogLease()
    initial = b.registers[11][132]
    b._local_wrist_command(SimpleNamespace(data=json.dumps(dict(command='JOG',
        axis=1, direction=1, sequence=1, expires_at_s=100.6, speed_level=level,
        limits=False, trip=False))))
    assert b.registers[11][116] == initial + round(round(.14 * 4096 / (2 * math.pi)) * level / 5)
    assert not b.warnings
    b.clock[0] = 100.7
    b._local_wrist_deadman()
    assert b._wrist_jog.deadline is None and b.registers[11][116] == initial
    assert b.registers[11][64] == 1


def test_jog_release_decelerates_to_a_stop_ahead_instead_of_snapping_back():
    b = bridge()
    regs = b.registers[11]
    regs[132], regs[116], regs[128], regs[108] = 2000, 2300, 50, 25
    b._local_jog_id = 11
    b._hold_local_wrist()
    # 50 x 0.229 rev/min under 25 x 214.577 rev/min^2 stops in about 50 ticks.
    stop = 2000 + round((50 * .229) ** 2 / (2 * 25 * 214.577) * 4096)
    assert regs[116] == stop and 2000 < stop < 2300


def test_jog_release_never_stops_beyond_the_commanded_goal_or_reverses():
    b = bridge()
    regs = b.registers[11]
    regs[132], regs[116], regs[128], regs[108] = 2000, 2010, 50, 25
    b._local_jog_id = 11
    b._hold_local_wrist()
    assert regs[116] == 2010
    regs[132], regs[116], regs[128] = 2000, 1990, -50
    b._hold_local_wrist()
    assert regs[116] == 1990
    regs[132], regs[116], regs[128], regs[108] = 2000, 2300, 50, 0
    b._hold_local_wrist()
    assert regs[116] == 2000  # acceleration 0 means no profile: hold in place


def test_delayed_pose_blocks_lock_without_interrupting_pose():
    b = bridge()
    send(b, 'HOME')
    move, before = b._pose_move, list(b.writes)
    assert move['pending']
    # The arm looks stationary while a delayed goal is still scheduled.
    for regs in b.registers.values():
        regs[132] = regs[116]
    with pytest.raises(LockError, match='pose'):
        b._require_manual_lock_arm_stopped()
    assert b._pose_move is move
    assert b.writes == before


def test_pose_while_lock_moves_is_rejected_without_hold_or_goal_write():
    b = bridge()
    b.toolchanger.target = 2799
    send(b, 'HOME')
    assert b._pose_move is None
    assert not b.writes
    assert b.warnings
    b.toolchanger.target = None
    send(b, 'HOME')
    assert b._pose_move is not None


@pytest.mark.parametrize('command', ['FREEDRIVE', 'FREEDRIVE_RELEASE'])
def test_lock_motion_blocks_freedrive_without_writes(command):
    b = bridge()
    b.toolchanger.target = 2799
    send(b, command)
    assert not b.writes


def test_rejected_request_does_not_stop_an_existing_pose():
    b = bridge()
    send(b, 'HOME')
    move, before = b._pose_move, list(b.writes)
    b.read_only = True
    send(b, 'FREEDRIVE_RELEASE')
    assert b._pose_move is move
    assert b.writes == before


def test_pose_write_failure_still_holds_started_joints():
    b = bridge()
    write = b._write_register

    def fail_second_goal(i, address, size, value, label):
        if i == 14 and label == 'pose goal':
            raise RuntimeError('fixture failed goal write')
        write(i, address, size, value, label)

    b._write_register = fail_second_goal
    send(b, 'HOME')
    assert b._pose_move is None
    assert {i for i, _, _, label in b.writes if label == 'pose hold'} == {11, 14}
    assert b.warnings


def test_pose_preserves_profile_delay_and_finishes_without_lock_readiness_gate():
    b = bridge()
    send(b, 'HOME')
    assert b._pose_move is not None  # UNLOCKED and lock gate=true must remain allowed.
    profiles = {i: v for i, a, v, _ in b.writes if a == 112}
    assert profiles == {11: 7, 14: 100, 13: 44, 12: 11}
    goals = lambda: [w for w in b.writes if w[3] in ('pose goal', 'pose delayed goal')]
    assert [w[0] for w in goals()] == [11, 14, 13]
    for _ in range(9):
        b.clock[0] += 0.1
        b._local_pose_watch()
    assert len(goals()) == 3
    b.clock[0] = 101.0
    b._local_pose_watch()
    assert [w[0] for w in goals()] == [11, 14, 13, 12]
    for regs in b.registers.values():
        regs[132] = regs[116]
    b._local_pose_watch()
    assert b._pose_move is None
    assert not any(w[3] == 'pose hold' for w in b.writes)
    b._require_manual_lock_arm_stopped()
    assert not b.warnings


def test_pose_and_lock_callbacks_take_turns_without_stopping_normal_motion():
    b = bridge()
    io = IO()
    b.toolchanger = ToolchangerLock(io)
    b.toolchanger.capture_zero('LOCKED')
    b.toolchanger.enable()
    io.follow = False
    b.manual_coupling_lock = True
    b.lock_sequence = ToolchangerSequence(b.toolchanger, lambda: None)
    replies = []
    b.toolchanger_status_pub = SimpleNamespace(publish=lambda m: replies.append(json.loads(m.data)))

    def poll():
        b.toolchanger.observe()
        b.lock_sequence.poll()

    b._poll_toolchanger = poll
    lock_request = SimpleNamespace(data=json.dumps(
        {'command': 'CHANGE', 'to': 'UNLOCKED', 'direction': 1}))
    send(b, 'HOME')
    arm_writes, lock_writes = list(b.writes), list(io.writes)
    b._toolchanger_command(lock_request)
    assert 'pose' in replies[-1]['command_error']
    assert b.writes == arm_writes and io.writes == lock_writes
    assert b._pose_move['pending']

    b.clock[0] = 101.0
    b._local_pose_watch()
    for regs in b.registers.values():
        regs[132] = regs[116]
    b._local_pose_watch()
    assert b._pose_move is None
    assert not any(label == 'pose hold' for _, _, _, label in b.writes)
    b._toolchanger_command(lock_request)
    assert b.toolchanger.target is not None
    arm_writes = list(b.writes)
    send(b, 'HOME')
    assert b._pose_move is None and b.writes == arm_writes

    io.sample['position'] = b.toolchanger.target
    poll()
    assert b.toolchanger.target is None and b.toolchanger.state == 'UNLOCKED'
    send(b, 'HOME')
    assert b._pose_move is not None
