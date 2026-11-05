"""Bridge tool change against fake registers: teach, full exchange, stop paths."""

import ast
import json
from pathlib import Path
import sys
import threading
import types
from types import SimpleNamespace

import pytest

from dynamixel_control import calib_math
from dynamixel_control.local_tool_change_bridge import LocalToolChangeBridge

SOURCE = Path(__file__).parents[1] / 'dynamixel_control/moveit_dynamixel_bridge.py'


def joint_config():
    tree = ast.parse(SOURCE.read_text())
    node = next(n for n in tree.body if isinstance(n, ast.Assign)
                and any(isinstance(t, ast.Name) and t.id == 'JOINT_CONFIG' for t in n.targets))
    return ast.literal_eval(node.value)


JOINTS = joint_config()


@pytest.fixture(autouse=True)
def fake_bridge_module(monkeypatch):
    module = types.ModuleType('dynamixel_control.moveit_dynamixel_bridge')
    module.JOINT_CONFIG = JOINTS
    for name in ('DXL_EXTENDED_MIN_TICK', 'DXL_EXTENDED_MAX_TICK',
                 'DXL_MINIMUM_POSITION_VALUE', 'DXL_MAXIMUM_POSITION_VALUE'):
        setattr(module, name, getattr(calib_math, name))
    monkeypatch.setitem(sys.modules, module.__name__, module)


class Lock:
    def __init__(self):
        self.state, self.target, self.error = 'UNLOCKED', None, ''
        self.moves = []

    def move(self, desired, direction):
        self.moves.append(desired)
        self.target = desired

    def settle(self):
        if self.target is not None:
            self.state, self.target = self.target, None


class Rig(LocalToolChangeBridge):
    def __init__(self, tmp_path):
        self.now = 0.0
        self.registers = {c['id']: {126: 0, 128: 0, 132: c['center'], 116: c['center'],
                                    112: 50} for c in JOINTS.values()}
        self.local_jog_trip_threshold = 500
        self.writes, self.warnings, self.infos = [], [], []
        self.active_ids = set(self.registers)
        self.read_only = self.mock_mode = self.emergency_stop_active = False
        self.control_mode, self.control_scope = 'MANUAL', 'FULL_ROBOT'
        self.manual_coupling_lock = True
        self.toolchanger = Lock()
        self.tool_discovered, self._physical_tool_detached, self.tool_detached = False, True, False
        self.tool_type = 'spur_1motor_gripper'
        self._pose_move = None
        self._wrist_jog = SimpleNamespace(deadline=None)
        self._bus_lock = threading.RLock()
        self._local_poses = {'home': {'arm_joint_2': 0.3, 'arm_joint_3': 0.6}}
        self.stopped_lock = 0
        self._tc_configure(str(tmp_path / 'slots.json'), list(JOINTS), 0.05, 0.01, 0.02,
                           clock=lambda: self.now)

    def _read_register(self, i, address, *args, **kwargs):
        return self.registers[i][address]

    def _write_register(self, i, address, size, value, label):
        if address == 116 and value >= 1 << 31:
            value -= 1 << 32
        self.writes.append((i, address, value, label))
        self.registers[i][address] = value

    def tick_to_rad(self, joint, tick):
        c = JOINTS[joint]
        return calib_math.tick_to_rad(c['center'], c['direction'], c['gear_ratio'], tick)

    def _joint_center(self, joint):
        return JOINTS[joint]['center']

    def _joint_gear_ratio(self, joint):
        return JOINTS[joint]['gear_ratio']

    def _pose_profile(self, joint, dxl_id):
        return 20

    def _require_manual_lock_arm_stopped(self):
        pass

    def _prepare_tool_for_lock(self):
        pass

    def _stop_toolchanger(self):
        self.stopped_lock += 1

    def _restore_jog_profile(self, move):
        for i in move['ids']:
            self.registers[i][112] = 50

    def get_logger(self):
        return SimpleNamespace(info=self.infos.append, warn=self.warnings.append,
                               error=self.warnings.append)

    def place(self, pose):
        for joint, rad in pose.items():
            c = JOINTS[joint]
            self.registers[c['id']][132] = int(round(calib_math.rad_to_tick(
                c['center'], c['direction'], c['gear_ratio'], rad)))

    def physics(self):
        """Servos reach their goals; the lock reaches its target; detection follows."""
        for regs in self.registers.values():
            regs[132] = regs[116]
        if self.toolchanger is not None:
            self.toolchanger.settle()

    def command(self, **request):
        self._tc_command(SimpleNamespace(data=json.dumps(request)))

    def run(self, steps=400, on_lock=None):
        for _ in range(steps):
            self.now += 0.05
            self._tc_tick()
            self.physics()
            if on_lock:
                on_lock(self)
            if not self._tc_running():
                return


def seat(j1):
    return {'arm_joint_1': j1, 'arm_joint_2': 0.8, 'arm_joint_3': 1.0,
            'arm_joint_4': 0.2, 'arm_joint_5': 0.0}


def above(base):
    """Taught pose over ``base`` with the same head pitch (-q2 + q3 + q4)."""
    return dict(base, arm_joint_2=base['arm_joint_2'] - 0.1,
                arm_joint_3=base['arm_joint_3'] + 0.05, arm_joint_4=base['arm_joint_4'] - 0.15)


def teach(rig):
    for slot, tool, j1 in (('1', 'spur_1motor_gripper', 0.1), ('2', 'dual_motor_gripper', -0.1)):
        for pose, which in ((seat(j1), 'seat'), (above(seat(j1)), 'above')):
            rig.place(pose)
            rig.command(command='CAPTURE', slot=slot, tool_type=tool, pose=which)
            assert not rig._tc_error, rig._tc_error
    snapshot = rig._tc_snapshot()
    assert snapshot['slots'] == {'1': 'spur_1motor_gripper', '2': 'dual_motor_gripper'}
    assert snapshot['incomplete'] == []


def attach_on_lock(rig):
    if rig.toolchanger.state == 'LOCKED':
        rig.tool_discovered, rig._physical_tool_detached = True, False
        rig.tool_type = 'spur_1motor_gripper'


def test_fetch_with_no_tool_descends_locks_rises_and_goes_home(tmp_path):
    rig = Rig(tmp_path)
    teach(rig)
    rig.command(command='START', tool_type='spur_1motor_gripper')
    assert rig._tc_running(), rig._tc_error
    rig.run(on_lock=attach_on_lock)
    assert rig._tool_change.state == 'DONE', rig._tool_change.error
    assert rig.toolchanger.moves == ['LOCKED']
    home = rig._local_poses['home']
    for joint, rad in home.items():
        c = JOINTS[joint]
        assert rig.registers[c['id']][116] == round(calib_math.rad_to_tick(
            c['center'], c['direction'], c['gear_ratio'], rad))
    assert all(regs[112] == 50 for regs in rig.registers.values())


def test_vertical_waypoints_are_streamed_with_matched_joint_speeds(tmp_path):
    rig = Rig(tmp_path)
    teach(rig)
    rig.command(command='START', tool_type='spur_1motor_gripper')
    rig.run(steps=80)
    vertical = [w for w in rig.writes if w[3] == 'tool change waypoint']
    assert len(vertical) >= 5 * len(JOINTS)
    j2 = JOINTS['arm_joint_2']['id']
    profiles = [w[2] for w in rig.writes
                if w[0] == j2 and w[3] == 'tool change vertical profile']
    assert profiles and all(p >= 1 for p in profiles)


def test_exchange_returns_the_attached_tool_first(tmp_path):
    rig = Rig(tmp_path)
    teach(rig)
    rig.tool_discovered, rig._physical_tool_detached = True, False
    rig.toolchanger.state = 'LOCKED'

    def detection(rig):
        if rig.toolchanger.state == 'UNLOCKED':
            rig.tool_discovered, rig._physical_tool_detached = False, True
        elif rig.toolchanger.state == 'LOCKED' and rig.toolchanger.moves == ['UNLOCKED', 'LOCKED']:
            rig.tool_discovered, rig._physical_tool_detached = True, False
            rig.tool_type = 'dual_motor_gripper'

    rig.command(command='START', tool_type='dual_motor_gripper')
    rig.run(on_lock=detection)
    assert rig._tool_change.state == 'DONE', rig._tool_change.error
    assert rig.toolchanger.moves == ['UNLOCKED', 'LOCKED']


def test_stop_command_holds_the_arm_and_stops_the_lock(tmp_path):
    rig = Rig(tmp_path)
    teach(rig)
    rig.command(command='START', tool_type='spur_1motor_gripper')
    rig.run(steps=3)
    rig.command(command='STOP')
    assert rig._tool_change.state == 'ABORTED'
    assert rig.stopped_lock == 1
    for regs in rig.registers.values():
        assert regs[116] == regs[132]


@pytest.mark.parametrize('change', ['fsm', 'estop', 'missing_joint', 'moving', 'no_home'])
def test_start_is_refused_when_unsafe(tmp_path, change):
    rig = Rig(tmp_path)
    teach(rig)
    if change == 'fsm':
        rig.control_mode = 'FSM'
    elif change == 'estop':
        rig.emergency_stop_active = True
    elif change == 'missing_joint':
        rig.active_ids.discard(JOINTS['arm_joint_4']['id'])
    elif change == 'moving':
        rig.registers[JOINTS['arm_joint_2']['id']][128] = 3
    else:
        rig._local_poses = {}
    before = list(rig.writes)
    rig.command(command='START', tool_type='spur_1motor_gripper')
    assert not rig._tc_running() and rig._tc_error
    assert rig.writes == before


def test_losing_manual_mode_mid_run_aborts_and_holds(tmp_path):
    rig = Rig(tmp_path)
    teach(rig)
    rig.command(command='START', tool_type='spur_1motor_gripper')
    rig.run(steps=2)
    rig.control_mode = 'FSM'
    rig.now += 0.05
    rig._tc_tick()
    assert rig._tool_change.state == 'ABORTED'
    assert 'MANUAL' in rig._tool_change.error


def test_dry_run_moves_the_arm_through_the_slot_without_a_tool_or_lock(tmp_path):
    rig = Rig(tmp_path)
    teach(rig)
    rig.toolchanger = None
    rig.command(command='START', tool_type='spur_1motor_gripper', dry_run=True)
    assert rig._tc_running(), rig._tc_error
    rig.run()
    assert rig._tool_change.state == 'DONE', rig._tool_change.error
    assert [w for w in rig.writes if w[3] == 'tool change waypoint']


def test_without_dry_run_a_missing_coupling_lock_still_refuses(tmp_path):
    rig = Rig(tmp_path)
    teach(rig)
    rig.toolchanger = None
    rig.command(command='START', tool_type='spur_1motor_gripper')
    assert not rig._tc_running() and 'coupling lock' in rig._tc_error


def test_capture_with_the_limit_switch_off_accepts_a_seat_past_a_limit(tmp_path):
    rig = Rig(tmp_path)
    pose = dict(seat(0.1), arm_joint_2=1.9)
    rig.place(pose)
    rig.command(command='CAPTURE', slot='1', tool_type='spur_1motor_gripper')
    assert 'joint limit' in rig._tc_error
    rig.command(command='CAPTURE', slot='1', tool_type='spur_1motor_gripper', limits=False)
    assert not rig._tc_error and '1' in rig._tc_store.data['slots']


@pytest.mark.parametrize('trip', [True, False])
def test_current_trip_aborts_the_exchange_only_when_switched_on(tmp_path, trip):
    rig = Rig(tmp_path)
    rig.local_jog_trip_threshold = 500
    for regs in rig.registers.values():
        regs[126] = 0
    teach(rig)
    rig.command(command='START', tool_type='spur_1motor_gripper', dry_run=True, trip=trip)
    rig.run(steps=3)
    rig.registers[JOINTS['arm_joint_2']['id']][126] = -900
    rig.run()
    if trip:
        assert rig._tool_change.state == 'ABORTED'
        assert 'current trip' in rig._tool_change.error
    else:
        assert rig._tool_change.state == 'DONE', rig._tool_change.error


def test_a_slot_without_the_above_pose_reports_incomplete_and_cannot_start(tmp_path):
    rig = Rig(tmp_path)
    rig.place(seat(0.1))
    rig.command(command='CAPTURE', slot='1', tool_type='spur_1motor_gripper', pose='seat')
    assert rig._tc_snapshot()['incomplete'] == ['1']
    rig.command(command='START', tool_type='spur_1motor_gripper', dry_run=True)
    assert not rig._tc_running() and '5 cm above' in rig._tc_error
