"""Bench tool change: return the attached tool to its slot, fetch the requested one.

Each slot is taught twice with the tool head pointing at the floor: the seated
(locked) pose and the pose about 5 cm straight above it.  Every visit is: move
to the above pose -> descend through interpolated joint waypoints -> lock or
unlock -> ascend the same way.  The run ends at the home pose.  ROS and motor
I/O belong to the caller; this module only orders the steps and refuses to
advance until the caller confirms arrival.
"""

import json
import math
import os
from pathlib import Path
import tempfile
import time

from dynamixel_control.joint_limits import JOINT_LIMITS
from dynamixel_control.tool_profiles import SUPPORTED_TOOL_TYPES
from dynamixel_control.vertical_path import joint_path, pitch_difference

SLOT_POSES = ('seat', 'above')
# Both taught poses must point the head the same way (hand-teaching error).
MAX_HEAD_DIFFERENCE_RAD = math.radians(5)


def _check_pose(slot_id, label, pose, joint_names, checked):
    if not isinstance(pose, dict) or set(pose) != set(joint_names):
        raise ValueError(f'slot {slot_id}: {label} must hold every arm joint')
    for name, value in pose.items():
        limits = JOINT_LIMITS[name]
        if type(value) not in (int, float) or not math.isfinite(value):
            raise ValueError(f'slot {slot_id}: invalid {label} value {name}')
        if checked and not limits['lower'] <= value <= limits['upper']:
            raise ValueError(f'slot {slot_id}: {label} outside joint limit {name}')


def validate_slots(data, joint_names):
    if not isinstance(data, dict) or data.get('version') != 1:
        raise ValueError('tool slot table version must be 1')
    slots = data.get('slots')
    if not isinstance(slots, dict):
        raise ValueError('tool slot table needs a slots object')
    tools = []
    for slot_id, slot in slots.items():
        if slot.get('tool_type') not in SUPPORTED_TOOL_TYPES:
            raise ValueError(f'slot {slot_id}: unknown tool type')
        tools.append(slot['tool_type'])
        # A pose taught with the bench limit switch OFF is stored as such.
        checked = slot.get('limits_checked', True) is not False
        taught = [label for label in SLOT_POSES if label in slot]
        if not taught:
            raise ValueError(f'slot {slot_id}: no taught pose')
        for label in taught:
            _check_pose(slot_id, label, slot[label], joint_names, checked)
        if len(taught) == 2 and pitch_difference(slot['seat'], slot['above']) > \
                MAX_HEAD_DIFFERENCE_RAD:
            raise ValueError(f'slot {slot_id}: seat and above head directions differ')
    if len(tools) != len(set(tools)):
        raise ValueError('each tool type must have exactly one slot')
    return data


def slot_complete(slot):
    return all(label in slot for label in SLOT_POSES)


class ToolSlotStore:
    """Slot table kept on disk; a capture is checked before it replaces the file."""

    def __init__(self, filename, joint_names):
        self.filename = filename
        self.joint_names = list(joint_names)
        self.data = dict(version=1, slots={})

    def load(self):
        if self.filename and Path(self.filename).is_file():
            data = json.loads(Path(self.filename).read_text())
            self.data = validate_slots(data, self.joint_names)
        return self.data

    def capture(self, slot_id, tool_type, pose, which='seat', limits=True):
        """Store the seat or the above pose of one slot.

        Re-teaching the seat drops a previous above pose so the pair always
        comes from one teaching session.  The above pose needs the seat first.
        """
        if not self.filename:
            raise ValueError('tool slot file is not configured')
        if not isinstance(slot_id, str) or not slot_id:
            raise ValueError('slot id is required')
        if which not in SLOT_POSES:
            raise ValueError('pose must be seat or above')
        pose = {name: float(pose[name]) for name in self.joint_names}
        data = json.loads(json.dumps(self.data))
        old = data['slots'].get(slot_id, {})
        if which == 'seat':
            slot = dict(tool_type=tool_type, seat=pose)
        else:
            if 'seat' not in old or old.get('tool_type') != tool_type:
                raise ValueError(f'teach the seat pose of slot {slot_id} for this tool first')
            slot = dict(old, above=pose)
        checked = bool(limits)
        if which == 'above':
            checked = checked and old.get('limits_checked', True) is not False
        slot.update(captured_at=time.time(), limits_checked=checked)
        data['slots'] = {key: value for key, value in data['slots'].items()
                         if value['tool_type'] != tool_type or key == slot_id}
        data['slots'][slot_id] = slot
        validate_slots(data, self.joint_names)
        self._save(data)
        self.data = data

    def _save(self, data):
        path = Path(self.filename)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile('w', dir=path.parent, delete=False) as stream:
                temporary = stream.name
                json.dump(data, stream, allow_nan=False, indent=1)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)


def plan(slots, attached_tool, desired_tool, home, *, rise_m, step_m, dry_run=False,
         limits=True):
    """Ordered steps; each is (name, payload).

    ``rise_m``/``step_m`` only set how many waypoints split the taught rise.
    ``dry_run`` rehearses the arm motion only: no coupling move and no tool
    detection check, for benches without a tool or coupling lock.
    """
    by_tool = {slot['tool_type']: key for key, slot in slots['slots'].items()}
    if desired_tool not in by_tool:
        raise ValueError(f'no taught slot for {desired_tool}')
    if attached_tool == desired_tool:
        raise ValueError('requested tool is already attached')
    if attached_tool is not None and attached_tool not in by_tool:
        raise ValueError(f'no taught slot to return {attached_tool}')
    if not 0 < step_m <= rise_m:
        raise ValueError('rise and step must be positive with step <= rise')
    count = math.ceil(rise_m / step_m - 1e-9)
    steps = []

    def visit(slot_id, lock):
        slot = slots['slots'][slot_id]
        if not slot_complete(slot):
            raise ValueError(f'slot {slot_id}: teach the pose 5 cm above the seat')
        path = joint_path(slot['seat'], slot['above'], count, limits=limits)
        steps.extend([
            ('MOVE_ABOVE', dict(slot=slot_id, poses=[path[-1]])),
            ('DESCEND', dict(slot=slot_id, poses=path[-2::-1])),
            *([] if dry_run else [(lock, dict(slot=slot_id))]),
            ('ASCEND', dict(slot=slot_id, poses=path[1:])),
        ])

    if attached_tool is not None:
        visit(by_tool[attached_tool], 'UNLOCK')
        if not dry_run:
            steps.append(('VERIFY_RELEASED', dict(tool=attached_tool)))
    visit(by_tool[desired_tool], 'LOCK')
    if not dry_run:
        steps.append(('VERIFY_ATTACHED', dict(tool=desired_tool)))
    steps.append(('HOME', dict(poses=[home])))
    return steps


class BenchToolChange:
    """Runs a plan one step at a time; any failure holds the arm and stops the lock.

    Callbacks: ``move(poses, vertical)`` starts streaming joint poses (vertical
    moves are straight-line descents/ascents), ``arrived(pose)`` is true once the
    arm sits stationary on ``pose``, ``lock(desired)`` starts a coupling move to
    'LOCKED'/'UNLOCKED', ``lock_state()`` returns 'MOVING', 'DONE' or an error
    string, ``attached_tool()`` reports the detected tool or None, ``hold()``
    freezes the arm, ``stop_lock()`` stops the coupling, ``permission()`` raises
    when manual control is no longer granted.
    """

    STEP_TIMEOUT_S = dict(MOVE_ABOVE=20.0, DESCEND=15.0, ASCEND=15.0, HOME=20.0,
                          LOCK=10.0, UNLOCK=10.0, VERIFY_RELEASED=5.0,
                          VERIFY_ATTACHED=5.0)

    def __init__(self, *, move, arrived, lock, lock_state, attached_tool, hold,
                 stop_lock, permission, clock=time.monotonic):
        self._move, self._arrived = move, arrived
        self._lock, self._lock_state = lock, lock_state
        self._attached_tool = attached_tool
        self._hold, self._stop_lock = hold, stop_lock
        self._permission = permission
        self._clock = clock
        self.state, self.error = 'IDLE', ''
        self.steps, self.index = [], -1
        self.slot = None

    @property
    def running(self):
        return self.state not in ('IDLE', 'DONE', 'ABORTED')

    def start(self, steps):
        if self.running:
            raise ValueError('a tool change is already running')
        self._permission()
        self.steps, self.index, self.error = list(steps), -1, ''
        try:
            self._advance()
        except Exception as exc:
            self.abort(str(exc))
            raise

    def poll(self):
        if not self.running:
            return self.state
        try:
            self._permission()
            name, payload = self.steps[self.index]
            if self._clock() > self._deadline:
                raise RuntimeError(f'{name} timed out')
            if name in ('LOCK', 'UNLOCK'):
                status = self._lock_state()
                if status == 'DONE':
                    self._advance()
                elif status != 'MOVING':
                    raise RuntimeError(f'{name} failed: {status}')
            elif name == 'VERIFY_RELEASED':
                if self._attached_tool() is None:
                    self._advance()
            elif name == 'VERIFY_ATTACHED':
                if self._attached_tool() == payload['tool']:
                    self._advance()
            elif self._arrived(payload['poses'][-1]):
                self._advance()
        except Exception as exc:
            self.abort(str(exc))
        return self.state

    def _advance(self):
        self.index += 1
        if self.index == len(self.steps):
            self.state = 'DONE'
            return
        name, payload = self.steps[self.index]
        self.state, self.slot = name, payload.get('slot', self.slot)
        self._deadline = self._clock() + self.STEP_TIMEOUT_S[name]
        if name in ('LOCK', 'UNLOCK'):
            self._lock('LOCKED' if name == 'LOCK' else 'UNLOCKED')
        elif 'poses' in payload:
            self._move(payload['poses'], vertical=name in ('DESCEND', 'ASCEND'))

    def abort(self, reason):
        self.state, self.error = 'ABORTED', str(reason)
        for action in (self._stop_lock, self._hold):
            try:
                action()
            except Exception as exc:
                self.error += f'; {exc}'
        return self.state
