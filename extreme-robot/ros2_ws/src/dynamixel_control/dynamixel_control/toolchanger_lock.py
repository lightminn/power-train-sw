"""ID1의 영점 기준 인덱스 잠금. 기계적 결합 센서 판정을 대신하지 않는다."""

import json
import math
import os
from pathlib import Path
import tempfile
import time


class LockError(RuntimeError):
    pass


class ToolchangerLock:
    actuator_id = 1
    model_number = 1060
    step_ticks = 256  # 4096 tick/rev, 1:1, 22.5 degrees.
    tolerance = 4
    # Bench-only two-point mode: states come from two recorded absolute ticks
    # (within state_tolerance), and LOCK/UNLOCK move straight to them.
    two_point = False
    locked_tick = None
    unlocked_tick = None
    state_tolerance = 20
    # (state, tick) assumed after an uncalibrated 22.5-degree move.
    assumed = None

    def __init__(self, io, zero_tick=None, zero_state='UNKNOWN', clock=time.monotonic):
        self.io = io
        self.clock = clock
        self.zero_tick = zero_tick
        self.zero_state = zero_state
        self.sample = None
        self.state = 'UNCALIBRATED'
        self.error = ''
        self.target = None
        self.deadline = None
        self.observed_at = None
        if zero_tick is not None:
            self._validate_zero(zero_tick, zero_state)

    @staticmethod
    def _validate_zero(tick, state):
        if type(tick) is not int or not 0 <= tick <= 4095:
            raise LockError('zero tick must be in [0,4095]')
        if state not in ('LOCKED', 'UNLOCKED'):
            raise LockError('zero state must be LOCKED or UNLOCKED')

    def observe(self):
        try:
            sample = self.io.read()
            if (sample['model'] != self.model_number or sample['mode'] != 3
                    or sample['drive_mode'] & 0x0c
                    or sample['error'] != 0
                    or type(sample['position']) is not int
                    or not 0 <= sample['position'] <= 4095):
                raise LockError('ID1 requires healthy XL430 mode 3, velocity profile, no auto-torque')
            self.sample = sample
            self.observed_at = self.clock()
            if self.target is not None:
                window = self.state_tolerance if self.two_point else self.tolerance
                if (abs(sample['position'] - self.target) <= window
                        and sample['velocity'] == 0 and sample['torque'] == 1):
                    self.target = self.deadline = None
                elif self.clock() >= self.deadline:
                    raise LockError('lock move timed out')
                else:
                    self.state = 'MOVING'
                    return self.state
            if self.two_point:
                position = sample['position']
                if (self.locked_tick is not None
                        and abs(position - self.locked_tick) <= self.state_tolerance):
                    self.state = 'LOCKED'
                elif (self.unlocked_tick is not None
                        and abs(position - self.unlocked_tick) <= self.state_tolerance):
                    self.state = 'UNLOCKED'
                elif (self.assumed is not None
                        and abs(position - self.assumed[1]) <= self.state_tolerance):
                    self.state = self.assumed[0]
                else:
                    self.state = 'BETWEEN'
            elif self.zero_tick is None:
                self.state = 'UNCALIBRATED'
            elif sample['velocity'] != 0:
                self.state = 'UNKNOWN'
            else:
                index = round((sample['position'] - self.zero_tick) / self.step_ticks)
                residual = abs(sample['position'] - self.zero_tick - index * self.step_ticks)
                if residual > self.tolerance:
                    self.state = 'UNKNOWN'
                elif index % 2 == 0:
                    self.state = self.zero_state
                else:
                    self.state = 'UNLOCKED' if self.zero_state == 'LOCKED' else 'LOCKED'
            self.error = ''
        except Exception as exc:
            self.state = 'FAULT'
            self.error = str(exc)
            self.sample = None
            self.observed_at = None
            if self.target is not None:
                try:
                    self.io.write('torque', 0)
                except Exception as stop_error:
                    self.error += f'; stop failed: {stop_error}'
            self.target = self.deadline = None
        return self.state

    @property
    def ready(self):
        return bool(self.state == 'LOCKED' and self.sample
                    and self.sample['torque'] == 1 and self.observed_at is not None
                    and self.clock() - self.observed_at <= 0.5)

    def capture_zero(self, state):
        if self.two_point:
            self.observe()
            if self.sample is None or self.target is not None or self.sample['velocity'] != 0:
                raise LockError('fresh stationary ID1 feedback required')
            self._validate_zero(self.sample['position'], state)
            if state == 'LOCKED':
                self.locked_tick = self.sample['position']
            else:
                self.unlocked_tick = self.sample['position']
            # Two points closer than two windows cannot both be real: keep the new one.
            if (self.locked_tick is not None and self.unlocked_tick is not None
                    and abs(self.locked_tick - self.unlocked_tick) <= 2 * self.state_tolerance):
                if state == 'LOCKED':
                    self.unlocked_tick = None
                else:
                    self.locked_tick = None
            self.zero_tick = self.locked_tick if self.locked_tick is not None else self.unlocked_tick
            self.zero_state = 'LOCKED' if self.locked_tick is not None else 'UNLOCKED'
            self.observe()
            return
        self.observe()
        if self.sample is None or self.target is not None:
            raise LockError('fresh stationary ID1 feedback required')
        if self.sample['torque'] != 0 or self.sample['velocity'] != 0:
            raise LockError('zero capture requires torque OFF and velocity zero')
        self._validate_zero(self.sample['position'], state)
        self.zero_tick = self.sample['position']
        self.zero_state = state
        self.observe()

    def enable(self):
        self.observe()
        if (self.zero_tick is None and not self.two_point) or self.sample is None or self.target is not None:
            raise LockError('valid zero and healthy ID1 required')
        if self.sample['velocity'] != 0:
            raise LockError('ID1 must be stationary')
        if self.sample['torque'] == 1:
            self._check_profile()
            return
        current = self.sample['position']
        try:
            self.io.write('acceleration', 1)
            self.io.write('velocity_profile', 5)
            self.io.write('goal', current)
            self.io.write('torque', 1)
            self.observe()
            if (self.sample is None or self.sample['torque'] != 1
                    or abs(self.sample['position'] - current) > self.tolerance):
                raise LockError('ID1 torque enable/readback changed reference')
            self._check_profile()
            if self.two_point:
                return
            # Correct only encoder-sized settling error, never advance an index on enable.
            indexed = self.zero_tick + round((current - self.zero_tick) / self.step_ticks) * self.step_ticks
            if indexed != current and abs(indexed - current) <= self.tolerance:
                self.io.write('goal', indexed)
        except Exception:
            self.stop()
            raise

    def _check_profile(self):
        if not (0 < self.sample['profile_velocity'] <= 5
                and 0 < self.sample['profile_acceleration'] <= 1):
            raise LockError('ID1 requires velocity profile <=5 and acceleration <=1')

    def move(self, desired, direction=1):
        if desired not in ('LOCKED', 'UNLOCKED') or type(direction) is not int or direction not in (-1, 1):
            raise LockError('LOCK/UNLOCK requires direction +1 or -1')
        if self.two_point:
            return self._move_two_point(desired, direction)
        self.observe()
        if self.state not in ('LOCKED', 'UNLOCKED') or self.sample['torque'] != 1:
            raise LockError('calibrated stationary index and enabled torque required')
        self._check_profile()
        if self.state == desired:
            return
        target = self.zero_tick + (round((self.sample['position'] - self.zero_tick)
                                        / self.step_ticks) + direction) * self.step_ticks
        if not 0 <= target <= 4095:
            raise LockError('single-turn boundary: use opposite direction; no wrap move')
        self.target = target
        self.deadline = self.clock() + 6.0
        self.state = 'MOVING'
        try:
            self.io.write('goal', target)
        except Exception:
            self.stop()
            raise

    def _move_two_point(self, desired, direction=1):
        target = self.locked_tick if desired == 'LOCKED' else self.unlocked_tick
        self.observe()
        if self.sample is None or self.target is not None:
            raise LockError('healthy idle ID1 required')
        if self.sample['torque'] != 1:
            self.enable()
            self.observe()
        self._check_profile()
        if target is None:
            # Not recorded: one 22.5-degree step from here, flipping direction
            # rather than wrapping past the single-turn range.
            position = self.sample['position']
            target = position + direction * self.step_ticks
            if not 0 <= target <= 4095:
                target = position - direction * self.step_ticks
            self.assumed = (desired, target)
        elif abs(self.sample['position'] - target) <= self.state_tolerance:
            return
        self.target = target
        # Profile velocity 5 is about 78 ticks/s; allow for the slow ramp.
        self.deadline = self.clock() + 4.0 + abs(self.sample['position'] - target) / 50.0
        self.state = 'MOVING'
        try:
            self.io.write('goal', target)
        except Exception:
            self.stop()
            raise

    def save_points(self, filename):
        path = Path(filename)
        tmp = path.with_suffix('.tmp')
        tmp.write_text(json.dumps({'locked_tick': self.locked_tick,
                                   'unlocked_tick': self.unlocked_tick}))
        os.replace(tmp, path)

    def load_points(self, filename):
        data = json.loads(Path(filename).read_text())
        for key in ('locked_tick', 'unlocked_tick'):
            value = data.get(key)
            if value is not None and (type(value) is not int or not 0 <= value <= 4095):
                raise LockError(f'invalid {key} in points file')
            setattr(self, key, value)
        self.zero_tick = self.locked_tick if self.locked_tick is not None else self.unlocked_tick
        self.zero_state = 'LOCKED' if self.locked_tick is not None else 'UNLOCKED'

    def stop(self):
        self.target = self.deadline = None
        self.state = 'UNKNOWN'
        self.sample = None
        self.observed_at = None
        self.io.write('torque', 0)

    def save_zero(self, filename):
        self._validate_zero(self.zero_tick, self.zero_state)
        payload = {'actuator_id': 1, 'model_number': 1060, 'gear_ratio': 1,
                   'step_ticks': 256, 'zero_tick': self.zero_tick,
                   'zero_state': self.zero_state}
        path = Path(filename)
        temporary = None
        try:
            with tempfile.NamedTemporaryFile(mode='w', dir=path.parent,
                                             delete=False) as stream:
                temporary = stream.name
                json.dump(payload, stream)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if temporary and os.path.exists(temporary):
                os.unlink(temporary)

    @staticmethod
    def load_zero(filename):
        data = json.loads(Path(filename).read_text())
        if not isinstance(data, dict) or any(data.get(k) != v for k, v in
                {'actuator_id': 1, 'model_number': 1060,
                 'gear_ratio': 1, 'step_ticks': 256}.items()):
            raise LockError('zero file does not match XL430 ID1 / 1:1 / 256 ticks')
        ToolchangerLock._validate_zero(data.get('zero_tick'), data.get('zero_state'))
        return data['zero_tick'], data['zero_state']

    def snapshot(self):
        sample = self.sample or {}
        return {'enabled': True, 'actuator_id': 1, 'model': 'XL430', 'gear_ratio': 1.0,
                'step_ticks': 256, 'step_degrees': 22.5, 'state': self.state,
                'zero_tick': self.zero_tick, 'zero_state': self.zero_state,
                'position': sample.get('position'), 'torque': sample.get('torque'),
                'target': self.target, 'ready': self.ready, 'error': self.error,
                'mechanically_confirmed': False,
                **({'two_point': True, 'locked_tick': self.locked_tick,
                    'unlocked_tick': self.unlocked_tick,
                    'state_tolerance': self.state_tolerance} if self.two_point else {}),
                'angle_rad': (None if self.zero_tick is None or not sample else
                              (sample['position'] - self.zero_tick) * 2 * math.pi / 4096)}
