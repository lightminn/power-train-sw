"""Explicit, ID5-only calibration session for the spur gripper.

This module intentionally knows no ROS actions and no Dynamixel SDK details.
The bridge supplies the narrow adapter; tests supply an in-memory adapter.
"""

from copy import deepcopy
import os
from pathlib import Path
import tempfile

import yaml

from dynamixel_control.tool_profiles import validate_profile


class CalibrationSessionError(RuntimeError):
    """A calibration operation was rejected before an unsafe write."""


class CalibrationSession:
    """Operator-driven endpoint capture with one-click, ID5-only jogs."""

    # Endpoint discovery often needs a coarse approach before the final
    # half-degree witness capture.  Keep this a finite one-click set rather
    # than accepting arbitrary GUI values.
    ALLOWED_DEGREES = frozenset((-5.0, -1.0, -0.5, 0.5, 1.0, 5.0))
    ID = 5
    # Captures are deliberately made at the operator's chosen command limits.
    # Until a separately measured mechanical margin exists, no hidden numeric
    # margin is fabricated; commands are limited to the witnessed endpoints.
    CAPTURED_ENDPOINT_MARGIN_TICKS = 0

    def __init__(self, bridge, profile):
        self.bridge = bridge
        self.profile = deepcopy(profile)
        self.active = False
        self.enabled = False
        self.validated = False
        self.captures = {}
        self.last_error = ''
        self.bridge.set_allowlist([self.ID])

    def start(self):
        self.active = True
        self.last_error = ''
        return self.snapshot()

    def stop(self):
        self.active = False
        self.enabled = False
        return self.snapshot()

    def enable(self):
        self._require_active()
        self._require_healthy_feedback()
        self.bridge.set_torque(self.ID, True)
        self.enabled = True
        return self.snapshot()

    def disable(self):
        self.bridge.set_torque(self.ID, False)
        self.enabled = False
        return self.snapshot()

    def jog_motor_degrees(self, delta_deg):
        self._require_active()
        if float(delta_deg) not in self.ALLOWED_DEGREES:
            raise CalibrationSessionError(
                'only one-click ±0.5°, ±1.0°, or ±5.0° jog is allowed')
        if not self.enabled:
            raise CalibrationSessionError('ID5 must be explicitly enabled before jog')
        self._require_healthy_feedback()
        if self.bridge.read_torque(self.ID) != 1:
            self.enabled = False
            raise CalibrationSessionError('actual ID5 Torque Enable is OFF')
        current = self._read_position()
        # 4096 ticks/rev is the motor control-table conversion, not an endpoint.
        delta_tick = int(round(float(delta_deg) * 4096.0 / 360.0))
        if delta_tick == 0:
            raise CalibrationSessionError('jog conversion produced zero ticks')
        self.bridge.goal_position(self.ID, current + delta_tick)
        return current + delta_tick

    def capture_open(self):
        return self._capture('open')

    def capture_close(self):
        return self._capture('close')

    def get_candidate(self):
        candidate = deepcopy(self.profile)
        candidate['actuator_ids'] = [self.ID]
        candidate['open_tick'] = self.captures.get('open')
        candidate['close_tick'] = self.captures.get('close')
        if candidate['open_tick'] is not None and candidate['close_tick'] is not None:
            low = min(candidate['open_tick'], candidate['close_tick'])
            high = max(candidate['open_tick'], candidate['close_tick'])
            candidate['safe_min_tick'] = low + self.CAPTURED_ENDPOINT_MARGIN_TICKS
            candidate['safe_max_tick'] = high - self.CAPTURED_ENDPOINT_MARGIN_TICKS
            # Direction is derived from the witnessed pair; no OPEN<CLOSE
            # convention is assumed for the external spur gear.
            candidate['direction'] = (
                1 if candidate['close_tick'] > candidate['open_tick'] else -1)
            candidate['motor_model'] = self.bridge.read_model(self.ID)
            candidate['calibrated'] = True
        return candidate

    def validate_candidate(self):
        candidate = self.get_candidate()
        if candidate.get('open_tick') is None or candidate.get('close_tick') is None:
            return ['both OPEN and CLOSE must be captured']
        if not all(isinstance(candidate[key], int)
                   for key in ('open_tick', 'close_tick')):
            return ['captured endpoints must be integer ticks']
        if candidate.get('open_tick') == candidate.get('close_tick'):
            return ['open and close captures must differ']
        try:
            self._require_healthy_feedback()
        except CalibrationSessionError as exc:
            return [str(exc)]
        expected = candidate.get('motor_model')
        if expected is None:
            return ['ID5 model identity unavailable']
        return validate_profile('spur_1motor_gripper', candidate)

    def validate(self):
        errors = self.validate_candidate()
        self.validated = not errors
        if errors:
            raise CalibrationSessionError('; '.join(errors))
        return self.get_candidate()

    def save(self, output_path):
        """Explicit file operation; never called by capture or jog.

        The caller chooses a temporary/mock path in tests.  The live profile is
        not a default and is never inferred here.
        """
        errors = self.validate_candidate()
        if errors or not self.validated:
            raise CalibrationSessionError(
                '; '.join(errors) if errors else 'validate before save')
        path = Path(output_path)
        with path.open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream) or {}
        profiles = document.get('tool_profiles')
        if not isinstance(profiles, dict):
            raise CalibrationSessionError('profile YAML has no tool_profiles mapping')
        profiles['spur_1motor_gripper'] = self.get_candidate()
        fd, temporary = tempfile.mkstemp(
            prefix=f'.{path.name}.', suffix='.tmp', dir=str(path.parent))
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(document, stream, sort_keys=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)
        return path

    def snapshot(self):
        return {'active': self.active, 'enabled': self.enabled,
                'captures': dict(self.captures),
                'candidate_valid': not self.validate_candidate(),
                'validated': self.validated,
                'safety_margin_ticks': self.CAPTURED_ENDPOINT_MARGIN_TICKS}

    def _capture(self, label):
        self._require_active()
        self._require_healthy_feedback()
        self.captures[label] = self._read_position()
        self.validated = False
        return self.captures[label]

    def _read_position(self):
        position = self.bridge.read_position(self.ID)
        if position is None:
            raise CalibrationSessionError('ID5 present position unavailable')
        return int(position)

    def _require_active(self):
        if not self.active:
            raise CalibrationSessionError('calibration session is not active')

    def _require_healthy_feedback(self):
        try:
            position = self.bridge.read_position(self.ID)
            hardware_error = self.bridge.read_hardware_error(self.ID)
        except Exception as exc:
            raise CalibrationSessionError(f'ID5 offline/read failed: {exc}') from exc
        if position is None:
            raise CalibrationSessionError('ID5 present position unavailable')
        if hardware_error != 0:
            raise CalibrationSessionError(f'ID5 hardware error: {hardware_error}')
