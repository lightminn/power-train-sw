"""Operator-witnessed endpoint calibration for the dual motor gripper."""

from copy import deepcopy
import os
from pathlib import Path
import tempfile

import yaml

from dynamixel_control.tool_profiles import validate_profile


class DualCalibrationError(RuntimeError):
    """The dual endpoint calibration operation was safely rejected."""


class DualCalibrationSession:
    """Capture independent ID3/ID4 endpoint pairs; never infer endpoints."""

    IDS = (3, 4)
    ALLOWED_DEGREES = frozenset((-5.0, -2.0, -1.0, -0.5,
                                 0.5, 1.0, 2.0, 5.0))

    def __init__(self, bridge, profile):
        self.bridge = bridge
        self.profile = deepcopy(profile)
        self.active = False
        self.validated = False
        self.captures = {}
        self.models = {}

    @property
    def state(self):
        if self.is_ready:
            return 'READY'
        if self.validated:
            return 'VALIDATED'
        if 'open' in self.captures and 'close' not in self.captures:
            return 'OPEN_CAPTURED_WAITING_FOR_CLOSE'
        if 'close' in self.captures and 'open' not in self.captures:
            return 'CLOSE_CAPTURED_WAITING_FOR_OPEN'
        if self.active:
            return 'CAPTURING'
        return 'RECALIBRATION_REQUIRED'

    @property
    def is_ready(self):
        return bool(self.profile.get('endpoint_calibration_verified'))

    def start(self):
        self.active = True
        self.validated = False
        self.captures = {}
        self.models = {}
        return self.snapshot()

    def stop(self):
        self.active = False
        return self.snapshot()

    def jog_motor_degrees(self, dxl_id, delta_deg):
        self._require_active()
        if float(delta_deg) not in self.ALLOWED_DEGREES:
            raise DualCalibrationError('allowed dual calibration steps are ±0.5, ±1, ±2, ±5°')
        self._require_healthy(torque_on=True)
        return self.bridge.dual_calibration_jog(int(dxl_id), float(delta_deg))

    def jog_pair_degrees(self, delta_deg):
        """Move ID3/ID4 by one equal calibration step via a paired write."""
        self._require_active()
        if float(delta_deg) not in self.ALLOWED_DEGREES:
            raise DualCalibrationError(
                'allowed dual calibration steps are ±0.5, ±1, ±2, ±5°')
        self._require_healthy(torque_on=True)
        return self.bridge.dual_calibration_pair_jog(float(delta_deg))

    def hold(self):
        """Hold both fresh positions when a hold-to-run key is released."""
        self._require_active()
        self._require_healthy(torque_on=True)
        return self.bridge.dual_calibration_hold()

    def capture_open(self):
        return self._capture('open')

    def capture_close(self):
        return self._capture('close')

    def get_candidate(self):
        candidate = deepcopy(self.profile)
        if set(self.captures) != {'open', 'close'}:
            return candidate
        open_ticks = dict(self.captures['open'])
        close_ticks = dict(self.captures['close'])
        candidate['open_ticks'] = open_ticks
        candidate['close_ticks'] = close_ticks
        candidate['motor_endpoints'] = {
            dxl_id: {'open': open_ticks[dxl_id], 'close': close_ticks[dxl_id]}
            for dxl_id in self.IDS}
        all_ticks = list(open_ticks.values()) + list(close_ticks.values())
        candidate['safe_min_tick'] = min(all_ticks)
        candidate['safe_max_tick'] = max(all_ticks)
        candidate['endpoint_calibration_verified'] = True
        candidate['endpoint_calibration_models'] = dict(self.models)
        return candidate

    def validate_candidate(self):
        if set(self.captures) != {'open', 'close'}:
            return ['both OPEN and CLOSE pairs must be captured']
        for label in ('open', 'close'):
            pair = self.captures[label]
            if set(pair) != set(self.IDS):
                return [f'{label} capture must contain IDs [3, 4]']
            if not all(isinstance(pair[dxl_id], int) for dxl_id in self.IDS):
                return [f'{label} capture must contain integer ticks']
        for dxl_id in self.IDS:
            if self.captures['open'][dxl_id] == self.captures['close'][dxl_id]:
                return [f'ID{dxl_id} OPEN and CLOSE must differ']
        try:
            self._require_healthy(torque_on=True)
        except DualCalibrationError as exc:
            return [str(exc)]
        candidate = self.get_candidate()
        errors = validate_profile('dual_motor_gripper', candidate)
        if errors:
            return errors
        for label, expected in (('open', 1.0), ('close', 0.0)):
            progress = self.normalized_progress(self.captures[label])
            if any(abs(progress[dxl_id] - expected) > 1e-9 for dxl_id in self.IDS):
                return [f'{label} normalized endpoint validation failed']
            if abs(progress[3] - progress[4]) > 1e-9:
                return [f'{label} normalized spread validation failed']
        return []

    def validate(self):
        errors = self.validate_candidate()
        self.validated = not errors
        if errors:
            raise DualCalibrationError('; '.join(errors))
        return self.get_candidate()

    def save(self, output_path):
        if not self.validated:
            raise DualCalibrationError('validate before save')
        errors = self.validate_candidate()
        if errors:
            raise DualCalibrationError('; '.join(errors))
        path = Path(output_path)
        with path.open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream) or {}
        profiles = document.get('tool_profiles')
        if not isinstance(profiles, dict):
            raise DualCalibrationError('profile YAML has no tool_profiles mapping')
        profiles['dual_motor_gripper'] = self.get_candidate()
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
        self.profile = self.get_candidate()
        self.active = False
        return path

    def reload(self, profile):
        self.profile = deepcopy(profile)
        self.active = False
        self.validated = False
        self.captures = {}
        self.models = {}

    def normalized_progress(self, positions):
        candidate = self.get_candidate()
        endpoints = candidate.get('motor_endpoints') or {}
        progress = {}
        for dxl_id in self.IDS:
            endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
            if not endpoint:
                raise DualCalibrationError(f'missing endpoint for ID{dxl_id}')
            span = endpoint['open'] - endpoint['close']
            if span == 0:
                raise DualCalibrationError(f'zero endpoint span for ID{dxl_id}')
            progress[dxl_id] = (positions[dxl_id] - endpoint['close']) / span
        return progress

    def snapshot(self):
        return {
            'state': self.state,
            'active': self.active,
            'validated': self.validated,
            'captures': deepcopy(self.captures),
            'candidate_valid': not self.validate_candidate(),
            'allowed_steps_deg': sorted(abs(value) for value in self.ALLOWED_DEGREES
                                        if value > 0),
        }

    def _capture(self, label):
        self._require_active()
        # A witnessed endpoint is a fresh read, never a movement.  Requiring
        # torque here would make a safely parked, torque-OFF mechanism move
        # merely to preserve the operator's current position.
        state = self._require_healthy(torque_on=False)
        self.captures[label] = {dxl_id: int(state[dxl_id]['position'])
                                for dxl_id in self.IDS}
        self.models = {dxl_id: state[dxl_id]['model'] for dxl_id in self.IDS}
        self.validated = False
        return dict(self.captures[label])

    def _require_active(self):
        if not self.active:
            raise DualCalibrationError('dual calibration session is not active')

    def _require_healthy(self, torque_on):
        try:
            state = self.bridge.read_dual_calibration_state()
        except Exception as exc:
            raise DualCalibrationError(f'dual calibration read failed: {exc}') from exc
        if set(state) != set(self.IDS):
            raise DualCalibrationError('dual calibration actuator allowlist is not [3, 4]')
        for dxl_id in self.IDS:
            sample = state[dxl_id]
            if sample.get('position') is None:
                raise DualCalibrationError(f'ID{dxl_id} present position unavailable')
            if sample.get('hardware_error') != 0:
                raise DualCalibrationError(f'ID{dxl_id} hardware error: {sample.get("hardware_error")}')
            if sample.get('model') is None:
                raise DualCalibrationError(f'ID{dxl_id} model unavailable')
            if torque_on and sample.get('torque') != 1:
                raise DualCalibrationError(f'ID{dxl_id} actual Torque Enable is OFF')
        return state
