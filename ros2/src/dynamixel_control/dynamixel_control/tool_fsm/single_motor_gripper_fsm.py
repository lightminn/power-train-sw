"""ID5-only FSM for a calibrated single-motor spur gripper."""

from dynamixel_control.tool_fsm.base import ToolCommandError, ToolFSM, ToolState
from dynamixel_control.tool_fsm.validation import validate_single_motor_startup


class SingleMotorGripperFSM(ToolFSM):
    TOOL_TYPE = 'spur_1motor_gripper'

    def startup(self):
        self.state = ToolState.INIT
        try:
            self.snapshot = validate_single_motor_startup(self.profile, self.bridge)
        except Exception as exc:  # bridge failures must never fall through to motion
            return self._fault(exc)
        if not self.profile.get('calibrated', False):
            self.state = ToolState.CALIBRATION_REQUIRED
            return self.state
        try:
            self._validated_targets()
        except ToolCommandError as exc:
            return self._fault(exc)
        self.state = ToolState.READY
        return self.state

    def command(self, command):
        command = str(command).upper()
        if command == 'STOP':
            return self.stop()
        if command == 'DISABLE':
            return self.disable()
        if command not in ('OPEN', 'CLOSE'):
            raise ToolCommandError(f'unsupported single-motor command {command}')
        if self.state == ToolState.CALIBRATION_REQUIRED:
            raise ToolCommandError('calibration required; OPEN/CLOSE blocked')
        if self.state not in (ToolState.READY, ToolState.OPEN, ToolState.CLOSED):
            raise ToolCommandError(f'{command} unavailable in {self.state.name}')
        targets = self._validated_targets()
        target = targets[command.lower()]
        if self.bridge.read_torque(5) != 1:
            raise ToolCommandError('actual ID5 Torque Enable is OFF')
        self.state = ToolState.OPENING if command == 'OPEN' else ToolState.CLOSING
        try:
            self.bridge.goal_position(5, target)
        except Exception as exc:
            return self._fault(exc)
        self.state = ToolState.OPEN if command == 'OPEN' else ToolState.CLOSED
        return self.state

    def stop(self):
        try:
            self.bridge.set_torque(5, False)
        except Exception as exc:
            return self._fault(exc)
        self.state = ToolState.STOPPED
        return self.state

    def disable(self):
        return self.stop()

    def _validated_targets(self):
        if self.actuator_ids != (5,):
            raise ToolCommandError('single-motor FSM allowlist is exactly ID5')
        values = {key: self.profile.get(key) for key in
                  ('open_tick', 'close_tick', 'safe_min_tick', 'safe_max_tick')}
        if any(value is None for value in values.values()):
            raise ToolCommandError('calibrated endpoint/range values are required')
        low, high = int(values['safe_min_tick']), int(values['safe_max_tick'])
        if low >= high:
            raise ToolCommandError('invalid safe range')
        targets = {'open': int(values['open_tick']), 'close': int(values['close_tick'])}
        if targets['open'] == targets['close']:
            raise ToolCommandError('open and close endpoints must differ')
        if any(not low <= target <= high for target in targets.values()):
            raise ToolCommandError('endpoint outside safe range')
        return targets
