"""Fail-closed FSM policy for the calibrated ID3/ID4 gripper."""

from dynamixel_control.tool_fsm.base import ToolCommandError, ToolFSM, ToolState
from dynamixel_control.tool_fsm.validation import validate_dual_motor_startup
from dynamixel_control.dual_gripper_targets import dual_relative_targets


class DualMotorGripperFSM(ToolFSM):
    TOOL_TYPE = 'dual_motor_gripper'

    def startup(self):
        self.state = ToolState.INIT
        try:
            self.snapshot = validate_dual_motor_startup(self.profile, self.bridge)
            self._validated_targets()
        except Exception as exc:
            return self._fault(exc)
        self.state = ToolState.READY
        return self.state

    def command(self, command):
        command = str(command).strip().upper()
        if command in ('STOP', 'DISABLE'):
            return self.stop()
        if command == 'HOLD':
            return self.hold()
        jog = command in ('JOG_OPEN', 'JOG_CLOSE')
        endpoint_command = command.removeprefix('JOG_') if jog else command
        if endpoint_command not in ('OPEN', 'CLOSE'):
            raise ToolCommandError(f'unsupported dual-motor command {command}')
        allowed = (ToolState.READY, ToolState.OPEN, ToolState.CLOSED)
        if jog:
            allowed += (ToolState.OPENING, ToolState.CLOSING)
        if self.state not in allowed:
            raise ToolCommandError(f'{command} unavailable in {self.state.name}')
        endpoints = self._validated_targets()
        targets = endpoints[endpoint_command.lower()]
        positions = {}
        for dxl_id in self.actuator_ids:
            if self.bridge.read_hardware_error(dxl_id) != 0:
                raise ToolCommandError(f'ID{dxl_id} hardware error')
            if self.bridge.read_torque(dxl_id) != 1:
                raise ToolCommandError(f'actual ID{dxl_id} Torque Enable is OFF')
            positions[dxl_id] = self.bridge.read_position(dxl_id)
            if positions[dxl_id] is None:
                raise ToolCommandError(f'ID{dxl_id} position unavailable')
        if jog:
            # Equal normalized increments preserve each motor's endpoint ratio.
            # Read actual positions every tick, so stalled motion cannot accrue
            # a queue of distant targets. Never alter calibration or profiles.
            try:
                targets = dual_relative_targets(
                    self.profile, self.actuator_ids, positions,
                    endpoint_command, step_ticks=100.0)
            except ValueError as exc:
                raise ToolCommandError(str(exc)) from exc
        opening = endpoint_command == 'OPEN'
        self.state = ToolState.OPENING if opening else ToolState.CLOSING
        try:
            if jog:
                self.bridge.start_dual_jog(targets)
            else:
                self.bridge.command_dual_targets(targets)
            # A completed endpoint command is a parked, torque-holding state.
            # Only explicit STOP/DISABLE or a safety/fault path may turn torque
            # off; do not route successful OPEN/CLOSE through stop().
            if not jog:
                for dxl_id in self.actuator_ids:
                    if self.bridge.read_torque(dxl_id) != 1:
                        raise ToolCommandError(
                            f'actual ID{dxl_id} Torque Enable dropped after command')
        except Exception as exc:
            return self._motion_fault(exc)
        if not jog:
            self.state = ToolState.OPEN if opening else ToolState.CLOSED
        return self.state

    def hold(self):
        if self.state not in (ToolState.OPENING, ToolState.CLOSING):
            raise ToolCommandError(f'HOLD unavailable in {self.state.name}')
        try:
            self.bridge.hold_dual_position()
        except Exception as exc:
            return self._motion_fault(exc)
        self.state = ToolState.READY
        return self.state

    def _motion_fault(self, reason):
        """Disable the pair and leave a motion fault in the stopped state."""
        errors = []
        for dxl_id in self.actuator_ids:
            try:
                self.bridge.set_torque(dxl_id, False)
            except Exception as exc:
                errors.append(f'ID{dxl_id}: {exc}')
        self.fault_reason = '; '.join(errors) if errors else str(reason)
        self.state = ToolState.STOPPED
        return self.state

    def stop(self):
        errors = []
        for dxl_id in self.actuator_ids:
            try:
                self.bridge.set_torque(dxl_id, False)
            except Exception as exc:
                errors.append(f'ID{dxl_id}: {exc}')
        if errors:
            return self._fault('; '.join(errors))
        self.state = ToolState.STOPPED
        return self.state

    def _validated_targets(self):
        if self.actuator_ids != (3, 4):
            raise ToolCommandError('dual-motor FSM allowlist is exactly ID3/ID4')
        if not self.profile.get('calibrated', False):
            raise ToolCommandError('dual motor profile is not calibrated')
        if not self.profile.get('endpoint_calibration_verified', False):
            raise ToolCommandError('dual endpoint calibration is not verified')
        low = self.profile.get('safe_min_tick')
        high = self.profile.get('safe_max_tick')
        endpoints = self.profile.get('motor_endpoints') or {}
        if low is None or high is None or int(low) >= int(high):
            raise ToolCommandError('invalid dual safe range')
        targets = {'open': {}, 'close': {}}
        for dxl_id in self.actuator_ids:
            endpoint = endpoints.get(dxl_id, endpoints.get(str(dxl_id)))
            if not endpoint or endpoint.get('open') == endpoint.get('close'):
                raise ToolCommandError(f'ID{dxl_id} endpoints are invalid')
            for name in targets:
                tick = int(endpoint[name])
                if not int(low) <= tick <= int(high):
                    raise ToolCommandError(f'ID{dxl_id} {name} outside safe range')
                targets[name][dxl_id] = tick
        return targets
