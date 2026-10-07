"""Tool-local state for the existing /cleaning/enable velocity adapter.

The arm mission retains responsibility for approach/contact/duration/retraction.
"""
from enum import Enum


class CleanerState(Enum):
    READY = 'READY'
    CLEANING = 'CLEANING'
    STOPPED = 'STOPPED'
    CALIBRATION_REQUIRED = 'CALIBRATION_REQUIRED'


class CleanerFSM:
    def __init__(self, profile, bridge):
        self.profile = dict(profile)
        self.bridge = bridge
        self.fault_reason = ''
        self.state = CleanerState.STOPPED

    def startup(self):
        self.state = (CleanerState.READY if self.bridge.tool_motion_allowed
                      else CleanerState.CALIBRATION_REQUIRED)
        return self.state

    def command(self, command):
        """Route the existing LEFT/RIGHT/STOP velocity contract via this FSM."""
        command = str(command).strip().upper()
        if command not in ('LEFT', 'RIGHT', 'STOP'):
            raise ValueError(f'unsupported cleaner command {command!r}')
        if command != 'STOP' and self.state not in (
                CleanerState.READY, CleanerState.CLEANING):
            raise RuntimeError(
                f'{command} unavailable in cleaner state {self.state.value}')
        dispatch = getattr(self.bridge, '_cleaner_direction_command', None)
        if dispatch is None or dispatch(command) is False:
            raise RuntimeError(f'cleaner {command} command was rejected')
        self.state = (CleanerState.READY if command == 'STOP'
                      else CleanerState.CLEANING)
        return self.state

    def observe_command(self, enabled):
        self.state = CleanerState.CLEANING if enabled else CleanerState.READY
