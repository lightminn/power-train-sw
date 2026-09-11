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

    def observe_command(self, enabled):
        self.state = CleanerState.CLEANING if enabled else CleanerState.READY
