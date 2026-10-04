"""Narrow hardware contract shared by tool FSMs."""

from enum import Enum, auto


class ToolState(Enum):
    DETACHED = auto()
    INIT = auto()
    CALIBRATION_REQUIRED = auto()
    READY = auto()
    OPENING = auto()
    OPEN = auto()
    CLOSING = auto()
    CLOSED = auto()
    STOPPED = auto()
    FAULT = auto()


class ToolCommandError(RuntimeError):
    """A command was rejected before a hardware write was attempted."""


class ToolFSM:
    """Base class.  Implementations receive a bridge, never an SDK object."""

    def __init__(self, profile, bridge):
        self.profile = dict(profile)
        self.bridge = bridge
        self.state = ToolState.DETACHED
        self.fault_reason = ''

    @property
    def actuator_ids(self):
        return tuple(self.profile.get('actuator_ids') or ())

    def _fault(self, reason):
        self.state = ToolState.FAULT
        self.fault_reason = str(reason)
        return self.state
