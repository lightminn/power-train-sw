import time
from dataclasses import dataclass
from typing import Optional, Tuple

RUN = "RUN"
MOTION_HOLD = "MOTION_HOLD"
ESTOP = "ESTOP"


@dataclass(frozen=True)
class SafetySnapshot:
    state: str
    estop_latched: bool
    first_source: Optional[str]
    first_detail: str
    tripped_at_s: Optional[float]
    active_estop_sources: Tuple[str, ...]
    hold_sources: Tuple[str, ...]


class SafetyInterlock:
    def __init__(self, clock=None):
        self._clock = clock or time.monotonic
        self._holds = {}
        self._active_estops = {}
        self._latched = False
        self._first_source = None
        self._first_detail = ""
        self._tripped_at_s = None

    def set_motion_hold(self, source, active, detail=""):
        if active:
            self._holds[source] = detail
        else:
            self._holds.pop(source, None)

    def set_estop_condition(self, source, active, detail=""):
        if active:
            self._active_estops[source] = detail
            self.trip_estop(source, detail)
        else:
            self._active_estops.pop(source, None)

    def trip_estop(self, source, detail=""):
        if not self._latched:
            self._latched = True
            self._first_source = source
            self._first_detail = detail
            self._tripped_at_s = self._clock()

    def reset_estop(self):
        if self._active_estops:
            return False
        self._latched = False
        self._first_source = None
        self._first_detail = ""
        self._tripped_at_s = None
        return True

    def snapshot(self):
        state = ESTOP if self._latched else MOTION_HOLD if self._holds else RUN
        return SafetySnapshot(
            state=state,
            estop_latched=self._latched,
            first_source=self._first_source,
            first_detail=self._first_detail,
            tripped_at_s=self._tripped_at_s,
            active_estop_sources=tuple(sorted(self._active_estops)),
            hold_sources=tuple(sorted(self._holds)),
        )
