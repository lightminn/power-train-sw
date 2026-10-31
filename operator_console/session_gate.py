"""Which sender session a console receiver follows (PR #15 spec §4.6 세션 게이트).

Shared by the terrain overlay (:5009) and terrain map (:5010) receivers.  Only
the session switch lives here; each receiver keeps its own ordering floors.
Pure Python.
"""
from __future__ import annotations

BOOT_SWITCH_SILENCE_S = 1.0

NEW, SAME, REJECT = "new", "same", "reject"


class SessionSelector:
    """Order senders by session ``(boot_id, start_mono_ns, instance)``, never by address.

    Within one boot a later ``(start, instance)`` replaces an earlier one at once and an
    earlier one is refused forever; a different boot is taken only after the current
    session has been silent for ``BOOT_SWITCH_SILENCE_S``.
    """

    def __init__(self) -> None:
        self.session: tuple[str, int, str] | None = None
        self._last_accepted_s: float | None = None

    def classify(self, key: tuple[str, int, str], now_s: float) -> str:
        current = self.session
        if current is None:
            return NEW
        if key == current:
            return SAME
        if key[0] == current[0]:
            return REJECT if key[1:] <= current[1:] else NEW
        return REJECT if now_s - self._last_accepted_s <= BOOT_SWITCH_SILENCE_S else NEW

    def commit(self, key: tuple[str, int, str], now_s: float) -> None:
        """Record an accepted packet of ``key`` (call only after the receiver accepted it)."""
        self.session = key
        self._last_accepted_s = now_s
