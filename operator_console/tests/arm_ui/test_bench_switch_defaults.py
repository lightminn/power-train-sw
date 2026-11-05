"""Bench joint limits and current trip both start OFF, on screen and on the wire."""
from __future__ import annotations

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.arm_ops import contract as K
from operator_console.arm_ui.keyboard_panel import KeyboardTeleopPanel

from _arm_ui_helpers import recording_callbacks, walk


def test_every_command_carries_limits_off_explicitly():
    # The bridge treats a missing key as ON, so OFF has to be sent.
    assert K.BENCH_JOG_FLAGS == {"limits": False, "trip": False}


def test_both_switches_start_off(monkeypatch):
    monkeypatch.setenv("ARM_LOCAL_ALL_AXES", "1")
    callbacks, _calls = recording_callbacks()
    panel = KeyboardTeleopPanel(callbacks)
    try:
        switches = [w for w in walk(panel.box) if isinstance(w, Gtk.Switch)]
        switches += [w for w in walk(panel.teleop_card) if isinstance(w, Gtk.Switch)]
        assert len(switches) == 2
        assert not any(switch.get_active() for switch in switches)
    finally:
        panel.dispose()
