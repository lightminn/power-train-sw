"""Display guard for the arm-UI widget tests.

Building GTK widgets needs a display connection even when nothing is shown, so
without one these modules cannot be collected at all.  A missing display is an
environment fact rather than a defect, so the whole directory is skipped
instead of failing — the same way the console's other GTK suites need a real
``gi`` interpreter.

    /usr/bin/python3 -m pytest operator_console/tests/arm_ui -q
    # 표시장치가 없을 때:
    broadwayd :7 & GDK_BACKEND=broadway BROADWAY_DISPLAY=:7 \\
        /usr/bin/python3 -m pytest operator_console/tests/arm_ui -q
"""
from __future__ import annotations


def _display_available() -> bool:
    try:
        import gi

        gi.require_version("Gtk", "3.0")
        gi.require_version("Gdk", "3.0")
        from gi.repository import Gdk, Gtk
    except (ImportError, ValueError):
        return False
    if not Gtk.init_check(None)[0]:
        return False
    return Gdk.Screen.get_default() is not None


collect_ignore_glob: list[str] = [] if _display_available() else ["test_*.py"]
