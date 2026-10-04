"""Display guard — only the GTK binding test needs one.

Everything else in this directory is pure and must keep running with no
display at all; that purity is itself asserted by
``test_the_pure_core_imports_without_gtk``.
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


collect_ignore_glob: list[str] = (
    [] if _display_available() else ["test_arm_ops_gtk_adapter.py"]
)
