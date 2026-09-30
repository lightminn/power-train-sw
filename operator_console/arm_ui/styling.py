"""Arm-UI-only CSS, scoped so it cannot reach the existing console screens.

Every selector below is a descendant of ``.arm-ui`` (or of the preview
window's own ``.arm-ui-preview``), and no existing widget carries either
class.  That is the whole containment strategy: the shared theme in
``app.py`` is not edited, not re-declared, and not overridden -- these rules
only ever match inside a widget this package built.

The colours are lifted from the console's competition-dark surface so the arm
tabs sit beside 실시간 화면 / 시스템 상태 without looking bolted on:
page ``#07101B``, card ``#0C1928`` on ``#263B52``, label ``#71879C``,
value ``#E8F0F8``, action button ``#173353`` on ``#315F91``.  The
``status-live`` / ``status-warn`` / ``status-bad`` / ``status-muted`` tone
classes are the console's existing vocabulary and are reused verbatim.
"""
from __future__ import annotations

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gtk  # noqa: E402


ARM_UI_STYLE_CLASS = "arm-ui"
PREVIEW_STYLE_CLASS = "arm-ui-preview"


ARM_UI_CSS = b"""
.arm-ui { background: #07101B; }
.arm-ui label { color: #D7E2EC; }

.arm-ui .arm-page-title { color: #F2F7FC; font-size: 22px; font-weight: 900; }
.arm-ui .arm-page-subtitle { color: #94A9BC; font-size: 11px; }

/* Fixture banner: preview data must never read as a live arm. */
.arm-ui .arm-fixture-banner {
  background: #4A3A12;
  color: #FFD77A;
  border: 1px solid #7A6120;
  border-radius: 7px;
  padding: 7px 11px;
  font-size: 11px;
  font-weight: 900;
}
.arm-ui .arm-fixture-banner label { color: #FFD77A; }

/* Card: same geometry and accent rule as .system-section. */
.arm-ui .arm-card {
  background: #0C1928;
  border: 1px solid #263B52;
  border-top: 3px solid #4B8BEA;
  border-radius: 11px;
  padding: 13px;
}
.arm-ui .arm-card-tool { border-top-color: #D8799B; }
.arm-ui .arm-card-authority { border-top-color: #8C7BEA; }
.arm-ui .arm-card-fsm { border-top-color: #55B9DE; }
.arm-ui .arm-card-teleop { border-top-color: #D9A64B; }
.arm-ui .arm-card-gripper { border-top-color: #55C995; }
.arm-ui .arm-card-diagnostics { border-top-color: #4B8BEA; }
.arm-ui .arm-card-arm-calibration { border-top-color: #4B8BEA; }
.arm-ui .arm-card-tool-calibration { border-top-color: #55C995; }

.arm-ui .arm-card-title { color: #F0F5FA; font-size: 15px; font-weight: 900; }
.arm-ui .arm-card-description { color: #859BAF; font-size: 10px; }

/* Metric tile: mirrors .system-metric. */
.arm-ui .arm-metric {
  background: #111F31;
  border: 1px solid #263A52;
  border-radius: 7px;
  padding: 8px 10px;
}
.arm-ui .arm-metric-label { color: #71879C; font-size: 9px; font-weight: 800; }
.arm-ui .arm-metric-value { color: #E8F0F8; font-size: 13px; font-weight: 900; }
.arm-ui .arm-metric-value.status-live { color: #8BE3B6; }
.arm-ui .arm-metric-value.status-warn { color: #FFD77A; }
.arm-ui .arm-metric-value.status-bad { color: #FF9BA8; }
.arm-ui .arm-metric-value.status-muted { color: #A8B6C4; }

/* Pill badge: same shape and tone pairs as .system-section-badge. */
.arm-ui .arm-badge {
  border-radius: 999px;
  padding: 4px 9px;
  font-size: 10px;
  font-weight: 900;
}
.arm-ui .arm-badge.status-live { color: #8BE3B6; background: #123D2D; }
.arm-ui .arm-badge.status-warn { color: #FFD77A; background: #493514; }
.arm-ui .arm-badge.status-bad { color: #FF9BA8; background: #4A1823; }
.arm-ui .arm-badge.status-muted { color: #A8B6C4; background: #263442; }

.arm-ui .arm-note { color: #9EB1C2; font-size: 10px; }
.arm-ui .arm-note.status-warn { color: #D8B668; }
.arm-ui .arm-note.status-bad { color: #FF9BA8; }
.arm-ui .arm-divider { background: #263B52; min-height: 1px; margin: 2px 0; }

/* Blocking reasons read as a list of facts, not a single vague banner. */
.arm-ui .arm-reason {
  background: #18283B;
  border: 1px solid #344A61;
  border-left: 3px solid #D9A64B;
  border-radius: 6px;
  padding: 5px 9px;
  color: #E1CE9E;
  font-size: 10px;
  font-weight: 800;
}
.arm-ui .arm-reason label { color: #E1CE9E; }

/* Buttons: the console's .integrated-operation / .ops-settings-panel style,
   including its explicit non-faded disabled state. */
.arm-ui button {
  background: #173353;
  color: #DDE8F3;
  border: 1px solid #315F91;
  border-radius: 7px;
  padding: 5px 10px;
  font-size: 11px;
  font-weight: 800;
}
.arm-ui button label { color: #DDE8F3; }
.arm-ui button:hover { background: #214A75; }
.arm-ui button:disabled {
  background: #111A28;
  color: #71869C;
  border-color: #26384E;
  opacity: 1;
}
.arm-ui button:disabled label { color: #71869C; }

.arm-ui button.arm-primary { background: #23538A; border-color: #4C86C6; }
.arm-ui button.arm-primary:hover { background: #2C6AAF; }
.arm-ui button.arm-danger { background: #4A1823; border-color: #8A3542; color: #FF9BA8; }
.arm-ui button.arm-danger label { color: #FF9BA8; }
.arm-ui button.arm-danger:hover { background: #61202E; }
/* The tone classes are declared after the base :disabled rule, so they would
   otherwise win and leave a disabled control looking actionable. */
.arm-ui button.arm-primary:disabled,
.arm-ui button.arm-danger:disabled {
  background: #111A28;
  color: #71869C;
  border-color: #26384E;
  opacity: 1;
}
.arm-ui button.arm-primary:disabled label,
.arm-ui button.arm-danger:disabled label { color: #71869C; }
.arm-ui button.arm-axis { min-width: 34px; padding: 4px 6px; }
.arm-ui button.arm-axis:checked { background: #26316C; border-color: #4C5BE5; color: #FFFFFF; }
.arm-ui button.arm-axis:checked label { color: #FFFFFF; }

.arm-ui combobox button { font-size: 11px; font-weight: 800; }
.arm-ui entry {
  background: #111F31;
  color: #E8F0F8;
  border: 1px solid #315F91;
  border-radius: 6px;
  padding: 3px 7px;
  font-size: 11px;
}
.arm-ui treeview, .arm-ui list, .arm-ui list row {
  background: #111F31;
  color: #D7E2EC;
}
.arm-ui list row:selected { background: #26316C; color: #FFFFFF; }

.arm-ui expander { color: #A8B6C4; font-size: 11px; }
.arm-ui expander > title { color: #A8B6C4; }

/* Monospace where a raw backend token is shown next to its label. */
.arm-ui .arm-mono {
  color: #B4C4D3;
  font-family: "JetBrains Mono", "D2Coding", "DejaVu Sans Mono", monospace;
  font-size: 10px;
}

.arm-ui .arm-shortcut {
  background: #18283B;
  border: 1px solid #344A61;
  border-radius: 5px;
  padding: 2px 7px;
  color: #B7C7D5;
  font-family: "JetBrains Mono", "D2Coding", "DejaVu Sans Mono", monospace;
  font-size: 10px;
  font-weight: 800;
}

.arm-ui flowbox, .arm-ui flowboxchild { background: transparent; }
.arm-ui flowboxchild:selected { background: transparent; }

.arm-ui scrollbar slider { background: #344A61; border-radius: 10px; min-width: 6px; min-height: 6px; }
.arm-ui scrolledwindow { background: #07101B; }

/* Standalone preview only -- the hosting app supplies these otherwise. */
window.arm-ui-preview { background: #07101B; }
window.arm-ui-preview .nav { background: #0B1726; border-bottom: 1px solid #203449; padding: 0 14px; }
window.arm-ui-preview .nav button {
  background: transparent;
  color: #93A8BB;
  border: none;
  border-bottom: 3px solid transparent;
  border-radius: 0;
  min-height: 42px;
  padding: 7px 22px;
  font-weight: 800;
}
window.arm-ui-preview .nav button label { color: #93A8BB; }
window.arm-ui-preview .nav button:checked {
  color: #FFFFFF;
  background: rgba(64,132,255,0.08);
  border-bottom: 3px solid #5A9BFF;
}
window.arm-ui-preview .nav button:checked label { color: #FFFFFF; }
window.arm-ui-preview .arm-preview-bar {
  background: #0B1726;
  border-bottom: 1px solid #22364B;
  padding: 8px 14px;
}
window.arm-ui-preview .arm-preview-title { color: #F7FAFC; font-size: 15px; font-weight: 900; }
window.arm-ui-preview .arm-preview-hint { color: #8EA3B8; font-size: 10px; }
"""


_INSTALLED_SCREENS: set[int] = set()


def install_arm_ui_css(screen: Gdk.Screen | None = None) -> bool:
    """Attach the arm CSS once per screen.  Returns True if it was attached now.

    Idempotent because both tabs call it: registering the same provider twice
    would leave a duplicate in the screen's provider chain for the life of the
    process.  Priority is APPLICATION, the same level the console uses, and the
    ``.arm-ui`` scoping keeps it from competing with the shared theme.
    """
    target = Gdk.Screen.get_default() if screen is None else screen
    if target is None:
        return False
    key = id(target)
    if key in _INSTALLED_SCREENS:
        return False
    provider = Gtk.CssProvider()
    provider.load_from_data(ARM_UI_CSS)
    Gtk.StyleContext.add_provider_for_screen(
        target, provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION,
    )
    _INSTALLED_SCREENS.add(key)
    return True
