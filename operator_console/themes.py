"""Single source of truth for the operator-console visual palette."""
from __future__ import annotations

import os


ACTIVE_THEME = "arctic"

THEMES: dict[str, dict[str, str]] = {
    "arctic": {
        "app_bg": "#F4F7FB",
        "header_bg": "#FFFFFF",
        "tab_bg": "#FFFFFF",
        "surface": "#FFFFFF",
        "surface_soft": "#EDF3F8",
        "surface_raised": "#EDF3F8",
        "surface_selected": "#E8F0FE",
        "video_stage": "#06111D",
        "video_overlay": "rgba(8,19,31,0.92)",
        "text_main": "#10243A",
        "text_secondary": "#71869A",
        "text_muted": "#8B9AAC",
        "text_on_dark": "#DCE6EF",
        "primary": "#2F7CF6",
        "primary_hover": "#246BE0",
        "primary_soft": "#E8F0FE",
        "ai_cyan": "#5B8DEF",
        "ai_cyan_bright": "#5B8DEF",
        "success": "#2FA36B",
        "warning": "#E3A132",
        "danger": "#D94B55",
        "danger_hover": "#C94C57",
        "offline": "#8B9AAC",
        "border": "#D8E2EC",
        "border_strong": "#263A4D",
        "divider": "#D8E2EC",
        "shadow": "rgba(16,36,58,0.04)",
    },
}


def active_theme_name() -> str:
    """Return the code default, with a screenshot-only environment override."""
    name = os.environ.get("OPERATOR_CONSOLE_THEME", ACTIVE_THEME).strip().lower()
    return name if name in THEMES else ACTIVE_THEME


def theme_tokens(name: str | None = None) -> dict[str, str]:
    selected = active_theme_name() if name is None else name
    return dict(THEMES[selected])
