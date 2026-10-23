"""Single source of truth for the operator-console visual palette."""
from __future__ import annotations

import os
import ctypes
import ctypes.util
from functools import lru_cache
from pathlib import Path


ACTIVE_THEME = "graphite"

THEMES: dict[str, dict[str, str]] = {
    "raktor": {
        "app_bg": "#111D23",
        "header_bg": "#15232A",
        "tab_bg": "#15232A",
        "surface": "#1B2B32",
        "surface_soft": "#20313A",
        "surface_raised": "#2B3E47",
        "surface_selected": "#374B52",
        "video_stage": "#182930",
        "video_overlay": "rgba(29,45,53,0.90)",
        "text_main": "#EDF1E9",
        "text_secondary": "#A6B8B8",
        "text_muted": "#9AAEB3",
        "text_on_dark": "#EDF1E9",
        "primary": "#E7D6AB",
        "primary_hover": "#F4E6C5",
        "primary_soft": "#3E3D32",
        "ai_cyan": "#A9CBC3",
        "ai_cyan_bright": "#BFDAD3",
        "success": "#7CC7A6",
        "warning": "#E4B86A",
        "danger": "#F08177",
        "danger_hover": "#FF9588",
        "offline": "#849A9C",
        "border": "#34474F",
        "border_strong": "#516871",
        "divider": "#2B3D46",
        "shadow": "rgba(0,0,0,0.16)",
    },
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


THEMES["graphite"] = dict(THEMES["raktor"],
    app_bg="#262626", header_bg="#303030", tab_bg="#303030",
    surface="#333333", surface_soft="#2D2D2D", surface_raised="#3E3E3E",
    surface_selected="#494949", video_stage="#292929", video_overlay="rgba(41,41,41,0.94)",
    text_main="#F2F2F2", text_secondary="#C9C9C9", text_muted="#ADADAD", text_on_dark="#F2F2F2",
    primary="#26E88A", primary_hover="#70F2B2", primary_soft="#254536",
    ai_cyan="#68DEA5", ai_cyan_bright="#9AF1C5", success="#26E88A",
    warning="#FFC34A", danger="#FF8A75", danger_hover="#FFA18F", offline="#969696",
    border="#454545", border_strong="#666666", divider="#3D3D3D", shadow="rgba(0,0,0,0.12)")


def active_theme_name() -> str:
    """Return the code default, with a screenshot-only environment override."""
    name = os.environ.get("OPERATOR_CONSOLE_THEME", ACTIVE_THEME).strip().lower()
    return name if name in THEMES else ACTIVE_THEME


def theme_tokens(name: str | None = None) -> dict[str, str]:
    selected = active_theme_name() if name is None else name
    return dict(THEMES[selected])


def theme_stylesheet() -> bytes:
    """Native GTK presentation layer; legacy theme remains selectable."""
    if active_theme_name() not in ("raktor", "graphite"):
        return b""
    colours = "\n".join(
        f"@define-color {name} {value};"
        for name, value in theme_tokens().items()
    ).encode("utf-8")
    base = (Path(__file__).parent / "assets" / "raktor.css").read_bytes()
    if active_theme_name() == "graphite":
        base += (Path(__file__).parent / "assets" / "graphite.css").read_bytes()
    return colours + base


@lru_cache(maxsize=1)
def install_bundled_fonts() -> tuple[str, ...]:
    """Register fonts for this process only; never change desktop preferences.

    Missing Fontconfig/fonts are nonfatal on hosts using another font backend.
    Call before creating the console's first Pango layout.
    """
    library = ctypes.util.find_library("fontconfig")
    if not library:
        return ()
    try:
        fontconfig = ctypes.CDLL(library)
        fontconfig.FcConfigGetCurrent.restype = ctypes.c_void_p
        fontconfig.FcConfigAppFontAddFile.argtypes = (ctypes.c_void_p, ctypes.c_char_p)
        fontconfig.FcConfigAppFontAddFile.restype = ctypes.c_int
        config = fontconfig.FcConfigGetCurrent()
        registered = []
        for name in ("Manrope-VF.ttf", "SUIT-Variable.ttf", "NoonnuBasicGothic-Regular.ttf"):
            path = Path(__file__).parent / "assets" / "fonts" / name
            if path.is_file() and fontconfig.FcConfigAppFontAddFile(config, os.fsencode(path)):
                registered.append(name)
        if registered:
            import gi
            gi.require_version("PangoCairo", "1.0")
            from gi.repository import PangoCairo
            PangoCairo.FontMap.get_default().changed()
        return tuple(registered)
    except (OSError, AttributeError):
        return ()
