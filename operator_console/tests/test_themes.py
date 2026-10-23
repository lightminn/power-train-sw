from operator_console.themes import (
    ACTIVE_THEME, THEMES, active_theme_name, theme_stylesheet, theme_tokens,
)


REQUIRED_TOKENS = {
    "app_bg", "header_bg", "tab_bg", "surface", "surface_soft",
    "surface_raised", "surface_selected", "video_stage", "video_overlay",
    "text_main", "text_secondary", "text_muted", "text_on_dark",
    "primary", "primary_hover", "primary_soft", "ai_cyan", "ai_cyan_bright",
    "success", "warning", "danger", "danger_hover", "offline",
    "border", "border_strong", "divider", "shadow",
}


def test_every_theme_has_the_complete_token_shape():
    assert set(THEMES) == {"arctic", "raktor", "graphite"}
    for theme in THEMES.values():
        assert set(theme) == REQUIRED_TOKENS


def test_arctic_semantic_tokens_match_the_requested_palette():
    theme = THEMES["arctic"]
    assert theme["app_bg"] == "#F4F7FB"
    assert theme["video_stage"] == "#06111D"
    assert theme["primary"] == "#2F7CF6"
    assert theme["success"] == "#2FA36B"
    assert theme["warning"] == "#E3A132"
    assert theme["danger"] == "#D94B55"


def test_code_default_is_graphite(monkeypatch):
    monkeypatch.delenv("OPERATOR_CONSOLE_THEME", raising=False)
    assert ACTIVE_THEME == "graphite"
    assert active_theme_name() == "graphite"


def test_capture_override_only_selects_an_existing_theme(monkeypatch):
    monkeypatch.setenv("OPERATOR_CONSOLE_THEME", "arctic")
    assert theme_tokens()["app_bg"] == "#F4F7FB"
    monkeypatch.setenv("OPERATOR_CONSOLE_THEME", "not-a-theme")
    assert active_theme_name() == ACTIVE_THEME


def _luminance(hex_colour):
    channels = [int(hex_colour[i:i + 2], 16) / 255 for i in (1, 3, 5)]
    linear = [x / 12.92 if x <= .04045 else ((x + .055) / 1.055) ** 2.4
              for x in channels]
    return sum(x * weight for x, weight in zip(linear, (.2126, .7152, .0722)))


def test_graphite_text_and_state_colours_remain_legible_on_surfaces():
    theme = THEMES["graphite"]
    for text in ("text_main", "text_secondary", "text_muted", "primary",
                 "success", "warning", "danger"):
        for surface in ("app_bg", "surface", "surface_soft"):
            light, dark = sorted((_luminance(theme[text]), _luminance(theme[surface])))
            assert (dark + .05) / (light + .05) >= 4.5, (text, surface)


def test_theme_override_also_selects_its_presentation_layer(monkeypatch):
    monkeypatch.setenv("OPERATOR_CONSOLE_THEME", "raktor")
    assert b"@define-color primary " in theme_stylesheet()
    monkeypatch.setenv("OPERATOR_CONSOLE_THEME", "arctic")
    assert theme_stylesheet() == b""
