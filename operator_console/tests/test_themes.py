from operator_console.themes import ACTIVE_THEME, THEMES, active_theme_name, theme_tokens
from operator_console.arm_ui.styling import ARM_UI_CSS


REQUIRED_TOKENS = {
    "app_bg", "header_bg", "tab_bg", "surface", "surface_soft",
    "surface_raised", "surface_selected", "video_stage", "video_overlay",
    "text_main", "text_secondary", "text_muted", "text_on_dark",
    "primary", "primary_hover", "primary_soft", "ai_cyan", "ai_cyan_bright",
    "success", "warning", "danger", "danger_hover", "offline",
    "border", "border_strong", "divider", "shadow",
}


def test_arctic_theme_has_the_complete_token_shape():
    assert set(THEMES) == {"arctic"}
    assert set(THEMES["arctic"]) == REQUIRED_TOKENS


def test_arctic_semantic_tokens_match_the_requested_palette():
    theme = THEMES["arctic"]
    assert theme["app_bg"] == "#F4F7FB"
    assert theme["video_stage"] == "#06111D"
    assert theme["primary"] == "#2F7CF6"
    assert theme["success"] == "#2FA36B"
    assert theme["warning"] == "#E3A132"
    assert theme["danger"] == "#D94B55"


def test_code_default_is_arctic(monkeypatch):
    monkeypatch.delenv("OPERATOR_CONSOLE_THEME", raising=False)
    assert ACTIVE_THEME == "arctic"
    assert active_theme_name() == "arctic"


def test_capture_override_only_selects_an_existing_theme(monkeypatch):
    monkeypatch.setenv("OPERATOR_CONSOLE_THEME", "arctic")
    assert theme_tokens()["app_bg"] == "#F4F7FB"
    monkeypatch.setenv("OPERATOR_CONSOLE_THEME", "not-a-theme")
    assert active_theme_name() == ACTIVE_THEME


def test_arm_operation_surfaces_use_opaque_purple_alpha_without_fading_text():
    """Panel/button alpha belongs to the surface, never a whole-widget opacity."""
    css = ARM_UI_CSS.decode("utf-8")
    assert "background: rgba(31, 22, 52, 0.94)" in css
    assert "background: rgba(77, 52, 122, 0.94)" in css
    assert "opacity: 0.8" not in css
    assert ".arm-ui label { color: #EEE8FA; }" in css
