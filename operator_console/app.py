#!/usr/bin/env python3
"""GTK console with RX-only observation and token-gated ops commands.

Observation never opens robot hardware or transmits on its receive channels.
Operator actions travel only through the authenticated ops channel; the robot
remains the SRT listener and the operator laptop remains an SRT caller.
"""
from __future__ import annotations

import argparse
import json
import math
import os
from pathlib import Path
import signal
import sys
import time
from collections.abc import Callable

from operator_console.pipelines import pipeline_description, srt_uri

import gi

gi.require_foreign("cairo")
gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
gi.require_version("Gst", "1.0")
gi.require_version("Rsvg", "2.0")
from gi.repository import Gdk, GLib, Gst, Gtk, Pango, Rsvg  # noqa: E402

from .arm_telemetry import (
    ArmTelemetrySnapshot,
    LatestArmTelemetryReceiver,
    arm_panel_summary,
    arm_source_freshness,
    arm_summary,
    temperature_state,
)
from .environment_telemetry import (
    LatestEnvironmentTelemetryReceiver,
    environment_source_state,
)
from .labels import (
    OFF_LABEL,
    ON_LABEL,
    ack_korean,
    freshness_korean,
    mode_korean,
)
from .metadata import (
    DisplayTargetTracker,
    LatestMetadataReceiver,
    MetadataFrame,
    OVERLAY_STALE_AFTER_S,
    displayable_detections,
    target_distance_m,
)
from .ops_client import ConsoleOpsClient
from .ops_panel import (
    GESTURE_HOLD,
    GESTURE_IMMEDIATE,
    GESTURE_SPACER,
    PANEL_ACTIONS,
    ConfirmFlow,
    PanelAction,
    component_mask_from_state,
    format_ops_status_line,
    mode_allows_action,
    next_estop_cause_event,
)
from .presentation import (
    mission_presentation,
    public_freshness,
    public_safety,
)
from .themes import theme_tokens
from .status_view import (
    CompetitionStatusDashboard,
    END_EFFECTOR_PURPOSES,
    EnvironmentSensorDashboard,
)
from .telemetry import (
    LatestTelemetryReceiver,
    TelemetrySnapshot,
    _format_hex,
    _format_number,
    _format_ros_rates,
    _format_rss,
    chassis_summary,
    chassis_component_states,
    mask_banner_text,
    power_summary,
    safety_banner_state,
)


DEFAULT_OPS_TOKEN_FILE = "~/.config/powertrain/ops_console.token"
# srtsrc(auto-reconnect=true)는 리스너 사망·재기동을 bus ERROR 없이 삼키므로,
# 프레임을 한 번이라도 본 스트림이 이 시간 넘게 stale 이면 강제 재시작한다.
VIDEO_STALE_RESTART_S = 5.0
# Stable source-level declaration consumed by the send-surface contract test.
# Operator-visible copy below is Korean; the transport boundary remains this.
SEND_SURFACE_CONTRACT = "OBSERVE: RX-ONLY  |  OPS: TOKEN-GATED  |  "


def estop_availability(
    *, token_available: bool, link_ready: bool,
) -> tuple[bool, str, str | None]:
    """Return E-STOP sensitivity and tooltip without a persistent warning."""
    if not token_available:
        return (
            False,
            "조작 토큰이 없어 비상정지 명령을 전송할 수 없습니다",
            None,
        )
    if not link_ready:
        return (
            False,
            "조작 채널이 연결되지 않아 비상정지를 전송할 수 없습니다",
            None,
        )
    return (
        True,
        "확인 없이 즉시 토큰 인증 비상정지 명령을 전송합니다",
        None,
    )


CONSOLE_CSS = b"""
window { background: #070B11; color: #FFFFFF; font-family: "Pretendard", "Noto Sans CJK KR", "Noto Sans KR", "SUIT", sans-serif; }
label { color: #f8fafc; }
.topbar { background: #0B1119; border-bottom: 1px solid rgba(160,185,210,0.14); padding: 7px 16px; min-height: 52px; }
.top-warning { background: #492025; color: #FFE8EA; border-radius: 5px; padding: 5px 9px; font-size: 11px; font-weight: 800; }
.top-alert { background: #5A3012; color: #FFF0D8; border-radius: 5px; padding: 5px 9px; font-size: 11px; font-weight: 800; }
.brand { color: #FFFFFF; font-size: 19px; font-weight: 900; }
.eyebrow { color: #C4D0DE; font-size: 10px; font-weight: 600; }
.health-strip { background: transparent; padding: 0; }
.status-item { background: transparent; padding: 0 2px; }
.status-chip { background: transparent; color: #A7B4C3; border: none; padding: 0; font-size: 10px; font-weight: 700; }
.status-chip.status-live { color: #E7F8EF; }
.status-chip.status-camera-live { color: #EAFEFF; }
.status-chip.status-progress { color: #C4D0DE; }
.status-chip.status-warn { color: #FFE590; }
.status-chip.status-bad { color: #FFB4B8; }
.status-chip.status-muted { color: #A7B4C3; }
.status-dot { background: #647386; border-radius: 999px; min-width: 7px; min-height: 7px; }
.status-dot.status-live { background: #2FD27A; }
.status-dot.status-camera-live { background: #00D5FF; }
.status-dot.status-progress { background: #4F7FCC; }
.status-dot.status-warn { background: #FFD24A; }
.status-dot.status-bad { background: #C62832; }
.status-dot.status-muted { background: #647386; }
.global-estop { background: #D86670; color: #ffffff; border: none; border-radius: 8px; padding: 7px 18px; }
.estop-title { color: #FFFFFF; font-size: 12px; font-weight: 900; }
.estop-subtitle { color: rgba(255,255,255,0.76); font-size: 7px; font-weight: 800; letter-spacing: 1px; }
.global-estop:hover { background: #C94C57; }
.global-estop:active { background: #B4232C; border: 1px solid rgba(255,255,255,0.55); }
.global-estop:disabled { background: #C9363E; color: #FFFFFF; opacity: 0.48; }
.demo-mode { background: #11243a; color: #32c5ff; border: 1px solid rgba(56,189,248,0.55); border-radius: 10px; padding: 7px 10px; font-size: 10px; font-weight: 800; }
.role-main { background: #3478f6; color: #ffffff; border-radius: 10px; padding: 3px 7px; font-size: 9px; font-weight: 900; }
.role-sub { background: #11243a; color: #a7b4c8; border: 1px solid rgba(148,163,184,0.18); border-radius: 10px; padding: 3px 7px; font-size: 9px; font-weight: 800; }
.swap-hint { color: #a7b4c8; font-size: 9px; }
.nav { background: #0B1119; padding: 0 16px; min-height: 43px; }
.nav button { background: transparent; color: #C4D0DE; border: none; border-bottom: 2px solid transparent; border-radius: 0; min-height: 42px; padding: 7px 24px; font-weight: 800; }
.nav button:hover { color: #FFFFFF; background: rgba(255,255,255,0.035); }
.nav button:checked { background: transparent; color: #FFFFFF; border-bottom: 3px solid #3478FF; }
.page { padding: 8px 10px 0 10px; }
.card { background: #0d1b2a; border: 1px solid rgba(148,163,184,0.18); border-radius: 10px; }
.card label { color: #f8fafc; }
.card > border { border: none; }
.card > label { color: #f8fafc; font-weight: 900; padding: 8px 10px 4px 10px; }
.video-card { background: #000204; border: 1px solid rgba(160,185,210,0.14); border-radius: 5px; }
.video-card label { color: #f8fafc; }
.video-overlay { background: rgba(7,17,31,0.78); border-bottom: 1px solid rgba(148,163,184,0.18); padding: 1px 3px; }
.camera-placeholder { background: transparent; }
.camera-standby-background { background: #06111D; }
.camera-placeholder-title { color: #FFFFFF; font-size: 14px; font-weight: 800; }
.camera-placeholder-detail { color: #C4D0DE; font-size: 10px; }
.log-expander { background: #0B1119; border-top: 1px solid rgba(160,185,210,0.14); padding: 2px 12px; min-height: 26px; }
.log-expander > title { color: #6f8098; font-size: 10px; font-weight: 700; }
.event-card { background: #0d1b2a; border-top: 1px solid rgba(148,163,184,0.18); }
.event-card text { background: #07111f; color: #a7b4c8; font-family: "JetBrains Mono", "D2Coding", monospace; font-size: 10px; }
.muted { color: #6f8098; }
.section-title { color: #f8fafc; font-size: 15px; font-weight: 900; }
.mission-summary { background: #0b1728; border: 1px solid #263b56; border-radius: 10px; padding: 10px; }
.mission-metric { background: #101f33; border: 1px solid #2b405c; border-radius: 9px; padding: 9px 12px; }
.metric-title { color: #6f8098; font-size: 9px; font-weight: 800; }
.metric-value { color: #f8fafc; font-size: 15px; font-weight: 900; }
.story-panel { background: #0d1b2a; border: 1px solid rgba(148,163,184,0.18); border-radius: 10px; padding: 11px; }
.story-panel label { color: #f8fafc; }
.story-panel .story-kicker { color: #32c5ff; font-size: 9px; font-weight: 900; }
.story-panel .story-phase { background: transparent; color: #f8fafc; border: none; border-bottom: 2px solid #3478f6; border-radius: 0; padding: 4px 0 8px 0; font-size: 30px; font-weight: 900; }
.story-panel .story-description { color: #a7b4c8; font-size: 11px; padding: 0 0 4px 0; }
.step { background: transparent; color: #6f8098; border-bottom: 2px solid #33465d; padding: 5px 2px; font-size: 9px; font-weight: 800; }
.step.active { color: #32c5ff; border-bottom-color: #3478f6; }
.step.completed { color: #26c281; border-bottom-color: #26c281; }
.story-card { background: #11243a; border: 1px solid rgba(148,163,184,0.18); border-radius: 10px; padding: 8px 10px; }
.target-card, .distance-card, .tool-card, .next-card, .safety-card { border-left: 2px solid #3478f6; }
.distance-card .metric-title { color: #6f8098; }
.distance-card .metric-value { color: #f8fafc; font-size: 20px; }
.readiness { background: #0d1b2a; border-radius: 11px; padding: 7px 12px; }
.readiness-title { color: #f8fafc; font-size: 11px; font-weight: 900; }
.readiness-detail { color: #a7b4c8; font-size: 9px; }
.system-card { background: #102238; border: none; border-radius: 11px; padding: 11px; }
.system-card.selected { background: #E8F0FE; border: 1px solid #2F7CF6; }
.system-name { color: #a7b4c8; font-size: 10px; font-weight: 800; }
.system-value { color: #f8fafc; font-size: 16px; font-weight: 900; }
.diagnostics { background: #0d1b2a; border-radius: 11px; padding: 8px; }
.danger-card { border: 1px solid rgba(239,68,68,0.35); background: #0d1b2a; border-radius: 10px; }
.danger-card label { color: #f8fafc; }
.progress-hud { background: #0D1C2B; border: 1px solid #263A4D; border-radius: 11px; padding: 7px 18px; }
.progress-caption { color: #71869A; font-size: 9px; font-weight: 800; }
.progress-current { color: #DCE6EF; font-size: 15px; font-weight: 900; }
.progress-track { background: transparent; }
.progress-connector { background: #536A80; min-height: 1px; margin: 0 9px; }
.progress-connector.active { background: #2D6EDB; }
.progress-connector.completed { background: #2D6EDB; }
.progress-marker { background: transparent; color: transparent; border: 1px solid #8293A7; border-radius: 999px; min-width: 9px; min-height: 9px; font-size: 1px; }
.progress-step { color: #8293A7; font-size: 10px; font-weight: 700; }
.progress-step.active { color: #F5F9FC; font-weight: 800; }
.progress-marker.active { background: #2F7CF6; color: transparent; border: 2px solid #8EB7FF; min-width: 11px; min-height: 11px; }
.progress-step.completed { color: #DCE8E3; }
.progress-marker.completed { background: #2F7CF6; color: transparent; border: 1px solid #2F7CF6; min-width: 9px; min-height: 9px; }

/* Visual language shared by the seven approved competition references. */
.console-shell { background: #030712; }
.side-rail { background: #070D1D; border-right: 1px solid #141C31; padding: 16px 0; min-width: 54px; }
.rail-logo { background: #6957F5; color: #FFFFFF; border-radius: 9px; min-width: 40px; min-height: 40px; font-size: 18px; font-weight: 900; }
.rail-icon { background: transparent; color: #7180A7; border: none; border-radius: 8px; min-width: 40px; min-height: 40px; font-size: 16px; }
.rail-icon:hover { background: #111936; color: #DDE4FF; }
.rail-icon.active { background: #172044; color: #B8C4FF; border: 1px solid #4D5EAD; }
.rail-icon label { color: #7180A7; font-size: 16px; }
.rail-icon:hover label, .rail-icon.active label { color: #DDE4FF; }
.topbar { background: #030712; border-bottom: none; padding: 18px 28px 10px 28px; min-height: 66px; }
.brand { font-size: 18px; }
.eyebrow { color: #596580; font-size: 9px; }
.global-estop { background: #641D42; border-radius: 8px; min-width: 78px; box-shadow: 0 0 24px rgba(195,45,111,0.22); }
.nav { background: #030712; padding: 0 28px 12px 28px; }
.nav stackswitcher { background: #080D18; border: 1px solid #20283A; border-radius: 7px; padding: 4px; }
.nav button { min-height: 34px; padding: 5px 22px; border: none; border-radius: 5px; }
.nav button:checked { color: #FFFFFF; background: #252B61; border: none; box-shadow: 0 0 18px rgba(76,91,229,0.28); }
.page { padding: 8px 28px 0 28px; }
.mission-rail { background: #EDF3F8; border-left: 1px solid #D8E2EC; border-radius: 0; padding: 15px 14px; }
.rail-heading-ko { color: #26384A; font-size: 17px; font-weight: 900; }
.rail-heading-en { color: #708195; font-size: 9px; font-weight: 900; letter-spacing: 1.3px; }
.rail-section { background: rgba(255,255,255,0.68); border: 1px solid rgba(180,197,214,0.62); border-radius: 7px; padding: 10px; }
.rail-section-title { color: #34495E; font-size: 13px; font-weight: 900; }
.rail-section-count { color: #607287; font-size: 12px; font-weight: 900; }
.rail-device-row { min-height: 34px; }
.rail-device-name { color: #516579; font-size: 11px; font-weight: 800; }
.rail-section-divider { background: rgba(180,197,214,0.62); min-height: 1px; margin: 1px 2px; }
.rail-preparation { background: rgba(255,255,255,0.68); border: 1px solid rgba(180,197,214,0.62); border-radius: 7px; padding: 10px; }
.rail-title { color: #34495E; font-size: 13px; font-weight: 900; }
.rail-description { color: #607287; font-size: 11px; }
.rail-data-row { border-bottom: 1px solid rgba(160,185,210,0.14); padding: 2px 0 10px 0; }
.rail-data-label { color: #8090A3; font-size: 9px; font-weight: 800; }
.rail-data-value { color: #FFFFFF; font-size: 17px; font-weight: 900; }
.rail-data-target, .rail-data-distance { color: #00D5FF; }
.rail-tool-selector { min-height: 34px; font-size: 12px; font-weight: 800; }
.rail-tool-selector button { padding: 5px 9px; }
menu {
  background: #FFFFFF;
  color: #17324A;
  border: 1px solid #C9D7E3;
  padding: 4px;
}
menuitem {
  background: #FFFFFF;
  color: #17324A;
  min-height: 28px;
  padding: 5px 10px;
}
menuitem label { color: #17324A; font-size: 12px; font-weight: 700; }
menuitem:hover, menuitem:active { background: #E8F2FF; color: #174EA6; }
menuitem:hover label, menuitem:active label { color: #174EA6; }
.rail-safety { background: rgba(255,255,255,0.68); border: 1px solid rgba(180,197,214,0.62); border-radius: 7px; padding: 9px 10px; }
.rail-safety-value { color: #C4D0DE; font-size: 11px; font-weight: 800; }
.rail-safety-value.status-live { color: #2FD27A; }
.rail-safety-value.status-warn { color: #FFE590; }
.rail-safety-value.status-bad { color: #FFB4B8; }
.main-camera-label { background: rgba(6,16,27,0.82); border-radius: 6px; padding: 8px 10px; }
.camera-name { color: #FFFFFF; font-size: 13px; font-weight: 900; }
.camera-connection-label { color: #B8C8D8; font-size: 9px; font-weight: 800; }
.event-filter { min-height: 26px; color: #526477; padding: 0; font-size: 13px; font-weight: 600; }
.event-filter check { min-width: 16px; min-height: 16px; margin-right: 7px; border: 1px solid #9BA9B7; border-radius: 3px; background: #F8FAFC; }
.event-filter check:checked { background: #2F7CF6; border-color: #2F7CF6; color: #FFFFFF; }
.event-filter-error { color: #D94B55; }
.event-filter-warning { color: #E3A132; }
.event-filter-info { color: #2F7CF6; }
.event-expander { background: #F8FAFC; border-top: 1px solid #DCE5ED; padding: 0 14px; }
.event-expander:hover { background: #EDF3F8; }
.event-expander > title { color: #71869A; }
.event-header { min-height: 46px; }
.event-status-title { color: #526477; font-size: 12px; font-weight: 800; }
.event-status-message { color: #8291A1; font-size: 12px; }
.event-collapse { min-height: 26px; padding: 2px 7px; color: #6F8295; font-size: 11px; font-weight: 600; background: transparent; border: none; box-shadow: none; }
.event-drawer-panel { background: #08131F; border-top: 1px solid #263A4D; padding: 12px 16px; }
.event-card { background: #F8FAFC; border: none; padding: 0; }
.event-card text { background: #FFFFFF; color: #263A4D; font-family: "Pretendard", "Noto Sans CJK KR", sans-serif; font-size: 11px; }
.event-operation-list { background: #F8FAFC; }
.event-column-header { min-height: 32px; background: #EEF3F7; border-bottom: 1px solid #DCE5ED; padding: 0 12px; margin-top: 8px; }
.event-column-title { color: #6F8295; font-size: 10px; font-weight: 600; }
.event-operation-row { background: #F8FAFC; border-bottom: 1px solid #DCE5ED; padding: 7px 12px; min-height: 36px; }
.event-operation-row:hover { background: #F7FAFC; }
.event-operation-time { color: #8291A1; font-size: 10px; font-family: "JetBrains Mono", "D2Coding", monospace; }
.event-operation-level { font-size: 10px; font-weight: 700; font-family: "JetBrains Mono", "D2Coding", monospace; }
.event-operation-level.event-level-error { color: #C43B43; }
.event-operation-level.event-level-warning { color: #C98718; }
.event-operation-level.event-level-info { color: #2D6EDB; }
.event-operation-message { color: #526477; font-size: 11px; }
.event-operation-module { color: #496983; font-size: 10px; font-weight: 700; font-family: "JetBrains Mono", "D2Coding", monospace; }
.event-operation-separator { color: #94A8B9; font-size: 10px; }
.event-operation-technical { color: #6F8295; font-size: 10px; font-family: "JetBrains Mono", "D2Coding", monospace; }
.event-operation-arrow { color: #8B9AAC; font-size: 10px; }
.event-operation-raw { color: #60788F; background: #F5F8FB; border-top: 1px solid #E1E8F0; padding: 5px 8px 3px 32px; font-family: "JetBrains Mono", "D2Coding", monospace; font-size: 9px; }
.developer-toggle { color: #71869A; font-size: 10px; }
.developer-box { background: #EDF3F8; border-top: 1px solid #D8E2EC; padding: 8px; }
.developer-box label { color: #71869A; font-size: 9px; }
.pip-frame { background: #020810; border: 1px solid rgba(150,175,198,0.60); border-radius: 7px; box-shadow: 0 8px 22px rgba(0,12,28,0.28); }
.pip-swap-button { min-height: 28px; min-width: 88px; padding: 3px 9px; border-radius: 6px; border: 1px solid rgba(166,190,214,0.72); background: rgba(8,25,42,0.90); color: #FFFFFF; font-size: 10px; font-weight: 800; }
.pip-swap-button:hover { background: rgba(35,83,130,0.96); border-color: #8EC5FF; }
.preparation-status-value { color: #8B9AAA; font-size: 11px; font-weight: 800; }
.preparation-status-dot { background: #8B9AAA; border-radius: 999px; min-width: 7px; min-height: 7px; }
.preparation-status-dot.prep-ready { background: #228764; }
.preparation-status-dot.prep-connecting { background: #2D6EDB; }
.preparation-status-dot.prep-attention { background: #D69A24; }
.preparation-status-dot.prep-offline { background: #8B9AAA; }
.preparation-status-value.prep-ready { color: #228764; }
.preparation-status-value.prep-connecting { color: #2D6EDB; }
.preparation-status-value.prep-attention { color: #D69A24; }
.preparation-status-dot.prep-error { background: #C43B43; }
.preparation-status-value.prep-error { color: #C43B43; }
.preparation-status-value.prep-offline { color: #8B9AAA; }
.status-overall { color: #17263A; font-size: 22px; font-weight: 900; }
.status-readiness { background: #FFFFFF; border: 1px solid #D0DBE5; border-radius: 8px; padding: 12px 14px; }
.status-priority { color: #C98718; font-size: 12px; font-weight: 700; }
.status-summary-card { background: #FFFFFF; border: 1px solid #D0DBE5; border-left: 3px solid #8796A8; border-radius: 8px; padding: 10px 12px; box-shadow: none; }
.status-summary-card:hover { background: #F4F8FE; border-color: #9CB9E8; }
.status-summary-card.selected { background: #E8F0FE; border-color: #2D6EDB; border-left-color: #2D6EDB; }
.status-summary-value { color: #8796A8; font-size: 16px; font-weight: 900; }
.status-summary-value.status-live { color: #218A63; }
.status-summary-value.status-progress { color: #2767D8; }
.status-summary-value.status-warn { color: #D49420; }
.status-summary-value.status-bad { color: #C43B43; }
.status-view-option { color: #526477; font-size: 12px; }
.status-view-option { min-height: 36px; padding: 0 16px; border-radius: 7px; border: 1px solid #C9D5E1; background: #FFFFFF; color: #607287; box-shadow: none; }
.status-view-option:hover { background: #E8F0FB; color: #17263A; }
.status-view-option:checked { background: #2D6EDB; color: #FFFFFF; border-color: #2D6EDB; }
.status-view-option:focus { border-color: #2D6EDB; box-shadow: inset 0 0 0 1px #2D6EDB; }
.status-panel { background: #FFFFFF; border: 1px solid #D0DBE5; border-radius: 8px; padding: 12px; }
.status-panel-title { color: #17263A; font-size: 16px; font-weight: 900; }
.status-panel-values { color: #34495E; font-size: 12px; }
.status-panel-updated { color: #8796A8; font-size: 10px; }
.power-metric, .detail-metric { background: #F5F8FC; border: 1px solid #D8E2EC; border-radius: 7px; padding: 9px 11px; }
.power-metric-title, .detail-metric-title { color: #71869A; font-size: 10px; font-weight: 700; }
.power-metric-value, .detail-metric-value { color: #17263A; font-size: 18px; font-weight: 900; }
.power-state-summary { background: #F8FAFC; border-left: 3px solid #2D6EDB; color: #34495E; padding: 8px 11px; font-size: 11px; }
.soc-progress trough { min-height: 6px; background: #E3EAF1; border: none; }
.soc-progress progress { min-height: 6px; background: #218A63; border: none; }
.soc-progress text { color: #607287; font-size: 10px; }
.display-options { background: #FFFFFF; border: 1px solid #D0DBE5; border-radius: 7px; padding: 8px 10px; }
.display-options-title { color: #17263A; font-size: 11px; font-weight: 900; }
.display-option { color: #17263A; font-size: 12px; padding: 0; }
.display-option check { min-width: 15px; min-height: 15px; margin-right: 7px; border: 1px solid #9BA9B7; border-radius: 3px; background: #F8FAFC; }
.display-option check:checked { background: #2D6EDB; border-color: #2D6EDB; color: #FFFFFF; }
button { border-radius: 10px; padding: 7px 10px; }
button:hover { border-color: rgba(56,189,248,0.55); }
scrollbar slider { background: #33465d; border-radius: 10px; min-width: 6px; min-height: 6px; }
"""


def _theme_css() -> bytes:
    token = theme_tokens()
    css = """
window {{ background: {app_bg}; color: {text_main}; }}
label {{ color: {text_main}; }}
.topbar {{ background: {header_bg}; border-bottom: 1px solid {divider}; }}
.brand, .readiness-title, .story-panel label, .system-value, .section-title, .metric-value {{ color: {text_main}; }}
.eyebrow {{ color: {text_secondary}; }}
.muted, .readiness-detail, .story-description, .metric-title, .system-name, .swap-hint {{ color: {text_muted}; }}
.status-chip {{ color: {offline}; }}
.status-chip.status-live {{ color: {success}; }}
.status-chip.status-camera-live {{ color: {ai_cyan}; }}
.status-chip.status-progress {{ color: #6F8EAA; }}
.status-chip.status-warn {{ color: {warning}; }}
.status-chip.status-bad {{ color: {danger}; }}
.status-chip.status-muted {{ color: {offline}; }}
.nav {{ background: {tab_bg}; border-bottom: 1px solid {divider}; }}
.nav button {{ background: transparent; color: {text_secondary}; border: none; border-bottom: 3px solid transparent; box-shadow: none; outline: none; }}
.nav button:hover {{ color: {text_main}; background: {surface_soft}; border: none; border-bottom: 3px solid transparent; box-shadow: none; }}
.nav button:checked {{ background: {surface}; color: {text_main}; border: none; border-bottom: 3px solid {primary}; box-shadow: none; outline: none; }}
.nav button:focus {{ border: none; border-bottom: 3px solid transparent; box-shadow: none; outline: none; }}
.nav button:checked:focus {{ border: none; border-bottom: 3px solid {primary}; box-shadow: none; outline: none; }}
.page {{ background: {app_bg}; }}
.story-panel, .readiness, .system-card, .diagnostics, .card {{ background: {surface}; border-color: {border}; }}
.story-card, .safety-row {{ background: {surface_soft}; border-color: {border}; }}
.card label, .card > label {{ color: {text_main}; }}
.mission-summary, .mission-metric {{ background: {surface}; border-color: {border}; }}
.demo-mode {{ background: {primary_soft}; color: {primary}; border-color: rgba(36,107,254,0.24); }}
.role-main {{ background: {primary}; color: #FFFFFF; }}
.role-sub {{ background: {surface_raised}; color: {text_secondary}; border-color: {border}; }}
.danger-card {{ background: {surface}; border-color: rgba(201,54,62,0.35); }}
.danger-card label {{ color: {text_main}; }}
.event-card, .event-card text, .event-operation-list, .event-operation-row {{ background: #F8FAFC; }}
.video-card {{ background: {video_stage}; border-color: {border_strong}; }}
.camera-placeholder {{ background: transparent; border-color: transparent; }}
.video-card label {{ color: {text_on_dark}; }}
.video-overlay {{ background: {video_overlay}; border-bottom-color: rgba(255,255,255,0.12); }}
.camera-placeholder-title {{ color: {text_on_dark}; }}
.camera-placeholder-detail {{ color: #AAB9C6; }}
.camera-name {{ color: {text_on_dark}; }}
.story-kicker {{ color: {primary}; }}
.step {{ color: {text_muted}; border-bottom-color: {border}; }}
.step.active {{ color: {primary}; border-bottom-color: {primary}; }}
.step.completed {{ color: {success}; border-bottom-color: {success}; }}
.safety-label {{ color: {text_secondary}; }}
.safety-value {{ color: {text_main}; }}
.status-dot.status-live {{ background: {success}; }}
.status-dot.status-camera-live {{ background: {ai_cyan}; }}
.status-dot.status-progress {{ background: #6F8EAA; }}
.status-dot.status-warn {{ background: {warning}; }}
.status-dot.status-bad {{ background: {danger}; }}
.status-dot.status-muted {{ background: {offline}; }}
.global-estop {{ background: #D86670; color: #FFFFFF; border: none; box-shadow: none; }}
.global-estop:hover {{ background: #C94C57; }}
.global-estop:active {{ background: #B4232C; }}
.global-estop:disabled {{ background: {danger}; color: #FFFFFF; opacity: 0.48; }}
.progress-hud {{ background: rgba(10,25,40,0.88); border: 1px solid rgba(126,163,194,0.20); box-shadow: 0 6px 18px rgba(0,10,24,0.24); }}
.progress-connector {{ background: #536A80; }}
.progress-connector.active {{ background: #2D6EDB; }}
.progress-connector.completed {{ background: #2D6EDB; }}
.progress-marker {{ background: transparent; color: transparent; border-color: #8293A7; }}
.progress-step {{ color: #8293A7; }}
.progress-step.active {{ color: {text_on_dark}; }}
.progress-marker.active {{ background: #2D6EDB; color: transparent; border-color: #2D6EDB; }}
.progress-step.completed {{ color: #DCE8E3; }}
.progress-marker.completed {{ background: transparent; color: #228764; border-color: #228764; }}
.mission-rail {{ background: #EDF3F8; border-left: 1px solid {divider}; }}
.rail-section, .rail-preparation, .rail-data-row, .rail-safety {{ background: rgba(255,255,255,0.68); border: 1px solid rgba(180,197,214,0.62); border-radius: 7px; box-shadow: none; }}
.rail-data-row {{ padding: 8px 10px; }}
.rail-title, .rail-data-value {{ color: {text_main}; }}
.rail-description {{ color: {text_secondary}; }}
.rail-data-label {{ color: {text_muted}; }}
.rail-data-target, .rail-data-distance {{ color: {ai_cyan}; }}
.rail-safety-value {{ color: {text_muted}; }}
.rail-safety-value.status-live {{ color: {success}; }}
.rail-safety-value.status-warn {{ color: {warning}; }}
.rail-safety-value.status-bad {{ color: {danger}; }}
.pip-frame {{ background: {video_stage}; border: 1px solid rgba(150,175,198,0.60); border-radius: 7px; box-shadow: 0 8px 22px rgba(0,12,28,0.28); }}
.pip-frame .video-overlay {{ background: rgba(10,25,40,0.94); }}
.camera-placeholder {{ background: transparent; }}
.camera-placeholder-title {{ color: {text_on_dark}; }}
.camera-placeholder-detail {{ color: #B8C8D8; }}
.log-expander, .event-drawer {{ background: {header_bg}; border-top: 1px solid {divider}; }}
.log-expander > title {{ color: {text_secondary}; }}
scrollbar slider {{ background: {border_strong}; }}

/* Premium command-center finish: restrained depth, crisp hierarchy. */
window {{ background: #EEF2F6; }}
.topbar {{
  background: #0B1726;
  border-bottom: 1px solid #22364B;
  padding: 11px 20px;
  min-height: 62px;
}}
.topbar .brand {{ color: #F7FAFC; font-size: 21px; font-weight: 900; }}
.topbar .eyebrow {{ color: #8EA3B8; font-size: 10px; letter-spacing: 0.4px; }}
.topbar .status-chip {{ color: #91A5B8; font-size: 10px; }}
.topbar .status-chip.status-live {{ color: #79D9AE; }}
.topbar .status-chip.status-camera-live {{ color: #7DC8FF; }}
.nav {{ background: #0B1726; border-bottom: 1px solid #203449; padding: 0 18px; }}
.nav button {{ color: #93A8BB; min-height: 48px; padding: 8px 28px; font-size: 13px; }}
.nav button label {{ color: #93A8BB; font-weight: 800; }}
.nav button:hover {{ color: #EAF1F7; background: rgba(255,255,255,0.035); }}
.nav button:hover label {{ color: #EAF1F7; }}
.nav button:checked {{ color: #FFFFFF; background: rgba(64,132,255,0.08); border-bottom-color: #5A9BFF; }}
.nav button:checked label {{ color: #FFFFFF; }}
.page {{ background: #EEF2F6; padding: 12px 14px 0 14px; }}
.global-estop {{
  background: #D85A64;
  border: 1px solid rgba(255,255,255,0.12);
  border-radius: 12px;
  box-shadow: 0 7px 18px rgba(103,22,31,0.25);
  padding: 8px 20px;
}}
.global-estop:hover {{ background: #C94B56; box-shadow: 0 9px 22px rgba(103,22,31,0.32); }}
.global-estop:active {{ background: #B93843; box-shadow: inset 0 2px 5px rgba(71,8,15,0.28); }}
.status-readiness {{
  background: #10243A;
  border: 1px solid #233B54;
  border-radius: 12px;
  padding: 15px 18px;
  box-shadow: 0 8px 22px rgba(22,42,62,0.10);
}}
.status-readiness .system-name {{ color: #8FA6BA; letter-spacing: 0.4px; }}
.status-overall {{ color: #F6F9FC; font-size: 24px; }}
.status-readiness .muted {{ color: #91A6B9; }}
.status-priority {{ color: #F0B74A; }}
.status-summary-card {{
  background: #FFFFFF;
  border: 1px solid #D7E0E9;
  border-left: 3px solid #8D9DAE;
  border-radius: 11px;
  padding: 12px 14px;
  box-shadow: 0 4px 14px rgba(27,48,70,0.055);
}}
.status-summary-card:hover {{
  background: #FBFDFF;
  border-color: #AFC7E3;
  box-shadow: 0 7px 18px rgba(27,48,70,0.09);
}}
.status-summary-card.selected {{
  background: #F1F6FF;
  border-color: #6E9FEA;
  border-left-color: #2F7CF6;
  box-shadow: 0 7px 20px rgba(47,124,246,0.12);
}}
.status-panel {{
  background: #FFFFFF;
  border: 1px solid #D7E0E9;
  border-radius: 12px;
  padding: 15px;
  box-shadow: 0 8px 24px rgba(27,48,70,0.065);
}}
.status-panel-title {{ color: #132A42; font-size: 17px; }}
.power-metric, .detail-metric {{
  background: #F5F8FB;
  border: 1px solid #DDE5ED;
  border-radius: 9px;
  padding: 11px 13px;
}}
.power-metric-value, .detail-metric-value {{ color: #112940; font-size: 18px; }}
.power-state-summary {{ background: #F4F8FD; border-left-color: #4B8CF5; border-radius: 7px; }}
.power-reference {{
  background: #10243A;
  color: #B8C9D9;
  border-radius: 8px;
  padding: 10px 12px;
  font-size: 10px;
}}
.mission-rail {{ background: #F3F6F9; border-left-color: #D6E0E9; padding: 17px 16px; }}
.rail-section, .rail-preparation, .rail-safety {{
  background: #FFFFFF;
  border-color: #D9E2EA;
  border-radius: 10px;
  box-shadow: 0 4px 12px rgba(24,48,70,0.045);
}}
.rail-heading-ko {{ color: #10243A; font-size: 19px; letter-spacing: -0.2px; }}
.rail-section {{
  border-top: 3px solid #2F7CF6;
  padding: 12px;
  box-shadow: 0 7px 18px rgba(24,48,70,0.075);
}}
.rail-section-title {{ color: #10243A; font-size: 14px; }}
.rail-section-count {{
  background: #E8F0FE;
  color: #246BE0;
  border-radius: 999px;
  padding: 3px 9px;
  font-size: 12px;
}}
.rail-device-row {{ min-height: 37px; }}
.rail-device-name {{ color: #3E556B; font-size: 12px; }}
.rail-preparation {{ padding: 12px; }}
.rail-title {{ color: #10243A; font-size: 14px; }}
.rail-description {{ color: #5F7488; font-size: 12px; }}
.video-card {{ border-color: #31465B; border-radius: 9px; box-shadow: 0 10px 26px rgba(3,12,23,0.24); }}
.pip-frame {{ border-color: #607B95; border-radius: 10px; box-shadow: 0 12px 30px rgba(0,12,28,0.34); }}
.display-options {{ background: #F7F9FC; border-color: #DDE5ED; border-radius: 9px; box-shadow: none; }}
.display-options-title {{ color: #52677B; font-size: 10px; }}
.display-option {{ color: #667B8F; font-size: 11px; }}
.event-expander {{
  background: #FAFCFE;
  border-top: 1px solid #D5DFE8;
  padding: 0 16px;
  box-shadow: 0 -4px 14px rgba(23,45,66,0.045);
}}
.event-expander:hover {{ background: #F6F9FC; }}
.event-header {{ min-height: 48px; }}
.event-status-title {{ color: #425A70; font-weight: 800; }}
.event-status-message {{ color: #8394A5; }}
.event-filter {{ color: #485F74; font-weight: 700; }}
.event-filter check {{ background: #FFFFFF; border-color: #9CABB9; border-radius: 4px; }}
.event-filter check:checked {{ background: #347FF0; border-color: #347FF0; }}
.event-column-header {{ background: #EDF2F7; border-bottom-color: #D7E0E9; }}
.event-operation-row {{ background: #FAFCFE; border-bottom-color: #E1E7ED; }}
.event-operation-row:hover {{ background: #F1F6FB; }}
scrollbar slider {{ background: #A8B6C4; border-radius: 999px; min-width: 7px; min-height: 7px; }}

/* Competition palette: one navy family, cyan technology, red only for danger. */
.mission-page {{ background: #091421; padding: 10px 12px 0 12px; }}
.mission-rail {{
  background: #102033;
  border-left: 1px solid #263B50;
  padding: 18px 16px;
}}
.mission-rail .rail-heading-ko {{ color: #F4F8FC; font-size: 20px; }}
.mission-rail .rail-section,
.mission-rail .rail-preparation,
.mission-rail .rail-technology {{
  background: #162A40;
  border: 1px solid #29425A;
  border-radius: 10px;
  padding: 12px;
  box-shadow: none;
}}
.mission-rail .rail-section {{ border-top: 3px solid #42BFF5; }}
.mission-rail .rail-section-title,
.mission-rail .rail-title {{ color: #F3F8FC; }}
.mission-rail .rail-section-count {{
  background: #153F5B;
  color: #7DDAFF;
}}
.mission-rail .rail-device-name {{ color: #D9E5EF; }}
.mission-rail .rail-section-divider {{ background: #29425A; }}
.mission-rail .rail-description {{ color: #9FB2C5; }}
.mission-rail .preparation-status-value.prep-ready {{ color: #69D6A1; }}
.mission-rail .preparation-status-value.prep-connecting {{ color: #72CFFF; }}
.mission-rail .preparation-status-value.prep-attention {{ color: #F2C86A; }}
.mission-rail .preparation-status-value.prep-error {{ color: #FF8089; }}
.mission-rail .preparation-status-value.prep-offline {{ color: #8297AA; }}
.rail-technology {{ padding: 12px; }}
.technology-row {{ min-height: 38px; }}
.technology-marker {{ background: #42BFF5; border-radius: 999px; }}
.technology-vision .technology-marker {{ background: #8B7CF6; }}
.technology-safety .technology-marker {{ background: #49C98A; }}
.technology-arm .technology-marker {{ background: #F2B84B; }}
.technology-name {{ color: #EDF5FB; font-size: 11px; font-weight: 900; }}
.technology-detail {{ color: #8FA6BA; font-size: 9px; }}
.mission-rail .display-options {{
  background: transparent;
  border: 1px solid #29425A;
  box-shadow: none;
}}
.mission-rail .display-options-title,
.mission-rail .display-option {{ color: #8FA6BA; }}
.mission-rail .display-option check {{ background: #102033; border-color: #587087; }}
.mission-rail .rail-data-row {{ background: #162A40; border-color: #29425A; }}
.mission-rail .rail-data-label {{ color: #8298AC; }}
.mission-rail .rail-data-value {{ color: #F4F8FC; }}
.mission-rail .rail-data-target,
.mission-rail .rail-data-distance {{ color: #72D6FF; }}

/* Category accents make the implemented subsystems scannable without a legend. */
.status-summary-card.category-drive {{ border-left-color: #2F7CF6; }}
.status-summary-card.category-power {{ border-left-color: #E3A132; }}
.status-summary-card.category-safety {{ border-left-color: #2FA36B; }}
.status-summary-card.category-camera {{ border-left-color: #26A7D8; }}
.status-summary-card.category-ai {{ border-left-color: #806CE8; }}
.status-summary-card.category-environment {{ border-left-color: #4DB89A; }}
.status-summary-card.category-arm {{ border-left-color: #D96C92; }}
.status-summary-card.category-drive.selected {{ border-left-color: #2F7CF6; }}
.status-summary-card.category-power.selected {{ border-left-color: #E3A132; }}
.status-summary-card.category-safety.selected {{ border-left-color: #2FA36B; }}
.status-summary-card.category-camera.selected {{ border-left-color: #26A7D8; }}
.status-summary-card.category-ai.selected {{ border-left-color: #806CE8; }}
.status-summary-card.category-environment.selected {{ border-left-color: #4DB89A; }}
.status-summary-card.category-arm.selected {{ border-left-color: #D96C92; }}

/* Bright exhibition mode: keep only the camera stage dark. */
window {{ background: #F3F7FB; }}
.topbar {{
  background: #FFFFFF;
  border-bottom: 1px solid #D9E4EE;
  box-shadow: 0 3px 14px rgba(20,48,72,0.06);
}}
.topbar .brand {{ color: #102A43; }}
.topbar .eyebrow {{ color: #70869A; }}
.topbar .status-chip {{ color: #6F8497; }}
.topbar .status-chip.status-live {{ color: #218A63; }}
.topbar .status-chip.status-camera-live {{ color: #247FC1; }}
.nav {{ background: #FFFFFF; border-bottom: 1px solid #D9E4EE; }}
.nav button {{ color: #71869A; }}
.nav button label {{ color: #71869A; }}
.nav button:hover {{ color: #102A43; background: #F2F6FA; }}
.nav button:hover label {{ color: #102A43; }}
.nav button:checked {{
  color: #174EA6;
  background: #EEF5FF;
  border-bottom-color: #2F7CF6;
}}
.nav button:checked label {{ color: #174EA6; }}
.mission-page {{ background: #EDF3F8; }}
.mission-rail {{
  background: #F7FAFD;
  border-left: 1px solid #D5E1EB;
}}
.mission-rail .rail-heading-ko {{ color: #102A43; }}
.mission-rail .rail-section,
.mission-rail .rail-preparation,
.mission-rail .rail-technology {{
  background: #FFFFFF;
  border-color: #D6E2EC;
  box-shadow: 0 5px 16px rgba(25,55,78,0.055);
}}
.mission-rail .rail-section {{ border-top-color: #2F7CF6; }}
.mission-rail .rail-section-title,
.mission-rail .rail-title {{ color: #17324A; }}
.mission-rail .rail-section-count {{ background: #E8F2FF; color: #246BE0; }}
.mission-rail .rail-device-name {{ color: #405A70; }}
.mission-rail .rail-section-divider {{ background: #DCE6EE; }}
.mission-rail .rail-description {{ color: #6C8295; }}
.mission-rail .preparation-status-value.prep-ready {{ color: #218A63; }}
.mission-rail .preparation-status-value.prep-connecting {{ color: #247FC1; }}
.mission-rail .preparation-status-value.prep-attention {{ color: #C98718; }}
.mission-rail .preparation-status-value.prep-error {{ color: #C43B43; }}
.mission-rail .preparation-status-value.prep-offline {{ color: #8799AA; }}
.technology-name {{ color: #213D54; }}
.technology-detail {{ color: #71869A; }}
.mission-rail .rail-data-row {{ background: #FFFFFF; border-color: #D6E2EC; }}
.mission-rail .rail-data-label {{ color: #7A8EA0; }}
.mission-rail .rail-data-value {{ color: #17324A; }}
.mission-rail .rail-data-target,
.mission-rail .rail-data-distance {{ color: #2478D4; }}
.mission-rail .display-options {{
  background: #FFFFFF;
  border-color: #D6E2EC;
  box-shadow: 0 4px 12px rgba(25,55,78,0.045);
}}
.mission-rail .display-options-title {{ color: #405A70; }}
.mission-rail .display-option {{ color: #314B62; }}
.mission-rail .display-option check {{ background: #FFFFFF; border-color: #91A4B5; }}
.mission-rail .display-option check:checked {{ background: #2F7CF6; border-color: #2F7CF6; }}
.event-expander {{ background: #FFFFFF; border-top-color: #D5E1EB; }}
.event-expander:hover {{ background: #F7FAFD; }}

/* Final competition reference override. */
window, .console-shell, .page {{
  background-color: #03050C;
  background-image: linear-gradient(160deg, #050813, #03050C 56%, #04060F);
  color: #EAEEF9;
}}
.side-rail {{ background: #071020; border-right: 1px solid #172039; }}
.topbar {{ background: transparent; border: none; box-shadow: none; padding: 15px 24px 8px 24px; }}
.topbar .brand {{ color: #EAEEF9; font-size: 20px; font-weight: 900; }}
.topbar .eyebrow {{ color: #596580; }}
.topbar .status-chip {{ color: #77829D; }}
.topbar .status-chip.status-live {{ color: #39D8B0; }}
.topbar .status-chip.status-camera-live {{ color: #42D6EB; }}
.global-estop {{ background: #6D1E3E; border: none; border-radius: 7px; box-shadow: 0 0 28px rgba(255,55,104,0.24); padding: 0; }}
.nav {{ background: transparent; border: none; padding: 0 24px 10px 24px; }}
.nav stackswitcher {{ background: rgba(10,14,28,0.58); border: 1px solid rgba(255,255,255,0.08); border-radius: 7px; padding: 3px; }}
.nav button {{ color: #646E96; min-height: 31px; padding: 0 20px; border: none; border-radius: 5px; font-size: 10px; }}
.nav button label {{ color: #7A86A4; }}
.nav button:hover {{ background: #111730; color: #E8ECFF; }}
.nav button:checked {{ color: #FFFFFF; background: #26316C; border: none; box-shadow: 0 0 18px rgba(76,111,255,0.28); }}
.nav button:checked label {{ color: #FFFFFF; }}
.status-summary-card {{ background: rgba(5,9,20,0.54); border: 1px solid rgba(152,162,196,0.15); border-left: 1px solid rgba(152,162,196,0.15); border-radius: 9px; padding: 13px 16px; box-shadow: none; }}
.status-summary-card.category-drive {{ border-radius: 9px; }}
.status-summary-card.category-ai {{ border-radius: 34px 34px 9px 9px; }}
.status-summary-card.category-power {{ border-radius: 44px; }}
.status-summary-card.category-network {{ border-radius: 18px 7px 22px 7px; }}
.status-summary-card.category-environment {{ border-radius: 12px 26px 12px 26px; }}
.status-summary-card.category-arm {{ border-radius: 7px 24px 7px 24px; }}
.status-summary-card.category-safety {{ border-radius: 30px 9px 30px 9px; }}
.status-summary-card label {{ color: #737F9E; }}
.status-summary-card .system-name {{ color: #737F9E; font-size: 10px; font-weight: 900; }}
.status-summary-card:hover {{ background: #0A1020; border-color: #354267; box-shadow: none; }}
.status-summary-card.selected {{ background: rgba(12,17,34,0.92); border: 1px solid rgba(116,137,255,0.65); box-shadow: 0 14px 34px rgba(76,111,255,0.30), inset 0 -12px 22px rgba(76,111,255,0.36); }}
.status-summary-card.selected label {{ color: #F3F5FF; }}
.status-summary-card.selected .system-name {{ color: #F7F8FF; }}
.status-summary-card.category-ai.selected {{ box-shadow: 0 14px 34px rgba(130,72,255,0.34), inset 0 -9px 17px rgba(128,72,255,0.45); }}
.status-summary-card.category-power.selected {{ box-shadow: 0 14px 34px rgba(255,174,55,0.32), inset 0 -9px 17px rgba(255,174,55,0.43); }}
.status-summary-card.category-network.selected {{ box-shadow: 0 14px 34px rgba(47,211,231,0.30), inset 0 -9px 17px rgba(47,211,231,0.42); }}
.status-summary-card.category-environment.selected {{ box-shadow: 0 14px 34px rgba(77,184,154,0.30), inset 0 -9px 17px rgba(77,184,154,0.42); }}
.status-summary-card.category-arm.selected {{ box-shadow: 0 14px 34px rgba(231,69,119,0.30), inset 0 -9px 17px rgba(231,69,119,0.42); }}
.status-summary-card.category-safety.selected {{ box-shadow: 0 14px 34px rgba(41,213,168,0.30), inset 0 -9px 17px rgba(41,213,168,0.42); }}
.status-summary-card.category-drive:not(.selected),
.status-summary-card.category-ai:not(.selected),
.status-summary-card.category-power:not(.selected),
.status-summary-card.category-network:not(.selected),
.status-summary-card.category-environment:not(.selected),
.status-summary-card.category-arm:not(.selected),
.status-summary-card.category-safety:not(.selected) {{ border-color: #171D2D; }}
.section-title {{ color: #737F9E; font-size: 11px; }}
.status-panel {{ background: #060A15; border: 1px solid #192132; border-radius: 9px; padding: 24px 28px; box-shadow: none; }}
.status-panel-title {{ color: #F2F4FF; font-size: 17px; }}
.status-panel-values {{ color: #76829F; }}
.status-panel-updated {{ color: #4D5874; }}
.status-panel drawingarea {{ background: #050A15; }}
.status-dot {{ border-radius: 999px; min-width: 8px; min-height: 8px; }}
.status-dot.status-live {{ background: #32D3A5; }}
.status-dot.status-warn {{ background: #F2A92F; }}
.status-dot.status-bad {{ background: #E34B72; }}
.status-dot.status-muted {{ background: #52607A; }}
.power-metric, .detail-metric {{ background: transparent; border: none; border-right: 1px solid #182033; border-radius: 0; padding: 8px 16px; }}
.power-metric-title, .detail-metric-title {{ color: #586582; }}
.power-metric-value, .detail-metric-value {{ color: #F1F3FC; font-size: 18px; }}
.mission-page {{ background: transparent; }}
.mission-page .video-card {{ background: #02040A; border: none; border-radius: 0; box-shadow: none; }}
.mission-page .pip-frame .video-card {{ background: rgba(7,12,24,0.90); border: 1px solid rgba(255,255,255,0.13); border-radius: 7px; box-shadow: 0 18px 44px rgba(0,0,0,0.42); }}
.mission-page .main-camera-label {{ background: rgba(5,8,18,0.52); border: 1px solid rgba(255,255,255,0.08); }}
.mission-rail {{ background: rgba(10,15,29,0.72); border: 1px solid rgba(255,255,255,0.11); border-radius: 9px; margin: 0 0 8px 0; box-shadow: 0 24px 54px rgba(0,0,0,0.42), inset 0 -10px 24px rgba(46,92,225,0.08); }}
.mission-rail .rail-heading-ko, .mission-rail .rail-section-title, .mission-rail .rail-title {{ color: #EDEFFF; }}
.mission-rail .rail-section, .mission-rail .rail-preparation, .mission-rail .rail-technology, .mission-rail .rail-data-row, .mission-rail .display-options {{ background: rgba(255,255,255,0.025); border-color: rgba(255,255,255,0.08); box-shadow: none; }}
.mission-rail .rail-device-name, .mission-rail .rail-description, .technology-name, .mission-rail .display-option {{ color: #7D89A5; }}
.event-expander {{ background: #070C17; border: 1px solid #20283A; }}
.event-expander:hover {{ background: #0A1120; }}
.soc-progress {{ min-height: 24px; margin: 28px 160px 16px 160px; color: #F4F6FF; }}
.soc-progress trough {{ background: #151B2A; border-radius: 999px; min-height: 18px; }}
.soc-progress progress {{ background: linear-gradient(to right, #FF9F43, #F252A0, #735BFF, #34CEE3); border-radius: 999px; }}
.soc-hero {{ color: #F2F4FF; font-size: 62px; font-weight: 900; margin: 70px 150px 0 150px; }}
.arm-setup {{ background: #090E1B; border: 1px solid #1D2639; border-radius: 8px; padding: 14px; }}
.competition-status {{ background: transparent; padding: 0 24px 0 24px; }}
.competition-pills {{ margin: 0 0 14px 0; }}
.competition-status-page {{
  background: rgba(255,255,255,0.038);
  border: 1px solid rgba(255,255,255,0.10);
  border-radius: 9px;
  box-shadow: 0 30px 72px rgba(0,0,0,0.44), inset 0 -12px 28px rgba(46,92,225,0.06);
}}
.competition-metric {{
  background: transparent;
  border-right: 1px solid #182033;
  padding: 26px 36px 22px 36px;
}}
.competition-metric-label {{
  color: #596682;
  font-size: 8px;
  font-weight: 700;
}}
.competition-metric-value {{
  color: #F0F2FC;
  font-size: 15px;
  font-weight: 900;
  font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace;
}}
.competition-status-page drawingarea {{ background: #050914; }}

/* Cohesive control-room palette: deep navy surfaces, blue interaction,
 * semantic colours reserved for live/warning/danger states. */
window, .console-shell, .page {{
  background-color: #060B14;
  background-image: linear-gradient(160deg, #09111E, #060B14 58%, #07101B);
  color: #E7EEF7;
}}
.side-rail {{ background: #08111E; border-right-color: #1B2A3D; }}
.rail-logo {{ background: #3978E6; box-shadow: 0 6px 18px rgba(57,120,230,0.24); }}
.rail-icon, .rail-icon label {{ color: #667A91; }}
.rail-icon:hover {{ background: #101D2D; color: #C4D7EB; }}
.rail-icon:hover label {{ color: #C4D7EB; }}
.rail-icon.active {{ background: #132742; color: #9DC8FF; border-color: #315F91; }}
.rail-icon.active label {{ color: #9DC8FF; }}
.topbar {{ background: rgba(7,14,25,0.96); border-bottom: 1px solid #16263A; }}
.topbar .brand {{ color: #EDF3FA; }}
.topbar .eyebrow, .topbar .status-chip {{ color: #75899F; }}
.topbar .status-chip.status-live {{ color: #55C995; }}
.topbar .status-chip.status-camera-live {{ color: #58BFE5; }}
.nav {{ background: rgba(7,14,25,0.96); border-bottom: 1px solid #16263A; }}
.nav stackswitcher {{ background: #0C1624; border-color: #203149; }}
.nav button, .nav button label {{ color: #7F92A8; }}
.nav button:hover {{ background: #122035; color: #DDE8F3; }}
.nav button:hover label {{ color: #DDE8F3; }}
.nav button:checked {{
  background: #183052;
  color: #F4F8FC;
  border: 1px solid #315F91;
  box-shadow: 0 5px 16px rgba(24,80,145,0.22);
}}
.nav button:checked label {{ color: #F4F8FC; }}
.global-estop {{
  background-color: #A82F3C;
  background-image: linear-gradient(145deg, #D85B64, #B63A47 52%, #8F2633);
  border: 1px solid #EE7A82;
  border-bottom-color: #6F1823;
  border-radius: 12px;
  box-shadow: 0 7px 0 #641722, 0 12px 24px rgba(137,29,42,0.28), inset 0 1px rgba(255,255,255,0.24);
  padding: 5px 12px;
}}
.global-estop:hover {{
  background-image: linear-gradient(145deg, #E96A73, #C84652 52%, #9E2E3A);
  border-color: #FF9AA1;
  box-shadow: 0 7px 0 #641722, 0 14px 28px rgba(164,36,51,0.34), inset 0 1px rgba(255,255,255,0.28);
}}
.global-estop:active {{
  background-image: linear-gradient(145deg, #9A2935, #B33542);
  border-color: #D75A65;
  box-shadow: 0 2px 0 #641722, inset 0 3px 8px rgba(76,10,20,0.34);
}}
.global-estop:disabled {{
  background-image: linear-gradient(145deg, #8C4E55, #71373E);
  border-color: #9A6268;
  box-shadow: 0 4px 0 #4C292D;
  opacity: 0.58;
}}
.estop-symbol {{
  background: #7D1E29;
  color: #FFFFFF;
  border: 1px solid rgba(255,255,255,0.38);
  border-radius: 999px;
  min-width: 24px;
  min-height: 24px;
  font-size: 15px;
  font-weight: 900;
}}
.estop-title {{ color: #FFFFFF; font-size: 12px; font-weight: 900; }}
.estop-subtitle {{ color: rgba(255,255,255,0.72); font-size: 7px; font-weight: 800; letter-spacing: 1.4px; }}

.status-readiness, .status-summary-card, .status-panel,
.competition-status-page, .mission-rail {{
  background: #0B1422;
  border-color: #203047;
  box-shadow: 0 12px 28px rgba(0,0,0,0.18);
}}
.status-readiness {{ background: #0E1A2B; }}
.status-overall, .status-panel-title, .status-summary-card.selected label,
.status-summary-card.selected .system-name {{ color: #EDF3FA; }}
.status-summary-card {{
  border-left: 3px solid #2B405A;
  border-radius: 10px;
}}
.status-summary-card.category-drive,
.status-summary-card.category-ai,
.status-summary-card.category-power,
.status-summary-card.category-network,
.status-summary-card.category-environment,
.status-summary-card.category-arm,
.status-summary-card.category-safety {{ border-radius: 10px; }}
.status-summary-card.category-drive:not(.selected),
.status-summary-card.category-ai:not(.selected),
.status-summary-card.category-power:not(.selected),
.status-summary-card.category-network:not(.selected),
.status-summary-card.category-environment:not(.selected),
.status-summary-card.category-arm:not(.selected),
.status-summary-card.category-safety:not(.selected) {{ border-color: #203047; border-left-color: #2B405A; }}
.status-summary-card.category-drive:not(.selected) {{
  background: #0C1727;
  border-left-color: #4B8BEA;
}}
.status-summary-card.category-ai:not(.selected) {{
  background: #151429;
  border-left-color: #8A6FF0;
}}
.status-summary-card.category-power:not(.selected) {{
  background: #201A12;
  border-left-color: #E1A43B;
}}
.status-summary-card.category-network:not(.selected) {{
  background: #0C1C25;
  border-left-color: #48BCD0;
}}
.status-summary-card.category-environment:not(.selected) {{
  background: #0D1F1C;
  border-left-color: #4DB89A;
}}
.status-summary-card.category-arm:not(.selected) {{
  background: #20131D;
  border-left-color: #D96C92;
}}
.status-summary-card.category-safety:not(.selected) {{
  background: #0D1C18;
  border-left-color: #4ABC83;
}}
.status-summary-card.category-drive:not(.selected) .system-name {{ color: #78A9EF; }}
.status-summary-card.category-ai:not(.selected) .system-name {{ color: #A08CF0; }}
.status-summary-card.category-power:not(.selected) .system-name {{ color: #D8AB59; }}
.status-summary-card.category-network:not(.selected) .system-name {{ color: #66BAC9; }}
.status-summary-card.category-environment:not(.selected) .system-name {{ color: #72CDB2; }}
.status-summary-card.category-arm:not(.selected) .system-name {{ color: #D985A4; }}
.status-summary-card.category-safety:not(.selected) .system-name {{ color: #6BC397; }}
.status-summary-card label, .status-summary-card .system-name {{ color: #8396AA; }}
.status-summary-card:hover {{ background: #0F1C2C; border-color: #30465F; }}
.status-summary-card.selected,
.status-summary-card.category-drive.selected,
.status-summary-card.category-ai.selected,
.status-summary-card.category-power.selected,
.status-summary-card.category-network.selected,
.status-summary-card.category-environment.selected,
.status-summary-card.category-arm.selected,
.status-summary-card.category-safety.selected {{
  background: #142642;
  border: 1px solid #4B86D9;
  border-left: 3px solid #5B9AF2;
  box-shadow: 0 8px 22px rgba(34,92,165,0.24);
}}
.status-summary-card.category-drive.selected {{
  background-color: #142642;
  background-image: linear-gradient(135deg, rgba(61,126,232,0.42), rgba(15,29,48,0.94) 72%);
  border-color: #5B9AF2;
  box-shadow: 0 10px 28px rgba(48,111,211,0.28), inset 0 -10px 22px rgba(61,126,232,0.14);
}}
.status-summary-card.category-ai.selected {{
  background-color: #211B42;
  background-image: linear-gradient(135deg, rgba(125,92,229,0.44), rgba(18,24,45,0.95) 72%);
  border-color: #957AF0;
  border-left-color: #A086F5;
  box-shadow: 0 10px 28px rgba(116,82,216,0.27), inset 0 -10px 22px rgba(125,92,229,0.14);
}}
.status-summary-card.category-power.selected {{
  background-color: #2A251A;
  background-image: linear-gradient(135deg, rgba(220,159,53,0.36), rgba(20,29,43,0.95) 72%);
  border-color: #D8A646;
  border-left-color: #E8B65A;
  box-shadow: 0 10px 28px rgba(205,145,41,0.22), inset 0 -10px 22px rgba(220,159,53,0.12);
}}
.status-summary-card.category-network.selected {{
  background-color: #102A36;
  background-image: linear-gradient(135deg, rgba(65,171,198,0.36), rgba(15,29,48,0.94) 72%);
  border-color: #50AFC8;
  border-left-color: #62C5D7;
  box-shadow: 0 10px 28px rgba(47,153,180,0.23), inset 0 -10px 22px rgba(65,171,198,0.12);
}}
.status-summary-card.category-environment.selected {{
  background-color: #123028;
  background-image: linear-gradient(135deg, rgba(67,172,140,0.40), rgba(15,29,48,0.94) 72%);
  border-color: #58C5A4;
  border-left-color: #6BD4B3;
  box-shadow: 0 10px 28px rgba(49,157,125,0.23), inset 0 -10px 22px rgba(67,172,140,0.13);
}}
.status-summary-card.category-arm.selected {{
  background-color: #321B2B;
  background-image: linear-gradient(135deg, rgba(207,87,132,0.40), rgba(28,24,42,0.95) 72%);
  border-color: #DB769A;
  border-left-color: #E184A6;
  box-shadow: 0 10px 28px rgba(190,72,116,0.25), inset 0 -10px 22px rgba(207,87,132,0.13);
}}
.status-summary-card.category-safety.selected {{
  background-color: #102A26;
  background-image: linear-gradient(135deg, rgba(62,177,128,0.36), rgba(15,29,48,0.94) 72%);
  border-color: #55C995;
  border-left-color: #68D09C;
  box-shadow: 0 10px 28px rgba(48,157,109,0.22), inset 0 -10px 22px rgba(62,177,128,0.12);
}}
.section-title, .status-panel-values {{ color: #8194AA; }}
.status-panel-updated, .power-metric-title, .detail-metric-title,
.competition-metric-label {{ color: #667B92; }}
.power-metric-value, .detail-metric-value, .competition-metric-value {{ color: #E8EFF7; }}
.power-metric, .detail-metric {{ border-right-color: #213148; }}
.power-state-summary, .power-reference {{ background: #101D2E; color: #9CB0C3; border-left-color: #4B88E6; }}
.sensor-tile {{
  background: #0B1625;
  border: 1px solid #24374D;
  border-top: 4px solid #4F83B8;
  border-radius: 12px;
  box-shadow: 0 10px 24px rgba(0,0,0,0.20);
}}
.sensor-tile-climate {{ border-top-color: #4F9BEF; }}
.sensor-tile-air {{ border-top-color: #947AEF; }}
.sensor-tile-gas {{ border-top-color: #E8A333; }}
.sensor-tile-flame {{ border-top-color: #4FC490; }}
.sensor-tile-name {{ color: #B7C8D8; font-size: 12px; font-weight: 800; }}
.sensor-tile-value {{ color: #F2F7FC; font-size: 22px; font-weight: 900; }}
.sensor-tile-note {{ color: #71879C; font-size: 9px; }}
.sensor-status {{ border-radius: 999px; padding: 3px 9px; font-size: 11px; font-weight: 900; }}
.sensor-status.status-live {{ color: #8BE3B6; background: #123D2D; }}
.sensor-status.status-warn {{ color: #FFD77A; background: #493514; }}
.sensor-status.status-bad {{ color: #FF9BA8; background: #4A1823; }}
.sensor-status.status-muted {{ color: #A8B6C4; background: #263442; }}
.sensor-overall-block {{ border-radius: 8px; padding: 3px 14px; border: 1px solid #2A4057; }}
.sensor-overall-block.status-live {{ background: #103526; border-color: #277A59; }}
.sensor-overall-block.status-warn {{ background: #3A2A12; border-color: #936A21; }}
.sensor-overall-block.status-bad {{ background: #421823; border-color: #A33D54; }}
.sensor-overall-block.status-muted {{ background: #202D3A; border-color: #536579; }}
.sensor-overall-count {{ font-size: 16px; font-weight: 900; }}
.sensor-overall-caption {{ font-size: 9px; font-weight: 800; }}
.sensor-overall-count.status-live, .sensor-overall-caption.status-live {{ color: #8BE3B6; }}
.sensor-overall-count.status-warn, .sensor-overall-caption.status-warn {{ color: #FFD77A; }}
.sensor-overall-count.status-bad, .sensor-overall-caption.status-bad {{ color: #FF9BA8; }}
.sensor-overall-count.status-muted, .sensor-overall-caption.status-muted {{ color: #A8B6C4; }}
.sensor-group-chip {{
  color: #8FA6BA;
  background: #152538;
  border-radius: 10px;
  padding: 2px 7px;
  font-size: 8px;
  font-weight: 800;
}}
.sensor-group-climate {{ color: #75B5F6; background: #112A44; }}
.sensor-group-air {{ color: #B29FF6; background: #251E48; }}
.sensor-group-gas {{ color: #F0BB64; background: #342513; }}
.sensor-group-flame {{ color: #75D5AF; background: #12342A; }}
.sensor-tile.sensor-flame-normal {{
  background: #0D211D;
  border-color: #2D765D;
  border-top-color: #4FC490;
}}
.sensor-tile.sensor-flame-detected {{
  background: #2A1018;
  border-color: #9E354F;
  border-top-color: #E44968;
}}
.status-view-option {{ background: #0E1928; color: #8296AB; border-color: #26384E; }}
.status-view-option:hover {{ background: #14243A; color: #D8E5F1; }}
.status-view-option:checked {{ background: #285DA5; color: #FFFFFF; border-color: #4B88E6; }}
.status-view-option:focus {{ border-color: #4B88E6; box-shadow: inset 0 0 0 1px #4B88E6; }}
.status-summary-value {{ color: #879AAF; }}
.status-summary-value.status-live {{ color: #55C995; }}
.status-summary-value.status-progress {{ color: #69A7F5; }}
.status-summary-value.status-warn {{ color: #E7B34F; }}
.status-summary-value.status-bad {{ color: #E56A74; }}

.mission-page {{ background: transparent; }}
.mission-rail {{ background: #0D1828; border-color: #22344C; }}
.mission-rail .rail-section,
.mission-rail .rail-preparation,
.mission-rail .rail-technology,
.mission-rail .rail-data-row,
.mission-rail .display-options {{ background: #111F31; border-color: #263A52; }}
.mission-rail .display-options {{
  background-color: #0E1D2C;
  background-image: linear-gradient(135deg, rgba(75,139,234,0.12), rgba(85,185,222,0.04));
  border-color: #2A4963;
}}
.mission-rail .rail-section {{ border-top-color: #4B8BEA; }}
.mission-rail .rail-heading-ko,
.mission-rail .rail-section-title,
.mission-rail .rail-title,
.mission-rail .rail-data-value {{ color: #E8F0F8; }}
.mission-rail .display-options-title {{ color: #70C7F2; }}
.mission-rail .rail-device-name,
.mission-rail .rail-description,
.technology-name {{ color: #8CA0B5; }}
.mission-rail .display-option,
.mission-rail .display-option label {{ color: #CFDCE8; }}
.mission-rail .rail-section-count {{ background: #173353; color: #76B7F6; }}
.mission-rail .rail-section-divider {{ background: #263A52; }}
.mission-rail .rail-data-label {{ color: #71869C; }}
.mission-rail .rail-data-target,
.mission-rail .rail-data-distance {{ color: #62BDE8; }}
.technology-marker {{ background: #4B8BEA; }}
.technology-vision .technology-marker {{ background: #55B9DE; }}
.technology-safety .technology-marker {{ background: #55C995; }}
.technology-arm .technology-marker {{ background: #7397E8; }}
.display-option check, .event-filter check {{ background: #0C1725; border-color: #526B84; }}
.display-option check:checked, .event-filter check:checked {{ background: #3978D6; border-color: #5795E8; }}
.mission-rail .display-option check {{ background: #091725; border-color: #66849E; }}
.mission-rail .display-option check:checked {{ background: #4B8BEA; border-color: #7AB4F3; color: #FFFFFF; }}
.rail-tool-selector button {{ background: #13243A; color: #DDE8F3; border-color: #31506D; }}
.end-effector-summary {{
  background-color: #0E1D2C;
  background-image: linear-gradient(135deg, rgba(75,139,234,0.14), rgba(77,184,154,0.07));
  border: 1px solid #2A4963;
  border-left: 3px solid #4DB89A;
  border-radius: 9px;
  padding: 10px;
}}
.end-effector-summary-title {{ color: #E8F0F8; font-size: 15px; font-weight: 900; }}
.end-effector-summary-state {{
  color: #A8B6C4;
  background: #263442;
  border-radius: 999px;
  padding: 2px 7px;
  font-size: 9px;
  font-weight: 900;
}}
.end-effector-summary-state.status-live {{ color: #8BE3B6; background: #123D2D; }}
.end-effector-summary-state.status-warn {{ color: #FFD77A; background: #493514; }}
.end-effector-summary-state.status-muted {{ color: #A8B6C4; background: #263442; }}
.end-effector-summary-purpose {{ color: #AFC0D0; font-size: 11px; }}
.end-effector-summary-reading {{ color: #72CDB2; font-size: 13px; font-weight: 900; }}
.end-effector-arm-label {{ color: #71869C; font-size: 9px; font-weight: 800; }}
.end-effector-arm-value {{ color: #E8F0F8; font-size: 11px; font-weight: 900; }}
.rail-data-speed {{ color: #8BE3B6; }}
.rail-tool-selector button {{ min-height: 34px; font-size: 12px; font-weight: 800; }}
.end-effector-detail-button {{
  background: #173353;
  color: #CDE5FF;
  border: 1px solid #315F91;
  border-radius: 7px;
  min-height: 28px;
  font-size: 10px;
  font-weight: 900;
}}
.end-effector-detail-button label {{ color: #CDE5FF; }}
.end-effector-detail-button:hover {{ background: #214A75; color: #FFFFFF; }}
.end-effector-detail-button:hover label {{ color: #FFFFFF; }}
window.end-effector-popup {{ background: #07101B; }}
.end-effector-popup-shell {{ background: #07101B; padding: 18px; }}
.end-effector-popup-heading {{ color: #F1F6FB; font-size: 22px; font-weight: 900; }}
.end-effector-popup-purpose {{ color: #9EB2C5; font-size: 13px; }}
.end-effector-popup-card {{
  background: #0D1B2B;
  border: 1px solid #263B52;
  border-left: 3px solid #4DB89A;
  border-radius: 10px;
  padding: 18px;
}}
.end-effector-popup-label {{ color: #71879C; font-size: 10px; font-weight: 800; }}
.end-effector-popup-value {{ color: #E8F0F8; font-size: 16px; font-weight: 900; }}
.environment-title {{ color: #F1F6FB; font-size: 22px; font-weight: 900; }}
.environment-subtitle {{ color: #9EB2C5; font-size: 12px; }}
.environment-connection {{ border-radius: 999px; padding: 6px 11px; font-size: 11px; font-weight: 900; }}
.environment-connection.status-live {{ color: #8BE3B6; background: #123D2D; }}
.environment-connection.status-warn {{ color: #FFD77A; background: #493514; }}
.environment-connection.status-muted {{ color: #A8B6C4; background: #263442; }}
.environment-summary {{ color: #DFE9F2; background: #0D1B2B; border: 1px solid #263B52; border-radius: 8px; padding: 8px 11px; font-size: 12px; font-weight: 800; }}
.sensor-group {{ background: #0A1725; border: 1px solid #243A51; border-radius: 10px; padding: 10px; }}
.sensor-group-panel-climate {{ border-top: 3px solid #4F9BEF; }}
.sensor-group-panel-air {{ border-top: 3px solid #947AEF; }}
.sensor-group-panel-hazard {{ border-top: 3px solid #E8A333; }}
.sensor-group-title {{ color: #F0F5FA; font-size: 16px; font-weight: 900; }}
.sensor-group-description {{ color: #8499AD; font-size: 10px; }}
.sensor-compact-tile {{ padding: 7px 9px; border-top-width: 1px; }}
.sensor-compact-tile .sensor-tile-value {{ font-size: 20px; }}
.sensor-diagnostic-note {{ color: #9EB1C2; font-size: 10px; }}
.system-status-dashboard {{ background: #07101B; }}
.system-status-title {{ color: #F2F7FC; font-size: 25px; font-weight: 900; }}
.system-status-subtitle {{ color: #94A9BC; font-size: 12px; }}
.system-overall-chip {{ border-radius: 999px; padding: 7px 12px; font-size: 11px; font-weight: 900; }}
.system-overall-chip.status-live {{ color: #8BE3B6; background: #123D2D; }}
.system-overall-chip.status-warn {{ color: #FFD77A; background: #493514; }}
.system-overall-chip.status-muted {{ color: #A8B6C4; background: #263442; }}
.system-section {{ background: #0C1928; border: 1px solid #263B52; border-top: 3px solid #4B8BEA; border-radius: 11px; padding: 16px; }}
.system-section-power {{ border-top-color: #D9A64B; }}
.system-section-communication {{ border-top-color: #55B9DE; }}
.system-section-safety {{ border-top-color: #55C995; }}
.system-section-title {{ color: #F0F5FA; font-size: 18px; font-weight: 900; }}
.system-section-description {{ color: #859BAF; font-size: 10px; }}
.system-section-badge {{ border-radius: 999px; padding: 4px 9px; font-size: 10px; font-weight: 900; }}
.system-section-badge.status-live {{ color: #8BE3B6; background: #123D2D; }}
.system-section-badge.status-warn {{ color: #FFD77A; background: #493514; }}
.system-section-badge.status-bad {{ color: #FF9BA8; background: #4A1823; }}
.system-section-badge.status-muted {{ color: #A8B6C4; background: #263442; }}
.system-metric {{ background: #111F31; border: 1px solid #263A52; border-radius: 7px; padding: 9px 10px; }}
.system-metric-label {{ color: #71879C; font-size: 9px; font-weight: 800; }}
.system-metric-value {{ color: #E8F0F8; font-size: 13px; font-weight: 900; }}
.system-diagnostic-note {{ color: #D8B668; font-size: 10px; }}
menu, menuitem {{ background: #101C2B; color: #DCE7F2; border-color: #2A4059; }}
menuitem label {{ color: #DCE7F2; }}
menuitem:hover, menuitem:active {{ background: #193454; color: #FFFFFF; }}
menuitem:hover label, menuitem:active label {{ color: #FFFFFF; }}

.event-expander, .event-drawer-panel, .event-card,
.event-card text, .event-operation-list, .event-operation-row {{
  background: #111F31;
  color: #D7E2EC;
  border-color: #30465F;
}}
.event-expander {{ background: #14243A; border-color: #36516D; }}
.event-expander:hover, .event-operation-row:hover {{ background: #192B40; }}
.event-column-header {{ background: #1A2B3F; border-bottom-color: #3A5068; }}
.event-column-title, .event-operation-time, .event-operation-technical,
.event-operation-arrow, .event-operation-separator {{ color: #A1B3C4; }}
.event-column-title {{ color: #B8C7D5; font-weight: 800; }}
.event-operation-time {{ color: #9DB2C5; }}
.event-operation-message {{ color: #F1F5F9; }}
.event-operation-module {{ color: #8DC4EE; }}
.event-operation-technical {{ color: #B4C4D3; }}
.event-operation-level.event-level-error {{ color: #FF7A84; }}
.event-operation-level.event-level-warning {{ color: #F3C25E; }}
.event-operation-level.event-level-info {{ color: #72B7FF; }}
.event-operation-raw, .developer-box {{ background: #18283B; color: #B7C7D5; border-color: #344A61; }}
.developer-box label {{ color: #AABBCB; }}
.event-status-title {{ color: #F0F5FA; }}
.event-status-message {{ color: #B7C6D5; }}
.event-collapse, .developer-toggle {{ color: #A3B5C6; }}
.event-filter {{ color: #C4D2DF; }}

.soc-progress trough {{ background: #182538; }}
.soc-progress progress {{ background: linear-gradient(to right, #356FD0, #4B8BEA, #56B8DA); }}
.arm-setup {{ background: #0D1827; border-color: #23364D; }}
.competition-status-page drawingarea, .status-panel drawingarea {{ background: #091321; }}
.competition-standby {{
  background-color: #0A1422;
  background-image: linear-gradient(145deg, rgba(52,108,180,0.12), rgba(7,15,27,0.18) 58%, rgba(75,59,145,0.08));
  border: 1px solid #22364D;
  border-radius: 12px;
  padding: 28px 34px;
  box-shadow: 0 22px 52px rgba(0,0,0,0.28), inset 0 1px rgba(255,255,255,0.035);
}}
.standby-hero {{
  background: #0D1A2A;
  border: 1px solid #263D56;
  border-radius: 11px;
  padding: 28px 34px;
  box-shadow: inset 0 -18px 36px rgba(30,76,130,0.10);
}}
.competition-standby .standby-kicker {{
  color: #70C7F2;
  font-size: 10px;
  font-weight: 900;
  letter-spacing: 1.6px;
}}
.competition-standby .standby-spinner {{ color: #70C7F2; }}
.competition-standby .standby-title {{ color: #F1F6FB; font-size: 27px; font-weight: 900; }}
.competition-standby .standby-description {{ color: #A9BDCF; font-size: 13px; }}
.competition-standby .standby-note {{ color: #71879B; font-size: 10px; }}
.standby-source-card {{
  background: #0D1B2B;
  border: 1px solid #263B52;
  border-top: 2px solid #4B8BEA;
  border-radius: 9px;
  padding: 14px 16px;
  box-shadow: 0 8px 20px rgba(0,0,0,0.16);
}}
.standby-source-card.standby-source-power {{ border-top-color: #D9A64B; }}
.standby-source-card.standby-source-arm {{ border-top-color: #D8799B; }}
.standby-source-card.standby-source-ai {{ border-top-color: #8C7BEA; }}
.competition-standby .standby-source-name {{ color: #E5EDF5; font-size: 12px; font-weight: 900; }}
.competition-standby .standby-source-endpoint {{
  color: #7290AA;
  font-size: 10px;
  font-family: "JetBrains Mono", "DejaVu Sans Mono", monospace;
}}
.competition-standby .standby-source-state {{ color: #91A5B8; font-size: 10px; }}
.standby-source-dot {{ background: #52708B; border-radius: 999px; }}
.progress-hud {{ background: #0E1B2B; border-color: #263B52; box-shadow: 0 8px 20px rgba(0,0,0,0.18); }}
.progress-connector {{ background: #344A61; }}
.progress-connector.active, .progress-connector.completed {{ background: #4B8BEA; }}
.progress-marker.active {{ background: #4B8BEA; border-color: #7AB4F3; }}
.progress-marker.completed {{ color: #55C995; border-color: #55C995; }}
scrollbar slider {{ background: #344A61; }}
""".format(**token)
    return css.encode("utf-8")


def _install_console_css() -> None:
    provider = Gtk.CssProvider()
    provider.load_from_data(CONSOLE_CSS + _theme_css())
    screen = Gdk.Screen.get_default()
    if screen is not None:
        Gtk.StyleContext.add_provider_for_screen(
            screen, provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION,
        )


def _style(widget: Gtk.Widget, *classes: str) -> Gtk.Widget:
    context = widget.get_style_context()
    for css_class in classes:
        context.add_class(css_class)
    return widget


def fit_overlay_transform(
    display_width: float, display_height: float,
    frame_width: float, frame_height: float,
) -> tuple[float, float, float]:
    """Return aspect-preserving scale and letterbox offsets."""
    scale = min(display_width / frame_width, display_height / frame_height)
    return (
        scale,
        (display_width - frame_width * scale) / 2.0,
        (display_height - frame_height * scale) / 2.0,
    )


def overlay_size_matches(
    metadata_width: int, metadata_height: int,
    video_width: int | None, video_height: int | None,
) -> bool:
    """metadata 좌표를 영상 위에 겹쳐도 되는지.

    송신부는 frame_width/height 를 파라미터 기본값으로 싣기 때문에 인식 해상도가
    바뀌면 bbox 가 조용히 어긋난다.  영상 해상도를 알 수 없으면(협상 전) 허용한다.
    """
    if video_width is None or video_height is None:
        return True
    return (metadata_width, metadata_height) == (video_width, video_height)


class MetadataCanvas(Gtk.DrawingArea):
    """Transparent D435 overlay drawn from latest-only UDP metadata."""
    def __init__(
        self, receiver: LatestMetadataReceiver,
        *, source_camera_id: str = "work",
        target_tracker: DisplayTargetTracker | None = None,
    ) -> None:
        super().__init__()
        self._receiver = receiver
        self._source_camera_id = source_camera_id
        self._target_tracker = target_tracker or DisplayTargetTracker()
        self._show_objects = True
        self._show_distance = True
        self._video_width: int | None = None
        self._video_height: int | None = None
        self.connect("draw", self._on_draw)

    def set_video_size(self, width: int, height: int) -> None:
        size = (int(width), int(height))
        if size == (self._video_width, self._video_height):
            return
        self._video_width, self._video_height = size
        self.queue_draw()

    def set_display_options(
        self, *, show_objects: bool, show_distance: bool,
    ) -> None:
        """Change drawing only; metadata reception and target logic continue."""
        self._show_objects = bool(show_objects)
        self._show_distance = bool(show_distance)
        self.queue_draw()

    def _on_draw(self, _widget: Gtk.DrawingArea, context: object) -> bool:
        frame = self._receiver.latest()
        if (
            frame is None
            or frame.source_camera_id != self._source_camera_id
            or time.monotonic() - frame.received_monotonic_s
            > OVERLAY_STALE_AFTER_S
        ):
            return False
        if not overlay_size_matches(
            frame.width, frame.height,
            self._video_width, self._video_height,
        ):
            return False
        allocation = self.get_allocation()
        scale, offset_x, offset_y = fit_overlay_transform(
            allocation.width, allocation.height, frame.width, frame.height,
        )
        context.set_source_rgba(0.0, 0.835, 1.0, 0.98)
        context.set_line_width(2.0)
        context.select_font_face("Sans", 0, 1)
        context.set_font_size(15.0)
        # 갱신 소유자는 _sync_overlay_rail 타이머 하나다.  draw 는 읽기만 한다 —
        # 그리기 횟수가 필터(EMA/median/연속성) 진행을 바꾸면 창이 가려졌을 때
        # 거리값이 달라진다.
        target_view = self._target_tracker.view()
        detections = list(displayable_detections(frame))
        if (
            target_view.held
            and target_view.detection is not None
            and target_view.detection not in detections
        ):
            detections.append(target_view.detection)
        for detection in detections:
            x, y, width, height = detection.bbox_xywh
            if self._show_objects:
                context.set_source_rgba(0.0, 0.835, 1.0, 0.98)
                context.set_line_width(4.0 if detection.is_pick_target else 2.0)
                context.rectangle(offset_x + x * scale, offset_y + y * scale,
                                  width * scale, height * scale)
                context.stroke()
            labels = []
            if self._show_objects:
                labels.append(
                    f"{detection.class_name}  {detection.confidence:.0%}"
                )
            if (
                self._show_distance
                and detection is target_view.detection
                and target_view.distance_m is not None
            ):
                labels.append(f"{target_view.distance_m:.2f} m")
            if not labels:
                continue
            text = "  ".join(labels)
            label_x = offset_x + x * scale
            label_y = max(18.0, offset_y + y * scale - 5.0)
            extents = context.text_extents(text)
            context.set_source_rgba(0.0, 0.059, 0.094, 0.88)
            context.rectangle(
                label_x - 3.0, label_y - extents.height - 4.0,
                extents.width + 9.0, extents.height + 7.0,
            )
            context.fill()
            if detection.is_pick_target:
                context.set_source_rgba(0.0, 0.341, 1.0, 1.0)
                context.rectangle(
                    label_x - 3.0, label_y - extents.height - 4.0,
                    3.0, extents.height + 7.0,
                )
                context.fill()
            context.set_source_rgba(0.42, 0.914, 1.0, 1.0)
            context.move_to(label_x + 2.0, label_y)
            context.show_text(text)
        return False


class ScanRing(Gtk.DrawingArea):
    """Static, restrained standby reticle for the camera empty state."""

    def __init__(self) -> None:
        super().__init__()
        self.set_size_request(86, 86)
        self.connect("draw", self._draw)

    def _draw(self, _widget: Gtk.DrawingArea, context: object) -> bool:
        allocation = self.get_allocation()
        cx, cy = allocation.width / 2.0, allocation.height / 2.0
        radius = min(allocation.width, allocation.height) * 0.38
        context.set_line_width(1.5)
        context.set_source_rgba(0.16, 0.83, 1.0, 0.20)
        context.arc(cx, cy, radius, 0.0, math.tau)
        context.stroke()
        context.set_line_width(2.0)
        context.set_source_rgba(0.16, 0.83, 1.0, 0.72)
        context.arc(cx, cy, radius, -0.55, 0.60)
        context.stroke()
        for offset in (-1, 1):
            context.move_to(cx + offset * radius * 0.62, cy)
            context.line_to(cx + offset * radius, cy)
            context.stroke()
        return False


class VideoStandbyCanvas(Gtk.DrawingArea):
    """Render the common rover asset as full, arm, or front framing."""

    CROPS = {
        "full": (0.0, 0.0, 390.0, 230.0),
        "arm": (72.0, 4.0, 190.0, 172.0),
        "front": (218.0, 27.0, 154.0, 170.0),
    }

    def __init__(self, focus: str = "full") -> None:
        super().__init__()
        self._focus = focus
        self._front_live = False
        self._work_live = False
        self._drive_ready = False
        self._safety_ready = False
        self._svg = Rsvg.Handle.new_from_file(str(
            Path(__file__).resolve().parent / "assets" / "mobile_robot_pictogram.svg"
        ))
        self.set_hexpand(True)
        self.set_vexpand(True)
        self.connect("draw", self._draw)

    def _draw(self, _widget: Gtk.DrawingArea, context: object) -> bool:
        allocation = self.get_allocation()
        width, height = allocation.width, allocation.height
        if width < 2 or height < 2:
            return False
        _has_size, design_w, design_h = self._svg.get_intrinsic_size_in_pixels()
        crop_x, crop_y, crop_w, crop_h = self.CROPS[self._focus]
        scale = min(
            max(1.0, width) / crop_w,
            max(1.0, height) / crop_h,
        )
        ox = (width - crop_w * scale) / 2.0 - crop_x * scale
        oy = (height - crop_h * scale) / 2.0 - crop_y * scale
        context.save()
        context.rectangle(0, 0, width, height)
        context.clip()
        context.translate(ox, oy)
        context.scale(scale, scale)
        viewport = Rsvg.Rectangle()
        viewport.x = 0
        viewport.y = 0
        viewport.width = design_w
        viewport.height = design_h
        self._svg.render_document(context, viewport)
        context.restore()
        return False

    def set_focus(self, focus: str) -> None:
        if focus != self._focus:
            self._focus = focus
            self.queue_draw()

    def set_states(
        self, *, front_live: bool, work_live: bool,
        drive_ready: bool, safety_ready: bool,
    ) -> None:
        values = (front_live, work_live, drive_ready, safety_ready)
        if values == (
            self._front_live, self._work_live,
            self._drive_ready, self._safety_ready,
        ):
            return
        (self._front_live, self._work_live,
         self._drive_ready, self._safety_ready) = values
        self.queue_draw()


class RoverPlaceholder(Gtk.Box):
    """Shared placeholder whose geometry depends only on MAIN/SUB slot."""

    MAIN_WIDTH = 290
    PREVIEW_WIDTH = 90
    SVG_ASPECT = 390 / 230

    def __init__(
        self, *, title: str, description: str, camera_kind: str,
    ) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=7)
        self._camera_kind = camera_kind
        self.set_halign(Gtk.Align.CENTER)
        self.set_valign(Gtk.Align.CENTER)
        self._rover = VideoStandbyCanvas()
        self._spacer = Gtk.Box()
        heading = Gtk.Label(label=title)
        heading.set_line_wrap(False)
        _style(heading, "camera-placeholder-title")
        detail = Gtk.Label(label=description)
        detail.set_line_wrap(False)
        _style(detail, "camera-placeholder-detail")
        self.pack_start(self._rover, False, False, 0)
        self.pack_start(self._spacer, False, False, 0)
        self.pack_start(heading, False, False, 0)
        self.pack_start(detail, False, False, 0)
        _style(self, "camera-placeholder")
        self.set_slot("MAIN")

    @property
    def rover(self) -> VideoStandbyCanvas:
        return self._rover

    def set_slot(self, role: str) -> None:
        width = self.MAIN_WIDTH if role == "MAIN" else self.PREVIEW_WIDTH
        focus = (
            "full" if role == "MAIN"
            else "arm" if self._camera_kind == "work"
            else "front"
        )
        aspect = (
            self.SVG_ASPECT if focus == "full"
            else 190 / 172 if focus == "arm"
            else 154 / 170
        )
        height = round(width / aspect)
        self._rover.set_focus(focus)
        self._rover.set_size_request(width, height)
        self._spacer.set_size_request(-1, 32 if role == "MAIN" else 10)
        self.set_margin_bottom(16 if role == "MAIN" else 0)


def draw_jet_in_watermark(
    context: object, handle: Rsvg.Handle, *, live: bool,
) -> None:
    """Render the finished wordmark SVG with state-dependent opacity."""
    opacity = 0.48 if live else 0.56
    _has_size, width, height = handle.get_intrinsic_size_in_pixels()
    viewport = Rsvg.Rectangle()
    viewport.x = 0
    viewport.y = 0
    viewport.width = width
    viewport.height = height
    context.push_group()
    handle.render_document(context, viewport)
    context.pop_group_to_source()
    context.paint_with_alpha(opacity)


class JetInWatermark(Gtk.DrawingArea):
    """Pass-through watermark owned by the main video viewport."""

    def __init__(self) -> None:
        super().__init__()
        self._live = False
        self._svg = Rsvg.Handle.new_from_file(str(
            Path(__file__).resolve().parent / "assets" / "jetin_wordmark.svg"
        ))
        self.set_size_request(132, 34)
        self.connect("draw", self._draw)

    def _draw(self, _widget: Gtk.DrawingArea, context: object) -> bool:
        allocation = self.get_allocation()
        _has_size, width, height = self._svg.get_intrinsic_size_in_pixels()
        scale = min(
            allocation.width / max(1.0, width),
            allocation.height / max(1.0, height),
        )
        context.save()
        context.scale(scale, scale)
        draw_jet_in_watermark(context, self._svg, live=self._live)
        context.restore()
        return False

    def set_live(self, live: bool) -> None:
        if self._live != live:
            self._live = live
            self.queue_draw()




class EventOperationRow(Gtk.ListBoxRow):
    """One public event with its exact raw source inline on demand."""

    def __init__(
        self, stamp: str, severity: str, public: str,
        source: str, message: str, technical: str,
        developer_visible: bool = False,
    ) -> None:
        super().__init__()
        self._expanded = False
        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        header = Gtk.Grid(column_spacing=0)
        stamp_label = Gtk.Label(label=stamp)
        stamp_label.set_xalign(0.0)
        stamp_label.set_size_request(100, -1)
        _style(stamp_label, "event-operation-time")
        public_label = Gtk.Label(label=public)
        public_label.set_xalign(0.0)
        public_label.set_hexpand(True)
        public_label.set_line_wrap(False)
        public_label.set_ellipsize(Pango.EllipsizeMode.END)
        _style(public_label, "event-operation-message")
        technical_box = Gtk.Box(spacing=5)
        technical_box.set_size_request(360, -1)
        module_label = Gtk.Label(label=source.upper())
        _style(module_label, "event-operation-module")
        separator = Gtk.Label(label="·")
        _style(separator, "event-operation-separator")
        technical_label = Gtk.Label(label=technical)
        technical_label.set_xalign(0.0)
        technical_label.set_ellipsize(Pango.EllipsizeMode.END)
        technical_label.set_hexpand(True)
        _style(technical_label, "event-operation-technical")
        technical_box.pack_start(module_label, False, False, 0)
        technical_box.pack_start(separator, False, False, 0)
        technical_box.pack_start(technical_label, True, True, 0)
        technical_box.set_no_show_all(not developer_visible)
        technical_box.set_visible(developer_visible)
        self._arrow = Gtk.Label(label="›")
        self._arrow.set_size_request(18, -1)
        _style(self._arrow, "event-operation-arrow")
        self._arrow.set_no_show_all(not developer_visible)
        self._arrow.set_visible(developer_visible)
        header.attach(stamp_label, 0, 0, 1, 1)
        header.attach(public_label, 1, 0, 1, 1)
        header.attach(technical_box, 2, 0, 1, 1)
        header.attach(self._arrow, 3, 0, 1, 1)
        self._raw = Gtk.Label(
            label=f"원본: [{stamp}] {severity} {source}: {message}",
        )
        self._raw.set_xalign(0.0)
        self._raw.set_line_wrap(True)
        self._raw.set_no_show_all(True)
        self._developer_visible = developer_visible
        _style(self._raw, "event-operation-raw")
        content.pack_start(header, False, False, 0)
        content.pack_start(self._raw, False, False, 0)
        self.add(content)
        _style(self, "event-operation-row")

    def toggle_detail(self) -> None:
        if not self._developer_visible:
            return
        self._expanded = not self._expanded
        self._arrow.set_text("⌄" if self._expanded else "›")
        self._raw.set_visible(self._expanded)


class EventLog(Gtk.Box):
    """User-facing Korean timeline; raw source/message stays in row details."""

    _MAX_LINES = 100

    def __init__(self) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        _style(self, "event-card")
        self._entries: list[tuple[str, str, str, str]] = []
        self._filters: dict[str, Gtk.CheckButton] = {}
        self.event_filter_state = {
            "ERROR": True, "WARNING": True, "INFO": True,
        }
        self._developer_visible = False
        self.filter_box = Gtk.Box(spacing=16)
        self.filter_box.set_valign(Gtk.Align.CENTER)
        filter_labels = {
            "ERROR": "위험", "WARNING": "확인 필요", "INFO": "정보",
        }
        for severity in ("ERROR", "WARNING", "INFO"):
            toggle = Gtk.CheckButton(label=filter_labels[severity])
            toggle.set_valign(Gtk.Align.CENTER)
            _style(toggle, "event-filter", f"event-filter-{severity.lower()}")
            toggle.connect("toggled", self._on_filter_toggled, severity)
            self.filter_box.pack_start(toggle, False, False, 0)
            self._filters[severity] = toggle
        column_header = Gtk.Grid(column_spacing=0)
        _style(column_header, "event-column-header")
        for column, title, width in (
            (0, "시간", 100),
            (1, "운용 기록", -1),
            (2, "기술 정보", 360),
            (3, "", 18),
        ):
            label = Gtk.Label(label=title)
            label.set_xalign(0.0)
            if width > 0:
                label.set_size_request(width, -1)
            if column == 1:
                label.set_hexpand(True)
            _style(label, "event-column-title")
            column_header.attach(label, column, 0, 1, 1)
            if column in (2, 3):
                label.set_no_show_all(True)
                label.hide()
                if column == 2:
                    self._technical_header = label
                else:
                    self._arrow_header = label
        self.pack_start(column_header, False, False, 0)
        self._operation_list = Gtk.ListBox()
        self._operation_list.set_selection_mode(Gtk.SelectionMode.NONE)
        self._operation_list.set_activate_on_single_click(True)
        self._operation_list.connect("row-activated", self._on_operation_activated)
        _style(self._operation_list, "event-operation-list")
        self._user_scroll = Gtk.ScrolledWindow()
        self._user_scroll.set_policy(
            Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC,
        )
        self._user_scroll.set_propagate_natural_height(True)
        self._user_scroll.set_max_content_height(210)
        self._user_scroll.add(self._operation_list)
        self.pack_start(self._user_scroll, True, True, 0)
        self._empty = Gtk.Label(label="선택한 분류의 기록이 없습니다")
        self._empty.set_halign(Gtk.Align.CENTER)
        self._empty.set_valign(Gtk.Align.CENTER)
        self._empty.set_margin_top(24)
        self._empty.set_margin_bottom(24)
        self._empty.set_no_show_all(True)
        _style(self._empty, "event-status-message")
        self.pack_start(self._empty, True, True, 0)
        for toggle in self._filters.values():
            toggle.set_active(True)

    def _on_operation_activated(
        self, _list: Gtk.ListBox, row: Gtk.ListBoxRow,
    ) -> None:
        if isinstance(row, EventOperationRow):
            row.toggle_detail()

    @staticmethod
    def _severity(source: str, message: str) -> str:
        normalized = f"{source} {message}".upper()
        if any(token in normalized for token in (
            "ESTOP", "CRIT", "FATAL", "TRACEBACK", "FAULT", "ERROR", "실패",
        )):
            return "ERROR"
        if any(token in normalized for token in (
            "STALE", "WAIT", "WARN", "UNAVAILABLE", "지연", "대기",
        )):
            return "WARNING"
        return "INFO"

    @staticmethod
    def _public_message(source: str, message: str) -> str:
        normalized = f"{source} {message}".lower()
        public_source = {
            "L515": "전방 화면",
            "D435": "작업 화면",
            "D435I": "작업 화면",
            "YOLO": "인식",
            "METADATA": "인식",
            "ARM": "로봇팔",
            "OPS": "조작 채널",
            "TELEMETRY": "로봇 상태",
            "CHASSIS": "주행 시스템",
        }.get(source.upper(), source)
        if "main view changed" in normalized:
            camera = message.split(":", 1)[-1].strip()
            return f"{camera}가 주 화면으로 전환되었습니다"
        if "fullscreen enabled" in normalized:
            return "전체 화면 모드가 활성화되었습니다"
        if "fullscreen disabled" in normalized:
            return "전체 화면 모드가 해제되었습니다"
        if source.upper() in {"SAFETY", "안전"} and any(token in normalized for token in (
            "unavailable", "waiting", "미수신",
        )):
            return "안전 장치 정보를 기다리고 있습니다"
        if "waiting for main stream" in normalized:
            return "전방 화면 연결을 기다리고 있습니다"
        if "reconnect scheduled" in normalized:
            return "화면 연결을 다시 시도하고 있습니다"
        if "frame flow live" in normalized:
            return f"{public_source} 영상이 연결되었습니다"
        if "stale" in normalized:
            return f"{public_source} 업데이트가 지연되고 있습니다"
        if "waiting" in normalized or "connecting" in normalized:
            return f"{public_source} 연결을 기다리고 있습니다"
        if any(token in normalized for token in (
            "srt", "udp", "tcp", "seq", "metadata", "frame", "traceback",
            "exception", "can ", "hz", "ms", "node",
        )):
            return f"{public_source} 상태를 확인하고 있습니다"
        return message if any("\uac00" <= char <= "\ud7a3" for char in message) \
            else f"{public_source} 상태가 업데이트되었습니다"

    @staticmethod
    def _technical_summary(message: str) -> str:
        normalized = " ".join(message.strip().split())
        if "(" in normalized and normalized.endswith(")"):
            parenthetical = normalized.rsplit("(", 1)[-1][:-1].strip()
            if parenthetical:
                normalized = parenthetical
        elif ":" in normalized:
            prefix = normalized.split(":", 1)[0].strip()
            if prefix and all(ord(char) < 128 for char in prefix):
                normalized = prefix
        if len(normalized) > 48:
            normalized = normalized[:47].rstrip() + "…"
        return normalized or "상태 업데이트"

    def _on_filter_toggled(self, button: Gtk.ToggleButton, severity: str) -> None:
        self.event_filter_state[severity] = button.get_active()
        self._render()

    def set_developer_visible(self, visible: bool) -> None:
        self._developer_visible = visible
        for label in (self._technical_header, self._arrow_header):
            label.set_no_show_all(not visible)
            label.set_visible(visible)
        self._render()

    def _render(self, *_args: object) -> None:
        for child in self._operation_list.get_children():
            self._operation_list.remove(child)
        visible_count = 0
        for stamp, severity, source, message in self._entries:
            if not self.event_filter_state[severity]:
                continue
            row = EventOperationRow(
                stamp, severity,
                self._public_message(source, message),
                source, message,
                self._technical_summary(message),
                developer_visible=self._developer_visible,
            )
            self._operation_list.add(row)
            visible_count += 1
        self._operation_list.show_all()
        self._user_scroll.set_visible(visible_count > 0)
        self._empty.set_visible(visible_count == 0)
        GLib.idle_add(self._scroll_operations_to_bottom)

    def _scroll_operations_to_bottom(self) -> bool:
        adjustment = self._user_scroll.get_vadjustment()
        adjustment.set_value(max(
            adjustment.get_lower(),
            adjustment.get_upper() - adjustment.get_page_size(),
        ))
        return GLib.SOURCE_REMOVE

    def add_event(self, source: str, message: str) -> None:
        stamp = time.strftime("%H:%M:%S")
        severity = self._severity(source, message)
        self._entries.append((stamp, severity, source, message))
        if len(self._entries) > self._MAX_LINES:
            self._entries = self._entries[-self._MAX_LINES:]
        self._render()

    def latest_public(self) -> tuple[str, str]:
        for _stamp, severity, source, message in reversed(self._entries):
            if self.event_filter_state[severity]:
                return severity, self._public_message(source, message)
        if not any(self.event_filter_state.values()):
            return "INFO", "표시할 이벤트 분류를 선택해 주세요"
        return "INFO", "선택한 분류의 기록이 없습니다"


class EventDrawer(Gtk.Box):
    """Event footer whose filters are outside the disclosure button."""

    def __init__(self, events: EventLog) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        _style(self, "event-expander")
        heading = Gtk.Box(spacing=12)
        _style(heading, "event-header")
        self._disclosure = Gtk.Button(label="›  이벤트 기록")
        self._disclosure.set_relief(Gtk.ReliefStyle.NONE)
        _style(self._disclosure, "event-status-title", "event-collapse")
        self._disclosure.connect("clicked", self._on_disclosure_clicked)
        self.latest = Gtk.Label(label="최근: 이벤트 수신 대기")
        self.latest.set_xalign(0.0)
        self.latest.set_ellipsize(Pango.EllipsizeMode.END)
        _style(self.latest, "event-status-message")
        heading.pack_start(self._disclosure, False, False, 0)
        heading.pack_start(self.latest, True, True, 0)
        # These are ordinary sibling controls, not children of a GtkExpander
        # label.  Every checkbox therefore owns its complete click area.
        heading.pack_end(events.filter_box, False, False, 0)
        events.filter_box.set_no_show_all(True)
        self.pack_start(heading, False, False, 0)
        self._events = events
        events.set_no_show_all(True)
        self.pack_start(events, False, True, 0)
        self.set_expanded(False)

    def _on_disclosure_clicked(self, _button: Gtk.Button) -> None:
        self.set_expanded(not self.get_expanded())

    def get_expanded(self) -> bool:
        return self._events.get_visible()

    def set_expanded(self, expanded: bool) -> None:
        expanded = bool(expanded)
        self._events.set_no_show_all(not expanded)
        self._events.filter_box.set_no_show_all(not expanded)
        if expanded:
            self._events.show_all()
            self._events.filter_box.show_all()
        else:
            self._events.hide()
            self._events.filter_box.hide()
        self._disclosure.set_label(
            "⌄  이벤트 기록" if expanded else "›  이벤트 기록"
        )


class FixedSizeSlot(Gtk.Bin):
    """자식의 natural size 를 전파하지 않는 고정 크기 컨테이너.

    Gtk 의 size request 는 '최소'라서, gtksink 가 영상 해상도를 natural size 로
    보고하면 Overlay 가 그 크기로 할당해 PiP 가 스테이지를 덮어버린다
    (2026-07-29 실기 관측).  이 슬롯은 지정된 크기만 요구한다.
    """

    # 첫 size-allocate 전에도 유효한 크기를 요구해야 한다.  1x1 로 두면 기동 시
    # "Negative content width -1 ... owner VideoPanel" GTK 경고가 뜬다(테두리·패딩이
    # 1 px 할당보다 크다).  최소 PiP 폭 300 과 848:480 비율을 초기값으로 쓴다.
    DEFAULT_SLOT_WIDTH = 300
    DEFAULT_SLOT_HEIGHT = 170

    def __init__(self) -> None:
        super().__init__()
        self._slot_width = self.DEFAULT_SLOT_WIDTH
        self._slot_height = self.DEFAULT_SLOT_HEIGHT

    def set_slot_size(self, width: int, height: int) -> None:
        width, height = max(1, int(width)), max(1, int(height))
        if (width, height) == (self._slot_width, self._slot_height):
            return
        self._slot_width, self._slot_height = width, height
        self.queue_resize()

    def do_get_preferred_width(self) -> tuple[int, int]:
        return self._slot_width, self._slot_width

    def do_get_preferred_height(self) -> tuple[int, int]:
        return self._slot_height, self._slot_height

    def do_get_preferred_width_for_height(self, _height: int) -> tuple[int, int]:
        return self._slot_width, self._slot_width

    def do_get_preferred_height_for_width(self, _width: int) -> tuple[int, int]:
        return self._slot_height, self._slot_height


class VideoPanel(Gtk.Box):
    """One read-only SRT receiver panel embedded in the console."""

    def __init__(self, name: str, host: str, port: int, latency_ms: int,
                 metadata_receiver: LatestMetadataReceiver | None = None,
                 target_tracker: DisplayTargetTracker | None = None,
                 event_sink: Callable[[str, str], None] | None = None) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL)
        _style(self, "video-card")
        self._name = name
        self._event_sink = event_sink
        self._pipeline = Gst.parse_launch(pipeline_description(host, port, latency_ms))
        self._sink = self._pipeline.get_by_name("video_sink")
        self._video_widget = self._sink.get_property("widget")
        self._video_widget.set_hexpand(True)
        self._video_widget.set_vexpand(True)
        self._video_widget.connect("realize", self._on_realize)
        self._status = Gtk.Label(label=f"{name}: 연결 중")
        self._status.set_ellipsize(Pango.EllipsizeMode.END)
        self._status.set_max_width_chars(38)
        self._fps = Gtk.Label(label="화면 FPS: 대기 중")
        self._role = Gtk.Label(label=name)
        _style(self._role, "camera-name")
        self._detail = Gtk.Label(label=f"SRT 호출자 → {host}:{port}, 지연 {latency_ms} ms")
        self._detail.set_xalign(0.0)
        self._detail.set_ellipsize(Pango.EllipsizeMode.END)
        header = Gtk.Box(spacing=10)
        self._header = header
        header.set_border_width(8)
        header.pack_start(self._role, False, False, 0)
        self._connection_dot = Gtk.Label(label="")
        self._connection_dot.set_size_request(7, 7)
        _style(self._connection_dot, "status-dot", "camera-dot")
        header.pack_start(self._connection_dot, False, False, 0)
        self._header_state = Gtk.Label(label="연결 대기")
        _style(self._header_state, "camera-connection-label")
        header.pack_start(self._header_state, False, False, 0)
        swap_hint = Gtk.Label(label="")
        self._swap_hint = swap_hint
        _style(swap_hint, "swap-hint")
        header.pack_end(swap_hint, False, False, 0)
        header_click = Gtk.EventBox()
        self._header_click = header_click
        header_click.add(header)
        header_click.set_halign(Gtk.Align.FILL)
        header_click.set_valign(Gtk.Align.START)
        _style(header_click, "video-overlay")
        if metadata_receiver is None:
            visual = self._video_widget
        else:
            metadata_overlay = Gtk.Overlay()
            metadata_overlay.add(self._video_widget)
            canvas = MetadataCanvas(
                metadata_receiver, target_tracker=target_tracker,
            )
            canvas.set_hexpand(True)
            canvas.set_vexpand(True)
            metadata_overlay.add_overlay(canvas)
            self._metadata_canvas = canvas
            visual = metadata_overlay
            self._metadata_receiver = metadata_receiver
            self._metadata_label = Gtk.Label(label="YOLO: UDP :5003 대기 중")
            self._metadata_state: tuple[str, int] | None = None
            GLib.timeout_add(100, self._refresh_metadata_status)

        video_stage = Gtk.Overlay()
        video_stage.set_hexpand(True)
        video_stage.set_vexpand(True)
        video_stage.add(visual)
        standby = Gtk.Overlay()
        standby.set_hexpand(True)
        standby.set_vexpand(True)
        standby_background = Gtk.Box()
        _style(standby_background, "camera-standby-background")
        standby.add(standby_background)
        placeholder = RoverPlaceholder(
            title=("전방 화면 준비 중"
                   if name == "전방 카메라" else "작업 화면 준비 중"),
            description=("로봇의 주행 영상을 연결하고 있습니다"
                         if name == "전방 카메라"
                         else "로봇팔 작업 영상을 연결하고 있습니다"),
            camera_kind=("front" if name == "전방 카메라" else "work"),
        )
        self._rover_placeholder = placeholder
        self._rover_status_overlay = placeholder.rover
        standby.add_overlay(placeholder)
        video_stage.add_overlay(standby)
        video_stage.add_overlay(header_click)
        self._placeholder = standby
        self.pack_start(video_stage, True, True, 0)

        self._sub_optional = []

        self._stopped = False
        self._retry_source_id: int | None = None
        self._frames = 0
        self._sample_start = time.monotonic()
        self._last_frame_monotonic: float | None = None
        self._last_fps: float | None = None
        self._video_size: tuple[int, int] | None = None
        self._pipeline_live = False
        self._reconnects = 0
        self._freshness_state = "connecting"
        sink_pad = self._sink.get_static_pad("sink")
        sink_pad.add_probe(Gst.PadProbeType.BUFFER, self._on_video_buffer)
        bus = self._pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self._on_bus_message)
        bus.enable_sync_message_emission()
        bus.connect("sync-message::element", self._on_sync_message)
        GLib.timeout_add(200, self._refresh_video_health)

    def set_header_action(self, action: Gtk.Widget) -> None:
        """Place the PiP action inside this panel's header without overlap."""
        parent = action.get_parent()
        if parent is not None:
            parent.remove(action)
        self._header.pack_end(action, False, False, 0)
        action.show_all()

    def set_rover_component_states(
        self, *, front_live: bool, work_live: bool,
        drive_ready: bool, safety_ready: bool,
    ) -> None:
        overlay = getattr(self, "_rover_status_overlay", None)
        if overlay is not None:
            overlay.set_states(
                front_live=front_live, work_live=work_live,
                drive_ready=drive_ready, safety_ready=safety_ready,
            )

    def set_role(self, role: str) -> None:
        normalized = "MAIN" if role == "MAIN" else "SUB"
        compact = normalized == "SUB"
        self._rover_placeholder.set_slot(normalized)
        self._status.set_max_width_chars(20 if compact else 38)
        self._swap_hint.set_text("")
        self._header_click.set_no_show_all(not compact)
        self._header_click.set_visible(compact)
        for widget in self._sub_optional:
            widget.set_no_show_all(compact)
            widget.set_visible(not compact)

    def set_metadata_display_options(
        self, *, show_objects: bool, show_distance: bool,
    ) -> None:
        canvas = getattr(self, "_metadata_canvas", None)
        if canvas is not None:
            canvas.set_display_options(
                show_objects=show_objects,
                show_distance=show_distance,
            )

    @property
    def last_fps(self) -> float | None:
        return self._last_fps

    @property
    def last_frame_age_s(self) -> float | None:
        if self._last_frame_monotonic is None:
            return None
        return max(0.0, time.monotonic() - self._last_frame_monotonic)

    def _on_realize(self, _area: Gtk.DrawingArea) -> None:
        self._pipeline.set_state(Gst.State.PLAYING)

    def _on_sync_message(self, _bus: Gst.Bus, message: Gst.Message) -> None:
        # ``gtksink`` owns a GTK widget, so it never requests a separate
        # X11/Wayland surface through ``prepare-window-handle``.
        del message

    def _on_video_buffer(
        self, pad: Gst.Pad, _info: Gst.PadProbeInfo,
    ) -> Gst.PadProbeReturn:
        self._frames += 1
        caps = pad.get_current_caps()
        if caps is not None and caps.is_fixed() and caps.get_size() > 0:
            structure = caps.get_structure(0)
            has_width, width = structure.get_int("width")
            has_height, height = structure.get_int("height")
            video_size = (width, height)
            if (
                has_width
                and has_height
                and width > 0
                and height > 0
                and video_size != self._video_size
            ):
                self._video_size = video_size
                canvas = getattr(self, "_metadata_canvas", None)
                if canvas is not None:
                    GLib.idle_add(canvas.set_video_size, width, height)
        now = time.monotonic()
        self._last_frame_monotonic = now
        elapsed = now - self._sample_start
        if elapsed >= 1.0:
            fps = self._frames / elapsed
            self._frames = 0
            self._sample_start = now
            self._last_fps = fps
            GLib.idle_add(self._fps.set_text, f"화면 FPS: {fps:.1f}")
        return Gst.PadProbeReturn.OK

    def _on_bus_message(self, _bus: Gst.Bus, message: Gst.Message) -> None:
        if message.type == Gst.MessageType.ERROR:
            self._placeholder.show_all()
            error, detail = message.parse_error()
            self._pipeline_live = False
            self._status.set_text(f"{self._name} 연결 대기")
            self._detail.set_text(str(error if not detail else f"{error}: {detail}"))
            self._schedule_reconnect()
        elif message.type == Gst.MessageType.EOS:
            self._placeholder.show_all()
            self._pipeline_live = False
            self._status.set_text(f"{self._name} 연결 대기")
            self._schedule_reconnect()
        elif message.type == Gst.MessageType.WARNING:
            warning, _detail = message.parse_warning()
            self._status.set_text(f"{self._name} 연결 대기")
        elif message.type == Gst.MessageType.STATE_CHANGED and message.src == self._pipeline:
            _old, new, _pending = message.parse_state_changed()
            if new == Gst.State.PLAYING:
                self._pipeline_live = True
                self._freshness_state = "waiting"
                self._emit_event("SRT pipeline connected; waiting for first frame")

    def _refresh_video_health(self) -> bool:
        if self._stopped:
            return False
        if not self._pipeline_live:
            self._placeholder.show_all()
            self._header_state.set_text("연결 대기")
            return True
        if self._last_frame_monotonic is None:
            self._placeholder.show_all()
            self._status.set_text(f"{self._name} 연결 대기")
            self._header_state.set_text("연결 대기")
            return True
        age_ms = (time.monotonic() - self._last_frame_monotonic) * 1000.0
        if age_ms > 1000.0:
            self._placeholder.show_all()
            if self._freshness_state != "stale":
                self._emit_event("frame became stale (>1000 ms)")
                self._freshness_state = "stale"
            self._status.set_text(f"{self._name} 정보 갱신 지연")
            self._header_state.set_text("갱신 지연")
            if (
                age_ms > VIDEO_STALE_RESTART_S * 1000.0
                and self._retry_source_id is None
            ):
                self._emit_event("stale watchdog: forcing pipeline restart")
                self._schedule_reconnect()
        else:
            self._placeholder.hide()
            if self._freshness_state != "live":
                self._emit_event("frame flow live")
                self._freshness_state = "live"
            self._status.set_text(f"{self._name} 연결됨")
            self._header_state.set_text("연결됨")
        return True

    def _emit_event(self, message: str) -> None:
        if self._event_sink is not None:
            self._event_sink(self._name, message)

    def health_state(self) -> str:
        if not self._pipeline_live:
            return "CONNECTING"
        if self._last_frame_monotonic is None:
            return "WAITING"
        return "STALE" if time.monotonic() - self._last_frame_monotonic > 1.0 else "LIVE"

    def _schedule_reconnect(self) -> None:
        if self._stopped or self._retry_source_id is not None:
            return
        self._reconnects += 1
        self._emit_event(f"SRT reconnect scheduled ({self._reconnects})")
        self._retry_source_id = GLib.timeout_add(1000, self._restart_pipeline)

    def _restart_pipeline(self) -> bool:
        self._retry_source_id = None
        if self._stopped:
            return False
        self._pipeline.set_state(Gst.State.NULL)
        # 재시작 후에도 죽기 전 프레임 시각이 남아 있으면 워치독이 곧바로
        # 다시 발화하고 표시도 거대한 stale 나이를 보인다 — 상태를 리셋한다.
        self._pipeline_live = False
        self._last_frame_monotonic = None
        self._last_fps = None
        self._frames = 0
        self._sample_start = time.monotonic()
        self._freshness_state = "connecting"
        self._pipeline.set_state(Gst.State.PLAYING)
        return False

    def _refresh_metadata_status(self) -> bool:
        receiver = getattr(self, "_metadata_receiver", None)
        if receiver is None:
            return False
        frame: MetadataFrame | None = receiver.latest()
        if frame is None:
            self._metadata_label.set_text("YOLO: UDP :5003 대기 중")
            self._report_metadata_state("waiting", 0)
        else:
            age_ms = (time.monotonic() - frame.received_monotonic_s) * 1000.0
            if age_ms > OVERLAY_STALE_AFTER_S * 1000.0:
                self._metadata_label.set_text(f"YOLO: 지연(STALE) ({age_ms:.0f} ms)")
                self._report_metadata_state("stale", len(frame.detections))
            else:
                summary = []
                for detection in frame.detections[:3]:
                    text = f"{detection.class_name} {detection.confidence:.2f}"
                    distance_m = target_distance_m(detection)
                    if distance_m is not None:
                        text += f" {distance_m:.2f}m"
                    summary.append(text)
                detail = " · ".join(summary) if summary else "탐지 없음"
                self._metadata_label.set_text(
                    f"YOLO: 객체 {len(frame.detections)}개 · {age_ms:.0f} ms · {detail}"
                )
                self._report_metadata_state("live", len(frame.detections))
        self._metadata_canvas.queue_draw()
        return True

    def _report_metadata_state(self, state: str, count: int) -> None:
        key = (state, count)
        if key == self._metadata_state:
            return
        self._metadata_state = key
        if state == "live":
            self._emit_event(f"YOLO metadata live ({count} objects)")
        elif state == "stale":
            self._emit_event(f"YOLO metadata stale ({count} objects)")
        else:
            self._emit_event("YOLO metadata waiting")

    def stop(self) -> None:
        self._stopped = True
        if self._retry_source_id is not None:
            GLib.source_remove(self._retry_source_id)
            self._retry_source_id = None
        self._pipeline.set_state(Gst.State.NULL)


class TelemetryPanel(Gtk.Frame):
    """Read-only latest snapshot display; unavailable is a valid visible state."""
    def __init__(self, receiver: LatestTelemetryReceiver, port: int,
                 event_sink: Callable[[str, str], None] | None = None) -> None:
        super().__init__(label="로봇 상태")
        self._receiver = receiver
        self._port = port
        self._event_sink = event_sink
        self._power_health_seen = False
        self._power_health_key: tuple[int | None, int | None] | None = None
        self._rs485_key: tuple[str, int | None, str] | None = None
        self._telemetry_link_state = "waiting"
        self._last_sequence: int | None = None
        self._labels: dict[str, Gtk.Label] = {}
        body = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        self._summary = Gtk.Label(label=power_summary(None))
        self._summary.set_xalign(0.0)
        self._summary.set_margin_start(10)
        self._summary.set_margin_top(6)
        body.pack_start(self._summary, False, False, 0)
        grid = Gtk.Grid(column_spacing=12, row_spacing=7, margin=10)
        rows = (("link", "수신"), ("rs485", "RS485"), ("power", "PDIST80B"),
                ("bringup", "기동 상태"))
        for row, (key, title) in enumerate(rows):
            name = Gtk.Label(label=title)
            name.set_xalign(0.0)
            value = Gtk.Label(label="미수신(UNAVAILABLE)")
            value.set_xalign(0.0)
            value.set_line_wrap(True)
            grid.attach(name, 0, row, 1, 1)
            grid.attach(value, 1, row, 1, 1)
            self._labels[key] = value
        details = Gtk.Expander(label="상세")
        details.add(grid)
        body.pack_start(details, False, False, 0)
        self.add(body)
        GLib.timeout_add(200, self._refresh)

    @staticmethod
    def _power_health_text(battery_flags: int | None, protection_flags: int | None) -> str:
        if protection_flags not in (None, 0):
            return f"보호 경고 {protection_flags:#04x}"
        if battery_flags not in (None, 0):
            return f"배터리 경고 {battery_flags:#04x}"
        if battery_flags == 0 and protection_flags == 0:
            return "정상"
        return "미수신(UNAVAILABLE)"

    def _report_power_health(self, battery_flags: int | None, protection_flags: int | None) -> None:
        key = (battery_flags, protection_flags)
        if self._power_health_seen and key == self._power_health_key:
            return
        self._power_health_seen = True
        self._power_health_key = key
        if self._event_sink is not None:
            self._event_sink("PDIST80B", self._power_health_text(battery_flags, protection_flags))

    def _emit_event(self, message: str) -> None:
        if self._event_sink is not None:
            self._event_sink("Telemetry", message)

    def _report_rs485(self, snapshot: TelemetrySnapshot) -> None:
        key = (snapshot.rs485_state, snapshot.rs485_consecutive_failures,
               snapshot.rs485_detail)
        if key == self._rs485_key:
            return
        self._rs485_key = key
        if self._event_sink is not None:
            failures = ("N/A" if snapshot.rs485_consecutive_failures is None
                        else str(snapshot.rs485_consecutive_failures))
            detail = snapshot.rs485_detail or "-"
            self._event_sink("RS485", f"{snapshot.rs485_state} · failures {failures} · {detail}")

    def _refresh(self) -> bool:
        snapshot: TelemetrySnapshot | None = self._receiver.latest()
        summary_label = getattr(self, "_summary", None)
        if summary_label is not None:
            summary_label.set_text(power_summary(snapshot))
        if snapshot is None:
            self._labels["link"].set_text(
                f"{freshness_korean('WAITING')} · UDP :{self._port}"
            )
            return True
        age_ms = (time.monotonic() - snapshot.received_monotonic_s) * 1000.0
        if age_ms > 1000.0:
            self._labels["link"].set_text(
                f"{freshness_korean('STALE')} · {age_ms:.0f} ms"
            )
            if self._telemetry_link_state != "stale":
                self._telemetry_link_state = "stale"
                self._emit_event(f"snapshot stale ({age_ms:.0f} ms)")
            return True
        if self._telemetry_link_state != "live":
            self._telemetry_link_state = "live"
            self._emit_event("snapshot live")
        if self._last_sequence is not None and snapshot.sequence != self._last_sequence:
            expected = self._last_sequence + 1
            if snapshot.sequence != expected:
                self._emit_event(f"sequence gap {self._last_sequence} → {snapshot.sequence}")
        self._last_sequence = snapshot.sequence
        self._labels["link"].set_text(
            f"{freshness_korean('LIVE')} · 순번 {snapshot.sequence} · {age_ms:.0f} ms"
        )
        failures = "N/A" if snapshot.rs485_consecutive_failures is None else str(
            snapshot.rs485_consecutive_failures)
        detail = snapshot.rs485_detail or "-"
        self._labels["rs485"].set_text(
            f"{snapshot.rs485_state} · 실패 {failures}\n{detail}"
        )
        self._report_rs485(snapshot)
        self._labels["power"].set_text(
            f"{_format_number(snapshot.voltage_v, 'V')} · {_format_number(snapshot.current_a, 'A')} · "
            f"{_format_number(snapshot.power_w, 'W')}\n"
            f"충전율 {'N/A' if snapshot.pdist_soc_percent is None else f'{snapshot.pdist_soc_percent}%'} · "
            f"충전 {_format_number(snapshot.pdist_charge_current_a, 'A')} · "
            f"배터리 {_format_hex(snapshot.pdist_battery_flags)} · "
            f"보호 {_format_hex(snapshot.pdist_protection_flags)}\n"
            f"{self._power_health_text(snapshot.pdist_battery_flags, snapshot.pdist_protection_flags)}"
        )
        self._labels["bringup"].set_text(
            f"유닛 {dict(snapshot.unit_status) or 'N/A'} · "
            f"Compose {dict(snapshot.compose_status) or 'N/A'}\n"
            f"저널 {snapshot.journal_tail[-1] if snapshot.journal_tail else 'N/A'}"
        )
        self._report_power_health(snapshot.pdist_battery_flags, snapshot.pdist_protection_flags)
        return True


class ArmTelemetryPanel(Gtk.Frame):
    """Read-only robot-arm motor and joint snapshot display."""
    def __init__(self, receiver: LatestArmTelemetryReceiver, port: int,
                 event_sink: Callable[[str, str], None] | None = None) -> None:
        super().__init__(label="로봇팔")
        self._receiver = receiver
        self._port = port
        self._event_sink = event_sink
        self._temperature_key: tuple[tuple[int, str], ...] | None = None
        self._labels: dict[str, Gtk.Label] = {}
        body = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        self._summary = Gtk.Label(label=arm_summary(None))
        self._summary.set_xalign(0.0)
        self._summary.set_margin_start(10)
        self._summary.set_margin_top(6)
        body.pack_start(self._summary, False, False, 0)
        grid = Gtk.Grid(column_spacing=12, row_spacing=7, margin=10)
        for row, (key, title) in enumerate((
            ("link", "수신"), ("motors", "모터"), ("joints", "관절"),
            ("detections", "탐지"),
        )):
            name = Gtk.Label(label=title)
            name.set_xalign(0.0)
            value = Gtk.Label(label="미수신(UNAVAILABLE)")
            value.set_xalign(0.0)
            value.set_line_wrap(True)
            grid.attach(name, 0, row, 1, 1)
            grid.attach(value, 1, row, 1, 1)
            self._labels[key] = value
        details = Gtk.Expander(label="상세")
        details.add(grid)
        body.pack_start(details, False, False, 0)
        self.add(body)
        GLib.timeout_add(200, self._refresh)

    def _report_temperature(self, snapshot: ArmTelemetrySnapshot) -> None:
        alerts = () if snapshot.dynamixel is None else tuple(
            (motor.id, temperature_state(motor.temperature_c))
            for motor in snapshot.dynamixel
            if temperature_state(motor.temperature_c) != "NORMAL"
        )
        if alerts == self._temperature_key:
            return
        self._temperature_key = alerts
        if not alerts or self._event_sink is None:
            return
        groups = []
        for state in ("CRIT", "WARN"):
            motor_ids = ", ".join(
                f"ID {motor_id}" for motor_id, alert_state in alerts
                if alert_state == state
            )
            if motor_ids:
                groups.append(f"{state}: {motor_ids}")
        self._event_sink("ARM", "temperature " + " · ".join(groups))

    def _refresh(self) -> bool:
        snapshot: ArmTelemetrySnapshot | None = self._receiver.latest()
        receive_age_s = (
            None
            if snapshot is None
            else time.monotonic() - snapshot.received_monotonic_s
        )
        summary_label = getattr(self, "_summary", None)
        if summary_label is not None:
            summary_label.set_text(arm_panel_summary(snapshot, receive_age_s))
        if snapshot is None:
            self._labels["link"].set_text(
                f"{freshness_korean('WAITING')} · UDP :{self._port}"
            )
            return True
        age_ms = receive_age_s * 1000.0
        if age_ms > 1000.0:
            self._labels["link"].set_text(
                f"{freshness_korean('STALE')} · {age_ms:.0f} ms"
            )
            return True
        self._labels["link"].set_text(
            f"{freshness_korean('LIVE')} · 순번 {snapshot.sequence} · {age_ms:.0f} ms"
        )
        motor_freshness = arm_source_freshness(snapshot.dynamixel_age_s)
        joints_freshness = arm_source_freshness(snapshot.joints_age_s)
        detections_freshness = arm_source_freshness(snapshot.detections_age_s)
        if motor_freshness != "LIVE":
            self._labels["motors"].set_text(freshness_korean(motor_freshness))
        elif snapshot.dynamixel is None:
            self._labels["motors"].set_text("미수신(UNAVAILABLE)")
        else:
            # Dynamixel current scaling is not confirmed by the arm team;
            # keep the console honest by displaying the unscaled raw value.
            self._labels["motors"].set_text("\n".join(
                f"ID {motor.id} · {motor.position_deg:+.1f}° · {motor.current} raw · "
                f"{motor.temperature_c}℃ [{temperature_state(motor.temperature_c)}]"
                for motor in snapshot.dynamixel
            ))
        if joints_freshness != "LIVE":
            self._labels["joints"].set_text(freshness_korean(joints_freshness))
        elif not snapshot.joint_names:
            self._labels["joints"].set_text("미수신(UNAVAILABLE)")
        else:
            self._labels["joints"].set_text(", ".join(
                f"{name} {math.degrees(position_rad):+.1f}°"
                for name, position_rad in zip(
                    snapshot.joint_names, snapshot.joint_position_rad,
                )
            ))
        self._labels["detections"].set_text(
            freshness_korean(detections_freshness)
        )
        if motor_freshness == "LIVE":
            self._report_temperature(snapshot)
        return True


class ChassisTelemetryPanel(Gtk.Frame):
    """Read-only chassis owner snapshot; never probes CAN directly."""
    def __init__(self, receiver: LatestTelemetryReceiver, port: int,
                 event_sink: Callable[[str, str], None] | None = None) -> None:
        super().__init__(label="차대")
        self._receiver = receiver
        self._port = port
        self._event_sink = event_sink
        self._wheel_health_key: tuple[int, int, int] | None = None
        self._labels: dict[str, Gtk.Label] = {}
        body = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        self._summary = Gtk.Label(label=chassis_summary(None))
        self._summary.set_xalign(0.0)
        self._summary.set_margin_start(10)
        self._summary.set_margin_top(6)
        body.pack_start(self._summary, False, False, 0)
        grid = Gtk.Grid(column_spacing=12, row_spacing=7, margin=10)
        for row, (key, title) in enumerate((("link", "수신"), ("odom", "주행계"),
                                             ("pose", "자세 x / y / yaw"),
                                             ("drive", "주행"), ("safety", "안전 / US-100"),
                                             ("feedback", "바퀴 피드백"), ("wheels", "바퀴별"),
                                             ("can", "CAN"), ("l515", "L515 게이트웨이"))):
            name = Gtk.Label(label=title)
            name.set_xalign(0.0)
            value = Gtk.Label(label="미수신(UNAVAILABLE)")
            value.set_xalign(0.0)
            value.set_line_wrap(True)
            grid.attach(name, 0, row, 1, 1)
            grid.attach(value, 1, row, 1, 1)
            self._labels[key] = value
        details = Gtk.Expander(label="상세")
        details.add(grid)
        body.pack_start(details, False, False, 0)
        self.add(body)
        GLib.timeout_add(200, self._refresh)

    def _refresh(self) -> bool:
        snapshot = self._receiver.latest()
        summary_label = getattr(self, "_summary", None)
        if summary_label is not None:
            summary_label.set_text(chassis_summary(snapshot))
        if snapshot is None:
            self._labels["link"].set_text(
                f"{freshness_korean('WAITING')} · UDP :{self._port}"
            )
            return True
        age_ms = (time.monotonic() - snapshot.received_monotonic_s) * 1000.0
        if age_ms > 1000.0:
            self._labels["link"].set_text(
                f"{freshness_korean('STALE')} · {age_ms:.0f} ms"
            )
            return True
        self._labels["link"].set_text(
            f"{freshness_korean('LIVE')} · 순번 {snapshot.sequence} · {age_ms:.0f} ms"
        )
        self._labels["odom"].set_text(snapshot.odometry_source)
        self._labels["pose"].set_text(
            f"{_format_number(snapshot.x_m, 'm')} / {_format_number(snapshot.y_m, 'm')} / "
            f"{_format_number(snapshot.yaw_rad, 'rad')}"
        )
        self._labels["drive"].set_text(snapshot.drive_state)
        if (
            snapshot.safety_status == "unavailable"
            or snapshot.safety_estop_required is None
        ):
            self._labels["safety"].set_text("미수신(UNAVAILABLE)")
        else:
            distance = _format_number(snapshot.safety_distance_mm, "mm")
            estop = (
                "ESTOP" if snapshot.safety_estop_required is True else "clear"
            )
            failures = ("N/A" if snapshot.safety_consecutive_failures is None
                        else str(snapshot.safety_consecutive_failures))
            detail = snapshot.safety_detail or "-"
            self._labels["safety"].set_text(
                f"{snapshot.safety_status} · {distance} · {estop} · 실패 {failures}\n{detail}"
            )
        if snapshot.wheel_count is None:
            self._labels["feedback"].set_text("미수신(UNAVAILABLE)")
        else:
            self._labels["feedback"].set_text(
                f"바퀴 {snapshot.wheel_count} · 고장 {snapshot.wheel_fault_count or 0} · "
                f"지연 {snapshot.wheel_stale_count or 0} · "
                f"축 {snapshot.wheel_axis_error_count or 0} · "
                f"조향 {snapshot.wheel_steer_fault_count or 0}"
            )
            health_key = (snapshot.wheel_stale_count or 0,
                          snapshot.wheel_axis_error_count or 0,
                          snapshot.wheel_steer_fault_count or 0)
            if health_key != self._wheel_health_key:
                self._wheel_health_key = health_key
                if self._event_sink is not None:
                    self._event_sink(
                        "WHEELS",
                        f"stale {health_key[0]} · axis error {health_key[1]} · steer fault {health_key[2]}",
                    )
        if snapshot.truncated and not snapshot.wheel_statuses:
            self._labels["wheels"].set_text("잘림(TRUNCATED) · 바퀴별 행 생략")
        elif not snapshot.wheel_statuses:
            self._labels["wheels"].set_text("미수신(UNAVAILABLE)")
        else:
            lines = []
            for wheel in snapshot.wheel_statuses:
                fault = (" 고장" if wheel.drive_axis_error or wheel.steer_fault else "")
                stale = " 지연" if wheel.stale else ""
                lines.append(
                    f"{wheel.name}: {wheel.mode} · {_format_number(wheel.drive_turns_per_s, 'r/s')} · "
                    f"{_format_number(wheel.steer_deg, 'deg')}{stale}{fault}"
                )
            self._labels["wheels"].set_text("\n".join(lines))
        self._labels["can"].set_text(snapshot.can_state)
        self._labels["l515"].set_text(
            f"{snapshot.l515_state} · {snapshot.l515_mode} · 원본 "
            f"{_format_number(snapshot.l515_color_hz, 'Hz')} / {_format_number(snapshot.l515_depth_hz, 'Hz')}\n"
            f"SRT 제출/전송/누락 {_format_number(snapshot.l515_submitted_hz, 'Hz')} / "
            f"{_format_number(snapshot.l515_sent_hz, 'Hz')} / {_format_number(snapshot.l515_drop_hz, 'Hz')}\n"
            f"정렬 Depth {_format_number(snapshot.l515_aligned_depth_age_ms, 'ms')} · "
            f"프로세스 {_format_number(snapshot.l515_process_cpu_percent, '% CPU')} / "
            f"{_format_rss(snapshot.l515_process_rss_bytes)}\n"
            f"ROS {_format_ros_rates(snapshot.l515_ros_topic_rates_hz)}\n"
            f"{snapshot.l515_detail or '-'}"
        )
        return True


class OpsPanel(Gtk.Frame):
    """Token-gated commands with inline two-step confirmation only."""

    def __init__(
        self,
        host: str,
        port: int,
        token_file: str,
        *,
        event_sink: Callable[[str, str], None],
        alert_sink: Callable[[str], None] | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        super().__init__(label="조작 (토큰 인증)")
        self._event_sink = event_sink
        self._alert_sink = alert_sink or (lambda _message: None)
        self._clock = clock
        self._client: ConsoleOpsClient | None = None
        self._flow: ConfirmFlow | None = None
        self._active_action: PanelAction | None = None
        self._hold_started_s: float | None = None
        self._pending_requests: dict[str, str] = {}
        self._latest_component_mask: dict[str, bool] | None = None
        self._latest_chassis_mode = "UNKNOWN"
        self._latest_estop_source = ""
        self._latest_estop_detail = ""
        self._latest_active_estop_sources: tuple[str, ...] = ()
        self._last_estop_cause_key: (
            tuple[str, str, tuple[str, ...]] | None
        ) = None
        self._latest_ack_text = "없음"
        self._action_buttons: dict[str, Gtk.Button] = {}

        body = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        body.set_border_width(8)
        self.add(body)

        token = self._read_token(token_file)
        if token is None:
            disabled = Gtk.Label(label="조작 토큰 없음 — 패널 비활성")
            disabled.set_xalign(0.0)
            disabled.set_line_wrap(True)
            body.pack_start(disabled, False, False, 0)
            return

        self._state_label = Gtk.Label(
            label="모드: 미수신(UNKNOWN) · 최근: 없음"
        )
        self._state_label.set_xalign(0.0)
        self._state_label.set_line_wrap(True)
        body.pack_start(self._state_label, False, False, 0)

        basic_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        advanced_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)

        for action in PANEL_ACTIONS:
            target = advanced_box if action.advanced else basic_box
            if action.gesture == GESTURE_SPACER:
                spacer = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
                spacer.pack_start(Gtk.Separator(), False, False, 0)
                note = Gtk.Label(label=action.label)
                note.set_xalign(0.0)
                note.set_line_wrap(True)
                spacer.pack_start(note, False, False, 0)
                target.pack_start(spacer, False, False, 3)
                continue

            button = Gtk.Button()
            if action.gesture == GESTURE_IMMEDIATE:
                emergency_label = Gtk.Label()
                emergency_label.set_markup(
                    '<span foreground="#b91c1c" weight="bold" size="large">'
                    + GLib.markup_escape_text(action.label)
                    + "</span>"
                )
                button.add(emergency_label)
                button.set_size_request(-1, 48)
            else:
                button.set_label(action.label)
            button.set_hexpand(True)
            if action.gesture == GESTURE_HOLD:
                button.connect("button-press-event", self._on_hold_press, action)
                button.connect("button-release-event", self._on_hold_release, action)
            elif action.gesture == GESTURE_IMMEDIATE:
                button.connect("clicked", self._on_immediate_clicked, action)
            else:
                button.connect("clicked", self._on_action_clicked, action)
            if action.action is not None:
                self._action_buttons[action.action] = button
            if action.bool_value_from_state is not None:
                if not mode_allows_action(action.action, "UNKNOWN"):
                    button.set_label(action.label + " · 대기에서만")
                button.set_sensitive(False)
            target.pack_start(button, False, False, 0)

        body.pack_start(basic_box, False, False, 0)
        advanced = Gtk.Expander(label="고급")
        advanced.set_expanded(False)
        advanced.add(advanced_box)
        body.pack_start(advanced, False, False, 0)

        self._confirm_strip = Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL,
            spacing=6,
        )
        self._confirm_strip.set_border_width(6)
        self._confirm_strip.set_no_show_all(True)
        self._confirm_copy = Gtk.Label()
        self._confirm_copy.set_xalign(0.0)
        self._confirm_copy.set_line_wrap(True)
        self._confirm_state = Gtk.Label()
        self._confirm_state.set_xalign(0.0)
        self._confirm_state.set_line_wrap(True)
        self._confirm_button = Gtk.Button(label="확인")
        self._confirm_button.connect("clicked", self._on_confirm_clicked)
        cancel = Gtk.Button(label="취소")
        cancel.connect("clicked", self._on_cancel_clicked)
        controls = Gtk.Box(spacing=6)
        controls.pack_start(self._confirm_button, True, True, 0)
        controls.pack_start(cancel, False, False, 0)
        self._confirm_strip.pack_start(self._confirm_copy, False, False, 0)
        self._confirm_strip.pack_start(self._confirm_state, False, False, 0)
        self._confirm_strip.pack_start(controls, False, False, 0)
        body.pack_start(self._confirm_strip, False, False, 4)

        self._client = ConsoleOpsClient(
            host,
            port,
            token,
            submit_sink=self._on_submit_response,
            state_sink=self._on_state,
        )
        self._flow = ConfirmFlow(
            clock=self._clock,
            state_provider=self._client.latest_state,
        )

    @staticmethod
    def _read_token(token_file: str) -> str | None:
        try:
            token = Path(token_file).expanduser().read_text(encoding="utf-8").strip()
        except OSError:
            return None
        return token or None

    @staticmethod
    def _state_text(state: dict) -> str:
        sources = state.get("active_estop_sources") or []
        source_text = ", ".join(str(source) for source in sources) or "없음"
        return (
            f"리비전 {state.get('revision', 'N/A')} · "
            f"권한 {state.get('authority_mode', 'UNKNOWN')} · "
            f"비상정지 {'래치됨(LATCHED)' if state.get('estop_latched') else '정상(CLEAR)'}\n"
            f"원인 {source_text} · "
            f"바퀴 정지 {state.get('wheels_stopped', 'UNKNOWN')}"
        )

    def _refresh_status_line(self) -> None:
        label = getattr(self, "_state_label", None)
        if label is None:
            return
        chassis_mode = getattr(self, "_latest_chassis_mode", "UNKNOWN")
        estop_source = getattr(self, "_latest_estop_source", "")
        estop_detail = getattr(self, "_latest_estop_detail", "")
        active_estop_sources = getattr(
            self, "_latest_active_estop_sources", ()
        )
        latest_ack = getattr(self, "_latest_ack_text", "없음")
        label.set_text(format_ops_status_line(
            chassis_mode,
            latest_ack,
            estop_source,
            estop_detail,
            active_estop_sources,
        ))

    def _emit(self, message: str) -> None:
        self._event_sink("OPS", message)

    def _begin(self, action: PanelAction) -> bool:
        if self._flow is None:
            self._emit(f"{action.action}: rejected — panel disabled")
            return False
        try:
            # 문자열이 아니라 행 객체를 넘긴다 — arm_lock_override 는 같은
            # action 이름의 걸기/취소 행 쌍이라 문자열 조회로는 구분 불가.
            pending = self._flow.begin(action)
        except (RuntimeError, ValueError) as exc:
            self._emit(f"{action.action}: rejected — {exc}")
            self._hide_confirmation()
            return False
        self._active_action = action
        confirm_text = action.confirm_text
        if action.bool_value_from_state is not None:
            enabled = pending.params.get("data") is True
            if action.action != "us100_enable" or enabled:
                direction = "켭니다" if enabled else "끕니다"
                confirm_text = f"{action.label}을 {direction}. 계속합니까?"
        self._confirm_copy.set_markup(
            "<b>{}</b>".format(GLib.markup_escape_text(confirm_text))
        )
        self._confirm_state.set_text(
            "상태 스냅샷: " + self._state_text(pending.state_snapshot)
        )
        self._confirm_button.set_label(f"확인: {action.label}")
        self._confirm_button.set_visible(action.gesture != GESTURE_HOLD)
        self._confirm_strip.set_no_show_all(False)
        self._confirm_strip.show_all()
        if action.gesture == GESTURE_HOLD:
            self._confirm_button.hide()
        return True

    def _on_action_clicked(self, _button: Gtk.Button, action: PanelAction) -> None:
        self._begin(action)

    def _on_immediate_clicked(
        self,
        _button: Gtk.Button,
        action: PanelAction,
    ) -> None:
        flow = self._flow
        if flow is None:
            self._emit(f"{action.action}: rejected — panel disabled")
            return
        flow.reset()
        self._submit({"action": action.action, "params": {}})

    def _on_hold_press(
        self,
        _button: Gtk.Button,
        _event: Gdk.EventButton,
        action: PanelAction,
    ) -> bool:
        if not self._begin(action):
            self._hold_started_s = None
            return False
        self._hold_started_s = self._clock()
        return False

    def _on_hold_release(
        self,
        _button: Gtk.Button,
        _event: Gdk.EventButton,
        action: PanelAction,
    ) -> bool:
        started_s = self._hold_started_s
        self._hold_started_s = None
        if started_s is None or self._flow is None:
            return False
        held_s = max(0.0, self._clock() - started_s)
        submit_kwargs = self._flow.confirm(action, held_s=held_s)
        if submit_kwargs is None:
            self._flow.reset()
            self._emit(
                f"{action.action}: rejected — hold {held_s:.2f} s; "
                "1.50 s and an unchanged revision are required"
            )
            self._hide_confirmation()
            return False
        self._submit(submit_kwargs)
        return False

    def _on_confirm_clicked(self, _button: Gtk.Button) -> None:
        action = self._active_action
        if action is None or self._flow is None:
            return
        submit_kwargs = self._flow.confirm(action)
        if submit_kwargs is None:
            self._emit(
                f"{action.action}: rejected — state revision changed; begin again"
            )
            self._hide_confirmation()
            return
        self._submit(submit_kwargs)

    def _submit(self, submit_kwargs: dict) -> None:
        action = str(submit_kwargs["action"])
        if self._client is None:
            self._emit(f"{action}: rejected — ops client unavailable")
            self._hide_confirmation()
            return
        try:
            request_id = self._client.submit(**submit_kwargs)
        except RuntimeError as exc:
            message = f"{action}: 전송 실패 · {exc}"
            self._emit(message)
            self._alert_sink(message)
        else:
            self._pending_requests[request_id] = action
            self._emit(f"{action}: submitted · request {request_id}")
        self._hide_confirmation()

    def _on_cancel_clicked(self, _button: Gtk.Button) -> None:
        if self._flow is not None:
            self._flow.reset()
        self._emit("confirmation cancelled")
        self._hide_confirmation()

    def _hide_confirmation(self) -> None:
        self._active_action = None
        self._hold_started_s = None
        strip = getattr(self, "_confirm_strip", None)
        if strip is not None:
            strip.hide()
            strip.set_no_show_all(True)

    def _on_state(self, state: dict | None) -> None:
        if state is None:
            state = {}
            if self._flow is not None:
                self._flow.reset()
            self._hide_confirmation()
        self._latest_component_mask = component_mask_from_state(state)
        chassis_mode = str(state.get("chassis_mode", "UNKNOWN"))
        estop_source = str(state.get("estop_source", ""))
        estop_detail = str(state.get("estop_detail", ""))
        active_estop_sources = tuple(
            str(source)
            for source in (state.get("active_estop_sources", ()) or ())
        )
        self._latest_chassis_mode = chassis_mode
        self._latest_estop_source = estop_source
        self._latest_estop_detail = estop_detail
        self._latest_active_estop_sources = active_estop_sources
        event_key, event_message = next_estop_cause_event(
            getattr(self, "_last_estop_cause_key", None),
            chassis_mode=chassis_mode,
            estop_source=estop_source,
            estop_detail=estop_detail,
            active_estop_sources=active_estop_sources,
        )
        self._last_estop_cause_key = event_key
        if event_message is not None:
            self._event_sink("안전", event_message)

        for action in PANEL_ACTIONS:
            if action.action is None or action.bool_value_from_state is None:
                continue
            button = self._action_buttons[action.action]
            allowed = mode_allows_action(action.action, chassis_mode)
            gate_hint = " · 대기에서만" if not allowed else ""
            try:
                next_enabled = action.bool_value_from_state(state)
            except RuntimeError:
                button.set_label(action.label + gate_hint)
                button.set_sensitive(False)
            else:
                current_state = OFF_LABEL if next_enabled else ON_LABEL
                button.set_label(
                    f"{action.label} [{current_state}]{gate_hint}"
                )
                button.set_sensitive(allowed)
        self._refresh_status_line()

    def latest_component_mask(self) -> dict[str, bool] | None:
        if self._latest_component_mask is None:
            return None
        return dict(self._latest_component_mask)

    def latest_chassis_mode(self) -> str:
        return self._latest_chassis_mode

    def ops_available(self) -> bool:
        return self._client is not None and self._flow is not None

    def link_ready(self) -> bool:
        """Return True only when token-gated controls have a live ops link."""
        return (
            self.ops_available()
            and self._client is not None
            and self._client.connected
        )

    def trigger_estop(self) -> None:
        """Use the same immediate, token-gated path as the Ops-page button."""
        action = next(
            item for item in PANEL_ACTIONS if item.action == "estop"
        )
        self._on_immediate_clicked(self._action_buttons.get("estop"), action)

    def _on_submit_response(self, response: dict) -> None:
        request_id = str(response.get("request_id", "unknown"))
        action = self._pending_requests.get(request_id, "unknown action")
        status = str(response.get("status", "UNKNOWN"))
        detail = str(response.get("detail", ""))
        message = f"{action}: {status} · request {request_id}"
        if detail:
            message += f" · {detail}"
        self._emit(message)
        self._latest_ack_text = ack_korean(status, detail)
        self._refresh_status_line()
        if (
            status == "OUTCOME_UNKNOWN"
            or status.startswith("FINAL_") and status != "FINAL_SUCCESS"
        ):
            self._alert_sink(message)
        if status.startswith("FINAL_") or status == "OUTCOME_UNKNOWN":
            self._pending_requests.pop(request_id, None)

    def close(self) -> None:
        if self._client is not None:
            self._client.close()


class DiagnosticCard(Gtk.Box):
    """Compact public rows with a separately controlled developer section."""

    def __init__(
        self, title: str, public_rows: tuple[tuple[str, str], ...],
        developer_rows: tuple[tuple[str, str], ...],
    ) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=7)
        _style(self, "system-card", "diagnostic-card")
        heading = Gtk.Label(label=title)
        heading.set_xalign(0.0)
        _style(heading, "section-title")
        self.pack_start(heading, False, False, 0)
        self.values: dict[str, Gtk.Label] = {}
        for key, label in public_rows:
            row = Gtk.Box(spacing=8)
            name = Gtk.Label(label=label)
            name.set_xalign(0.0)
            name.set_line_wrap(True)
            value = Gtk.Label(label="정보 없음")
            value.set_xalign(1.0)
            value.set_line_wrap(True)
            _style(name, "system-name")
            _style(value, "preparation-status-value")
            row.pack_start(name, True, True, 0)
            row.pack_end(value, False, False, 0)
            self.pack_start(row, False, False, 0)
            self.values[key] = value
        self.developer_box = Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL, spacing=4,
        )
        _style(self.developer_box, "developer-box")
        self.developer_values: dict[str, Gtk.Label] = {}
        for key, label in developer_rows:
            row = Gtk.Box(spacing=8)
            name = Gtk.Label(label=label)
            name.set_xalign(0.0)
            value = Gtk.Label(label="없음")
            value.set_xalign(1.0)
            value.set_selectable(True)
            row.pack_start(name, True, True, 0)
            row.pack_end(value, False, False, 0)
            self.developer_box.pack_start(row, False, False, 0)
            self.developer_values[key] = value
        self.developer_box.set_no_show_all(True)
        self.developer_box.hide()
        self.pack_end(self.developer_box, False, False, 0)

    def show_developer(self, visible: bool) -> None:
        self.developer_box.set_no_show_all(not visible)
        if visible:
            self.developer_box.show_all()
        else:
            self.developer_box.hide()


class OperatorConsole(Gtk.Window):
    def __init__(self, host: str, d435_port: int, l515_port: int, metadata_port: int,
                 latency_ms: int, telemetry_port: int, chassis_telemetry_port: int,
                 arm_telemetry_port: int = 5007,
                 environment_telemetry_port: int = 5008,
                 ops_host: str | None = None, ops_port: int = 9001,
                 ops_token_file: str = DEFAULT_OPS_TOKEN_FILE,
                 smoke_probe_file: str | None = None,
                 input_source: str = "LIVE") -> None:
        super().__init__(title="파워트레인 운영 콘솔")
        _install_console_css()
        self._smoke_probe_path = (
            Path(smoke_probe_file) if smoke_probe_file else None
        )
        # Compact laptop-first default. The user can maximize or use F11 when
        # the full display is useful; status chips wrap at narrower widths.
        self.set_default_size(1100, 680)
        self.set_position(Gtk.WindowPosition.CENTER)
        self.connect("destroy", self._on_destroy)
        self.connect("key-press-event", self._on_key_press)
        self._fullscreen = False
        self._metadata_receiver = LatestMetadataReceiver(metadata_port)
        self._display_target_tracker = DisplayTargetTracker()
        self._telemetry_receiver = LatestTelemetryReceiver(telemetry_port)
        self._chassis_receiver = LatestTelemetryReceiver(chassis_telemetry_port)
        self._arm_receiver = LatestArmTelemetryReceiver(arm_telemetry_port)
        self._environment_receiver = LatestEnvironmentTelemetryReceiver(
            environment_telemetry_port,
        )
        self._events = EventLog()
        self._mission_events = EventLog()
        self._d435 = VideoPanel("작업 카메라", host, d435_port, latency_ms,
                                metadata_receiver=self._metadata_receiver,
                                target_tracker=self._display_target_tracker,
                                event_sink=self._add_event)
        self._l515 = VideoPanel("전방 카메라", host, l515_port, latency_ms,
                                event_sink=self._add_event)
        # Fixed judge layout: forward view owns the stage; work view is 848:480 PiP.
        videos = Gtk.Overlay()
        videos.set_hexpand(True)
        videos.set_vexpand(True)
        videos.add(self._l515)
        pip_frame = Gtk.Frame()
        pip_frame.set_shadow_type(Gtk.ShadowType.NONE)
        pip_frame.set_halign(Gtk.Align.END)
        pip_frame.set_valign(Gtk.Align.END)
        pip_frame.set_margin_end(18)
        pip_frame.set_margin_bottom(18)
        _style(pip_frame, "pip-frame")
        self._pip_slot = FixedSizeSlot()
        self._pip_slot.add(self._d435)
        self._swap_button = Gtk.Button(label="화면 전환")
        self._swap_button.set_halign(Gtk.Align.END)
        self._swap_button.set_valign(Gtk.Align.CENTER)
        self._swap_button.set_size_request(88, 28)
        _style(self._swap_button, "pip-swap-button")
        self._swap_button.set_tooltip_text(
            "이 작은 카메라 영상을 큰 화면으로 전환합니다 (단축키 V)"
        )
        self._swap_button.connect(
            "clicked",
            lambda _button: self.swap_camera_views(
                self._d435 if self._main_video is self._l515 else self._l515,
                user_initiated=True,
            ),
        )
        self._d435.set_header_action(self._swap_button)
        pip_frame.add(self._pip_slot)
        videos.add_overlay(pip_frame)
        self._watermark = JetInWatermark()
        self._watermark.set_halign(Gtk.Align.END)
        self._watermark.set_valign(Gtk.Align.START)
        # The wordmark SVG has ~24 px of intrinsic right whitespace; a 12 px
        # container margin moves the visible ink right while preserving a
        # >=24 px safe edge at supported viewport sizes.
        self._watermark.set_margin_end(12)
        self._watermark.set_margin_top(20)
        videos.add_overlay(self._watermark)
        videos.set_overlay_pass_through(self._watermark, True)
        display_options = Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL, spacing=3,
        )
        _style(display_options, "display-options")
        display_title = Gtk.Label(label="영상 표시")
        display_title.set_xalign(0.0)
        _style(display_title, "display-options-title")
        self._show_objects = Gtk.CheckButton(
            label="인식 대상"
        )
        self._show_distance = Gtk.CheckButton(label="대상 거리")
        self.overlay_view_state = {
            "show_detection": True,
            "show_distance": True,
        }
        self._show_objects.set_active(True)
        self._show_distance.set_active(True)
        _style(self._show_objects, "display-option")
        _style(self._show_distance, "display-option")
        self._show_objects.connect("toggled", self._on_display_option_toggled)
        self._show_distance.connect("toggled", self._on_display_option_toggled)
        display_options.pack_start(display_title, False, False, 0)
        display_options.pack_start(self._show_objects, False, False, 0)
        display_options.pack_start(self._show_distance, False, False, 0)
        self._display_options = display_options
        videos.connect("size-allocate", self._on_video_area_allocated)
        self._videos = videos
        self._pip_frame = pip_frame
        self._main_video = self._l515
        self._l515.set_role("MAIN")
        self._d435.set_role("SUB")
        self._health = Gtk.Box(spacing=14)
        _style(self._health, "health-strip")
        # Keep only states that affect an operator's immediate go/no-go decision.
        # Camera/YOLO/arm detail remains visible in the mission/system pages.
        chip_names = ("network", "power", "camera", "safety")
        self._health_chips: dict[str, Gtk.Label] = {}
        self._health_dots: dict[str, Gtk.Label] = {}
        for chip_name in chip_names:
            chip = Gtk.Label(label="대기")
            chip.set_xalign(0.0)
            dot = Gtk.Label(label="")
            dot.set_size_request(7, 7)
            _style(dot, "status-dot", "status-muted")
            item = Gtk.Box(spacing=6)
            item.set_valign(Gtk.Align.CENTER)
            _style(item, "status-item")
            item.pack_start(dot, False, False, 0)
            item.pack_start(chip, False, False, 0)
            _style(chip, "status-chip")
            self._health_chips[chip_name] = chip
            self._health_dots[chip_name] = dot
            self._health.pack_start(item, False, False, 0)
        self._last_safety_banner: str | None = None
        self._last_l515_transport: str | None = None
        layout = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)

        topbar = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        _style(topbar, "topbar")
        title_row = Gtk.Box(spacing=12)
        title_row.set_margin_end(4)
        title_copy = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=1)
        title_copy.set_margin_top(9)
        title = Gtk.Label(label="재난 대응 로봇 관제 시스템")
        title.set_xalign(0.0)
        _style(title, "brand")
        eyebrow = Gtk.Label(label="실시간 임무 수행 및 안전 상태 확인")
        eyebrow.set_xalign(0.0)
        _style(eyebrow, "eyebrow")
        eyebrow.set_no_show_all(True)
        eyebrow.hide()
        title_copy.pack_start(title, False, False, 0)
        title_copy.pack_start(eyebrow, False, False, 0)
        title_row.pack_start(title_copy, False, False, 0)
        title_row.pack_start(Gtk.Box(), True, True, 0)
        self._global_estop = Gtk.Button()
        self._global_estop.set_size_request(124, 42)
        estop_content = Gtk.Box(spacing=9)
        estop_content.set_halign(Gtk.Align.CENTER)
        estop_content.set_valign(Gtk.Align.CENTER)
        estop_symbol = Gtk.Label(label="!")
        estop_symbol.set_size_request(24, 24)
        estop_symbol.set_halign(Gtk.Align.CENTER)
        estop_symbol.set_valign(Gtk.Align.CENTER)
        _style(estop_symbol, "estop-symbol")
        estop_copy = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        estop_title = Gtk.Label(label="긴급 정지")
        _style(estop_title, "estop-title")
        estop_subtitle = Gtk.Label(label="E-STOP")
        _style(estop_subtitle, "estop-subtitle")
        estop_copy.pack_start(estop_title, False, False, 0)
        estop_copy.pack_start(estop_subtitle, False, False, 0)
        estop_content.pack_start(estop_symbol, False, False, 0)
        estop_content.pack_start(estop_copy, False, False, 0)
        self._global_estop.add(estop_content)
        self._global_estop.set_tooltip_text(
            "확인 없이 즉시 토큰 인증 비상정지 명령을 전송합니다"
        )
        _style(self._global_estop, "global-estop")
        self._global_estop.connect(
            "clicked", lambda _button: self._ops_panel.trigger_estop()
        )
        self._global_estop.set_margin_start(12)
        self._global_estop.set_margin_end(20)
        title_row.pack_end(self._global_estop, False, False, 0)
        title_row.pack_end(self._health, False, False, 6)
        topbar.pack_start(title_row, False, False, 0)
        self._estop_availability_warning = Gtk.Label()
        self._estop_availability_warning.set_xalign(0.0)
        self._estop_availability_warning.set_no_show_all(True)
        _style(self._estop_availability_warning, "top-warning")
        topbar.pack_start(
            self._estop_availability_warning, False, False, 2,
        )
        self._alert_label = Gtk.Label()
        self._alert_label.set_xalign(0.0)
        self._alert_label.set_line_wrap(True)
        self._alert_label.set_no_show_all(True)
        _style(self._alert_label, "top-alert")
        topbar.pack_start(self._alert_label, False, False, 2)
        self._alert_serial = 0
        self._readiness_title = Gtk.Label(label="시스템 점검 중")
        self._readiness_detail = Gtk.Label(
            label="통신 및 안전 장치 연결 확인 중"
        )
        layout.pack_start(topbar, False, False, 0)

        telemetry = TelemetryPanel(self._telemetry_receiver, telemetry_port,
                                   event_sink=self._add_event)
        chassis = ChassisTelemetryPanel(self._chassis_receiver, chassis_telemetry_port,
                                        event_sink=self._add_event)
        arm = ArmTelemetryPanel(self._arm_receiver, arm_telemetry_port,
                                event_sink=self._add_event)
        for panel in (telemetry, chassis, arm):
            _style(panel, "card")
        self._ops_panel = OpsPanel(
            host if ops_host is None else ops_host,
            ops_port,
            ops_token_file,
            event_sink=self._add_event,
            alert_sink=self._show_alert,
        )
        self._refresh_estop_availability()
        _style(self._ops_panel, "danger-card")

        mission_page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        _style(mission_page, "page", "mission-page")
        mission_body = Gtk.Box(spacing=12)
        mission_body.set_hexpand(True)
        mission_body.set_vexpand(True)
        mission_body.connect("size-allocate", self._on_mission_body_allocated)

        # The video keeps only a compact camera identity/status label.
        mission_top = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        mission_top.set_halign(Gtk.Align.START)
        mission_top.set_valign(Gtk.Align.START)
        mission_top.set_margin_start(20)
        mission_top.set_margin_top(20)
        camera_label_row = Gtk.Box(spacing=7)
        _style(camera_label_row, "main-camera-label")
        self._main_camera_dot = Gtk.Label(label="")
        self._main_camera_dot.set_size_request(7, 7)
        _style(self._main_camera_dot, "status-dot", "status-muted")
        self._main_camera_label = Gtk.Label(label="전방 화면")
        _style(self._main_camera_label, "camera-name")
        self._main_camera_state_label = Gtk.Label(label="연결 대기")
        _style(self._main_camera_state_label, "camera-connection-label")
        camera_label_row.pack_start(self._main_camera_dot, False, False, 0)
        camera_label_row.pack_start(self._main_camera_label, False, False, 0)
        camera_label_row.pack_start(
            self._main_camera_state_label, False, False, 0,
        )
        mission_top.pack_start(camera_label_row, False, False, 0)
        videos.add_overlay(mission_top)
        self._mission_top = mission_top

        rail = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        rail.set_hexpand(False)
        rail.set_vexpand(True)
        _style(rail, "mission-rail")
        self._mission_rail = rail
        rail_heading = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        rail_heading_ko = Gtk.Label(label="실시간 운용")
        rail_heading_ko.set_xalign(0.0)
        _style(rail_heading_ko, "rail-heading-ko")
        rail_heading.pack_start(rail_heading_ko, False, False, 0)
        # The approved layout starts directly with the device card.
        # Keep no legacy mission heading above it.

        system_check = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=1)
        _style(system_check, "rail-section")
        system_check_heading = Gtk.Box(spacing=8)
        check_title = Gtk.Label(label="주요 장치")
        check_title.set_xalign(0.0)
        _style(check_title, "rail-section-title")
        self._readiness_count = Gtk.Label(label="0 / 4")
        self._readiness_count.set_xalign(1.0)
        _style(self._readiness_count, "rail-section-count")
        system_check_heading.pack_start(check_title, True, True, 0)
        system_check_heading.pack_end(self._readiness_count, False, False, 0)
        system_check.pack_start(system_check_heading, False, False, 2)
        self._preparation_status: dict[str, tuple[Gtk.Label, Gtk.Label]] = {}
        device_specs = (
            ("front", "전방 화면", "연결 중"),
            ("work", "작업 화면", "연결 중"),
            ("drive", "주행 시스템", "정보 없음"),
            ("safety", "안전 장치", "정보 없음"),
        )
        for index, (key, name, initial_value) in enumerate(device_specs):
            if index:
                divider = Gtk.Separator(orientation=Gtk.Orientation.HORIZONTAL)
                _style(divider, "rail-section-divider")
                system_check.pack_start(divider, False, False, 0)
            row = Gtk.Box(spacing=7)
            _style(row, "rail-device-row")
            dot = Gtk.Label(label="")
            dot.set_size_request(7, 7)
            initial_tone = "prep-connecting" if index < 2 else "prep-offline"
            _style(dot, "preparation-status-dot", initial_tone)
            name_label = Gtk.Label(label=name)
            name_label.set_xalign(0.0)
            _style(name_label, "rail-device-name")
            value = Gtk.Label(label=initial_value)
            value.set_xalign(1.0)
            _style(value, "preparation-status-value", initial_tone)
            row.pack_start(dot, False, False, 0)
            row.pack_start(name_label, True, True, 0)
            row.pack_end(value, False, False, 0)
            system_check.pack_start(row, False, False, 0)
            self._preparation_status[key] = (dot, value)
        # The same four states are already always visible in the top health
        # strip.  Keep these widgets as the readiness data model, but do not
        # spend scarce mission-rail height on a duplicate device list.
        system_check.set_no_show_all(True)
        system_check.hide()

        preparation = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
        _style(preparation, "rail-preparation")
        preparation_title = Gtk.Label(label="현재 임무")
        preparation_title.set_xalign(0.0)
        _style(preparation_title, "rail-title")
        preparation_detail = Gtk.Label(label="현재 단계    운용 준비")
        preparation_detail.set_xalign(0.0)
        preparation_detail.set_line_wrap(True)
        _style(preparation_detail, "rail-description")
        preparation_target = Gtk.Label(label="작업 대상    대상 탐지 대기")
        preparation_target.set_xalign(0.0)
        preparation_target.set_line_wrap(True)
        _style(preparation_target, "rail-description")
        preparation.pack_start(preparation_title, False, False, 0)
        preparation.pack_start(preparation_detail, False, False, 0)
        self._rail_preparation = preparation
        self._rail_preparation_detail = preparation_detail
        self._rail_preparation_target = preparation_target

        technology = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
        _style(technology, "rail-technology")
        technology_title = Gtk.Label(label="구현 기술")
        technology_title.set_xalign(0.0)
        _style(technology_title, "rail-title")
        technology.pack_start(technology_title, False, False, 0)
        for name, detail, tone in (
            ("4륜 독립 구동·조향", "험지 기동과 제자리 방향 전환", "drive"),
            ("RGB-D 인공지능 인식", "물체 종류·방향·거리 동시 판단", "vision"),
            ("독립 충돌 방지", "US-100 감지와 즉시 정지", "safety"),
            ("다관절 작업 장치", "인식 대상 접근·파지 작업", "arm"),
        ):
            row = Gtk.Box(spacing=9)
            _style(row, "technology-row", f"technology-{tone}")
            marker = Gtk.Label(label="")
            marker.set_size_request(5, 30)
            _style(marker, "technology-marker")
            copy = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
            name_label = Gtk.Label(label=name)
            name_label.set_xalign(0.0)
            detail_label = Gtk.Label(label=detail)
            detail_label.set_xalign(0.0)
            detail_label.set_ellipsize(Pango.EllipsizeMode.END)
            _style(name_label, "technology-name")
            _style(detail_label, "technology-detail")
            copy.pack_start(name_label, False, False, 0)
            copy.pack_start(detail_label, False, False, 0)
            row.pack_start(marker, False, False, 0)
            row.pack_start(copy, True, True, 0)
            technology.pack_start(row, False, False, 0)
        self._rail_technology = technology

        # These are live view controls, not explanatory content. Keep them in
        # the compact rail so judges/operators can verify the AI overlay.
        rail.pack_end(display_options, False, False, 0)

        rail_data = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=11)
        rail_data.set_no_show_all(False)
        self._mission_metrics = {}
        self._mission_metric_rows: dict[str, Gtk.Box] = {}
        for key, heading in (
            ("speed", "평균 속도"),
            ("target", "최신 인식"),
            ("distance", "대상 거리"),
        ):
            row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
            _style(row, "rail-data-row")
            row_heading = Gtk.Label(label=heading)
            row_heading.set_xalign(0.0)
            _style(row_heading, "rail-data-label")
            row_value = Gtk.Label(label="")
            row_value.set_xalign(0.0)
            row_value.set_line_wrap(False)
            _style(row_value, "rail-data-value", f"rail-data-{key}")
            row.pack_start(row_heading, False, False, 0)
            row.pack_start(row_value, False, False, 0)
            rail_data.pack_start(row, False, False, 0)
            self._mission_metrics[key] = row_value
            self._mission_metric_rows[key] = row

        tool_row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
        _style(tool_row, "end-effector-summary")
        tool_header = Gtk.Box(spacing=8)
        tool_heading = Gtk.Label(label="로봇팔 · 도구")
        tool_heading.set_xalign(0.0)
        _style(tool_heading, "end-effector-summary-title")
        self._mission_tool_state = Gtk.Label(label="선택 대기")
        self._mission_tool_state.set_xalign(1.0)
        _style(
            self._mission_tool_state,
            "end-effector-summary-state", "status-muted",
        )
        tool_header.pack_start(tool_heading, True, True, 0)
        tool_header.pack_end(self._mission_tool_state, False, False, 0)
        self._mission_tool_selector = Gtk.ComboBoxText()
        for tool in ("미확인", *END_EFFECTOR_PURPOSES.keys()):
            self._mission_tool_selector.append_text(tool)
        self._mission_tool_selector.set_active(0)
        self._mission_tool_selector.set_tooltip_text(
            "작업에 사용할 엔드이펙터를 선택합니다. 로봇 적용은 제어 연동 후 활성화됩니다."
        )
        _style(self._mission_tool_selector, "rail-tool-selector")
        self._mission_tool_purpose = Gtk.Label(label="사용할 이펙터를 선택하세요")
        self._mission_tool_purpose.set_xalign(0.0)
        self._mission_tool_purpose.set_line_wrap(True)
        self._mission_tool_purpose.set_max_width_chars(34)
        _style(self._mission_tool_purpose, "end-effector-summary-purpose")
        self._mission_tool_reading = Gtk.Label(label="상세 데이터 없음")
        self._mission_tool_reading.set_xalign(0.0)
        self._mission_tool_reading.set_ellipsize(Pango.EllipsizeMode.END)
        _style(self._mission_tool_reading, "end-effector-summary-reading")
        arm_summary_grid = Gtk.Grid(column_spacing=7, row_spacing=5)
        arm_summary_grid.set_column_homogeneous(True)
        self._mission_arm_mode = Gtk.Label(label="연동 예정")
        self._mission_arm_mode.set_xalign(0.0)
        self._mission_arm_load = Gtk.Label(label="정보 없음")
        self._mission_arm_load.set_xalign(0.0)
        for column, heading_text, value in (
            (0, "조종 모드", self._mission_arm_mode),
            (1, "관절 부하", self._mission_arm_load),
        ):
            summary = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
            summary_heading = Gtk.Label(label=heading_text)
            summary_heading.set_xalign(0.0)
            _style(summary_heading, "end-effector-arm-label")
            _style(value, "end-effector-arm-value")
            summary.pack_start(summary_heading, False, False, 0)
            summary.pack_start(value, False, False, 0)
            arm_summary_grid.attach(summary, column, 0, 1, 1)
        self._mission_tool_detail = Gtk.Button(label="선택한 이펙터 상세 보기  ↗")
        self._mission_tool_detail.set_tooltip_text(
            "선택한 엔드이펙터의 상세 정보를 별도 창으로 엽니다"
        )
        _style(self._mission_tool_detail, "end-effector-detail-button")
        self._mission_tool_detail.connect(
            "clicked", lambda _button: self._show_end_effector_popup(),
        )
        tool_row.pack_start(tool_header, False, False, 0)
        tool_row.pack_start(self._mission_tool_selector, False, False, 0)
        tool_row.pack_start(self._mission_tool_purpose, False, False, 0)
        tool_row.pack_start(self._mission_tool_reading, False, False, 0)
        tool_row.pack_start(arm_summary_grid, False, False, 0)
        tool_row.pack_start(self._mission_tool_detail, False, False, 0)
        rail_data.pack_start(tool_row, False, False, 0)
        self._mission_metric_rows["tool"] = tool_row
        rail.pack_start(rail_data, False, False, 0)
        rail_data.show_all()
        self._rail_data = rail_data

        self._mission_metrics["safety"] = self._preparation_status["safety"][1]

        mission_body.pack_start(videos, True, True, 0)
        mission_body.pack_end(rail, False, False, 0)
        self._mission_body = mission_body
        mission_page.pack_start(mission_body, True, True, 0)
        mission_event_expander, self._mission_event_latest = (
            self._make_event_expander(self._mission_events)
        )
        self._mission_event_expander = mission_event_expander
        mission_page.pack_end(mission_event_expander, False, True, 0)

        systems_page = CompetitionStatusDashboard(
            input_source=input_source,
            power_port=telemetry_port,
            chassis_port=chassis_telemetry_port,
            arm_port=arm_telemetry_port,
            metadata_port=metadata_port,
        )
        self._robot_status = systems_page
        self._environment_status = EnvironmentSensorDashboard(
            port=environment_telemetry_port,
        )
        self._build_end_effector_popup()
        self._syncing_end_effector_selectors = False
        self._mission_tool_selector.connect(
            "changed", self._on_mission_end_effector_changed,
        )
        self._robot_status.connect_end_effector_changed(
            self._on_status_end_effector_changed,
        )
        systems_scroll = Gtk.ScrolledWindow()
        systems_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        systems_scroll.add(systems_page)
        self._systems_scroll = systems_scroll

        ops_page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        ops_page.set_border_width(14)
        ops_heading = Gtk.Label(label="안전 조작")
        ops_heading.set_xalign(0.0)
        _style(ops_heading, "section-title")
        ops_note = Gtk.Label(
            label="모든 명령은 역할 토큰과 상태 재검증을 통과해야 합니다. "
                  "비상정지를 제외한 동작은 확인 절차를 거칩니다."
        )
        ops_note.set_xalign(0.0)
        ops_note.set_line_wrap(True)
        _style(ops_note, "muted")
        ops_page.pack_start(ops_heading, False, False, 0)
        ops_page.pack_start(ops_note, False, False, 0)
        ops_page.pack_start(self._ops_panel, False, False, 0)

        stack = Gtk.Stack()
        stack.set_hhomogeneous(False)
        stack.set_vhomogeneous(False)
        # Navigation must remain immediate while two video sinks and telemetry
        # drawings are active.  Crossfade composites both full-size pages.
        stack.set_transition_type(Gtk.StackTransitionType.NONE)
        stack.set_transition_duration(0)
        stack.add_titled(mission_page, "mission", "실시간 화면")
        stack.add_titled(systems_scroll, "systems", "시스템 상태")
        # The token-gated controls remain implemented for a future maintenance
        # surface, but are not exposed in the judge-facing competition console.
        self._stack = stack
        switcher = Gtk.StackSwitcher()
        switcher.set_stack(stack)
        switcher.set_halign(Gtk.Align.START)
        switcher.set_margin_start(0)
        switcher.set_size_request(238, 34)
        nav = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        _style(nav, "nav")
        nav.pack_start(switcher, False, False, 0)
        layout.pack_start(nav, False, False, 0)
        layout.pack_start(stack, True, True, 0)
        event_expander = EventDrawer(self._events)
        self._event_latest = event_expander.latest
        self._event_expander = event_expander
        stack.connect("notify::visible-child-name", self._on_page_changed)
        layout.pack_start(event_expander, False, True, 0)
        # Preserve the original two-tab information architecture.  Navigation
        # stays in the header instead of introducing an unrelated side rail.
        self.add(layout)
        GLib.idle_add(
            lambda: (self._on_page_changed(self._stack, None), False)[1]
        )
        GLib.timeout_add(250, self._refresh_health)
        # Camera roles never change from connection/metadata/FSM callbacks.
        # Swapping is wired only from the explicit button and V shortcut.

    @staticmethod
    def _make_event_expander(events: EventLog) -> tuple[EventDrawer, Gtk.Label]:
        """Build the screenshot-matched event footer for one page."""
        drawer = EventDrawer(events)
        return drawer, drawer.latest

    def _add_event(self, source: str, message: str) -> None:
        """Keep the Mission footer and the other-tab footer identical."""
        self._events.add_event(source, message)
        self._mission_events.add_event(source, message)

    def _build_diagnostic_cards(self) -> tuple[DiagnosticCard, ...]:
        drive = DiagnosticCard(
            "주행 시스템",
            (("receive", "데이터 수신"), ("mode", "주행 모드"),
             ("drive", "구동 상태"), ("position", "위치 추정"),
             ("obstacle", "장애물 안전"), ("motor", "모터 통신")),
            (("pose", "x / y / yaw"), ("wheels", "개별 바퀴 상태"),
             ("can", "CAN 원본 상태"), ("channel", "수신 채널"),
             ("last", "마지막 수신")),
        )
        power = DiagnosticCard(
            "전원 시스템",
            (("receive", "데이터 수신"), ("soc", "배터리 잔량"),
             ("voltage", "전압"), ("current", "전류"),
             ("protection", "보호 상태"), ("device", "전원 장치 연결")),
            (("raw", "원본 상태"), ("rs485", "RS485 상태"),
             ("pdist", "PDIST80B 상태"), ("channel", "수신 채널"),
             ("last", "마지막 수신")),
        )
        safety = DiagnosticCard(
            "안전 장치",
            (("receive", "데이터 수신"), ("obstacle", "장애물 안전"),
             ("hold", "일시 정지"), ("estop", "비상 정지"),
             ("device", "안전 장치 연결"), ("last", "마지막 업데이트")),
            (("raw", "원본 안전 상태"), ("us100", "US-100 원본 상태"),
             ("channel", "수신 채널"), ("last_raw", "마지막 수신")),
        )
        front = DiagnosticCard(
            "전방 카메라",
            (("connection", "영상 연결"), ("receive", "영상 수신"),
             ("position", "현재 표시 위치"), ("frame", "마지막 프레임"),
             ("update", "업데이트 상태")),
            (("raw", "원본 상태"), ("port", "수신 채널"),
             ("last_raw", "마지막 프레임 원본")),
        )
        work = DiagnosticCard(
            "작업 카메라",
            (("connection", "영상 연결"), ("receive", "영상 수신"),
             ("position", "현재 표시 위치"), ("frame", "마지막 프레임"),
             ("update", "업데이트 상태")),
            (("raw", "원본 상태"), ("port", "수신 채널"),
             ("metadata", "AI 정보 채널"), ("last_raw", "마지막 프레임 원본")),
        )
        arm = DiagnosticCard(
            "로봇팔·작업",
            (("receive", "데이터 수신"), ("stage", "현재 작업 단계"),
             ("motion", "로봇팔 동작"), ("joints", "관절 상태"),
             ("tool", "장착 도구"), ("target", "대상 인식"),
             ("done", "작업 완료 신호")),
            (("fsm", "원본 FSM 상태"), ("motor", "모터 원본 상태"),
             ("joints", "개별 관절 값"), ("channel", "수신 채널"),
             ("last", "마지막 수신")),
        )
        front.developer_values["port"].set_text("SRT :5000")
        work.developer_values["port"].set_text("SRT :5002")
        work.developer_values["metadata"].set_text("UDP :5003")
        power.developer_values["channel"].set_text("UDP :5004")
        drive.developer_values["channel"].set_text("UDP :5005")
        safety.developer_values["channel"].set_text("UDP :5005")
        arm.developer_values["channel"].set_text("UDP :5007")
        self._diagnostic_by_name = {
            "drive": drive, "power": power, "safety": safety,
            "front_camera": front, "work_camera": work, "arm": arm,
        }
        return drive, power, safety, front, work, arm

    def _on_developer_toggle(
        self, switch: Gtk.Switch, _param: object,
    ) -> None:
        for card in self._diagnostic_cards:
            card.show_developer(switch.get_active())
        self._events.set_developer_visible(switch.get_active())
        self._mission_events.set_developer_visible(switch.get_active())

    def _on_summary_card_clicked(
        self, _widget: Gtk.Widget, event: Gdk.EventButton, key: str,
    ) -> bool:
        if event.button != 1:
            return False
        self._selected_diagnostic = key
        self._diagnostic_stack.set_visible_child_name(self._selected_diagnostic)
        for card_key, widget in self._summary_cards.items():
            context = widget.get_style_context()
            context.remove_class("selected")
            if card_key == key:
                context.add_class("selected")
        return True

    def _set_summary_card(
        self, key: str, status: str, reason: str = "",
    ) -> None:
        self._system_values[key].set_text(status)
        self._system_reasons[key].set_text(reason)
        tone = (
            "status-live" if status == "정상"
            else "status-bad" if status in {"연결 끊김", "비상 정지"}
            else "status-warn" if status in {"확인 필요", "일시 정지", "업데이트 지연"}
            else "status-muted"
        )
        context = self._system_dots[key].get_style_context()
        for candidate in ("status-live", "status-bad", "status-warn", "status-muted"):
            context.remove_class(candidate)
        context.add_class(tone)

    @staticmethod
    def _received_text(snapshot: object | None, state: str) -> str:
        if snapshot is None:
            return "없음"
        age = max(0.0, time.monotonic() - snapshot.received_monotonic_s)
        return f"{age:.1f}초 전" if state == "LIVE" else "업데이트 지연"

    def _refresh_diagnostics(
        self, power_snapshot: TelemetrySnapshot | None,
        chassis_snapshot: TelemetrySnapshot | None,
        arm_snapshot: ArmTelemetrySnapshot | None,
        power_state: str, chassis_state: str, arm_state: str,
        yolo_state: str, safety_public: str,
    ) -> None:
        drive = self._diagnostic_by_name["drive"]
        mission = mission_presentation(
            self._ops_panel.latest_chassis_mode(),
            "" if chassis_snapshot is None else chassis_snapshot.drive_state,
        )
        drive.values["receive"].set_text(public_freshness(
            chassis_state, waiting="수신 대기"))
        drive.values["mode"].set_text(mission.title)
        drive.values["drive"].set_text(
            "수신 대기" if chassis_snapshot is None
            else public_freshness(chassis_snapshot.drive_state))
        has_pose = chassis_snapshot is not None and any(value is not None for value in (
            chassis_snapshot.x_m, chassis_snapshot.y_m, chassis_snapshot.yaw_rad))
        drive.values["position"].set_text("정상" if has_pose else "정보 없음")
        drive.values["obstacle"].set_text(safety_public)
        drive.values["motor"].set_text(
            "수신 대기" if chassis_snapshot is None
            else public_freshness(chassis_snapshot.can_state))
        drive.developer_values["pose"].set_text(
            "없음" if not has_pose else
            f"{chassis_snapshot.x_m} / {chassis_snapshot.y_m} / {chassis_snapshot.yaw_rad}")
        drive.developer_values["wheels"].set_text(
            "없음" if chassis_snapshot is None else
            f"{len(chassis_snapshot.wheel_statuses)}개")
        drive.developer_values["can"].set_text(
            "없음" if chassis_snapshot is None else chassis_snapshot.can_state)
        drive.developer_values["last"].set_text(
            self._received_text(chassis_snapshot, chassis_state))

        power = self._diagnostic_by_name["power"]
        power.values["receive"].set_text(public_freshness(
            power_state, waiting="수신 대기"))
        power.values["soc"].set_text(
            "정보 없음" if power_snapshot is None or power_snapshot.pdist_soc_percent is None
            else f"{power_snapshot.pdist_soc_percent}%")
        power.values["voltage"].set_text(
            "정보 없음" if power_snapshot is None or power_snapshot.voltage_v is None
            else f"{power_snapshot.voltage_v:.1f} V")
        power.values["current"].set_text(
            "정보 없음" if power_snapshot is None or power_snapshot.current_a is None
            else f"{power_snapshot.current_a:.1f} A")
        flags = None if power_snapshot is None else (
            power_snapshot.pdist_battery_flags, power_snapshot.pdist_protection_flags)
        power.values["protection"].set_text(
            "수신 대기" if flags is None or flags == (None, None)
            else "정상" if flags == (0, 0) else "확인 필요")
        power.values["device"].set_text(
            "수신 대기" if power_snapshot is None else
            public_freshness(power_snapshot.rs485_state, waiting="수신 대기"))
        power.developer_values["raw"].set_text(power_state)
        power.developer_values["rs485"].set_text(
            "없음" if power_snapshot is None else power_snapshot.rs485_state)
        power.developer_values["pdist"].set_text("없음" if flags is None else str(flags))
        power.developer_values["last"].set_text(
            self._received_text(power_snapshot, power_state))

        safety = self._diagnostic_by_name["safety"]
        raw_safety = "" if chassis_snapshot is None else chassis_snapshot.safety_status
        hold = mission.title == "일시 정지"
        estop = (
            chassis_snapshot is not None
            and chassis_snapshot.safety_estop_required is True
        )
        safety.values["receive"].set_text(public_freshness(
            chassis_state, waiting="수신 대기"))
        safety.values["obstacle"].set_text(safety_public)
        safety.values["hold"].set_text("일시 정지" if hold else
                                       "정보 없음" if chassis_snapshot is None else "정상")
        safety.values["estop"].set_text("비상 정지" if estop else
                                        "정보 없음" if chassis_snapshot is None else "정상")
        safety.values["device"].set_text(
            "연결 확인 중" if chassis_snapshot is None else safety_public)
        safety.values["last"].set_text(self._received_text(
            chassis_snapshot, chassis_state))
        safety.developer_values["raw"].set_text(chassis_state)
        safety.developer_values["us100"].set_text(raw_safety or "없음")
        safety.developer_values["last_raw"].set_text(self._received_text(
            chassis_snapshot, chassis_state))

        for key, panel in (
            ("front_camera", self._l515), ("work_camera", self._d435),
        ):
            card = self._diagnostic_by_name[key]
            state = panel.health_state()
            live = state == "LIVE"
            card.values["connection"].set_text(
                "연결 완료" if live else "연결 확인 필요"
                if state == "STALE" else "연결 중")
            card.values["receive"].set_text(
                "영상 수신 중" if live else "수신 대기")
            card.values["position"].set_text(
                "주 화면" if panel is self._main_video else "보조 화면")
            last_frame = panel._last_frame_monotonic
            card.values["frame"].set_text(
                "정보 없음" if last_frame is None
                else f"{max(0.0, time.monotonic() - last_frame):.1f}초 전")
            card.values["update"].set_text(
                "정상" if live else "업데이트 지연"
                if state == "STALE" else "수신 대기")
            card.developer_values["raw"].set_text(state)
            card.developer_values["last_raw"].set_text(
                "없음" if last_frame is None else str(last_frame))

        arm = self._diagnostic_by_name["arm"]
        arm.values["receive"].set_text(public_freshness(
            arm_state, waiting="수신 대기"))
        arm.values["stage"].set_text(mission.title)
        arm.values["motion"].set_text(public_freshness(arm_state))
        arm.values["joints"].set_text(
            "수신 대기" if arm_snapshot is None
            else "정상" if arm_snapshot.joint_names else "정보 없음")
        arm.values["tool"].set_text("정보 없음")
        arm.values["target"].set_text(public_freshness(
            yolo_state, waiting="대상 탐지 대기"))
        arm.values["done"].set_text("완료" if mission.title == "완료" else "수신 대기")
        arm.developer_values["fsm"].set_text(
            self._ops_panel.latest_chassis_mode() or "없음")
        arm.developer_values["motor"].set_text(arm_state)
        arm.developer_values["joints"].set_text(
            "없음" if arm_snapshot is None else str(arm_snapshot.joint_position_rad))
        arm.developer_values["last"].set_text(
            self._received_text(arm_snapshot, arm_state))

    @staticmethod
    def _telemetry_state(snapshot: TelemetrySnapshot | None) -> str:
        if snapshot is None:
            return "UNAVAILABLE"
        return "STALE" if time.monotonic() - snapshot.received_monotonic_s > 1.0 else "LIVE"

    @staticmethod
    def _chip_tone(state: str) -> str:
        normalized = state.upper()
        if normalized in {"LIVE", "CLEAR", "OK", "NORMAL"}:
            return "status-live"
        if normalized in {"ESTOP", "FAULT", "ERROR", "CRITICAL"}:
            return "status-bad"
        if normalized in {"STALE", "DEGRADED"}:
            return "status-warn"
        if normalized in {"CHECKING", "VERIFYING"}:
            return "status-progress"
        if normalized in {"WAITING", "CONNECTING", "UNAVAILABLE", "UNKNOWN"}:
            return "status-muted"
        return "status-muted"

    def _set_health_chip(
        self, key: str, title: str, state: str, tone: str | None = None,
    ) -> None:
        chip = self._health_chips[key]
        chip.set_text(title)
        chip.set_tooltip_text(state)
        selected_tone = self._chip_tone(state) if tone is None else tone
        candidates = ("status-live", "status-camera-live", "status-progress",
                      "status-warn", "status-bad", "status-muted")
        for widget in (self._health_dots[key], chip):
            context = widget.get_style_context()
            for candidate in candidates:
                context.remove_class(candidate)
            context.add_class(selected_tone)

    def _set_preparation_status(
        self, key: str, text: str, tone: str,
    ) -> None:
        dot, value = self._preparation_status[key]
        value.set_text(text)
        candidates = (
            "prep-ready", "prep-connecting", "prep-attention", "prep-error",
            "prep-offline",
        )
        for widget in (dot, value):
            context = widget.get_style_context()
            for candidate in candidates:
                context.remove_class(candidate)
            context.add_class(tone)

    @staticmethod
    def _video_preparation_state(state: str) -> tuple[str, str, bool]:
        if state == "LIVE":
            return "준비 완료", "prep-ready", True
        if state == "STALE":
            return "확인 필요", "prep-attention", False
        if state in {"CONNECTING", "WAITING"}:
            return "연결 중", "prep-connecting", False
        return "정보 없음", "prep-offline", False

    def swap_camera_views(
        self, selected: VideoPanel, *, user_initiated: bool,
    ) -> bool:
        if not user_initiated:
            return False
        if selected is self._main_video:
            return False
        secondary = self._main_video
        self._videos.remove(secondary)
        self._pip_slot.remove(selected)
        self._videos.add(selected)
        self._pip_slot.add(secondary)
        secondary.set_header_action(self._swap_button)
        selected.set_role("MAIN")
        secondary.set_role("SUB")
        self._main_video = selected
        self._main_camera_label.set_text({
            "전방 카메라": "전방 화면",
            "작업 카메라": "작업 화면",
        }.get(selected._name, selected._name))
        self._videos.show_all()
        self._add_event("VIDEO", f"main view changed: {selected._name}")
        return True

    def _on_display_option_toggled(
        self, _button: Gtk.CheckButton,
    ) -> None:
        self.overlay_view_state["show_detection"] = (
            self._show_objects.get_active()
        )
        self.overlay_view_state["show_distance"] = (
            self._show_distance.get_active()
        )
        self._d435.set_metadata_display_options(
            show_objects=self.overlay_view_state["show_detection"],
            show_distance=self.overlay_view_state["show_distance"],
        )
        self._sync_overlay_rail(self._metadata_receiver.latest())

    def _on_page_changed(self, stack: Gtk.Stack, _param: object) -> None:
        """Use the in-page Mission footer; keep one footer on every tab."""
        mission_visible = stack.get_visible_child_name() == "mission"
        if hasattr(self, "_rail_buttons"):
            for index, button in enumerate(self._rail_buttons):
                context = button.get_style_context()
                context.remove_class("active")
                if index == (0 if mission_visible else 1):
                    context.add_class("active")
        # The approved robot-status references use the full height for the
        # subsystem visualization.  Mission keeps its own compact event bar;
        # the status surface intentionally has no second global footer.
        self._event_expander.set_visible(False)
        if not mission_visible:
            self._mission_event_expander.set_expanded(False)
            self._event_expander.set_expanded(False)

    def _on_rail_navigation(
        self, _button: Gtk.Button, destination: str,
    ) -> None:
        if destination == "mission":
            self._stack.set_visible_child_name("mission")
            return
        self._stack.set_visible_child_name("systems")
        if destination == "systems":
            self._robot_status._card_buttons["drive"].clicked()
        elif destination in {"arm", "safety"}:
            self._robot_status._card_buttons[destination].clicked()
        elif destination == "events":
            self._event_expander.set_expanded(True)

    def _build_end_effector_popup(self) -> None:
        """Keep end-effector detail off the main navigation surface."""
        popup = Gtk.Window(title="엔드이펙터 상세")
        popup.set_default_size(760, 460)
        popup.set_position(Gtk.WindowPosition.CENTER_ON_PARENT)
        popup.set_transient_for(self)
        popup.set_destroy_with_parent(True)
        popup.set_type_hint(Gdk.WindowTypeHint.DIALOG)
        popup.connect("delete-event", self._hide_end_effector_popup)
        _style(popup, "end-effector-popup")

        shell = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        _style(shell, "end-effector-popup-shell")
        popup_bar = Gtk.Box(spacing=10)
        self._end_effector_popup_context = Gtk.Label(
            label="선택한 엔드이펙터 상세"
        )
        self._end_effector_popup_context.set_xalign(0.0)
        _style(self._end_effector_popup_context, "end-effector-popup-heading")
        close = Gtk.Button(label="닫기")
        _style(close, "end-effector-detail-button")
        close.connect("clicked", lambda _button: popup.hide())
        popup_bar.pack_start(
            self._end_effector_popup_context, True, True, 0,
        )
        popup_bar.pack_end(close, False, False, 0)
        shell.pack_start(popup_bar, False, False, 0)

        detail_stack = Gtk.Stack()
        detail_stack.set_transition_type(Gtk.StackTransitionType.NONE)
        detail_stack.set_hexpand(True)
        detail_stack.set_vexpand(True)

        generic = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=16)
        generic.set_border_width(18)
        purpose = Gtk.Label(label="사용할 엔드이펙터를 선택하세요")
        purpose.set_xalign(0.0)
        purpose.set_line_wrap(True)
        _style(purpose, "end-effector-popup-purpose")
        generic.pack_start(purpose, False, False, 0)
        self._end_effector_popup_purpose = purpose

        info_card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        _style(info_card, "end-effector-popup-card")
        self._end_effector_popup_values: dict[str, Gtk.Label] = {}
        for key, label_text in (
            ("selection", "선택 상태"),
            ("attachment", "장착 확인"),
            ("operation", "로봇팔 조종 모드"),
            ("load", "관절 부하"),
            ("telemetry", "도구 데이터"),
        ):
            row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
            label = Gtk.Label(label=label_text)
            label.set_xalign(0.0)
            _style(label, "end-effector-popup-label")
            value = Gtk.Label(label="정보 없음")
            value.set_xalign(0.0)
            value.set_line_wrap(True)
            _style(value, "end-effector-popup-value")
            row.pack_start(label, False, False, 0)
            row.pack_start(value, False, False, 0)
            info_card.pack_start(row, False, False, 0)
            self._end_effector_popup_values[key] = value
        generic.pack_start(info_card, True, True, 0)

        detail_stack.add_named(generic, "generic")
        environment_scroll = Gtk.ScrolledWindow()
        environment_scroll.set_policy(
            Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC,
        )
        environment_scroll.add(self._environment_status)
        detail_stack.add_named(environment_scroll, "environment")
        shell.pack_start(detail_stack, True, True, 0)
        popup.add(shell)
        self._end_effector_popup = popup
        self._end_effector_detail_stack = detail_stack

    def _hide_end_effector_popup(
        self, popup: Gtk.Window, _event: object,
    ) -> bool:
        popup.hide()
        return True

    def _show_end_effector_popup(self) -> None:
        selected = self._mission_tool_selector.get_active_text() or "미확인"
        self._refresh_end_effector_summary()
        self._end_effector_popup_context.set_text(f"{selected} 상세")
        # Gtk.Stack may restore its first child during the popup's initial
        # show_all(), so select the requested detail after realizing widgets.
        self._end_effector_popup.show_all()
        if selected == "환경 센서 모듈":
            self._end_effector_detail_stack.set_visible_child_name("environment")
            self._end_effector_popup.resize(1260, 820)
        else:
            self._end_effector_detail_stack.set_visible_child_name("generic")
            self._end_effector_popup.resize(760, 460)
        self._end_effector_popup.present()

    def _set_end_effector_summary_state(
        self, text: str, css_class: str,
    ) -> None:
        context = self._mission_tool_state.get_style_context()
        for candidate in ("status-live", "status-warn", "status-muted"):
            context.remove_class(candidate)
        context.add_class(css_class)
        self._mission_tool_state.set_text(text)

    def _refresh_end_effector_summary(
        self,
        *,
        arm_snapshot: ArmTelemetrySnapshot | None = None,
        arm_state: str | None = None,
        environment_state: str | None = None,
    ) -> None:
        selected = self._mission_tool_selector.get_active_text() or "미확인"
        purpose = END_EFFECTOR_PURPOSES.get(
            selected, "사용할 엔드이펙터를 선택하세요",
        )
        self._mission_tool_purpose.set_text(purpose)
        self._end_effector_popup_purpose.set_text(purpose)

        if arm_snapshot is None:
            arm_snapshot = self._arm_receiver.latest()
        if arm_state is None:
            arm_state = self._telemetry_state(arm_snapshot)
        if environment_state is None:
            environment_state = environment_source_state(
                self._environment_receiver.latest()
            )

        observed_type = None if arm_snapshot is None else arm_snapshot.end_effector_type
        observed_display = {
            "gripper_a": "그리퍼 1",
            "gripper_b": "그리퍼 2",
            "환경 센싱 모듈": "환경 센서 모듈",
            "environment_sensor": "환경 센서 모듈",
            "cleaning_module": "청소 모듈",
        }.get(observed_type or "", observed_type)
        observed_match = (
            arm_state == "LIVE"
            and arm_snapshot is not None
            and observed_display == selected
        )
        if selected == "미확인":
            state_text, state_css = "선택 대기", "status-muted"
            reading = "상세 데이터 없음"
        elif selected == "환경 센서 모듈":
            if environment_state == "LIVE":
                state_text, state_css = "수신 중", "status-live"
            else:
                state_text, state_css = "미연결", "status-muted"
            reading = self._environment_status.probe_values()[0]
        elif observed_match and arm_snapshot is not None:
            if arm_snapshot.end_effector_attached is True:
                state_text, state_css = "장착 확인", "status-live"
            elif arm_snapshot.end_effector_attached is False:
                state_text, state_css = "미체결", "status-warn"
            else:
                state_text, state_css = "확인 필요", "status-warn"
            reading = " · ".join(filter(None, (
                arm_snapshot.end_effector_id,
                arm_snapshot.end_effector_interface,
            ))) or "로봇팔 수신 중"
        else:
            state_text, state_css = "선택됨", "status-warn"
            reading = "로봇팔 장착 정보 대기"

        self._set_end_effector_summary_state(state_text, state_css)
        self._mission_tool_reading.set_text(reading)
        attachment = "정보 없음"
        telemetry = "수신 대기"
        operation = "연동 예정 · 조종 모드 상태 필드 없음"
        load = "정보 없음"
        if arm_state == "LIVE" and arm_snapshot is not None:
            if arm_snapshot.end_effector_attached is True:
                attachment = "체결"
            elif arm_snapshot.end_effector_attached is False:
                attachment = "미체결"
            if arm_snapshot.end_effector_id:
                attachment += f" · ID {arm_snapshot.end_effector_id}"
            if arm_snapshot.dynamixel is not None:
                if arm_snapshot.dynamixel:
                    peak_current = max(
                        abs(motor.current) for motor in arm_snapshot.dynamixel
                    )
                    load = (
                        f"최고 {peak_current} raw · "
                        f"{len(arm_snapshot.dynamixel)}개 관절"
                    )
                else:
                    load = "관절 모터 0개"
            interface = arm_snapshot.end_effector_interface or "인터페이스 정보 없음"
            telemetry = (
                f"{interface} · 선택 항목과 관측값 일치"
                if observed_match else
                f"{interface} · 선택 항목과 관측값 불일치 또는 미수신"
            )
        self._mission_arm_mode.set_text("연동 예정")
        self._mission_arm_load.set_text(load)
        self._end_effector_popup_values["selection"].set_text(
            f"{selected} · {state_text}"
        )
        self._end_effector_popup_values["attachment"].set_text(attachment)
        self._end_effector_popup_values["operation"].set_text(operation)
        self._end_effector_popup_values["load"].set_text(load)
        self._end_effector_popup_values["telemetry"].set_text(telemetry)

    def _on_mission_end_effector_changed(self, *_args: object) -> None:
        if self._syncing_end_effector_selectors:
            return
        selected = self._mission_tool_selector.get_active_text() or "미확인"
        self._syncing_end_effector_selectors = True
        try:
            self._robot_status.select_end_effector(selected)
        finally:
            self._syncing_end_effector_selectors = False
        self._refresh_end_effector_summary()
        self._show_end_effector_popup()

    def _on_status_end_effector_changed(self, *_args: object) -> None:
        if self._syncing_end_effector_selectors:
            return
        selected = self._robot_status.selected_end_effector()
        model = self._mission_tool_selector.get_model()
        self._syncing_end_effector_selectors = True
        try:
            for index, row in enumerate(model):
                if row[0] == selected:
                    self._mission_tool_selector.set_active(index)
                    break
        finally:
            self._syncing_end_effector_selectors = False
        self._refresh_end_effector_summary()

    def _on_video_area_allocated(
        self, _widget: Gtk.Overlay, allocation: Gdk.Rectangle,
    ) -> None:
        if allocation.width < 1:
            return
        # The compact placeholder needs ~150 px height for its header, rover,
        # and two text rows; smaller PiP slots clip their own content.
        pip_width = min(360, max(270, int(allocation.width * 0.27)))
        # D435i transport is 848x480.  Preserve the exact native ratio instead
        # of the close-but-not-identical 16:9 approximation.
        pip_height = int(round(pip_width * 480 / 848))
        self._pip_slot.set_slot_size(pip_width, pip_height)

    def _on_mission_body_allocated(
        self, _widget: Gtk.Box, allocation: Gdk.Rectangle,
    ) -> None:
        rail_width = min(330, max(235, int(allocation.width * 0.21)))
        if allocation.width >= 1500:
            rail_width = max(290, rail_width)
        self._mission_rail.set_size_request(rail_width, -1)

    def _sync_overlay_rail(self, metadata: MetadataFrame | None) -> None:
        """Keep target cards live independently from video overlay choices."""
        now_s = time.monotonic()
        fresh = (
            metadata is not None
            and now_s - metadata.received_monotonic_s
            <= OVERLAY_STALE_AFTER_S
            and metadata.source_camera_id == "work"
        )
        # DisplayTargetTracker 갱신은 이 타이머 경로 한 곳에서만 수행한다.
        target_view = self._display_target_tracker.update(
            metadata if fresh else None, now_s=now_s,
        )
        target = target_view.detection

        self._rail_data.set_no_show_all(False)
        self._rail_data.show()
        self._mission_metric_rows["tool"].show_all()
        self._mission_metric_rows["target"].show_all()
        if target is None:
            self._mission_metrics["target"].set_text("인식 대상 없음")
        else:
            self._mission_metrics["target"].set_text(
                f"{target.class_name}  {target.confidence:.0%}"
            )
        self._mission_metric_rows["distance"].show_all()
        if metadata is not None and not fresh:
            self._mission_metrics["distance"].set_text("거리 정보 지연")
        elif target_view.distance_m is None:
            self._mission_metrics["distance"].set_text(
                target_view.distance_state
            )
        else:
            self._mission_metrics["distance"].set_text(
                f"{target_view.distance_m:.2f} m"
            )

    def _refresh_estop_availability(self) -> None:
        sensitive, tooltip, _warning = estop_availability(
            token_available=self._ops_panel.ops_available(),
            link_ready=self._ops_panel.link_ready(),
        )
        self._global_estop.set_sensitive(sensitive)
        self._global_estop.set_tooltip_text(tooltip)
        self._estop_availability_warning.hide()

    def _show_alert(self, message: str) -> None:
        """Show a command failure above the main content for eight seconds."""
        self._alert_serial += 1
        serial = self._alert_serial
        self._alert_label.set_text(message)
        self._alert_label.show()

        def hide_if_current() -> bool:
            if serial == self._alert_serial:
                self._alert_label.hide()
            return False

        GLib.timeout_add(8000, hide_if_current)

    def _refresh_health(self) -> bool:
        self._refresh_estop_availability()
        snapshot = self._telemetry_receiver.latest()
        chassis_snapshot = self._chassis_receiver.latest()
        arm_snapshot = self._arm_receiver.latest()
        environment_snapshot = self._environment_receiver.latest()
        metadata = self._metadata_receiver.latest()
        if metadata is None:
            yolo = "WAITING"
        else:
            yolo = (
                "STALE"
                if time.monotonic() - metadata.received_monotonic_s
                > OVERLAY_STALE_AFTER_S
                else "LIVE"
            )
        telemetry = self._telemetry_state(snapshot)
        chassis_state = self._telemetry_state(chassis_snapshot)
        arm_state = self._telemetry_state(arm_snapshot)
        environment_state = environment_source_state(environment_snapshot)
        odom, drive, can = chassis_component_states(chassis_snapshot)
        telemetry_mask = (
            chassis_snapshot.component_mask
            if chassis_state == "LIVE" and chassis_snapshot is not None
            else None
        )
        component_mask = (
            telemetry_mask
            if telemetry_mask is not None
            else self._ops_panel.latest_component_mask()
        )
        mask_text = mask_banner_text(component_mask)
        safety, safety_color = safety_banner_state(
            chassis_snapshot,
            component_mask=component_mask,
            telemetry_live=chassis_state == "LIVE",
        )
        if safety != self._last_safety_banner:
            self._add_event("SAFETY", safety)
            self._last_safety_banner = safety
        if chassis_state != "LIVE" or chassis_snapshot is None:
            l515_transport = "L515 transport unavailable"
        elif (chassis_snapshot.l515_submitted_hz is None
              or chassis_snapshot.l515_sent_hz is None):
            l515_transport = "L515 transport waiting"
        elif (chassis_snapshot.l515_drop_hz or 0.0) > 0.1:
            l515_transport = (
                f"L515 SRT drop {chassis_snapshot.l515_drop_hz:.2f} Hz "
                f"(submit {chassis_snapshot.l515_submitted_hz:.2f}, "
                f"sent {chassis_snapshot.l515_sent_hz:.2f})"
            )
        else:
            l515_transport = "L515 SRT transport normal"
        if l515_transport != self._last_l515_transport:
            self._add_event("L515", l515_transport)
            self._last_l515_transport = l515_transport
        network_public = public_freshness(
            chassis_state, waiting="연결 대기", unavailable="연결 대기",
        )
        self._set_health_chip(
            "network", "로봇 연결", network_public,
            self._chip_tone(chassis_state),
        )
        power_ok = (
            telemetry == "LIVE" and snapshot is not None
            and snapshot.pdist_battery_flags in (None, 0)
            and snapshot.pdist_protection_flags in (None, 0)
        )
        power_public = (
            "정상" if power_ok
            else "확인 필요" if telemetry == "LIVE"
            else public_freshness(telemetry)
        )
        self._set_health_chip(
            "power", "전원", power_public,
            "status-live" if power_ok else "status-warn"
            if telemetry == "LIVE" else self._chip_tone(telemetry),
        )
        l515_video = self._l515.health_state()
        d435_video = self._d435.health_state()
        main_video_state = self._main_video.health_state()
        self._watermark.set_live(main_video_state == "LIVE")
        self._main_camera_state_label.set_text(
            "연결 완료" if main_video_state == "LIVE"
            else "업데이트 지연" if main_video_state == "STALE"
            else "연결 중"
        )
        main_dot_context = self._main_camera_dot.get_style_context()
        for candidate in ("status-live", "status-warn", "status-muted"):
            main_dot_context.remove_class(candidate)
        main_dot_context.add_class(
            "status-live" if main_video_state == "LIVE"
            else "status-warn" if main_video_state == "STALE"
            else "status-muted"
        )
        if l515_video == d435_video == "LIVE":
            camera_public, camera_tone = "정상", "status-camera-live"
        elif "LIVE" in {l515_video, d435_video}:
            camera_public, camera_tone = "일부 연결", "status-warn"
        elif "STALE" in {l515_video, d435_video}:
            camera_public, camera_tone = "업데이트 지연", "status-warn"
        else:
            camera_public, camera_tone = "연결 대기", "status-muted"
        self._set_health_chip(
            "camera", "카메라", camera_public, camera_tone,
        )
        us100_enabled = (
            None if component_mask is None else component_mask.get("us100")
        )
        safety_public, safety_public_tone = public_safety(
            telemetry_live=chassis_state == "LIVE",
            estop_required=(None if chassis_snapshot is None
                            else chassis_snapshot.safety_estop_required),
            us100_enabled=us100_enabled,
        )
        safety_tone = {
            "success": "status-live", "warning": "status-warn",
            "danger": "status-bad", "offline": "status-muted",
        }[safety_public_tone]
        self._set_health_chip(
            "safety", "안전 장치", safety_public, safety_tone,
        )

        front_text, front_tone, front_ready = (
            self._video_preparation_state(l515_video)
        )
        self._set_preparation_status("front", front_text, front_tone)

        if d435_video == "LIVE" and arm_state == "LIVE":
            work_text, work_tone, work_ready = (
                "준비 완료", "prep-ready", True,
            )
        elif "STALE" in {d435_video, arm_state}:
            work_text, work_tone, work_ready = (
                "확인 필요", "prep-attention", False,
            )
        elif d435_video in {"CONNECTING", "WAITING"}:
            work_text, work_tone, work_ready = (
                "연결 중", "prep-connecting", False,
            )
        else:
            work_text, work_tone, work_ready = (
                "정보 없음", "prep-offline", False,
            )
        self._set_preparation_status("work", work_text, work_tone)

        if drive == "LIVE" and can == "LIVE":
            drive_text, drive_tone, drive_ready = (
                "준비 완료", "prep-ready", True,
            )
        elif "STALE" in {drive, can} or chassis_state == "LIVE":
            drive_text, drive_tone, drive_ready = (
                "확인 필요", "prep-attention", False,
            )
        elif chassis_state == "UNAVAILABLE":
            drive_text, drive_tone, drive_ready = (
                "정보 없음", "prep-offline", False,
            )
        else:
            drive_text, drive_tone, drive_ready = (
                "연결 중", "prep-connecting", False,
            )
        self._set_preparation_status("drive", drive_text, drive_tone)

        safety_ready = safety_public_tone == "success"
        safety_prep_tone = {
            "success": "prep-ready",
            "warning": "prep-attention",
            "danger": "prep-error",
            "offline": "prep-offline",
        }[safety_public_tone]
        safety_prep_text = (
            "준비 완료" if safety_ready
            else "확인 필요" if safety_public_tone in {"warning", "danger"}
            else "정보 없음"
        )
        self._set_preparation_status(
            "safety", safety_prep_text, safety_prep_tone,
        )

        readiness = {
            "front": front_ready,
            "work": work_ready,
            "drive": drive_ready,
            "safety": safety_ready,
        }
        ready_count = sum(readiness.values())

        safety_value = self._mission_metrics["safety"]
        safety_value.set_text(
            "연결 확인 중" if safety_public_tone == "offline" else safety_public
        )
        safety_context = safety_value.get_style_context()
        for candidate in ("status-live", "status-warn", "status-bad", "status-muted"):
            safety_context.remove_class(candidate)
        safety_context.add_class({
            "success": "status-live", "warning": "status-warn",
            "danger": "status-bad", "offline": "status-muted",
        }[safety_public_tone])
        wheel_speeds = (
            () if chassis_state != "LIVE" or chassis_snapshot is None else
            tuple(
                abs(wheel.drive_turns_per_s)
                for wheel in chassis_snapshot.wheel_statuses
                if wheel.drive_turns_per_s is not None and not wheel.stale
            )
        )
        self._mission_metrics["speed"].set_text(
            "정보 없음" if not wheel_speeds
            else f"{sum(wheel_speeds) / len(wheel_speeds):.2f} turn/s"
        )
        self._sync_overlay_rail(metadata)
        target = self._display_target_tracker.view().detection
        self._rail_preparation_target.set_text(
            "작업 대상    대상 탐지 대기"
            if target is None else f"작업 대상    {target.class_name}"
        )
        chassis_mode = self._ops_panel.latest_chassis_mode()
        drive_state = "" if chassis_snapshot is None else chassis_snapshot.drive_state
        mission = mission_presentation(chassis_mode, drive_state)
        self._rail_preparation_detail.set_text(f"현재 단계    {mission.title}")
        self._readiness_count.set_text(f"{ready_count} / 4")
        drive_summary = (
            "정상" if drive == "LIVE" and can == "LIVE"
            else "업데이트 지연" if chassis_state == "STALE"
            else "확인 필요" if chassis_state == "LIVE"
            else "정보 없음"
        )
        power_summary_text = (
            "정상" if power_ok else "확인 필요"
            if telemetry == "LIVE" else "업데이트 지연"
            if telemetry == "STALE" else "정보 없음"
        )
        front_summary = (
            "정상" if l515_video == "LIVE" else "연결 끊김"
            if l515_video == "STALE" else "연결 중"
        )
        work_summary = (
            "정상" if d435_video == "LIVE" else "연결 끊김"
            if d435_video == "STALE" else "연결 중"
        )
        arm_summary_text = (
            "정상" if arm_state == "LIVE" else "업데이트 지연"
            if arm_state == "STALE" else "정보 없음"
        )
        safety_summary = (
            "사용 안 함" if us100_enabled is False else "비상 정지"
            if chassis_snapshot is not None and
            chassis_snapshot.safety_estop_required is True else "일시 정지"
            if mission.title == "일시 정지" else "정상"
            if safety_public_tone == "success" else "확인 필요"
            if safety_public_tone in {"warning", "danger"} else "정보 없음"
        )
        self._robot_status.update(
            power=snapshot,
            chassis=chassis_snapshot,
            arm=arm_snapshot,
            metadata=metadata,
            front_video_state=l515_video,
            work_video_state=d435_video,
            front_fps=self._l515.last_fps,
            work_fps=self._d435.last_fps,
            front_frame_age_s=self._l515.last_frame_age_s,
            work_frame_age_s=self._d435.last_frame_age_s,
            control_link_ready=self._ops_panel.link_ready(),
            chassis_mode=chassis_mode,
        )
        self._environment_status.update(environment_snapshot)
        self._refresh_end_effector_summary(
            arm_snapshot=arm_snapshot,
            arm_state=arm_state,
            environment_state=environment_state,
        )
        self._l515.set_rover_component_states(
            front_live=l515_video == "LIVE",
            work_live=d435_video == "LIVE" and arm_state == "LIVE",
            drive_ready=drive_summary == "정상",
            safety_ready=safety_summary == "정상",
        )
        self._d435.set_rover_component_states(
            front_live=l515_video == "LIVE",
            work_live=d435_video == "LIVE" and arm_state == "LIVE",
            drive_ready=drive_summary == "정상",
            safety_ready=safety_summary == "정상",
        )
        _severity, latest_event = self._events.latest_public()
        self._event_latest.set_text(f"최근: {latest_event}")
        self._mission_event_latest.set_text(f"최근: {latest_event}")
        ready = (
            chassis_state == "LIVE" and power_ok
            and l515_video == d435_video == "LIVE"
            and safety_public_tone == "success"
        )
        if ready:
            self._readiness_title.set_text("로봇 운용 준비 완료")
            self._readiness_detail.set_text(
                "전원, 카메라, 통신, 안전 장치가 준비되었습니다"
            )
        else:
            waiting_items = []
            if chassis_state != "LIVE":
                waiting_items.append("로봇 연결")
            if not power_ok:
                waiting_items.append("전원")
            if l515_video != "LIVE" or d435_video != "LIVE":
                waiting_items.append("카메라")
            if safety_public_tone != "success":
                waiting_items.append("안전 장치")
            self._readiness_title.set_text("시스템 점검 중")
            self._readiness_detail.set_text(
                " · ".join(waiting_items) + "의 연결을 확인하고 있습니다"
            )
        self._write_smoke_probe({
            "telemetry": telemetry,
            "chassis": chassis_state,
            "metadata": yolo,
            "arm": arm_state,
            "environment": environment_state,
            "environment_climate": self._environment_status.probe_values()[0],
            "environment_air": self._environment_status.probe_values()[1],
            "environment_hazard": self._environment_status.probe_values()[2],
            "video_l515": self._l515.health_state(),
            "video_d435": self._d435.health_state(),
            "safety_banner": safety,
            "global_estop_visible": self._global_estop.get_visible(),
            "main_video": self._main_video._name,
            "rover_l515_width": self._l515._rover_status_overlay.get_size_request()[0],
            "rover_d435_width": self._d435._rover_status_overlay.get_size_request()[0],
        })
        return True

    def _write_smoke_probe(self, states: dict) -> None:
        """runtime_smoke 전용 관측 창구 — `--smoke-probe-file` 없으면 no-op.

        GTK 패널 상태는 라벨 마크업 안에만 있어서 외부에서 볼 수 없다. 그래서
        하니스가 "기동했고 traceback 없음" 이상을 단언하지 못했고, 수신 스레드를
        통째로 죽여도 PASS 했다(2026-07-18 적대적 리뷰 R06 #10). 여기서 패널별
        freshness 를 파일로 흘려 하니스가 **실제로 LIVE 에 도달했는지** 단언한다.
        진단 전용이며 운용 경로에는 영향이 없다.
        """
        if self._smoke_probe_path is None:
            return
        try:
            self._smoke_probe_path.write_text(
                json.dumps(states), encoding="utf-8",
            )
        except OSError:
            pass

    def _on_destroy(self, *_args: object) -> None:
        self._d435.stop()
        self._l515.stop()
        self._metadata_receiver.close()
        self._telemetry_receiver.close()
        self._chassis_receiver.close()
        self._arm_receiver.close()
        self._environment_receiver.close()
        self._ops_panel.close()
        Gtk.main_quit()

    def _on_key_press(self, _widget: Gtk.Window, event: Gdk.EventKey) -> bool:
        if event.keyval in (Gdk.KEY_v, Gdk.KEY_V):
            self.swap_camera_views(
                self._d435 if self._main_video is self._l515 else self._l515,
                user_initiated=True,
            )
            return True
        if event.keyval != Gdk.KEY_F11:
            return False
        self._fullscreen = not self._fullscreen
        if self._fullscreen:
            self.fullscreen()
            self._add_event("CONSOLE", "fullscreen enabled")
        else:
            self.unfullscreen()
            self._add_event("CONSOLE", "fullscreen disabled")
        return True


def _add_unix_signal_watch(
    stop_signal: int, handler: Callable[..., bool],
) -> None:
    """GLib 메인 루프에서 유닉스 신호를 받는다.

    `GLib.unix_signal_add` 는 PyGObject 3.56 에서 deprecated 라 기동할 때마다
    경고를 찍는다.  새 `GLibUnix.signal_add` 가 있으면 그쪽을 쓴다.
    """
    try:
        gi.require_version("GLibUnix", "2.0")
        from gi.repository import GLibUnix
    except (ValueError, ImportError):
        GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, stop_signal, handler)
    else:
        signal_add = getattr(GLibUnix, "signal_add", None)
        if signal_add is None:
            GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, stop_signal, handler)
        else:
            signal_add(GLib.PRIORITY_DEFAULT, stop_signal, handler)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.8.106")
    parser.add_argument("--d435-port", type=int, default=5002)
    parser.add_argument("--l515-port", type=int, default=5000)
    parser.add_argument("--metadata-port", type=int, default=5003)
    parser.add_argument("--telemetry-port", type=int, default=5004)
    parser.add_argument("--chassis-telemetry-port", type=int, default=5005)
    parser.add_argument("--arm-telemetry-port", type=int, default=5007)
    parser.add_argument("--environment-telemetry-port", type=int, default=5008)
    parser.add_argument(
        "--ops-host",
        default=None,
        help="ops broker host (default: same value as --host)",
    )
    parser.add_argument("--ops-port", type=int, default=9001)
    parser.add_argument("--ops-token-file", default=DEFAULT_OPS_TOKEN_FILE)
    parser.add_argument("--latency-ms", type=int, default=60)
    parser.add_argument(
        "--smoke-probe-file",
        default=None,
        help=("진단 전용: 패널별 freshness 를 이 경로에 JSON 으로 흘린다. "
              "runtime_smoke 가 LIVE 도달을 단언하는 데 쓴다. 운용 시 생략."),
    )
    parser.add_argument(
        "--input-source",
        choices=("LIVE", "REPLAY", "TEST"),
        default="LIVE",
        help="developer-info provenance label only; does not change receivers",
    )
    args = parser.parse_args()
    Gst.init(None)
    console = OperatorConsole(args.host, args.d435_port, args.l515_port,
                              args.metadata_port, args.latency_ms, args.telemetry_port,
                              args.chassis_telemetry_port,
                              args.arm_telemetry_port,
                              args.environment_telemetry_port,
                              ops_host=args.host if args.ops_host is None else args.ops_host,
                              ops_port=args.ops_port,
                              ops_token_file=args.ops_token_file,
                              smoke_probe_file=args.smoke_probe_file,
                              input_source=args.input_source)
    console.show_all()
    console.maximize()

    def _quit_on_signal(*_args: object) -> bool:
        # Ctrl+C 를 창 닫기와 같은 경로로 흘린다.  기본 SIGINT 는 Gtk.main()
        # 안의 C 프레임을 파이썬 예외로 깨뜨려 정리 없이 나가 버린다.
        console.destroy()
        return GLib.SOURCE_REMOVE

    for stop_signal in (signal.SIGINT, signal.SIGTERM):
        _add_unix_signal_watch(stop_signal, _quit_on_signal)
    Gtk.main()

    # 여기 오면 OperatorConsole._on_destroy 가 이미 파이프라인·수신 소켓·ops
    # 클라이언트를 정리했다.  그런데 libc 의 exit() 는 그 뒤 libsrt 의 전역
    # 소멸자(srt::CUDTUnited::~CUDTUnited)를 부르고, 그것이 자기 워커 스레드
    # (SRT:RcvQ/SndQ)가 아직 살아있는 채로 큐를 파괴한다.  2026-07-29 실사용
    # 코어 덤프에서 메인 스레드는 pthread_cond_destroy 에 멈춰 있었고(창은
    # 닫혔는데 터미널이 안 돌아온다) 수신 워커는 SEGV_MAPERR 로 죽었다.
    # 우리 정리는 이미 끝났으므로 그 전역 소멸자 경로를 아예 타지 않는다.
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
