"""로봇팔 수동조작 탭 — the manual-operation tab widget.

Layout, top to bottom:

* 도구 — the detected tool and the change candidate, kept visually and
  functionally apart.  Choosing a candidate changes nothing on the robot; only
  ``변경 요청`` reports the intent, and completion is shown only once the
  backend reports the new active tool.
* 제어권 — detected-tool card directly below the observed tool.  Asking never
  enables a motion control; only an approval does.
* 상태 — arm FSM, ``/arm_status``-style contract state and tool FSM as three
  separate readings, plus the concrete reasons operation is blocked.
* 키보드 텔레옵 — :mod:`keyboard_panel`.
* 도구 전용 조작 — beside the detected-tool card, single vs dual vs
  information-only.
* 현재 도구 진단 — collapsible, active tool only.

The tab owns no data source and no timer of its own: it is rendered by
:meth:`ArmManualTab.update_state` and torn down by :meth:`ArmManualTab.dispose`.
"""
from __future__ import annotations

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from .. import labels as console_labels
from . import contracts as C
from .keyboard_panel import KeyboardTeleopPanel
from .styling import ARM_UI_STYLE_CLASS, install_arm_ui_css
from .tool_panels import ToolDiagnosticsPanel, ToolPanelStack
from .widgets import (
    Badge,
    Card,
    GateGroup,
    Metric,
    SignalTracker,
    button,
    divider,
    label,
    metric_grid,
    reflow,
    reflow_row,
    scrolled,
    style,
    tri_state,
)


TAB_TITLE = "로봇팔 수동조작"
TAB_NAME = "arm_manual"


def link_tone(state: C.ArmUiState) -> str:
    return {
        C.LINK_LIVE: "status-live",
        C.LINK_STALE: "status-warn",
        C.LINK_WAITING: "status-muted",
    }.get(state.link.state, "status-bad")


def request_tone(status: str) -> str:
    return {
        C.REQUEST_COMPLETED: "status-live",
        C.REQUEST_PENDING: "status-warn",
        C.REQUEST_IN_PROGRESS: "status-warn",
        C.REQUEST_REJECTED: "status-bad",
        C.REQUEST_TIMEOUT: "status-bad",
        C.REQUEST_DISCONNECTED: "status-bad",
    }.get(status, "status-muted")


def authority_tone(status: str) -> str:
    return {
        C.AUTHORITY_GRANTED: "status-live",
        C.AUTHORITY_REQUESTED: "status-warn",
        C.AUTHORITY_REJECTED: "status-bad",
        C.AUTHORITY_TIMEOUT: "status-bad",
    }.get(status, "status-muted")


class ArmManualTab:
    """Builds the manual tab and refreshes it from injected state."""

    def __init__(self, callbacks: C.ArmUiCallbacks | None = None) -> None:
        install_arm_ui_css()
        self._callbacks = callbacks or C.ArmUiCallbacks()
        self._state = C.default_state()
        self.tracker = SignalTracker()
        self.gate = GateGroup()

        page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=11)
        page.set_border_width(14)
        page.set_hexpand(True)
        style(page, ARM_UI_STYLE_CLASS)

        page.pack_start(self._build_header(), False, False, 0)
        self._fixture_banner = label(
            "미리보기 fixture 데이터입니다 — 실제 로봇팔 상태가 아닙니다.",
            "arm-fixture-banner", wrap=True,
        )
        self._fixture_banner.set_no_show_all(True)
        page.pack_start(self._fixture_banner, False, False, 0)

        # A FlowBox, not a two-column Box: at narrow widths the cards stack
        # instead of forcing the whole page to scroll sideways.
        top = reflow(2)
        top.set_row_spacing(11)
        top.set_column_spacing(11)
        self.tool_panels = ToolPanelStack(self._callbacks)
        top.add(self._build_tool_card())
        # Keep the active tool's controls beside its identity.  The operator
        # should not have to scan below teleop to tell which tool is moving.
        top.add(self.tool_panels.stack)
        page.pack_start(top, False, False, 0)

        page.pack_start(self._build_state_card(), False, False, 0)

        self.keyboard = KeyboardTeleopPanel(self._callbacks)
        page.pack_start(self.keyboard.box, False, False, 0)

        self.diagnostics = ToolDiagnosticsPanel()
        page.pack_start(self.diagnostics.box, False, False, 0)

        self._page = page
        self.widget = scrolled(page)
        style(self.widget, ARM_UI_STYLE_CLASS)
        # A tab switch unmaps the page; that must end any hold in flight.
        self.tracker.connect(page, "unmap", lambda _w: self._on_unmap())
        self.tracker.connect(page, "map", lambda _w: self.keyboard.set_active(True))

        self.update_state(self._state)

    # --- construction ---------------------------------------------------
    def _build_header(self) -> Gtk.Widget:
        header = Gtk.Box(spacing=12)
        copy = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
        copy.set_hexpand(True)
        copy.pack_start(label(TAB_TITLE, "arm-page-title", wrap=True), False, False, 0)
        copy.pack_start(
            label(
                "감지된 도구, 제어권, 상태와 수동 조작을 한 곳에 모읍니다. "
                "보정은 캘리브레이션 탭에 있습니다.",
                "arm-page-subtitle", wrap=True,
            ),
            False, False, 0,
        )
        header.pack_start(copy, True, True, 0)
        self._link_badge = Badge("상태 미수신")
        header.pack_end(self._link_badge.label, False, False, 0)
        return header

    def _build_tool_card(self) -> Gtk.Widget:
        card = Card(
            "도구",
            "감지된 도구와 변경 후보는 분리되어 있습니다. 선택만으로는 "
            "교체 명령이 나가지 않습니다.",
            accent="tool",
        )
        self._tool_badge = card.badge

        detected = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        detected.pack_start(label("현재 감지 도구", "arm-metric-label"), False, False, 0)
        self._detected_name = label("감지 대기", "arm-card-title")
        detected.pack_start(self._detected_name, False, False, 0)
        self._tool_id = Metric("도구 ID", width_chars=12)
        self._tool_interface = Metric("인터페이스", width_chars=12)
        self._tool_attached = Metric("장착")
        self._tool_actuators = Metric("actuator", width_chars=12)
        detected.pack_start(metric_grid(
            (self._tool_id, self._tool_interface,
             self._tool_attached, self._tool_actuators), columns=2,
        ), False, False, 0)
        card.pack(detected)
        card.pack(divider())

        # Authority belongs to the detected tool, not to a separate column:
        # a MANUAL grant applies to whichever observed tool is active.
        card.pack(self._build_authority_section())
        card.pack(divider())

        candidate = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        candidate.pack_start(label("변경할 도구", "arm-metric-label"), False, False, 0)
        row = Gtk.Box(spacing=7)
        self._candidate = Gtk.ComboBoxText()
        # An explicit "not chosen" row: an empty combo reads as a rendering
        # bug, and picking it back returns the operator to no candidate.
        self._candidate.append("", "— 선택 안 함 —")
        for kind in C.SELECTABLE_TOOL_KINDS:
            self._candidate.append(kind, C.TOOL_KIND_KOREAN[kind])
        self._candidate.set_active_id("")
        self._candidate.set_tooltip_text(
            "후보만 고릅니다. 상태 갱신이 이 선택을 덮어쓰지 않습니다.",
        )
        row.pack_start(self._candidate, False, False, 0)
        self._change_button = button("변경 요청", "arm-primary")
        request = self._callbacks.request_tool_change
        if request is not None:
            self.tracker.connect(
                self._change_button, "clicked", lambda _b: self._request_change(),
            )
        self.gate.add(
            self._change_button, request, C.CAP_TOOL_CHANGE,
            extra=lambda _state: (
                "" if self._candidate.get_active_id() else "변경 후보 미선택"
            ),
            hint="선택한 후보로 명시적 도구 변경을 요청합니다",
        )
        row.pack_start(self._change_button, False, False, 0)
        candidate.pack_start(row, False, False, 0)
        self.tracker.connect(
            self._candidate, "changed", lambda _c: self.gate.apply(self._state),
        )

        status_row = Gtk.Box(spacing=8)
        status_row.pack_start(label("요청 상태", "arm-metric-label"), False, False, 0)
        self._change_badge = Badge("요청 없음")
        status_row.pack_start(self._change_badge.label, False, False, 0)
        candidate.pack_start(status_row, False, False, 0)
        self._change_detail = label("", "arm-note", wrap=True)
        candidate.pack_start(self._change_detail, False, False, 0)
        self._mismatch = label("", "arm-reason", wrap=True)
        self._mismatch.set_no_show_all(True)
        candidate.pack_start(self._mismatch, False, False, 0)
        card.pack(candidate)
        card.box.set_valign(Gtk.Align.START)
        return card.box

    def _request_change(self) -> None:
        kind = self._candidate.get_active_id()
        request = self._callbacks.request_tool_change
        if request is not None and kind:
            request(kind)

    def _build_authority_section(self) -> Gtk.Widget:
        """Build the authority controls directly below the observed tool."""
        section = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        heading = Gtk.Box(spacing=8)
        heading.pack_start(label("제어권", "arm-metric-label"), False, False, 0)
        self._authority_badge = Badge("확인 불가")
        heading.pack_start(self._authority_badge.label, False, False, 0)
        section.pack_start(heading, False, False, 0)
        section.pack_start(
            label(
                "요청만으로 조작은 열리지 않으며, 팔이 MANUAL 승인을 보고해야 합니다.",
                "arm-note", wrap=True,
            ),
            False, False, 0,
        )

        buttons: list[Gtk.Widget] = []
        request = self._callbacks.request_control_mode
        self._manual_button = button("수동 제어 요청", "arm-primary")
        self._fsm_button = button("FSM 제어 요청")
        for widget, mode in (
            (self._manual_button, C.MODE_MANUAL), (self._fsm_button, C.MODE_FSM),
        ):
            if request is not None:
                self.tracker.connect(
                    widget, "clicked", lambda _b, m=mode: request(m),
                )
            # Requesting is deliberately *not* gated on holding a grant, or a
            # console that has none could never obtain one.
            self.gate.add(widget, request, C.CAP_CONTROL_MODE)
            buttons.append(widget)
        release = self._callbacks.release_control_mode
        self._release_button = button("제어권 반납")
        if release is not None:
            self.tracker.connect(self._release_button, "clicked", lambda _b: release())
        self.gate.add(
            self._release_button, release, C.CAP_CONTROL_MODE,
            extra=lambda state: (
                "" if state.authority.granted_mode else "보유한 제어권 없음"
            ),
        )
        buttons.append(self._release_button)
        section.pack_start(reflow_row(buttons, columns=3), False, False, 0)

        enable = self._callbacks.set_tool_enabled
        self._tool_enable_button = button("도구 토크 활성화", "arm-primary")
        if enable is not None:
            self.tracker.connect(
                self._tool_enable_button, "clicked", lambda _b: enable(True),
            )
        self.gate.add(
            self._tool_enable_button, enable, C.CAP_TOOL_ENABLE,
            hint="현재 감지된 도구 actuator에만 토크를 인가합니다",
        )
        section.pack_start(self._tool_enable_button, False, False, 0)

        restart = self._callbacks.restart_bridge
        self._restart_button = button("브릿지 재시작", "arm-danger")
        if restart is not None:
            self.tracker.connect(
                self._restart_button, "clicked", lambda _b: restart(),
            )
        self.gate.add(
            self._restart_button, restart, C.CAP_BRIDGE_RESTART,
            hint="현재 도구를 재탐색하고 bridge의 도구 FSM을 다시 초기화합니다",
        )
        section.pack_start(self._restart_button, False, False, 0)

        self._requested = Metric("요청한 모드", width_chars=12)
        self._granted = Metric("승인된 모드", width_chars=12)
        section.pack_start(
            metric_grid((self._requested, self._granted), columns=2),
            False, False, 0,
        )
        self._authority_detail = label("", "arm-note", wrap=True)
        section.pack_start(self._authority_detail, False, False, 0)
        return section

    def _build_state_card(self) -> Gtk.Widget:
        card = Card(
            "상태 · 조작 불가 이유",
            "팔 FSM, 계약 상태, 도구 FSM을 구분해 표시합니다. "
            "제어권은 차체 운용 모드와 별개입니다.",
            accent="fsm",
        )
        self._state_badge = card.badge
        self._arm_fsm = Metric("팔 FSM", width_chars=16)
        self._arm_status = Metric("팔 상태 계약", width_chars=16)
        self._tool_fsm = Metric("도구 FSM", width_chars=16)
        self._link_metric = Metric("상태 수신", width_chars=16)
        card.pack(metric_grid(
            (self._arm_fsm, self._arm_status, self._tool_fsm, self._link_metric),
            columns=4,
        ))
        self._reasons = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        card.pack(self._reasons)
        return card.box

    def _on_unmap(self) -> None:
        self.keyboard.set_active(False)
        self.keyboard.release_all()
        self.tool_panels.release_holds()

    # --- public API -----------------------------------------------------
    def attach_keys(self, toplevel: Gtk.Window) -> None:
        """Route teleop keys from ``toplevel``.  Optional; call once."""
        self.keyboard.attach_keys(toplevel)

    def update_state(self, state: C.ArmUiState) -> None:
        """Redraw everything from ``state``.  Safe to call at any rate."""
        state = C.cleared_for_tool(state)
        previous = self._state
        self._state = state

        if state.detected_tool.generation != previous.detected_tool.generation:
            # A confirmed tool change ends every hold before the panels swap.
            self.keyboard.release_all()
            self.tool_panels.release_holds()

        self.gate.apply(state)
        self._update_header(state)
        self._update_tool(state)
        self._update_authority(state)
        self._update_state_card(state)
        self.keyboard.update(state)
        self.tool_panels.update(state)
        self.diagnostics.update(state)
        self._page.show_all()
        self._fixture_banner.set_visible(state.is_fixture)
        if state.developer_mode:
            self._fixture_banner.set_text(
                "개발자 모드 — 제어권·capability UI 게이트 우회. "
                "정지·도구 식별·기존 FSM 안전 게이트는 유지됩니다.",
            )
        elif state.is_fixture:
            self._fixture_banner.set_text(
                "미리보기 fixture 데이터입니다 — 실제 로봇팔 상태가 아닙니다.",
            )
        self._fixture_banner.set_visible(state.is_fixture or state.developer_mode)
        self._mismatch.set_visible(bool(self._mismatch.get_text()))

    def _update_header(self, state: C.ArmUiState) -> None:
        text = console_labels.freshness_korean(state.link.state)
        if state.link.age_s is not None:
            text = f"{text} · {state.link.age_s:.1f}s"
        self._link_badge.set(text, link_tone(state))

    def _update_tool(self, state: C.ArmUiState) -> None:
        tool = state.detected_tool
        if not C.is_live(state):
            # Never redraw a remembered tool as a currently detected one.
            self._detected_name.set_text("감지 대기 — 상태 미수신")
            for metric in (self._tool_id, self._tool_interface,
                           self._tool_attached, self._tool_actuators):
                metric.set("")
            if self._tool_badge is not None:
                self._tool_badge.set("미수신", "status-muted")
        else:
            self._detected_name.set_text(tool.label())
            self._tool_id.set(tool.tool_id)
            self._tool_interface.set(tool.interface)
            attached_text, attached_tone = tri_state(tool.attached, "체결", "미체결")
            self._tool_attached.set(attached_text, attached_tone)
            self._tool_actuators.set(
                ", ".join(str(value) for value in tool.actuator_ids),
            )
            if self._tool_badge is not None:
                if tool.kind == C.TOOL_UNKNOWN:
                    self._tool_badge.set("감지 대기", "status-muted")
                else:
                    self._tool_badge.set(
                        C.TOOL_KIND_KOREAN.get(tool.kind, tool.kind), "status-live",
                    )

        change = state.tool_change
        self._change_badge.set(
            C.REQUEST_KOREAN.get(change.status, change.status),
            request_tone(change.status),
        )
        detail = change.detail
        if change.requested_kind:
            requested = C.TOOL_KIND_KOREAN.get(
                change.requested_kind, change.requested_kind,
            )
            detail = f"요청 대상: {requested}" + (f" · {detail}" if detail else "")
        self._change_detail.set_text(detail)

        mismatch = ""
        if (
            change.requested_kind
            and C.is_live(state)
            and change.requested_kind != tool.kind
            and change.status in (C.REQUEST_COMPLETED, C.REQUEST_REJECTED,
                                  C.REQUEST_TIMEOUT)
        ):
            mismatch = (
                f"요청({C.TOOL_KIND_KOREAN.get(change.requested_kind, change.requested_kind)})과 "
                f"실제 감지({C.TOOL_KIND_KOREAN.get(tool.kind, tool.kind)})가 다릅니다. "
                "화면은 실제 감지 도구를 따릅니다."
            )
            if change.detail:
                mismatch = f"{mismatch} 사유: {change.detail}"
        self._mismatch.set_text(mismatch)

    def _update_authority(self, state: C.ArmUiState) -> None:
        authority = state.authority
        self._requested.set(
            authority.requested_mode,
            "status-warn" if authority.requested_mode else "status-muted",
        )
        granted_tone = "status-live" if C.has_manual_grant(state) else (
            "status-warn" if authority.granted_mode else "status-muted"
        )
        self._granted.set(authority.granted_mode, granted_tone)
        if self._authority_badge is not None:
            self._authority_badge.set(
                C.AUTHORITY_KOREAN.get(authority.status, authority.status),
                authority_tone(authority.status),
            )
        detail = authority.detail
        if authority.status == C.AUTHORITY_REQUESTED:
            detail = detail or "요청을 보냈고 팔 측 승인을 기다리는 중입니다."
        elif authority.status == C.AUTHORITY_TIMEOUT:
            detail = detail or "응답이 없습니다. 다시 요청할 수 있습니다."
        self._authority_detail.set_text(detail)

    def _update_state_card(self, state: C.ArmUiState) -> None:
        live = C.is_live(state)
        tone = "status-live" if live else "status-muted"
        self._arm_fsm.set(state.arm_fsm.text() if live else "", tone)
        self._arm_status.set(state.arm_status.text() if live else "", tone)
        self._tool_fsm.set(state.tool_fsm.text() if live else "", tone)
        self._link_metric.set(
            console_labels.freshness_korean(state.link.state), link_tone(state),
        )

        for child in tuple(self._reasons.get_children()):
            self._reasons.remove(child)
        reasons = list(state.block_reasons)
        gate_reasons = self.gate.reasons(state)
        texts = [reason.korean for reason in reasons]
        for reason in gate_reasons:
            if reason not in texts:
                texts.append(reason)
        if self._state_badge is not None:
            if not live:
                self._state_badge.set("상태 미수신", "status-muted")
            elif texts:
                self._state_badge.set(f"조작 제한 {len(texts)}건", "status-warn")
            else:
                self._state_badge.set("조작 가능", "status-live")
        if not texts:
            self._reasons.pack_start(
                label(
                    "표시할 조작 제한 사유가 없습니다.", "arm-note",
                ), False, False, 0,
            )
        for text in texts:
            self._reasons.pack_start(
                label(text, "arm-reason", wrap=True), False, False, 0,
            )
        self._reasons.show_all()

    def dispose(self) -> None:
        """Release holds, drop handlers and timers.  Idempotent."""
        self.keyboard.set_active(False)
        self.keyboard.dispose()
        self.tool_panels.dispose()
        self.tracker.dispose()
