"""로봇팔·도구 캘리브레이션 탭 — the calibration tab widget.

The tab splits into two areas selected inside the tab itself: 로봇팔 and
현재 도구.

* 로봇팔 runs in a fixed order — 기어비 → 영점 → 가동범위 — and each step
  reports four *separate* outcomes: measured, verified, temporarily applied and
  permanently saved.  They are never merged into one "done", because a
  temporarily applied parameter and a saved range file are different things and
  an operator who confuses them loses work.
* 현재 도구 covers endpoint calibration for whichever tool is actually
  attached: one endpoint pair for a single gripper, two plus a synchronisation
  result for a dual gripper.  A tool with no calibration implementation renders
  as 지원하지 않음 rather than an idle step.

No measurement, arithmetic or persistence happens here.  Buttons report intent
through the injected callbacks; the step states arrive from outside.
"""
from __future__ import annotations

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from .. import labels as console_labels
from . import contracts as C
from .manual_tab import link_tone
from .styling import ARM_UI_STYLE_CLASS, install_arm_ui_css
from .widgets import (
    Badge,
    Card,
    GateGroup,
    HoldButton,
    Metric,
    SignalTracker,
    button,
    divider,
    label,
    metric_grid,
    reflow_row,
    scrolled,
    style,
)


TAB_TITLE = "로봇팔·도구 캘리브레이션"
TAB_NAME = "arm_calibration"

# The arm flow is ordered; a later step cannot be measured before the earlier
# one is done, and the UI says so instead of silently accepting the click.
ARM_STEP_ORDER: tuple[tuple[str, str, str], ...] = (
    ("gear_ratio", "1. 기어비", "축별 기어비 측정과 검증. 결과는 파라미터 임시 적용 대상입니다."),
    ("zero", "2. 영점", "영점 측정과 검증. 기어비 확정 후 수행합니다."),
    ("range", "3. 가동범위", "가동범위 측정. 저장은 범위 파일 저장이며 도구 YAML 저장과 다릅니다."),
)


def step_tone(step: C.CalibrationStep) -> str:
    if not step.supported:
        return "status-muted"
    return {
        C.STEP_DONE: "status-live",
        C.STEP_RUNNING: "status-warn",
        C.STEP_FAILED: "status-bad",
        C.STEP_ABORTED: "status-bad",
    }.get(step.status, "status-muted")


def step_badge_text(step: C.CalibrationStep) -> str:
    if not step.supported:
        return C.STEP_KOREAN[C.STEP_UNSUPPORTED]
    return C.STEP_KOREAN.get(step.status, step.status)


class _ArmStepRow:
    """One arm calibration step: its four outcomes and its four actions."""

    def __init__(
        self,
        key: str,
        title: str,
        description: str,
        callbacks: C.ArmUiCallbacks,
        gate: GateGroup,
        tracker: SignalTracker,
        *,
        prerequisite: str = "",
    ) -> None:
        self.key = key
        self._prerequisite = prerequisite
        self.card = Card(title, description, accent="arm-calibration")
        self._badge = self.card.badge

        self._measured = Metric("측정 결과", width_chars=14)
        self._verified = Metric("검증 결과", width_chars=14)
        self._applied = Metric("임시 적용", width_chars=10)
        self._saved = Metric("영구 저장", width_chars=10)
        self.card.pack(metric_grid(
            (self._measured, self._verified, self._applied, self._saved), columns=4,
        ))

        actions: list[Gtk.Widget] = []
        send = callbacks.arm_calibration_command
        for action, text, css in (
            (C.ACTION_MEASURE, "측정", ""),
            (C.ACTION_VERIFY, "검증", ""),
            (C.ACTION_APPLY_TEMPORARY, "임시 적용", ""),
            (C.ACTION_SAVE, "저장", "arm-primary"),
        ):
            widget = button(text, *(css,) if css else ())
            if send is not None:
                tracker.connect(
                    widget, "clicked",
                    lambda _b, a=action: send(self.key, a),
                )
            gate.add(
                widget, send, C.CAP_ARM_CALIBRATION,
                extra=lambda state, a=action: self._blocked(state, a),
                hint=self._hint(action),
            )
            actions.append(widget)
        self.card.pack(reflow_row(actions, columns=4))
        self._detail = label("", "arm-note", wrap=True)
        self.card.pack(self._detail)
        self.box = self.card.box

    @staticmethod
    def _hint(action: str) -> str:
        return {
            C.ACTION_MEASURE: "측정을 시작합니다",
            C.ACTION_VERIFY: "측정값을 검증합니다",
            C.ACTION_APPLY_TEMPORARY: "임시로만 적용합니다 — 영구 저장이 아닙니다",
            C.ACTION_SAVE: "결과를 영구 저장합니다",
        }.get(action, "")

    def _find(self, state: C.ArmUiState) -> C.CalibrationStep | None:
        return next(
            (step for step in state.calibration.arm_steps if step.key == self.key),
            None,
        )

    def _blocked(self, state: C.ArmUiState, action: str) -> str:
        step = self._find(state)
        if step is not None and not step.supported:
            return "지원하지 않음"
        if self._prerequisite:
            earlier = next(
                (item for item in state.calibration.arm_steps
                 if item.key == self._prerequisite),
                None,
            )
            if earlier is None or earlier.status != C.STEP_DONE:
                return "이전 단계 미완료"
        if step is None:
            return "단계 상태 미수신"
        if action in (C.ACTION_VERIFY, C.ACTION_APPLY_TEMPORARY) and not step.measured:
            return "측정 결과 없음"
        if action == C.ACTION_SAVE and not step.verified:
            return "검증 결과 없음"
        return ""

    def update(self, state: C.ArmUiState) -> None:
        step = self._find(state)
        if step is None:
            for metric in (self._measured, self._verified, self._applied, self._saved):
                metric.set("")
            if self._badge is not None:
                self._badge.set("상태 미수신", "status-muted")
            self._detail.set_text(
                "이 단계의 상태를 수신하지 못했습니다. 미수신을 완료로 표시하지 않습니다.",
            )
            return
        self._measured.set(step.measured)
        self._verified.set(step.verified)
        self._applied.set(
            "적용됨(임시)" if step.applied_temporarily else "미적용",
            "status-warn" if step.applied_temporarily else "status-muted",
        )
        self._saved.set(
            "저장됨" if step.saved else "미저장",
            "status-live" if step.saved else "status-muted",
        )
        if self._badge is not None:
            self._badge.set(step_badge_text(step), step_tone(step))
        detail = step.detail
        if step.applied_temporarily and not step.saved:
            detail = (detail + " " if detail else "") + (
                "임시 적용 상태입니다 — 저장하지 않으면 유지되지 않습니다."
            )
        self._detail.set_text(detail)


class _ToolCalibrationArea:
    """Endpoint calibration for the currently attached tool."""

    def __init__(
        self,
        callbacks: C.ArmUiCallbacks,
        gate: GateGroup,
        tracker: SignalTracker,
    ) -> None:
        self._callbacks = callbacks
        # Session buttons live for the whole tab, so they share the tab's gate
        # and tracker.  The per-target rows are rebuilt whenever the attached
        # tool changes and therefore own a gate and tracker of their own --
        # otherwise every rebuild would pile dead widgets onto the tab's.
        self._gate = gate
        self._tracker = tracker
        self.target_gate = GateGroup()
        self.target_tracker = SignalTracker()
        self.holds: list[HoldButton] = []

        self.card = Card(
            "현재 도구 끝점 보정",
            "장착된 도구에만 적용됩니다. 보정 구현이 없는 도구는 "
            "지원하지 않음으로 표시합니다.",
            accent="tool-calibration",
        )
        self._badge = self.card.badge

        session: list[Gtk.Widget] = []
        send = callbacks.tool_calibration_command
        self._start = button("보정 시작", "arm-primary")
        self._cancel = button("취소", "arm-danger")
        self._verify = button("검증")
        self._save = button("저장", "arm-primary")
        for widget, action, extra in (
            (self._start, C.ACTION_START, self._blocked_start),
            (self._cancel, C.ACTION_CANCEL, self._blocked_in_session),
            (self._verify, C.ACTION_VERIFY, self._blocked_verify),
            (self._save, C.ACTION_SAVE, self._blocked_save),
        ):
            if send is not None:
                tracker.connect(
                    widget, "clicked",
                    lambda _b, a=action: send(self._session_target(), a),
                )
            gate.add(widget, send, C.CAP_TOOL_CALIBRATION, extra=extra)
            session.append(widget)
        self.card.pack(reflow_row(session, columns=4))
        self.card.pack(divider())

        self._targets = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=9)
        self.card.pack(self._targets)

        sync_row = Gtk.Box(spacing=8)
        sync_row.pack_start(label("듀얼 동기화 결과", "arm-metric-label"), False, False, 0)
        self._sync_badge = Badge()
        sync_row.pack_start(self._sync_badge.label, False, False, 0)
        self._sync_note = label("", "arm-note")
        sync_row.pack_start(self._sync_note, False, False, 0)
        self._sync_row = sync_row
        self._sync_row.set_no_show_all(True)
        self.card.pack(sync_row)

        self._unsupported = label("", "arm-reason", wrap=True)
        self._unsupported.set_no_show_all(True)
        self.card.pack(self._unsupported)
        self.box = self.card.box
        self._state = C.default_state()
        self._target_widgets: dict[str, tuple[Metric, Metric]] = {}
        self._built_kind = "<unbuilt>"

    # --- gating predicates ---------------------------------------------
    def _session_target(self) -> str:
        kind = self._state.detected_tool.kind
        return C.TARGET_BOTH if kind == C.TOOL_DUAL_GRIPPER else C.TARGET_SINGLE

    def _supported(self, state: C.ArmUiState) -> bool:
        kind = state.detected_tool.kind
        if kind == C.TOOL_SINGLE_GRIPPER:
            return C.CAP_TOOL_CALIBRATION in state.capabilities
        if kind == C.TOOL_DUAL_GRIPPER:
            return C.CAP_DUAL_TOOL_CALIBRATION in state.capabilities
        return False

    def _blocked_start(self, state: C.ArmUiState) -> str:
        if not self._supported(state):
            return "지원하지 않음 — 이 도구의 보정 구현이 없음"
        if state.calibration.session_active:
            return "이미 보정 세션이 진행 중"
        return ""

    def _blocked_in_session(self, state: C.ArmUiState) -> str:
        if not self._supported(state):
            return "지원하지 않음 — 이 도구의 보정 구현이 없음"
        return "" if state.calibration.session_active else "보정 세션이 시작되지 않음"

    def _blocked_verify(self, state: C.ArmUiState) -> str:
        blocked = self._blocked_in_session(state)
        if blocked:
            return blocked
        steps = state.calibration.tool_steps
        if not steps or not any(step.measured for step in steps):
            return "캡처된 끝점 없음"
        return ""

    def _blocked_save(self, state: C.ArmUiState) -> str:
        blocked = self._blocked_in_session(state)
        if blocked:
            return blocked
        steps = state.calibration.tool_steps
        if not steps or not all(step.verified for step in steps if step.supported):
            return "검증 결과 없음"
        return ""

    def _blocked_capture(self, state: C.ArmUiState) -> str:
        return self._blocked_in_session(state)

    # --- per-target rows ------------------------------------------------
    def _targets_for(self, kind: str) -> tuple[str, ...]:
        if kind == C.TOOL_SINGLE_GRIPPER:
            return (C.TARGET_SINGLE,)
        if kind == C.TOOL_DUAL_GRIPPER:
            return (C.TARGET_LEFT, C.TARGET_RIGHT)
        return ()

    def _rebuild_targets(self, kind: str) -> None:
        for held in self.holds:
            held.force_release()
        self.holds.clear()
        self.target_tracker.dispose()
        self.target_gate = GateGroup()
        for child in tuple(self._targets.get_children()):
            self._targets.remove(child)
            child.destroy()
        self._target_widgets.clear()
        for target in self._targets_for(kind):
            self._targets.pack_start(self._target_row(target), False, False, 0)
        self._built_kind = kind
        self._targets.show_all()

    def _target_row(self, target: str) -> Gtk.Widget:
        row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        row.pack_start(
            label(f"{C.TARGET_KOREAN[target]} 끝점", "arm-card-title"), False, False, 0,
        )
        controls: list[Gtk.Widget] = []
        jog_start = self._callbacks.tool_calibration_jog_start
        jog_stop = self._callbacks.tool_calibration_jog_stop
        for text, direction in (("◀ 조그", -1), ("조그 ▶", +1)):
            held = HoldButton(
                text,
                on_press=(
                    None if jog_start is None
                    else (lambda t=target, d=direction: jog_start(t, d))
                ),
                on_release=(
                    None if jog_stop is None else (lambda t=target: jog_stop(t))
                ),
                tracker=self.target_tracker,
            )
            held.set_sensitive(False)
            self.target_gate.add(
                held, jog_start, C.CAP_TOOL_CALIBRATION,
                extra=self._blocked_capture,
                hint="누르는 동안만 보정용 조그 의도를 전달합니다",
            )
            self.holds.append(held)
            controls.append(held)

        send = self._callbacks.tool_calibration_command
        for text, action in (
            ("열림 캡처", C.ACTION_CAPTURE_OPEN),
            ("닫힘 캡처", C.ACTION_CAPTURE_CLOSE),
        ):
            widget = button(text)
            widget.set_sensitive(False)
            if send is not None:
                self.target_tracker.connect(
                    widget, "clicked",
                    lambda _b, t=target, a=action: send(t, a),
                )
            self.target_gate.add(
                widget, send, C.CAP_TOOL_CALIBRATION, extra=self._blocked_capture,
            )
            controls.append(widget)
        row.pack_start(reflow_row(controls, columns=4), False, False, 0)

        open_metric = Metric("열림 끝점", width_chars=12)
        close_metric = Metric("닫힘 끝점", width_chars=12)
        row.pack_start(
            metric_grid((open_metric, close_metric), columns=2), False, False, 0)
        self._target_widgets[target] = (open_metric, close_metric)
        return row

    def update(self, state: C.ArmUiState) -> None:
        self._state = state
        kind = state.detected_tool.kind
        if kind != self._built_kind:
            self._rebuild_targets(kind)

        supported = self._supported(state)
        self._unsupported.set_text(
            "" if supported else
            f"{C.TOOL_KIND_KOREAN.get(kind, kind)}의 끝점 보정은 지원하지 않습니다. "
            "새 보정 알고리즘 추가는 이 화면의 범위가 아닙니다.",
        )
        self._unsupported.set_visible(not supported)

        steps = {step.key: step for step in state.calibration.tool_steps}
        for target, (open_metric, close_metric) in self._target_widgets.items():
            step = steps.get(target)
            if step is None:
                open_metric.set("")
                close_metric.set("")
                continue
            # ``measured`` carries the captured endpoint pair as the backend
            # words it; the UI shows it verbatim rather than re-deriving units.
            open_metric.set(step.measured)
            close_metric.set(step.verified)

        if self._badge is not None:
            if not supported:
                self._badge.set(C.STEP_KOREAN[C.STEP_UNSUPPORTED], "status-muted")
            elif state.calibration.session_active:
                self._badge.set("세션 진행 중", "status-warn")
            elif state.calibration.unsaved_results:
                self._badge.set("미저장 결과 있음", "status-warn")
            else:
                self._badge.set("세션 없음", "status-muted")

        is_dual = kind == C.TOOL_DUAL_GRIPPER
        self._sync_row.set_visible(is_dual)
        if is_dual:
            sync = state.diagnostics.sync_state
            tone = {
                C.SYNC_SYNCED: "status-live",
                C.SYNC_DRIFT: "status-warn",
                C.SYNC_FAULT: "status-bad",
            }.get(sync, "status-muted")
            self._sync_badge.set(C.SYNC_KOREAN.get(sync, sync), tone)
            error = state.diagnostics.sync_error_deg
            self._sync_note.set_text(
                "" if error is None else f"좌우 편차 {error:.2f}°",
            )

    def release_holds(self) -> None:
        for held in self.holds:
            held.force_release()

    def apply_gate(self, state: C.ArmUiState) -> None:
        self.target_gate.apply(state)

    def dispose(self) -> None:
        self.release_holds()
        self.target_tracker.dispose()


class ArmCalibrationTab:
    """Builds the calibration tab and refreshes it from injected state."""

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

        self._session_note = label("", "arm-reason", wrap=True)
        self._session_note.set_no_show_all(True)
        page.pack_start(self._session_note, False, False, 0)

        arm_area = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=9)
        arm_area.pack_start(
            label(
                "기어비 → 영점 → 가동범위 순서로 진행합니다. "
                "임시 적용과 영구 저장은 구분됩니다.",
                "arm-page-subtitle", wrap=True,
            ),
            False, False, 0,
        )
        self._arm_rows: list[_ArmStepRow] = []
        previous_key = ""
        for key, title, description in ARM_STEP_ORDER:
            row = _ArmStepRow(
                key, title, description, self._callbacks, self.gate, self.tracker,
                prerequisite=previous_key,
            )
            self._arm_rows.append(row)
            arm_area.pack_start(row.box, False, False, 0)
            previous_key = key

        self.tool_area = _ToolCalibrationArea(
            self._callbacks, self.gate, self.tracker,
        )
        tool_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=9)
        tool_box.pack_start(self.tool_area.box, False, False, 0)

        self.inner_stack = Gtk.Stack()
        self.inner_stack.set_transition_type(Gtk.StackTransitionType.NONE)
        self.inner_stack.set_hhomogeneous(False)
        self.inner_stack.set_vhomogeneous(False)
        self.inner_stack.add_titled(arm_area, "arm", "로봇팔")
        self.inner_stack.add_titled(tool_box, "tool", "현재 도구")
        switcher = Gtk.StackSwitcher()
        switcher.set_stack(self.inner_stack)
        switcher.set_halign(Gtk.Align.START)
        selector = Gtk.Box(spacing=8)
        style(selector, "nav")
        selector.pack_start(switcher, False, False, 0)
        page.pack_start(selector, False, False, 0)
        page.pack_start(self.inner_stack, False, False, 0)

        self._page = page
        self.widget = scrolled(page)
        style(self.widget, ARM_UI_STYLE_CLASS)
        self.tracker.connect(page, "unmap", lambda _w: self.tool_area.release_holds())

        self.update_state(self._state)

    def _build_header(self) -> Gtk.Widget:
        header = Gtk.Box(spacing=12)
        copy = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
        copy.set_hexpand(True)
        copy.pack_start(label(TAB_TITLE, "arm-page-title", wrap=True), False, False, 0)
        copy.pack_start(
            label(
                "측정 · 검증 · 임시 적용 · 저장을 구분해 표시합니다. "
                "이 화면은 계산이나 저장을 직접 수행하지 않습니다.",
                "arm-page-subtitle", wrap=True,
            ),
            False, False, 0,
        )
        header.pack_start(copy, True, True, 0)
        self._link_badge = Badge("상태 미수신")
        header.pack_end(self._link_badge.label, False, False, 0)
        return header

    def update_state(self, state: C.ArmUiState) -> None:
        state = C.cleared_for_tool(state)
        self._state = state
        if not C.is_live(state):
            self.tool_area.release_holds()
        text = console_labels.freshness_korean(state.link.state)
        if state.link.age_s is not None:
            text = f"{text} · {state.link.age_s:.1f}s"
        self._link_badge.set(text, link_tone(state))
        for row in self._arm_rows:
            row.update(state)
        # The tool area rebuilds its per-target rows when the tool changes, so
        # the gate must run *after* it -- a freshly built button that has never
        # been gated would otherwise sit enabled until the next update.
        self.tool_area.update(state)
        self.gate.apply(state)
        self.tool_area.apply_gate(state)

        notes: list[str] = []
        calibration = state.calibration
        if calibration.session_active:
            target = calibration.session_target or "보정"
            notes.append(
                f"{target} 세션 진행 중 — 일반 조그와 도구 변경이 차단됩니다.",
            )
        if calibration.unsaved_results:
            notes.append("저장되지 않은 보정 결과가 있습니다.")
        if not C.is_live(state) and (calibration.session_active
                                     or calibration.unsaved_results):
            notes.append(
                "통신이 끊긴 상태입니다. 진행 상태를 완료로 표시하지 않습니다.",
            )
        if calibration.detail:
            notes.append(calibration.detail)
        self._session_note.set_text(" ".join(notes))
        self._page.show_all()
        self._session_note.set_visible(bool(notes))
        self._fixture_banner.set_visible(state.is_fixture)

    @property
    def gated_widgets(self) -> tuple:
        """Every gated control in the tab, including the rebuilt tool rows."""
        return self.gate.widgets + self.tool_area.target_gate.widgets

    def dispose(self) -> None:
        self.tool_area.dispose()
        self.tracker.dispose()
