"""Per-tool control panels and the current-tool diagnostics view.

A single gripper and a dual gripper get genuinely different panels, not one
command panel wearing two skins: the dual panel addresses each side
independently, adds a synchronised jog, and carries a synchronisation verdict
that has no meaning for a single gripper.  Tools that are not grippers
(청소 모듈 · 환경 센서 모듈) get an information-only panel -- cleaner direction
control and any developer drive path are deliberately not offered here.

Only the active tool is ever shown.  :func:`contracts.cleared_for_tool` drops
any reading whose actuator id or generation does not match the detected tool,
so a late reply for the previous tool blanks the panel instead of leaving stale
numbers behind.
"""
from __future__ import annotations

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from . import contracts as C
from .widgets import (
    Badge,
    Card,
    GateGroup,
    HoldButton,
    Metric,
    SignalTracker,
    divider,
    format_number,
    format_ratio,
    label,
    metric_grid,
    reflow,
    reflow_row,
    style,
    tri_state,
)


class _ToolPanelBase:
    """Common plumbing: one tracker, one gate group, one root box."""

    kind = C.TOOL_UNKNOWN

    def __init__(self, callbacks: C.ArmUiCallbacks) -> None:
        self._callbacks = callbacks
        self.tracker = SignalTracker()
        self.gate = GateGroup()
        self.holds: list[HoldButton] = []
        self.box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=9)
        self.box.set_hexpand(True)

    def _hold(
        self, text: str, target: str, direction: int,
    ) -> HoldButton:
        start = self._callbacks.tool_jog_start
        stop = self._callbacks.tool_jog_stop
        held = HoldButton(
            text,
            on_press=(None if start is None else (lambda: start(target, direction))),
            on_release=(None if stop is None else (lambda: stop(target))),
            tracker=self.tracker,
        )
        self.gate.add(
            held, start, C.CAP_GRIPPER_COMMAND,
            hint="누르는 동안만 조그 의도를 전달합니다",
        )
        self.holds.append(held)
        return held

    def _command(self, text: str, target: str, command: str, *classes: str) -> Gtk.Button:
        widget = Gtk.Button(label=text)
        style(widget, *classes)
        send = self._callbacks.tool_command
        if send is not None:
            self.tracker.connect(
                widget, "clicked", lambda _b: send(target, command),
            )
        self.gate.add(widget, send, C.CAP_GRIPPER_COMMAND)
        return widget

    def release_holds(self) -> None:
        """End every in-flight jog -- called on tool change, blur and dispose."""
        for held in self.holds:
            held.force_release()

    def update(self, state: C.ArmUiState) -> None:
        self.gate.apply(state)

    def clear(self) -> None:
        """Blank every reading in this panel.

        A ``Gtk.Stack`` keeps its non-visible children alive, so a panel that
        is merely switched away from still holds the previous tool's actuator
        ids and numbers -- and would flash them the moment that tool came back.
        Re-rendering from the disconnected default is what actually empties it.
        """
        self.update(C.default_state())

    def dispose(self) -> None:
        self.release_holds()
        self.tracker.dispose()


class SingleGripperPanel(_ToolPanelBase):
    """단일 그리퍼: one actuator, one open/close axis."""

    kind = C.TOOL_SINGLE_GRIPPER

    def __init__(self, callbacks: C.ArmUiCallbacks) -> None:
        super().__init__(callbacks)
        card = Card(
            "단일 그리퍼 조작",
            "열기 · 닫기 · 정지와 전용 조그. 토크 인가가 필요한 보정 조작은 "
            "캘리브레이션 탭에 있습니다.",
            accent="gripper",
        )
        self._badge = card.badge

        card.pack(reflow_row([
            self._command("열기", C.TARGET_SINGLE, C.COMMAND_OPEN),
            self._command("닫기", C.TARGET_SINGLE, C.COMMAND_CLOSE),
            self._command("정지", C.TARGET_SINGLE, C.COMMAND_STOP, "arm-danger"),
        ], columns=3))
        card.pack(reflow_row([
            label("전용 조그", "arm-metric-label"),
            self._hold("◀ 닫힘 방향", C.TARGET_SINGLE, -1),
            self._hold("열림 방향 ▶", C.TARGET_SINGLE, +1),
        ], columns=3))
        card.pack(divider())

        self._position = Metric("현재 위치")
        self._opening = Metric("열림 정도")
        self._load = Metric("부하")
        self._torque = Metric("토크")
        card.pack(metric_grid(
            (self._position, self._opening, self._load, self._torque), columns=4,
        ))
        self.box.pack_start(card.box, False, False, 0)

    def update(self, state: C.ArmUiState) -> None:
        super().update(state)
        motors = state.diagnostics.motors
        motor = motors[0] if motors else None
        if motor is None:
            for metric in (self._position, self._opening, self._load, self._torque):
                metric.set("")
            if self._badge is not None:
                self._badge.set("판독 없음", "status-muted")
            return
        self._position.set(format_number(motor.position_deg, "°"))
        self._opening.set(format_ratio(motor.opening_ratio))
        self._load.set(format_number(motor.load_percent, "%", 0))
        torque_text, torque_tone = tri_state(motor.torque_on, "인가", "해제")
        self._torque.set(torque_text, torque_tone)
        if self._badge is not None:
            if motor.error:
                self._badge.set(motor.error, "status-bad")
            elif motor.online is None:
                self._badge.set("온라인 여부 미보고", "status-muted")
            elif motor.online:
                self._badge.set(f"ID {motor.actuator_id} 온라인", "status-live")
            else:
                self._badge.set(f"ID {motor.actuator_id} 오프라인", "status-bad")


class DualGripperPanel(_ToolPanelBase):
    """듀얼 그리퍼: two actuators addressed separately, plus a synced jog."""

    kind = C.TOOL_DUAL_GRIPPER

    def __init__(self, callbacks: C.ArmUiCallbacks) -> None:
        super().__init__(callbacks)
        card = Card(
            "듀얼 그리퍼 조작",
            "좌·우를 개별 지령하고, 동기 조그는 별도 경로로 전달합니다. "
            "동기화 판정은 백엔드 값을 그대로 표시합니다.",
            accent="gripper",
        )
        self._badge = card.badge

        sides = reflow(2)
        sides.set_row_spacing(9)
        sides.set_column_spacing(9)
        self._side_metrics: dict[str, tuple[Metric, Metric, Metric]] = {}
        for target in (C.TARGET_LEFT, C.TARGET_RIGHT):
            column = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
            column.pack_start(
                label(f"{C.TARGET_KOREAN[target]} 그리퍼", "arm-card-title"),
                False, False, 0,
            )
            column.pack_start(reflow_row([
                self._command("열기", target, C.COMMAND_OPEN),
                self._command("닫기", target, C.COMMAND_CLOSE),
                self._command("정지", target, C.COMMAND_STOP, "arm-danger"),
                self._hold("◀", target, -1),
                self._hold("▶", target, +1),
            ], columns=5), False, False, 0)
            position = Metric("위치")
            opening = Metric("열림 정도")
            load = Metric("부하")
            column.pack_start(
                metric_grid((position, opening, load), columns=3), False, False, 0)
            self._side_metrics[target] = (position, opening, load)
            column.set_valign(Gtk.Align.START)
            sides.add(column)
        card.pack(sides)
        card.pack(divider())

        # The row label already says 동기, so the buttons do not repeat it --
        # this row is the widest in the tab and the shorter labels are what
        # keep the page inside a narrow window.
        card.pack(reflow_row([
            label("동기 조작", "arm-metric-label"),
            self._sync_command("열기", C.COMMAND_OPEN),
            self._sync_command("닫기", C.COMMAND_CLOSE),
            self._sync_command("정지", C.COMMAND_STOP, "arm-danger"),
            self._sync_hold("◀ 조그", -1),
            self._sync_hold("조그 ▶", +1),
        ], columns=6))

        sync_row = Gtk.Box(spacing=8)
        sync_row.pack_start(label("동기화 상태", "arm-metric-label"), False, False, 0)
        self._sync_badge = Badge()
        sync_row.pack_start(self._sync_badge.label, False, False, 0)
        self._sync_error = label("", "arm-note")
        sync_row.pack_start(self._sync_error, False, False, 0)
        card.pack(sync_row)
        self.box.pack_start(card.box, False, False, 0)

    def _sync_command(self, text: str, command: str, *classes: str) -> Gtk.Button:
        widget = Gtk.Button(label=text)
        style(widget, *classes)
        send = self._callbacks.tool_command
        if send is not None:
            self.tracker.connect(
                widget, "clicked", lambda _b: send(C.TARGET_BOTH, command),
            )
        self.gate.add(widget, send, C.CAP_DUAL_SYNC)
        return widget

    def _sync_hold(self, text: str, direction: int) -> HoldButton:
        start = self._callbacks.tool_jog_start
        stop = self._callbacks.tool_jog_stop
        held = HoldButton(
            text,
            on_press=(
                None if start is None
                else (lambda: start(C.TARGET_BOTH, direction))
            ),
            on_release=(None if stop is None else (lambda: stop(C.TARGET_BOTH))),
            tracker=self.tracker,
        )
        self.gate.add(
            held, start, C.CAP_DUAL_SYNC,
            hint="누르는 동안만 양쪽 동기 조그 의도를 전달합니다",
        )
        self.holds.append(held)
        return held

    def update(self, state: C.ArmUiState) -> None:
        super().update(state)
        motors = state.diagnostics.motors
        # Left/right come from the reading's own role when the backend states
        # it; ordering is only the fallback so a role-less feed still renders.
        by_target: dict[str, C.ToolMotorReading] = {}
        for index, motor in enumerate(motors[:2]):
            target = motor.role if motor.role in (
                C.TARGET_LEFT, C.TARGET_RIGHT,
            ) else (C.TARGET_LEFT if index == 0 else C.TARGET_RIGHT)
            by_target.setdefault(target, motor)
        for target, (position, opening, load) in self._side_metrics.items():
            motor = by_target.get(target)
            if motor is None:
                position.set("")
                opening.set("")
                load.set("")
                continue
            position.set(format_number(motor.position_deg, "°"))
            opening.set(format_ratio(motor.opening_ratio))
            load.set(format_number(motor.load_percent, "%", 0))

        sync = state.diagnostics.sync_state
        tone = {
            C.SYNC_SYNCED: "status-live",
            C.SYNC_DRIFT: "status-warn",
            C.SYNC_FAULT: "status-bad",
        }.get(sync, "status-muted")
        self._sync_badge.set(C.SYNC_KOREAN.get(sync, sync), tone)
        error = state.diagnostics.sync_error_deg
        self._sync_error.set_text(
            "" if error is None else f"좌우 편차 {error:.2f}°",
        )
        if self._badge is not None:
            ids = ", ".join(str(motor.actuator_id) for motor in motors)
            if not motors:
                self._badge.set("판독 없음", "status-muted")
            else:
                self._badge.set(f"ID {ids}", "status-live")


class InformationOnlyToolPanel(_ToolPanelBase):
    """청소 모듈 · 환경 센서 모듈 · 감지 대기: information, no actuation.

    Gripper controls are not merely disabled here, they are absent -- the tool
    has none.  Cleaner direction control and developer drive paths are out of
    scope for this UI by instruction, and the panel says so rather than leaving
    the operator to wonder where the buttons went.
    """

    kind = C.TOOL_UNKNOWN

    def __init__(self, callbacks: C.ArmUiCallbacks) -> None:
        super().__init__(callbacks)
        card = Card(
            "현재 도구 정보",
            "이 도구에는 그리퍼 조작이 없습니다. 청소 모듈 방향 제어와 "
            "개발자 직접 구동은 이 화면의 범위가 아닙니다.",
            accent="gripper",
        )
        self._badge = card.badge
        self._kind = Metric("도구 종류", width_chars=14)
        self._tool_id = Metric("도구 ID", width_chars=12)
        self._interface = Metric("인터페이스", width_chars=14)
        self._attached = Metric("장착")
        card.pack(metric_grid(
            (self._kind, self._tool_id, self._interface, self._attached), columns=4,
        ))
        self._note = label("", "arm-note", wrap=True)
        card.pack(self._note)
        self.box.pack_start(card.box, False, False, 0)

    def update(self, state: C.ArmUiState) -> None:
        super().update(state)
        tool = state.detected_tool
        self._kind.set(
            C.TOOL_KIND_KOREAN.get(tool.kind, tool.kind),
            "status-muted" if tool.kind == C.TOOL_UNKNOWN else "status-live",
        )
        self._tool_id.set(tool.tool_id)
        self._interface.set(tool.interface)
        attached_text, attached_tone = tri_state(tool.attached, "체결", "미체결")
        self._attached.set(attached_text, attached_tone)
        if tool.kind == C.TOOL_ENVIRONMENT_SENSOR:
            self._note.set_text(
                "환경 센서 모듈의 측정값은 기존 환경 텔레메트리 경로에 그대로 "
                "남아 있습니다. 이 탭은 장착·상태만 표시합니다.",
            )
        elif tool.kind == C.TOOL_CLEANER:
            self._note.set_text(
                "청소 모듈은 장착·상태 표시만 제공합니다.",
            )
        else:
            self._note.set_text(
                "활성 도구가 관측되지 않았습니다. 임의의 도구를 장착된 것으로 "
                "표시하지 않습니다.",
            )
        if self._badge is not None:
            if tool.kind == C.TOOL_UNKNOWN:
                self._badge.set("감지 대기", "status-muted")
            else:
                self._badge.set("조작 없음", "status-muted")


class CleanerPanel(_ToolPanelBase):
    """Existing cleaner FSM only: LEFT/RIGHT/STOP, never raw motor drive."""

    kind = C.TOOL_CLEANER

    def __init__(self, callbacks: C.ArmUiCallbacks) -> None:
        super().__init__(callbacks)
        card = Card(
            "청소 모듈 조작",
            "기존 엔드이펙터 FSM의 좌·우·정지 명령만 전달합니다.",
            accent="gripper",
        )
        card.pack(reflow_row([
            self._command("좌회전", C.TARGET_BOTH, C.COMMAND_LEFT),
            self._command("우회전", C.TARGET_BOTH, C.COMMAND_RIGHT),
            self._command("정지", C.TARGET_BOTH, C.COMMAND_STOP, "arm-danger"),
        ], columns=3))
        self.box.pack_start(card.box, False, False, 0)


class ToolPanelStack:
    """Shows exactly one tool panel, chosen by the detected tool kind.

    Switching panels force-releases every hold in the outgoing panel first, so
    a jog started on the previous tool cannot survive the change.
    """

    def __init__(self, callbacks: C.ArmUiCallbacks) -> None:
        self.single = SingleGripperPanel(callbacks)
        self.dual = DualGripperPanel(callbacks)
        self.cleaner = CleanerPanel(callbacks)
        self.other = InformationOnlyToolPanel(callbacks)
        self._panels = {
            C.TOOL_SINGLE_GRIPPER: self.single,
            C.TOOL_DUAL_GRIPPER: self.dual,
            C.TOOL_CLEANER: self.cleaner,
        }
        self.stack = Gtk.Stack()
        self.stack.set_transition_type(Gtk.StackTransitionType.NONE)
        self.stack.set_hhomogeneous(False)
        self.stack.set_vhomogeneous(False)
        self.stack.add_named(self.single.box, C.TOOL_SINGLE_GRIPPER)
        self.stack.add_named(self.dual.box, C.TOOL_DUAL_GRIPPER)
        self.stack.add_named(self.cleaner.box, C.TOOL_CLEANER)
        self.stack.add_named(self.other.box, "other")
        self._active_name = "other"
        self.stack.set_visible_child_name("other")

    @property
    def active_name(self) -> str:
        return self._active_name

    @property
    def active_panel(self) -> _ToolPanelBase:
        return self._panels.get(self._active_name, self.other)

    def update(self, state: C.ArmUiState) -> None:
        name = (
            state.detected_tool.kind
            if state.detected_tool.kind in self._panels
            else "other"
        )
        if name != self._active_name:
            outgoing = self.active_panel
            outgoing.release_holds()
            outgoing.clear()
            self._active_name = name
            self.stack.set_visible_child_name(name)
        # Only the visible panel is refreshed: an off-screen panel holding a
        # previous tool's numbers would be a leak waiting for the next switch.
        self.active_panel.update(state)

    def release_holds(self) -> None:
        for panel in (self.single, self.dual, self.cleaner, self.other):
            panel.release_holds()

    def dispose(self) -> None:
        for panel in (self.single, self.dual, self.cleaner, self.other):
            panel.dispose()


class ToolDiagnosticsPanel:
    """Current-tool diagnostics, collapsible, rebuilt from scratch each update.

    Rebuilding rather than updating in place is what guarantees no row from a
    previous tool can survive: there is no widget to leave behind.
    """

    def __init__(self) -> None:
        self.card = Card(
            "현재 도구 진단",
            "활성 도구의 actuator만 표시합니다. 다른 도구의 값은 남기지 않습니다.",
            accent="diagnostics",
        )
        self.expander = Gtk.Expander(label="모터별 상세")
        self.expander.set_expanded(False)
        self._rows = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self.expander.add(self._rows)
        self.card.pack(self.expander)
        self._empty = label("", "arm-note", wrap=True)
        self.card.pack(self._empty)
        self.box = self.card.box

    def update(self, state: C.ArmUiState) -> None:
        for child in tuple(self._rows.get_children()):
            self._rows.remove(child)
        motors = state.diagnostics.motors
        badge = self.card.badge
        if not motors:
            self._empty.set_text(
                "표시할 진단 값이 없습니다. 미수신 상태를 정상으로 표시하지 않습니다.",
            )
            if badge is not None:
                badge.set("판독 없음", "status-muted")
            self._rows.show_all()
            return
        self._empty.set_text("")
        faults = sum(1 for motor in motors if motor.error)
        if badge is not None:
            if faults:
                badge.set(f"오류 {faults}건", "status-bad")
            else:
                badge.set(f"{len(motors)}축 수신", "status-live")
        for motor in motors:
            self._rows.pack_start(self._row(motor), False, False, 0)
        self._rows.show_all()

    @staticmethod
    def _row(motor: C.ToolMotorReading) -> Gtk.Box:
        row = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        header = Gtk.Box(spacing=8)
        title = f"ID {motor.actuator_id}"
        if motor.role:
            title = f"{title} · {C.TARGET_KOREAN.get(motor.role, motor.role)}"
        header.pack_start(label(title, "arm-card-title"), True, True, 0)
        online_text, online_tone = tri_state(motor.online, "온라인", "오프라인")
        online_badge = Badge(online_text or "온라인 여부 미보고", online_tone)
        header.pack_end(online_badge.label, False, False, 0)
        row.pack_start(header, False, False, 0)

        metrics = [
            ("위치", format_number(motor.position_deg, "°")),
            ("열림 정도", format_ratio(motor.opening_ratio)),
            ("부하", format_number(motor.load_percent, "%", 0)),
            ("온도", format_number(motor.temperature_c, "℃", 0)),
        ]
        tiles: list[Metric] = []
        for title_text, value in metrics:
            tile = Metric(title_text)
            tile.set(value)
            tiles.append(tile)
        torque = Metric("토크")
        torque_text, torque_tone = tri_state(motor.torque_on, "인가", "해제")
        torque.set(torque_text, torque_tone)
        tiles.append(torque)
        mode = Metric("운전 모드", width_chars=12)
        mode.set(motor.operating_mode)
        tiles.append(mode)
        row.pack_start(metric_grid(tiles, columns=3), False, False, 0)
        if motor.error:
            row.pack_start(label(motor.error, "arm-note", "status-bad", wrap=True),
                           False, False, 0)
        return row
