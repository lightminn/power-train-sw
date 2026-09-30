"""Keyboard-teleop area: axis selection, jog, speed, shortcuts, pose management.

What this panel does: it shows the selected axis, each axis' current angle, the
commanded speed, the shortcut sheet and the saved-pose list, and it reports
operator intent through the injected callbacks.

What it deliberately does not do: decide the wire format of a jog, own the arm,
or claim its shortcut table is authoritative.  :data:`KEY_BINDINGS` is the
console-side presentation of the bindings and is marked provisional in the UI
itself -- reconciling it against the arm's own ``teleop_vocab`` (and producing
the gap table) is the data-correspondence work, not this skeleton's.

The hold contract is the delicate part.  A jog must never outlive its press, so
every exit path is wired: key release, pointer leave, focus-out, the tab being
switched away (``unmap``), the window closing, the feed going non-LIVE, and the
MANUAL grant being withdrawn.  :meth:`KeyboardTeleopPanel.release_all` is the
single funnel and is idempotent.
"""
from __future__ import annotations

from typing import Callable

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gtk  # noqa: E402

from . import contracts as C
from .widgets import (
    Card,
    GateGroup,
    HoldButton,
    Metric,
    SignalTracker,
    button,
    divider,
    format_number,
    label,
    metric_grid,
    reflow_row,
    shortcut_row,
    style,
)


# Provisional console-side binding table.  Meaning column, not wire format.
KEY_BINDINGS: tuple[tuple[str, str], ...] = (
    ("1 – 6", "축 선택"),
    ("↑ / W", "선택 축 + 방향 조그 (누르는 동안)"),
    ("↓ / S", "선택 축 − 방향 조그 (누르는 동안)"),
    ("[ / ]", "명령 속도 감소 / 증가"),
    ("Space", "정지 (기존 stop 의미: 토크 해제)"),
    ("T", "복귀 (현재 자세 유지)"),
    ("H", "home (저장된 자세)"),
)

# Axis count: 1–5 are arm joints; 6 is the legacy gripper axis and is offered
# only when the active tool actually supports it, never remapped onto a tool
# that does not have that joint.
ARM_AXIS_COUNT = 5
GRIPPER_AXIS_INDEX = 6


def _keyval_to_intent(keyval: int) -> tuple[str, object] | None:
    """Translate a keyval into a UI intent, or ``None`` to leave it alone."""
    name = Gdk.keyval_name(keyval) or ""
    lowered = name.lower()
    if lowered in {"1", "2", "3", "4", "5", "6"}:
        return ("axis", int(lowered))
    if lowered in {"up", "w"}:
        return ("jog", +1)
    if lowered in {"down", "s"}:
        return ("jog", -1)
    if lowered == "bracketleft":
        return ("speed", -1)
    if lowered == "bracketright":
        return ("speed", +1)
    if lowered == "space":
        return ("stop", None)
    if lowered == "t":
        return ("resume", None)
    if lowered == "h":
        return ("home", None)
    return None


def focus_is_text_entry(toplevel: Gtk.Window | None) -> bool:
    """True when typing should reach a text widget instead of the arm.

    Without this the pose-name field would be unusable: every arrow key and
    every letter would be swallowed as a teleop command.
    """
    if toplevel is None:
        return False
    focused = toplevel.get_focus()
    return isinstance(
        focused, (Gtk.Entry, Gtk.TextView, Gtk.SearchEntry, Gtk.SpinButton),
    )


class KeyboardTeleopPanel:
    """The keyboard-teleop card plus its optional window-level key handling."""

    def __init__(self, callbacks: C.ArmUiCallbacks) -> None:
        self._callbacks = callbacks
        self.tracker = SignalTracker()
        self.gate = GateGroup()
        self._state = C.default_state()
        self._active = False
        self._key_jog_axis: int | None = None
        self._key_handlers: list[tuple[Gtk.Window, int]] = []
        # Guards the exclusive axis toggles against re-entrant "toggled"
        # signals while the panel is writing them from state.
        self._suppress_axis_signal = False

        self.box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=9)
        card = Card(
            "키보드 텔레옵",
            "축을 고르고 누르는 동안만 조그 의도를 전달합니다. "
            "차체 게임패드 입력과는 대상·권한이 분리됩니다.",
            accent="teleop",
        )
        self._badge = card.badge

        self._axis_buttons: dict[int, Gtk.ToggleButton] = {}
        axis_widgets: list[Gtk.Widget] = [label("축 선택", "arm-metric-label")]
        for index in range(1, GRIPPER_AXIS_INDEX + 1):
            toggle = Gtk.ToggleButton(label=str(index))
            style(toggle, "arm-axis")
            self._wire_axis(toggle, index)
            axis_widgets.append(toggle)
            self._axis_buttons[index] = toggle
        card.pack(reflow_row(axis_widgets, columns=7))

        self._jog_minus = self._jog_hold("− 방향", -1)
        self._jog_plus = self._jog_hold("+ 방향", +1)
        self._slower = self._simple("속도 −", callbacks.change_speed, C.CAP_TELEOP_JOG,
                                    lambda: callbacks.change_speed(-1))
        self._faster = self._simple("속도 +", callbacks.change_speed, C.CAP_TELEOP_JOG,
                                    lambda: callbacks.change_speed(+1))
        card.pack(reflow_row(
            [
                label("조그", "arm-metric-label"),
                self._jog_minus, self._jog_plus, self._slower, self._faster,
            ],
            columns=5,
        ))

        self._stop = self._simple(
            "정지 (Space)", callbacks.stop_motion, C.CAP_TELEOP_JOG,
            callbacks.stop_motion, "arm-danger",
        )
        self._resume = self._simple(
            "복귀 (T)", callbacks.resume_hold, C.CAP_TELEOP_JOG, callbacks.resume_hold,
        )
        self._home = self._simple(
            "home (H)", callbacks.go_home, C.CAP_TELEOP_POSE, callbacks.go_home,
        )
        card.pack(reflow_row([self._stop, self._resume, self._home], columns=3))
        card.pack(divider())

        self._selected = Metric("선택 축", width_chars=12)
        self._angle = Metric("현재각")
        self._speed = Metric("명령 속도", width_chars=12)
        self._moving = Metric("동작 상태", width_chars=12)
        card.pack(metric_grid(
            (self._selected, self._angle, self._speed, self._moving), columns=4,
        ))

        self._angles = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        angles_expander = Gtk.Expander(label="축별 현재각")
        angles_expander.set_expanded(False)
        angles_expander.add(self._angles)
        card.pack(angles_expander)

        shortcuts = Gtk.Expander(label="단축키 안내 (원본 TUI 대조 전 잠정 표기)")
        shortcuts.set_expanded(False)
        shortcut_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
        shortcut_box.pack_start(shortcut_row(KEY_BINDINGS), False, False, 0)
        shortcut_box.pack_start(
            label(
                "표기된 키는 콘솔 화면의 잠정 안내입니다. 실제 바인딩·누락 여부는 "
                "팔 측 텔레옵 정본과의 대조 결과로 확정합니다.",
                "arm-note", "status-warn", wrap=True,
            ),
            False, False, 0,
        )
        shortcuts.add(shortcut_box)
        card.pack(shortcuts)
        self.box.pack_start(card.box, False, False, 0)

        self.box.pack_start(self._build_pose_card(), False, False, 0)

    # --- construction helpers ------------------------------------------
    def _wire_axis(self, toggle: Gtk.ToggleButton, index: int) -> None:
        select = self._callbacks.select_axis

        def on_toggled(widget: Gtk.ToggleButton) -> None:
            if self._suppress_axis_signal:
                return
            if not widget.get_active():
                # Axis selection is exclusive; re-clicking the active axis must
                # not leave the operator with no axis at all.
                self._suppress_axis_signal = True
                widget.set_active(True)
                self._suppress_axis_signal = False
                return
            self.release_all()
            if select is not None:
                select(index)

        self.tracker.connect(toggle, "toggled", on_toggled)
        self.gate.add(
            toggle, select, C.CAP_TELEOP_JOG,
            extra=lambda state, axis=index: (
                "현재 도구가 6번 축을 지원하지 않음"
                if axis == GRIPPER_AXIS_INDEX
                and state.detected_tool.kind not in C.GRIPPER_TOOL_KINDS
                else ""
            ),
            hint="축을 선택합니다. 선택만으로 명령이 나가지 않습니다.",
        )

    def _jog_hold(self, text: str, direction: int) -> HoldButton:
        start = self._callbacks.jog_start
        stop = self._callbacks.jog_stop
        held = HoldButton(
            text,
            on_press=(None if start is None else (lambda: self._start_jog(direction))),
            on_release=(None if stop is None else self._stop_jog),
            tracker=self.tracker,
        )
        self.gate.add(
            held, start, C.CAP_TELEOP_JOG,
            extra=lambda state: (
                "" if state.teleop.selected_axis else "선택된 축 없음"
            ),
            hint="누르는 동안만 조그 의도를 전달합니다",
        )
        return held

    def _simple(
        self,
        text: str,
        callback: C.Callback,
        capability: str,
        action: Callable[[], object] | None,
        *classes: str,
        extra: Callable[[C.ArmUiState], str] | None = None,
    ) -> Gtk.Button:
        widget = button(text, *classes)
        if callback is not None and action is not None:
            self.tracker.connect(widget, "clicked", lambda _b: action())
        self.gate.add(widget, callback, capability, extra=extra)
        return widget

    def _build_pose_card(self) -> Gtk.Widget:
        card = Card(
            "자세 관리",
            "저장된 자세의 목록·이동·삭제. 측정·보정 세션은 캘리브레이션 탭에 "
            "있으며 이 화면에서는 시작되지 않습니다.",
            accent="teleop",
            with_badge=False,
        )
        self._pose_name = Gtk.Entry()
        self._pose_name.set_placeholder_text("저장할 자세 이름")
        self._pose_name.set_width_chars(14)
        self._pose_save = self._simple(
            "현재 자세 저장", self._callbacks.save_pose, C.CAP_TELEOP_POSE,
            lambda: self._callbacks.save_pose(self._pose_name.get_text().strip()),
            extra=lambda _state: (
                "" if self._pose_name.get_text().strip() else "자세 이름 없음"
            ),
        )
        card.pack(reflow_row([self._pose_name, self._pose_save], columns=2))
        # The save button's own precondition is the entry's text, which no
        # state update touches, so re-gate on every keystroke.
        self.tracker.connect(
            self._pose_name, "changed", lambda _e: self.gate.apply(self._state),
        )

        self._pose_list = Gtk.ComboBoxText()
        self._pose_list.set_tooltip_text("저장된 자세를 고릅니다. 선택만으로 이동하지 않습니다.")

        needs_selection: Callable[[C.ArmUiState], str] = lambda _state: (
            "" if self._pose_list.get_active_text() else "선택된 자세 없음"
        )
        self._pose_move = self._simple(
            "선택 자세로 이동", self._callbacks.move_to_pose, C.CAP_TELEOP_POSE,
            lambda: self._with_selected_pose(self._callbacks.move_to_pose),
            extra=needs_selection,
        )
        self._pose_delete = self._simple(
            "삭제", self._callbacks.delete_pose, C.CAP_TELEOP_POSE,
            lambda: self._with_selected_pose(self._callbacks.delete_pose),
            "arm-danger",
            extra=needs_selection,
        )
        card.pack(reflow_row(
            [self._pose_list, self._pose_move, self._pose_delete], columns=3,
        ))
        self.tracker.connect(
            self._pose_list, "changed", lambda _c: self.gate.apply(self._state),
        )
        return card.box

    def _with_selected_pose(self, callback: C.Callback) -> None:
        name = self._pose_list.get_active_text()
        if callback is not None and name:
            callback(name)

    # --- jog lifecycle --------------------------------------------------
    def _start_jog(self, direction: int) -> None:
        axis = self._state.teleop.selected_axis
        start = self._callbacks.jog_start
        if axis is None or start is None:
            return
        self._key_jog_axis = axis
        start(axis, direction)

    def _stop_jog(self) -> None:
        axis = self._key_jog_axis
        self._key_jog_axis = None
        stop = self._callbacks.jog_stop
        if axis is not None and stop is not None:
            stop(axis)

    def release_all(self) -> None:
        """End every hold and key-driven jog.  Idempotent by construction."""
        self._jog_minus.force_release()
        self._jog_plus.force_release()
        self._stop_jog()

    # --- window-level key handling -------------------------------------
    def attach_keys(self, toplevel: Gtk.Window) -> None:
        """Listen for teleop keys on ``toplevel`` while this panel is active.

        Handlers are recorded so :meth:`dispose` removes them from a window
        that outlives the tab.
        """
        press = toplevel.connect("key-press-event", self._on_key_press)
        release = toplevel.connect("key-release-event", self._on_key_release)
        focus_out = toplevel.connect(
            "focus-out-event", lambda _w, _e: (self.release_all(), False)[1],
        )
        for handler_id in (press, release, focus_out):
            self._key_handlers.append((toplevel, handler_id))

    def set_active(self, active: bool) -> None:
        """Arm or disarm key handling; disarming always ends any hold."""
        if not active:
            self.release_all()
        self._active = active

    @property
    def active(self) -> bool:
        return self._active

    def _keys_allowed(self, widget: Gtk.Widget) -> bool:
        if not self._active:
            return False
        toplevel = widget.get_toplevel() if isinstance(widget, Gtk.Widget) else None
        window = toplevel if isinstance(toplevel, Gtk.Window) else None
        if focus_is_text_entry(window):
            return False
        # A modal dialog owns the keyboard while it is up.
        if window is not None and not window.is_active():
            return False
        return True

    def _on_key_press(self, widget: Gtk.Widget, event: Gdk.EventKey) -> bool:
        if not self._keys_allowed(widget):
            return False
        intent = _keyval_to_intent(event.keyval)
        if intent is None:
            return False
        kind, value = intent
        if kind == "axis":
            axis = int(value)
            enabled, _reason = C.gate(
                self._state, self._callbacks.select_axis, C.CAP_TELEOP_JOG,
            )
            if not enabled:
                return False
            toggle = self._axis_buttons.get(axis)
            if toggle is not None and toggle.get_sensitive():
                toggle.set_active(True)
            return True
        if kind == "jog":
            if self._key_jog_axis is not None:
                return True  # auto-repeat: one press, one jog
            enabled, _reason = C.gate(
                self._state, self._callbacks.jog_start, C.CAP_TELEOP_JOG,
            )
            if not enabled:
                return False
            self._start_jog(int(value))
            return True
        return self._dispatch_simple(kind, value)

    def _dispatch_simple(self, kind: str, value: object) -> bool:
        mapping: dict[str, tuple[C.Callback, str, Callable[[], object] | None]] = {
            "speed": (
                self._callbacks.change_speed, C.CAP_TELEOP_JOG,
                None if self._callbacks.change_speed is None
                else (lambda: self._callbacks.change_speed(int(value))),
            ),
            "stop": (
                self._callbacks.stop_motion, C.CAP_TELEOP_JOG,
                self._callbacks.stop_motion,
            ),
            "resume": (
                self._callbacks.resume_hold, C.CAP_TELEOP_JOG,
                self._callbacks.resume_hold,
            ),
            "home": (
                self._callbacks.go_home, C.CAP_TELEOP_POSE, self._callbacks.go_home,
            ),
        }
        entry = mapping.get(kind)
        if entry is None:
            return False
        callback, capability, action = entry
        enabled, _reason = C.gate(self._state, callback, capability)
        if not enabled or action is None:
            return False
        if kind == "stop":
            self.release_all()
        action()
        return True

    def _on_key_release(self, widget: Gtk.Widget, event: Gdk.EventKey) -> bool:
        intent = _keyval_to_intent(event.keyval)
        if intent is None or intent[0] != "jog":
            return False
        # Release always runs, even when the press would no longer be allowed:
        # losing permission mid-press must still end the jog.
        self._stop_jog()
        return True

    # --- refresh --------------------------------------------------------
    def update(self, state: C.ArmUiState) -> None:
        previous = self._state
        self._state = state
        # Any loss of permission or freshness ends an in-flight jog.
        if not C.is_live(state) or not C.has_manual_grant(state):
            self.release_all()
        elif state.detected_tool.generation != previous.detected_tool.generation:
            self.release_all()

        self.gate.apply(state)
        teleop = state.teleop
        self._suppress_axis_signal = True
        for index, toggle in self._axis_buttons.items():
            toggle.set_active(index == teleop.selected_axis)
        self._suppress_axis_signal = False

        if teleop.selected_axis is None:
            self._selected.set("")
            self._angle.set("")
        else:
            self._selected.set(f"{teleop.selected_axis}번 축", "status-live")
            current = next(
                (axis for axis in teleop.axes if axis.index == teleop.selected_axis),
                None,
            )
            self._angle.set(
                "" if current is None else format_number(current.angle_deg, "°", 2),
            )
        if teleop.speed_level is None and teleop.speed_ratio is None:
            self._speed.set("")
        else:
            parts = []
            if teleop.speed_level is not None:
                parts.append(f"{teleop.speed_level}단")
            if teleop.speed_ratio is not None:
                parts.append(f"{teleop.speed_ratio * 100:.0f}%")
            self._speed.set(" · ".join(parts), "status-live")
        self._moving.set(*(("동작 중", "status-warn") if teleop.jogging
                           else ("정지", "status-live") if C.is_live(state)
                           else ("", "status-muted")))
        if self._badge is not None:
            if not C.is_live(state):
                self._badge.set("상태 미수신", "status-muted")
            elif not C.has_manual_grant(state):
                self._badge.set("수동 제어권 미승인", "status-warn")
            else:
                self._badge.set("수동 조작 가능", "status-live")

        for child in tuple(self._angles.get_children()):
            self._angles.remove(child)
        if teleop.axes:
            tiles = []
            for axis in teleop.axes:
                tile = Metric(f"{axis.index}. {axis.name or '축'}")
                tile.set(
                    format_number(axis.angle_deg, "°", 2),
                    "status-warn" if axis.moving else "status-live",
                )
                tiles.append(tile)
            self._angles.pack_start(metric_grid(tiles, columns=3), False, False, 0)
        else:
            self._angles.pack_start(
                label("축 상태를 수신하지 못했습니다.", "arm-note"), False, False, 0,
            )
        self._angles.show_all()

        current_pose = self._pose_list.get_active_text()
        if tuple(self._pose_names()) != teleop.poses:
            self._pose_list.remove_all()
            for name in teleop.poses:
                self._pose_list.append_text(name)
            if current_pose in teleop.poses:
                self._pose_list.set_active(teleop.poses.index(current_pose))

    def _pose_names(self) -> list[str]:
        model = self._pose_list.get_model()
        return [] if model is None else [row[0] for row in model]

    def dispose(self) -> None:
        self.release_all()
        for window, handler_id in self._key_handlers:
            try:
                if window.handler_is_connected(handler_id):
                    window.disconnect(handler_id)
            except (AttributeError, TypeError):
                continue
        self._key_handlers.clear()
        self.tracker.dispose()
