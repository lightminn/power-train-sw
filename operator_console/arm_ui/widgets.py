"""Shared widget builders and the enable/disable gate used by both arm tabs.

Three things live here that the tabs would otherwise each reinvent:

* ``style`` / ``card`` / ``metric`` / ``badge`` -- the console's existing card,
  tile and pill shapes, so the arm tabs reuse the visual language instead of
  inventing one.
* :class:`GateGroup` -- one place that decides sensitivity.  A control is live
  only when its callback exists, its capability is reported, the feed is LIVE
  and the state permits it; otherwise it is insensitive *and* carries a tooltip
  naming the obstacle.
* :class:`HoldButton` -- press-and-hold jog with every release path wired, so a
  jog cannot outlive the press that started it.

:class:`SignalTracker` records every handler and timer id so a tab can undo all
of them in ``dispose()``.
"""
from __future__ import annotations

from typing import Callable, Iterable

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk, Pango  # noqa: E402

from . import contracts as C


def style(widget: Gtk.Widget, *classes: str) -> Gtk.Widget:
    """Add CSS classes.  Mirrors ``status_view._style`` so the idiom matches."""
    context = widget.get_style_context()
    for css_class in classes:
        context.add_class(css_class)
    return widget


def set_tone(widget: Gtk.Widget, tone: str) -> None:
    """Swap one of the console's four tone classes, removing the other three."""
    context = widget.get_style_context()
    for candidate in ("status-live", "status-warn", "status-bad", "status-muted"):
        context.remove_class(candidate)
    context.add_class(tone)


class SignalTracker:
    """Remembers connected handlers and GLib sources so they can all be undone.

    A tab that forgets one leaves a callback firing against a destroyed widget
    after the window closes, which is exactly the class of defect the console's
    execution smoke exists to catch.
    """

    def __init__(self) -> None:
        self._handlers: list[tuple[object, int]] = []
        self._sources: list[int] = []

    def connect(self, widget: object, signal: str, handler: Callable) -> int:
        handler_id = widget.connect(signal, handler)
        self._handlers.append((widget, handler_id))
        return handler_id

    def add_timeout(self, interval_ms: int, handler: Callable) -> int:
        source_id = GLib.timeout_add(interval_ms, handler)
        self._sources.append(source_id)
        return source_id

    def dispose(self) -> None:
        for widget, handler_id in self._handlers:
            try:
                if widget.handler_is_connected(handler_id):
                    widget.disconnect(handler_id)
            except (AttributeError, TypeError):
                # The widget is already finalised; nothing left to disconnect.
                continue
        self._handlers.clear()
        for source_id in self._sources:
            GLib.source_remove(source_id)
        self._sources.clear()

    @property
    def handler_count(self) -> int:
        return len(self._handlers)

    @property
    def source_count(self) -> int:
        return len(self._sources)


class GateGroup:
    """Applies :func:`contracts.gate` to a set of controls on every update."""

    def __init__(self) -> None:
        self._entries: list[
            tuple[Gtk.Widget, C.Callback, str, Callable[[C.ArmUiState], str] | None, str]
        ] = []

    def add(
        self,
        widget: Gtk.Widget,
        callback: C.Callback,
        capability: str,
        *,
        extra: Callable[[C.ArmUiState], str] | None = None,
        hint: str = "",
    ) -> Gtk.Widget:
        """Register one control.

        ``extra`` returns an additional blocking reason (empty string = fine)
        for conditions only the owning panel knows, such as "no pose selected".
        ``hint`` is the tooltip shown while the control *is* usable.
        """
        self._entries.append((widget, callback, capability, extra, hint))
        return widget

    def apply(self, state: C.ArmUiState) -> None:
        for widget, callback, capability, extra, hint in self._entries:
            enabled, reason = C.gate(state, callback, capability)
            if enabled and extra is not None:
                extra_reason = extra(state)
                if extra_reason:
                    enabled, reason = False, extra_reason
            widget.set_sensitive(enabled)
            widget.set_tooltip_text(hint if enabled else f"조작 불가 — {reason}")

    def reasons(self, state: C.ArmUiState) -> tuple[str, ...]:
        """Every distinct blocking reason currently in force, for tests."""
        found: list[str] = []
        for widget, callback, capability, extra, _hint in self._entries:
            enabled, reason = C.gate(state, callback, capability)
            if enabled and extra is not None:
                extra_reason = extra(state)
                if extra_reason:
                    enabled, reason = False, extra_reason
            if not enabled and reason not in found:
                found.append(reason)
        return tuple(found)

    @property
    def widgets(self) -> tuple[Gtk.Widget, ...]:
        return tuple(entry[0] for entry in self._entries)


class HoldButton(Gtk.Button):
    """A button that reports intent only while it is physically held.

    ``on_press`` fires once on press and ``on_release`` once on the matching
    release.  Release is also driven by the pointer leaving, focus moving away,
    the widget being unmapped (tab switch or window close) and by
    :meth:`force_release`, which the tab calls when authority or the link drops.
    A latched ``_held`` flag makes every one of those paths idempotent, so the
    release intent is reported exactly once per press.
    """

    def __init__(
        self,
        label: str,
        *,
        on_press: Callable[[], None] | None,
        on_release: Callable[[], None] | None,
        tracker: SignalTracker,
    ) -> None:
        super().__init__(label=label)
        self._on_press = on_press
        self._on_release = on_release
        self._held = False
        tracker.connect(self, "pressed", lambda _b: self._press())
        tracker.connect(self, "released", lambda _b: self.force_release())
        tracker.connect(self, "leave-notify-event", lambda _b, _e: self._leave())
        tracker.connect(self, "unmap", lambda _b: self.force_release())
        tracker.connect(self, "focus-out-event", lambda _b, _e: self._leave())

    @property
    def held(self) -> bool:
        return self._held

    def _press(self) -> None:
        if self._held or not self.get_sensitive():
            return
        self._held = True
        if self._on_press is not None:
            self._on_press()

    def _leave(self) -> bool:
        self.force_release()
        return False

    def force_release(self) -> None:
        """End the hold from any path.  Safe to call when nothing is held."""
        if not self._held:
            return
        self._held = False
        if self._on_release is not None:
            self._on_release()

    def set_sensitive(self, sensitive: bool) -> None:  # type: ignore[override]
        # Losing sensitivity mid-hold must end the jog; GTK will never send the
        # matching "released" once the button stops accepting input.
        if not sensitive:
            self.force_release()
        super().set_sensitive(sensitive)


def label(text: str, *classes: str, xalign: float = 0.0, wrap: bool = False) -> Gtk.Label:
    widget = Gtk.Label(label=text)
    widget.set_xalign(xalign)
    if wrap:
        widget.set_line_wrap(True)
        widget.set_line_wrap_mode(Pango.WrapMode.WORD_CHAR)
    style(widget, *classes)
    return widget


class Metric:
    """A label/value tile.  ``set`` writes the value and its tone together."""

    EMPTY = "정보 없음"

    def __init__(self, title: str, *, width_chars: int = 10) -> None:
        self.box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
        style(self.box, "arm-metric")
        self.box.pack_start(label(title, "arm-metric-label"), False, False, 0)
        self.value = label(self.EMPTY, "arm-metric-value", "status-muted")
        self.value.set_ellipsize(Pango.EllipsizeMode.END)
        # width_chars is a *minimum*, so a generous one keeps the whole page
        # from ever shrinking.  Cap the floor and let max_width_chars carry the
        # natural (wide-window) size instead.
        self.value.set_width_chars(min(width_chars, 8))
        self.value.set_max_width_chars(max(width_chars, 22))
        self.box.pack_start(self.value, False, False, 0)

    def set(self, text: str, tone: str = "status-muted") -> None:
        self.value.set_text(text or self.EMPTY)
        set_tone(self.value, tone if text else "status-muted")
        self.value.set_tooltip_text(text or self.EMPTY)


class Badge:
    """A pill badge in the console's four tones."""

    def __init__(self, text: str = "정보 없음", tone: str = "status-muted") -> None:
        self.label = label(text, "arm-badge", tone, xalign=0.5)

    def set(self, text: str, tone: str) -> None:
        self.label.set_text(text)
        set_tone(self.label, tone)


class Card:
    """A titled section card with an optional badge and description."""

    def __init__(
        self,
        title: str,
        description: str = "",
        accent: str = "",
        *,
        with_badge: bool = True,
    ) -> None:
        self.box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=9)
        self.box.set_hexpand(True)
        classes = ["arm-card"]
        if accent:
            classes.append(f"arm-card-{accent}")
        style(self.box, *classes)

        header = Gtk.Box(spacing=8)
        header.pack_start(label(title, "arm-card-title"), True, True, 0)
        self.badge: Badge | None = None
        if with_badge:
            self.badge = Badge()
            header.pack_end(self.badge.label, False, False, 0)
        self.box.pack_start(header, False, False, 0)
        if description:
            self.box.pack_start(
                label(description, "arm-card-description", wrap=True), False, False, 0,
            )

    def pack(self, widget: Gtk.Widget, expand: bool = False) -> None:
        self.box.pack_start(widget, expand, expand, 0)


def reflow(columns: int) -> Gtk.FlowBox:
    """A row container that drops to fewer columns instead of overflowing.

    A ``Gtk.Grid`` keeps its column count no matter how narrow the window
    gets, which pushes the whole page into horizontal scrolling.  A FlowBox
    with ``min_children_per_line = 1`` re-wraps instead, so the arm tabs stay
    readable down to phone-ish widths without a sideways scrollbar.
    """
    flow = Gtk.FlowBox()
    flow.set_selection_mode(Gtk.SelectionMode.NONE)
    flow.set_max_children_per_line(max(1, columns))
    flow.set_min_children_per_line(1)
    flow.set_homogeneous(True)
    flow.set_row_spacing(8)
    flow.set_column_spacing(8)
    flow.set_hexpand(True)
    return flow


def metric_grid(metrics: Iterable[Metric], columns: int = 2) -> Gtk.FlowBox:
    flow = reflow(columns)
    for item in metrics:
        flow.add(item.box)
    return flow


def reflow_row(widgets: Iterable[Gtk.Widget], columns: int = 6) -> Gtk.Box:
    """A compact left-aligned row of controls.

    Deliberately a ``Gtk.Box`` and not a FlowBox: a FlowBox gives every child
    in a line the same width, which spreads a handful of buttons across the
    card and breaks the console's compact button rhythm.  These rows are short
    enough (three to six small buttons) to fit the narrow layout as they are;
    the width that actually forces the page wide comes from the metric tiles
    and the side-by-side cards, and those do reflow -- see :func:`reflow`.

    ``columns`` is accepted for symmetry with :func:`reflow` and ignored.
    """
    row = Gtk.Box(spacing=6)
    for widget in widgets:
        widget.set_valign(Gtk.Align.CENTER)
        row.pack_start(widget, False, False, 0)
    return row


def button(text: str, *classes: str) -> Gtk.Button:
    widget = Gtk.Button(label=text)
    style(widget, *classes)
    return widget


def shortcut_row(pairs: Iterable[tuple[str, str]]) -> Gtk.FlowBox:
    """The key-binding cheat sheet.

    A FlowBox so the hints reflow instead of clipping when the window is
    narrow -- the arm tabs must stay readable at the console's smallest size.
    """
    flow = Gtk.FlowBox()
    flow.set_selection_mode(Gtk.SelectionMode.NONE)
    flow.set_max_children_per_line(6)
    flow.set_min_children_per_line(2)
    flow.set_row_spacing(4)
    flow.set_column_spacing(6)
    flow.set_homogeneous(False)
    for keys, meaning in pairs:
        row = Gtk.Box(spacing=5)
        row.pack_start(label(keys, "arm-shortcut"), False, False, 0)
        row.pack_start(label(meaning, "arm-note"), False, False, 0)
        flow.add(row)
    return flow


def scrolled(child: Gtk.Widget) -> Gtk.ScrolledWindow:
    """Wrap a page so a small window scrolls instead of truncating it."""
    window = Gtk.ScrolledWindow()
    window.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
    window.set_hexpand(True)
    window.set_vexpand(True)
    window.add(child)
    return window


def divider() -> Gtk.Box:
    line = Gtk.Box()
    style(line, "arm-divider")
    return line


def format_number(value: float | None, unit: str, digits: int = 1) -> str:
    """Format a reading, or say it is missing.  Never substitutes a zero."""
    if value is None:
        return ""
    return f"{value:.{digits}f}{unit}"


def format_ratio(value: float | None) -> str:
    if value is None:
        return ""
    return f"{value * 100.0:.0f}%"


def tri_state(value: bool | None, yes: str, no: str) -> tuple[str, str]:
    """Render an optional bool without collapsing ``None`` into ``False``."""
    if value is None:
        return "", "status-muted"
    return (yes, "status-live") if value else (no, "status-warn")
