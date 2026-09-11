"""Shared helpers for the arm-UI widget tests.

Not named ``test_*`` so pytest treats it as a plain module; the arm_ui test
directory is on ``sys.path`` during collection, so ``import _arm_ui_helpers``
works from either test module.
"""
from __future__ import annotations

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from operator_console.arm_ui import contracts as C


CALLBACK_NAMES = tuple(
    field for field in C.ArmUiCallbacks.__dataclass_fields__
)


def walk(widget):
    """Yield ``widget`` and every descendant."""
    yield widget
    if isinstance(widget, Gtk.Container):
        for child in widget.get_children():
            yield from walk(child)


def label_texts(widget) -> list[str]:
    return [item.get_text() for item in walk(widget) if isinstance(item, Gtk.Label)]


def all_text(widget) -> str:
    return " ".join(label_texts(widget))


def find_buttons(widget, text: str) -> list[Gtk.Button]:
    return [
        item for item in walk(widget)
        if isinstance(item, Gtk.Button) and item.get_label() == text
    ]


def recording_callbacks() -> tuple[C.ArmUiCallbacks, list[tuple]]:
    """Every callback wired to a recorder, so intent is observable in tests."""
    calls: list[tuple] = []

    def record(name: str):
        def handler(*args: object) -> None:
            calls.append((name, args))
        return handler

    callbacks = C.ArmUiCallbacks(
        **{name: record(name) for name in CALLBACK_NAMES}
    )
    return callbacks, calls
