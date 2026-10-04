"""Standalone preview for the two arm tabs.  Fixture data only.

Run it without touching ``app.py``:

    /usr/bin/python3 -m operator_console.arm_ui
    /usr/bin/python3 -m operator_console.arm_ui --scenario dual
    /usr/bin/python3 -m operator_console.arm_ui --seconds 5      # 자동 종료

Headless (no Xvfb needed — GTK's broadway backend serves the same widgets):

    broadwayd :5 &
    GDK_BACKEND=broadway BROADWAY_DISPLAY=:5 \\
        /usr/bin/python3 -m operator_console.arm_ui --seconds 5

Every state shown here comes from :mod:`fixtures` and is stamped
``is_fixture=True``, so both tabs render their fixture banner.  The intent log
on the right records each callback the tabs fire, which is what makes
"선택만으로는 명령이 나가지 않는다" visible: changing the tool dropdown logs
nothing until 변경 요청 is pressed.
"""
from __future__ import annotations

import argparse
import sys
import time

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk  # noqa: E402

from . import contracts as C
from . import fixtures
from .calibration_tab import TAB_NAME as CALIBRATION_NAME
from .calibration_tab import TAB_TITLE as CALIBRATION_TITLE
from .calibration_tab import ArmCalibrationTab
from .manual_tab import TAB_NAME as MANUAL_NAME
from .manual_tab import TAB_TITLE as MANUAL_TITLE
from .manual_tab import ArmManualTab
from .styling import PREVIEW_STYLE_CLASS, install_arm_ui_css
from .widgets import label, style


MAX_LOG_LINES = 200


class PreviewWindow(Gtk.Window):
    """A two-tab shell plus a fixture picker and an intent log."""

    def __init__(self, scenario: str = "single") -> None:
        super().__init__(title="로봇팔 UI 미리보기 (fixture)")
        install_arm_ui_css()
        style(self, PREVIEW_STYLE_CLASS)
        self.set_default_size(1280, 860)
        self.entries: list[str] = []

        callbacks = self._build_callbacks()
        self.manual = ArmManualTab(callbacks)
        self.calibration = ArmCalibrationTab(callbacks)

        layout = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        layout.pack_start(self._build_bar(scenario), False, False, 0)

        self.stack = Gtk.Stack()
        self.stack.set_transition_type(Gtk.StackTransitionType.NONE)
        self.stack.set_hhomogeneous(False)
        self.stack.set_vhomogeneous(False)
        self.stack.add_titled(self.manual.widget, MANUAL_NAME, MANUAL_TITLE)
        self.stack.add_titled(self.calibration.widget, CALIBRATION_NAME,
                              CALIBRATION_TITLE)
        switcher = Gtk.StackSwitcher()
        switcher.set_stack(self.stack)
        switcher.set_halign(Gtk.Align.START)
        nav = Gtk.Box(spacing=8)
        style(nav, "nav")
        nav.pack_start(switcher, False, False, 0)
        layout.pack_start(nav, False, False, 0)

        body = Gtk.Box(spacing=0)
        body.pack_start(self.stack, True, True, 0)
        body.pack_start(self._build_log(), False, False, 0)
        layout.pack_start(body, True, True, 0)
        self.add(layout)

        self.manual.attach_keys(self)
        self.connect("destroy", lambda _w: self.shutdown())
        self.apply_scenario(scenario)

    # --- construction ---------------------------------------------------
    def _build_bar(self, scenario: str) -> Gtk.Widget:
        bar = Gtk.Box(spacing=10)
        style(bar, "arm-preview-bar")
        bar.pack_start(
            label("로봇팔 UI 미리보기", "arm-preview-title"), False, False, 0)
        bar.pack_start(
            label(
                "fixture 데이터 전용 — 실제 로봇팔·ROS 연결 없음",
                "arm-preview-hint",
            ),
            False, False, 0,
        )
        self._picker = Gtk.ComboBoxText()
        for key, (title, _factory) in fixtures.SCENARIOS.items():
            self._picker.append(key, title)
        self._picker.set_active_id(
            scenario if scenario in fixtures.SCENARIOS else "single",
        )
        self._picker.connect("changed", self._on_scenario_changed)
        bar.pack_end(self._picker, False, False, 0)
        bar.pack_end(label("상태 시나리오", "arm-preview-hint"), False, False, 0)
        return bar

    def _build_log(self) -> Gtk.Widget:
        box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        style(box, "arm-ui")
        box.set_size_request(300, -1)
        box.set_border_width(12)
        box.pack_start(label("콜백 기록 (의도만 전달)", "arm-card-title"), False, False, 0)
        box.pack_start(
            label(
                "버튼이 호출한 콜백만 기록됩니다. 드롭다운 선택은 "
                "여기에 남지 않습니다 — 선택은 명령이 아닙니다.",
                "arm-note", wrap=True,
            ),
            False, False, 0,
        )
        self._log_label = label("", "arm-mono", wrap=True)
        scroller = Gtk.ScrolledWindow()
        scroller.set_policy(Gtk.PolicyType.AUTOMATIC, Gtk.PolicyType.AUTOMATIC)
        scroller.set_vexpand(True)
        scroller.add(self._log_label)
        box.pack_start(scroller, True, True, 0)
        return box

    def _build_callbacks(self) -> C.ArmUiCallbacks:
        def record(name: str):
            def handler(*args: object) -> None:
                detail = ", ".join(repr(value) for value in args)
                self.log(f"{name}({detail})")
            return handler

        return C.ArmUiCallbacks(
            request_tool_change=record("request_tool_change"),
            request_control_mode=record("request_control_mode"),
            release_control_mode=record("release_control_mode"),
            select_axis=record("select_axis"),
            jog_start=record("jog_start"),
            jog_stop=record("jog_stop"),
            change_speed=record("change_speed"),
            stop_motion=record("stop_motion"),
            resume_hold=record("resume_hold"),
            go_home=record("go_home"),
            save_pose=record("save_pose"),
            move_to_pose=record("move_to_pose"),
            delete_pose=record("delete_pose"),
            tool_command=record("tool_command"),
            tool_jog_start=record("tool_jog_start"),
            tool_jog_stop=record("tool_jog_stop"),
            arm_calibration_command=record("arm_calibration_command"),
            tool_calibration_command=record("tool_calibration_command"),
            tool_calibration_jog_start=record("tool_calibration_jog_start"),
            tool_calibration_jog_stop=record("tool_calibration_jog_stop"),
        )

    # --- behaviour ------------------------------------------------------
    def log(self, text: str) -> None:
        stamped = f"{time.strftime('%H:%M:%S')}  {text}"
        self.entries.append(stamped)
        del self.entries[:-MAX_LOG_LINES]
        self._log_label.set_text("\n".join(reversed(self.entries)))
        print(f"[intent] {stamped}", flush=True)

    def _on_scenario_changed(self, combo: Gtk.ComboBoxText) -> None:
        key = combo.get_active_id()
        if key:
            self.apply_scenario(key)

    def apply_scenario(self, key: str) -> None:
        _title, factory = fixtures.SCENARIOS.get(
            key, fixtures.SCENARIOS["disconnected"],
        )
        state = factory()
        self.manual.update_state(state)
        self.calibration.update_state(state)

    def shutdown(self) -> None:
        """Tear both tabs down; safe to call more than once."""
        self.manual.dispose()
        self.calibration.dispose()


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="로봇팔 UI 골격 미리보기 (fixture 전용)",
    )
    parser.add_argument(
        "--scenario", default="single", choices=sorted(fixtures.SCENARIOS),
        help="시작 시 표시할 fixture 시나리오",
    )
    parser.add_argument(
        "--seconds", type=float, default=0.0,
        help="지정 시간 뒤 자동 종료 (0 = 수동 종료)",
    )
    parser.add_argument(
        "--cycle", action="store_true",
        help="모든 fixture 시나리오를 1초 간격으로 순회한다",
    )
    args = parser.parse_args(argv)

    window = PreviewWindow(args.scenario)
    window.connect("destroy", Gtk.main_quit)
    window.show_all()

    if args.cycle:
        order = list(fixtures.SCENARIOS)
        index = {"value": 0}

        def advance() -> bool:
            index["value"] = (index["value"] + 1) % len(order)
            window._picker.set_active_id(order[index["value"]])
            return True

        GLib.timeout_add_seconds(1, advance)

    if args.seconds > 0:
        def quit_now() -> bool:
            window.shutdown()
            Gtk.main_quit()
            return False

        GLib.timeout_add(int(args.seconds * 1000), quit_now)

    Gtk.main()
    return 0


if __name__ == "__main__":  # pragma: no cover - manual entry point
    sys.exit(main())
