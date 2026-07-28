"""RX-only robot status dashboard and lightweight Cairo visualizations."""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
import math
import time
from typing import Iterable

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk, Pango  # noqa: E402

from .arm_telemetry import ArmTelemetrySnapshot, temperature_state
from .metadata import (
    MetadataFrame,
    displayable_detections,
    pick_display_target,
    target_distance_m,
)
from .telemetry import TelemetrySnapshot, WheelStatus


STALE_AFTER_S = 1.0
GRAPH_WINDOW_S = 60.0
GRAPH_REFRESH_MS = 200
MAX_GRAPH_SAMPLES = 600


def power_card_state(
    power: object | None, *, fresh: bool,
) -> tuple[str, str]:
    """Judge power-device health separately from datagram freshness."""
    if power is None:
        return "정보 없음", "전원 장치 정보 수신 대기"
    if not fresh:
        return "확인 필요", "전원 정보 수신 지연"
    rs485 = str(getattr(power, "rs485_state", "") or "").strip().upper()
    if rs485 not in ("LIVE", "OK", "VALID", "NORMAL"):
        return "확인 필요", f"전원 링크 {rs485 or '상태 없음'}"
    if (
        getattr(power, "voltage_v", None) is None
        and getattr(power, "pdist_soc_percent", None) is None
    ):
        return "확인 필요", "전원 계측값 없음"
    if (
        getattr(power, "pdist_battery_flags", None) not in (None, 0)
        or getattr(power, "pdist_protection_flags", None) not in (None, 0)
    ):
        return "확인 필요", "보호 상태 확인 필요"
    return "정상", str(getattr(power, "rs485_state", "") or "정상")


def _style(widget: Gtk.Widget, *classes: str) -> Gtk.Widget:
    context = widget.get_style_context()
    for css_class in classes:
        context.add_class(css_class)
    return widget


@dataclass(frozen=True)
class TimedSample:
    timestamp_s: float
    value: float | None


class TimedSeries:
    """A bounded 60-second series; ``None`` explicitly represents a gap."""

    def __init__(
        self,
        *,
        window_s: float = GRAPH_WINDOW_S,
        max_samples: int = MAX_GRAPH_SAMPLES,
    ) -> None:
        self.window_s = float(window_s)
        self.max_samples = int(max_samples)
        self._samples: deque[TimedSample] = deque(maxlen=self.max_samples)

    def append(self, timestamp_s: float, value: float | None) -> None:
        timestamp_s = float(timestamp_s)
        if not math.isfinite(timestamp_s):
            raise ValueError("timestamp must be finite")
        if value is not None:
            value = float(value)
            if not math.isfinite(value):
                raise ValueError("value must be finite")
        self._samples.append(TimedSample(timestamp_s, value))
        cutoff = timestamp_s - self.window_s
        while self._samples and self._samples[0].timestamp_s < cutoff:
            self._samples.popleft()

    def mark_gap(self, timestamp_s: float) -> None:
        if not self._samples or self._samples[-1].value is not None:
            self.append(timestamp_s, None)

    def samples(self) -> tuple[TimedSample, ...]:
        return tuple(self._samples)

    def __len__(self) -> int:
        return len(self._samples)


class Sparkline(Gtk.DrawingArea):
    """Small multi-series graph drawn without a heavyweight plotting runtime."""

    COLORS = (
        (0.176, 0.431, 0.859),
        (0.094, 0.686, 0.788),
        (0.129, 0.541, 0.388),
        (0.831, 0.580, 0.125),
    )

    def __init__(self, series: Iterable[tuple[str, TimedSeries]], unit: str) -> None:
        super().__init__()
        self._series = tuple(series)
        self._unit = unit
        self._stale = False
        self.set_size_request(260, 118)
        self.connect("draw", self._draw)

    def set_stale(self, stale: bool) -> None:
        self._stale = bool(stale)
        self.queue_draw()

    def _draw(self, _widget: Gtk.DrawingArea, cr: object) -> bool:
        allocation = self.get_allocation()
        width, height = allocation.width, allocation.height
        left, right, top, bottom = 38.0, 10.0, 12.0, 24.0
        plot_w = max(1.0, width - left - right)
        plot_h = max(1.0, height - top - bottom)
        cr.set_source_rgb(0.965, 0.976, 0.988)
        cr.rectangle(0, 0, width, height)
        cr.fill()
        cr.set_line_width(1.0)
        cr.set_source_rgba(0.376, 0.447, 0.529, 0.15)
        for index in range(5):
            y = top + plot_h * index / 4.0
            cr.move_to(left, y)
            cr.line_to(left + plot_w, y)
            cr.stroke()
        all_samples = [
            sample
            for _name, series in self._series
            for sample in series.samples()
            if sample.value is not None
        ]
        cr.select_font_face("Noto Sans CJK KR", 0, 0)
        cr.set_font_size(9.0)
        cr.set_source_rgb(0.376, 0.447, 0.529)
        if not all_samples:
            cr.move_to(left + 8.0, top + plot_h / 2.0)
            cr.show_text("정보 수신 대기")
            return False
        now_s = max(sample.timestamp_s for sample in all_samples)
        values = [float(sample.value) for sample in all_samples if sample.value is not None]
        low, high = min(values), max(values)
        if low == high:
            pad = max(abs(low) * 0.1, 1.0)
        else:
            pad = (high - low) * 0.12
        low -= pad
        high += pad
        cr.move_to(3.0, top + 8.0)
        cr.show_text(f"{high:.1f}")
        cr.move_to(3.0, top + plot_h)
        cr.show_text(f"{low:.1f}")
        for seconds, label in ((60, "-60s"), (30, "-30s"), (0, "현재")):
            x = left + plot_w * (1.0 - seconds / GRAPH_WINDOW_S)
            cr.move_to(x - (0 if seconds == 60 else 10.0), height - 7.0)
            cr.show_text(label)
        for color_index, (name, series) in enumerate(self._series):
            color = self.COLORS[color_index % len(self.COLORS)]
            cr.set_source_rgba(*color, 0.40 if self._stale else 0.98)
            cr.set_line_width(1.8)
            drawing = False
            latest_value: float | None = None
            for sample in series.samples():
                if sample.value is None:
                    drawing = False
                    continue
                latest_value = sample.value
                x = left + plot_w * (
                    1.0 - min(GRAPH_WINDOW_S, now_s - sample.timestamp_s) / GRAPH_WINDOW_S
                )
                y = top + plot_h * (high - sample.value) / (high - low)
                if drawing:
                    cr.line_to(x, y)
                else:
                    cr.move_to(x, y)
                    drawing = True
            cr.stroke()
            for sample in series.samples():
                if sample.value is None:
                    continue
                x = left + plot_w * (
                    1.0 - min(GRAPH_WINDOW_S, now_s - sample.timestamp_s) / GRAPH_WINDOW_S
                )
                y = top + plot_h * (high - sample.value) / (high - low)
                cr.arc(x, y, 2.2, 0.0, math.tau)
                cr.fill()
            cr.set_source_rgba(*color, 1.0)
            cr.move_to(left + color_index * 78.0, 10.0)
            cr.show_text(name)
            if latest_value is not None:
                current = f"{latest_value:.2f} {self._unit}"
                extents = cr.text_extents(current)
                cr.move_to(width - extents.width - 10.0, 10.0 + color_index * 12.0)
                cr.show_text(current)
        if self._stale:
            cr.set_source_rgb(0.831, 0.580, 0.125)
            text = "갱신 지연"
            extents = cr.text_extents(text)
            cr.move_to(width - extents.width - 10.0, 10.0)
            cr.show_text(text)
        return False


class SteeringCanvas(Gtk.DrawingArea):
    """Compact top view whose wheel directions follow received steer angles."""

    POSITIONS = (
        ("FL", 0.28, 0.24),
        ("FR", 0.72, 0.24),
        ("RL", 0.28, 0.76),
        ("RR", 0.72, 0.76),
    )

    def __init__(self) -> None:
        super().__init__()
        self._angles: dict[str, float | None] = {}
        self.set_size_request(220, 150)
        self.connect("draw", self._draw)

    def update_wheels(self, wheels: tuple[WheelStatus, ...]) -> None:
        candidates = [wheel for wheel in wheels if wheel.steer_deg is not None]
        keyed: dict[str, float | None] = {}
        aliases = {
            "front_left": "FL", "fl": "FL", "front_right": "FR", "fr": "FR",
            "rear_left": "RL", "rl": "RL", "rear_right": "RR", "rr": "RR",
        }
        for wheel in candidates:
            compact = wheel.name.strip().lower()
            key = next(
                (alias for name, alias in aliases.items() if name in compact),
                None,
            )
            if key is not None:
                keyed[key] = wheel.steer_deg
        for (label, _x, _y), wheel in zip(self.POSITIONS, candidates):
            keyed.setdefault(label, wheel.steer_deg)
        self._angles = keyed
        self.queue_draw()

    def _draw(self, _widget: Gtk.DrawingArea, cr: object) -> bool:
        allocation = self.get_allocation()
        width, height = allocation.width, allocation.height
        cr.set_source_rgb(0.965, 0.976, 0.988)
        cr.rectangle(0, 0, width, height)
        cr.fill()
        cr.set_source_rgb(0.82, 0.86, 0.90)
        cr.rectangle(width * 0.35, height * 0.16, width * 0.30, height * 0.68)
        cr.fill()
        cr.select_font_face("Noto Sans CJK KR", 0, 1)
        cr.set_font_size(10.0)
        for label, px, py in self.POSITIONS:
            x, y = width * px, height * py
            angle = self._angles.get(label)
            cr.save()
            cr.translate(x, y)
            if angle is not None:
                cr.rotate(math.radians(angle))
                cr.set_source_rgb(0.176, 0.431, 0.859)
            else:
                cr.set_source_rgb(0.529, 0.588, 0.659)
            cr.set_line_width(5.0)
            cr.move_to(-17.0, 0)
            cr.line_to(17.0, 0)
            cr.stroke()
            cr.restore()
            cr.set_source_rgb(0.376, 0.447, 0.529)
            cr.move_to(x - 10.0, y + 20.0)
            cr.show_text(
                label if angle is None else f"{label} {angle:+.0f}°"
            )
        return False


class BarList(Gtk.DrawingArea):
    """Compact horizontal bars for joints or motor temperatures."""

    def __init__(self, *, temperature: bool = False) -> None:
        super().__init__()
        self._rows: tuple[tuple[str, float], ...] = ()
        self._temperature = temperature
        self.set_size_request(260, 130)
        self.connect("draw", self._draw)

    def set_rows(self, rows: Iterable[tuple[str, float]]) -> None:
        self._rows = tuple(rows)
        self.queue_draw()

    def _draw(self, _widget: Gtk.DrawingArea, cr: object) -> bool:
        allocation = self.get_allocation()
        width, height = allocation.width, allocation.height
        cr.set_source_rgb(0.965, 0.976, 0.988)
        cr.rectangle(0, 0, width, height)
        cr.fill()
        cr.select_font_face("Noto Sans CJK KR", 0, 0)
        cr.set_font_size(10.0)
        if not self._rows:
            cr.set_source_rgb(0.376, 0.447, 0.529)
            cr.move_to(12.0, height / 2.0)
            cr.show_text("정보 수신 대기")
            return False
        row_h = min(25.0, (height - 8.0) / len(self._rows))
        values = [abs(value) for _name, value in self._rows]
        scale = max(values + [1.0])
        for index, (name, value) in enumerate(self._rows):
            y = 6.0 + index * row_h
            cr.set_source_rgb(0.376, 0.447, 0.529)
            cr.move_to(7.0, y + 11.0)
            suffix = "℃" if self._temperature else "°"
            cr.show_text(f"{name}  {value:+.1f}{suffix}")
            bar_x, bar_y, bar_w = 116.0, y + 2.0, max(20.0, width - 124.0)
            cr.set_source_rgb(0.88, 0.91, 0.94)
            cr.rectangle(bar_x, bar_y, bar_w, 10.0)
            cr.fill()
            if self._temperature:
                state = temperature_state(int(round(value)))
                color = {
                    "NORMAL": (0.129, 0.541, 0.388),
                    "WARN": (0.831, 0.580, 0.125),
                    "CRIT": (0.769, 0.231, 0.263),
                }[state]
            else:
                color = (0.176, 0.431, 0.859)
            cr.set_source_rgb(*color)
            cr.rectangle(bar_x, bar_y, bar_w * min(1.0, abs(value) / scale), 10.0)
            cr.fill()
        return False


class StatusPanel(Gtk.Box):
    def __init__(self, title: str) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        _style(self, "status-panel")
        heading = Gtk.Label(label=title)
        heading.set_xalign(0.0)
        _style(heading, "status-panel-title")
        self.values = Gtk.Label(label="정보 수신 대기")
        self.values.set_xalign(0.0)
        self.values.set_line_wrap(True)
        _style(self.values, "status-panel-values")
        self.body = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
        self.updated = Gtk.Label(label="마지막 업데이트 · 정보 수신 대기")
        self.updated.set_xalign(0.0)
        _style(self.updated, "status-panel-updated")
        self.pack_start(heading, False, False, 0)
        self.pack_start(self.values, False, False, 0)
        self.pack_start(self.body, True, True, 0)
        self.pack_end(self.updated, False, False, 0)

    def set_data_available(self, available: bool, empty_text: str) -> None:
        self.body.set_visible(available)
        if not available and empty_text:
            self.values.set_text(empty_text)


class RobotStatusDashboard(Gtk.Box):
    """Status-tab composition. It has no command client or transmit callback."""

    PANEL_ORDER = ("drive", "power", "arm", "safety", "ai", "network")
    DEFAULT_VISIBLE = {
        "drive": True, "power": True, "arm": False, "safety": False,
        "ai": False, "network": False,
    }

    def __init__(
        self, *, input_source: str = "LIVE",
        power_port: int = 5004, chassis_port: int = 5005,
        arm_port: int = 5007, metadata_port: int = 5003,
    ) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=14)
        self._input_source = input_source
        self._source_ports = (
            power_port, chassis_port, arm_port, metadata_port,
        )
        self.set_border_width(14)
        self._last_sequences: dict[str, int] = {}
        self._last_updates: dict[str, float] = {}
        readiness = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        _style(readiness, "status-readiness")
        self._overall = Gtk.Label(label="운용 준비 전")
        self._overall.set_xalign(0.0)
        _style(self._overall, "status-overall")
        overall_title = Gtk.Label(label="운용 준비 상태")
        overall_title.set_xalign(0.0)
        _style(overall_title, "system-name")
        self._required_count = Gtk.Label(label="필수 시스템 0 / 4 준비")
        self._required_count.set_xalign(0.0)
        _style(self._required_count, "muted")
        self._priority = Gtk.Label(label="우선 확인 · 상태 정보 수신 대기")
        self._priority.set_xalign(0.0)
        self._priority.set_ellipsize(Pango.EllipsizeMode.END)
        _style(self._priority, "status-priority")
        readiness.pack_start(overall_title, False, False, 0)
        readiness.pack_start(self._overall, False, False, 0)
        readiness.pack_start(self._required_count, False, False, 0)
        readiness.pack_start(self._priority, False, False, 0)
        self.pack_start(readiness, False, False, 0)

        issues_title = Gtk.Label(label="우선 확인할 문제")
        issues_title.set_xalign(0.0)
        _style(issues_title, "section-title")
        self._issues = Gtk.Label(label="정보 수신 대기")
        self._issues.set_xalign(0.0)
        self._issues.set_line_wrap(True)
        _style(self._issues, "status-issues")
        self.pack_start(issues_title, False, False, 0)
        self.pack_start(self._issues, False, False, 0)
        self._cards: dict[str, tuple[Gtk.Label, Gtk.Label, Gtk.Label]] = {}
        card_grid = Gtk.Grid(column_spacing=10, row_spacing=10)
        card_grid.set_column_homogeneous(True)
        specs = (
            ("drive", "주행 시스템"), ("power", "전원 시스템"),
            ("safety", "안전 장치"), ("camera", "카메라·통신"),
            ("arm", "로봇팔·작업 장치"), ("ai", "AI 인식"),
        )
        for index, (key, title) in enumerate(specs):
            card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
            card.set_size_request(-1, 98)
            _style(card, "status-summary-card", "summary-offline")
            title_row = Gtk.Box(spacing=7)
            dot = Gtk.Label(label="")
            dot.set_size_request(8, 8)
            _style(dot, "status-dot", "status-muted")
            name = Gtk.Label(label=title)
            name.set_xalign(0.0)
            _style(name, "system-name")
            title_row.pack_start(dot, False, False, 0)
            title_row.pack_start(name, True, True, 0)
            state = Gtk.Label(label="정보 없음")
            state.set_xalign(0.0)
            _style(state, "status-summary-value")
            reason = Gtk.Label(label="정보 수신 대기")
            reason.set_xalign(0.0)
            reason.set_ellipsize(Pango.EllipsizeMode.END)
            _style(reason, "muted")
            card.pack_start(title_row, False, False, 0)
            card.pack_start(state, False, False, 0)
            card.pack_start(reason, False, False, 0)
            card_grid.attach(card, index % 3, index // 3, 1, 1)
            self._cards[key] = (state, reason, dot)
        self.pack_start(card_grid, False, False, 0)

        option_title = Gtk.Label(label="실시간 보기")
        option_title.set_xalign(0.0)
        _style(option_title, "section-title")
        self.pack_start(option_title, False, False, 0)
        options = Gtk.Box(spacing=16)
        self._toggles: dict[str, Gtk.ToggleButton] = {}
        labels = {
            "drive": "주행", "power": "전원", "arm": "로봇팔",
            "safety": "안전", "ai": "AI 인식", "network": "영상·통신",
        }
        for key in self.PANEL_ORDER:
            toggle = Gtk.ToggleButton(label=labels[key])
            toggle.set_active(self.DEFAULT_VISIBLE[key])
            toggle.connect("toggled", self._on_view_toggled, key)
            _style(toggle, "status-view-option")
            options.pack_start(toggle, False, False, 0)
            self._toggles[key] = toggle
        self.pack_start(options, False, False, 0)

        self.drive_speed = TimedSeries()
        self.power_voltage = TimedSeries()
        self.safety_distance = TimedSeries()
        self.ai_confidence = TimedSeries()
        self.ai_distance = TimedSeries()
        self.video_fps = TimedSeries()
        self._drive_graph = Sparkline((("Wheel", self.drive_speed),), "turn/s")
        self._power_graph = Sparkline((("Voltage", self.power_voltage),), "V")
        self._safety_graph = Sparkline((("Distance", self.safety_distance),), "mm")
        self._ai_graph = Sparkline(
            (("Confidence", self.ai_confidence), ("Distance", self.ai_distance)),
            "normalized / m",
        )
        self._video_graph = Sparkline((("FPS", self.video_fps),), "fps")
        self._steering = SteeringCanvas()
        self._joints = BarList()
        self._temperatures = BarList(temperature=True)
        self._panels = {
            key: StatusPanel(title)
            for key, title in (
                ("drive", "주행"), ("power", "전원"),
                ("arm", "로봇팔·작업 장치"), ("safety", "안전"),
                ("ai", "AI 인식"), ("network", "영상·통신"),
            )
        }
        self._panels["drive"].body.pack_start(self._drive_graph, False, False, 0)
        self._panels["drive"].body.pack_start(self._steering, False, False, 0)
        self._soc_bar = Gtk.ProgressBar()
        self._soc_bar.set_show_text(True)
        self._soc_bar.set_no_show_all(True)
        _style(self._soc_bar, "soc-progress")
        self._panels["power"].body.pack_start(self._soc_bar, False, False, 0)
        self._panels["power"].body.pack_start(self._power_graph, False, False, 0)
        self._panels["arm"].body.pack_start(self._joints, False, False, 0)
        self._panels["arm"].body.pack_start(self._temperatures, False, False, 0)
        self._panels["safety"].body.pack_start(self._safety_graph, False, False, 0)
        self._panels["ai"].body.pack_start(self._ai_graph, False, False, 0)
        self._panels["network"].body.pack_start(self._video_graph, False, False, 0)
        self._panel_flow = Gtk.FlowBox()
        self._panel_flow.set_selection_mode(Gtk.SelectionMode.NONE)
        self._panel_flow.set_column_spacing(12)
        self._panel_flow.set_row_spacing(12)
        self._panel_flow.set_min_children_per_line(1)
        self._panel_flow.set_max_children_per_line(2)
        self._panel_flow.set_homogeneous(True)
        for key in self.PANEL_ORDER:
            self._panel_flow.add(self._panels[key])
        self._no_panels = Gtk.Label(label="표시할 실시간 정보가 없습니다")
        self._no_panels.set_halign(Gtk.Align.CENTER)
        self._no_panels.set_margin_top(28)
        self._no_panels.set_margin_bottom(28)
        self._no_panels.set_no_show_all(True)
        _style(self._no_panels, "muted")
        self.pack_start(self._panel_flow, False, False, 0)
        self.pack_start(self._no_panels, False, False, 0)

        developer_row = Gtk.Box(spacing=8)
        developer_label = Gtk.Label(label="개발자 정보 보기")
        developer_label.set_xalign(0.0)
        self._developer_toggle = Gtk.Switch()
        self._developer_toggle.set_active(False)
        self._developer_toggle.connect("notify::active", self._toggle_developer)
        developer_row.pack_start(developer_label, True, True, 0)
        developer_row.pack_end(self._developer_toggle, False, False, 0)
        self._developer = Gtk.Label(label="")
        self._developer.set_xalign(0.0)
        self._developer.set_line_wrap(True)
        self._developer.set_selectable(True)
        self._developer.set_no_show_all(True)
        _style(self._developer, "developer-box")
        self.pack_start(developer_row, False, False, 0)
        self.pack_start(self._developer, False, False, 0)
        self._apply_panel_visibility()
        GLib.timeout_add(GRAPH_REFRESH_MS, self._redraw_graphs)

    @property
    def developer_visible(self) -> bool:
        return self._developer_toggle.get_active()

    def view_enabled(self, key: str) -> bool:
        return self._toggles[key].get_active()

    def _toggle_developer(self, *_args: object) -> None:
        self._developer.set_visible(self.developer_visible)

    def _on_view_toggled(self, _button: Gtk.ToggleButton, _key: str) -> None:
        self._apply_panel_visibility()

    def _apply_panel_visibility(self) -> None:
        visible_count = 0
        for key, panel in self._panels.items():
            visible = self.view_enabled(key)
            visible_count += int(visible)
            panel.set_no_show_all(not visible)
            child = panel.get_parent()
            if isinstance(child, Gtk.FlowBoxChild):
                child.set_no_show_all(not visible)
                if visible:
                    child.show_all()
                else:
                    child.hide()
            elif visible:
                panel.show_all()
            else:
                panel.hide()
        self._panel_flow.set_visible(visible_count > 0)
        self._no_panels.set_visible(visible_count == 0)

    def _redraw_graphs(self) -> bool:
        for graph in (
            self._drive_graph, self._power_graph, self._safety_graph,
            self._ai_graph, self._video_graph,
        ):
            graph.queue_draw()
        return True

    @staticmethod
    def _fresh(snapshot: object | None, now_s: float) -> bool:
        received = getattr(snapshot, "received_monotonic_s", None)
        return received is not None and now_s - received <= STALE_AFTER_S

    @staticmethod
    def _state(snapshot: object | None, now_s: float) -> str:
        if snapshot is None:
            return "정보 없음"
        return "정상" if RobotStatusDashboard._fresh(snapshot, now_s) else "확인 필요"

    def _set_card(self, key: str, state: str, reason: str) -> None:
        value, reason_label, dot = self._cards[key]
        value.set_text(state)
        reason_label.set_text(reason)
        tone = {
            "정상": "status-live", "연결 중": "status-progress",
            "확인 필요": "status-warn", "연결 끊김": "status-bad",
            "사용 안 함": "status-muted", "정보 없음": "status-muted",
        }.get(state, "status-muted")
        for candidate in (
            "status-live", "status-progress", "status-warn",
            "status-bad", "status-muted",
        ):
            dot.get_style_context().remove_class(candidate)
            value.get_style_context().remove_class(candidate)
        dot.get_style_context().add_class(tone)
        value.get_style_context().add_class(tone)

    def update(
        self,
        *,
        power: TelemetrySnapshot | None,
        chassis: TelemetrySnapshot | None,
        arm: ArmTelemetrySnapshot | None,
        metadata: MetadataFrame | None,
        front_video_state: str,
        work_video_state: str,
        front_fps: float | None,
        work_fps: float | None,
        front_frame_age_s: float | None = None,
        work_frame_age_s: float | None = None,
        now_s: float | None = None,
    ) -> None:
        now_s = time.monotonic() if now_s is None else float(now_s)
        power_fresh = self._fresh(power, now_s)
        chassis_fresh = self._fresh(chassis, now_s)
        arm_fresh = self._fresh(arm, now_s)
        metadata_fresh = (
            metadata is not None
            and now_s - metadata.received_monotonic_s <= 0.25
        )
        if power is not None and self._last_sequences.get("power") != power.sequence:
            self._last_sequences["power"] = power.sequence
            self._last_updates["power"] = power.received_monotonic_s
            self.power_voltage.append(power.received_monotonic_s, power.voltage_v)
        elif not power_fresh:
            self.power_voltage.mark_gap(now_s)
        if chassis is not None and self._last_sequences.get("chassis") != chassis.sequence:
            self._last_sequences["chassis"] = chassis.sequence
            self._last_updates["chassis"] = chassis.received_monotonic_s
            speeds = [
                abs(wheel.drive_turns_per_s)
                for wheel in chassis.wheel_statuses
                if wheel.drive_turns_per_s is not None
            ]
            self.drive_speed.append(
                chassis.received_monotonic_s,
                None if not speeds else sum(speeds) / len(speeds),
            )
            self.safety_distance.append(
                chassis.received_monotonic_s, chassis.safety_distance_mm,
            )
        elif not chassis_fresh:
            self.drive_speed.mark_gap(now_s)
            self.safety_distance.mark_gap(now_s)
        if metadata is not None and self._last_sequences.get("metadata") != metadata.sequence:
            self._last_sequences["metadata"] = metadata.sequence
            self._last_updates["metadata"] = metadata.received_monotonic_s
            target = pick_display_target(metadata)
            self.ai_confidence.append(
                metadata.received_monotonic_s,
                None if target is None else target.confidence,
            )
            self.ai_distance.append(
                metadata.received_monotonic_s,
                target_distance_m(target),
            )
        elif not metadata_fresh:
            self.ai_confidence.mark_gap(now_s)
            self.ai_distance.mark_gap(now_s)
        available_fps = [value for value in (front_fps, work_fps) if value is not None]
        if available_fps:
            self.video_fps.append(now_s, sum(available_fps) / len(available_fps))
        else:
            self.video_fps.mark_gap(now_s)

        drive_state = self._state(chassis, now_s)
        if chassis_fresh and chassis is not None:
            wheel_issue = any(
                wheel.stale or wheel.drive_axis_error or wheel.steer_fault
                for wheel in chassis.wheel_statuses
            )
            if chassis.safety_estop_required is True or wheel_issue:
                drive_state = "확인 필요"
            elif (
                chassis.drive_state.lower().startswith("unavailable")
                or chassis.can_state.lower().startswith("unavailable")
            ):
                drive_state = "정보 없음"
            drive_reason = (
                "구동 모터 상태 확인 필요" if wheel_issue
                else "CAN 상태 확인 필요"
                if chassis.can_state.lower().startswith("unavailable")
                else chassis.drive_state
            )
        else:
            drive_reason = "주행 정보 수신 대기"
        power_state, power_reason = power_card_state(power, fresh=power_fresh)
        safety_state = self._state(chassis, now_s)
        safety_reason = "안전 정보 수신 대기"
        if chassis_fresh and chassis is not None:
            if chassis.component_mask is not None and chassis.component_mask.get("us100") is False:
                safety_state, safety_reason = "사용 안 함", "US-100 비활성"
            elif chassis.safety_estop_required is True:
                safety_state, safety_reason = "확인 필요", chassis.safety_detail or "비상 정지"
            elif chassis.safety_estop_required is False:
                safety_reason = chassis.safety_status
            else:
                safety_state, safety_reason = "정보 없음", "안전 판정 정보 없음"
        camera_states = {front_video_state, work_video_state}
        if camera_states == {"LIVE"}:
            camera_state, camera_reason = "정상", "전방·작업 영상 수신 중"
        elif "LIVE" in camera_states:
            camera_state, camera_reason = "확인 필요", "일부 영상만 수신 중"
        elif "STALE" in camera_states:
            camera_state, camera_reason = "연결 끊김", "영상 갱신 지연"
        else:
            camera_state, camera_reason = "연결 중", "영상 연결 대기"
        arm_state = self._state(arm, now_s)
        arm_reason = (
            "로봇팔 정보 수신 대기"
            if arm is None else "관절·모터 상태 수신 중"
        )
        ai_state = "정상" if metadata_fresh else (
            "확인 필요" if metadata is not None else "정보 없음"
        )
        ai_reason = (
            "인식 결과 수신 중" if metadata_fresh
            else "AI 정보 갱신 지연" if metadata is not None
            else "AI 정보 수신 대기"
        )
        states = {
            "drive": (drive_state, drive_reason),
            "power": (power_state, power_reason),
            "safety": (safety_state, safety_reason),
            "camera": (camera_state, camera_reason),
            "arm": (arm_state, arm_reason),
            "ai": (ai_state, ai_reason),
        }
        for key, (state, reason) in states.items():
            self._set_card(key, state, reason)
        required_keys = ("drive", "power", "safety", "camera")
        required_ready = sum(states[key][0] == "정상" for key in required_keys)
        work_attention = any(states[key][0] != "정상" for key in ("arm", "ai"))
        self._required_count.set_text(f"필수 시스템 {required_ready} / 4 준비")
        problems = [
            (key, reason)
            for key, (state, reason) in states.items()
            if state != "정상"
        ]
        if required_ready == len(required_keys):
            self._overall.set_text(
                "운용 가능 · 작업 기능 확인 필요"
                if work_attention else "운용 가능"
            )
        else:
            self._overall.set_text("운용 준비 전")
        self._priority.set_text(
            "우선 확인 · 확인 필요한 항목이 없습니다"
            if not problems else f"우선 확인 · {problems[0][1]}"
        )

        if chassis_fresh and chassis is not None:
            speeds = [
                wheel.drive_turns_per_s
                for wheel in chassis.wheel_statuses
                if wheel.drive_turns_per_s is not None
            ]
            speed_text = (
                "바퀴 속도 정보 없음"
                if not speeds else f"평균 바퀴 속도 {sum(abs(v) for v in speeds) / len(speeds):.2f} turn/s"
            )
            self._panels["drive"].values.set_text(
                f"{speed_text}   ·   주행 상태 {chassis.drive_state}\n"
                f"Command / Actual 차체 속도(m/s) 정보 없음   ·   CAN {chassis.can_state}"
            )
            self._steering.update_wheels(chassis.wheel_statuses)
            self._panels["drive"].set_data_available(
                bool(chassis.wheel_statuses),
                "",
            )
        else:
            self._steering.update_wheels(())
            self._panels["drive"].set_data_available(
                False,
                "속도 정보 수신 대기\n주행 데이터 수신 후 속도와 조향 상태를 표시합니다",
            )
        if power_fresh and power is not None:
            voltage = "정보 없음" if power.voltage_v is None else f"{power.voltage_v:.1f} V"
            soc = (
                "정보 없음" if power.pdist_soc_percent is None
                else f"{power.pdist_soc_percent}%"
            )
            protection = (
                "확인 필요" if power.pdist_protection_flags not in (None, 0)
                else "정상" if power.pdist_protection_flags == 0
                else "정보 없음"
            )
            self._panels["power"].values.set_text(
                f"배터리 전압 {voltage}   ·   SOC {soc}   ·   보호 상태 {protection}\n"
                "전원 Rail 정보 없음"
            )
            self._panels["power"].set_data_available(
                power.voltage_v is not None,
                "",
            )
            if power.pdist_soc_percent is None:
                self._soc_bar.hide()
            else:
                soc_value = max(0.0, min(100.0, power.pdist_soc_percent))
                self._soc_bar.set_fraction(soc_value / 100.0)
                self._soc_bar.set_text(f"SOC {soc_value:.0f}%")
                self._soc_bar.show()
        else:
            self._soc_bar.hide()
            self._panels["power"].set_data_available(
                False,
                "전원 정보 수신 대기\n배터리 데이터 수신 후 전압 추이를 표시합니다",
            )
        if arm_fresh and arm is not None:
            self._joints.set_rows(
                (name, math.degrees(position))
                for name, position in zip(arm.joint_names, arm.joint_position_rad)
            )
            self._temperatures.set_rows(
                (f"ID {motor.id}", float(motor.temperature_c))
                for motor in (arm.dynamixel or ())
            )
            self._panels["arm"].values.set_text(
                "동작 모드 정보 없음   ·   엔드이펙터 정보 없음\n"
                f"관절 {len(arm.joint_names)}개   ·   다이나믹셀 "
                f"{'정보 없음' if arm.dynamixel is None else f'{len(arm.dynamixel)}개'}"
            )
            self._panels["arm"].set_data_available(
                bool(arm.joint_names or arm.dynamixel),
                "",
            )
        else:
            self._joints.set_rows(())
            self._temperatures.set_rows(())
            self._panels["arm"].set_data_available(
                False,
                "로봇팔 정보 수신 대기\n관절 데이터 수신 후 각도와 온도를 표시합니다",
            )
        if chassis_fresh and chassis is not None:
            distance = (
                "정보 없음" if chassis.safety_distance_mm is None
                else f"{chassis.safety_distance_mm:.0f} mm"
            )
            estop = (
                "발동됨" if chassis.safety_estop_required is True
                else "정상" if chassis.safety_estop_required is False
                else "정보 없음"
            )
            self._panels["safety"].values.set_text(
                f"긴급 정지 {estop}   ·   근접 안전 센서 {chassis.safety_status}\n"
                f"장애물 거리 {distance}   ·   MOTION HOLD {chassis.drive_state}"
            )
            self._panels["safety"].set_data_available(
                chassis.safety_distance_mm is not None,
                "",
            )
        else:
            self._panels["safety"].set_data_available(
                False,
                "안전 정보 수신 대기\n안전 상태 수신 후 핵심 상태를 표시합니다",
            )
        target = None
        if metadata_fresh and metadata is not None:
            target = pick_display_target(metadata)
        if target is None:
            self._panels["ai"].set_data_available(
                False,
                "대상 탐지 대기\n탐지 결과 수신 후 신뢰도와 거리를 표시합니다",
            )
        else:
            distance = (
                "정보 없음"
                if target_distance_m(target) is None
                else f"{target_distance_m(target):.2f} m"
            )
            yaw = (
                "정보 없음" if target.yaw_rad is None
                else f"{math.degrees(target.yaw_rad):+.0f}°"
            )
            self._panels["ai"].values.set_text(
                f"탐지 대상 {target.class_name}   ·   신뢰도 {target.confidence:.0%}\n"
                f"거리 {distance}   ·   방향 {yaw}   ·   "
                f"{'Pick Target' if target.is_pick_target else '일반 탐지'}"
            )
            self._panels["ai"].set_data_available(True, "")
        front_age = (
            "정보 없음" if front_frame_age_s is None
            else f"{front_frame_age_s:.1f}s"
        )
        work_age = (
            "정보 없음" if work_frame_age_s is None
            else f"{work_frame_age_s:.1f}s"
        )
        metadata_age = (
            "정보 없음" if metadata is None
            else f"{max(0.0, now_s - metadata.received_monotonic_s):.1f}s"
        )
        drop_rate = (
            None if chassis is None or chassis.l515_drop_hz is None
            else f"{chassis.l515_drop_hz:.1f} Hz"
        )
        self._panels["network"].values.set_text(
            f"전방 영상 {front_video_state}   ·   작업 영상 {work_video_state}\n"
            f"Video FPS {front_fps if front_fps is not None else '정보 없음'} / "
            f"{work_fps if work_fps is not None else '정보 없음'}   ·   "
            f"Last Frame {front_age} / {work_age}\n"
            f"Metadata Age {metadata_age}"
            + ("" if drop_rate is None else f"   ·   L515 Drop {drop_rate}")
        )
        self._panels["network"].set_data_available(
            bool(available_fps),
            "",
        )
        stale = {
            "drive": not chassis_fresh, "power": not power_fresh,
            "safety": not chassis_fresh, "ai": not metadata_fresh,
            "network": front_video_state != "LIVE" and work_video_state != "LIVE",
        }
        for key, graph in (
            ("drive", self._drive_graph), ("power", self._power_graph),
            ("safety", self._safety_graph), ("ai", self._ai_graph),
            ("network", self._video_graph),
        ):
            graph.set_stale(stale[key])
        for key, source in (
            ("drive", chassis), ("power", power), ("arm", arm),
            ("safety", chassis), ("ai", metadata),
        ):
            received = getattr(source, "received_monotonic_s", None)
            text = (
                "정보 수신 대기" if received is None
                else f"{max(0.0, now_s - received):.1f}초 전"
            )
            self._panels[key].updated.set_text(f"마지막 업데이트 · {text}")
        self._panels["network"].updated.set_text("마지막 업데이트 · 영상 수신 상태 기준")
        public_names = {
            "drive": "주행 시스템", "power": "전원 시스템",
            "safety": "안전 장치", "camera": "카메라·통신",
            "arm": "로봇팔·작업 장치", "ai": "AI 인식",
        }
        issues = [
            f"{public_names[key]} — {reason}" for key, reason in problems[:3]
        ]
        self._issues.set_text(
            "확인 필요한 항목이 없습니다"
            if not issues else "\n".join(f"• {item}" for item in issues)
        )
        def age_text(source: object | None) -> str:
            received = getattr(source, "received_monotonic_s", None)
            return "None" if received is None else f"{now_s - received:.3f}"

        self._developer.set_text(
            f"Telemetry (RX-only) · source={self._input_source}\n"
            f"power seq={getattr(power, 'sequence', None)} age={age_text(power)}\n"
            f"chassis seq={getattr(chassis, 'sequence', None)} age={age_text(chassis)}\n"
            f"arm seq={getattr(arm, 'sequence', None)} age={age_text(arm)}\n"
            f"metadata seq={getattr(metadata, 'sequence', None)} age={age_text(metadata)}\n"
            "sources=UDP :{}/:{}/:{}/:{} · control writes=none".format(
                *self._source_ports,
            )
        )
