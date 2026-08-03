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
    pick_display_target,
    target_distance_m,
)
from .telemetry import TelemetrySnapshot, WheelStatus


STALE_AFTER_S = 1.0
GRAPH_WINDOW_S = 60.0
GRAPH_REFRESH_MS = 200
MAX_GRAPH_SAMPLES = 600

PDIST_BATTERY_ALARMS = {
    2: "과전압 보호", 3: "저전압 보호",
    4: "충전 과온도", 5: "충전 저온",
    6: "방전 과온도", 7: "방전 저온",
}
PDIST_PROTECTION_ALARMS = {
    0: "충전 과전류", 1: "방전 과전류", 2: "단락 보호",
}
PDIST_BATTERY_STATES = {0: "수동 충전기", 1: "자동 충전기"}
PDIST_CONTROL_STATES = {
    5: "외부 제어", 6: "통신 제어", 7: "충전 중",
}


def _flag_names(value: int | None, mapping: dict[int, str]) -> tuple[str, ...]:
    if value is None:
        return ()
    return tuple(name for bit, name in mapping.items() if value & (1 << bit))


def pdist_alarm_names(
    battery_flags: int | None, protection_flags: int | None,
) -> tuple[str, ...]:
    return (
        *_flag_names(battery_flags, PDIST_BATTERY_ALARMS),
        *_flag_names(protection_flags, PDIST_PROTECTION_ALARMS),
    )


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
    if pdist_alarm_names(
        getattr(power, "pdist_battery_flags", None),
        getattr(power, "pdist_protection_flags", None),
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

    def __init__(
        self, *, temperature: bool = False, suffix: str = "°",
    ) -> None:
        super().__init__()
        self._rows: tuple[tuple[str, float], ...] = ()
        self._temperature = temperature
        self._suffix = suffix
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
            suffix = "℃" if self._temperature else self._suffix
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
        self.set_size_request(-1, 360)
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

    PANEL_ORDER = ("drive", "power", "arm", "safety", "network")
    CARD_TO_PANEL = {"camera": "network"}

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

        self._cards: dict[str, tuple[Gtk.Label, Gtk.Label, Gtk.Label]] = {}
        self._card_buttons: dict[str, Gtk.Button] = {}
        card_grid = Gtk.Grid(column_spacing=10, row_spacing=10)
        card_grid.set_column_homogeneous(True)
        specs = (
            ("drive", "주행 시스템"), ("power", "전원 시스템"),
            ("safety", "안전 장치"), ("camera", "카메라·통신"),
            ("arm", "로봇팔·작업 장치"),
        )
        for index, (key, title) in enumerate(specs):
            card = Gtk.Button()
            card.set_relief(Gtk.ReliefStyle.NONE)
            card.set_size_request(-1, 98)
            card.set_tooltip_text(f"{title} 상세 정보 보기")
            _style(card, "status-summary-card", "summary-offline")
            content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
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
            content.pack_start(title_row, False, False, 0)
            content.pack_start(state, False, False, 0)
            content.pack_start(reason, False, False, 0)
            card.add(content)
            panel_key = self.CARD_TO_PANEL.get(key, key)
            card.connect("clicked", self._on_card_clicked, panel_key)
            card_grid.attach(card, index % 3, index // 3, 1, 1)
            self._cards[key] = (state, reason, dot)
            self._card_buttons[panel_key] = card
        self.pack_start(card_grid, False, False, 0)

        self._selected_panel = "drive"
        self._detail_title = Gtk.Label(label="주행 시스템 상세 정보")
        self._detail_title.set_xalign(0.0)
        _style(self._detail_title, "section-title")
        self.pack_start(self._detail_title, False, False, 0)

        self.drive_speed = TimedSeries()
        self.power_voltage = TimedSeries()
        self.power_current = TimedSeries()
        self.safety_distance = TimedSeries()
        self.ai_confidence = TimedSeries()
        self.ai_distance = TimedSeries()
        self.video_fps = TimedSeries()
        self._drive_graph = Sparkline((("Wheel", self.drive_speed),), "turn/s")
        self._power_graph = Sparkline((("Voltage", self.power_voltage),), "V")
        self._power_current_graph = Sparkline(
            (("Current", self.power_current),), "A",
        )
        self._safety_graph = Sparkline((("Distance", self.safety_distance),), "mm")
        self._ai_graph = Sparkline(
            (("Confidence", self.ai_confidence), ("Distance", self.ai_distance)),
            "normalized / m",
        )
        self._video_graph = Sparkline((("FPS", self.video_fps),), "fps")
        self._steering = SteeringCanvas()
        self._joints = BarList()
        self._temperatures = BarList(temperature=True)
        self._arm_currents = BarList(suffix=" raw")
        self._panels = {
            key: StatusPanel(title)
            for key, title in (
                ("drive", "주행"), ("power", "전원"),
                ("arm", "로봇팔·작업 장치"), ("safety", "안전"),
                ("network", "카메라·통신·AI 상태"),
            )
        }
        self._detail_metrics: dict[str, dict[str, Gtk.Label]] = {}

        def add_metric_grid(
            panel_key: str, specs: tuple[tuple[str, str], ...],
        ) -> None:
            grid = Gtk.Grid(column_spacing=8, row_spacing=8)
            grid.set_column_homogeneous(True)
            metrics: dict[str, Gtk.Label] = {}
            for index, (key, title) in enumerate(specs):
                metric = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
                _style(metric, "detail-metric")
                heading = Gtk.Label(label=title)
                heading.set_xalign(0.0)
                _style(heading, "detail-metric-title")
                value = Gtk.Label(label="수신 대기")
                value.set_xalign(0.0)
                value.set_ellipsize(Pango.EllipsizeMode.END)
                _style(value, "detail-metric-value")
                metric.pack_start(heading, False, False, 0)
                metric.pack_start(value, False, False, 0)
                grid.attach(metric, index % 3, index // 3, 1, 1)
                metrics[key] = value
            self._panels[panel_key].body.pack_start(grid, False, False, 0)
            self._detail_metrics[panel_key] = metrics

        add_metric_grid("drive", (
            ("speed", "평균 바퀴 속도"), ("state", "주행 상태"),
            ("can", "CAN 통신"), ("odom", "오도메트리"),
            ("pose", "로봇 자세"), ("wheels", "4WS 상태"),
        ))
        add_metric_grid("safety", (
            ("estop", "긴급 정지"), ("sensor", "근접 센서"),
            ("distance", "장애물 거리"), ("failures", "연속 실패"),
        ))
        add_metric_grid("arm", (
            ("joints", "관절 수"), ("motors", "다이나믹셀"),
            ("mode", "조종 모드"), ("tool", "엔드이펙터"),
        ))
        add_metric_grid("network", (
            ("front", "전방 카메라"), ("work", "작업 카메라"),
            ("fps", "표시 FPS"), ("metadata", "AI 데이터"),
            ("drop", "영상 드랍"),
        ))
        self._panels["drive"].body.pack_start(self._drive_graph, False, False, 0)
        self._panels["drive"].body.pack_start(self._steering, False, False, 0)
        self._power_metrics: dict[str, Gtk.Label] = {}
        power_grid = Gtk.Grid(column_spacing=8, row_spacing=8)
        power_grid.set_column_homogeneous(True)
        for index, (key, title) in enumerate((
            ("voltage", "입력 전압"), ("current", "방전 전류"),
            ("soc", "배터리 잔량"), ("power", "사용 전력"),
            ("charge", "충전 전류"), ("link", "전원 통신"),
        )):
            metric = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
            _style(metric, "power-metric")
            heading = Gtk.Label(label=title)
            heading.set_xalign(0.0)
            _style(heading, "power-metric-title")
            value = Gtk.Label(label="수신 대기")
            value.set_xalign(0.0)
            value.set_ellipsize(Pango.EllipsizeMode.END)
            _style(value, "power-metric-value")
            metric.pack_start(heading, False, False, 0)
            metric.pack_start(value, False, False, 0)
            power_grid.attach(metric, index % 3, index // 3, 1, 1)
            self._power_metrics[key] = value
        self._panels["power"].body.pack_start(power_grid, False, False, 0)
        self._power_state = Gtk.Label(label="보호 상태 · 수신 대기\n운용 상태 · 수신 대기")
        self._power_state.set_xalign(0.0)
        self._power_state.set_line_wrap(True)
        _style(self._power_state, "power-state-summary")
        self._panels["power"].body.pack_start(
            self._power_state, False, False, 0,
        )
        self._soc_bar = Gtk.ProgressBar()
        self._soc_bar.set_show_text(True)
        self._soc_bar.set_no_show_all(True)
        _style(self._soc_bar, "soc-progress")
        self._panels["power"].body.pack_start(self._soc_bar, False, False, 0)
        power_graphs = Gtk.Box(spacing=8)
        power_graphs.set_homogeneous(True)
        power_graphs.pack_start(self._power_graph, True, True, 0)
        power_graphs.pack_start(self._power_current_graph, True, True, 0)
        self._panels["power"].body.pack_start(power_graphs, False, False, 0)
        self._panels["arm"].body.pack_start(self._joints, False, False, 0)
        self._panels["arm"].body.pack_start(self._arm_currents, False, False, 0)
        self._panels["arm"].body.pack_start(self._temperatures, False, False, 0)
        self._panels["safety"].body.pack_start(self._safety_graph, False, False, 0)
        self._panels["network"].body.pack_start(self._video_graph, False, False, 0)
        self._panel_flow = Gtk.FlowBox()
        self._panel_flow.set_selection_mode(Gtk.SelectionMode.NONE)
        self._panel_flow.set_column_spacing(12)
        self._panel_flow.set_row_spacing(12)
        self._panel_flow.set_min_children_per_line(1)
        self._panel_flow.set_max_children_per_line(1)
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
        return key == self._selected_panel

    def _toggle_developer(self, *_args: object) -> None:
        self._developer.set_visible(self.developer_visible)

    def _on_card_clicked(self, _button: Gtk.Button, key: str) -> None:
        self._selected_panel = key
        self._detail_title.set_text(
            f"{self._panels[key].get_children()[0].get_text()} 상세 정보"
        )
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
        for key, button in self._card_buttons.items():
            context = button.get_style_context()
            context.remove_class("selected")
            if key == self._selected_panel:
                context.add_class("selected")

    def _redraw_graphs(self) -> bool:
        for graph in (
            self._drive_graph, self._power_graph, self._power_current_graph,
            self._safety_graph,
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
            "연동 예정": "status-progress",
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
            self.power_current.append(power.received_monotonic_s, power.current_a)
        elif not power_fresh:
            self.power_voltage.mark_gap(now_s)
            self.power_current.mark_gap(now_s)
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
        }
        for key, (state, reason) in states.items():
            self._set_card(key, state, reason)
        required_keys = ("drive", "power", "safety", "camera")
        required_ready = sum(states[key][0] == "정상" for key in required_keys)
        work_attention = states["arm"][0] != "정상" or ai_state != "정상"
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
            drive_metrics = self._detail_metrics["drive"]
            drive_metrics["speed"].set_text(
                "수신 대기" if not speeds
                else f"{sum(abs(v) for v in speeds) / len(speeds):.2f} turn/s"
            )
            drive_metrics["state"].set_text(chassis.drive_state)
            drive_metrics["can"].set_text(chassis.can_state)
            drive_metrics["odom"].set_text(chassis.odometry_source)
            drive_metrics["pose"].set_text(
                "수신 대기" if chassis.yaw_rad is None
                else f"yaw {math.degrees(chassis.yaw_rad):+.1f}°"
            )
            drive_metrics["wheels"].set_text(
                f"{len(chassis.wheel_statuses)}개 · fault "
                f"{chassis.wheel_fault_count if chassis.wheel_fault_count is not None else '--'}"
            )
            self._panels["drive"].values.set_text(
                "주행·4WS 상태   ·   최근 60초 속도와 실시간 조향각"
            )
            self._steering.update_wheels(chassis.wheel_statuses)
            self._panels["drive"].set_data_available(
                bool(chassis.wheel_statuses),
                "",
            )
        else:
            for value in self._detail_metrics["drive"].values():
                value.set_text("수신 대기")
            self._steering.update_wheels(())
            self._panels["drive"].set_data_available(
                True, "",
            )
            self._panels["drive"].values.set_text(
                "주행·4WS 상태   ·   최근 60초 속도와 실시간 조향각"
            )
        if power_fresh and power is not None:
            voltage = "정보 없음" if power.voltage_v is None else f"{power.voltage_v:.1f} V"
            soc = (
                "정보 없음" if power.pdist_soc_percent is None
                else f"{power.pdist_soc_percent}%"
            )
            alarms = pdist_alarm_names(
                power.pdist_battery_flags, power.pdist_protection_flags,
            )
            protection = " · ".join(alarms) if alarms else (
                "정상" if power.pdist_battery_flags is not None
                and power.pdist_protection_flags is not None else "정보 없음"
            )
            charger_states = _flag_names(
                power.pdist_battery_flags, PDIST_BATTERY_STATES,
            )
            control_states = _flag_names(
                power.pdist_protection_flags, PDIST_CONTROL_STATES,
            )
            operating_states = (*charger_states, *control_states)
            self._panels["power"].values.set_text(
                "PDIST80B 전원 분배 시스템   ·   DC 24–48 V / 최대 80 A"
            )
            self._power_metrics["voltage"].set_text(voltage)
            self._power_metrics["current"].set_text(
                "수신 대기" if power.current_a is None else f"{power.current_a:.1f} A"
            )
            self._power_metrics["soc"].set_text(soc)
            self._power_metrics["power"].set_text(
                "수신 대기" if power.power_w is None else f"{power.power_w:.0f} W"
            )
            self._power_metrics["charge"].set_text(
                "수신 대기" if power.pdist_charge_current_a is None
                else f"{power.pdist_charge_current_a:.1f} A"
            )
            self._power_metrics["link"].set_text(power.rs485_state or "수신 대기")
            self._power_state.set_text(
                f"보호 상태  ·  {protection}\n"
                f"운용 상태  ·  {' · '.join(operating_states) if operating_states else '활성 상태 없음'}\n"
                f"진단  ·  BMS 0xEE   Battery "
                f"{'--' if power.pdist_battery_flags is None else f'0x{power.pdist_battery_flags:02X}'}   "
                f"Protection {'--' if power.pdist_protection_flags is None else f'0x{power.pdist_protection_flags:02X}'}   "
                f"통신 실패 {power.rs485_consecutive_failures if power.rs485_consecutive_failures is not None else '--'}회"
            )
            self._panels["power"].set_data_available(
                True,
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
            for value in self._power_metrics.values():
                value.set_text("수신 대기")
            self._power_state.set_text(
                "보호 상태  ·  수신 대기\n"
                "운용 상태  ·  수신 대기\n"
                "진단  ·  BMS Monitor PID 0xEE / RS485 57,600 bps"
            )
            self._panels["power"].values.set_text(
                "PDIST80B 전원 분배 시스템   ·   DC 24–48 V / 최대 80 A"
            )
            self._panels["power"].set_data_available(
                True, "",
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
            self._arm_currents.set_rows(
                (f"ID {motor.id}", float(motor.current))
                for motor in (arm.dynamixel or ())
            )
            arm_metrics = self._detail_metrics["arm"]
            arm_metrics["joints"].set_text(f"{len(arm.joint_names)}개")
            arm_metrics["motors"].set_text(
                "수신 대기" if arm.dynamixel is None else f"{len(arm.dynamixel)}개"
            )
            arm_metrics["mode"].set_text("연동 예정")
            arm_metrics["tool"].set_text("연동 예정")
            self._panels["arm"].values.set_text(
                "로봇팔 상태   ·   관절각·전류(raw)·온도 시각화"
            )
            self._panels["arm"].set_data_available(
                bool(arm.joint_names or arm.dynamixel),
                "",
            )
        else:
            for key, value in self._detail_metrics["arm"].items():
                value.set_text("연동 예정" if key in ("mode", "tool") else "수신 대기")
            self._joints.set_rows(())
            self._temperatures.set_rows(())
            self._arm_currents.set_rows(())
            self._panels["arm"].set_data_available(
                True, "",
            )
            self._panels["arm"].values.set_text(
                "로봇팔 상태   ·   관절각·전류(raw)·온도 시각화"
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
            safety_metrics = self._detail_metrics["safety"]
            safety_metrics["estop"].set_text(estop)
            safety_metrics["sensor"].set_text(chassis.safety_status)
            safety_metrics["distance"].set_text(distance)
            safety_metrics["failures"].set_text(
                "수신 대기" if chassis.safety_consecutive_failures is None
                else f"{chassis.safety_consecutive_failures}회"
            )
            self._panels["safety"].values.set_text(
                f"충돌 방지·E-STOP 상태   ·   {chassis.safety_detail or '상세 원인 없음'}"
            )
            self._panels["safety"].set_data_available(
                chassis.safety_distance_mm is not None,
                "",
            )
        else:
            for value in self._detail_metrics["safety"].values():
                value.set_text("수신 대기")
            self._panels["safety"].set_data_available(
                True, "",
            )
            self._panels["safety"].values.set_text(
                "충돌 방지·E-STOP 상태   ·   최근 60초 장애물 거리"
            )
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
        network_metrics = self._detail_metrics["network"]
        network_metrics["front"].set_text(front_video_state)
        network_metrics["work"].set_text(work_video_state)
        network_metrics["fps"].set_text(
            "수신 대기" if not available_fps
            else " / ".join(f"{value:.1f}" for value in available_fps)
        )
        network_metrics["metadata"].set_text(
            "정상" if metadata_fresh else "수신 대기" if metadata is None else "갱신 지연"
        )
        network_metrics["drop"].set_text(drop_rate or "수신 대기")
        self._panels["network"].values.set_text(
            "카메라·통신 상태   ·   최근 60초 표시 FPS"
            + f"\nFrame age {front_age} / {work_age}   ·   Metadata age {metadata_age}"
        )
        self._panels["network"].set_data_available(
            True, "",
        )
        stale = {
            "drive": not chassis_fresh, "power": not power_fresh,
            "safety": not chassis_fresh, "ai": not metadata_fresh,
            "network": front_video_state != "LIVE" and work_video_state != "LIVE",
        }
        for key, graph in (
            ("drive", self._drive_graph), ("power", self._power_graph),
            ("power", self._power_current_graph),
            ("safety", self._safety_graph), ("ai", self._ai_graph),
            ("network", self._video_graph),
        ):
            graph.set_stale(stale[key])
        for key, source in (
            ("drive", chassis), ("power", power), ("arm", arm),
            ("safety", chassis),
        ):
            received = getattr(source, "received_monotonic_s", None)
            text = (
                "정보 수신 대기" if received is None
                else f"{max(0.0, now_s - received):.1f}초 전"
            )
            self._panels[key].updated.set_text(f"마지막 업데이트 · {text}")
        self._panels["network"].updated.set_text("마지막 업데이트 · 영상 수신 상태 기준")
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
