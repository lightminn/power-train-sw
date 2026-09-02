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
from .environment_telemetry import (
    EnvironmentTelemetrySnapshot,
    environment_source_state,
)
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

END_EFFECTOR_PURPOSES = {
    # The handoff confirms four physical categories.  Product names for the
    # second gripper and cleaner are not yet authoritative, so the console
    # deliberately uses neutral inventory labels instead of inventing them.
    "그리퍼 1": "파지 작업용 장착 도구 · 세부 형식은 장착 정보로 확인",
    "그리퍼 2": "보조 파지 작업용 장착 도구 · 세부 형식은 장착 정보로 확인",
    "청소 모듈": "먼지·오염물·표면 정리 작업",
    "환경 센서 모듈": "온습도·기압·공기질·가스·불꽃 관측",
}

PDIST_BATTERY_ALARMS = {
    2: "과전압 보호", 3: "저전압 보호",
    4: "충전 과온 보호", 5: "충전 저온 보호",
    6: "방전 과온 보호", 7: "방전 저온 보호",
}
PDIST_PROTECTION_ALARMS = {
    0: "충전 과전류", 1: "방전 과전류", 2: "단락 보호",
}
PDIST_BATTERY_STATES = {0: "수동 충전기 연결", 1: "자동 충전기 연결"}
PDIST_CONTROL_STATES = {
    5: "외부 제어", 6: "충전 중",
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
        return "정보 없음", "전원 장치 정보 없음"
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
        self.set_size_request(260, 360)
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
        cr.set_source_rgb(0.035, 0.075, 0.129)
        cr.rectangle(0, 0, width, height)
        cr.fill()
        cr.set_line_width(1.0)
        cr.set_source_rgba(0.27, 0.39, 0.53, 0.24)
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
        cr.set_source_rgb(0.47, 0.56, 0.66)
        if not all_samples:
            cr.move_to(left + 8.0, top + plot_h / 2.0)
            cr.show_text("정보 없음")
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
        self.set_size_request(220, 390)
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
        cr.set_source_rgb(0.035, 0.075, 0.129)
        cr.rectangle(0, 0, width, height)
        cr.fill()

        # Reference composition: four wheel indicators around a compact
        # top-view chassis, leaving the large status stage intentionally calm.
        box_w = min(300.0, width * 0.25)
        box_h = min(260.0, height * 0.62)
        box_x = (width - box_w) / 2.0
        box_y = (height - box_h) / 2.0
        cr.set_source_rgba(0.16, 0.34, 0.61, 0.24)
        cr.rectangle(box_x, box_y, box_w, box_h)
        cr.fill()
        cr.set_source_rgb(0.29, 0.54, 0.92)
        cr.set_line_width(2.0)
        cr.rectangle(box_x, box_y, box_w, box_h)
        cr.stroke()

        axle_x1, axle_x2 = box_x + box_w * 0.36, box_x + box_w * 0.64
        axle_y1, axle_y2 = box_y + box_h * 0.34, box_y + box_h * 0.66
        cr.move_to(axle_x1, axle_y1)
        cr.line_to(axle_x2, axle_y1)
        cr.move_to(axle_x1, axle_y2)
        cr.line_to(axle_x2, axle_y2)
        cr.move_to((axle_x1 + axle_x2) / 2.0, axle_y1)
        cr.line_to((axle_x1 + axle_x2) / 2.0, axle_y2)
        cr.stroke()
        for cx, cy in (
            (axle_x1, axle_y1), (axle_x2, axle_y1),
            (axle_x1, axle_y2), (axle_x2, axle_y2),
        ):
            cr.arc(cx, cy, 9.0, 0.0, math.tau)
            cr.stroke()

        cr.select_font_face("Noto Sans CJK KR", 0, 1)
        cr.set_font_size(12.0)
        wheel_layout = {
            "FL": (width * 0.17, height * 0.27),
            "FR": (width * 0.72, height * 0.27),
            "RL": (width * 0.17, height * 0.72),
            "RR": (width * 0.72, height * 0.72),
        }
        track_w = min(250.0, width * 0.20)
        for label, _px, _py in self.POSITIONS:
            x, y = wheel_layout[label]
            angle = self._angles.get(label)
            cr.set_line_width(8.0)
            cr.set_line_cap(1)
            cr.set_source_rgb(0.09, 0.15, 0.23)
            cr.move_to(x, y)
            cr.line_to(x + track_w, y)
            cr.stroke()
            fraction = 0.0 if angle is None else min(1.0, abs(angle) / 45.0)
            if angle is not None:
                cr.set_source_rgb(0.29, 0.54, 0.92)
                cr.move_to(x, y)
                cr.line_to(x + max(12.0, track_w * fraction), y)
                cr.stroke()
            cr.set_source_rgb(0.47, 0.56, 0.66)
            text = label if angle is None else f"{label}   {angle:+.1f}°"
            extents = cr.text_extents(text)
            cr.move_to(x + (track_w - extents.width) / 2.0, y + 34.0)
            cr.show_text(
                text
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
        self.set_size_request(260, 360)
        self.connect("draw", self._draw)

    def set_rows(self, rows: Iterable[tuple[str, float]]) -> None:
        self._rows = tuple(rows)
        self.queue_draw()

    def _draw(self, _widget: Gtk.DrawingArea, cr: object) -> bool:
        allocation = self.get_allocation()
        width, height = allocation.width, allocation.height
        cr.set_source_rgb(0.035, 0.075, 0.129)
        cr.rectangle(0, 0, width, height)
        cr.fill()
        cr.select_font_face("Noto Sans CJK KR", 0, 0)
        cr.set_font_size(10.0)
        if not self._rows:
            cr.set_source_rgb(0.47, 0.56, 0.66)
            cr.move_to(12.0, height / 2.0)
            cr.show_text("정보 없음")
            return False
        values = [abs(value) for _name, value in self._rows]
        scale = max(values + [1.0])
        count = len(self._rows)
        slot_w = width / max(1, count)
        bar_w = min(72.0, slot_w * .42)
        top, bottom = 30.0, height - 46.0
        bar_h = max(20.0, bottom - top)
        for index, (name, value) in enumerate(self._rows):
            x = slot_w * (index + .5) - bar_w / 2
            cr.set_source_rgb(0.09, 0.15, 0.23)
            cr.rectangle(x, top, bar_w, bar_h)
            cr.fill()
            if self._temperature:
                state = temperature_state(int(round(value)))
                color = {
                    "NORMAL": (0.129, 0.541, 0.388),
                    "WARN": (0.831, 0.580, 0.125),
                    "CRIT": (0.769, 0.231, 0.263),
                }[state]
            else:
                color = (0.29, 0.54, 0.92)
            fill_h = bar_h * min(1.0, abs(value) / scale)
            cr.set_source_rgb(*color)
            cr.rectangle(x, bottom - fill_h, bar_w, fill_h)
            cr.fill()
            cr.set_source_rgb(0.47, 0.56, 0.66)
            label = name
            extents = cr.text_extents(label)
            cr.move_to(x + (bar_w - extents.width) / 2, height - 18)
            cr.show_text(label)
        return False


class ReferenceStatusCanvas(Gtk.DrawingArea):
    """Large subsystem visual using the approved competition composition."""

    def __init__(self, kind: str) -> None:
        super().__init__()
        self.kind = kind
        self.set_size_request(300, 390)
        self.connect("draw", self._draw)

    def _draw(self, _widget: Gtk.DrawingArea, cr: object) -> bool:
        width = self.get_allocation().width
        height = self.get_allocation().height
        cr.set_source_rgb(0.035, 0.075, 0.129)
        cr.paint()
        cr.set_line_width(3.0)
        if self.kind == "ai":
            cr.set_source_rgba(0.34, 0.73, 0.87, 0.14)
            cr.rectangle(width * .31, height * .22, width * .38, height * .56)
            cr.fill()
            cr.set_source_rgb(0.34, 0.73, 0.87)
            x1, x2, y1, y2 = width * .31, width * .69, height * .22, height * .78
            for sx, sy, dx, dy in ((x1,y1,1,1),(x2,y1,-1,1),(x1,y2,1,-1),(x2,y2,-1,-1)):
                cr.move_to(sx, sy + dy * 54)
                cr.line_to(sx, sy)
                cr.line_to(sx + dx * 54, sy)
                cr.stroke()
            cx, cy = width / 2, height / 2
            for radius in (50, 92):
                cr.arc(cx, cy, radius, 0, math.tau)
                cr.stroke()
            cr.arc(cx, cy, 9, 0, math.tau)
            cr.fill()
            caption = "WAITING"
        elif self.kind == "safety":
            cr.set_source_rgb(0.10, 0.62, 0.48)
            cx, cy = width / 2, height * .78
            for radius in (92, 170, 245):
                cr.arc(cx, cy, radius, math.pi, math.tau)
                cr.stroke()
            cr.set_source_rgba(0.10, 0.62, 0.48, .25)
            cr.rectangle(cx - 42, cy - 20, 84, 50)
            cr.fill()
            caption = "거리 정보 대기"
        else:
            for index, (name, y) in enumerate(zip(
                ("FRONT", "ARM", "CTRL"), (height * .28, height * .50, height * .72),
            )):
                end = width * (.57 if index < 2 else .84)
                cr.set_source_rgb(0.34, 0.73, 0.87)
                cr.move_to(width * .24, y)
                cr.line_to(end, y)
                cr.stroke()
                cr.arc(end, y, 8, 0, math.tau)
                cr.fill()
                cr.select_font_face("monospace", 0, 0)
                cr.set_font_size(14)
                cr.set_source_rgb(0.47, 0.56, 0.66)
                cr.move_to(width * .13, y + 5)
                cr.show_text(name)
            return False
        cr.select_font_face("monospace", 0, 0)
        cr.set_font_size(14)
        cr.set_source_rgb(0.47, 0.56, 0.66)
        extents = cr.text_extents(caption)
        cr.move_to((width - extents.width) / 2, height * .91)
        cr.show_text(caption)
        return False


class StandbyLinkCanvas(Gtk.DrawingArea):
    """Layered RX topology used while every robot source is silent."""

    ENDPOINTS = (
        ("5005", 0.20, (0.29, 0.55, 0.92)),
        ("5004", 0.40, (0.89, 0.64, 0.23)),
        ("5007", 0.60, (0.85, 0.42, 0.57)),
        ("5003", 0.80, (0.49, 0.39, 0.91)),
    )

    def __init__(self) -> None:
        super().__init__()
        self.set_size_request(430, 250)
        self.connect("draw", self._draw)

    def _draw(self, _widget: Gtk.DrawingArea, cr: object) -> bool:
        width = float(self.get_allocation().width)
        height = float(self.get_allocation().height)
        hub_x, hub_y = width * 0.25, height * 0.50
        endpoint_x = width * 0.79

        # A faint technical grid gives the empty state depth without implying
        # that any robot measurement has been received.
        cr.set_line_width(1.0)
        cr.set_source_rgba(0.31, 0.49, 0.68, 0.10)
        for index in range(1, 8):
            x = width * index / 8.0
            cr.move_to(x, height * 0.08)
            cr.line_to(x, height * 0.92)
        for index in range(1, 5):
            y = height * index / 5.0
            cr.move_to(width * 0.05, y)
            cr.line_to(width * 0.95, y)
        cr.stroke()

        cr.set_source_rgba(0.30, 0.59, 0.92, 0.11)
        for radius in (34.0, 54.0, 76.0):
            cr.arc(hub_x, hub_y, radius, 0.0, math.tau)
            cr.stroke()
        cr.set_source_rgba(0.36, 0.68, 0.98, 0.18)
        cr.arc(hub_x, hub_y, 27.0, 0.0, math.tau)
        cr.fill()
        cr.set_source_rgb(0.43, 0.75, 0.98)
        cr.arc(hub_x, hub_y, 7.0, 0.0, math.tau)
        cr.fill()

        cr.select_font_face("JetBrains Mono", 0, 1)
        cr.set_font_size(11.0)
        cr.set_source_rgb(0.72, 0.84, 0.94)
        cr.move_to(hub_x - 20.0, hub_y + 4.0)
        cr.show_text("RX")

        for port, fraction_y, color in self.ENDPOINTS:
            y = height * fraction_y
            cr.set_source_rgba(*color, 0.42)
            cr.move_to(hub_x + 36.0, hub_y)
            cr.curve_to(width * 0.47, hub_y, width * 0.55, y, endpoint_x, y)
            cr.stroke()
            cr.set_source_rgba(*color, 0.16)
            cr.arc(endpoint_x, y, 14.0, 0.0, math.tau)
            cr.fill()
            cr.set_source_rgb(*color)
            cr.arc(endpoint_x, y, 4.5, 0.0, math.tau)
            cr.fill()
            cr.set_source_rgb(0.56, 0.68, 0.79)
            cr.move_to(endpoint_x + 22.0, y + 4.0)
            cr.show_text(port)
        return False


class SensorTrend(Gtk.DrawingArea):
    """Threshold-free 60-second trend: direction, not safety classification."""

    def __init__(self, color: tuple[float, float, float]) -> None:
        super().__init__()
        self._series = TimedSeries()
        self._color = color
        self._stale = False
        self.set_size_request(-1, 68)
        self.set_hexpand(True)
        self.connect("draw", self._draw)

    def append(self, timestamp_s: float, value: float | None) -> None:
        self._series.append(timestamp_s, value)
        self.queue_draw()

    def mark_stale(self, timestamp_s: float) -> None:
        self._stale = True
        self._series.mark_gap(timestamp_s)
        self.queue_draw()

    def set_live(self) -> None:
        self._stale = False
        self.queue_draw()

    def _draw(self, _widget: Gtk.DrawingArea, cr: object) -> bool:
        width = float(self.get_allocation().width)
        height = float(self.get_allocation().height)
        left, right, top, bottom = 5.0, 5.0, 15.0, 6.0
        plot_w = max(1.0, width - left - right)
        plot_h = max(1.0, height - top - bottom)
        cr.set_source_rgba(0.12, 0.19, 0.27, 0.72)
        cr.rectangle(0.0, 8.0, width, height - 8.0)
        cr.fill()
        cr.set_source_rgba(0.34, 0.47, 0.59, 0.22)
        cr.set_line_width(1.0)
        cr.move_to(left, top + plot_h / 2.0)
        cr.line_to(left + plot_w, top + plot_h / 2.0)
        cr.stroke()

        samples = tuple(
            sample for sample in self._series.samples()
            if sample.value is not None
        )
        cr.select_font_face("Noto Sans CJK KR", 0, 0)
        cr.set_font_size(8.0)
        cr.set_source_rgb(0.43, 0.53, 0.63)
        cr.move_to(left, 8.0)
        cr.show_text("최근 60초 · 자동 스케일")
        if len(samples) < 2:
            cr.set_source_rgb(0.39, 0.48, 0.58)
            cr.move_to(left + 4.0, top + plot_h * 0.67)
            cr.show_text("추세 수집 중")
            return False

        now_s = samples[-1].timestamp_s
        values = [float(sample.value) for sample in samples]
        low, high = min(values), max(values)
        if math.isclose(low, high):
            pad = max(abs(low) * 0.002, 0.1)
        else:
            pad = (high - low) * 0.16
        low -= pad
        high += pad
        cr.set_source_rgba(*self._color, 0.43 if self._stale else 0.96)
        cr.set_line_width(2.0)
        drawing = False
        last_x = last_y = 0.0
        for sample in self._series.samples():
            if sample.value is None:
                drawing = False
                continue
            age = min(GRAPH_WINDOW_S, max(0.0, now_s - sample.timestamp_s))
            x = left + plot_w * (1.0 - age / GRAPH_WINDOW_S)
            y = top + plot_h * (high - sample.value) / (high - low)
            if drawing:
                cr.line_to(x, y)
            else:
                cr.move_to(x, y)
                drawing = True
            last_x, last_y = x, y
        cr.stroke()
        if drawing:
            cr.arc(last_x, last_y, 3.0, 0.0, math.tau)
            cr.fill()
        return False


class EnvironmentSensorDashboard(Gtk.Box):
    """Dedicated visual dashboard for Raspberry Pi environmental readings."""

    def __init__(self, *, port: int = 5008) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        self.set_border_width(14)
        self._port = int(port)
        self._probe_climate = "수신 대기"
        self._probe_air = "수신 대기"
        self._probe_hazard = "수신 대기"
        self._last_sequence: int | None = None

        heading = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=12)
        title_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        title = Gtk.Label(label="환경 측정 현황")
        title.set_xalign(0.0)
        _style(title, "environment-title")
        subtitle = Gtk.Label(
            label="현재 환경과 위험 신호를 한 화면에서 확인합니다 · 관측 전용"
        )
        subtitle.set_xalign(0.0)
        _style(subtitle, "environment-subtitle")
        self._connection = Gtk.Label(label="●  연결 대기")
        self._connection.set_xalign(1.0)
        _style(self._connection, "environment-connection", "status-muted")
        title_box.pack_start(title, False, False, 0)
        title_box.pack_start(subtitle, False, False, 0)
        heading.pack_start(title_box, True, True, 0)
        heading.pack_end(self._connection, False, False, 0)
        self.pack_start(heading, False, False, 0)

        self._summary = Gtk.Label(
            label="센서 패킷을 기다리고 있습니다"
        )
        self._summary.set_xalign(0.0)
        self._summary.set_line_wrap(True)
        _style(self._summary, "environment-summary")
        self.pack_start(self._summary, False, False, 0)

        self._overall_status: dict[str, Gtk.Label] = {}
        status_row = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
        for key, text, css in (
            ("live", "정상", "status-live"),
            ("warn", "주의", "status-warn"),
            ("muted", "오류", "status-muted"),
        ):
            block = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=8)
            block.set_size_request(120, 34)
            _style(block, "sensor-overall-block", css)
            count = Gtk.Label(label="0")
            count.set_xalign(0.5)
            _style(count, "sensor-overall-count", css)
            caption = Gtk.Label(label=text)
            caption.set_xalign(0.5)
            _style(caption, "sensor-overall-caption", css)
            block.pack_start(caption, True, True, 0)
            block.pack_end(count, False, False, 0)
            status_row.pack_start(block, True, True, 0)
            self._overall_status[key] = count
        self.pack_start(status_row, False, False, 0)

        self._values: dict[str, Gtk.Label] = {}
        self._statuses: dict[str, Gtk.Label] = {}
        self._notes: dict[str, Gtk.Label] = {}
        self._trends: dict[str, SensorTrend] = {}

        self._tiles: dict[str, Gtk.Box] = {}

        def make_tile(
            key: str,
            label_text: str,
            group_key: str,
            note_text: str,
            color: tuple[float, float, float],
            *,
            show_trend: bool = False,
        ) -> Gtk.Box:
            card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=5)
            card.set_border_width(11)
            card.set_hexpand(True)
            _style(
                card, "sensor-tile", "sensor-compact-tile",
                f"sensor-tile-{group_key}",
            )
            header = Gtk.Box(spacing=8)
            name = Gtk.Label(label=label_text)
            name.set_xalign(0.0)
            _style(name, "sensor-tile-name")
            status = Gtk.Label(label="수신 대기")
            status.set_xalign(1.0)
            _style(status, "sensor-status", "status-muted")
            value = Gtk.Label(label="—")
            value.set_xalign(0.0)
            value.set_selectable(True)
            value.set_ellipsize(Pango.EllipsizeMode.END)
            _style(value, "sensor-tile-value")
            trend = SensorTrend(color)
            note = Gtk.Label(label=note_text)
            note.set_xalign(0.0)
            note.set_ellipsize(Pango.EllipsizeMode.END)
            _style(note, "sensor-tile-note")
            header.pack_start(name, True, True, 0)
            header.pack_end(status, False, False, 0)
            card.pack_start(header, False, False, 0)
            card.pack_start(value, False, False, 0)
            if show_trend:
                card.pack_start(trend, True, True, 0)
            card.pack_end(note, False, False, 0)
            self._values[key] = value
            self._statuses[key] = status
            self._notes[key] = note
            self._trends[key] = trend
            self._tiles[key] = card
            return card

        def make_group(
            title_text: str, description: str, group_key: str,
            tile_specs: tuple[tuple[object, ...], ...],
        ) -> Gtk.Box:
            group = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
            group.set_hexpand(True)
            group.set_vexpand(True)
            _style(group, "sensor-group", f"sensor-group-panel-{group_key}")
            group_title = Gtk.Label(label=title_text)
            group_title.set_xalign(0.0)
            _style(group_title, "sensor-group-title")
            group_description = Gtk.Label(label=description)
            group_description.set_xalign(0.0)
            group_description.set_line_wrap(True)
            _style(group_description, "sensor-group-description")
            group.pack_start(group_title, False, False, 0)
            group.pack_start(group_description, False, False, 0)
            for spec in tile_specs:
                key, label_text, note_text, color, show_trend = spec
                group.pack_start(
                    make_tile(
                        str(key), str(label_text), group_key, str(note_text),
                        color, show_trend=bool(show_trend),
                    ),
                    bool(show_trend), bool(show_trend), 0,
                )
            return group

        groups = Gtk.Grid(column_spacing=12, row_spacing=0)
        groups.set_column_homogeneous(True)
        groups.set_hexpand(True)
        groups.set_vexpand(True)
        groups.attach(make_group(
            "핵심 환경", "현재 공간의 기본 환경",
            "climate", (
                ("temperature", "온도", "최근 60초 추세", (0.31, 0.61, 0.94), True),
                ("humidity", "습도", "상대습도", (0.31, 0.61, 0.94), False),
                ("pressure", "기압", "대기압", (0.31, 0.61, 0.94), False),
            ),
        ), 0, 0, 1, 1)
        groups.attach(make_group(
            "공기질", "SGP30 기반 변화 관측",
            "air", (
                ("eco2", "eCO₂", "실제 CO₂가 아닌 계산 추정값", (0.58, 0.48, 0.94), True),
                ("tvoc", "TVOC", "휘발성 유기화합물", (0.58, 0.48, 0.94), False),
            ),
        ), 1, 0, 1, 1)
        groups.attach(make_group(
            "위험 감지", "불꽃은 판정값, 가스는 교정 전 참고값",
            "hazard", (
                ("co", "CO", "MQ-7 · 기준가스 교정 전", (0.91, 0.64, 0.20), False),
                ("lpg", "LPG", "MQ-2 · 기준가스 교정 전", (0.91, 0.64, 0.20), False),
                ("flame", "불꽃", "최근 60초 센서 전압", (0.20, 0.77, 0.55), True),
            ),
        ), 2, 0, 1, 1)
        self.pack_start(groups, True, True, 0)

        diagnostics = Gtk.Expander(label="교정·수신 진단")
        diagnostics.set_expanded(False)
        diagnostic_body = Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL, spacing=5,
        )
        diagnostic_body.set_border_width(10)
        self._link = Gtk.Label(label=f"UDP :{self._port} · 수신 대기")
        self._link.set_xalign(0.0)
        _style(self._link, "developer-box")
        self._source_detail = Gtk.Label(
            label=("CO/LPG는 기준가스 교정 전 추정값이며 절대 안전 판정에 "
                   "사용하지 않습니다. eCO₂는 SGP30 계산 추정값입니다.")
        )
        self._source_detail.set_xalign(0.0)
        self._source_detail.set_line_wrap(True)
        _style(self._source_detail, "sensor-diagnostic-note")
        diagnostic_body.pack_start(self._source_detail, False, False, 0)
        diagnostic_body.pack_start(self._link, False, False, 0)
        diagnostics.add(diagnostic_body)
        self.pack_end(diagnostics, False, False, 0)

    def _set_sensor_status(self, key: str, text: str, css_class: str) -> None:
        label = self._statuses[key]
        for name in ("status-live", "status-warn", "status-bad", "status-muted"):
            label.get_style_context().remove_class(name)
        label.get_style_context().add_class(css_class)
        label.set_text(f"● {text}")
        tile = self._tiles[key]
        for name in (
            "sensor-state-live", "sensor-state-warn",
            "sensor-state-bad", "sensor-state-muted",
        ):
            tile.get_style_context().remove_class(name)
        tile.get_style_context().add_class(
            {
                "status-live": "sensor-state-live",
                "status-warn": "sensor-state-warn",
                "status-bad": "sensor-state-bad",
                "status-muted": "sensor-state-muted",
            }[css_class]
        )

    def _update_overall_status(self, counts: dict[str, int]) -> None:
        for key, count in counts.items():
            if key in self._overall_status:
                self._overall_status[key].set_text(str(count))

    @staticmethod
    def _format(value: float | None, unit: str, digits: int) -> str:
        return "—" if value is None else f"{value:.{digits}f} {unit}"

    def update(
        self,
        snapshot: EnvironmentTelemetrySnapshot | None,
        *,
        now_s: float | None = None,
    ) -> None:
        current = time.monotonic() if now_s is None else float(now_s)
        state = environment_source_state(snapshot, now_s=current)
        if snapshot is None:
            self._link.set_text(f"UDP :{self._port} · 수신 대기")
            self._connection.set_text("●  연결 대기")
            connection_context = self._connection.get_style_context()
            connection_context.remove_class("status-live")
            connection_context.remove_class("status-warn")
            connection_context.add_class("status-muted")
            self._summary.set_text("센서 패킷을 기다리고 있습니다")
            for value in self._values.values():
                value.set_text("—")
            for key in self._statuses:
                self._set_sensor_status(key, "오류", "status-muted")
            self._update_overall_status({"live": 0, "warn": 0, "muted": len(self._statuses)})
            flame_context = self._values["flame"].get_style_context()
            for css_class in ("status-live", "status-bad"):
                flame_context.remove_class(css_class)
            flame_context.add_class("status-muted")
            flame_tile_context = self._tiles["flame"].get_style_context()
            for css_class in (
                "sensor-flame-normal", "sensor-flame-detected",
            ):
                flame_tile_context.remove_class(css_class)
            for trend in self._trends.values():
                trend.mark_stale(current)
            self._probe_climate = self._probe_air = self._probe_hazard = "수신 대기"
            return

        age = max(0.0, current - snapshot.received_monotonic_s)
        self._link.set_text(
            f"UDP :{self._port} · {state} · {age:.1f}초 전 · seq {snapshot.sequence}"
        )
        connection_context = self._connection.get_style_context()
        for css_class in ("status-live", "status-warn", "status-muted"):
            connection_context.remove_class(css_class)
        if state == "LIVE":
            self._connection.set_text("●  실시간 수신")
            connection_context.add_class("status-live")
        else:
            self._connection.set_text("●  갱신 지연")
            connection_context.add_class("status-warn")
        if self._last_sequence != snapshot.sequence:
            self._last_sequence = snapshot.sequence
            sample_time = snapshot.received_monotonic_s
            for key, value in (
                ("temperature", snapshot.temperature_c),
                ("humidity", snapshot.humidity_pct),
                ("pressure", snapshot.pressure_hpa),
                ("eco2", snapshot.eco2_ppm),
                ("tvoc", snapshot.tvoc_ppb),
                ("co", snapshot.co_estimated_ppm),
                ("lpg", snapshot.lpg_estimated_ppm),
                ("flame", snapshot.flame_voltage_v),
            ):
                self._trends[key].append(sample_time, value)
        for trend in self._trends.values():
            if state == "LIVE":
                trend.set_live()
            else:
                trend.mark_stale(current)
        self._values["temperature"].set_text(
            self._format(snapshot.temperature_c, "°C", 2)
        )
        self._values["humidity"].set_text(
            self._format(snapshot.humidity_pct, "%", 2)
        )
        self._values["pressure"].set_text(
            self._format(snapshot.pressure_hpa, "hPa", 2)
        )
        self._values["eco2"].set_text(
            self._format(snapshot.eco2_ppm, "ppm", 0)
        )
        self._values["tvoc"].set_text(
            self._format(snapshot.tvoc_ppb, "ppb", 0)
        )
        self._values["co"].set_text(
            "≈" + self._format(snapshot.co_estimated_ppm, "ppm", 1)
            if snapshot.co_estimated_ppm is not None else "—"
        )
        self._values["lpg"].set_text(
            "≈" + self._format(snapshot.lpg_estimated_ppm, "ppm", 1)
            if snapshot.lpg_estimated_ppm is not None else "—"
        )
        flame_text = (
            "정보 없음" if snapshot.flame_detected is None
            else "감지" if snapshot.flame_detected else "감지 없음"
        )
        voltage = self._format(snapshot.flame_voltage_v, "V", 3)
        self._values["flame"].set_text(f"{flame_text} · {voltage}")
        flame_context = self._values["flame"].get_style_context()
        for css_class in ("status-live", "status-bad", "status-muted"):
            flame_context.remove_class(css_class)
        flame_context.add_class(
            "status-muted" if snapshot.flame_detected is None
            else "status-bad" if snapshot.flame_detected
            else "status-live"
        )
        flame_tile_context = self._tiles["flame"].get_style_context()
        for css_class in (
            "sensor-flame-normal", "sensor-flame-detected",
        ):
            flame_tile_context.remove_class(css_class)
        if snapshot.flame_detected is not None:
            flame_tile_context.add_class(
                "sensor-flame-detected"
                if snapshot.flame_detected else "sensor-flame-normal"
            )
        threshold = self._format(snapshot.flame_threshold_v, "V", 3)
        self._notes["flame"].set_text(
            f"{threshold} 미만이면 불꽃 감지"
        )

        # Status badges are deliberately conservative: gas ppm values are
        # marked attention until the MQ sensors are calibrated against a
        # reference gas, while flame is a binary hazard decision.
        for key, value in (
            ("temperature", snapshot.temperature_c),
            ("humidity", snapshot.humidity_pct),
            ("pressure", snapshot.pressure_hpa),
            ("eco2", snapshot.eco2_ppm),
            ("tvoc", snapshot.tvoc_ppb),
        ):
            self._set_sensor_status(
                key,
                "정상" if state == "LIVE" and value is not None else "오류",
                "status-live" if state == "LIVE" and value is not None else "status-muted",
            )
        self._set_sensor_status(
            "co", "주의" if state == "LIVE" and snapshot.co_estimated_ppm is not None else "오류",
            "status-warn" if state == "LIVE" and snapshot.co_estimated_ppm is not None else "status-muted",
        )
        self._set_sensor_status(
            "lpg", "주의" if state == "LIVE" and snapshot.lpg_estimated_ppm is not None else "오류",
            "status-warn" if state == "LIVE" and snapshot.lpg_estimated_ppm is not None else "status-muted",
        )
        self._set_sensor_status(
            "flame",
            "주의" if snapshot.flame_detected else "정상"
            if state == "LIVE" and snapshot.flame_detected is not None else "오류",
            "status-warn" if snapshot.flame_detected else "status-live"
            if state == "LIVE" and snapshot.flame_detected is not None else "status-muted",
        )
        status_counts = {"live": 0, "warn": 0, "muted": 0}
        for label in self._statuses.values():
            text = label.get_text()
            if "정상" in text:
                status_counts["live"] += 1
            elif "주의" in text:
                status_counts["warn"] += 1
            else:
                status_counts["muted"] += 1
        self._update_overall_status(status_counts)
        self._notes["eco2"].set_text(
            "SGP30 예열 중" if snapshot.sgp30_warming_up
            else "SGP30 계산 추정값"
        )
        error_text = "" if not snapshot.errors else " · 오류: " + " | ".join(snapshot.errors)
        self._summary.set_text(
            ("종합 상태 · 최신값 수신 중" if state == "LIVE"
             else "종합 상태 · 마지막 값 표시 · 갱신 지연")
            + f"  |  불꽃 {flame_text}  |  CO/LPG 교정 전{error_text}"
        )
        self._source_detail.set_text(
            "CO/LPG는 기준가스 교정 전 추정값이며 절대 안전 판정에 "
            "사용하지 않습니다. eCO₂는 SGP30 계산 추정값입니다.\n"
            f"source={snapshot.source} · seq={snapshot.sequence}"
            + (f" · Pi 시각={snapshot.source_timestamp}"
               if snapshot.source_timestamp else "")
        )
        self._probe_climate = (
            f"{snapshot.temperature_c} °C · {snapshot.humidity_pct} % · "
            f"{snapshot.pressure_hpa} hPa"
        )
        self._probe_air = f"eCO₂ {snapshot.eco2_ppm} ppm · TVOC {snapshot.tvoc_ppb} ppb"
        flame = "감지" if snapshot.flame_detected else "정상"
        self._probe_hazard = (
            f"CO≈{snapshot.co_estimated_ppm} ppm · "
            f"LPG≈{snapshot.lpg_estimated_ppm} ppm · 불꽃 {flame}"
        )

    def probe_values(self) -> tuple[str, str, str]:
        return self._probe_climate, self._probe_air, self._probe_hazard


class StatusPanel(Gtk.Box):
    def __init__(self, title: str) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        self.set_size_request(-1, 625)
        _style(self, "status-panel")
        heading = Gtk.Label(label=title)
        heading.set_xalign(0.0)
        _style(heading, "status-panel-title")
        heading.set_no_show_all(True)
        heading.hide()
        self.values = Gtk.Label(label="정보 없음")
        self.values.set_xalign(0.0)
        self.values.set_line_wrap(True)
        _style(self.values, "status-panel-values")
        self.values.set_no_show_all(True)
        self.values.hide()
        self.body = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
        self.updated = Gtk.Label(label="마지막 업데이트 · 정보 없음")
        self.updated.set_xalign(0.0)
        _style(self.updated, "status-panel-updated")
        self.updated.set_no_show_all(True)
        self.updated.hide()
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

    PANEL_ORDER = ("drive", "power", "safety", "network", "ai", "arm")
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
        self._priority = Gtk.Label(label="우선 확인 · 상태 정보 없음")
        self._priority.set_xalign(0.0)
        self._priority.set_ellipsize(Pango.EllipsizeMode.END)
        _style(self._priority, "status-priority")
        readiness.pack_start(overall_title, False, False, 0)
        readiness.pack_start(self._overall, False, False, 0)
        readiness.pack_start(self._required_count, False, False, 0)
        readiness.pack_start(self._priority, False, False, 0)
        # Keep the aggregate readiness calculation for internal safety logic,
        # but omit its large duplicate banner from the judge-facing status tab.

        self._cards: dict[str, tuple[Gtk.Label, Gtk.Label, Gtk.Label]] = {}
        self._card_buttons: dict[str, Gtk.Button] = {}
        card_grid = Gtk.Grid(column_spacing=12, row_spacing=0)
        card_grid.set_column_homogeneous(True)
        # Operator-first order: mission-critical values precede subsystem health.
        # Safety is expressed as a stop cause, not as another device card.
        specs = (
            ("drive", "주행"), ("ai", "인식"),
            ("power", "전원"), ("camera", "통신"),
            ("arm", "로봇팔"), ("safety", "안전"),
        )
        for index, (key, title) in enumerate(specs):
            card = Gtk.Button()
            card.set_relief(Gtk.ReliefStyle.NONE)
            card.set_size_request(-1, 104)
            card.set_tooltip_text(f"{title} 상세 정보 보기")
            _style(
                card, "status-summary-card", "summary-offline",
                f"category-{key}",
            )
            content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
            title_row = Gtk.Box(spacing=7)
            dot = Gtk.Label(label="")
            dot.set_size_request(8, 8)
            _style(dot, "status-dot", "status-muted")
            dot.set_no_show_all(True)
            dot.hide()
            name = Gtk.Label(label=title)
            name.set_xalign(0.5)
            name.set_halign(Gtk.Align.CENTER)
            _style(name, "system-name")
            title_row.pack_start(dot, False, False, 0)
            title_row.pack_start(name, True, True, 0)
            state = Gtk.Label(label="정보 없음")
            state.set_xalign(0.0)
            _style(state, "status-summary-value")
            state.set_no_show_all(True)
            state.hide()
            reason = Gtk.Label(label="정보 없음")
            reason.set_xalign(0.0)
            reason.set_ellipsize(Pango.EllipsizeMode.END)
            _style(reason, "muted")
            reason.set_no_show_all(True)
            reason.hide()
            content.pack_start(title_row, False, False, 0)
            content.pack_start(state, False, False, 0)
            content.pack_start(reason, False, False, 0)
            card.add(content)
            panel_key = self.CARD_TO_PANEL.get(key, key)
            card.connect("clicked", self._on_card_clicked, panel_key)
            card_grid.attach(card, index, 0, 1, 1)
            self._cards[key] = (state, reason, dot)
            self._card_buttons[panel_key] = card
        self.pack_start(card_grid, False, False, 0)

        self._selected_panel = "drive"
        self._detail_title = Gtk.Label(label="주행 시스템 상세 정보")
        self._detail_title.set_xalign(0.0)
        _style(self._detail_title, "section-title")
        self._detail_title.set_no_show_all(True)
        self._detail_title.hide()
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
                ("ai", "AI 인식·작업 대상"),
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
                value = Gtk.Label(label="정보 없음")
                value.set_xalign(0.0)
                value.set_ellipsize(Pango.EllipsizeMode.END)
                _style(value, "detail-metric-value")
                metric.pack_start(heading, False, False, 0)
                metric.pack_start(value, False, False, 0)
                grid.attach(metric, index % 3, index // 3, 1, 1)
                if index >= 3:
                    metric.set_no_show_all(True)
                    metric.hide()
                metrics[key] = value
            self._panels[panel_key].body.pack_start(grid, False, False, 0)
            self._detail_metrics[panel_key] = metrics

        add_metric_grid("drive", (
            ("speed", "평균 속도"), ("state", "운용 모드"),
            ("pose", "오도메트리"),
            ("wheels", "4륜 구동·조향 상태"),
        ))
        add_metric_grid("safety", (
            ("distance", "감지 거리"), ("enabled", "자동 정지"),
            ("arm_temperature", "최고 온도"), ("estop", "현재 정지 판정"),
            ("sensor", "작동한 안전 로직"),
            ("dynamixel", "다이나믹셀 안전 상태"),
        ))
        add_metric_grid("arm", (
            ("type", "엔드이펙터"), ("operation", "조종 모드"),
            ("joints", "관절 부하"), ("mounted", "체결 상태"), ("id", "툴 ID"),
            ("interface", "툴 인터페이스"),
        ))
        add_metric_grid("network", (
            ("front", "전방 영상"), ("work", "작업 영상"),
            ("control", "제어 지연"), ("ov5640", "OV5640 영상 통신"),
            ("rgb_depth", "RGB·Depth 처리"),
            ("metadata", "AI 데이터 통신"),
        ))
        add_metric_grid("ai", (
            ("live", "인식 상태"), ("target", "작업 대상"),
            ("confidence", "신뢰도"), ("distance", "대상 거리"),
            ("direction", "대상 방향"), ("detections", "인식된 물체"),
        ))
        self._panels["drive"].body.pack_start(self._steering, False, False, 0)
        self._panels["ai"].body.pack_start(
            ReferenceStatusCanvas("ai"), False, False, 0,
        )
        self._panels["network"].body.pack_start(
            ReferenceStatusCanvas("network"), False, False, 0,
        )
        self._panels["safety"].body.pack_start(
            ReferenceStatusCanvas("safety"), False, False, 0,
        )
        self._power_metrics: dict[str, Gtk.Label] = {}
        power_grid = Gtk.Grid(column_spacing=8, row_spacing=8)
        power_grid.set_column_homogeneous(True)
        for index, (key, title) in enumerate((
            ("voltage", "입력전압"), ("discharge", "방전전류"),
            ("power", "순간 전력"), ("charge", "충전전류"),
            ("soc", "배터리 SoC"), ("operating", "충전·제어 상태"),
            ("rs485", "PDIST80B 통신"),
        )):
            metric = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
            _style(metric, "power-metric")
            heading = Gtk.Label(label=title)
            heading.set_xalign(0.0)
            _style(heading, "power-metric-title")
            value = Gtk.Label(label="정보 없음")
            value.set_xalign(0.0)
            value.set_ellipsize(Pango.EllipsizeMode.END)
            value.set_width_chars(12)
            value.set_max_width_chars(18)
            _style(value, "power-metric-value")
            metric.pack_start(heading, False, False, 0)
            metric.pack_start(value, False, False, 0)
            power_grid.attach(metric, index % 3, index // 3, 1, 1)
            if index >= 3:
                metric.set_no_show_all(True)
                metric.hide()
            self._power_metrics[key] = value
        self._panels["power"].body.pack_start(power_grid, False, False, 0)
        self._power_state = Gtk.Label(label="보호 상태 · 정보 없음")
        self._power_state.set_xalign(0.0)
        self._power_state.set_line_wrap(True)
        _style(self._power_state, "power-state-summary")
        self._power_state.set_no_show_all(True)
        self._power_state.hide()
        self._panels["power"].body.pack_start(
            self._power_state, False, False, 0,
        )
        self._soc_bar = Gtk.ProgressBar()
        self._soc_bar.set_show_text(True)
        self._soc_bar.set_no_show_all(True)
        _style(self._soc_bar, "soc-progress")
        self._soc_hero = Gtk.Label(label="—")
        self._soc_hero.set_xalign(0.12)
        _style(self._soc_hero, "soc-hero")
        self._panels["power"].body.pack_start(
            self._soc_hero, False, False, 0,
        )
        self._panels["power"].body.pack_start(self._soc_bar, False, False, 0)
        self._panels["arm"].body.pack_start(self._joints, False, False, 0)
        arm_setup = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        _style(arm_setup, "arm-setup")
        arm_setup.set_no_show_all(True)
        arm_setup.hide()
        setup_title = Gtk.Label(label="로봇팔·엔드이펙터 운용 설정")
        setup_title.set_xalign(0.0)
        _style(setup_title, "section-title")
        arm_setup.pack_start(setup_title, False, False, 0)
        setup_grid = Gtk.Grid(column_spacing=10, row_spacing=8)
        setup_grid.set_column_homogeneous(True)

        def add_selector(
            column: int, title: str, values: tuple[str, ...],
        ) -> Gtk.ComboBoxText:
            field = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
            heading = Gtk.Label(label=title)
            heading.set_xalign(0.0)
            _style(heading, "detail-metric-title")
            selector = Gtk.ComboBoxText()
            for value in values:
                selector.append_text(value)
            selector.set_active(0)
            field.pack_start(heading, False, False, 0)
            field.pack_start(selector, False, False, 0)
            setup_grid.attach(field, column, 0, 1, 1)
            return selector

        self._arm_mode_selector = add_selector(
            0, "조종 모드", (
                "마스터–슬레이브 텔레옵", "원격 조종", "자율",
            ),
        )
        self._end_effector_selector = add_selector(
            1, "엔드이펙터 수동 확인값", (
                "미확인", *END_EFFECTOR_PURPOSES.keys(),
            ),
        )
        self._sensor_interface_selector = add_selector(
            2, "툴 인터페이스", (
                "미확인", "기계식", "동력·제어", "센서 데이터", "비전 데이터",
            ),
        )
        self._arm_mode_selector.connect("changed", self._on_arm_plan_changed)
        self._end_effector_selector.connect("changed", self._on_arm_plan_changed)
        self._sensor_interface_selector.connect("changed", self._on_arm_plan_changed)
        arm_setup.pack_start(setup_grid, False, False, 0)
        self._end_effector_purpose = Gtk.Label(
            label="종류를 선택하면 로봇팔팀 정의 용도를 표시합니다"
        )
        self._end_effector_purpose.set_xalign(0.0)
        self._end_effector_purpose.set_line_wrap(True)
        _style(self._end_effector_purpose, "muted")
        arm_setup.pack_start(self._end_effector_purpose, False, False, 0)
        setup_footer = Gtk.Box(spacing=10)
        self._arm_setup_state = Gtk.Label(
            label="로봇 미전송 · 제어 토픽/서비스 계약 대기"
        )
        self._arm_setup_state.set_xalign(0.0)
        _style(self._arm_setup_state, "status-priority")
        self._arm_setup_apply = Gtk.Button(label="적용 (연동 대기)")
        self._arm_setup_apply.set_sensitive(False)
        self._arm_setup_apply.set_tooltip_text(
            "로봇팔팀 제어 토픽/서비스와 상태 응답 계약 후 활성화됩니다"
        )
        setup_footer.pack_start(self._arm_setup_state, True, True, 0)
        setup_footer.pack_end(self._arm_setup_apply, False, False, 0)
        arm_setup.pack_start(setup_footer, False, False, 0)
        self._panels["arm"].body.pack_start(arm_setup, False, False, 0)
        self._can_nodes = Gtk.Label(label="CAN 노드 정보 없음")
        self._can_nodes.set_xalign(0.0)
        self._can_nodes.set_line_wrap(True)
        _style(self._can_nodes, "developer-box")
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

        # Raw sequence/port diagnostics remain available to tests and support
        # tooling, but are intentionally absent from the competition UI.
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
        self._apply_panel_visibility()
        GLib.timeout_add(GRAPH_REFRESH_MS, self._redraw_graphs)

    def _on_arm_plan_changed(self, *_args: object) -> None:
        tool = self.end_effector_plan_text()
        selected = self._end_effector_selector.get_active_text() or "미확인"
        self._end_effector_purpose.set_text(
            END_EFFECTOR_PURPOSES.get(
                selected, "장착 종류를 수신하거나 수동으로 확인해 주세요",
            )
        )
        self._arm_setup_state.set_text(
            f"수동 확인값 · {tool} · 로봇 상태값 아님"
        )

    def end_effector_plan_text(self) -> str:
        observed = getattr(self, "_observed_end_effector", None)
        if observed is not None and observed.end_effector_type:
            attached = (
                "체결" if observed.end_effector_attached is True
                else "미체결" if observed.end_effector_attached is False
                else "체결 미확인"
            )
            return f"{observed.end_effector_type} · {attached}"
        tool = self._end_effector_selector.get_active_text() or "미확인"
        return f"{tool} · 수동 확인"

    def selected_end_effector(self) -> str:
        """Return the operator's explicit end-effector selection."""
        return self._end_effector_selector.get_active_text() or "미확인"

    def select_end_effector(self, tool: str) -> bool:
        """Synchronize an external selector with the arm setup selector."""
        model = self._end_effector_selector.get_model()
        for index, row in enumerate(model):
            if row[0] == tool:
                self._end_effector_selector.set_active(index)
                return True
        return False

    def connect_end_effector_changed(self, callback: object) -> int:
        """Notify another view when the operator changes the selection."""
        return self._end_effector_selector.connect("changed", callback)

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
            "연동 예정": "status-progress", "정지 없음": "status-live",
            "정지 중": "status-bad", "보호 해제": "status-warn",
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
        control_link_ready: bool | None = None,
        chassis_mode: str | None = None,
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
            drive_reason = "주행 정보 없음"
        power_state, power_reason = power_card_state(power, fresh=power_fresh)
        safety_state = self._state(chassis, now_s)
        safety_reason = "안전 정보 없음"
        if chassis_fresh and chassis is not None:
            if chassis.component_mask is not None and chassis.component_mask.get("us100") is False:
                safety_state, safety_reason = "보호 해제", "US-100 자동 정지 비활성"
            elif chassis.safety_estop_required is True:
                safety_state = "정지 중"
                safety_reason = chassis.safety_detail or chassis.safety_status or "원인 미수신"
            elif chassis.safety_estop_required is False:
                safety_state, safety_reason = "정지 없음", "작동 중인 안전 로직 없음"
            else:
                safety_state, safety_reason = "정보 없음", "안전 판정 정보 없음"
        arm_temperature_alerts = ()
        if arm_fresh and arm is not None and arm.dynamixel is not None:
            arm_temperature_alerts = tuple(
                motor for motor in arm.dynamixel
                if temperature_state(motor.temperature_c) != "NORMAL"
            )
        if arm_temperature_alerts and safety_state == "정지 없음":
            safety_state = "확인 필요"
            safety_reason = (
                f"다이나믹셀 온도 확인 · {len(arm_temperature_alerts)}개 모터"
            )
        camera_states = {front_video_state, work_video_state}
        if camera_states == {"LIVE"}:
            camera_state, camera_reason = "정상", "전방·작업 영상 수신 중"
        elif "LIVE" in camera_states:
            camera_state, camera_reason = "확인 필요", "일부 영상만 수신 중"
        elif "STALE" in camera_states:
            camera_state, camera_reason = "연결 끊김", "영상 갱신 지연"
        else:
            camera_state, camera_reason = "연결 중", "영상 미연결"
        arm_state = self._state(arm, now_s)
        arm_reason = (
            "로봇팔 정보 없음"
            if arm is None else "관절 상태 수신 중 · 설정 명령 연동 대기"
        )
        ai_state = "정상" if metadata_fresh else (
            "확인 필요" if metadata is not None else "정보 없음"
        )
        ai_reason = (
            "인식 결과 수신 중" if metadata_fresh
            else "AI 정보 갱신 지연" if metadata is not None
            else "AI 정보 없음"
        )
        states = {
            "drive": (drive_state, drive_reason),
            "power": (power_state, power_reason),
            "safety": (safety_state, safety_reason),
            "camera": (camera_state, camera_reason),
            "ai": (ai_state, ai_reason),
            "arm": (arm_state, arm_reason),
        }
        for key, (state, reason) in states.items():
            self._set_card(key, state, reason)
        required_keys = ("drive", "power", "safety", "camera")
        required_ready = sum(
            states[key][0] in ("정상", "정지 없음")
            for key in required_keys
        )
        work_attention = states["arm"][0] != "정상" or ai_state != "정상"
        self._required_count.set_text(
            f"필수 시스템 {required_ready} / {len(required_keys)} 준비"
        )
        problems = [
            (key, reason)
            for key, (state, reason) in states.items()
            if state not in ("정상", "정지 없음")
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
                "정보 없음" if not speeds
                else f"{sum(abs(v) for v in speeds) / len(speeds):.2f} turn/s"
            )
            displayed_mode = chassis_mode if chassis_mode and chassis_mode != "UNKNOWN" else None
            drive_metrics["state"].set_text(
                chassis.drive_state if displayed_mode is None
                else f"{displayed_mode} · {chassis.drive_state}"
            )
            odometry_source = chassis.odometry_source.strip()
            source_label = (
                "출처 미확인" if not odometry_source
                or odometry_source.lower().startswith("unavailable")
                else odometry_source
            )
            drive_metrics["pose"].set_text(
                "정보 없음" if chassis.yaw_rad is None else
                f"x {chassis.x_m if chassis.x_m is not None else float('nan'):.2f} m · "
                f"y {chassis.y_m if chassis.y_m is not None else float('nan'):.2f} m · "
                f"차체 {math.degrees(chassis.yaw_rad):+.1f}° · {source_label}"
            )
            drive_metrics["wheels"].set_text(
                f"{len(chassis.wheel_statuses)}개 · fault "
                f"{chassis.wheel_fault_count if chassis.wheel_fault_count is not None else '--'}"
            )
            node_lines = []
            for wheel in chassis.wheel_statuses:
                state = "TIMEOUT" if wheel.stale else "FAULT" if (
                    wheel.drive_axis_error or wheel.steer_fault
                ) else "ONLINE"
                if state != "ONLINE":
                    node_lines.append(f"{wheel.name} · {state}")
            self._can_nodes.set_text(
                "\n".join(
                    f"{wheel.name}  ·  "
                    f"속도 {wheel.drive_turns_per_s:.2f} turn/s  ·  "
                    f"조향 {wheel.steer_deg:+.1f}°  ·  "
                    + (
                        "통신 지연" if wheel.stale else
                        "모터 오류" if wheel.drive_axis_error or wheel.steer_fault
                        else "정상"
                    )
                    for wheel in chassis.wheel_statuses
                    if wheel.drive_turns_per_s is not None
                    and wheel.steer_deg is not None
                ) or (
                    "모터 이상\n" + "\n".join(node_lines) if node_lines
                    else f"모터 통신 정상 · {len(chassis.wheel_statuses)}개"
                )
            )
            self._panels["drive"].values.set_text(
                "구동·조향 모터와 현재 이동 상태"
            )
            self._steering.update_wheels(chassis.wheel_statuses)
            self._panels["drive"].set_data_available(
                bool(chassis.wheel_statuses),
                "",
            )
        else:
            for value in self._detail_metrics["drive"].values():
                value.set_text("정보 없음")
            self._steering.update_wheels(())
            self._can_nodes.set_text("CAN 노드 정보 없음")
            self._panels["drive"].set_data_available(
                True, "",
            )
            self._panels["drive"].values.set_text(
                "구동·조향 모터와 현재 이동 상태"
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
                "배터리 잔량과 전원 보호 상태"
            )
            self._power_metrics["voltage"].set_text(voltage)
            self._power_metrics["discharge"].set_text(
                "정보 없음" if power.current_a is None
                else f"{power.current_a:.1f} A"
            )
            self._power_metrics["charge"].set_text(
                "정보 없음" if power.pdist_charge_current_a is None
                else f"{power.pdist_charge_current_a:.1f} A"
            )
            measured_power = power.power_w
            if measured_power is None and power.voltage_v is not None and power.current_a is not None:
                measured_power = power.voltage_v * power.current_a
            self._power_metrics["power"].set_text(
                "정보 없음" if measured_power is None else f"{measured_power:.0f} W"
            )
            self._power_metrics["soc"].set_text(soc)
            self._power_metrics["operating"].set_text(
                "대기" if not operating_states else " · ".join(operating_states)
            )
            rs485_detail = power.rs485_detail.strip()
            self._power_metrics["rs485"].set_text(
                power.rs485_state if not rs485_detail
                else f"{power.rs485_state} · {rs485_detail}"
            )
            self._power_state.set_text(
                f"BMS 보호 상태  ·  {protection}"
            )
            self._panels["power"].set_data_available(
                True,
                "",
            )
            if power.pdist_soc_percent is None:
                self._soc_hero.set_text("—")
                self._soc_bar.hide()
            else:
                soc_value = max(0.0, min(100.0, power.pdist_soc_percent))
                self._soc_hero.set_text(f"{soc_value:.0f}%")
                self._soc_bar.set_fraction(soc_value / 100.0)
                self._soc_bar.set_text(f"SoC {soc_value:.0f}%")
                self._soc_bar.show()
        else:
            self._soc_hero.set_text("—")
            self._soc_bar.hide()
            for value in self._power_metrics.values():
                value.set_text("정보 없음")
            self._power_state.set_text(
                "배터리·전원 보호 상태  ·  정보 없음"
            )
            self._panels["power"].values.set_text(
                "배터리 잔량과 전원 보호 상태"
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
            arm_metrics["joints"].set_text(
                "정보 없음" if not arm.joint_names
                else f"{len(arm.joint_names)}축 수신 중"
            )
            self._panels["arm"].values.set_text(
                "관절 상태와 작업 장치 운용 계획"
            )
            self._panels["arm"].set_data_available(
                bool(arm.joint_names or arm.dynamixel),
                "",
            )
        else:
            for value in self._detail_metrics["arm"].values():
                value.set_text("정보 없음")
            self._joints.set_rows(())
            self._temperatures.set_rows(())
            self._arm_currents.set_rows(())
            self._panels["arm"].set_data_available(
                True, "",
            )
            self._panels["arm"].values.set_text(
                "관절 상태와 작업 장치 운용 계획"
            )
        arm_metrics = self._detail_metrics["arm"]
        self._observed_end_effector = arm if arm_fresh else None
        selected_mode = (
            self._arm_mode_selector.get_active_text()
            or "마스터–슬레이브 텔레옵"
        )
        arm_metrics["operation"].set_text(
            f"{selected_mode} · 수동 확인값"
        )
        observed_type = None if not arm_fresh or arm is None else arm.end_effector_type
        observed_id = None if not arm_fresh or arm is None else arm.end_effector_id
        observed_attached = (
            None if not arm_fresh or arm is None else arm.end_effector_attached
        )
        observed_interface = (
            None if not arm_fresh or arm is None else arm.end_effector_interface
        )
        manual_type = self._end_effector_selector.get_active_text() or "미확인"
        manual_interface = (
            self._sensor_interface_selector.get_active_text() or "미확인"
        )
        arm_metrics["type"].set_text(
            observed_type if observed_type else f"{manual_type} · 수동 확인"
        )
        arm_metrics["mounted"].set_text(
            "체결됨" if observed_attached is True
            else "미체결" if observed_attached is False
            else "상태 필드 미수신"
        )
        arm_metrics["id"].set_text(observed_id or "상태 필드 미수신")
        arm_metrics["interface"].set_text(
            observed_interface
            if observed_interface else f"{manual_interface} · 수동 확인"
        )
        displayed_type = observed_type or manual_type
        self._end_effector_purpose.set_text(
            END_EFFECTOR_PURPOSES.get(
                displayed_type, "장착 종류를 수신하거나 수동으로 확인해 주세요",
            )
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
            safety_enabled = (
                None if chassis.component_mask is None
                else chassis.component_mask.get("us100")
            )
            safety_metrics["enabled"].set_text(
                "정보 없음" if safety_enabled is None
                else "활성 · 자동 정지" if safety_enabled
                else "비활성 · 자동 정지 안 함"
            )
            self._panels["safety"].values.set_text(
                f"충돌 방지·E-STOP 상태   ·   {chassis.safety_detail or '상세 원인 없음'}"
            )
            self._panels["safety"].set_data_available(
                True,
                "",
            )
        else:
            for value in self._detail_metrics["safety"].values():
                value.set_text("정보 없음")
            self._panels["safety"].set_data_available(
                True, "",
            )
            self._panels["safety"].values.set_text(
                "충돌 방지 센서와 긴급 정지 상태"
            )
        safety_metrics = self._detail_metrics["safety"]
        highest_arm_temp = None
        if arm_fresh and arm is not None:
            highest_arm_temp = max(
                (motor.temperature_c for motor in (arm.dynamixel or ())),
                default=None,
            )
        safety_metrics["arm_temperature"].set_text(
            "정보 없음" if highest_arm_temp is None
            else f"{highest_arm_temp} ℃ · {temperature_state(highest_arm_temp)}"
        )
        if not arm_fresh or arm is None or arm.dynamixel is None:
            dynamixel_safety = "정보 없음"
        elif not arm.dynamixel:
            dynamixel_safety = "모터 0개"
        else:
            warning_count = sum(
                temperature_state(motor.temperature_c) != "NORMAL"
                for motor in arm.dynamixel
            )
            dynamixel_safety = (
                f"{len(arm.dynamixel)}개 정상" if not warning_count
                else f"{warning_count}/{len(arm.dynamixel)}개 온도 확인 필요"
            )
        safety_metrics["dynamixel"].set_text(dynamixel_safety)
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
        network_metrics["ov5640"].set_text("미연결 · 영상 소스/포트 미구현")
        color_hz = None if chassis is None else chassis.l515_color_hz
        depth_hz = None if chassis is None else chassis.l515_depth_hz
        network_metrics["rgb_depth"].set_text(
            "정보 없음" if color_hz is None and depth_hz is None else
            f"RGB {color_hz:.1f} Hz · Depth {depth_hz:.1f} Hz"
            if color_hz is not None and depth_hz is not None else
            f"RGB {color_hz:.1f} Hz" if color_hz is not None
            else f"Depth {depth_hz:.1f} Hz"
        )
        network_metrics["metadata"].set_text(
            "정상" if metadata_fresh else "정보 없음" if metadata is None else "갱신 지연"
        )
        network_metrics["control"].set_text(
            "LIVE" if control_link_ready is True else
            "UNAVAILABLE" if control_link_ready is False else "정보 없음"
        )
        self._panels["network"].values.set_text(
            "전방·작업 카메라와 RGB-D·AI 연결 상태"
        )
        self._panels["network"].set_data_available(
            True, "",
        )
        ai_metrics = self._detail_metrics["ai"]
        target = pick_display_target(metadata) if metadata_fresh else None
        ai_metrics["live"].set_text(
            "인식 결과 갱신 중" if metadata_fresh
            else "갱신 지연" if metadata is not None else "수신 대기"
        )
        ai_metrics["target"].set_text("없음" if target is None else target.class_name)
        ai_metrics["confidence"].set_text(
            "정보 없음" if target is None else f"{target.confidence * 100:.0f}%"
        )
        distance = target_distance_m(target)
        ai_metrics["distance"].set_text(
            "정보 없음" if distance is None else f"{distance:.2f} m"
        )
        ai_metrics["direction"].set_text(
            "정보 없음" if target is None or target.yaw_rad is None
            else f"정면 기준 {math.degrees(target.yaw_rad):+.1f}°"
        )
        ai_metrics["detections"].set_text(
            "정보 없음" if metadata is None else f"{len(metadata.detections)}개"
        )
        self._panels["ai"].values.set_text(
            "현재 작업 대상과 접근 거리"
        )
        self._panels["ai"].set_data_available(True, "")
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
                "정보 없음" if received is None
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
            f"gateway cpu={getattr(chassis, 'l515_process_cpu_percent', None)}% "
            f"rss={getattr(chassis, 'l515_process_rss_bytes', None)} bytes\n"
            f"units={getattr(chassis, 'unit_status', ())}\n"
            f"containers={getattr(chassis, 'compose_status', ())}\n"
            "unwired=cmd/actual velocity, rail states, HW E-stop, RSSI/RTT, arm mode/tool\n"
            "sources=UDP :{}/:{}/:{}/:{} · control writes=none".format(
                *self._source_ports,
            )
        )


class _LegacyCompetitionStatusDashboard(RobotStatusDashboard):
    """Photo-matched presentation rebuilt on top of the RX-only data model."""

    SPECS = (
        ("drive", "주행", ("speed", "state", "pose")),
        ("ai", "인식", ("live", "target", "confidence")),
        ("power", "전원", ("voltage", "discharge", "power")),
        ("network", "통신", ("front", "work", "control")),
        ("arm", "로봇팔", ("type", "operation", "joints")),
        ("safety", "안전", ("distance", "enabled", "arm_temperature")),
    )

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        for child in tuple(self.get_children()):
            self.remove(child)
        self.set_border_width(0)
        self.set_spacing(0)
        self.set_hexpand(True)
        self.set_vexpand(True)
        _style(self, "competition-status")

        pills = Gtk.Grid(column_spacing=18)
        pills.set_column_homogeneous(True)
        pills.set_hexpand(True)
        _style(pills, "competition-pills")
        self._photo_buttons: dict[str, Gtk.Button] = {}
        self._card_buttons = {}
        for index, (key, title, _metric_keys) in enumerate(self.SPECS):
            button = Gtk.Button(label=title)
            button.set_relief(Gtk.ReliefStyle.NONE)
            # The compact subsystem strip is a fixed visual landmark in the
            # approved 1600 x 1000 composition.
            button.set_size_request(-1, 88)
            _style(button, "status-summary-card", f"category-{key}")
            button.connect("clicked", self._on_photo_panel_clicked, key)
            pills.attach(button, index, 0, 1, 1)
            self._photo_buttons[key] = button
            self._card_buttons[key] = button
        self.pack_start(pills, False, False, 0)

        self._photo_stack = Gtk.Stack()
        self._photo_stack.set_hexpand(True)
        self._photo_stack.set_vexpand(True)
        # Status canvases are large and update from several telemetry sources.
        # A crossfade keeps both pages composited during every switch and is
        # noticeably expensive on the operator laptop's integrated GPU.
        self._photo_stack.set_transition_type(Gtk.StackTransitionType.NONE)
        self._photo_stack.set_transition_duration(0)
        self._photo_values: dict[str, dict[str, Gtk.Label]] = {}
        titles = {
            "drive": ("평균 속도", "운용 모드", "오도메트리"),
            "ai": ("인식 상태", "작업 대상", "신뢰도"),
            "power": ("입력전압", "방전전류", "순간 전력"),
            "network": ("전방 영상", "작업 영상", "제어 지연"),
            "arm": ("엔드이펙터", "조종 모드", "관절 부하"),
            "safety": ("감지 거리", "자동 정지", "최고 온도"),
        }
        visuals = {
            "drive": self._steering,
            "ai": ReferenceStatusCanvas("ai"),
            "power": Gtk.Box(orientation=Gtk.Orientation.VERTICAL),
            "network": ReferenceStatusCanvas("network"),
            "arm": self._joints,
            "safety": ReferenceStatusCanvas("safety"),
        }
        power_visual = visuals["power"]
        for widget in (self._soc_hero, self._soc_bar):
            parent = widget.get_parent()
            if isinstance(parent, Gtk.Container):
                parent.remove(widget)
        power_visual.pack_start(self._soc_hero, True, True, 0)
        power_visual.pack_start(self._soc_bar, False, False, 22)
        for key, _title, metric_keys in self.SPECS:
            page = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=0)
            page.set_hexpand(True)
            page.set_vexpand(True)
            _style(page, "competition-status-page", f"competition-{key}")
            metrics = Gtk.Grid(column_spacing=0)
            metrics.set_column_homogeneous(True)
            values: dict[str, Gtk.Label] = {}
            for index, (metric_key, heading) in enumerate(
                zip(metric_keys, titles[key])
            ):
                cell = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
                _style(cell, "competition-metric")
                label = Gtk.Label(label=heading)
                label.set_xalign(0)
                _style(label, "competition-metric-label")
                value = Gtk.Label(label="정보 없음")
                value.set_xalign(0)
                value.set_ellipsize(Pango.EllipsizeMode.END)
                _style(value, "competition-metric-value")
                cell.pack_start(label, False, False, 0)
                cell.pack_start(value, False, False, 0)
                metrics.attach(cell, index, 0, 1, 1)
                values[metric_key] = value
            page.pack_start(metrics, False, False, 0)
            visual = visuals[key]
            parent = visual.get_parent()
            if isinstance(parent, Gtk.Container):
                parent.remove(visual)
            page.pack_start(visual, True, True, 0)
            self._photo_values[key] = values
            self._photo_stack.add_named(page, key)

        standby = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=26)
        standby.set_hexpand(True)
        standby.set_vexpand(True)
        _style(standby, "competition-standby")
        hero = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=42)
        hero.set_hexpand(True)
        hero.set_vexpand(True)
        _style(hero, "standby-hero")
        hero_copy = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        hero_copy.set_hexpand(True)
        hero_copy.set_valign(Gtk.Align.CENTER)
        kicker_row = Gtk.Box(spacing=8)
        waiting_dot = Gtk.Spinner()
        waiting_dot.start()
        _style(waiting_dot, "standby-spinner")
        kicker = Gtk.Label(label="RX-ONLY · LISTENING")
        kicker.set_xalign(0.0)
        _style(kicker, "standby-kicker")
        kicker_row.pack_start(waiting_dot, False, False, 0)
        kicker_row.pack_start(kicker, False, False, 0)
        title = Gtk.Label(label="로봇 상태 수신 대기")
        title.set_xalign(0.0)
        _style(title, "standby-title")
        description = Gtk.Label(
            label=("상태 패킷이 도착하면 선택한 시스템의 실시간 진단 화면으로 "
                   "자동 전환됩니다.")
        )
        description.set_xalign(0.0)
        description.set_line_wrap(True)
        _style(description, "standby-description")
        note = Gtk.Label(label="현재 콘솔은 관측 전용이며 로봇에 명령을 보내지 않습니다")
        note.set_xalign(0.0)
        _style(note, "standby-note")
        hero_copy.pack_start(kicker_row, False, False, 0)
        hero_copy.pack_start(title, False, False, 0)
        hero_copy.pack_start(description, False, False, 0)
        hero_copy.pack_start(note, False, False, 8)
        hero.pack_start(hero_copy, True, True, 0)
        hero.pack_end(StandbyLinkCanvas(), False, False, 0)
        standby.pack_start(hero, True, True, 0)

        sources = Gtk.Grid(column_spacing=12)
        sources.set_column_homogeneous(True)
        sources.set_hexpand(True)
        _style(sources, "standby-sources")
        power_port, chassis_port, arm_port, metadata_port = self._source_ports
        source_specs = (
            ("drive", "차체 · 안전", f"UDP :{chassis_port}"),
            ("power", "전원", f"UDP :{power_port}"),
            ("arm", "로봇팔", f"UDP :{arm_port}"),
            ("ai", "AI 인식", f"UDP :{metadata_port}"),
        )
        for index, (key, name, endpoint) in enumerate(source_specs):
            card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
            _style(card, "standby-source-card", f"standby-source-{key}")
            name_label = Gtk.Label(label=name)
            name_label.set_xalign(0.0)
            _style(name_label, "standby-source-name")
            endpoint_label = Gtk.Label(label=endpoint)
            endpoint_label.set_xalign(0.0)
            _style(endpoint_label, "standby-source-endpoint")
            state_row = Gtk.Box(spacing=7)
            dot = Gtk.Label(label="")
            dot.set_size_request(7, 7)
            _style(dot, "standby-source-dot")
            state = Gtk.Label(label="수신 대기")
            state.set_xalign(0.0)
            _style(state, "standby-source-state")
            state_row.pack_start(dot, False, False, 0)
            state_row.pack_start(state, False, False, 0)
            card.pack_start(name_label, False, False, 0)
            card.pack_start(endpoint_label, False, False, 0)
            card.pack_start(state_row, False, False, 3)
            sources.attach(card, index, 0, 1, 1)
        standby.pack_end(sources, False, False, 0)

        self._status_content = Gtk.Stack()
        self._status_content.set_hexpand(True)
        self._status_content.set_vexpand(True)
        self._status_content.set_transition_type(Gtk.StackTransitionType.CROSSFADE)
        self._status_content.set_transition_duration(180)
        self._status_content.add_named(standby, "standby")
        self._status_content.add_named(self._photo_stack, "details")
        self._status_content.set_visible_child_name("standby")
        self._details_requested = False
        self.pack_start(self._status_content, True, True, 0)
        self._selected_panel = "drive"
        self._select_photo_panel("drive")

    def _on_photo_panel_clicked(self, _button: Gtk.Button, key: str) -> None:
        self._details_requested = True
        self._selected_panel = key
        self._select_photo_panel(key)
        self._status_content.set_visible_child_name("details")

    def _select_photo_panel(self, key: str) -> None:
        self._photo_stack.set_visible_child_name(key)
        for candidate, button in self._photo_buttons.items():
            context = button.get_style_context()
            context.remove_class("selected")
            if candidate == key:
                context.add_class("selected")

    def update(self, **kwargs: object) -> None:
        super().update(**kwargs)
        now_s = float(kwargs.get("now_s") or time.monotonic())
        source_active = any(
            self._fresh(kwargs.get(key), now_s)
            for key in ("power", "chassis", "arm", "metadata")
        )
        video_active = any(
            kwargs.get(key) == "LIVE"
            for key in ("front_video_state", "work_video_state")
        )
        self._status_content.set_visible_child_name(
            "details"
            if source_active or video_active or self._details_requested
            else "standby"
        )
        source_groups = {
            "drive": self._detail_metrics["drive"],
            "ai": self._detail_metrics["ai"],
            "power": self._power_metrics,
            "network": self._detail_metrics["network"],
            "arm": self._detail_metrics["arm"],
            "safety": self._detail_metrics["safety"],
        }
        for panel, values in self._photo_values.items():
            sources = source_groups[panel]
            for key, label in values.items():
                label.set_text(sources[key].get_text())


class CompetitionStatusDashboard(RobotStatusDashboard):
    """Unified RX-only power, communication, and safety surface.

    The base class continues to own the bounded data series and decoders.  This
    presentation intentionally does not expose the former drive, AI, and arm
    detail pages: their operator-facing values live on the mission page.
    """

    def __init__(self, **kwargs: object) -> None:
        super().__init__(**kwargs)
        for child in tuple(self.get_children()):
            self.remove(child)
        self.set_border_width(18)
        self.set_spacing(14)
        self.set_hexpand(True)
        self.set_vexpand(True)
        _style(self, "system-status-dashboard")

        heading = Gtk.Box(spacing=14)
        heading_copy = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
        title = Gtk.Label(label="시스템 상태")
        title.set_xalign(0.0)
        _style(title, "system-status-title")
        subtitle = Gtk.Label(
            label="전원 · 통신 · 안전을 통합해 운용에 필요한 상태만 표시합니다"
        )
        subtitle.set_xalign(0.0)
        _style(subtitle, "system-status-subtitle")
        heading_copy.pack_start(title, False, False, 0)
        heading_copy.pack_start(subtitle, False, False, 0)
        self._system_overall = Gtk.Label(label="●  상태 정보 수신 대기")
        self._system_overall.set_xalign(1.0)
        _style(self._system_overall, "system-overall-chip", "status-muted")
        heading.pack_start(heading_copy, True, True, 0)
        heading.pack_end(self._system_overall, False, False, 0)
        self.pack_start(heading, False, False, 0)

        self._system_values: dict[str, Gtk.Label] = {}
        self._system_badges: dict[str, Gtk.Label] = {}

        def metric(key: str, label_text: str) -> Gtk.Box:
            card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
            _style(card, "system-metric")
            label = Gtk.Label(label=label_text)
            label.set_xalign(0.0)
            _style(label, "system-metric-label")
            value = Gtk.Label(label="정보 없음")
            value.set_xalign(0.0)
            value.set_ellipsize(Pango.EllipsizeMode.END)
            value.set_width_chars(12)
            value.set_max_width_chars(18)
            value.set_tooltip_text("수신 텔레메트리에 값이 없으면 정보 없음으로 표시됩니다")
            _style(value, "system-metric-value")
            card.pack_start(label, False, False, 0)
            card.pack_start(value, False, False, 0)
            self._system_values[key] = value
            return card

        def section(
            key: str, title_text: str, description: str,
            metrics: tuple[tuple[str, str], ...],
        ) -> Gtk.Box:
            panel = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
            panel.set_hexpand(True)
            panel.set_vexpand(True)
            _style(panel, "system-section", f"system-section-{key}")
            section_header = Gtk.Box(spacing=8)
            title_label = Gtk.Label(label=title_text)
            title_label.set_xalign(0.0)
            _style(title_label, "system-section-title")
            badge = Gtk.Label(label="정보 없음")
            badge.set_xalign(1.0)
            _style(badge, "system-section-badge", "status-muted")
            section_header.pack_start(title_label, True, True, 0)
            section_header.pack_end(badge, False, False, 0)
            description_label = Gtk.Label(label=description)
            description_label.set_xalign(0.0)
            description_label.set_line_wrap(True)
            _style(description_label, "system-section-description")
            grid = Gtk.Grid(column_spacing=8, row_spacing=8)
            grid.set_column_homogeneous(True)
            for index, spec in enumerate(metrics):
                grid.attach(metric(*spec), index % 2, index // 2, 1, 1)
            panel.pack_start(section_header, False, False, 0)
            panel.pack_start(description_label, False, False, 0)
            panel.pack_start(grid, False, False, 0)
            self._system_badges[key] = badge
            return panel

        columns = Gtk.Grid(column_spacing=12, row_spacing=0)
        columns.set_column_homogeneous(True)
        columns.set_hexpand(True)
        columns.set_vexpand(True)
        power = section(
            "power", "전원 · PDIST80B",
            "계측값, 배터리 상태, 보호 플래그와 RS485 수신 상태",
            (
                ("power_soc", "배터리 잔량"),
                ("power_voltage", "입력전압"),
                ("power_discharge", "방전전류"),
                ("power_charge", "충전전류"),
                ("power_watts", "순간 전력"),
                ("power_operating", "충전·제어 상태"),
                ("power_protection", "보호 상태"),
                ("power_rs485", "PDIST80B 통신"),
            ),
        )
        soc_parent = self._soc_bar.get_parent()
        if isinstance(soc_parent, Gtk.Container):
            soc_parent.remove(self._soc_bar)
        self._soc_bar.set_no_show_all(False)
        power.pack_start(self._soc_bar, False, False, 0)
        communication = section(
            "communication", "통신",
            "로봇 상태, 영상, 인식 결과와 조작 채널의 최신 수신 상태",
            (
                ("comm_robot", "로봇 텔레메트리"),
                ("comm_power", "전원 텔레메트리"),
                ("comm_front", "전방 영상"),
                ("comm_work", "작업 영상"),
                ("comm_ai", "인식 데이터"),
                ("comm_control", "조작 채널"),
            ),
        )
        safety = section(
            "safety", "안전",
            "비상정지, 장애물 자동 정지와 로봇팔 온도 보호 상태",
            (
                ("safety_estop", "비상정지"),
                ("safety_auto", "장애물 자동 정지"),
                ("safety_distance", "감지 거리"),
                ("safety_sensor", "안전 판정"),
                ("safety_arm_temp", "로봇팔 최고 온도"),
                ("safety_dynamixel", "관절 모터 상태"),
            ),
        )
        columns.attach(power, 0, 0, 1, 1)
        columns.attach(communication, 1, 0, 1, 1)
        columns.attach(safety, 2, 0, 1, 1)
        self.pack_start(columns, True, True, 0)

        diagnostics = Gtk.Expander(label="고급 진단 · 원본 채널 및 누락 필드")
        diagnostics.set_expanded(False)
        diagnostic_body = Gtk.Box(
            orientation=Gtk.Orientation.VERTICAL, spacing=8,
        )
        diagnostic_body.set_border_width(10)
        missing = Gtk.Label(
            label=("현재 계약 미포함: 전압 다중 PID 편차, 알람 누적 횟수, "
                   "개별 분배 출력 상태 · 값은 추정하지 않습니다")
        )
        missing.set_xalign(0.0)
        missing.set_line_wrap(True)
        _style(missing, "system-diagnostic-note")
        self._developer.set_no_show_all(False)
        diagnostic_body.pack_start(missing, False, False, 0)
        diagnostic_body.pack_start(self._developer, False, False, 0)
        diagnostics.add(diagnostic_body)
        self.pack_end(diagnostics, False, False, 0)

    @staticmethod
    def _value_or_information(value: str) -> str:
        normalized = value.strip()
        return normalized if normalized and normalized not in {"—", "N/A"} else "정보 없음"

    def _set_badge(self, key: str, text: str, tone: str) -> None:
        badge = self._system_badges[key]
        for candidate in ("status-live", "status-warn", "status-bad", "status-muted"):
            badge.get_style_context().remove_class(candidate)
        badge.get_style_context().add_class(tone)
        badge.set_text(text)

    def update(self, **kwargs: object) -> None:
        super().update(**kwargs)
        now_s = float(kwargs.get("now_s") or time.monotonic())
        power = kwargs.get("power")
        chassis = kwargs.get("chassis")
        arm = kwargs.get("arm")
        metadata = kwargs.get("metadata")
        power_fresh = self._fresh(power, now_s)
        chassis_fresh = self._fresh(chassis, now_s)
        arm_fresh = self._fresh(arm, now_s)
        metadata_fresh = self._fresh(metadata, now_s)

        for target, source in (
            ("power_voltage", "voltage"),
            ("power_discharge", "discharge"),
            ("power_charge", "charge"),
            ("power_watts", "power"),
            ("power_soc", "soc"),
            ("power_operating", "operating"),
            ("power_rs485", "rs485"),
        ):
            self._system_values[target].set_text(
                self._value_or_information(self._power_metrics[source].get_text())
            )
        protection = self._power_state.get_text().replace("BMS 보호 상태  ·  ", "")
        self._system_values["power_protection"].set_text(
            self._value_or_information(protection)
        )

        self._system_values["comm_robot"].set_text(
            "실시간 수신" if chassis_fresh else
            "갱신 지연" if chassis is not None else "수신 대기"
        )
        self._system_values["comm_power"].set_text(
            "실시간 수신" if power_fresh else
            "갱신 지연" if power is not None else "수신 대기"
        )
        network = self._detail_metrics["network"]
        self._system_values["comm_front"].set_text(network["front"].get_text())
        self._system_values["comm_work"].set_text(network["work"].get_text())
        self._system_values["comm_ai"].set_text(
            "실시간 수신" if metadata_fresh else
            "갱신 지연" if metadata is not None else "수신 대기"
        )
        self._system_values["comm_control"].set_text(network["control"].get_text())

        safety = self._detail_metrics["safety"]
        for target, source in (
            ("safety_estop", "estop"),
            ("safety_auto", "enabled"),
            ("safety_distance", "distance"),
            ("safety_sensor", "sensor"),
            ("safety_arm_temp", "arm_temperature"),
            ("safety_dynamixel", "dynamixel"),
        ):
            self._system_values[target].set_text(
                self._value_or_information(safety[source].get_text())
            )

        power_state, _power_reason = power_card_state(power, fresh=power_fresh)
        self._set_badge(
            "power", power_state,
            "status-live" if power_state == "정상" else
            "status-warn" if power is not None else "status-muted",
        )
        comm_ready = chassis_fresh and power_fresh and (
            kwargs.get("front_video_state") == "LIVE"
            or kwargs.get("work_video_state") == "LIVE"
        )
        self._set_badge(
            "communication", "정상" if comm_ready else
            "확인 필요" if chassis is not None or power is not None else "정보 없음",
            "status-live" if comm_ready else
            "status-warn" if chassis is not None or power is not None else "status-muted",
        )
        estop = getattr(chassis, "safety_estop_required", None)
        arm_temperature_attention = bool(
            arm_fresh
            and getattr(arm, "dynamixel", None)
            and any(
                temperature_state(motor.temperature_c) != "NORMAL"
                for motor in arm.dynamixel
            )
        )
        safety_ready = (
            chassis_fresh and estop is False and not arm_temperature_attention
        )
        self._set_badge(
            "safety", "정상" if safety_ready else
            "비상정지" if estop is True else
            "확인 필요" if arm_temperature_attention else
            "확인 필요" if chassis is not None else "정보 없음",
            "status-live" if safety_ready else
            "status-bad" if estop is True else
            "status-warn" if chassis is not None else "status-muted",
        )

        overall_ok = power_state == "정상" and comm_ready and safety_ready
        overall_attention = any(source is not None for source in (power, chassis, arm, metadata))
        overall_context = self._system_overall.get_style_context()
        for candidate in ("status-live", "status-warn", "status-muted"):
            overall_context.remove_class(candidate)
        if overall_ok:
            self._system_overall.set_text("●  운용 준비 완료")
            overall_context.add_class("status-live")
        elif overall_attention:
            self._system_overall.set_text("●  확인 필요한 항목 있음")
            overall_context.add_class("status-warn")
        else:
            self._system_overall.set_text("●  상태 정보 수신 대기")
            overall_context.add_class("status-muted")
        self._developer.show()
