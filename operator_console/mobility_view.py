"""E2FESTA mobility view for terrain-aware six-wheel coordination."""
from __future__ import annotations

import math
import time
from collections.abc import Callable, Mapping

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gtk, Pango  # noqa: E402

from .telemetry import TelemetrySnapshot, WheelStatus


def _style(widget: Gtk.Widget, *classes: str) -> Gtk.Widget:
    context = widget.get_style_context()
    for css_class in classes:
        context.add_class(css_class)
    return widget


def _number(value: float | None, suffix: str, digits: int = 1) -> str:
    if value is None or not math.isfinite(value):
        return "정보 없음"
    return f"{value:.{digits}f}{suffix}"


def _degrees(value: float | None) -> str:
    return _number(None if value is None else math.degrees(value), "°")


def active_faults(snapshot: TelemetrySnapshot | None) -> tuple[str, ...]:
    """Build bounded, operator-facing faults without inventing source data."""
    if snapshot is None:
        return ("차대 텔레메트리 미수신",)
    faults: list[str] = []
    can_state = snapshot.can_state.strip()
    if not can_state or can_state.lower().startswith(("unavailable", "unhealthy")):
        faults.append(f"CAN · {can_state or '정보 없음'}")
    if snapshot.safety_estop_required is True or "ESTOP" in snapshot.drive_state.upper():
        faults.append("비상정지 · " + (snapshot.safety_detail or snapshot.safety_status))
    for wheel in snapshot.wheel_statuses:
        details = []
        if wheel.stale:
            details.append("통신 지연")
        if wheel.drive_axis_error:
            details.append(f"축 오류 0x{wheel.drive_axis_error:X}")
        if wheel.steer_fault:
            details.append(f"조향 오류 0x{wheel.steer_fault:X}")
        if details:
            faults.append(f"{wheel.name} · " + " · ".join(details))
    faults.extend(f"지형 · {reason}" for reason in snapshot.terrain_reject_reasons)
    if snapshot.stuck_candidate is True:
        faults.append("구동 · 끼임 후보")
    elif snapshot.slip_candidate is True:
        faults.append("구동 · 슬립 후보")
    return tuple(faults[:8])


def wheel_health_text(wheel: WheelStatus) -> str:
    if wheel.stale:
        return "통신 지연"
    if wheel.drive_axis_error or wheel.steer_fault or wheel.mode == "FAULT":
        return "오류"
    return "정상"


def _candidate_text(value: bool | None, *, detected: str) -> str:
    if value is None:
        return "정보 없음"
    return detected if value else "감지 없음"


def _maximum_abs(values: tuple[float | None, ...]) -> float | None:
    finite = [abs(value) for value in values if value is not None and math.isfinite(value)]
    return max(finite, default=None)


def mission_mobility_values(
    snapshot: TelemetrySnapshot | None,
) -> dict[str, str]:
    """Return compact, source-honest values for the main mission rail."""
    unavailable = {
        "path": "정보 없음",
        "control": "정보 없음",
        "attitude": "정보 없음",
        "motors": "정보 없음",
    }
    if snapshot is None:
        return unavailable

    confidence = (
        "" if snapshot.terrain_confidence is None
        else f" · 신뢰도 {snapshot.terrain_confidence * 100:.0f}%"
    )
    path = (
        "경로 확보" + confidence
        if snapshot.terrain_path_available is True else
        "경로 차단" + confidence
        if snapshot.terrain_path_available is False else
        "판정 대기"
    )
    scale = (
        "" if snapshot.degradation_speed_scale is None
        else f" · 속도 {snapshot.degradation_speed_scale * 100:.0f}%"
    )
    control = (snapshot.controller_fsm_state or "판정 대기") + scale
    attitude = "Roll {} · Pitch {}".format(
        _degrees(snapshot.roll_rad), _degrees(snapshot.pitch_rad),
    )
    healthy_wheels = sum(
        wheel_health_text(wheel) == "정상" for wheel in snapshot.wheel_statuses
    )
    wheel_count = len(snapshot.wheel_statuses)
    maximum_current = _maximum_abs(tuple(
        wheel.drive_current_a for wheel in snapshot.wheel_statuses
    ))
    contact = (
        " · 끼임 후보" if snapshot.stuck_candidate is True else
        " · 슬립 후보" if snapshot.slip_candidate is True else ""
    )
    motors = (
        "정보 없음" if not wheel_count else
        f"{healthy_wheels}/{wheel_count} 정상 · max |Iq| "
        f"{_number(maximum_current, ' A', 1)}{contact}"
    )
    return {
        "path": path,
        "control": control,
        "attitude": attitude,
        "motors": motors,
    }


class RoverCoordinationGraphic(Gtk.DrawingArea):
    """Interactive engineering view of the six-wheel chassis.

    This is deliberately a lightweight Cairo status view rather than a CAD
    renderer.  Dragging or scrolling changes only the viewing angle; it never
    emits a robot command.  Clicking a wheel selects the live values already
    present in the chassis telemetry contract.
    """

    _WHEEL_POSITIONS = {
        "front_left": (-1, -1),
        "mid_left": (-1, 0),
        "rear_left": (-1, 1),
        "front_right": (1, -1),
        "mid_right": (1, 0),
        "rear_right": (1, 1),
    }
    _WHEEL_LABELS = {
        "front_left": "FL", "mid_left": "ML", "rear_left": "RL",
        "front_right": "FR", "mid_right": "MR", "rear_right": "RR",
    }

    def __init__(self) -> None:
        super().__init__()
        self.set_size_request(330, 210)
        self.set_hexpand(True)
        self.set_vexpand(True)
        _style(self, "mobility-rover-stage")
        self._snapshot: TelemetrySnapshot | None = None
        self._authority = "UNAVAILABLE"
        self._yaw_deg = 28.0
        self._selected_wheel = "front_left"
        self._drag_x: float | None = None
        self._wheel_screen_positions: dict[str, tuple[float, float]] = {}
        self.add_events(
            Gdk.EventMask.BUTTON_PRESS_MASK
            | Gdk.EventMask.BUTTON_RELEASE_MASK
            | Gdk.EventMask.POINTER_MOTION_MASK
            | Gdk.EventMask.SCROLL_MASK
        )
        self.connect("draw", self._draw)
        self.connect("button-press-event", self._on_button_press)
        self.connect("button-release-event", self._on_button_release)
        self.connect("motion-notify-event", self._on_motion)
        self.connect("scroll-event", self._on_scroll)

    @staticmethod
    def _normalise_angle(value: float) -> float:
        return float(value) % 360.0

    def set_view_angle(self, yaw_deg: float) -> None:
        """Set the read-only camera orbit angle; useful for UI automation too."""
        self._yaw_deg = self._normalise_angle(yaw_deg)
        self.queue_draw()

    def select_wheel(self, name: str) -> None:
        if name not in self._WHEEL_POSITIONS:
            raise ValueError(f"unknown wheel: {name}")
        self._selected_wheel = name
        self.queue_draw()

    def set_state(
        self,
        snapshot: TelemetrySnapshot | None,
        ops_state: Mapping[str, object] | None,
    ) -> None:
        self._snapshot = snapshot
        self._authority = (
            "UNAVAILABLE" if ops_state is None
            else str(ops_state.get("authority_mode", "UNAVAILABLE"))
        )
        self.queue_draw()

    @staticmethod
    def _colour(hex_value: str) -> tuple[float, float, float]:
        value = hex_value.lstrip("#")
        return tuple(int(value[index:index + 2], 16) / 255 for index in (0, 2, 4))

    @classmethod
    def _source(cls, context: object, hex_value: str, alpha: float = 1.0) -> None:
        red, green, blue = cls._colour(hex_value)
        context.set_source_rgba(red, green, blue, alpha)

    @classmethod
    def _text(
        cls,
        context: object,
        text: str,
        x: float,
        y: float,
        *,
        colour: str,
        size: float,
        bold: bool = False,
    ) -> None:
        cls._source(context, colour)
        context.select_font_face("Sans", 0, 1 if bold else 0)
        context.set_font_size(size)
        context.move_to(x, y)
        context.show_text(text)

    def _wheel_colour(self, wheel: WheelStatus | None) -> str:
        if wheel is None:
            return "#52677B"
        if wheel.stale:
            return "#E7B34F"
        if wheel.drive_axis_error or wheel.steer_fault or wheel.mode == "FAULT":
            return "#E56A74"
        return "#55C995"

    def _project(
        self, x: float, y: float, z: float, centre_x: float, centre_y: float,
    ) -> tuple[float, float]:
        yaw = math.radians(self._yaw_deg)
        rotated_x = x * math.cos(yaw) - y * math.sin(yaw)
        rotated_y = x * math.sin(yaw) + y * math.cos(yaw)
        return (
            centre_x + rotated_x * 45.0,
            centre_y + rotated_y * 17.0 - z * 34.0,
        )

    def _on_button_press(self, _widget: Gtk.Widget, event: object) -> bool:
        if getattr(event, "button", None) != 1:
            return False
        x, y = float(event.x), float(event.y)
        closest = min(
            self._wheel_screen_positions.items(),
            key=lambda item: math.hypot(item[1][0] - x, item[1][1] - y),
            default=None,
        )
        if closest is not None and math.hypot(
            closest[1][0] - x, closest[1][1] - y,
        ) <= 22.0:
            self._selected_wheel = closest[0]
        self._drag_x = x
        self.queue_draw()
        return True

    def _on_button_release(self, _widget: Gtk.Widget, event: object) -> bool:
        if getattr(event, "button", None) == 1:
            self._drag_x = None
            return True
        return False

    def _on_motion(self, _widget: Gtk.Widget, event: object) -> bool:
        if self._drag_x is None:
            return False
        x = float(event.x)
        self.set_view_angle(self._yaw_deg + (x - self._drag_x) * 0.8)
        self._drag_x = x
        return True

    def _on_scroll(self, _widget: Gtk.Widget, event: object) -> bool:
        direction = getattr(event, "direction", None)
        if direction == Gdk.ScrollDirection.UP:
            delta = 10.0
        elif direction == Gdk.ScrollDirection.DOWN:
            delta = -10.0
        else:
            delta = -float(getattr(event, "delta_y", 0.0)) * 10.0
        self.set_view_angle(self._yaw_deg + delta)
        return True

    def _draw(self, _widget: Gtk.Widget, context: object) -> bool:
        width = float(self.get_allocated_width())
        height = float(self.get_allocated_height())
        centre_x = width * 0.50
        centre_y = height * 0.48
        snapshot = self._snapshot

        path_colour = "#52677B"
        path_label = "PATH · 정보 없음"
        if snapshot is not None and snapshot.terrain_path_available is True:
            path_colour, path_label = "#55B9DE", "PATH · OPEN"
        elif snapshot is not None and snapshot.terrain_path_available is False:
            path_colour, path_label = "#E56A74", "PATH · BLOCKED"

        context.set_line_width(2.0)
        self._source(context, path_colour, 0.9)
        context.move_to(centre_x - 50, 19)
        context.line_to(centre_x + 50, 19)
        context.stroke()
        self._text(
            context, path_label, max(10.0, centre_x - 42), 13,
            colour=path_colour, size=8, bold=True,
        )

        by_name = {
            wheel.name: wheel for wheel in snapshot.wheel_statuses
        } if snapshot is not None else {}
        world_positions = {
            name: (side * 1.18, row * 0.86, -0.28)
            for name, (side, row) in self._WHEEL_POSITIONS.items()
        }
        self._wheel_screen_positions = {
            name: self._project(*position, centre_x, centre_y)
            for name, position in world_positions.items()
        }

        # Draw the far wheels and suspension first so the body has depth.
        ordered_wheels = sorted(
            self._wheel_screen_positions.items(), key=lambda item: item[1][1],
        )
        self._source(context, "#315F91", 0.9)
        context.set_line_width(3.0)
        for name, (wheel_x, wheel_y) in ordered_wheels:
            side, _row = self._WHEEL_POSITIONS[name]
            anchor_x, anchor_y = self._project(
                side * 0.58, 0.0, 0.0, centre_x, centre_y,
            )
            context.move_to(anchor_x, anchor_y)
            context.line_to(wheel_x, wheel_y)
            context.stroke()

        body_border = "#4B8BEA"
        if snapshot is not None and snapshot.stuck_candidate is True:
            body_border = "#E56A74"
        elif snapshot is not None and snapshot.slip_candidate is True:
            body_border = "#E7B34F"
        body_world = ((-0.72, -0.95), (0.72, -0.95), (0.72, 0.95), (-0.72, 0.95))
        lower = [self._project(x, y, 0.0, centre_x, centre_y) for x, y in body_world]
        upper = [self._project(x, y, 0.42, centre_x, centre_y) for x, y in body_world]
        self._source(context, "#0B1A2A")
        for index in range(4):
            next_index = (index + 1) % 4
            context.move_to(*lower[index])
            context.line_to(*lower[next_index])
            context.line_to(*upper[next_index])
            context.line_to(*upper[index])
            context.close_path()
            context.fill()
        self._source(context, "#153555")
        context.move_to(*upper[0])
        for point in upper[1:]:
            context.line_to(*point)
        context.close_path()
        context.fill_preserve()
        self._source(context, body_border)
        context.set_line_width(2.0)
        context.stroke()

        self._text(
            context, "6-WHEEL", centre_x - 24, centre_y - 15,
            colour="#CDE5FF", size=7, bold=True,
        )
        controller = "정보 없음" if snapshot is None else snapshot.controller_fsm_state
        self._text(
            context, controller[:18], centre_x - 38, centre_y + 1,
            colour="#F1F6FB", size=9, bold=True,
        )
        self._text(
            context, "MODE · " + self._authority[:14], centre_x - 38, centre_y + 15,
            colour="#8EA4B8", size=7,
        )

        for name, (wheel_x, wheel_y) in ordered_wheels:
            wheel = by_name.get(name)
            colour = self._wheel_colour(wheel)
            self._source(context, colour)
            context.arc(wheel_x, wheel_y, 12, 0, math.tau)
            context.fill()
            self._source(context, "#07101B")
            context.arc(wheel_x, wheel_y, 6.5, 0, math.tau)
            context.fill()
            if name == self._selected_wheel:
                self._source(context, "#FFFFFF")
                context.set_line_width(1.5)
                context.arc(wheel_x, wheel_y, 16, 0, math.tau)
                context.stroke()
            label_x = wheel_x - 9
            self._text(
                context, self._WHEEL_LABELS[name], label_x, wheel_y + 25,
                colour=colour, size=7, bold=True,
            )

        maximum_current = _maximum_abs(tuple(
            wheel.drive_current_a for wheel in snapshot.wheel_statuses
        )) if snapshot is not None else None
        load = _number(maximum_current, " A", 1)
        selected = by_name.get(self._selected_wheel)
        selected_text = self._WHEEL_LABELS[self._selected_wheel] + " · NO DATA"
        if selected is not None:
            selected_text = (
                f"{self._WHEEL_LABELS[self._selected_wheel]} · ACT "
                f"{_number(selected.drive_turns_per_s, ' r/s', 2)} · CMD "
                f"{_number(selected.command_turns_per_s, ' r/s', 2)} · Iq "
                f"{_number(selected.drive_current_a, ' A', 1)}"
            )
        self._text(
            context, selected_text[:58], 10, height - 27,
            colour="#D9E8F5", size=7, bold=True,
        )
        self._text(
            context, f"DRAG/SCROLL: ORBIT · CLICK: SELECT · {self._yaw_deg:.0f} DEG",
            10, height - 11, colour="#71879C", size=7,
        )
        self._text(
            context, "MAX |Iq| · " + load, max(190.0, width - 100), height - 11,
            colour="#72CDB2", size=7, bold=True,
        )
        return False


class MobilityDashboard(Gtk.Box):
    """Read-only live surface plus shortcuts into the existing safe ops flow."""

    def __init__(self, action_sink: Callable[[str], None]) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=12)
        self.set_border_width(18)
        _style(self, "mobility-dashboard")
        self._action_sink = action_sink

        heading_row = Gtk.Box(spacing=16)
        _style(heading_row, "mobility-header")
        title_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        kicker = Gtk.Label(label="E2FESTA · MIXED-TERRAIN MOBILITY")
        kicker.set_xalign(0.0)
        _style(kicker, "mobility-kicker")
        title = Gtk.Label(label="6륜 협조구동")
        title.set_xalign(0.0)
        _style(title, "mobility-title")
        subtitle = Gtk.Label(
            label=("붕괴 건축물·산업 재난 접근로에서 접지력을 활용하고, "
                   "공회전은 줄이며 과부하 바퀴는 보호합니다.")
        )
        subtitle.set_xalign(0.0)
        subtitle.set_line_wrap(True)
        _style(subtitle, "mobility-subtitle")
        title_box.pack_start(kicker, False, False, 0)
        title_box.pack_start(title, False, False, 0)
        title_box.pack_start(subtitle, False, False, 0)
        heading_row.pack_start(title_box, True, True, 0)
        self._live = Gtk.Label(label="수신 대기")
        _style(self._live, "mobility-live-pill", "status-muted")
        heading_row.pack_end(self._live, False, False, 0)
        self.pack_start(heading_row, False, False, 0)

        summary = Gtk.Grid(column_spacing=10, row_spacing=10)
        summary.set_column_homogeneous(True)
        self._terrain = self._metric_card(
            "01 · 전방 지형인지", ("path", "confidence", "geometry", "quality"),
            "mobility-terrain-card",
        )
        self._attitude = self._metric_card(
            "02 · 차체 안정성", ("roll", "pitch", "yaw", "pose"),
            "mobility-attitude-card",
        )
        self._traction = self._metric_card(
            "03 · 접지·상대부하", ("contact", "slip", "load", "feedback"),
            "mobility-traction-card",
        )
        self._fsm = self._metric_card(
            "04 · 협조구동 판단", ("controller", "drive", "section", "mission"),
            "mobility-control-stage-card",
        )
        summary.attach(self._terrain[0], 0, 0, 1, 1)
        summary.attach(self._attitude[0], 1, 0, 1, 1)
        summary.attach(self._traction[0], 2, 0, 1, 1)
        summary.attach(self._fsm[0], 3, 0, 1, 1)
        self._summary_grid = summary
        self._summary_cards = (
            self._terrain[0], self._attitude[0],
            self._traction[0], self._fsm[0],
        )
        self.pack_start(summary, False, False, 0)

        evidence = Gtk.Box(spacing=14)
        _style(evidence, "mobility-evidence-bar")
        evidence_title = Gtk.Label(
            label="균일구동 ↔ 협조구동 시험 설계 · 결과 연동 전"
        )
        evidence_title.set_xalign(0.0)
        _style(evidence_title, "mobility-evidence-title")
        evidence.pack_start(evidence_title, False, False, 0)
        evidence_metrics = Gtk.Label(
            label="통과 성공률 · 통과 시간 · 슬립 지속 · 최대 Roll/Pitch · 바퀴별 Iq"
        )
        evidence_metrics.set_xalign(0.0)
        _style(evidence_metrics, "mobility-evidence-metrics")
        evidence.pack_start(evidence_metrics, True, True, 0)
        evidence_rule = Gtk.Label(label="동일 조건 · 조건별 5회 이상 · 로그 기반")
        _style(evidence_rule, "mobility-evidence-rule")
        evidence.pack_end(evidence_rule, False, False, 0)
        self.pack_start(evidence, False, False, 0)

        lower = Gtk.Grid(column_spacing=12, row_spacing=12)
        lower.set_column_homogeneous(True)
        motors = self._build_motor_card()
        faults = self._build_fault_card()
        controls = self._build_control_card()
        lower.attach(motors, 0, 0, 2, 1)
        lower.attach(faults, 2, 0, 1, 1)
        lower.attach(controls, 0, 1, 3, 1)
        self._lower_grid = lower
        self._lower_cards = (motors, faults, controls)
        self._compact_layout: bool | None = None
        self.pack_start(lower, True, True, 0)
        self.connect("size-allocate", self._on_size_allocated)

    def _on_size_allocated(
        self, _widget: Gtk.Widget, allocation: Gdk.Rectangle,
    ) -> None:
        """Reflow cards instead of squeezing their labels at narrow scales."""
        compact = allocation.width < 1120
        if compact == self._compact_layout:
            return
        self._compact_layout = compact
        for card in self._summary_cards:
            self._summary_grid.remove(card)
        for card in self._lower_cards:
            self._lower_grid.remove(card)
        if compact:
            for index, card in enumerate(self._summary_cards):
                self._summary_grid.attach(card, index % 2, index // 2, 1, 1)
            motors, faults, controls = self._lower_cards
            self._lower_grid.set_column_homogeneous(False)
            self._lower_grid.attach(motors, 0, 0, 1, 1)
            self._lower_grid.attach(faults, 0, 1, 1, 1)
            self._lower_grid.attach(controls, 0, 2, 1, 1)
        else:
            for index, card in enumerate(self._summary_cards):
                self._summary_grid.attach(card, index, 0, 1, 1)
            motors, faults, controls = self._lower_cards
            self._lower_grid.set_column_homogeneous(True)
            self._lower_grid.attach(motors, 0, 0, 2, 1)
            self._lower_grid.attach(faults, 2, 0, 1, 1)
            self._lower_grid.attach(controls, 0, 1, 3, 1)
        self._summary_grid.show_all()
        self._lower_grid.show_all()

    @staticmethod
    def _metric_card(
        title: str, keys: tuple[str, ...], accent_class: str,
    ) -> tuple[Gtk.Box, dict[str, Gtk.Label]]:
        card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
        _style(card, "system-card", "mobility-card", "mobility-stage-card", accent_class)
        heading = Gtk.Label(label=title)
        heading.set_xalign(0.0)
        _style(heading, "mobility-stage-title")
        card.pack_start(heading, False, False, 0)
        labels: dict[str, Gtk.Label] = {}
        for index, key in enumerate(keys):
            label = Gtk.Label(label="정보 없음")
            label.set_xalign(0.0)
            label.set_ellipsize(Pango.EllipsizeMode.END)
            _style(label, "mobility-value")
            if index == 0:
                _style(label, "mobility-primary-value")
            card.pack_start(label, False, False, 0)
            labels[key] = label
        return card, labels

    def _build_motor_card(self) -> Gtk.Box:
        card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        _style(card, "system-card", "mobility-card")
        heading_row = Gtk.Box(spacing=8)
        heading = Gtk.Label(label="6륜 협조 출력")
        heading.set_xalign(0.0)
        _style(heading, "mobility-panel-title")
        heading_row.pack_start(heading, True, True, 0)
        source = Gtk.Label(label="Hall 실측 · ODrive Iq")
        _style(source, "mobility-source-chip")
        heading_row.pack_end(source, False, False, 0)
        card.pack_start(heading_row, False, False, 0)
        grid = Gtk.Grid(column_spacing=12, row_spacing=5)
        _style(grid, "mobility-wheel-grid")
        headers = ("바퀴", "Hall 실측", "협조 지령", "Iq 상대부하", "조향", "상태")
        for column, text in enumerate(headers):
            label = Gtk.Label(label=text)
            label.set_xalign(0.0)
            _style(label, "detail-metric-title")
            grid.attach(label, column, 0, 1, 1)
        self._wheel_rows: list[tuple[Gtk.Label, ...]] = []
        for row in range(6):
            labels = tuple(Gtk.Label(label="—", xalign=0.0) for _ in headers)
            for column, label in enumerate(labels):
                label.set_ellipsize(Pango.EllipsizeMode.END)
                _style(label, "mobility-wheel-value")
                grid.attach(label, column, row + 1, 1, 1)
            self._wheel_rows.append(labels)
        content = Gtk.Box(spacing=14)
        grid.set_hexpand(True)
        content.pack_start(grid, True, True, 0)
        self._rover_graphic = RoverCoordinationGraphic()
        content.pack_end(self._rover_graphic, True, True, 0)
        card.pack_start(content, True, True, 0)
        self._motor_note = Gtk.Label(
            label=("Iq는 접지·끼임 판단을 위한 상대 부하 지표입니다. "
                   "절대 하중이나 소비에너지로 해석하지 않습니다.")
        )
        self._motor_note.set_xalign(0.0)
        self._motor_note.set_line_wrap(True)
        _style(self._motor_note, "muted")
        card.pack_end(self._motor_note, False, False, 0)
        return card

    def _build_fault_card(self) -> Gtk.Box:
        card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=7)
        _style(card, "system-card", "mobility-card")
        heading = Gtk.Label(label="운용 판단 · 안전복구")
        heading.set_xalign(0.0)
        _style(heading, "mobility-panel-title")
        self._fault_summary = Gtk.Label(label="수신 대기")
        self._fault_summary.set_xalign(0.0)
        _style(self._fault_summary, "mobility-decision", "status-muted")
        self._recovery_hint = Gtk.Label(
            label="지속 슬립·과부하 시 정지 → 후진 → 수동 복귀"
        )
        self._recovery_hint.set_xalign(0.0)
        self._recovery_hint.set_line_wrap(True)
        _style(self._recovery_hint, "mobility-recovery-hint")
        self._faults = Gtk.Label(label="차대 텔레메트리 미수신")
        self._faults.set_xalign(0.0)
        self._faults.set_yalign(0.0)
        self._faults.set_line_wrap(True)
        self._faults.set_selectable(True)
        _style(self._faults, "mobility-faults")
        card.pack_start(heading, False, False, 0)
        card.pack_start(self._fault_summary, False, False, 0)
        card.pack_start(self._recovery_hint, False, False, 0)
        card.pack_start(self._faults, True, True, 0)
        return card

    def _build_control_card(self) -> Gtk.Box:
        card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        _style(card, "system-card", "mobility-card", "mobility-control-card")
        top = Gtk.Box(spacing=10)
        title = Gtk.Label(label="시연 조작")
        title.set_xalign(0.0)
        _style(title, "mobility-panel-title")
        self._authority = Gtk.Label(label="권한 · 정보 없음")
        _style(self._authority, "role-sub")
        top.pack_start(title, True, True, 0)
        top.pack_end(self._authority, False, False, 0)
        card.pack_start(top, False, False, 0)
        buttons = Gtk.Box(spacing=8)
        for action, text in (
            ("authority_manual", "수동 운용"),
            ("authority_auto", "협조구동"),
            ("authority_idle", "안전 대기"),
            ("steer_mode_skid", "조향 모드"),
            ("__settings__", "복구 · 설정"),
        ):
            button = Gtk.Button(label=text)
            button.set_hexpand(True)
            button.connect(
                "clicked", lambda _button, selected=action: self._action_sink(selected)
            )
            _style(button, "mobility-control-button")
            buttons.pack_start(button, True, True, 0)
        card.pack_start(buttons, False, False, 0)
        note = Gtk.Label(
            label=("속도·방향은 승인된 DualSense 경로만 사용합니다. "
                   "모든 모드 전환은 토큰 인증·상태 재검증·확인을 거칩니다.")
        )
        note.set_xalign(0.0)
        note.set_line_wrap(True)
        _style(note, "muted")
        card.pack_start(note, False, False, 0)
        return card

    @staticmethod
    def _set_tone(label: Gtk.Label, tone: str) -> None:
        context = label.get_style_context()
        for candidate in ("status-live", "status-warn", "status-bad", "status-muted"):
            context.remove_class(candidate)
        context.add_class(tone)

    def update(
        self,
        snapshot: TelemetrySnapshot | None,
        *,
        ops_state: Mapping[str, object] | None,
        now_s: float | None = None,
    ) -> None:
        now_s = time.monotonic() if now_s is None else float(now_s)
        fresh = snapshot is not None and now_s - snapshot.received_monotonic_s <= 1.0
        self._live.set_text("실시간" if fresh else "갱신 지연" if snapshot else "수신 대기")
        self._set_tone(self._live, "status-live" if fresh else "status-warn" if snapshot else "status-muted")
        if not fresh or snapshot is None:
            for labels in (
                self._terrain[1], self._attitude[1], self._traction[1], self._fsm[1],
            ):
                for label in labels.values():
                    label.set_text("정보 없음")
            for row in self._wheel_rows:
                for label in row:
                    label.set_text("—")
            faults = (
                active_faults(None) if snapshot is None
                else ("차대 텔레메트리 갱신 지연",)
            )
            self._fault_summary.set_text("운용 보류 · 최신 상태 확인 필요")
            self._set_tone(self._fault_summary, "status-warn")
            self._faults.set_text("\n".join(f"• {fault}" for fault in faults))
            self._rover_graphic.set_state(None, ops_state)
            self._update_authority(ops_state)
            return

        terrain = self._terrain[1]
        path_text = (
            "주행 경로 확보" if snapshot.terrain_path_available is True else
            "경로 차단" if snapshot.terrain_path_available is False else "정보 없음"
        )
        terrain["path"].set_text("경로 · " + path_text)
        self._set_tone(
            terrain["path"],
            "status-live" if snapshot.terrain_path_available is True
            else "status-bad" if snapshot.terrain_path_available is False
            else "status-muted",
        )
        confidence = snapshot.terrain_confidence
        terrain["confidence"].set_text(
            "신뢰도 · " + ("정보 없음" if confidence is None else f"{confidence * 100:.0f}%")
        )
        terrain["geometry"].set_text(
            "오프셋 {} · 방향 {}".format(
                _number(snapshot.terrain_path_offset_m, " m", 2),
                _degrees(snapshot.terrain_heading_error_rad),
            )
        )
        terrain["quality"].set_text(
            "경사 {} · 뱅크 {} · 거칠기 {}".format(
                _degrees(snapshot.terrain_slope_rad),
                _degrees(snapshot.terrain_bank_rad),
                _number(snapshot.terrain_roughness_m, " m", 3),
            )
        )

        attitude = self._attitude[1]
        attitude["roll"].set_text("Roll · " + _degrees(snapshot.roll_rad))
        attitude["pitch"].set_text("Pitch · " + _degrees(snapshot.pitch_rad))
        attitude["yaw"].set_text("Yaw · " + _degrees(snapshot.yaw_rad))
        attitude["pose"].set_text(
            "위치 · x {} · y {}".format(
                _number(snapshot.x_m, " m", 2), _number(snapshot.y_m, " m", 2),
            )
        )

        traction = self._traction[1]
        healthy_wheels = sum(
            1 for wheel in snapshot.wheel_statuses if wheel_health_text(wheel) == "정상"
        )
        feedback_text = (
            f"{healthy_wheels}/{len(snapshot.wheel_statuses)} 정상"
            if snapshot.wheel_statuses else "정보 없음"
        )
        maximum_current = _maximum_abs(tuple(
            wheel.drive_current_a for wheel in snapshot.wheel_statuses
        ))
        if snapshot.stuck_candidate is True:
            contact_text = "끼임 후보"
        else:
            contact_text = _candidate_text(
                snapshot.slip_candidate, detected="슬립 후보",
            )
        traction["contact"].set_text("접지 판단 · " + contact_text)
        traction["slip"].set_text(
            "슬립 · " + _candidate_text(snapshot.slip_candidate, detected="후보 감지")
        )
        traction["load"].set_text(
            "최대 |Iq| · " + _number(maximum_current, " A", 1)
        )
        traction["feedback"].set_text("바퀴 피드백 · " + feedback_text)
        self._set_tone(
            traction["contact"],
            "status-bad" if snapshot.stuck_candidate is True
            else "status-warn" if snapshot.slip_candidate is True
            else "status-live" if snapshot.slip_candidate is False
            else "status-muted",
        )

        fsm = self._fsm[1]
        fsm["drive"].set_text("차대 · " + snapshot.drive_state)
        controller_reasons = ", ".join(snapshot.controller_fsm_reasons)
        fsm["controller"].set_text(
            "자율 · " + snapshot.controller_fsm_state
            + (f" · {controller_reasons}" if controller_reasons else "")
        )
        fsm["mission"].set_text(
            "임무 · " + snapshot.mission_fsm_state
            + (f" · {snapshot.mission_fsm_reason}" if snapshot.mission_fsm_reason else "")
        )
        fsm["section"].set_text(
            f"구간 · {snapshot.section_fsm_section} / {snapshot.section_fsm_phase}"
        )
        maximum_tilt = _maximum_abs((snapshot.roll_rad, snapshot.pitch_rad))
        if maximum_tilt is not None:
            attitude["pose"].set_text(
                "최대 기울기 · " + _degrees(maximum_tilt)
                + " · 위치 x {} / y {}".format(
                    _number(snapshot.x_m, " m", 2), _number(snapshot.y_m, " m", 2),
                )
            )

        for index, row in enumerate(self._wheel_rows):
            if index >= len(snapshot.wheel_statuses):
                for label in row:
                    label.set_text("—")
                continue
            wheel = snapshot.wheel_statuses[index]
            values = (
                wheel.name,
                _number(wheel.drive_turns_per_s, " r/s", 2),
                _number(wheel.command_turns_per_s, " r/s", 2),
                _number(wheel.drive_current_a, " A", 1),
                _number(wheel.steer_deg, "°", 1),
                wheel_health_text(wheel),
            )
            for label, value in zip(row, values, strict=True):
                label.set_text(value)
            self._set_tone(
                row[-1],
                "status-live" if wheel_health_text(wheel) == "정상" else "status-bad",
            )

        faults = active_faults(snapshot)
        self._fault_summary.set_text(
            "주행 지속 가능 · 활성 오류 없음"
            if not faults else f"안전 확인 필요 · {len(faults)}건"
        )
        self._set_tone(
            self._fault_summary, "status-live" if not faults else "status-warn",
        )
        self._faults.set_text(
            "활성 오류가 없습니다" if not faults
            else "\n".join(f"• {fault}" for fault in faults)
        )
        self._rover_graphic.set_state(snapshot, ops_state)
        self._update_authority(ops_state)

    def _update_authority(self, ops_state: Mapping[str, object] | None) -> None:
        authority = "정보 없음" if ops_state is None else str(
            ops_state.get("authority_mode", "정보 없음")
        )
        self._authority.set_text("권한 · " + authority)
