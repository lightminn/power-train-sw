"""E2FESTA mobility view for terrain-aware six-wheel coordination."""
from __future__ import annotations

import math
import time
from pathlib import Path
from collections.abc import Callable, Mapping

import gi

gi.require_version("Gtk", "3.0")
gi.require_version("Gdk", "3.0")
from gi.repository import Gdk, Gtk, Pango  # noqa: E402

from .telemetry import TelemetrySnapshot, WheelStatus
from .rover_3d import Rover3DView
from .themes import theme_tokens
from .obstacle_telemetry import (
    ObstacleTelemetrySnapshot,
    obstacle_source_state,
)


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
    wheel_names = {
        "front_left": "좌측 전륜", "front_right": "우측 전륜",
        "mid_left": "좌측 중륜", "mid_right": "우측 중륜",
        "rear_left": "좌측 후륜", "rear_right": "우측 후륜",
    }
    for wheel in snapshot.wheel_statuses:
        details = []
        if wheel.stale:
            details.append("통신 지연")
        if wheel.drive_axis_error:
            details.append("구동축 오류")
        if wheel.steer_fault:
            details.append("조향 오류")
        if details:
            faults.append(
                f"{wheel_names.get(wheel.name, wheel.name)} · " + " · ".join(details)
            )
    reason_names = {
        "low_confidence": "지형 판단 신뢰도 부족",
        "forward obstacle": "전방 장애물",
        "no_path": "통과 경로 없음",
    }
    faults.extend(
        reason_names.get(reason, f"지형 판단 보류 · {reason}")
        for reason in snapshot.terrain_reject_reasons
    )
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
    """Return judge-facing conclusions for the main mission rail.

    The mission surface should answer whether the rover can keep moving and
    what the controller is doing.  Engineering measurements remain available
    on the cooperative-drive page instead of being repeated here.
    """
    unavailable = {
        "path": "정보 없음",
        "control": "정보 없음",
        "attitude": "정보 없음",
        "motors": "정보 없음",
    }
    if snapshot is None:
        return unavailable

    confidence = "" if snapshot.terrain_confidence is None else (
        f" · 판단 신뢰도 {snapshot.terrain_confidence * 100:.0f}%"
    )
    path = (
        "통과 가능" + confidence
        if snapshot.terrain_path_available is True else
        "차단" + confidence
        if snapshot.terrain_path_available is False else
        "지형 판단 대기"
    )
    scale = (
        "" if snapshot.degradation_speed_scale is None
        else f" · 속도 {snapshot.degradation_speed_scale * 100:.0f}%"
    )
    controller_copy = {
        "TRACKING": "경로 추종 중",
        "SPEED_LIMIT": "험지 진입 감속",
        "TRACTION_ASSIST": "접지력 보조 중",
        "SAFE_HOLD": "안전 정지 유지",
    }.get(snapshot.controller_fsm_state, "제어 판단 대기")
    control = controller_copy + scale
    maximum_tilt = _maximum_abs((snapshot.roll_rad, snapshot.pitch_rad))
    attitude = (
        "차체 자세 정보 없음" if maximum_tilt is None
        else f"차체 기울기 {_degrees(maximum_tilt)}"
    )
    healthy_wheels = sum(
        wheel_health_text(wheel) == "정상" for wheel in snapshot.wheel_statuses
    )
    wheel_count = len(snapshot.wheel_statuses)
    contact = (
        " · 끼임 후보" if snapshot.stuck_candidate is True else
        " · 슬립 후보" if snapshot.slip_candidate is True else ""
    )
    motors = "정보 없음"
    if wheel_count:
        motors = f"{healthy_wheels}/{wheel_count} 정상{contact}"
    return {
        "path": path,
        "control": control,
        "attitude": attitude,
        "motors": motors,
    }


def mobility_story_values(
    snapshot: TelemetrySnapshot | None,
) -> dict[str, str]:
    """Translate telemetry into one bounded situation → response narrative."""
    if snapshot is None:
        return {
            "title": "주행 판단 대기",
            "detail": "지형 · 접지 · 구동 상태 수신 중",
            "action": "대응  |  협조구동 대기",
            "tone": "status-muted",
            "terrain": "지형 정보 수신 대기",
            "judgement": "접지 상태 판단 대기",
            "control": "제어 대응 대기",
            "result": "주행 결과 확인 대기",
        }

    wheels = snapshot.wheel_statuses
    healthy_wheels = sum(wheel_health_text(wheel) == "정상" for wheel in wheels)
    wheel_count = len(wheels)
    scale = snapshot.degradation_speed_scale
    scale_text = ""
    if scale is not None:
        scale_text = f" · 기준 속도의 {scale * 100:.0f}%"
    confidence = snapshot.terrain_confidence
    confidence_text = "" if confidence is None else f" · 신뢰도 {confidence * 100:.0f}%"
    maximum_tilt = _maximum_abs((snapshot.roll_rad, snapshot.pitch_rad))
    tilt_text = "기울기 정보 없음" if maximum_tilt is None else (
        f"최대 기울기 {_degrees(maximum_tilt)}"
    )

    terrain = (
        "통과 가능한 경로" + confidence_text
        if snapshot.terrain_path_available is True else
        "전방 경로가 차단됨" + confidence_text
        if snapshot.terrain_path_available is False else
        "경로 판단 대기"
    )
    wheel_health = (
        "바퀴 정보 없음" if not wheel_count
        else f"바퀴 {healthy_wheels}/{wheel_count} 정상"
    )

    if snapshot.stuck_candidate is True or snapshot.terrain_path_available is False:
        return {
            "title": "전방 경로 차단 · 안전 정지",
            "detail": "원인  |  장애물 · 끼임 · 과부하 후보",
            "action": "대응  |  출력 차단 · 후진 대기",
            "tone": "status-bad",
            "terrain": terrain,
            "judgement": "끼임·과부하 가능성",
            "control": "출력 차단 · 안전 정지",
            "result": "로봇 보호 상태 유지",
        }
    if snapshot.slip_candidate is True:
        return {
            "title": "바퀴 슬립 감지 · 접지력 보강",
            "detail": "판단  |  공회전 출력 감소 · 접지 바퀴 활용",
            "action": "대응  |  6륜 출력 재분배" + scale_text,
            "tone": "status-warn",
            "terrain": terrain,
            "judgement": "슬립·접지 저하 후보",
            "control": "6륜 출력 재분배" + scale_text,
            "result": "저속 통과 시도 중",
        }
    if scale is not None and scale < 0.999:
        return {
            "title": "험지 접근 · 선제 감속",
            "detail": f"판단  |  경사 구간 · {tilt_text}",
            "action": "대응  |  속도 제한" + scale_text,
            "tone": "status-warn",
            "terrain": terrain,
            "judgement": f"험지 접근 · {tilt_text}",
            "control": "선제 감속" + scale_text,
            "result": "안정 자세로 접근 중",
        }
    if wheel_count and healthy_wheels < wheel_count:
        return {
            "title": f"바퀴 {wheel_count - healthy_wheels}개 점검 필요",
            "detail": "원인  |  통신 지연 또는 구동 오류",
            "action": "대응  |  저속 유지 · 안전 확인",
            "tone": "status-warn",
            "terrain": terrain,
            "judgement": wheel_health,
            "control": "주행 제한 · 점검 요청",
            "result": "조건부 주행 가능",
        }
    return {
        "title": "경로 확보 · 접지 안정",
        "detail": "판단  |  6륜 피드백 정상",
        "action": "대응  |  경로 추종 · 균형 구동" + scale_text,
        "tone": "status-live",
        "terrain": terrain,
        "judgement": "접지 이상 없음",
        "control": "6륜 균형 구동",
        "result": "주행 지속 가능",
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
        context.select_font_face("눈누 기초고딕 Regular", 0, 1 if bold else 0)
        context.set_font_size(size)
        context.move_to(x, y)
        context.show_text(text)

    def _wheel_colour(self, wheel: WheelStatus | None) -> str:
        palette = theme_tokens()
        if wheel is None:
            return palette["offline"]
        if wheel.stale:
            return palette["warning"]
        if wheel.drive_axis_error or wheel.steer_fault or wheel.mode == "FAULT":
            return palette["danger"]
        return palette["success"]

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
        palette = theme_tokens()

        path_colour = palette["offline"]
        path_label = "PATH CHECK"
        if snapshot is not None and snapshot.terrain_path_available is True:
            path_colour, path_label = palette["ai_cyan"], "PATH OPEN"
        elif snapshot is not None and snapshot.terrain_path_available is False:
            path_colour, path_label = palette["danger"], "PATH BLOCKED"

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
        self._source(context, palette["border_strong"], 0.9)
        context.set_line_width(3.0)
        for name, (wheel_x, wheel_y) in ordered_wheels:
            side, _row = self._WHEEL_POSITIONS[name]
            anchor_x, anchor_y = self._project(
                side * 0.58, 0.0, 0.0, centre_x, centre_y,
            )
            context.move_to(anchor_x, anchor_y)
            context.line_to(wheel_x, wheel_y)
            context.stroke()

        body_border = palette["primary"]
        if snapshot is not None and snapshot.stuck_candidate is True:
            body_border = palette["danger"]
        elif snapshot is not None and snapshot.slip_candidate is True:
            body_border = palette["warning"]
        body_world = ((-0.72, -0.95), (0.72, -0.95), (0.72, 0.95), (-0.72, 0.95))
        lower = [self._project(x, y, 0.0, centre_x, centre_y) for x, y in body_world]
        upper = [self._project(x, y, 0.42, centre_x, centre_y) for x, y in body_world]
        self._source(context, palette["video_stage"])
        for index in range(4):
            next_index = (index + 1) % 4
            context.move_to(*lower[index])
            context.line_to(*lower[next_index])
            context.line_to(*upper[next_index])
            context.line_to(*upper[index])
            context.close_path()
            context.fill()
        self._source(context, palette["surface_raised"])
        context.move_to(*upper[0])
        for point in upper[1:]:
            context.line_to(*point)
        context.close_path()
        context.fill_preserve()
        self._source(context, body_border)
        context.set_line_width(2.0)
        context.stroke()

        self._text(
            context, "6-WHEEL COORD", centre_x - 31, centre_y - 15,
            colour=palette["ai_cyan"], size=7, bold=True,
        )
        controller = "WAITING" if snapshot is None else {
            "TRACKING": "TRACKING",
            "SPEED_LIMIT": "SLOW APPROACH",
            "TRACTION_ASSIST": "TRACTION ASSIST",
            "SAFE_HOLD": "SAFE HOLD",
        }.get(snapshot.controller_fsm_state, "CHECK STATE")
        self._text(
            context, controller[:18], centre_x - 38, centre_y + 1,
            colour=palette["text_main"], size=9, bold=True,
        )
        self._text(
            context, "AUTH · " + self._authority[:14], centre_x - 38, centre_y + 15,
            colour=palette["text_secondary"], size=7,
        )

        for name, (wheel_x, wheel_y) in ordered_wheels:
            wheel = by_name.get(name)
            colour = self._wheel_colour(wheel)
            self._source(context, colour)
            context.arc(wheel_x, wheel_y, 12, 0, math.tau)
            context.fill()
            self._source(context, palette["app_bg"])
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
            health = {
                "정상": "OK", "통신 지연": "LINK DELAY", "오류": "FAULT",
            }.get(wheel_health_text(selected), "CHECK")
            selected_text = (
                f"WHEEL {self._WHEEL_LABELS[self._selected_wheel]} · {health}"
            )
        self._text(
            context, selected_text[:58], 10, height - 27,
            colour=palette["text_main"], size=7, bold=True,
        )
        self._text(
            context, f"DRAG: ORBIT · CLICK: SELECT · {self._yaw_deg:.0f} DEG",
            10, height - 11, colour=palette["text_muted"], size=7,
        )
        self._text(
            context, "PEAK LOAD · " + load, max(190.0, width - 100), height - 11,
            colour=palette["ai_cyan"], size=7, bold=True,
        )
        return False


class MobilityDashboard(Gtk.Box):
    """Read-only live surface plus shortcuts into the existing safe ops flow."""

    def __init__(self, action_sink: Callable[[str], None], *, input_source="LIVE") -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        self.set_border_width(18)
        _style(self, "mobility-dashboard")
        self._action_sink = action_sink
        self._input_source = input_source
        provider = Gtk.CssProvider()
        provider.load_from_path(str(Path(__file__).parent / "assets" / "mobility-fit.css"))
        Gtk.StyleContext.add_provider_for_screen(
            Gdk.Screen.get_default(), provider, Gtk.STYLE_PROVIDER_PRIORITY_APPLICATION + 2)

        heading_row = Gtk.Box(spacing=16)
        _style(heading_row, "mobility-header")
        title_box = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        kicker = Gtk.Label(label="E2FESTA · MIXED-TERRAIN MOBILITY")
        kicker.set_xalign(0.0)
        _style(kicker, "mobility-kicker")
        kicker.set_no_show_all(True)
        title = Gtk.Label(label="6륜 협조구동")
        title.set_xalign(0.0)
        _style(title, "mobility-title")
        subtitle = Gtk.Label(
            label="재난 현장 선행 탐사  |  지형 예측 · 접지 판단 · 바퀴별 협조제어"
        )
        subtitle.set_xalign(0.0)
        subtitle.set_line_wrap(True)
        _style(subtitle, "mobility-subtitle")
        subtitle.set_no_show_all(True)
        title_box.pack_start(kicker, False, False, 0)
        title_box.pack_start(title, False, False, 0)
        title_box.pack_start(subtitle, False, False, 0)
        heading_row.pack_start(title_box, True, True, 0)
        self._live = Gtk.Label(label="수신 대기")
        self._live.set_valign(Gtk.Align.CENTER)
        _style(self._live, "mobility-live-pill", "status-muted")
        heading_row.pack_end(self._live, False, False, 0)
        self.pack_start(heading_row, False, False, 0)

        story = Gtk.Box(spacing=18)
        _style(story, "mobility-story")
        story_copy = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        story_kicker = Gtk.Label(label="현재 로봇 판단")
        story_kicker.set_xalign(0.0)
        _style(story_kicker, "mobility-story-kicker")
        story_kicker.set_no_show_all(True)
        self._story_title = Gtk.Label(label="주행 판단 대기")
        self._story_title.set_xalign(0.0)
        self._story_title.set_line_wrap(True)
        _style(self._story_title, "mobility-story-title", "status-muted")
        self._story_detail = Gtk.Label(
            label="지형 · 접지 · 구동 상태 수신 중"
        )
        self._story_detail.set_xalign(0.0)
        self._story_detail.set_line_wrap(True)
        _style(self._story_detail, "mobility-story-detail")
        self._story_detail.set_no_show_all(True)
        story_copy.pack_start(story_kicker, False, False, 0)
        story_copy.pack_start(self._story_title, False, False, 0)
        story_copy.pack_start(self._story_detail, False, False, 0)
        self._story_action = Gtk.Label(label="대응  |  협조구동 대기")
        self._story_action.set_xalign(0.5)
        self._story_action.set_line_wrap(True)
        self._story_action.set_size_request(310, -1)
        _style(self._story_action, "mobility-story-action", "status-muted")
        story.pack_start(story_copy, True, True, 0)
        story.pack_end(self._story_action, False, False, 0)
        self.pack_start(story, False, False, 0)

        summary = Gtk.Grid(column_spacing=10, row_spacing=10)
        summary.set_column_homogeneous(True)
        self._terrain = self._metric_card(
            "01 · 앞의 지형", ("path", "confidence", "geometry", "quality"),
            "mobility-terrain-card",
        )
        self._attitude = self._metric_card(
            "02 · 주행 위험 판단", ("roll", "pitch", "yaw", "pose"),
            "mobility-attitude-card",
        )
        self._traction = self._metric_card(
            "03 · 바퀴 제어 대응", ("contact", "slip", "load", "feedback"),
            "mobility-traction-card",
        )
        self._fsm = self._metric_card(
            "04 · 현재 주행 결과", ("controller", "drive", "section", "mission"),
            "mobility-control-stage-card",
        )
        for group, hidden_keys in (
            (self._terrain[1], ("geometry", "quality")),
            (self._attitude[1], ("yaw", "pose")),
            (self._traction[1], ("load", "feedback")),
            (self._fsm[1], ("section", "mission")),
        ):
            for key in hidden_keys:
                group[key].set_no_show_all(True)
                group[key].hide()
        summary.attach(self._terrain[0], 0, 0, 1, 1)
        summary.attach(self._attitude[0], 1, 0, 1, 1)
        summary.attach(self._traction[0], 0, 1, 1, 1)
        summary.attach(self._fsm[0], 1, 1, 1, 1)
        self._summary_grid = summary
        self._summary_cards = (
            self._terrain[0], self._attitude[0],
            self._traction[0], self._fsm[0],
        )

        evidence = Gtk.Box(spacing=14)
        _style(evidence, "mobility-evidence-bar")
        evidence_title = Gtk.Label(
            label="정량 검증 · 결과 데이터 연결 전"
        )
        evidence_title.set_xalign(0.0)
        _style(evidence_title, "mobility-evidence-title")
        evidence.pack_start(evidence_title, False, False, 0)
        evidence_metrics = Gtk.Label(
            label="비교 기준  |  장애물 통과율 · 슬립 지속시간 · 조건별 5회 이상"
        )
        evidence_metrics.set_xalign(0.0)
        _style(evidence_metrics, "mobility-evidence-metrics")
        evidence.pack_start(evidence_metrics, True, True, 0)
        evidence_expander = Gtk.Expander(label="정량 검증 · 결과 데이터 연결 전")
        evidence_expander.set_no_show_all(True)
        evidence_expander.add(evidence)
        self.pack_start(evidence_expander, False, False, 0)

        lower = Gtk.Grid(column_spacing=12, row_spacing=12)
        lower.set_column_homogeneous(True)
        motors = self._build_motor_card()
        faults = self._build_fault_card()
        controls = self._build_control_card()
        status_column = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=10)
        status_column.pack_start(summary, False, False, 0)
        status_column.pack_start(faults, True, True, 0)
        lower.attach(motors, 0, 0, 2, 1)
        lower.attach(status_column, 2, 0, 1, 1)
        lower.attach(controls, 0, 1, 3, 1)
        self._lower_grid = lower
        self._lower_cards = (motors, status_column, controls)
        self._compact_layout: bool | None = None
        self.pack_start(lower, True, True, 0)
        self.connect("size-allocate", self._on_size_allocated)

    def _on_size_allocated(
        self, _widget: Gtk.Widget, allocation: Gdk.Rectangle,
    ) -> None:
        """Reflow cards instead of squeezing their labels at narrow scales."""
        compact = allocation.width < 920
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
                self._summary_grid.attach(card, index % 2, index // 2, 1, 1)
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
        card = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=3)
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
        heading = Gtk.Label(label="로봇 자세 · 6륜 제어 상태")
        heading.set_xalign(0.0)
        _style(heading, "mobility-panel-title")
        heading_row.pack_start(heading, True, True, 0)
        source = Gtk.Label(label="센서 → 판단 → 제어")
        source.set_no_show_all(True)
        _style(source, "mobility-source-chip")
        heading_row.pack_end(source, False, False, 0)
        card.pack_start(heading_row, False, False, 0)

        content = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        story = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=8)
        story.set_hexpand(True)
        story.set_no_show_all(True)
        self._coordination_summary = Gtk.Label(label="제어 대기")
        self._coordination_summary.set_xalign(0.0)
        self._coordination_summary.set_line_wrap(True)
        _style(self._coordination_summary, "mobility-coordination-summary")
        self._coordination_detail = Gtk.Label(
            label="입력  |  휠 속도 · Iq · 접지 상태"
        )
        self._coordination_detail.set_xalign(0.0)
        self._coordination_detail.set_line_wrap(True)
        self._coordination_detail.set_no_show_all(True)
        _style(self._coordination_detail, "mobility-coordination-detail")
        story.pack_start(self._coordination_summary, False, False, 0)
        story.pack_start(self._coordination_detail, False, False, 0)
        self._rover_graphic = Rover3DView(input_source=self._input_source)
        self._motor_card = card
        content.pack_start(story, False, False, 0)
        content.pack_end(self._rover_graphic, True, True, 0)
        card.pack_start(content, True, True, 0)

        technical = Gtk.Expander(label="바퀴별 기술 데이터 보기")
        technical.set_no_show_all(True)
        technical.set_expanded(False)
        technical_body = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        technical_body.set_border_width(8)
        grid = Gtk.Grid(column_spacing=12, row_spacing=5)
        _style(grid, "mobility-wheel-grid")
        headers = ("위치", "실제 회전", "제어 목표", "상대부하", "조향", "진단")
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
        grid.set_hexpand(True)
        technical_body.pack_start(grid, False, False, 0)
        self._motor_note = Gtk.Label(
            label=("상대부하는 ODrive Iq를 이용한 접지·끼임 판단 보조지표입니다. "
                   "절대 하중이나 소비에너지로 해석하지 않습니다.")
        )
        self._motor_note.set_xalign(0.0)
        self._motor_note.set_line_wrap(True)
        _style(self._motor_note, "muted")
        technical_body.pack_end(self._motor_note, False, False, 0)
        technical.add(technical_body)
        _style(technical, "mobility-technical-expander")
        card.pack_end(technical, False, False, 0)
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
            label="이상이 있으면 정지하고 연결 상태를 확인하세요"
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
        card = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=10)
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
            ("__terrain_preview__", "지형 보기"),
        ):
            button = Gtk.Button(label=text)
            button.set_hexpand(True)
            button.connect(
                "clicked", lambda _button, selected=action: self._action_sink(selected)
            )
            _style(button, "mobility-control-button")
            buttons.pack_start(button, True, True, 0)
        card.pack_start(buttons, True, True, 0)
        note = Gtk.Label(
            label=("속도·방향은 승인된 DualSense 경로만 사용합니다. "
                   "모든 모드 전환은 토큰 인증·상태 재검증·확인을 거칩니다.")
        )
        note.set_xalign(0.0)
        note.set_line_wrap(True)
        _style(note, "muted")
        card.set_tooltip_text(note.get_text())
        return card

    @staticmethod
    def _set_tone(label: Gtk.Label, tone: str) -> None:
        context = label.get_style_context()
        for candidate in ("status-live", "status-warn", "status-bad", "status-muted"):
            context.remove_class(candidate)
        context.add_class(tone)

    def _show_story(self, snapshot: TelemetrySnapshot | None) -> dict[str, str]:
        story = mobility_story_values(snapshot)
        self._story_title.set_text(story["title"])
        self._story_detail.set_text(story["detail"])
        self._story_action.set_text(story["action"])
        self._set_tone(self._story_title, story["tone"])
        self._set_tone(self._story_action, story["tone"])
        self._coordination_summary.set_text(story["control"])
        self._coordination_detail.set_text(
            "입력  |  휠 속도 · Iq · 접지 상태"
        )
        return story

    def update(
        self,
        snapshot: TelemetrySnapshot | None,
        *,
        ops_state: Mapping[str, object] | None,
        obstacle: ObstacleTelemetrySnapshot | None = None,
        now_s: float | None = None,
    ) -> None:
        now_s = time.monotonic() if now_s is None else float(now_s)
        fresh = snapshot is not None and now_s - snapshot.received_monotonic_s <= 1.0
        self._live.set_text("실시간" if fresh else "갱신 지연" if snapshot else "수신 대기")
        self._set_tone(self._live, "status-live" if fresh else "status-warn" if snapshot else "status-muted")
        if not fresh or snapshot is None:
            self._show_story(None)
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

        story = self._show_story(snapshot)
        terrain = self._terrain[1]
        obstacle_state = obstacle_source_state(obstacle, now_s=now_s)
        if obstacle_state == "STALE":
            terrain["path"].set_text("장애물 인지 갱신 지연")
            terrain["confidence"].set_text("마지막 결과를 현재 값으로 표시하지 않음")
            terrain["geometry"].set_text("좌 · 정보 없음  |  중앙 · 정보 없음  |  우 · 정보 없음")
            terrain["quality"].set_text("새 유효 프레임 수신 대기")
            self._set_tone(terrain["path"], "status-warn")
        elif obstacle is not None and obstacle.state != "OK":
            title = (
                "바닥 인지 불가 · 주의"
                if obstacle.state == "NO_GROUND" else "L515 인지 오류 · 주의"
            )
            terrain["path"].set_text(title)
            terrain["confidence"].set_text(
                f"순번 {obstacle.sequence} · "
                + (f"처리 {obstacle.processing_ms:.0f} ms" if obstacle.processing_ms is not None else "처리시간 정보 없음")
            )
            terrain["geometry"].set_text("좌 · 인지 불가  |  중앙 · 인지 불가  |  우 · 인지 불가")
            terrain["quality"].set_text(obstacle.detail or "장애물 없음으로 해석하지 않습니다")
            self._set_tone(terrain["path"], "status-bad")
        elif obstacle is not None:
            def zone_text(name: str) -> str:
                zone = obstacle.zones[name]
                if not zone.valid:
                    return "인지 불가"
                if zone.detected is False:
                    return "관측 범위 내 미검출"
                return "거리 정보 없음" if zone.distance_m is None else f"{zone.distance_m:.2f} m"

            nearest = min(
                (zone.distance_m for zone in obstacle.zones.values()
                 if zone.valid and zone.detected is True and zone.distance_m is not None),
                default=None,
            )
            terrain["path"].set_text(
                "장애물 인지 정상" if nearest is None else f"최근접 장애물 {nearest:.2f} m"
            )
            terrain["confidence"].set_text(
                f"L515 실시간 · 순번 {obstacle.sequence} · "
                + (f"처리 {obstacle.processing_ms:.0f} ms" if obstacle.processing_ms is not None else "처리시간 정보 없음")
            )
            terrain["geometry"].set_text(
                f"좌 · {zone_text('left')}  |  중앙 · {zone_text('center')}  |  우 · {zone_text('right')}"
            )
            terrain["quality"].set_text("관측 불가 구역과 미검출을 구분해 표시")
            self._set_tone(terrain["path"], "status-live" if nearest is None else "status-warn")
        else:
            terrain["path"].set_text(story["terrain"])
            self._set_tone(
                terrain["path"],
                "status-live" if snapshot.terrain_path_available is True
                else "status-bad" if snapshot.terrain_path_available is False
                else "status-muted",
            )
            confidence = snapshot.terrain_confidence
            terrain["confidence"].set_text(
                f"L515  |  단차 · 경사 · 장애물  |  신뢰도 {confidence * 100:.0f}%"
                if confidence is not None else "지형 정보 대기"
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
        attitude["roll"].set_text(story["judgement"])
        attitude["pitch"].set_text(
            "차체 최대 기울기 · "
            + _degrees(_maximum_abs((snapshot.roll_rad, snapshot.pitch_rad)))
        )
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
        traction["contact"].set_text(story["control"])
        traction["slip"].set_text(
            f"바퀴 {feedback_text}"
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
        fsm["controller"].set_text(story["result"])
        fsm["drive"].set_text(
            "실시간 상태 · " + (
                "안전 정지" if snapshot.stuck_candidate is True else
                "저속 통과 중" if snapshot.slip_candidate is True or (
                    snapshot.degradation_speed_scale is not None
                    and snapshot.degradation_speed_scale < 0.999
                ) else "주행 유지"
            )
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
