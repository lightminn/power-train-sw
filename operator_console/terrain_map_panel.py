"""The "지형 지도" console page (spec §4.10): GTK drawing around ``map_rgba``.

Operator visual aid only — the panel sends nothing to the robot.  Text goes
through PangoCairo (cairo's toy text API has no Hangul fallback).
"""
from __future__ import annotations

from collections import deque
import math
import time

import cairo
import gi
import numpy as np

gi.require_version("Gtk", "3.0")
gi.require_version("PangoCairo", "1.0")
from gi.repository import Gdk, GLib, Gtk, Pango, PangoCairo  # noqa: E402

from .terrain_map import DisplayConfig, LatestMapReceiver, MapRenderer, MapSnapshot, MapView  # noqa: E402

LINK_LOST_S = 0.5
TILT_MAX_AGE_S = 1.0          # same as the producer's slope_max_age_s default
REDRAW_MS = 200
DEFAULT_M_PER_PX = 0.01
ZOOM_RANGE = (0.002, 0.08)
FIXED_NOTE = "지도는 지나온 관측 기록 · 분류는 그 칸을 본 위치 기준 · 오래된 칸은 흐리게 · 실시간 장애물 감지가 아님"
LEGEND = (
    ("돌출", (0.89, 0.18, 0.18)),
    ("도랑·낙차", (0.16, 0.38, 0.93)),
    ("주행 가능(높이 음영)", (0.60, 0.60, 0.55)),
    ("관측 부족", (0.50, 0.50, 0.50)),
    ("미관측", (0.07, 0.09, 0.12)),
)
BACKGROUND = (0.07, 0.09, 0.12)
REASON_TEXT = {
    None: "", "ODOM_MISSING": "주행 기록 없음", "POSE_GAP": "자세 끊김", "WHEELS_STALE": "바퀴 신호 끊김",
    "BIAS_PENDING": "로버를 1초 정지하세요", "GROUND_NOT_FOUND": "바닥 못 찾음", "NOT_SUPPORT": "바닥 아님(지지면 불일치)",
    "DEPTH_STALE": "깊이 영상 끊김", "DEPTH_ENCODING": "깊이 형식 오류", "DEPTH_SHAPE_MISMATCH": "깊이 해상도 불일치",
    "CAMERA_INFO_MISSING": "카메라 정보 없음", "PROCESSING_ERROR": "처리 오류",
}


def surface_from_rgba(rgba: np.ndarray) -> tuple[cairo.ImageSurface, np.ndarray]:
    """uint8 RGBA (straight alpha) → cairo ARGB32 (premultiplied, native BGRA).

    The surface borrows the returned buffer: keep it alive until painting is done.
    """
    h, w = rgba.shape[:2]
    stride = cairo.ImageSurface.format_stride_for_width(cairo.FORMAT_ARGB32, w)
    a = rgba[..., 3:4].astype(np.uint16)
    pre = (rgba[..., :3].astype(np.uint16) * a // 255).astype(np.uint8)
    buf = np.zeros((h, stride), np.uint8)
    view = buf[:, : w * 4].reshape(h, w, 4)
    view[..., 0], view[..., 1], view[..., 2], view[..., 3] = pre[..., 2], pre[..., 1], pre[..., 0], rgba[..., 3]
    return cairo.ImageSurface.create_for_data(memoryview(buf), cairo.FORMAT_ARGB32, w, h, stride), buf


class MapPanel(Gtk.Box):
    def __init__(self, receiver: LatestMapReceiver, display: DisplayConfig | None = None) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=0)
        self.receiver = receiver
        self.display = display or DisplayConfig()
        self._renderer = MapRenderer()
        self._m_per_px = DEFAULT_M_PER_PX
        self._follow = True
        self._center = (0.0, 0.0)
        self._drag: tuple[float, float, tuple[float, float]] | None = None
        self._packet_times: deque[float] = deque()
        self._last_count = 0
        self.last_meta: dict = {}
        self.unknown_glyphs = 0
        bar = Gtk.Box(orientation=Gtk.Orientation.HORIZONTAL, spacing=6)
        self.follow_button = Gtk.ToggleButton(label="로버 따라가기")
        self.follow_button.set_active(True)
        self.follow_button.connect("toggled", self._on_follow)
        self.clear_button = Gtk.Button(label="화면에서 지우기")
        self.clear_button.connect("clicked", lambda _b: (self.receiver.clear_display(), self.area.queue_draw()))
        bar.pack_start(self.follow_button, False, False, 0)
        bar.pack_start(self.clear_button, False, False, 0)
        self.pack_start(bar, False, False, 4)
        self.area = Gtk.DrawingArea()
        self.area.set_hexpand(True)
        self.area.set_vexpand(True)
        self.area.add_events(Gdk.EventMask.SCROLL_MASK | Gdk.EventMask.BUTTON_PRESS_MASK
                             | Gdk.EventMask.BUTTON_RELEASE_MASK | Gdk.EventMask.POINTER_MOTION_MASK)
        self.area.connect("draw", self._on_draw)
        self.area.connect("scroll-event", self._on_scroll)
        self.area.connect("button-press-event", self._on_press)
        self.area.connect("button-release-event", lambda *_a: setattr(self, "_drag", None))
        self.area.connect("motion-notify-event", self._on_motion)
        self.pack_start(self.area, True, True, 0)
        self._source = GLib.timeout_add(REDRAW_MS, self._tick)
        self.connect("destroy", self._on_destroy)

    # --- state for the probe / status line ----------------------------------------
    @staticmethod
    def status_age(snap: MapSnapshot, now_s: float) -> float | None:
        """Age of the last status packet — tiles alone never make a pose current (Codex T9 P2)."""
        if snap.status is None or snap.status.received_s is None:
            return None
        return now_s - snap.status.received_s

    def probe(self, now_s: float | None = None) -> dict:
        now_s = time.monotonic() if now_s is None else now_s
        snap = self.receiver.snapshot(now_s)
        age = self.status_age(snap, now_s)
        state = "WAITING" if age is None else ("STALE" if age > LINK_LOST_S else "LIVE")
        return {"terrain_map": state, "terrain_map_tiles": len(snap.tiles)}

    def _packet_rate(self, snap: MapSnapshot, now_s: float) -> float:
        count = self.receiver.accepted_packet_count
        if count != self._last_count:
            for _ in range(min(count - self._last_count, 200)):
                self._packet_times.append(now_s)
            self._last_count = count
        while self._packet_times and now_s - self._packet_times[0] > 2.0:
            self._packet_times.popleft()
        return len(self._packet_times) / 2.0

    # --- interaction -------------------------------------------------------------------
    def _on_follow(self, button: Gtk.ToggleButton) -> None:
        if not button.get_active():
            self._center = self._view_center()          # stay where the operator was looking
        self._follow = button.get_active()
        self.area.queue_draw()

    def _on_scroll(self, _w, event) -> bool:
        factor = 1.0 / 1.2 if event.direction == Gdk.ScrollDirection.UP else 1.2
        if event.direction == Gdk.ScrollDirection.SMOOTH:
            factor = 1.2 ** event.delta_y
        self._m_per_px = min(max(self._m_per_px * factor, ZOOM_RANGE[0]), ZOOM_RANGE[1])
        self.area.queue_draw()
        return True

    def _on_press(self, _w, event) -> bool:
        if event.button == 1:
            self._drag = (event.x, event.y, self._view_center())
            self.follow_button.set_active(False)
        return True

    def _on_motion(self, _w, event) -> bool:
        if self._drag is None:
            return False
        x0, y0, (cx, cy) = self._drag
        phi = self._rotation()
        dx, dy = (event.x - x0) * self._m_per_px, (event.y - y0) * self._m_per_px
        # undo the screen rotation, then the y flip (screen down = world -y before rotation)
        lx = math.cos(-phi) * dx - math.sin(-phi) * dy
        ly = math.sin(-phi) * dx + math.cos(-phi) * dy
        self._center = (cx - lx, cy + ly)
        self.area.queue_draw()
        return True

    def _tick(self) -> bool:
        if self.area.get_mapped() and self.area.is_drawable():
            self.area.queue_draw()                              # hidden page: nothing is drawn
        return True

    def _on_destroy(self, *_a) -> None:
        if self._source is not None:
            GLib.source_remove(self._source)
            self._source = None

    # --- geometry ------------------------------------------------------------------------
    def _pose(self, snap: MapSnapshot):
        p = None if snap.status is None else snap.status.pose
        return None if p is None else (p.x_m, p.y_m, p.yaw_rad)

    def _view_center(self) -> tuple[float, float]:
        pose = self._pose(self.receiver.snapshot(time.monotonic()))
        if self._follow and pose is not None:
            return pose[0], pose[1]
        return self._center

    def _rotation(self) -> float:
        """Screen rotation φ: heading up while following, world +y up otherwise."""
        pose = self._pose(self.receiver.snapshot(time.monotonic()))
        if self._follow and pose is not None:
            return pose[2] - math.pi / 2.0
        return 0.0

    # --- drawing -------------------------------------------------------------------------
    def _text(self, cr, text: str, font: str, width: float | None = None):
        layout = PangoCairo.create_layout(cr)
        layout.set_font_description(Pango.FontDescription(font))
        if width is not None:
            layout.set_width(int(max(40.0, width) * Pango.SCALE))
            layout.set_wrap(Pango.WrapMode.WORD_CHAR)
        layout.set_text(text, -1)
        self.unknown_glyphs += layout.get_unknown_glyphs_count()
        return layout

    def draw_into(self, cr, width: int, height: int, now_s: float) -> None:
        """Everything the page shows, for the widget and for tests (offscreen surface)."""
        self.unknown_glyphs = 0
        snap = self.receiver.snapshot(now_s)
        cr.set_source_rgb(*BACKGROUND)
        cr.paint()
        pose = self._pose(snap)
        age = self.status_age(snap, now_s)
        current = age is not None and age <= LINK_LOST_S
        if self._follow and pose is not None:
            cx, cy, phi = pose[0], pose[1], pose[2] - math.pi / 2.0
        else:
            (cx, cy), phi = self._center, 0.0
        # Never zoom out past what the render budget can draw (Codex T9 P2).
        cell = snap.status.cell_m if snap.status is not None else 0.05
        limit = 0.98 * cell * math.sqrt(self.display.max_render_pixels) / math.hypot(width, height)
        self._m_per_px = min(self._m_per_px, limit)
        mpp = self._m_per_px
        reach = math.hypot(width, height) / 2.0 * mpp
        view = MapView(cx - reach, cy - reach, 2 * reach, 2 * reach, int(math.hypot(width, height)) + 1,
                       int(math.hypot(width, height)) + 1)
        rgba, meta = self._renderer.render(snap, view, self.display, now_s)
        self.last_meta = meta
        cr.save()
        cr.translate(width / 2.0, height / 2.0)
        cr.rotate(phi)
        if meta.get("drawn"):
            k = meta["m_per_px"] / mpp
            cr.save()
            cr.translate((meta["origin_x_m"] - cx) / mpp, (cy - meta["top_y_m"]) / mpp)
            cr.scale(k, k)
            surface, buffer = surface_from_rgba(rgba)
            cr.set_source_surface(surface, 0, 0)
            cr.get_source().set_filter(cairo.FILTER_GOOD)
            cr.paint()
            surface.finish()
            del buffer
            cr.restore()
        self._grid(cr, cx, cy, reach, mpp)
        self._trajectory(cr, snap, cx, cy, mpp)
        if pose is not None:
            self._rover(cr, pose, cx, cy, mpp, current)
        cr.restore()
        if snap.status is not None and not meta.get("drawn"):
            note = self._text(cr, f"지도 표시 불가: {meta.get('reason', '-')}", "Sans Bold 14")
            cr.set_source_rgba(1.0, 0.8, 0.2, 1.0)
            cr.move_to(16, height / 2.0)
            PangoCairo.show_layout(cr, note)
        self._scale_bar(cr, width, height, mpp)
        self._overlay_text(cr, snap, width, height, now_s)

    def _grid(self, cr, cx, cy, reach, mpp) -> None:
        cr.set_source_rgba(1.0, 1.0, 1.0, 0.10)
        cr.set_line_width(1.0)
        for x in range(math.floor(cx - reach), math.ceil(cx + reach) + 1):
            cr.move_to((x - cx) / mpp, -reach / mpp)
            cr.line_to((x - cx) / mpp, reach / mpp)
        for y in range(math.floor(cy - reach), math.ceil(cy + reach) + 1):
            cr.move_to(-reach / mpp, (cy - y) / mpp)
            cr.line_to(reach / mpp, (cy - y) / mpp)
        cr.stroke()

    def _trajectory(self, cr, snap: MapSnapshot, cx, cy, mpp) -> None:
        if len(snap.trajectory) < 2:
            return
        cr.set_source_rgba(1.0, 0.85, 0.2, 0.85)
        cr.set_line_width(2.0)
        first, *rest = snap.trajectory
        cr.move_to((first[0] - cx) / mpp, (cy - first[1]) / mpp)
        for x, y in rest:
            cr.line_to((x - cx) / mpp, (cy - y) / mpp)
        cr.stroke()

    def _rover(self, cr, pose, cx, cy, mpp, current: bool = True) -> None:
        x, y, yaw = (pose[0] - cx) / mpp, (cy - pose[1]) / mpp, pose[2]
        cr.save()
        cr.translate(x, y)
        cr.rotate(-yaw)
        cr.move_to(16, 0)
        cr.line_to(-10, -9)
        cr.line_to(-10, 9)
        cr.close_path()
        cr.set_source_rgba(*((1.0, 1.0, 1.0, 0.95) if current else (0.55, 0.55, 0.55, 0.6)))
        cr.fill_preserve()
        cr.set_source_rgba(0.0, 0.0, 0.0, 0.9)
        cr.set_line_width(1.5)
        cr.stroke()
        if not current:
            cr.rotate(yaw)
            label = self._text(cr, "마지막 자세", "Sans 10")
            cr.set_source_rgba(0.8, 0.8, 0.8, 0.9)
            cr.move_to(12, 10)
            PangoCairo.show_layout(cr, label)
        cr.restore()

    def _scale_bar(self, cr, width, height, mpp) -> None:
        metres = 1.0 if 1.0 / mpp <= width / 3 else 0.5
        length = metres / mpp
        x0, y0 = 16.0, height - 70.0
        cr.set_source_rgba(1.0, 1.0, 1.0, 0.9)
        cr.set_line_width(3.0)
        cr.move_to(x0, y0)
        cr.line_to(x0 + length, y0)
        cr.stroke()
        layout = self._text(cr, f"{metres:g} m", "Sans 10")
        cr.move_to(x0, y0 - 18)
        PangoCairo.show_layout(cr, layout)

    def _overlay_text(self, cr, snap: MapSnapshot, width, height, now_s) -> None:
        s = snap.status
        rate = self._packet_rate(snap, now_s)
        address = self.receiver.last_address or "-"
        if s is None:
            line = f"지형 지도 수신 대기 · :{self.receiver.port}"
        else:
            reason = REASON_TEXT.get(s.reason, s.reason or "")
            corrector = s.corrector.name + (" 적용" if s.corrector.applied else "")
            age = "-" if s.received_s is None else f"{now_s - s.received_s:.1f}s"
            line = (f"{s.status}{(' · ' + reason) if reason else ''} · epoch {s.map_epoch} · 거리 {s.distance_m:.1f} m"
                    f" · 자세 끊김 {s.pose_gaps} · 타일 {len(snap.tiles)}/{s.tile_count} · 수신 {age} 전"
                    f" · {rate:.0f} pkt/s · {address} · 보정 {corrector}"
                    f" · 경계 +{s.thresholds.obstacle_height_m * 100:.0f}/-{s.thresholds.ditch_depth_m * 100:.0f} cm")
        layout = self._text(cr, line, "Sans Bold 11", width - 24)
        _, h = layout.get_pixel_size()
        cr.set_source_rgba(0.0, 0.0, 0.0, 0.65)
        cr.rectangle(0, 0, width, h + 12)
        cr.fill()
        cr.set_source_rgba(0.95, 0.96, 0.98, 1.0)
        cr.move_to(12, 6)
        PangoCairo.show_layout(cr, layout)
        h += self._tilt_lines(cr, s, snap, width, h, now_s)
        if s is not None and s.reason == "BIAS_PENDING":
            hint = self._text(cr, "로버를 1초 정지하세요", "Sans Bold 22")
            w, _ = hint.get_pixel_size()
            cr.set_source_rgba(1.0, 0.8, 0.2, 1.0)
            cr.move_to((width - w) / 2.0, h + 24)
            PangoCairo.show_layout(cr, hint)
        status_age = self.status_age(snap, now_s)
        if (snap.last_rx_s is not None and now_s - snap.last_rx_s > LINK_LOST_S) or (
                status_age is not None and status_age > LINK_LOST_S):
            lost = self._text(cr, "수신 끊김", "Sans Bold 32")
            w, lh = lost.get_pixel_size()
            cr.set_source_rgba(0.0, 0.0, 0.0, 0.6)
            cr.rectangle((width - w) / 2.0 - 16, height / 2.0 - lh, w + 32, lh + 16)
            cr.fill()
            cr.set_source_rgba(1.0, 0.35, 0.3, 1.0)
            cr.move_to((width - w) / 2.0, height / 2.0 - lh + 8)
            PangoCairo.show_layout(cr, lost)
        # legend + fixed note at the bottom
        note = self._text(cr, FIXED_NOTE, "Sans 10", width - 24)
        _, nh = note.get_pixel_size()
        top = height - nh - 40
        cr.set_source_rgba(0.0, 0.0, 0.0, 0.65)
        cr.rectangle(0, top - 6, width, height - top + 6)
        cr.fill()
        x = 12.0
        for label, rgb in LEGEND:
            cr.set_source_rgb(*rgb)
            cr.rectangle(x, top + 2, 14, 14)
            cr.fill()
            text = self._text(cr, label, "Sans 10")
            cr.set_source_rgba(0.95, 0.96, 0.98, 1.0)
            cr.move_to(x + 18, top)
            PangoCairo.show_layout(cr, text)
            x += 18 + text.get_pixel_size()[0] + 14
        cr.set_source_rgba(0.85, 0.88, 0.92, 1.0)
        cr.move_to(12, top + 24)
        PangoCairo.show_layout(cr, note)

    def _tilt_lines(self, cr, s, snap, width, top, now_s) -> float:
        """Plane tilt (r11) under the status line while the observation is younger than 1 s
        (its age at send + time since the packet arrived).  Returns the height used."""
        if s is None or s.plane_tilt is None or s.received_s is None:
            return 0.0
        if s.plane_tilt.age_ms / 1000.0 + (now_s - s.received_s) > TILT_MAX_AGE_S:
            return 0.0
        lines = [f"기준면 기울기(차체 기준) 앞 {s.plane_tilt.pitch_deg:+.1f}° 옆 {s.plane_tilt.roll_deg:+.1f}° "
                 f"· 기준면 = 판정기가 고른 가까운 바닥"]
        g = s.gravity
        if g is not None:
            lines.append(f"정지 구간 측정 · 기준면 경사(중력 기준) 앞 {g.slope_pitch_deg:+.1f}° 옆 {g.slope_roll_deg:+.1f}° "
                         f"(전체 {g.slope_total_deg:.1f}°) · 차체 앞 들림 {g.body_pitch_deg:+.1f}° "
                         f"왼쪽 들림 {g.body_roll_deg:+.1f}°")
        layout = self._text(cr, "\n".join(lines), "Sans 10", width - 24)
        _, h = layout.get_pixel_size()
        cr.set_source_rgba(0.0, 0.0, 0.0, 0.55)
        cr.rectangle(0, top + 12, width, h + 8)
        cr.fill()
        cr.set_source_rgba(0.80, 0.90, 1.0, 1.0)
        cr.move_to(12, top + 16)
        PangoCairo.show_layout(cr, layout)
        return h + 8

    def _on_draw(self, widget, cr) -> bool:
        a = widget.get_allocation()
        self.draw_into(cr, a.width, a.height, time.monotonic())
        return False
