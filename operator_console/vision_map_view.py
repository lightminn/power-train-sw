"""GTK presentation of prepared vision rasters. Map computation stays in vision_render."""
from __future__ import annotations

from collections import OrderedDict
from dataclasses import replace
import time

import cairo
import gi

gi.require_version("Gtk", "3.0")
gi.require_version("PangoCairo", "1.0")
from gi.repository import Gdk, GLib, Gtk, Pango, PangoCairo  # noqa: E402

from .terrain_map_panel import REASON_TEXT, note_and_legend
from .vision_render import LINK_LOST_S, MapRenderWorker, MapSettings


class MapCanvas(Gtk.DrawingArea):
    def __init__(self):
        super().__init__()
        self.set_hexpand(True)
        self.set_vexpand(True)
        self.raster = None
        self.surface = None
        self.message = "지도 미수신"
        self._layouts = OrderedDict()
        self.connect("draw", self._draw)
        self.connect("destroy", lambda *_: self.set_raster(None))

    def set_raster(self, raster):
        if self.surface is not None:
            self.surface.finish()
        self.raster = raster  # surface borrows this buffer
        self.surface = None if raster is None else cairo.ImageSurface.create_for_data(
            raster.bgra, cairo.FORMAT_ARGB32, raster.width, raster.height, raster.width*4)
        self.queue_draw()

    def _draw(self, _widget, cr):
        w, h = self.get_allocated_width(), self.get_allocated_height()
        cr.set_source_rgb(.07, .085, .10)
        cr.paint()
        if self.surface is not None:
            cr.save()
            cr.scale(w/self.raster.width, h/self.raster.height)
            cr.set_source_surface(self.surface, 0, 0)
            cr.paint()
            cr.restore()
        if self.message:
            key = (self.message, w)
            layout = self._layouts.pop(key, None)
            if layout is None:
                layout = PangoCairo.create_layout(cr)
                layout.set_font_description(Pango.FontDescription("Sans Bold 10"))
                layout.set_width(max(40,w-16)*Pango.SCALE)
                layout.set_wrap(Pango.WrapMode.WORD_CHAR)
                layout.set_text(self.message, -1)
            self._layouts[key] = layout
            while len(self._layouts) > 64:
                self._layouts.popitem(last=False)
            _, lh = layout.get_pixel_size()
            cr.set_source_rgba(0,0,0,.78)
            cr.rectangle(0,0,w,lh+10)
            cr.fill()
            cr.set_source_rgb(1,.85,.45)
            cr.move_to(8,5)
            PangoCairo.show_layout(cr, layout)
        return False


class VisionMaps:
    """Two coordinated views. A hidden page suspends its worker, never its receiver."""

    def __init__(self, receiver, display, criteria):
        self.receiver, self.criteria = receiver, criteria
        self.settings = MapSettings()
        self.worker = MapRenderWorker(receiver, display, criteria)
        self.top = MapCanvas()
        self.oblique = MapCanvas()
        self.top_slot = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        self.oblique_slot = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=2)
        self.top_detail = Gtk.Label(xalign=0)
        self.oblique_detail = Gtk.Label(xalign=0)
        top_legend = " · ".join(label for label, _color in note_and_legend(display)[1])
        self.top.set_tooltip_text(note_and_legend(display)[0])
        for slot, canvas, detail, legend in (
            (self.top_slot, self.top, self.top_detail, "송신 지도: 빨강 돌출 · 파랑 도랑·낙차 · 높이 음영\n"+top_legend),
            (self.oblique_slot, self.oblique, self.oblique_detail,
             "표시 기준: 초록 통과 · 노랑 주의 · 빨강 불가\n꺼짐: 하늘 주의 · 보라 불가 · 회색 미확인"),
        ):
            slot.pack_start(canvas, True, True, 0)
            detail.set_ellipsize(Pango.EllipsizeMode.END)
            detail.set_max_width_chars(1)
            slot.pack_start(detail, False, False, 0)
            label = Gtk.Label(label=legend, xalign=0)
            label.set_line_wrap(True)
            label.set_max_width_chars(38)
            slot.pack_start(label, False, False, 0)
        self.controls = Gtk.Box(spacing=6)
        self.follow = Gtk.ToggleButton(label="로버 따라가기")
        self.follow.set_active(True)
        self.follow.connect("toggled", self._follow)
        self.controls.pack_start(self.follow, False, False, 0)
        for text, factor in (("−", 1.2), ("+", 1/1.2)):
            button = Gtk.Button(label=text)
            button.connect("clicked", lambda _b, f=factor: self.zoom(f))
            self.controls.pack_start(button, False, False, 0)
        self.provenance = Gtk.Label(label=f"{criteria.version} · {criteria.profile} · {criteria.sha256[:8]}", xalign=0)
        self.provenance.set_ellipsize(Pango.EllipsizeMode.MIDDLE)
        self.provenance.set_max_width_chars(1)
        self.provenance.set_hexpand(True)
        self.provenance.set_tooltip_text(f"{criteria.path}\nSHA256 {criteria.sha256}\nTop은 송신 기준, Oblique는 표시 기준을 사용합니다.\n관측 기록이며 주행 허가가 아닙니다.")
        self.controls.pack_start(self.provenance, True, True, 0)
        self._drag = None
        self._serial = -1
        self.images = None
        self.state = "WAITING"
        self.displayed_count = 0
        self._timer = None
        self._closed = False
        for canvas in (self.top, self.oblique):
            canvas.connect("size-allocate", self._resize)
            canvas.add_events(Gdk.EventMask.SCROLL_MASK | Gdk.EventMask.SMOOTH_SCROLL_MASK)
            canvas.connect("scroll-event", self._scroll)
        # Panning the world Top view also chooses the origin/heading for Oblique.
        self.top.add_events(Gdk.EventMask.BUTTON_PRESS_MASK | Gdk.EventMask.BUTTON_RELEASE_MASK |
                            Gdk.EventMask.POINTER_MOTION_MASK)
        self.top.connect("button-press-event", self._press)
        self.top.connect("button-release-event", lambda *_: setattr(self,"_drag",None) or True)
        self.top.connect("motion-notify-event", self._motion)
        self.top_slot.connect("map", self._mapped)
        self.top_slot.connect("unmap", self._unmapped)
        self.top_slot.connect("destroy", self.close)

    def _resize(self, *_):
        settings = replace(self.settings,
            top_size=(self.top.get_allocated_width(),self.top.get_allocated_height()),
            oblique_size=(self.oblique.get_allocated_width(),self.oblique.get_allocated_height()))
        if settings != self.settings:
            self.settings = settings
            self.worker.configure(settings)

    def _mapped(self, *_):
        if self._closed:
            return
        self._resize()
        self.worker.configure(self.settings, active=True)
        if self._timer is None:
            self._timer = GLib.timeout_add(100, self._poll)

    def _unmapped(self, *_):
        self.worker.configure(active=False)
        if self._timer is not None:
            GLib.source_remove(self._timer)
            self._timer = None
        self.images = None
        self.state = "WAITING"
        self.top.message = self.oblique.message = "지도 수신 확인 중"
        self.top.set_raster(None)
        self.oblique.set_raster(None)

    def zoom(self, factor):
        self.settings = replace(self.settings, scale=min(4.0,max(.25,self.settings.scale*factor)))
        self.worker.configure(self.settings)

    def _scroll(self, _widget, event):
        if event.direction == Gdk.ScrollDirection.SMOOTH:
            factor = 1.2 ** max(-5,min(5,event.delta_y))
        else:
            factor = 1/1.2 if event.direction == Gdk.ScrollDirection.UP else 1.2
        self.zoom(factor)
        return True

    def _follow(self, button):
        center, yaw = self.settings.center, self.settings.yaw
        if self.images is not None:
            center, yaw = self.images.center, self.images.yaw
        self.settings = replace(self.settings, follow=button.get_active(), center=center, yaw=yaw)
        self.worker.configure(self.settings)

    def _press(self, _widget, event):
        if event.button == 1:
            self.follow.set_active(False)
            self._drag = event.x,event.y,self.settings.center
        return True

    def _motion(self, _widget, event):
        if self._drag is None:
            return False
        x,y,(cx,cy) = self._drag
        ratio = (self.top.raster.width/self.top.get_allocated_width()
                 if self.top.raster is not None else 1.0)
        raster_height = self.top.raster.height if self.top.raster is not None else self.top.get_allocated_height()
        mpp = max(.01,3.2/max(1,raster_height))*self.settings.scale*ratio
        self.settings = replace(self.settings, center=(cx-(event.x-x)*mpp,cy+(event.y-y)*mpp))
        self.worker.configure(self.settings)
        return True

    def _poll(self):
        serial, images, error = self.worker.latest()
        if serial != self._serial and images is not None:
            self._serial, self.images = serial, images
            self.top.set_raster(images.top)
            self.oblique.set_raster(images.oblique)
            self.top_detail.set_text(images.top_note)
            self.oblique_detail.set_text(images.oblique_note)
            self.displayed_count += 1
        st = None if self.images is None else self.images.status
        age = None if st is None or st.received_s is None else max(0,time.monotonic()-st.received_s)
        self.state = "WAITING" if age is None else "STALE" if age > LINK_LOST_S else "LIVE"
        if error:
            self.state = "ERROR"
            self.top.set_raster(None)
            self.oblique.set_raster(None)
        status = (f"지도 미수신 · UDP :{self.receiver.port}" if age is None else
                  f"{'수신 끊김 · 과거 관측' if age > LINK_LOST_S else '수신 중'} · UDP :{self.receiver.port} · {age:.1f}초 전")
        if st is not None and st.status != "OK":
            status += " · " + REASON_TEXT.get(st.reason, st.reason or st.status)
        self.top.message = "지도 처리 오류: "+error if error else status
        self.oblique.message = ("지도 처리 오류: "+error if error else
            status + (" · 현재 투영 보류" if self.state != "LIVE" else ""))
        if self.state == "STALE":
            # Withdraw the live pose projection immediately, before the next 2 Hz result.
            self.oblique.set_raster(None)
        self.top.queue_draw()
        self.oblique.queue_draw()
        return True

    def close(self, *_):
        if self._closed:
            return
        self._closed = True
        self._unmapped()
        self.worker.close()
