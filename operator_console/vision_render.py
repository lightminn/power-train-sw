"""Bounded, latest-only map raster work. This module never calls GTK or opens sockets."""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import threading
import time

import cv2
import numpy as np

from lidar_dev import live_map
from terrain_map import contract as C
from .terrain_map import DisplayConfig, MapRenderer, MapView
from .vision_config import VisionCriteria

PERIOD_S = 0.5
LINK_LOST_S = 0.5


@dataclass(frozen=True)
class MapSettings:
    top_size: tuple[int, int] = (480, 180)
    oblique_size: tuple[int, int] = (480, 180)
    scale: float = 1.0
    follow: bool = True
    center: tuple[float, float] = (0.0, 0.0)
    yaw: float = 0.0


@dataclass(frozen=True)
class Raster:
    width: int
    height: int
    bgra: bytearray


def raster(rgba) -> Raster:
    """Worker-side premultiplied BGRA; GTK only wraps the finished buffer in cairo."""
    alpha = rgba[..., 3:4].astype(np.uint16)
    rgb = (rgba[..., :3].astype(np.uint16) * alpha // 255).astype(np.uint8)
    bgra = np.concatenate((rgb[..., ::-1], rgba[..., 3:4]), axis=2)
    return Raster(rgba.shape[1], rgba.shape[0], bytearray(bgra.tobytes()))


@dataclass(frozen=True)
class MapImages:
    top: Raster | None
    oblique: Raster | None
    status: object
    center: tuple[float, float]
    yaw: float
    top_note: str
    oblique_note: str
    render_ms: float
    created_s: float


def _size(size):
    w, h = (max(64,int(value)) for value in size)
    shrink = min(1.0,900/w,480/h)
    return max(2,round(w*shrink)), max(2,round(h*shrink))


def _arrow(image, point, tip, current):
    x, y = point
    if 0 <= x < image.shape[1] and 0 <= y < image.shape[0]:
        color = (255, 220, 50, 255) if current else (145, 145, 145, 255)
        cv2.arrowedLine(image, (round(x), round(y)), tuple(round(v) for v in tip),
                        color, 2, cv2.LINE_AA, tipLength=0.45)


class VisionRenderer:
    """One instance/cache owned by one worker. Existing map/classification code is reused."""

    def __init__(self, display: DisplayConfig, criteria: VisionCriteria):
        self.display = replace(display, max_render_pixels=min(display.max_render_pixels, 500_000))
        self.criteria = criteria
        self.top_renderer = MapRenderer()

    def render(self, snap, settings: MapSettings, now_s: float) -> MapImages:
        started = time.monotonic()
        st = snap.status
        pose = None if st is None else st.pose
        current = st is not None and st.received_s is not None and now_s - st.received_s <= LINK_LOST_S
        center, yaw = settings.center, settings.yaw
        if settings.follow and pose is not None:
            # Compact four-view layout: include the forward terrain, with the rover near the bottom.
            _, height = _size(settings.top_size)
            ahead = .3 * max(.01,3.2/height) * settings.scale * height
            yaw = pose.yaw_rad
            center = pose.x_m+ahead*math.cos(yaw), pose.y_m+ahead*math.sin(yaw)
        top, top_note = self._top(snap, settings, center, yaw, current, now_s)
        oblique, oblique_note = self._oblique(snap, settings, center, yaw, current, now_s)
        return MapImages(top, oblique, st, center, yaw, top_note, oblique_note,
                         (time.monotonic() - started) * 1000, time.monotonic())

    def _top(self, snap, settings, center, yaw, current, now):
        if snap.status is None:
            return None, "지도 미수신"
        w, h = _size(settings.top_size)
        mpp = max(.01,3.2/h) * settings.scale
        reach = math.hypot(w, h) * mpp / 2
        cx, cy = center
        side = math.ceil(math.hypot(w, h)) + 2
        rgba, meta = self.top_renderer.render(snap, MapView(cx - reach, cy - reach,
            2 * reach, 2 * reach, side, side), self.display, now)
        if not meta.get("drawn"):
            return None, "지도 표시 범위 초과" if meta.get("reason") else "관측 지도 없음"
        phi = yaw - math.pi / 2 if settings.follow else 0.0
        c, s = math.cos(phi), math.sin(phi)
        k = meta["m_per_px"] / mpp
        ox, oy = meta["origin_x_m"] - cx, meta["top_y_m"] - cy
        matrix = np.array([[c*k, -s*k, w/2 + (c*ox+s*oy)/mpp],
                           [s*k, c*k, h/2 + (s*ox-c*oy)/mpp]], np.float32)
        image = cv2.warpAffine(rgba, matrix, (w, h), flags=cv2.INTER_NEAREST,
                               borderValue=(18, 22, 28, 255))
        pose = snap.status.pose
        if pose is not None:
            def point(x, y):
                dx, dy = x - cx, y - cy
                return w/2 + (c*dx+s*dy)/mpp, h/2 + (s*dx-c*dy)/mpp
            start = point(pose.x_m, pose.y_m)
            tip = point(pose.x_m + 18*mpp*math.cos(pose.yaw_rad),
                        pose.y_m + 18*mpp*math.sin(pose.yaw_rad))
            _arrow(image, start, tip, current and snap.status.status == "OK")
        # A physical scale bar is independent of the map's upsample factor.
        length_m = max(0.1, 10 ** math.floor(math.log10(w*mpp/4)))
        length_px = round(length_m/mpp)
        cv2.line(image, (12, h-12), (12+length_px, h-12), (230,230,230,255), 2)
        return raster(image), f"눈금 {length_m:g} m · 송신 기준 +{snap.status.thresholds.obstacle_height_m*100:g}/−{snap.status.thresholds.ditch_depth_m*100:g} cm"

    def _oblique(self, snap, settings, center, yaw, current, now):
        st = snap.status
        if st is None:
            return None, "지도 미수신"
        if not current:
            return None, "수신 끊김 · 현재 자세로 재투영하지 않음"
        if st.pose is None or st.status != "OK":
            return None, "로버 자세 미수신 · 전방 투영 보류"
        # Origin = rover base, not an unverified camera mounting translation.
        # In manual mode the same grid helper views the selected world location.
        virtual = snap if settings.follow else replace(snap, status=replace(st,
            pose=C.Pose(center[0], center[1], yaw)))
        fwd = min(2.5 * settings.scale, 160 * st.cell_m)
        lat = min(1.25 * settings.scale, 80 * st.cell_m)
        # Clip before tmap_local_grid assembles a dense tile rectangle.
        px, py, heading = (virtual.status.pose.x_m, virtual.status.pose.y_m, virtual.status.pose.yaw_rad)
        c, s = math.cos(heading), math.sin(heading)
        corners = [(px+f*c+l*s, py+f*s-l*c) for f in (0, fwd) for l in (-lat, lat)]
        span = st.cell_m * st.tile_cells
        xmin, xmax = min(p[0] for p in corners)-span, max(p[0] for p in corners)+span
        ymin, ymax = min(p[1] for p in corners)-span, max(p[1] for p in corners)+span
        tiles = tuple(t for t in snap.tiles if xmin <= (t.tx+1)*span and t.tx*span <= xmax
                      and ymin <= (t.ty+1)*span and t.ty*span <= ymax)
        grid = live_map.tmap_local_grid(replace(virtual, tiles=tiles), now, (0.0, 0.0),
            fwd, lat, self.display.stale_display_s, C)
        if grid is None:
            return None, "전방 관측 지도 없음"
        levels, dip = live_map.classify_heights(grid["Z"], grid["known"], self.criteria.rules)
        filtered, dip_filtered, _ = live_map.filter_hazards(levels, dip, min_neighbors=1)
        # The source helper marks removed spikes as pass; here their evidence is withheld.
        # Sender classes use different thresholds, so never veto policy risks using codes.
        held = (levels >= 2) & (filtered < 2)
        known = grid["known"] & ~held
        filtered[held] = 0
        known = live_map.tidy_low(grid["Z"], known, filtered,
            self.display.tidy_low_m, self.display.tidy_min_neighbours)
        colors = live_map.cell_colors(grid["Z"], known, filtered, dip_filtered, "class", grid["cell"])
        w, h = _size(settings.oblique_size)
        raw = live_map.oblique_image(grid["Z"], colors, known, W=w, Hh=300, fwd_max=fwd)
        # The notebook caption advertises keys unavailable in the console. Keep its raster/metric ticks.
        image = cv2.resize(raw[24:], (w, h), interpolation=cv2.INTER_NEAREST)
        rgba = np.dstack((image[..., ::-1], np.full((h, w), 255, np.uint8)))
        # Project the actual rover into the virtual local grid; do not invent an origin pose.
        dx, dy = st.pose.x_m-px, st.pose.y_m-py
        f, l = dx*c+dy*s, dx*s-dy*c
        if 0 <= f < fwd and -lat <= l <= lat:
            y = (286 - 236 * f / fwd - 24) / 276 * h
            inset = 0.14 * 0.35 / 2 * f/fwd
            x = (inset + (1-2*inset) * (l+lat)/(2*lat))*w
            angle = st.pose.yaw_rad-heading
            _arrow(rgba, (x, min(y,h-10)), (x-16*math.sin(angle), min(y,h-10)-16*math.cos(angle)), True)
        origin = "로버 기준" if settings.follow else "선택 위치 기준"
        return raster(rgba), f"{origin} 전방 {fwd:g} m · 폭 {2*lat:g} m · 높이 표시 −15~+30 cm"


class MapRenderWorker:
    """Single cache owner, latest settings/result only, no idle callback queue."""

    def __init__(self, receiver, display, criteria):
        self.receiver = receiver
        self.display, self.criteria = display, criteria
        self._condition = threading.Condition()
        self._settings = MapSettings()
        self._active = False
        self._closed = False
        self._generation = 0
        self._serial = 0
        self._result = None
        self.render_count = 0
        self.last_error = None
        self._thread = threading.Thread(target=self._run, name="vision-map-render", daemon=True)
        self._thread.start()

    def configure(self, settings=None, *, active=None):
        with self._condition:
            changed = settings is not None and settings != self._settings
            changed |= active is not None and active != self._active
            if changed:
                self._generation += 1
                self._result = None
            if settings is not None:
                self._settings = settings
            if active is not None:
                self._active = active
            self._condition.notify_all()

    def latest(self):
        with self._condition:
            return self._serial, self._result, self.last_error

    def close(self):
        with self._condition:
            self._closed = True
            self._active = False
            self._generation += 1
            self._result = None
            self._condition.notify_all()
        self._thread.join(timeout=0.3)

    def _run(self):
        renderer = VisionRenderer(self.display, self.criteria)
        due = 0.0
        while True:
            with self._condition:
                while not self._closed and (not self._active or time.monotonic() < due):
                    if not self._active:
                        due = 0.0
                    self._condition.wait(None if not self._active else max(0, due-time.monotonic()))
                if self._closed:
                    return
                settings, generation = self._settings, self._generation
            now = time.monotonic()
            due = now + PERIOD_S
            try:
                result = renderer.render(self.receiver.snapshot(now), settings, now)
                error = None
            except Exception as exc:
                result, error = None, f"{type(exc).__name__}: {exc}"
            with self._condition:
                if generation == self._generation and self._active and not self._closed:
                    self._result, self.last_error = result, error
                    self._serial += 1
                    self.render_count += 1
