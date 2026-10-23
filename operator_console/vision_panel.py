"""Vision page layout. Receivers and the camera remain owned by OperatorConsole."""
from __future__ import annotations

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import Gtk  # noqa: E402

from .terrain_map import DisplayConfig
from .vision_config import resolve_criteria
from .vision_map_view import VisionMaps
from .vision_attitude_view import AttitudePanel


class VisionPanel(Gtk.Box):
    """Four simultaneous slots sharing the console's receivers and camera."""

    def __init__(self, *, map_receiver, chassis_receiver, overlay_enabled: bool,
                 on_overlay_toggled, map_display=None, criteria=None) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self.map_receiver = map_receiver
        self.chassis_receiver = chassis_receiver
        self.get_style_context().add_class("vision-page")
        self.set_border_width(8)
        self.show_terrain = Gtk.CheckButton(label="지형 장애물")
        self.show_terrain.set_active(overlay_enabled)
        self.show_terrain.connect("toggled", on_overlay_toggled)
        self.show_terrain.set_tooltip_text("실시간 화면과 같은 지형 표시 설정을 사용합니다")
        self.maps = VisionMaps(map_receiver, map_display or DisplayConfig(), criteria or resolve_criteria())
        bar = Gtk.Box(spacing=10)
        bar.pack_start(self.show_terrain, False, False, 0)
        bar.pack_start(self.maps.controls, True, True, 0)
        self.pack_start(bar, False, False, 0)

        grid = Gtk.Grid(column_spacing=8, row_spacing=8)
        grid.set_column_homogeneous(True)
        grid.set_row_homogeneous(True)
        grid.set_hexpand(True)
        grid.set_vexpand(True)
        self.camera_slot = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.attitude_slot = AttitudePanel(chassis_receiver,
            self.maps.worker.latest_status,
            map_port=map_receiver.port)
        self.top_slot = self.maps.top_slot
        self.oblique_slot = self.maps.oblique_slot
        for title, slot, column, row in (
            ("L515 카메라", self.camera_slot, 0, 0),
            ("차체 자세", self.attitude_slot, 1, 0),
            ("Top view", self.top_slot, 0, 1),
            ("Oblique view · 2.5D", self.oblique_slot, 1, 1),
        ):
            frame = Gtk.Frame(label=title)
            frame.get_style_context().add_class("vision-card")
            frame.set_hexpand(True)
            frame.set_vexpand(True)
            frame.add(slot)
            grid.attach(frame, column, row, 1, 1)
        self.pack_start(grid, True, True, 0)
        note = Gtk.Label(label="지도는 지나온 관측 기록 · 실시간 장애물 감지가 아님 · 표시 기준은 주행 허가가 아님", xalign=0)
        note.get_style_context().add_class("muted")
        note.set_line_wrap(True)
        note.set_max_width_chars(1)
        self.pack_start(note, False, False, 0)

    def set_overlay_enabled(self, enabled: bool) -> None:
        if self.show_terrain.get_active() != enabled:
            self.show_terrain.set_active(enabled)
