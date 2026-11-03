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
                 on_overlay_toggled, map_display=None, criteria=None, calibration=None) -> None:
        super().__init__(orientation=Gtk.Orientation.VERTICAL, spacing=6)
        self.map_receiver = map_receiver
        self.chassis_receiver = chassis_receiver
        self.get_style_context().add_class("vision-page")
        self.set_border_width(18)
        self.show_terrain = Gtk.CheckButton(label="장애물 표시")
        self.show_terrain.set_active(overlay_enabled)
        self.show_terrain.connect("toggled", on_overlay_toggled)
        self.show_terrain.set_tooltip_text("실시간 화면과 같은 지형 표시 설정을 사용합니다")
        self.maps = VisionMaps(map_receiver, map_display or DisplayConfig(), criteria or resolve_criteria())
        bar = Gtk.Box(spacing=12)
        for item in (self.maps.state_badge, self.maps.provenance):
            self.maps.controls.remove(item)
        title = Gtk.Label(label="비전 관측", xalign=0)
        title.get_style_context().add_class("vision-page-title")
        bar.pack_start(title, True, True, 0)
        bar.pack_end(self.show_terrain, False, False, 0)
        bar.pack_end(self.maps.state_badge, False, False, 0)
        self.pack_start(bar, False, False, 0)

        layout = Gtk.Paned(orientation=Gtk.Orientation.HORIZONTAL)
        layout.set_wide_handle(True)
        layout.set_hexpand(True)
        layout.set_vexpand(True)
        side = Gtk.Paned(orientation=Gtk.Orientation.VERTICAL)
        side.set_wide_handle(True)
        self.camera_slot = Gtk.Box(orientation=Gtk.Orientation.VERTICAL)
        self.attitude_slot = AttitudePanel(chassis_receiver,
            self.maps.worker.latest_status,
            map_port=map_receiver.port,calibration=calibration)
        self.top_slot = self.maps.top_slot
        self.oblique_slot = self.maps.oblique_slot
        attitude_scroll = Gtk.ScrolledWindow()
        attitude_scroll.set_policy(Gtk.PolicyType.NEVER, Gtk.PolicyType.AUTOMATIC)
        attitude_scroll.add(self.attitude_slot)
        cards = []
        for title, slot in (("L515 카메라", self.camera_slot),
                            ("차체 자세 · 측정 상태", attitude_scroll)):
            frame = Gtk.Frame(label=title)
            frame.get_style_context().add_class("vision-card")
            frame.set_hexpand(True)
            frame.set_vexpand(True)
            frame.add(slot)
            cards.append(frame)
        map_card = Gtk.Frame()
        map_card.get_style_context().add_class("vision-card")
        self.map_stack = Gtk.Stack()
        self.map_stack.set_transition_type(Gtk.StackTransitionType.CROSSFADE)
        self.map_stack.set_transition_duration(180)
        self.map_stack.add_titled(self.top_slot, "top", "Top · 평면")
        self.map_stack.add_titled(self.oblique_slot, "oblique", "Oblique · 2.5D")
        switcher = Gtk.StackSwitcher()
        switcher.set_stack(self.map_stack)
        switcher.get_style_context().add_class("vision-map-switcher")
        map_heading = Gtk.Box(spacing=10)
        map_heading.pack_start(Gtk.Label(label="지형 지도 · 기록"), False, False, 0)
        map_heading.pack_start(switcher, False, False, 0)
        self.map_settings = Gtk.ToggleButton(label="지도 설정")
        self.map_settings.set_tooltip_text(self.maps.provenance.get_tooltip_text())
        map_heading.pack_end(self.map_settings, False, False, 0)
        map_card.set_label_widget(map_heading)
        map_card.set_tooltip_text("지도는 과거 관측 기록입니다. 주행 판단에는 현재 카메라와 안전 상태를 함께 확인하세요.")
        map_body = Gtk.Box(orientation=Gtk.Orientation.VERTICAL, spacing=4)
        map_body.pack_start(self.map_stack, True, True, 0)
        map_options = Gtk.Revealer()
        map_options.set_transition_type(Gtk.RevealerTransitionType.SLIDE_DOWN)
        map_options.add(self.maps.controls)
        self.map_settings.connect("toggled", lambda button: map_options.set_reveal_child(button.get_active()))
        map_body.pack_start(map_options, False, False, 0)
        map_card.add(map_body)
        side.pack1(map_card, resize=True, shrink=False)
        side.pack2(cards[1], resize=True, shrink=False)
        layout.pack1(cards[0], resize=True, shrink=False)
        layout.pack2(side, resize=True, shrink=False)
        self._split_sizes = {}
        for pane, ratio in ((layout, .57), (side, .62)):
            pane.connect("size-allocate", self._initial_split, ratio)
        self.pack_start(layout, True, True, 0)
        self.connect("destroy", self.close)

    def _initial_split(self, pane, allocation, ratio):
        # Set a useful first layout; subsequent drags remain under operator control.
        if pane in self._split_sizes:
            return
        size = allocation.height if pane.get_orientation() == Gtk.Orientation.VERTICAL else allocation.width
        if size > 100:
            self._split_sizes[pane] = size
            pane.set_position(round(size * ratio))

    def close(self, *_):
        self.maps.close()
        self.attitude_slot.close()

    def probe(self):
        """Diagnostic observation of displayed state; no receive/compute work."""
        panel, maps = self.attitude_slot, self.maps
        view = panel.view
        return dict(
            mapped=self.get_mapped(),
            slots_mapped=[slot.get_mapped() for slot in
                          (self.camera_slot, self.top_slot, self.oblique_slot, panel)],
            map_state=maps.state, map_displayed=maps.displayed_count,
            map_rasters=[maps.top.raster is not None, maps.oblique.raster is not None],
            map_messages=[maps.top.message, maps.oblique.message],
            map_timer=maps._timer is not None, map_renders=maps.worker.render_count,
            map_error=maps.worker.last_error,
            attitude_timer=panel._timer is not None, attitude_refreshes=panel.refresh_count,
            link=None if view is None else view.link_state,
            raw_pitch=None if view is None else view.raw_pitch.value,
            raw_pitch_state=None if view is None else view.raw_pitch.state,
            packet_age_s=None if view is None else view.packet_age_s,
            raw_label=panel.raw.get_text(), body_label=panel.body.get_text(),
            raw_source_label=panel.raw_source.get_text(),
            joints_label=panel.joints.get_text(), source_label=panel.joint_source.get_text(),
            joint_states=[] if view is None else [r.state for _, r in view.suspension],
            joint_values=[] if view is None else [r.value for _, r in view.suspension],
            wheel_labels={name:[label.get_text() for label in labels[1:]]
                          for name, labels in panel.wheel_labels.items()},
            wheel_states=[] if view is None else [w.speed.state for w in view.wheels],
        )

    def set_overlay_enabled(self, enabled: bool) -> None:
        if self.show_terrain.get_active() != enabled:
            self.show_terrain.set_active(enabled)
