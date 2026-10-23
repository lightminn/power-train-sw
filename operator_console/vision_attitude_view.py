"""Compact GTK attitude panel driven solely by shared receiver snapshots."""
from __future__ import annotations

import math
import time

import gi

gi.require_version("Gtk", "3.0")
from gi.repository import GLib, Gtk, Pango  # noqa: E402

from .themes import theme_tokens
from .vision_attitude import AttitudeTracker, STATE_TEXT


def _label(text=""):
    label = Gtk.Label(label=text,xalign=0)
    label.set_ellipsize(Pango.EllipsizeMode.END)
    label.set_max_width_chars(1)
    label.set_hexpand(True)
    return label


def _set(label, text, tooltip=None):
    if label.get_text()!=text:
        label.set_text(text)
    label.set_tooltip_text(text if tooltip is None else tooltip)


def _age(age):
    return "미수신" if age is None else f"{age:.1f}초 전"


class ChassisSketch(Gtk.DrawingArea):
    """Wheel feedback schematic. Raw IMU is deliberately never a body rotation input."""

    def __init__(self):
        super().__init__()
        self.set_size_request(100,60)
        self.set_hexpand(True)
        self.set_vexpand(True)
        self.wheels = ()
        self.connect("draw",self._draw)
        self.set_tooltip_text("차체 개략도 · 기울기 미적용\n사각 바퀴 방향은 유효한 조향 피드백, ×는 미수신/오래됨입니다.")

    def set_wheels(self,wheels):
        self.wheels=wheels
        self.queue_draw()

    def _draw(self,_widget,cr):
        colors=theme_tokens()
        def source(name):
            color=colors[name].lstrip('#')
            cr.set_source_rgb(*(int(color[i:i+2],16)/255 for i in (0,2,4)))
        w,h=self.get_allocated_width(),self.get_allocated_height()
        cr.set_line_width(1.5)
        source("text_muted")
        cr.set_dash([4,3])
        cr.rectangle(w*.37,h*.20,w*.26,h*.60)
        cr.stroke()
        cr.set_dash([])
        for i,wheel in enumerate(self.wheels):
            x=w*(.25 if i%2==0 else .75)
            y=h*(.25+.25*(i//2))
            cr.save(); cr.translate(x,y)
            usable=wheel.steer.state=="LIVE"
            source("danger" if wheel.fault else "success" if usable and wheel.speed.state=="LIVE"
                   else "warning" if wheel.steer.state=="STALE" or wheel.speed.state=="STALE" else "offline")
            if usable:
                cr.rotate(-math.radians(wheel.steer.value))
                cr.rectangle(-5,-8,10,16); cr.stroke()
                cr.move_to(0,0); cr.line_to(0,-9); cr.stroke()
            else:
                cr.arc(0,0,7,0,math.tau); cr.stroke()
                cr.move_to(-4,-4); cr.line_to(4,4)
                cr.move_to(-4,4); cr.line_to(4,-4); cr.stroke()
            cr.restore()
        return False


class AttitudePanel(Gtk.Box):
    def __init__(self,receiver,map_status_provider,*,map_port):
        super().__init__(orientation=Gtk.Orientation.VERTICAL,spacing=2)
        self.get_style_context().add_class("vision-attitude")
        self.receiver=receiver
        self.port=receiver.port
        self.map_port=map_port
        self._map_status_provider=map_status_provider
        self.tracker=AttitudeTracker()
        self.view=None
        self.refresh_count=0
        self._timer=None
        self._closed=False
        self.body=_label("차체 자세: 보정 미확인 · 개략도 기울기 미적용")
        self.pack_start(self.body,False,False,0)
        main=Gtk.Box(spacing=8)
        left=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=1)
        self.raw=_label("IMU 원시 R 미수신 / P 미수신")
        self.raw_source=_label()
        self.sketch=ChassisSketch()
        left.pack_start(self.raw,False,False,0)
        left.pack_start(self.raw_source,False,False,0)
        left.pack_start(self.sketch,True,True,0)
        main.pack_start(left,True,True,0)
        right=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=1)
        table=Gtk.Grid(column_spacing=7,row_spacing=0)
        for col,title in enumerate(("바퀴","조향 °","속도 rev/s")):
            table.attach(_label(title),col,0,1,1)
        self.wheel_labels={}
        from .vision_attitude import WHEELS
        for row,(name,label) in enumerate(WHEELS,1):
            labels=(_label(label),_label("미수신"),_label("미수신"))
            for col,value in enumerate(labels):
                table.attach(value,col,row,1,1)
            self.wheel_labels[name]=labels
        right.pack_start(table,False,False,0)
        self.wheel_source=_label()
        right.pack_start(self.wheel_source,False,False,0)
        main.pack_start(right,True,True,0)
        self.pack_start(main,True,True,0)
        self.joint_source=_label()
        self.joints=_label("좌 로커 미수신 · 우 로커 미수신 · 좌 보기 미수신 · 우 보기 미수신")
        self.pack_start(self.joint_source,False,False,0)
        self.pack_start(self.joints,False,False,0)
        self.plane=_label()
        self.still=_label()
        self.pack_start(self.plane,False,False,0)
        self.pack_start(self.still,False,False,0)
        self.connect("map",self._mapped)
        self.connect("unmap",self._unmapped)
        self.connect("destroy",self.close)

    def _mapped(self,*_):
        if self._closed:
            return
        self._refresh()
        if self._timer is None:
            self._timer=GLib.timeout_add(200,self._refresh)

    def _unmapped(self,*_):
        if self._timer is not None:
            GLib.source_remove(self._timer)
            self._timer=None

    def _refresh(self):
        if self._closed:
            return False
        view=self.tracker.update(self.receiver.latest(),self._map_status_provider(),time.monotonic(),
            telemetry_port=self.port,map_port=self.map_port)
        self.view=view
        self.refresh_count+=1
        _set(self.raw,f"IMU 원시 R {view.raw_roll.text()} / P {view.raw_pitch.text()}",
             view.raw_roll.source+"\nIMU 추정 quaternion을 Euler로 변환한 값이며 장착 회전·영점·부호는 미확인입니다.\n"+view.comparison)
        _set(self.raw_source,f"UDP :{self.port} · 패킷 {_age(view.packet_age_s)} · "+
             ("수신 중" if view.link_state=="LIVE" else "수신 끊김" if view.link_state=="STALE"
              else STATE_TEXT.get(view.link_state,view.link_state)),
             view.raw_roll.source+f"\n원 IMU 측정 시각은 패킷에 없습니다. 거부 패킷 {self.receiver.invalid_packet_count}개")
        _set(self.wheel_source,f"패킷 {_age(view.packet_age_s)} · Hall<0.3 참고",
             f"UDP :{self.port} · 패킷 수신 후 {_age(view.packet_age_s)}\nHall 속도는 |값|<0.3 rev/s에서 실제 정지와 다를 수 있습니다. 수신값을 0이나 지령으로 바꾸지 않습니다.")
        for wheel in view.wheels:
            labels=self.wheel_labels[wheel.name]
            _set(labels[0],wheel.label+(" !" if wheel.fault else ""))
            for label,value,suffix,digits in ((labels[1],wheel.steer,"",1),(labels[2],wheel.speed,"",2)):
                _set(label,value.text(suffix,digits),value.source+f"\n{value.age_basis} {_age(value.age_s)} · {value.state}")
        self.sketch.set_wheels(view.wheels)
        joint_status = ("선택 각도 필드 수신" if any(r.state!="WAITING" for _,r in view.suspension)
                        else "엔코더 각도 미수신 · 송신 경로 연결 대기")
        _set(self.joint_source,f"로커·보기 · 측정 {_age(view.suspension_age_s)} · {view.suspension_source}",
             view.suspension_source+f"\n기준: {view.suspension_frame}\n{joint_status}\n부호·영점 합의 전 값은 차체 자세에 적용하지 않습니다.")
        _set(self.joints," · ".join(f"{name} {value.text()}" for name,value in view.suspension),
             "\n".join(f"{name}: {value.text()} · {value.source} · {value.age_basis} {_age(value.age_s)}" for name,value in view.suspension)+f"\n기준: {view.suspension_frame}")
        _set(self.plane,f"지도 기준면 앞 {view.plane_pitch.text()} / 옆 {view.plane_roll.text()} · {_age(view.plane_pitch.age_s)}",
             view.plane_pitch.source+"\n기준면 기울기는 차체 자세가 아닙니다. "+view.comparison)
        if view.still is None:
            text="정지 기록: 미수신 · 비교 보류(좌표·보정 미확인)"
            tip=f"UDP :{self.map_port} · gravity · 정지 창 측정\n원시 IMU와 기준·부호·시각이 달라 차이 계산을 보류합니다."
        else:
            text=(f"정지 기록 앞 {view.still.body_pitch_deg:+.1f}° / 좌 {view.still.body_roll_deg:+.1f}°"
                  f" · {_age(view.still_age_s)} · 비교 보류")
            tip=(f"UDP :{self.map_port} · gravity · 정지 창 기록(현재 주행 자세 아님)\n"
                 f"측정 {_age(view.still_age_s)} · 지도 수신 {view.map_link_state}\n"
                 f"기준면 경사 앞 {view.still.slope_pitch_deg:+.1f}° / 옆 {view.still.slope_roll_deg:+.1f}°\n"
                 "차체 값은 앞/왼쪽 들림이며 Euler 쌍이 아닙니다. "+view.comparison)
        _set(self.still,text,tip)
        return True

    def close(self,*_):
        self._closed=True
        self._unmapped()
