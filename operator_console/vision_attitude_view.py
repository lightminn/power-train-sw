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


def _detail_label(text=""):
    label = Gtk.Label(label=text, xalign=0)
    label.set_line_wrap(True)
    label.set_line_wrap_mode(Pango.WrapMode.WORD_CHAR)
    label.set_max_width_chars(40)
    label.set_hexpand(True)
    return label


def _set(label, text, tooltip=None):
    if label.get_text()!=text:
        label.set_text(text)
    label.set_tooltip_text(text if tooltip is None else tooltip)


def _tone(label, state):
    context = label.get_style_context()
    target = ("vision-value-fault" if state == "FAULT" else
              "vision-value-live" if state == "LIVE" else
              "vision-value-stale" if state == "STALE" else "vision-value-muted")
    if context.has_class(target):
        return
    for name in ("vision-value-live", "vision-value-stale", "vision-value-fault", "vision-value-muted"):
        context.remove_class(name)
    context.add_class(target)


def _age(age):
    return "미수신" if age is None else f"{age:.1f}초 전"


class ChassisSketch(Gtk.DrawingArea):
    """Wheel feedback schematic. Raw IMU is deliberately never a body rotation input."""

    def __init__(self):
        super().__init__()
        self.set_size_request(140,112)
        self.set_hexpand(False)
        self.set_vexpand(False)
        self.set_valign(Gtk.Align.START)
        self.get_style_context().add_class("vision-chassis-sketch")
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
        for axle_y in (.25, .50, .75):
            cr.move_to(w*.16,h*axle_y)
            cr.line_to(w*.84,h*axle_y)
            cr.stroke()
        source("surface_raised")
        cr.move_to(w*.41,h*.12)
        cr.line_to(w*.59,h*.12)
        cr.line_to(w*.70,h*.27)
        cr.line_to(w*.70,h*.91)
        cr.line_to(w*.30,h*.91)
        cr.line_to(w*.30,h*.27)
        cr.close_path()
        cr.fill_preserve()
        source("text_muted")
        cr.stroke()
        source("primary")
        cr.move_to(w*.50,h*.02)
        cr.line_to(w*.42,h*.11)
        cr.line_to(w*.58,h*.11)
        cr.close_path()
        cr.fill()
        for i,wheel in enumerate(self.wheels):
            x=w*(.17 if i%2==0 else .83)
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
    def __init__(self,receiver,map_status_provider,*,map_port,calibration=None):
        super().__init__(orientation=Gtk.Orientation.VERTICAL,spacing=5)
        self.get_style_context().add_class("vision-attitude")
        self.set_border_width(4)
        self.receiver=receiver
        self.port=receiver.port
        self.map_port=map_port
        self._map_status_provider=map_status_provider
        self.tracker=AttitudeTracker(calibration)
        self.view=None
        self.refresh_count=0
        self._timer=None
        self._closed=False
        top_space=Gtk.Box()
        self.pack_start(top_space,True,True,0)
        summary = Gtk.Box(spacing=10)
        summary.set_valign(Gtk.Align.START)
        text_column = Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=4)
        text_column.set_valign(Gtk.Align.START)
        self.body=_detail_label("보정 미확인 · 개략도 기울기 미적용")
        self.body.get_style_context().add_class("vision-attitude-heading")
        text_column.pack_start(self.body,False,False,0)
        self.raw=_label("IMU 원시 R 미수신 / P 미수신")
        self.raw_source=_label()
        self.raw_source.get_style_context().add_class("vision-attitude-source")
        self.sketch=ChassisSketch()
        summary.pack_start(self.sketch,False,False,0)
        summary.pack_start(text_column,True,True,0)
        self.pack_start(summary,False,False,0)
        text_column.pack_start(self.raw,False,False,0)
        table=Gtk.Grid(column_spacing=7,row_spacing=3)
        table.set_tooltip_text("속도 단위: rev/s. Hall 값은 절댓값 0.3 rev/s 미만에서 실제 정지와 다를 수 있습니다.")
        for col,title in enumerate(("좌 바퀴","조향 °","속도", "우 바퀴","조향 °","속도")):
            heading=_label(title)
            heading.get_style_context().add_class("vision-attitude-table-head")
            table.attach(heading,col,0,1,1)
        self.wheel_labels={}
        from .vision_attitude import WHEELS
        for index,(name,label) in enumerate(WHEELS):
            labels=(_label(label),_label("미수신"),_label("미수신"))
            for value in labels:
                value.get_style_context().add_class("vision-attitude-table-value")
            for col,value in enumerate(labels):
                table.attach(value,(index % 2)*3+col,index//2+1,1,1)
            self.wheel_labels[name]=labels
        self.wheel_summary=_label("바퀴 측정 대기")
        text_column.pack_start(self.wheel_summary,False,False,0)
        measurements=Gtk.Expander(label="바퀴 6개 · 자세 상세 보기")
        self._measurements=measurements
        measurement_body=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=5)
        measurements.add(measurement_body)
        measurement_body.pack_start(table,False,False,0)
        self.pack_start(measurements,False,False,0)
        bottom_space=Gtk.Box()
        self.pack_end(bottom_space,True,True,0)
        self.wheel_source=_label()
        self.wheel_source.get_style_context().add_class("vision-attitude-source")
        details=Gtk.Expander(label="로커·보기 / 지도 기준면")
        self._details=details
        details.get_style_context().add_class("vision-attitude-details")
        detail_body=Gtk.Box(orientation=Gtk.Orientation.VERTICAL,spacing=5)
        details.add(detail_body)
        measurement_body.pack_start(details,False,False,0)
        joint_title=_label("로커 · 보기 각도")
        joint_title.get_style_context().add_class("vision-attitude-section")
        detail_body.pack_start(joint_title,False,False,0)
        self.joint_source=_label()
        self.joint_source.get_style_context().add_class("vision-attitude-source")
        self.joints=_detail_label("좌 로커 미수신 · 우 로커 미수신\n좌 보기 미수신 · 우 보기 미수신")
        detail_body.pack_start(self.joint_source,False,False,0)
        detail_body.pack_start(self.joints,False,False,0)
        map_title=_label("지도 기준면 · 정지 기록")
        map_title.get_style_context().add_class("vision-attitude-section")
        detail_body.pack_start(map_title,False,False,0)
        self.plane=_detail_label()
        self.still=_detail_label()
        detail_body.pack_start(self.plane,False,False,0)
        detail_body.pack_start(self.still,False,False,0)
        self._initial_details_set=False
        self.connect("size-allocate",self._initial_details)
        self.connect("map",self._mapped)
        self.connect("unmap",self._unmapped)
        self.connect("destroy",self.close)

    def _initial_details(self,_widget,allocation):
        if self._initial_details_set or allocation.width<600 or allocation.height<380:
            return
        self._initial_details_set=True
        self._measurements.set_expanded(True)
        self._details.set_expanded(True)
        self._measurements.get_child().show_all()
        self._details.get_child().show_all()

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
        body_text = "보정 미확인 · 개략도 기울기 미적용"
        if view.body_state!='UNVERIFIED':
            body_text = (f"차체 R {view.body_roll.text()} / P {view.body_pitch.text()} · 설정 보정"
                         if view.body_roll is not None and view.body_pitch is not None
                         else "차체 자세: "+STATE_TEXT.get(view.body_state,view.body_state))
        _set(self.body,body_text,view.calibration_reference+"\n"+view.comparison)
        self.refresh_count+=1
        _set(self.raw,f"IMU R {view.raw_roll.text()} · P {view.raw_pitch.text()} · {_age(view.packet_age_s)}",
             view.raw_roll.source+"\nIMU 추정 quaternion을 Euler로 변환한 값이며 장착 회전·영점·부호는 미확인입니다.\n"+view.comparison)
        _tone(self.raw, view.link_state)
        link_text = ("수신 중" if view.link_state == "LIVE" else "수신 끊김" if view.link_state == "STALE"
                     else STATE_TEXT.get(view.link_state, view.link_state))
        _set(self.raw_source, f"{link_text} · 패킷 {_age(view.packet_age_s)} · UDP :{self.port}",
             view.raw_roll.source+f"\n원 IMU 측정 시각은 패킷에 없습니다. 거부 패킷 {self.receiver.invalid_packet_count}개")
        _tone(self.raw_source, view.link_state)
        _set(self.wheel_source,f"패킷 {_age(view.packet_age_s)} · Hall<0.3 참고",
             f"UDP :{self.port} · 패킷 수신 후 {_age(view.packet_age_s)}\nHall 속도는 |값|<0.3 rev/s에서 실제 정지와 다를 수 있습니다. 수신값을 0이나 지령으로 바꾸지 않습니다.")
        for wheel in view.wheels:
            labels=self.wheel_labels[wheel.name]
            _set(labels[0],wheel.label+(" · 오류" if wheel.fault else ""))
            _tone(labels[0], "FAULT" if wheel.fault else "LIVE")
            for label,value,suffix,digits in ((labels[1],wheel.steer,"",1),(labels[2],wheel.speed,"",2)):
                _set(label,value.text(suffix,digits),value.source+f"\n{value.age_basis} {_age(value.age_s)} · {value.state}")
                _tone(label, "FAULT" if wheel.fault else value.state)
        live_wheels = sum(w.speed.state == "LIVE" and w.steer.state == "LIVE" and not w.fault
                          for w in view.wheels)
        faults = sum(bool(w.fault) for w in view.wheels)
        _set(self.wheel_summary, f"바퀴 {live_wheels}/6 정상 수신" +
             (f" · 오류 {faults}" if faults else "") +
             (" · 나머지 미수신/지연" if live_wheels + faults < 6 else ""))
        _tone(self.wheel_summary, "FAULT" if faults else "LIVE" if live_wheels == 6 else "STALE")
        self.sketch.set_wheels(view.wheels)
        joint_status = ("선택 각도 필드 수신" if any(r.state!="WAITING" for _,r in view.suspension)
                        else "엔코더 각도 미수신 · 송신 경로 연결 대기")
        _set(self.joint_source,f"로커·보기 · 측정 {_age(view.suspension_age_s)} · {view.suspension_source}",
             view.suspension_source+f"\n기준: {view.suspension_frame}\n{joint_status}\n부호·영점 합의 전 값은 차체 자세에 적용하지 않습니다.")
        _set(self.joints,"\n".join(" · ".join(f"{name} {value.text()}" for name,value in view.suspension[start:start+2])
                                  for start in (0, 2)),
             "\n".join(f"{name}: {value.text()} · {value.source} · {value.age_basis} {_age(value.age_s)}" for name,value in view.suspension)+f"\n기준: {view.suspension_frame}")
        _set(self.plane,f"지도 기준면 앞 {view.plane_pitch.text()} / 옆 {view.plane_roll.text()} · {_age(view.plane_pitch.age_s)}",
             view.plane_pitch.source+"\n기준면 기울기는 차체 자세가 아닙니다. "+view.comparison)
        if view.still is None:
            text="정지 기록: 미수신 · 비교 보류(좌표·보정 미확인)"
            tip=f"UDP :{self.map_port} · gravity · 정지 창 측정\n원시 IMU와 기준·부호·시각이 달라 차이 계산을 보류합니다."
        else:
            text=(f"정지 기록 앞 {view.still.body_pitch_deg:+.1f}° / 좌 {view.still.body_roll_deg:+.1f}°"
                  f" · {_age(view.still_age_s)} · {view.comparison}")
            tip=(f"UDP :{self.map_port} · gravity · 정지 창 기록(현재 주행 자세 아님)\n"
                 f"측정 {_age(view.still_age_s)} · 지도 수신 {view.map_link_state}\n"
                 f"기준면 경사 앞 {view.still.slope_pitch_deg:+.1f}° / 옆 {view.still.slope_roll_deg:+.1f}°\n"
                 "차체 값은 앞/왼쪽 들림이며 Euler 쌍이 아닙니다. "+view.comparison)
        _set(self.still,text,tip)
        return True

    def close(self,*_):
        self._closed=True
        self._unmapped()
