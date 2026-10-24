"""Read-only attitude semantics. No mounting correction or synthetic physical readings."""
from __future__ import annotations

from dataclasses import dataclass
import math

from .telemetry import TelemetrySnapshot

TELEMETRY_MAX_AGE_S = 1.0
MAP_LINK_MAX_AGE_S = 0.5
PLANE_MAX_AGE_S = 1.0
WHEELS = (
    ("front_left", "앞좌"), ("front_right", "앞우"),
    ("mid_left", "중좌"), ("mid_right", "중우"),
    ("rear_left", "뒤좌"), ("rear_right", "뒤우"),
)
JOINTS = (
    ("ml_rocker_joint", "좌 로커"), ("mr_rocker_joint", "우 로커"),
    ("bl_bogie_joint", "좌 보기"), ("br_bogie_joint", "우 보기"),
)
STATE_TEXT = {"WAITING": "미수신", "STALE": "오래됨", "INVALID": "유효하지 않음",
              "AGE_UNKNOWN": "시간 미확인", "UNVERIFIED": "보정 미확인"}


def finite(value):
    return isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value)


def elapsed(received, now):
    if not finite(received) or not finite(now) or now < received:
        return None
    return now-received


@dataclass(frozen=True)
class Reading:
    value: float | None
    state: str
    age_s: float | None
    source: str
    age_basis: str

    def text(self, suffix="°", digits=1):
        if self.state != "LIVE":
            return STATE_TEXT.get(self.state, self.state)
        return f"{self.value:+.{digits}f}{suffix}"


def reading(value, age, source, *, max_age=TELEMETRY_MAX_AGE_S,
            stale=False, age_basis="패킷 수신 후", radians=False):
    if value is None:
        return Reading(None,"WAITING",age,source,age_basis)
    if not finite(value):
        return Reading(None,"INVALID",age,source,age_basis)
    if age is None:
        return Reading(None,"AGE_UNKNOWN",None,source,age_basis)
    if stale or age > max_age:
        return Reading(None,"STALE",age,source,age_basis)
    return Reading(math.degrees(value) if radians else value,"LIVE",age,source,age_basis)


@dataclass(frozen=True)
class WheelReading:
    name: str
    label: str
    steer: Reading
    speed: Reading
    fault: bool


@dataclass(frozen=True)
class StillRecord:
    body_pitch_deg: float
    body_roll_deg: float
    slope_pitch_deg: float
    slope_roll_deg: float
    observed_s: float


@dataclass(frozen=True)
class AttitudeView:
    link_state: str
    packet_age_s: float | None
    raw_roll: Reading
    raw_pitch: Reading
    body_state: str
    wheels: tuple[WheelReading, ...]
    suspension: tuple[tuple[str, Reading], ...]
    suspension_source: str
    suspension_frame: str
    suspension_age_s: float | None
    plane_pitch: Reading
    plane_roll: Reading
    still: StillRecord | None
    still_age_s: float | None
    map_link_state: str
    comparison: str


class AttitudeTracker:
    """Only the last still-window record is held, within its map session/epoch."""

    def __init__(self):
        self._map_key = None
        self._still = None

    def update(self, snapshot: TelemetrySnapshot | None, status, now_s: float,
               *, telemetry_port=5005, map_port=5010) -> AttitudeView:
        age = None if snapshot is None else elapsed(snapshot.received_monotonic_s,now_s)
        link = "WAITING" if snapshot is None else "AGE_UNKNOWN" if age is None else (
            "LIVE" if age <= TELEMETRY_MAX_AGE_S else "STALE")
        imu_source = f"UDP :{telemetry_port} · /imu/filtered 유래 · 장착 보정 미확인"
        roll = reading(None if snapshot is None else snapshot.roll_rad,age,imu_source,radians=True)
        pitch = reading(None if snapshot is None else snapshot.pitch_rad,age,imu_source,radians=True)
        raw_wheels = () if snapshot is None else snapshot.wheel_statuses
        wheels = []
        for name,label in WHEELS:
            candidates = [w for w in raw_wheels if w.name == name]
            wheel = candidates[0] if len(candidates)==1 else None
            source = f"UDP :{telemetry_port} · wheel_statuses.{name}"
            steer = reading(None if wheel is None else wheel.steer_deg,age,source,
                            stale=wheel is not None and wheel.stale)
            speed = reading(None if wheel is None else wheel.drive_turns_per_s,age,source,
                            stale=wheel is not None and wheel.stale)
            if len(candidates)>1:
                steer = speed = Reading(None,"INVALID",age,source,"패킷 수신 후")
            wheels.append(WheelReading(name,label,steer,speed,
                bool(wheel and (wheel.drive_axis_error or wheel.steer_fault or wheel.mode=="FAULT"))))
        values = {} if snapshot is None else dict(snapshot.suspension_joints_rad)
        source_name = None if snapshot is None else snapshot.suspension_source
        reference = None if snapshot is None else snapshot.suspension_frame
        source = f"UDP :{telemetry_port} · {source_name or ('출처 미표기' if values else '출처 미수신')}"
        measured_age = None if snapshot is None else snapshot.suspension_age_ms
        joint_age = (measured_age/1000+age if values and finite(measured_age) and measured_age>=0 and age is not None else None)
        joints = tuple((label,reading(values.get(key),joint_age,source,
            stale=link!="LIVE",age_basis="측정 후",radians=True)) for key,label in JOINTS)
        map_age = None if status is None else elapsed(status.received_s,now_s)
        map_link = "WAITING" if status is None else "AGE_UNKNOWN" if map_age is None else (
            "LIVE" if map_age<=MAP_LINK_MAX_AGE_S else "STALE")
        plane = None if status is None else status.plane_tilt
        plane_age = None if plane is None or map_age is None else plane.age_ms/1000+map_age
        plane_source = f"UDP :{map_port} · 기준면(차체 기준) · 실제 장착 보정 미확인"
        plane_pitch = reading(None if plane is None else plane.pitch_deg,plane_age,plane_source,
            stale=map_link!="LIVE",max_age=PLANE_MAX_AGE_S,age_basis="측정 후")
        plane_roll = reading(None if plane is None else plane.roll_deg,plane_age,plane_source,
            stale=map_link!="LIVE",max_age=PLANE_MAX_AGE_S,age_basis="측정 후")
        if status is not None:
            key = (status.session_key,status.map_epoch)
            if key != self._map_key:
                self._map_key, self._still = key, None
            gravity = status.gravity
            if gravity is not None and plane_age is not None and map_link=="LIVE" and plane_age<=PLANE_MAX_AGE_S:
                self._still = StillRecord(gravity.body_pitch_deg,gravity.body_roll_deg,
                    gravity.slope_pitch_deg,gravity.slope_roll_deg,now_s-plane_age)
        still_age = None if self._still is None else elapsed(self._still.observed_s,now_s)
        return AttitudeView(link,age,roll,pitch,"UNVERIFIED",tuple(wheels),joints,source,
            reference or "영점·부호 미확인",joint_age,plane_pitch,plane_roll,self._still,still_age,map_link,
            "비교 보류 · 좌표·보정·시각 미확인")
