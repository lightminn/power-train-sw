"""Explicitly reviewed gravity rotation, disabled until a real calibration is supplied."""
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path


@dataclass(frozen=True)
class Calibration:
    confirmed: bool = False
    quaternion: tuple = ()
    roll_sign: int = 1
    pitch_sign: int = 1
    reference: str = '미확인'
    sha256: str = ''
    compare_map: bool = False

    def body_up(self, roll, pitch):
        if not self.confirmed:
            return None
        r,p = roll*self.roll_sign,pitch*self.pitch_sign
        u = (-math.sin(p),math.sin(r)*math.cos(p),math.cos(r)*math.cos(p))
        x,y,z,w = self.quaternion
        matrix = ((1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)),
                  (2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)),
                  (2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)))
        return tuple(sum(a*b for a,b in zip(row,u)) for row in matrix)


def resolve_calibration(path=None, *, config=None, config_dir=None, environ=None):
    config = config or {}
    environ = os.environ if environ is None else environ
    value = path if path is not None else config.get('attitude_calibration')
    if value is None:
        value = environ.get('L515_ATTITUDE_CALIBRATION')
    if value is None:
        return Calibration()
    if not isinstance(value,(str,Path)) or not str(value).strip():
        raise ValueError('attitude_calibration: JSON path required')
    file = Path(value).expanduser()
    if not file.is_absolute() and config_dir and path is None and config.get('attitude_calibration') is not None:
        file = Path(config_dir)/file
    if file.stat().st_size>16384:
        raise ValueError('attitude calibration file too large')
    raw = file.read_bytes()
    try:
        data = json.loads(raw)
    except (ValueError,UnicodeError,RecursionError) as exc:
        raise ValueError("attitude calibration must be valid JSON") from exc
    if not isinstance(data,dict) or type(data.get('schema_version')) is not int or data['schema_version']!=1:
        raise ValueError('attitude calibration schema_version must be 1')
    if type(data.get('confirmed')) is not bool:
        raise ValueError('attitude calibration confirmed must be boolean')
    digest = hashlib.sha256(raw).hexdigest()
    if not data['confirmed']:
        return Calibration(sha256=digest)
    for key in ('confirmed_by','reference','source_basis','target_frame'):
        if not isinstance(data.get(key),str) or not data[key].strip() or len(data[key])>160 or any(ord(c)<32 for c in data[key]):
            raise ValueError(f'attitude calibration {key} required')
    if data['source_basis']!='udp5005_imu_filtered_euler_xyz' or data['target_frame']!='base_link':
        raise ValueError('attitude calibration source/target basis unsupported')
    q = data.get('sensor_to_body_xyzw')
    try:
        valid = isinstance(q,list) and len(q)==4 and all(type(v) in (int,float) and math.isfinite(v) for v in q)
    except OverflowError:
        valid = False
    if not valid:
        raise ValueError('attitude calibration finite xyzw quaternion required')
    if abs(sum(v*v for v in q)-1)>1e-4:
        raise ValueError('attitude calibration quaternion must be unit length')
    for key in ('raw_roll_sign','raw_pitch_sign'):
        if type(data.get(key)) is not int or data[key] not in (-1,1):
            raise ValueError(f'attitude calibration {key} must be +1/-1')
    compare = data.get('map_frame_confirmed',False)
    if type(compare) is not bool:
        raise ValueError('map_frame_confirmed must be boolean')
    if compare and (not isinstance(data.get('map_reference'),str) or not data['map_reference'].strip()):
        raise ValueError('map_reference required before comparing map coordinates')
    return Calibration(True,tuple(q),data['raw_roll_sign'],data['raw_pitch_sign'],
                       data['reference']+' · '+data['confirmed_by'],digest,compare)
