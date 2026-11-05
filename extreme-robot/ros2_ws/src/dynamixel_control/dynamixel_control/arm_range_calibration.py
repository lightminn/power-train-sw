"""GUI range calibration for arm joints 1-4: measure -> verify -> apply -> save.

Same method as scripts/measure_joint_limits.py, driven from the console instead
of a terminal: while measuring, the bridge feeds every joint reading here and
the running min/max is kept; the operator moves each joint slowly to both hard
stops (by hand in freedrive, or by jog with limits OFF). The margin is taken
off the hard stops so the servo's settling error never hits them.

Only the range step exists. Changing the zero (center) or gear ratio would shift
every saved pose, slot teaching and the 3D mapping, so those stay script-only.
Joint 5 is excluded: its jog limit is the separate zero +-114 tick window.

This module holds no ROS or bus code so the rules are testable on their own.
"""
from __future__ import annotations

import json
import math
import os
import time

AXES = ('arm_joint_1', 'arm_joint_2', 'arm_joint_3', 'arm_joint_4')
MARGIN_RAD = math.radians(2.0)
# A joint that moved less than this was not swept; it keeps its current limits.
MIN_SWEEP_RAD = math.radians(5.0)

IDLE, RUNNING, DONE, FAILED = 'IDLE', 'RUNNING', 'DONE', 'FAILED'


def _deg(rad):
    return f'{math.degrees(rad):+.1f}°'


def _describe(limits):
    return ' / '.join(f'{axis[-1]}축 {_deg(lo)}~{_deg(hi)}'
                      for axis, (lo, hi) in sorted(limits.items()))


class ArmRangeCalibration:
    def __init__(self, save_path, clock=time.time):
        self.save_path = save_path
        self._clock = clock
        self.status = IDLE
        self.recording = False
        self._low = {}
        self._high = {}
        self.measured = {}
        self.verified = {}
        self.applied = False
        self.saved = False
        self.detail = ''

    # -- measure ------------------------------------------------------------
    def observe(self, names, positions):
        if not self.recording:
            return
        for name, value in zip(names, positions):
            if name in AXES and math.isfinite(value):
                self._low[name] = min(self._low.get(name, value), value)
                self._high[name] = max(self._high.get(name, value), value)

    def measure(self):
        """Toggle: the first press starts recording, the second ends it."""
        if not self.recording:
            self.recording = True
            self._low, self._high = {}, {}
            self.measured, self.verified = {}, {}
            self.applied = self.saved = False
            self.status = RUNNING
            self.detail = ('측정 중 — 각 축을 양쪽 끝까지 천천히 움직인 뒤 측정을 한 번 더 누르세요. '
                           '하드스톱에 세게 부딪히지 마세요.')
            return
        self.recording = False
        swept = {axis: (self._low[axis], self._high[axis]) for axis in self._low
                 if self._high[axis] - self._low[axis] >= MIN_SWEEP_RAD}
        if not swept:
            self.status = FAILED
            self.detail = (f'{math.degrees(MIN_SWEEP_RAD):.0f}° 이상 움직인 축이 없습니다 — '
                           '측정을 다시 시작하세요.')
            return
        self.measured = swept
        self.status = DONE
        skipped = [axis[-1] for axis in AXES if axis not in swept]
        self.detail = '측정 완료.' + (
            f' {",".join(skipped)}축은 움직이지 않아 기존 범위를 유지합니다.' if skipped else '')

    # -- verify -------------------------------------------------------------
    def verify(self, poses):
        """Turn the sweep into limits and check every saved pose still fits.

        A saved pose outside the new range almost always means the joint was not
        pushed all the way to its stop (2026-08-07 bench lesson), so it fails.
        """
        if self.recording or not self.measured:
            raise ValueError('측정 결과 없음')
        limits = {axis: (lo + MARGIN_RAD, hi - MARGIN_RAD)
                  for axis, (lo, hi) in self.measured.items()}
        outside = []
        for name, joints in sorted((poses or {}).items()):
            for axis, (lo, hi) in limits.items():
                value = joints.get(axis)
                if value is not None and not lo <= value <= hi:
                    outside.append(f'{name}의 {axis[-1]}축 {_deg(value)}')
        if outside:
            self.verified = {}
            self.status = FAILED
            self.detail = ('저장 자세가 새 범위 밖입니다 — 끝까지 밀지 않았을 수 있습니다: '
                           + ', '.join(outside[:4]))
            return
        self.verified = limits
        self.applied = self.saved = False
        self.status = DONE
        self.detail = f'검증 통과(하드스톱에서 {math.degrees(MARGIN_RAD):.0f}° 여유를 뺌).'

    # -- apply / save -------------------------------------------------------
    def apply_to(self, table):
        """Write the verified limits into a joint_limits.JOINT_LIMITS-like dict."""
        if not self.verified:
            raise ValueError('검증 결과 없음')
        stamp = time.strftime('%Y-%m-%d', time.localtime(self._clock()))
        for axis, (lo, hi) in self.verified.items():
            entry = table.setdefault(axis, {})
            entry.update(lower=lo, upper=hi, confidence='measured',
                         source=f'{stamp} GUI 가동범위 측정 (여유 2°)')
        self.applied = True
        self.detail = '임시 적용됨 — 브리지를 재시작하면 저장된 값(또는 기본값)으로 돌아갑니다.'

    def save(self):
        if not self.verified:
            raise ValueError('검증 결과 없음')
        if not self.save_path:
            raise ValueError('저장 파일이 설정되지 않음(arm_joint_limits_file)')
        data = load_file(self.save_path)
        stamp = time.strftime('%Y-%m-%d %H:%M', time.localtime(self._clock()))
        for axis, (lo, hi) in self.verified.items():
            data[axis] = {'lower': lo, 'upper': hi, 'measured_at': stamp,
                          'margin_deg': math.degrees(MARGIN_RAD)}
        tmp = self.save_path + '.tmp'
        with open(tmp, 'w') as f:
            json.dump(data, f, indent=2, ensure_ascii=False, sort_keys=True)
        os.replace(tmp, self.save_path)
        self.saved = True
        self.detail = f'저장됨 — 다음 기동부터 적용됩니다 ({self.save_path}).'

    def handle(self, action, table, poses):
        if action == 'measure':
            self.measure()
        elif action == 'verify':
            self.verify(poses)
        elif action == 'apply_temporary':
            self.apply_to(table)
        elif action == 'save':
            self.save()
        else:
            raise ValueError(f'지원하지 않는 동작: {action}')

    def snapshot(self):
        return {
            'status': self.status,
            'recording': self.recording,
            'measured': _describe({a: self.measured[a] for a in self.measured}),
            'verified': _describe(self.verified),
            'applied': self.applied,
            'saved': self.saved,
            'detail': self.detail,
        }


def load_file(path):
    if not path or not os.path.isfile(path):
        return {}
    with open(path) as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError('limits file must hold an object')
    return data


def apply_saved(path, table):
    """Load saved GUI limits into the table at startup. Returns the axes changed."""
    changed = []
    for axis, entry in load_file(path).items():
        if axis not in AXES:
            continue
        lo, hi = float(entry['lower']), float(entry['upper'])
        if not (math.isfinite(lo) and math.isfinite(hi) and lo < hi):
            raise ValueError(f'invalid saved limits for {axis}')
        table.setdefault(axis, {}).update(
            lower=lo, upper=hi, confidence='measured',
            source=f"{entry.get('measured_at', '?')} GUI 가동범위 측정 (저장 파일)")
        changed.append(axis)
    return changed
