#!/usr/bin/env python3
"""Jetson 자원 지표 — `/proc`, `/sys` 를 직접 읽는다 (의존성 0).

서멀 스로틀링은 현장에서 YOLO FPS 가 떨어지는 1순위 원인인데 지금 아무도
보지 않는다. psutil 같은 새 의존성 없이 stdlib 파일 읽기로 충분하다.

읽기에 실패해도 절대 예외를 올리지 않는다 — 모니터가 자기 부수 기능 때문에
죽으면 안 되고, Jetson 이 아닌 개발 머신에서는 없는 경로도 있다.
"""

import glob
import os

_prev_cpu = {'total': None, 'idle': None}


def _read(path):
    try:
        with open(path, 'r') as fh:
            return fh.read()
    except OSError:
        return None


def cpu_percent():
    """`/proc/stat` 두 시점 차분. 첫 호출은 None(기준선만 잡는다)."""
    raw = _read('/proc/stat')
    if not raw:
        return None
    line = raw.split('\n', 1)[0].split()
    if len(line) < 5 or line[0] != 'cpu':
        return None
    values = [int(v) for v in line[1:]]
    total = sum(values)
    idle = values[3] + (values[4] if len(values) > 4 else 0)

    prev_total, prev_idle = _prev_cpu['total'], _prev_cpu['idle']
    _prev_cpu['total'], _prev_cpu['idle'] = total, idle
    if prev_total is None or total <= prev_total:
        return None
    d_total = total - prev_total
    d_idle = idle - prev_idle
    return round(100.0 * (d_total - d_idle) / d_total, 1)


def memory():
    raw = _read('/proc/meminfo')
    if not raw:
        return None
    fields = {}
    for line in raw.split('\n'):
        parts = line.split(':')
        if len(parts) == 2:
            fields[parts[0]] = parts[1].strip().split()[0]
    try:
        total = int(fields['MemTotal']) / 1024.0
        avail = int(fields['MemAvailable']) / 1024.0
    except (KeyError, ValueError, IndexError):
        return None
    return {'total_mb': round(total), 'used_mb': round(total - avail),
            'percent': round(100.0 * (total - avail) / total, 1)}


def thermal():
    """thermal_zone 별 온도 [°C]. Jetson 은 CPU/GPU/SOC 등이 여러 존으로 나뉜다."""
    zones = []
    for path in sorted(glob.glob('/sys/class/thermal/thermal_zone*')):
        raw = _read(os.path.join(path, 'temp'))
        if not raw or not raw.strip().lstrip('-').isdigit():
            continue
        name = (_read(os.path.join(path, 'type')) or os.path.basename(path)).strip()
        value = int(raw.strip())
        # 커널은 보통 밀리도(m°C)로 준다. 1000 미만이면 이미 °C 로 본다.
        zones.append({'name': name,
                      'celsius': round(value / 1000.0, 1) if abs(value) >= 1000 else value})
    return zones


def snapshot():
    try:
        return {'cpu_percent': cpu_percent(), 'memory': memory(), 'thermal': thermal()}
    except Exception:  # noqa: BLE001 — 부수 기능이 모니터를 죽이면 안 된다
        return {'cpu_percent': None, 'memory': None, 'thermal': []}
