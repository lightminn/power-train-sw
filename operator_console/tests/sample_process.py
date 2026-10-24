"""Read-only Linux console CPU/RSS/environment sampler; no video connections.

This records process metrics only. It cannot certify FPS, draw gaps or hardware.
"""
import argparse
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import time

from .vision_benchmark import machine, ac_online


def process(pid):
    base = Path('/proc')/str(pid)
    raw = (base/'stat').read_text()
    fields = raw[raw.rfind(')')+2:].split()
    return dict(ticks=int(fields[11])+int(fields[12]), start_ticks=int(fields[19]),
                threads=int(fields[17]), rss_kib=int((base/'statm').read_text().split()[1])*
                os.sysconf('SC_PAGE_SIZE')/1024)


def sample(pid, seconds, output, require_ac=False):
    if pid <= 0 or not .2 <= seconds <= 86400:
        raise ValueError('positive PID and .2..86400 seconds required')
    if output.exists():
        raise ValueError('output exists; choose a new filename')
    if not output.parent.is_dir():
        raise ValueError('output directory does not exist')
    started = time.monotonic()
    initial = previous = process(pid)
    before = machine()
    if require_ac and not ac_online(before):
        raise ValueError('online AC power required')
    previous_time = started
    rows = []
    reason = None
    try:
        while time.monotonic()-started < seconds:
            time.sleep(min(1., max(0., seconds-(time.monotonic()-started))))
            now = time.monotonic()
            observed = process(pid)
            if observed['start_ticks'] != initial['start_ticks']:
                reason = 'PID reused'
                break
            environment = machine()
            counts = {}
            for path in Path('/sys/devices/system/cpu').glob('cpu*/thermal_throttle/*throttle_count'):
                try:
                    counts[str(path)] = int(path.read_text())
                except (OSError, ValueError):
                    counts[str(path)] = None
            environment['thermal_throttle_counts'] = counts
            rows.append(dict(timestamp_utc=datetime.now(timezone.utc).isoformat(),
                elapsed_s=now-started,
                cpu_percent=(observed['ticks']-previous['ticks'])/
                            os.sysconf('SC_CLK_TCK')/(now-previous_time)*100,
                rss_kib=observed['rss_kib'], threads=observed['threads'], environment=environment))
            previous, previous_time = observed, now
            if require_ac and not ac_online(environment):
                reason = 'AC power disconnected'
                break
    except (OSError, ProcessLookupError) as exc:
        reason = f'process unavailable: {type(exc).__name__}'
    except KeyboardInterrupt:
        reason = 'interrupted'
    elapsed = time.monotonic()-started
    completed = reason is None and elapsed >= seconds
    report = dict(status='complete' if completed else 'incomplete', reason=reason,
        scope='process CPU/RSS/environment only; not FPS/draw/latency/hardware acceptance',
        pid=pid, process_start_ticks=initial['start_ticks'], requested_s=seconds, elapsed_s=elapsed,
        cpu_percent_mean=((previous['ticks']-initial['ticks'])/os.sysconf('SC_CLK_TCK')/
                          (previous_time-started)*100) if rows else None,
        rss_kib_max=max((r['rss_kib'] for r in rows), default=None),
        environment_before=before, samples=rows)
    output.write_text(json.dumps(report, indent=2, allow_nan=False)+'\n')
    return report


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--pid', type=int, required=True)
    parser.add_argument('--seconds', type=float, default=600)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--require-ac', action='store_true')
    args = parser.parse_args()
    try:
        report = sample(args.pid, args.seconds, args.output, args.require_ac)
    except (OSError, ValueError) as exc:
        parser.exit(2, str(exc)+'\n')
    print(json.dumps({k:v for k,v in report.items() if k not in ('samples','environment_before')}))
    return 0 if report['status']=='complete' else 1


if __name__ == '__main__':
    raise SystemExit(main())
