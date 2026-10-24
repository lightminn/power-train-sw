import json
from pathlib import Path
import subprocess
import sys

from operator_console.tests.sample_process import sample


def test_sampler_observes_process_without_stopping_it(tmp_path):
    target = subprocess.Popen([sys.executable, '-c', 'import time; time.sleep(10)'])
    try:
        report = sample(target.pid, .25, tmp_path/'sample.json')
        assert target.poll() is None
        assert report['status']=='complete'
        assert report['elapsed_s']>=.25
        assert report['samples'] and report['rss_kib_max']>0
        assert report['cpu_percent_mean']>=0
        assert json.loads((tmp_path/'sample.json').read_text())['pid']==target.pid
        assert 'not FPS/draw/latency/hardware acceptance' in report['scope']
    finally:
        target.terminate(); target.wait()


def test_sampler_does_not_invent_values_after_process_exit(tmp_path, monkeypatch):
    from operator_console.tests import sample_process
    target = subprocess.Popen([sys.executable, '-c', 'import sys; sys.stdin.read()'], stdin=subprocess.PIPE)
    try:
        original = sample_process.process
        def observe_then_exit(pid):
            value = original(pid)
            target.stdin.close()
            target.wait()
            return value
        monkeypatch.setattr(sample_process, 'process', observe_then_exit)
        report = sample(target.pid, .5, tmp_path/'sample.json')
        assert report['status']=='incomplete'
        assert report['cpu_percent_mean'] is None
        assert report['rss_kib_max'] is None
        assert not report['samples']
    finally:
        target.wait()
