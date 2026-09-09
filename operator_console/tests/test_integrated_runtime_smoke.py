"""The smoke must observe a real absent-pad child, not supervisor fallback."""
import json
import os
import selectors
import shlex
import subprocess
import sys

import pytest

from operator_console import integrated_runtime_smoke as smoke


@pytest.mark.parametrize('pad, expected', [
    ({}, False),
    ({'pad_connected': False, 'age_s': .1, 'detail': '패드 프로세스 재연결 대기'}, False),
    ({'pad_connected': False, 'age_s': .1, 'detail': 'pygame unavailable: missing'}, False),
    ({'pad_connected': False, 'age_s': 1.1, 'detail': 'waiting for gamepad'}, False),
    ({'pad_connected': False, 'age_s': .1, 'detail': 'waiting for gamepad'}, True),
])
def test_smoke_requires_fresh_child_origin_status(pad, expected):
    assert smoke._child_status_received({'operation': {'pad': pad}}) is expected


def test_smoke_cli_forwards_separate_controller_interpreter(monkeypatch):
    seen = []
    monkeypatch.setattr(smoke, 'run_smoke', lambda **kwargs: (seen.append(kwargs) or True, 'fixture'))
    monkeypatch.setattr(sys, 'argv', ['smoke', '--controller-python', '/chosen/python'])
    assert smoke.main() == 0
    assert seen == [{'controller_python': '/chosen/python'}]


def test_smoke_controller_wrapper_runs_real_child_with_no_device_enumeration(tmp_path):
    pytest.importorskip('pygame', reason='Run controller wrapper gate with pygame-enabled Python')
    wrapper = smoke._controller_fixture(tmp_path, sys.executable)
    env = dict(os.environ, XDG_RUNTIME_DIR=str(tmp_path), SDL_VIDEODRIVER='dummy',
               SDL_AUDIODRIVER='dummy', PYGAME_HIDE_SUPPORT_PROMPT='1')
    child = subprocess.Popen([str(wrapper), '-m', 'motor_control.laptop.controller_process'],
                             env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True)
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(child.stdout, selectors.EVENT_READ)
            assert selector.select(5), 'no controller status received'
            status = json.loads(child.stdout.readline())
        assert status['detail'] == 'waiting for gamepad'
        assert not status['pad_connected'] and not status['input_connected']
        child.stdin.close()
        assert child.wait(timeout=5) == 0
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=3)


@pytest.mark.parametrize('initialization', ['pygame.init', 'pygame.joystick.init'])
def test_smoke_wrapper_blocks_sdl_initialization_before_real_child_loop(tmp_path, initialization):
    pytest.importorskip('pygame', reason='Run sentinel gate with pygame-enabled Python')
    # The interpreter shim installs sentinels BEFORE the generated wrapper runs.
    # Neither RED nor GREEN may call the real SDL initializer or inspect devices.
    bootstrap = """import pygame, sys
pygame.init = lambda: None
pygame.joystick.init = lambda: None
def forbidden_init():
    raise RuntimeError('unisolated SDL initialization: INITIALIZATION')
INITIALIZATION = forbidden_init
assert sys.argv[1] == '-c'
code = sys.argv[2]
sys.argv = [sys.argv[0]] + sys.argv[3:]
exec(compile(code, '<generated-smoke-wrapper>', 'exec'))
""".replace('INITIALIZATION', initialization)
    interpreter = tmp_path / 'sentinel-python'
    interpreter.write_text('#!/bin/sh\nexec ' + shlex.quote(sys.executable)
                           + ' -c ' + shlex.quote(bootstrap) + ' "$@"\n')
    interpreter.chmod(0o700)
    wrapper = smoke._controller_fixture(tmp_path, str(interpreter))
    env = dict(os.environ, XDG_RUNTIME_DIR=str(tmp_path), PYGAME_HIDE_SUPPORT_PROMPT='1')
    child = subprocess.Popen([str(wrapper), '-m', 'motor_control.laptop.controller_process'],
                             env=env, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.PIPE, text=True)
    try:
        with selectors.DefaultSelector() as selector:
            selector.register(child.stdout, selectors.EVENT_READ)
            assert selector.select(5), 'no controller status received'
            line = child.stdout.readline()
        assert line, child.stderr.read()
        status = json.loads(line)
        assert status['detail'] == 'waiting for gamepad', status
        assert status['pad_connected'] is False and status['input_connected'] is False
        child.stdin.close()
        assert child.wait(timeout=5) == 0
    finally:
        if child.poll() is None:
            child.kill()
            child.wait(timeout=3)
