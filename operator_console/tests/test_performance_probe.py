import pytest
from operator_console.performance_probe import Events


def test_no_event_is_not_a_zero_fps_or_zero_delay_measurement():
    assert Events().snapshot(100)==dict(count=0,hz=None,max_gap_s=None)


def test_observed_rate_and_silent_tail_are_both_measured():
    events=Events()
    for t in (10.,10.1,10.2):
        events.add(t)
    result=events.snapshot(10.9)
    assert result['hz']==pytest.approx(10)
    assert result['max_gap_s']==pytest.approx(.7)


@pytest.mark.parametrize('integrated', [False, True])
def test_monitor_forwards_normal_console_arguments_without_another_instance(monkeypatch, tmp_path, integrated):
    from operator_console import app, vision_monitor, __main__ as entry
    import sys
    observed = object()
    calls = []
    monkeypatch.setattr(vision_monitor, 'observed_console', lambda original, output: observed)
    monkeypatch.setattr(app, 'main', lambda: calls.append(sys.argv[1:]))
    monkeypatch.setattr(entry, 'main', lambda args: calls.append(args))
    monkeypatch.setattr(app, 'OperatorConsole', app.OperatorConsole)
    remaining = ['--page', 'vision', '--terrain-display-delay-ms', '0']
    monkeypatch.setattr(sys, 'argv', ['monitor', '--metrics-output', str(tmp_path/'events.json'),
                                    *(['--integrated'] if integrated else []), *remaining])
    vision_monitor.main()
    assert app.OperatorConsole is observed
    assert calls == [remaining]
