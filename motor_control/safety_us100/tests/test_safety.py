from safety_us100.verdict import Verdict, SAFE, WARN, STOP
from safety_us100.config import SafetyConfig


def test_level_names():
    assert SAFE == "safe"
    assert WARN == "warn"
    assert STOP == "stop"


def test_verdict_holds_level_and_distance():
    v = Verdict(level=SAFE, distance_mm=500.0)
    assert v.level == "safe"
    assert v.distance_mm == 500.0


def test_config_default_values():
    c = SafetyConfig()
    assert c.warn_mm == 400.0
    assert c.stop_mm == 200.0
    assert c.hysteresis_mm == 30.0
    assert c.fail_stop_count == 3
    assert c.port == "/dev/ttyTHS1"
    assert c.baud == 9600
from safety_us100.evaluator import evaluate


def test_far_distance_is_safe():
    cfg = SafetyConfig()
    assert evaluate(500.0, cfg, prev_level=None) == SAFE


def test_mid_distance_is_warn():
    cfg = SafetyConfig()
    assert evaluate(300.0, cfg, prev_level=None) == WARN


def test_near_distance_is_stop():
    cfg = SafetyConfig()
    assert evaluate(150.0, cfg, prev_level=None) == STOP


def test_no_reading_is_stop():
    cfg = SafetyConfig()
    assert evaluate(None, cfg, prev_level=None) == STOP


def test_escalation_is_immediate():
    cfg = SafetyConfig()
    assert evaluate(150.0, cfg, prev_level=SAFE) == STOP


def test_hysteresis_holds_stop_near_threshold():
    cfg = SafetyConfig()
    assert evaluate(210.0, cfg, prev_level=STOP) == STOP


def test_release_from_stop_after_margin():
    cfg = SafetyConfig()
    assert evaluate(250.0, cfg, prev_level=STOP) == WARN


def test_hysteresis_holds_warn_near_threshold():
    cfg = SafetyConfig()
    assert evaluate(410.0, cfg, prev_level=WARN) == WARN
    assert evaluate(440.0, cfg, prev_level=WARN) == SAFE
from safety_us100.fake_sensor import FakeUs100


def test_fake_returns_readings_in_order():
    s = FakeUs100([500.0, 300.0, 150.0])
    assert s.read() == 500.0
    assert s.read() == 300.0
    assert s.read() == 150.0


def test_fake_repeats_last_after_end():
    s = FakeUs100([400.0])
    s.read()
    assert s.read() == 400.0
    assert s.read() == 400.0


def test_fake_can_return_none():
    s = FakeUs100([None])
    assert s.read() is None
from safety_us100.safety_monitor import SafetyMonitor


def test_initial_verdict_is_stop():
    mon = SafetyMonitor(FakeUs100([500.0]), SafetyConfig())
    assert mon.verdict().level == STOP


def test_far_reading_gives_safe():
    mon = SafetyMonitor(FakeUs100([500.0]), SafetyConfig())
    mon.tick()
    assert mon.verdict().level == SAFE
    assert mon.verdict().distance_mm == 500.0


def test_near_reading_gives_stop():
    mon = SafetyMonitor(FakeUs100([150.0]), SafetyConfig())
    mon.tick()
    assert mon.verdict().level == STOP


def test_transient_failures_keep_previous():
    mon = SafetyMonitor(FakeUs100([500.0, None, None, None]), SafetyConfig(fail_stop_count=3))
    mon.tick()
    assert mon.verdict().level == SAFE
    mon.tick()
    assert mon.verdict().level == SAFE
    mon.tick()
    assert mon.verdict().level == SAFE


def test_persistent_failure_gives_stop():
    mon = SafetyMonitor(FakeUs100([500.0, None, None, None]), SafetyConfig(fail_stop_count=3))
    mon.tick()
    mon.tick()
    mon.tick()
    mon.tick()
    assert mon.verdict().level == STOP
    assert mon.verdict().distance_mm is None


def test_recovery_to_safe():
    mon = SafetyMonitor(FakeUs100([150.0, 500.0]), SafetyConfig())
    mon.tick()
    assert mon.verdict().level == STOP
    mon.tick()
    assert mon.verdict().level == SAFE


def test_no_chatter_with_hysteresis():
    mon = SafetyMonitor(FakeUs100([150.0, 210.0, 190.0, 210.0]), SafetyConfig())
    levels = []
    for _ in range(4):
        mon.tick()
        levels.append(mon.verdict().level)
    assert levels == [STOP, STOP, STOP, STOP]


def test_verdict_schema():
    mon = SafetyMonitor(FakeUs100([500.0]), SafetyConfig())
    mon.tick()
    v = mon.verdict()
    assert hasattr(v, "level")
    assert hasattr(v, "distance_mm")
