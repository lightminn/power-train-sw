import importlib
import sys

from motor_control.laptop.haptic_arbiter import Rumble


class FakeClock:
    def __init__(self, now=0.0):
        self.now = float(now)

    def __call__(self):
        return self.now


class FakeArbiter:
    def __init__(self, decisions=(), color=(0, 0, 255)):
        self.decisions = list(decisions)
        self.color = color
        self.decide_calls = 0

    def decide(self):
        self.decide_calls += 1
        if self.decisions:
            return self.decisions.pop(0)
        return None

    def lightbar(self):
        return self.color


class FakeBackend:
    def __init__(self, *, fail_rumble=False):
        self.fail_rumble = fail_rumble
        self.rumble_calls = []
        self.lightbar_calls = []
        self.trigger_calls = []
        self.closed = False

    def rumble(self, low, high, duration_ms):
        self.rumble_calls.append((low, high, duration_ms))
        if self.fail_rumble:
            raise RuntimeError("hid write failed")

    def lightbar(self, color):
        self.lightbar_calls.append(color)

    def trigger_lock(self, locked):
        self.trigger_calls.append(bool(locked))

    def close(self):
        self.closed = True


def _module():
    return importlib.import_module("motor_control.laptop.dualsense_output")


def test_run_once_forwards_arbiter_rumble_and_lightbar_to_backend():
    output_module = _module()
    clock = FakeClock()
    decision = Rumble(low=0.25, high=0.75, duration_ms=120)
    arbiter = FakeArbiter([decision], color=(0, 0, 255))
    backend = FakeBackend()
    output = output_module.DualSenseOutput(
        arbiter,
        backend_factory=lambda: backend,
        clock=clock,
    )

    output.run_once()

    assert backend.rumble_calls == [(0.25, 0.75, 120)]
    assert backend.lightbar_calls == [(0, 0, 255)]


def test_backend_exception_warns_once_and_permanently_disables_output(capsys):
    output_module = _module()
    arbiter = FakeArbiter(
        [
            Rumble(low=0.5, high=0.5, duration_ms=100),
            Rumble(low=0.8, high=0.8, duration_ms=100),
        ]
    )
    backend = FakeBackend(fail_rumble=True)
    output = output_module.DualSenseOutput(
        arbiter,
        backend_factory=lambda: backend,
        clock=FakeClock(),
    )

    output.run_once()
    output.run_once()

    assert backend.rumble_calls == [(0.5, 0.5, 100)]
    assert arbiter.decide_calls == 1
    assert backend.closed is True
    assert capsys.readouterr().err.count("DualSense haptics disabled") == 1


def test_missing_optional_backend_quietly_disables_output(capsys):
    output_module = _module()
    arbiter = FakeArbiter([Rumble(low=1.0, high=1.0, duration_ms=100)])
    output = output_module.DualSenseOutput(
        arbiter,
        backend_factory=lambda: None,
        clock=FakeClock(),
    )

    output.run_once()
    output.run_once()

    assert arbiter.decide_calls == 0
    assert capsys.readouterr().err == ""


def test_trigger_fx_defaults_off_and_never_calls_trigger_api():
    output_module = _module()
    backend = FakeBackend()
    output = output_module.DualSenseOutput(
        FakeArbiter(color=(0, 0, 255)),
        backend_factory=lambda: backend,
        clock=FakeClock(),
    )

    output.run_once()

    assert backend.trigger_calls == []


def test_module_import_succeeds_without_importing_pydualsense(monkeypatch):
    monkeypatch.setitem(sys.modules, "pydualsense", None)
    sys.modules.pop("motor_control.laptop.dualsense_output", None)

    output_module = _module()

    assert "pydualsense" not in output_module.__dict__
