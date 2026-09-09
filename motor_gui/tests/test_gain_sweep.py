from motor_gui.tools import gain_sweep
from motor_gui.backend.transport.fake import FakeTransport


def test_gain_sweep_uses_worker_arm_gate_and_runs_a_complete_fake_step(monkeypatch, capsys):
    applied = []
    class TracedFake(FakeTransport):
        def apply(self, command):
            applied.append(command)
            return super().apply(command)
    monkeypatch.setattr(gain_sweep, "_make_transport", lambda *args, **kwargs: TracedFake())
    monkeypatch.setattr(gain_sweep, "COMBOS", [(8.0, 0.015, 0.0)])
    gain_sweep.run("fake", step=.1, bw=2, settle=.1)
    output = capsys.readouterr().out
    assert "TRIP at arm" not in output
    assert "done" in output
    assert any(c["op"] == "set_input" and c.get("args", {}).get("pos") == .1 for c in applied)
