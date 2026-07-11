import signal

from l515_dashboard import gateway_main
from l515_dashboard.gateway import GatewayState


class FakeGateway:
    def __init__(self, trigger):
        self.state=GatewayState.STOPPED; self.last_error=None; self.fatal_error=None; self.trigger=trigger
        self.shutdown_calls=0
    def start(self): self.state=GatewayState.RUNNING
    def run_once(self): self.trigger(signal.SIGTERM, None)
    def shutdown(self): self.shutdown_calls += 1; self.state=GatewayState.STOPPED
    def ros_fatal(self, exc): self.last_error=str(exc); self.state=GatewayState.FAULT


def test_sigterm_routes_through_single_shutdown(monkeypatch):
    handlers={}
    monkeypatch.setattr(gateway_main.signal, "signal", lambda sig, fn: handlers.setdefault(sig, fn))
    fake=FakeGateway(lambda sig, frame: handlers[sig](sig, frame))
    monkeypatch.setattr(gateway_main, "build_gateway", lambda: fake)
    monkeypatch.setattr(gateway_main.time, "sleep", lambda _: None)
    assert gateway_main.main() == 0
    assert fake.shutdown_calls == 1


def test_recoverable_error_does_not_make_clean_stop_fail(monkeypatch):
    handlers={}
    monkeypatch.setattr(gateway_main.signal,"signal",lambda sig,fn:handlers.setdefault(sig,fn))
    fake=FakeGateway(lambda sig,frame:handlers[sig](sig,frame)); fake.last_error="old gst error"
    monkeypatch.setattr(gateway_main,"build_gateway",lambda:fake)
    monkeypatch.setattr(gateway_main.time,"sleep",lambda _:None)
    assert gateway_main.main() == 0
