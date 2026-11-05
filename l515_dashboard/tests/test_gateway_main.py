import signal

from l515_dashboard import gateway_main
from l515_dashboard.gateway import GatewayState


class FakeGateway:
    def __init__(self, trigger):
        self.state=GatewayState.STOPPED; self.last_error=None; self.fatal_error=None
        self.shutdown_requested=False; self.trigger=trigger
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


def test_srt_output_rate_defaults_to_30_without_environment():
    config = gateway_main.config_from_env({})
    assert (config.fps, config.stream_fps) == (30, 30)


def test_srt_output_rate_is_selected_by_environment():
    config = gateway_main.config_from_env({"L515_SRT_FPS": "15"})
    assert (config.fps, config.stream_fps) == (30, 15)


def test_invalid_srt_output_rate_fails_before_the_camera_opens():
    import pytest
    for value in ("", "abc", "20", "15.0"):
        with pytest.raises(ValueError, match="L515_SRT_FPS"):
            gateway_main.config_from_env({"L515_SRT_FPS": value})
