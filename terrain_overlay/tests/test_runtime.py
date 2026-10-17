"""Sender tick logic without ROS (spec §4.1, §4.4, §7.1 node-tick cases)."""
import numpy as np
import pytest

from terrain_overlay import runtime as runtime_module
from terrain_overlay.camera import CameraModel
from terrain_overlay.contract import parse_packet
from terrain_overlay.runtime import RuntimeConfig, SenderSession, TerrainOverlayRuntime
from terrain_overlay.tests.scenes import COLOR, DEPTH, default_scene, floor_only

SESSION = SenderSession("0082f17fdc0a49969ff0fe1dc4979560", 1000, "3fa95c0e81d2b7a4")


class FakeClock:
    def __init__(self):
        self.t = 0.0

    def __call__(self):
        return self.t


def wall():
    return np.full((DEPTH.height, DEPTH.width), 4000, np.uint16)


def ready(config=None, errors=None):
    rt = TerrainOverlayRuntime(
        config or RuntimeConfig(), SESSION,
        log_error=(errors.append if errors is not None else lambda _m: None),
        unix_ns=lambda: 1_760_000_000_000_000_000,
        clock=FakeClock(),
    )
    rt.set_depth_info(DEPTH)
    rt.set_color_info(COLOR)
    return rt


def frame(rt, now):
    rt._clock.t = now
    raw = rt.tick()
    assert raw is not None
    return parse_packet(raw)


def slow_processing(rt, monkeypatch, seconds):
    """Make every detector call advance the fake clock, like real CPU time."""
    original = rt._detector.process

    def process(*args):
        rt._clock.t += seconds
        return original(*args)

    monkeypatch.setattr(rt._detector, "process", process)


def test_ok_packet_for_the_default_scene():
    rt = ready()
    rt.on_depth(default_scene(), stamp_ns=111, rx_s=10.0)
    packet = frame(rt, 10.03)
    assert (packet.status, packet.reason) == ("OK", None)
    assert packet.ground.state == "FRESH"
    assert packet.ground.age_ms == pytest.approx(30.0, abs=0.1)
    assert packet.depth_age_ms == pytest.approx(30.0, abs=0.1)
    assert packet.depth_stamp_ns == 111
    assert packet.zones["center"] == pytest.approx(1.20, abs=0.03)
    assert len(packet.obstacles) == 1 and packet.obstacles[0].in_path
    assert packet.distance_origin == "l515_depth_optical"
    assert packet.alignment_verified is False
    assert packet.proc_ms is not None
    assert packet.session_key == ("0082f17fdc0a49969ff0fe1dc4979560", 1000, "3fa95c0e81d2b7a4")


def test_camera_info_and_color_size_gate_processing():
    rt = TerrainOverlayRuntime(RuntimeConfig(), SESSION, clock=FakeClock())
    rt.on_depth(default_scene(), 1, 0.0)
    assert frame(rt, 0.05).reason == "CAMERA_INFO_MISSING"
    rt.set_depth_info(DEPTH)
    rt.set_color_info(CameraModel(615.0, 615.0, 320.0, 240.0, 640, 480))
    assert frame(rt, 0.06).reason == "COLOR_SHAPE_MISMATCH"
    rt.set_color_info(COLOR)
    assert frame(rt, 0.07).status == "OK"


def test_depth_shape_mismatch():
    rt = ready()
    rt.on_depth(np.zeros((240, 320), np.uint16), 1, 0.0)
    assert frame(rt, 0.05).reason == "DEPTH_SHAPE_MISMATCH"


def test_bad_encoding_is_reported_until_a_good_frame_arrives():
    rt = ready()
    rt.on_depth(default_scene(), 1, 0.0)
    rt.on_bad_depth(0.1)
    assert frame(rt, 0.15).reason == "DEPTH_ENCODING"
    rt.on_depth(default_scene(), 2, 0.2)
    assert frame(rt, 0.25).status == "OK"


def test_stale_contract_at_10_hz():
    rt = ready()
    rt.on_depth(default_scene(), 1, 0.0)
    statuses = [(round(t, 2), frame(rt, t).reason) for t in np.arange(0.05, 0.75, 0.1)]
    first_stale = next(t for t, reason in statuses if reason == "DEPTH_STALE")
    assert first_stale <= 0.5 + 0.1 + 0.05
    assert all(reason is None for t, reason in statuses if t <= 0.5)


def test_resend_keeps_the_stamp_and_ages_in_ms():
    rt = ready()
    rt.on_depth(default_scene(), 7, 0.0)
    first, second = frame(rt, 0.05), frame(rt, 0.15)
    assert first.depth_stamp_ns == second.depth_stamp_ns == 7
    assert second.ground.state == "FRESH"
    assert second.ground.age_ms == pytest.approx(150.0, abs=0.1)
    assert second.depth_age_ms == pytest.approx(150.0, abs=0.1)
    assert second.sequence == first.sequence + 1


def test_case_ga_expiry_is_judged_on_receive_time():
    # (가) t=0 FRESH, a frame received at 0.59 fails the fit: detector expires it.
    rt = ready()
    rt.on_depth(floor_only(), 1, 0.0)
    assert frame(rt, 0.01).ground.state == "FRESH"
    rt.on_depth(wall(), 2, 0.59)
    packet = frame(rt, 0.60)
    assert (packet.status, packet.reason) == ("NO_GROUND", "GROUND_EXPIRED")


def test_case_na_send_time_expiry_overrides_a_held_result():
    # (나) the 0.45 frame fails (detector: HELD) but is sent at 0.56.
    rt = ready()
    rt.on_depth(floor_only(), 1, 0.0)
    frame(rt, 0.01)
    rt.on_depth(wall(), 2, 0.45)
    packet = frame(rt, 0.56)
    assert (packet.status, packet.reason) == ("NO_GROUND", "GROUND_EXPIRED")


def test_cached_held_result_expires_on_resend():
    rt = ready()
    rt.on_depth(floor_only(), 1, 0.0)
    frame(rt, 0.01)
    rt.on_depth(wall(), 2, 0.30)
    assert frame(rt, 0.31).ground.state == "HELD"
    assert frame(rt, 0.45).ground.state == "HELD"
    assert frame(rt, 0.55).reason == "GROUND_EXPIRED"


def test_processing_exception_becomes_a_processing_error(monkeypatch):
    errors = []
    rt = ready(errors=errors)

    def boom(*_args, **_kwargs):
        raise RuntimeError("synthetic core failure")

    monkeypatch.setattr(rt._detector, "process", boom)
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.05)
    assert (packet.status, packet.reason, packet.proc_ms) == ("SENSOR_ERROR", "PROCESSING_ERROR", None)
    assert errors and "synthetic core failure" in errors[0]


def test_self_check_failure_sends_a_clean_processing_error(monkeypatch):
    errors = []
    rt = ready(errors=errors)
    original = rt._payload

    def corrupt(*args):
        payload = original(*args)
        payload["ground"]["camera_tilt_deg"] = 99.0
        return payload

    monkeypatch.setattr(rt, "_payload", corrupt)
    rt.on_depth(default_scene(), 5, 0.0)
    packet = frame(rt, 0.05)
    assert (packet.status, packet.reason) == ("SENSOR_ERROR", "PROCESSING_ERROR")
    assert packet.ground is None and packet.obstacles == () and packet.proc_ms is None
    assert all(value is None for value in packet.zones.values())
    assert packet.depth_stamp_ns == 5
    assert any("self-check" in message for message in errors)


def test_unsendable_error_packet_is_dropped(monkeypatch):
    rt = ready()
    monkeypatch.setattr(rt, "_payload", lambda *a: {"bad": float("nan")})
    monkeypatch.setattr(runtime_module, "PACKET_TYPE", "broken")
    rt.on_depth(default_scene(), 5, 0.0)
    rt._clock.t = 0.05
    assert rt.tick() is None
    assert rt.sequence == 0


def test_case_da_processing_time_counts_toward_expiry_and_ages(monkeypatch):
    # (다) a failed-fit frame received at 0.40 is HELD for the detector, but
    # processing takes 0.20 s, so at send time (0.61) the plane is 0.61 s old.
    rt = ready()
    rt.on_depth(floor_only(), 1, 0.0)
    frame(rt, 0.01)
    slow_processing(rt, monkeypatch, 0.20)
    rt.on_depth(wall(), 2, 0.40)
    packet = frame(rt, 0.41)
    assert (packet.status, packet.reason) == ("NO_GROUND", "GROUND_EXPIRED")
    assert packet.depth_age_ms == pytest.approx(210.0, abs=0.1)


def test_processing_time_is_included_in_fresh_ages(monkeypatch):
    rt = ready()
    slow_processing(rt, monkeypatch, 0.08)
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.02)
    assert packet.ground.state == "FRESH"
    assert packet.ground.age_ms == pytest.approx(100.0, abs=0.1)
    assert packet.depth_age_ms == pytest.approx(100.0, abs=0.1)


def test_depth_that_goes_stale_during_processing_is_reported_stale(monkeypatch):
    rt = ready()
    slow_processing(rt, monkeypatch, 0.30)
    rt.on_depth(default_scene(), 1, 0.0)
    assert frame(rt, 0.25).reason == "DEPTH_STALE"  # sent at 0.55 > 0.5


def test_processing_longer_than_a_tick_is_logged_and_bounds_stale(monkeypatch):
    # Codex 2026-10-17: a 0.6 s process delays the DEPTH_STALE send to 0.68 s.
    # The bound is stale + 1/hz + processing; the overrun is logged so the
    # Jetson load check (§7.3-2) can catch it.
    errors = []
    rt = ready(errors=errors)
    slow_processing(rt, monkeypatch, 0.6)
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.08)
    assert (packet.reason, packet.depth_age_ms) == ("DEPTH_STALE", 680.0)
    assert any("exceeds the 100 ms tick" in message for message in errors)


def test_failed_processing_still_counts_toward_the_maximum(monkeypatch):
    errors = []
    rt = ready(errors=errors)

    def slow_failure(*_args):
        rt._clock.t += 0.6
        raise RuntimeError("synthetic core failure")

    monkeypatch.setattr(rt._detector, "process", slow_failure)
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.08)
    assert (packet.reason, packet.depth_age_ms) == ("DEPTH_STALE", 680.0)
    assert rt.processing_overruns == 1
    assert rt.max_processing_s == pytest.approx(0.6)
    assert any("exceeds the 100 ms tick" in message for message in errors)


def test_overruns_use_their_own_log_sink(monkeypatch):
    errors, overruns = [], []
    rt = TerrainOverlayRuntime(RuntimeConfig(), SESSION, log_error=errors.append,
                               log_overrun=overruns.append, clock=FakeClock())
    rt._record_processing_time(0.2)
    assert len(overruns) == 1 and errors == []


def test_overrun_is_judged_before_rounding(monkeypatch):
    rt = ready()
    slow_processing(rt, monkeypatch, 0.10004)  # rounds to 100.0 ms
    rt.on_depth(default_scene(), 1, 0.0)
    frame(rt, 0.0)
    assert rt.processing_overruns == 1


def test_front_offset_switches_the_distance_origin():
    rt = ready(RuntimeConfig(front_offset_m=0.19))
    rt.on_depth(default_scene(), 1, 0.0)
    packet = frame(rt, 0.05)
    assert packet.distance_origin == "rover_front"
    assert packet.zones["center"] == pytest.approx(1.01, abs=0.03)


@pytest.mark.parametrize("kwargs", [
    {"publish_hz": 4.9}, {"publish_hz": 10.1}, {"depth_stale_s": 0.0},
    {"ground_hold_s": -1.0}, {"path_half_width_m": 0.0}, {"front_offset_m": -0.1},
    {"publish_hz": float("nan")},
])
def test_invalid_config_is_refused(kwargs):
    with pytest.raises(ValueError):
        RuntimeConfig(**kwargs)


def test_session_identity(tmp_path):
    boot = tmp_path / "boot_id"
    boot.write_text("0082f17f-dc0a-4996-9ff0-fe1dc4979560\n")
    first = SenderSession.create(boot_id_path=boot, monotonic_ns=lambda: 42)
    second = SenderSession.create(boot_id_path=boot, monotonic_ns=lambda: 42)
    assert first.boot_id == "0082f17fdc0a49969ff0fe1dc4979560"
    assert len(first.instance) == 16
    assert (first.boot_id, first.start_mono_ns) == (second.boot_id, second.start_mono_ns)
    assert first.instance != second.instance  # equal start values still differ


def test_reprocesses_after_camera_info_changes():
    rt = ready()
    rt.on_depth(default_scene(), 1, 0.0)
    assert frame(rt, 0.05).status == "OK"
    rt.set_depth_info(CameraModel(400.0, 400.0, 320.0, 240.0, 640, 480))
    assert rt._cache is None
    frame(rt, 0.06)
    assert rt._cache is not None
