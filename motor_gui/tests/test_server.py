import pytest
from fastapi.testclient import TestClient

from motor_gui.backend.server import _build_parser, _make_transport, create_app


def _client() -> TestClient:
    return TestClient(create_app(track="fake"))


def test_capabilities_endpoint():
    with _client() as c:
        r = c.get("/api/capabilities")
        assert r.status_code == 200
        caps = r.json()
        assert caps["track"] == "fake"
        assert "odrive.pos" in caps["signals"]
        assert "drive_gear_ratio" not in caps


def test_command_endpoint_acks():
    with _client() as c:
        armed = c.post("/api/command",
                       json={"target": "odrive", "op": "arm", "args": {}})
        assert armed.json()["ok"] is True
        r = c.post("/api/command", json={"target": "odrive", "op": "set_input",
                                         "args": {"vel": 3.0}})
        assert r.status_code == 200
        assert r.json()["ok"] is True


def test_command_endpoint_rejects_unknown():
    with _client() as c:
        r = c.post("/api/command", json={"target": "ghost", "op": "estop",
                                         "args": {}})
        assert r.json()["ok"] is False


def test_telemetry_websocket_streams_samples():
    with _client() as c:
        with c.websocket_connect("/ws/telemetry") as ws:
            msg = ws.receive_json()
            assert "t_mono" in msg and "odrive.vel" in msg


def test_record_start_stop(tmp_path):
    with _client() as c:
        path = str(tmp_path / "log.csv")
        r1 = c.post("/api/record/start", json={"path": path, "fmt": "csv"})
        assert r1.json()["ok"] is True
        r2 = c.post("/api/record/stop")
        assert r2.json()["ok"] is True


def test_make_transport_ak_track_capabilities():
    t = _make_transport("ak")
    caps = t.capabilities()                     # connect 없이 (정적 조각)
    assert caps["track"] == "ak"
    assert caps["devices"] == ["ak"]
    assert caps["control_modes"]["ak"] == ["position", "velocity", "duty"]
    assert "set_param" in caps["commands"]["ak"]


def test_reconnect_endpoint_ok():
    with _client() as c:
        r = c.post("/api/reconnect")
        assert r.status_code == 200
        assert r.json()["ok"] is True


def test_tunable_profile_endpoints_require_explicit_profile_choice():
    with _client() as c:
        profiles = c.get("/api/tunable_profiles")
        assert profiles.status_code == 200
        assert set(profiles.json()) == {"x2212", "bl70200"}

        assert c.post("/api/command", json={"target": "odrive", "op": "arm",
                                            "args": {}}).json()["ok"] is True
        applied = c.post("/api/tunable_profiles/apply",
                         json={"profile": "bl70200"})
        assert applied.status_code == 200
        assert applied.json()["ok"] is True
        assert applied.json()["profile"] == "bl70200"


def test_frontend_requires_profile_selection_before_apply():
    with _client() as client:
        app_js = client.get("/app.js").text
    assert 'fetch("/api/tunable_profiles")' in app_js
    assert 'fetch("/api/tunable_profiles/apply"' in app_js


def test_estop_reset_endpoint_returns_idle_without_arming():
    with _client() as client:
        assert client.post("/api/command", json={
            "target": "odrive", "op": "arm", "args": {}
        }).json()["ok"] is True
        assert client.post("/api/command", json={
            "target": "odrive", "op": "estop", "args": {}
        }).json()["ok"] is True

        rejected = client.post("/api/command", json={
            "target": "odrive", "op": "set_input", "args": {"vel": 1.0}
        }).json()
        assert rejected["ok"] is False
        assert "estop active" in rejected["detail"]

        reset = client.post("/api/command", json={
            "target": "odrive", "op": "reset", "args": {}
        }).json()
        assert reset["ok"] is True
        safety = client.get("/api/safety").json()
        assert safety == {"estop_latched": False,
                          "armed": {"odrive": False, "ak": False}}


def test_frontend_exposes_reset_and_device_arm_controls():
    with _client() as client:
        html = client.get("/").text
        app_js = client.get("/app.js").text
    assert "E-STOP RESET" in html
    assert 'op: "arm"' in app_js
    assert 'op: "disarm"' in app_js
    assert 'op: "reset"' in app_js


def test_make_transport_odrive_can_track():
    t = _make_transport("odrive_can")
    caps = t.capabilities()                     # connect 없이 (정적 조각)
    assert caps["track"] == "can"
    assert caps["devices"] == ["odrive"]
    assert caps["control_modes"]["odrive"] == ["position", "position_traj", "velocity"]
    assert "set_param" in caps["commands"]["odrive"]
    tk = {t["key"]: t for t in caps["tunables"]["odrive"]}
    assert "torque_constant" in tk
    assert caps["drive_gear_ratio"] == 5.0


def test_ak_track_default_id_is_1():
    t = _make_transport("ak")
    assert t.device_ids() == {"ak": {"id": 1, "min": 1, "max": 127, "label": "AK 모터 ID"}}
    assert t.capabilities()["can_ids"]["ak"]["id"] == 1


def test_odrive_can_default_node_is_11():
    t = _make_transport("odrive_can")
    assert t.device_ids()["odrive"]["id"] == 11
    assert t.capabilities()["can_ids"]["odrive"]["id"] == 11


def test_can_id_endpoint_rejects_empty():
    with _client() as c:
        r = c.post("/api/can_id", json={})
        assert r.status_code == 200
        assert r.json()["ok"] is False


def test_drive_gear_ratio_cli_defaults_and_parses_explicit_value():
    parser = _build_parser()
    assert parser.parse_args([]).drive_gear_ratio == 5.0
    assert parser.parse_args(["--drive-gear-ratio", "1.0"]).drive_gear_ratio == 1.0


@pytest.mark.parametrize("value", ["0", "-1", "inf", "nan"])
def test_drive_gear_ratio_cli_rejects_nonpositive_or_nonfinite(value):
    with pytest.raises(SystemExit):
        _build_parser().parse_args(["--drive-gear-ratio", value])


@pytest.mark.parametrize("track", ["usb", "odrive_can", "can"])
def test_make_transport_injects_ratio_into_odrive_tracks(track):
    caps = _make_transport(track, drive_gear_ratio=7.0).capabilities()
    assert caps["drive_gear_ratio"] == 7.0


def test_make_transport_does_not_apply_ratio_to_ak_or_fake():
    ak = _make_transport("ak", drive_gear_ratio=9.0)
    fake = _make_transport("fake", drive_gear_ratio=9.0)
    fake.connect()
    fake.apply({"target": "odrive", "op": "set_input", "args": {"vel": 1.0}})
    assert "drive_gear_ratio" not in ak.capabilities()
    assert "drive_gear_ratio" not in fake.capabilities()
    assert fake._target == 1.0


def test_odrive_capabilities_endpoint_exposes_applied_ratio(monkeypatch):
    from motor_gui.backend.worker import HardwareWorker

    monkeypatch.setattr(HardwareWorker, "start", lambda self: None)
    monkeypatch.setattr(HardwareWorker, "stop", lambda self: None)
    with TestClient(create_app(track="odrive_can", drive_gear_ratio=6.0)) as client:
        payload = client.get("/api/capabilities").json()
    assert payload["drive_gear_ratio"] == 6.0


def test_frontend_labels_wheel_velocity_and_displays_ratio():
    with _client() as client:
        html = client.get("/").text
        app_js = client.get("/app.js").text
        plots_js = client.get("/plots.js").text
    assert "wheel rev/s" in html
    assert "drive_gear_ratio" in app_js
    assert "wheel rev/s" in plots_js
