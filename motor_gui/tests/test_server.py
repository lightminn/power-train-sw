from fastapi.testclient import TestClient

from motor_gui.backend.server import create_app


def _client() -> TestClient:
    return TestClient(create_app(track="fake"))


def test_capabilities_endpoint():
    with _client() as c:
        r = c.get("/api/capabilities")
        assert r.status_code == 200
        caps = r.json()
        assert caps["track"] == "fake"
        assert "odrive.pos" in caps["signals"]


def test_command_endpoint_acks():
    with _client() as c:
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
    from motor_gui.backend.server import _make_transport
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


def test_make_transport_odrive_can_track():
    from motor_gui.backend.server import _make_transport
    t = _make_transport("odrive_can")
    caps = t.capabilities()                     # connect 없이 (정적 조각)
    assert caps["track"] == "can"
    assert caps["devices"] == ["odrive"]
    assert caps["control_modes"]["odrive"] == ["position", "position_traj", "velocity"]
    assert "set_param" in caps["commands"]["odrive"]
    tk = {t["key"]: t for t in caps["tunables"]["odrive"]}
    assert "torque_constant" in tk


def test_ak_track_default_id_is_1():
    from motor_gui.backend.server import _make_transport
    t = _make_transport("ak")
    assert t.device_ids() == {"ak": {"id": 1, "min": 1, "max": 127, "label": "AK 모터 ID"}}
    assert t.capabilities()["can_ids"]["ak"]["id"] == 1


def test_odrive_can_default_node_is_11():
    from motor_gui.backend.server import _make_transport
    t = _make_transport("odrive_can")
    assert t.device_ids()["odrive"]["id"] == 11
    assert t.capabilities()["can_ids"]["odrive"]["id"] == 11


def test_can_id_endpoint_rejects_empty():
    with _client() as c:
        r = c.post("/api/can_id", json={})
        assert r.status_code == 200
        assert r.json()["ok"] is False
