import json

from motor_control.laptop import ops_channel_client as client


class FakeInput:
    def __init__(self, pressed=(), dpad_y=0):
        self.pressed = set(pressed)
        self.dpad_y = dpad_y

    def button(self, name):
        return name in self.pressed


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeSocket:
    def __init__(self):
        self.sent = []
        self.received = []
        self.closed = False

    def sendall(self, payload):
        self.sent.append(payload)

    def recv(self, _size):
        if not self.received:
            raise BlockingIOError
        return self.received.pop(0)

    def close(self):
        self.closed = True


class SocketConnector:
    def __init__(self, *sockets):
        self.sockets = list(sockets)

    def __call__(self, _host, _port):
        return self.sockets.pop(0)


def _wire_records(sock):
    return [json.loads(record) for record in sock.sent]


def _request_records(sock):
    return [
        record for record in _wire_records(sock) if not record.get("hello")
    ]


def _response(
    request_id,
    status,
    *,
    revision=1,
    detail="test",
    queried_request_id=None,
):
    payload = {
        "request_id": request_id,
        "status": status,
        "state_revision": revision,
        "detail": detail,
    }
    if queried_request_id is not None:
        payload["queried_request_id"] = queried_request_id
    return (json.dumps(payload) + "\n").encode("utf-8")


def test_square_create_hold_emits_clear_once_until_released():
    detector = client.RecoveryChordDetector()
    held = FakeInput({"square", "create"})

    assert detector.update(held, now_ns=0) == []
    assert detector.update(held, now_ns=1_999_999_999) == []
    assert detector.update(held, now_ns=2_000_000_000) == [
        {"action": "clear_transient_hold"}
    ]
    assert detector.update(held, now_ns=3_000_000_000) == []


def test_partial_release_resets_clear_hold_timer():
    detector = client.RecoveryChordDetector()
    held = FakeInput({"square", "create"})
    partial = FakeInput({"square"})

    assert detector.update(held, now_ns=0) == []
    assert detector.update(held, now_ns=1_500_000_000) == []
    assert detector.update(partial, now_ns=1_500_000_001) == []
    assert detector.update(held, now_ns=2_000_000_000) == []
    assert detector.update(held, now_ns=3_999_999_999) == []
    assert detector.update(held, now_ns=4_000_000_000) == [
        {"action": "clear_transient_hold"}
    ]


def test_dpad_holds_emit_manual_and_auto_once_each():
    detector = client.RecoveryChordDetector()
    down = FakeInput(dpad_y=-1)

    assert detector.update(down, now_ns=0) == []
    assert detector.update(down, now_ns=999_999_999) == []
    assert detector.update(down, now_ns=1_000_000_000) == [
        {"action": "authority_manual"}
    ]
    assert detector.update(down, now_ns=1_500_000_000) == []

    assert detector.update(FakeInput(), now_ns=1_600_000_000) == []
    up = FakeInput(dpad_y=1)
    assert detector.update(up, now_ns=2_000_000_000) == []
    assert detector.update(up, now_ns=3_000_000_000) == [
        {"action": "authority_auto"}
    ]


def test_emergency_chords_emit_begin_immediately_and_execute_after_hold():
    detector = client.RecoveryChordDetector()
    estop = FakeInput({"l1", "r1", "square"})

    assert detector.update(estop, now_ns=0) == [
        {"action": "estop_reset", "phase": "begin"}
    ]
    assert detector.update(estop, now_ns=4_999_999_999) == []
    assert detector.update(estop, now_ns=5_000_000_000) == [
        {"action": "estop_reset", "phase": "execute"}
    ]
    assert detector.update(estop, now_ns=5_100_000_000) == []

    assert detector.update(FakeInput({"l1", "r1"}), now_ns=5_200_000_000) == []
    arm = FakeInput({"l1", "r1", "triangle"})
    assert detector.update(arm, now_ns=6_000_000_000) == [
        {"action": "arm", "phase": "begin"}
    ]
    assert detector.update(arm, now_ns=8_999_999_999) == []
    assert detector.update(arm, now_ns=9_000_000_000) == [
        {"action": "arm", "phase": "execute"}
    ]


def test_submit_sends_immediately_retransmits_every_250ms_and_stops_at_2s():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )

    request_id = ops.submit("authority_manual")
    assert _wire_records(sock)[0]["hello"] is True
    assert len(_request_records(sock)) == 1

    clock.now = 0.249
    assert ops.pump() == []
    assert len(_request_records(sock)) == 1
    for tick in (0.25, 0.50, 0.75, 1.00, 1.25, 1.50, 1.75):
        clock.now = tick
        assert ops.pump() == []
    assert len(_request_records(sock)) == 8

    clock.now = 2.0
    assert ops.pump() == []
    assert len(_request_records(sock)) == 8
    assert _request_records(sock)[0]["request_id"] == request_id


def test_retransmissions_keep_the_same_request_id():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )

    request_id = ops.submit("clear_transient_hold")
    clock.now = 0.25
    ops.pump()
    clock.now = 0.50
    ops.pump()

    requests = _request_records(sock)
    assert {record["request_id"] for record in requests} == {request_id}
    assert {record["sequence"] for record in requests} == {0}


def test_ops_state_push_updates_latest_snapshot():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )
    push = {
        "push": "ops_state",
        "revision": 7,
        "authority_mode": "IDLE",
    }
    sock.received.append((json.dumps(push) + "\n").encode("utf-8"))

    assert ops.pump() == [push]
    assert ops.latest_ops_state() == push


def test_receive_buffer_recovers_after_oversized_unterminated_input():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )

    sock.received.append(b"x" * (4 * 1024 + 1))
    assert ops.pump() == []

    push = {
        "push": "ops_state",
        "revision": 8,
        "authority_mode": "TELEOP",
    }
    sock.received.append((json.dumps(push) + "\n").encode("utf-8"))
    assert ops.pump() == [push]
    assert ops.latest_ops_state() == push


def test_pending_disconnect_recovers_final_via_correlated_status_query():
    clock = FakeClock()
    first = FakeSocket()
    second = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=SocketConnector(first, second),
    )
    request_id = ops.submit("estop_reset", phase="execute")
    first.received.extend([
        _response(request_id, "PENDING", revision=3, detail="accepted"),
        b"",
    ])

    assert ops.pump() == [
        {
            "request_id": request_id,
            "status": "PENDING",
            "state_revision": 3,
            "detail": "accepted",
        }
    ]
    assert first.closed is True

    clock.now = 1.0
    assert ops.pump() == []
    queries = _request_records(second)
    assert len(queries) == 1
    assert queries[0]["action"] == "status_query"
    assert queries[0]["params"] == {"request_id": request_id}
    assert queries[0]["request_id"] != request_id

    second.received.append(
        _response(
            queries[0]["request_id"],
            "FINAL_SUCCESS",
            revision=4,
            detail="reset complete",
            queried_request_id=request_id,
        )
    )
    assert ops.pump() == [
        {
            "request_id": request_id,
            "status": "FINAL_SUCCESS",
            "state_revision": 4,
            "detail": "reset complete",
        }
    ]


def test_reconnect_after_deadline_queries_without_resending_original():
    clock = FakeClock()
    first = FakeSocket()
    second = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=SocketConnector(first, second),
    )
    request_id = ops.submit("estop_reset", phase="execute")
    first.received.append(b"")
    assert ops.pump() == []
    assert first.closed is True

    clock.now = 2.1
    assert ops.pump() == []
    assert _request_records(second) == []

    clock.now = 2.35
    assert ops.pump() == []
    queries = _request_records(second)
    assert len(queries) == 1
    assert queries[0]["action"] == "status_query"
    assert queries[0]["params"] == {"request_id": request_id}

    second.received.append(
        _response(
            queries[0]["request_id"],
            "FINAL_SUCCESS",
            revision=5,
            detail="reset complete",
            queried_request_id=request_id,
        )
    )
    clock.now = 2.36
    assert ops.pump() == [
        {
            "request_id": request_id,
            "status": "FINAL_SUCCESS",
            "state_revision": 5,
            "detail": "reset complete",
        }
    ]


def test_submit_connector_crossing_deadline_does_not_send_original():
    clock = FakeClock()
    sock = FakeSocket()
    attempts = 0

    def connector(_host, _port):
        nonlocal attempts
        attempts += 1
        if attempts == 1:
            raise OSError("offline")
        clock.now = 3.1
        return sock

    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=connector,
    )
    clock.now = 1.0
    request_id = ops.submit("arm", phase="execute")

    assert _request_records(sock) == []
    clock.now = 3.35
    assert ops.pump() == []
    queries = _request_records(sock)
    assert len(queries) == 1
    assert queries[0]["action"] == "status_query"
    assert queries[0]["params"] == {"request_id": request_id}


def test_pending_stops_original_retransmit_but_pending_queries_continue():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )
    request_id = ops.submit("arm", phase="execute")
    sock.received.append(_response(request_id, "PENDING"))
    assert ops.pump()[0]["status"] == "PENDING"

    clock.now = 0.25
    assert ops.pump() == []
    first_query = _request_records(sock)[-1]
    assert first_query["action"] == "status_query"
    sock.received.append(
        _response(
            first_query["request_id"],
            "PENDING",
            queried_request_id=request_id,
        )
    )

    clock.now = 0.26
    assert ops.pump() == [
        {
            "request_id": request_id,
            "status": "PENDING",
            "state_revision": 1,
            "detail": "test",
        }
    ]
    clock.now = 0.50
    assert ops.pump() == []

    records = _request_records(sock)
    originals = [record for record in records if record["action"] == "arm"]
    queries = [
        record for record in records if record["action"] == "status_query"
    ]
    assert len(originals) == 1
    assert len(queries) == 2
    assert all(
        record["params"] == {"request_id": request_id}
        for record in queries
    )


def test_late_pending_after_request_deadline_still_enters_recovery():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )
    request_id = ops.submit("arm", phase="execute")

    clock.now = 2.0
    assert ops.pump() == []
    sock.received.append(_response(request_id, "PENDING"))
    clock.now = 2.01
    assert ops.pump()[0]["status"] == "PENDING"

    clock.now = 2.25
    assert ops.pump() == []
    queries = [
        record for record in _request_records(sock)
        if record["action"] == "status_query"
    ]
    assert len(queries) == 1
    assert queries[0]["params"] == {"request_id": request_id}


def test_awaiting_final_abandons_once_after_server_maximum_window():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )
    request_id = ops.submit("estop_reset", phase="execute")
    sock.received.append(_response(request_id, "PENDING", revision=8))
    assert ops.pump()[0]["status"] == "PENDING"

    sent_before_cutoff = len(_request_records(sock))
    clock.now = 11.999
    assert ops.pump() == []
    assert len(_request_records(sock)) == sent_before_cutoff
    clock.now = 12.0
    responses = ops.pump()
    assert len(responses) == 1
    assert responses[0]["request_id"] == request_id
    assert responses[0]["status"] == "OUTCOME_UNKNOWN"
    assert responses[0]["state_revision"] == 8

    clock.now = 12.5
    assert ops.pump() == []


def test_late_terminal_after_local_unknown_is_not_surfaced_twice():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )
    request_id = ops.submit("estop_reset", phase="execute")
    sock.received.append(_response(request_id, "PENDING"))
    assert ops.pump()[0]["status"] == "PENDING"

    clock.now = 12.0
    assert ops.pump()[0]["status"] == "OUTCOME_UNKNOWN"
    sock.received.append(_response(request_id, "FINAL_SUCCESS"))
    clock.now = 12.002
    assert ops.pump() == []


def test_status_query_reply_within_recovery_window_resolves_target():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )
    request_id = ops.submit("arm", phase="execute")
    sock.received.append(_response(request_id, "PENDING"))
    assert ops.pump()[0]["status"] == "PENDING"

    clock.now = 9.9
    assert ops.pump() == []
    query = _request_records(sock)[-1]
    sock.received.append(
        _response(
            query["request_id"],
            "FINAL_SUCCESS",
            queried_request_id=request_id,
        )
    )

    clock.now = 11.901
    assert ops.pump() == [
        {
            "request_id": request_id,
            "status": "FINAL_SUCCESS",
            "state_revision": 1,
            "detail": "test",
        }
    ]


def test_query_level_rejection_does_not_resolve_awaiting_target():
    clock = FakeClock()
    sock = FakeSocket()
    ops = client.OpsChannelClient(
        "jetson",
        9001,
        "tok-controller",
        clock=clock,
        connector=lambda _host, _port: sock,
    )
    request_id = ops.submit("arm", phase="execute")
    sock.received.append(_response(request_id, "PENDING"))
    assert ops.pump()[0]["status"] == "PENDING"

    clock.now = 0.25
    assert ops.pump() == []
    queries = [
        record for record in _request_records(sock)
        if record["action"] == "status_query"
    ]
    sock.received.append(
        _response(
            queries[-1]["request_id"],
            "FINAL_REJECTED",
            detail="rate limit exceeded",
        )
    )
    clock.now = 0.26
    assert ops.pump() == []

    clock.now = 0.50
    assert ops.pump() == []
    queries = [
        record for record in _request_records(sock)
        if record["action"] == "status_query"
    ]
    assert len(queries) == 2
