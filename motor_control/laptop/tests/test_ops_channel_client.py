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


def _wire_records(sock):
    return [json.loads(record) for record in sock.sent]


def _request_records(sock):
    return [record for record in _wire_records(sock) if not record.get("hello")]


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
