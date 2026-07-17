import inspect

from operator_console import ops_client


class FakeThread:
    instances = []

    def __init__(self, *, target, name, daemon):
        self.target = target
        self.name = name
        self.daemon = daemon
        self.started = False
        self.joined = False
        self.join_timeout = None
        self.instances.append(self)

    def start(self):
        self.started = True

    def join(self, timeout=None):
        self.joined = True
        self.join_timeout = timeout


class FakeClient:
    def __init__(self):
        self.submitted = []
        self.responses = []
        self.closed = False

    def submit(
        self,
        action,
        *,
        params=None,
        request_id=None,
        expected_state_revision=None,
    ):
        self.submitted.append({
            "action": action,
            "params": params,
            "request_id": request_id,
            "expected_state_revision": expected_state_revision,
        })
        return request_id

    def pump(self):
        responses, self.responses = self.responses, []
        return responses

    def close(self):
        self.closed = True


def _console(monkeypatch, *, submit_sink=None, state_sink=None, schedule=None):
    FakeThread.instances = []
    monkeypatch.setattr(ops_client.threading, "Thread", FakeThread)
    client = FakeClient()
    console = ops_client.ConsoleOpsClient(
        "robot",
        9001,
        "tok-console",
        submit_sink=submit_sink or (lambda _response: None),
        state_sink=state_sink or (lambda _state: None),
        schedule=schedule or (lambda callback: callback()),
        client_factory=lambda _host, _port, _token: client,
    )
    return console, client, FakeThread.instances[-1]


def test_submit_is_queued_then_forwarded_with_the_returned_request_id(monkeypatch):
    console, client, _thread = _console(monkeypatch)

    request_id = console.submit(
        "authority_manual",
        {"source": "console"},
        expected_state_revision=7,
    )
    assert client.submitted == []

    console.run_once()

    assert client.submitted == [{
        "action": "authority_manual",
        "params": {"source": "console"},
        "request_id": request_id,
        "expected_state_revision": 7,
    }]
    channel_parameters = inspect.signature(
        ops_client.ops_channel_client.OpsChannelClient.submit
    ).parameters
    assert "request_id" in channel_parameters
    assert "expected_state_revision" in channel_parameters


def test_ack_is_delivered_to_submit_sink_through_schedule(monkeypatch):
    scheduled = []
    received = []

    def schedule(callback):
        scheduled.append(callback)
        return callback()

    console, client, _thread = _console(
        monkeypatch,
        submit_sink=received.append,
        schedule=schedule,
    )
    request_id = console.submit("status_query")
    console.run_once()
    ack = {"request_id": request_id, "status": "FINAL_SUCCESS"}
    client.responses.append(ack)

    console.run_once()

    assert len(scheduled) == 1
    assert received == [ack]


def test_push_updates_latest_state_and_reaches_state_sink(monkeypatch):
    received = []
    console, client, _thread = _console(
        monkeypatch,
        state_sink=received.append,
    )
    console.run_once()
    push = {
        "push": "ops_state",
        "revision": 9,
        "authority_mode": "IDLE",
    }
    client.responses.append(push)

    console.run_once()

    assert received == [push]
    assert console.latest_state() == push
    push["revision"] = 10
    assert console.latest_state()["revision"] == 9


def test_send_queue_drops_oldest_when_more_than_sixteen_are_pending(monkeypatch):
    console, client, _thread = _console(monkeypatch)

    for index in range(17):
        console.submit("action-%d" % index)

    assert console.dropped_send_count == 1
    console.run_once()
    assert [item["action"] for item in client.submitted] == [
        "action-%d" % index for index in range(1, 17)
    ]


def test_close_stops_client_and_joins_worker(monkeypatch):
    console, client, thread = _console(monkeypatch)
    console.run_once()

    console.close()

    assert client.closed is True
    assert thread.joined is True
