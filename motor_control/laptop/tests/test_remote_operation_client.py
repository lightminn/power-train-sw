import json
import sys

from motor_control.laptop import remote_operation_client as client


class FakeController:
    def __init__(self):
        self.axes = {0: -0.25, 2: -1.0, 3: 0.5, 5: 1.0}
        self.buttons = {1: 0, 3: 0, 4: 1, 8: 0, 9: 0}
        self.hat = (0, 0)

    def get_axis(self, index):
        return self.axes.get(index, 0.0)

    def get_button(self, index):
        return self.buttons.get(index, 0)

    def get_hat(self, _index):
        return self.hat


class FakeSocket:
    def __init__(self, fail_sends=0):
        self.fail_sends = fail_sends
        self.sent = []
        self.closed = False

    def sendall(self, payload):
        if self.fail_sends:
            self.fail_sends -= 1
            raise OSError("link down")
        self.sent.append(payload)

    def close(self):
        self.closed = True


def test_module_import_and_encoding_do_not_require_pygame():
    assert "pygame" not in client.__dict__
    assert "pygame" not in sys.modules or sys.modules["pygame"] is not client

    sample = client.ClientInput(
        requested_mode="DRIVE",
        deadman=True,
        left_x=0.25,
        right_y=-0.5,
        left_trigger=0.0,
        right_trigger=0.75,
        dpad_x=0,
        dpad_y=0,
        mode_chord=False,
        estop_edge=False,
    )
    payload = client.encode_frame(
        sample,
        session_id="3d594650-a06d-42c4-9693-284b3b973c3a",
        sequence=7,
        client_monotonic_ns=123,
    )
    wire = json.loads(payload)
    assert wire["schema_version"] == 1
    assert wire["sequence"] == 7
    assert wire["axes"]["right_trigger"] == 0.75

    assert payload.endswith(b"\n")
    assert len(payload) <= 2 * 1024


def test_default_guid_mapping_uses_measured_dualsense_indices():
    mapping = client.mapping_for_guid("unknown-guid")
    assert mapping["left_x_axis"] == 0
    assert mapping["right_trigger_axis"] == 5
    assert mapping["left_trigger_axis"] == 2
    assert mapping["square_button"] == 3
    assert mapping["circle_button"] == 1
    assert mapping["config_version"] == "v1-initial-candidate"


def test_fake_controller_maps_raw_input_and_emits_estop_edge_only_once():
    joystick = FakeController()
    reader = client.DualSenseInputAdapter(
        joystick,
        client.mapping_for_guid("fake-guid"),
    )
    sample = reader.sample(now_ns=0)
    assert sample.left_x == -0.25
    assert sample.right_y == 0.5
    assert sample.left_trigger == 0.0
    assert sample.right_trigger == 1.0
    assert sample.deadman is True

    joystick.buttons[1] = 1
    assert reader.sample(now_ns=1).estop_edge is True
    assert reader.sample(now_ns=2).estop_edge is False


def test_mode_chord_must_be_held_for_one_second_before_request_changes():
    joystick = FakeController()
    reader = client.DualSenseInputAdapter(
        joystick,
        client.mapping_for_guid("fake-guid"),
    )
    joystick.buttons[8] = 1
    joystick.buttons[9] = 1
    assert reader.sample(now_ns=0).requested_mode == "DRIVE"
    assert reader.sample(now_ns=999_999_999).requested_mode == "DRIVE"
    switched = reader.sample(now_ns=1_000_000_000)
    assert switched.requested_mode == "ARM"
    assert switched.mode_chord is True
    assert reader.sample(now_ns=2_000_000_000).requested_mode == "ARM"

    joystick.buttons[8] = 0
    joystick.buttons[9] = 0
    reader.sample(now_ns=2_100_000_000)
    joystick.buttons[8] = 1
    joystick.buttons[9] = 1
    reader.sample(now_ns=3_000_000_000)
    assert reader.sample(now_ns=4_000_000_000).requested_mode == "DRIVE"


def test_send_failure_reconnects_with_new_session_and_sequence_zero():
    first = FakeSocket(fail_sends=1)
    second = FakeSocket()
    sockets = iter((first, second))

    sender = client.RemoteOperationTransmitter(
        "jetson",
        9000,
        connector=lambda _host, _port: next(sockets),
        session_id_factory=iter(
            (
                "1b32fe4c-e3bd-43b4-974f-9b6633e7a436",
                "839aab12-8410-4f6f-a125-93febd0ef174",
            )
        ).__next__,
    )
    sample = client.ClientInput()
    sender.send(sample, client_monotonic_ns=10)
    assert first.closed
    first_wire = json.loads(second.sent[0])
    assert first_wire["session_id"] == "839aab12-8410-4f6f-a125-93febd0ef174"
    assert first_wire["sequence"] == 0

    sender.send(sample, client_monotonic_ns=20)
    second_wire = json.loads(second.sent[1])
    assert second_wire["session_id"] == first_wire["session_id"]
    assert second_wire["sequence"] == 1


def test_connect_forever_retries_fake_socket_until_success():
    attempts = []
    sleeps = []
    expected = FakeSocket()

    def connector(_host, _port):
        attempts.append(1)
        if len(attempts) < 3:
            raise OSError("not yet")
        return expected

    result = client.connect_with_retry(
        "jetson",
        9000,
        connector=connector,
        sleep_fn=sleeps.append,
        retries=0,
    )
    assert result is expected
    assert len(attempts) == 3
    assert sleeps == [1.0, 1.0]


def test_reconnect_callback_replaces_preserved_arm_mode_before_send():
    first = FakeSocket(fail_sends=1)
    second = FakeSocket()
    sockets = iter((first, second))
    reconnects = []

    def reset_input_mode():
        reconnects.append(1)
        return client.ClientInput(requested_mode="DRIVE")

    sender = client.RemoteOperationTransmitter(
        "jetson",
        connector=lambda _host, _port: next(sockets),
        session_id_factory=iter(
            (
                "ffb3d603-6be1-47ff-8a1e-e6e1030b37d9",
                "e131183c-c192-4f49-9d0a-36275a23be61",
            )
        ).__next__,
        on_reconnect=reset_input_mode,
    )
    sender.send(
        client.ClientInput(
            requested_mode="ARM",
            deadman=True,
            right_y=0.8,
        ),
        client_monotonic_ns=10,
    )
    wire = json.loads(second.sent[0])
    assert wire["mode"] == "DRIVE"
    assert wire["deadman"] is False
    assert reconnects == [1, 1]
