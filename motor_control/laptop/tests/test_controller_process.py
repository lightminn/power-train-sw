import json
import os
from pathlib import Path
import queue
import selectors
import socketserver
import subprocess
import sys
import threading
import time

import pytest

from motor_control.laptop import controller_process as controller
from motor_control.laptop.remote_operation_client import ClientInput


REPO_ROOT = Path(__file__).resolve().parents[3]


class FakeClock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


class FakeCommandReader:
    def __init__(self, *batches):
        self.batches = list(batches)

    def poll(self):
        if self.batches:
            return self.batches.pop(0)
        return controller.CommandBatch((), False, ())


class FakePadSource:
    def __init__(self, sample=None):
        self.connected = sample is not None
        self.sample = sample
        self.sample_after_reset = sample
        self.reset_count = 0
        self.closed = False

    def poll(self, *, now_ns):
        del now_ns
        return controller.PadReading(
            self.connected,
            self.sample if self.connected else None,
            "pad ready" if self.connected else "waiting for gamepad",
            self.connected,
        )

    def reset_for_new_connection(self):
        self.reset_count += 1
        self.sample = self.sample_after_reset

    def close(self):
        self.closed = True


class FakeSocket:
    def __init__(self):
        self.blocking = None
        self.sent = []
        self.closed = False

    def setblocking(self, value):
        self.blocking = value

    def sendall(self, record):
        self.sent.append(record)

    def recv(self, _size):
        raise BlockingIOError

    def close(self):
        self.closed = True


def _neutral_sample(**changes):
    values = {
        "requested_mode": "DRIVE",
        "deadman": False,
        "left_x": 0.0,
        "right_y": 0.0,
        "left_trigger": 0.0,
        "right_trigger": 0.0,
        "dpad_x": 0,
        "dpad_y": 0,
        "mode_chord": False,
        "estop_edge": False,
        "assist_bypass": False,
    }
    values.update(changes)
    return ClientInput(**values)


def _connect_command():
    return {
        "op": "connect",
        "host": "127.0.0.1",
        "input_port": 9000,
        "lease_id": "lease-new",
        "ticket": "secret-ticket",
    }


def _drive_until(process, predicate, attempts=100):
    for _ in range(attempts):
        process.tick()
        if predicate():
            return
        time.sleep(0.005)
    raise AssertionError("condition did not become true")


def _readline_with_timeout(stream, timeout_s=3.0):
    selector = selectors.DefaultSelector()
    selector.register(stream, selectors.EVENT_READ)
    try:
        assert selector.select(timeout_s), "controller child produced no status"
        return stream.readline()
    finally:
        selector.close()


def test_real_subprocess_waits_for_pad_and_stops_on_parent_eof():
    env = os.environ.copy()
    env.update(
        {
            "PYGAME_HIDE_SUPPORT_PROMPT": "1",
            "SDL_VIDEODRIVER": "dummy",
            "SDL_AUDIODRIVER": "dummy",
            "PYTHONPATH": os.pathsep.join(
                (
                    str(REPO_ROOT),
                    str(REPO_ROOT / "motor_control"),
                    str(REPO_ROOT / "ros2/src/powertrain_ros"),
                )
            ),
        }
    )
    process = subprocess.Popen(
        # Isolate device enumeration so a user's attached pad cannot invalidate
        # this absent-device fixture. The real child, pygame and EOF path run.
        [sys.executable, "-c",
         "import pygame, runpy; pygame.joystick.get_count = lambda: 0; "
         "runpy.run_module('motor_control.laptop.controller_process', run_name='__main__')"],
        cwd=REPO_ROOT,
        env=env,
        stdin=subprocess.PIPE,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
    )
    try:
        assert process.stdout is not None
        first = json.loads(_readline_with_timeout(process.stdout))
        assert first["pad_connected"] is False
        assert first["neutral"] is False
        assert first["input_connected"] is False
        assert "pad" in first["detail"].lower()

        assert process.stdin is not None
        process.stdin.close()
        assert process.wait(timeout=3.0) == 0
    finally:
        if process.poll() is None:
            process.kill()
            process.wait(timeout=3.0)


def test_unplug_closes_the_authenticated_input_channel_immediately():
    clock = FakeClock()
    pad = FakePadSource(_neutral_sample())
    sock = FakeSocket()
    statuses = []
    process = controller.ControllerProcess(
        command_reader=FakeCommandReader(
            controller.CommandBatch((_connect_command(),), False, ())
        ),
        pad_source=pad,
        open_channel=lambda *_args: sock,
        status_writer=statuses.append,
        clock=clock,
        monotonic_ns=lambda: int(clock.now * 1_000_000_000),
    )
    try:
        _drive_until(process, lambda: bool(sock.sent))
        assert sock.blocking is False

        pad.connected = False
        process.tick()

        assert sock.closed is True
        assert statuses[-1]["pad_connected"] is False
        assert statuses[-1]["neutral"] is False
        assert statuses[-1]["input_connected"] is False
    finally:
        process.close()


def test_parent_heartbeat_expiry_and_eof_stop_without_leaving_input_open():
    for terminal_batch, advance in (
        (controller.CommandBatch((), True, ()), 0.0),
        (controller.CommandBatch((), False, ()), 2.1),
    ):
        clock = FakeClock()
        pad = FakePadSource(_neutral_sample())
        sock = FakeSocket()
        reader = FakeCommandReader(
            controller.CommandBatch((_connect_command(),), False, ()),
            terminal_batch,
        )
        process = controller.ControllerProcess(
            command_reader=reader,
            pad_source=pad,
            open_channel=lambda *_args: sock,
            status_writer=lambda _status: None,
            clock=clock,
            monotonic_ns=lambda: int(clock.now * 1_000_000_000),
            heartbeat_timeout_s=2.0,
        )
        try:
            process.tick()
            clock.now += advance
            process.tick()
            assert process.running is False
            assert sock.closed is True or not sock.sent
        finally:
            process.close()


def test_heartbeat_refresh_disconnect_and_shutdown_commands_control_lifetime():
    clock = FakeClock()
    pad = FakePadSource(_neutral_sample())
    sock = FakeSocket()
    reader = FakeCommandReader(
        controller.CommandBatch((_connect_command(),), False, ())
    )
    process = controller.ControllerProcess(
        command_reader=reader,
        pad_source=pad,
        open_channel=lambda *_args: sock,
        status_writer=lambda _status: None,
        clock=clock,
        monotonic_ns=lambda: int(clock.now * 1_000_000_000),
        heartbeat_timeout_s=2.0,
    )
    try:
        _drive_until(process, lambda: bool(sock.sent))
        clock.now = 1.5
        reader.batches.append(
            controller.CommandBatch(({"op": "heartbeat"},), False, ())
        )
        process.tick()
        clock.now = 3.0
        process.tick()
        assert process.running is True

        reader.batches.append(
            controller.CommandBatch(({"op": "disconnect"},), False, ())
        )
        process.tick()
        assert sock.closed is True
        assert process.running is True

        reader.batches.append(
            controller.CommandBatch(({"op": "shutdown"},), False, ())
        )
        process.tick()
        assert process.running is False
    finally:
        process.close()


def test_input_connected_status_stays_non_neutral_until_post_reset_sample():
    clock = FakeClock()
    pad = FakePadSource(_neutral_sample())
    sock = FakeSocket()
    statuses = []
    process = controller.ControllerProcess(
        command_reader=FakeCommandReader(
            controller.CommandBatch((_connect_command(),), False, ())
        ),
        pad_source=pad,
        open_channel=lambda *_args: sock,
        status_writer=statuses.append,
        clock=clock,
        monotonic_ns=lambda: 0,
    )
    try:
        _drive_until(
            process,
            lambda: bool(statuses and statuses[-1]["input_connected"]),
        )
        assert statuses[-1]["neutral"] is False
        assert sock.sent == []

        process.tick()
        assert sock.sent
        assert statuses[-1]["neutral"] is True
    finally:
        process.close()


def test_reconnect_resets_adapter_and_first_wire_frame_uses_fresh_sample():
    clock = FakeClock()
    held = _neutral_sample(deadman=True, right_trigger=1.0, estop_edge=True)
    released = _neutral_sample()
    pad = FakePadSource(held)
    pad.sample_after_reset = released
    first = FakeSocket()
    second = FakeSocket()
    sockets = iter((first, second))
    reader = FakeCommandReader(
        controller.CommandBatch((_connect_command(),), False, ()),
    )
    process = controller.ControllerProcess(
        command_reader=reader,
        pad_source=pad,
        open_channel=lambda *_args: next(sockets),
        status_writer=lambda _status: None,
        clock=clock,
        monotonic_ns=lambda: int(clock.now * 1_000_000_000),
        session_id_factory=iter(("session-one", "session-two")).__next__,
    )
    try:
        _drive_until(process, lambda: bool(first.sent))
        first_wire = json.loads(first.sent[0])
        assert first_wire["session_id"] == "session-one"
        assert first_wire["sequence"] == 0
        assert first_wire["deadman"] is False
        assert first_wire["axes"]["right_trigger"] == 0.0
        assert first_wire["estop_edge"] is False

        pad.sample = held
        pad.sample_after_reset = released
        reader.batches.append(
            controller.CommandBatch((_connect_command(),), False, ())
        )
        _drive_until(process, lambda: bool(second.sent))
        second_wire = json.loads(second.sent[0])
        assert first.closed is True
        assert second_wire["session_id"] == "session-two"
        assert second_wire["sequence"] == 0
        assert second_wire["deadman"] is False
        assert second_wire["estop_edge"] is False
        assert pad.reset_count == 2
    finally:
        process.close()


def test_estop_edge_is_encoded_and_parent_ops_state_feeds_haptics():
    clock = FakeClock()
    pad = FakePadSource(_neutral_sample(estop_edge=True))
    sock = FakeSocket()

    class Arbiter:
        def __init__(self):
            self.states = []

        def feed_ops_state(self, state, received_s):
            self.states.append((state, received_s))

    arbiter = Arbiter()
    state = {"authority_mode": "TELEOP", "estop_latched": False}
    process = controller.ControllerProcess(
        command_reader=FakeCommandReader(
            controller.CommandBatch(
                (_connect_command(), {"op": "ops_state", "state": state}),
                False,
                (),
            )
        ),
        pad_source=pad,
        open_channel=lambda *_args: sock,
        status_writer=lambda _status: None,
        haptic_arbiter=arbiter,
        clock=clock,
        monotonic_ns=lambda: int(clock.now * 1_000_000_000),
    )
    try:
        _drive_until(process, lambda: bool(sock.sent))
        assert json.loads(sock.sent[0])["estop_edge"] is True
        assert arbiter.states == [(state, 0.0)]
    finally:
        process.close()


@pytest.mark.parametrize(
    "change",
    [
        {"deadman": True},
        {"left_x": 0.1},
        {"right_y": -0.1},
        {"left_trigger": 0.1},
        {"right_trigger": 0.1},
        {"dpad_x": 1},
        {"dpad_y": -1},
        {"mode_chord": True},
        {"estop_edge": True},
        {"assist_bypass": True},
    ],
)
def test_neutral_requires_every_control_to_be_released(change):
    assert controller.is_neutral(
        _neutral_sample(), buttons_released=True
    ) is True
    assert controller.is_neutral(
        _neutral_sample(**change), buttons_released=True
    ) is False
    assert controller.is_neutral(
        _neutral_sample(), buttons_released=False
    ) is False
    assert controller.is_neutral(None, buttons_released=True) is False


@pytest.mark.parametrize("held_button", [1, 8], ids=("circle", "create"))
def test_raw_held_button_never_becomes_neutral_after_edge_or_chord_settles(
    held_button,
):
    class Joystick:
        def __init__(self):
            self.buttons = {held_button: 1}

        def init(self):
            pass

        def get_init(self):
            return True

        def get_guid(self):
            return "fake-guid"

        def get_axis(self, index):
            return -1.0 if index in (2, 5) else 0.0

        def get_button(self, index):
            return self.buttons.get(index, 0)

        def get_numbuttons(self):
            return 10

        def get_hat(self, _index):
            return (0, 0)

        def quit(self):
            pass

    joystick = Joystick()

    class Pygame:
        def __init__(self):
            self.drained = 0
            self.joystick = type(
                "Joysticks",
                (),
                {
                    "init": staticmethod(lambda: None),
                    "get_count": staticmethod(lambda: 1),
                    "Joystick": staticmethod(lambda _index: joystick),
                },
            )()
            self.event = type(
                "Events",
                (),
                {
                    "pump": staticmethod(lambda: None),
                    "get": self._drain,
                },
            )()

        def _drain(self):
            self.drained += 1
            return []

        def init(self):
            pass

        def quit(self):
            pass

    pygame = Pygame()
    source = controller.PygamePadSource(pygame)
    try:
        first = source.poll(now_ns=0)
        second = source.poll(now_ns=1)
        assert first.buttons_released is False
        assert second.buttons_released is False
        assert controller.is_neutral(
            second.sample, buttons_released=second.buttons_released
        ) is False
        if held_button == 1:
            assert first.sample.estop_edge is True
            assert second.sample.estop_edge is False
        else:
            assert first.sample.mode_chord is False
            assert second.sample.mode_chord is False
        assert pygame.drained == 2
    finally:
        source.close()


def test_blocked_authentication_attempt_never_blocks_pad_polling_or_shutdown():
    clock = FakeClock()
    pad = FakePadSource(_neutral_sample())
    entered = threading.Event()
    release = threading.Event()
    late_socket = FakeSocket()

    def blocked_open(*_args):
        entered.set()
        release.wait(2.0)
        return late_socket

    process = controller.ControllerProcess(
        command_reader=FakeCommandReader(
            controller.CommandBatch((_connect_command(),), False, ())
        ),
        pad_source=pad,
        open_channel=blocked_open,
        status_writer=lambda _status: None,
        clock=clock,
        monotonic_ns=lambda: 0,
    )
    started = time.monotonic()
    process.tick()
    elapsed = time.monotonic() - started
    assert entered.wait(0.2)
    assert elapsed < 0.1

    process.close()
    release.set()
    deadline = time.monotonic() + 1.0
    while not late_socket.closed and time.monotonic() < deadline:
        time.sleep(0.005)
    assert late_socket.closed is True


def test_jsonl_reader_bounds_records_and_preserves_following_command():
    read_fd, write_fd = os.pipe()
    stream = os.fdopen(read_fd, "r", encoding="utf-8")
    try:
        os.write(write_fd, b"x" * (controller.MAX_COMMAND_BYTES + 1) + b"\n")
        os.write(write_fd, b'{"op":"heartbeat"}\n')
        os.close(write_fd)
        batch = controller.JsonlCommandReader(stream).poll()
        assert batch.commands == ({"op": "heartbeat"},)
        assert batch.eof is True
        assert batch.errors == ("command record exceeds 4 KiB",)
    finally:
        stream.close()


def test_local_singleton_rejects_duplicate_then_releases(tmp_path):
    path = tmp_path / "controller.lock"
    first = controller.ProcessLock(path)
    second = controller.ProcessLock(path)
    first.acquire()
    try:
        with pytest.raises(RuntimeError, match="already running"):
            second.acquire()
    finally:
        first.close()
    second.acquire()
    second.close()
    assert path.exists()


def test_authenticated_input_proxy_receives_real_encoded_frame(tmp_path):
    from powertrain_runtime.client import SessionClient
    from powertrain_runtime.session import SessionServer

    received = queue.Queue()

    class RecordingHandler(socketserver.BaseRequestHandler):
        def handle(self):
            buffer = bytearray()
            while chunk := self.request.recv(4096):
                buffer.extend(chunk)
                while b"\n" in buffer:
                    line, _, rest = buffer.partition(b"\n")
                    buffer[:] = rest
                    received.put(bytes(line))

    target = socketserver.ThreadingTCPServer(("127.0.0.1", 0), RecordingHandler)
    target.daemon_threads = True
    target_thread = threading.Thread(target=target.serve_forever, daemon=True)
    target_thread.start()
    token = tmp_path / "token"
    token.write_text("paired-secret-with-enough-entropy", encoding="utf-8")
    server = SessionServer(
        {
            "robot_id": "robot-one",
            "token_file": str(token),
            "host": "127.0.0.1",
            "session_port": 0,
            "input_port": 0,
            "ops_port": 0,
            "input_target_port": target.server_address[1],
            "ops_target_port": target.server_address[1],
            "destination_file": str(tmp_path / "session.json"),
            "lease_timeout_s": 1.0,
        },
        lambda: {"status": "SUCCEEDED"},
        lambda: {"status": "SUCCEEDED"},
        lambda: {"status": "IDLE", "ops_state": {
            "chassis_mode": "IDLE", "authority_mode": "IDLE",
            "estop_latched": False, "wheels_stopped": True,
            "field_age_s": {"authority": 0, "safety": 0, "wheels": 0},
        }},
    ).start()
    client = SessionClient(
        {
            "robot_id": "robot-one",
            "token_file": str(token),
            "hosts": ["127.0.0.1"],
            "session_port": server.session_port,
            "timeout_s": 0.5,
        }
    )
    snapshot = client.connect()
    process = controller.ControllerProcess(
        command_reader=FakeCommandReader(
            controller.CommandBatch(
                (
                    {
                        "op": "connect",
                        "host": client.host,
                        "input_port": snapshot["input_port"],
                        "lease_id": snapshot["lease_id"],
                        "ticket": snapshot["ticket"],
                    },
                ),
                False,
                (),
            )
        ),
        pad_source=FakePadSource(_neutral_sample(estop_edge=True)),
        status_writer=lambda _status: None,
        session_id_factory=lambda: "fresh-controller-session",
    )
    try:
        _drive_until(process, lambda: not received.empty())
        wire = json.loads(received.get_nowait())
        assert wire["session_id"] == "fresh-controller-session"
        assert wire["sequence"] == 0
        assert wire["estop_edge"] is True
    finally:
        process.close()
        client.close()
        server.close()
        target.shutdown()
        target.server_close()
