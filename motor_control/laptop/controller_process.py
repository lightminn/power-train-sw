"""Supervised DualSense input child for the integrated laptop runtime.

The parent controls this process with bounded JSONL records on stdin. This
module is the only integrated-runtime process that imports pygame; policy and
wire semantics continue to come from :mod:`remote_operation_client`.
"""

from dataclasses import dataclass
import errno
import fcntl
import json
import os
from pathlib import Path
import queue
import sys
import threading
import time
import uuid

from .dualsense_output import DualSenseOutput
from .haptic_arbiter import HapticArbiter
from .remote_operation_client import (
    ClientInput,
    DualSenseInputAdapter,
    SEND_HZ,
    encode_frame,
    mapping_for_guid,
)


MAX_COMMAND_BYTES = 4 * 1024
MAX_COMMAND_BUFFER_BYTES = 8 * 1024
MAX_COMMAND_READS_PER_TICK = 8
MAX_STATUS_BYTES = 1024
STATUS_INTERVAL_S = 0.5
HEARTBEAT_TIMEOUT_S = 2.0
CONNECT_RETRY_S = 0.5


@dataclass(frozen=True)
class CommandBatch:
    commands: tuple
    eof: bool
    errors: tuple


@dataclass(frozen=True)
class PadReading:
    connected: bool
    sample: ClientInput | None
    detail: str
    buttons_released: bool = False


def is_neutral(sample, *, buttons_released=False):
    """Return true only for a fully released, normalized input sample."""
    if not isinstance(sample, ClientInput):
        return False
    return (
        buttons_released
        and not sample.deadman
        and sample.left_x == 0.0
        and sample.right_y == 0.0
        and sample.left_trigger == 0.0
        and sample.right_trigger == 0.0
        and sample.dpad_x == 0
        and sample.dpad_y == 0
        and not sample.mode_chord
        and not sample.estop_edge
        and not sample.assist_bypass
    )


class JsonlCommandReader:
    """Nonblocking, bounded JSONL reader for the parent pipe."""

    def __init__(self, stream):
        self._fd = stream.fileno()
        os.set_blocking(self._fd, False)
        self._buffer = bytearray()
        self._eof = False
        self._discarding = False

    @staticmethod
    def _decode(line):
        try:
            value = json.loads(line.decode("utf-8"))
        except (UnicodeDecodeError, json.JSONDecodeError) as exc:
            return None, "invalid command JSON: %s" % exc
        if not isinstance(value, dict):
            return None, "command must be a JSON object"
        return value, None

    def _consume_line(self, line, commands, errors):
        if self._discarding:
            self._discarding = False
            errors.append("command record exceeds 4 KiB")
            return
        if len(line) > MAX_COMMAND_BYTES:
            errors.append("command record exceeds 4 KiB")
            return
        if not line.strip():
            return
        command, error = self._decode(line)
        if error is None:
            commands.append(command)
        else:
            errors.append(error)

    def poll(self):
        commands = []
        errors = []
        if not self._eof:
            for _ in range(MAX_COMMAND_READS_PER_TICK):
                try:
                    chunk = os.read(self._fd, 4096)
                except BlockingIOError:
                    break
                except OSError as exc:
                    if exc.errno in (errno.EAGAIN, errno.EWOULDBLOCK):
                        break
                    errors.append("parent command pipe failed: %s" % exc)
                    self._eof = True
                    break
                if not chunk:
                    self._eof = True
                    break
                self._buffer.extend(chunk)
                while b"\n" in self._buffer:
                    line, _, rest = self._buffer.partition(b"\n")
                    self._buffer[:] = rest
                    self._consume_line(line, commands, errors)
                if len(self._buffer) > MAX_COMMAND_BUFFER_BYTES:
                    self._buffer.clear()
                    self._discarding = True

        if self._eof and self._buffer:
            line = bytes(self._buffer)
            self._buffer.clear()
            self._consume_line(line, commands, errors)
        return CommandBatch(tuple(commands), self._eof, tuple(errors))


class JsonlStatusWriter:
    """Best-effort nonblocking writer retaining at most one small record."""

    def __init__(self, stream):
        self._fd = stream.fileno()
        os.set_blocking(self._fd, False)
        self._pending = bytearray()

    @staticmethod
    def _encode(status):
        record = (
            json.dumps(status, allow_nan=False, separators=(",", ":")) + "\n"
        ).encode("utf-8")
        if len(record) > MAX_STATUS_BYTES:
            raise ValueError("status record exceeds 1 KiB")
        return record

    def flush(self):
        if not self._pending:
            return
        try:
            written = os.write(self._fd, self._pending)
        except (BlockingIOError, BrokenPipeError):
            return
        except OSError:
            self._pending.clear()
            return
        del self._pending[:written]

    def __call__(self, status):
        self.flush()
        if self._pending:
            return
        self._pending.extend(self._encode(status))
        self.flush()


class PygamePadSource:
    """Hotplug-aware adapter around pygame's first joystick."""

    def __init__(self, pygame_module, *, mapping_version=None):
        self._pygame = pygame_module
        self._mapping_version = mapping_version
        self._joystick = None
        self._adapter = None
        pygame_module.init()
        pygame_module.joystick.init()

    def _drop(self):
        joystick = self._joystick
        self._joystick = None
        self._adapter = None
        if joystick is not None and hasattr(joystick, "quit"):
            try:
                joystick.quit()
            except Exception:
                pass

    def _open(self):
        joystick = self._pygame.joystick.Joystick(0)
        joystick.init()
        guid = (
            joystick.get_guid()
            if hasattr(joystick, "get_guid")
            else joystick.get_name()
        )
        self._joystick = joystick
        self._adapter = DualSenseInputAdapter(
            joystick,
            mapping_for_guid(guid, self._mapping_version),
        )

    def poll(self, *, now_ns):
        try:
            self._pygame.event.pump()
            # pygame requires event handling to keep hotplug state current.
            self._pygame.event.get()
            if self._pygame.joystick.get_count() <= 0:
                self._drop()
                return PadReading(False, None, "waiting for gamepad")
            if self._joystick is None:
                self._open()
            if hasattr(self._joystick, "get_init") and not self._joystick.get_init():
                self._drop()
                return PadReading(False, None, "gamepad disconnected")
            sample = self._adapter.sample(now_ns=now_ns)
            button_count = self._joystick.get_numbuttons()
            buttons_released = all(
                not self._joystick.get_button(index)
                for index in range(button_count)
            )
            return PadReading(
                True, sample, "gamepad connected", buttons_released
            )
        except Exception as exc:
            self._drop()
            return PadReading(False, None, "gamepad unavailable: %s" % exc)

    def reset_for_new_connection(self):
        if self._adapter is not None:
            self._adapter.reset_for_new_connection()

    def close(self):
        self._drop()
        self._pygame.quit()


def _default_open_channel(host, port, lease_id, ticket, kind):
    from powertrain_runtime.client import open_session_channel

    return open_session_channel(host, port, lease_id, ticket, kind)


def _safe_detail(value):
    text = str(value).replace("\r", " ").replace("\n", " ").strip()
    return text[:256] or "unknown"


class ControllerProcess:
    """One tickable controller/input-channel supervisor."""

    def __init__(
        self,
        *,
        command_reader,
        pad_source,
        status_writer,
        open_channel=_default_open_channel,
        haptic_arbiter=None,
        clock=time.monotonic,
        monotonic_ns=time.monotonic_ns,
        session_id_factory=lambda: str(uuid.uuid4()),
        heartbeat_timeout_s=HEARTBEAT_TIMEOUT_S,
    ):
        self.command_reader = command_reader
        self.pad_source = pad_source
        self.status_writer = status_writer
        self.open_channel = open_channel
        self.haptic_arbiter = haptic_arbiter
        self.clock = clock
        self.monotonic_ns = monotonic_ns
        self.session_id_factory = session_id_factory
        self.heartbeat_timeout_s = float(heartbeat_timeout_s)
        self.running = True
        self._last_parent_s = float(clock())
        self._last_status_s = float("-inf")
        self._last_status = None
        self._target = None
        self._socket = None
        self._session_id = None
        self._sequence = 0
        self._awaiting_fresh_sample = False
        self._last_connect_error = None
        self._next_connect_s = float("-inf")
        self._generation = 0
        self._wanted_generation = None
        self._pending_generation = None
        self._connector_lock = threading.Lock()
        self._connect_results = queue.Queue(maxsize=8)

    def _close_socket(self):
        sock = self._socket
        self._socket = None
        self._session_id = None
        self._sequence = 0
        self._awaiting_fresh_sample = False
        if sock is not None:
            try:
                sock.close()
            except OSError:
                pass

    def _cancel_connection(self):
        with self._connector_lock:
            self._generation += 1
            self._wanted_generation = None
            self._pending_generation = None
        self._close_socket()
        while True:
            try:
                _generation, sock, _error = self._connect_results.get_nowait()
            except queue.Empty:
                break
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass

    def _begin_connect(self):
        if self._target is None or self._pending_generation is not None:
            return
        self._generation += 1
        generation = self._generation
        self._wanted_generation = generation
        self._pending_generation = generation
        target = dict(self._target)

        def worker():
            sock = None
            error = None
            try:
                sock = self.open_channel(
                    target["host"],
                    target["input_port"],
                    target["lease_id"],
                    target["ticket"],
                    "input",
                )
                sock.setblocking(False)
            except (OSError, ValueError, TypeError) as exc:
                error = _safe_detail(exc)
                if sock is not None:
                    try:
                        sock.close()
                    except OSError:
                        pass
                    sock = None
            with self._connector_lock:
                if self._wanted_generation != generation:
                    if sock is not None:
                        try:
                            sock.close()
                        except OSError:
                            pass
                    return
                try:
                    self._connect_results.put_nowait((generation, sock, error))
                except queue.Full:
                    if sock is not None:
                        try:
                            sock.close()
                        except OSError:
                            pass

        threading.Thread(
            target=worker,
            name="controller-input-connect",
            daemon=True,
        ).start()

    def _accept_connect_results(self, now_s):
        while True:
            try:
                generation, sock, error = self._connect_results.get_nowait()
            except queue.Empty:
                return
            with self._connector_lock:
                current = self._wanted_generation == generation
                if current:
                    self._pending_generation = None
            if not current or not self.running or self._target is None:
                if sock is not None:
                    sock.close()
                continue
            if sock is None:
                self._last_connect_error = error or "input channel rejected"
                self._next_connect_s = now_s + CONNECT_RETRY_S
                continue
            self._socket = sock
            self._session_id = str(self.session_id_factory())
            self._sequence = 0
            self.pad_source.reset_for_new_connection()
            self._awaiting_fresh_sample = True
            self._last_connect_error = None

    def _set_target(self, command, now_s):
        required = ("host", "input_port", "lease_id", "ticket")
        if any(key not in command for key in required):
            self._last_connect_error = "connect command missing required field"
            return
        try:
            target = {
                "host": str(command["host"]),
                "input_port": int(command["input_port"]),
                "lease_id": str(command["lease_id"]),
                "ticket": str(command["ticket"]),
            }
        except (TypeError, ValueError, OverflowError):
            self._last_connect_error = "invalid connect command"
            return
        if not target["host"] or not target["lease_id"] or not target["ticket"]:
            self._last_connect_error = "invalid connect command"
            return
        self._cancel_connection()
        self._target = target
        self._last_connect_error = None
        self._next_connect_s = now_s

    def _terminate(self, detail):
        self.running = False
        self._target = None
        self._cancel_connection()
        self._emit_status(False, None, detail, force=True)

    def _handle_commands(self, batch, now_s):
        for error in batch.errors:
            self._last_connect_error = _safe_detail(error)
        for command in batch.commands:
            op = command.get("op")
            if op == "heartbeat":
                self._last_parent_s = now_s
            elif op == "connect":
                self._last_parent_s = now_s
                self._set_target(command, now_s)
            elif op == "disconnect":
                self._last_parent_s = now_s
                self._target = None
                self._cancel_connection()
                self._last_connect_error = None
            elif op == "shutdown":
                self._last_parent_s = now_s
                self._terminate("shutdown requested")
                return
            elif op == "ops_state":
                self._last_parent_s = now_s
                state = command.get("state")
                if self.haptic_arbiter is not None and isinstance(state, dict):
                    self.haptic_arbiter.feed_ops_state(state, received_s=now_s)
            else:
                self._last_connect_error = "unknown command"
        if batch.eof and self.running:
            self._terminate("parent stdin EOF")

    def _status(
        self, pad_connected, sample, detail, *, buttons_released=False
    ):
        return {
            "pad_connected": bool(pad_connected),
            "neutral": bool(
                pad_connected
                and is_neutral(sample, buttons_released=buttons_released)
            ),
            "input_connected": self._socket is not None,
            "detail": _safe_detail(detail),
        }

    def _emit_status(
        self,
        pad_connected,
        sample,
        detail,
        *,
        buttons_released=False,
        force=False,
    ):
        now_s = float(self.clock())
        status = self._status(
            pad_connected,
            sample,
            detail,
            buttons_released=buttons_released,
        )
        if (
            force
            or status != self._last_status
            or now_s - self._last_status_s >= STATUS_INTERVAL_S
        ):
            self.status_writer(status)
            self._last_status = status
            self._last_status_s = now_s

    def _drop_for_retry(self, now_s, detail):
        self._close_socket()
        self._last_connect_error = _safe_detail(detail)
        self._next_connect_s = now_s + CONNECT_RETRY_S

    def _send(self, sample, now_ns, now_s):
        record = encode_frame(
            sample,
            session_id=self._session_id,
            sequence=self._sequence,
            client_monotonic_ns=now_ns,
        )
        try:
            self._socket.sendall(record)
            self._sequence += 1
            try:
                reply = self._socket.recv(1024)
            except BlockingIOError:
                return
            if reply == b"":
                self._drop_for_retry(now_s, "input channel closed")
        except (BlockingIOError, OSError) as exc:
            self._drop_for_retry(now_s, "input send failed: %s" % exc)

    def tick(self):
        if not self.running:
            return False
        now_s = float(self.clock())
        batch = self.command_reader.poll()
        self._handle_commands(batch, now_s)
        if not self.running:
            return False
        if now_s - self._last_parent_s > self.heartbeat_timeout_s:
            self._terminate("parent heartbeat timeout")
            return False

        now_ns = int(self.monotonic_ns())
        reading = self.pad_source.poll(now_ns=now_ns)
        if not reading.connected or reading.sample is None:
            if self._socket is not None or self._pending_generation is not None:
                self._cancel_connection()
            self._emit_status(False, None, reading.detail)
            return True

        self._accept_connect_results(now_s)
        status_sample = reading.sample
        status_buttons_released = reading.buttons_released
        if self._awaiting_fresh_sample:
            self._awaiting_fresh_sample = False
            status_sample = None
            status_buttons_released = False
        elif self._socket is not None:
            self._send(reading.sample, now_ns, now_s)

        if (
            self._target is not None
            and self._socket is None
            and self._pending_generation is None
            and now_s >= self._next_connect_s
        ):
            self._begin_connect()

        if self._socket is not None:
            detail = "input connected"
        elif self._pending_generation is not None:
            detail = "connecting input"
        elif self._target is None:
            detail = "waiting for parent connect"
        else:
            detail = self._last_connect_error or "waiting to reconnect input"
        self._emit_status(
            True,
            status_sample,
            detail,
            buttons_released=status_buttons_released,
        )
        return True

    def close(self):
        self.running = False
        self._target = None
        self._cancel_connection()
        self.pad_source.close()


class ProcessLock:
    def __init__(self, path=None):
        if path is None:
            runtime = os.environ.get("XDG_RUNTIME_DIR")
            base = Path(runtime) if runtime else Path("/tmp")
            path = base / ("powertrain-controller-%d.lock" % os.getuid())
        self.path = os.fspath(path)
        self._fd = None

    def acquire(self):
        fd = os.open(self.path, os.O_RDWR | os.O_CREAT, 0o600)
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError:
            os.close(fd)
            raise RuntimeError("controller process already running") from None
        self._fd = fd

    def close(self):
        if self._fd is not None:
            fd = self._fd
            self._fd = None
            os.close(fd)


def main():
    os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
    writer = JsonlStatusWriter(sys.stdout)
    lock = ProcessLock()
    try:
        lock.acquire()
    except (OSError, RuntimeError) as exc:
        writer(
            {
                "pad_connected": False,
                "neutral": False,
                "input_connected": False,
                "detail": _safe_detail(exc),
            }
        )
        return 3

    pad_source = None
    haptic_output = None
    process = None
    try:
        try:
            import pygame

            pad_source = PygamePadSource(pygame)
        except (ImportError, OSError, RuntimeError) as exc:
            writer(
                {
                    "pad_connected": False,
                    "neutral": False,
                    "input_connected": False,
                    "detail": "pygame unavailable: %s" % _safe_detail(exc),
                }
            )
            return 2

        arbiter = HapticArbiter(clock=time.monotonic)
        haptic_output = DualSenseOutput(arbiter, clock=time.monotonic)
        haptic_output.start()
        process = ControllerProcess(
            command_reader=JsonlCommandReader(sys.stdin),
            pad_source=pad_source,
            status_writer=writer,
            haptic_arbiter=arbiter,
        )
        interval_s = 1.0 / SEND_HZ
        while process.running:
            started = time.monotonic()
            process.tick()
            writer.flush()
            remaining = interval_s - (time.monotonic() - started)
            if remaining > 0.0 and process.running:
                time.sleep(remaining)
        return 0
    except KeyboardInterrupt:
        return 0
    finally:
        if process is not None:
            process.close()
        elif pad_source is not None:
            pad_source.close()
        if haptic_output is not None:
            haptic_output.close()
        writer.flush()
        lock.close()


if __name__ == "__main__":
    raise SystemExit(main())
