#!/usr/bin/env python3
"""DualSense raw input client for the versioned Jetson teleop gateway.

Only this laptop-side executable imports pygame.  The wire encoder, mapping,
and reconnecting transmitter stay importable in pygame-free unit tests.
"""

import argparse
from dataclasses import dataclass
import json
import math
import socket
import time
import uuid


DEFAULT_HOST = "192.168.8.106"
DEFAULT_PORT = 9000
SEND_HZ = 30.0
SCHEMA_VERSION = 2
MAX_RECORD_BYTES = 2 * 1024
MODE_CHORD_HOLD_NS = 1_000_000_000
TRIGGER_DEADZONE = 0.03
# 실측 DualSense 휴지 드리프트 left_x -0.0118 / right_y +0.0431 (2026-07-17 벤치).
# 게이트웨이 중립 게이트는 정확히 0.0을 요구하므로(계약 유지) 입력 정형화는
# 어댑터 몫이다 — 데드존이 없으면 영원히 DISCONNECTED에 갇힌다.
STICK_DEADZONE = 0.08


def normalize_stick(raw):
    value = _finite_clamped(raw, -1.0, 1.0)
    return value if abs(value) > STICK_DEADZONE else 0.0


# SDL GUID-specific, versioned configuration.  The wildcard is the measured
# initial DualSense candidate and intentionally remains labeled as such.
# FS-style v1a will replace/add GUID entries only after operator agreement;
# changing that future mapping must not silently mutate this version.
SDL_GUID_CONFIGS = {
    "*": {
        "v1-initial-candidate": {
            "config_version": "v1-initial-candidate",
            "left_x_axis": 0,          # measured LX
            "right_y_axis": 3,         # initial candidate; verify per GUID
            "right_trigger_axis": 5,   # measured RT
            "left_trigger_axis": 2,    # measured LT
            "square_button": 3,        # measured square; reserved
            "circle_button": 1,        # measured circle / E-stop
            "deadman_button": 4,       # L1 initial candidate
            "create_button": 8,        # initial candidate
            "options_button": 9,       # initial candidate
            "dpad_hat": 0,
        },
        # Versioned initial candidate: R1 hold bypasses assist.  This may
        # change only through a new mapping after HIL and operator feedback.
        "v2-initial-candidate": {
            "config_version": "v2-initial-candidate",
            "left_x_axis": 0,          # measured LX
            "right_y_axis": 3,         # initial candidate; verify per GUID
            "right_trigger_axis": 5,   # measured RT
            "left_trigger_axis": 2,    # measured LT
            "square_button": 3,        # measured square; reserved
            "circle_button": 1,        # measured circle / E-stop
            "deadman_button": 4,       # L1 initial candidate
            "assist_bypass_button": 5,  # R1 hold initial candidate
            "create_button": 8,        # initial candidate
            "options_button": 9,       # initial candidate
            "dpad_hat": 0,
        }
    }
}


@dataclass(frozen=True)
class ClientInput:
    requested_mode: str = "DRIVE"
    deadman: bool = False
    left_x: float = 0.0
    right_y: float = 0.0
    left_trigger: float = 0.0
    right_trigger: float = 0.0
    dpad_x: int = 0
    dpad_y: int = 0
    mode_chord: bool = False
    estop_edge: bool = False
    assist_bypass: bool = False


def mapping_for_guid(guid, config_version=None):
    """Return a copy so callers cannot mutate the versioned source table."""
    versions = SDL_GUID_CONFIGS.get(str(guid).lower())
    if versions is None:
        versions = SDL_GUID_CONFIGS["*"]
    if config_version is None:
        config_version = next(reversed(versions))
    try:
        mapping = dict(versions[config_version])
    except KeyError:
        raise ValueError(
            "unknown mapping version %r for SDL GUID %s"
            % (config_version, guid)
        ) from None
    if SCHEMA_VERSION >= 2 and "assist_bypass_button" not in mapping:
        raise ValueError(
            "mapping version %r has no assist_bypass button for schema v2"
            % config_version
        )
    return mapping


def _finite_clamped(value, low, high):
    value = float(value)
    if not math.isfinite(value):
        return 0.0
    return max(low, min(high, value))


def normalize_trigger(raw):
    value = (_finite_clamped(raw, -1.0, 1.0) + 1.0) / 2.0
    return value if value > TRIGGER_DEADZONE else 0.0


class DualSenseInputAdapter:
    """Map one pygame-compatible joystick object into stable wire semantics."""

    def __init__(self, joystick, mapping):
        self.joystick = joystick
        self.mapping = dict(mapping)
        self.requested_mode = "DRIVE"
        self._chord_started_ns = None
        self._chord_latched = False
        self._last_circle = False

    def reset_for_new_connection(self):
        """Never carry a requested mode or button edge into a new session."""
        self.requested_mode = "DRIVE"
        self._chord_started_ns = None
        self._chord_latched = False
        self._last_circle = False

    def _button(self, name):
        return bool(self.joystick.get_button(self.mapping[name]))

    def sample(self, *, now_ns):
        now_ns = int(now_ns)
        create = self._button("create_button")
        options = self._button("options_button")
        chord = create and options
        if chord:
            if self._chord_started_ns is None:
                self._chord_started_ns = now_ns
            if (
                not self._chord_latched
                and now_ns - self._chord_started_ns >= MODE_CHORD_HOLD_NS
            ):
                self.requested_mode = (
                    "ARM" if self.requested_mode == "DRIVE" else "DRIVE"
                )
                self._chord_latched = True
        else:
            self._chord_started_ns = None
            self._chord_latched = False

        circle = self._button("circle_button")
        estop_edge = circle and not self._last_circle
        self._last_circle = circle
        dpad_x, dpad_y = self.joystick.get_hat(self.mapping["dpad_hat"])

        return ClientInput(
            requested_mode=self.requested_mode,
            deadman=self._button("deadman_button"),
            left_x=normalize_stick(
                self.joystick.get_axis(self.mapping["left_x_axis"])
            ),
            right_y=normalize_stick(
                self.joystick.get_axis(self.mapping["right_y_axis"])
            ),
            left_trigger=normalize_trigger(
                self.joystick.get_axis(self.mapping["left_trigger_axis"])
            ),
            right_trigger=normalize_trigger(
                self.joystick.get_axis(self.mapping["right_trigger_axis"])
            ),
            dpad_x=int(max(-1, min(1, dpad_x))),
            dpad_y=int(max(-1, min(1, dpad_y))),
            mode_chord=chord,
            estop_edge=estop_edge,
            assist_bypass=self._button("assist_bypass_button"),
        )


def encode_frame(
    sample,
    *,
    session_id,
    sequence,
    client_monotonic_ns,
):
    payload = {
        "schema_version": SCHEMA_VERSION,
        "session_id": str(session_id),
        "sequence": int(sequence),
        "client_monotonic_ns": int(client_monotonic_ns),
        "mode": sample.requested_mode,
        "deadman": bool(sample.deadman),
        "axes": {
            "left_x": float(sample.left_x),
            "right_y": float(sample.right_y),
            "left_trigger": float(sample.left_trigger),
            "right_trigger": float(sample.right_trigger),
        },
        "dpad": {"x": int(sample.dpad_x), "y": int(sample.dpad_y)},
        "mode_chord": bool(sample.mode_chord),
        "estop_edge": bool(sample.estop_edge),
        "assist_bypass": bool(sample.assist_bypass),
    }
    try:
        record = (
            json.dumps(
                payload,
                allow_nan=False,
                separators=(",", ":"),
                sort_keys=True,
            )
            + "\n"
        ).encode("utf-8")
    except (TypeError, ValueError, OverflowError) as exc:
        raise ValueError("remote input cannot be encoded: %s" % exc) from exc
    if len(record) > MAX_RECORD_BYTES:
        raise ValueError("remote-input record exceeds 2 KiB")
    return record


def open_gateway_socket(host, port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    try:
        sock.settimeout(3.0)
        sock.connect((host, int(port)))
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_KEEPALIVE, 1)
        if hasattr(socket, "TCP_USER_TIMEOUT"):
            sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_USER_TIMEOUT, 5000)
        sock.setblocking(False)
        return sock
    except BaseException:
        sock.close()
        raise


def connect_with_retry(
    host,
    port,
    *,
    connector=open_gateway_socket,
    sleep_fn=time.sleep,
    retries=0,
):
    """Connect forever when retries is zero, matching the legacy client."""
    attempt = 0
    while retries == 0 or attempt < retries:
        attempt += 1
        try:
            return connector(host, port)
        except OSError:
            if retries and attempt >= retries:
                break
            sleep_fn(1.0)
    return None


class RemoteOperationTransmitter:
    """Reconnect transparently, but create a new session on every socket."""

    def __init__(
        self,
        host,
        port=DEFAULT_PORT,
        *,
        connector=open_gateway_socket,
        session_id_factory=lambda: str(uuid.uuid4()),
        sleep_fn=time.sleep,
        on_reconnect=None,
    ):
        self.host = host
        self.port = int(port)
        self.connector = connector
        self.session_id_factory = session_id_factory
        self.sleep_fn = sleep_fn
        self.on_reconnect = on_reconnect
        self.sock = None
        self.session_id = None
        self.sequence = 0
        self._status_buffer = bytearray()

    def _connect(self):
        self.sock = connect_with_retry(
            self.host,
            self.port,
            connector=self.connector,
            sleep_fn=self.sleep_fn,
            retries=0,
        )
        self.session_id = str(self.session_id_factory())
        self.sequence = 0
        self._status_buffer.clear()
        if self.on_reconnect is not None:
            return self.on_reconnect()
        return None

    def _drop(self):
        if self.sock is not None:
            try:
                self.sock.close()
            except OSError:
                pass
        self.sock = None
        self.session_id = None
        self.sequence = 0
        self._status_buffer.clear()

    def send(self, sample, *, client_monotonic_ns):
        while True:
            if self.sock is None:
                replacement = self._connect()
                if replacement is not None:
                    sample = replacement
            record = encode_frame(
                sample,
                session_id=self.session_id,
                sequence=self.sequence,
                client_monotonic_ns=client_monotonic_ns,
            )
            try:
                self.sock.sendall(record)
            except (BlockingIOError, OSError):
                self._drop()
                continue
            self.sequence += 1
            return

    def receive_status(self):
        if self.sock is None or not hasattr(self.sock, "recv"):
            return []
        try:
            chunk = self.sock.recv(1024)
        except (BlockingIOError, OSError):
            return []
        if not chunk:
            return []
        self._status_buffer.extend(chunk)
        lines = []
        while b"\n" in self._status_buffer:
            line, _, rest = self._status_buffer.partition(b"\n")
            self._status_buffer[:] = rest
            lines.append(line.decode("utf-8", errors="replace"))
        return lines

    def close(self):
        self._drop()


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="WP5.2 DRIVE/ARM remote operation client"
    )
    parser.add_argument("--host", default=DEFAULT_HOST)
    parser.add_argument("--port", type=int, default=DEFAULT_PORT)
    parser.add_argument("--mapping-version", default=None)
    args = parser.parse_args(argv)

    # pygame belongs only to the laptop executable path.
    import pygame

    pygame.init()
    pygame.joystick.init()
    if pygame.joystick.get_count() == 0:
        raise SystemExit("DualSense controller not found")
    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    guid = (
        joystick.get_guid()
        if hasattr(joystick, "get_guid")
        else joystick.get_name()
    )
    mapping = mapping_for_guid(guid, args.mapping_version)
    adapter = DualSenseInputAdapter(joystick, mapping)

    def input_after_reconnect():
        adapter.reset_for_new_connection()
        pygame.event.pump()
        return adapter.sample(now_ns=time.monotonic_ns())

    transmitter = RemoteOperationTransmitter(
        args.host,
        args.port,
        on_reconnect=input_after_reconnect,
    )
    interval_s = 1.0 / SEND_HZ

    print(
        "controller=%s guid=%s mapping=%s gateway=%s:%d"
        % (
            joystick.get_name(),
            guid,
            mapping["config_version"],
            args.host,
            args.port,
        )
    )
    try:
        while True:
            started = time.monotonic()
            pygame.event.pump()
            sample = adapter.sample(now_ns=time.monotonic_ns())
            transmitter.send(
                sample,
                client_monotonic_ns=time.monotonic_ns(),
            )
            for status in transmitter.receive_status():
                if status.startswith("S "):
                    print("\r%s" % status, end="", flush=True)
            remaining = interval_s - (time.monotonic() - started)
            if remaining > 0.0:
                time.sleep(remaining)
    except KeyboardInterrupt:
        pass
    finally:
        transmitter.close()
        pygame.quit()
        print()


if __name__ == "__main__":
    main()
