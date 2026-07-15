"""Versioned, fail-closed remote-input wire contract.

The transport is newline-delimited JSON.  This module deliberately has no
ROS, socket, or pygame imports so the same contract can be exercised on a
laptop and on the Jetson.  TCP connection lifetime is part of the contract:
every connection must start with a previously unseen UUIDv4 session.
"""

from dataclasses import dataclass
import json
import math
import uuid


SCHEMA_VERSION = 1
MAX_RECORD_BYTES = 2 * 1024
DEFAULT_INPUT_TIMEOUT_S = 0.20
CONTRACT_VIOLATION = "CONTRACT_VIOLATION:"
MODES = ("DRIVE", "ARM")

_TOP_LEVEL_FIELDS = {
    "schema_version",
    "session_id",
    "sequence",
    "client_monotonic_ns",
    "mode",
    "deadman",
    "axes",
    "dpad",
    "mode_chord",
    "estop_edge",
}
_AXIS_FIELDS = {
    "left_x",
    "right_y",
    "left_trigger",
    "right_trigger",
}
_DPAD_FIELDS = {"x", "y"}


@dataclass(frozen=True)
class NormalizedAxes:
    left_x: float
    right_y: float
    left_trigger: float
    right_trigger: float


@dataclass(frozen=True)
class DPad:
    x: int
    y: int


@dataclass(frozen=True)
class RemoteInputFrame:
    schema_version: int
    session_id: str
    sequence: int
    client_monotonic_ns: int
    mode: str
    deadman: bool
    axes: NormalizedAxes
    dpad: DPad
    mode_chord: bool
    estop_edge: bool
    received_monotonic_s: float
    input_timeout_s: float = DEFAULT_INPUT_TIMEOUT_S

    def is_fresh(self, now_s: float) -> bool:
        """Use only Jetson-local receive age for the safety decision."""
        try:
            age_s = float(now_s) - self.received_monotonic_s
        except (TypeError, ValueError):
            return False
        return (
            math.isfinite(age_s)
            and age_s >= -1e-12
            and age_s <= self.input_timeout_s + 1e-12
        )


@dataclass(frozen=True)
class ParseResult:
    frame: RemoteInputFrame = None
    reason: str = ""


def _violation(detail):
    return ParseResult(reason="%s %s" % (CONTRACT_VIOLATION, detail))


def _exact_fields(value, fields, label):
    if not isinstance(value, dict):
        raise ValueError("%s must be an object" % label)
    missing = fields - set(value)
    extra = set(value) - fields
    if missing:
        raise ValueError("%s missing fields: %s" % (label, sorted(missing)))
    if extra:
        raise ValueError("%s unknown fields: %s" % (label, sorted(extra)))


def _strict_int(value, label, minimum=0):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("%s must be an integer" % label)
    if value < minimum:
        raise ValueError("%s must be >= %d" % (label, minimum))
    return value


def _strict_bool(value, label):
    if not isinstance(value, bool):
        raise ValueError("%s must be a boolean" % label)
    return value


def _axis(value, label, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError("axis %s must be numeric" % label)
    result = float(value)
    if not math.isfinite(result) or result < low or result > high:
        raise ValueError(
            "axis %s must be finite and in [%s, %s]"
            % (label, low, high)
        )
    return result


def _dpad_axis(value, label):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError("dpad.%s must be an integer" % label)
    if value not in (-1, 0, 1):
        raise ValueError("dpad.%s must be -1, 0, or 1" % label)
    return value


def _session_id(value):
    if not isinstance(value, str):
        raise ValueError("session_id must be a UUIDv4 string")
    try:
        parsed = uuid.UUID(value)
    except (AttributeError, ValueError):
        raise ValueError("session_id must be a UUIDv4 string") from None
    if parsed.version != 4 or str(parsed) != value:
        raise ValueError("session_id must be a canonical UUIDv4 string")
    return value


class RemoteInputDecoder:
    """Decode one TCP stream while retaining used sessions across reconnects."""

    def __init__(self, *, input_timeout_s=DEFAULT_INPUT_TIMEOUT_S):
        self.input_timeout_s = float(input_timeout_s)
        if (
            not math.isfinite(self.input_timeout_s)
            or self.input_timeout_s <= 0.0
        ):
            raise ValueError("input_timeout_s must be finite and positive")
        self._seen_sessions = set()
        self._active = False
        self._buffer = bytearray()
        self._discard_oversize = False
        self._bound_session = None
        self._last_sequence = None

    def start_connection(self):
        self._active = True
        self._buffer.clear()
        self._discard_oversize = False
        self._bound_session = None
        self._last_sequence = None

    def end_connection(self):
        results = []
        if self._buffer and not self._discard_oversize:
            results.append(_violation("partial record at TCP close"))
        self._active = False
        self._buffer.clear()
        self._discard_oversize = False
        self._bound_session = None
        self._last_sequence = None
        return results

    def feed(self, data, *, receive_monotonic_s):
        if not self._active:
            raise RuntimeError("start_connection() is required before feed()")
        if not isinstance(data, (bytes, bytearray)):
            return [_violation("record bytes required")]
        try:
            received_s = float(receive_monotonic_s)
        except (TypeError, ValueError):
            return [_violation("invalid Jetson receive monotonic time")]
        if not math.isfinite(received_s):
            return [_violation("invalid Jetson receive monotonic time")]

        self._buffer.extend(data)
        results = []
        while True:
            newline = self._buffer.find(b"\n")
            if newline < 0:
                if len(self._buffer) > MAX_RECORD_BYTES:
                    self._buffer.clear()
                    self._discard_oversize = True
                    results.append(_violation("record exceeds 2 KiB"))
                break

            record = bytes(self._buffer[: newline + 1])
            del self._buffer[: newline + 1]
            if self._discard_oversize:
                self._discard_oversize = False
                continue
            if len(record) > MAX_RECORD_BYTES:
                results.append(_violation("record exceeds 2 KiB"))
                continue
            results.append(self._decode_record(record, received_s))
        return results

    def _decode_record(self, record, received_s):
        try:
            text = record[:-1].decode("utf-8", errors="strict")
        except UnicodeDecodeError:
            return _violation("record is not valid UTF-8")
        try:
            payload = json.loads(text)
        except (json.JSONDecodeError, RecursionError):
            return _violation("malformed JSON")

        try:
            _exact_fields(payload, _TOP_LEVEL_FIELDS, "record")
            version = _strict_int(payload["schema_version"], "schema_version")
            if version != SCHEMA_VERSION:
                raise ValueError("unrecognized schema_version: %s" % version)
            session_id = _session_id(payload["session_id"])
            sequence = _strict_int(payload["sequence"], "sequence")
            client_ns = _strict_int(
                payload["client_monotonic_ns"],
                "client_monotonic_ns",
            )
            mode = payload["mode"]
            if mode not in MODES:
                raise ValueError("unrecognized mode: %r" % mode)
            deadman = _strict_bool(payload["deadman"], "deadman")
            mode_chord = _strict_bool(
                payload["mode_chord"],
                "mode_chord",
            )
            estop_edge = _strict_bool(
                payload["estop_edge"],
                "estop_edge",
            )

            axes = payload["axes"]
            _exact_fields(axes, _AXIS_FIELDS, "axes")
            parsed_axes = NormalizedAxes(
                left_x=_axis(axes["left_x"], "left_x", -1.0, 1.0),
                right_y=_axis(axes["right_y"], "right_y", -1.0, 1.0),
                left_trigger=_axis(
                    axes["left_trigger"],
                    "left_trigger",
                    0.0,
                    1.0,
                ),
                right_trigger=_axis(
                    axes["right_trigger"],
                    "right_trigger",
                    0.0,
                    1.0,
                ),
            )

            dpad = payload["dpad"]
            _exact_fields(dpad, _DPAD_FIELDS, "dpad")
            dpad_x = _dpad_axis(dpad["x"], "x")
            dpad_y = _dpad_axis(dpad["y"], "y")
            parsed_dpad = DPad(dpad_x, dpad_y)

            if self._bound_session is None:
                if session_id in self._seen_sessions:
                    raise ValueError(
                        "session_id was already used by a prior TCP connection"
                    )
            elif session_id != self._bound_session:
                raise ValueError("session_id changed inside one TCP connection")
            if (
                self._last_sequence is not None
                and sequence <= self._last_sequence
            ):
                raise ValueError(
                    "sequence must increase strictly (last=%d got=%d)"
                    % (self._last_sequence, sequence)
                )
        except (KeyError, TypeError, ValueError) as exc:
            return _violation(str(exc))

        if self._bound_session is None:
            self._bound_session = session_id
            self._seen_sessions.add(session_id)
        self._last_sequence = sequence
        return ParseResult(
            frame=RemoteInputFrame(
                schema_version=version,
                session_id=session_id,
                sequence=sequence,
                client_monotonic_ns=client_ns,
                mode=mode,
                deadman=deadman,
                axes=parsed_axes,
                dpad=parsed_dpad,
                mode_chord=mode_chord,
                estop_edge=estop_edge,
                received_monotonic_s=received_s,
                input_timeout_s=self.input_timeout_s,
            )
        )
