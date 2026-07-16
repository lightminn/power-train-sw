import json
import uuid

import pytest

from powertrain_ros.remote_input import (
    CONTRACT_VIOLATION,
    MAX_RECORD_BYTES,
    RemoteInputDecoder,
)


def _record(session_id=None, sequence=0, **overrides):
    payload = {
        "schema_version": 2,
        "session_id": session_id or str(uuid.uuid4()),
        "sequence": sequence,
        "client_monotonic_ns": 10,
        "mode": "DRIVE",
        "deadman": False,
        "axes": {
            "left_x": 0.0,
            "right_y": 0.0,
            "left_trigger": 0.0,
            "right_trigger": 0.0,
        },
        "dpad": {"x": 0, "y": 0},
        "mode_chord": False,
        "estop_edge": False,
        "assist_bypass": False,
    }
    payload.update(overrides)
    return (json.dumps(payload, separators=(",", ":")) + "\n").encode()


def _one(decoder, record, now_s=1.0):
    results = decoder.feed(record, receive_monotonic_s=now_s)
    assert len(results) == 1
    return results[0]


def _assert_violation(result, detail=None):
    assert result.frame is None
    assert result.reason.startswith(CONTRACT_VIOLATION)
    if detail:
        assert detail in result.reason


def test_accepts_30hz_v2_frames_and_uses_jetson_receive_age_only():
    session_id = str(uuid.uuid4())
    decoder = RemoteInputDecoder(input_timeout_s=0.20)
    decoder.start_connection()

    for sequence in range(6):
        client_ns = 10**18 if sequence % 2 else 0
        result = _one(
            decoder,
            _record(
                session_id,
                sequence,
                client_monotonic_ns=client_ns,
            ),
            now_s=sequence / 30.0,
        )
        assert result.reason == ""
        assert result.frame.sequence == sequence
        assert result.frame.assist_bypass is False

    frame = result.frame
    assert frame.is_fresh(frame.received_monotonic_s + 0.20)
    assert not frame.is_fresh(frame.received_monotonic_s + 0.200001)


@pytest.mark.parametrize("sequence", [5, 4])
def test_rejects_duplicate_and_rollback_sequences(sequence):
    session_id = str(uuid.uuid4())
    decoder = RemoteInputDecoder()
    decoder.start_connection()
    assert _one(decoder, _record(session_id, 5)).frame is not None

    _assert_violation(
        _one(decoder, _record(session_id, sequence), now_s=1.1),
        "sequence",
    )


@pytest.mark.parametrize(
    "override, detail",
    [
        ({"schema_version": 1}, "schema_version"),
        ({"mode": "FLY"}, "mode"),
        (
            {
                "axes": {
                    "left_x": float("nan"),
                    "right_y": 0.0,
                    "left_trigger": 0.0,
                    "right_trigger": 0.0,
                }
            },
            "axis",
        ),
        (
            {
                "axes": {
                    "left_x": 1.01,
                    "right_y": 0.0,
                    "left_trigger": 0.0,
                    "right_trigger": 0.0,
                }
            },
            "axis",
        ),
        (
            {
                "axes": {
                    "left_x": 0.0,
                    "right_y": 0.0,
                    "left_trigger": -0.01,
                    "right_trigger": 0.0,
                }
            },
            "axis",
        ),
        ({"dpad": {"x": True, "y": 0}}, "dpad"),
        ({"assist_bypass": 1}, "assist_bypass"),
    ],
)
def test_rejects_unknown_contract_values_and_invalid_axes(override, detail):
    decoder = RemoteInputDecoder()
    decoder.start_connection()
    _assert_violation(_one(decoder, _record(**override)), detail)


def test_rejects_malformed_oversize_and_partial_records_without_frames():
    malformed = RemoteInputDecoder()
    malformed.start_connection()
    _assert_violation(_one(malformed, b"{bad json}\n"), "JSON")

    oversize = RemoteInputDecoder()
    oversize.start_connection()
    result = _one(
        oversize,
        b"{" + b"x" * MAX_RECORD_BYTES + b"}\n",
    )
    _assert_violation(result, "2 KiB")

    partial = RemoteInputDecoder()
    partial.start_connection()
    assert partial.feed(_record()[:-1], receive_monotonic_s=1.0) == []
    results = partial.end_connection()
    assert len(results) == 1
    _assert_violation(results[0], "partial")


def test_each_tcp_connection_requires_a_new_random_session():
    old_session = str(uuid.uuid4())
    new_session = str(uuid.uuid4())
    decoder = RemoteInputDecoder()

    decoder.start_connection()
    assert _one(decoder, _record(old_session, 0)).frame is not None
    assert decoder.end_connection() == []

    decoder.start_connection()
    _assert_violation(_one(decoder, _record(old_session, 0)), "session")
    assert _one(decoder, _record(new_session, 0), now_s=1.1).frame is not None


def test_session_cannot_be_replaced_inside_one_connection():
    first = str(uuid.uuid4())
    decoder = RemoteInputDecoder()
    decoder.start_connection()
    assert _one(decoder, _record(first, 0)).frame is not None

    _assert_violation(
        _one(decoder, _record(str(uuid.uuid4()), 1), now_s=1.1),
        "session",
    )


def test_record_limit_includes_newline_and_encoder_boundary_is_strict():
    decoder = RemoteInputDecoder()
    decoder.start_connection()
    assert len(_record()) < MAX_RECORD_BYTES

    result = _one(decoder, b" " * MAX_RECORD_BYTES + b"\n")
    _assert_violation(result, "2 KiB")


def test_one_chunk_has_bounded_results_and_summarizes_newline_flood():
    decoder = RemoteInputDecoder()
    decoder.start_connection()

    results = decoder.feed(b"\n" * 4096, receive_monotonic_s=1.0)

    assert len(results) <= 64
    assert results[-1].frame is None
    assert "suppressed" in results[-1].reason

    # The flood is fully consumed; the next valid frame is not poisoned by a
    # retained suffix from the oversized result batch.
    accepted = decoder.feed(_record(sequence=0), receive_monotonic_s=1.1)
    assert len(accepted) == 1
    assert accepted[0].frame is not None
