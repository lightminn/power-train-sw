import pytest

from l515_dashboard.protocol import ProtocolError, decode_request, encode_message


def request(kind="get_status", payload=None):
    return {"protocol_version": 1, "request_id": "r1", "type": kind,
            "payload": {} if payload is None else payload}


def test_newline_json_round_trip():
    wire = encode_message(request(), 1024)
    assert wire.endswith(b"\n")
    assert decode_request(wire[:-1], 1024)["request_id"] == "r1"


@pytest.mark.parametrize("change", [
    {"protocol_version": 2}, {"type": "unknown"},
    {"request_id": ""}, {"payload": []},
])
def test_invalid_envelopes_are_rejected(change):
    message = request(); message.update(change)
    with pytest.raises(ProtocolError):
        decode_request(encode_message(message, 1024).rstrip(b"\n"), 1024)


def test_invalid_command_payload_and_oversize_are_rejected():
    with pytest.raises(ProtocolError):
        decode_request(encode_message(request("set_streaming", {"enabled": 1}), 1024).rstrip(), 1024)
    with pytest.raises(ProtocolError):
        decode_request(b"{" + b"x" * 100, 32)


def test_validation_error_preserves_parsed_request_id():
    with pytest.raises(ProtocolError) as caught:
        decode_request(encode_message(request("unknown"), 1024).rstrip(), 1024)
    assert caught.value.request_id == "r1"


def test_invalid_request_id_is_never_echoed():
    message=request(); message["request_id"]=7
    with pytest.raises(ProtocolError) as caught:
        decode_request(encode_message(message,1024).rstrip(),1024)
    assert caught.value.request_id is None
