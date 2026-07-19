"""CoordSender — 좌표 UDP 채널이 영상과 독립적으로 살아남는지 검증.

연막탄 등으로 영상(SRT)이 깨지거나 끊겨도, 좌표는 작은 UDP 데이터그램이라
별도 채널로 계속 도착해야 한다. 특히 "표적 없음"(빈 dets 라도 패킷은 옴)과
"통신 두절"(패킷 자체가 안 옴)을 수신측이 구분할 수 있어야 하므로, 검출이
하나도 없는 프레임에서도 하트비트 패킷을 반드시 보내는지가 핵심 회귀 포인트다.
"""
import json
import socket

import pytest

from yolo_depth_3d import CoordSender


@pytest.fixture
def udp_listener():
    recv = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    recv.bind(("127.0.0.1", 0))
    recv.settimeout(1.0)
    yield recv
    recv.close()


def test_heartbeat_sent_even_with_no_detections(udp_listener):
    port = udp_listener.getsockname()[1]
    sender = CoordSender("127.0.0.1", port)

    sender.send(frame_idx=42, w=848, h=480, dets=[])

    data, _ = udp_listener.recvfrom(4096)
    pkt = json.loads(data.decode())
    assert pkt["frame"] == 42
    assert pkt["w"] == 848
    assert pkt["h"] == 480
    assert pkt["dets"] == []  # 빈 리스트지만 패킷 자체는 도착 — "표적 없음"
    assert "t" in pkt  # 수신측이 마지막 도착 시각과 비교해 두절을 판정하는 근거
    assert isinstance(pkt["session_id"], str) and pkt["session_id"]


def test_detection_payload_round_trips_through_udp(udp_listener):
    port = udp_listener.getsockname()[1]
    sender = CoordSender("127.0.0.1", port)
    dets = [{"cls": "person", "conf": 0.87, "box": [10, 20, 110, 220],
             "xyz": [0.5, -0.1, 3.2], "d": 3.24, "az": 8.9, "el": -1.8}]

    sender.send(frame_idx=7, w=848, h=480, dets=dets)

    data, _ = udp_listener.recvfrom(4096)
    pkt = json.loads(data.decode())
    assert pkt["dets"] == dets


def test_consecutive_frames_are_each_a_separate_datagram(udp_listener):
    port = udp_listener.getsockname()[1]
    sender = CoordSender("127.0.0.1", port)

    for i in range(5):
        sender.send(frame_idx=i, w=848, h=480, dets=[])

    seen = []
    for _ in range(5):
        data, _ = udp_listener.recvfrom(4096)
        seen.append(json.loads(data.decode())["frame"])
    assert seen == [0, 1, 2, 3, 4]


def test_send_swallows_transient_socket_error():
    """일시적 네트워크 오류로 예외가 새면 검출 루프 전체가 죽는다 — 흡수돼야 함."""

    class BoomSocket:
        def sendto(self, *_args, **_kwargs):
            raise OSError("network unreachable")

    sender = CoordSender("127.0.0.1", 5001)
    sender._sock = BoomSocket()  # 실소켓은 C 타입이라 메서드 monkeypatch 불가 — 통째로 교체

    sender.send(frame_idx=0, w=848, h=480, dets=[])  # 예외 없이 반환돼야 함


def test_payload_has_stable_nonempty_session_id_without_network(monkeypatch):
    payloads = []

    class CaptureSocket:
        def sendto(self, data, _address):
            payloads.append(json.loads(data.decode()))

    capture = CaptureSocket()
    monkeypatch.setattr(socket, "socket", lambda *_args, **_kwargs: capture)
    sender = CoordSender("127.0.0.1", 5001)

    sender.send(frame_idx=1, w=1280, h=720, dets=[])
    sender.send(frame_idx=2, w=1280, h=720, dets=[])

    assert payloads[0]["session_id"] == payloads[1]["session_id"]
    assert isinstance(payloads[0]["session_id"], str)
    assert payloads[0]["session_id"]
