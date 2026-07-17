"""클라이언트 미러 상수 == 서버 계약 (기존 SCHEMA_VERSION 미러 패턴).

laptop/ops_channel_client.py 는 powertrain_ros 를 import 하지 않는 미러
구현이다 — 값이 어긋나면 재전송/비상 hold 의미가 양단에서 갈라진다.
"""
import importlib.util
from pathlib import Path

from powertrain_ros import ops_contract as server

_CLIENT_PATH = (
    Path(__file__).resolve().parents[4]
    / "motor_control/laptop/ops_channel_client.py"
)
_spec = importlib.util.spec_from_file_location(
    "ops_channel_client_mirror", _CLIENT_PATH
)
client = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(client)


def test_client_mirror_matches_server_contract():
    assert client.OPS_SCHEMA_VERSION == server.SCHEMA_VERSION
    assert client.OPS_DEFAULT_PORT == server.DEFAULT_PORT
    assert client.RETRANSMIT_INTERVAL_S == server.RETRANSMIT_INTERVAL_S
    assert client.REQUEST_DEADLINE_S == server.REQUEST_DEADLINE_S
    assert client.EMERGENCY_HOLD_S == server.EMERGENCY_HOLD_S
