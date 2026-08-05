"""구동 트랜스포트 모드 파일 — 기동 시 읽는 단일 근거.

ROS-free 순수 모듈이라 호스트 pytest 로 검증한다 (ops_contract.py 와 같은 이유).
"""
from powertrain_ros import transport_mode


def test_reads_usb(tmp_path):
    path = tmp_path / "drive_transport"
    path.write_text("usb\n", encoding="utf-8")

    assert transport_mode.read(path) == "usb"


def test_reads_can(tmp_path):
    path = tmp_path / "drive_transport"
    path.write_text("can", encoding="utf-8")

    assert transport_mode.read(path) == "can"


def test_ignores_surrounding_whitespace_and_case(tmp_path):
    path = tmp_path / "drive_transport"
    path.write_text("  USB  \n", encoding="utf-8")

    assert transport_mode.read(path) == "usb"


def test_missing_file_falls_back_to_can(tmp_path):
    assert transport_mode.read(tmp_path / "absent") == "can"


def test_unrecognised_value_falls_back_to_can(tmp_path):
    """오타 하나로 조향 없는 스택이 뜨면 안 된다 — 보수적 기본값."""
    path = tmp_path / "drive_transport"
    path.write_text("usbb", encoding="utf-8")

    assert transport_mode.read(path) == "can"


def test_empty_file_falls_back_to_can(tmp_path):
    path = tmp_path / "drive_transport"
    path.write_text("", encoding="utf-8")

    assert transport_mode.read(path) == "can"


def test_directory_instead_of_file_falls_back_to_can(tmp_path):
    """읽기 실패는 예외가 아니라 can 으로 떨어져야 한다 (기동을 막지 않는다)."""
    directory = tmp_path / "drive_transport"
    directory.mkdir()

    assert transport_mode.read(directory) == "can"


def test_default_steering_mode_pairs_with_the_transport():
    """usb 는 skid 만 가능하다 — launch 기본값이 이 함수 하나에서 나온다."""
    assert transport_mode.default_steering_mode("usb") == "skid"
    assert transport_mode.default_steering_mode("can") == "ackermann"
    assert transport_mode.default_steering_mode("garbage") == "ackermann"
