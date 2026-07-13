"""class_ids — 클래스 이름 CSV 필터를 ultralytics class id 리스트로 변환."""
import pytest

from yolo_depth_3d import class_ids


class FakeModel:
    names = {0: "person", 1: "bottle", 2: "car"}


def test_known_class_names_map_to_ids():
    assert class_ids(FakeModel(), "person,car") == [0, 2]


def test_whitespace_around_names_is_stripped():
    assert class_ids(FakeModel(), " person , bottle ") == [0, 1]


def test_empty_string_means_no_filter():
    assert class_ids(FakeModel(), "") is None
    assert class_ids(FakeModel(), "   ") is None


def test_unknown_class_name_exits_with_message():
    with pytest.raises(SystemExit) as exc_info:
        class_ids(FakeModel(), "dog")
    assert "dog" in str(exc_info.value)
