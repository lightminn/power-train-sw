from pathlib import Path
import stat

import pytest

from chassis.mission_id_store import INT32_MAX, MissionIdStore


STORE_MODULE = Path(__file__).resolve().parents[1] / "mission_id_store.py"


def test_mission_id_store_module_exists():
    assert STORE_MODULE.is_file()


def test_missing_store_allocates_one_and_persists_before_return(tmp_path):
    path = tmp_path / "mission_id"

    result = MissionIdStore(path).allocate()

    assert result.accepted is True
    assert result.mission_id == 1
    assert result.hold_reason == ""
    assert path.read_text(encoding="ascii") == "1\n"


def test_each_allocation_advances_the_persisted_positive_int32(tmp_path):
    path = tmp_path / "mission_id"
    store = MissionIdStore(path)

    assert store.allocate().mission_id == 1
    assert store.allocate().mission_id == 2
    assert MissionIdStore(path).allocate().mission_id == 3
    assert path.read_text(encoding="ascii") == "3\n"


def test_atomic_commit_order_is_file_fsync_replace_directory_fsync(
    tmp_path,
    monkeypatch,
):
    import chassis.mission_id_store as module

    path = tmp_path / "mission_id"
    path.write_text("40\n", encoding="ascii")
    events = []
    real_fsync = module.os.fsync
    real_replace = module.os.replace

    def recording_fsync(fd):
        kind = "dir_fsync" if stat.S_ISDIR(module.os.fstat(fd).st_mode) else "file_fsync"
        events.append(kind)
        return real_fsync(fd)

    def recording_replace(source, destination):
        events.append("replace")
        return real_replace(source, destination)

    monkeypatch.setattr(module.os, "fsync", recording_fsync)
    monkeypatch.setattr(module.os, "replace", recording_replace)

    result = MissionIdStore(path).allocate()

    assert result.accepted is True
    assert result.mission_id == 41
    assert events == ["file_fsync", "replace", "dir_fsync"]


@pytest.mark.parametrize("stored", ("", "garbage\n", "0\n", "-1\n", "1.5\n"))
def test_corrupt_or_nonpositive_store_refuses_new_work(tmp_path, stored):
    path = tmp_path / "mission_id"
    path.write_text(stored, encoding="ascii")

    result = MissionIdStore(path).allocate()

    assert result.accepted is False
    assert result.mission_id is None
    assert result.hold_reason.startswith("mission_id_store:")
    assert path.read_text(encoding="ascii") == stored


def test_int32_max_refuses_new_work_without_wrapping(tmp_path):
    path = tmp_path / "mission_id"
    path.write_text(f"{INT32_MAX}\n", encoding="ascii")

    result = MissionIdStore(path).allocate()

    assert result.accepted is False
    assert result.mission_id is None
    assert result.hold_reason == "mission_id_store:int32_exhausted"
    assert path.read_text(encoding="ascii") == f"{INT32_MAX}\n"


def test_io_failure_returns_hold_signal_and_preserves_previous_id(
    tmp_path,
    monkeypatch,
):
    import chassis.mission_id_store as module

    path = tmp_path / "mission_id"
    path.write_text("9\n", encoding="ascii")

    def fail_replace(_source, _destination):
        raise OSError("injected replace failure")

    monkeypatch.setattr(module.os, "replace", fail_replace)

    result = MissionIdStore(path).allocate()

    assert result.accepted is False
    assert result.mission_id is None
    assert result.hold_reason.startswith("mission_id_store:io_error:")
    assert path.read_text(encoding="ascii") == "9\n"
    assert list(tmp_path.iterdir()) == [path]
