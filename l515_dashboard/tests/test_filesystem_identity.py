import os

import pytest

from l515_dashboard.filesystem_identity import (
    PathOwnershipConflict,
    path_identity,
    quarantine_remove,
)


def test_quarantine_removal_preserves_canonical_successor(tmp_path, monkeypatch):
    path = tmp_path / "gateway.sock"
    path.write_text("owned")
    expected = path_identity(path)
    real_rename = os.rename

    def replace_during_quarantine(source, destination):
        real_rename(source, destination)
        if str(source) == str(path):
            path.write_text("successor")

    monkeypatch.setattr(os, "rename", replace_during_quarantine)
    assert quarantine_remove(path, expected)
    assert path.read_text() == "successor"
    assert list(tmp_path.glob(".gateway.sock.quarantine-*")) == []


def test_identity_mismatch_is_restored_without_deletion(tmp_path):
    path = tmp_path / "gateway.sock"
    path.write_text("unknown")
    actual = path_identity(path)
    expected = (actual[0], actual[1], actual[2] + 1, actual[3])
    assert not quarantine_remove(path, expected)
    assert path.read_text() == "unknown"
    assert list(tmp_path.glob(".gateway.sock.quarantine-*")) == []


def test_unknown_quarantine_is_preserved_when_canonical_recreated(
        tmp_path, monkeypatch):
    path = tmp_path / "gateway.sock"
    path.write_text("owned")
    expected = path_identity(path)
    real_rename = os.rename

    def recreate_canonical(source, destination):
        if str(source) == str(path):
            path.unlink()
            path.write_text("unknown")
        real_rename(source, destination)
        if str(source) == str(path):
            path.write_text("successor")

    monkeypatch.setattr(os, "rename", recreate_canonical)
    with pytest.raises(PathOwnershipConflict) as caught:
        quarantine_remove(path, expected)
    assert path.read_text() == "successor"
    quarantine = caught.value.quarantine_path
    assert quarantine.read_text() == "unknown"
