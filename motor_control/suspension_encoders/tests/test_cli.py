import json

import pytest

from suspension_encoders import __main__ as cli
from suspension_encoders import reader as reader_module
from suspension_encoders.config import DEFAULT_CONFIG_PATH, load_config
from suspension_encoders.reader import SuspensionEncoderReader
from suspension_encoders.tests.fake_bus import FakeAs5048bBus


@pytest.mark.parametrize("counts, mean, spread", [
    ([16383, 0, 1], 0, 1),
    ([16380, 16382, 2], 16383, 3),
    ([500, 502, 504], 502, 2),
])
def test_circular_mean_handles_wrap(counts, mean, spread):
    assert cli.circular_mean_counts(counts) == (mean, spread)


def _reader(bus):
    return SuspensionEncoderReader(load_config(DEFAULT_CONFIG_PATH), bus=bus)


def test_capture_zero_averages_each_channel():
    bus = FakeAs5048bBus({0x40: 16383, 0x41: 200, 0x42: 300, 0x43: 400})
    zeros, problems = cli.capture_zero(
        _reader(bus), joints={"ml_rocker_joint", "mr_rocker_joint", "bl_bogie_joint", "br_bogie_joint"},
        samples=5, period_s=0.0, max_spread_counts=23, sleep=lambda s: None,
    )
    assert problems == []
    assert zeros == {"ml_rocker_joint": 16383, "mr_rocker_joint": 200,
                     "bl_bogie_joint": 300, "br_bogie_joint": 400}


def test_capture_zero_reports_missing_and_moving_channels():
    bus = FakeAs5048bBus({0x40: 100, 0x41: 200, 0x42: 300})
    angles = iter(range(0, 1000, 100))

    def sleep(_):
        bus.set_angle(0x42, next(angles))

    zeros, problems = cli.capture_zero(
        _reader(bus), joints={"ml_rocker_joint", "bl_bogie_joint", "br_bogie_joint"},
        samples=4, period_s=0.0, max_spread_counts=23, sleep=sleep,
    )
    assert zeros == {"ml_rocker_joint": 100}
    assert any(p.startswith("br_bogie_joint: sample 0") for p in problems)
    assert any(p.startswith("bl_bogie_joint: spread") for p in problems)


@pytest.fixture
def config_copy(tmp_path, monkeypatch):
    path = tmp_path / "suspension_encoders.json"
    path.write_text(DEFAULT_CONFIG_PATH.read_text(encoding="utf-8"), encoding="utf-8")
    bus = FakeAs5048bBus({0x40: 10, 0x41: 20, 0x42: 30, 0x43: 40})
    monkeypatch.setattr(reader_module, "LinuxI2cBus", lambda number: bus)
    monkeypatch.setattr(cli.time, "sleep", lambda s: None)
    return path, bus


def test_capture_zero_dry_run_leaves_file_untouched(config_copy, capsys):
    path, _ = config_copy
    before = path.read_text(encoding="utf-8")
    assert cli.main(["--config", str(path), "capture-zero", "--samples", "3"]) == 0
    assert path.read_text(encoding="utf-8") == before
    assert "dry run" in capsys.readouterr().out


def test_capture_zero_write_updates_only_zero_counts(config_copy):
    path, bus = config_copy
    original = json.loads(path.read_text(encoding="utf-8"))
    assert cli.main(["--config", str(path), "capture-zero", "--samples", "3", "--write"]) == 0
    written = json.loads(path.read_text(encoding="utf-8"))
    assert [c["zero_counts"] for c in written["channels"]] == [10, 20, 30, 40]
    for before, after in zip(original["channels"], written["channels"]):
        assert {k: v for k, v in before.items() if k != "zero_counts"} == \
            {k: v for k, v in after.items() if k != "zero_counts"}
    assert written["note"] == original["note"]
    assert not any(read[1] != 0xFA for read in bus.reads)  # readout registers only


def test_capture_zero_refuses_when_a_sensor_is_missing(config_copy, capsys):
    path, bus = config_copy
    bus.remove(0x43)
    before = path.read_text(encoding="utf-8")
    assert cli.main(["--config", str(path), "capture-zero", "--samples", "2", "--write"]) == 1
    assert path.read_text(encoding="utf-8") == before
    assert "refused" in capsys.readouterr().err


def test_capture_zero_subset_writes_only_named_joint(config_copy):
    path, bus = config_copy
    bus.remove(0x43)
    assert cli.main(["--config", str(path), "capture-zero", "--samples", "2",
                     "--joint", "ml_rocker_joint", "--write"]) == 0
    written = json.loads(path.read_text(encoding="utf-8"))
    assert [c["zero_counts"] for c in written["channels"]] == [10, None, None, None]


def test_read_command_prints_errors_and_angles(config_copy, capsys):
    path, bus = config_copy
    bus.remove(0x42)
    assert cli.main(["--config", str(path), "read", "--count", "1", "--hz", "50"]) == 0
    out = capsys.readouterr().out
    assert "ml_rocker_joint  0x40 raw=   10" in out
    assert "bl_bogie_joint   0x42 ERROR OSError" in out


def test_scan_reports_otp_zero_register(config_copy, monkeypatch, capsys):
    path, bus = config_copy
    from suspension_encoders import __main__ as module
    monkeypatch.setattr(module, "LinuxI2cBus", lambda number: bus)
    bus.set_angle(0x41, 20, zero_reg=77)
    bus.remove(0x43)
    assert cli.main(["--config", str(path), "scan"]) == 0
    out = capsys.readouterr().out
    assert "zero_reg=77  WARN" in out
    assert "found 3 AS5048B device(s)" in out
    assert "0x43 br_bogie_joint   no AS5048B" in out


def test_capture_zero_with_reference_angle_makes_that_pose_read_the_reference():
    from suspension_encoders.reader import joint_angle_rad
    import math
    bus = FakeAs5048bBus({0x40: 1000, 0x41: 1000, 0x42: 1000, 0x43: 1000})
    reader = _reader(bus)
    # (with_zero_counts clears verified; angles are checked directly below)
    zeros, problems = cli.capture_zero(
        reader, joints={"ml_rocker_joint", "mr_rocker_joint"}, samples=2, period_s=0.0,
        max_spread_counts=23, reference_deg={"ml_rocker_joint": 10.0, "mr_rocker_joint": 10.0},
        sleep=lambda s: None,
    )
    assert problems == []
    for channel in reader.config.with_zero_counts(zeros).channels[:2]:
        # ml sign +1, mr sign -1: both must read +10 deg at the captured pose.
        assert math.degrees(joint_angle_rad(1000, channel)) == pytest.approx(10.0, abs=0.022)


@pytest.mark.parametrize("bad", ["ml_rocker_joint", "nope=1", "ml_rocker_joint=200", "ml_rocker_joint=nan"])
def test_capture_zero_rejects_bad_reference(config_copy, bad):
    path, _ = config_copy
    assert cli.main(["--config", str(path), "capture-zero", "--reference", bad]) == 2


def test_recapture_clears_verified(config_copy):
    path, _ = config_copy
    data = json.loads(path.read_text(encoding="utf-8"))
    for channel in data["channels"]:
        channel.update(zero_counts=1000, verified=True)
    path.write_text(json.dumps(data), encoding="utf-8")
    assert cli.main(["--config", str(path), "capture-zero", "--samples", "2",
                     "--joint", "mr_rocker_joint", "--write"]) == 0
    written = json.loads(path.read_text(encoding="utf-8"))["channels"]
    assert [(c["zero_counts"], c["verified"]) for c in written] == [
        (1000, True), (20, False), (1000, True), (1000, True)]


def test_capture_zero_refuses_bus_override(config_copy, capsys):
    path, _ = config_copy
    before = path.read_text(encoding="utf-8")
    assert cli.main(["--config", str(path), "--bus", "8", "capture-zero", "--write"]) == 2
    assert path.read_text(encoding="utf-8") == before


def test_capture_zero_refuses_when_config_changed_during_capture(config_copy, monkeypatch):
    path, _ = config_copy
    original_capture = cli.capture_zero

    def capture_then_edit(*args, **kwargs):
        result = original_capture(*args, **kwargs)
        data = json.loads(path.read_text(encoding="utf-8"))
        data["channels"][0]["address"], data["channels"][1]["address"] = "0x41", "0x40"
        path.write_text(json.dumps(data), encoding="utf-8")
        return result

    monkeypatch.setattr(cli, "capture_zero", capture_then_edit)
    assert cli.main(["--config", str(path), "capture-zero", "--samples", "2", "--write"]) == 1
    assert [c["zero_counts"] for c in json.loads(path.read_text())["channels"]] == [None] * 4


def test_scan_has_no_probe_all_option():
    with pytest.raises(SystemExit):
        cli.build_parser().parse_args(["scan", "--all"])
