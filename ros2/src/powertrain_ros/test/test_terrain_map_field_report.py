"""terrain_map_field_report — field-check numbers from a bag and the node's decisions log (Task 12 prep)."""
import json

import pytest

from powertrain_ros import terrain_map_field_report as FR
from terrain_map.tests.odometry_sim import Command

import terrain_map_fixture as F
from test_terrain_map_replay import CFG, live, receipt, write_bag, write_log


@pytest.fixture(scope="module")
def drive_bag(tmp_path_factory):
    events, _ = F.scenario(F.drive_then(Command("skid", v=0.3)), 4.0)
    gap = [e for e in events if not (e.topic == F.GYRO_TOPIC and 2.505 < e.stamp_s - F.START < 2.595)]
    received = [(e.stamp_s + (0.02 if e.topic in (F.DEPTH_TOPIC, F.INFO_TOPIC) else 0.0), e.topic, e.message)
                for e in gap]
    root = tmp_path_factory.mktemp("field")
    write_bag(root / "bag", sorted(received, key=lambda r: r[0]))
    return root / "bag", received, root


def test_bag_report_streams_intervals_support_and_freshness(drive_bag):
    bag, _, _ = drive_bag
    report = FR.analyze_bag(str(bag), CFG)
    gyro, wheel = report["streams"]["gyro"]["header"], report["streams"]["wheel"]["header"]
    assert gyro["over_s"]["0.06"] == 1 and gyro["p50_s"] == pytest.approx(0.01, abs=1e-6)
    assert wheel["p99_s"] == pytest.approx(0.02, abs=1e-6)
    depth = report["depth"]
    assert depth["frames"] == depth["ok"] > 15 and depth["fresh_ratio"] > 0.9
    support = report["support"]
    assert support["support_base_height_m"] == pytest.approx(0.471, abs=0.01) and support["can_enable"]


def test_window_limits_the_analysis(drive_bag):
    bag, _, _ = drive_bag
    full = FR.analyze_bag(str(bag), CFG)
    still = FR.analyze_bag(str(bag), CFG, window=(1.6, 2.4))            # stamps relative to the bag's first one
    assert 0 < still["depth"]["frames"] < full["depth"]["frames"]


def test_cli_combines_bag_and_decisions_and_judges_the_criteria(drive_bag, tmp_path, capsys):
    bag, received, root = drive_bag
    records = []
    live(received, records.append)
    write_log(root / "d.jsonl", records)
    out = tmp_path / "report.json"
    assert FR.main(["--bag", str(bag), "--decisions", str(root / "d.jsonl"), "--param", "pixel_stride=1",
                    "--out", str(out)]) == 0
    report = json.loads(out.read_text())
    assert report["decisions"]["frames"]["processed"] > 0 and "loss_rate" in report["decisions"]["frames"]
    text = capsys.readouterr().out
    assert "loss rate" in text and "proc_ms p95" in text and "support_base_height_m" in text


def test_late_arrivals_do_not_look_like_publisher_gaps(tmp_path):
    """A late /wheel_states message is a receive gap and a reorder, not a chassis tick gap."""
    events, _ = F.scenario(F.drive_then(Command("skid", v=0.3)), 3.0, surface=None)
    received = [(e.stamp_s + (0.12 if e.topic == F.WHEEL_TOPIC and abs(e.stamp_s - F.START - 2.0) < 1e-9 else 0.0),
                 e.topic, e.message) for e in events]
    write_bag(tmp_path / "bag", sorted(received, key=lambda r: r[0]))
    wheel = FR.analyze_bag(str(tmp_path / "bag"), CFG)["streams"]["wheel"]
    assert wheel["header"]["p99_s"] == pytest.approx(0.02, abs=1e-6) and wheel["header"]["over_s"]["0.06"] == 0
    assert wheel["reordered"] == 1
