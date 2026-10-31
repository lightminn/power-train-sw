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
    assert support["support_base_height_m"] == pytest.approx(0.471, abs=0.01)
    assert support["can_enable"] is None                         # one recording, no flat reference: measurement only


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
    assert "fusion lock hold max" in text and "decisions write max" in text   # r14 lock timing is judged


def test_a_slow_decisions_write_fails_the_lock_gate(drive_bag, tmp_path, capsys):
    """r14 Codex rounds 2–4: a log write that held the map lock must not pass."""
    bag, received, root = drive_bag
    records = []
    live(received, records.append)
    result = next(r for r in records if r.get("k") == "result" and "rec_ms_max" in r)
    result["rec_ms_max"] = 120.0
    write_log(root / "slow.jsonl", records)
    assert FR.main(["--decisions", str(root / "slow.jsonl"), "--out", str(tmp_path / "r.json")]) == 0
    line = next(l for l in capsys.readouterr().out.splitlines() if "fusion lock hold max" in l)
    assert "decisions write max 120.0 ms" in line and line.endswith("FAIL)")


def test_a_log_without_its_closing_line_is_not_judged(drive_bag, tmp_path, capsys):
    """r14 Codex round 5: missing timing must not read as 0 ms and pass."""
    bag, received, root = drive_bag
    records = []
    live(received, records.append)
    write_log(root / "cut.jsonl", [r for r in records if r.get("k") != "timing_end"])
    assert FR.main(["--decisions", str(root / "cut.jsonl"), "--out", str(tmp_path / "r.json")]) == 0
    out = capsys.readouterr().out
    assert "fusion lock: not judged" in out and "fusion lock hold max" not in out


def test_late_arrivals_do_not_look_like_publisher_gaps(tmp_path):
    """A late /wheel_states message is a receive gap and a reorder, not a chassis tick gap."""
    events, _ = F.scenario(F.drive_then(Command("skid", v=0.3)), 3.0, surface=None)
    received = [(e.stamp_s + (0.12 if e.topic == F.WHEEL_TOPIC and abs(e.stamp_s - F.START - 2.0) < 1e-9 else 0.0),
                 e.topic, e.message) for e in events]
    write_bag(tmp_path / "bag", sorted(received, key=lambda r: r[0]))
    wheel = FR.analyze_bag(str(tmp_path / "bag"), CFG)["streams"]["wheel"]
    assert wheel["header"]["p99_s"] == pytest.approx(0.02, abs=1e-6) and wheel["header"]["over_s"]["0.06"] == 0
    assert wheel["reordered"] == 1


# --- Codex field-tool review -------------------------------------------------------------
def ditch_bag(path):
    from terrain_map.tests.world_render import BasePose, Mount, camera_in_world, flat, with_ditch
    fx = float(camera_in_world(BasePose(), Mount())[0][0])
    surface = with_ditch(flat(), fx + 0.8, fx + 3.2, -1.2, 1.2, 0.20)     # the plane fits the ditch bottom
    events, _ = F.scenario(lambda t: Command("stop"), 2.5, surface=surface)
    write_bag(path, [(e.stamp_s, e.topic, e.message) for e in events])


def test_rejected_support_planes_never_feed_the_height_suggestion(tmp_path):
    ditch_bag(tmp_path / "ditch")
    report = FR.analyze_bag(str(tmp_path / "ditch"), CFG)
    assert report["depth"]["near_band"] > 0 and report["depth"]["support_ok"] == 0
    assert report["support"]["n"] == 0 and not report["support"]["can_enable"]
    assert report["rejected_base_heights"]["n"] == report["depth"]["near_band"]


def test_header_intervals_respect_clock_resets(tmp_path):
    first, _ = F.scenario(lambda t: Command("stop"), 2.0, surface=None)
    again, _ = F.scenario(lambda t: Command("stop"), 1.0, surface=None)   # the clock restarts 2 s back
    items = [(e.stamp_s, e.topic, e.message) for e in first] + [(e.stamp_s + 10.0, e.topic, e.message) for e in again]
    write_bag(tmp_path / "reset", items)
    gyro = FR.analyze_bag(str(tmp_path / "reset"), CFG)["streams"]["gyro"]["header"]
    assert gyro["clock_resets"] == 1 and gyro["max_s"] == pytest.approx(0.01, abs=1e-6)


def test_poses_from_several_bags_are_pooled_against_a_fixed_reference(drive_bag, tmp_path, capsys):
    bag, _, _ = drive_bag
    assert FR.main(["--bag", str(bag), "--window", "0:1.4", "--bag", str(bag), "--window", "1.6:3.0",
                    "--reference-height", "0.471", "--out", str(tmp_path / "p.json")]) == 0
    pooled = json.loads((tmp_path / "p.json").read_text())["support_pooled"]
    assert pooled["reference"] == "given" and pooled["n"] > 0 and pooled["support_base_height_m"] == 0.471
    assert "pooled over 2 recordings" in capsys.readouterr().out
