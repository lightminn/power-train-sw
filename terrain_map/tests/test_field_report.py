"""Field report helpers (Task 12 preparation): decisions-log summary, intervals, support suggestion."""
import json
import math

import numpy as np
import pytest

from terrain_map import field_report as R
from terrain_map.config import MapConfig
from terrain_map.tests import test_runtime as T


def test_interval_stats_count_gaps_and_percentiles():
    stamps = [0.0, 0.01, 0.02, 0.03, 0.10, 0.11, 0.50]          # one 70 ms gap, one 390 ms gap
    s = R.interval_stats(stamps, gaps_s=(0.06, 0.30))
    assert s["count"] == 6 and s["max_s"] == pytest.approx(0.39)
    assert s["over_s"] == {"0.06": 2, "0.3": 1} and s["p50_s"] == pytest.approx(0.01)
    assert R.interval_stats([1.0])["count"] == 0


def test_support_suggestion_checks_the_spec_relation():
    rng = np.random.default_rng(0)
    steady = 0.471 + rng.uniform(-0.01, 0.01, 300)
    s = R.support_suggestion(steady, MapConfig())
    assert s["support_base_height_m"] == pytest.approx(0.471, abs=0.002)
    assert s["support_base_variation_m"] == pytest.approx(0.01, abs=0.002) and s["can_enable"]
    wild = 0.471 + rng.uniform(-0.05, 0.05, 300)                 # var 5 cm: 8 cm thresholds cannot hold it
    assert not R.support_suggestion(wild, MapConfig())["can_enable"]
    assert R.support_suggestion([], MapConfig())["can_enable"] is False


def live_records():
    events, depth = T.scenario()
    records = []
    clock = T.Clock()
    rt = T.runtime(clock, decisions=records.append)
    rt.on_camera_info(T.CAM)
    T.drive(rt, clock, T.live_trouble(events), depth)
    return rt, [json.loads(json.dumps(r)) for r in records]


def test_decisions_summary_matches_the_runtime_counters():
    rt, records = live_records()
    s = R.summarize_decisions(records)
    assert s["frames"]["processed"] == rt.scheduler.processed and s["frames"]["lost"] == rt.scheduler.lost
    total = rt.scheduler.processed + rt.scheduler.lost
    assert s["frames"]["loss_rate"] == pytest.approx(rt.scheduler.lost / total)
    assert s["epochs"]["count"] == rt.odometry.epoch_changes and "stall" in s["epochs"]["by_cause"]
    assert s["samples"]["late"] == rt.odometry.late_dropped
    assert s["proc_ms"]["n"] == sum(s["results"].values()) and s["proc_ms"]["p95"] >= s["proc_ms"]["p50"] >= 0  # fake clock
    assert set(s["results"]) <= {"processed", "GROUND_NOT_FOUND", "NOT_SUPPORT", "PROCESSING_ERROR", "epoch_changed"}


def test_decisions_summary_needs_a_session_choice_when_runs_are_mixed():
    _, first = live_records()
    _, second = live_records()
    second[0] = dict(second[0], instance="c" * 16)
    with pytest.raises(ValueError):
        R.summarize_decisions(first + second)
    assert R.summarize_decisions(first + second, session="c" * 16)["frames"]["processed"] > 0


def test_results_carry_processing_time():
    _, records = live_records()
    results = [r for r in records if r.get("k") == "result"]
    assert results and all(isinstance(r["ms"], float) and math.isfinite(r["ms"]) for r in results)


# --- Codex field-tool review -------------------------------------------------------------
def test_variation_covers_every_pose_not_just_the_middle_ninety_percent():
    heights = [0.471] * 96 + [0.411] * 4                     # a few low poses are real variation
    s = R.support_suggestion(heights, MapConfig())
    assert s["support_base_variation_m"] >= 0.06 and not s["can_enable"]


def test_a_fixed_reference_pools_poses_from_separate_recordings():
    poses = [[0.411] * 50, [0.471] * 50, [0.531] * 50]       # each pose steady on its own
    assert all(R.support_suggestion(p, MapConfig())["support_base_variation_m"] < 0.002 for p in poses)
    pooled = R.support_suggestion([h for p in poses for h in p], MapConfig(), reference_m=0.471)
    assert pooled["support_base_height_m"] == 0.471 and pooled["support_base_variation_m"] >= 0.06
    assert not pooled["can_enable"] and pooled["reference"] == "given"


def test_interval_stats_split_at_clock_resets():
    s = R.interval_stats([100.0, 100.02, 10.0, 10.02], regression_s=1.0)
    assert s["clock_resets"] == 1 and s["max_s"] == pytest.approx(0.02) and s["count"] == 2


def test_processing_errors_and_missing_results_are_losses():
    rt, records = live_records()
    for r in records:
        if r.get("k") == "result":
            r["r"] = "PROCESSING_ERROR"
    s = R.summarize_decisions(records)
    assert s["frames"]["loss_rate"] == pytest.approx(1.0) and s["frames"]["errors"] == rt.scheduler.processed
    _, records = live_records()
    results = [k for k, r in enumerate(records) if r.get("k") == "result"]
    del records[results[0]]                                   # a processed frame without its result
    s = R.summarize_decisions(records)
    assert s["frames"]["incomplete"] == 1 and s["frames"]["loss_rate"] > 0


def test_processing_time_covers_correction_and_fusion():
    events, depth = T.scenario()
    records = []
    clock = T.Clock()
    rt = T.runtime(clock, decisions=records.append)
    rt.on_camera_info(T.CAM)
    fuse = rt.map.fuse

    def slow_fuse(*args, **kwargs):
        clock.now += 0.2                                      # fusion takes 200 ms
        return fuse(*args, **kwargs)
    rt.map.fuse = slow_fuse
    T.drive(rt, clock, events, depth)
    fused = [r for r in records if r.get("k") == "result" and r["r"] == "processed"]
    assert fused and all(r["ms"] >= 200.0 for r in fused)
