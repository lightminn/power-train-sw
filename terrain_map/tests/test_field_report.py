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
    s = R.support_suggestion(steady, MapConfig(), reference_m=0.471)
    assert s["support_base_height_m"] == 0.471
    assert s["support_base_variation_m"] == pytest.approx(0.01, abs=0.002) and s["can_enable"] is True
    wild = 0.471 + rng.uniform(-0.05, 0.05, 300)                 # var 5 cm: 8 cm thresholds cannot hold it
    assert R.support_suggestion(wild, MapConfig(), reference_m=0.471)["can_enable"] is False
    assert R.support_suggestion([], MapConfig(), reference_m=0.471)["can_enable"] is False


def test_without_a_flat_reference_there_is_no_approval():
    """Re-review: re-centring on rough-ground samples hides their offset from the flat height."""
    s = R.support_suggestion([0.411] * 50 + [0.431] * 50, MapConfig())
    assert s["can_enable"] is None and "reference" in s["approval"]


def live_records():
    events, depth = T.scenario()
    records = []
    clock = T.Clock()
    rt = T.runtime(clock, decisions=records.append)
    rt.on_camera_info(T.CAM)
    T.drive(rt, clock, T.live_trouble(events), depth)
    rt.close_log()
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
    assert results and all(isinstance(r["total_ms"], float) and math.isfinite(r["total_ms"]) for r in results)


# --- Codex field-tool review -------------------------------------------------------------
def test_variation_covers_every_pose_not_just_the_middle_ninety_percent():
    heights = [0.471] * 96 + [0.411] * 4                     # a few low poses are real variation
    s = R.support_suggestion(heights, MapConfig())
    assert s["support_base_variation_m"] >= 0.06 and not s["can_enable"]   # None: no reference → no approval


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
    fuse = rt.map.fuse_cells

    def slow_fuse(*args, **kwargs):
        clock.now += 0.2                                      # fusion takes 200 ms
        return fuse(*args, **kwargs)
    rt.map.fuse_cells = slow_fuse
    T.drive(rt, clock, events, depth)
    fused = [r for r in records if r.get("k") == "result" and r["r"] == "processed"]
    assert fused and all(r["total_ms"] >= 200.0 > r["detect_ms"] for r in fused)
    assert all(r["fuse_ms"] >= 200.0 for r in fused)        # r14: the fusion lock hold carries it
    s = R.summarize_decisions(records)
    assert s["fuse_ms"]["n"] == len(fused) and s["fuse_ms"]["max"] >= 200.0
    assert s["bin_ms"]["n"] == s["lock_wait_ms"]["n"] == len(fused)


def test_a_log_before_r14_has_no_lock_timing():
    _, records = live_records()
    records = [r for r in records if r.get("k") != "timing_end"]
    for r in records:
        if r.get("k") == "result":
            for k in ("bin_ms", "lock_wait_ms", "fuse_ms", "rec_ms_max", "hold_ms_prev"):
                r.pop(k)
    s = R.summarize_decisions(records)
    assert s["fuse_ms"] == s["lock_wait_ms"] == s["bin_ms"] == s["rec_ms_max"] == s["hold_ms"] == {"n": 0}
    assert s["timing_complete"] is False


def test_lock_timing_is_complete_only_with_the_closing_line():
    _, records = live_records()
    assert R.summarize_decisions(records)["timing_complete"] is True
    cut = [r for r in records if r.get("k") != "timing_end"]
    assert R.summarize_decisions(cut)["timing_complete"] is False


def test_an_old_log_with_detection_time_only_is_not_judged():
    """Re-review: before total_ms the log's ms covered detection only."""
    _, records = live_records()
    for r in records:
        if r.get("k") == "result":
            r["ms"] = r.pop("detect_ms")
            r.pop("total_ms")
    s = R.summarize_decisions(records)
    assert s["proc_ms"] == {"n": 0} and "detection" in s["proc_ms_note"]
    _, records = live_records()
    s = R.summarize_decisions(records)
    assert s["proc_ms"]["n"] > 0 and s["proc_ms_note"] is None


def test_a_failed_log_write_leaves_the_lock_timing_unjudged():
    """r14 Codex round 6: a write error, then the shutdown's closing line, must not read as complete."""
    events, depth = T.scenario()
    records, clock = [], T.Clock()
    state = {"results": 0}

    def log(entry):
        if entry.get("k") == "result":
            state["results"] += 1
            if state["results"] == 5:
                clock.now += 0.2
                raise OSError("disk full")
        records.append(entry)
    rt = T.runtime(clock, decisions=log)
    rt.on_camera_info(T.CAM)
    with pytest.raises(OSError):
        T.drive(rt, clock, events, depth)
    rt.close_log()                                           # the node's finally -> destroy_node path
    end = records[-1]
    assert end["k"] == "timing_end" and end["write_failed"] is True and end["hold_ms_prev"] >= 200.0
    s = R.summarize_decisions([json.loads(json.dumps(r)) for r in records])
    assert s["timing_complete"] is False and s["frames"]["incomplete"] == 1
