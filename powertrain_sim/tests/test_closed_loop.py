from __future__ import annotations

import json
import math
from pathlib import Path
import re
import subprocess
import sys

import pytest


pytest.importorskip("mujoco")

from powertrain_sim.closed_loop import TerrainAutonomyDriver, run_closed_loop
from powertrain_sim.mujoco_fast.runner import run_scenario
from powertrain_sim.procedural import GenerationParameters, generate_scenario
from powertrain_sim.scenario import parse_scenario


DEV_SEED = 0


def _flat_document(*, seed: int = DEV_SEED):
    return generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            # 폐루프는 종단 낙하 앞 fail-closed 정지가 정답 — 95% 완주 불가.
            expected_completion=False,
        ),
        seed=seed,
        seed_class="dev",
    )


def _deterministic_metrics(report):
    values = report.to_dict()
    values.pop("wall_clock_runtime_s")
    values.pop("max_estimator_runtime_ms")
    return values


def test_dev_seed_closed_loop_moves_without_fail_open_and_is_deterministic(tmp_path):
    scenario = parse_scenario(_flat_document())

    first = run_closed_loop(scenario, tmp_path / "first")
    second = run_closed_loop(scenario, tmp_path / "second")

    # Measured WP6-S P1 regression anchor (0.805 on the 2.5 m dev seed: the
    # fail-closed stop leaves ~0.55 m ≈ the front-corner radius before the
    # terminal drop).  Change only after reviewing a dev seed run.
    assert first.completion_ratio > 0.75
    assert first.fail_open_count == 0
    assert _deterministic_metrics(first) == _deterministic_metrics(second)
    assert first.passed, first.reasons


def test_initial_depth_loss_holds_then_recovers_without_fail_open(tmp_path):
    document = _flat_document(seed=1)
    dropout_end_s = 0.70
    document["faults"]["sensor_dropouts"] = [
        {
            "stream": "depth",
            "start_s": 0.0,
            "end_s": dropout_end_s,
        }
    ]
    scenario = parse_scenario(document)
    driver = TerrainAutonomyDriver(scenario)
    observations: list[tuple[float, bool, bool]] = []

    def observe_hold(elapsed_s, snapshot):
        actual_hold, should_hold = driver.hold_state(elapsed_s, snapshot)
        observations.append((elapsed_s, actual_hold, should_hold))
        return actual_hold, should_hold

    report = run_scenario(
        scenario,
        tmp_path / "dropout",
        command_source=driver.command,
        hold_state_source=observe_hold,
        depth_tap=driver.on_depth,
    )

    during_dropout = [
        actual_hold
        for elapsed_s, actual_hold, _ in observations
        if elapsed_s < dropout_end_s
    ]
    assert during_dropout and all(during_dropout)
    assert any(
        not actual_hold
        for elapsed_s, actual_hold, _ in observations
        if elapsed_s > dropout_end_s
    )
    assert report.fail_open_count == 0
    assert math.isfinite(report.max_recovery_time_s)
    # should_hold와 decision이 같은 terrain 참조를 쓰므로(위상 아티팩트 제거)
    # 입력 회복 tick에 컨트롤러도 같은 tick에 TRACKING으로 복귀한다 — 지연 0.
    assert report.max_recovery_time_s == 0.0


def test_hidden_evaluation_cli_records_hash_and_matches_report_exit(tmp_path):
    run_directory = tmp_path / "hidden"
    completed = subprocess.run(
        [
            sys.executable,
            "-m",
            "powertrain_sim.hidden_eval",
            "17",
            str(run_directory),
            "--seed-class",
            "hidden_evaluation",
        ],
        check=False,
        capture_output=True,
        text=True,
    )

    metrics = json.loads(
        (run_directory / "metrics.json").read_text(encoding="utf-8")
    )
    scenario_text = (run_directory / "scenario.yaml").read_text(encoding="utf-8")
    assert completed.returncode == (0 if metrics["passed"] else 1)
    assert completed.stdout.strip().startswith("MetricsReport[")
    assert re.search(r"^# canonical_json_sha256: [0-9a-f]{64}$", scenario_text, re.MULTILINE)
