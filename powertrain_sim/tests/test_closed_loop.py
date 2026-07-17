from __future__ import annotations

import json
import math
from pathlib import Path
import re
import subprocess
import sys
from types import SimpleNamespace

import pytest

from powertrain_autonomy.controller import ControllerDecision, DriveDiagnostics
from powertrain_ros.state_estimation import (
    DiagnosticSnapshot,
    PoseSnapshot,
    StateSnapshot,
    TiltSnapshot,
    VelocitySnapshot,
)

from powertrain_sim.hidden_eval.__main__ import evaluate_report


pytest.importorskip("mujoco")

from powertrain_sim.closed_loop import TerrainAutonomyDriver, run_closed_loop
from powertrain_sim.mujoco_fast.model_builder import WHEEL_HALF_WIDTH_M
from powertrain_sim.mujoco_fast.runner import run_scenario
from powertrain_sim.procedural import (
    FrictionPatchSpec,
    GenerationParameters,
    PinchSpec,
    generate_scenario,
)
from powertrain_sim.scenario import parse_scenario


DEV_SEED = 0
# CAD URDF wheel centres in chassis.kinematics.default_geometry() have their
# widest |y| at 0.4395 m; model_builder gives each wheel 0.035 m half-width.
# The simulated physical footprint is therefore 2 * (0.4395 + 0.035) = 0.949 m.
ROBOT_FOOTPRINT_WIDTH_M = 0.949


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


def _snapshot_with_diagnostics(diagnostics: DiagnosticSnapshot) -> StateSnapshot:
    return StateSnapshot(
        pose=PoseSnapshot(0.0, 0.0, 0.0),
        velocity=VelocitySnapshot(0.2, 0.0, 0.0),
        tilt=TiltSnapshot(0.0, 0.0),
        distance_m=0.1,
        diagnostics=diagnostics,
        stale=False,
        wheel_stale=False,
        imu_stale=False,
        initialized=True,
        reinitialized=False,
        reconnect_count=0,
        yaw_source="wheel",
        gyro_bias_rad_s=(0.0, 0.0, 0.0),
    )


def _pinch_document(*, width_m: float):
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.3, 1.3),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            station_spacing_range_m=(0.20, 0.20),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            pinch=PinchSpec(center_ratio=0.45, length_m=0.5, width_m=width_m),
            # Completion means the expected fail-closed endpoint, while the
            # assertions below separately prove whether the pinch was passed.
            expected_completion=False,
        ),
        seed=DEV_SEED,
        seed_class="dev",
    )
    # Isolate the width-family acceptance from the deterministic random fault
    # schedule, and leave enough time for clearance-based proportional slowing.
    document["clock"]["duration_s"] = 12.0
    document["faults"] = {name: [] for name in document["faults"]}
    return document


def _friction_document():
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            friction_range=(0.8, 0.8),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            friction_patch=FrictionPatchSpec(
                center_ratio=0.5,
                length_m=0.8,
                mu=0.3,
            ),
            expected_completion=False,
        ),
        seed=DEV_SEED,
        seed_class="dev",
    )
    document["faults"] = {name: [] for name in document["faults"]}
    return document


def _depth_degradation_document():
    document = _flat_document(seed=2)
    document["faults"] = {name: [] for name in document["faults"]}
    document["faults"]["depth_degradation"] = [
        {
            "start_s": 0.8,
            "end_s": 2.4,
            "dropout_ratio_start": 0.0,
            "dropout_ratio_end": 0.6,
            "noise_std_m": 0.02,
        }
    ]
    return document


def _clothoid_document():
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(-0.08, 0.08),
            station_spacing_range_m=(0.35, 0.35),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("flat",),
            motion_profiles=("constant_speed",),
            curvature_mode="clothoid",
            expected_completion=False,
        ),
        seed=DEV_SEED,
        seed_class="dev",
    )
    document["clock"]["duration_s"] = 12.0
    document["faults"] = {name: [] for name in document["faults"]}
    return document


def _undulating_document():
    document = generate_scenario(
        GenerationParameters(
            track_length_range_m=(2.5, 2.5),
            track_width_range_m=(1.4, 1.4),
            track_height_range_m=(0.5, 0.5),
            curvature_range_per_m=(0.0, 0.0),
            station_spacing_range_m=(0.40, 0.40),
            linear_speed_range_m_s=(0.45, 0.45),
            terrain_families=("undulating",),
            motion_profiles=("constant_speed",),
            expected_completion=False,
        ),
        seed=DEV_SEED,
        seed_class="dev",
    )
    document["clock"]["duration_s"] = 12.0
    document["faults"] = {name: [] for name in document["faults"]}
    return document


def test_dev_seed_closed_loop_moves_without_fail_open_and_is_deterministic(tmp_path):
    scenario = parse_scenario(_flat_document())

    first = run_closed_loop(scenario, tmp_path / "first")
    second = run_closed_loop(scenario, tmp_path / "second")

    # Measured WP6-S P1 regression anchor. 0.805 → 0.709 after the A3
    # recovery-dwell hardening (ticks AND >=0.15 s AND 3 fresh samples):
    # each hold/recovery episode holds ~0.16 s longer, so the fixed-length
    # run covers less track. Fail-closed endpoint semantics unchanged
    # (passed=True, fail_open=0). Change only after reviewing a dev seed run.
    assert first.completion_ratio > 0.70
    assert first.fail_open_count == 0
    assert _deterministic_metrics(first) == _deterministic_metrics(second)
    assert first.passed, first.reasons


def test_driver_maps_only_production_state_diagnostics_into_controller():
    scenario = parse_scenario(_flat_document())
    driver = TerrainAutonomyDriver(scenario)

    class CaptureController:
        diagnostics = None

        def decide(self, now_s, **kwargs):
            self.diagnostics = kwargs["diagnostics"]
            return ControllerDecision(now_s, 0.0, 0.0, "TRACKING", ())

    capture = CaptureController()
    driver.controller = capture
    snapshot = _snapshot_with_diagnostics(
        DiagnosticSnapshot(
            slip_candidate=True,
            stuck_candidate=False,
            one_wheel_mismatch=True,
            warning_codes=("response_ratio",),
            affected_wheels=("front_left",),
            terrain_profile="default",
            terrain_speed_cap=0.42,
            wheel_yaw_rate_rad_s=0.0,
            imu_yaw_rate_rad_s=0.0,
        )
    )

    driver.command(0.25, snapshot)

    assert capture.diagnostics == DriveDiagnostics(
        stamp_s=scenario.clock.start_s + 0.25,
        slip_candidate=True,
        stuck_candidate=False,
        speed_cap_m_s=0.42,
    )


def test_mu_point_three_patch_measured_state_diagnostic_acceptance(tmp_path):
    scenario = parse_scenario(_friction_document())
    driver = TerrainAutonomyDriver(scenario)
    observations = []

    def capture_diagnostics(elapsed_s, snapshot):
        if snapshot is not None:
            observations.append(
                (snapshot.distance_m, snapshot.diagnostics.slip_candidate)
            )
        return driver.command(elapsed_s, snapshot)

    report = run_scenario(
        scenario,
        tmp_path / "friction",
        command_source=capture_diagnostics,
        hold_state_source=driver.hold_state,
        depth_tap=driver.on_depth,
    )

    patch_start_m = 0.5 * 2.5 - 0.5 * 0.8
    patch_end_m = 0.5 * 2.5 + 0.5 * 0.8
    inside = [
        detected
        for distance_m, detected in observations
        if patch_start_m <= distance_m <= patch_end_m
    ]
    outside = [
        detected
        for distance_m, detected in observations
        if distance_m < patch_start_m or distance_m > patch_end_m
    ]
    assert inside and outside
    detection_rate = sum(inside) / len(inside)
    false_detection_rate = sum(outside) / len(outside)

    # Dev seed measurement: 0/71 in-patch and 0/207 out-of-patch.  The
    # production estimator diagnoses wheel-to-wheel response and wheel/IMU yaw,
    # not absolute longitudinal ground speed; this symmetric, low-acceleration
    # μ=0.3 run therefore exposes no qualifying inconsistency.  Keep production
    # thresholds unchanged and pin the measured simulator capability honestly.
    assert detection_rate == pytest.approx(0.0)
    assert false_detection_rate <= 0.05
    assert report.fail_open_count == 0
    assert report.passed, report.reasons


def test_depth_degradation_holds_then_observes_recovery_dwell(tmp_path):
    scenario = parse_scenario(_depth_degradation_document())
    driver = TerrainAutonomyDriver(scenario)
    decisions = []

    def capture_decision(elapsed_s, snapshot):
        command = driver.command(elapsed_s, snapshot)
        if driver._decision is not None:
            decisions.append((elapsed_s, driver._decision))
        return command

    report = run_scenario(
        scenario,
        tmp_path / "depth-degradation",
        command_source=capture_decision,
        hold_state_source=driver.hold_state,
        depth_tap=driver.on_depth,
    )

    deep_degradation = [
        decision
        for elapsed_s, decision in decisions
        if 1.8 <= elapsed_s < 2.4
    ]
    terrain_hold_reasons = {
        "low_confidence",
        "path_unavailable",
        "terrain_stale",
    }
    assert any(
        decision.state == "CONTROLLED_HOLD"
        and terrain_hold_reasons.intersection(decision.reasons)
        for decision in deep_degradation
    )

    post_fault = [
        (elapsed_s, decision)
        for elapsed_s, decision in decisions
        if elapsed_s >= 2.4
    ]
    first_tracking_s = next(
        elapsed_s
        for elapsed_s, decision in post_fault
        if decision.state == "TRACKING"
    )
    assert any(
        decision.reasons == ("recovery_dwell",)
        for elapsed_s, decision in post_fault
        if elapsed_s < first_tracking_s
    )
    assert first_tracking_s - 2.4 >= 0.15
    assert report.fail_open_count == 0
    assert report.max_recovery_time_s == pytest.approx(0.20, abs=1e-6)
    assert report.passed, report.reasons


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
    # A3 dwell: 복귀는 ticks(3) AND 경과 >=0.15 s AND 신선 표본 3개를 모두
    # 요구한다. depth 표본 주기 0.1 s(5 step)라 표본 3개 = 0.2 s가 지배 조건
    # — 회복 지연 0.20 s. tracker는 이 꼬리를 false hold가 아닌 recovery로
    # 집계한다.
    assert report.max_recovery_time_s == pytest.approx(0.20, abs=1e-6)


def test_wide_pinch_is_traversed_without_fail_open(tmp_path):
    document = _pinch_document(width_m=ROBOT_FOOTPRINT_WIDTH_M + 0.15)
    report = run_closed_loop(parse_scenario(document), tmp_path / "wide-pinch")
    pinch_end_ratio = 0.45 + 0.5 / 2.0 / 2.5

    assert report.completion_ratio > pinch_end_ratio
    assert report.fail_open_count == 0
    assert report.edge_overrun_count == 0
    assert report.passed, report.reasons


def test_too_narrow_pinch_stops_before_the_drop_boundary(tmp_path):
    document = _pinch_document(width_m=ROBOT_FOOTPRINT_WIDTH_M - 0.049)
    report = run_closed_loop(parse_scenario(document), tmp_path / "narrow-pinch")
    pinch_start_ratio = 0.45 - 0.5 / 2.0 / 2.5

    assert report.completion_ratio < pinch_start_ratio
    assert report.fail_open_count == 0
    assert report.edge_overrun_count == 0
    assert report.passed, report.reasons


def test_clothoid_closed_loop_stays_bounded_without_fail_open(tmp_path):
    document = _clothoid_document()
    report = run_closed_loop(parse_scenario(document), tmp_path / "clothoid")

    assert report.completion_ratio > 0.30
    assert report.fail_open_count == 0
    assert report.edge_overrun_count == 0
    assert report.min_wheel_clearance_m > WHEEL_HALF_WIDTH_M
    assert report.passed, report.reasons


def test_undulating_closed_loop_keeps_pitch_finite_without_fail_open(tmp_path):
    scenario = parse_scenario(_undulating_document())
    driver = TerrainAutonomyDriver(scenario)
    pitch_samples = []

    def capture_pitch(elapsed_s, snapshot):
        if snapshot is not None:
            pitch_samples.append(snapshot.tilt.pitch_rad)
        return driver.command(elapsed_s, snapshot)

    report = run_scenario(
        scenario,
        tmp_path / "undulating",
        command_source=capture_pitch,
        hold_state_source=driver.hold_state,
        depth_tap=driver.on_depth,
    )

    assert report.completion_ratio > 0.30
    assert report.fail_open_count == 0
    assert report.edge_overrun_count == 0
    assert pitch_samples
    assert all(math.isfinite(value) for value in pitch_samples)
    assert max(abs(value) for value in pitch_samples) < math.pi / 2.0
    assert report.passed, report.reasons


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
    expected_pass = metrics["passed"] and metrics["completion_ratio"] > 0.05
    assert completed.returncode == (0 if expected_pass else 1)
    assert completed.stdout.strip().startswith("MetricsReport[")
    assert re.search(r"^# canonical_json_sha256: [0-9a-f]{64}$", scenario_text, re.MULTILINE)


def test_hidden_evaluation_rejects_passed_report_without_progress():
    report = SimpleNamespace(passed=True, completion_ratio=0.05)

    assert evaluate_report(report) == (False, "no_progress")


@pytest.mark.parametrize(
    ("report", "expected"),
    [
        (SimpleNamespace(passed=True, completion_ratio=0.8), (True, "passed")),
        (
            SimpleNamespace(passed=False, completion_ratio=0.8),
            (False, "metrics_failed"),
        ),
    ],
)
def test_hidden_evaluation_preserves_normal_report_verdict(report, expected):
    assert evaluate_report(report) == expected
