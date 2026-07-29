from __future__ import annotations

from io import StringIO
import json
import math
import re

import pytest


pytest.importorskip("mujoco")

from chassis.kinematics import default_geometry
from powertrain_autonomy.controller import AutonomyControllerConfig
from powertrain_sim import family_scenarios
from powertrain_sim.campaign import (
    CampaignConfigurationError,
    DEV_SEEDS,
    FAMILIES,
    build_family_document,
    run_campaign,
)
from powertrain_sim.family_scenarios import (
    ROBOT_FOOTPRINT_WIDTH_M,
    TRAINING_TRACK_LENGTH_M,
    TRAINING_TRACK_WIDTH_M,
    flat_document,
)
from powertrain_sim.procedural import canonical_json_sha256


TWO_FAMILIES = ("flat", "smog")
ROW_KEYS = {
    "family",
    "seed",
    "scenario_sha256",
    "passed",
    "completion",
    "fail_open",
    "edge_overrun",
    "recovery",
}


def test_two_family_dev_matrix_is_deterministic_and_writes_report_schema(tmp_path):
    first_stdout = StringIO()
    first = run_campaign(
        tmp_path / "first",
        families=TWO_FAMILIES,
        seed_class="dev",
        stdout=first_stdout,
    )
    second = run_campaign(
        tmp_path / "second",
        families=TWO_FAMILIES,
        seed_class="dev",
    )

    assert first == second
    assert set(first) == {
        "schema_version",
        "seed_class",
        "families",
        "seeds",
        "passed",
        "results",
    }
    assert first["schema_version"] == 1
    assert first["seed_class"] == "dev"
    assert first["families"] == list(TWO_FAMILIES)
    assert first["seeds"] == list(DEV_SEEDS)
    assert len(first["results"]) == len(TWO_FAMILIES) * len(DEV_SEEDS)
    assert all(set(row) == ROW_KEYS for row in first["results"])
    assert [row["family"] for row in first["results"]] == list(TWO_FAMILIES)
    assert all(isinstance(row["passed"], bool) for row in first["results"])
    assert all(isinstance(row["completion"], float) for row in first["results"])
    assert all(isinstance(row["fail_open"], int) for row in first["results"])
    assert all(isinstance(row["edge_overrun"], int) for row in first["results"])
    assert all(isinstance(row["recovery"], float) for row in first["results"])

    on_disk = json.loads(
        (tmp_path / "first" / "campaign.json").read_text(encoding="utf-8")
    )
    assert on_disk == first
    lines = first_stdout.getvalue().strip().splitlines()
    assert lines[0].split() == [
        "family",
        "seed",
        "passed",
        "completion",
        "fail_open",
        "edge_overrun",
        "recovery",
    ]
    assert len(lines) == 1 + len(first["results"])


def test_hidden_matrix_records_only_canonical_hash_and_metrics(tmp_path):
    hidden_seed = 17
    output = tmp_path / "hidden"

    report = run_campaign(
        output,
        families=TWO_FAMILIES,
        seed_class="hidden",
        seeds=(hidden_seed,),
    )

    assert report["seed_class"] == "hidden"
    assert report["seeds"] == [hidden_seed]
    assert not list(output.rglob("scenario.json"))
    assert not list(output.rglob("scenario.yaml"))
    assert "scenario" not in json.dumps(report, sort_keys=True).replace(
        "scenario_sha256", ""
    )
    for row in report["results"]:
        expected_document = build_family_document(
            row["family"],
            seed=hidden_seed,
            seed_class="hidden_evaluation",
        )
        assert row["scenario_sha256"] == canonical_json_sha256(expected_document)
        assert re.fullmatch(r"[0-9a-f]{64}", row["scenario_sha256"])
        assert set(row) == ROW_KEYS


def test_regression_seed_class_is_delegated_to_environment_manifest(tmp_path):
    with pytest.raises(
        CampaignConfigurationError,
        match=r"tests/fixtures/environment/manifest\.yaml",
    ):
        run_campaign(
            tmp_path / "regression",
            families=("flat",),
            seed_class="regression",
        )


def test_robot_footprint_width_matches_production_geometry():
    """캠페인 차폭 기준은 production 기하와 함께 바뀌어야 한다."""
    geometry = default_geometry()
    widest_wheel_center_m = max(abs(wheel.y) for wheel in geometry.wheels)
    # m4_campaign.WHEEL_HALF_WIDTH_M 를 미러한다(Isaac import 는 피한다).
    wheel_half_width_m = 0.035
    derived_footprint_width_m = 2.0 * (
        widest_wheel_center_m + wheel_half_width_m
    )

    assert ROBOT_FOOTPRINT_WIDTH_M == pytest.approx(
        derived_footprint_width_m,
        abs=1e-9,
    )


@pytest.mark.parametrize(
    "family",
    [
        name
        for name in FAMILIES
        if name not in ("pinch", "narrow_curve", "follow")
    ],
)
def test_training_track_is_long_and_wide_enough_for_the_real_rover(family):
    document = build_family_document(family, seed=0, seed_class="dev")

    widths = document["track"]["width_m"]
    assert min(widths) == pytest.approx(TRAINING_TRACK_WIDTH_M, abs=1e-6)
    # 차폭 789 mm 대비 편측 여유 405.5 mm
    assert (min(widths) - ROBOT_FOOTPRINT_WIDTH_M) / 2.0 > 0.30

    centerline = document["track"]["centerline_m"]
    span = max(point[0] for point in centerline) - min(
        point[0] for point in centerline
    )
    assert span == pytest.approx(TRAINING_TRACK_LENGTH_M, rel=0.15)


def test_pinch_family_keeps_its_deliberate_narrowing():
    """폭 확대가 pinch 의 의도적 좁힘을 덮어쓰면 안 된다."""
    document = build_family_document("pinch", seed=0, seed_class="dev")

    widths = document["track"]["width_m"]
    assert min(widths) < ROBOT_FOOTPRINT_WIDTH_M + 0.20
    assert max(widths) > min(widths)


def test_pinch_family_exercises_clearance_speed_ramp():
    """이 게이트가 없으면 전 구간 ramp 포화로 캠페인이 조용히 판별력을 잃는다."""
    document = build_family_document("pinch", seed=0, seed_class="dev")
    config = AutonomyControllerConfig()

    widths = document["track"]["width_m"]
    clearance_m = (min(widths) - ROBOT_FOOTPRINT_WIDTH_M) / 2.0
    assert clearance_m > config.clearance_hold_m
    assert clearance_m < config.clearance_full_m


def test_narrow_curve_width_tracks_controller_full_clearance():
    """컨트롤러 full-clearance 재튜닝이 family 폭과 조용히 어긋나면 안 된다."""
    config = AutonomyControllerConfig()

    assert (
        family_scenarios.NARROW_CURVE_TRACK_WIDTH_M
        == ROBOT_FOOTPRINT_WIDTH_M + 2 * config.clearance_full_m
    )


def test_narrow_curve_puts_clearance_hold_within_reach():
    """이 family가 없으면 어떤 campaign family도 clearance hold를 시험하지 않는다.

    검증된 clothoid 횡오차 0.379 m보다 작은 0.11 m를 보수적인 횡오차
    allowance로 잡아, 복도는 맞지만 치우치면 hold에 닿는 폭인지 확인한다.
    """
    document = build_family_document("narrow_curve", seed=0, seed_class="dev")
    config = AutonomyControllerConfig()
    lateral_error_allowance_m = 0.11

    minimum_width_m = min(document["track"]["width_m"])
    centred_clearance_m = (
        minimum_width_m - ROBOT_FOOTPRINT_WIDTH_M
    ) / 2.0

    assert centred_clearance_m > config.clearance_hold_m
    assert (
        centred_clearance_m
        <= config.clearance_full_m + lateral_error_allowance_m
    )
    assert (
        centred_clearance_m - lateral_error_allowance_m
        <= config.clearance_hold_m
    )


def test_narrow_curve_centerline_changes_heading():
    document = build_family_document("narrow_curve", seed=0, seed_class="dev")

    points = document["track"]["centerline_m"]
    headings_rad = [
        math.atan2(right[1] - left[1], right[0] - left[0])
        for left, right in zip(points, points[1:])
    ]
    heading_excursion_rad = max(headings_rad) - min(headings_rad)

    # ±0.08 /m 를 15 m에서 선형 전이하면 약 0.30 rad가 생긴다.
    # 이산 station에서도 의미 있는 곡선을 요구하되 0.20 rad로 여유를 둔다.
    assert heading_excursion_rad > 0.20


def test_pinch_family_has_time_to_reach_its_own_narrowing():
    """도달 못할 좁힘은 아무것도 시험하지 않고 edge_overrun=0만 깨끗이 남긴다.

    Isaac 실측/명목 속도 비는 0.34 / 0.45 = 0.76이며, 0.6으로 내림해
    보수적으로 좁힘 도달 여유를 검증한다.
    """
    document = build_family_document("pinch", seed=0, seed_class="dev")

    far_edge_x = max(
        point[0]
        for point, width in zip(
            document["track"]["centerline_m"],
            document["track"]["width_m"],
        )
        if width < TRAINING_TRACK_WIDTH_M
    )
    # pinch_document 의 linear_speed_range_m_s=(0.45, 0.45) 를 미러한다.
    nominal_speed_m_s = 0.45
    duration_s = document["clock"]["duration_s"]

    assert duration_s * nominal_speed_m_s * 0.6 > far_edge_x


def test_undulating_family_matches_the_measured_course_profile():
    document = build_family_document("undulating", seed=0, seed_class="dev")

    heights = document["track"]["height_m"]
    peak_to_peak = max(heights) - min(heights)
    # 대회 코스 실측: 0.085 <-> 0.388 m (peak-to-peak 0.303 m)
    assert peak_to_peak == pytest.approx(0.30, abs=0.05)


def test_clock_duration_covers_the_longer_track():
    document = build_family_document("flat", seed=0, seed_class="dev")

    speed = document["motion"]["linear_speed_m_s"]
    duration = document["clock"]["duration_s"]
    assert speed * duration >= TRAINING_TRACK_LENGTH_M


def test_transient_hold_repin_applies_to_dev_documents_only():
    dev = flat_document(seed=0, seed_class="dev")
    hidden = flat_document(seed=0, seed_class="hidden_evaluation")

    assert dev["expected_metrics"]["false_hold_count"] == 20
    assert dev["expected_metrics"]["max_recovery_time_s"] == 0.3
    assert hidden["expected_metrics"]["false_hold_count"] == 0
    assert hidden["expected_metrics"]["max_recovery_time_s"] == 0.25
