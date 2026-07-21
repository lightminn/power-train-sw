from __future__ import annotations

from io import StringIO
import json
import re

import pytest


pytest.importorskip("mujoco")

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


@pytest.mark.parametrize(
    "family",
    [name for name in FAMILIES if name not in ("pinch", "follow")],
)
def test_training_track_is_long_and_wide_enough_for_the_real_rover(family):
    document = build_family_document(family, seed=0, seed_class="dev")

    widths = document["track"]["width_m"]
    assert min(widths) == pytest.approx(TRAINING_TRACK_WIDTH_M, abs=1e-6)
    # 차폭 949 mm 대비 편측 여유 325 mm
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
