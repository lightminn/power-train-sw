from __future__ import annotations

import hashlib
import json
from pathlib import Path

import pytest
import yaml

try:
    from jsonschema import Draft202012Validator
except ModuleNotFoundError:
    # 컨테이너 이미지에는 jsonschema가 없다. 스키마 문서 검증은 host(conda base)
    # 가 수행하고, 시나리오 강제는 어디서나 parse_scenario가 정본으로 수행한다.
    Draft202012Validator = None

from powertrain_sim.procedural import (
    GenerationParameters,
    canonical_json_sha256,
    generate_scenario,
)
from powertrain_sim.scenario import load_scenario, parse_scenario
from scripts.run_autonomy_regression import (
    ManifestError,
    compare_backend_results,
    load_manifest,
    run_regression,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
FIXTURE_ROOT = REPO_ROOT / "tests" / "fixtures" / "environment"
MANIFEST_PATH = FIXTURE_ROOT / "manifest.yaml"
SCHEMA_PATH = FIXTURE_ROOT / "scenario.schema.yaml"

SOURCES = {"analytic", "replay", "mujoco", "closed_loop"}
FIXTURE_CLASSES = {
    "fog_smoke",
    "shadow_backlight",
    "reflective_surface",
    "occlusion",
    "depth_hole_jump",
    "below_floor",
    "lead_occlusion",
    "marker_duplicate",
    "bank_transition",
    "slip_stuck",
}
REQUIRED_ENTRY_KEYS = {
    "id",
    "source",
    "scenario",
    "fixture_class",
    "expected",
    "tolerance",
    "sensor_contract_version",
    "checksum",
}
SCENARIO_KEYS = {
    "schema_version",
    "scenario_id",
    "description",
    "units",
    "frames",
    "clock",
    "prng",
    "track",
    "motion",
    "sensors",
    "faults",
    "expected_metrics",
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_documented_scenario_schema_matches_parser_top_level_contract():
    document = yaml.safe_load(SCHEMA_PATH.read_text(encoding="utf-8"))

    assert document["$schema"].endswith("schema")
    assert document["title"] == "Powertrain simulator-neutral scenario v1"
    assert set(document["required"]) == SCENARIO_KEYS
    assert set(document["properties"]) == SCENARIO_KEYS
    assert document["properties"]["schema_version"]["const"] == 1
    assert document["properties"]["prng"]["properties"]["algorithm"]["const"] == "PCG64"
    assert document["additionalProperties"] is False


def test_documented_schema_accepts_parser_valid_repository_and_zero_limit_cases():
    if Draft202012Validator is None:
        pytest.skip("jsonschema is host-only; parse_scenario stays authoritative")
    schema = yaml.safe_load(SCHEMA_PATH.read_text(encoding="utf-8"))
    Draft202012Validator.check_schema(schema)
    validator = Draft202012Validator(schema)
    for scenario_path in sorted((REPO_ROOT / "powertrain_sim/scenarios").glob("*.yaml")):
        document = yaml.safe_load(scenario_path.read_text(encoding="utf-8"))
        parse_scenario(document)
        validator.validate(document)

    zero_limit = yaml.safe_load(
        (REPO_ROOT / "powertrain_sim/scenarios/flat_straight_5m.yaml").read_text(
            encoding="utf-8"
        )
    )
    zero_limit["expected_metrics"]["max_estimator_runtime_ms"] = 0.0
    parse_scenario(zero_limit)
    validator.validate(zero_limit)


def test_manifest_has_all_required_classes_enums_paths_and_checksums():
    raw = yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))
    entries = raw["fixtures"]

    assert raw["manifest_version"] == 1
    assert len(entries) >= 8
    assert all(set(entry) == REQUIRED_ENTRY_KEYS for entry in entries)
    assert {entry["source"] for entry in entries} <= SOURCES
    assert {entry["fixture_class"] for entry in entries} <= FIXTURE_CLASSES
    assert all(entry["sensor_contract_version"] == 1 for entry in entries)
    assert all(len(entry["checksum"]) == 64 for entry in entries)
    assert {entry["source"] for entry in entries} >= {
        "analytic",
        "replay",
        "mujoco",
        "closed_loop",
    }

    referenced_paths = {
        entry["scenario"]
        for entry in entries
        if not entry["scenario"].startswith("procedural:")
    }
    assert referenced_paths >= {
        "powertrain_sim/scenarios/flat_straight_5m.yaml",
        "powertrain_sim/scenarios/pivot_90deg.yaml",
        "powertrain_sim/scenarios/bank_transition.yaml",
        "powertrain_sim/scenarios/wide_fov_drop_track.yaml",
    }
    assert sum(entry["scenario"].startswith("procedural:dev:") for entry in entries) >= 2
    assert sum(entry["source"] == "analytic" and "reject_reasons" in entry["expected"] for entry in entries) >= 2

    loaded = load_manifest(MANIFEST_PATH, repo_root=REPO_ROOT)
    assert len(loaded) == len(entries)
    for entry in entries:
        reference = entry["scenario"]
        if reference.startswith("procedural:"):
            _, seed_class, seed_text = reference.split(":")
            generated = generate_scenario(
                GenerationParameters(),
                seed=int(seed_text),
                seed_class=seed_class,
            )
            assert entry["checksum"] == canonical_json_sha256(generated)
        else:
            scenario_path = REPO_ROOT / reference
            assert scenario_path.is_file()
            assert entry["checksum"] == _sha256(scenario_path)
            load_scenario(scenario_path)


def test_checksum_drift_fails_before_backend_execution(tmp_path):
    scenario_dir = tmp_path / "scenarios"
    scenario_dir.mkdir()
    scenario = scenario_dir / "flat.yaml"
    source = REPO_ROOT / "powertrain_sim/scenarios/flat_straight_5m.yaml"
    scenario.write_bytes(source.read_bytes())
    manifest = tmp_path / "manifest.yaml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "manifest_version": 1,
                "fixtures": [
                    {
                        "id": "drift-check",
                        "source": "analytic",
                        "scenario": "scenarios/flat.yaml",
                        "fixture_class": "shadow_backlight",
                        "expected": {"passed": True},
                        "tolerance": {"completion": 0.01},
                        "sensor_contract_version": 1,
                        "checksum": _sha256(scenario),
                    }
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    load_manifest(manifest, repo_root=tmp_path)

    scenario.write_bytes(scenario.read_bytes() + b"\n")

    with pytest.raises(ManifestError, match="checksum mismatch"):
        load_manifest(manifest, repo_root=tmp_path)


def test_procedural_reference_checksum_drift_is_rejected(tmp_path):
    generated = generate_scenario(GenerationParameters(), seed=7, seed_class="dev")
    manifest = tmp_path / "manifest.yaml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "manifest_version": 1,
                "fixtures": [
                    {
                        "id": "procedural-check",
                        "source": "analytic",
                        "scenario": "procedural:dev:7",
                        "fixture_class": "bank_transition",
                        "expected": {"passed": True},
                        "tolerance": {"completion": 0.01},
                        "sensor_contract_version": 1,
                        "checksum": "0" + canonical_json_sha256(generated)[1:],
                    }
                ],
            },
            sort_keys=False,
        ),
        encoding="utf-8",
    )

    with pytest.raises(ManifestError, match="checksum mismatch"):
        load_manifest(manifest, repo_root=tmp_path)


def test_runner_executes_small_analytic_and_recorded_replay_manifest(tmp_path):
    scenario_dir = tmp_path / "scenarios"
    scenario_dir.mkdir()
    scenario = scenario_dir / "flat.yaml"
    document = yaml.safe_load(
        (REPO_ROOT / "powertrain_sim/scenarios/flat_straight_5m.yaml").read_text(
            encoding="utf-8"
        )
    )
    document["scenario_id"] = "runner_small"
    document["clock"]["duration_s"] = 0.1
    document["expected_metrics"]["completion"] = False
    scenario.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    checksum = _sha256(scenario)
    manifest = tmp_path / "manifest.yaml"
    fixtures = []
    for source in ("analytic", "replay"):
        fixtures.append(
            {
                "id": "same-fixture",
                "source": source,
                "scenario": "scenarios/flat.yaml",
                "fixture_class": "shadow_backlight",
                "expected": {"passed": True},
                "tolerance": {"completion": 0.0},
                "sensor_contract_version": 1,
                "checksum": checksum,
            }
        )
    manifest.write_text(
        yaml.safe_dump(
            {"manifest_version": 1, "fixtures": fixtures},
            sort_keys=False,
        ),
        encoding="utf-8",
    )
    out = tmp_path / "result.json"

    exit_code = run_regression(manifest, out, repo_root=tmp_path)

    result = json.loads(out.read_text(encoding="utf-8"))
    assert exit_code == 0
    assert [item["status"] for item in result["results"]] == ["PASS", "PASS"]
    assert result["comparisons"] == [
        {
            "backends": ["analytic", "replay"],
            "differences": {"completion": 0.0},
            "id": "same-fixture",
            "status": "PASS",
        }
    ]
    for item in result["results"]:
        assert set(
            (
                "fail_open",
                "false_hold",
                "min_clearance",
                "runtime_s",
                "reject_reasons",
            )
        ) <= set(item)


def test_backend_comparison_fails_when_shared_metric_exceeds_tolerance():
    results = [
        {
            "id": "same",
            "source": "analytic",
            "status": "PASS",
            "completion": 0.90,
            "min_clearance": None,
        },
        {
            "id": "same",
            "source": "mujoco",
            "status": "PASS",
            "completion": 0.95,
            "min_clearance": 0.11,
        },
    ]

    comparisons = compare_backend_results(
        results,
        tolerance_by_id={"same": {"completion": 0.01}},
    )

    assert comparisons == [
        {
            "backends": ["analytic", "mujoco"],
            "differences": {"completion": pytest.approx(0.05)},
            "id": "same",
            "status": "FAIL",
        }
    ]
