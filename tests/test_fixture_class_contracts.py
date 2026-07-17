from __future__ import annotations

import hashlib
from pathlib import Path

import pytest
import yaml

from scripts.run_autonomy_regression import (
    FIXTURE_CLASSES,
    FIXTURE_CLASS_CONTRACTS,
    ManifestError,
    load_manifest,
)


REPO_ROOT = Path(__file__).resolve().parents[1]
MANIFEST_PATH = REPO_ROOT / "tests/fixtures/environment/manifest.yaml"


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _write_single_entry_manifest(
    tmp_path: Path,
    *,
    scenario: Path,
    fixture_class: str,
    contract: str,
) -> Path:
    manifest = tmp_path / "manifest.yaml"
    manifest.write_text(
        yaml.safe_dump(
            {
                "manifest_version": 1,
                "fixtures": [
                    {
                        "id": "contract-check",
                        "source": "analytic",
                        "scenario": scenario.relative_to(tmp_path).as_posix(),
                        "fixture_class": fixture_class,
                        "contract": contract,
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
    return manifest


def test_fixture_class_contract_map_covers_every_supported_label():
    assert set(FIXTURE_CLASS_CONTRACTS) == set(FIXTURE_CLASSES)
    assert all(callable(validator) for validator in FIXTURE_CLASS_CONTRACTS.values())


def test_repository_manifest_marks_only_data_evidenced_entries_executable():
    raw = yaml.safe_load(MANIFEST_PATH.read_text(encoding="utf-8"))
    contracts = {
        (entry["id"], entry["source"]): entry["contract"]
        for entry in raw["fixtures"]
    }

    assert {
        pair for pair, contract in contracts.items() if contract == "executable"
    } == {
        ("bank-transition", "mujoco"),
        ("procedural-dev-0", "analytic"),
        ("depth-hole-bank", "analytic"),
    }
    assert set(contracts.values()) == {"executable", "declared-only"}

    loaded = load_manifest(MANIFEST_PATH, repo_root=REPO_ROOT)
    assert {(entry.id, entry.source): entry.contract for entry in loaded} == contracts


def test_executable_bank_transition_rejects_flat_track_data(tmp_path):
    scenario_dir = tmp_path / "scenarios"
    scenario_dir.mkdir()
    scenario = scenario_dir / "flat.yaml"
    scenario.write_bytes(
        (REPO_ROOT / "powertrain_sim/scenarios/flat_straight_5m.yaml").read_bytes()
    )
    manifest = _write_single_entry_manifest(
        tmp_path,
        scenario=scenario,
        fixture_class="bank_transition",
        contract="executable",
    )

    with pytest.raises(ManifestError, match="contract violation.*bank_transition"):
        load_manifest(manifest, repo_root=tmp_path)


def test_declared_only_does_not_invent_missing_shadow_model(tmp_path):
    scenario_dir = tmp_path / "scenarios"
    scenario_dir.mkdir()
    scenario = scenario_dir / "flat.yaml"
    scenario.write_bytes(
        (REPO_ROOT / "powertrain_sim/scenarios/flat_straight_5m.yaml").read_bytes()
    )
    manifest = _write_single_entry_manifest(
        tmp_path,
        scenario=scenario,
        fixture_class="shadow_backlight",
        contract="declared-only",
    )

    loaded = load_manifest(manifest, repo_root=tmp_path)

    assert loaded[0].contract == "declared-only"


def test_executable_depth_hole_rejects_scenario_without_depth_defect(tmp_path):
    scenario_dir = tmp_path / "scenarios"
    scenario_dir.mkdir()
    scenario = scenario_dir / "bank-no-depth-defect.yaml"
    document = yaml.safe_load(
        (REPO_ROOT / "powertrain_sim/scenarios/bank_transition.yaml").read_text(
            encoding="utf-8"
        )
    )
    document["faults"]["depth_holes"] = []
    document["faults"]["depth_spikes"] = []
    scenario.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    manifest = _write_single_entry_manifest(
        tmp_path,
        scenario=scenario,
        fixture_class="depth_hole_jump",
        contract="executable",
    )

    with pytest.raises(ManifestError, match="contract violation.*depth_hole_jump"):
        load_manifest(manifest, repo_root=tmp_path)


def test_executable_depth_hole_accepts_scheduled_spike_jump(tmp_path):
    scenario_dir = tmp_path / "scenarios"
    scenario_dir.mkdir()
    scenario = scenario_dir / "bank-spike-only.yaml"
    document = yaml.safe_load(
        (REPO_ROOT / "powertrain_sim/scenarios/bank_transition.yaml").read_text(
            encoding="utf-8"
        )
    )
    document["faults"]["depth_holes"] = []
    scenario.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
    manifest = _write_single_entry_manifest(
        tmp_path,
        scenario=scenario,
        fixture_class="depth_hole_jump",
        contract="executable",
    )

    loaded = load_manifest(manifest, repo_root=tmp_path)

    assert loaded[0].contract == "executable"
