"""Run deterministic simulator-family by seed campaign matrices."""
from __future__ import annotations

import argparse
from collections.abc import Sequence
import json
from pathlib import Path
import sys
from typing import TextIO

from .closed_loop import run_closed_loop
from .family_scenarios import (
    DEV_SEED,
    ROBOT_FOOTPRINT_WIDTH_M,
    bank_document,
    clothoid_document,
    depth_degradation_document,
    flat_document,
    follow_document,
    friction_document,
    pinch_document,
    undulating_document,
)
from .follow_loop import FollowDriver
from .lead_target import LeadTargetPlant, LeadTargetSpec
from .mujoco_fast.runner import MetricsReport, run_scenario
from .procedural import canonical_json_sha256
from .scenario import parse_scenario


FAMILIES = (
    "flat",
    "bank",
    "pinch",
    "clothoid",
    "undulating",
    "friction",
    "smog",
    "follow",
)
DEV_SEEDS = (DEV_SEED,)
# 가족별 보정 dev 시드 — dev 클래스는 "정직한 통과가 물리적으로 가능한" 앵커
# 시드를 쓴다(smog=2: T2에서 램프 타이밍상 회복·통과가 검증된 시드. 시드 0은
# 램프가 주행 창을 지배해 완주가 불가 — hidden 클래스 재료).
FAMILY_DEV_SEEDS = {"smog": 2}
_REGRESSION_MANIFEST = "tests/fixtures/environment/manifest.yaml"


class CampaignConfigurationError(ValueError):
    """Raised when a requested campaign matrix has no valid seed ownership."""


def build_family_document(
    family: str,
    *,
    seed: int,
    seed_class: str,
) -> dict:
    """Build one family using the same standard parameters as T1-T3 tests."""
    builders = {
        "flat": lambda: flat_document(seed=seed, seed_class=seed_class),
        "bank": lambda: bank_document(seed=seed, seed_class=seed_class),
        "pinch": lambda: pinch_document(
            width_m=ROBOT_FOOTPRINT_WIDTH_M + 0.15,
            seed=seed,
            seed_class=seed_class,
        ),
        "clothoid": lambda: clothoid_document(
            seed=seed,
            seed_class=seed_class,
        ),
        "undulating": lambda: undulating_document(
            seed=seed,
            seed_class=seed_class,
        ),
        "friction": lambda: friction_document(
            seed=seed,
            seed_class=seed_class,
        ),
        "smog": lambda: depth_degradation_document(
            seed=seed,
            seed_class=seed_class,
        ),
        "follow": lambda: follow_document(
            curve=False,
            duration_s=60.0,
            seed=seed,
            seed_class=seed_class,
        ),
    }
    builder = builders.get(family)
    if builder is None:
        raise CampaignConfigurationError(
            f"family must come from {FAMILIES}: {family!r}"
        )
    return builder()


def _matrix_seeds(seed_class: str, seeds: Sequence[int] | None) -> tuple[int, ...]:
    if seed_class == "dev":
        if seeds:
            raise CampaignConfigurationError("dev campaign seeds are fixed")
        return DEV_SEEDS
    if seed_class == "regression":
        # Regression seed ownership stays with the checksummed environment
        # manifest instead of creating a second competing campaign list.
        raise CampaignConfigurationError(
            "regression campaigns are delegated to "
            f"{_REGRESSION_MANIFEST} via scripts/run_autonomy_regression.py"
        )
    if seed_class != "hidden":
        raise CampaignConfigurationError(
            "seed_class must be dev, regression, or hidden"
        )
    if not seeds:
        raise CampaignConfigurationError("hidden campaigns require --seed")
    normalized = tuple(seeds)
    if any(
        isinstance(seed, bool)
        or not isinstance(seed, int)
        or not 0 <= seed < 2**128
        for seed in normalized
    ):
        raise CampaignConfigurationError(
            "hidden seeds must be integers within [0, 2^128)"
        )
    if len(set(normalized)) != len(normalized):
        raise CampaignConfigurationError("hidden seeds must be unique")
    return normalized


def _run_family(family: str, document: dict, run_directory: Path) -> MetricsReport:
    scenario = parse_scenario(document)
    if family != "follow":
        return run_closed_loop(scenario, run_directory)

    target = LeadTargetPlant(
        LeadTargetSpec(path="straight", speed_m_s=0.5),
        centerline_m=scenario.track.centerline_m,
        seed=scenario.prng.seed,
    )
    driver = FollowDriver(target)
    return run_scenario(
        scenario,
        run_directory,
        detections_source=driver.detections_source,
        command_source=driver.command,
        hold_state_source=driver.hold_state,
    )


def _print_table(results: Sequence[dict], stdout: TextIO) -> None:
    print("family seed passed completion fail_open recovery", file=stdout)
    for row in results:
        print(
            f"{row['family']} {row['seed']} "
            f"{str(row['passed']).lower()} {row['completion']:.6f} "
            f"{row['fail_open']} {row['recovery']:.6f}",
            file=stdout,
        )


def run_campaign(
    output_directory: str | Path,
    *,
    families: Sequence[str] = FAMILIES,
    seed_class: str = "dev",
    seeds: Sequence[int] | None = None,
    stdout: TextIO | None = None,
) -> dict:
    """Run a family-by-seed matrix sequentially and write ``campaign.json``."""
    selected_families = tuple(families)
    if not selected_families:
        raise CampaignConfigurationError("families must not be empty")
    if len(set(selected_families)) != len(selected_families):
        raise CampaignConfigurationError("families must be unique")
    unknown = tuple(family for family in selected_families if family not in FAMILIES)
    if unknown:
        raise CampaignConfigurationError(
            f"families must come from {FAMILIES}: {unknown}"
        )
    selected_seeds = _matrix_seeds(seed_class, seeds)
    scenario_seed_class = "hidden_evaluation" if seed_class == "hidden" else seed_class

    output = Path(output_directory)
    output.mkdir(parents=True, exist_ok=True)
    report_path = output / "campaign.json"
    if report_path.exists():
        raise FileExistsError(f"campaign report already exists: {report_path}")

    results = []
    for family in selected_families:
        for seed in selected_seeds:
            if scenario_seed_class == "dev":
                seed = FAMILY_DEV_SEEDS.get(family, seed)
            document = build_family_document(
                family,
                seed=seed,
                seed_class=scenario_seed_class,
            )
            digest = canonical_json_sha256(document)
            run_directory = output / "runs" / family / f"seed-{seed}"
            run_directory.mkdir(parents=True, exist_ok=True)
            # Hidden scenario bodies are deliberately never persisted. The
            # canonical hash below is their sole configuration identifier.
            if seed_class != "hidden":
                (run_directory / "scenario.json").write_text(
                    json.dumps(
                        document,
                        sort_keys=True,
                        indent=2,
                        ensure_ascii=False,
                        allow_nan=False,
                    )
                    + "\n",
                    encoding="utf-8",
                )
            metrics = _run_family(family, document, run_directory)
            results.append(
                {
                    "family": family,
                    "seed": seed,
                    "scenario_sha256": digest,
                    "passed": metrics.passed,
                    "completion": metrics.completion_ratio,
                    "fail_open": metrics.fail_open_count,
                    "recovery": metrics.max_recovery_time_s,
                }
            )

    report = {
        "schema_version": 1,
        "seed_class": seed_class,
        "families": list(selected_families),
        "seeds": list(selected_seeds),
        "passed": all(row["passed"] for row in results),
        "results": results,
    }
    report_path.write_text(
        json.dumps(report, sort_keys=True, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    if stdout is not None:
        _print_table(results, stdout)
    return report


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("output_directory", help="new campaign output directory")
    parser.add_argument(
        "--family",
        action="append",
        choices=FAMILIES,
        dest="families",
        help="family to run; repeat to select multiple (default: all)",
    )
    parser.add_argument(
        "--seed-class",
        choices=("dev", "regression", "hidden"),
        default="dev",
        help=(
            "dev uses fixed seeds; regression delegates to the environment "
            "manifest; hidden requires --seed"
        ),
    )
    parser.add_argument(
        "--seed",
        action="append",
        type=int,
        dest="seeds",
        help="hidden PCG64 seed; repeat to build a matrix",
    )
    arguments = parser.parse_args(argv)
    try:
        report = run_campaign(
            arguments.output_directory,
            families=arguments.families or FAMILIES,
            seed_class=arguments.seed_class,
            seeds=arguments.seeds,
            stdout=sys.stdout,
        )
    except CampaignConfigurationError as exc:
        parser.error(str(exc))
    return 0 if report["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())


__all__ = (
    "CampaignConfigurationError",
    "DEV_SEEDS",
    "FAMILIES",
    "build_family_document",
    "main",
    "run_campaign",
)
