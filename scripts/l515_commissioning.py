#!/usr/bin/env python3
"""Evaluate L515 mounting candidates and freeze explicitly approved settings."""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import sys
import tempfile
from typing import Any

import yaml


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from powertrain_autonomy.sensor_qualification import (  # noqa: E402
    PitchMetrics,
    PitchRequirements,
    qualify_pitch_bracket,
)


PITCH_FIELDS = (
    "near_blind_spot_m",
    "coverage_min_m",
    "coverage_max_m",
    "footprint_clearance_m",
    "below_floor_separation_m",
)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        formatter_class=argparse.RawDescriptionHelpFormatter,
        description=(
            "Record raw L515 terrain metrics for the 20/25/30 degree bracket. "
            "The reproducible bracket and reference-plane fixture are a mechanical-team "
            "handoff; a temporary mount angle is never inferred as the production angle."
        )
    )
    parser.add_argument(
        "--mode",
        choices=("dry-run", "commissioning", "production"),
        required=True,
        help=(
            "dry-run renders and hashes without YAML writes; commissioning may freeze an "
            "explicit passing candidate; production is measurement-only"
        ),
    )
    parser.add_argument("--input", type=Path, required=True, help="candidate input JSON")
    parser.add_argument("--jsonl", type=Path, required=True, help="raw metric JSONL output")
    parser.add_argument("--csv", type=Path, required=True, help="raw metric CSV output")
    parser.add_argument(
        "--approve-pitch",
        type=float,
        help="explicit candidate to hash/freeze; never selected automatically",
    )
    parser.add_argument(
        "--output-yaml",
        type=Path,
        default=ROOT / "ros2/src/powertrain_ros/config/l515_terrain.yaml",
        help="frozen YAML path (written only in commissioning mode)",
    )
    return parser


def _load_input(path: Path) -> tuple[dict[str, Any], bytes]:
    encoded = path.read_bytes()
    loaded = json.loads(encoded)
    if not isinstance(loaded, dict):
        raise ValueError("commissioning input must be a JSON object")
    return loaded, encoded


def _fixture_reasons(fixture: dict[str, Any]) -> tuple[str, ...]:
    reasons: list[str] = []
    if fixture.get("bracket_owner") != "mechanical_team":
        reasons.append("fixture_bracket_owner")
    if fixture.get("reference_plane_owner") != "mechanical_team":
        reasons.append("fixture_reference_plane_owner")
    pitches = sorted(float(value) for value in fixture.get("reproducible_pitch_deg", ()))
    if pitches != [20.0, 25.0, 30.0]:
        reasons.append("fixture_pitch_reproducibility")
    if fixture.get("reference_plane_available") is not True:
        reasons.append("fixture_reference_plane_missing")
    return tuple(reasons)


def _qualify(data: dict[str, Any]):
    requirements = PitchRequirements(**data["requirements"])
    raw_candidates = data["candidates"]
    if not isinstance(raw_candidates, list):
        raise ValueError("candidates must be a JSON array")
    metrics = tuple(
        PitchMetrics(
            pitch_deg=candidate["pitch_deg"],
            **{field: candidate["raw_metrics"][field] for field in PITCH_FIELDS},
        )
        for candidate in raw_candidates
    )
    bracket = qualify_pitch_bracket(metrics, requirements=requirements)
    raw_by_pitch = {float(candidate["pitch_deg"]): candidate for candidate in raw_candidates}
    records = []
    for qualified in bracket.candidates:
        pitch = qualified.metrics.pitch_deg
        raw = raw_by_pitch[pitch]
        records.append(
            {
                "pitch_deg": pitch,
                "raw_metrics": dict(raw["raw_metrics"]),
                "passed": qualified.passed,
                "reject_reasons": list(qualified.reject_reasons),
                "roi": raw["roi"],
                "depth_thresholds": raw["depth_thresholds"],
                "base_link_to_l515_link": raw["base_link_to_l515_link"],
            }
        )
    return bracket, records, raw_by_pitch


def _write_metrics(jsonl_path: Path, csv_path: Path, records: list[dict[str, Any]]) -> None:
    jsonl_path.parent.mkdir(parents=True, exist_ok=True)
    with jsonl_path.open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n")

    csv_path.parent.mkdir(parents=True, exist_ok=True)
    fieldnames = (
        "pitch_deg",
        "passed",
        "reject_reasons",
        *PITCH_FIELDS,
        "roi_json",
        "depth_thresholds_json",
        "base_link_to_l515_link_json",
        "raw_metrics_json",
    )
    with csv_path.open("w", encoding="utf-8", newline="") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        for record in records:
            row = {
                "pitch_deg": record["pitch_deg"],
                "passed": record["passed"],
                "reject_reasons": ";".join(record["reject_reasons"]),
                "roi_json": json.dumps(record["roi"], sort_keys=True),
                "depth_thresholds_json": json.dumps(
                    record["depth_thresholds"], sort_keys=True
                ),
                "base_link_to_l515_link_json": json.dumps(
                    record["base_link_to_l515_link"], sort_keys=True
                ),
                "raw_metrics_json": json.dumps(record["raw_metrics"], sort_keys=True),
            }
            row.update({field: record["raw_metrics"][field] for field in PITCH_FIELDS})
            writer.writerow(row)


def _select_approval(
    approve_pitch: float,
    *,
    bracket,
    records: list[dict[str, Any]],
    fixture_reasons: tuple[str, ...],
) -> dict[str, Any]:
    if fixture_reasons:
        raise ValueError("mechanical fixture qualification failed: " + ", ".join(fixture_reasons))
    if bracket.reject_reasons:
        raise ValueError("pitch bracket qualification failed: " + ", ".join(bracket.reject_reasons))
    selected = next(
        (record for record in records if record["pitch_deg"] == approve_pitch),
        None,
    )
    if selected is None:
        raise ValueError(f"approved pitch {approve_pitch:g} is not a measured candidate")
    if not selected["passed"]:
        raise ValueError(
            f"approved pitch {approve_pitch:g} failed: "
            + ", ".join(selected["reject_reasons"])
        )
    return selected


def _render_frozen_yaml(
    *,
    selected: dict[str, Any],
    source_input_sha256: str,
) -> bytes:
    document = {
        "schema_version": 1,
        "qualification": {
            "status": "approved",
            "production_enabled": True,
            "fixture_owner": "mechanical_team",
            "required_pitch_candidates_deg": [20, 25, 30],
            "source_input_sha256": source_input_sha256,
            "selected_raw_metrics": selected["raw_metrics"],
        },
        "mount": {"pitch_deg": selected["pitch_deg"]},
        "terrain": {
            "backend": "numpy",
            "roi": selected["roi"],
            "depth_thresholds": selected["depth_thresholds"],
        },
        "tf": {
            "base_link_to_l515_link": selected["base_link_to_l515_link"],
        },
    }
    header = (
        "# Generated only by explicit L515 commissioning approval.\n"
        "# The 20/25/30 degree bracket and reference plane are a mechanical-team handoff.\n"
        "# Runtime production measurement must not modify this file.\n"
    )
    return (header + yaml.safe_dump(document, sort_keys=False)).encode("utf-8")


def _atomic_write(path: Path, content: bytes) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    file_descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent,
        prefix=f".{path.name}.",
    )
    try:
        with os.fdopen(file_descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_name, path)
    except BaseException:
        try:
            os.unlink(temporary_name)
        except FileNotFoundError:
            pass
        raise


def main(argv: list[str] | None = None) -> int:
    parser = _parser()
    args = parser.parse_args(argv)
    if args.mode == "production" and args.approve_pitch is not None:
        parser.error("production mode is measurement-only; --approve-pitch is forbidden")
    if args.mode == "commissioning" and args.approve_pitch is None:
        parser.error("commissioning mode requires an explicit --approve-pitch")

    try:
        data, input_bytes = _load_input(args.input)
        fixture_reasons = _fixture_reasons(data.get("fixture", {}))
        bracket, records, _ = _qualify(data)
        _write_metrics(args.jsonl, args.csv, records)

        summary: dict[str, Any] = {
            "mode": args.mode,
            "candidate_count": len(records),
            "fixture_passed": not fixture_reasons,
            "fixture_reject_reasons": list(fixture_reasons),
            "bracket_reject_reasons": list(bracket.reject_reasons),
            "yaml_written": False,
        }
        if args.approve_pitch is not None:
            selected = _select_approval(
                args.approve_pitch,
                bracket=bracket,
                records=records,
                fixture_reasons=fixture_reasons,
            )
            rendered = _render_frozen_yaml(
                selected=selected,
                source_input_sha256=hashlib.sha256(input_bytes).hexdigest(),
            )
            summary["approved_pitch_deg"] = selected["pitch_deg"]
            summary["yaml_sha256"] = hashlib.sha256(rendered).hexdigest()
            if args.mode == "commissioning":
                _atomic_write(args.output_yaml, rendered)
                summary["yaml_written"] = True
        elif args.mode == "production" and args.output_yaml.exists():
            summary["existing_yaml_sha256"] = hashlib.sha256(
                args.output_yaml.read_bytes()
            ).hexdigest()
        print(json.dumps(summary, sort_keys=True))
        return 0
    except (KeyError, TypeError, ValueError, OSError, json.JSONDecodeError) as error:
        parser.error(str(error))


if __name__ == "__main__":
    raise SystemExit(main())
