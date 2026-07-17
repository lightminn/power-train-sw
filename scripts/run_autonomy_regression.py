#!/usr/bin/env python3
"""Run the checksummed WP5.3 autonomy environment regression manifest."""
from __future__ import annotations

import argparse
from collections import defaultdict
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import hashlib
import importlib
import json
import math
from pathlib import Path
import re
import tempfile
import time
from typing import Any

import yaml

from chassis.kinematics import default_geometry
from powertrain_autonomy.terrain.depth_quality import analyze_depth_quality
from powertrain_ros.state_estimation import StateEstimator, StateEstimatorConfig
from powertrain_sim.fixtures import FixtureStreams, generate_fixture
from powertrain_sim.procedural import (
    GenerationParameters,
    canonical_json_sha256,
    generate_scenario,
)
from powertrain_sim.recording import Replayer, RunWriter
from powertrain_sim.scenario import Scenario, load_scenario, parse_scenario


REPO_ROOT = Path(__file__).resolve().parents[1]
SOURCES = frozenset({"analytic", "replay", "mujoco", "closed_loop"})
FIXTURE_CLASSES = frozenset(
    {
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
)
ENTRY_KEYS = frozenset(
    {
        "id",
        "source",
        "scenario",
        "fixture_class",
        "contract",
        "expected",
        "tolerance",
        "sensor_contract_version",
        "checksum",
    }
)
CONTRACT_MODES = frozenset({"executable", "declared-only"})
COMPARABLE_METRICS = frozenset(
    {"completion", "min_clearance", "fail_open", "false_hold"}
)
_ID_PATTERN = re.compile(r"^[a-z0-9][a-z0-9-]*$")
_CHECKSUM_PATTERN = re.compile(r"^[0-9a-f]{64}$")


class ManifestError(ValueError):
    """The regression manifest is invalid or has drifted."""


class BackendSkipped(RuntimeError):
    """An explicitly optional backend is not installed or implemented."""


def _depth_defect_contract(scenario: Scenario) -> str | None:
    rows, cols = (int(value) for value in scenario.sensors["depth"]["shape_px"])
    pixel_count = rows * cols
    for hole in scenario.faults["depth_holes"]:
        row_start, row_stop = (int(value) for value in hole["rows"])
        col_start, col_stop = (int(value) for value in hole["cols"])
        area_ratio = (
            (row_stop - row_start) * (col_stop - col_start) / pixel_count
        )
        if float(hole["end_s"]) > float(hole["start_s"]) and area_ratio >= 0.01:
            return None
    for spike in scenario.faults["depth_spikes"]:
        if (
            float(spike["end_s"]) > float(spike["start_s"])
            and abs(float(spike["offset_m"])) >= 1.0
        ):
            return None
    return (
        "requires a depth-hole interval covering at least 1% of the ROI "
        "or a scheduled spike jump of at least 1 m"
    )


def _occlusion_contract(scenario: Scenario) -> str | None:
    for dropout in scenario.faults["sensor_dropouts"]:
        if (
            dropout["stream"] == "depth"
            and float(dropout["end_s"]) > float(dropout["start_s"])
        ):
            return None
    return "requires a non-empty depth visibility-loss interval"


def _below_floor_contract(scenario: Scenario) -> str | None:
    if any(height < 0.0 for height in scenario.track.height_m):
        return None
    return "requires negative-elevation samples; this scenario contains none"


def _bank_transition_contract(scenario: Scenario) -> str | None:
    bank = scenario.track.bank_rad
    if (
        len(bank) >= 3
        and abs(bank[0]) <= 1e-12
        and abs(bank[-1]) <= 1e-12
        and max(abs(value) for value in bank[1:-1]) > 1e-6
        and max(bank) - min(bank) > 1e-6
    ):
        return None
    return "requires zero-bank endpoints and a non-zero varying interior"


def _slip_stuck_contract(scenario: Scenario) -> str | None:
    for slip in scenario.faults["wheel_slip"]:
        if (
            float(slip["end_s"]) > float(slip["start_s"])
            and float(slip["measurement_scale"]) < 1.0
        ):
            return None
    return "requires a non-empty wheel-slip interval with measurement loss"


def _required_depth_model(field: str) -> Callable[[Scenario], str | None]:
    def validate(scenario: Scenario) -> str | None:
        if field in scenario.sensors["depth"]:
            return None
        return f"requires sensors.depth.{field}, which is absent"

    return validate


def _required_fault_group(field: str) -> Callable[[Scenario], str | None]:
    def validate(scenario: Scenario) -> str | None:
        if scenario.faults.get(field):
            return None
        return f"requires faults.{field}, which is absent"

    return validate


FIXTURE_CLASS_CONTRACTS: Mapping[
    str, Callable[[Scenario], str | None]
] = {
    "fog_smoke": _depth_defect_contract,
    "shadow_backlight": _required_depth_model("shadow_backlight_model"),
    "reflective_surface": _required_depth_model("reflective_surface_model"),
    "occlusion": _occlusion_contract,
    "depth_hole_jump": _depth_defect_contract,
    "below_floor": _below_floor_contract,
    "lead_occlusion": _required_fault_group("lead_occlusion"),
    "marker_duplicate": _required_fault_group("marker_duplicates"),
    "bank_transition": _bank_transition_contract,
    "slip_stuck": _slip_stuck_contract,
}


@dataclass(frozen=True)
class ManifestEntry:
    id: str
    source: str
    scenario_reference: str
    fixture_class: str
    contract: str
    expected: Mapping[str, Any]
    tolerance: Mapping[str, float]
    sensor_contract_version: int
    checksum: str
    scenario: Scenario


def _mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise ManifestError(f"{label} must be a mapping")
    return value


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_reference(reference: str, repo_root: Path) -> tuple[Scenario, str]:
    if reference.startswith("procedural:"):
        parts = reference.split(":")
        if len(parts) != 3 or not parts[1] or not parts[2]:
            raise ManifestError(
                "procedural scenario must be procedural:<seed_class>:<seed>"
            )
        seed_class = parts[1]
        try:
            seed = int(parts[2], 10)
        except ValueError as exc:
            raise ManifestError("procedural seed must be a base-10 integer") from exc
        document = generate_scenario(
            GenerationParameters(),
            seed=seed,
            seed_class=seed_class,
        )
        return parse_scenario(document), canonical_json_sha256(document)

    relative = Path(reference)
    if relative.is_absolute() or ".." in relative.parts:
        raise ManifestError("scenario paths must be repository-relative")
    root = repo_root.resolve()
    scenario_path = (root / relative).resolve()
    try:
        scenario_path.relative_to(root)
    except ValueError as exc:
        raise ManifestError("scenario path escapes repository root") from exc
    if not scenario_path.is_file():
        raise ManifestError(f"scenario path does not exist: {reference}")
    return load_scenario(scenario_path), _sha256(scenario_path)


def _validate_expected(value: Any, label: str) -> dict[str, Any]:
    expected = dict(_mapping(value, label))
    if set(expected) not in ({"passed"}, {"reject_reasons"}):
        raise ManifestError(
            f"{label} must contain exactly passed or reject_reasons"
        )
    if "passed" in expected:
        if type(expected["passed"]) is not bool:
            raise ManifestError(f"{label}.passed must be boolean")
        return expected
    reasons = expected["reject_reasons"]
    if (
        not isinstance(reasons, list)
        or not reasons
        or any(not isinstance(reason, str) or not reason for reason in reasons)
        or len(set(reasons)) != len(reasons)
    ):
        raise ManifestError(
            f"{label}.reject_reasons must be a non-empty unique string list"
        )
    return {"reject_reasons": list(reasons)}


def _validate_tolerance(value: Any, label: str) -> dict[str, float]:
    raw = dict(_mapping(value, label))
    if not raw:
        raise ManifestError(f"{label} must not be empty")
    unknown = set(raw) - COMPARABLE_METRICS
    if unknown:
        raise ManifestError(
            f"{label} has unsupported metrics: {', '.join(sorted(unknown))}"
        )
    output = {}
    for metric, tolerance in raw.items():
        if (
            isinstance(tolerance, bool)
            or not isinstance(tolerance, (int, float))
            or not math.isfinite(float(tolerance))
            or float(tolerance) < 0.0
        ):
            raise ManifestError(f"{label}.{metric} must be finite and nonnegative")
        output[str(metric)] = float(tolerance)
    return output


def _validate_fixture_contract(
    *,
    fixture_class: str,
    contract: str,
    scenario: Scenario,
    label: str,
) -> None:
    if contract == "declared-only":
        return
    violation = FIXTURE_CLASS_CONTRACTS[fixture_class](scenario)
    if violation is not None:
        raise ManifestError(
            f"{label}.contract violation for {fixture_class}: {violation}"
        )


def load_manifest(
    manifest_path: str | Path,
    *,
    repo_root: str | Path = REPO_ROOT,
) -> tuple[ManifestEntry, ...]:
    """Load and checksum every entry before any backend can execute."""
    path = Path(manifest_path)
    root = Path(repo_root)
    try:
        document = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, yaml.YAMLError) as exc:
        raise ManifestError(f"cannot read manifest: {exc}") from exc
    top = _mapping(document, "manifest")
    if set(top) != {"manifest_version", "fixtures"}:
        raise ManifestError("manifest must contain manifest_version and fixtures")
    if top["manifest_version"] != 1:
        raise ManifestError("manifest_version must be 1")
    fixtures = top["fixtures"]
    if not isinstance(fixtures, list) or not fixtures:
        raise ManifestError("fixtures must be a non-empty list")

    entries = []
    seen_pairs = set()
    tolerances_by_id: dict[str, dict[str, float]] = {}
    for index, raw_entry in enumerate(fixtures):
        label = f"fixtures[{index}]"
        entry = _mapping(raw_entry, label)
        if set(entry) != ENTRY_KEYS:
            missing = sorted(ENTRY_KEYS - set(entry))
            extra = sorted(set(entry) - ENTRY_KEYS)
            raise ManifestError(
                f"{label} keys differ; missing={missing}, extra={extra}"
            )
        fixture_id = entry["id"]
        source = entry["source"]
        fixture_class = entry["fixture_class"]
        contract = entry["contract"]
        reference = entry["scenario"]
        checksum = entry["checksum"]
        if not isinstance(fixture_id, str) or not _ID_PATTERN.fullmatch(fixture_id):
            raise ManifestError(f"{label}.id must be a stable kebab-case identifier")
        if source not in SOURCES:
            raise ManifestError(f"{label}.source is unsupported")
        pair = (fixture_id, source)
        if pair in seen_pairs:
            raise ManifestError(f"duplicate fixture id/backend pair: {pair}")
        seen_pairs.add(pair)
        if fixture_class not in FIXTURE_CLASSES:
            raise ManifestError(f"{label}.fixture_class is unsupported")
        if contract not in CONTRACT_MODES:
            raise ManifestError(
                f"{label}.contract must be executable or declared-only"
            )
        if not isinstance(reference, str) or not reference:
            raise ManifestError(f"{label}.scenario must be non-empty")
        if entry["sensor_contract_version"] != 1:
            raise ManifestError(f"{label}.sensor_contract_version must be 1")
        if not isinstance(checksum, str) or not _CHECKSUM_PATTERN.fullmatch(checksum):
            raise ManifestError(f"{label}.checksum must be lowercase SHA-256")
        expected = _validate_expected(entry["expected"], f"{label}.expected")
        tolerance = _validate_tolerance(entry["tolerance"], f"{label}.tolerance")
        previous_tolerance = tolerances_by_id.setdefault(fixture_id, tolerance)
        if previous_tolerance != tolerance:
            raise ManifestError(
                f"fixture {fixture_id} uses inconsistent backend tolerances"
            )

        try:
            scenario, actual_checksum = _load_reference(reference, root)
        except (TypeError, ValueError) as exc:
            if isinstance(exc, ManifestError):
                raise
            raise ManifestError(f"{label}.scenario is invalid: {exc}") from exc
        if actual_checksum != checksum:
            raise ManifestError(
                f"checksum mismatch for {fixture_id}/{source}: "
                f"expected {checksum}, got {actual_checksum}"
            )
        _validate_fixture_contract(
            fixture_class=str(fixture_class),
            contract=str(contract),
            scenario=scenario,
            label=label,
        )
        entries.append(
            ManifestEntry(
                id=fixture_id,
                source=str(source),
                scenario_reference=reference,
                fixture_class=str(fixture_class),
                contract=str(contract),
                expected=expected,
                tolerance=tolerance,
                sensor_contract_version=1,
                checksum=checksum,
                scenario=scenario,
            )
        )
    return tuple(entries)


def _track_length(scenario: Scenario) -> float:
    return sum(
        math.dist(left, right)
        for left, right in zip(
            scenario.track.centerline_m,
            scenario.track.centerline_m[1:],
        )
    )


class _ProductionConsumers:
    def __init__(self) -> None:
        self.estimator = StateEstimator(
            default_geometry(),
            StateEstimatorConfig(bias_samples=0),
        )
        self.previous_depth = None
        self.reject_reasons: list[str] = []
        self.last_stamp_s = 0.0

    def wheel(self, sample) -> None:
        decision = self.estimator.update_wheels(sample, now_s=sample.stamp_s)
        if not decision.accepted:
            raise RuntimeError(
                f"production state estimator rejected wheel: {decision.reason}"
            )
        self.last_stamp_s = max(self.last_stamp_s, sample.stamp_s)

    def imu(self, sample) -> None:
        decision = self.estimator.update_imu(sample, now_s=sample.stamp_s)
        if not decision.accepted:
            raise RuntimeError(
                f"production state estimator rejected IMU: {decision.reason}"
            )
        self.last_stamp_s = max(self.last_stamp_s, sample.stamp_s)

    def depth(self, frame) -> None:
        result = analyze_depth_quality(
            frame.depth_roi,
            depth_scale_m=frame.depth_scale_m,
            intrinsics=frame.intrinsics,
            frame_stamp_s=frame.stamp_s,
            previous=self.previous_depth,
        )
        self.previous_depth = result.snapshot()
        for reason in result.reject_reasons:
            if reason not in self.reject_reasons:
                self.reject_reasons.append(reason)
        self.last_stamp_s = max(self.last_stamp_s, frame.stamp_s)

    def result(self, scenario: Scenario, runtime_s: float) -> dict[str, Any]:
        snapshot = self.estimator.snapshot(now_s=self.last_stamp_s)
        completion = min(1.0, max(0.0, snapshot.distance_m / _track_length(scenario)))
        completed = completion >= 0.95
        passed = completed == bool(scenario.expected_metrics["completion"])
        return {
            "passed": passed,
            "completion": completion,
            "min_clearance": None,
            "fail_open": 0,
            "false_hold": 0,
            "runtime_s": runtime_s,
            "reject_reasons": list(self.reject_reasons),
            "failure_reasons": [] if passed else [
                "completion does not match scenario expected_metrics"
            ],
        }


def _consume_fixture_direct(fixture: FixtureStreams, consumers: _ProductionConsumers) -> None:
    timeline = (
        [(sample.stamp_s, 0, consumers.wheel, sample) for sample in fixture.wheel_states]
        + [(sample.stamp_s, 1, consumers.imu, sample) for sample in fixture.imu]
        + [(frame.stamp_s, 2, consumers.depth, frame) for frame in fixture.depth]
    )
    for _stamp, _order, callback, value in sorted(
        timeline, key=lambda item: (item[0], item[1])
    ):
        callback(value)


def _record_fixture(fixture: FixtureStreams, run_directory: Path) -> None:
    streams = (
        [(sample.stamp_s, 0, "wheel", sample) for sample in fixture.wheel_states]
        + [(sample.stamp_s, 1, "imu", sample) for sample in fixture.imu]
        + [(frame.stamp_s, 2, "depth", frame) for frame in fixture.depth]
        + [(frame.stamp_s, 3, "truth", frame) for frame in fixture.ground_truth]
    )
    with RunWriter(run_directory, run_id=fixture.scenario_id) as writer:
        for _stamp, _order, stream, value in sorted(
            streams, key=lambda item: (item[0], item[1])
        ):
            if stream == "wheel":
                writer.write_wheel(value)
            elif stream == "imu":
                writer.write_imu(value)
            elif stream == "depth":
                writer.write_depth(value)
            else:
                writer.write_ground_truth(value)


def _run_analytic(entry: ManifestEntry, _run_directory: Path) -> dict[str, Any]:
    started = time.perf_counter()
    fixture = generate_fixture(entry.scenario)
    consumers = _ProductionConsumers()
    _consume_fixture_direct(fixture, consumers)
    return consumers.result(entry.scenario, time.perf_counter() - started)


def _run_replay(entry: ManifestEntry, run_directory: Path) -> dict[str, Any]:
    started = time.perf_counter()
    fixture = generate_fixture(entry.scenario)
    _record_fixture(fixture, run_directory)
    consumers = _ProductionConsumers()
    Replayer(run_directory).replay(
        wheel=consumers.wheel,
        imu=consumers.imu,
        depth=consumers.depth,
    )
    return consumers.result(entry.scenario, time.perf_counter() - started)


def _require_mujoco() -> None:
    try:
        importlib.import_module("mujoco")
    except ModuleNotFoundError as exc:
        if exc.name == "mujoco":
            raise BackendSkipped("mujoco is not installed") from exc
        raise


def _report_result(report, reject_reasons: Sequence[str] = ()) -> dict[str, Any]:
    return {
        "passed": bool(report.passed),
        "completion": float(report.completion_ratio),
        "min_clearance": float(report.min_wheel_clearance_m),
        "fail_open": int(report.fail_open_count),
        "false_hold": int(report.false_hold_count),
        "runtime_s": float(report.wall_clock_runtime_s),
        "reject_reasons": list(reject_reasons),
        "failure_reasons": list(report.reasons),
    }


def _run_mujoco(entry: ManifestEntry, run_directory: Path) -> dict[str, Any]:
    _require_mujoco()
    runner = importlib.import_module("powertrain_sim.mujoco_fast.runner")
    depth = _ProductionConsumers()
    report = runner.run_scenario(
        entry.scenario,
        run_directory,
        depth_tap=depth.depth,
    )
    return _report_result(report, depth.reject_reasons)


def _run_closed_loop(entry: ManifestEntry, run_directory: Path) -> dict[str, Any]:
    _require_mujoco()
    try:
        module = importlib.import_module("powertrain_sim.closed_loop")
    except ModuleNotFoundError as exc:
        if exc.name == "powertrain_sim.closed_loop":
            raise BackendSkipped("powertrain_sim.closed_loop is not implemented") from exc
        raise
    return _report_result(module.run_closed_loop(entry.scenario, run_directory))


BACKEND_RUNNERS: Mapping[
    str, Callable[[ManifestEntry, Path], dict[str, Any]]
] = {
    "analytic": _run_analytic,
    "replay": _run_replay,
    "mujoco": _run_mujoco,
    "closed_loop": _run_closed_loop,
}


def _expectation_failure(entry: ManifestEntry, result: Mapping[str, Any]) -> str | None:
    if "passed" in entry.expected:
        if result["passed"] != entry.expected["passed"]:
            return (
                f"backend passed={result['passed']} does not match "
                f"expected {entry.expected['passed']}"
            )
        return None
    missing = sorted(
        set(entry.expected["reject_reasons"]) - set(result["reject_reasons"])
    )
    if missing:
        return f"missing expected reject reasons: {', '.join(missing)}"
    return None


def _execute_entry(
    entry: ManifestEntry,
    run_directory: Path,
    *,
    backend_runners: Mapping[
        str, Callable[[ManifestEntry, Path], dict[str, Any]]
    ],
) -> dict[str, Any]:
    base = {
        "id": entry.id,
        "source": entry.source,
        "scenario": entry.scenario_reference,
        "fixture_class": entry.fixture_class,
        "contract": entry.contract,
        "checksum": entry.checksum,
    }
    try:
        result = dict(backend_runners[entry.source](entry, run_directory))
    except BackendSkipped as exc:
        return {
            **base,
            "status": "SKIPPED",
            "passed": None,
            "completion": None,
            "min_clearance": None,
            "fail_open": None,
            "false_hold": None,
            "runtime_s": 0.0,
            "reject_reasons": [],
            "failure_reasons": [],
            "skip_reason": str(exc),
        }
    except Exception as exc:  # The JSON report must retain backend failures.
        return {
            **base,
            "status": "FAIL",
            "passed": False,
            "completion": None,
            "min_clearance": None,
            "fail_open": None,
            "false_hold": None,
            "runtime_s": 0.0,
            "reject_reasons": [],
            "failure_reasons": [f"{type(exc).__name__}: {exc}"],
        }
    expectation_failure = _expectation_failure(entry, result)
    if expectation_failure is not None:
        result.setdefault("failure_reasons", []).append(expectation_failure)
    return {
        **base,
        "status": "FAIL" if expectation_failure else "PASS",
        **result,
    }


def compare_backend_results(
    results: Sequence[Mapping[str, Any]],
    *,
    tolerance_by_id: Mapping[str, Mapping[str, float]],
) -> list[dict[str, Any]]:
    """Compare every available shared metric for fixture IDs with 2+ backends."""
    grouped: dict[str, list[Mapping[str, Any]]] = defaultdict(list)
    for result in results:
        if result["status"] != "SKIPPED":
            grouped[str(result["id"])].append(result)
    comparisons = []
    for fixture_id, group in grouped.items():
        if len(group) < 2:
            continue
        differences = {}
        failed = False
        for metric, tolerance in tolerance_by_id[fixture_id].items():
            values = [item.get(metric) for item in group]
            comparable = [float(value) for value in values if value is not None]
            if len(comparable) != len(group):
                continue
            difference = max(comparable) - min(comparable)
            differences[metric] = difference
            failed = failed or difference > float(tolerance)
        comparisons.append(
            {
                "id": fixture_id,
                "backends": [str(item["source"]) for item in group],
                "status": "FAIL" if failed else "PASS",
                "differences": differences,
            }
        )
    return comparisons


def _write_json(path: Path, document: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(
        json.dumps(document, sort_keys=True, indent=2, allow_nan=False) + "\n",
        encoding="utf-8",
    )


def run_regression(
    manifest_path: str | Path,
    out_path: str | Path,
    *,
    repo_root: str | Path = REPO_ROOT,
    backend_runners: Mapping[
        str, Callable[[ManifestEntry, Path], dict[str, Any]]
    ] = BACKEND_RUNNERS,
) -> int:
    """Execute a validated manifest and return its CLI-compatible exit code."""
    output = Path(out_path)
    try:
        entries = load_manifest(manifest_path, repo_root=repo_root)
    except ManifestError as exc:
        _write_json(
            output,
            {
                "schema_version": 1,
                "manifest": str(manifest_path),
                "results": [],
                "comparisons": [],
                "manifest_error": str(exc),
                "summary": {"passed": 0, "failed": 1, "skipped": 0},
            },
        )
        return 1

    with tempfile.TemporaryDirectory(prefix="powertrain-regression-") as temporary:
        root = Path(temporary)
        results = [
            _execute_entry(
                entry,
                root / f"{index:03d}-{entry.id}-{entry.source}",
                backend_runners=backend_runners,
            )
            for index, entry in enumerate(entries)
        ]
    tolerances = {entry.id: entry.tolerance for entry in entries}
    comparisons = compare_backend_results(
        results,
        tolerance_by_id=tolerances,
    )
    failed = sum(result["status"] == "FAIL" for result in results) + sum(
        comparison["status"] == "FAIL" for comparison in comparisons
    )
    summary = {
        "passed": sum(result["status"] == "PASS" for result in results),
        "failed": failed,
        "skipped": sum(result["status"] == "SKIPPED" for result in results),
    }
    _write_json(
        output,
        {
            "schema_version": 1,
            "manifest": str(manifest_path),
            "results": results,
            "comparisons": comparisons,
            "summary": summary,
        },
    )
    return 1 if failed else 0


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--manifest", required=True)
    parser.add_argument("--out", required=True)
    arguments = parser.parse_args(argv)
    return run_regression(arguments.manifest, arguments.out)


if __name__ == "__main__":
    raise SystemExit(main())
