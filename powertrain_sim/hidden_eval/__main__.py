"""Generate and run one deterministic WP6-S hidden-seed closed loop."""
from __future__ import annotations

import argparse
from collections.abc import Sequence
from pathlib import Path

from ..closed_loop import run_closed_loop
from ..procedural import (
    GenerationParameters,
    canonical_json_sha256,
    generate_scenario,
    scenario_yaml,
)
from ..scenario import SEED_CLASSES, parse_scenario


def evaluate_report(report) -> tuple[bool, str]:
    """Apply hidden-evaluation guards without mutating the metrics report."""

    if not report.passed:
        return False, "metrics_failed"
    if report.completion_ratio <= 0.05:
        return False, "no_progress"
    return True, "passed"


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("seed", type=int, help="PCG64 scenario seed")
    parser.add_argument("run_dir", help="new or empty run output directory")
    parser.add_argument(
        "--seed-class",
        choices=sorted(SEED_CLASSES),
        default="hidden_evaluation",
    )
    arguments = parser.parse_args(argv)

    # 폐루프는 고가 트랙 종단 낙하 앞에서 fail-closed 정지하는 것이 정답이라
    # 95% 완주 boolean은 False가 정직한 기대값이다(procedural.py 주석 참조).
    document = generate_scenario(
        GenerationParameters(expected_completion=False),
        seed=arguments.seed,
        seed_class=arguments.seed_class,
    )
    digest = canonical_json_sha256(document)
    output = Path(arguments.run_dir)
    output.mkdir(parents=True, exist_ok=True)
    (output / "scenario.yaml").write_text(
        f"# canonical_json_sha256: {digest}\n{scenario_yaml(document)}",
        encoding="utf-8",
    )
    report = run_closed_loop(parse_scenario(document), output)
    passed, reason = evaluate_report(report)
    print(report.summary())
    if reason == "no_progress":
        print("hidden_eval=no_progress")
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
